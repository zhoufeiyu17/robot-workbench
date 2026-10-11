"""Shared, loopback-first HTTP service. No robot connection occurs on page load."""
import argparse
from collections import OrderedDict
from copy import deepcopy
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import mimetypes
from pathlib import Path
import secrets
import threading
import time
import uuid
from urllib.parse import parse_qs, urlsplit,urlencode
from robot_workbench.workflows import CALIBRATION_TOOLS,HISTORY_ACTIONS,CalibrationWorkflow
from robot_workbench.joint_bounds import position_noise_rad, project_joint_positions
from robot_workbench.sample_clock import host_sample_status

ROOT = Path(__file__).resolve().parents[1]
BUSINESSES = ('calibration', 'collection')
WARM_C_ACTIONS = ('generate_plan','execute','install',*CALIBRATION_TOOLS)
WARM_C_REASON = '当前为复用标定的数采诊断；完整标定请使用空库验收入口'


class Device:
    def __init__(self, device_id, environment, factory, *, label=None, drive=None):
        self.id, self.environment, self.factory = device_id, environment, factory
        self.label = label or device_id
        self.runtime = None
        self.credentials = {}
        self.workflows, self.media = {}, {}
        self.drive = drive
        self.lock = threading.Lock()
        self.closed = threading.Event()
        self.thread = None
        self.error = None
        self.preview = None
        self.collection_preparation = None
        self.stop_latched = False
        self.connecting = False
        self.release_receipt = None

    def connect(self, body):
        if self.environment == 'physical' and body.get('arms_supported') is not True:
            raise ValueError('连接前请确认此刻双臂已可靠支撑')
        if not self.lock.acquire(blocking=False):
            raise ValueError('设备连接正在准备')
        try:
            self.connecting = True
            if self.runtime is not None:
                raise ValueError('设备已连接；控制权交接不会断开设备')
            if self.thread is not None and self.thread.is_alive():
                raise ValueError('上次设备连接仍在收尾，请等待断开确认')
            if self.factory is None:
                raise ValueError('设备控制后端尚未配置')
            runtime = self.factory()
            installed = (self.release_receipt or {}).get('calibration_snapshot_id')
            if installed and runtime.calibration_snapshot_id is None:
                runtime.notify_calibration_installed(installed)
            credentials = {name: runtime.register_business(name) for name in BUSINESSES}
            self.credentials = credentials
            self.runtime = runtime
            self.release_receipt = None
            self.error = None
            if self.stop_latched:
                runtime.submit(credentials[runtime.holder], {'action':'stop'})
            self.closed.clear()
            def loop():
                try:
                    if self.drive is not None:
                        self.drive(runtime, self.closed)
                        return
                    while not self.closed.is_set():
                        started = time.monotonic()
                        if not runtime.tick():
                            break
                        self.closed.wait(max(0., .01-(time.monotonic()-started)))
                except Exception as error:
                    self.error = str(error)
                    runtime.submit(self.credentials[runtime.holder], {'action': 'stop'})
                finally:
                    if runtime.release_receipt is not None:
                        try:
                            if runtime.fd is not None:
                                runtime.close()
                            self.closed.set()
                            with self.lock:
                                if self.runtime is runtime:
                                    self.runtime = None
                                    self.credentials = {}
                        except Exception as error:
                            runtime.release_receipt.update(status='unconfirmed', message='连接收尾未确认：'+str(error))
                        self.release_receipt = deepcopy(runtime.release_receipt)
            self.thread = threading.Thread(target=loop, daemon=True, name='device-owner-'+self.environment)
            self.thread.start()
            return {'message': '设备已连接，等待健康反馈；尚未执行运动'}
        finally:
            self.connecting = False
            self.lock.release()

    def submit(self, business, command):
        if command.get('action') == 'stop':
            self.stop_latched = True
        runtime, credentials = self.runtime, self.credentials
        if runtime is None:
            if command == {'action': 'stop'}:
                return {'message': '设备未连接，没有可发送的运动命令'}
            raise ValueError('请先连接设备')
        try:
            return runtime.submit(credentials.get(business), command)
        except PermissionError:
            if runtime.fd is not None and runtime is self.runtime:
                raise
            if command == {'action':'stop'}:
                return {'message':'设备连接已撤权或正在收尾，没有新的运动命令'}
            raise ValueError('设备控制连接已变化，请重新核验状态')

    def snapshot(self, business):
        runtime = self.runtime
        if runtime is None:
            return dict(connected=False, state='CONNECTING' if self.connecting else 'DISCONNECTED', holder=None, feedback_healthy=False,
                        release_receipt=deepcopy(self.release_receipt),
                        allowed_operations={key: dict(allowed=key == 'stop', reason='请先连接设备')
                                            for key in ('stop','preview','confirm','handoff')})
        credential = self.credentials.get(business)
        if self.runtime is not runtime:
            return self.snapshot(business)
        try:
            result = runtime.snapshot(credential)
        except PermissionError:
            if runtime.fd is not None:
                raise
            return dict(connected=False,state='DISCONNECTED',holder=None,feedback_healthy=False,
                release_receipt=deepcopy(runtime.release_receipt),
                allowed_operations={key:dict(allowed=key=='stop',reason='设备连接正在收尾')
                                    for key in ('stop','preview','confirm','handoff')})
        receipt = result.get('release_receipt')
        if receipt and receipt['status']=='requested' and time.monotonic()-receipt['requested_monotonic_s']>=2:
            receipt.update(status='unconfirmed',message='释放超过 2 秒仍未确认；请查看设备诊断，不能重新连接')
        result['sample_wall_ns'] = (result.get('calibration_feedback') or {}).get('sample_wall_ns')
        result['motion_status'] = (result.get('motion') or {}).get('status')
        if result.get('motion') is None and result.get('connected') and result.get('automatic_enabled') is False:
            result['motion_status'] = 'no_automatic_task'
        if self.error:
            result['service_error'] = self.error
        return result


class Application:
    def __init__(self, devices):
        if any(key not in ('fake','physical') or device.environment != key for key,device in devices.items()):
            raise ValueError('device environment namespace mismatch')
        self.devices = devices
        self.csrf = secrets.token_urlsafe(32)
        self.server_instance_id = str(uuid.uuid4())  # Public lifetime identity, not an authorization token.
        self.lock = threading.Lock()
        self.revision = 0
        self.display_actions=threading.Lock()
        self.preview_contexts = {}
        self.jobs, self.requests, self.reviews = {}, OrderedDict(), {}
        self.manual_tasks = {}
        self.generations = {environment: 0 for environment in devices}

    def device(self, environment):
        if environment not in self.devices:
            raise ValueError('该环境尚未配置，不能切换到另一个环境代替执行')
        return self.devices[environment]

    def preview_state(self,device,runtime=None):
        provider=device.preview
        value=provider.state() if provider else dict(connected=False,status='unconfigured',error=None,
            recovery=dict(allowed=False,message='当前环境尚未启用目标图像服务；实际相机画面不能代替目标姿态预览。'))
        raw=value.get('revisions') or {};revisions=dict(target=raw.get('target',0),overview=raw.get('view',0),mesh=raw.get('mesh',0))
        runtime=device.snapshot('calibration') if runtime is None else runtime
        binding=(value.get('renderer_instance_id'),*(runtime.get(key) for key in ('runtime_nonce','command_epoch','calibration_revision','calibration_snapshot_id','holder','execution_instance_id')))
        changed=False
        with self.lock:
            previous=self.preview_contexts.get(device.environment)
            if previous is None or previous['binding']!=binding:
                reset=previous is None or previous['binding'][0]!=binding[0]
                previous=dict(binding=binding,id=secrets.token_hex(16),floors={view:0 if reset else revision+1 for view,revision in revisions.items()})
                self.preview_contexts[device.environment]=previous
                changed=True
                # Publish and bind together; this only invalidates bounded in-memory preview slots.
                if provider and hasattr(provider,'bind_context'):provider.bind_context(previous['id'])
            display_context,floors=previous['id'],previous['floors']
        if changed and provider and hasattr(provider,'bind_context'):
            value=provider.state()
        frames={}
        for view in ('target','overview','mesh'):
            meta=(value.get('frames') or {}).get(view) or {}
            completed=meta.get('client_revision') if view=='target' else meta.get('client_view_revision') if view=='overview' else meta.get('revision')
            ready=bool(meta and value.get('connected')) and type(completed) is int and floors[view]<=completed<=revisions[view]
            if view!='mesh':ready=ready and meta.get('client_view_revision')==revisions['overview']
            frames[view]=dict(ready=ready,url='/api/preview-media?'+urlencode(dict(environment=device.environment,view=view)),
                revision=completed,view_revision=meta.get('client_view_revision',revisions['overview']),frame_id=meta.get('frame_id'),
                source_wall_ns=meta.get('source_wall_ns',meta.get('source_timestamp_ns')),error=value.get('mesh_error') if view=='mesh' else value.get('error'))
        selection=value.get('mesh') or {}
        return dict(value,renderer_id=value.get('renderer_instance_id'),display_context=display_context,target_floor=floors['target'],revision_floors=dict(floors),revisions=revisions,frames=frames,
            diagnostic_calibration=deepcopy(getattr(device,'diagnostic_calibration',None)) if device.environment=='fake' else None,
            diagnostic_candidates=deepcopy(getattr(device,'diagnostic_mesh_candidates',[])) if device.environment=='fake' else [],
            mesh=dict(requested=selection.get('enabled',False),applied=bool(selection.get('enabled') and frames['mesh']['ready']),
                      camera=selection.get('camera','head'),candidate_id=selection.get('candidate_id')))

    def preview_media(self,environment,view,*,revision=None,display_context=None,renderer_id=None,view_revision=None,wait_ms=0):
        device=self.device(environment)
        if device.preview is None:raise ValueError('目标图像服务未配置')
        if revision is not None:
            before=self.preview_state(device)
            if (view!='target' or type(revision) is not int or revision<0 or display_context!=before['display_context']
                    or renderer_id!=before['renderer_id'] or view_revision!=before['revisions']['overview']):
                raise ValueError('exact target request belongs to another display context')
            raw,metadata=device.preview.media(view,revision,display_context=display_context,renderer_instance_id=renderer_id,view_revision=view_revision,wait_ms=wait_ms)
        else:raw,metadata=device.preview.media(view)
        state=self.preview_state(device)
        if revision is not None and (display_context!=state['display_context'] or renderer_id!=state['renderer_id'] or view_revision!=state['revisions']['overview'] or metadata.get('client_revision')!=revision):
            raise ValueError('exact target context changed while waiting')
        if metadata.get('renderer_instance_id')!=state.get('renderer_id'):raise ValueError('renderer instance changed while reading display frame')
        revision=metadata.get('client_revision') if view=='target' else metadata.get('client_view_revision') if view=='overview' else metadata.get('revision')
        current=type(revision) is int and state['revision_floors'][view]<=revision<=state['revisions'][view]
        if not current or view!='target' and revision!=state['revisions'][view] or view!='mesh' and metadata.get('client_view_revision')!=state['revisions']['overview']:
            raise ValueError('display frame belongs to an older request')
        selection=state['mesh']
        if view=='mesh' and metadata.get('environment')!=environment:raise ValueError('mesh observation belongs to another device environment')
        if view=='mesh' and (not selection['requested'] or metadata.get('candidate_id')!=selection['candidate_id'] or metadata.get('camera')!=selection['camera']):
            raise ValueError('candidate mesh selection changed')
        return raw,dict(metadata,revision=revision,view_revision=state['revisions']['overview'],view=view,environment=environment,
            server_instance_id=self.server_instance_id,display_context=state['display_context'],renderer_id=metadata['renderer_instance_id'],source_wall_ns=metadata.get('source_wall_ns',metadata.get('source_timestamp_ns')))

    @staticmethod
    def preparation_view(preparation):
        phase=preparation['status'];busy=preparation['busy'];installed=preparation.get('installed') or {}
        titles={'awaiting_calibration':'等待标定安装','queued':'数采环境等待准备','verifying_installation':'核验当前已安装标定',
            'preparing_profile':'根据已安装标定构建数采配置','converting_planning_assets':'转换数采规划资产',
            'initializing_detector':'初始化物体检测服务','initializing_planner':'初始化轨迹规划服务',
            'ready':'数采环境已就绪','failed':'数采环境准备失败'}
        title=titles.get(phase,'准备数采环境')
        detail=preparation.get('error') or (preparation.get('worker') or {}).get('message') or title
        retained=preparation.get('installed_calibration_preserved')
        return dict(status='running' if busy else 'failed' if phase=='failed' else 'idle',busy=busy,
            stage_id='collection.setup.'+phase,title=title,
            message=('已安装标定保留，无需重新采集。' if retained else '')+str(detail),
            primary_action=dict(id='prepare_collection',label='重新准备数采环境',allowed=preparation['can_retry'],reason=preparation['retry_reason']),
            progress=dict(indeterminate=True,unit=title) if busy else None,
            details=[dict(label='头部标定',value=installed.get('head_version') or installed.get('head_id') or '尚未安装'),
                     dict(label='双腕标定',value=installed.get('wrist_version') or installed.get('wrist_id') or '尚未安装'),
                     dict(label='本次驻留剩余时间',value=f"{preparation['remaining_wall_s']:.0f} 秒")],
            outcomes=dict(calibration='已安装并保留' if retained else '等待有效安装',preparation=phase),preparation=preparation)

    def state(self, environment, business):
        if business not in BUSINESSES:
            raise ValueError('未知业务')
        device = self.device(environment)
        runtime = device.snapshot(business)
        workflow = device.workflows.get(business)
        readiness=None
        if business=='calibration' and isinstance(workflow,CalibrationWorkflow):
            reader=getattr(workflow.app,'hardware_reader',None)
            readiness=host_sample_status((runtime.get('calibration_feedback') or {}).get('sample_wall_ns'),
                time.time_ns(),max_age_s=min(.5,getattr(reader,'max_age_s',.5)))
            gate=runtime['allowed_operations'].get('confirm',{})
            if not readiness['host_sample_recent'] and gate.get('allowed'):
                runtime['allowed_operations']['confirm']=dict(allowed=False,reason=readiness['reason'])
        with self.lock:
            self.revision += 1
            revision = self.revision
            job = deepcopy(self.jobs.get((environment,business)))
            review = deepcopy(self.reviews.get((environment,business)))
            manual_task = deepcopy(self.manual_tasks.get((environment,business)))
        value = workflow.snapshot() if workflow else dict(status='unconfigured',busy=False,
            stage_id='configuration',title='配置业务后端',message='设备控制可独立验证；当前业务的采集、求解与数据路径尚未配置。',
            primary_action=None,progress=None,outcomes={})
        if readiness is not None:value['control_readiness']=readiness
        preparation=device.collection_preparation.snapshot() if business=='collection' and device.collection_preparation else None
        preparing=bool(preparation and (workflow is None or preparation['busy'] or preparation['status']=='failed' and not value.get('busy')))
        if preparing:
            value=self.preparation_view(preparation)
            review=None
        if review is not None:
            if (review['runtime_nonce'] != runtime.get('runtime_nonce') or review['epoch'] != runtime.get('command_epoch')
                    or review.get('calibration_revision') != runtime.get('calibration_revision')):
                review = None
            elif time.monotonic() > review['deadline']:
                review = None
            elif workflow and review.get('workflow_revision') is not None and review['workflow_revision']!=workflow.manual_revision():
                review=None
            elif workflow and review.get('workflow_generation') is not None and review['workflow_generation']!=workflow.generation:
                review=None
            elif review['action']=='manual_execute' and workflow and (value.get('busy') or
                    review.get('manual_revision') is not None and review['manual_revision']!=workflow.manual_revision()):
                review=None
        manual = self.manual_state(manual_task,runtime,review,value.get('busy'))
        if manual:
            if manual['status']=='completed' and value.get('primary_action'):
                manual['primary_action']=value['primary_action']
            value.update(manual)
        elif manual_task:
            with self.lock:
                if (self.manual_tasks.get((environment,business)) or {}).get('id')==manual_task['id']:
                    self.manual_tasks.pop((environment,business),None)
        if review:
            value['review'] = {key:review[key] for key in ('id','summary','physical_clearance_required')}
            if review.get('speed_scale') is not None:value['review']['speed_scale']=review['speed_scale']
            execution_gate=runtime['allowed_operations']['confirm' if review['owner_preview'] else 'preview']
            value['primary_action'] = dict(id=review['action'],label='审阅并执行本批',
                allowed=execution_gate['allowed'],reason=execution_gate['reason'])
        if job and job['status'] in ('running','failed'):
            value['request'] = job
            if job['status'] == 'running':
                value['busy'] = True
                if value.get('primary_action'):
                    value['primary_action'].update(allowed=False,reason='当前请求仍在处理')
        allowed = runtime.setdefault('allowed_operations',{})
        connect_ready=device.runtime is None and not device.connecting and not (device.thread and device.thread.is_alive())
        allowed['connect'] = dict(allowed=bool(connect_ready and device.factory is not None),
                                  reason=('设备控制后端尚未配置' if device.factory is None else
                                          None if connect_ready else '设备仍连接或正在收尾，请等待明确断开'))
        allowed.setdefault('release',dict(allowed=False,reason='请先连接设备'))
        if any(flow.snapshot().get('busy') for flow in device.workflows.values()):
            allowed['release']=dict(allowed=False,reason='请先结束或取消业务任务，再释放设备连接')
        holder = runtime.get('holder')
        # This is an explicit operator handoff endpoint, not an application
        # pretending to be the current holder when sending a motion command.
        if device.runtime is not None:
            source = device.snapshot(holder)['allowed_operations']['handoff']
            allowed['handoff'] = source
            if any(flow.snapshot().get('busy') for flow in device.workflows.values()):
                allowed['handoff'] = dict(allowed=False,reason='业务任务仍在进行；请先结束或取消当前任务')
        motion_reason = allowed['preview']['reason']
        manual_ready=business=='calibration' and allowed['preview']['allowed'] and not value.get('busy')
        manual_reason=motion_reason or ('当前标定任务正在进行' if value.get('busy') else None)
        if business!='calibration':manual_reason='人工姿态工具需要交接到标定业务'
        if environment=='physical' and not (workflow and hasattr(workflow,'manual_preview')):
            manual_ready=False;manual_reason='物理人工规划需要标定路径校验服务'
        allowed['manual_preview']=dict(allowed=bool(manual_ready),reason=manual_reason)
        queue=value.get('manual_queue') or dict(available=False,reason='未配置标定服务，人工队列不可用')
        for name in ('manual_queue','manual_queue_preview'):
            allowed[name]=dict(allowed=bool(manual_ready and queue.get('available')),reason=manual_reason or queue.get('reason'))
        allowed['manual_execute'] = dict(allowed=bool(review and review['action']=='manual_execute') and allowed['confirm']['allowed'],
            reason=allowed['confirm']['reason'] or (None if review else '请先生成并审阅人工姿态'))
        allowed['recover'] = dict(allowed=bool(runtime.get('recoverable_fault')) and holder==business,
            reason=None if runtime.get('recoverable_fault') and holder==business else '当前没有可确认恢复的故障；设备故障请查看诊断')
        preview=self.preview_state(device,runtime)
        for action in ('preview_target','preview_overview','preview_mesh'):
            allowed[action]=dict(allowed=bool(preview.get('connected')),reason=None if preview.get('connected') else '目标图像服务尚未就绪，请查看画面恢复指引')
        allowed['preview_recover']=dict(allowed=bool(device.preview and hasattr(device.preview,'recover') and preview.get('recovery',{}).get('allowed')),
            reason=preview.get('recovery',{}).get('message'))
        for action in ('cancel','mesh','select_result','install','generate_plan','execute',*CALIBRATION_TOOLS,*HISTORY_ACTIONS):
            allowed.setdefault(action,dict(allowed=False,reason='当前业务后端尚未配置'))
        if workflow:
            for action in ('cancel','mesh','select_result','install','generate_plan','execute',*CALIBRATION_TOOLS,*HISTORY_ACTIONS):
                if hasattr(workflow,'allowed'):
                    allowed[action]=workflow.allowed(action)
                if business=='calibration' and action in (*CALIBRATION_TOOLS,'generate_plan') and not allowed['preview']['allowed']:
                    allowed[action]=dict(allowed=False,reason=allowed['preview']['reason'])
        if readiness is not None and not readiness['host_sample_recent']:
            for action in ('execute','manual_execute'):
                if allowed[action]['allowed']:allowed[action]=dict(allowed=False,reason=readiness['reason'])
        if preparation:
            allowed['prepare_collection']=dict(allowed=preparation['can_retry'],reason=preparation['retry_reason'])
            if preparing:
                for action in ('generate_plan','execute'):
                    allowed[action]=dict(allowed=False,reason='先完成数采环境准备；已安装标定仍保留')
        if business=='calibration' and (preview.get('diagnostic_calibration') or {}).get('mode')=='warm_diagnostic':
            for action in WARM_C_ACTIONS:allowed[action]=dict(allowed=False,reason=WARM_C_REASON)
        primary = value.get('primary_action')
        if primary and primary['id'] in (*CALIBRATION_TOOLS,'generate_plan') and not allowed[primary['id']]['allowed']:
            primary.update(allowed=False,reason=allowed[primary['id']]['reason'])
        if readiness is not None and primary and primary['id'] in ('execute','manual_execute') and not allowed[primary['id']]['allowed']:
            primary.update(allowed=False,reason=allowed[primary['id']]['reason'])
        if primary and primary['id']!='prepare_collection' and (not runtime['connected'] or holder != business):
            primary.update(allowed=False,reason='请先连接设备' if not runtime['connected'] else '请先交接控制权限：当前由 '+str(holder)+' 持有')
        raw = (runtime.get('calibration_feedback') or {}).get('qpos')
        limits = runtime.get('joint_limits')
        manual = {}
        if raw and limits:
            from arx_env.joint_head_jaw_mapping import feedback_gaps
            from arx_env.joint_head_real_motion import DEFAULT_VELOCITY
            manual=dict(joint_names=[side+' J'+str(i) for side in ('左','右') for i in range(1,7)],
                positions=raw[:6]+raw[7:13], limits=limits[:6]+limits[7:13],jaw_gaps_mm=feedback_gaps(raw),
                position_noise_rad=position_noise_rad(environment),
                default_joint_speed_rad_s=DEFAULT_VELOCITY,default_jaw_speed_mm_s=10.)
        manual['queue']=queue
        calibration=device.workflows.get('calibration')
        arrival=(calibration.snapshot().get('arrival_statistics') if calibration and hasattr(calibration,'snapshot') else None) or {'error':'未配置标定到位统计；协议 Fake 不生成统计数据'}
        return dict(schema='robot-workbench-ui.v1',server_instance_id=self.server_instance_id,
            revision=revision,server_wall_ns=time.time_ns(),csrf=self.csrf,
            device=dict(id=device.id,label=device.label,environment=environment),runtime=runtime,
            workflow=value,media=[provider.snapshot() for provider in device.media.values()],
            results=workflow.results() if workflow else [],diagnostics=[
                {'label':'控制器状态','value':runtime.get('state','未知')},
                {'label':'控制器当前原因','value':runtime.get('reason') or '未提供当前状态原因'},
                {'label':'服务线程异常','value':device.error or '未报告服务线程异常'},*value.get('diagnostics',[])],
            mesh=workflow.mesh() if workflow and hasattr(workflow,'mesh') else {'requested':False,'applied':False},manual=manual,
            arrival_statistics=arrival,preview=preview)

    def action(self, action, body):
        if not isinstance(body,dict) or body.get('business') not in BUSINESSES:
            raise ValueError('业务与请求对象必需')
        environment,business=body.get('environment'),body['business']
        device=self.device(environment)
        request_id=body.get('request_id')
        if not isinstance(request_id,str) or not 1 <= len(request_id) <= 128:
            raise ValueError('有效的单次请求身份必需')
        key=(environment,business,request_id)
        original_body=deepcopy(body)
        with self.lock:
            if key in self.requests:
                previous=self.requests[key]
                if previous['body'] != body or previous['action'] != action:
                    raise ValueError('同一请求身份不可绑定不同操作')
                return previous['response']
        if action=='stop':
            result=device.submit(business,{'action':'stop'})
            self.invalidate(environment,clear_manual=True)
            for workflow in device.workflows.values():
                threading.Thread(target=workflow.cancel,daemon=True).start()
            return {**result,'message':'停止已受理；请查看控制器确认状态'}
        if (environment=='fake' and business=='calibration' and action in WARM_C_ACTIONS
                and (getattr(device,'diagnostic_calibration',None) or {}).get('mode')=='warm_diagnostic'):
            raise ValueError(WARM_C_REASON)
        if action in ('preview_target','preview_overview','preview_mesh','preview_recover'):
            provider=device.preview
            if provider is None:raise ValueError('目标图像服务未配置')
            fields={'preview_target':{'q12','jaw_gap_mm','display_context','renderer_id','view_revision','presented_through_revision'},'preview_overview':{'yaw','pitch','zoom','pan_y'},
                    'preview_mesh':{'enabled','camera','candidate_id'},'preview_recover':set()}[action]
            if set(body)-{'environment','business','request_id'}-fields:raise ValueError('unknown display action field')
            # Serialize only these cache/queue writes. Owner stop/state never takes this lock.
            with self.display_actions:
                with self.lock:
                    previous=self.requests.get(key)
                    if previous:
                        if previous['body']!=original_body or previous['action']!=action:raise ValueError('同一请求身份不可绑定不同操作')
                        return previous['response']
                if action=='preview_target':
                    context=self.preview_state(device)
                    if any(key in body for key in ('display_context','renderer_id','view_revision','presented_through_revision')) and (
                            body.get('display_context')!=context['display_context'] or body.get('renderer_id')!=context['renderer_id']
                            or body.get('view_revision')!=context['revisions']['overview']):
                        raise ValueError('target input or presentation ACK belongs to another display context')
                    q=body['q12'];corrections=[]
                    runtime=device.runtime
                    limits=None
                    if runtime:
                        with runtime.owner.lock:limits=runtime.owner.cache.get('joint_limits')
                    if limits:
                        q,corrections=project_joint_positions(q,limits[:6]+limits[7:13],environment=environment)
                    result=provider.target_pose(q,body['jaw_gap_mm'],display_context=context['display_context'],
                        renderer_instance_id=context['renderer_id'],view_revision=context['revisions']['overview'],
                        presented_through_revision=body.get('presented_through_revision'))
                    result=dict(result,display_context=context['display_context'],renderer_id=context['renderer_id'],view_revision=context['revisions']['overview'],
                        position_normalization=dict(environment=environment,corrections=corrections))
                elif action=='preview_overview':result=provider.overview(body['yaw'],body['pitch'],body['zoom'],body.get('pan_y',0.))
                elif action=='preview_mesh':result=provider.mesh(body['enabled'],camera=body.get('camera','head'),candidate_id=body.get('candidate_id'))
                else:
                    if not hasattr(provider,'recover') or not provider.state().get('recovery',{}).get('allowed'):raise ValueError('渲染器当前不可恢复')
                    result=provider.recover()
                with self.lock:
                    self.requests[key]=dict(body=original_body,action=action,response=result)
                    while len(self.requests)>256:self.requests.popitem(last=False)
            return result
        if action=='handoff':
            runtime=device.runtime
            if runtime is None:raise ValueError('设备未连接')
            if any(flow.snapshot().get('busy') for flow in device.workflows.values()):
                raise ValueError('业务任务仍在进行；请先结束或取消当前任务')
            result=runtime.handoff(device.credentials[runtime.holder],body.get('target_business'))
            self.invalidate(environment,clear_manual=True)
            return {'message':'控制权限已交接，保持连接',**result}
        if action=='release':
            if set(body)-{'environment','business','request_id','arms_supported'}:
                raise ValueError('unknown release field')
            if environment=='physical' and body.get('arms_supported') is not True:
                raise ValueError('释放前请确认此刻双臂已可靠支撑；释放会撤掉保持')
            if any(flow.snapshot().get('busy') for flow in device.workflows.values()):
                raise ValueError('请先结束或取消业务任务，再释放设备连接')
            runtime=device.runtime
            if runtime is None:raise ValueError('设备未连接')
            with self.lock:
                if any(job.get('status')=='running' for (env,_),job in self.jobs.items() if env==environment):
                    raise ValueError('当前请求尚未结束，请等待收尾后释放连接')
                result=runtime.request_release(device.credentials[business])
                result={**result,'message':'释放已受理；请等待设备连接已释放的确认'}
                self.requests[key]=dict(body=original_body,action=action,response=result)
                while len(self.requests)>256:self.requests.popitem(last=False)
            self.invalidate(environment,clear_manual=True)
            return result
        if action=='prepare_collection':
            if business!='collection' or set(body)!={'environment','business','request_id'} or device.collection_preparation is None:
                raise ValueError('当前数采后台准备入口未配置')
            state=device.collection_preparation.snapshot()
            if not state['can_retry']:raise ValueError(state['retry_reason'] or '当前数采准备不能重试')
        if business=='collection' and action in ('generate_plan','execute') and device.collection_preparation:
            if device.collection_preparation.snapshot()['status']!='ready':
                raise ValueError('数采环境尚未准备成功；已有标定保留，请先完成环境准备')
        if action in ('manual_execute','execute'):
            review_id=body.get('review_id')
            with self.lock:
                if (self.jobs.get((environment,business)) or {}).get('status')=='running':
                    raise ValueError('当前请求仍在处理，不能执行旧审批')
                review=self.reviews.get((environment,business))
                if not review or review['id'] != review_id or review['action'] != action or time.monotonic()>review['deadline']:
                    raise ValueError('审批已过期或不属于当前任务，请重新生成计划')
                if body.get('physical_clearance') is not True:
                    raise ValueError('执行前必须明确确认本批轨迹与净空')
                if review.get('speed_scale') is not None and (type(body.get('speed_scale',review['speed_scale'])) not in (int,float)
                        or body.get('speed_scale',review['speed_scale'])!=review['speed_scale']):
                    raise ValueError('执行速度已变化，请重新审阅')
            workflow=device.workflows.get(business)
            if workflow and review.get('workflow_revision') is not None and review['workflow_revision']!=workflow.manual_revision():
                raise ValueError('标定计划或求解来源已变化，请重新审阅')
            if workflow and review.get('workflow_generation') is not None and review['workflow_generation']!=workflow.generation:
                raise ValueError('数采任务已变化，请重新审阅')
            if action=='manual_execute' and workflow and (workflow.snapshot().get('busy') or
                    review.get('manual_revision') is not None and review['manual_revision']!=workflow.manual_revision()):
                raise ValueError('人工队列或标定任务已变化，请重新预览')
            status=device.snapshot(business)
            if (status.get('runtime_nonce'),status.get('command_epoch')) != (review['runtime_nonce'],review['epoch']):
                raise ValueError('控制上下文已变化，请重新审阅')
            with self.lock:
                if (self.reviews.get((environment,business)) or {}).get('id')!=review_id:
                    raise ValueError('审批已撤销，请重新生成计划')
                self.reviews.pop((environment,business))
                task=self.manual_tasks.get((environment,business))
                if action=='manual_execute' and task and task['id']==review_id:
                    task['submitted_monotonic_s']=time.monotonic()
            if action=='manual_execute':
                try:
                    return device.submit(business,dict(action='confirm',token=review['token'],
                        confirmation='START_REVIEWED_SINGLE_ARM',physical_clearance=True))
                except Exception:
                    with self.lock:self.manual_tasks.pop((environment,business),None)
                    raise
            body={**body,'token':review['token']}
            if review.get('speed_scale') is not None:body['speed_scale']=review['speed_scale']
        if action not in {'connect','prepare_collection','manual_preview','manual_queue','manual_queue_preview','generate_plan','execute','recover','cancel','install','select_result','mesh',*CALIBRATION_TOOLS,*HISTORY_ACTIONS}:
            raise ValueError('未知操作')
        if action not in ('connect','prepare_collection','mesh','select_result','cancel',*HISTORY_ACTIONS):
            if device.runtime is None:raise ValueError('请先连接设备')
            if device.runtime.holder != business:
                raise PermissionError('请先交接控制权限：当前由 '+device.runtime.holder+' 持有')
        with self.lock:
            if key in self.requests:
                previous=self.requests[key]
                if previous['body']!=original_body or previous['action']!=action:
                    raise ValueError('同一请求身份不可绑定不同操作')
                return previous['response']
            active=self.jobs.get((environment,business))
            if active and active['status']=='running':raise ValueError('当前请求仍在处理；停止始终可用')
            job=dict(id=secrets.token_hex(12),action=action,status='running',started_wall_ns=time.time_ns())
            self.jobs[(environment,business)]=job
            if action in ('generate_plan','manual_preview','manual_queue','manual_queue_preview','cancel',*CALIBRATION_TOOLS):
                self.manual_tasks.pop((environment,business),None)
                self.reviews.pop((environment,business),None)
            response=dict(message='请求已受理，请查看当前任务状态',job_id=job['id'])
            self.requests[key]=dict(body=original_body,action=action,response=response)
            generation=self.generations[environment]
            if action=='connect':device.stop_latched=False
            while len(self.requests)>256:self.requests.popitem(last=False)
        def work():
            try:
                with self.lock:
                    if generation != self.generations[environment]:
                        job.update(status='cancelled',finished_wall_ns=time.time_ns())
                        return
                result=self.perform(device,business,action,body)
                with self.lock:
                    job.update(status='completed' if generation==self.generations[environment] else 'cancelled',
                               result=result,finished_wall_ns=time.time_ns())
            except Exception as error:
                with self.lock:job.update(status='failed',error=str(error),finished_wall_ns=time.time_ns())
        threading.Thread(target=work,daemon=True,name='workbench-'+action).start()
        return response

    @staticmethod
    def manual_state(task, runtime, review, workflow_busy):
        if not task or workflow_busy or (task['runtime_nonce'],task['epoch'],'calibration') != (
                runtime.get('runtime_nonce'),runtime.get('command_epoch'),runtime.get('holder')):
            return None
        submitted=task.get('submitted_monotonic_s')
        status,stage,title,message='awaiting_review','manual.review','人工轨迹待审阅',task['summary']
        if submitted is None:
            if not review or review['id']!=task['id']:return None
        else:
            status,stage,title,message='pending','manual.confirmation','人工执行等待控制器回执','执行请求已受理，等待 Runtime 确认本次轨迹。'
            binding=runtime.get('execution_calibration') or {}
            plans=task['plan_sha256'];motion=runtime.get('motion') or {}
            if (plans and list(binding.get('plan_sha256',[]))==plans and binding.get('command_epoch')==task['epoch']
                    and binding.get('accepted_monotonic_s',0)>=submitted and motion.get('plan_sha256') in plans):
                status,stage,title,message='executing','manual.execute','人工轨迹执行中',motion.get('reason') or 'Runtime 正在执行本次审阅的轨迹。'
                if motion.get('status')=='READY' and motion['plan_sha256']==plans[-1]:
                    status,stage,title='completed','manual.complete','人工轨迹已完成'
                    message='本次轨迹最后一个路径点已收到 READY 回执，动作已结束并保持当前位置。'
                elif motion.get('status') in ('FAULT','STOPPED'):
                    status,stage,title='failed','manual.failed','人工轨迹未完成'
        return dict(id=task['id'],status=status,busy=status in ('pending','executing'),stage_id=stage,title=title,
            message=message,progress=None,overall_progress=None,review=None,
            primary_action=dict(id='manual_tools',label='使用人工规划工具',allowed=False,reason='请使用下方人工规划工具生成新预览'),
            details=[{'label':'审批身份','value':task['id']},{'label':'轨迹身份','value':' / '.join(task['plan_sha256'])}],
            outcomes=dict(motion=status,data='not_applicable',simulation='not_applicable',task=status))

    def invalidate(self, environment, *, clear_manual=False):
        with self.lock:
            self.generations[environment] += 1
            for business in BUSINESSES:
                key=(environment,business)
                self.reviews.pop(key,None)
                if clear_manual or not (self.manual_tasks.get(key) or {}).get('submitted_monotonic_s'):
                    self.manual_tasks.pop(key,None)
        for workflow in self.device(environment).workflows.values():workflow.invalidate()

    def invalidate_collection_preparation(self,device):
        """Replacing D preparation revokes its review/checkpoint, never C or motors."""
        with self.lock:self.reviews.pop((device.environment,'collection'),None)
        workflow=device.workflows.get('collection')
        if workflow is not None:workflow.invalidate()

    def perform(self, device, business, action, body):
        if action=='connect':return device.connect(body)
        if action=='prepare_collection':return device.collection_preparation.retry()
        runtime=device.runtime
        if runtime is None and action not in ('mesh','select_result',*HISTORY_ACTIONS):raise ValueError('请先连接设备')
        if runtime is not None and runtime.holder != business and action not in ('mesh','select_result','cancel',*HISTORY_ACTIONS):
            raise PermissionError('请先交接控制权限：当前由 '+runtime.holder+' 持有')
        if action in ('manual_preview','manual_queue','manual_queue_preview'):
            if business!='calibration':raise ValueError('人工姿态工具需要交接到标定业务')
            gate=device.snapshot(business)['allowed_operations']['preview']
            if not gate['allowed']:raise ValueError(gate['reason'] or '当前控制器不允许人工规划')
            workflow=device.workflows.get(business)
            with self.lock:self.reviews.pop((device.environment,business),None)
            if action=='manual_queue':
                if not workflow or not hasattr(workflow,'manual_queue'):raise ValueError('未配置标定动作队列')
                return workflow.manual_queue(body)
            if workflow and hasattr(workflow,'manual_preview'):
                preview=workflow.manual_preview(body,queue=action=='manual_queue_preview')
            else:
                if action=='manual_queue_preview' or device.environment!='fake':raise ValueError('未配置标定路径校验与队列服务')
                from arx_env.joint_head_real_motion import DEFAULT_VELOCITY
                command=dict(action='preview_workbench',waypoints=[dict(q12=body.get('q12'),
                    speeds12=body.get('speeds12',[DEFAULT_VELOCITY]*12),jaw_gap_mm=body.get('jaw_gap_mm'),jaw_speed_mm_s=body.get('jaw_speed_mm_s',10.))])
                preview=device.submit(business,command)
            self.add_review(device,business,preview,'manual_execute',
                f"人工{'队列' if action=='manual_queue_preview' else '姿态'}预览：{len(preview.get('plans',[]))} 个路径点，{preview['total_motion_duration_s']:.2f} 秒；按冻结的关节/夹爪速度整体执行")
            return {'message':'人工姿态已预览，等待明确审阅执行'}
        if action=='recover':
            state=device.snapshot(business)
            return device.submit(business,dict(action='recover',confirmation='ACKNOWLEDGE_MOTION_FAULT',
                owner_pid=state['pid'],command_epoch=state['command_epoch']))
        workflow=device.workflows.get(business)
        if workflow is None:raise ValueError('当前业务后端尚未配置')
        if business=='calibration' and action in (*CALIBRATION_TOOLS,'generate_plan'):
            gate=device.snapshot(business)['allowed_operations']['preview']
            if not gate['allowed']:raise ValueError(gate['reason'] or '当前设备不允许更改标定任务')
        result=workflow.action(action,body)
        if isinstance(result,dict) and result.get('preview'):
            self.add_review(device,business,result['preview'],'execute',result['summary'])
        if action=='install':self.invalidate(device.environment)
        return result

    def add_review(self, device, business, preview, action, summary):
        state=device.snapshot(business)
        workflow=device.workflows.get(business)
        manual_revision=(preview.get('review_generation',workflow.manual_revision())
                         if action=='manual_execute' and workflow and hasattr(workflow,'manual_revision') else None)
        with self.lock:
            generation=preview.get('workflow_generation')
            if generation is not None and (workflow is None or generation!=workflow.generation):return
            if business=='collection' and device.collection_preparation:
                if device.collection_preparation.snapshot()['status']!='ready':return
            review=dict(id=secrets.token_hex(16),action=action,
                token=preview['token'],summary=summary,physical_clearance_required=device.environment=='physical',
                speed_scale=preview.get('speed_scale') if business=='collection' else None,
                owner_preview=bool(preview.get('owner_pid')),
                manual_revision=manual_revision,
                workflow_revision=workflow.manual_revision() if business=='calibration' and workflow and hasattr(workflow,'manual_revision') else None,
                workflow_generation=generation,
                deadline=time.monotonic()+min(30.,preview.get('expires_in_s',30.)),
                runtime_nonce=preview.get('runtime_nonce',state['runtime_nonce']),epoch=preview['command_epoch'],
                calibration_revision=state.get('calibration_revision'))
            self.reviews[(device.environment,business)]=review
            if action=='manual_execute':
                self.manual_tasks[(device.environment,business)]=dict(review,
                    plan_sha256=[plan['sha256'] for plan in preview.get('plans',[])])


def handler(app):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'
        timeout = 5
        disable_nagle_algorithm = True
        def log_message(self,*args):pass
        def send(self,code,value,kind='application/json',headers=None):
            raw=json.dumps(value,ensure_ascii=False,allow_nan=False).encode() if kind=='application/json' else value
            self.send_response(code)
            self.send_header('Content-Type',kind)
            if code!=304:self.send_header('Content-Length',str(len(raw)))
            self.send_header('Cache-Control','no-store')
            self.send_header('X-Content-Type-Options','nosniff')
            if self.close_connection:self.send_header('Connection','close')
            for key,value in (headers or {}).items():self.send_header(key,str(value))
            self.end_headers()
            try:self.wfile.write(raw)
            except (BrokenPipeError,ConnectionResetError):pass
        def valid_host(self):
            return self.headers.get('Host') in self.server.allowed_hosts
        def do_GET(self):
            if not self.valid_host():
                self.close_connection=True;return self.send(403,{'error':'invalid Host'})
            if self.headers.get('Transfer-Encoding') is not None or self.headers.get_all('Content-Length',[]) not in ([],['0']):
                self.close_connection=True;return self.send(409,{'error':'GET request body is not supported'})
            path=urlsplit(self.path);query=parse_qs(path.query)
            try:
                if path.path=='/api/state':
                    return self.send(200,app.state(query.get('environment',['fake'])[0],query.get('business',['calibration'])[0]))
                if path.path=='/api/preview-media':
                    exact={}
                    if 'revision' in query:
                        exact=dict(revision=int(query['revision'][0]),display_context=query.get('display_context',[None])[0],
                            renderer_id=query.get('renderer_id',[None])[0],view_revision=int(query.get('view_revision',['-1'])[0]),wait_ms=int(query.get('wait_ms',['0'])[0]))
                    raw,meta=app.preview_media(query.get('environment',['fake'])[0],query.get('view',['target'])[0],**exact)
                    return self.send(200,raw,'image/jpeg',{'X-Frame-Metadata':json.dumps(meta,allow_nan=False,separators=(',',':'))})
                if path.path=='/api/calibration-file':
                    workflow=app.device(query.get('environment',['fake'])[0]).workflows.get('calibration')
                    if not workflow:raise ValueError('标定服务未配置')
                    return self.send(200,workflow.download(query.get('result_id',[''])[0]),
                        headers={'Content-Disposition':'attachment; filename="calibration.json"'})
                if path.path=='/api/result-media':
                    device=app.device(query.get('environment',['fake'])[0])
                    business=query.get('business',['calibration'])[0]
                    if business not in BUSINESSES:raise ValueError('未知业务')
                    workflow=device.workflows.get(business)
                    result_id=query.get('result_id',[''])[0]
                    file=workflow.media(result_id) if workflow else None
                    if file is None:return self.send(404,{'error':'本轮媒体尚未就绪'})
                    from robot_workbench.http_media import serve_file
                    kind=mimetypes.guess_type(str(file))[0]
                    current_plan=(kind=='application/json' and business=='collection'
                        and (workflow.snapshot().get('trajectory') or {}).get('id')==result_id)
                    if kind not in ('video/mp4','image/png','image/jpeg') and not current_plan:raise ValueError('不支持的回放媒体类型')
                    return serve_file(self,file,content_type=kind)
                if path.path.startswith('/api/media/'):
                    device=app.device(query.get('environment',['fake'])[0])
                    name=path.path.removeprefix('/api/media/')
                    if name not in device.media:return self.send(404,{'error':'unknown media'})
                    raw,meta=device.media[name].frame()
                    headers={'ETag':meta['etag'],'X-Frame-Id':meta['frame_id'],'X-Source-Wall-Ns':meta['source_wall_ns']}
                    if self.headers.get('If-None-Match')==meta['etag']:return self.send(304,b'','image/jpeg',headers)
                    return self.send(200,raw,'image/jpeg',headers)
                resource=path.path.removeprefix('/web') if path.path.startswith('/web/') else path.path
                if resource not in ('/','/index.html','/app.js','/style.css'):
                    return self.send(404,{'error':'not found'})
                file=ROOT/'web'/('index.html' if resource=='/' else resource[1:])
                return self.send(200,file.read_bytes(),mimetypes.guess_type(str(file))[0] or 'application/octet-stream')
            except (ValueError,KeyError,PermissionError) as error:self.send(409,{'error':str(error)})
            except Exception as error:self.send(503,{'error':str(error)})
        def do_POST(self):
            # A rejected or partial body must never become the next request on this socket.
            keep_alive=not self.close_connection;self.close_connection=True
            if (not self.valid_host() or self.headers.get('Origin') not in (None,'http://'+self.headers['Host'])
                    or not secrets.compare_digest(self.headers.get('X-CSRF-Token',''),app.csrf)):
                return self.send(403,{'error':'invalid request origin or token'})
            try:
                lengths=self.headers.get_all('Content-Length',[])
                if self.headers.get('Transfer-Encoding') is not None or len(lengths)!=1 or not lengths[0].isascii() or not lengths[0].isdigit():
                    raise ValueError('one explicit Content-Length and no Transfer-Encoding required')
                size=int(lengths[0])
                if size>4*1024*1024:return self.send(413,{'error':'small JSON required'})
                if size<=0:raise ValueError('small JSON required')
                path=urlsplit(self.path).path
                if not path.startswith('/api/actions/'):return self.send(404,{'error':'not found'})
                raw=self.rfile.read(size)
                if len(raw)!=size:raise ValueError('request body is incomplete')
                self.close_connection=not keep_alive
                result=app.action(path.removeprefix('/api/actions/'),json.loads(raw))
                self.send(202 if result.get('job_id') else 200,result)
            except (ValueError,KeyError,TypeError,PermissionError) as error:self.send(409,{'error':str(error)})
            except Exception as error:self.send(503,{'error':str(error)})
    return Handler


def loopback_browser_hosts(browser_ports):
    if (not isinstance(browser_ports,(list,tuple)) or
            any(type(port) is not int or not 1<=port<=65535 for port in browser_ports)):
        raise ValueError('browser_ports must contain explicit integer loopback ports in 1..65535')
    return {f'{host}:{port}' for port in browser_ports for host in ('127.0.0.1','localhost')}


def serve(app, host='127.0.0.1', port=8120, *, browser_ports=()):
    if host not in ('127.0.0.1','localhost'):
        raise ValueError('external binding requires a separate reviewed deployment entry')
    browser_hosts=loopback_browser_hosts(browser_ports)
    server=ThreadingHTTPServer((host,port),handler(app))
    server.daemon_threads=True
    server.allowed_hosts={f'127.0.0.1:{server.server_port}',f'localhost:{server.server_port}'}|browser_hosts
    return server


def protocol_fake(root):
    """Explicit L1 developer option: ideal feedback, no images or physics claims."""
    import sys
    sys.path[:0]=[str(ROOT/'calibration'),str(ROOT/'calibration/infra'),str(ROOT/'rule_based_workbench')]
    from examples.scripts.debug import joint_head_real_owner as ref
    from examples.scripts.debug.joint_head_fake_hardware import FakePlant,FakeSafety
    from arx_env.joint_head_jaw_mapping import command_sdk
    from robot_workbench.runtime import Runtime
    def factory():
        origin=time.monotonic()
        clock=lambda:time.monotonic()-origin
        clock_id='protocol-'+secrets.token_hex(16)
        class ProtocolPlant(FakePlant):
            def read_calibration_state(self):
                state=super().read_calibration_state()
                now=clock();stamp=round(now*1e9)
                state.update(sample_time_s=now,sdk_timestamps=[now,now+41.],
                    environment='fake',simulation_only=True,clock_domain='simulation',source_clock_id=clock_id,
                    source_time_ns=stamp,physics_step=stamp,step_kind='protocol_clock_tick',evidence_source='fake')
                return state
        limits=[[-3.14,3.14]]*6+[[0.,command_sdk('left',83.)]]+[[-3.14,3.14]]*6+[[0.,command_sdk('right',83.)]]
        return Runtime(ref,FakeSafety,lambda:ProtocolPlant(dict(q=[0.]*12,jaw_gap_mm=[80.,80.]),limits),
            device_id='protocol-fake',environment='fake',lock_root=root,
            control_clock=clock,
            initialize_backend=lambda backend:backend.set_damping_hold(manual=True))
    return Device('protocol-fake','fake',factory,label='协议 Fake · 无物理仿真图像')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',type=int,default=8120)
    parser.add_argument('--state-dir',type=Path,required=True)
    parser.add_argument('--protocol-fake',action='store_true',help='explicit L1 control/UI check; no calibration or PhysX qualification')
    args=parser.parse_args()
    if not args.protocol_fake:parser.error('use an explicitly configured device adapter, or --protocol-fake for L1 only')
    app=Application({'fake':protocol_fake(args.state_dir),
                     'physical':Device('physical-unconfigured','physical',None,label='真机 · 未配置')})
    server=serve(app,port=args.port)
    print(f'Robot Workbench: http://127.0.0.1:{server.server_port}',flush=True)
    try:server.serve_forever()
    finally:
        server.server_close()
        # No implicit physical release. This CLI owns only its explicit L1 fake.
        device=app.devices['fake']
        if device.runtime:
            runtime=device.runtime
            if not runtime.release_receipt or runtime.release_receipt.get('sdk_close_returned'):
                runtime.owner.submit({'action':'release','confirmation':'ARMS_SUPPORTED_RELEASE'})
            device.closed.wait(.05)
            device.closed.set()


if __name__=='__main__':main()
