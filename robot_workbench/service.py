"""Shared, loopback-first HTTP service. No robot connection occurs on page load."""
import argparse
from collections import OrderedDict
from copy import deepcopy
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import mimetypes
import re
from pathlib import Path
import secrets
import threading
import time
import uuid
from urllib.parse import parse_qs, urlsplit,urlencode
from robot_workbench.workflows import CALIBRATION_TOOLS,COLLECTION_TOOLS,HISTORY_ACTIONS,CalibrationWorkflow,collection_live_view
from robot_workbench.joint_bounds import position_noise_rad, project_joint_positions
from robot_workbench.sample_clock import host_sample_status
from robot_workbench.inference_owner import episode_duration
from robot_workbench.inference_workflow import InferenceWorkflow

ROOT = Path(__file__).resolve().parents[1]
BUSINESSES = ('calibration', 'collection', 'inference', 'research')
INFERENCE_MODEL_TOOLS = ('inference_refresh','inference_select','inference_prepare')
INFERENCE_TOOLS = (*INFERENCE_MODEL_TOOLS, 'inference_preview')
WARM_C_ACTIONS = ('generate_plan','execute','install',*CALIBRATION_TOOLS)
WARM_C_REASON = '当前为复用标定的数采诊断；完整标定请使用空库验收入口'


def frozen_collection_scope(device):
    """Only the existing successful frozen restore supplies this server-side scope."""
    playground=getattr(device.workflows.get('collection'),'playground',None)
    if device.environment!='physical' or playground is None:return False
    with playground.lock:
        environment=playground.state.get('environment') or {}
        return environment.get('ready') is True and environment.get('scope')=='frozen_plan_and_current_owner_only'


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
        self.stop_serial = 0
        self.connecting = False
        self.release_receipt = None
        self.maintenance = False
        self.capture_results=None

    def configure_capture_results(self,root):
        from .collection_history import CaptureResults
        if self.capture_results is not None:raise ValueError('capture result root already bound')
        self.capture_results=CaptureResults(root,environment=self.environment,device_id=self.id)
        if self.runtime is not None:self._bind_capture_results(self.runtime)

    def _bind_capture_results(self,runtime):
        if self.capture_results is None:return
        callback=self.capture_results.notify
        if callable(getattr(runtime,'set_capture_result_handler',None)):runtime.set_capture_result_handler(callback)
        else:runtime.capture_result_handler=callback

    def connect(self, body, *, active=None,maintenance_actor=None):
        if getattr(getattr(self,'_operations_application',None),'operations_managed',False) and maintenance_actor is None:
            raise ValueError('operation_required：设备接入必须通过运维维护窗口执行')
        if self.environment == 'physical' and body.get('arms_supported') is not True:
            raise ValueError('连接前请确认此刻双臂已可靠支撑')
        if not self.lock.acquire(blocking=False):
            raise ValueError('设备连接正在准备')
        try:
            if self.maintenance:
                from .maintenance import ControlActor
                if not isinstance(maintenance_actor,ControlActor) or maintenance_actor.action!='connect_control':
                    raise ValueError('服务维护中，不能连接设备')
                maintenance_actor.require(self.runtime)
            elif maintenance_actor is not None:raise ValueError('受限管理接入必须持有维护窗口')
            self.connecting = True
            retirement=None
            if self.runtime is not None:
                if (maintenance_actor is None or self.environment!='physical'
                        or not getattr(self.runtime.owner.backend,'terminal_failure',None)):
                    raise ValueError('设备已连接；控制权交接不会断开设备')
                maintenance_actor.require(self.runtime)
                remote=getattr(self.runtime,'remote_control_owner',False)
                if remote:self.closed.set()  # Stop only this business fact observer.
                if self.thread is not None and self.thread.is_alive():self.thread.join(timeout=2)
                if self.thread is not None and self.thread.is_alive():raise ValueError('保护后的控制线程仍在收尾')
                retirement=self.runtime.retire_protected()
                if remote:self.runtime.detach()
            if self.thread is not None and self.thread.is_alive():
                raise ValueError('上次设备连接仍在收尾，请等待断开确认')
            if self.factory is None:
                raise ValueError('设备控制后端尚未配置')
            current=lambda:(active is None or active()) and (maintenance_actor is None or maintenance_actor.current())
            if not current():raise ValueError('本次连接请求已撤销')
            if maintenance_actor is not None:self.stop_latched=False
            runtime = self.factory()
            runtime.prior_control_retirement=retirement
            if maintenance_actor is not None:
                if getattr(runtime,'remote_control_owner',False):
                    try:runtime.begin_maintenance()
                    except Exception as error:
                        # Acquisition already succeeded in the independent Owner.
                        # Keep its authoritative state visible even when the next
                        # management step fails; never report it as disconnected.
                        self._attach_existing_owner_locked(runtime)
                        self.error='SDK仍连接，接入维护未完成；已请求停止：'+str(error)
                        try:maintenance_actor.adopt(runtime)
                        except Exception as adopt_error:self.error+='；维护实例绑定待核验：'+str(adopt_error)
                        try:runtime.request_stop()
                        except Exception as stop_error:self.error+='；停止结果未确认：'+str(stop_error)
                        raise ValueError(self.error)from error
                else:
                    with runtime.lock:runtime.maintenance_active=True
            installed = (self.release_receipt or {}).get('calibration_snapshot_id')
            if installed and runtime.calibration_snapshot_id is None:
                runtime.notify_calibration_installed(installed)
            credentials = {name: runtime.register_business(name) for name in BUSINESSES}
            self.credentials = credentials
            self.runtime = runtime
            self._bind_capture_results(runtime)
            self.release_receipt = None
            self.error = None
            if maintenance_actor is not None:maintenance_actor.adopt(runtime)
            if self.stop_latched or not current():
                self.stop_latched = True
                runtime.submit(credentials[runtime.holder], {'action':'stop'})
            self.closed.clear()
            if getattr(runtime,'remote_control_owner',False):
                self._start_remote_monitor(runtime)
                return {'message':'已接入独立Owner；业务进程仅订阅控制状态，尚未执行运动'}
            def loop():
                probe=None
                try:
                    if self.drive is not None:
                        self.drive(runtime, self.closed)
                        return
                    import gc
                    probe=runtime.owner.timing_probe
                    probe.thread_id=threading.get_ident()
                    gc.callbacks.append(probe.gc_callback)
                    while not self.closed.is_set():
                        started = time.monotonic()
                        probe.begin(runtime.owner.event)
                        if not probe.call('runtime_tick',runtime.tick):
                            break
                        probe.call('idle_wait',self.closed.wait,max(0., .01-(time.monotonic()-started)))
                except Exception as error:
                    self.error = str(error)
                    runtime.submit(self.credentials[runtime.holder], {'action': 'stop'})
                finally:
                    if probe is not None:
                        probe.begin(runtime.owner.event);probe.flush(runtime.owner.event)
                        if probe.gc_callback in gc.callbacks:gc.callbacks.remove(probe.gc_callback)
                    if runtime.release_receipt is not None:
                        try:
                            if runtime.fd is not None:
                                runtime.close()
                            self.closed.set()
                            with self.lock:
                                self.release_receipt = deepcopy(runtime.release_receipt)
                                if self.runtime is runtime:
                                    self.runtime = None
                                    self.credentials = {}
                            if self.capture_results is not None:self.capture_results.notify()
                        except Exception as error:
                            runtime.release_receipt.update(status='unconfirmed', message='连接收尾未确认：'+str(error))
                        self.release_receipt = deepcopy(runtime.release_receipt)
            self.thread = threading.Thread(target=loop, daemon=True, name='device-owner-'+self.environment)
            self.thread.start()
            return {'message': '设备已连接，等待健康反馈；尚未执行运动'}
        finally:
            self.connecting = False
            self.lock.release()

    def _start_remote_monitor(self,runtime):
        def monitor():
            try:runtime.monitor(self.closed)
            except Exception as error:
                self.error='独立Owner状态读取失败：'+str(error)
            finally:
                receipt=runtime.release_receipt
                if receipt and receipt.get('status')=='released':
                    try:runtime.end_maintenance()
                    except Exception as error:
                        self.error='SDK已释放，但维护租约清理未确认：'+str(error)
                        self.release_receipt=dict(receipt,maintenance_cleanup_confirmed=False)
                        return  # Retain the client/capability; do not admit another acquire.
                    runtime.detach()
                    self.release_receipt=dict(deepcopy(receipt),maintenance_cleanup_confirmed=True);self.closed.set()
                    with self.lock:
                        if self.runtime is runtime:self.runtime=None;self.credentials={}
                    if self.capture_results is not None:self.capture_results.notify()
        self.thread=threading.Thread(target=monitor,daemon=True,name='control-facts-'+self.environment)
        self.thread.start()

    def attach_existing_owner(self,runtime):
        """Read-only attachment after business restart; never acquire/re-enable SDK."""
        if not getattr(runtime,'remote_control_owner',False)or runtime.nonce is None:raise ValueError('existing independent Owner required')
        with self.lock:
            if self.runtime is not None:raise ValueError('business already attached to an Owner')
            self._attach_existing_owner_locked(runtime)

    def _attach_existing_owner_locked(self,runtime):
        self.runtime=runtime;self.credentials={name:runtime.register_business(name)for name in BUSINESSES}
        self._bind_capture_results(runtime)
        self.error=None;self.closed.clear();self._start_remote_monitor(runtime)

    def submit(self, business, command):
        if command.get('action') == 'stop':
            if command!={'action':'stop'}:raise ValueError('explicit stop command required')
            return self.request_stop()
        if getattr(getattr(self,'_operations_application',None),'operations_managed',False) and command.get('action') in ('release','recover'):
            raise ValueError('operation_required：设备释放与恢复必须通过运维维护窗口执行')
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

    def request_stop(self):
        self.stop_latched=True;self.stop_serial+=1
        runtime=self.runtime
        if runtime is None:return dict(message='设备未连接，没有可发送的运动命令')
        return runtime.request_stop()

    def fact_snapshot(self):
        value=self.snapshot(None);value.pop('allowed_operations',None)
        ready=self.runtime is None and not self.connecting and not (self.thread and self.thread.is_alive())
        value.setdefault('control_operations',{})['connect']=dict(allowed=bool(ready and self.factory is not None and not self.maintenance),
            reason='服务维护中' if self.maintenance else None if ready and self.factory is not None else '设备控制后端未配置或上次连接仍在收尾')
        value['observed_wall_ns']=time.time_ns()
        return value

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
            result = runtime.fact_snapshot() if business is None else runtime.snapshot(credential)
        except PermissionError:
            if runtime.fd is not None:
                raise
            return dict(connected=False,state='DISCONNECTED',holder=None,feedback_healthy=False,
                release_receipt=deepcopy(runtime.release_receipt),
                allowed_operations={key:dict(allowed=key=='stop',reason='设备连接正在收尾')
                                    for key in ('stop','preview','confirm','handoff')})
        except (ConnectionError,ValueError)as error:
            if not getattr(runtime,'remote_control_owner',False):raise
            return dict(connected=None,state='UNKNOWN',holder=None,feedback_healthy=False,stationary=False,
                automatic_enabled=None,runtime_nonce=runtime.nonce,service_error='独立Owner当前不可核验：'+str(error),
                allowed_operations={key:dict(allowed=key=='stop',reason='独立Owner当前不可核验；请检查运维状态')
                                    for key in('stop','preview','confirm','handoff')})
        receipt = result.get('release_receipt')
        if receipt and receipt['status']=='requested' and time.monotonic()-receipt['requested_monotonic_s']>=2:
            receipt.update(status='unconfirmed',message='释放超过 2 秒仍未确认；请查看设备诊断，不能重新连接')
        result['sample_wall_ns'] = (result.get('calibration_feedback') or {}).get('sample_wall_ns')
        result['motion_status'] = (result.get('motion') or {}).get('status')
        if result.get('motion') is None and result.get('connected') and result.get('automatic_enabled') is False:
            result['motion_status'] = 'no_automatic_task'
        if self.error:
            result['service_error'] = self.error
        elif result.get('feedback_source_error'):
            result['service_error']='ROS反馈暂不可用：'+result['feedback_source_error']
        return result


class Application:
    def __init__(self, devices):
        if any(key not in ('fake','physical') or device.environment != key for key,device in devices.items()):
            raise ValueError('device environment namespace mismatch')
        self.devices = devices
        self.csrf = secrets.token_urlsafe(32)
        self.server_instance_id = str(uuid.uuid4())  # Public lifetime identity, not an authorization token.
        self.lock = threading.RLock()
        self.policy_write_lock = threading.Lock()
        self.revision = 0
        self.display_actions=threading.Lock()
        self.preview_contexts = {}
        self.jobs, self.requests, self.reviews = {}, OrderedDict(), {}
        self.manual_tasks = {}
        self.generations = {environment: 0 for environment in devices}
        self.inference_generations = {environment: 0 for environment in devices}
        self.maintenance = {}
        self.action_calls = {environment:0 for environment in devices}
        self.operations_managed = False  # Trusted deployment assembly enables the operations-only lifecycle.
        self.operator_tasks={}
        self.operator_focus={}
        for device in devices.values():device._operations_application=self

    def _operator_focus_action(self,device,workflow,body):
        fields={'environment','business','request_id','runtime_nonce','epoch','client_id','active'}
        if set(body)!=fields or type(body['active'])is not bool or not isinstance(body['client_id'],str)or not 0<len(body['client_id'])<=128:
            raise ValueError('exact foreground lease required')
        with self.lock:
            previous=self.operator_focus.get(device.environment,{})
            if not body['active']:
                if previous.get('client_id')==body['client_id']:
                    self.operator_focus.pop(device.environment,None);workflow.operator_pedal_gesture=None
                return dict(active=False)
            state=device.snapshot('inference');runtime=device.runtime
            if runtime is None or runtime.nonce!=body['runtime_nonce']or state['command_epoch']!=body['epoch']:
                raise ValueError('foreground lease requires the current Owner identity')
            same=(previous.get('client_id')==body['client_id'] and previous.get('runtime_nonce')==runtime.nonce
                and previous.get('expires',0)>time.monotonic())
            if not same:workflow.operator_pedal_gesture=None
            self.operator_focus[device.environment]=dict(client_id=body['client_id'],runtime_nonce=runtime.nonce,
                started_wall=previous['started_wall']if same else time.time(),
                expires=time.monotonic()+workflow.operator_controls['focus_timeout_s'])
        return dict(active=True)

    def _operator_model_active(self,device,state):
        mode=state.get('inference')or{}
        job=self.jobs.get((device.environment,'inference'),{})
        return (mode.get('mode')=='MODEL' or mode.get('transition_stage')in('RESUMING','MODEL_ADMITTED')
            or job.get('action')=='inference_run'and job.get('status')=='running')

    def _operator_pedal_action(self,device,workflow,command,reference,occurred_at,focus):
        last=getattr(workflow,'operator_pedal_reference',None)
        if last and last[0]==reference.source_instance_id and reference.sequence<=last[1]:return dict(accepted=False)
        workflow.operator_pedal_reference=(reference.source_instance_id,reference.sequence)
        state=device.snapshot('inference');mode=state.get('inference')or{}
        active=self._operator_model_active(device,state)
        session=getattr(workflow,'operator_session',None);run=getattr(workflow,'current',None)
        held=bool(session and run and run['id']==session['run_id']and run.get('ended_wall')is None
            and session['runtime_nonce']==state.get('runtime_nonce') and state.get('holder')=='inference'
            and state.get('feedback_healthy')and state.get('stationary')and not state.get('recoverable_fault')
            and mode.get('runner_status')=='RUNNING'and mode.get('mode')=='HOLD'
            and not mode.get('transition_stage')and not active)
        context=(reference.source_instance_id,focus.get('client_id'),focus.get('started_wall'),
            state.get('runtime_nonce'),session.get('run_id')if session else None)
        gesture=getattr(workflow,'operator_pedal_gesture',None)
        body=dict(environment=device.environment,business='inference',
            request_id='pedal-'+reference.source_instance_id.replace(':','_')+'-'+str(reference.sequence),
            runtime_nonce=device.runtime.nonce,epoch=state['command_epoch'])
        if command==11:
            # evdev occurred_at has microsecond resolution; compare in that
            # domain so the inclusive window edge does not depend on float ULPs.
            second=bool(gesture and gesture['context']==context
                and 0<round((occurred_at-gesture['occurred_at'])*1_000_000)
                    <=round(workflow.operator_controls['double_press_window_s']*1_000_000))
            if second:
                workflow.operator_pedal_gesture=None
                if not(gesture['eligible']and held and gesture['epoch']==state['command_epoch']):return dict(accepted=False)
                view=workflow.snapshot().get('inference')or{}
                if view.get('model_ready')is not True:raise ValueError('当前模型尚未就绪，双踩未启动')
                request={key:body[key]for key in ('environment','business','request_id')}
                request.update({key:session[key]for key in ('seed','duration_s','model_id','expected_bundle_digest')})
                request['wam_stall_recovery']=session.get('wam_stall_recovery',False)
                request['recording_enabled']=session.get('recording_enabled',False)
                request['memory_start']='continue'
                request.update(expected_context=self.inference_context(device,state,workflow),state_wall_ns=time.time_ns(),
                    physical_clearance=True,scene_reset=True,arms_supported=True)
                return self.action('inference_run',request)
            workflow.operator_pedal_gesture=dict(context=context,occurred_at=occurred_at,
                eligible=held,epoch=state['command_epoch'])
            return self.action('inference_hold',body)
        workflow.operator_pedal_gesture=None
        if command==22:return self.action('inference_hold'if active else'inference_teach',body)
        if command==33:
            if active:return dict(accepted=False)
            body['physical_clearance']=True
            return self.action('inference_home',body)
        return dict(accepted=False)

    def install_operator_pedal(self,device,workflow,descriptor):
        """Bind the declared event source; never read evdev in the business process."""
        from .ros_data import RosNode,SourceDirectory,source_descriptor,json_value
        closed=threading.Event();workflow.operator_input_closed=closed
        def work():
            node=None
            try:
                plane=workflow.ros_plane
                node=RosNode('infer_pedal_'+device.id,namespace=plane['namespace'],domain_id=plane['domain_id'],localhost_only=False)
                declared=source_descriptor(json.loads(Path(descriptor).read_text()))
                if declared['device_id']!=device.id or declared['stream_id']!='pedal'or declared['kind']!='event':
                    raise ValueError('registered pedal belongs to another device/source role')
                directory=SourceDirectory(node,device.id)
                source=directory.resolve({'pedal':dict(source_instance_id=declared['source_instance_id'],kind='event')})[0]
                publishers=node.node.get_publishers_info_by_topic(source['topic'])
                if len(publishers)!=1:raise ValueError('one pedal publisher required')
                publisher_gid=bytes(publishers[0].endpoint_gid)
                workflow.operator_pedal_source=source;workflow.operator_input_status=dict(state='waiting',source_instance_id=source['source_instance_id'])
                workflow.operator_last_seen=time.monotonic()
                def receive(message):
                    ref=message.reference;value=json_value(message.payload_json)
                    if (ref.source_instance_id!=source['source_instance_id']or ref.stream_id!='pedal'
                            or directory.values.get('pedal')!=source):return
                    publishers=node.node.get_publishers_info_by_topic(source['topic'])
                    if len(publishers)!=1 or bytes(publishers[0].endpoint_gid)!=publisher_gid:
                        workflow.operator_pedal_gesture=None
                        workflow.operator_input_status=dict(state='unavailable',error='原踏板发布者已变化或存在多个发布者')
                        return
                    if not 0<=time.time()-ref.source_time_ns/1e9<=workflow.operator_controls['event_max_age_s']:return
                    workflow.operator_last_seen=time.monotonic()
                    if message.event!='pedal_command':
                        if value.get('connected')is False:
                            workflow.operator_pedal_gesture=None
                            workflow.operator_input_status=dict(state='unavailable',error='踏板已断开，等待同一踏板重连')
                            return
                        if workflow.operator_input_status.get('state')in('waiting','unavailable'):
                            workflow.operator_input_status=dict(state='ready',source_instance_id=source['source_instance_id'])
                        return
                    occurred=value.get('occurred_at')
                    if type(occurred)not in(int,float):return
                    age=time.time()-occurred
                    if not 0<=age<=workflow.operator_controls['event_max_age_s']:return
                    with self.lock:focus=deepcopy(self.operator_focus.get(device.environment,{}))
                    runtime=device.runtime
                    if (not focus or focus['expires']<time.monotonic()or runtime is None
                            or runtime.nonce!=focus['runtime_nonce']or occurred<focus['started_wall']):return
                    command=value.get('command')
                    if command not in(11,22,33):return
                    try:
                        result=self._operator_pedal_action(device,workflow,command,ref,occurred,focus)
                        workflow.operator_input_status=dict(state='ready',last_command=command,source_time_ns=ref.source_time_ns,
                            sequence=ref.sequence,request_id='pedal-'+ref.source_instance_id.replace(':','_')+'-'+str(ref.sequence),
                            accepted=result.get('accepted',True))
                    except Exception as error:workflow.operator_input_status=dict(state='rejected',last_command=command,error=str(error))
                node.subscribe(source['topic'],node.types.DataEvent,receive)
                while not closed.wait(.1):
                    if directory.values.get('pedal')!=source:
                        workflow.operator_pedal_gesture=None
                        raise ValueError('原踏板来源已停止或身份变化，请重新绑定')
                    if time.monotonic()-workflow.operator_last_seen>workflow.operator_controls['event_max_age_s']:
                        workflow.operator_pedal_gesture=None
                        workflow.operator_input_status=dict(state='unavailable',error='踏板 ROS 心跳中断，请检查踏板连接与来源服务')
            except Exception as error:workflow.operator_input_status=dict(state='unavailable',error=str(error))
            finally:
                if node is not None:node.close()
        workflow.operator_input_thread=threading.Thread(target=work,daemon=True,name='infer-pedal-consumer')
        workflow.operator_input_thread.start()

    def _operator_action(self,device,workflow,action,body):
        fields={'environment','business','request_id','runtime_nonce','epoch'}
        if action=='inference_home':fields.add('physical_clearance')
        if set(body)!=fields or body['business']!='inference':raise ValueError('exact operator intent and current Owner identity required')
        runtime=device.runtime
        if runtime is None or runtime.nonce!=body['runtime_nonce']:raise ValueError('operator Owner changed')
        state=device.snapshot('inference')
        if state['command_epoch']!=body['epoch']or state['holder']!='inference':raise ValueError('operator epoch or control ownership changed')
        if state.get('recoverable_fault')or not state.get('feedback_healthy'):raise ValueError('operator action cannot recover a control fault')
        if action=='inference_memory_clear':
            mode=(workflow.snapshot().get('inference')or{}).get('normal_mode')or{}
            if (mode.get('mode')!='HOLD'or mode.get('transition_stage')
                    or self.operator_tasks.get(device.environment,{}).get('status')=='running'):
                raise ValueError('请先进入 HOLD 并等待当前操作结束，再清空记忆')
            return workflow.clear_memory(body['request_id'],body['epoch'])
        if action=='inference_home'and body['physical_clearance']is not True:raise ValueError('home requires current workspace clearance')
        if self._operator_model_active(device,state):
            if action=='inference_home':return dict(accepted=False)
            if action=='inference_teach':action='inference_hold'
        if action=='inference_teach':
            return self._operator_teach(device,workflow,body)
        if action=='inference_home':
            return self._operator_native_home(device,workflow,body)
        with self.lock:
            previous=self.operator_tasks.get(device.environment,{})
            if previous.get('kind')=='teach'and previous.get('status')=='running':
                previous['finish_request_id']=body['request_id']
                workflow.operator_finish.set()
                if action=='inference_hold':return dict(accepted=True,operator_task=deepcopy(previous))
            job=self.jobs.get((device.environment,'inference'),{})
            if job.get('status')=='running'and job.get('action')=='inference_run':
                job['operator_stop_request_id']=body['request_id']
        previous=self.operator_tasks.get(device.environment,{})
        if previous.get('kind')=='home'and previous.get('status')=='running':
            return dict(accepted=True,waiting_native_home=True,operator_task=deepcopy(previous))
        return workflow.request_operator_stop(body['request_id'])

    def _operator_native_home(self,device,workflow,body):
        session=getattr(workflow,'operator_session',None)
        run=workflow.current
        if not session or run is None or run['id']!=session['run_id']or run.get('ended_wall')is not None:
            raise ValueError('请先准备控制与踏板，再执行原生回零')
        with self.lock:
            previous=self.operator_tasks.get(device.environment,{})
            if previous.get('status')=='running':raise ValueError('等待当前踏板操作完成')
            task=dict(request_id=body['request_id'],run_id=run['id'],kind='home',status='running',
                phase='waiting_inference_hold',runtime_nonce=device.runtime.nonce)
            self.operator_tasks[device.environment]=task;workflow.operator_task=task
            run['operator_positioning']=task
        runtime=device.runtime;generation=self.generations[device.environment]
        deadline=time.monotonic()+workflow.operator_controls['timeout_s']
        def current():
            if (device.runtime is not runtime or self.generations[device.environment]!=generation
                    or workflow.current is not run):raise ValueError('原生回零已撤销')
            mode=run['runner'].snapshot()
            if mode.get('runner_status')in('STOPPED','FAULT'):raise ValueError(mode.get('reason')or'原生回零控制已停止')
            if time.monotonic()>deadline:raise TimeoutError('原生回零完成回执超时')
            return mode
        def request(target):
            mode=current();key=uuid.uuid5(uuid.NAMESPACE_URL,run['id']+'/'+body['request_id']+'/'+target).hex
            workflow.request_normal_mode(dict(session_id=mode['mode_session_id'],runtime_nonce=runtime.nonce,
                request_id=key,epoch=mode['command_epoch'],mode=target,target=target))
            while True:
                mode=current();receipt=mode.get('mode_receipts',{}).get(key,{})
                if receipt.get('status')in('failed','rejected'):raise ValueError(receipt.get('error')or'原生回零请求被拒绝')
                if receipt.get('completed'):return receipt
                time.sleep(.01)
        def work():
            try:
                mode=current()
                if mode.get('mode')!='HOLD'or mode.get('transition_stage')=='RESUMING':request('HOLD')
                else:
                    while current().get('transition_stage'):time.sleep(.01)
                task['phase']='native_homing'
                receipt=request('HOME')
                task.update(status='completed',phase='holding',receipt=receipt,sdk_released=False)
            except Exception as error:
                if device.runtime is runtime and self.generations[device.environment]==generation:device.request_stop()
                task.update(status='failed'if self.generations[device.environment]==generation else'stopped',
                    phase='terminated',error=str(error),sdk_released=False)
        threading.Thread(target=work,daemon=True,name='infer-native-home').start()
        return dict(accepted=True,operator_task=deepcopy(task))

    def _operator_teach(self,device,workflow,body):
        """Positioning Run using the existing recorded HOLD/TEACH/HOLD/END chain."""
        environment=device.environment
        with self.lock:
            previous=self.operator_tasks.get(environment,{})
            if previous.get('status')=='running':
                if previous.get('kind')!='teach':
                    raise ValueError('请先停止或等待当前回零/保持操作完成')
                previous.setdefault('finish_request_id',body['request_id'])
                workflow.operator_finish.set()
                return dict(accepted=True,operator_task=deepcopy(previous))
            if device.maintenance or self.maintenance.get(environment):
                raise ValueError('工作台正在维护，请等待完成后设置人工起点')
            if workflow.normal_modes is None or workflow.normal_initial_mode!='HOLD':
                raise ValueError('当前控制配置尚未启用共享 TEACH/HOLD 模式')
            if workflow._model_gate()['ready'] is not True:
                raise ValueError('请先准备当前推理模型，再设置人工起点')
            session=getattr(workflow,'operator_session',None)
            if not session or workflow.current is None or workflow.current['id']!=session['run_id']or workflow.current.get('ended_wall')is not None:
                raise ValueError('请先准备控制与踏板，再切换拖动与保持')
            runtime=device.runtime
            generation=self.generations[environment]
            task=dict(id=body['request_id'],request_id=body['request_id'],action='inference_teach',
                kind='teach',status='running',phase='waiting_inference_hold',started_wall_ns=time.time_ns(),
                runtime_nonce=runtime.nonce)
            self.operator_tasks[environment]=task
            workflow.operator_task=task
            workflow.operator_finish=threading.Event()
            task['run_id']=session['run_id']
            workflow.current['operator_positioning']=task

        timeout=workflow.operator_controls['timeout_s']
        def current():
            return self.generations[environment]==generation and device.runtime is runtime and runtime.nonce==task['runtime_nonce']
        def normal_state():
            if not current():raise ValueError('人工定位已被停止或控制实例已变化')
            run=workflow.current
            if run is None or run['id']!=task['run_id']:raise ValueError('人工定位 Run 身份已变化')
            mode=run['runner'].snapshot()if run.get('runner')else{}
            if mode.get('runner_status')in('STOPPED','FAULT'):
                facts=device.snapshot('inference')
                raise ValueError(mode.get('reason')or(facts.get('motion')or{}).get('reason')or facts.get('reason')or'人工定位已停止')
            return mode
        def wait_mode(request_id,target):
            deadline=time.monotonic()+timeout
            while True:
                mode=normal_state()
                receipt=(mode.get('mode_receipts')or{}).get(request_id,{})
                if receipt.get('status')in('failed','rejected'):
                    raise ValueError(receipt.get('error')or receipt.get('reason')or'模式转换未通过')
                if receipt.get('completed')and mode.get('mode')==target and not mode.get('transition_stage'):
                    return receipt
                if time.monotonic()>=deadline:raise TimeoutError('人工定位模式回执等待超时')
                time.sleep(.05)
        def request_mode(target,request_id):
            mode=normal_state()
            request_id=uuid.uuid5(uuid.NAMESPACE_URL,task['run_id']+'/'+request_id+'/'+target).hex
            workflow.request_normal_mode(dict(session_id=mode['mode_session_id'],runtime_nonce=runtime.nonce,
                request_id=request_id,epoch=mode['command_epoch'],mode='HOLD'if target=='END'else target,target=target))
            return wait_mode(request_id,'HOLD'if target=='END'else target)
        def work():
            try:
                task['phase']='entering_teach'
                mode=normal_state()
                if mode.get('transition_stage')=='RESUMING':request_mode('HOLD',body['request_id']+'-hold')
                request_mode('TEACH',body['request_id']+'-teach')
                task['phase']='teaching'
                while not workflow.operator_finish.wait(.05):normal_state()
                task['phase']='locking_start'
                held=request_mode('HOLD',task['finish_request_id']+'-hold')
                task.update(status='completed',phase='start_ready',hold_reference_id=held['reference_id'],
                    finished_wall_ns=time.time_ns(),sdk_released=False)
            except Exception as error:
                if current():device.request_stop()
                task.update(status='failed'if current()else'stopped',phase='terminated',error=str(error),
                    finished_wall_ns=time.time_ns(),sdk_released=False)
        threading.Thread(target=work,daemon=True,name='infer-operator-teach').start()
        return dict(accepted=True,operator_task=deepcopy(task))

    def inference_context(self, device, runtime, workflow):
        return dict(server_instance_id=self.server_instance_id,device_id=device.id,
            connected=runtime.get('connected') is True,runtime_nonce=runtime.get('runtime_nonce'),
            command_epoch=runtime.get('command_epoch'),holder=runtime.get('holder'),
            calibration_revision=runtime.get('calibration_revision'),
            execution_instance_id=runtime.get('execution_instance_id'),
            execution_settings_revision=(runtime.get('execution_settings') or {}).get('revision'),
            workflow_generation=getattr(workflow,'generation',None),
            application_generation=self.generations[device.environment],
            inference_generation=self.inference_generations[device.environment])

    def triton_read(self, environment, job_id=None):
        client = getattr(self.device(environment), 'triton_preview', None)
        if client is None:
            if job_id is not None:raise ValueError('当前设备未配置Triton输出预览')
            return dict(configured=False, available=False, sources=[], models=[], reason='当前设备未配置Triton服务')
        return client.job(job_id) if job_id is not None else client.catalog()

    def triton_action(self, action, body):
        if not isinstance(body, dict) or body.get('business') != 'inference':
            raise ValueError('Triton操作仅属于推理输出预览')
        device = self.device(body.get('environment'))
        client = getattr(device, 'triton_preview', None)
        if client is None:raise ValueError('当前设备未配置Triton输出预览')
        runtime = device.snapshot('inference')
        with self.lock:
            if self.maintenance.get(device.environment) or device.maintenance or device.connecting:
                raise ValueError('设备维护中，暂不启动Triton任务')
        if runtime.get('automatic_enabled') or any(x.snapshot().get('busy') for x in device.workflows.values()):
            raise ValueError('请先结束当前设备动作或采集，再启动Triton任务')
        return client.start(action, {k:v for k,v in body.items() if k not in ('environment','business')})

    def inference_run_reason(self, device, workflow, runtime, *, ignore_job=None,
                             workflow_view=None,workflow_views=None,holder_runtime=None):
        if self.operator_tasks.get(device.environment,{}).get('status')=='running':
            return '请先完成当前位置保持与定位录制封存'
        if not workflow or not all(callable(getattr(workflow,name,None)) for name in ('bind_device','preview_episode','execute_review')):
            return '当前推理工作流尚未支持 Run，请使用分步诊断'
        if getattr(workflow,'control_mapping_pending',None):return workflow.control_mapping_pending
        value=workflow.snapshot() if workflow_view is None else workflow_view;inference=value.get('inference') or {}
        session=getattr(workflow,'operator_session',None)
        mode=inference.get('normal_mode')or{}
        held_session=bool(session and session['run_id']==value.get('id')
            and session['runtime_nonce']==runtime.get('runtime_nonce')
            and mode.get('runner_status')=='RUNNING'and mode.get('mode')=='HOLD'
            and not mode.get('transition_stage'))
        if (inference.get('input_evidence')or{}).get('error'):
            return '推理证据写入失败，请检查存储；不会静默继续新一轮推理'
        if not (inference.get('model_ready') or (inference.get('model') or {}).get('loaded')):
            return '先选择 checkpoint，并等待模型加载与预热完成'
        views=(flow.snapshot() for flow in device.workflows.values()) if workflow_views is None else workflow_views.values()
        if any(view.get('busy') and not(held_session and view.get('id')==value.get('id'))for view in views):
            return '业务任务正在处理，不能接管控制'
        with self.lock:
            if any(row.get('status')=='running' and row.get('id')!=ignore_job
                    for (environment,_),row in self.jobs.items() if environment==device.environment):
                return '当前设备仍有请求正在处理'
        if not runtime.get('connected'):
            if device.connecting or device.thread and device.thread.is_alive():return '设备连接正在准备或收尾'
            return '设备控制尚未接入，请先在运维接入设备；Run不会自动连接'
        if str(runtime.get('state','')).upper()=='FAULT' or runtime.get('recoverable_fault'):
            return '设备存在控制故障，请先恢复并核验，不会由 Run 自动恢复'
        if runtime.get('service_error'):return '设备控制线程存在异常，请先核验设备连接'
        if held_session:
            return None if runtime.get('feedback_healthy')and runtime.get('stationary')else'等待健康静止反馈'
        holder=runtime.get('holder')
        source=holder_runtime if holder_runtime is not None else device.snapshot(holder) if holder in BUSINESSES else runtime
        # UI readiness is stable across normal 100 Hz reads. Actual admission still
        # waits at Runtime's existing 100 ms boundary and checks every idle guard.
        gate=source.get('idle_readiness') or (source.get('allowed_operations') or {}).get('preview' if holder=='inference' else 'handoff') or {}
        if not gate.get('allowed'):return gate.get('reason') or '当前控制器不允许接入本轮推理'
        return None

    def run_inference(self, device, workflow, body, job, generation, inference_generation, flow_generation):
        environment=device.environment
        job['memory_start']=body.get('memory_start','continue')
        if body.get('prepare_only')and workflow.normal_modes is None:
            raise ValueError('预备控制会话要求共享正常模式')
        runtime=device.runtime
        binding=self.inference_context(device,device.snapshot('inference'),workflow)
        def current():
            return (self.generations[environment]==generation and self.inference_generations[environment]==inference_generation
                and workflow.generation==flow_generation)
        def check():
            if not current():raise ValueError('本轮 Run 已撤销，不会继续连接或执行')
            if job.get('operator_stop_request_id'):raise ValueError('operator stopped before trajectory submission')
            info=workflow.snapshot().get('inference') or {}
            selected=info.get('selected_model_id') or (info.get('model') or {}).get('id')
            digest=info.get('bundle_digest') or (info.get('model') or {}).get('bundle_digest')
            if selected!=body['model_id'] or digest!=body['expected_bundle_digest']:
                raise ValueError('本轮模型选择或 checkpoint 内容已变化')
            if device.runtime is not runtime:raise ValueError('设备控制实例已变化，本轮已撤销')
            actual=self.inference_context(device,device.snapshot('inference'),workflow)
            if actual!=binding:raise ValueError('本轮控制上下文已变化，请重新确认')
        def stage(name):
            check()
            with self.lock:job['stage']=name
        if runtime is None:raise ValueError('设备控制尚未接入，请先在运维接入设备；Run不会自动连接')
        reason=self.inference_run_reason(device,workflow,device.snapshot('inference'),ignore_job=job['id'])
        if reason:raise ValueError(reason)
        if runtime.holder!='inference':
            stage('handoff')
            runtime.admit_business('inference',active=current)
            binding=self.inference_context(device,device.snapshot('inference'),workflow)
            check()
        session=getattr(workflow,'operator_session',None)
        if session and workflow.current and workflow.current['id']==session['run_id']and workflow.current.get('ended_wall')is None:
            if body.get('prepare_only'):raise ValueError('控制会话已准备好')
            if any(body[name]!=session[name]for name in ('seed','duration_s','model_id','expected_bundle_digest')):
                raise ValueError('控制会话参数已冻结，请结束会话后重新准备')
            if body.get('wam_stall_recovery',False)!=session.get('wam_stall_recovery',False):
                raise ValueError('WAM trick mode differs from the prepared control session')
            if body.get('recording_enabled',False)!=session.get('recording_enabled',False):
                raise ValueError('录制选择已变化，请结束旧会话后重新准备')
            with self.lock:
                if job.get('operator_stop_request_id'):
                    return dict(message='启动已停止，当前位置继续保持',
                        execution=dict(operator_stopped=True,control_session_open=True,recording_open=bool((getattr(workflow,'current',None)or{}).get('recorder'))))
                workflow.current.pop('operator_stop_request_id',None)
                workflow.current.pop('operator_stop_command',None)
                workflow.current.pop('operator_positioning',None)
            job['execution_submitted']=True
            episode=self.run_normal_episode(device,workflow,job,current,runtime,runtime.nonce,body['duration_s'],keep_session=True)
            return dict(message='本轮推理已保持，控制会话与踏板可继续使用',execution=dict(normal_episode=episode))
        stage('binding');bound=workflow.bind_device()
        if isinstance(bound,dict)and type(bound.get('generation'))is int:
            if bound['generation']!=flow_generation+1 or workflow.generation!=bound['generation']:
                raise ValueError('设备绑定代次与本轮接入不一致')
            flow_generation=bound['generation']
            binding=self.inference_context(device,device.snapshot('inference'),workflow)
        check()
        stage('reviewing');review=workflow.preview_episode(body['seed'],duration_s=body['duration_s'],
            recording_enabled=body.get('recording_enabled',False),
            **(dict(wam_stall_recovery=True)if body.get('wam_stall_recovery')else{}));check()
        preview=review.get('preview',{})
        token=preview.get('token')
        if not isinstance(token,str) or not token:raise ValueError('本轮执行审阅未生成有效令牌')
        if (type(preview.get('duration_s')) not in (int,float) or preview['duration_s']!=body['duration_s']
                or type(preview.get('seed')) is not int or preview['seed']!=body['seed']):
            raise ValueError('本轮审阅的时长或种子与已确认的 Run 不一致')
        stage('confirming')
        # Order the final enqueue against stop/cancel. No model/camera work in this lock.
        with self.lock:
            if not current() or device.runtime is not runtime:raise ValueError('本轮 Run 在执行前已撤销')
            if job.get('operator_stop_request_id'):raise ValueError('operator stopped before trajectory submission')
            result=workflow.execute_review(token,physical_clearance=True)
            job['execution_submitted']=True
            job['stage']='submitted'
        normal_episode=None
        if (getattr(workflow,'normal_modes',None) is not None
                and getattr(workflow,'reference_planning',None) is not None
                and workflow.normal_initial_mode=='HOLD'):
            normal_episode=self.run_normal_episode(device,workflow,job,current,runtime,binding['runtime_nonce'],body['duration_s'],
                prepare_only=True)
            workflow.operator_session=dict(run_id=normal_episode['execution_instance_id'],runtime_nonce=runtime.nonce,
                recording_enabled=body.get('recording_enabled',False),
                wam_stall_recovery=body.get('wam_stall_recovery',False),
                **{name:body[name]for name in ('seed','duration_s','model_id','expected_bundle_digest')})
            if body.get('prepare_only'):
                return dict(message='控制会话已准备，踏板可直接切换模式',execution=normal_episode)
            normal_episode=self.run_normal_episode(device,workflow,job,current,runtime,binding['runtime_nonce'],body['duration_s'],
                keep_session=True)
            result=dict(result,normal_episode=normal_episode)
        return dict(message='本轮推理已保持，控制会话与踏板可继续使用'if normal_episode else'本轮推理已提交；请查看实际执行与录像状态',execution=result,
            model_id=body['model_id'],seed=body['seed'],duration_s=body['duration_s'])

    def run_normal_episode(self,device,workflow,job,current,runtime,nonce,duration,*,prepare_only=False,keep_session=False):
        """Use the existing Run worker to join reviewed HOLD, MODEL and END receipts."""
        deadline=time.monotonic()+10.
        execution_id=None
        model_request=None
        end_request=None
        stop_reason=None
        normal_reasons={'normal MODEL episode budget exhausted',
            'complete next segment exceeds remaining episode budget',
            'explicit single-segment acceptance budget finished; normal HOLD requires original Raw gate'}
        while time.monotonic()<deadline:
            if not current() or device.runtime is not runtime:
                raise ValueError('本轮 Run 已撤销，后续 MODEL/END 未提交')
            view=workflow.snapshot()
            mode=(view.get('inference')or{}).get('normal_mode')or{}
            # RemoteRunner starts with no Owner snapshot. Wait within the same
            # deadline; once bound, a missing or changed identity still rejects.
            if mode and (execution_id is not None or mode.get('runtime_nonce') is not None):
                execution_id=execution_id or view['id']
                if view['id']!=execution_id or mode.get('runtime_nonce')!=nonce:
                    raise ValueError('本轮推理执行身份已变化')
                if mode.get('runner_status')in('FAULT','STOPPED'):
                    raise ValueError(mode.get('reason')or'本轮推理已停止或故障')
                receipts=mode.get('mode_receipts')or{}
                held=mode.get('mode')=='HOLD' and not mode.get('transition_stage')
                initialized=any(row.get('op')=='normal_initialize' and row.get('completed')is True
                    for row in receipts.values())
                if prepare_only and held and initialized:
                    return dict(execution_instance_id=execution_id,mode='HOLD',control_session_open=True,recording_open=bool((getattr(workflow,'current',None)or{}).get('recorder')))
                positioning=(getattr(workflow,'current',None)or{}).get('operator_positioning')
                if positioning and positioning.get('status')=='running':
                    deadline=time.monotonic()+10.
                    time.sleep(.01)
                    continue
                target=None
                operator_stop=job.get('operator_stop_request_id')or(getattr(workflow,'current',None)or{}).get('operator_stop_request_id')
                if operator_stop and not held and not mode.get('transition_stage'):
                    workflow.request_operator_stop(operator_stop)
                if model_request is None and held and initialized:
                    if operator_stop and keep_session:
                        return dict(execution_instance_id=execution_id,reason='operator stopped in position HOLD',
                            control_session_open=True,recording_open=bool((getattr(workflow,'current',None)or{}).get('recorder')),operator_stopped=True)
                    target='END'if operator_stop else'MODEL'
                elif model_request is not None and end_request is None and held:
                    model_receipt=receipts.get(model_request)or{}
                    if model_receipt.get('completed')is True or model_receipt.get('status')in('failed','rejected'):
                        stop_reason=mode.get('reason')or model_receipt.get('error')or model_receipt.get('reason')or 'MODEL ended before its budget'
                        if keep_session:
                            if not operator_stop and not positioning and stop_reason not in normal_reasons:
                                raise ValueError('推理已保持：'+str(stop_reason))
                            return dict(execution_instance_id=execution_id,reason=stop_reason,control_session_open=True,recording_open=bool((getattr(workflow,'current',None)or{}).get('recorder')),
                                operator_stopped=bool(operator_stop or positioning))
                        target='END'
                if end_request is not None:
                    ending=receipts.get(end_request)or{}
                    if ending.get('completed')is True and mode.get('runner_status')=='READY' and not view.get('busy'):
                        if operator_stop:
                            return dict(execution_instance_id=execution_id,reason='operator stopped in position HOLD',
                                operator_stopped=True,ending=ending)
                        if receipts.get(model_request,{}).get('completed')is not True or stop_reason not in normal_reasons:
                            raise ValueError('推理提前保持并已正常结束：'+str(stop_reason))
                        return dict(execution_instance_id=execution_id,reason=stop_reason,ending=ending)
                    if ending.get('status')in('failed','rejected'):raise ValueError('本轮正常结束未被接纳')
                if target is not None:
                    request_id=job.get('request_id')if target=='MODEL'else None
                    request_id=request_id or uuid.uuid4().hex
                    request=dict(session_id=mode['mode_session_id'],runtime_nonce=nonce,
                        request_id=request_id,epoch=mode['command_epoch'],
                        mode='MODEL'if target=='MODEL'else'HOLD',target=target)
                    if target=='MODEL'and callable(getattr(workflow,'begin_memory',None)):
                        workflow.begin_memory(job.get('memory_start','continue'))
                    with self.lock:
                        if not current() or device.runtime is not runtime:
                            raise ValueError('本轮 Run 已撤销，后续 MODEL/END 未提交')
                        if target=='MODEL'and(job.get('operator_stop_request_id')or
                                (getattr(workflow,'current',None)or{}).get('operator_stop_request_id')):
                            continue
                        workflow.request_normal_mode(request)
                        job['stage']='model_running'if target=='MODEL'else'ending'
                    if target=='MODEL':
                        model_request=request_id
                        deadline=time.monotonic()+duration+10.
                    else:end_request=request_id
            time.sleep(.02)
        raise ValueError('本轮正常模式回执等待超时，请核验原请求与控制状态')

    def device(self, environment):
        if environment not in self.devices:
            raise ValueError('该环境尚未配置，不能切换到另一个环境代替执行')
        return self.devices[environment]

    def control_policy(self, environment, business):
        if business not in BUSINESSES:raise ValueError('unknown business')
        runtime=self.device(environment).runtime
        if runtime is None or not getattr(runtime,'supports_control_policy',False):
            raise ValueError('当前Owner尚未提供统一策略接口')
        policy=runtime.control_policy_snapshot(business)
        flow=self.device(environment).workflows.get('inference')
        if (business=='inference' and policy['origins'].get('execution.execute_steps')=='product_default'
                and hasattr(flow,'execute_steps')):
            from .execution_settings import policy_hash
            policy['parameters']['execution']['execute_steps']=flow.execute_steps
            policy['origins']['execution.execute_steps']='legacy_workflow_settings'
            policy['policy_hash']=policy_hash(policy['parameters'])
        return policy

    def environment_state(self,environment):
        """Environment facts stay independent from a business's permissions/view."""
        device=self.device(environment);runtime=device.fact_snapshot()
        views={}
        for name,flow in device.workflows.items():
            reader=getattr(flow,'environment_status',None)
            views[name]=reader() if callable(reader) else flow.snapshot()
        flow=device.workflows.get('inference');info=(views.get('inference') or {}).get('inference') or {}
        if callable(getattr(flow,'environment_status',None)):model=deepcopy(views.get('inference')or{})
        else:
            selected=info.get('model') or {}
            model=dict(selected_model_id=info.get('selected_model_id') or selected.get('id'),bundle_digest=info.get('bundle_digest') or selected.get('bundle_digest'),
                loaded=selected.get('loaded'),ready=info.get('model_ready'),model_ready=info.get('model_ready'),
                status=selected.get('status','unknown'),error=selected.get('error'),generation=info.get('generation'),observed_wall_ns=time.time_ns())
        preparation=device.collection_preparation or device.workflows.get('collection')
        collection=(deepcopy(preparation.environment_status()) if callable(getattr(preparation,'environment_status',None)) else
            deepcopy(preparation.snapshot()) if preparation is not None else dict(status='unconfigured',ready=False,reason='当前环境未配置数采准备'))
        with self.lock:
            jobs=[{key:deepcopy(row[key]) for key in ('id','action','status','stage','started_wall_ns','error') if key in row}
                for (env,_),row in self.jobs.items() if env==environment]
            from .maintenance import public_state
            maintenance=public_state(self.maintenance.get(environment));action_calls=self.action_calls[environment]
        recording=runtime.get('recording') or {}
        from .gc_timing import snapshot as gc_snapshot
        hold_status=(runtime.get('control_operation')or{}).get('status')
        protection=runtime.get('control_protection')or{}
        exited=(runtime.get('connected')is False and protection.get('worker_exit_confirmed')is True
                and protection.get('supervisor_exit_confirmed')is True)
        hold_busy=hold_status in ('queued','verifying_feedback','holding','stopping') or hold_status=='unknown' and not exited
        activity=dict(busy=bool(device.connecting or any(row.get('busy') for row in views.values()) or collection.get('busy')
            or any(row.get('status')=='running' for row in jobs) or action_calls or runtime.get('automatic_enabled')
            or hold_busy
            or recording and recording.get('done') is not True),connecting=device.connecting,requests=jobs,
            businesses={name:{key:view.get(key) for key in ('busy','status','stage_id')} for name,view in views.items()},
            recording_active=bool(recording and recording.get('done') is not True))
        return dict(schema='robot_workbench.environment.v1',projection='backend_environment',device=dict(id=device.id,environment=environment,label=device.label),
            server_instance_id=self.server_instance_id,csrf=self.csrf,server_wall_ns=time.time_ns(),runtime=runtime,
            control_timing=dict(gc=gc_snapshot()),
            operations_managed=self.operations_managed,
            activity=activity,maintenance=maintenance,model=model,collection=collection,
            cameras=[provider.snapshot() for name,provider in device.media.items() if name in ('head','left','right')])

    def maintenance_action(self,action,body):
        if not isinstance(body,dict):raise ValueError('维护请求对象必需')
        from .maintenance import operate
        return operate(self,self.device(body.get('environment')),action,body)

    def stop_device(self,device):
        with self.lock:
            window=self.maintenance.get(device.environment)
            if window is not None:window.update(_cancelled=True,error='Stop已撤销维护计划')
        result=device.request_stop()
        self.invalidate(device.environment,clear_manual=True,workflow_async=True)
        for workflow in device.workflows.values():
            callback=getattr(workflow,'cancel',None)
            if callable(callback):threading.Thread(target=callback,daemon=True).start()
        return {**result,'message':'停止已受理；请查看控制器确认状态'}

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
        titles={'awaiting_calibration':'等待标定安装','awaiting_prepare':'等待运维准备数采服务','queued':'数采服务等待准备','verifying_installation':'核验当前已安装标定',
            'preparing_profile':'根据已安装标定构建数采配置','converting_planning_assets':'转换数采规划资产',
            'initializing_detector':'初始化物体检测服务','initializing_planner':'初始化轨迹规划服务',
            'initializing_scene':'初始化常驻场景服务','warming_planner':'初始化场景与预热规划模型','ready':'数采常驻服务已就绪','failed':'数采服务准备失败'}
        title=titles.get(phase,'准备数采环境')
        detail=preparation.get('error') or (preparation.get('progress') or {}).get('label') or (preparation.get('worker') or {}).get('message') or title
        retained=preparation.get('installed_calibration_preserved')
        warmup=deepcopy(preparation.get('warmup') or dict(status='not_started',
            message='先在服务运维准备资产与服务，再在数采页初始化与预热。'))
        warmup.update(stage_label=title,elapsed_s=(preparation.get('progress') or {}).get('elapsed_s'))
        actions={name:dict(id=action,label=label,allowed=preparation['can_retry'] if name=='prepare' else False,
                    reason=preparation['retry_reason'] if name=='prepare' else '先完成初始化与预热')
            for name,action,label in (('prepare','collection_prepare','初始化与预热'),('plan','collection_plan','采集新观测并生成计划'),
                                      ('execute','execute','审阅并执行本批'))}
        return dict(status='running' if busy else 'failed' if phase=='failed' else 'idle',busy=busy,
            stage_id='collection.setup.'+phase,title=title,
            message=('已安装标定保留，无需重新采集。' if retained else '')+str(detail),
            primary_action=dict(id='prepare_collection',label='重新准备数采环境',allowed=preparation['can_retry'],reason=preparation['retry_reason']),
            collection=dict(warmup=warmup,actions=actions,timing=preparation.get('progress') or {}),
            progress=dict(indeterminate=True,unit=title) if busy else None,
            details=[dict(label='头部标定',value=installed.get('head_version') or installed.get('head_id') or '尚未安装'),
                     dict(label='双腕标定',value=installed.get('wrist_version') or installed.get('wrist_id') or '尚未安装'),
                     dict(label='服务期限',value='长期运行 · 手动停止') if preparation.get('persistent_service') is True else
                     dict(label='本次驻留剩余时间',value=f"{preparation['remaining_wall_s']:.0f} 秒")],
            outcomes=dict(calibration='已安装并保留' if retained else '等待有效安装',preparation=phase),preparation=preparation)

    def state(self, environment, business, *, results_before=None, results_after=None):
        if business not in BUSINESSES:
            raise ValueError('未知业务')
        paging = results_before is not None or results_after is not None
        device = self.device(environment)
        if paging and business!='inference'and device.capture_results is None:raise ValueError('当前业务未配置录制历史分页')
        runtime = device.snapshot(business)
        workflow = device.workflows.get(business)
        manual_workflow=device.workflows.get('calibration')
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
        precision_job=job if job and job['action'].startswith('precision_')else None
        if precision_job:job=None  # Study progress belongs to its own panel.
        workflow_views={name:flow.snapshot() for name,flow in device.workflows.items()}
        value = workflow_views[business] if workflow else dict(status='unconfigured',busy=False,
            stage_id='configuration',title='配置业务后端',message='设备控制可独立验证；当前业务的采集、求解与数据路径尚未配置。',
            primary_action=None,progress=None,outcomes={})
        if business=='collection':
            value=collection_live_view(value,runtime)
            from robot_workbench.cup_return import view as cup_return_view
            value=cup_return_view(device,runtime,value)
        if readiness is not None:value['control_readiness']=readiness
        preparation=device.collection_preparation.snapshot() if business=='collection' and device.collection_preparation else None
        preparing=bool(preparation and not getattr(device,'cup_return_task',None)
            and (workflow is None or preparation['busy'] or preparation['status']!='ready'
                and not value.get('busy') and not frozen_collection_scope(device)))
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
                    review.get('manual_revision') is not None and review['manual_revision']!=device.workflows['calibration'].manual_revision()):
                review=None
        business_task=deepcopy(value)
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
            value['review']['expires_at_wall_ns']=time.time_ns()+int(max(0.,review['deadline']-time.monotonic())*1e9)
            for name in ('request_job_id','total_motion_duration_s','waypoint_jaw_gap_mm','offset_xy_mm','origin_position_m','goal_position_m','seed','duration_s','execution_duration_s'):
                if name in review:value['review'][name]=review[name]
            if review.get('speed_scale') is not None:value['review']['speed_scale']=review['speed_scale']
            execution_gate=runtime['allowed_operations']['confirm' if review['owner_preview'] else 'preview']
            value['primary_action'] = dict(id=review['action'],label=f"确认执行本轮 {review.get('execution_duration_s',review.get('duration_s',120)):g} 秒推理" if business=='inference' else '审阅并执行本批',
                allowed=execution_gate['allowed'],reason=execution_gate['reason'])
        if job and job['status'] in ('running','failed','cancelled'):
            value['request'] = job
            if job['status'] == 'running':
                value['busy'] = True
                if job['action']=='inference_run':
                    phase=job.get('stage','binding')
                    value.update(stage_id='inference.'+phase,title='推理进行中'if phase=='model_running'else'本轮正常结束中'if phase=='ending'else'准备本轮推理',message={
                        'connecting':'正在连接设备并等待健康反馈','handoff':'正在交接控制权',
                        'binding':'正在核对模型、标定与相机','reviewing':'正在准备本轮执行审阅',
                        'confirming':'正在提交已确认的本轮推理','submitted':'本轮已提交，等待控制回执',
                        'model_running':'按所选时长持续观测、推理并执行完整片段，可随时 Stop',
                        'ending':'轨迹已保持，正在等待本轮 END 回执'}.get(phase,phase))
                elif job['action']=='inference_teach':
                    value.update(stage_id='inference.manual_start',title='人工设置推理起点',
                        message='踏板2切换重力补偿拖动与位置保持，保持完成后可开始下一轮Infer。')
                if value.get('primary_action'):
                    value['primary_action'].update(allowed=False,reason='当前请求仍在处理')
            elif job['action']=='inference_run' and job['status']=='failed':
                value.update(status='failed',message=job.get('error')or'本轮推理未完成')
        allowed = runtime.setdefault('allowed_operations',{})
        execution_settings=runtime.get('execution_settings') or {}
        settings_reason=execution_settings.get('update_reason') or '请先连接设备'
        settings_allowed=execution_settings.get('update_allowed') is True
        with self.lock:
            settings_job_busy=any(item.get('status')=='running' for (env,_),item in self.jobs.items() if env==environment)
        workflows_busy=any(view.get('busy') for view in workflow_views.values())
        if settings_job_busy or workflows_busy:
            settings_allowed=False;settings_reason='请先结束当前请求或业务任务，再应用执行误差设置'
        allowed['execution_settings']=dict(allowed=settings_allowed,reason=None if settings_allowed else settings_reason)
        policy_supported=bool(getattr(device.runtime,'supports_control_policy',False))
        allowed['control_policy']=dict(allowed=settings_allowed and policy_supported,
            reason=('当前Owner尚未提供统一策略接口' if not policy_supported else None if settings_allowed else settings_reason))
        connect_ready=device.runtime is None and not device.connecting and not (device.thread and device.thread.is_alive())
        allowed['connect'] = dict(allowed=bool(connect_ready and device.factory is not None),
                                  reason=('设备控制后端尚未配置' if device.factory is None else
                                          None if connect_ready else '设备仍连接或正在收尾，请等待明确断开'))
        allowed.setdefault('release',dict(allowed=False,reason='请先连接设备'))
        if workflows_busy:
            allowed['release']=dict(allowed=False,reason='请先结束或取消业务任务，再释放设备连接')
        holder = runtime.get('holder')
        holder_runtime=runtime
        # This is an explicit operator handoff endpoint, not an application
        # pretending to be the current holder when sending a motion command.
        if device.runtime is not None:
            holder_runtime=runtime if holder==business else device.snapshot(holder)
            source = holder_runtime['allowed_operations']['handoff']
            allowed['handoff'] = source
            if workflows_busy:
                allowed['handoff'] = dict(allowed=False,reason='业务任务仍在进行；请先结束或取消当前任务')
        motion_reason = allowed['preview']['reason']
        manual_ready=allowed['preview']['allowed'] and not workflows_busy and not settings_job_busy
        manual_reason=motion_reason or ('当前业务任务或请求正在进行' if workflows_busy or settings_job_busy else None)
        if environment=='physical' and not (manual_workflow and hasattr(manual_workflow,'manual_preview')):
            manual_ready=False;manual_reason='物理人工规划需要标定路径校验服务'
        if business!='calibration' and 'shared_manual_trajectory_v1' not in runtime.get('capabilities',[]):
            manual_ready=False;manual_reason='当前Owner尚未加载共用人工轨迹入口，请在服务运维核验控制版本'
        allowed['manual_preview']=dict(allowed=bool(manual_ready),reason=manual_reason)
        queue=(workflow_views.get('calibration') or {}).get('manual_queue') or dict(available=False,reason='未配置标定服务，人工队列不可用')
        for name in ('manual_queue','manual_queue_preview'):
            allowed[name]=dict(allowed=bool(business=='calibration' and manual_ready and queue.get('available')),reason='标定动作队列仅属于标定业务' if business!='calibration' else manual_reason or queue.get('reason'))
        allowed['manual_execute'] = dict(allowed=bool(review and review['action']=='manual_execute') and allowed['confirm']['allowed'],
            reason=allowed['confirm']['reason'] or (None if review else '请先生成并审阅人工姿态'))
        allowed['recover'] = dict(allowed=bool(runtime.get('recoverable_fault')) and holder==business,
            reason=None if runtime.get('recoverable_fault') and holder==business else '当前没有可确认恢复的故障；设备故障请查看诊断')
        if self.operations_managed:
            for action in ('connect','release','recover'):
                allowed[action]=dict(allowed=False,reason='operation_required：请从统一运维入口执行设备生命周期操作')
        preview=self.preview_state(device,runtime)
        if preview and callable(getattr(manual_workflow,'mesh_candidates',None)):
            preview['mesh_candidates']=manual_workflow.mesh_candidates()
        if environment=='physical' and preview and callable(getattr(manual_workflow,'mesh_calibrations',None)):
            preview['mesh_calibrations']=manual_workflow.mesh_calibrations()
        for action in ('preview_target','preview_overview','preview_mesh'):
            supported=preview.get('capabilities',{}).get(action.removeprefix('preview_'),True)
            ready=bool(preview.get('connected')) and supported
            allowed[action]=dict(allowed=ready,reason=None if ready else
                ('此设备仅提供实际图像上的 mesh 叠加，未配置目标姿态渲染器' if not supported else '目标图像服务尚未就绪，请查看画面恢复指引'))
        allowed['preview_recover']=dict(allowed=bool(device.preview and hasattr(device.preview,'recover') and preview.get('recovery',{}).get('allowed')),
            reason=preview.get('recovery',{}).get('message'))
        for action in ('cancel','mesh','select_result','install','generate_plan','execute',*CALIBRATION_TOOLS,*COLLECTION_TOOLS,*INFERENCE_TOOLS,*HISTORY_ACTIONS):
            allowed.setdefault(action,dict(allowed=False,reason='当前业务后端尚未配置'))
        if workflow:
            for action in ('cancel','mesh','select_result','install','generate_plan','execute',*CALIBRATION_TOOLS,*COLLECTION_TOOLS,*INFERENCE_TOOLS,*HISTORY_ACTIONS):
                if hasattr(workflow,'allowed'):
                    allowed[action]=(workflow.allowed(action,current=workflow_views[business])
                        if isinstance(workflow,InferenceWorkflow) else workflow.allowed(action))
                if business=='calibration' and action in (*CALIBRATION_TOOLS,'generate_plan') and not allowed['preview']['allowed']:
                    allowed[action]=dict(allowed=False,reason=allowed['preview']['reason'])
        if readiness is not None and not readiness['host_sample_recent']:
            for action in ('execute','manual_execute'):
                if allowed[action]['allowed']:allowed[action]=dict(allowed=False,reason=readiness['reason'])
        if preparation:
            operation=getattr(device.collection_preparation,'collection_prepare_operation',None) or getattr(device,'collection_prepare_operation',None)
            reason=preparation['retry_reason'] if callable(operation) else '数采环境准备需要运维 Operation；请从统一运维入口发起'
            allowed['prepare_collection']=dict(allowed=preparation['can_retry'] and callable(operation),reason=reason)
            if workflow is None:
                allowed['collection_prepare']=dict(allowed=False,reason='请先在服务运维准备当前数采资产与常驻服务')
            if preparing:
                for action in ('generate_plan','execute'):
                    allowed[action]=dict(allowed=False,reason='先完成数采环境准备；已安装标定仍保留')
        if business=='calibration' and (preview.get('diagnostic_calibration') or {}).get('mode')=='warm_diagnostic':
            for action in WARM_C_ACTIONS:allowed[action]=dict(allowed=False,reason=WARM_C_REASON)
        primary = value.get('primary_action')
        if primary and primary['id']=='prepare_collection':primary.update(allowed['prepare_collection'])
        if primary and primary['id'] in (*CALIBRATION_TOOLS,'generate_plan') and not allowed[primary['id']]['allowed']:
            primary.update(allowed=False,reason=allowed[primary['id']]['reason'])
        if readiness is not None and primary and primary['id'] in ('execute','manual_execute') and not allowed[primary['id']]['allowed']:
            primary.update(allowed=False,reason=allowed[primary['id']]['reason'])
        if primary and primary['id'] not in ('prepare_collection','inference_run',*INFERENCE_MODEL_TOOLS) and (not runtime['connected'] or holder != business):
            primary.update(allowed=False,reason='请先连接设备' if not runtime['connected'] else '请先交接控制权限：当前由 '+str(holder)+' 持有')
        if business=='collection':
            for action in (*COLLECTION_TOOLS,'execute'):
                reason=('请先连接设备' if not runtime['connected'] else '请先交接控制权限：当前由 '+str(holder)+' 持有' if holder!=business else
                    '请先完成数采环境准备' if preparing and action!='collection_prepare' else '请先生成并审阅当前执行计划' if action=='execute' and not review else None)
                if reason:allowed[action]=dict(allowed=False,reason=reason)
            for entry in value.get('collection',{}).get('actions',{}).values():entry.update(allowed[entry['id']])
        if business=='inference':
            operator_task=getattr(workflow,'operator_task',{})if workflow is not None else{}
            operator_busy=operator_task.get('status')=='running'
            operator_ready=(runtime.get('connected')and holder=='inference'and runtime.get('feedback_healthy')
                and not runtime.get('recoverable_fault')and 'FAULT'not in str(runtime.get('state')))
            allowed['inference_hold']=dict(allowed=bool(operator_ready),reason=None if operator_ready else'需要当前推理控制权与健康连接')
            memory_mode=(value.get('inference',{}).get('normal_mode')or{})
            memory_ready=bool(operator_ready and not operator_busy and getattr(getattr(workflow,'client',None),'memory_flow',None)
                and memory_mode.get('mode')=='HOLD'and not memory_mode.get('transition_stage'))
            allowed['inference_memory_clear']=dict(allowed=memory_ready,reason=None if memory_ready else'请先准备 WAM-MEM 并进入 HOLD')
            session=getattr(workflow,'operator_session',None)
            session_ready=bool(session and value.get('id')==session['run_id']
                and (value.get('inference',{}).get('normal_mode')or{}).get('runner_status')=='RUNNING')
            model_active=self._operator_model_active(device,runtime)
            teach_active=operator_busy and operator_task.get('kind')=='teach'
            teach_ready=bool(operator_ready and (model_active or teach_active or not operator_busy
                and session_ready and (value.get('inference')or{}).get('model_ready')))
            allowed['inference_teach']=dict(allowed=teach_ready,
                reason=None if teach_ready else '请先准备控制与踏板，等待当前模式转换完成')
            allowed['inference_home']=dict(allowed=bool(operator_ready and session_ready and not operator_busy and not model_active and getattr(workflow,'operator_controls',{})),
                reason='推理过程中踏板3不执行回零'if model_active else'等待当前踏板操作完成'if operator_busy else None if operator_ready and session_ready else'请先准备控制与踏板')
            for action in ('inference_preview','execute'):
                reason=('请先连接设备' if not runtime['connected'] else '请先交接控制权限：当前由 '+str(holder)+' 持有' if holder!=business else
                    '当前推理请求或执行尚未结束' if value.get('busy') else
                    '请先准备并审阅本轮推理' if action=='execute' and not review else
                    allowed['confirm' if action=='execute' else 'preview'].get('reason') if not allowed['confirm' if action=='execute' else 'preview']['allowed'] else None)
                if reason:allowed[action]=dict(allowed=False,reason=reason)
            if primary and primary['id'] in (*INFERENCE_TOOLS,'execute'):primary.update(allowed[primary['id']])
            reason=self.inference_run_reason(device,workflow,runtime,workflow_view=value,
                workflow_views=workflow_views,holder_runtime=holder_runtime)
            allowed['inference_run']=dict(allowed=reason is None,reason=reason)
            if workflow and hasattr(workflow,'refresh_models'):
                for action in INFERENCE_MODEL_TOOLS:
                    if value.get('busy'):allowed[action]=dict(allowed=False,reason='当前推理请求或执行尚未结束')
                    else:allowed[action]=dict(allowed=True,reason=None)
            info=value.setdefault('inference',{})
            session=getattr(workflow,'operator_session',None)
            info['operator_session']=(deepcopy(session)if session and value.get('id')==session['run_id']
                and (info.get('normal_mode')or{}).get('runner_status')=='RUNNING'else None)
            info['run_context']=self.inference_context(device,runtime,workflow)
            info['run_action']=dict(id='inference_run',**allowed['inference_run'])
            if primary and primary['id'] in INFERENCE_MODEL_TOOLS:primary.update(allowed[primary['id']])
        return_task=getattr(device,'cup_return_task',None) if business=='collection' else None
        return_install=(preparation or {}).get('installed') or {}
        restored=((getattr(device,'scene_recovery_state',{}) or {}).get('last_completed') or {}).get('task_kind') in ('cup_return','scene_initialization')
        recovery_error=getattr(device,'scene_recovery_error',None)
        physical_return=getattr(device,'physical_cup_return_candidate',None)
        return_source=bool(getattr(device,'cup_return_inputs',None)) if environment=='fake' else bool(physical_return and Path(physical_return).is_file())
        if environment=='physical' and return_source:
            from robot_workbench.cup_return import installed_pair
            return_install=installed_pair(device)
        return_allowed=bool(business=='collection'
            and return_source and return_install.get('verified') and not value.get('busy')
            and not restored and not recovery_error and holder=='collection' and allowed['preview']['allowed'])
        allowed['return_cup']=dict(allowed=return_allowed,
            reason=None if return_allowed else str(recovery_error) if recovery_error else '场景已恢复；完成下一轮放杯后可再次恢复' if restored else
            '需已安装标定和完成检查的还原计划，且当前无动作')
        if return_task:
            if preparation and 'warmup' not in value['collection']:
                value['collection']['warmup']=dict(status='ready' if preparation['status']=='ready' else 'not_started',
                    message=preparation.get('message') or '普通数采尚未预热；场景恢复无需检测器或 cuRobo 预热。')
            allowed['execute']=dict(allowed=bool(review and review.get('purpose')=='cup_return' and allowed['confirm']['allowed']),
                reason=allowed['confirm']['reason'] or (None if review else '请重新生成取回计划并审阅'))
            value['collection']['actions']['execute'].update(allowed['execute'])
        if return_task and value.get('busy'):
            for name in ('collection_prepare','collection_plan','generate_plan','return_cup'):
                allowed[name]=dict(allowed=False,reason='请等待取回动作结束')
        raw = (runtime.get('calibration_feedback') or {}).get('qpos')
        limits = runtime.get('joint_limits')
        geometry = getattr(getattr(manual_workflow, 'app', None), 'geometry', None)
        model_limits = getattr(geometry, 'limits', None)
        limit_provider=getattr(getattr(manual_workflow,'app',None),'manual_joint_limits',None)
        if callable(limit_provider):model_limits=limit_provider()
        arm_limits = limits[:6]+limits[7:13] if limits else (
            model_limits.tolist() if hasattr(model_limits, 'tolist') else model_limits)
        manual = {}
        if arm_limits is not None:
            from arx_env.joint_head_jaw_mapping import feedback_gaps,command_max_mm,feedback_max_mm,feedback_min_mm
            from arx_env.joint_head_real_motion import DEFAULT_VELOCITY
            gaps=feedback_gaps(raw) if raw else [None,None]
            valid=(runtime.get('calibration_feedback') or {}).get('gripper_feedback_valid',[True,True])
            manual=dict(joint_names=[side+' J'+str(i) for side in ('左','右') for i in range(1,7)],
                positions=raw[:6]+raw[7:13] if raw else None, limits=arm_limits,
                jaw_gaps_mm=[value if valid[i] else None for i,value in enumerate(gaps)],
                jaw_command_ranges_mm=[[0.,command_max_mm(side)]for side in ('left','right')],
                jaw_feedback_ranges_mm=[[feedback_min_mm(side),feedback_max_mm(side)]for side in ('left','right')],
                disabled_grippers=list(runtime.get('passive_jaws') or []),
                position_noise_rad=position_noise_rad(environment),
                default_joint_speed_rad_s=DEFAULT_VELOCITY,default_jaw_speed_mm_s=10.)
        manual['queue']=queue
        manual['execution']=deepcopy(value) if manual_task and value.get('stage_id','').startswith('manual.') else None
        if manual or job and job['action'].startswith('manual_'):value['business_task']=business_task
        calibration=device.workflows.get('calibration')
        arrival=(workflow_views.get('calibration') or {}).get('arrival_statistics') or {'error':'未配置标定到位统计；协议 Fake 不生成统计数据'}
        history=getattr(device,'collection_history',None) if business=='collection' else None
        results_page = None
        if business == 'inference' and callable(getattr(workflow, 'results_page', None)):
            results_page = workflow.results_page(before=results_before, after=results_after)
            results = results_page.pop('results')
        else:
            results = workflow.results() if workflow and not paging else []
            if device.capture_results is not None and business in device.capture_results.BUSINESSES:
                results_page=device.capture_results.page(business,before=results_before,after=results_after)
                results+=results_page.pop('results')
            elif paging:raise ValueError('当前推理服务未配置历史分页')
        inference_flow=device.workflows.get('inference')
        inference_execution=inference_flow.execution_settings() if hasattr(inference_flow,'execution_settings') else None
        allowed['inference_execution_settings']=dict(allowed=bool(inference_execution and inference_execution['allowed'] and not settings_job_busy),
            reason='当前请求尚未结束' if settings_job_busy else inference_execution.get('reason') if inference_execution else '当前环境未配置推理服务')
        from .maintenance import public_state
        with self.lock:maintenance=public_state(self.maintenance.get(environment))
        if maintenance:
            for action,gate in allowed.items():
                if action!='stop':gate.update(allowed=False,reason='服务维护中：'+maintenance['purpose'])
            if value.get('primary_action'):value['primary_action'].update(allowed=False,reason='服务维护中')
        study=getattr(device,'precision_study',None)
        precision=study.snapshot(runtime,business)if study else None
        if precision:
            precision['request']=precision_job
            reason=('服务维护中'if maintenance else '当前请求尚未结束'if settings_job_busy else precision['reason'])
            allowed['precision_preview']=dict(allowed=reason is None,reason=reason)
            allowed['precision_execute']=dict(allowed=reason is None and precision['execute_allowed'],reason=reason or '先准备并审阅研究动作')
            allowed['precision_analyze']=dict(allowed=not maintenance and not settings_job_busy and bool(precision['latest'])
                and precision['latest'].get('status')not in('submitted','running'),reason='需已有记录且执行已收尾')
            allowed['precision_settings']=dict(allowed=not maintenance and not settings_job_busy and precision.get('settings_allowed',False),reason='研究仍在执行或维护中，不能保存配置')
            for name in('precision_select','precision_notes'):
                allowed[name]=dict(allowed=not maintenance and not settings_job_busy and bool(precision.get('history')),reason='需已有已收尾的研究记录')
        return dict(schema='robot-workbench-ui.v1',server_instance_id=self.server_instance_id,
            revision=revision,revision_semantics='response_order_within_server_instance',server_wall_ns=time.time_ns(),csrf=self.csrf,
            device=dict(id=device.id,label=device.label,environment=environment),runtime=runtime,
            scene_assets=({**preparation['scene_assets'],**{k:preparation.get(k)for k in ('status','busy','can_retry','retry_reason')}}
                if preparation and preparation.get('scene_assets') else None),
            workflow=value,media=[provider.snapshot() for provider in device.media.values()],
            results=results+(history.results() if history else []),results_page=results_page,diagnostics=[
                {'label':'控制器状态','value':runtime.get('state','未知')},
                {'label':'控制器当前原因','value':runtime.get('reason') or '未提供当前状态原因'},
                {'label':'服务线程异常','value':device.error or '未报告服务线程异常'},
                *([{'label':'历史录制目录未就绪','value':device.collection_history_error}] if getattr(device,'collection_history_error',None) else []),
                *value.get('diagnostics',[])],
            mesh=workflow.mesh() if workflow and hasattr(workflow,'mesh') else {'requested':False,'applied':False},manual=manual,
            arrival_statistics=arrival,preview=preview,inference_execution=inference_execution,
            maintenance=maintenance,precision_study=precision)

    def action(self, action, body):
        if not isinstance(body,dict) or body.get('business') not in BUSINESSES:
            raise ValueError('业务与请求对象必需')
        environment,business=body.get('environment'),body['business']
        device=self.device(environment)
        from .maintenance import ACTIONS as MAINTENANCE_ACTIONS, apply as maintenance_apply
        if action in MAINTENANCE_ACTIONS:
            if not isinstance(body.get('request_id'),str) or not 1<=len(body['request_id'])<=128:
                raise ValueError('有效的维护请求身份必需')
            return maintenance_apply(self,device,action,body)
        if action=='release' and 'maintenance_token' in body:
            return maintenance_apply(self,device,'release_control',body)
        if action=='stop':return self._action(action,body)
        with self.lock:
            maintenance=self.maintenance.get(environment)
            if maintenance:
                raise ValueError('服务维护中，请等待维护结果；停止仍可独立使用')
            self.action_calls[environment]+=1
        try:
            return self._action(action,body)
        finally:
            with self.lock:self.action_calls[environment]-=1

    def _action(self, action, body):
        environment,business=body.get('environment'),body['business']
        device=self.device(environment)
        if self.operations_managed and action in ('connect','release','recover'):
            raise ValueError('operation_required：设备接入、释放与恢复必须通过运维维护窗口执行')
        if action=='prepare_collection' and not callable(
                getattr(device.collection_preparation,'collection_prepare_operation',None) or getattr(device,'collection_prepare_operation',None)):
            raise ValueError('operation_required：数采环境准备需要运维 Operation；请从统一运维入口发起')
        if action=='collection_prepare':
            workflow=device.workflows.get('collection')
            if business!='collection' or workflow is None:raise ValueError('请先在服务运维准备数采资产与常驻服务')
            permission=workflow.allowed(action)
            if permission.get('allowed')is not True:raise ValueError(permission.get('reason') or '业务预热条件尚未满足')
        if action in (*INFERENCE_TOOLS,'inference_run'):
            if body.get('wam_stall_recovery')and not getattr(device.workflows.get('inference'),'wam_stall_recovery_available',True):
                raise ValueError('当前部署未开放 Trick，请使用普通推理')
            extra=({'model_id','expected_bundle_digest'} if action=='inference_select' else set())
            if action=='inference_preview':extra|={'reference_feedforward_sha256','recording_enabled'}
            if action=='inference_run':extra|={'model_id','expected_bundle_digest','expected_context','state_wall_ns','physical_clearance','scene_reset','arms_supported','prepare_only','wam_stall_recovery','memory_start','recording_enabled'}
            if business!='inference' or set(body)-{'environment','business','request_id','seed','duration_s'}-extra:
                raise ValueError('推理请求包含未支持的字段；模型须来自 checkpoint 目录')
            if type(body.get('seed',0)) is not int or not 0<=body.get('seed',0)<2**32:
                raise ValueError('随机种子须为 0～4294967295 的整数')
            episode_duration(body.get('duration_s',120))
            if action=='inference_run' and not {'seed','model_id','expected_bundle_digest','expected_context','state_wall_ns'}.issubset(body):
                raise ValueError('Run 缺少本轮模型、种子或控制上下文')
            if 'prepare_only'in body and type(body['prepare_only'])is not bool:
                raise ValueError('prepare_only requires an explicit boolean')
            if 'wam_stall_recovery'in body and type(body['wam_stall_recovery'])is not bool:
                raise ValueError('wam_stall_recovery requires an explicit boolean')
            if 'recording_enabled'in body and type(body['recording_enabled'])is not bool:raise ValueError('explicit recording selection required')
            if body.get('memory_start','continue')not in('fresh','continue'):raise ValueError('choose fresh or retained memory')
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
        if action=='inference_delete_recordings':
            if business!='inference'or set(body)!={'environment','business','request_id','run_ids','confirmed'}or body['confirmed']is not True:
                raise ValueError('explicit inference recording selection and deletion confirmation required')
            flow=device.workflows.get('inference')
            if flow is None:raise ValueError('推理业务未配置')
            result=flow.recording_files(body['run_ids'])
            with self.lock:self.requests[key]=dict(body=original_body,action=action,response=result)
            return result
        if action in ('inference_hold','inference_home','inference_teach','inference_memory_clear'):
            flow=device.workflows.get('inference')
            if flow is None:raise ValueError('inference workflow is not configured')
            result=self._operator_action(device,flow,action,body)
            with self.lock:self.requests[key]=dict(body=original_body,action=action,response=result)
            return result
        if action=='inference_focus':
            return self._operator_focus_action(device,device.workflows['inference'],body)
        if action in ('precision_preview','precision_execute','precision_analyze','precision_settings','precision_select','precision_notes'):
            if business!='research':raise ValueError('控制研究操作须使用research业务身份')
            fields={'precision_preview':{'kind','speed'},'precision_execute':{'review_id'},'precision_analyze':set(),
                'precision_settings':{'settings','expected_revision'},'precision_select':{'record_id'},'precision_notes':{'record_id','text','expected_revision'}}[action]
            if action=='precision_preview' and 'feedforward_artifact' in body:fields=fields|{'feedforward_artifact'}
            if set(body)!=({'environment','business','request_id'}|fields):raise ValueError('精度研究请求字段不完整或未知')
            if not getattr(device,'precision_study',None):raise ValueError('当前设备未配置精度研究模块')
            gate=self.state(environment,business)['runtime']['allowed_operations'][action]
            if not gate['allowed']:raise ValueError(gate['reason']or '当前不能执行研究操作')
        if action=='inference_execution_settings':
            if business not in ('calibration','inference') or set(body)!={'environment','business','request_id','execute_steps','expected_revision'}:
                raise ValueError('请从标定或推理页面提交执行长度和当前设置版本')
            flow=device.workflows.get('inference')
            if not hasattr(flow,'set_execution_steps'):raise ValueError('当前环境未配置推理服务')
            with self.lock:
                previous=self.requests.get(key)
                if previous:
                    if previous['body']!=original_body:raise ValueError('同一请求身份不可绑定不同操作')
                    return previous['response']
                if any(job.get('status')=='running' for (env,_),job in self.jobs.items() if env==environment):
                    raise ValueError('当前请求尚未结束，请稍后应用推理设置')
                value=flow.set_execution_steps(body['execute_steps'],body['expected_revision'])
                self.reviews.pop((environment,'inference'),None)
                self.inference_generations[environment]+=1
                result=dict(inference_execution=value,message='推理执行长度已保存；旧推理确认已撤销，下一轮生效。')
                self.requests[key]=dict(body=original_body,action=action,response=result)
                while len(self.requests)>256:self.requests.popitem(last=False)
            return result
        if action=='inference_run':body=dict(body,duration_s=body.get('duration_s',120))
        if action=='stop':
            return self.stop_device(device)
        if action=='cancel' and business=='inference':
            if set(body)-{'environment','business','request_id','end_test'}or type(body.get('end_test',False))is not bool:
                raise ValueError('结束推理测试需要显式布尔 end_test')
            workflow=device.workflows.get('inference')
            with self.lock:
                self.inference_generations[environment]+=1
                self.reviews.pop((environment,business),None)
                active=self.jobs.get((environment,business)) or {}
                own_connection=active.get('action')=='inference_run' and active.get('status')=='running'
            if own_connection:device.stop_latched=True
            if device.runtime is not None and (device.runtime.holder=='inference' or own_connection):
                device.submit(business,{'action':'stop'})
            if workflow:
                if body.get('end_test'):workflow.end_test(request_id)
                else:threading.Thread(target=workflow.cancel,daemon=True).start()
            response=dict(message='本次测试结束与清空记忆已受理'if body.get('end_test')else'本轮推理准备或执行已撤销，不会自动重试')
            with self.lock:self.requests[key]=dict(body=original_body,action=action,response=response)
            return response
        if action not in ('preview_target','preview_overview','preview_mesh','preview_recover','mesh','select_result',*HISTORY_ACTIONS):
            with self.lock:
                if any(row.get('status')=='running' and row.get('action')=='inference_run'
                        for (env,_),row in self.jobs.items() if env==environment):
                    raise ValueError('本轮 Run 正在接入或提交；可停止或取消后再更改配置')
        if action in ('inference_mode','inference_finish'):
            workflow=device.workflows.get('inference')
            if business!='inference' or workflow is None:raise ValueError('推理业务未配置')
            call=workflow.request_normal_mode if action=='inference_mode'else workflow.request_normal_finish
            return call({k:v for k,v in body.items() if k not in ('environment','business')})
        if action=='inference_run':
            workflow=device.workflows.get('inference');runtime=device.snapshot('inference')
            reason=self.inference_run_reason(device,workflow,runtime)
            if reason:raise ValueError(reason)
            if body.get('physical_clearance') is not True or body.get('scene_reset') is not True:
                raise ValueError('Run 前须确认场景已复位、工作区无人并允许本轮控制')
            if not runtime.get('connected') and environment=='physical' and body.get('arms_supported') is not True:
                raise ValueError('连接前须在本次 Run 中确认双臂已可靠支撑')
            stamp=body.get('state_wall_ns')
            if type(stamp) is not int or not 0<=time.time_ns()-stamp<=5_000_000_000:
                raise ValueError('Run 页面状态已失鲜，请重新核验后确认')
            if body.get('expected_context')!=self.inference_context(device,runtime,workflow):
                raise ValueError('Run 控制上下文已变化，请重新确认')
            info=workflow.snapshot().get('inference') or {}
            if (not isinstance(body.get('model_id'),str) or body['model_id']!=(info.get('selected_model_id') or (info.get('model') or {}).get('id'))
                    or not isinstance(body.get('expected_bundle_digest'),str)
                    or body['expected_bundle_digest']!=(info.get('bundle_digest') or (info.get('model') or {}).get('bundle_digest'))):
                raise ValueError('Run checkpoint 选择或内容已变化，请重新确认')
        if (environment=='fake' and business=='calibration' and action in WARM_C_ACTIONS
                and (getattr(device,'diagnostic_calibration',None) or {}).get('mode')=='warm_diagnostic'):
            raise ValueError(WARM_C_REASON)
        if action in ('preview_target','preview_overview','preview_mesh','preview_recover'):
            provider=device.preview
            if provider is None:raise ValueError('目标图像服务未配置')
            fields={'preview_target':{'q12','jaw_gap_mm','display_context','renderer_id','view_revision','presented_through_revision'},'preview_overview':{'yaw','pitch','zoom','pan_y','top_view'},
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
                elif action=='preview_overview':result=provider.overview(body['yaw'],body['pitch'],body['zoom'],body.get('pan_y',0.),**({'top_view':body['top_view']}if 'top_view'in body else{}))
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
        if action=='control_policy':
            if set(body)!={'environment','business','request_id','changes','expected_revision'}:
                raise ValueError('统一策略提交须包含字段变更与当前版本')
            runtime=device.runtime
            if runtime is None or not getattr(runtime,'supports_control_policy',False):
                raise ValueError('当前Owner尚未提供统一策略接口')
            with self.policy_write_lock:
                with self.lock:
                    previous=self.requests.get(key)
                    if previous:
                        if previous['body']!=original_body or previous['action']!=action:
                            raise ValueError('同一请求身份不可绑定不同操作')
                        return previous['response']
                    if any(job.get('status')=='running' for (env,_),job in self.jobs.items() if env==environment):
                        raise ValueError('当前请求尚未结束，请稍后应用统一策略')
                    if any(flow.snapshot().get('busy') for flow in device.workflows.values()):
                        raise ValueError('业务任务仍在进行，请稍后应用统一策略')
                changes=deepcopy(body['changes'])
                initial=runtime.control_policy_snapshot(business)
                flow=device.workflows.get('inference')
                if (business=='inference' and initial['origins'].get('execution.execute_steps')=='product_default'
                        and hasattr(flow,'execute_steps') and isinstance(changes,dict)):
                    changes.setdefault('execution.execute_steps',flow.execute_steps)
                policy=runtime.update_control_policy(device.credentials[business],
                    dict(expected_revision=body['expected_revision'],changes=changes))
                result=dict(control_policy=policy,message='统一策略已保存；下一轮使用新策略，请重新审阅轨迹')
                with self.lock:
                    self.requests[key]=dict(body=original_body,action=action,response=result)
                    while len(self.requests)>256:self.requests.popitem(last=False)
            self.invalidate(environment,clear_manual=True)
            return result
        if action=='execution_settings':
            if set(body)!={'environment','business','request_id','settings','expected_revision'}:
                raise ValueError('执行误差设置须包含完整设置与当前版本')
            runtime=device.runtime
            if runtime is None:raise ValueError('请先连接设备')
            with self.policy_write_lock:
                with self.lock:
                    previous=self.requests.get(key)
                    if previous:
                        if previous['body']!=original_body or previous['action']!=action:raise ValueError('同一请求身份不可绑定不同操作')
                        return previous['response']
                    if any(job.get('status')=='running' for (env,_),job in self.jobs.items() if env==environment):
                        raise ValueError('当前请求尚未结束，请稍后应用执行误差设置')
                    if any(flow.snapshot().get('busy') for flow in device.workflows.values()):
                        raise ValueError('业务任务仍在进行，请稍后应用执行误差设置')
                if type(body['expected_revision']) is not int or body['expected_revision']!=runtime.execution_settings_snapshot(business)['revision']:
                    raise ValueError('执行误差设置已变化，请重新读取当前设置后应用')
                result=runtime.update_execution_settings(device.credentials[business],body['settings'],expected_revision=body['expected_revision'])
                result=dict(execution_settings=result,message='执行误差设置已生效；原执行审批已撤销，请重新审阅轨迹')
                with self.lock:
                    self.requests[key]=dict(body=original_body,action=action,response=result)
                    while len(self.requests)>256:self.requests.popitem(last=False)
            self.invalidate(environment,clear_manual=True)
            return result
        if action=='prepare_collection':
            if business!='collection' or set(body)!={'environment','business','request_id'} or device.collection_preparation is None:
                raise ValueError('当前数采后台准备入口未配置')
            state=device.collection_preparation.snapshot()
            if not state['can_retry']:raise ValueError(state['retry_reason'] or '当前数采准备不能重试')
        if business=='collection' and action in ('generate_plan','execute','collection_plan') and device.collection_preparation:
            return_review=action=='execute' and (self.reviews.get((environment,business)) or {}).get('purpose')=='cup_return'
            preparation=device.collection_preparation.snapshot()
            if (preparation.get('requested_asset_pair_id')!=preparation.get('applied_asset_pair_id')
                    and not return_review):
                raise ValueError('所选场景资产尚未准备成功，旧位置与计划不能用于新组合')
            if preparation['status']!='ready' and not return_review:
                restoring=action=='generate_plan' and body.get('restore_frozen_plan') is True
                if (preparation.get('busy') or action=='collection_plan' or body.get('replan')
                        or 'operator_assisted_request_id' in body or not (restoring or frozen_collection_scope(device))):
                    raise ValueError('数采环境尚未准备成功；已有标定保留，请先完成环境准备')
                workflow=device.workflows.get('collection');playground=getattr(workflow,'playground',None)
                generation=self.generations[environment]
                if playground is None:raise ValueError('当前没有可恢复的冻结计划')
                # Reuse the complete artifact/binding/current-joint validator.
                # A request flag never certifies warmup, control, or a valid plan.
                playground.restore_prepared()
                state=playground.snapshot()
                if (generation!=self.generations[environment] or not frozen_collection_scope(device)
                        or not state.get('can_execute') or state.get('plan',{}).get('validated') is not True):
                    raise ValueError('冻结计划或当前控制状态未通过恢复核验')
        if (getattr(device,'cup_return_task',None) or {}).get('submitted') and action not in ('manual_execute','execute'):
            from robot_workbench.cup_return import view as return_view
            if return_view(device,device.snapshot(business),{}).get('busy'):
                raise ValueError('取回动作尚未结束；停止始终可用')
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
                if review.get('purpose')=='cup_return' and body.get('offset_xy_mm',[0.,0.])!=review.get('offset_xy_mm',[0.,0.]):
                    raise ValueError('场景恢复偏移已变化，请重新生成并确认')
                if business=='inference' and action!='manual_execute' and (body.get('seed',review.get('seed'))!=review.get('seed')
                        or episode_duration(body.get('duration_s',120))!=review.get('duration_s',120)):
                    raise ValueError('本轮推理种子或时长已变化，请重新准备并审阅')
                if review.get('purpose')=='cup_return' and (getattr(device,'cup_return_task',None) or {}).get('review_id')!=review_id:
                    raise ValueError('场景恢复任务已撤销，请重新生成并确认')
                if review.get('speed_scale') is not None and (type(body.get('speed_scale',review['speed_scale'])) not in (int,float)
                        or body.get('speed_scale',review['speed_scale'])!=review['speed_scale']):
                    raise ValueError('执行速度已变化，请重新审阅')
            workflow=device.workflows.get(business)
            if workflow and review.get('workflow_revision') is not None and review['workflow_revision']!=workflow.manual_revision():
                raise ValueError('标定计划或求解来源已变化，请重新审阅')
            if workflow and review.get('workflow_generation') is not None and review['workflow_generation']!=workflow.generation:
                raise ValueError('数采任务已变化，请重新审阅')
            if action=='manual_execute' and workflow and (workflow.snapshot().get('busy') or
                    review.get('manual_revision') is not None and review['manual_revision']!=device.workflows['calibration'].manual_revision()):
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
                return_task=None
                if review.get('purpose')=='cup_return':
                    return_task=getattr(device,'cup_return_task',None)
                    if not return_task or return_task.get('review_id')!=review_id:
                        raise ValueError('场景恢复任务已撤销，请重新生成并确认')
                    return_task.update(submitted=True,submitted_monotonic_s=time.monotonic())
            if action=='manual_execute' or review.get('purpose')=='cup_return':
                try:
                    if return_task is not None:
                        from robot_workbench.physical_cup_return import mark_submitted
                        mark_submitted(device,review_id)
                    result=device.submit(business,dict(action='confirm',token=review['token'],
                        confirmation='START_REVIEWED_SINGLE_ARM',physical_clearance=True))
                    return result
                except Exception:
                    with self.lock:
                        self.manual_tasks.pop((environment,business),None)
                        if return_task is not None and getattr(device,'cup_return_task',None) is return_task:
                            return_task['submitted']=False
                    raise
            body={**body,'token':review['token']}
            if review.get('speed_scale') is not None:body['speed_scale']=review['speed_scale']
        if action not in {'precision_preview','precision_execute','precision_analyze','precision_settings','precision_select','precision_notes','connect','prepare_collection','inference_run','fake_scene_update','return_cup','manual_preview','manual_queue','manual_queue_preview','generate_plan','execute','recover','cancel','install','select_result','mesh',*CALIBRATION_TOOLS,*COLLECTION_TOOLS,*INFERENCE_TOOLS,*HISTORY_ACTIONS}:
            raise ValueError('未知操作')
        if business=='collection' and action in ('generate_plan','collection_plan','collection_prepare','fake_scene_update'):
            with self.lock:
                if (self.reviews.get((environment,business)) or {}).get('purpose')=='cup_return':
                    self.reviews.pop((environment,business),None)
            from robot_workbench.cup_return import clear
            clear(device)
        if action not in ('precision_analyze','precision_settings','precision_select','precision_notes','connect','prepare_collection','inference_run',*INFERENCE_MODEL_TOOLS,'mesh','select_result','cancel',*HISTORY_ACTIONS):
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
            if action=='inference_run' and any(row.get('status')=='running' for (env,_),row in self.jobs.items() if env==environment):
                raise ValueError('设备其他请求正在处理，本轮 Run 未开始')
            if action=='inference_run' and body['expected_context']!=self.inference_context(device,device.snapshot('inference'),device.workflows['inference']):
                raise ValueError('本轮 Run 受理前控制上下文已变化')
            job=dict(id=secrets.token_hex(12),action=action,status='running',started_wall_ns=time.time_ns(),request_id=request_id)
            self.jobs[(environment,business)]=job
            if action in ('generate_plan','manual_preview','manual_queue','manual_queue_preview','cancel',*CALIBRATION_TOOLS,*INFERENCE_TOOLS,'collection_plan'):
                self.manual_tasks.pop((environment,business),None)
                self.reviews.pop((environment,business),None)
            response=dict(message='请求已受理，请查看当前任务状态',job_id=job['id'])
            self.requests[key]=dict(body=original_body,action=action,response=response)
            generation=self.generations[environment]
            inference_generation=self.inference_generations[environment]
            flow_generation=getattr(device.workflows.get('inference'),'generation',None)
            if action=='connect' or action=='inference_run' and device.runtime is None:device.stop_latched=False
            while len(self.requests)>256:self.requests.popitem(last=False)
        def work():
            trace=None
            trace_flow=device.workflows.get('inference') if action=='inference_run'and body.get('recording_enabled',False) else None
            try:
                if trace_flow is not None and hasattr(trace_flow,'begin_debug_request'):
                    trace=trace_flow.begin_debug_request(job,original_body)
                    if trace and trace.get('artifact_error'):job['artifact_error']=trace['artifact_error']
                with self.lock:
                    if generation != self.generations[environment]:
                        job.update(status='cancelled',finished_wall_ns=time.time_ns())
                        return
                result=(self.run_inference(device,device.workflows['inference'],body,job,generation,inference_generation,flow_generation)
                        if action=='inference_run' else self.perform(device,business,action,body,request_job_id=job['id']))
                with self.lock:
                    valid=generation==self.generations[environment] and (action!='inference_run' or inference_generation==self.inference_generations[environment])
                    job.update(status='completed' if valid else 'cancelled',
                               result=result,finished_wall_ns=time.time_ns())
            except Exception as error:
                with self.lock:
                    if action=='inference_run'and job.get('operator_stop_request_id')and not job.get('execution_submitted'):
                        device.workflows['inference'].invalidate()
                        job.update(status='completed',result=dict(operator_stopped=True,reason='operator stopped before trajectory submission'),finished_wall_ns=time.time_ns())
                    else:job.update(status='cancelled' if generation!=self.generations[environment] or action=='inference_run' and inference_generation!=self.inference_generations[environment] else 'failed',error=str(error),finished_wall_ns=time.time_ns())
            finally:
                if trace is not None:
                    try:trace_flow.finish_debug_request(trace,deepcopy(job),device.snapshot('inference'))
                    except Exception as error:job['artifact_error']=str(error)
        threading.Thread(target=work,daemon=True,name='workbench-'+action).start()
        return response

    @staticmethod
    def manual_state(task, runtime, review, workflow_busy):
        if not task or workflow_busy or (task['runtime_nonce'],task['epoch'],task.get('business','calibration')) != (
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
            rejection=runtime.get('last_confirm_rejection') or {}
            if (plans and rejection.get('plan_sha256')==plans and rejection.get('command_epoch')==task['epoch']
                    and rejection.get('motion_started') is False and rejection.get('monotonic_s',0)>=submitted):
                expired=rejection.get('reason')=='review_expired'
                status,stage,title='expired' if expired else 'rejected','manual.rejected','人工动作未执行'
                message=('本次确认已过期，动作未执行；重新点击执行重新检查。' if expired else
                         '本次动作在开始前已撤销，未执行；请核对状态后重新点击执行。')
            elif (plans and list(binding.get('plan_sha256',[]))==plans and binding.get('command_epoch')==task['epoch']
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

    def invalidate(self, environment, *, clear_manual=False, workflow_async=False,preserve_collection=False):
        with self.lock:
            self.generations[environment] += 1
            for business in BUSINESSES:
                key=(environment,business)
                self.reviews.pop(key,None)
                if clear_manual or not (self.manual_tasks.get(key) or {}).get('submitted_monotonic_s'):
                    self.manual_tasks.pop(key,None)
        for business,workflow in self.device(environment).workflows.items():
            if preserve_collection and business=='collection':continue
            if workflow_async:
                callback=getattr(workflow,'invalidate',None)
                if callable(callback):threading.Thread(target=callback,daemon=True).start()
            else:workflow.invalidate()

    def invalidate_collection_preparation(self,device):
        """Replacing D preparation revokes its review/checkpoint, never C or motors."""
        with self.lock:self.reviews.pop((device.environment,'collection'),None)
        workflow=device.workflows.get('collection')
        if workflow is not None:workflow.invalidate()

    def perform(self, device, business, action, body, *, request_job_id=None):
        if action.startswith('precision_'):
            study=getattr(device,'precision_study',None)
            if study is None:raise ValueError('当前设备未配置精度研究')
            if action=='precision_preview':return study.prepare(business,body['kind'],body['speed'],body.get('feedforward_artifact'))
            if action=='precision_execute':return study.execute(business,body['review_id'])
            if action=='precision_analyze':return study.analyze()
            if action=='precision_settings':return study.update_settings(body['settings'],body['expected_revision'])
            if action=='precision_select':return study.select(body['record_id'])
            if action=='precision_notes':return study.notes(body['record_id'],body['text'],body['expected_revision'])
        if action=='connect':return device.connect(body)
        if action=='prepare_collection':return device.collection_preparation.retry()
        if business=='inference' and action in INFERENCE_MODEL_TOOLS:
            workflow=device.workflows.get('inference')
            if workflow is None:raise ValueError('推理工作流未配置')
            if action=='inference_refresh':return workflow.refresh_models()
            if action=='inference_select':
                return workflow.select_model(body.get('model_id'),expected_bundle_digest=body.get('expected_bundle_digest'))
            if device.runtime is not None and device.runtime.holder=='inference':
                return workflow.action(action,body)  # Preserve the connected prepare/bind diagnostic flow.
            if hasattr(workflow,'prepare_model'):return workflow.prepare_model(seed=body.get('seed',0))
            return workflow.action(action,body)  # Legacy fixed-model diagnostic endpoint.
        runtime=device.runtime
        if runtime is None and action not in ('mesh','select_result',*HISTORY_ACTIONS):raise ValueError('请先连接设备')
        if runtime is not None and runtime.holder != business and action not in ('mesh','select_result','cancel',*HISTORY_ACTIONS):
            raise PermissionError('请先交接控制权限：当前由 '+runtime.holder+' 持有')
        if action=='return_cup':
            if business!='collection':raise ValueError('场景恢复只允许数采业务')
            if set(body)-{'environment','business','request_id','offset_xy_mm'}:raise ValueError('场景恢复只接受前后、左右偏移，不接受其他目标或速度')
            from robot_workbench.cup_return import installed_pair
            if not installed_pair(device).get('verified'):
                raise ValueError('请先核验原轨迹使用的标定安装组')
            with self.lock:self.reviews.pop((device.environment,business),None)
            from robot_workbench.cup_return import prepare,clear
            clear(device)
            result=prepare(device,body.get('offset_xy_mm'))
            self.add_review(device,business,result['preview'],'execute',result['summary'],request_job_id=request_job_id)
            return {'message':'场景恢复计划已准备，请确认执行'}
        if action=='fake_scene_update':
            if device.environment!='fake' or business!='collection' or not callable(getattr(device,'fake_scene_update',None)):
                raise ValueError('场景移动仅允许完整 fake 数采环境')
            if set(body)-{'environment','business','request_id','cup_xy_m','machine_xy_m'}:
                raise ValueError('场景移动只接受杯子和咖啡机的平面偏移')
            return device.fake_scene_update({key:body[key] for key in ('cup_xy_m','machine_xy_m') if key in body})
        if action in ('manual_preview','manual_queue','manual_queue_preview'):
            if action!='manual_preview' and business!='calibration':raise ValueError('标定动作队列仅属于标定业务')
            gate=device.snapshot(business)['allowed_operations']['preview']
            if not gate['allowed']:raise ValueError(gate['reason'] or '当前控制器不允许人工规划')
            workflow=device.workflows.get('calibration')
            if any(flow.snapshot().get('busy') for flow in device.workflows.values()):raise ValueError('请先结束当前业务任务，再进行人工规划')
            with self.lock:self.reviews.pop((device.environment,business),None)
            if action=='manual_queue':
                if not workflow or not hasattr(workflow,'manual_queue'):raise ValueError('未配置标定动作队列')
                return workflow.manual_queue(body)
            if workflow and hasattr(workflow,'manual_preview'):
                preview=workflow.manual_preview(body,queue=action=='manual_queue_preview',**({'business':business} if business!='calibration' else {}))
            else:
                if action=='manual_queue_preview' or device.environment!='fake':raise ValueError('未配置标定路径校验与队列服务')
                from arx_env.joint_head_real_motion import DEFAULT_VELOCITY
                command=dict(action='preview_workbench',waypoints=[dict(q12=body.get('q12'),
                    speeds12=body.get('speeds12',[DEFAULT_VELOCITY]*12),jaw_gap_mm=body.get('jaw_gap_mm'),jaw_speed_mm_s=body.get('jaw_speed_mm_s',10.))])
                preview=device.submit(business,command)
            self.add_review(device,business,preview,'manual_execute',
                f"人工{'队列' if action=='manual_queue_preview' else '姿态'}：{len(preview.get('plans',[]))} 个路径点，预计运动 {preview['total_motion_duration_s']:.2f} 秒；按冻结的关节/夹爪速度整体执行"+
                ('。粗球自碰撞检查通过；未检查外部障碍物。' if (preview.get('collision_check') or {}).get('scope')=='robot_self_spheres' else ''),
                request_job_id=request_job_id)
            return {'message':'轨迹已准备，等待确认执行'}
        if action=='recover':
            state=device.snapshot(business)
            return device.submit(business,dict(action='recover',confirmation='ACKNOWLEDGE_MOTION_FAULT',
                owner_pid=state['pid'],command_epoch=state['command_epoch']))
        workflow=device.workflows.get(business)
        if workflow is None:raise ValueError('当前业务后端尚未配置')
        if action in INFERENCE_TOOLS:
            gate=device.snapshot(business)['allowed_operations']['preview']
            if not gate['allowed']:raise ValueError(gate['reason'] or '当前设备不允许准备推理任务')
            if hasattr(workflow,'allowed'):
                gate=workflow.allowed(action)
                if not gate['allowed']:raise ValueError(gate.get('reason') or '当前推理操作不可用')
        if business=='calibration' and action in (*CALIBRATION_TOOLS,'generate_plan'):
            gate=device.snapshot(business)['allowed_operations']['preview']
            if not gate['allowed']:raise ValueError(gate['reason'] or '当前设备不允许更改标定任务')
        result=workflow.action(action,body)
        if isinstance(result,dict) and result.get('preview'):
            self.add_review(device,business,result['preview'],'execute',result['summary'])
        if action=='install':self.invalidate(device.environment)
        return result

    def add_review(self, device, business, preview, action, summary, *, request_job_id=None):
        state=device.snapshot(business)
        workflow=device.workflows.get(business)
        manual_provider=device.workflows.get('calibration')
        manual_revision=(preview.get('review_generation',manual_provider.manual_revision())
                         if action=='manual_execute' and manual_provider and hasattr(manual_provider,'manual_revision') else None)
        with self.lock:
            generation=preview.get('workflow_generation')
            if generation is not None and (workflow is None or generation!=workflow.generation):return
            if business=='collection' and device.collection_preparation and preview.get('purpose')!='cup_return':
                if device.collection_preparation.snapshot()['status']!='ready':
                    if not frozen_collection_scope(device):return
                    with workflow.playground.lock:plan=workflow.playground.state.get('plan') or {}
                    if plan.get('validated') is not True or plan.get('timeline_digest')!=preview['token']:return
            review=dict(id=secrets.token_hex(16),action=action,
                token=preview['token'],summary=summary,physical_clearance_required=device.environment=='physical',
                purpose=preview.get('purpose'),
                speed_scale=preview.get('speed_scale') if business=='collection' else None,
                owner_preview=bool(preview.get('owner_pid')),
                manual_revision=manual_revision,
                workflow_revision=workflow.manual_revision() if business=='calibration' and workflow and hasattr(workflow,'manual_revision') else None,
                workflow_generation=generation,
                deadline=time.monotonic()+min(30.,preview.get('expires_in_s',30.)),
                runtime_nonce=preview.get('runtime_nonce',state['runtime_nonce']),epoch=preview['command_epoch'],
                calibration_revision=state.get('calibration_revision'))
            self.reviews[(device.environment,business)]=review
            if business=='inference':
                review.update(seed=preview.get('seed',0),duration_s=preview.get('duration_s',120),
                              execution_duration_s=preview.get('execution_duration_s',preview.get('duration_s',120)))
            if preview.get('purpose')=='cup_return':
                review['request_job_id']=request_job_id
                device.cup_return_task['review_id']=review['id']
                for name in ('offset_xy_mm','origin_position_m','goal_position_m'):
                    if name in preview:review[name]=deepcopy(preview[name])
            if action=='manual_execute':
                from arx_env.joint_head_jaw_mapping import command_gap_mm
                review.update(request_job_id=request_job_id,total_motion_duration_s=preview['total_motion_duration_s'],
                    waypoint_jaw_gap_mm=[[command_gap_mm(side,plan['q_target'][i]) for side,i in (('left',6),('right',13))]
                                        for plan in preview.get('plans',[])])
                self.manual_tasks[(device.environment,business)]=dict(review,business=business,
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
                if path.path=='/api/inference-diagnostics':
                    if (set(query)-{'environment','after','owner_after','limit'}or 'environment'not in query
                            or any(len(v)!=1 for v in query.values())):
                        raise ValueError('one explicit environment and bounded diagnostic cursors required')
                    flow=app.device(query['environment'][0]).workflows.get('inference')
                    if flow is None:raise ValueError('推理业务未配置')
                    options={k:int(query[k][0])for k in ('after','owner_after','limit')if k in query}
                    return self.send(200,flow.pipeline_diagnostics(**options))
                if path.path=='/api/inference-replay-metadata':
                    if set(query)!={'environment','run_id'}or any(len(v)!=1 for v in query.values()):
                        raise ValueError('one explicit environment and recording Run required')
                    flow=app.device(query['environment'][0]).workflows.get('inference')
                    if flow is None:raise ValueError('推理业务未配置')
                    return self.send(200,flow.replay_metadata(query['run_id'][0]))
                if path.path=='/api/inference-recordings':
                    if set(query)!={'environment','business'}or any(len(v)!=1 for v in query.values())or query['business'][0]!='inference':
                        raise ValueError('one explicit inference environment required')
                    flow=app.device(query['environment'][0]).workflows.get('inference')
                    if flow is None:raise ValueError('推理业务未配置')
                    return self.send(200,dict(recordings=flow.recording_files()))
                if path.path=='/api/triton/catalog' or path.path.startswith('/api/triton/jobs/'):
                    if set(query)!={'environment'} or len(query['environment'])!=1:raise ValueError('one explicit environment required')
                    job_id=path.path.removeprefix('/api/triton/jobs/') if path.path.startswith('/api/triton/jobs/') else None
                    return self.send(200,app.triton_read(query['environment'][0],job_id))
                if path.path=='/api/state':
                    paging = {}
                    for key in ('results_before', 'results_after'):
                        if key in query:
                            if len(query[key]) != 1:raise ValueError('one history cursor value required')
                            paging[key] = int(query[key][0])
                    from .state_contract import page_state
                    return self.send(200,page_state(app.state(query.get('environment',['fake'])[0],query.get('business',['calibration'])[0],**paging)))
                if path.path=='/api/control-policy':
                    if set(query)!={'environment','business'} or any(len(values)!=1 for values in query.values()):
                        raise ValueError('one environment and business required')
                    return self.send(200,app.control_policy(query['environment'][0],query['business'][0]))
                if path.path=='/api/environment':
                    if set(query)!={'environment'} or len(query['environment'])!=1:raise ValueError('one explicit environment required')
                    return self.send(200,app.environment_state(query['environment'][0]))
                if path.path=='/api/scene-assets/preview':
                    if set(query)!={'environment','asset_id','catalog_digest'}or any(len(v)!=1 for v in query.values()):
                        raise ValueError('明确的资产、环境与目录版本必需')
                    preparation=app.device(query['environment'][0]).collection_preparation
                    catalog=getattr(preparation,'asset_catalog',None)
                    if catalog is None:raise ValueError('场景资产目录尚未登记')
                    raw,kind=catalog.preview(query['asset_id'][0],query['catalog_digest'][0])
                    return self.send(200,raw,kind,{'Cache-Control':'private, max-age=3600'})
                if path.path=='/api/fake-scene':
                    if query.get('environment',[''])[0]!='fake' or query.get('business',[''])[0]!='collection':
                        raise ValueError('仿真场景实测只读入口仅用于 Fake 数采诊断')
                    device=app.device('fake')
                    snapshot=getattr(device,'fake_scene_snapshot',None)
                    if not callable(snapshot):raise ValueError('当前设备未配置仿真场景实测')
                    return self.send(200,snapshot())
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
                    result=workflow.download(query.get('result_id',[''])[0])
                    name='calibration-'+result['sha256'][:12]+'.json' if result.get('schema')=='robot_workbench.calibration_bundle.v1' else 'calibration.json'
                    return self.send(200,result,headers={'Content-Disposition':'attachment; filename="'+name+'"'})
                if path.path=='/api/result-media':
                    device=app.device(query.get('environment',['fake'])[0])
                    business=query.get('business',['calibration'])[0]
                    if business not in BUSINESSES:raise ValueError('未知业务')
                    workflow=device.workflows.get(business)
                    result_id=query.get('result_id',[''])[0]
                    returned=getattr(device,'cup_return_task',None) if business=='collection' else None
                    return_media=bool(returned and returned.get('media_id')==result_id)
                    file=Path(returned['directory'])/'plan.json' if return_media else workflow.media(result_id) if workflow else None
                    if file is None and device.capture_results is not None and business in device.capture_results.BUSINESSES:
                        file=device.capture_results.media(business,result_id)
                    history=getattr(device,'collection_history',None) if business=='collection' else None
                    if file is None and history is not None:file=history.media(result_id)
                    if file is None:return self.send(404,{'error':'本轮或所选历史媒体尚未就绪'})
                    from robot_workbench.http_media import serve_file
                    kind=mimetypes.guess_type(str(file))[0]
                    current_plan=(kind=='application/json' and business=='collection'
                        and (return_media or workflow and (workflow.snapshot().get('trajectory') or {}).get('id')==result_id))
                    debug_receipt=(business=='inference' and result_id.startswith('debug-')
                        and kind=='application/json' and file.name=='debug-receipt.json')
                    capture_receipt=(result_id.startswith('capture-')and kind=='application/json'
                        and re.fullmatch(r'receipt-r\d{6}\.json',file.name))
                    if kind not in ('video/mp4','image/png','image/jpeg') and not current_plan and not debug_receipt and not capture_receipt:raise ValueError('不支持的回放媒体类型')
                    return serve_file(self,file,content_type=kind)
                if path.path=='/api/recording-quality':
                    device=app.device(query.get('environment',['fake'])[0]);business=query.get('business',[''])[0]
                    if business not in ('collection','inference'):raise ValueError('recording business required')
                    result_id=query.get('result_id',[''])[0];workflow=device.workflows.get(business)
                    file=workflow.media(result_id) if workflow else None
                    from .recording_quality import download_for_playback
                    download=download_for_playback(Path(file).parent) if file and Path(file).suffix=='.mp4' else None
                    history=getattr(device,'collection_history',None) if business=='collection' else None
                    if download is None and history is not None:
                        reader=getattr(history,'quality_report',None)
                        if reader:download=reader(result_id)
                    if download is None:return self.send(404,{'error':'quality report not available'})
                    report=download['report']
                    value=dict(report,download_identity=dict(result_id=result_id,sidecar_sha256=download['sidecar_sha256'],
                        raw_manifest_sha256=report['raw_manifest_sha256'],environment=device.environment,business=business))
                    return self.send(200,value,headers={'Content-Disposition':'attachment; filename="recording-quality.json"'})
                if path.path.startswith('/api/media/'):
                    device=app.device(query.get('environment',['fake'])[0])
                    name=path.path.removeprefix('/api/media/')
                    if name not in device.media:return self.send(404,{'error':'unknown media'})
                    raw,meta=device.media[name].frame()
                    headers={'ETag':meta['etag'],'X-Frame-Id':meta['frame_id'],'X-Source-Wall-Ns':meta['source_wall_ns']}
                    if self.headers.get('If-None-Match')==meta['etag']:return self.send(304,b'','image/jpeg',headers)
                    return self.send(200,raw,'image/jpeg',headers)
                resource=path.path.removeprefix('/web') if path.path.startswith('/web/') else path.path
                if resource not in ('/','/index.html','/app.js','/style.css','/session.js','/site_cameras.js','/triton.js',
                                    '/physical_endpoint.js','/operations.js','/model_probe.js','/target_viewer.js',
                                    '/vendor/three.module.js','/vendor/OrbitControls.js'):
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
                if not path.startswith(('/api/actions/','/api/maintenance/','/api/triton/')):return self.send(404,{'error':'not found'})
                raw=self.rfile.read(size)
                if len(raw)!=size:raise ValueError('request body is incomplete')
                self.close_connection=not keep_alive
                result=(app.triton_action(path.removeprefix('/api/triton/'),json.loads(raw)) if path.startswith('/api/triton/') else
                    app.maintenance_action(path.removeprefix('/api/maintenance/'),json.loads(raw))
                    if path.startswith('/api/maintenance/') else app.action(path.removeprefix('/api/actions/'),json.loads(raw)))
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
