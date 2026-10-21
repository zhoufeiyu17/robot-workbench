"""Exclusive service maintenance; expiry and action resumption are always explicit."""
from copy import deepcopy
import secrets
import time

ACTIONS=('ops_maintenance_begin','ops_maintenance_end','ops_rebind_cameras',
         'ops_restart_cameras','ops_start_cameras','ops_stop_cameras','ops_prepare_model','ops_invalidate_model','ops_prepare_collection')


def public_state(window):
    return ({key:deepcopy(window[key]) for key in ('purpose','started_wall_ns','status','phase',
        'active_action','error','target') if key in window} if window else None)


def busy_reason(device):
    if device.runtime is not None and getattr(device.runtime,'_hold_verification',None) is not None:
        return '固定保持验证或Stop回执尚未完成'
    for flow in device.workflows.values():
        # Collection's business view is cached; reuse its live operations facts.
        reader=getattr(flow,'environment_status',None)
        current=reader() if callable(reader) else {}
        if type(current.get('busy')) is not bool:current=flow.snapshot()
        if current.get('busy'):
            return '业务仍在执行或收尾，请先停止任务并等待完成'
    preparation=device.collection_preparation
    if preparation is not None and preparation.snapshot().get('busy'):
        return '数采环境仍在准备或收尾'
    if device.connecting or device.runtime is None and device.thread is not None and device.thread.is_alive():
        return '设备连接仍在准备或收尾'
    return None


ALIASES={
    'begin':'ops_maintenance_begin','end':'ops_maintenance_end',
    'start_cameras':'ops_start_cameras','restart_cameras':'ops_restart_cameras',
    'stop_cameras':'ops_stop_cameras','invalidate_model':'ops_invalidate_model',
    'rebind_cameras':'ops_rebind_cameras','prepare_model':'ops_prepare_model',
    'prepare_collection':'ops_prepare_collection',
    'connect_control':'connect_control','release_control':'release_control',
    'recover_control':'recover_control','verify_hold':'verify_hold','stop':'stop','inspect':'inspect','receipt':'receipt'}
PURPOSES={
    'connect_control':{'connect_control','ops_start_cameras','ops_rebind_cameras'},'release_control':{'release_control'},
    'recover_control':{'recover_control','release_control','connect_control','ops_start_cameras','ops_rebind_cameras'},
    'verify_hold':{'verify_hold'},'prepare_collection':{'ops_prepare_collection'},
    'start_cameras':{'ops_start_cameras','ops_rebind_cameras'},
    'stop_cameras':{'ops_stop_cameras'},'stop_model':{'ops_invalidate_model'},
    'restart_cameras':{'ops_restart_cameras','ops_rebind_cameras'},
    'rebind_cameras':{'ops_rebind_cameras'},'prepare_model':{'ops_prepare_model'},
    'restart_model':{'ops_prepare_model'},'restart_workbench':{'release_control','ops_prepare_model'},
    'stop_workbench':{'release_control'},
    'prepare_environment':{'ops_start_cameras','ops_rebind_cameras','ops_prepare_model','ops_prepare_collection'},
    **{name:{'ops_rebind_cameras'} for name in ('reset_camera_head','reset_camera_left','reset_camera_right')}}


def target(application,device):
    runtime=device.runtime
    return dict(server_instance_id=application.server_instance_id,device_id=device.id,
                runtime_nonce=runtime.nonce if runtime is not None else None)


class ControlActor:
    """A private, window-scoped actor; it never registers as a business holder."""
    def __init__(self,application,device,window,action):
        self.application,self.device,self.window,self.action=application,device,window,action
    def _same_window(self):
        return (self.application.maintenance.get(self.device.environment) is self.window
                and self.window.get('active_action')==self.action)
    def same_window(self):
        with self.application.lock:return self._same_window()
    def current(self):
        with self.application.lock:
            return (self._same_window() and not self.window.get('_cancelled')
                    and getattr(self.device,'stop_serial',0)==self.window['_stop_serial']
                    and self.device.runtime is self.window['_runtime'])
    def require(self,runtime):
        if not self.current() or runtime is not self.window['_runtime']:
            raise ValueError('维护实例或Stop代次已变化；本次操作已撤销')
    def adopt(self,runtime):
        with self.application.lock:
            if not self._same_window() or self.device.runtime is not runtime:
                raise ValueError('不能接管未知的Runtime变化')
            previous=self.window['_runtime']
            if self.action=='connect_control':
                retired=getattr(previous,'protected_retirement',None)
                if ((previous is not None and (not retired or retired.get('worker_exit_confirmed') is not True or previous.fd is not None))
                        or runtime is None or runtime is previous):raise ValueError('管理接入只能绑定本次新Runtime或已确认退役的保护实例')
            elif self.action=='release_control':
                receipt=previous.release_receipt if previous is not None else None
                if runtime is not None or not receipt or receipt.get('status')!='released' or receipt.get('sdk_close_returned') is not True:
                    raise ValueError('SDK释放与设备锁收尾未确认')
            else:raise ValueError('此维护动作不能替换Runtime')
            self.window['_runtime']=runtime
            self.window['target']=target(self.application,self.device)
    def lifecycle_admission(self,step):
        if self.action!='ops_prepare_collection':raise PermissionError('此维护动作不能管理数采服务生命周期')
        if not isinstance(step,str) or step not in {'start:collection','start:detector','start:planner','start:scene','recover:detector','recover:planner','recover:scene','stop:detector','stop:planner','stop:scene'}:
            raise ValueError('未知数采生命周期步骤')
        return self.same_window() if step.startswith('stop:') else self.current()


def apply(application,device,action,body):
    """Legacy ops_* adapter; all maintenance behavior lives in operate()."""
    value={k:v for k,v in body.items() if k!='business'}
    value.setdefault('expected_target',target(application,device))
    return operate(application,device,action,value)


def operate(application,device,action,body):
    action=ALIASES.get(action,action)
    if action not in {*ALIASES.values()}:raise ValueError('未知维护动作')
    begin=action=='ops_maintenance_begin'
    fields={'environment','request_id','expected_target'}|({'purpose'} if begin else set() if action=='stop' else {'maintenance_token'})
    if action=='ops_prepare_model':fields|={'model_id','expected_bundle_digest','expected_model_id'}
    if action=='ops_prepare_collection':fields|={'service_role','restart'}
    if action=='receipt':fields.add('lookup_request_id')
    if action in ('connect_control','release_control','recover_control','verify_hold'):fields.add('arms_supported')
    if not isinstance(body,dict) or set(body)-fields:raise ValueError('维护请求包含未知字段')
    if action=='ops_prepare_collection' and {'service_role','restart'}&set(body):
        if body.get('service_role') not in ('detector','planner','scene') or body.get('restart')is not True:
            raise ValueError('单角色重启只接受 detector/planner/scene 与 restart=true')
        if device.collection_preparation is None:raise ValueError('旧固定配置无已验证parts/installation_identity，不支持单角色重启')
    if body.get('environment')!=device.environment or not isinstance(body.get('request_id'),str) or not 1<=len(body['request_id'])<=128:
        raise ValueError('明确的环境与维护请求身份必需')
    expected=body.get('expected_target')
    required={'server_instance_id','device_id'}| (set() if action in ('stop','inspect','receipt') else {'runtime_nonce'})
    if (not isinstance(expected,dict) or not required<=set(expected) or set(expected)-{'server_instance_id','device_id','runtime_nonce'}
            or expected.get('server_instance_id')!=application.server_instance_id or expected.get('device_id')!=device.id):
        raise ValueError('维护请求的设备或服务实例已变化')
    if begin and body.get('purpose') not in PURPOSES:raise ValueError('未支持的维护操作')
    for name in ('model_id','expected_model_id'):
        if name in body and (not isinstance(body[name],str) or not body[name]):raise ValueError('明确的原模型ID必需')
    environment=device.environment;key=(environment,'maintenance',body['request_id'])
    if action=='receipt':
        lookup=body.get('lookup_request_id');token=body.get('maintenance_token')
        if not isinstance(lookup,str) or not 1<=len(lookup)<=128:raise ValueError('原始维护请求身份必需')
        with application.lock:
            prior=application.requests.get((environment,'maintenance',lookup))
            if prior is None:return dict(status='unknown',request_id=lookup,reason='request_not_retained')
            expected_token=prior['body'].get('maintenance_token')
            if not isinstance(token,str) or not expected_token or not secrets.compare_digest(token,expected_token):
                raise ValueError('原始维护凭证不匹配')
            return dict(status='failed' if prior.get('error') else 'completed' if prior.get('completed') else 'running',
                request_id=lookup,action=prior['action'],response=deepcopy(prior.get('response')),
                error=prior.get('error'),target=target(application,device))
    if action=='inspect':
        with application.lock:
            window=application.maintenance.get(environment);token=body.get('maintenance_token')
            if not window or not isinstance(token,str) or not secrets.compare_digest(token,window['token']):
                raise ValueError('维护身份已失效，请重新核验服务')
            current=target(application,device)
            return dict(target=current,runtime_same=device.runtime is window['_runtime'] and current==window['target'],
                active_action=window.get('active_action'),status=window['status'],phase=window.get('phase'),error=window.get('error'))
    with application.lock:
        window=application.maintenance.get(environment);prior=application.requests.get(key)
        if begin and window and window['_begin_key']==key:
            if window['_begin_body']!=body:raise ValueError('同一请求身份不可绑定不同操作')
            return dict(public_state(window),token=window['token'],target=deepcopy(window['target']))
        if prior:
            if prior['action']!=action or prior['body']!=body:raise ValueError('同一请求身份不可绑定不同操作')
            if prior.get('error'):raise ValueError(prior['error'])
            return deepcopy(prior['response'])
    if action=='stop':
        result=application.stop_device(device)
        with application.lock:application.requests[key]=dict(body=deepcopy(body),action=action,response=deepcopy(result))
        return result
    with application.lock:
        window=application.maintenance.get(environment)
        if expected!=target(application,device):raise ValueError('维护请求Runtime已变化，请核验当前实例')
        if application.action_calls[environment] or any(job.get('status')=='running' for (env,_),job in application.jobs.items() if env==environment):
            raise ValueError('当前请求仍在处理，等待收尾后再维护')
        if begin:
            if window:raise ValueError('已有维护操作，请核验其结果后结束维护')
            if not device.lock.acquire(blocking=False):raise ValueError('设备连接正在准备或收尾')
            try:
                if device.connecting or device.runtime is None and device.thread is not None and device.thread.is_alive():
                    raise ValueError('设备连接正在准备或收尾')
                window=dict(token=secrets.token_hex(24),purpose=body['purpose'],started_wall_ns=time.time_ns(),
                    status='starting',phase='checking_idle',active_action=action,error=None,target=deepcopy(expected),
                    _runtime=device.runtime,_begin_key=key,_begin_body=deepcopy(body),_ready=False,
                    _stop_serial=getattr(device,'stop_serial',0),_cancelled=False)
                application.maintenance[environment]=window;device.maintenance=True
            finally:device.lock.release()
            response=dict(public_state(window),token=window['token'],target=deepcopy(window['target']))
        else:
            token=body.get('maintenance_token')
            if not window or not isinstance(token,str) or not secrets.compare_digest(token,window['token']):
                raise ValueError('维护身份已失效，请重新核验服务')
            if window.get('active_action'):raise ValueError('维护操作尚未返回，请等待处理完成')
            if expected!=window['target'] or device.runtime is not window['_runtime']:
                raise ValueError('维护票据绑定的Runtime已变化')
            if action!='ops_maintenance_end':
                if action not in PURPOSES[window['purpose']]:raise ValueError('维护票据不包含此操作作用域')
                if not window['_ready']:raise ValueError('维护尚未就绪，请先核验失败原因并结束维护')
                if window.get('_cancelled') or getattr(device,'stop_serial',0)!=window['_stop_serial']:
                    raise ValueError('Stop已撤销本维护计划，请核验后结束维护')
            window.update(status='working',active_action=action,phase=action,error=None)
            response=dict(status='running',action=action)
        entry=dict(body=deepcopy(body),action=action,response=response);application.requests[key]=entry
        while len(application.requests)>256:application.requests.popitem(last=False)
    flow=device.workflows.get('inference');claimed=False;actor=ControlActor(application,device,window,action)
    try:
        if begin:
            reason=busy_reason(device)
            if reason:raise ValueError(reason)
            if window['_runtime'] is not None:window['_runtime'].begin_maintenance()
            claimed=True
            if flow is not None and hasattr(flow,'pause_observations'):flow.pause_observations()
            # Model lifecycle has no dependency on frozen collection geometry.
            # Every maintenance entry still revokes all execution approvals.
            application.invalidate(environment,clear_manual=True,preserve_collection=window['purpose'] in
                ('prepare_collection','prepare_model','restart_model','stop_model'))
            if not actor.current():raise ValueError('Stop已撤销维护准入')
            window['_ready']=True;result=None
        elif action=='ops_maintenance_end':
            reason=busy_reason(device)
            if reason:raise ValueError(reason)
            receipt=getattr(window['_runtime'],'release_receipt',None)
            if receipt and receipt.get('status')!='released':raise ValueError('SDK释放尚未确认，保持维护冻结')
            if flow is not None and hasattr(flow,'resume_observations'):flow.resume_observations()
            if window['_runtime'] is not None:window['_runtime'].end_maintenance()
            with application.lock:
                application.maintenance.pop(environment);device.maintenance=False
                receipt=application.requests.get(window['_begin_key'])
                if receipt:receipt['response']=dict(public_state(window),token=window['token'],status='ended',active_action=None,target=deepcopy(window['target']))
            result=dict(ended=True,motion_resumed=False,target=deepcopy(window['target']))
        else:
            reason=busy_reason(device)
            if reason:raise ValueError(reason)
            actor.require(device.runtime)
            if action=='connect_control':
                device.connect(body,maintenance_actor=actor)
                deadline=time.monotonic()+4
                while time.monotonic()<deadline:
                    actor.require(device.runtime);facts=device.fact_snapshot()
                    feedback=facts.get('calibration_feedback') or {}
                    age=device.runtime.owner.control_time()-feedback.get('sample_time_s',-1)
                    if facts.get('feedback_healthy') is True and facts.get('stationary') is True and 0<=age<=.1:break
                    if str(facts.get('state')).upper() in ('FAULT','EMERGENCY','FAULT_STOP_UNCONFIRMED'):raise ValueError('接入后设备报告控制故障，维护保持冻结')
                    time.sleep(.01)
                else:raise ValueError('接入后未取得新鲜健康静止反馈，维护保持冻结')
                result=dict(connected=True,runtime=facts,target=deepcopy(window['target']))
            elif action=='release_control':
                if device.environment=='physical' and body.get('arms_supported') is not True:raise ValueError('释放前必须确认双臂已可靠支撑')
                runtime=device.runtime
                if runtime is None:raise ValueError('设备未连接')
                runtime.control_release(actor)
                deadline=time.monotonic()+10
                while time.monotonic()<deadline:
                    if device.runtime is None:
                        if device.thread is not None and device.thread.is_alive():
                            time.sleep(.01);continue
                        actor.adopt(None);break
                    actor.require(runtime);time.sleep(.01)
                else:raise ValueError('SDK释放尚未确认，维护保持冻结')
                result=dict(connected=False,release_receipt=deepcopy(device.release_receipt),target=deepcopy(window['target']))
            elif action=='recover_control':
                runtime=device.runtime
                if runtime is None:raise ValueError('设备未连接；恢复不会自动接入')
                result=runtime.control_recover(actor);result['target']=deepcopy(window['target'])
            elif action=='verify_hold':
                if device.environment=='physical' and body.get('arms_supported') is not True:raise ValueError('固定保持验证前必须确认当前双臂支撑条件')
                runtime=device.runtime
                if runtime is None:raise ValueError('设备未连接；固定保持验证不会自动接入')
                result=runtime.verify_hold(actor);result['target']=deepcopy(window['target'])
            elif action in ('ops_rebind_cameras','ops_restart_cameras','ops_start_cameras','ops_stop_cameras'):
                cameras=getattr(device,'camera_service',None)
                if cameras is None:raise ValueError('当前部署未配置共同相机服务')
                if action=='ops_rebind_cameras':
                    deadline=time.monotonic()+5
                    while True:
                        actor.require(device.runtime)
                        try:result=cameras.reconnect(allow_current=True);break
                        except ValueError:
                            if not cameras.reconnect_state().get('active_calls') or time.monotonic()>=deadline:raise
                            time.sleep(.02)
                else:
                    callback=getattr(device,{'ops_restart_cameras':'camera_restart','ops_start_cameras':'camera_start',
                        'ops_stop_cameras':'camera_stop'}[action],None)
                    if callback is None:raise ValueError('本机相机源由独立服务管理，请使用管理入口')
                    def progress(phase,fraction=None,detail=None):
                        actor.require(device.runtime)
                        with application.lock:window['phase']=phase
                    result=callback(progress)
                if flow is not None:
                    flow._revoke_device_review()
                    with flow.lock:
                        flow.prepared=None;flow.cache=None;flow.support_cache=None
                        flow.cache_error=('相机源已停止' if action=='ops_stop_cameras' else '相机绑定已更新')+'；旧审阅失效，模型热状态保留'
            elif action=='ops_invalidate_model':
                callback=getattr(flow,'operations_invalidate_model',None)
                if not callable(callback):raise ValueError('当前环境未装配运维模型失效hook')
                result=callback(still_current=actor.current)
            elif action=='ops_prepare_model':
                callback=getattr(flow,'operations_prepare_model',None)
                if not callable(callback):raise ValueError('当前环境未装配运维模型准备hook')
                result=callback(**{name:body[name] for name in ('model_id','expected_bundle_digest','expected_model_id') if name in body},still_current=actor.current)
            elif action=='ops_prepare_collection':
                if device.runtime is None:raise ValueError('数采准备需要先在运维接入设备；不会自动连接')
                device.runtime.admit_business('collection',active=actor.current,maintenance_actor=actor)
                from .cup_return import clear
                clear(device,maintenance_actor=actor)
                callback=getattr(device.collection_preparation or device.workflows.get('collection'),'operations_prepare',None)
                if not callable(callback):raise ValueError('当前环境未装配运维数采准备hook')
                result=callback(still_current=actor.current,lifecycle_admission=actor.lifecycle_admission,
                    **{name:body[name] for name in ('service_role','restart') if name in body})
            actor.require(device.runtime)
        with application.lock:
            window.update(status='ready',phase='idle',active_action=None)
            if begin:result=dict(public_state(window),token=window['token'],target=deepcopy(window['target']))
            entry['response']=deepcopy(result)
            entry['completed']=True
        return result
    except Exception as error:
        with application.lock:
            window.update(status='failed',error=str(error),active_action=None)
            if begin and claimed:
                entry['response']=dict(public_state(window),token=window['token'],target=deepcopy(window['target']))
                return deepcopy(entry['response'])
            if begin:application.maintenance.pop(environment,None);device.maintenance=False
            entry['error']=str(error)
            entry['completed']=True
        raise
