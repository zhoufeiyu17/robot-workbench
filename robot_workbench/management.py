"""Persistent local management plane. No browser-provided commands or SDK calls."""
from contextlib import contextmanager
from copy import deepcopy
from http.client import HTTPConnection, HTTPSConnection, HTTPException
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import mimetypes
import os
from pathlib import Path
import re
import secrets
import shutil
import stat
import subprocess
import tempfile
import threading
import time
from urllib.parse import parse_qs, urlsplit, urlencode

from .managed_services import ManagedService, ServiceSpec
from .operations import Operation, OperationError, OperationService
from .machine_profile import MachineProfile, Deployment
from .environment import describe as describe_environment, validate_profiles

ROOT=Path(__file__).resolve().parents[1]
LIMIT=2*1024*1024
HOP_HEADERS={'connection','keep-alive','proxy-authenticate','proxy-authorization','te','trailer','transfer-encoding','upgrade','proxy-connection'}
CAMERA_RESETS={f'reset_camera_{role}':role for role in ('head','left','right')}
COMPUTE_RESTARTS={f'restart_{role}':role for role in ('detector','planner','scene')}


def _camera_stop_verified(value):
    return value.get('stopped')is True and (value.get('exit_verified')is True or (
        value.get('exit_verified')is False and value.get('source_available')is False and value.get('process_preserved')is True))


def _operation_step_refs(action,phase):
    """References describe the stage being observed, never assert its success."""
    uc={'status_check':'UC02','prepare_environment':'UC01','prepare_model':'UC04','connect_control':'UC03',
        'recover_control':'UC08','recover_maintenance':'UC08','exit_environment':'UC09','reclaim_control':'UC10'}.get(action)
    if uc is None:return []
    if phase=='verifying_postconditions':
        return [dict(UC01='UC01-S05',UC02='UC02-S01',UC03='UC03-S04',UC04='UC04-S04',
            UC08='UC08-S03',UC09='UC09-S07',UC10='UC10-S04')[uc]]
    known={'checking_service_dependencies','waiting_model_endpoint','warming_model','verifying_model_identity',
        'camera_service','waiting_camera_source','rebinding_camera_clients','resetting_camera_usb',
        'restoring_warm_model_binding','verifying_old_maintenance','verifying_observation_before_control',
        'releasing_faulted_control','connecting_control','acknowledging_recoverable_fault','releasing_control',
        'stopping_current_control','waiting_control_stop','preparing_exit_home','returning_home',
        'waiting_exit_home','waiting_home_stop','stopping_environment','restoring_collection_web',
        'waiting_collection_web','checking_collection_handoff','stopping_collection','waiting_collection_exit',
        'model_start_failed','model_prepare_failed','camera_start_failed','backend_start_failed',
        'camera_reset_unknown','camera_reset_recovery_unknown',
        'stopping_collection_web','waiting_collection_web_exit','waiting_control_owner','recovering_workbench_control'}
    service_phase=any(phase==prefix+name for prefix in('start_','restart_','stop_')for name in
        ('workbench','model','cameras','control_owner','triton_control','recording','detector','planner','scene'))
    if phase not in known and not service_phase:return []
    if uc=='UC09':
        step=('S01'if phase in('stopping_current_control','waiting_control_stop')else
            'S02'if 'home'in phase else'S03'if phase=='releasing_control'else
            'S06'if 'collection_web'in phase else'S08'if phase=='stop_recording'else'S05')
    elif uc=='UC10':step='S01'if phase=='checking_collection_handoff'else'S02'if 'collection'in phase else'S04'if phase=='recovering_workbench_control'else'S03'
    elif uc=='UC01':step='S02'if phase=='start_recording'else'S04'if phase in('rebinding_camera_clients','restoring_warm_model_binding')else'S03'
    elif uc=='UC03':step='S03'if phase=='connecting_control'else'S01'
    elif uc=='UC08':step='S01'if phase=='checking_service_dependencies'else'S03'if phase=='verifying_observation_before_control'else'S02'
    elif uc=='UC04':step='S03'if phase=='verifying_model_identity'else'S02'
    else:step='S01'
    return [uc+'-'+step]


def stable_identity(value):
    return {key:value.get(key) for key in ('pid','start_ticks','script','boot_id','pgid','sid')} if value else None


def hold_verified(control):
    """Validate the complete fixed-hold receipt without reading live state or IO."""
    if not isinstance(control,dict):return False
    held=control.get('held_source_s');target=control.get('fixed_target')
    samples=control.get('samples');returns=control.get('sdk_returns');stop=control.get('stop_receipt')
    if not isinstance(returns,list) or not all(isinstance(row,dict) for row in returns):return False
    transport=control.get('command_transport','point_send')
    if transport not in ('point_send','native_window'):return False
    operation,count=('queue_reference','sdk_window_count') if transport=='native_window' else ('send','sdk_send_count')
    outputs=[row for row in returns if row.get('operation')==operation and 'error'not in row]
    if transport=='native_window' and (control.get('sdk_send_count')!=0 or any(
            row.get('target')!=target or row.get('zero_velocity')is not True or row.get('zero_added_torque')is not True
            or type(row.get('node_count'))is not int or row['node_count']<2
            or not isinstance(row.get('reference_id'),str) or not re.fullmatch('[0-9a-f]{64}',row['reference_id'])
            for row in outputs)):
        return False
    feedback=control.get('jaw_feedback_mm');command=control.get('jaw_command_mm');clipped=control.get('jaw_command_clipped')
    finite=lambda value:type(value) in (int,float) and math.isfinite(value)
    maxima=control.get('jaw_command_max_mm',[83.,83.])
    if not isinstance(maxima,list) or len(maxima)!=2 or not all(finite(v) and 0<v<=100 for v in maxima):return False
    if (not isinstance(feedback,list) or len(feedback)!=2 or not all(map(finite,feedback))
            or not isinstance(command,list) or len(command)!=2 or not all(finite(value) and 0<=value<=maxima[i] for i,value in enumerate(command))
            or not isinstance(clipped,dict) or set(clipped)!={'left','right'}
            or any(type(clipped[side]) is not bool or clipped[side]!=(abs(feedback[i]-command[i])>1e-6)
                for i,side in enumerate(('left','right')))):
        return False
    return bool(control.get('schema')=='workbench.fixed_hold_verification.v1'
        and control.get('status')=='completed' and control.get('verified') is True and control.get('duration_s')==5.
        and finite(held) and held>=5.
        and control.get('target_source')=='fresh_joint_feedback_and_bounded_calibrated_jaw_command'
        and control.get('joint_offsets_applied') is False and control.get('requested_target') is None
        and isinstance(target,list) and len(target)==14 and all(map(finite,target))
        and isinstance(samples,list) and len(samples)>0 and type(control.get('sample_count')) is int
        and control['sample_count']==len(samples)
        and type(control.get(count)) is int and control[count]>0 and control[count]==len(outputs)
        and isinstance(stop,dict) and stop.get('stop_hold_return_confirmed') is True)


def private_save(path,value):
    temporary=None
    try:
        with tempfile.NamedTemporaryFile(mode='w',dir=path.parent,prefix='.management-',delete=False) as stream:
            temporary=Path(stream.name);json.dump(value,stream,allow_nan=False);stream.flush();os.fsync(stream.fileno())
        os.replace(temporary,path)
        fd=os.open(path.parent,os.O_RDONLY)
        try:os.fsync(fd)
        finally:os.close(fd)
    finally:
        if temporary is not None:temporary.unlink(missing_ok=True)


def private_read(path,default):
    try:fd=os.open(path,os.O_RDONLY|os.O_NOFOLLOW)
    except FileNotFoundError:return default
    with os.fdopen(fd) as stream:
        if os.fstat(stream.fileno()).st_mode&0o077:raise ValueError('管理恢复文件必须仅当前用户可读写')
        raw=stream.read(LIMIT+1)
    if len(raw)>LIMIT:raise ValueError('管理恢复文件超过大小限制')
    return json.loads(raw)


def attempt_read(path,root):
    """Only the trusted launcher's bounded, regular receipt files may be read."""
    if path.is_symlink() or not path.resolve().is_relative_to(root):raise ValueError('启动收据路径越出已登记目录')
    fd=os.open(path,os.O_RDONLY|os.O_NOFOLLOW|os.O_NONBLOCK)
    with os.fdopen(fd) as stream:
        info=os.fstat(stream.fileno())
        if not stat.S_ISREG(info.st_mode) or info.st_size>LIMIT:raise ValueError('启动收据不是有界普通文件')
        raw=stream.read(LIMIT+1)
    if len(raw)>LIMIT:raise ValueError('启动收据超过大小限制')
    value=json.loads(raw)
    if not isinstance(value,dict):raise ValueError('启动收据必须是JSON对象')
    return value


def attempt_error(value):
    if not isinstance(value,str) or not value.strip():return '后端最终收据明确报告启动失败'
    if re.search(r'(?i)(?:^|[^a-z0-9])(?:[a-z0-9_]*token|secret|password|authorization|api[_-]?key)(?:[^a-z0-9]|$)|://[^/\s]*@',value):
        return '启动错误包含凭据相关内容，请查看受控后端日志'
    return value.strip().splitlines()[0][:2048]


class BackendRejection(ValueError):pass


def json_request(port,path,body=None,*,timeout=5,headers=None):
    connection=HTTPConnection('127.0.0.1',port,timeout=timeout)
    try:
        data=json.dumps(body,allow_nan=False).encode() if body is not None else None
        connection.request('POST' if body is not None else 'GET',path,data,
            {'Content-Type':'application/json','Connection':'close',**(headers or {})})
        response=connection.getresponse();raw=response.read(LIMIT+1)
        if len(raw)>LIMIT:raise ValueError('管理状态超过大小限制')
        value=json.loads(raw)
        if not isinstance(value,dict):raise ValueError('管理接口必须返回JSON对象')
        if response.status not in (200,202):raise BackendRejection(value.get('error') or 'HTTP '+str(response.status))
        return value
    finally:connection.close()


class WorkbenchManager:
    def __init__(self,config,*,services=None,monitor=True):
        if (not isinstance(config,dict) or config.get('schema')!='robot_workbench.manager.v1' or config.get('environment') not in ('fake','physical')
                or not isinstance(config.get('device_id'),str) or not config['device_id']
                or type(config.get('backend_port')) is not int or not 1024<=config['backend_port']<=65535
                or not isinstance(config.get('state_root'),str) or not Path(config['state_root']).is_absolute()
                or '..' in Path(config['state_root']).parts):
            raise ValueError('explicit trusted manager configuration required')
        self.config=deepcopy(config);self.environment=config['environment'];self.port=config['backend_port']
        if config.get('model_lifecycle','managed')not in ('managed','external'):
            raise ValueError('model_lifecycle must be managed or external')
        if config.get('control_lifecycle','managed')not in ('managed','external'):
            raise ValueError('control_lifecycle must be managed or external')
        if self.control_external and (not config.get('control_owner_descriptor') or not self.model_external
                or not isinstance(config.get('control_consumer_id'),str) or not config['control_consumer_id']):
            raise ValueError('external control requires a dedicated consumer identity/descriptor and external model lifecycle')
        handoff_url = config.get('consumer_handoff_url') or config.get('collection_handoff_url')
        if handoff_url is not None:
            if ((not self.control_external and not config.get('collection_handoff'))
                    or not isinstance(handoff_url, str) or len(handoff_url) > 2048):
                raise ValueError('consumer_handoff_url requires external control and a bounded URL')
            destination = urlsplit(handoff_url)
            if (destination.scheme not in ('http', 'https') or not destination.hostname
                    or destination.username is not None or destination.password is not None
                    or any(character.isspace() for character in handoff_url)):
                raise ValueError('consumer_handoff_url must be an explicit HTTP(S) workspace URL without credentials')
            destination.port  # Validate malformed ports before any service operation.
        handoff_web=config.get('consumer_handoff_web')
        if handoff_web is not None:
            if (not handoff_url or not isinstance(handoff_web,dict)
                    or set(handoff_web)!={'unit','files','status_url'}):
                raise ValueError('registered collection Web unit/files/status URL required')
            endpoint=urlsplit(handoff_web['status_url'])
            destination=urlsplit(handoff_url)
            if (endpoint.scheme!='http' or endpoint.hostname!=destination.hostname
                    or endpoint.port!=destination.port or endpoint.path!='/api/status'
                    or endpoint.username or endpoint.query or endpoint.fragment):
                raise ValueError('collection Web status must match the configured destination')
        self.triton_preview=None
        if config.get('triton_preview'):
            from .triton_preview import TritonPreviewClient
            self.triton_preview=TritonPreviewClient(config['triton_preview'])
        descriptor=config.get('control_owner_descriptor')
        if descriptor is not None and (self.environment!='physical'or not isinstance(descriptor,str)
                or not Path(descriptor).is_absolute()or '..'in Path(descriptor).parts):
            raise ValueError('trusted absolute physical control Owner descriptor required')
        if type(config.get('backend_requires_idle_gpu',False)) is not bool:raise ValueError('backend_requires_idle_gpu must be boolean')
        self.backend_auth=None;self.stop_generation=0;self.resource_cache={};self.resource_checked=0.
        self.profiles=validate_profiles(config.get('connection_profiles', [dict(id=config['device_id'],
            label=config.get('machine_profile',{}).get('label',config['device_id']),device_id=config['device_id'],
            environment=self.environment,manager_url='/',configured=True,
            robot_address=config.get('robot_address',''),compute_address=config.get('compute_address',''),
            topology='physical_local' if self.environment=='physical' else 'fullfake_dsw')]))
        self.timeouts=dict(state=2.,backend=90.,model=90.,camera=45.,release=15.,operation=300.)
        overrides=config.get('timeouts',{})
        if (not isinstance(overrides,dict) or set(overrides)-set(self.timeouts)
                or any(type(v) not in (int,float) or not math.isfinite(v) or v<=0 for v in overrides.values())):
            raise ValueError('bounded trusted manager timeouts required')
        self.timeouts.update(overrides)
        if 'model_port' in config and (type(config['model_port']) is not int or not 1024<=config['model_port']<=65535):
            raise ValueError('explicit loopback model_port required')
        if config.get('model_port')==self.port:raise ValueError('model_port must differ from backend_port')
        if 'camera_descriptor' in config and (not isinstance(config['camera_descriptor'],str)
                or not Path(config['camera_descriptor']).is_absolute() or '..' in Path(config['camera_descriptor']).parts):
            raise ValueError('absolute trusted camera_descriptor required')
        if 'native_rgb' in config and (config['device_id']!='arx6' or not isinstance(config['native_rgb'],dict)
                or set(config['native_rgb'])!={'directory','owner_path'}
                or any(not Path(p).is_absolute()or '..'in Path(p).parts for p in config['native_rgb'].values())):
            raise ValueError('explicit ARX6 raw RGB subscriber binding required')
        pointer=config.get('backend_attempt_pointer')
        if pointer is not None and (not isinstance(pointer,str) or not Path(pointer).is_absolute()
                or '..' in Path(pointer).parts or Path(pointer).name!='current.json' or Path(pointer).is_symlink()):
            raise ValueError('absolute trusted backend_attempt_pointer current.json required')
        self.backend_attempt_pointer=Path(pointer).absolute() if pointer is not None else None
        self.backend_attempt_root=self.backend_attempt_pointer.parent.resolve() if pointer is not None else None
        if services is None and (not isinstance(config.get('services'),dict) or 'workbench' not in config['services']):
            raise ValueError('trusted services registry with workbench required')
        self.instance=secrets.token_hex(16);self.csrf=secrets.token_urlsafe(32)
        def managed(name,value):
            if value.get('kind')=='docker_model':
                from .docker_model_service import DockerModelService,DockerModelSpec
                return DockerModelService(DockerModelSpec(name=name,**value))
            return ManagedService(ServiceSpec(name=name,**value))
        self.services=services if services is not None else {name:managed(name,value) for name,value in config['services'].items()}
        if 'workbench' not in self.services:raise ValueError('workbench service required')
        if descriptor and 'control_owner'not in self.services:raise ValueError('independent Owner must be a registered managed service')
        reset=config.get('camera_reset')
        if reset is not None and (not isinstance(reset,dict) or set(reset)!={'python','profile','root'}
                or any(not isinstance(p,str) or not Path(p).is_absolute() or '..' in Path(p).parts for p in reset.values())
                or not re.fullmatch(r'python(?:\d+(?:\.\d+)*)?',Path(reset['python']).name)):
            raise ValueError('trusted camera_reset requires absolute Python, profile and owner root')
        handoff=config.get('collection_handoff')
        if handoff is not None:
            url=urlsplit(handoff.get('status_url','')) if isinstance(handoff,dict) else None
            if (not isinstance(handoff,dict) or set(handoff)!={'unit','files','status_url'}
                    or not (re.fullmatch(r'prometheus-arx[0-9]+-web\.service',str(handoff.get('unit')))
                        or config.get('device_id')=='arx6'and handoff.get('unit')=='prometheus-web.service')
                    or not isinstance(handoff['files'],dict) or not handoff['files']
                    or any(not Path(p).is_absolute()or not re.fullmatch('[a-f0-9]{64}',str(h))for p,h in handoff['files'].items())
                    or url.scheme!='http' or url.hostname!='127.0.0.1' or not url.port or url.path!='/api/status'
                    or url.query or url.fragment or url.username):
                raise ValueError('pinned collection user unit and loopback status API required')
        self.machine_profile=MachineProfile(config);self.deployment=Deployment(ROOT,self.services)
        self.deployment.bind_configuration(config)
        self.root=Path(config['state_root']);self.root.mkdir(parents=True,exist_ok=True)
        self.health_lock=threading.Lock();self.health_cache={};self.closed=threading.Event()
        self.window_path=self.root/'maintenance-private.json';self.recovery_path=self.root/'recovery.json'
        self.window=private_read(self.window_path,None)
        self.recovered=set(private_read(self.recovery_path,{}).get('resolved_job_ids',[]))
        from .control_alerts import ControlAlerts
        alerts_path=self.root/'control-alerts.json'
        self.control_alerts=ControlAlerts(private_read(alerts_path,{}),lambda value:private_save(alerts_path,value))
        if self.window is not None and (self.window.get('environment')!=self.environment or self.window.get('device_id')!=config['device_id']):
            raise ValueError('维护恢复记录与当前部署不符')
        definitions=[
            ('status_check','检查环境状态','只读核对进程、工作台、相机和模型状态。',False),
            ('start_cameras','启动相机服务','启动或复用本设备相机；后端已退出时先恢复未接机的网页接口。不连接机器人，不执行动作。',False),
            ('stop_cameras','停止相机服务','停止本设备采样服务并保留控制保持和模型；不会等待新帧或自动重启。',False),
            ('restart_cameras','重启相机服务','暂停相机观测并重绑新实例；保留机器人保持和模型权重。后端已退出时先恢复未接机的网页接口。',False),
            ('rebind_cameras','重新绑定相机','重新核验三路新鲜帧和采集身份；旧推理确认作废，不恢复旧录制。',False),
            ('prepare_model','准备并预热模型','加载当前选择的模型并预热；不连接机器人，不执行动作。',False),
            ('stop_model','停止模型服务','核验本任务worker归属后停止模型并撤销业务绑定；相机和SDK保持不变。',False),
            ('restart_model','重启模型服务','停止本任务模型进程再启动，重新预热当前模型；机器人保持不变。',False),
            ('prepare_environment','准备当前环境','准备观测基础及明确选定的能力；复用已有热服务，不自动连接或执行动作。',False),
            ('restart_workbench','重载工作台后端','结束本设备控制连接并重载业务后端；物理设备会撤掉SDK保持，需可靠支撑。',self.environment=='physical'),
            ('stop_workbench','停止工作台后端','确认控制已释放后停止业务后端；独立相机、模型和管理服务保留，不自动启动。',self.environment=='physical'),
            ('recover_maintenance','核验并结束遗留维护','重核服务身份与业务空闲后结束遗留维护；不重试旧动作、不恢复运动。',False),
            ('connect_control','接入设备控制','检查基础环境并接入本设备；物理设备需现场支撑。不会执行旧任务。',self.environment=='physical'),
            ('reclaim_control','清理现有控制并接入','正常停止当前工作台任务和已登记的数采，再接入工作台。数采停机可能回零；健康连接停止旧动作后复用，必要时释放重接会撤掉保持。保留数据；退出不明时停止，不强杀、不续跑旧动作。',self.recovery_requires_support),
            ('exit_environment','一键退出工作台环境','停止当前任务，经正常规划以0.1rad/s平滑回零、保留夹爪开度，然后释放SDK保持，停止工作台模型、相机、业务和控制进程，恢复原数采网页。回零会移动双臂；失败不继续释放，不强杀、不自动开始采集。管理页保留。',False),
            ('release_control','释放设备控制','撤掉SDK保持并核验连接收尾；已有数据保留。',self.environment=='physical'),
            ('recover_control','中断恢复','可确认的运动/软件故障原地恢复，不重接、不续跑旧动作；需要释放重接时才要求可靠支撑。',self.recovery_requires_support),
            ('verify_hold','5秒原地保持验证','按当前实测关节姿态保持5秒；夹爪沿既有0–83mm命令限幅，反馈超出时可能略收拢。不回零、不抓取，结束后停止并留证。',self.environment=='physical'),
            ('prepare_collection','准备数采计算环境','核验当前资产并按需准备已声明的常驻服务；需要已接入的健康静止反馈。不采新观测、不构建任务场景、不做业务预热。',False),
        ]
        definitions += [(action,'软件重置'+{'head':'头部','left':'左腕','right':'右腕'}[role]+'相机',
            '只对该物理相机执行一次USB软件复位；三路画面会暂时暂停并重绑。禁止动作或录制期间执行；机器人SDK保持不变。',False)
            for action,role in CAMERA_RESETS.items()]
        definitions += [(action,'重启'+{'detector':'检测','planner':'规划','scene':'场景'}[role]+'服务',
            '只重启所选计算进程，保留安装资产、其他健康服务与机器人保持；旧计划失效，数采需重新预热。',False)
            for action,role in COMPUTE_RESTARTS.items()]
        if descriptor:
            definitions=[(name,label,'仅重载业务HTTP与工作流；独立Owner、SDK保持、相机与模型进程保留。'if name in('restart_workbench','stop_workbench')else impact,
                False if name in('restart_workbench','stop_workbench')else support)for name,label,impact,support in definitions]
        if self.model_external:
            definitions=[(name,'释放本工作台模型绑定'if name=='stop_model'else label,
                '只释放本工作台的模型会话；共享服务与其他消费者保持运行。'if name=='stop_model'else
                '共享模型由服务管理员管理，本工作台不能重启它。'if name=='restart_model'else
                impact.replace('停止工作台模型、相机、业务和控制进程','释放本工作台模型会话，停止相机、业务和控制进程；共享模型服务保留')
                if name=='exit_environment'else impact,support)for name,label,impact,support in definitions]
        if self.control_external:
            definitions=[(name,'退出本工作台'if name=='exit_environment'else'接入本工作台控制'if name=='connect_control'else label,
                '结束本工作台业务并释放自己的模型绑定和控制claim；共享Owner、SDK、相机和其他消费者保留，不启动数采。'
                if name=='exit_environment'else
                '重载本工作台消费者，通过专属身份申请已连接Owner；其他消费者持有时拒绝，不接入SDK。'
                if name=='connect_control'else impact,
                False if name in ('connect_control','exit_environment')else support)for name,label,impact,support in definitions]
        if handoff_url:
            definitions = [
                (name, '退出工作台并启动正式数采'if config.get('collection_handoff_url') else'退出工作台并启动数采平台'if handoff_web else'切换到数采工作台',
                 '结束本工作台并归还控制，核验数采平台就绪后打开 '+handoff_url, False)
                if name == 'exit_environment' else (name, label, impact, support)
                for name, label, impact, support in definitions
            ]
        if self.config.get('backend_requires_idle_gpu'):
            definitions=[(name,label,impact+(' 此部署启动plant要求空闲GPU；若模型服务运行，先显式停止模型。'
                if name in ('restart_workbench','prepare_environment','connect_control','recover_control') else ''),support)
                for name,label,impact,support in definitions]
        use_cases=dict(status_check='UC02',prepare_environment='UC01',prepare_model='UC04',
            connect_control='UC03',recover_control='UC08',recover_maintenance='UC08',
            rebind_cameras='UC12',start_cameras='UC12',restart_cameras='UC12')
        use_cases.update({action:'UC12' for action in CAMERA_RESETS})
        if handoff_url:use_cases.update(exit_environment='UC09',reclaim_control='UC10')
        self.operations=OperationService([Operation(name,label,impact,
            check=lambda request,n=name:self.check(n,request),
            run=lambda request,progress,cancel,n=name:self.run(n,request,progress),
            verify=lambda request,result:self.verify(result),requires_support=support,timeout_s=self.timeouts['operation'],
            parameter_keys=('model_id','expected_bundle_digest') if name=='prepare_model' else ('capabilities',) if name=='prepare_environment' else ('asset_pair_id','expected_catalog_digest') if name=='prepare_collection' else (),
            use_case_id=use_cases.get(name))
            for name,label,impact,support in definitions],target=self.target,receipt_dir=self.root/'operations')
        self.refresh()
        self.thread=None
        if monitor:
            self.thread=threading.Thread(target=self.monitor,daemon=True,name='management-health');self.thread.start()

    def state(self):
        value=json_request(self.port,'/api/environment?environment='+self.environment,timeout=self.timeouts['state'])
        if (value.get('schema')!='robot_workbench.environment.v1' or value.get('device',{}).get('id')!=self.config['device_id']
                or value['device'].get('environment')!=self.environment or not value.get('server_instance_id')
                or not isinstance(value.get('csrf'),str) or not value['csrf']
                or type(value.get('runtime',{}).get('connected')) is not bool or not isinstance(value.get('activity'),dict)):
            raise ValueError('工作台设备身份与管理部署不符')
        self.backend_auth=dict(csrf=value['csrf'],server_instance_id=value['server_instance_id'],device_id=self.config['device_id'])
        return value

    def target(self):
        with self.health_lock:health=deepcopy(self.health_cache)
        return dict(environment=self.environment,device_id=self.config['device_id'],manager_instance=self.instance,
            deployment_sha256=self.deployment.sha256,
            backend_instance=health.get('backend_instance'),
            runtime_nonce=health.get('runtime',{}).get('runtime_nonce'),
            command_epoch=health.get('runtime',{}).get('command_epoch'),
            services={name:stable_identity(row.get('identity')) for name,row in health.get('services',{}).items()})

    def refresh(self):
        value=dict(checked_wall_ns=time.time_ns(),services={name:service.snapshot() for name,service in self.services.items()},
            deployment=self.deployment.snapshot(check_files=True),machine_profile=self.machine_profile.snapshot(),resources=self.resources())
        camera_error=self.camera_source_error(value['services'].get('cameras',{}))
        if camera_error:value['services']['cameras']['error']=camera_error
        try:
            state=self.state();runtime=state['runtime'];model=deepcopy(state.get('model') or {})
            model.setdefault('id',model.get('selected_model_id'));model.setdefault('model_ready',model.get('ready'))
            value.update(backend_reachable=True,backend_instance=state['server_instance_id'],runtime=runtime,
                busy=state['activity'].get('busy') is True,activity=state['activity'],maintenance=state.get('maintenance'),
                model=model,model_generation=model.get('generation'),inference_control_ready=model.get('control_ready'),
                collection=state.get('collection'),
                control_timing=state.get('control_timing'),
                cameras=state.get('cameras',[]))
            self.control_alerts.observe(runtime.get('control_protection'))
        except Exception as error:
            value.update(backend_reachable=False,error=str(error))
            attempt=self.backend_attempt(value['services']['workbench'])
            if attempt is not None:
                value['backend_attempt']=attempt
                if attempt['status']=='failed':value['error']='最近一次后端异常：'+attempt['error']
        if self.config.get('control_owner_descriptor'):
            try:
                from .control_owner import ControlChannel
                runtime=ControlChannel(self.config['control_owner_descriptor'],timeout=1.).call('inspect')
                value.update(runtime=runtime,control_owner_reachable=True)
                self.control_alerts.observe(runtime.get('control_protection'))
            except (OSError,ValueError,ConnectionError)as error:
                # Never fall back to an old business projection as current authority.
                value.update(runtime=dict(connected=None,feedback_healthy=False,state='UNKNOWN',reason=str(error)),
                    control_owner_reachable=False)
        model=value['services'].get('model',{})
        if model:model['lifecycle']='external'if self.model_external else 'managed'
        control_owner=value['services'].get('control_owner',{})
        if control_owner:control_owner['lifecycle']='external'if self.control_external else 'managed'
        if 'recording' in self.services:
            try:
                from .recording_process import recording_call
                recorded=recording_call(os.environ['WORKBENCH_RECORDING_SERVICE'],'inspect',timeout=self.timeouts['state'])
                if recorded['pid']!=(value['services']['recording'].get('identity')or{}).get('pid'):
                    raise ValueError('recording service identity differs')
                value['recording_service']=recorded
            except (KeyError,OSError,ValueError) as error:
                value['recording_service']=dict(ready=False,status='unknown',error=str(error))
        value['model_service']=dict(status='unknown',model_id=None,loaded=None,warmup=None)
        if model.get('state')=='stopped' and model.get('running') is False:
            value['model_service'].update(status='stopped',loaded=False)
        elif model.get('state')=='running' and model.get('owned') is True and self.config.get('model_port'):
            try:
                observed=json_request(self.config['model_port'],'/status',timeout=self.timeouts['state'])
                if (observed.get('service_pid')!=(model.get('identity') or {}).get('pid')
                        or type(observed.get('loaded')) is not bool
                        or observed.get('status') not in ('not_loaded','loading','ready','error','closed')):
                    raise ValueError('model service identity or status differs')
                warmup=observed.get('warmup')
                value['model_service']=dict(status=observed['status'],model_id=observed.get('model_id'),loaded=observed['loaded'],
                    service_pid=observed.get('service_pid'),worker_pid=observed.get('worker_pid'),
                    backend=observed.get('backend'),bundle_digest=observed.get('bundle_digest'),
                    fact=deepcopy(observed.get('fact')),
                    observed_wall_ns=time.time_ns(),error=observed.get('error'),
                    warmup={key:deepcopy(warmup[key]) for key in ('completed','wall_s','seed','input','output_sha256','timing') if key in warmup}
                        if isinstance(warmup,dict) else None)
            except (OSError,HTTPException,ValueError) as error:
                value['model_service'].update(error=str(error),observed_wall_ns=time.time_ns())
        with self.health_lock:self.health_cache=value
        return value

    def resources(self):
        if time.monotonic()-self.resource_checked<10:return deepcopy(self.resource_cache)
        value=dict(checked_wall_ns=time.time_ns(),scope='manager_host_readonly',cpu_count=os.cpu_count(),
            reservation=dict(status='declared' if self.config.get('resource_record') else 'unknown',
                record=self.config.get('resource_record'),note='实时使用数据不等于GPU独占授权；准备操作仍执行既有资源准入'))
        try:
            disk=shutil.disk_usage(self.root);value['storage']=dict(total_bytes=disk.total,free_bytes=disk.free)
        except OSError as error:value['storage']=dict(status='unknown',reason=str(error))
        tool=shutil.which('nvidia-smi')
        if tool:
            try:
                output=subprocess.check_output([tool,'--query-gpu=index,uuid,memory.used,memory.total,utilization.gpu',
                    '--format=csv,noheader,nounits'],text=True,timeout=2,stderr=subprocess.DEVNULL)
                cards=[]
                for line in output.splitlines():
                    index,uuid,used,total,utilization=[part.strip() for part in line.split(',')]
                    cards.append(dict(index=int(index),uuid=uuid,memory_used_mib=int(used),memory_total_mib=int(total),utilization_percent=int(utilization)))
                value['gpu']=dict(status='observed',cards=cards)
            except (OSError,ValueError,subprocess.SubprocessError) as error:value['gpu']=dict(status='unknown',reason=str(error))
        else:value['gpu']=dict(status='unsupported',reason='本管理主机无nvidia-smi；远端计算状态按对应环境核验')
        self.resource_cache=value;self.resource_checked=time.monotonic()
        return deepcopy(value)

    def monitor(self):
        while not self.closed.wait(1):self.refresh()

    @property
    def model_external(self):
        return self.config.get('model_lifecycle','managed')=='external'

    @property
    def control_external(self):
        return self.config.get('control_lifecycle','managed')=='external'

    def managed_service_names(self):
        if self.control_external:return {'workbench'}
        return set(self.services)-({'model'}if self.model_external else set())

    def service_dependencies(self,action,health,parameters=None):
        """Finite operation scopes, not a second readiness authority."""
        required={'workbench'};parameters=parameters or {}
        if self.control_external:return required
        if action=='exit_environment':return self.managed_service_names()
        if 'recording' in self.services and action in ('prepare_environment','connect_control','recover_control','restart_workbench'):
            required.add('recording')
        wanted=set(parameters.get('capabilities',['observe'])) if action=='prepare_environment' else set()
        if action in ('prepare_model','restart_model') or action=='stop_model'and not self.model_external or 'inference' in wanted:required.add('model')
        if self.environment=='physical' and (action in ('start_cameras','restart_cameras','stop_cameras','rebind_cameras',
                'connect_control','restart_workbench',*CAMERA_RESETS) or 'observe' in wanted
                or action in ('prepare_environment','recover_control') and (not health.get('backend_reachable')
                    or action=='recover_control' and not health.get('runtime',{}).get('recoverable_fault')
                        and not self.control_healthy(health.get('runtime')))):
            required.add('cameras')
        return required

    def maintenance_services(self,health):
        required={'workbench'}
        if self.window:
            scope=self.window.get('affected_services')
            if 'affected_services' in self.window:
                if (not isinstance(scope,list) or not scope or any(not isinstance(name,str) for name in scope) or len(set(scope))!=len(scope)
                        or any(name not in self.services for name in scope)):
                    raise ValueError('私密维护作用域损坏，保持冻结')
                required.update(scope)
            else:
                purpose=(self.window.get('begin_body') or {}).get('purpose')
                if purpose not in self.operations.operations:required.update(self.services)
                else:
                    # Legacy prepare_environment tickets did not save capabilities.
                    parameters={'capabilities':['observe','inference','collection']} if purpose=='prepare_environment' else {}
                    required.update(self.service_dependencies(purpose,health,parameters)&self.services.keys())
        unresolved=set(self.unresolved())
        for job in self.operations.jobs.values():
            if job['job_id'] in unresolved:
                request=job['request'];required.update(self.service_dependencies(request['action'],health,request.get('parameters')))
        return required

    def dependency_evidence(self,action,health,parameters=None):
        required=self.service_dependencies(action,health,parameters)
        if action in ('recover_maintenance','recover_control') and (self.window or self.unresolved()):
            required.update(self.maintenance_services(health))
        return [dict(service=name,requirement='known_identity' if name=='workbench' or action=='recover_maintenance'
                else 'registered_adapter_then_strict_identity_verification',
            observed=deepcopy(health.get('services',{}).get(name,{})),checked_wall_ns=health.get('checked_wall_ns'),
            outcome='requires_adapter_verification' if health.get('services',{}).get(name,{}).get('state')=='unknown' else 'observed')
            for name in sorted(required)]

    def operation_domains(self,action,parameters=None):
        """Write sets for unresolved history; a live maintenance window is exclusive."""
        if action=='status_check':return set()
        if action in ('prepare_model','restart_model','stop_model'):return {'model'}
        if action in ('start_cameras','stop_cameras','restart_cameras','rebind_cameras',*CAMERA_RESETS):return {'cameras'}
        if action in ('prepare_collection',*COMPUTE_RESTARTS):return {'collection'}
        if action=='prepare_environment':
            wanted=(parameters or {}).get('capabilities',['observe'])
            return {{'observe':'cameras','inference':'model','collection':'collection'}[name] for name in wanted}
        return {'control','cameras','model','collection'}

    def blocking_history(self,action,health,parameters=None):
        domains=self.operation_domains(action,parameters)
        unresolved=set(self.unresolved())
        return [job['job_id'] for job in self.operations.jobs.values() if job['job_id'] in unresolved
            and domains&self.operation_domains(job['request']['action'],job['request'].get('parameters'))]

    def check(self,action,request=None):
        with self.health_lock:h=deepcopy(self.health_cache)
        parameters=(request or {}).get('parameters') or {}
        allowed={'model_id','expected_bundle_digest'} if action=='prepare_model' else {'capabilities'} if action=='prepare_environment' else {'asset_pair_id','expected_catalog_digest'} if action=='prepare_collection' else set()
        if not isinstance(parameters,dict) or set(parameters)-allowed:return '此操作不接受这些参数'
        if 'model_id' in parameters and (not isinstance(parameters['model_id'],str) or not parameters['model_id'] or len(parameters['model_id'])>256):return '模型ID无效'
        if 'expected_bundle_digest' in parameters and (not isinstance(parameters['expected_bundle_digest'],str) or not re.fullmatch('[a-f0-9]{64}',parameters['expected_bundle_digest'])):return '模型bundle身份无效'
        if {'asset_pair_id','expected_catalog_digest'}&set(parameters):
            if (not re.fullmatch('[A-Za-z0-9_.-]{1,128}',str(parameters.get('asset_pair_id','')))
                    or not re.fullmatch('[a-f0-9]{64}',str(parameters.get('expected_catalog_digest','')))):
                return '场景资产组合和目录版本无效'
        if 'capabilities' in parameters and (not isinstance(parameters['capabilities'],list) or not parameters['capabilities']
                or any(name not in ('observe','inference','collection') for name in parameters['capabilities']) or len(set(parameters['capabilities']))!=len(parameters['capabilities'])):return '准备能力只能选择观测、推理或数采'
        if action=='status_check':return None
        if action=='restart_model'and self.model_external:return '共享模型服务由独立管理员管理；本工作台只能释放自己的模型绑定'
        if action in CAMERA_RESETS:
            if self.environment!='physical':return 'Fake 没有物理 USB 相机；此软件重置操作不支持仿真'
            try:self.camera_reset_command(CAMERA_RESETS[action])
            except ValueError as error:return str(error)
        if h.get('deployment',{}).get('files_unchanged') is not True:return '部署源文件已变化，请重新启动管理器并刷新页面；诊断仍可查看'
        if time.time_ns()-h.get('checked_wall_ns',0)>10_000_000_000:
            raise OperationError('environment_state_unknown','环境状态已过期，请先检查状态',unknown=True)
        if h.get('services',{}).get('workbench',{}).get('state')=='unknown':
            raise OperationError('process_identity_unknown','工作台进程身份未知，请先检查诊断；不会自动停止未知控制进程',unknown=True)
        if self.control_external:
            allowed={'status_check','connect_control','restart_workbench','stop_workbench','exit_environment','prepare_model','stop_model'}
            if action not in allowed:return '共享设备生命周期由管理员负责；本工作台只能接入或退出自己的消费者'
            owner=(h.get('runtime')or{}).get('control_owner')or{}
            if not h.get('control_owner_reachable'):
                raise OperationError('owner_state_unknown','共享Owner状态尚未核验',unknown=True)
            if owner.get('shared_consumers')is not True:return '共享Owner或消费者能力尚未核验'
            holder=owner.get('consumer_holder');own=holder==self.config['control_consumer_id']
            if action in ('connect_control','restart_workbench')and holder is not None and not own:
                return '其他消费者持有执行权，未接管或重载工作台'
            if own and ((h.get('runtime')or{}).get('automatic_enabled')or h.get('busy')):
                return '本消费者仍在执行或收尾，请先结束自己的任务'
            if self.window is not None or self.unresolved():return '本工作台仍有未确认维护，请先核验原回执'
            return None
        if action=='reclaim_control':
            if self.environment!='physical' or not self.config.get('control_owner_descriptor'):
                return '一键接入需要已登记的物理设备与独立控制Owner'
            if not self.config.get('collection_handoff'):return '本机尚未登记数采交接接口'
            if h.get('services',{}).get('control_owner',{}).get('state')not in ('running','stopped'):
                return '控制Owner进程身份未知，未清理或接入'
            if (h.get('maintenance')or{}).get('active_action'):return '旧维护动作仍未返回，先核验原回执'
            return None
        if action=='exit_environment':
            if self.environment!='physical' or not self.config.get('control_owner_descriptor')or not self.config.get('collection_handoff'):
                return '退出交接需要已登记的真机Owner与原数采入口'
            if self.window is not None or h.get('maintenance')or self.unresolved():return '存在未核验的维护，请先核验并结束遗留维护'
            if any(h.get('services',{}).get(name,{}).get('state')not in ('running','stopped')for name in self.managed_service_names()):return '服务身份未知，不能确认退出范围'
            runtime=h.get('runtime')or{}
            if runtime.get('connected'):
                if not h.get('backend_reachable')or runtime.get('feedback_healthy')is not True or 'FAULT'in str(runtime.get('state')):
                    return '当前控制或反馈异常，无法走正常回零释放；先按中断恢复流程核验'
            elif h.get('services',{}).get('control_owner',{}).get('state')=='running'and not h.get('control_owner_reachable'):
                return '控制Owner状态未知，未确认SDK已退出'
            return None
        pending=self.window is not None or bool(h.get('maintenance')) or bool(
            self.unresolved() if action in ('recover_maintenance','recover_control') else self.blocking_history(action,h,parameters))
        cold_backend=(h.get('services',{}).get('workbench',{}).get('state')=='stopped'
            and h.get('services',{}).get('workbench',{}).get('running') is False)
        if (self.config.get('backend_requires_idle_gpu') and h.get('services',{}).get('model',{}).get('running') is True
                and (action=='restart_workbench' or cold_backend and action in ('prepare_environment','connect_control','recover_control','start_cameras','restart_cameras'))):
            return '本部署启动plant需要空闲GPU；请先在运维停止模型，再启动或恢复工作台，不会自动停止模型或放宽GPU检查'
        if self.environment=='physical' and action=='recover_control' and 'cameras' in self.service_dependencies(action,h):
            camera_error=self.camera_registration_error()
            if camera_error:return camera_error
        if action in ('recover_maintenance','recover_control'):
            if action=='recover_control' and not pending:
                if h.get('busy') or h.get('runtime',{}).get('automatic_enabled'):return '先停止当前任务并等待收尾，再恢复控制'
                return None
            if not pending:return '没有需要恢复的遗留维护或未知操作'
            if (h.get('maintenance') or {}).get('active_action'):return '旧维护动作仍未结束，不能解除冻结'
            if h.get('busy') or h.get('runtime',{}).get('automatic_enabled'):return '业务仍在执行，不能解除冻结'
            if not h.get('backend_reachable') and h.get('services',{}).get('workbench',{}).get('running') is not False:return '后端不可达且进程未确认退出，不能解除冻结'
            if h.get('maintenance') and self.window is None:return '没有本管理器的私密维护凭证，不能接管其他维护窗口'
            try:scope=self.maintenance_services(h)
            except ValueError as error:return str(error)
            if any(h.get('services',{}).get(name,{}).get('state') not in ('running','stopped') for name in scope):
                return '遗留维护作用域内的服务身份未知，保持冻结；无关服务不会自动处理'
            return None
        if pending:return '存在遗留维护或未知操作，请先“核验并结束遗留维护”'
        if self.config.get('control_owner_descriptor')and action in('restart_workbench','stop_workbench'):
            if not h.get('control_owner_reachable'):return '独立Owner状态未知，请先核验；不会停止未知控制进程'
            if h.get('busy')or h.get('runtime',{}).get('automatic_enabled'):return '当前业务仍在执行，请先停止任务并等待收尾'
            return None
        if action in ('restart_model','prepare_model','stop_model') and ('model' not in self.services or 'model_port' not in self.config):return '当前部署未配置可核验的模型服务或model_port'
        if self.environment=='physical' and action=='stop_cameras' and 'cameras' not in self.services:return '相机服务尚未纳入此设备管理'
        if action=='prepare_environment' and 'model' in self.services and 'model_port' not in self.config:return '当前部署缺少模型暖机核验端口model_port'
        if self.environment=='physical' and action in ('start_cameras','restart_cameras','prepare_environment','connect_control',*CAMERA_RESETS):
            camera_error=self.camera_registration_error()
            if camera_error:return camera_error
        if self.environment=='physical' and action=='rebind_cameras' and h.get('services',{}).get('cameras',{}).get('state') not in ('running','stopped'):
            return '相机源身份未知，不能重绑；请先检查或通过受控相机操作核验恢复'
        if not h.get('backend_reachable'):
            if action=='stop_workbench' and h.get('services',{}).get('workbench',{}).get('running') is False:return None
            if cold_backend and (action=='stop_model' or action=='stop_cameras' and 'cameras' in self.services):return None
            if cold_backend and action in ('start_cameras','restart_cameras'):return None
            if action not in ('prepare_environment','restart_workbench','connect_control'):return '工作台后端不可达，请先恢复工作台'
            if h.get('services',{}).get('workbench',{}).get('running') is not False:return '后端进程仍存活但HTTP不可达，不能确认SDK已释放'
            return None
        if h.get('busy') or h.get('runtime',{}).get('automatic_enabled'):return '当前业务仍在执行，请先停止任务并等待收尾'
        runtime=h.get('runtime',{})
        if action=='connect_control' and runtime.get('connected') is True:return '设备控制已经接入'
        if action in COMPUTE_RESTARTS and not h.get('collection',{}).get('installation_identity'):
            return '当前固定配置无已验证parts/installation_identity，不支持单角色重启'
        if action in ('release_control','verify_hold','prepare_collection',*COMPUTE_RESTARTS) and runtime.get('connected') is not True:return '请先接入设备控制'
        if action in ('verify_hold','prepare_collection',*COMPUTE_RESTARTS) and (runtime.get('feedback_healthy') is not True or runtime.get('stationary') is not True):return '此操作需要已接入、新鲜、健康且静止的实际反馈'
        if action in COMPUTE_RESTARTS and COMPUTE_RESTARTS[action] not in (h.get('collection',{}).get('services') or {}):
            return '当前已验证配置未启用常驻'+{'detector':'检测','planner':'规划','scene':'场景'}[COMPUTE_RESTARTS[action]]+'服务；不能单独重启未配置的角色'
        return None

    def unresolved(self):
        with self.operations.lock:
            return [job['job_id'] for job in self.operations.jobs.values() if job['status'] in ('interrupted_unknown','timeout_unknown','unknown')
                    and job['request']['action']!='status_check' and job['job_id'] not in self.recovered]

    @staticmethod
    def backend_target(state):
        return dict(server_instance_id=state['server_instance_id'],device_id=state['device']['id'],
                    runtime_nonce=state.get('runtime',{}).get('runtime_nonce'))

    def post(self,action,*,token=None,timeout=60,**fields):
        state=self.state()
        if self.window is not None and state['server_instance_id']!=self.window['backend_instance']:
            raise OperationError('backend_changed','后端实例已变化，旧维护动作没有发送给新实例')
        action={'ops_maintenance_end':'end','release':'release_control'}.get(action,action.removeprefix('ops_'))
        expected=self.window.get('expected_target') if self.window is not None else self.backend_target(state)
        body=dict(environment=self.environment,expected_target=expected,request_id=secrets.token_hex(16),**fields)
        if token is not None:body['maintenance_token']=token
        tracked=action not in ('inspect','receipt') and self.window is not None
        if tracked:
            if self.window.get('pending_call'):raise OperationError('maintenance_result_unknown','原维护调用尚未取得结果，禁止重复发送',unknown=True)
            self.save_window(dict(self.window,pending_call=dict(action=action,body=body,started_wall_ns=time.time_ns())))
        try:
            result=json_request(self.port,'/api/maintenance/'+action,body,timeout=timeout,
                headers={'Origin':'http://127.0.0.1:'+str(self.port),'X-CSRF-Token':state['csrf']})
        except BackendRejection:
            if tracked:self.save_window(dict(self.window,pending_call=None))
            raise
        except (OSError,HTTPException,ValueError):
            if not tracked:raise
            result=self.read_pending_call()
        self.validate_control_receipt(action,result,expected)
        if tracked:self.save_window(dict(self.window,pending_call=None))
        return result

    def recovery_requires_support(self, request=None):
        runtime=self.health_cache.get('runtime') or {}
        return self.environment=='physical' and not (self.control_healthy(runtime)
            or runtime.get('connected')is True and bool(runtime.get('recoverable_fault')))

    def read_pending_call(self):
        """Query the original command result; never resend a timed-out command."""
        pending=self.window.get('pending_call') if self.window else None
        if not pending:return None
        state=self.state();body=pending['body']
        if state['server_instance_id']!=self.window['backend_instance']:
            raise OperationError('backend_changed','后端实例变化，旧命令结果保持未知',unknown=True)
        result=json_request(self.port,'/api/maintenance/receipt',dict(environment=self.environment,
            expected_target=body['expected_target'],request_id=secrets.token_hex(16),
            lookup_request_id=body['request_id'],maintenance_token=body['maintenance_token']),
            timeout=self.timeouts['state'],headers={'Origin':'http://127.0.0.1:'+str(self.port),'X-CSRF-Token':state['csrf']})
        expected_action={'release':'release_control'}.get(pending['action'],'ops_'+pending['action'] if pending['action'] in
            ('prepare_model','prepare_collection','rebind_cameras','invalidate_model','start_cameras','stop_cameras','restart_cameras','end') else pending['action'])
        if pending['action']=='end':expected_action='ops_maintenance_end'
        if (result.get('request_id')!=body['request_id'] or result.get('action')!=expected_action
                or (result.get('target')or{}).get('server_instance_id')!=self.window['backend_instance']
                or result.get('status') not in ('completed','failed')):
            raise OperationError('maintenance_result_unknown','原调用结果尚未确认；未重新执行命令',unknown=True,
                details=dict(request_id=body['request_id'],action=pending['action'],observed_status=result.get('status')))
        if result['status']=='failed':
            self.save_window(dict(self.window,pending_call=None,last_call=dict(request_id=body['request_id'],action=pending['action'],status='failed')))
            raise ValueError(result.get('error')or'原维护调用失败')
        return result['response']

    def validate_control_receipt(self,action,result,expected):
        if action in ('connect_control','release_control','recover_control'):
            if not isinstance(result.get('target'),dict):raise OperationError('control_receipt_unknown','控制操作缺少Runtime身份回执，维护保持冻结',unknown=True)
            target=result['target']
            if target.get('server_instance_id')!=expected['server_instance_id'] or target.get('device_id')!=expected['device_id']:
                raise OperationError('target_changed','控制回执身份变化，维护冻结保持',unknown=True)
            if action=='release_control' and (result.get('connected') is not False
                    or target.get('runtime_nonce') is not None or not self.release_verified(result.get('release_receipt'))):
                raise OperationError('release_unconfirmed','SDK释放回执尚未确认，未停止后端',unknown=True)
            if action=='connect_control' and (result.get('connected') is not True
                    or not self.control_healthy(result.get('runtime'))
                    or target.get('runtime_nonce')!=(result.get('runtime') or {}).get('runtime_nonce')):
                raise OperationError('connection_unconfirmed','接入未返回匹配的新鲜健康静止Runtime',unknown=True)
            if action=='recover_control':
                checks=dict(target_matches=target==expected,recovered=result.get('recovered')is True,
                    motion_not_resumed=result.get('motion_resumed')is False,
                    current_control_healthy=self.control_healthy(result.get('runtime')))
                if not all(checks.values()):
                    runtime=result.get('runtime')or{}
                    raise OperationError('recovery_unconfirmed','恢复未返回同一Runtime的新鲜无运动回执',unknown=True,
                        details=dict(checks=checks,expected_target=expected,observed_target=target,
                            runtime={key:deepcopy(runtime.get(key))for key in ('state','runtime_nonce','command_epoch',
                                'sample_age_s','connected','feedback_healthy','stationary','automatic_enabled','control_coherence')}))
            self.save_window(dict(self.window,expected_target=target))

    @staticmethod
    def release_verified(receipt):
        return isinstance(receipt,dict) and receipt.get('status')=='released' and receipt.get('sdk_close_returned') is True

    @staticmethod
    def control_healthy(runtime):
        from .state_contract import control_healthy
        return control_healthy(runtime)

    def save_window(self,value):
        private_save(self.window_path,value);self.window=value

    def finish_window(self,*,inspect_runtime=False):
        window=self.window
        if window is None:return
        if window.get('backend_exit_verified'):self.save_window(None);return
        services={name:service.snapshot() for name,service in self.services.items()}
        if any(services.get(name,{}).get('state') not in ('running','stopped')
                for name in self.maintenance_services(dict(self.health_cache,services=services))):
            raise OperationError('maintenance_cleanup_unknown','维护作用域内服务身份未核验，保持冻结',unknown=True)
        try:state=self.state()
        except (OSError,HTTPException,ValueError):
            if self.services['workbench'].snapshot().get('running') is False:self.save_window(None);return
            raise OperationError('maintenance_cleanup_unknown','后端不可核验，旧维护冻结仍保留',unknown=True) from None
        if state['server_instance_id']!=window['backend_instance']:
            raise OperationError('maintenance_cleanup_unknown','后端实例变化但旧进程退出未留证，旧token未发给新实例',unknown=True)
        if window.get('pending_call'):
            pending=window['pending_call']
            result=self.read_pending_call()
            self.validate_control_receipt(pending['action'],result,pending['body']['expected_target'])
            self.save_window(dict(self.window,pending_call=None,last_call=dict(request_id=pending['body']['request_id'],action=pending['action'],status='completed')))
            window=self.window
        if not state.get('maintenance'):self.save_window(None);return
        if state['maintenance'].get('active_action'):raise OperationError('maintenance_busy','旧维护动作仍未结束，保持冻结')
        token=window.get('token')
        if token is None:
            value=json_request(self.port,'/api/maintenance/begin',window['begin_body'],timeout=self.timeouts['state'],
                headers={'Origin':'http://127.0.0.1:'+str(self.port),'X-CSRF-Token':state['csrf']})
            token=value.get('token')
            if not isinstance(token,str) or not token or value.get('status')=='starting':raise OperationError('maintenance_busy','旧维护准入尚未结束')
            self.save_window(dict(window,token=token))
        if inspect_runtime:
            value=self.post('inspect',token=token,timeout=self.timeouts['state'])
            target=value.get('target') or {}
            if (not {'target','runtime_same','active_action','status','phase','error'}<=set(value)
                    or value.get('runtime_same') is not True or value.get('active_action') is not None
                    or not isinstance(value.get('status'),str)
                    or set(target)!={'server_instance_id','device_id','runtime_nonce'}
                    or target['server_instance_id']!=window['backend_instance'] or target['device_id']!=window['device_id']):
                raise OperationError('maintenance_cleanup_unknown','同票据Runtime或活动操作尚未核验，维护冻结保留',unknown=True)
            self.save_window(dict(self.window,expected_target=target,recovery_inspection=value))
        elif self.backend_target(state)!=self.window.get('expected_target'):
            raise OperationError('maintenance_cleanup_unknown','Runtime在操作期间发生变化，需显式核验同票据；未自动追随新实例',unknown=True)
        if self.post('ops_maintenance_end',token=token,timeout=self.timeouts['state']).get('ended') is not True:
            raise OperationError('maintenance_cleanup_unknown','维护结束未确认')
        self.save_window(None)

    @contextmanager
    def maintenance(self,purpose,*,affected_services=None):
        state=self.state();target=self.backend_target(state)
        affected_services=sorted(affected_services if affected_services is not None else self.service_dependencies(purpose,self.health_cache))
        if not affected_services or any(name not in self.services for name in affected_services):
            raise ValueError('维护作用域必须来自本部署固定服务')
        body=dict(environment=self.environment,request_id=secrets.token_hex(16),purpose=purpose,expected_target=target)
        window=dict(environment=self.environment,device_id=self.config['device_id'],backend_instance=state['server_instance_id'],expected_target=target,begin_body=body,token=None,backend_exit_verified=False,
            affected_services=affected_services)
        self.save_window(window)
        value=json_request(self.port,'/api/maintenance/begin',body,timeout=self.timeouts['state'],
            headers={'Origin':'http://127.0.0.1:'+str(self.port),'X-CSRF-Token':state['csrf']})
        token=value.get('token')
        if not isinstance(token,str) or not token:raise OperationError('maintenance_unknown','维护准入没有返回有效凭证')
        self.save_window(dict(window,token=token))
        try:
            if value.get('status')!='ready':raise ValueError(value.get('error') or '维护准入未就绪')
            yield token
        except Exception as error:
            try:self.finish_window()
            except Exception as cleanup:
                raise OperationError('maintenance_cleanup_unknown',
                    '操作失败：'+attempt_error(str(error))+'；维护收尾尚未确认：'+attempt_error(str(cleanup)),
                    unknown=True) from error
            raise
        else:
            self.finish_window()

    def backend_attempt(self,service,*,started_after_ns=None,expected_identity=None):
        if self.backend_attempt_pointer is None:return None
        if (service.get('state')!='stopped' or service.get('running') is not False or service.get('children')
                or not (service.get('owned') is True or (service.get('unit_state') or {}).get('ActiveState') in ('inactive','failed'))
                or not service.get('identity') or expected_identity is not None
                and stable_identity(service['identity'])!=stable_identity(expected_identity)):
            return dict(status='starting' if service.get('state')=='running' else 'unknown',
                        error='进程或子进程退出尚未确认；未将最终文件判作本次启动结果')
        try:
            current=attempt_read(self.backend_attempt_pointer,self.backend_attempt_root)
            output=current.get('output')
            if (not isinstance(output,str) or not Path(output).is_absolute() or '..' in Path(output).parts
                    or Path(output).parent.resolve()!=self.backend_attempt_root or Path(output).is_symlink()
                    or not re.fullmatch(r'attempt-[0-9]+',Path(output).name)
                    or type(current.get('pid')) is not int or current['pid']<=0
                    or not isinstance(current.get('start_ticks'),str) or not current['start_ticks'].isdigit()
                    or type(current.get('started_wall_ns')) is not int or current['started_wall_ns']<=0):
                raise ValueError('启动指针身份或目录格式不可核验')
            if started_after_ns is not None and current['started_wall_ns']<started_after_ns:
                return dict(status='unknown',error='最新指针仍属于旧启动，尚无本次启动收据')
            exited=attempt_read(self.backend_attempt_root/(Path(output).name+'-exit.json'),self.backend_attempt_root)
            if (any(exited.get(key)!=current[key] for key in ('pid','start_ticks','output','started_wall_ns'))
                    or exited.get('status')!='exited' or type(exited.get('exit_code')) is not int
                    or type(exited.get('finished_wall_ns')) is not int or exited['finished_wall_ns']<current['started_wall_ns']):
                raise ValueError('启动身份或退出收据尚不可核验')
            final=attempt_read(Path(output)/'final.json',self.backend_attempt_root)
            if final.get('status')=='failed':return dict(status='failed',error=attempt_error(final.get('error')))
            return dict(status='unknown',error='进程已退出，但没有明确失败的最终启动收据')
        except (OSError,ValueError,TypeError):
            return dict(status='unknown',error='本次启动与退出收据尚不可核验，请查看受控后端日志')

    def wait_backend(self,progress=None,*,started_service=None):
        deadline=time.monotonic()+self.timeouts['backend'];error=None;attempt=None
        while time.monotonic()<deadline:
            try:return self.state()
            except Exception as exc:error=exc
            attempt=self.backend_attempt(self.services['workbench'].snapshot(),
                started_after_ns=(started_service or {}).get('startup_admitted_wall_ns'),
                expected_identity=(started_service or {}).get('identity'))
            if attempt and attempt['status']=='failed':
                if progress is not None:progress('backend_start_failed',detail=attempt)
                raise OperationError('backend_start_failed','工作台启动失败：'+attempt['error'])
            time.sleep(.05)
        if attempt is not None:
            raise OperationError('backend_start_unknown','工作台启动状态未知：'+str(attempt['error']),unknown=True)
        raise TimeoutError('工作台未恢复可读状态：'+str(error))

    def wait_model(self,started,progress):
        progress('waiting_model_endpoint')
        expected=stable_identity(started.get('identity'));deadline=time.monotonic()+self.timeouts['model']
        while time.monotonic()<deadline:
            state=self.services['model'].snapshot()
            same=expected is not None and stable_identity(state.get('identity'))==expected
            if same and state.get('state')=='stopped' and state.get('running') is False and state.get('owned') is True and not state.get('children'):
                detail=dict(status='failed',error='已登记模型服务进程及其子进程已退出：'+attempt_error(state.get('reason')))
                progress('model_start_failed',detail=detail)
                raise OperationError('model_start_failed',detail['error'])
            if same and state.get('state')=='running' and state.get('owned') is True:
                try:value=json_request(self.config['model_port'],'/status',timeout=self.timeouts['state'])
                except (OSError,HTTPException,ValueError):pass
                else:
                    if value.get('service_pid')!=expected['pid'] or type(value.get('loaded')) is not bool:
                        raise OperationError('model_identity_unknown','模型HTTP服务PID或状态与本次启动身份不符，未发送预热请求')
                    observed=self.services['model'].snapshot()
                    if observed.get('state')=='running' and observed.get('owned') is True and stable_identity(observed.get('identity'))==expected:
                        return {key:value.get(key) for key in ('service_pid','status','loaded','model_id')}
            time.sleep(.05)
        raise OperationError('model_start_unknown','等待模型HTTP入口超时；模型进程或服务身份尚未核验，未发送预热请求')

    def service_action(self,name,action,progress,*,abort_stopped=False):
        if abort_stopped and (not self.control_external or name!='workbench' or action!='stop' or self.window is not None):
            raise OperationError('consumer_exit_scope','已停退出仅用于明确的共享消费者停止操作')
        if self.control_external and name!='workbench':
            raise OperationError('external_control_lifecycle','共享服务由独立管理员管理，本工作台未执行进程操作')
        if name=='model'and self.model_external:
            raise OperationError('external_model_lifecycle','共享模型进程生命周期属于独立服务管理员，工作台未执行进程操作')
        detached=None
        if name=='workbench'and action in ('stop','restart')and self.model_external:
            before=self.services[name].snapshot()
            if before.get('running')is True:
                if self.window is not None:
                    detached=self.stop_model(self.window['token'],progress)
                else:
                    with self.maintenance('stop_consumer'if abort_stopped else'stop_model',affected_services={'workbench'})as token:
                        detached=self.stop_model(token,progress)
                        if abort_stopped:
                            detached['consumer_exit']=self.post('detach_stopped_consumer',token=token,timeout=self.timeouts['state'])
            elif before.get('state')=='stopped'and before.get('running')is False:
                detached=self.stop_model(None,progress,backend_absent=True)
        progress(action+'_'+name)
        if name=='recording' and action in ('stop','restart') and self.services[name].snapshot().get('running') is True:
            from .recording_process import recording_call
            # Refusal preserves both the service and its active writers.
            recording_call(os.environ['WORKBENCH_RECORDING_SERVICE'],'shutdown',timeout=self.timeouts['state'])
        admitted=time.time_ns()
        result=getattr(self.services[name],action)()
        if name=='recording' and action in ('start','restart'):
            from .recording_process import recording_call
            deadline=time.monotonic()+self.timeouts['backend']
            while True:
                try:
                    state=recording_call(os.environ['WORKBENCH_RECORDING_SERVICE'],'inspect',timeout=self.timeouts['state'])
                    if state['ready'] and state['pid']==(result.get('identity')or{}).get('pid'):break
                except (KeyError,OSError,ValueError):pass
                if time.monotonic()>=deadline:raise OperationError('recording_start_unknown','常驻记录服务就绪未确认',unknown=True)
                time.sleep(.05)
        if detached is not None:result=dict(result,shared_model_detachment=detached)
        expected='stopped' if action=='stop' else 'running'
        observed=self.services[name].snapshot()
        if (result.get('state')!=expected or observed.get('state')!=expected
                or expected=='stopped' and (result.get('running') is not False or observed.get('running') is not False)
                or expected=='running' and (result.get('owned') is not True or observed.get('owned') is not True
                    or stable_identity(result.get('identity'))!=stable_identity(observed.get('identity')))):
            raise OperationError('service_state_unknown',result.get('reason','进程或退出状态未核验'),unknown=True)
        if name=='workbench' and action=='start' and not result.get('reused'):
            result=dict(result,startup_admitted_wall_ns=admitted)
        return result

    def _ros_camera_profile(self):
        path=self.config.get('publication_files',{}).get('physical_profile')
        if not path:return None
        profile=json.loads(Path(path).read_text())
        if not isinstance(profile,dict):raise ValueError('physical camera profile must be an object')
        if profile.get('data_plane')is None:return None
        from .ros_data import plane_config
        plane_config(profile['data_plane'])
        if profile.get('robot_id')!=self.config['device_id']or profile.get('environment')!='physical':
            raise ValueError('registered ROS camera profile belongs to another device/environment')
        return profile

    def camera_registration_error(self):
        if 'cameras' not in self.services:return '相机服务尚未纳入当前设备管理'
        if self.config.get('camera_descriptor')or self.config.get('native_rgb'):return None
        try:
            if self._ros_camera_profile()is not None:return None
        except (OSError,ValueError,TypeError,AttributeError)as error:
            return 'ROS相机来源配置无效，'+str(error)
        return '相机服务需要已登记的ROS来源配置或可信descriptor'

    def camera_source_error(self,service):
        if (self.environment!='physical' or not self.config.get('camera_descriptor')
                or service.get('state') not in ('running','stopped')):return None
        identity=stable_identity(service.get('identity'))
        if identity is None:return None
        path=Path(self.config['camera_descriptor']).parent/'state.json'
        try:
            if path.stat().st_size<=65536:
                source=json.loads(path.read_text())
                if source.get('pid')==identity['pid'] and source.get('phase')in ('error','degraded'):
                    return attempt_error(str(source.get('error') or '相机采集服务退出'))
        except (OSError,ValueError,TypeError):pass
        return None

    def camera_action(self,action,token,progress,*,bind=True):
        progress('camera_service')
        if self.environment=='fake':
            return self.post('ops_start_cameras' if action=='start' else 'ops_restart_cameras',token=token,timeout=self.timeouts['camera'])
        if 'cameras' not in self.services:raise ValueError('相机服务尚未纳入此设备管理')
        profile=self._ros_camera_profile()
        result=self.service_action('cameras',action,progress)
        progress('waiting_camera_source')
        if profile is not None:
            from .ros_inputs import RosCameraView
            from .rgbd_service import CAMERAS
            intrinsics={name:json.loads(Path(profile['cameras'][role]['intrinsics']).read_text())for role,name in CAMERAS.items()}
            client=RosCameraView(profile['data_plane'],profile['robot_id'],intrinsics,'physical')
            deadline=time.monotonic()+self.timeouts['camera'];error=None
            try:
                while time.monotonic()<deadline:
                    try:
                        for camera in intrinsics:client.latest_rgbd(camera)
                        break
                    except (LookupError,ValueError)as exc:error=exc
                    if self.services['cameras'].snapshot().get('state')=='stopped':
                        raise OperationError('camera_start_failed','已登记ROS相机源退出，请检查相机日志')
                    time.sleep(.05)
                else:raise TimeoutError('ROS相机未恢复原始帧：'+str(error))
            finally:client.close()
            if not bind:return dict(service=result,binding=dict(status='deferred_until_business_start'))
            progress('rebinding_camera_clients')
            return dict(service=result,binding=self.post('ops_rebind_cameras',token=token,timeout=15))
        if self.config.get('native_rgb'):
            from .prometheus_rgb import RGBClient,CAMERAS
            from .mixed366_control import SERIALS
            client=RGBClient(**self.config['native_rgb'])
            frames={role:client.latest_rgb(camera)['metadata']for role,camera in zip(('head','left','right'),CAMERAS)}
            if {role:row['serial']for role,row in frames.items()}!=SERIALS:raise ValueError('raw RGB camera identities differ')
            return dict(service=result,binding=dict(raw_rgb=True,frames=frames,hardware_opened=False))
        from .rgbd_service import CommonCameraService
        deadline=time.monotonic()+self.timeouts['camera'];error=None
        while time.monotonic()<deadline:
            try:
                client=CommonCameraService.connect(self.config['camera_descriptor'])
                for channel in client.frames:client.latest_rgbd(channel)
                break
            except Exception as exc:error=exc
            observed=self.services['cameras'].snapshot()
            expected=stable_identity(result.get('identity'))
            if (expected is not None and observed.get('state')=='stopped' and observed.get('running') is False
                    and stable_identity(observed.get('identity'))==expected):
                detail=self.camera_source_error(observed) or '已登记相机服务及其子进程已退出'
                raise OperationError('camera_start_failed',detail+'；请检查对应相机与USB链路，再显式启动相机服务')
            time.sleep(.05)
        else:raise TimeoutError('相机未恢复真实帧：'+str(error))
        if not bind:return dict(service=result,binding=dict(status='deferred_until_business_start'))
        progress('rebinding_camera_clients')
        return dict(service=result,binding=self.post('ops_rebind_cameras',token=token,timeout=15))

    def camera_reset_command(self,role):
        if role not in CAMERA_RESETS.values() or self.environment!='physical':raise ValueError('只支持三个固定物理相机角色')
        spec=getattr(self.services.get('cameras'),'spec',None)
        if spec is None or Path(spec.expected_script).name!='serve_camera_sources.py':
            raise ValueError('当前相机源没有登记兼容的软件复位程序')
        configured=self.config.get('camera_reset');declared=None
        if spec.kind=='process':
            command=list(spec.command)
            def argument(name):
                if command.count(name)!=1 or command.index(name)+1>=len(command):raise ValueError('相机源缺少固定'+name+'配置')
                return command[command.index(name)+1]
            declared=dict(python=command[0],profile=argument('--profile'),root=argument('--root'))
            if configured is not None and configured!=declared:raise ValueError('相机复位配置与已登记采集服务不一致')
        selected=configured or declared
        if (selected is None or any(not Path(p).is_absolute() or '..' in Path(p).parts for p in selected.values())
                or not re.fullmatch(r'python(?:\d+(?:\.\d+)*)?',Path(selected['python']).name)):
            raise ValueError('当前相机源尚未登记可信的复位Python/profile/root')
        if any(not Path(p).is_file() for p in (selected['python'],selected['profile'],spec.expected_script)):
            raise ValueError('登记的相机复位Python、profile或源程序不可用')
        return [selected['python'],'-B',spec.expected_script,'--profile',selected['profile'],'--root',selected['root'],'--reset-role',role]

    def reset_camera(self,role,token,progress):
        from .rgbd_service import CommonCameraService,CAMERAS
        command=self.camera_reset_command(role);profile=json.loads(Path(command[command.index('--profile')+1]).read_text())
        if profile.get('data_plane')is not None:
            registered=self._ros_camera_profile()
            matches=bool(registered and profile['data_plane']==registered['data_plane']
                and all(profile['cameras'][r]['serial']==registered['cameras'][r]['serial']for r in CAMERAS))
        else:
            descriptor=json.loads(Path(self.config['camera_descriptor']).read_text())
            matches=(descriptor.get('environment')=='physical'
                and all(profile['cameras'][r]['serial']==descriptor['intrinsics'][camera]['serial']for r,camera in CAMERAS.items()))
        if (profile.get('environment')!='physical'
                or profile.get('robot_id',self.config['device_id'])!=self.config['device_id'] or not matches):
            raise OperationError('camera_reset_precondition','复位profile、设备或当前相机descriptor身份不一致')
        state=self.services['cameras'].snapshot()
        if state.get('state')=='running':
            if profile.get('data_plane')is not None:
                from .recording_process import recording_call
                recorded=recording_call(os.environ['WORKBENCH_RECORDING_SERVICE'],'inspect',timeout=self.timeouts['state'])
                if recorded.get('can_stop')is not True:raise OperationError('camera_reset_precondition','录制仍在进行或封存，禁止USB软件复位')
            else:
                client=CommonCameraService.connect(self.config['camera_descriptor'])
                if any(row.get('run_id') for row in client.snapshot()['recording_channels'].values()):
                    raise OperationError('camera_reset_precondition','相机仍有录制租约，禁止USB软件复位')
        elif state.get('state')!='stopped' or state.get('running') is not False:
            raise OperationError('camera_reset_precondition','相机进程或子进程状态未知，禁止USB软件复位')
        stopped=self.service_action('cameras','stop',progress)
        progress('resetting_camera_usb',detail=dict(role=role,three_streams_paused=True,controller_untouched=True))
        spec=self.services['cameras'].spec
        try:
            reset=subprocess.run(command,stdin=subprocess.DEVNULL,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,
                timeout=self.timeouts['camera'],env=dict(os.environ,**(spec.env or {})))
            if reset.returncode:
                tail=reset.stderr.strip().splitlines()
                raise ValueError(attempt_error(tail[-1] if tail else '相机复位程序退出'))
            evidence=json.loads(reset.stdout.strip().splitlines()[-1])
            if (evidence.get('status')!='enumerated' or evidence.get('role')!=role or evidence.get('reset_calls')!=1
                    or evidence.get('serial')!=profile['cameras'][role]['serial']):raise ValueError('复位程序未返回匹配设备的枚举证据')
            evidence={key:evidence.get(key) for key in ('status','role','serial','reset_calls','disappearance_observed','raw_frames_verified')}
        except (OSError,ValueError,IndexError,subprocess.TimeoutExpired) as error:
            progress('camera_reset_unknown',detail=dict(status='unknown',error=attempt_error(str(error)),camera_service=stopped))
            self.refresh()
            raise OperationError('camera_reset_unknown','单路相机复位结果未知；相机源保持停止，未重试复位或重启机器人SDK：'+attempt_error(str(error))) from None
        progress('waiting_camera_source')
        try:restored=self.camera_action('start',token,progress)
        except (OSError,HTTPException,ValueError,TimeoutError) as error:
            progress('camera_reset_recovery_unknown',detail=dict(status='unknown',reset=evidence,
                camera_service=self.services['cameras'].snapshot(),error=attempt_error(str(error))))
            self.refresh()
            raise OperationError('camera_reset_recovery_unknown','已请求一次USB复位并取得设备枚举；三路采集恢复尚未核验，不自动重试：'+attempt_error(str(error))) from None
        return dict(reset=evidence,cameras=restored,controller_untouched=True)

    def model_prepare_error(self,error):
        message=attempt_error(str(error))
        try:
            state=self.services['model'].snapshot();value=json_request(self.config['model_port'],'/status',timeout=self.timeouts['state'])
            if (state.get('owned') is not True or state.get('state')!='running' or value.get('status')!='error'
                    or value.get('service_pid')!=(state.get('identity') or {}).get('pid')):return message
            tail=value.get('stderr_tail','')
            if not isinstance(tail,str):return message
            line=next((line.strip() for line in reversed(tail[-8192:].splitlines())
                       if re.fullmatch(r'(?:[A-Za-z_]\w*\.)*[A-Za-z_]*\w*(?:Error|Exception):[^\r\n]*',line.strip())),None)
            return message+'；'+attempt_error(line) if line else message
        except (OSError,HTTPException,ValueError,KeyError):return message

    def camera_evidence(self):
        """Wait only for actual HTTP images; never repeat the camera action."""
        deadline=time.monotonic()+self.timeouts['camera'];error=None
        while time.monotonic()<deadline:
            cameras={}
            try:
                for role in ('head','left','right'):
                    remaining=deadline-time.monotonic()
                    if remaining<=0:raise TimeoutError('相机图像验证等待已到期')
                    connection=HTTPConnection('127.0.0.1',self.port,timeout=min(self.timeouts['state'],remaining))
                    try:
                        connection.request('GET','/api/media/'+role+'?environment='+self.environment,headers={'Connection':'close'})
                        response=connection.getresponse();raw=response.read(LIMIT+1)
                        stamp=response.getheader('X-Source-Wall-Ns');frame=response.getheader('X-Frame-Id')
                        kind=response.getheader('Content-Type','').split(';')[0]
                        image=(kind=='image/jpeg' and raw.startswith(b'\xff\xd8') and raw.endswith(b'\xff\xd9')
                            or kind=='image/png' and raw.startswith(b'\x89PNG\r\n\x1a\n'))
                        if (response.status!=200 or not image
                                or not raw or len(raw)>LIMIT or not stamp or not stamp.isdigit() or not frame):
                            raise ValueError(role+'相机没有返回实际图像、帧身份和源时间')
                        age=(time.time_ns()-int(stamp))/1e9
                        if not 0<=age<=2.:raise ValueError(role+'相机画面陈旧或源时钟不匹配')
                        cameras[role]=dict(frame_id=frame,host_receive_ns=int(stamp),source_age_s=age,bytes=len(raw),healthy=True)
                    finally:connection.close()
                if any(not 0<=(time.time_ns()-row['host_receive_ns'])/1e9<=2. for row in cameras.values()):
                    raise ValueError('三路相机没有同时保持新鲜')
                if time.monotonic()>=deadline:raise TimeoutError('相机图像验证等待已到期')
                return cameras
            except (OSError,HTTPException,ValueError) as exc:error=exc
            remaining=deadline-time.monotonic()
            if remaining>0:time.sleep(min(.05,remaining))
        raise TimeoutError('三路相机HTTP图像未在等待时限内就绪：'+str(error)) from None

    def model_evidence(self,selected,bundle_digest=None):
        service=self.services['model'].snapshot()
        if service.get('state')!='running' or service.get('owned') is not True:raise ValueError('模型服务进程身份未核验')
        value=json_request(self.config['model_port'],'/status',timeout=self.timeouts['state'])
        if (value.get('service_pid')!=(service.get('identity') or {}).get('pid') or value.get('model_id')!=selected
                or value.get('loaded') is not True or value.get('status')!='ready' or (value.get('warmup') or {}).get('completed') is not True):
            raise ValueError('模型服务身份、选择或暖机结果未核验')
        if value.get('backend') is not None and (bundle_digest is None or value.get('bundle_digest')!=bundle_digest):
            raise ValueError('模型模式或实现摘要与所选运行时不一致')
        return dict(service_identity=stable_identity(service['identity']),worker_pid=value.get('worker_pid'),
            model_id=selected,backend=value.get('backend'),bundle_digest=value.get('bundle_digest'),warmup=deepcopy(value['warmup']),loaded=True)

    def warm_model_binding(self,state=None):
        if 'model' not in self.services or 'model_port' not in self.config:return None
        observed=self.health_cache.get('model_service') or {}
        if (observed.get('loaded') is not True or observed.get('status')!='ready'
                or (observed.get('warmup') or {}).get('completed') is not True):return None
        info=(state or {}).get('model') or {}
        selected=info.get('selected_model_id') or info.get('id') or observed.get('model_id')
        bundle_digest=info.get('bundle_digest') or observed.get('bundle_digest')
        evidence=self.model_evidence(selected,bundle_digest)
        if type(evidence['worker_pid']) is not int or evidence['worker_pid']<=0:
            raise OperationError('model_identity_unknown','暖模型worker身份未核验，请先检查模型服务')
        return dict(model_id=selected,bundle_digest=bundle_digest,evidence=evidence)

    def _publication_control_valid(self,runtime,expected_owner):
        if runtime.get('automatic_enabled'):return False
        if expected_owner is None:return runtime.get('connected')is False
        return bool(self.config.get('control_owner_descriptor')and all(runtime.get(key)==value for key,value in expected_owner.items()))

    def restore_warm_model_binding(self,binding,progress,*,expected_owner=None):
        if binding is None:return dict(status='not_requested',reason='重载前没有已核验的暖模型；未自动加载')
        purpose='prepare_model' if self.control_external else 'restart_workbench'
        with self.maintenance(purpose) as token:
            state=self.state();info=state.get('model') or {}
            if not self._publication_control_valid(state['runtime'],expected_owner):
                raise OperationError('unexpected_connection','后端重载后出现意外设备连接，未恢复模型绑定')
            selected=info.get('selected_model_id') or info.get('id')
            digest=info.get('bundle_digest')
            if (selected!=binding['model_id'] or binding['bundle_digest'] is not None and digest!=binding['bundle_digest']):
                raise OperationError('model_rebind_required','后端已重载，但模型选择或bundle身份已变化；请核验并显式选择原模型，未自动切换或加载')
            current=self.model_evidence(selected,binding['bundle_digest'])
            if any(current[key]!=binding['evidence'][key] for key in ('service_identity','worker_pid')):
                raise OperationError('model_rebind_required','后端已重载，但原暖模型进程身份已变化；请核验模型服务后显式准备')
            progress('restoring_warm_model_binding')
            prepared=self.post('ops_prepare_model',token=token,expected_model_id=selected,timeout=30)
            if prepared.get('model_ready') is not True or prepared.get('control_ready') is not False:
                raise OperationError('model_rebind_required','暖模型工作区绑定未确认；请核验模型，不会连接或恢复机器人动作')
            evidence=self.model_evidence(selected,binding['bundle_digest'])
            if any(evidence[key]!=binding['evidence'][key] for key in ('service_identity','worker_pid')):
                raise OperationError('model_rebind_required','恢复绑定期间暖模型进程发生变化，请重新核验')
            return dict(status='restored',prepared=prepared,evidence=evidence)

    def complete_publication(self,binding,progress,*,expected_owner=None):
        """Shared closure for backend reloads and new-code publications; never acquire SDK."""
        state=self.state()
        if not self._publication_control_valid(state['runtime'],expected_owner):
            raise OperationError('unexpected_connection','发布收口要求设备未连接；不会代替操作者释放或接入')
        if not self.deployment.snapshot(check_files=True,force=True)['files_unchanged']:
            raise OperationError('deployment_changed','发布期间组合清单发生变化')
        restored=(self.restore_warm_model_binding(binding,progress,expected_owner=expected_owner)
            if expected_owner is not None else self.restore_warm_model_binding(binding,progress))
        health=self.refresh()
        if (health.get('backend_reachable') is not True or health.get('maintenance')
                or not self._publication_control_valid(health.get('runtime',{}),expected_owner)or self.window is not None):
            raise OperationError('publication_unconfirmed','发布收口状态未核验，未接入SDK',unknown=True)
        if binding is not None and health.get('model',{}).get('model_ready') is not True:
            raise OperationError('model_rebind_required','热模型工作区绑定仍未就绪，发布尚未验收')
        return restored

    def track_child(self,name,pid):
        if type(pid) is not int or pid<=0:raise OperationError('child_identity_unknown','缺少已连接SDK或模型worker的真实PID，未停止服务')
        tracker=getattr(self.services[name],'track_child',None)
        if tracker is None:raise OperationError('child_identity_unknown','服务适配器不能核验子进程归属，未停止服务')
        result=tracker(pid)
        if result.get('state')!='running' or result.get('owned') is not True:
            raise OperationError('child_identity_unknown',result.get('reason','子进程身份未核验'))

    def _prepare_observation_and_model(self,token,progress,started_model):
        from concurrent.futures import ThreadPoolExecutor
        # Only raw source readiness runs in parallel. The shared maintenance
        # RPC is still serialized: model preparation, then camera client bind.
        with ThreadPoolExecutor(max_workers=1)as worker:
            camera=worker.submit(self.camera_action,'start',token,progress,bind=False)
            model=self.prepare_model('prepare_environment',token,progress,{},started_service=started_model)
            cameras=camera.result()
        if not self.config.get('native_rgb'):
            progress('rebinding_camera_clients')
            cameras=dict(cameras,binding=self.post('ops_rebind_cameras',token=token,timeout=15))
        return dict(cameras=cameras,model=model)

    def prepare_model(self,action,token,progress,parameters,*,started_service=None):
        prior=self.services['model'].snapshot()
        if self.model_external and action=='restart_model':
            raise OperationError('external_model_lifecycle','共享模型重启属于独立服务管理员')
        if action=='restart_model' and prior.get('running'):
            old=json_request(self.config['model_port'],'/status',timeout=self.timeouts['state'])
            if old.get('service_pid')!=(prior.get('identity') or {}).get('pid'):
                raise OperationError('model_identity_unknown','模型HTTP身份与本任务进程不一致，未停止服务')
            if old.get('worker_pid') is not None or old.get('loaded') is True:self.track_child('model',old.get('worker_pid'))
        service=(started_service if started_service is not None else prior if self.model_external else
            self.service_action('model','restart' if action=='restart_model' else 'start',progress))
        if self.model_external and (service.get('state')!='running'or service.get('owned')is not True):
            raise OperationError('external_model_unavailable','共享模型服务未就绪，请由服务管理员启动；工作台未启动进程')
        if action=='restart_model' and prior.get('running') and stable_identity(prior.get('identity'))==stable_identity(service.get('identity')):
            raise OperationError('model_not_restarted','模型进程身份未变化，重启未核验')
        self.wait_model(service,progress)
        progress('warming_model')
        try:prepared=self.post('ops_prepare_model',token=token,timeout=120,**parameters)
        except (OSError,HTTPException,ValueError) as error:
            if isinstance(error,OperationError):raise
            message=self.model_prepare_error(error)
            progress('model_prepare_failed',detail=dict(status='failed',error=message))
            raise OperationError('model_prepare_failed',message) from None
        if prepared.get('model_ready') is not True or prepared.get('control_ready') is not False:
            raise OperationError('model_not_prepared','模型准备未确认或意外绑定了设备控制')
        progress('verifying_model_identity')
        return dict(service=service,prepared=prepared,evidence=self.model_evidence(prepared.get('model_id'),prepared.get('bundle_digest')))

    def stop_model(self,token,progress,*,backend_absent=False):
        if self.model_external:
            if backend_absent:
                return dict(lifecycle='external',process_lifecycle_changed=False,binding=dict(invalidated=False,
                    pin_release=dict(status='unconfirmed_backend_absent',expiry='awaiting_server_TTL_or_admin_shutdown')))
            progress('releasing_consumer_model_binding')
            binding=self.post('invalidate_model',token=token,timeout=self.timeouts['state'])
            if (binding.get('invalidated')is not True or binding.get('model_ready')is not False
                    or (binding.get('pin_release')or{}).get('status')not in ('released','not_acquired')):
                raise OperationError('model_binding_unknown','本工作台模型pin释放未确认；共享服务保持，未执行进程操作',unknown=True)
            return dict(lifecycle='external',process_lifecycle_changed=False,binding=binding)
        prior=self.services['model'].snapshot()
        if prior.get('running'):
            state=json_request(self.config['model_port'],'/status',timeout=self.timeouts['state'])
            if state.get('service_pid')!=(prior.get('identity') or {}).get('pid'):
                raise OperationError('model_identity_unknown','模型HTTP与已登记进程不一致，未停止',unknown=True)
            if state.get('loaded') is True or state.get('worker_pid') is not None and type(state.get('worker_exit_code')) is not int:
                self.track_child('model',state.get('worker_pid'))
        binding=None
        if not backend_absent:
            progress('invalidating_model_binding')
            binding=self.post('invalidate_model',token=token,timeout=self.timeouts['state'])
            if binding.get('invalidated') is not True or binding.get('model_ready') is not False:
                raise OperationError('model_binding_unknown','业务绑定撤销未核验，模型服务继续运行',unknown=True)
        stopped=self.service_action('model','stop',progress)
        if backend_absent:return dict(service=stopped,binding=dict(status='not_applicable',reason='business backend exit verified; no live model binding'))
        return dict(service=stopped,binding=binding)

    def run(self,action,request,progress):
        generation=self.stop_generation
        last_report=None;report_lock=threading.Lock()
        def report(*args,**kwargs):
            nonlocal last_report
            with report_lock:
                if generation!=self.stop_generation:
                    raise OperationError('operator_stop','停止已撤销后续准备或接入步骤；已开始的服务状态需读回')
                if last_report==(args,kwargs):return  # Polling the same phase must not fsync an identical job at control rate.
                last_report=deepcopy((args,kwargs))
                phase=args[0]if args else kwargs['phase'];refs=_operation_step_refs(action,phase)
                if refs:
                    fraction=args[1]if len(args)>1 else kwargs.get('fraction')
                    detail=args[2]if len(args)>2 else kwargs.get('detail')
                    if detail is None or isinstance(detail,dict):
                        return progress(phase,fraction,dict(detail or{},step_refs=refs))
                return progress(*args,**kwargs)
        try:
            result=self._run(action,request,report)
            report('verifying_postconditions')
            if action!='status_check':
                before=result.get('before',{})
                result['dependencies']=before.pop('dependencies',[])
            return result
        except (OSError,HTTPException) as error:
            raise OperationError('operation_state_unknown',str(error),unknown=True) from None

    def _run(self,action,request,progress):
        observation_backend_start=None
        self.refresh()
        if self.target()!=request['expected_target']:raise OperationError('target_changed','执行前目标身份已变化，请重新审阅')
        reason=self.check(action,request)
        if reason:raise ValueError(reason)
        if self.control_external and action in ('exit_environment','stop_workbench','restart_workbench','connect_control'):
            return self.consumer_lifecycle(action,request,progress)
        if action=='reclaim_control':return self.reclaim_control(request,progress)
        if action=='exit_environment':return self.exit_environment(request,progress)
        if action=='recover_control' and self.recovery_requires_support(request) and request.get('arms_supported')is not True:
            raise OperationError('support_required','当前需要释放重接；未确认支撑，不执行恢复。')
        before=self.target();parameters=request.get('parameters') or {}
        before['dependencies']=self.dependency_evidence(action,self.health_cache,parameters)
        if action!='status_check':progress('checking_service_dependencies',detail=dict(dependencies=before['dependencies']))
        scope=self.service_dependencies(action,self.health_cache,parameters)
        preserved={name:deepcopy(row) for name,row in self.health_cache.get('services',{}).items() if name not in scope}
        wanted=set(parameters.get('capabilities',['observe'])) if action=='prepare_environment' else set()
        if action=='status_check':return dict(action=action,health=self.refresh(),before=before)
        if self.config.get('control_owner_descriptor')and action in('restart_workbench','stop_workbench'):
            owner=self.health_cache['runtime']
            binding=self.warm_model_binding()if action=='restart_workbench'else None
            stopped=self.service_action('workbench','stop',progress)
            result=dict(action=action,before=before,stopped_backend=stopped,release=None,
                preserved_services=preserved,independent_owner=dict(pid=owner.get('pid'),runtime_nonce=owner.get('runtime_nonce')))
            if action=='restart_workbench':
                started=self.service_action('workbench','start',progress)
                state=self.wait_backend(progress,started_service=started)
                actual=state['runtime']
                if any(actual.get(key)!=result['independent_owner'][key]for key in('pid','runtime_nonce')):
                    raise OperationError('owner_changed','业务重载后独立Owner身份发生变化；未自动接入或重放')
                result.update(started_backend=started,backend=state['server_instance_id'],
                    model_binding=self.complete_publication(binding,progress,expected_owner=result['independent_owner']))
            return result
        if action=='recover_maintenance' or action=='recover_control' and (self.window is not None or self.unresolved()):
            progress('verifying_old_maintenance');unresolved=self.unresolved();self.finish_window(inspect_runtime=True);health=self.refresh()
            if health.get('maintenance') or health.get('busy') or health.get('runtime',{}).get('automatic_enabled'):
                raise OperationError('maintenance_cleanup_unknown','遗留维护或业务活动仍未结束',unknown=True)
            self.recovered.update(unresolved);private_save(self.recovery_path,dict(resolved_job_ids=sorted(self.recovered)))
            if action=='recover_maintenance':return dict(action=action,before=before,resolved_job_ids=unresolved,maintenance_ended=True,motion_resumed=False,preserved_services=preserved)
            runtime=health.get('runtime') or {}
            if self.control_healthy(runtime) and not runtime.get('recoverable_fault'):
                return dict(action=action,before=before,maintenance_ended=True,motion_resumed=False,
                    control=dict(already_operable=True,runtime=runtime,motion_resumed=False,sdk_reconnected=False),preserved_services=preserved)
        if action=='recover_control' and self.control_healthy(self.health_cache.get('runtime')) and not self.health_cache['runtime'].get('recoverable_fault'):
            return dict(action=action,before=before,motion_resumed=False,
                control=dict(already_operable=True,runtime=self.health_cache['runtime'],motion_resumed=False,sdk_reconnected=False),preserved_services=preserved)
        if not self.health_cache.get('backend_reachable'):
            backend=self.services['workbench'].snapshot()
            if backend.get('state')=='stopped' and backend.get('running') is False:
                if action=='stop_model':
                    return dict(action=action,before=before,stopped_model=self.stop_model(None,progress,backend_absent=True),
                        backend_absent=True,preserved_services=preserved)
                if action=='stop_cameras' and 'cameras' in self.services:
                    return dict(action=action,before=before,stopped_cameras=self.service_action('cameras','stop',progress),
                        backend_absent=True,preserved_services=preserved)
            if action=='stop_workbench':
                stopped=self.services['workbench'].snapshot()
                if stopped.get('state')!='stopped' or stopped.get('running') is not False:
                    raise OperationError('backend_exit_unknown','后端退出尚未确认，未启动或停止未知进程',unknown=True)
                return dict(action=action,before=before,stopped_backend=stopped,already_stopped=True,release=None,preserved_services=preserved,
                    shared_model_detachment=self.stop_model(None,progress,backend_absent=True)if self.model_external else None)
            if action=='recover_control' and self.services['workbench'].snapshot().get('running') is not False:
                raise OperationError('backend_state_unknown','后端进程仍在但接口不可达；保持冻结，不停止未知控制。请检查连接或现场停止后核验退役',unknown=True)
            if action=='recover_control' and self.environment=='physical':
                camera_error=self.camera_registration_error()
                if camera_error:raise ValueError(camera_error)
            binding=self.warm_model_binding() if action=='restart_workbench' else None
            # Physical bootstrap requires the independent source descriptor; Fake owns its source inside the backend.
            if 'recording' in self.services:self.service_action('recording','start',progress)
            if 'pedal' in self.services:self.service_action('pedal','start',progress)
            if self.environment=='physical' and 'cameras' in self.services:self.camera_action('start',None,progress,bind=False)
            started_service=self.service_action('workbench','start',progress)
            started=self.wait_backend(progress,started_service=started_service)
            if started['runtime']['connected'] or started['runtime'].get('automatic_enabled'):
                raise OperationError('unexpected_connection','启动后端后出现本次操作未允许的设备连接')
            if action in ('start_cameras','restart_cameras'):
                observation_backend_start=dict(instance=started['server_instance_id'],
                    identity=stable_identity(started_service['identity']))
            if action=='restart_workbench':return dict(action=action,before=before,backend=self.state()['server_instance_id'],
                model_binding=self.complete_publication(binding,progress))
        with self.maintenance('prepare_collection' if action in COMPUTE_RESTARTS else action,affected_services=scope) as token:
            result={}
            if 'recording' in self.services and action in ('prepare_environment','connect_control','recover_control'):
                result['recording']=self.service_action('recording','start',progress)
            if 'pedal' in self.services and action in ('prepare_environment','connect_control','recover_control'):
                result['pedal']=self.service_action('pedal','start',progress)
            # Start the independent process before waiting for camera frames;
            # retain its identity so an early exit cannot silently start a new one.
            started_model=(self.service_action('model','start',progress)
                if {'observe','inference'}<=wanted and 'model'in self.services and not self.model_external else None)
            parallel=started_model is not None and self.environment=='physical'
            if parallel:result.update(self._prepare_observation_and_model(token,progress,started_model))
            if not parallel and (action in ('start_cameras','restart_cameras') or 'observe' in wanted):
                result['cameras']=self.camera_action('restart' if action=='restart_cameras' else 'start',token,progress)
            elif action=='stop_cameras':
                progress('stopping_cameras')
                result['stopped_cameras']=(self.post('stop_cameras',token=token,timeout=self.timeouts['camera'])
                    if self.environment=='fake' else self.service_action('cameras','stop',progress))
                if self.environment=='fake' and not _camera_stop_verified(result['stopped_cameras']):
                    raise OperationError('camera_exit_unknown','相机源停止未核验，采样状态需人工核验',unknown=True)
            elif action=='rebind_cameras':result['cameras']=self.post('ops_rebind_cameras',token=token,timeout=15)
            elif action in CAMERA_RESETS:result['cameras']=self.reset_camera(CAMERA_RESETS[action],token,progress)
            if not parallel and (action in ('prepare_model','restart_model') or 'inference' in wanted):
                if 'model' not in self.services:raise ValueError('当前档案未配置模型服务；标定与观测可单独准备')
                result['model']=self.prepare_model(action,token,progress,parameters if action=='prepare_model' else {},
                    **({'started_service':started_model}if started_model is not None else {}))
            if action=='stop_model':result['stopped_model']=self.stop_model(token,progress)
            if action=='prepare_collection' or action in COMPUTE_RESTARTS or 'collection' in wanted:
                progress('preparing_collection')
                options=(dict(service_role=COMPUTE_RESTARTS[action],restart=True) if action in COMPUTE_RESTARTS
                    else parameters if action=='prepare_collection' else {})
                result['collection']=self.post('prepare_collection',token=token,timeout=self.timeouts['operation'],**options)
            if action in ('connect_control','recover_control'):
                runtime=self.state()['runtime']
                retire=(action=='recover_control' and runtime.get('connected') is True
                    and not runtime.get('recoverable_fault') and not self.control_healthy(runtime))
                if action=='connect_control' or runtime.get('connected') is False or retire:
                    if self.environment=='physical' and request.get('arms_supported')is not True:
                        raise OperationError('support_required','恢复条件已变化，需要释放重接；未确认支撑，不执行后续接入。')
                    result['cameras']=self.camera_action('start',token,progress)
                    progress('verifying_observation_before_control')
                    try:result['observation_before_control']=self.camera_evidence()
                    except (OSError,HTTPException,ValueError) as error:
                        raise OperationError('observation_not_ready','接入前观测条件未达到，未发送SDK接入：'+str(error)) from None
                    if retire:
                        progress('releasing_faulted_control')
                        # post() requires a matching released target and confirmed
                        # SDK close before the new connect request can be sent.
                        result['release']=self.post('release_control',token=token,
                            arms_supported=request['arms_supported'],timeout=self.timeouts['release'])
                    progress('connecting_control')
                    result['control']=self.post('connect_control',token=token,arms_supported=request['arms_supported'],timeout=max(20,self.timeouts['release']))
                    if action=='recover_control':result['control'].update(reconnected=True,motion_resumed=False,sdk_reconnected=True)
                elif runtime.get('recoverable_fault'):
                    progress('acknowledging_recoverable_fault')
                    result['control']=self.post('recover_control',token=token,arms_supported=request['arms_supported'],timeout=max(20,self.timeouts['release']))
                elif self.control_healthy(runtime):
                    result['control']=dict(already_operable=True,runtime=runtime,motion_resumed=False,sdk_reconnected=False)
                else:
                    raise OperationError('control_retirement_required','当前连接不能按可恢复运动故障处理；先检查现场支撑与停止条件，再使用“释放设备控制”核验SDK退出，之后重新接入。释放未确认时保持冻结')
            elif action in ('release_control','verify_hold'):
                progress(action)
                result['control']=self.post(action,token=token,arms_supported=request['arms_supported'],timeout=max(20,self.timeouts['release']))
            if action in ('restart_workbench','stop_workbench'):
                state=self.state();binding=self.warm_model_binding(state) if action=='restart_workbench' else None
                if state['runtime'].get('connected'):
                    self.track_child('control_owner'if self.config.get('control_owner_descriptor')else'workbench',state['runtime'].get('pid'))
                    progress('releasing_control')
                    result['release']=self.post('release',token=token,arms_supported=request['arms_supported'],timeout=self.timeouts['release'])
                    current=self.state()
                    if current['server_instance_id']!=before['backend_instance']:raise OperationError('backend_changed','释放过程中后端实例变化')
                    if current['runtime'].get('connected') is not False:raise OperationError('release_unconfirmed','控制释放尚未确认，未重启',unknown=True)
                elif state['runtime'].get('release_receipt') and not self.release_verified(state['runtime']['release_receipt']):
                    raise OperationError('release_unconfirmed','旧SDK释放回执尚未确认，未停止后端',unknown=True)
                result['stopped_backend']=self.service_action('workbench','stop',progress)
                self.save_window(dict(self.window,backend_exit_verified=True))
                if action=='restart_workbench':
                    result['started_backend']=self.service_action('workbench','start',progress)
                    self.wait_backend(progress,started_service=result['started_backend'])
                    result['backend']=self.state()['server_instance_id']
            result.update(action=action,before=before,requested_capabilities=sorted(wanted),preserved_services=preserved)
            if observation_backend_start is not None:result['observation_backend_start']=observation_backend_start
        if action=='restart_workbench':result['model_binding']=self.complete_publication(binding,progress)
        return result

    def verify(self,result):
        action=result['action'];evidence={}
        if result.get('consumer_lifecycle'):
            health=self.refresh();owner=(health.get('runtime')or{}).get('control_owner')or{}
            holder=owner.get('consumer_holder');backend=health.get('services',{}).get('workbench',{})
            start=action in ('restart_workbench','connect_control')
            same=all(row.get('state')in ('running','stopped')
                and stable_identity(health.get('services',{}).get(n,{}).get('identity'))==stable_identity(row.get('identity'))
                and health.get('services',{}).get(n,{}).get('state')==row.get('state')
                for n,row in result['preserved_services'].items())
            same_runtime=self.same_shared_runtime(result['before'],health)
            ok=(health.get('control_owner_reachable')is True and same and same_runtime and
                (holder==self.config['control_consumer_id']and owner.get('consumer_attached')is True
                    and health.get('backend_reachable')is True if start else
                 holder!=self.config['control_consumer_id']and backend.get('state')=='stopped'and backend.get('running')is False))
            return dict(ok=ok,consumer_detached=not start and holder!=self.config['control_consumer_id'],
                independent_services_preserved=same,shared_runtime_preserved=same_runtime,
                sdk_released=False,collection_started=False,health=health)
        if action=='exit_environment':
            from scripts.workbench_deploy import verify_unit,http_json
            health=self.refresh();states=health.get('services',{})
            stopped=all(states.get(name,{}).get('state')=='stopped'and states[name].get('running')is False for name in self.managed_service_names())
            pin=self.config['collection_handoff'];unit=verify_unit({k:pin[k]for k in ('unit','files')});web=http_json(pin['status_url'])
            released=result.get('sdk_was_disconnected')is True or self.release_verified(result.get('release_receipt')or{})
            web_ready=(unit['ActiveState']=='active'and int(unit['MainPID'])>0
                and web.get('robot_vin')==self.config['device_id']and web.get('orphan_alive')is False)
            return dict(ok=stopped and released and web_ready,
                resources=states,environment_exited=stopped and released,sdk_release_verified=released,collection_web_ready=web_ready,collection_web=dict(ready=web_ready,manager_alive=web.get('manager_alive'),collect_status=web.get('collect_status')),
                collection_started_by_operation=False,motion_resumed=False)
        try:
            if action in ('start_cameras','restart_cameras','rebind_cameras',*CAMERA_RESETS) or 'cameras' in result:
                evidence['cameras']=self.camera_evidence()
        except (OSError,HTTPException,ValueError,KeyError) as error:
            return dict(ok=False,reason=str(error),health=self.refresh(),evidence=evidence,motion_started=False)
        # Waiting for preview encoding can outlive the first health snapshot.
        health=self.refresh()
        if action=='status_check':return dict(ok=True,health=health,inspection_only=True)
        backend=health.get('services',{}).get('workbench',{})
        independent={}
        for name,row in result.get('preserved_services',{}).items():
            current=health.get('services',{}).get(name,{})
            known=all(part.get('state') in ('running','stopped') for part in (row,current))
            same=(row.get('state'),row.get('running'),stable_identity(row.get('identity')))==(
                current.get('state'),current.get('running'),stable_identity(current.get('identity')))
            independent[name]=dict(status='not_verified' if not known else 'preserved' if same else 'observed_changed',
                operated=False,before=deepcopy(row),after=deepcopy(current))
        preserved=all(row['status']!='observed_changed' for row in independent.values())
        preservation=dict(independent_services=independent,independent_services_preserved=(
            None if any(row['status']=='not_verified' for row in independent.values()) else preserved))
        evidence.update(preservation)
        if action=='recover_maintenance':return dict(ok=self.window is None and not self.unresolved() and not health.get('maintenance'),health=health,evidence=evidence,motion_resumed=False)
        if action=='stop_workbench':
            receipt=result.get('release')
            ok=(self.window is None and backend.get('state')=='stopped' and backend.get('running') is False and preserved
                and (receipt is None or self.release_verified(receipt.get('release_receipt'))))
            return dict(ok=ok,unknown=backend.get('state')=='unknown',health=health,
                evidence=dict(backend_exit=backend,release=receipt,**preservation),motion_started=False)
        if result.get('backend_absent') and action in ('stop_model','stop_cameras'):
            stopped=health.get('services',{}).get('model' if action=='stop_model' else 'cameras',{})
            if action=='stop_model'and self.model_external:
                return dict(ok=self.window is None and backend.get('state')=='stopped'and backend.get('running')is False,
                    evidence=dict(shared_model_detachment=result['stopped_model']),pin_release_confirmed=False,
                    process_lifecycle_changed=False,health=health,motion_started=False)
            return dict(ok=self.window is None and backend.get('state')=='stopped' and backend.get('running') is False
                and stopped.get('state')=='stopped' and stopped.get('running') is False and preserved,
                unknown=stopped.get('state')=='unknown',health=health,
                evidence=dict(service_exit=stopped,backend_exit=backend,**preservation),motion_started=False)
        ok=(health.get('backend_reachable') is True and not health.get('maintenance') and self.window is None
            and backend.get('state')=='running' and backend.get('owned') is True and preserved)
        if action=='restart_workbench':
            isolated=result.get('independent_owner')
            same_owner=(all(health.get('runtime',{}).get(key)==value for key,value in isolated.items())
                if isolated else health.get('runtime',{}).get('connected')is False)
            ok=ok and health.get('backend_instance')==result.get('backend') and health.get('backend_instance')!=result['before'].get('backend_instance') and same_owner
        elif result.get('observation_backend_start'):
            started=result['observation_backend_start']
            ok=ok and action in ('start_cameras','restart_cameras') and health.get('runtime',{}).get('connected') is False \
                and health.get('backend_instance')==started['instance'] and stable_identity(backend.get('identity'))==started['identity']
        elif action not in ('prepare_environment','connect_control','recover_control') and (health.get('backend_instance')!=result['before'].get('backend_instance')
                or stable_identity(backend.get('identity'))!=result['before'].get('services',{}).get('workbench')):ok=False
        try:
            if 'model' in result:
                selected=result['model']['prepared'].get('model_id');evidence['model']=self.model_evidence(selected,result['model']['prepared'].get('bundle_digest'))
                ok=ok and (health.get('model') or {}).get('loaded') is True and health['model'].get('id')==selected
            if action=='restart_workbench' and (result.get('model_binding') or {}).get('status')=='restored':
                restored=result['model_binding'];selected=restored['prepared']['model_id'];evidence['model']=self.model_evidence(selected,restored['prepared'].get('bundle_digest'))
                ok=ok and (health.get('model') or {}).get('loaded') is True and health['model'].get('id')==selected
                ok=ok and all(evidence['model'][key]==restored['evidence'][key] for key in ('service_identity','worker_pid'))
        except (OSError,HTTPException,ValueError,KeyError) as error:return dict(ok=False,reason=str(error),health=health,evidence=evidence,motion_started=False)
        runtime=health.get('runtime') or {}
        if action in ('connect_control','recover_control'):
            control=result.get('control') or {}
            ok=ok and self.control_healthy(runtime) and self.control_healthy(control.get('runtime'))
            if action=='connect_control':ok=ok and control.get('connected') is True
            else:ok=ok and control.get('motion_resumed') is False and (control.get('recovered') is True or control.get('already_operable') is True or control.get('reconnected') is True)
            evidence['control']=control
            if result.get('requested_action')=='reclaim_control':
                handoff=(result.get('reclaim')or{}).get('collection')or{}
                from scripts.workbench_deploy import unit_state
                released=unit_state(handoff['unit']) if handoff.get('unit') else {}
                ok=ok and handoff.get('exit_verified')is True and released.get('ActiveState')=='inactive' and released.get('MainPID')=='0'
                evidence['collection_handoff']=handoff
        elif action=='release_control':
            receipt=runtime.get('release_receipt') or (result.get('control') or {}).get('release_receipt') or {}
            ok=ok and runtime.get('connected') is False and self.release_verified(receipt)
            evidence['release']=receipt
        elif action=='verify_hold':
            control=result.get('control') or {}
            ok=ok and hold_verified(control)
            evidence['hold']=control
        elif action=='stop_model':
            stopped=health.get('services',{}).get('model',{})
            if self.model_external:
                binding=(result.get('stopped_model')or{}).get('binding')or{}
                ok=ok and binding.get('invalidated')is True and (binding.get('pin_release')or{}).get('status')in ('released','not_acquired')
                evidence['shared_model_detachment']=result.get('stopped_model')
            else:
                ok=ok and stopped.get('state')=='stopped' and stopped.get('running') is False and (health.get('model') or {}).get('loaded') is False
                evidence['model_exit']=stopped
        elif action=='stop_cameras':
            stopped=result.get('stopped_cameras') or {}
            ok=ok and (_camera_stop_verified(stopped) if self.environment=='fake'
                else health.get('services',{}).get('cameras',{}).get('state')=='stopped' and health['services']['cameras'].get('running') is False)
            evidence['camera_exit']=stopped
        elif action=='prepare_collection' or action=='prepare_environment' and 'collection' in result:
            collection=result.get('collection') or {}
            actual=health.get('collection') or {};steps=collection.get('service_steps') or []
            ok=ok and collection.get('services_ready')is True and actual.get('services_ready')is True
            ok=ok and bool(steps) and {row['role'] for row in steps}==set(actual.get('declared_roles') or [])
            for row in steps:
                proof=(row.get('evidence') or {}).get('service') or {};observed=(actual.get('services') or {}).get(row['role']) or {}
                ok=ok and type(proof.get('pid'))is int and observed.get('status')=='ready' and observed.get('pid')==proof['pid'] and str(observed.get('process_start_ticks'))==str(proof.get('process_start_ticks'))
            ok=ok and collection.get('ready')is actual.get('ready') and collection.get('needs_warmup')is actual.get('needs_warmup')
            evidence['collection']=collection
        elif action in COMPUTE_RESTARTS:
            role=COMPUTE_RESTARTS[action];collection=result.get('collection') or {}
            proof=((collection.get('service_step') or {}).get('evidence') or {}).get('service') or {}
            actual=(health.get('collection',{}).get('services') or {}).get(role) or {}
            ok=ok and collection.get('service_role')==role and collection.get('service_ready')is True and collection.get('ready')is False and collection.get('needs_warmup')is True
            ok=ok and type(proof.get('pid'))is int and actual.get('status')=='ready' and actual.get('pid')==proof['pid'] and str(actual.get('process_start_ticks'))==str(proof.get('process_start_ticks'))
            ok=ok and health.get('collection',{}).get('ready')is False
            evidence['collection_service']=collection
        return dict(ok=bool(ok),unknown=health.get('backend_reachable') is not True,health=health,
                    evidence=evidence,motion_started=action=='verify_hold')

    def stop_current_control(self,request,progress):
        """Shared stop barrier for reclaim and exit; no release or new motion."""
        runtime=self.health_cache.get('runtime')or{}
        stopped=None
        if runtime.get('connected')is True:
            progress('stopping_current_control')
            stopped=self._request_control_stop(request['request_id']+'-stop')
            if stopped.get('status')!='accepted':raise OperationError('stop_unconfirmed','独立Owner未确认接收停止请求',unknown=True)
            barrier=((stopped.get('receipt')or{}).get('stop')or{}).get('barrier_epoch')
            if type(barrier)is not int:raise OperationError('stop_unconfirmed','停止代次未核验，未继续接入',unknown=True)
            state=self.state()
            json_request(self.port,'/api/maintenance/stop',dict(environment=self.environment,
                request_id=secrets.token_hex(16),expected_target=self.backend_target(state)),timeout=self.timeouts['state'],
                headers={'Origin':'http://127.0.0.1:'+str(self.port),'X-CSRF-Token':state['csrf']})
            deadline=time.monotonic()+self.timeouts['release']
            while time.monotonic()<deadline:
                progress('waiting_control_stop');health=self.refresh();current=health.get('runtime')or{}
                if (current.get('runtime_nonce')!=runtime.get('runtime_nonce')
                        or current.get('connected')is not True):
                    raise OperationError('control_changed','停止期间控制实例或连接变化，未继续接入',unknown=True)
                stop_receipt=current.get('stop_receipt')or{}
                if (stop_receipt.get('stop_hold_return_confirmed')is True
                        and type(stop_receipt.get('barrier_epoch'))is int and stop_receipt['barrier_epoch']>=barrier
                        and current.get('feedback_healthy')is True and current.get('stationary')is True
                        and not health.get('busy') and not current.get('automatic_enabled')):break
                time.sleep(.05)
            else:raise OperationError('stop_unconfirmed','停止、静止反馈或任务收尾未确认，未释放或重新接入',unknown=True)
        return stopped

    def home_for_exit(self,request,progress):
        """Use the existing reviewed manual path, native motion gains and protection."""
        keep_jaws=getattr(self,'config',{}).get('exit_home_keep_jaws',False)
        business='calibration'
        expected=self.target();nonce=(self.health_cache.get('runtime')or{})['runtime_nonce'];submitted=False
        def view():
            value=json_request(self.port,'/api/state?environment=physical&business='+business,timeout=self.timeouts['state'])
            r=value['runtime']
            if value.get('server_instance_id')!=expected['backend_instance']or r.get('runtime_nonce')!=nonce:
                raise OperationError('home_identity_changed','回零期间设备/业务身份变化，未释放')
            if not r.get('connected')or not r.get('feedback_healthy')or'FAULT'in str(r.get('state')):
                raise OperationError('home_feedback_failed','回零反馈异常，未释放')
            return value
        def action(name,body):
            s=view();return json_request(self.port,'/api/actions/'+name,dict(environment=self.environment,business=business,
                request_id='exit-home-'+secrets.token_hex(12),**body),timeout=max(10,self.timeouts['state']),
                headers={'Origin':'http://127.0.0.1:'+str(self.port),'X-CSRF-Token':s['csrf']})
        try:
            progress('preparing_exit_home');s=view()
            if s['runtime']['holder']!=business:action('handoff',dict(target_business=business))
            completed=[]
            for side in (('left','right') if keep_jaws else (None,)):
                if side=='right':
                    stopped=self._request_control_stop(request['request_id']+'-between-arms',expected_nonce=nonce)
                    barrier=((stopped.get('receipt')or{}).get('stop')or{}).get('barrier_epoch')
                    if stopped.get('status')!='accepted' or type(barrier)is not int:
                        raise OperationError('home_stop_unconfirmed','两臂之间的停止请求未确认')
                    deadline=time.monotonic()+self.timeouts['operation']
                    while time.monotonic()<deadline:
                        progress('waiting_home_stop');state=view()['runtime'];receipt=state.get('stop_receipt')or{}
                        if (receipt.get('barrier_epoch')==barrier and receipt.get('prior_calls_complete')is True
                                and receipt.get('stop_hold_return_confirmed')is True and state.get('stationary')
                                and state.get('state')=='STOPPED_DAMPING_HOLD'):
                            break
                        time.sleep(.05)
                    else:raise OperationError('home_stop_unconfirmed','两臂之间的停止保持未确认')
                s=view();manual=s['manual']
                if side is None:
                    gaps=[min(hi,max(lo,value))for value,(lo,hi)in zip(manual['jaw_gaps_mm'],manual['jaw_command_ranges_mm'])]
                    body=dict(q12=[0.]*12,speeds12=[.1]*12,jaw_gap_mm=gaps,jaw_speed_mm_s=5.)
                else:
                    target=list(manual['positions']);start=0 if side=='left'else 6
                    target[start:start+6]=[0.]*6
                    body=dict(active_arm=side,q12=target,speeds12=[.1]*12)
                job=action('manual_preview',body)
                deadline=time.monotonic()+self.timeouts['operation'];review=None
                while time.monotonic()<deadline:
                    progress('preparing_exit_home');s=view();candidate=s.get('workflow',{}).get('review');preparation=s.get('workflow',{}).get('request')or{}
                    if candidate and candidate.get('request_job_id')==job['job_id']:review=candidate;break
                    if preparation.get('id')==job['job_id']and preparation.get('status')=='failed':raise OperationError('home_plan_failed',preparation.get('error')or'回零规划失败')
                    time.sleep(.1)
                if review is None:raise OperationError('home_plan_unconfirmed','回零审阅未确认，未执行或释放')
                progress('returning_home');submitted=True
                action('manual_execute',dict(review_id=review['id'],physical_clearance=True))
                deadline=time.monotonic()+self.timeouts['operation']
                while time.monotonic()<deadline:
                    progress('waiting_exit_home');s=view();state=s['runtime'];execution=s.get('manual',{}).get('execution')or{}
                    if execution.get('id')==review['id']:
                        if execution.get('status')in ('failed','rejected','expired'):raise OperationError('home_failed',execution.get('message')or'回零未完成')
                        recording=state.get('recording')
                        if (execution.get('status')=='completed'and state.get('stationary')and not state.get('automatic_enabled')
                                and (not recording or recording.get('done')is True)):
                            completed.append(dict(arm=side,review_id=review['id'],command_epoch=state['command_epoch'],
                                run_id=state.get('execution_instance_id')))
                            break
                    time.sleep(.1)
                else:raise OperationError('home_unconfirmed','回零结束、静止反馈或录制封存未确认，未释放')
            result=dict(completed=True,review_id=review['id'],runtime_nonce=nonce,command_epoch=state['command_epoch'],
                speed_rad_s=.1,final_joint_positions=s['manual']['positions'],static_precision_qualified=False)
            if completed[-1].get('run_id'):result['run_id']=completed[-1]['run_id']
            if keep_jaws:result.update(arms=completed,jaw_policy='retain_native_begin_command; arm-only target plans')
            else:result['jaw_gap_mm']=gaps
            return result
        except BaseException as error:
            if submitted and not(isinstance(error,OperationError)and error.code=='operator_stop'):
                self._request_control_stop(request['request_id']+'-home-failed-stop',expected_nonce=nonce)
            raise

    @staticmethod
    def same_shared_runtime(before,health):
        runtime=health.get('runtime') or {}
        nonce=before.get('runtime_nonce')
        return (isinstance(nonce,str) and bool(nonce) and runtime.get('runtime_nonce')==nonce
                and runtime.get('connected')is True and runtime.get('feedback_healthy')is True)

    def consumer_lifecycle(self,action,request,progress):
        """Only this backend closes its pin and detaches its dedicated control capability."""
        before=self.target();preserved={n:deepcopy(row)for n,row in self.health_cache.get('services',{}).items()if n!='workbench'}
        start=action in ('restart_workbench','connect_control')
        binding=self.warm_model_binding()if start else None
        aborted=(action in ('exit_environment','stop_workbench')
            and (self.health_cache.get('runtime',{}).get('control_owner')or{}).get('consumer_holder')==self.config['control_consumer_id']
            and (self.health_cache.get('runtime',{}).get('inference')or{}).get('runner_status')=='STOPPED')
        stopped=(self.service_action('workbench','stop',progress,abort_stopped=True)if aborted
            else self.service_action('workbench','stop',progress))
        health=self.refresh();owner=(health.get('runtime')or{}).get('control_owner')or{}
        if not self.same_shared_runtime(before,health):
            raise OperationError('shared_runtime_changed','共享SDK身份或健康反馈已变化，切换未完成',unknown=True)
        if not health.get('control_owner_reachable')or owner.get('consumer_holder')==self.config['control_consumer_id']:
            raise OperationError('consumer_detach_unconfirmed','本消费者claim仍保留或状态未知；未释放SDK、未停止共享服务',unknown=True)
        result=dict(action=action,before=before,consumer_lifecycle=True,stopped_backend=stopped,
            preserved_services=preserved,sdk_released=False,collection_started=False)
        if action == 'exit_environment' and self.config.get('consumer_handoff_url'):
            if self.config.get('consumer_handoff_web'):
                from scripts.workbench_deploy import restore_collection_web
                result['collection_web']=restore_collection_web(self.config['consumer_handoff_web'],
                    self.config['device_id'],progress=progress)
            result['next_url'] = self.config['consumer_handoff_url']
        if start:
            started=self.service_action('workbench','start',progress)
            self.wait_backend(progress,started_service=started)
            health=self.refresh();owner=(health.get('runtime')or{}).get('control_owner')or{}
            if owner.get('consumer_holder')!=self.config['control_consumer_id']or owner.get('consumer_attached')is not True:
                raise OperationError('consumer_attach_rejected','未取得本工作台消费者claim；没有接入或接管SDK')
            result['started_backend']=started
            result['model_binding']=self.complete_publication(binding,progress,
                expected_owner={k:health['runtime'].get(k)for k in ('pid','runtime_nonce')})
        return result

    def exit_environment(self,request,progress):
        """Normal home/release, verified resource exit, then restore the collector Web."""
        if self.control_external:return self.consumer_lifecycle('exit_environment',request,progress)
        from scripts.workbench_deploy import verify_unit,http_json,restore_collection_web
        from .control_owner import ControlChannel
        pin=self.config['collection_handoff'];before=self.target();unit=verify_unit({k:pin[k]for k in ('unit','files')})
        runtime=self.health_cache.get('runtime')or{}
        if unit['ActiveState']=='active':
            collection=http_json(pin['status_url'])
            if collection.get('robot_vin')!=self.config['device_id']or(runtime.get('connected')and collection.get('manager_alive')is not False):
                raise OperationError('control_owner_conflict','数采控制归属冲突或未知，未回零或释放')
        result=dict(action='exit_environment',before=before,sdk_was_disconnected=runtime.get('connected')is False,steps=[],collection_started=False)
        report=progress
        def progress(phase,fraction=None,detail=None):
            report(phase,fraction=fraction,detail=dict(exit=deepcopy(result),current=detail))
        if runtime.get('connected'):
            result['stop']=self.stop_current_control(request,progress)
            result['home']=self.home_for_exit(request,progress)
            self.refresh();progress('releasing_control')
            # The reviewed normal-home route authorizes withdrawing hold. This is
            # the legacy release acknowledgement, not a new claim of human support.
            release=self._run('release_control',dict(request,expected_target=self.target(),arms_supported=True),progress)
            proof=self.verify(release)
            if proof.get('ok')is not True:raise OperationError('release_unconfirmed','SDK释放未确认，后续服务保持不动',unknown=True)
            result.update(release_receipt=(proof.get('evidence')or{}).get('release'),release_basis='reviewed_normal_home_completed')
        owner=self.services['control_owner'].snapshot()
        if owner['state']=='running':
            inspected=ControlChannel(self.config['control_owner_descriptor'],timeout=2).call('inspect')
            if inspected.get('connected')is not False or inspected.get('runtime_nonce')is not None:
                raise OperationError('control_not_retired','SDK或Runtime仍存活，未停止Owner或恢复数采',unknown=True)
            result['sdk_was_disconnected']=True if 'release_receipt'not in result else result['sdk_was_disconnected']
        elif owner['state']=='stopped'and owner.get('running')is False:
            result['sdk_was_disconnected']=True
        else:raise OperationError('control_exit_unknown','控制Owner退出状态未知，未继续交接',unknown=True)
        if 'recording' in self.services:
            from .recording_process import recording_call
            recorded=recording_call(os.environ['WORKBENCH_RECORDING_SERVICE'],'inspect',timeout=self.timeouts['state'])
            if not recorded['can_stop']:
                raise OperationError('recording_seal_pending','录制会话尚未封存或writer尚未退出；相机与记录服务保持运行',unknown=True)
        for action,role in (('stop_model','model'),('stop_cameras','cameras')):
            if role not in self.services:continue
            self.refresh();progress('stopping_environment',detail=dict(service=role))
            step=self._run(action,dict(request,expected_target=self.target()),progress);proof=self.verify(step)
            if proof.get('ok')is not True:raise OperationError('resource_exit_unconfirmed',role+'退出未确认，未继续交接',unknown=True)
            result['steps'].append(dict(service=role,verification=proof))
        if 'triton_control'in self.services:
            if self.services['triton_control'].snapshot()['state']=='running'and self.triton_preview.catalog().get('busy')is not False:
                raise OperationError('triton_busy','转换工具仍有任务，未停止或恢复数采网页')
            progress('stopping_environment',detail=dict(service='triton_control'))
            result['steps'].append(dict(service='triton_control',result=self.service_action('triton_control','stop',progress)))
        for role in tuple(name for name in ('pedal','recording','workbench','control_owner')if name in self.services):
            progress('stopping_environment',detail=dict(service=role))
            result['steps'].append(dict(service=role,result=self.service_action(role,'stop',progress)))
        states={name:service.snapshot()for name,service in self.services.items()}
        if any(states[name].get('state')!='stopped'or states[name].get('running')is not False for name in self.managed_service_names()):
            raise OperationError('environment_exit_unconfirmed','工作台仍有未确认退出的进程，未恢复数采',unknown=True)
        result['collection_web']=restore_collection_web(pin,self.config['device_id'],progress=progress,timeout=self.timeouts['backend'])
        result['resources']=states;result['environment_exited']=True
        if self.config.get('collection_handoff_url'):result['next_url']=self.config['collection_handoff_url']
        return result

    def reclaim_control(self,request,progress):
        """Compose the existing stop, collection shutdown and recovery paths."""
        from scripts.workbench_deploy import handoff_collection,http_json,verify_unit
        pin=self.config['collection_handoff'];initial=self.target()
        runtime=self.health_cache.get('runtime')or{}
        unit=verify_unit({k:pin[k]for k in ('unit','files')})
        if unit['ActiveState']=='active' and runtime.get('connected')is True:
            collection=http_json(pin['status_url'])
            if collection.get('robot_vin')!=self.config['device_id'] or collection.get('manager_alive')is not False:
                raise OperationError('control_owner_conflict','数采和工作台控制归属冲突或未知，未接入第二个SDK')
        stopped=self.stop_current_control(request,progress)
        progress('checking_collection_handoff')
        handed=handoff_collection(pin,self.config['device_id'],stop_active=True,progress=progress,
                                 timeout=min(240.,self.timeouts['operation']))
        self.refresh()
        if self.target()['manager_instance']!=initial['manager_instance'] or self.target()['deployment_sha256']!=initial['deployment_sha256']:
            raise OperationError('target_changed','部署身份在交接期间变化，未接入SDK')
        if any(self.target().get('services',{}).get(name)!=initial.get('services',{}).get(name)
                for name in ('control_owner','workbench')):
            raise OperationError('target_changed','控制或业务服务在交接期间被其他实例替换，未接入SDK')
        if self.window is not None or self.unresolved():
            progress('verifying_old_maintenance')
            self._run('recover_maintenance',dict(request,expected_target=self.target()),progress)
            self.refresh()
        owner=self.services['control_owner'].snapshot()
        if owner.get('state')=='stopped':
            self.service_action('control_owner','start',progress)
            from .control_owner import ControlChannel
            deadline=time.monotonic()+self.timeouts['state']*5
            while time.monotonic()<deadline:
                progress('waiting_control_owner')
                try:
                    inspected=ControlChannel(self.config['control_owner_descriptor'],timeout=1.).call('inspect')
                    if inspected.get('connected')is False:break
                except (OSError,ValueError,ConnectionError):pass
                time.sleep(.1)
            else:raise OperationError('owner_start_unconfirmed','控制Owner启动未核验，未接入SDK',unknown=True)
            self.refresh()
        current=self.health_cache.get('runtime')or{}
        action='connect_control' if current.get('state')=='DISCONNECTED' else 'recover_control'
        progress('recovering_workbench_control')
        result=self._run(action,dict(request,expected_target=self.target()),progress)
        result['requested_action']='reclaim_control'
        result['reclaim']=dict(before=initial,stop=stopped,collection=handed,old_motion_resumed=False)
        return result

    def stop(self,body):
        if (not isinstance(body,dict) or set(body)!={'expected_device_id','request_id'}
                or body['expected_device_id']!=self.config['device_id'] or not isinstance(body['request_id'],str)
                or not re.fullmatch(r'[A-Za-z0-9_-]{1,128}',body['request_id'])):
            raise ValueError('停止请求需要当前设备身份和请求编号')
        self.stop_generation+=1
        return self._request_control_stop(body['request_id'])

    def _request_control_stop(self,request_id,*,expected_nonce=None):
        if self.config.get('control_owner_descriptor'):
            try:
                from .control_owner import ControlChannel
                result=ControlChannel(self.config['control_owner_descriptor'],timeout=2.).call('stop',**({}if expected_nonce is None else dict(expected_nonce=expected_nonce)))
                return dict(status='accepted',receipt=result,message='停止已直达独立Owner；实际停止与保持以设备回执为准')
            except (OSError,ValueError,ConnectionError,RuntimeError)as error:
                return dict(status='unknown',stopped=False,message='独立Owner停止结果尚未核验：'+str(error))
        auth=self.backend_auth
        if auth is None:
            try:self.state();auth=self.backend_auth
            except (OSError,HTTPException,ValueError):pass
        if auth is None:return dict(status='unknown',stopped=False,message='设备状态未知，无法确认软件停止；请使用现场停止入口')
        try:
            result=json_request(self.port,'/api/maintenance/stop',dict(environment=self.environment,
                request_id=request_id,expected_target={key:auth[key] for key in ('server_instance_id','device_id')}),
                timeout=2,headers={'Origin':'http://127.0.0.1:'+str(self.port),'X-CSRF-Token':auth['csrf']})
            return dict(status='accepted',receipt=result,message='停止请求已提交，实际停止与保持以设备回执为准')
        except (OSError,HTTPException,ValueError) as error:
            return dict(status='unknown',stopped=False,message='软件停止结果尚未核验：'+str(error))

    def environment_snapshot(self,*,summary=False):
        view=describe_environment(self.config,self.snapshot(summary=True),self.profiles)
        view['projection']='manager_environment_summary' if summary else 'manager_environment'
        if summary:
            # Aliases are restored by the common browser client, without retransmitting them.
            for key in ('runtime','services','model','collection','resources','activity','maintenance'):
                if key in view['health']:view['health'].pop(key)
            view['health'].pop('control_timing',None)
            for capability in view['capabilities'].values():capability.pop('identity',None)
            runtime=view['runtime']
            view['runtime']={key:runtime[key] for key in ('pid','state','reason','connected','holder',
                'runtime_nonce','command_epoch','calibration_revision','calibration_snapshot_id','hold',
                'feedback_healthy','stationary','sample_age_s','automatic_enabled','recoverable_fault',
                'initialization_error','service_error','release_receipt','maintenance_active',
                'control_publication','control_coherence') if key in runtime}
            jobs={}
            def reference(job):
                if job is None:return None
                jobs[job['job_id']]=job
                return job['job_id']
            view['active_job']=reference(view['active_job'])
            view['recent_jobs']=[reference(job) for job in view['recent_jobs']]
            for row in view['operations'].values():row['latest_job']=reference(row['latest_job'])
            view['jobs']=jobs
            view['detail_url']='/api/operations'
        return view

    def probe_target(self,body):
        if (not isinstance(body,dict) or set(body)-{'profile_id','robot_address','compute_address','expected_target'}
                or body.get('expected_target')!=self.target()):raise ValueError('当前环境身份已变化，请刷新再检查地址')
        candidates=self.profiles
        for field in ('profile_id','robot_address','compute_address'):
            value=body.get(field)
            if value:
                if not isinstance(value,str):raise ValueError('接入字段必须是文本')
                candidates=[row for row in candidates if row.get('id' if field=='profile_id' else field)==value]
        if len(candidates)!=1:raise ValueError('地址未匹配唯一已登记档案；请检查IP并选择对应实例，不会猜测端口或部署未知设备')
        profile=candidates[0]
        if profile['manager_url']=='/':return self.environment_snapshot()
        if self.health_cache.get('runtime',{}).get('connected') or self.health_cache.get('busy'):
            raise ValueError('请先停止任务并在运维释放当前设备，再切换接入目标')
        parsed=urlsplit(profile['manager_url'])
        connection=(HTTPSConnection if parsed.scheme=='https' else HTTPConnection)(parsed.hostname,parsed.port,timeout=3)
        try:
            connection.request('GET','/api/environment?environment='+profile['environment'])
            response=connection.getresponse();raw=response.read(LIMIT+1)
            if response.status!=200 or len(raw)>LIMIT:raise ValueError('目标尚未提供兼容运维服务，请先完成部署')
            target=json.loads(raw)
            if (target.get('schema')!='robot_workbench.environment.v1'
                    or target.get('target',{}).get('device_id')!=profile['device_id']
                    or target['target'].get('environment')!=profile['environment']):raise ValueError('目标身份或接口不匹配，不接入控制')
            return dict(redirect_url=profile['manager_url'],environment=target,controller_acquired=False)
        finally:connection.close()

    def snapshot(self,*,summary=False):
        with self.health_lock:health=deepcopy(self.health_cache)
        profile=health.pop('machine_profile',{});deployment=health.pop('deployment',self.deployment.snapshot())
        age=(time.time_ns()-health.get('checked_wall_ns',0))/1e9
        health['freshness']=dict(valid=0<=age<=10.,age_s=age,maximum_age_s=10.,
            source='management_health_observation',sampled_wall_ns=health.get('checked_wall_ns'))
        if not health['freshness']['valid']:
            health['backend_reachable']=None;health['error']='领域事实缓存已过期，当前状态未知；独立停止仍可使用'
        available=health.get('backend_reachable') is True
        capabilities=dict(control_connected=health.get('runtime',{}).get('connected') if available else None,
            model_loaded=(health.get('model_service') or {}).get('loaded'),
            workflow_model_bound=(health.get('model') or {}).get('loaded') if available else None,
            inference_control_ready=health.get('inference_control_ready') if available else None,
            camera_frames=(all(row.get('ready') is True for row in health['cameras'])
                           if available and len(health.get('cameras',[]))==3 else None))
        return dict(csrf=self.csrf,health=health,machine_profile=profile,deployment=deployment,
            control_alerts=self.control_alerts.snapshot(health.get('runtime',{}).get('control_protection'),
                reachable=available,connected=health.get('runtime',{}).get('connected')),
            current_capabilities=capabilities,
            recovery=dict(required=self.window is not None or bool(self.unresolved()),
            unresolved_job_ids=self.unresolved()),**self.operations.snapshot(summary=summary))

    def require_deployment(self,headers):
        values=headers.get_all('X-Workbench-Deployment',[])
        if values!=[self.deployment.sha256]:
            raise OperationError('deployment_mismatch','页面与管理器的部署版本不一致，请刷新页面；诊断和安全停止仍可使用')
        if not self.deployment.snapshot(check_files=True,force=True)['files_unchanged']:
            raise OperationError('deployment_changed','部署源文件已变化，请重新启动管理器并刷新页面；诊断和安全停止仍可使用')

    def close(self):
        self.closed.set()
        display=getattr(self,'target_display',None)
        if display is not None:display.close()
        if self.thread is not None:self.thread.join(3)
        self.operations.close()


def serve(manager,*,host,port,allowed_hosts=()):
    """Same-origin UI proxy; operations and status survive backend death."""
    assets=ROOT/'web'
    class Handler(BaseHTTPRequestHandler):
        protocol_version='HTTP/1.1';timeout=5;disable_nagle_algorithm=True
        def log_message(self,*args):pass
        def handle_expect_100(self):
            self.send(417,{'error':'Expect is not supported'});return False
        def send(self,status,raw,kind='application/json',headers=None):
            if isinstance(raw,(dict,list)):raw=json.dumps(raw,allow_nan=False).encode()
            self.send_response(status);self.send_header('Content-Type',kind);self.send_header('Content-Length',str(len(raw)))
            self.send_header('X-Content-Type-Options','nosniff')
            self.send_header('X-Workbench-Deployment',manager.deployment.sha256)
            for name,value in (headers or{}).items():self.send_header(name,value)
            self.send_header('Cache-Control','no-store');self.send_header('Connection','close');self.end_headers();self.close_connection=True
            try:
                if self.command!='HEAD':self.wfile.write(raw)
            except (BrokenPipeError,ConnectionResetError):pass
        def route(self):
            self.close_connection=True
            authority=self.headers.get('Host','');origin=self.headers.get('Origin')
            if (self.headers.get_all('Host',[])!=[authority] or authority not in self.server.allowed_hosts
                    or len(self.headers.get_all('Origin',[]))>1 or origin not in (None,'http://'+authority)):
                self.send(403,{'error':'invalid Host or Origin'});return
            raw_target=self.raw_requestline.split()[1].decode('iso-8859-1')
            try:parsed=urlsplit(raw_target)
            except ValueError:return self.send(400,{'error':'invalid request target'})
            path=parsed.path
            if not raw_target.startswith('/') or raw_target.startswith('//') or parsed.netloc or parsed.scheme or parsed.fragment or '\\' in raw_target:
                self.send(400,{'error':'invalid request target'});return
            if self.headers.get_all('Transfer-Encoding',[]) or self.headers.get_all('Expect',[]):
                self.send(400,{'error':'unsupported request framing'});return
            body=None
            if self.command=='POST':
                lengths=self.headers.get_all('Content-Length',[])
                if len(lengths)!=1 or not lengths[0].isascii() or not lengths[0].isdigit() or not 0<int(lengths[0])<=LIMIT:
                    self.send(400,{'error':'bounded Content-Length required'});return
                try:body=self.rfile.read(int(lengths[0]))
                except OSError:return self.send(408,{'error':'request body timed out'})
                if len(body)!=int(lengths[0]):return self.send(400,{'error':'incomplete request body'})
            elif self.headers.get_all('Content-Length',[]) not in ([],['0']):
                return self.send(400,{'error':'GET/HEAD body is not supported'})
            try:
                if self.command in ('GET','HEAD') and path in ('/','/index.html','/operations'):
                    query=parse_qs(parsed.query,keep_blank_values=True)
                    if 'environment' not in query:
                        query['environment']=[manager.environment]
                        return self.send(303,b'',headers={'Location':path+'?'+urlencode(query,doseq=True)})
                if path.startswith('/api/triton/'):
                    query=parse_qs(parsed.query,keep_blank_values=True)
                    if self.command in ('GET','HEAD'):
                        if query!={'environment':[manager.environment]}:raise ValueError('明确的当前设备环境必需')
                        if manager.triton_preview is None:
                            return self.send(200,dict(configured=False,available=False,sources=[],models=[],reason='当前设备未配置Triton服务'))
                        if path=='/api/triton/catalog':return self.send(200,manager.triton_preview.catalog())
                        if path.startswith('/api/triton/jobs/'):return self.send(200,manager.triton_preview.job(path.rsplit('/',1)[-1]))
                        return self.send(404,{'error':'unknown Triton endpoint'})
                    if self.command!='POST' or query:return self.send(404,{'error':'unknown Triton endpoint'})
                    if self.headers.get_all('X-CSRF-Token',[])!=[manager.csrf]:return self.send(403,{'error':'invalid Triton authorization'})
                    manager.require_deployment(self.headers)
                    if manager.triton_preview is None:raise ValueError('当前设备未配置Triton输出预览')
                    value=json.loads(body)
                    if not isinstance(value,dict) or value.get('environment')!=manager.environment or value.get('business')!='inference':raise ValueError('Triton操作仅允许当前设备的推理输出预览')
                    health=manager.refresh()
                    if (manager.operations.active is not None or not health.get('backend_reachable') or health.get('busy') or health.get('maintenance')
                            or (health.get('runtime') or {}).get('automatic_enabled')):
                        raise ValueError('设备忙或状态未核验，暂不启动Triton任务')
                    result=manager.triton_preview.start(path.removeprefix('/api/triton/'),{k:v for k,v in value.items()if k not in('environment','business')})
                    return self.send(202,result)
                if path=='/api/deployment' and self.command in ('GET','HEAD'):
                    return self.send(200,manager.deployment.snapshot(check_files=True))
                if path=='/api/environment' and self.command in ('GET','HEAD'):
                    query=parse_qs(parsed.query,keep_blank_values=True)
                    if (set(query)-{'environment','view'} or query.get('environment',[manager.environment])!=[manager.environment]
                            or query.get('view',['full']) not in (['full'],['summary'])):
                        raise ValueError('当前入口没有管理所选环境')
                    return self.send(200,manager.environment_snapshot(summary=query.get('view')==['summary']))
                if path=='/api/session' and self.command in ('GET','HEAD'):
                    snapshot=manager.snapshot();running=snapshot['health'].get('backend_reachable') is True
                    return self.send(200,dict(schema='fake-session.v1',managed=True,manager_instance_id=manager.instance,
                        csrf=manager.csrf,generation=0,profile='full_fake',status='ready' if running else 'stopped',
                        persistent_service=True,deadline_wall_ns=None,message='旧入口只读；启停恢复统一由服务运维执行',
                        allowed={},current_run=snapshot['target'].get('backend_instance')))
                if path in ('/api/session/start','/api/session/stop') and self.command=='POST':
                    return self.send(409,dict(code='operation_required',error='旧Session入口只读；请从统一运维启动或停止工作台'))
                if path.startswith('/api/operations'):
                    query=parse_qs(parsed.query,keep_blank_values=True)
                    if set(query)-{'environment'}:raise ValueError('unknown operation query')
                    env=query.get('environment',[manager.environment])
                    if env!=[manager.environment]:raise ValueError('当前入口没有管理所选环境')
                    if self.command in ('GET','HEAD') and path=='/api/operations':return self.send(200,manager.snapshot())
                    if self.command in ('GET','HEAD') and re.fullmatch('/api/operations/jobs/[a-f0-9]{32}',path):
                        with manager.operations.lock:
                            job=next((deepcopy(row) for row in manager.operations.jobs.values() if row['job_id']==path.rsplit('/',1)[-1]),None)
                        return self.send(200,job) if job is not None else self.send(404,{'error':'操作回执不存在'})
                    tokens=self.headers.get_all('X-CSRF-Token',[])
                    if self.command!='POST' or len(tokens)!=1 or not secrets.compare_digest(tokens[0],manager.csrf):
                        return self.send(403,{'error':'invalid operation authorization'})
                    if self.headers.get('Content-Type','').split(';')[0].strip().lower()!='application/json':raise ValueError('JSON operation body required')
                    value=json.loads(body)
                    if not isinstance(value,dict):raise ValueError('operation object required')
                    if path=='/api/operations/stop':return self.send(200,manager.stop(value))
                    if path!='/api/operations/action' or value.get('action')!='status_check':manager.require_deployment(self.headers)
                    if path=='/api/operations/target':return self.send(200,manager.probe_target(value))
                    if path=='/api/operations/acknowledge-control-warning':
                        if (set(value)!={'event_id','environment','device_id'} or value['environment']!=manager.environment
                                or value['device_id']!=manager.config['device_id']):raise ValueError('告警所属设备或环境不匹配')
                        return self.send(200,manager.control_alerts.acknowledge(value['event_id']))
                    if path=='/api/operations/action':return self.send(200,manager.operations.start(**value))
                    if path=='/api/operations/cancel':return self.send(200,manager.operations.cancel(**value))
                    return self.send(404,{'error':'unknown operation endpoint'})
                if self.command in ('GET','HEAD') and (path in ('/','/index.html','/operations','/session') or path.startswith('/web/')):
                    name='operations.html' if path=='/operations' else 'session.html' if path=='/session' else 'index.html' if path in ('/','/index.html') else path.removeprefix('/web/')
                    file=assets/name
                    if ('/' in name and name not in ('vendor/three.module.js','vendor/OrbitControls.js') or not file.is_file() or file.is_symlink() or file.stat().st_size>LIMIT
                            or file.suffix not in ('.html','.css','.js','.mjs','.svg','.png','.jpg','.ico')):
                        return self.send(404,{'error':'unknown static asset'})
                    mime=mimetypes.guess_type(name)[0] or 'application/octet-stream'
                    raw=file.read_bytes()
                    if file.suffix in ('.js','.mjs'):raw=raw.replace(b'__WORKBENCH_DEPLOYMENT_SHA__',manager.deployment.sha256.encode())
                    return self.send(200,raw,mime+('; charset=utf-8' if mime.startswith('text/') or mime.endswith('javascript') else ''))
                if self.command=='POST' and path!='/api/actions/stop':manager.require_deployment(self.headers)
                from .target_display_gateway import route as display_route
                if display_route(manager,self,parsed,body):return
                connection=HTTPConnection('127.0.0.1',manager.port,timeout=10)
                response_started=False
                try:
                    excluded=HOP_HEADERS|{'host','origin','content-length'}|{k.strip().lower() for k in self.headers.get('Connection','').split(',')}
                    headers={k:v for k,v in self.headers.items() if k.lower() not in excluded}
                    if origin:headers['Origin']='http://127.0.0.1:'+str(manager.port)
                    headers['Connection']='close'
                    connection.request(self.command,self.path,body,headers)
                    response=connection.getresponse()
                    excluded=HOP_HEADERS|{k.strip().lower() for k in response.getheader('Connection','').split(',')}
                    self.send_response(response.status)
                    for key,value in response.getheaders():
                        if key.lower() not in excluded:self.send_header(key,value)
                    self.send_header('Connection','close');self.end_headers();response_started=True
                    if self.command!='HEAD':
                        while True:
                            raw=response.read(64*1024)
                            if not raw:break
                            self.wfile.write(raw)
                    return
                except (OSError,HTTPException):
                    if not response_started:raise
                    return  # A partial stream is closed, never replaced by a second response.
                finally:connection.close()
            except (OSError,HTTPException) as error:self.send(503,{'error':'工作台后端暂不可达，请使用环境与服务面板核验或恢复','detail':str(error)})
            except (ValueError,TypeError,KeyError) as error:self.send(409,{'error':str(error),'code':getattr(error,'code','operation_rejected')})
        do_GET=route
        do_HEAD=route
        do_POST=route
    server=ThreadingHTTPServer((host,port),Handler);server.daemon_threads=True
    server.allowed_hosts={f'{host}:{server.server_port}',*allowed_hosts}
    if host=='127.0.0.1':server.allowed_hosts.add(f'localhost:{server.server_port}')
    return server
