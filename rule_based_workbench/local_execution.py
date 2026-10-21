"""Frozen single-host compute: 734 GPU1 or 739 GPU0 for either device environment."""
import hashlib
from http.client import HTTPConnection
import fcntl
import json
import os
from pathlib import Path
import re
import stat
import subprocess
import xml.etree.ElementTree as ET

TARGET=dict(transport='local',instance_id='dsw-3lduujsvlem1a91wdl',gpu_index=1,
            gpu_uuid='GPU-ffc2b1a7-9e85-ec83-d289-f7d1b45ccd64',vulkan_gpu=1)
PHYSICAL_TARGET=dict(transport='local',instance_id='dsw-nygxkzfd5znx4if0t2',gpu_index=0,
            gpu_uuid='GPU-195270a3-30b8-39b1-1ac4-4c9e1aebf228',vulkan_gpu=0)


def compute_resource(execution):return {key:execution.get(key) for key in TARGET}


def contract_compute_resource(contract):
    """Choose only an explicitly declared, already approved host/GPU identity."""
    identity=contract.get('identity',{});resources=contract.get('resources',{})
    instance=identity.get('dsw_instance_id',identity.get('instance'))
    if 'instance' in identity and identity['instance']!=instance:
        raise ValueError('compute instance identity fields disagree')
    for target in (TARGET,PHYSICAL_TARGET):
        if (instance==target['instance_id']
                and type(resources.get('gpu_index')) is int and resources['gpu_index']==target['gpu_index']
                and resources.get('cuda_visible_devices')==[target['gpu_index']]
                and all(type(index) is int for index in resources['cuda_visible_devices'])
                and resources.get('gpu_uuid')==target['gpu_uuid']):return dict(target)
    raise ValueError('explicit approved instance, GPU index, CUDA mask and UUID must select 734 GPU1 or 739 GPU0')


def validate_shared_model(spec):
    fields={'service_receipt','service_spec_sha256','service_script','service_script_sha256','service_argv_sha256',
            'status_port','worker_script','worker_script_sha256','model_id','checkpoint_sha256','max_memory_mib'}
    if not isinstance(spec,dict) or set(spec)!=fields:raise ValueError('explicit frozen shared-model ownership fields required')
    if (spec['model_id']!='base566-epoch007-step58982'
            or spec['checkpoint_sha256']!='c1bbd7c86ab478217ca985c41f4aef1a4e1752813c7051519d63c8e384cb8e80'):
        raise ValueError('shared GPU model admission currently supports only the explicitly approved Base566 epoch7 checkpoint')
    if any(not isinstance(spec[key],str) or not Path(spec[key]).is_absolute() or '..' in Path(spec[key]).parts
           for key in ('service_receipt','service_script','worker_script')):
        raise ValueError('absolute frozen shared-model receipt and script paths required')
    if any(not isinstance(spec[key],str) or not re.fullmatch('[0-9a-f]{64}',spec[key])
           for key in fields if key.endswith('_sha256')):
        raise ValueError('shared-model code, deployment and argv SHA256 pins required')
    if (type(spec['status_port']) is not int or not 1024<=spec['status_port']<=65535
            or type(spec['max_memory_mib']) is not int or spec['max_memory_mib']<=0):
        raise ValueError('bounded loopback model port and explicit memory budget required')
    return spec


def owner_spec(execution):
    """The declared compute coordinator; a fake plant is its legacy adapter."""
    if ('owner' in execution)==('plant' in execution):
        raise ValueError('exactly one explicit compute owner or fake plant is required')
    return execution['owner'] if 'owner' in execution else execution['plant']


def owner_scope(execution):return owner_spec(execution)['scope']


def validate(config, contract=None):
    value=config.get('execution')
    if value is None:
        return None
    environment=value.get('environment','fake') if isinstance(value,dict) else None
    physical=environment=='physical'
    if environment not in ('fake','physical') or config.get('environment',environment)!=environment:
        raise ValueError('compute execution environment differs from device inputs')
    fields={*TARGET,'session_id','owner' if physical else 'plant','max_processes','min_free_memory_mib'}
    if isinstance(value,dict) and 'environment' in value:fields.add('environment')
    if isinstance(value,dict) and value.get('max_processes') in (5,6):fields.add('roles')
    if isinstance(value,dict) and 'shared_model' in value:
        fields.add('shared_model');validate_shared_model(value['shared_model'])
    if (not isinstance(value,dict) or set(value)!=fields
            or compute_resource(value) not in (TARGET,PHYSICAL_TARGET)
            or any(type(value[k]) is not int for k in ('gpu_index','vulkan_gpu'))):
        raise ValueError('execution must freeze exactly 734 GPU1/CUDA1/Vulkan1 or 739 GPU0/CUDA0/Vulkan0')
    if not isinstance(value['session_id'],str) or not value['session_id']:
        raise ValueError('local execution requires its owning Session')
    if (type(value['max_processes']) is not int or value['max_processes'] not in (4,5,6)
            or type(value['min_free_memory_mib']) is not int or value['min_free_memory_mib']<=0):
        raise ValueError('explicit bounded local process/memory budgets required')
    plant=owner_spec(value)
    required={'session_id','pid','process_start_ticks','scope','script','script_sha256','argv','max_memory_mib'}
    if physical:required.add('gpu_context_required')
    if (not isinstance(plant,dict) or set(plant)!=required or plant['session_id']!=value['session_id']
            or type(plant['pid']) is not int or plant['pid']<=0
            or not isinstance(plant['process_start_ticks'],str) or not plant['process_start_ticks'].isdigit()
            or type(plant['max_memory_mib']) is not int or plant['max_memory_mib']<=0
            or not isinstance(plant['argv'],list) or not plant['argv'] or any(not isinstance(v,str) for v in plant['argv'])
            or not isinstance(plant['script_sha256'],str) or not re.fullmatch('[0-9a-f]{64}',plant['script_sha256'])):
        raise ValueError('current Session compute owner PID/start/scope/code/budget evidence required')
    if physical and type(plant['gpu_context_required']) is not bool:
        raise ValueError('physical compute owner must explicitly declare whether it owns a GPU context')
    for key in ('scope','script'):
        path=Path(plant[key])
        if not path.is_absolute() or '..' in path.parts:
            raise ValueError('absolute compute owner paths required')
    if ((Path(plant['script']).name=='serve_physx_device.py' if physical else Path(plant['script']).name!='serve_physx_device.py') or plant['script'] not in plant['argv']
            or not any(plant['argv'][i:i+2]==['--output',plant['scope']] for i in range(len(plant['argv'])-1))):
        raise ValueError('compute owner command must bind its exact owned output scope and device environment')
    if not physical and (plant['argv'].count('--gpu')!=1 or
            plant['argv'][plant['argv'].index('--gpu')+1:plant['argv'].index('--gpu')+2]!=[str(value['gpu_index'])]):
        raise ValueError('fake plant command must bind the declared physical GPU index')
    if value['max_processes'] in (5,6):
        roles=value.get('roles')
        expected={'preview','auxiliary'}|({'scene'} if value['max_processes']==6 else set())
        if not isinstance(roles,dict) or set(roles)!=expected:
            raise ValueError('execution requires exactly the declared preview/auxiliary/scene registration slots')
        for role,spec in roles.items():
            required={'root','registration','max_memory_mib'}|({'script','script_sha256','config_path','config_sha256','token_file','port'}
                if role=='preview' else {'entrypoints'})
            if role=='preview' and isinstance(spec,dict) and 'persistent_service' in spec:
                required.add('persistent_service')
                if spec['persistent_service'] is not True or not physical and '--persistent' not in plant['argv']:
                    raise ValueError('persistent preview requires the explicit persistent plant')
            if not isinstance(spec,dict) or set(spec)!=required or type(spec['max_memory_mib']) is not int or spec['max_memory_mib']<=0:
                raise ValueError('explicit '+role+' scope/code/config/memory contract required')
            for name in ('root','registration'):
                path=Path(spec[name])
                if not path.is_absolute() or '..' in path.parts or not path.is_relative_to(plant['scope']):
                    raise ValueError('GPU role registration escaped the resident scope')
            if role=='preview':
                if type(spec['port']) is not int or not 0<spec['port']<65536:raise ValueError('frozen preview port required')
                pins={spec['script']:spec['script_sha256'],spec['config_path']:spec['config_sha256']}
                if not Path(spec['token_file']).is_relative_to(spec['root']):raise ValueError('private preview token scope required')
            else:pins=spec['entrypoints']
            if not isinstance(pins,dict) or not pins or any(not Path(p).is_absolute() or not isinstance(h,str) or not re.fullmatch('[0-9a-f]{64}',h) for p,h in pins.items()):
                raise ValueError('GPU role entrypoints/config require absolute SHA pins')
    if contract is not None:
        if (contract.get('execution')!=value or contract.get('identity',{}).get('session_id')!=value['session_id']
                or contract['resources'].get('max_workers_per_instance')!=value['max_processes']):
            raise ValueError('local execution differs from frozen Session/resource contract')
        from run_pipeline import below,pinned_files
        fs=contract['filesystem'];pins={k:v for group in pinned_files(contract) for k,v in group.items()}
        if (not any(below(plant['scope'],p) for p in fs['writable_roots'])
                or pins.get(plant['script'])!=plant['script_sha256']):
            raise ValueError('compute owner scope/code must be declared and pinned in the same contract')
    return value


def role_spec_digest(spec):return hashlib.sha256(json.dumps(spec,sort_keys=True,separators=(',',':')).encode()).hexdigest()


def role_argv(execution,role,argv,entrypoint):
    """Check the actual Python command against the frozen role, not a PID label."""
    spec=execution['roles'][role]
    if entrypoint not in argv:raise ValueError('GPU role entrypoint absent from actual argv')
    tail=argv[argv.index(entrypoint)+1:]
    def argument(name):
        if tail.count(name)!=1 or tail.index(name)+1>=len(tail):raise ValueError('GPU role argument missing/duplicated: '+name)
        return tail[tail.index(name)+1]
    output=Path(argument('--output'))
    if not output.is_absolute() or '..' in output.parts or output==Path(spec['root']) or not output.is_relative_to(spec['root']):
        raise ValueError('GPU role output escaped its private scope')
    if json.loads(argument('--execution-json'))!=execution:raise ValueError('GPU role execution contract differs')
    if role=='preview':
        expected={'--config':spec['config_path'],'--token-file':spec['token_file'],'--port':str(spec['port']),'--render-gpu':str(execution['vulkan_gpu'])}
        if entrypoint!=spec['script'] or any(argument(k)!=v for k,v in expected.items()) or '--display-only' not in tail:
            raise ValueError('preview code/config/port/display-only binding differs')
        duration=float(argument('--duration-s'))
        if (spec.get('persistent_service') is True and duration!=0) or (not spec.get('persistent_service') and not 0<duration<=14400):
            raise ValueError('preview lifetime must match its explicit persistent or bounded role')
    else:
        if entrypoint not in spec['entrypoints']:raise ValueError('unregistered auxiliary GPU entrypoint')
        if '--gpu' in tail and argument('--gpu')!=str(execution['vulkan_gpu']):raise ValueError('auxiliary Vulkan GPU differs')


_role_locks=[]
def register_role(execution,role,entrypoint,*,instance_id=None,proc_root='/proc'):
    """Called before native GPU imports; owns one fixed slot until process exit."""
    validate({'execution':execution})
    if execution['max_processes']==4:return lambda status:None
    spec=execution['roles'][role];entrypoint=str(Path(entrypoint).resolve());pid=os.getpid();proc=Path(proc_root)/str(pid)
    argv=[v.decode() for v in (proc/'cmdline').read_bytes().split(b'\0') if v]
    role_argv(execution,role,argv,entrypoint)
    sha=hashlib.sha256(Path(entrypoint).read_bytes()).hexdigest()
    expected=spec['script_sha256'] if role=='preview' else spec['entrypoints'][entrypoint]
    if sha!=expected:raise ValueError('GPU role code changed after contract freeze')
    if role=='preview' and hashlib.sha256(Path(spec['config_path']).read_bytes()).hexdigest()!=spec['config_sha256']:
        raise ValueError('preview config changed after contract freeze')
    if os.environ.get('CUDA_VISIBLE_DEVICES')!=str(execution['gpu_index']) or os.environ.get('ROBOT_WORKBENCH_SESSION_ID')!=execution['session_id']:
        raise ValueError('GPU role Session/CUDA mask differs')
    path=Path(spec['registration']);path.parent.mkdir(parents=True,exist_ok=True,mode=0o700)
    fd=os.open(str(path)+'.lock',os.O_RDWR|os.O_CREAT|os.O_NOFOLLOW,0o600)
    try:fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BaseException:os.close(fd);raise
    _role_locks.append(fd)
    state=dict(role=role,session_id=execution['session_id'],spec_sha256=role_spec_digest(spec),pid=pid,
        process_start_ticks=(proc/'stat').read_text().rsplit(')',1)[1].split()[19],argv=argv,
        entrypoint=entrypoint,entrypoint_sha256=sha,renderer_instance_id=instance_id,
        config_sha256=spec.get('config_sha256'))
    def update(status):
        state['status']=status
        temporary=path.with_suffix('.partial')
        with temporary.open('w') as stream:json.dump(state,stream,allow_nan=False)
        temporary.chmod(0o600);temporary.replace(path)
    update('starting');return update


def register_auxiliary(args,entrypoint):
    if Path(entrypoint).name=='playground_geometry.py' and (getattr(args,'backend','native')=='native' or getattr(args,'initial',None)):
        return
    if getattr(args,'execution_json',None):
        execution=json.loads(args.execution_json)
        if execution.get('max_processes') in (5,6):register_role(execution,'auxiliary',entrypoint)


def register_scene(args,entrypoint):
    execution=json.loads(args.execution_json)
    if execution.get('max_processes')!=6:raise ValueError('scene exporter requires its dedicated sixth role')
    return register_role(execution,'scene',entrypoint)


def _process(pid, start, *, gpu, proc_root, expected_argv=None, required=(), session=None,expected_env=None):
    proc=Path(proc_root)/str(pid)
    if (proc/'stat').read_text().rsplit(')',1)[1].split()[19]!=str(start):
        raise ValueError('GPU process PID/start time changed')
    argv=[v.decode() for v in (proc/'cmdline').read_bytes().split(b'\0') if v]
    if expected_argv is not None and argv!=expected_argv or any(v not in argv for v in required):
        raise ValueError('GPU process command/scope differs')
    env=dict(v.split('=',1) for v in (proc/'environ').read_text().split('\0') if '=' in v)
    if env.get('CUDA_VISIBLE_DEVICES')!=str(gpu):raise ValueError('owned process CUDA mask differs from frozen GPU')
    if session is not None and env.get('ROBOT_WORKBENCH_SESSION_ID')!=session:
        raise ValueError('GPU worker belongs to another Session')
    if any(env.get(k)!=v for k,v in (expected_env or {}).items()):
        raise ValueError('detector runtime root/pinned identity changed; start a new private scope')


def detector_environment(config,contract=None):
    from run_pipeline import pinned_files,below,read_contract_bytes
    root=config['detection_runtime_root']
    contract=contract or json.loads(read_contract_bytes(config))
    pins={k:v for group in pinned_files(contract) for k,v in group.items() if below(k,root)}
    if not pins:raise ValueError('detector runtime has no frozen pins')
    identity=hashlib.sha256(json.dumps(pins,sort_keys=True).encode()).hexdigest()
    return dict(ROBOT_WORKBENCH_DETECTOR_RUNTIME_ROOT=root,ROBOT_WORKBENCH_DETECTOR_RUNTIME_IDENTITY=identity)


def _model_status(port):
    connection=HTTPConnection('127.0.0.1',port,timeout=2)
    try:
        connection.request('GET','/status',headers={'Connection':'close'})
        response=connection.getresponse();raw=response.read(256*1024+1)
        if response.status!=200 or len(raw)>256*1024:raise ValueError('bounded shared model status unavailable')
        value=json.loads(raw)
        if not isinstance(value,dict):raise ValueError('shared model status must be an object')
        return value
    finally:connection.close()


def _proc_identity(root,pid):
    path=Path(root)/str(pid);fields=(path/'stat').read_text().rsplit(')',1)[1].split()
    return dict(pid=pid,start_ticks=fields[19],ppid=int(fields[1]),pgid=int(fields[2]),sid=int(fields[3]),
                uid=path.stat().st_uid,process_state=fields[0])


def _model_receipt(path):
    try:fd=os.open(path,os.O_RDONLY|os.O_NOFOLLOW|os.O_NONBLOCK)
    except FileNotFoundError:return None
    with os.fdopen(fd) as stream:
        info=os.fstat(stream.fileno())
        if not stat.S_ISREG(info.st_mode) or info.st_mode&0o077 or info.st_uid!=os.getuid() or info.st_size>65536:
            raise ValueError('shared-model ownership receipt must be bounded and private to this user')
        raw=stream.read(65537)
    if len(raw)>65536:raise ValueError('shared-model ownership receipt exceeds its size bound')
    return raw


def shared_model_process(execution,*,proc_root='/proc'):
    """Admit exactly the current validated child of a frozen managed model service."""
    spec=execution.get('shared_model')
    if spec is None:return None
    path=Path(spec['service_receipt'])
    raw=_model_receipt(path)
    if raw is None:return None
    receipt=json.loads(raw)
    if receipt.get('schema')!='robot_workbench.managed_service.v1' or receipt.get('spec_sha256')!=spec['service_spec_sha256']:
        raise ValueError('shared model managed-service deployment differs')
    identity=receipt.get('identity')
    if identity is None and receipt.get('phase')=='starting':return None
    if not isinstance(identity,dict) or type(identity.get('pid')) is not int or identity['pid']<=0:
        raise ValueError('shared model service receipt has no verified identity')
    root=Path(proc_root);service_pid=identity['pid']
    if not (root/str(service_pid)).exists():return None
    service=_proc_identity(root,service_pid)
    if service['process_state'] in ('Z','X'):return None
    if (receipt.get('phase')!='running' or identity.get('script')!=spec['service_script']
            or any(identity.get(key)!=service[key] for key in ('start_ticks','pgid','sid','uid'))
            or service['uid']!=os.getuid() or service['pgid']!=service_pid or service['sid']!=service_pid
            or identity.get('boot_id')!=(root/'sys/kernel/random/boot_id').read_text().strip()):
        raise ValueError('shared model service PID/start/boot/session identity changed')
    for key in ('service','worker'):
        if hashlib.sha256(Path(spec[key+'_script']).read_bytes()).hexdigest()!=spec[key+'_script_sha256']:
            raise ValueError('shared model '+key+' code changed')
    argv=[v.decode() for v in (root/str(service_pid)/'cmdline').read_bytes().split(b'\0') if v]
    if role_spec_digest(argv)!=spec['service_argv_sha256'] or spec['service_script'] not in argv:
        raise ValueError('shared model service command differs from frozen deployment')
    if argv.count('--port')!=1 or argv[argv.index('--port')+1:argv.index('--port')+2]!=[str(spec['status_port'])]:
        raise ValueError('shared model HTTP port differs from its frozen command')
    _process(service_pid,identity['start_ticks'],gpu=execution['gpu_index'],proc_root=root,
             expected_argv=argv,session=execution['session_id'])
    try:status=_model_status(spec['status_port'])
    except OSError:return None  # No worker is admitted; any live GPU PID still fails the common unowned-process check.
    if status.get('service_pid')!=service_pid:raise ValueError('shared model HTTP service PID differs')
    pid=status.get('worker_pid')
    if pid is None and status.get('loaded') is False:return None
    if (type(pid) is not int or pid<=0 or pid==service_pid or status.get('loaded') is not True
            or status.get('status')!='ready' or status.get('worker_exit_code') is not None
            or status.get('model_id')!=spec['model_id']):
        raise ValueError('shared model worker is not currently validated for the frozen checkpoint')
    worker=_proc_identity(root,pid)
    if worker['ppid']!=service_pid or worker['uid']!=service['uid'] or worker['process_state'] in ('Z','X'):
        raise ValueError('model worker is not the live direct child of its owned service')
    worker_argv=[v.decode() for v in (root/str(pid)/'cmdline').read_bytes().split(b'\0') if v]
    if (len(worker_argv)!=6 or worker_argv[1:4]!=['-u','-B',spec['worker_script']]
            or worker_argv[4]!='--bundle-json'):
        raise ValueError('shared model worker command differs from the fixed Base566 launcher')
    bundle=json.loads(worker_argv[5])
    from robot_workbench.inference_model import validate_bundle,validate_worker_identity
    validate_bundle(bundle,verify_files=False)
    if (bundle['model_id']!=spec['model_id'] or bundle['python']!=worker_argv[0]
            or bundle['artifacts']['checkpoint']['sha256']!=spec['checkpoint_sha256']
            or bundle['artifacts']['worker']['path']!=spec['worker_script']
            or bundle['artifacts']['worker']['sha256']!=spec['worker_script_sha256']):
        raise ValueError('shared model worker manifest differs from frozen model/code/checkpoint')
    validate_worker_identity(status.get('worker_identity') or {},spec['model_id'],bundle=bundle)
    _process(pid,worker['start_ticks'],gpu=execution['gpu_index'],proc_root=root,
             expected_argv=worker_argv,session=execution['session_id'])
    after=_model_status(spec['status_port'])
    worker_after=_proc_identity(root,pid);service_after=_proc_identity(root,service_pid)
    fields=('pid','start_ticks','ppid','pgid','sid','uid')
    if (any(after.get(key)!=status.get(key) for key in ('service_pid','worker_pid','worker_identity','model_id','loaded','status','worker_exit_code'))
            or any(worker_after[key]!=worker[key] or service_after[key]!=service[key] for key in fields)
            or worker_after['process_state'] in ('Z','X') or service_after['process_state'] in ('Z','X')
            or _model_receipt(path)!=raw):
        raise ValueError('shared model service or worker changed during GPU admission')
    return dict(service_pid=service_pid,worker_pid=pid,worker_start_ticks=worker['start_ticks'],
                model_id=spec['model_id'],checkpoint_sha256=spec['checkpoint_sha256'],max_memory_mib=spec['max_memory_mib'])


def gpu_check(config, *, require=None, exclude=None, proc_root='/proc', query=subprocess.check_output):
    """Read-only admission: actual graphics+compute PIDs, never stop/discover owners."""
    execution=validate(config)
    if execution is None:raise ValueError('explicit local resource contract required')
    plant=owner_spec(execution);known={};required=set()
    physical=execution.get('environment')=='physical'
    owner_uses_gpu=plant.get('gpu_context_required',True)
    if hashlib.sha256(Path(plant['script']).read_bytes()).hexdigest()!=plant['script_sha256']:
        raise ValueError('compute owner script changed after scope registration')
    _process(plant['pid'],plant['process_start_ticks'],gpu=execution['gpu_index'],proc_root=proc_root,expected_argv=plant['argv'],
        session=execution['session_id'])
    known[plant['pid']]=plant['max_memory_mib']
    if owner_uses_gpu:required.add(plant['pid'])
    shared=shared_model_process(execution,proc_root=proc_root)
    if shared:
        if shared['worker_pid'] in known:raise ValueError('shared model PID assigned to two scopes')
        known[shared['worker_pid']]=shared['max_memory_mib'];required.add(shared['worker_pid'])
    for role in ('detector','planner'):
        if role==exclude:continue
        spec=config.get(role+'_service')
        if spec is None:
            if role==require:raise ValueError(role+' service is not configured')
            continue
        path=Path(spec['root'])/'status.json'
        if not path.is_file():
            if role==require:raise ValueError(role+' service is not ready')
            continue
        state=json.loads(path.read_text())
        pid=state.get('pid');start=state.get('process_start_ticks')
        if type(pid) is not int or pid<=0:raise ValueError('invalid '+role+' PID')
        proc=Path(proc_root)/str(pid)
        if not proc.exists():
            if role==require:raise ValueError(role+' service exited')
            continue
        if role=='planner':
            from planner_service import load_private_spec
            private=load_private_spec(Path(spec['allowed_output_root'])/'planner-spec.json',spec['spec_sha256'])
            if private.get('session_id')!=execution['session_id']:
                raise ValueError('planner private scope belongs to another Session')
            if any(state.get(k)!=v for k,v in {**private,'spec_sha256':spec['spec_sha256']}.items()):raise ValueError('planner private identity differs')
            tokens=[config['workbench_code']+'/curobo_worker.py','--service-spec-file',spec['allowed_output_root']+'/planner-spec.json']
            uses_gpu=state.get('model_initialized') is True
        else:
            if (any(state.get(k)!=spec[k] for k in ('owner_token','identity','socket','allowed_read_roots','allowed_output_root'))
                    or state.get('service_root')!=spec['root']):raise ValueError('detector private identity differs')
            if (type(state.get('physical_gpu')) is not int or state['physical_gpu']!=execution['gpu_index']
                    or type(state.get('logical_cuda_index')) is not int or state['logical_cuda_index']!=0):
                raise ValueError('local detector must declare the frozen physical GPU and logical CUDA0')
            tokens=[config['detection_script'],'--serve','--service-root',spec['root'],'--owner-token',spec['owner_token']]
            uses_gpu=state.get('status')=='ready'
        _process(pid,start,gpu=execution['gpu_index'],proc_root=proc_root,required=tokens,session=execution['session_id'],
                 expected_env=detector_environment(config) if role=='detector' else None)
        if role==require and state.get('status')!='ready':raise ValueError(role+' is not ready')
        if pid in known:raise ValueError('GPU process assigned to two scopes')
        known[pid]=spec['max_memory_mib']
        if uses_gpu:required.add(pid)
    for role,spec in execution.get('roles',{}).items():
        if role==exclude:continue
        path=Path(spec['registration'])
        if not path.is_file():
            if role==require:raise ValueError(role+' GPU role has not registered')
            continue
        state=json.loads(path.read_text());pid=state.get('pid');proc=Path(proc_root)/str(pid)
        if type(pid) is not int or not proc.exists():
            if role==require:raise ValueError(role+' GPU role is not alive')
            continue
        if state.get('spec_sha256')!=role_spec_digest(spec) or state.get('session_id')!=execution['session_id'] or state.get('role')!=role:
            raise ValueError('GPU role private registration identity differs')
        _process(pid,state.get('process_start_ticks'),gpu=execution['gpu_index'],proc_root=proc_root,expected_argv=state.get('argv'),session=execution['session_id'])
        entry=state.get('entrypoint');role_argv(execution,role,state['argv'],entry)
        expected=spec['script_sha256'] if role=='preview' else spec['entrypoints'][entry]
        if state.get('entrypoint_sha256')!=expected or hashlib.sha256(Path(entry).read_bytes()).hexdigest()!=expected:
            raise ValueError('registered GPU role code changed')
        if role=='preview' and (state.get('config_sha256')!=spec['config_sha256']
                or hashlib.sha256(Path(spec['config_path']).read_bytes()).hexdigest()!=spec['config_sha256']
                or not isinstance(state.get('renderer_instance_id'),str) or not re.fullmatch('[0-9a-f]{32}',state['renderer_instance_id'])):
            raise ValueError('preview renderer instance/config binding differs')
        if pid in known:raise ValueError('GPU PID registered under more than one role')
        known[pid]=spec['max_memory_mib']
        if state.get('status')=='ready':required.add(pid)
        if role==require and state.get('status')!='ready':raise ValueError(role+' GPU role not ready')
    xml=ET.fromstring(query(['nvidia-smi','-i',str(execution['gpu_index']),'-q','-x'],text=True))
    cards=xml.findall('gpu')
    if len(cards)!=1 or cards[0].findtext('uuid')!=execution['gpu_uuid']:
        raise ValueError('actual GPU UUID differs from frozen resource')
    card=cards[0]
    def mib(text):
        match=re.fullmatch(r'([0-9]+) MiB',(text or '').strip())
        if not match:raise ValueError('GPU memory accounting unavailable')
        return int(match[1])
    free=mib(card.findtext('fb_memory_usage/free'));seen=set()
    rows=[]
    for row in card.findall('processes/process_info'):
        pid=int(row.findtext('pid'));memory=mib(row.findtext('used_memory'))
        if pid==plant['pid'] and not owner_uses_gpu:raise ValueError('declared CPU compute owner unexpectedly owns a GPU context')
        if pid not in known:raise ValueError('selected GPU has an unowned process')
        if memory>known[pid]:raise ValueError('owned GPU process exceeds memory budget')
        seen.add(pid);rows.append(dict(pid=pid,memory_mib=memory,type=row.findtext('type')))
    if not required<=seen:raise ValueError('declared owner/initialized worker GPU context missing')
    if len(seen)>=execution['max_processes'] and require is None:
        raise ValueError('no declared GPU process slot available')
    minimum=max([execution['min_free_memory_mib'],*[s['min_free_memory_mib'] for s in
        (config.get('detector_service'),config.get('planner_service')) if s]])
    if len(seen)>execution['max_processes'] or free<minimum:
        raise ValueError('local GPU process/free-memory budget exceeded')
    result=dict(local_owned=True,gpu=execution['gpu_index'],gpu_uuid=execution['gpu_uuid'],free_mib=free,processes=rows)
    if shared:result['shared_model']=shared
    return result


def check_command(config, *, require=None, exclude=None):
    from run_pipeline import python_command
    script=('import json,sys; sys.path[:0]=[sys.argv[1],sys.argv[1].rsplit("/",1)[0]]; '
            'from local_execution import gpu_check; print(json.dumps(gpu_check(json.loads(sys.argv[2]),require=sys.argv[3] or None,exclude=sys.argv[4] or None)))')
    remote={key:value for key,value in config.items() if key!='compute_transport'}
    return python_command(script,config['workbench_code'],json.dumps(remote),require or '',exclude or '')


def stamp_plan(plan, resource):
    if resource not in (TARGET,PHYSICAL_TARGET):raise ValueError('unrecognized local compute resource')
    plan.update(instance_id=resource['instance_id'],gpu=resource['gpu_index'],
                compute_resource=dict(resource,logical_cuda_index=0))
    return plan
