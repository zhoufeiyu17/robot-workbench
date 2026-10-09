"""One workbench-owned MotionGen process, reachable only over a private Unix socket."""
import json
import hashlib
from pathlib import Path, PurePosixPath
import re
import subprocess
import threading
from uuid import uuid4

RUNTIME_KEYS=('native_code','assets','deps','curobo_root','workbench_code')


def frozen_identity(config,contract):
    from run_pipeline import pinned_files,digest
    pins={k:v for group in pinned_files(contract) for k,v in group.items()}
    paths={k:config[k] for k in RUNTIME_KEYS}
    return dict(input_pins_sha256=digest(json.dumps(pins,sort_keys=True).encode()),runtime_paths=paths,
                identity=digest(json.dumps(dict(input_pins=pins,runtime_paths=paths),sort_keys=True).encode()))


def private_spec_arguments(config):
    spec=config['planner_service']
    return dict(spec_path=spec['allowed_output_root']+'/planner-spec.json',
        spec_sha256=spec['spec_sha256'],workbench_code=config['workbench_code'])


def decode_private_spec(payload,expected_sha256):
    if hashlib.sha256(payload).hexdigest()!=expected_sha256:raise ValueError('planner private spec identity differs')
    spec=json.loads(payload);validate_spec(spec,private=True)
    return spec


def load_private_spec(path,expected_sha256):
    import stat
    file=Path(path)
    if file.is_symlink() or not file.is_file() or file.stat().st_size>65536 or stat.S_IMODE(file.stat().st_mode)!=0o600:
        raise ValueError('planner spec must be a bounded private regular file')
    payload=file.read_bytes()
    spec=decode_private_spec(payload,expected_sha256)
    if file.resolve()!=Path(spec['allowed_output_root']+'/planner-spec.json').resolve():
        raise ValueError('planner private spec is outside its owned output root')
    return spec


def startup_config(template,directory,*,shared_config=None):
    """Freeze one private worker and jobs scope for this application lifetime."""
    from run_pipeline import digest,pinned_files,load_config
    base=json.loads(Path(template).read_text())
    if base.get('planning_backend')!='motiongen':return Path(template)
    contract=json.loads(Path(base['contract_path']).read_text())
    session=contract.get('identity',{}).get('session_id')
    if not session:raise ValueError('MotionGen startup requires the owning Session in its contract')
    token=uuid4().hex; old=base['remote_run_root']; jobs=old+'-planner-jobs-'+token[:8]
    root='/tmp/rbw-planner-'+token[:24]
    previous=base.get('planner_service') or {}
    local=bool(base.get('execution'))
    frozen=frozen_identity(base,contract) if local else None
    if local:contract['planner_inputs']={k:v for group in pinned_files(contract) for k,v in group.items()}
    if shared_config is not None:
        shared,_,_=load_config(Path(shared_config))
        spec=shared.get('planner_service')
        if not spec:raise ValueError('主工作台尚未配置共享 MotionGen 服务')
        # A shared process has one frozen code/assets identity. Never silently
        # replace another mode's observation or calibration inputs to match it.
        expected=frozen['identity'] if local else digest(json.dumps(pinned_files(contract),sort_keys=True).encode())
        if spec['identity']!=expected or shared.get('execution')!=base.get('execution'):
            raise ValueError('双 DSW 与真机规划代码或资产身份不一致，不能共享规划进程')
        for key in ('native_code','assets','deps','curobo_root','workbench_code'):
            if base[key]!=shared[key]:raise ValueError('共享规划 runtime 路径不一致：'+key)
        spec=dict(spec);root=spec['root'];jobs=spec['allowed_output_root']
        payload=(Path(shared_config).parent/'planner-spec.json').read_bytes()
        decode_private_spec(payload,spec['spec_sha256'])
    else:
        spec=None
    private=dict(root=root,socket=root+'/p',owner_token=uuid4().hex,session_id=session,
        identity=digest(json.dumps(pinned_files(contract),sort_keys=True).encode()),
        allowed_read_roots=[*contract['filesystem']['readonly_roots'],jobs],allowed_output_root=jobs,
        max_memory_mib=12288,min_free_memory_mib=16384,
        max_lifetime_seconds=int(contract['resources'].get('service_lifetime_seconds',contract['resources']['max_wall_time_minutes']*60)))
    if local:
        from local_execution import TARGET,validate
        validate(base,contract)
        private.update(frozen,compute_resource=TARGET)
    if spec is None:
        payload=json.dumps(private,sort_keys=True,separators=(',',':')).encode()
        spec={k:v for k,v in private.items() if k!='owner_token'}
        spec['spec_sha256']=digest(payload)
    base['remote_run_root']=jobs+'/template'+('-'+token[:8] if shared_config else '')
    base['planner_service']=spec
    fs=contract['filesystem']
    fs['writable_roots']=[base['remote_run_root'] if value==old else value for value in fs['writable_roots']
        if value not in (previous.get('root'),previous.get('allowed_output_root'))]
    fs['writable_roots'] += [jobs,root]
    contract['planner_service']=dict(spec)
    destination=Path(directory)/'startups'/('planner-'+token)
    destination.mkdir(parents=True,mode=0o700,exist_ok=False)
    (destination/'planner-spec.json').write_bytes(payload);(destination/'planner-spec.json').chmod(0o600)
    base['contract_path']=str((destination/'contract.json').resolve())
    config=destination/'config.json';config.write_text(json.dumps(base,ensure_ascii=False,indent=2));config.chmod(0o600)
    if 'config_sha256' in contract.get('provenance',{}):
        contract['provenance']['config_sha256']={config.name:digest(config.read_bytes())}
    (destination/'contract.json').write_text(json.dumps(contract,ensure_ascii=False,indent=2))
    (destination/'contract.json').chmod(0o600)
    load_config(config)  # Pure contract/path validation before launching any process.
    return config


def validate_spec(spec,*,private=False):
    fields={'root','socket','session_id','identity','allowed_read_roots','allowed_output_root',
            'max_memory_mib','min_free_memory_mib','max_lifetime_seconds'}
    fields.add('owner_token' if private else 'spec_sha256')
    local_fields={'input_pins_sha256','runtime_paths','compute_resource'}
    if isinstance(spec,dict) and set(spec)&local_fields:
        fields|=local_fields
    if not isinstance(spec,dict) or set(spec)!=fields:raise ValueError('invalid planner service fields')
    if 'input_pins_sha256' in spec:
        from local_execution import TARGET
        if (spec['compute_resource']!=TARGET or not isinstance(spec['input_pins_sha256'],str)
                or not re.fullmatch('[0-9a-f]{64}',spec['input_pins_sha256'])
                or not isinstance(spec['runtime_paths'],dict) or set(spec['runtime_paths'])!=set(RUNTIME_KEYS)
                or any(not isinstance(v,str) or not v.startswith('/') for v in spec['runtime_paths'].values())):
            raise ValueError('invalid frozen planner code/model inputs')
    root=PurePosixPath(spec['root'])
    if (len(root.parts)!=3 or root.parts[1]!='tmp' or not root.name.startswith('rbw-planner-')
            or len(root.name)<=len('rbw-planner-') or PurePosixPath(spec['socket']).parent!=root
            or len(spec['socket'].encode())>=104):raise ValueError('planner socket must be in an owned temporary root')
    if private and (not isinstance(spec['owner_token'],str) or len(spec['owner_token'])<16):raise ValueError('planner ownership nonce required')
    if not private and (not isinstance(spec['spec_sha256'],str) or not re.fullmatch('[0-9a-f]{64}',spec['spec_sha256'])):
        raise ValueError('planner private spec SHA required')
    if not isinstance(spec['session_id'],str) or not spec['session_id']:
        raise ValueError('planner owner/session identity required')
    if not isinstance(spec['identity'],str) or not re.fullmatch('[0-9a-f]{64}',spec['identity']):raise ValueError('planner frozen identity required')
    for key in ('max_memory_mib','min_free_memory_mib','max_lifetime_seconds'):
        if type(spec[key]) is not int or spec[key]<=0:raise ValueError('positive bounded planner budgets required')
    if not isinstance(spec['allowed_read_roots'],list) or not spec['allowed_read_roots']:
        raise ValueError('planner read boundaries required')
    for value in [*spec['allowed_read_roots'],spec['allowed_output_root']]:
        if not isinstance(value,str) or not PurePosixPath(value).is_absolute() or '..' in PurePosixPath(value).parts:
            raise ValueError('absolute planner path boundaries required')


def validate_config(config,contract):
    spec=config.get('planner_service')
    if spec is None:return
    from run_pipeline import below,digest,pinned_files
    validate_spec(spec)
    if spec!=contract.get('planner_service') or config.get('planning_backend')!='motiongen':
        raise ValueError('planner service must match a frozen MotionGen contract')
    fs=contract['filesystem']; resources=contract['resources']
    owned_output=config['execution']['plant']['scope'] if config.get('execution') else '/mnt/data/feiyu.zhou/runs/robot_sim'
    if (spec['root'] not in fs['writable_roots'] or spec['allowed_output_root'] not in fs['writable_roots']
            or not below(config['remote_run_root'],spec['allowed_output_root'])
            or not below(spec['allowed_output_root'],owned_output)):
        raise ValueError('planner output is outside the frozen task scope')
    expected_workers=config['execution']['max_processes'] if config.get('execution') else 2
    if resources.get('max_workers_per_instance')!=expected_workers:raise ValueError('planner/Isaac worker budget differs')
    if spec['session_id']!=contract.get('identity',{}).get('session_id'):
        raise ValueError('planner Session differs from frozen contract owner')
    if spec['allowed_output_root'] not in spec['allowed_read_roots']:
        raise ValueError('planner must read current task outputs')
    if any(p!=spec['allowed_output_root'] and not any(below(p,r) for r in fs['readonly_roots']) for p in spec['allowed_read_roots']):
        raise ValueError('planner read root not authorized')
    pins=pinned_files(contract)
    if config.get('execution'):
        actual={k:v for group in pins for k,v in group.items()}
        frozen=dict(input_pins=contract.get('planner_inputs'),runtime_paths=spec.get('runtime_paths'))
        if (not isinstance(frozen['input_pins'],dict) or not frozen['input_pins']
                or any(actual.get(k)!=v for k,v in frozen['input_pins'].items())
                or spec.get('input_pins_sha256')!=digest(json.dumps(frozen['input_pins'],sort_keys=True).encode())
                or frozen['runtime_paths']!={k:config[k] for k in RUNTIME_KEYS}
                or spec['identity']!=digest(json.dumps(frozen,sort_keys=True).encode())):
            raise ValueError('planner frozen code/model inputs differ; create a new scope')
    elif spec['identity']!=digest(json.dumps(pins,sort_keys=True).encode()):raise ValueError('planner pins identity differs')
    code=contract.get('provenance',{}).get('code_sha256',{})
    required=('curobo_worker.py','planner_service.py','motiongen_plan.py','motiongen_motion.py','gpu_acceptance.py',
              'curobo_robot_model.py','cup_contact.py','cup_envelope.py','region_self_collision.py')
    if any(config['workbench_code']+'/'+name not in code for name in required):
        raise ValueError('planner executable and solver dependencies must be pinned')


OWNED_GPU_CHECK=r'''import json,pathlib,subprocess,sys
c=json.loads(sys.argv[1]); sys.path.insert(0,c['workbench_code'])
from planner_service import load_private_spec
s=load_private_spec(c['spec_path'],c['spec_sha256']); state=json.loads((pathlib.Path(s['root'])/'status.json').read_text())
assert all(state[k]==v for k,v in s.items()),'planner ownership/identity differs'
assert state['status']=='ready','planner is not ready'
pid=int(state['pid']); proc=pathlib.Path('/proc')/str(pid)
assert (proc/'stat').read_text().rsplit(')',1)[1].split()[19]==str(state['process_start_ticks']),'planner PID reused'
cmd=(proc/'cmdline').read_bytes().split(b'\0')
assert (c['workbench_code']+'/curobo_worker.py').encode() in cmd and b'--service-spec-file' in cmd,'planner command identity differs'
uuid,free=[v.strip() for v in subprocess.check_output(['nvidia-smi','-i','0','--query-gpu=uuid,memory.free','--format=csv,noheader,nounits'],text=True).strip().split(',')]
rows=subprocess.check_output(['nvidia-smi','--query-compute-apps=gpu_uuid,pid,used_gpu_memory','--format=csv,noheader,nounits'],text=True)
selected=[list(map(str.strip,r.split(','))) for r in rows.splitlines() if r.split(',')[0].strip()==uuid]
assert all(int(r[1])==pid for r in selected) and len(selected)<=1,'selected GPU has an unowned process'
assert selected or not state['model_initialized'],'initialized planner GPU context missing'
assert not selected or int(selected[0][2])<=s['max_memory_mib'],'planner exceeds memory budget'
assert int(free)>=s['min_free_memory_mib'],'insufficient free memory for Isaac'
print(json.dumps({'planner_owned':True,'model_initialized':state['model_initialized'],'free_mib':int(free)}))
'''


INSTALL_SPEC=r'''import hashlib,json,os,pathlib,sys
payload=sys.stdin.buffer.readline(65537).rstrip(b'\n'); assert len(payload)<=65536 and hashlib.sha256(payload).hexdigest()==sys.argv[2],'planner startup spec identity differs'
s=json.loads(payload); root=pathlib.Path(s['root']); output=pathlib.Path(s['allowed_output_root']); file=output/'planner-spec.json'
for mapping in json.loads(sys.argv[1]):
 for p,sha in mapping.items():assert hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()==sha,'planner file identity differs '+p
if sys.argv[3]=='resume':
 assert not output.is_symlink() and not file.is_symlink() and file.read_bytes()==payload,'planner restart scope differs'
 if root.exists():
  assert not root.is_symlink(),'planner runtime root is a symlink'
  state=json.loads((root/'status.json').read_text())
  assert all(state.get(k)==v for k,v in s.items()),'planner restart ownership differs'
  proc=pathlib.Path('/proc')/str(int(state['pid']))
  assert not proc.exists() or (proc/'stat').read_text().rsplit(')',1)[1].split()[19]!=str(state['process_start_ticks']),'previous planner is still alive'
  assert {p.name for p in root.iterdir()}<= {'status.json','status.partial','p'},'unexpected planner runtime files'
  for p in root.iterdir():p.unlink()
  root.rmdir()
else:
 assert not root.exists(),'planner root already exists'
 output.mkdir(parents=True,mode=0o700,exist_ok=False)
 with file.open('xb') as stream:stream.write(payload)
 os.chmod(file,0o600)
'''


class PlannerService:
    def __init__(self):
        self.process=None; self.log=None; self.state={'status':'stopped','model_initialized':False}
        self.lock=threading.Lock()
        self.closed=False

    def start(self,config_path,directory,*,resume=False):
        from run_pipeline import load_config,python_command,transport,lane,pinned_files,GPU_CHECK
        from detector_service import SUPERVISOR
        config,contract,_=load_config(Path(config_path)); spec=config.get('planner_service')
        if spec is None:return
        if self.process is not None:raise ValueError('planner service already started')
        install=config['curobo_root']; wb=config['workbench_code']; cache=spec['allowed_output_root']+'/cache'
        private=private_spec_arguments(config)
        payload=(Path(config_path).parent/'planner-spec.json').read_bytes()
        decoded=decode_private_spec(payload,private['spec_sha256'])
        if {k:v for k,v in decoded.items() if k!='owner_token'}!={k:v for k,v in spec.items() if k!='spec_sha256'}:
            raise ValueError('local private planner scope differs from frozen contract')
        args=['env','CUDA_VISIBLE_DEVICES='+str(lane(config,'dsw-scene')['gpu']),'PYTHONNOUSERSITE=1','PYTHONDONTWRITEBYTECODE=1',
              'PYTHONPATH='+':'.join((wb,str(Path(wb).parent),install+'/python',install+'/deps',config['deps'],config['native_code']+'/source/robot_sim')),
              'CUDA_HOME='+install+'/cuda-overlay','LD_LIBRARY_PATH='+install+'/cuda-overlay/lib64',
              'TORCH_EXTENSIONS_DIR='+cache+'/torch','WARP_CACHE_PATH='+cache+'/warp','XDG_CACHE_HOME='+cache,
              'CUDA_CACHE_PATH='+cache+'/cuda',
              'OMP_NUM_THREADS=1','MKL_NUM_THREADS=1','OPENBLAS_NUM_THREADS=1','MAX_JOBS=1',
              '/isaac-sim/python.sh','-u','-B',wb+'/curobo_worker.py','--native-code',config['native_code'],
              '--assets',config['assets'],'--allow-output-root',spec['allowed_output_root'],
              '--service-spec-file',private['spec_path'],'--spec-sha256',private['spec_sha256']]
        for root in spec['allowed_read_roots']:args+=['--allow-read-root',root]
        if config.get('execution'):args.insert(1,'ROBOT_WORKBENCH_SESSION_ID='+config['execution']['session_id'])
        # The same stdin-owned supervisor already used by the detector bounds
        # this child; worker EOF/socket cancellation never searches for PIDs.
        if config.get('execution'):
            from local_execution import check_command
            guard=check_command(config,exclude='planner')
        else:guard=python_command(GPU_CHECK,0)
        if config.get('execution'):
            from detector_service import LOCAL_SUPERVISOR
            supervisor=python_command(LOCAL_SUPERVISOR,json.dumps(args),spec['max_lifetime_seconds'],'hold-stdin',wb)
        else:supervisor=python_command(SUPERVISOR,json.dumps(args),spec['max_lifetime_seconds'],'hold-stdin')
        command=guard+' 1>&2 && '+python_command(INSTALL_SPEC,json.dumps(pinned_files(contract)),private['spec_sha256'],'resume' if resume else 'new')+' && '+('exec ' if config.get('execution') else '')+supervisor
        Path(directory).mkdir(parents=True,exist_ok=True)
        self.log=(Path(directory)/'planner-service.log').open('a')
        self.process=subprocess.Popen(transport(config,'dsw-scene',command),stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=self.log,text=True,bufsize=1)
        self.process.stdin.write(payload.decode()+'\n');self.process.stdin.flush()
        self.config=config
        self.config_path,self.directory=Path(config_path),Path(directory)
        self.closed=False
        self.state={'status':'starting','model_initialized':False}
        def read_state():
            try:
                for line in self.process.stdout:
                    self.log.write(line);self.log.flush()
                    value=json.loads(line)
                    if value.get('passed') is False:raise ValueError(value.get('error','planner startup failed'))
                    state=value.get('service')
                    if not isinstance(state,dict) or any(state.get(k)!=v for k,v in spec.items()) or 'owner_token' in state:
                        raise ValueError('planner startup state ownership differs')
                    with self.lock:self.state=state
            except Exception as error:
                with self.lock:self.state={'status':'failed','error':str(error),'model_initialized':False}
                if self.process.stdin and not self.process.stdin.closed:self.process.stdin.close()
        self.thread=threading.Thread(target=read_state,daemon=True)
        self.thread.start()

    def status(self):
        if self.closed:return {'status':'stopped','model_initialized':False}
        if self.process is not None and self.process.poll() is not None:
            return {'status':'failed','error':'规划服务已退出，需重新准备环境','model_initialized':False}
        with self.lock:return dict(self.state)

    def close(self):
        self.closed=True
        if self.process is not None:
            if self.process.stdin and not self.process.stdin.closed:
                try:self.process.stdin.close()
                except BrokenPipeError:pass
            try:self.process.wait(timeout=30)
            except subprocess.TimeoutExpired:
                self.process.terminate()
                try:self.process.wait(timeout=30 if getattr(self,'config',{}).get('execution') else 5)
                except subprocess.TimeoutExpired:self.process.kill();self.process.wait(timeout=5)
        if getattr(self,'thread',None) is not None:self.thread.join(timeout=5)
        if self.log is not None:self.log.close()
        self.state={'status':'stopped','model_initialized':False}

    def recover(self,*,progress=None,timeout=40):
        """Cancel the owned GPU child, then restart only that planner scope."""
        import time
        config_path,directory=self.config_path,self.directory
        self.close()
        self.process=None;self.log=None;self.thread=None
        from detector_service import wait_for_gpu_idle
        def waiting(remaining):
            if progress:progress('等待 GPU 释放（最多 30 秒）',next_label='恢复规划服务')
        wait_for_gpu_idle('dsw-scene',progress=waiting,**({'config':self.config,'exclude':'planner'} if self.config.get('execution') else {}))
        if progress:progress('当前规划进程已退出，正在恢复规划服务',next_label='场景重建及规划')
        self.start(config_path,directory,resume=True)
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            state=self.status()
            if state['status']=='ready':return
            if state['status']=='failed':raise RuntimeError(state.get('error','规划服务恢复失败'))
            time.sleep(.05)
        self.close()
        raise TimeoutError('规划服务恢复超时；环境尚未就绪')
