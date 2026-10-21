"""Owned persistent scene exporter; shares input scope, never control or planner state."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import subprocess
import threading
from uuid import uuid4

from planner_service import PlannerService,validate_spec,decode_private_spec,load_private_spec


def private_spec_arguments(config):
    spec=config['scene_service']
    return dict(spec_path=spec['root']+'/private-spec.json',spec_sha256=spec['spec_sha256'])


def finger_envelope_pin(config,contract):
    from run_pipeline import pinned_files,below
    pin=dict(path=config['finger_envelope'],sha256=config['finger_envelope_sha256'])
    pins={k:v for group in pinned_files(contract) for k,v in group.items()}
    if (pins.get(pin['path'])!=pin['sha256']
            or not any(below(pin['path'],root) for root in contract['filesystem']['readonly_roots'])):
        raise ValueError('scene finger envelope must be exactly SHA-pinned and authorized readonly')
    return pin


def startup_config(template,directory):
    from run_pipeline import load_config
    cfg,contract,_=load_config(Path(template))
    if not cfg.get('planner_service'):raise ValueError('scene service requires the owned planner scope')
    token=uuid4().hex;root='/tmp/rbw-scene-'+token[:24]
    private=deepcopy(cfg['planner_service']);private.pop('spec_sha256')
    private.update(root=root,socket=root+'/p',owner_token=uuid4().hex,
        finger_envelope=finger_envelope_pin(cfg,contract))
    role=(cfg.get('execution') or {}).get('roles',{}).get('scene')
    if cfg.get('execution') and role is None:raise ValueError('explicit scene GPU role required')
    if role:private['max_memory_mib']=role['max_memory_mib']
    validate_spec(private,private=True,kind='scene')
    payload=json.dumps(private,sort_keys=True,separators=(',',':')).encode()
    public={k:v for k,v in private.items() if k!='owner_token'}
    public['spec_sha256']=hashlib.sha256(payload).hexdigest()
    cfg['scene_service']=public;contract['scene_service']=deepcopy(public)
    contract['filesystem']['writable_roots'].append(root)
    destination=Path(directory)/'startups'/('scene-'+token)
    destination.mkdir(parents=True,mode=0o700,exist_ok=False)
    (destination/'scene-spec.json').write_bytes(payload);(destination/'scene-spec.json').chmod(0o600)
    # The planner still consumes its existing, unchanged capability.
    (destination/'planner-spec.json').write_bytes((Path(template).parent/'planner-spec.json').read_bytes())
    (destination/'planner-spec.json').chmod(0o600)
    cfg['contract_path']=str((destination/'contract.json').resolve())
    config=destination/'config.json';config.write_text(json.dumps(cfg,ensure_ascii=False,indent=2))
    if 'config_sha256' in contract.get('provenance',{}):
        contract['provenance']['config_sha256']={config.name:hashlib.sha256(config.read_bytes()).hexdigest()}
    (destination/'contract.json').write_text(json.dumps(contract,ensure_ascii=False,indent=2))
    load_config(config)
    return config


def validate_config(config,contract):
    spec=config.get('scene_service')
    if spec is None:return
    validate_spec(spec,kind='scene')
    if spec['finger_envelope']!=finger_envelope_pin(config,contract):
        raise ValueError('scene startup finger envelope differs from frozen contract')
    planner=config.get('planner_service') or {}
    if (contract.get('scene_service')!=spec or spec['root'] not in contract['filesystem']['writable_roots']
            or any(spec.get(k)!=planner.get(k) for k in ('session_id','identity','allowed_output_root',
                'allowed_read_roots','runtime_paths','input_pins_sha256','compute_resource','persistent_service','max_lifetime_seconds'))):
        raise ValueError('scene exporter and owned planner input identities differ')
    from run_pipeline import pinned_files
    pins={k:v for group in pinned_files(contract) for k,v in group.items()}
    if any(config['workbench_code']+'/'+name not in pins for name in ('scene_service.py','scene_worker.py','playground_sim.py')):
        raise ValueError('scene service implementation must be SHA-pinned')


INSTALL_SPEC=r'''import hashlib,json,os,pathlib,sys
payload=sys.stdin.buffer.readline(65537).rstrip(b'\n');assert len(payload)<=65536 and hashlib.sha256(payload).hexdigest()==sys.argv[2]
s=json.loads(payload);root=pathlib.Path(s['root']);output=pathlib.Path(s['allowed_output_root'])
for mapping in json.loads(sys.argv[1]):
 for name,digest in mapping.items():assert hashlib.sha256(pathlib.Path(name).read_bytes()).hexdigest()==digest,name
assert output.is_dir() and not output.is_symlink(),'planner jobs scope is not ready'
planner=json.loads((output/'planner-spec.json').read_text())
assert all(planner.get(k)==s.get(k) for k in ('identity','session_id','allowed_output_root','allowed_read_roots')),'scene/planner scope mismatch'
if sys.argv[3]=='resume' and root.exists():
 assert not root.is_symlink() and (root/'private-spec.json').read_bytes()==payload,'scene restart ownership differs'
 state=json.loads((root/'status.json').read_text());assert all(state.get(k)==v for k,v in s.items() if k!='owner_token')
 proc=pathlib.Path('/proc')/str(int(state['pid']))
 assert not proc.exists() or (proc/'stat').read_text().rsplit(')',1)[1].split()[19]!=str(state['process_start_ticks']),'old scene worker is alive'
 assert {p.name for p in root.iterdir()}<={'private-spec.json','status.json','status.partial','p'},'unknown scene runtime files'
 for p in root.iterdir():p.unlink()
 root.rmdir()
root.mkdir(mode=0o700,exist_ok=False)
with (root/'private-spec.json').open('xb') as stream:stream.write(payload)
os.chmod(root/'private-spec.json',0o600)
'''


class SceneService(PlannerService):
    resource_kind='scene'

    def start(self,config_path,directory,*,resume=False,launch=None):
        from run_pipeline import load_config,python_command,transport,lane,pinned_files,isaac_environment,AFFINITY_EXEC
        from detector_service import LOCAL_SUPERVISOR,SUPERVISOR
        from launch_environment import clean_command,HOST_PYTHON
        cfg,contract,_=load_config(Path(config_path));spec=cfg['scene_service'];validate_config(cfg,contract)
        if self.process is not None:raise ValueError('scene service already started')
        payload=(Path(config_path).parent/'scene-spec.json').read_bytes()
        private=decode_private_spec(payload,spec['spec_sha256'],kind='scene')
        if {k:v for k,v in private.items() if k!='owner_token'}!={k:v for k,v in spec.items() if k!='spec_sha256'}:
            raise ValueError('scene private scope differs from frozen contract')
        args=['/isaac-sim/python.sh','-u','-B',cfg['workbench_code']+'/scene_worker.py',
              '--service-spec-file',spec['root']+'/private-spec.json','--spec-sha256',spec['spec_sha256'],
              '--assets',cfg['assets'],'--native-code',cfg['native_code'],
              '--finger-envelope',cfg['finger_envelope'],'--finger-envelope-sha256',cfg['finger_envelope_sha256'],
              '--output',spec['allowed_output_root']+'/scene-worker-'+uuid4().hex,
              '--gpu',str(lane(cfg,'dsw-scene')['gpu']),'--instance',lane(cfg,'dsw-scene')['instance_id'],
              '--environment',cfg.get('environment','physical')]
        if cfg.get('execution'):args+=['--execution-json',json.dumps(cfg['execution'])]
        if cfg.get('cup_contact_preset') is not None:args+=['--cup-contact-preset',json.dumps(cfg['cup_contact_preset'])]
        args=clean_command([HOST_PYTHON,'-I','-B','-c',AFFINITY_EXEC,*args],isaac_environment(cfg,'dsw-scene'))
        lifetime='persistent' if spec.get('persistent_service') else spec['max_lifetime_seconds']
        supervisor=(python_command(LOCAL_SUPERVISOR,json.dumps(args),lifetime,'hold-stdin',cfg['workbench_code'])
            if cfg.get('execution') else python_command(SUPERVISOR,json.dumps(args),lifetime,'hold-stdin'))
        if cfg.get('execution'):
            from local_execution import check_command
            guard=check_command(cfg,exclude='scene')+' 1>&2 && '
        else:
            from detector_service import gpu_check_command
            guard=gpu_check_command(cfg,'dsw-scene')+' 1>&2 && '
        command=guard+python_command(INSTALL_SPEC,json.dumps(pinned_files(contract)),spec['spec_sha256'],'resume' if resume else 'new')+' && '+supervisor
        Path(directory).mkdir(parents=True,exist_ok=True)
        self.log=(Path(directory)/'scene-service.log').open('a')
        self.generation+=1;generation=self.generation
        def create():
            self.process=subprocess.Popen(transport(cfg,'dsw-scene',command),stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=self.log,text=True,bufsize=1)
            return self.process
        try:process=launch(create) if launch is not None else create()
        except BaseException:
            self.log.close();self.log=None;raise
        self.process.stdin.write(payload.decode()+'\n');self.process.stdin.flush()
        self.config=cfg;self.config_path=Path(config_path);self.directory=Path(directory);self.closed=False
        self.state=dict(status='starting',model_initialized=False)
        log=self.log
        def read_state():
            try:
                for line in process.stdout:
                    log.write(line);log.flush();value=json.loads(line)
                    state=value.get('service')
                    if not isinstance(state,dict) or any(state.get(k)!=v for k,v in spec.items()) or 'owner_token' in state:
                        raise ValueError('scene startup state ownership differs')
                    with self.lock:
                        if generation==self.generation and not self.closed:self.state=state
            except Exception as error:
                with self.lock:
                    if generation==self.generation and not self.closed:self.state=dict(status='failed',model_initialized=False,error=str(error))
                if process.stdin and not process.stdin.closed:process.stdin.close()
        self.thread=threading.Thread(target=read_state,daemon=True);self.thread.start()
