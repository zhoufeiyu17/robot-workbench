"""One workbench-owned persistent detector; SSH stdin lifetime owns its child."""
from __future__ import annotations
import json
from pathlib import Path, PurePosixPath
import shlex
import subprocess
import threading
import time


def validate_config(config, contract):
    s = config.get('detector_service')
    if s is None:
        return
    from run_pipeline import absolute, below, pinned_files
    required = {'root','socket','owner_token','identity','allowed_read_roots','allowed_output_root',
                'max_memory_mib','min_free_memory_mib','max_lifetime_seconds'}
    from launch_environment import persistent_service
    persistent=persistent_service(contract['resources'])
    if persistent:required.add('persistent_service')
    if not isinstance(s,dict) or set(s) != required or s != contract.get('detector_service'):
        raise ValueError('detector service must exactly match frozen contract')
    for k in ('root','socket','allowed_output_root'):
        absolute(s[k])
    root_parts=PurePosixPath(s['root']).parts
    task_tmp=(len(root_parts)>=4 and root_parts[1]=='tmp' and root_parts[2].startswith('rbw-detector-') and len(root_parts[2])>len('rbw-detector-'))
    if (s['root'] not in contract['filesystem']['writable_roots'] or
        PurePosixPath(s['socket']).parent != PurePosixPath(s['root']) or len(s['socket'].encode()) >= 104 or
        not below(config['remote_run_root'],s['allowed_output_root']) or
        not (below(s['root'],'/mnt/data/feiyu.zhou/runs') or task_tmp)):
        raise ValueError('invalid detector output/socket boundaries')
    if not isinstance(s['owner_token'],str) or len(s['owner_token']) < 16:
        raise ValueError('service ownership nonce missing')
    if not isinstance(s['allowed_read_roots'],list) or not s['allowed_read_roots']:
        raise ValueError('service read boundaries missing')
    for p in s['allowed_read_roots']:
        absolute(p)
        if p != s['allowed_output_root'] and not any(below(p,r) for r in contract['filesystem']['readonly_roots']):
            raise ValueError('service read root not authorized')
    if s['allowed_output_root'] not in s['allowed_read_roots']:
        raise ValueError('service must read capture within jobs root')
    for k in ('max_memory_mib','min_free_memory_mib'):
        if type(s[k]) is not int or s[k] <= 0:
            raise ValueError('positive integer service budgets required')
    if persistent and (s.get('persistent_service') is not True or s['max_lifetime_seconds'] is not None):
        raise ValueError('persistent detector must freeze null service lifetime')
    if not persistent and (type(s['max_lifetime_seconds']) is not int or s['max_lifetime_seconds'] < 0):
        raise ValueError('service lifetime must be nonnegative; zero means owned workbench lifetime')
    if config.get('execution') and s['max_lifetime_seconds']==0:
        raise ValueError('local detector requires a finite resident-bounded lifetime')
    if contract['resources'].get('max_workers_per_instance') != (config['execution']['max_processes'] if config.get('execution') else 2):
        raise ValueError('service needs explicit two-worker authorization')
    pins = {k:v for mapping in pinned_files(contract) for k,v in mapping.items()}
    files = {'adapter_sha256':config['detection_script'], 'service_sha256':str(PurePosixPath(config['detection_script']).with_name('isaac_detection_service.py')),
             'node_config_sha256':config['detection_node_config']}
    if set(s['identity']) != set(files) or any(s['identity'][k] != 'sha256:'+pins.get(p,'') for k,p in files.items()):
        raise ValueError('service identity must match contract file pins')


# This supervisor owns the Popen object, never searches or kills unrelated PIDs.
SUPERVISOR = r'''import json,os,select,signal,subprocess,sys,time
args=json.loads(sys.argv[1]); lifetime=None if sys.argv[2]=='persistent' else int(sys.argv[2])
assert lifetime is None or lifetime>=0,'explicit service lifetime required'
hold_stdin=len(sys.argv)>3 and sys.argv[3]=='hold-stdin'
if select.select([sys.stdin],[],[],0)[0] and not os.read(sys.stdin.fileno(),1):sys.exit(0)
child=subprocess.Popen(args,stdin=subprocess.PIPE if hold_stdin else subprocess.DEVNULL,start_new_session=True)
end=time.monotonic()+lifetime if lifetime else None
try:
 while child.poll() is None and (end is None or time.monotonic()<end):
  if select.select([sys.stdin],[],[],1)[0] and not os.read(sys.stdin.fileno(),1): break
finally:
 if hold_stdin: child.stdin.close()
 if child.poll() is None:
  os.killpg(child.pid,signal.SIGTERM)
  try: child.wait(timeout=20)
  except subprocess.TimeoutExpired: os.killpg(child.pid,signal.SIGKILL); child.wait()
sys.exit(child.returncode or 0)
'''


LOCAL_SUPERVISOR = r'''import json,os,select,signal,subprocess,sys,time
sys.path.insert(0,sys.argv[4])
from owned_process import wait_owned
args=json.loads(sys.argv[1]); lifetime=None if sys.argv[2]=='persistent' else int(sys.argv[2]); hold_stdin=sys.argv[3]=='hold-stdin'
assert lifetime is None or lifetime>0,'local worker requires an explicit lifetime'
def stop(signum,frame):raise SystemExit(128+signum)
signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
if select.select([sys.stdin],[],[],0)[0] and not os.read(sys.stdin.fileno(),1):sys.exit(0)
child=subprocess.Popen(args,stdin=subprocess.PIPE if hold_stdin else subprocess.DEVNULL,start_new_session=True)
end=None if lifetime is None else time.monotonic()+lifetime
try:
 while child.poll() is None and (end is None or time.monotonic()<end):
  if select.select([sys.stdin],[],[],1 if end is None else min(1,max(0,end-time.monotonic())))[0] and not os.read(sys.stdin.fileno(),1):break
finally:
 if hold_stdin:child.stdin.close()
 wait_owned(child,.001,grace=20)
sys.exit(child.returncode or 0)
'''


def launch_command(c):
    from run_pipeline import python_command, AFFINITY_EXEC,lane
    from launch_environment import HOST_PYTHON
    s=c['detector_service']
    # ponytail: legacy capability argv is also part of frozen GPU PID matching;
    # migrate the launcher and validator together, not during scoped recovery.
    args=['env','CUDA_VISIBLE_DEVICES='+str(lane(c,'simulated-real')['gpu']),'PYTHONPATH=','PYTHONNOUSERSITE=1','PYTHONDONTWRITEBYTECODE=1',
          'OMP_NUM_THREADS=4','MKL_NUM_THREADS=4','MAX_JOBS=4','CUDA_HOME='+c['detection_cuda_home'],
          'PATH='+c['detection_cuda_home']+'/bin:'+str(PurePosixPath(c['detection_python']).parent)+':/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin',
          HOST_PYTHON,'-I','-B','-c',AFFINITY_EXEC,c['detection_python'],'-u','-B',c['detection_script'],
          '--serve','--service-root',s['root'],'--socket',s['socket'],'--owner-token',s['owner_token'],
          '--node-config',c['detection_node_config'],'--allowed-output-root',s['allowed_output_root']]
    for root in s['allowed_read_roots']:
        args += ['--allowed-read-root',root]
    if c.get('detection_runtime_root'):
        from local_execution import detector_environment
        args[1:1]=[k+'='+v for k,v in detector_environment(c).items()]
    if c.get('execution'):
        args.insert(1,'ROBOT_WORKBENCH_SESSION_ID='+c['execution']['session_id'])
        return python_command(LOCAL_SUPERVISOR,json.dumps(args),'persistent' if s.get('persistent_service') else s['max_lifetime_seconds'],'no-stdin',c['workbench_code'])
    return python_command(SUPERVISOR,json.dumps(args),'persistent' if s.get('persistent_service') else s['max_lifetime_seconds'])


def wait_for_gpu_idle(side,*,cancel=lambda:False,progress=None,config=None,exclude=None):
    """Bounded read-only release wait; never stop another GPU occupant."""
    from run_pipeline import transport,python_command,GPU_CHECK
    deadline=time.monotonic()+30
    while True:
        if cancel():return False
        remaining=deadline-time.monotonic()
        if remaining<=0:raise RuntimeError('等待 GPU 释放超过 30 秒；未启动服务，也未停止占用进程')
        if config and config.get('execution'):
            from local_execution import check_command
            command=check_command(config,exclude=exclude)
        else:command=python_command(GPU_CHECK,0)
        checked=subprocess.run(transport(config or {},side,command),capture_output=True,text=True,timeout=remaining)
        if checked.returncode==0:return True
        detail=checked.stderr[-1200:]
        reasons=('selected GPU has a compute process','selected GPU is not idle')
        if config and config.get('execution'):reasons+=('selected GPU has an unowned process',)
        if not any(reason in detail for reason in reasons):
            raise RuntimeError('GPU preflight failed: '+detail)
        if cancel():return False
        remaining=max(0,deadline-time.monotonic())
        if progress:progress(remaining)
        time.sleep(min(1,remaining))


RETIRE_ROOT = r'''import json,os,pathlib,stat,sys,uuid
payload=sys.stdin.buffer.readline(65537);assert len(payload)<=65536,'bounded retirement request required'
c=json.loads(payload);s=c['spec'];root=pathlib.Path(s['root']);parent=root.parent
assert parent.parent==pathlib.Path('/tmp') and parent.name.startswith('rbw-detector-') and root.name=='service','unsupported detector retirement scope'
assert str(parent) in c['writable_roots'] and str(root) in c['writable_roots'],'retirement parent is not authorized'
assert parent.resolve(strict=True)==parent and not parent.is_symlink(),'detector parent is not canonical'
info=parent.stat();assert info.st_uid==os.getuid() and stat.S_IMODE(info.st_mode)==0o700,'private detector parent required'
if not root.exists() and not root.is_symlink():
 print(json.dumps(dict(retired_root=None,observation='root_absent')));sys.exit(0)
assert root.resolve(strict=True)==root and not root.is_symlink(),'detector root is not canonical'
info=root.stat();assert info.st_uid==os.getuid() and stat.S_IMODE(info.st_mode)==0o700,'private detector root required'
parent_fd=os.open(parent,os.O_RDONLY|os.O_DIRECTORY|os.O_NOFOLLOW)
root_fd=os.open(root.name,os.O_RDONLY|os.O_DIRECTORY|os.O_NOFOLLOW,dir_fd=parent_fd)
try:
 assert all(st.st_uid==os.getuid() and stat.S_IMODE(st.st_mode)==0o700 for st in (os.fstat(parent_fd),os.fstat(root_fd))),'opened retirement scope is not private'
 fd=os.open('status.json',os.O_RDONLY|os.O_NOFOLLOW,dir_fd=root_fd)
 with os.fdopen(fd,'rb') as f:
  st=os.fstat(f.fileno());assert stat.S_ISREG(st.st_mode) and st.st_uid==os.getuid() and st.st_size<=65536,'bounded private detector status required'
  raw=f.read(65537);assert len(raw)<=65536,'detector status too large'
 state=json.loads(raw)
 assert all(state.get(k)==s[k] for k in ('owner_token','identity','socket','allowed_read_roots','allowed_output_root')) and state.get('service_root')==str(root),'detector retirement identity differs'
 assert state.get('boot_id')==pathlib.Path('/proc/sys/kernel/random/boot_id').read_text().strip(),'detector boot identity unknown or changed'
 assert all(type(state.get(k))is int for k in ('gpu','logical_cuda_index','physical_gpu')) and state['gpu']==0 and state['logical_cuda_index']==0 and state['physical_gpu']==c['physical_gpu'],'detector GPU identity differs'
 pid=state.get('pid');ticks=state.get('process_start_ticks');assert type(pid)is int and pid>0 and type(ticks)is int and ticks>0,'detector process identity missing'
 proc=pathlib.Path('/proc')/str(pid);observation='process_exited'
 try:current=(proc/'stat').read_text().rsplit(')',1)[1].split()[19]
 except FileNotFoundError:
  assert not proc.exists(),'detector process status unknown'
 else:
  assert current!=str(ticks),'previous detector is still alive'
  observation='pid_reused_old_process_exited'
 def identity(st):return st.st_dev,st.st_ino
 assert identity(os.stat(parent,follow_symlinks=False))==identity(os.fstat(parent_fd)),'detector parent changed during retirement'
 assert identity(os.stat(root.name,dir_fd=parent_fd,follow_symlinks=False))==identity(os.fstat(root_fd)),'detector root changed during retirement'
 retired=root.name+'-retired-'+uuid.uuid4().hex
 assert not (parent/retired).exists(),'retirement destination exists'
 os.rename(root.name,retired,src_dir_fd=parent_fd,dst_dir_fd=parent_fd)
 count=sum(p.name.startswith(root.name+'-retired-') for p in parent.iterdir())
 print(json.dumps(dict(retired_root=str(parent/retired),retired_count=count,observation=observation)))
finally:
 os.close(root_fd);os.close(parent_fd)
'''


SERVICE_PROBE = r'''import json,pathlib,sys,time
sys.path[:0]=[sys.argv[1],str(pathlib.Path(sys.argv[1]).parent)]
from local_execution import gpu_check
raw=sys.stdin.buffer.readline(4194305);assert len(raw)<=4194304,'bounded service probe required'
c=json.loads(raw);role=sys.argv[2];assert role in ('detector','planner','scene')
spec=c[role+'_service'];path=pathlib.Path(spec['root'])/'status.json'
assert not path.is_symlink() and path.stat().st_size<=65536,'bounded service status required'
s=json.loads(path.read_bytes());assert s['status']=='ready','service not ready'
if role=='detector':
 assert all(s.get(k)==spec[k] for k in ('owner_token','identity','socket','allowed_read_roots','allowed_output_root')) and s.get('service_root')==spec['root'],'detector identity differs'
else:
 assert all(s.get(k)==v for k,v in spec.items()),'worker identity differs'
cold_scene=role=='scene' and s.get('model_initialized')is not True
resource=gpu_check(c,require=None if cold_scene else role)
if cold_scene:
 from planner_service import load_private_spec
 load_private_spec(path.parent/'private-spec.json',spec['spec_sha256'],kind='scene')
 registered=json.loads(pathlib.Path(c['execution']['roles']['scene']['registration']).read_bytes())
 assert (registered['pid'],str(registered['process_start_ticks']))==(s['pid'],str(s['process_start_ticks'])),'scene registration differs'
print(json.dumps(dict(service={k:s.get(k) for k in ('status','pid','process_start_ticks','boot_id','model_initialized')},resource=resource,checked_wall_ns=time.time_ns(),needs_warmup=role!='detector' and s.get('model_initialized')is not True)))
'''


def probe_service(config,role):
    """New operations verify actual owned workers; cached readiness is insufficient."""
    from run_pipeline import transport,python_command
    side='simulated-real' if role=='detector' else 'dsw-scene'
    public={k:v for k,v in config.items() if k!='compute_transport'}
    command=python_command(SERVICE_PROBE,config['workbench_code'],role)
    result=subprocess.run(transport(config,side,command),input=json.dumps(public)+'\n',capture_output=True,text=True,timeout=25)
    if result.returncode:
        detail=result.stderr[-1200:];token=(config.get('detector_service') or {}).get('owner_token')
        if token:detail=detail.replace(token,'<redacted>')
        raise RuntimeError('计算服务实时核验失败；保留当前服务：'+detail)
    return json.loads(result.stdout)


class DetectorService:
    def __init__(self):
        self._lock=threading.Lock(); self._process=None; self._closed=False; self._started=time.monotonic(); self._initialization_elapsed=None
        self._generation=0;self._thread=None
        self._state={'status':'stopped','stage':'not-started','message':'检测服务尚未启动','progress':0,'elapsed_s':0,'eta_s':None,'error':None}

    def start(self, config_path, state_directory,*,launch=None):
        from run_pipeline import load_config
        c,contract,_=load_config(Path(config_path))
        self._local=bool(c.get('execution'))
        if not c.get('detector_service'):
            self._state.update(status='ready',stage='legacy',message='按任务初始化检测器',progress=100)
            return
        if self._process is not None or getattr(self,'_thread',None) is not None:
            raise RuntimeError('detector service already started')
        self.config,self.contract=c,contract
        self.config_path,self.directory=Path(config_path),Path(state_directory)
        self._closed=False;self._generation+=1;self._initialization_elapsed=None
        self._started=time.monotonic(); self._state.update(status='starting',stage='preflight',message='正在核验检测器输入与 GPU',progress=0,error=None)
        directory=Path(state_directory); directory.mkdir(parents=True,exist_ok=True)
        self._thread=threading.Thread(target=self._run,args=(c,contract,directory,self._generation,launch),daemon=True)
        self._thread.start()

    def _run(self,c,contract,directory,generation=None,launch=None):
        from run_pipeline import transport,python_command,GPU_CHECK,pinned_files
        check=r'''import hashlib,json,pathlib,sys
c=json.loads(sys.argv[1]); s=c['detector_service']; assert not pathlib.Path(s['root']).exists(),'service root already exists'
assert pathlib.Path(s['root']).parent.is_dir(),'service parent absent'
for pins in json.loads(sys.argv[2]):
 for name,sha in pins.items():
  h=hashlib.sha256()
  with open(name,'rb') as f:
   for b in iter(lambda:f.read(1048576),b''): h.update(b)
  assert h.hexdigest()==sha,'service input hash mismatch '+name
'''
        generation=self._generation if generation is None else generation
        token=c['detector_service']['owner_token']
        def current():return generation==self._generation and not self._closed
        def spawn(callback):return launch(callback) if launch is not None else callback()
        try:
            if not self._wait_for_gpu(c):return
            if c.get('execution'):
                from local_execution import check_command
                guard=check_command(c,exclude='detector')
            else:guard=python_command(GPU_CHECK,0)
            command=guard+' && '+python_command(check,json.dumps(c),json.dumps(pinned_files(contract)))+' && '+('exec ' if c.get('execution') else '')+launch_command(c)
            with (directory/'detector-service.stderr.log').open('a') as err, (directory/'detector-service.stdout.log').open('a') as log:
                def create():
                    with self._lock:
                        if not current():raise InterruptedError('detector startup revoked')
                        self._process=subprocess.Popen(transport(c,'simulated-real',command),stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=err,text=True,bufsize=1)
                        return self._process
                process=spawn(create)
                budget_verified=False
                for line in process.stdout:
                    log.write(line.replace(token,'<redacted>')); log.flush()
                    try: data=json.loads(line)
                    except (ValueError,TypeError):continue
                    if not isinstance(data,dict) or 'initialization' not in data:continue
                    spec=c['detector_service']
                    if any(data.get(k)!=spec[v] for k,v in [('owner_token','owner_token'),('identity','identity'),('service_root','root'),('socket','socket')]):
                        raise RuntimeError('Detector status ownership mismatch')
                    if data.get('status')=='ready' and not budget_verified:
                        checked=subprocess.run(transport(c,'simulated-real',gpu_check_command(c,'simulated-real')),capture_output=True,text=True,timeout=20)
                        if checked.returncode:raise RuntimeError('Detector resource preflight failed: '+checked.stderr[-1200:])
                        budget_verified=True
                    state=dict(data['initialization']); state['status']=data.get('status',state.get('status','starting'))
                    state={key:value.replace(token,'<redacted>') if isinstance(value,str) else value for key,value in state.items()}
                    state.update({key:data[key] for key in ('pid','process_start_ticks','boot_id') if key in data})
                    state.setdefault('progress',state.get('progress_percent'))
                    with self._lock:
                        if not current():continue
                        if state['status']=='ready' and self._initialization_elapsed is None:
                            self._initialization_elapsed=state.get('elapsed_s',round(time.monotonic()-self._started,1))
                        if self._initialization_elapsed is not None:
                            state['elapsed_s']=self._initialization_elapsed
                        self._state=state
                code=process.wait()
                if current():
                    detail=self.status().get('error')
                    raise RuntimeError(detail or 'Detector SSH exited with status '+str(code))
        except Exception as exc:
            if 'process' in locals() and process.stdin and not process.stdin.closed:process.stdin.close()
            with self._lock:
                if current():self._state.update(status='failed',stage='failed',message=str(exc).replace(token,'<redacted>'),error=str(exc).replace(token,'<redacted>'))

    def _wait_for_gpu(self,config=None):
        config=config or {}
        """Allow prior shutdown to finish; never kill occupants or retry model launch."""
        def cancelled():
            with self._lock:
                return self._closed
        def progress(remaining):
            with self._lock:
                self._state.update(status='starting',stage='preflight',progress=0,error=None,eta_s=remaining,
                                   message=f"检查本 Session GPU{config['execution']['gpu_index']} 进程与预算（最多 30 秒，不停止其他进程）" if config.get('execution') else
                                   '等待 GPU 释放（最多 30 秒，只读检查，不停止占用进程）')
        return wait_for_gpu_idle('simulated-real',cancel=cancelled,progress=progress,
            **({'config':config,'exclude':'detector'} if config.get('execution') else {}))

    def status(self):
        with self._lock:
            result=dict(self._state)
        if result.get('stage')=='preflight':result['elapsed_s']=round(time.monotonic()-self._started,1)
        result.setdefault('elapsed_s',round(time.monotonic()-self._started,1))
        return result

    def probe(self):return probe_service(self.config,'detector')

    def close(self):
        with self._lock:
            self._closed=True; self._generation+=1;process=self._process
            self._state.update(status='stopped',stage='stopped',message='检测服务已停止')
        if process is not None:
            if process.stdin and not process.stdin.closed:process.stdin.close()
            try:process.wait(timeout=30)
            except subprocess.TimeoutExpired:
                process.terminate()
                try:process.wait(timeout=30 if getattr(self,'_local',False) else 5)
                except subprocess.TimeoutExpired:process.kill();process.wait(timeout=5)
        if self._thread is not None:
            self._thread.join(timeout=5)
            if self._thread.is_alive():
                with self._lock:self._state.update(status='unknown',error='检测服务读取线程未退出；保留旧入口，不启动新进程')
                raise TimeoutError(self._state['error'])

    def recover(self,*,progress=None,timeout=180,restart=False,active=lambda:True,launch=None):
        deadline=time.monotonic()+timeout
        while True:
            if not active():raise InterruptedError('检测服务恢复已撤销')
            state=self.status();status=state['status']
            if status=='ready':
                if not restart:return state
                break
            if status in ('failed','stopped'):break
            if status=='unknown':
                if (not self._closed or not hasattr(self,'config') or self._thread is not None and self._thread.is_alive()
                        or self._process is not None and self._process.poll()is None):
                    raise RuntimeError(state.get('error') or '检测服务状态未知，保留原进程')
                break  # A new request may reverify an already retired local scope.
            if time.monotonic()>=deadline:raise TimeoutError('检测服务仍在处理请求，未停止或重启')
            if progress:progress('等待当前检测请求结束',next_label='恢复检测服务')
            time.sleep(.05)
        self.close()
        if not active():raise InterruptedError('检测服务退役后操作已撤销，未启动新进程')
        from run_pipeline import transport,python_command,lane
        payload=dict(spec=self.config['detector_service'],physical_gpu=lane(self.config,'simulated-real')['gpu'],
            writable_roots=self.contract['filesystem']['writable_roots'])
        try:
            retired=subprocess.run(transport(self.config,'simulated-real',python_command(RETIRE_ROOT)),
                input=json.dumps(payload)+'\n',text=True,capture_output=True,timeout=20)
        except (OSError,subprocess.TimeoutExpired) as error:
            with self._lock:self._state.update(status='unknown',error='检测服务退役核验未知：'+str(error)[:500])
            raise RuntimeError(self._state['error']) from error
        if retired.returncode:
            with self._lock:self._state.update(status='unknown',error='检测服务退役未核验；保留旧目录：'+retired.stderr[-900:])
            raise RuntimeError(self._state['error'])
        self.last_retirement=json.loads(retired.stdout)
        if not active():raise InterruptedError('检测服务退役后操作已撤销，未启动新进程')
        self._process=self._thread=None
        self.start(self.config_path,self.directory,launch=launch)
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            if not active():
                self.close();raise InterruptedError('检测服务启动后操作已撤销；仅收尾本次启动')
            state=self.status()
            if state['status']=='ready':return state
            if state['status'] in ('failed','unknown'):raise RuntimeError(state.get('error') or '检测服务恢复失败')
            time.sleep(.05)
        raise TimeoutError('检测服务恢复超时；保留当前入口与证据')

OWNED_GPU_CHECK = r'''import json,pathlib,subprocess,sys
c=json.loads(sys.argv[1]); s=c['detector_service']; state=json.loads((pathlib.Path(s['root'])/'status.json').read_text())
assert state['status']=='ready','detector service is not ready'
assert state['owner_token']==s['owner_token'] and state['identity']==s['identity'],'detector ownership/identity mismatch'
assert state['socket']==s['socket'] and state['service_root']==s['root'] and state['gpu']==0,'detector endpoint mismatch'
assert state['allowed_read_roots']==s['allowed_read_roots'] and state['allowed_output_root']==s['allowed_output_root'],'detector path boundaries mismatch'
pid=int(state['pid']); proc=pathlib.Path('/proc')/str(pid)
assert (proc/'stat').read_text().rsplit(')',1)[1].split()[19]==str(state['process_start_ticks']),'detector PID was reused'
cmd=(proc/'cmdline').read_bytes().split(b'\0'); expected=[c['detection_script'],'--serve','--owner-token',s['owner_token'],'--service-root',s['root']]
assert all(v.encode() in cmd for v in expected),'detector command identity mismatch'
line=subprocess.check_output(['nvidia-smi','-i','0','--query-gpu=uuid,memory.free','--format=csv,noheader,nounits'],text=True).strip()
uuid,free=[v.strip() for v in line.split(',')]
rows=subprocess.check_output(['nvidia-smi','--query-compute-apps=gpu_uuid,pid,used_gpu_memory','--format=csv,noheader,nounits'],text=True)
selected=[list(map(str.strip,row.split(','))) for row in rows.splitlines() if row.split(',')[0].strip()==uuid]
assert len(selected)==1 and int(selected[0][1])==pid,'selected GPU has an unowned process or detector is absent'
assert int(selected[0][2])<=s['max_memory_mib'],'detector exceeds memory budget'
assert int(free)>=s['min_free_memory_mib'],'insufficient free memory reserved for Isaac'
print(json.dumps({'detector_owned':True,'gpu':0,'memory_mib':int(selected[0][2]),'free_mib':int(free)}))
'''


def gpu_check_command(config, side):
    from run_pipeline import python_command,GPU_CHECK,PIPELINE_LANES
    if config.get('execution'):
        from local_execution import check_command
        role='detector' if side=='simulated-real' else 'planner'
        return check_command(config,require=role if config.get(role+'_service') else None)
    if side=='simulated-real' and config.get('detector_service'):
        return python_command(OWNED_GPU_CHECK,json.dumps(config))
    if side=='dsw-scene' and config.get('planner_service'):
        from planner_service import OWNED_GPU_CHECK as planner_check,private_spec_arguments
        return python_command(planner_check,json.dumps(private_spec_arguments(config)))
    return python_command(GPU_CHECK,PIPELINE_LANES[side]['gpu'])
