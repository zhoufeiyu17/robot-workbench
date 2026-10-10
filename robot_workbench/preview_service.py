"""Owned lifetime of the existing C display renderer; no Owner control calls."""
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import secrets
import socket
import subprocess
import threading
import time

from robot_workbench.target_preview import RendererClient,TargetPreview,PendingFrame

ROOT=Path(__file__).resolve().parents[1]


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write(path,value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True,mode=0o700)
    temporary=path.with_suffix('.partial')
    with temporary.open('w') as stream:json.dump(value,stream,allow_nan=False,indent=2)
    temporary.chmod(0o600);temporary.replace(path)


def resident_execution(contract,output,*,proc_root='/proc'):
    from local_execution import TARGET
    root=Path(output).resolve();details=contract['collection_setup'];pid=os.getpid();proc=Path(proc_root)/str(pid)
    script=str(ROOT/'scripts/serve_physx_device.py')
    value=dict(TARGET,session_id=contract['identity']['session_id'],max_processes=contract['resources']['max_workers'],
        min_free_memory_mib=details['min_free_memory_mib'],plant=dict(session_id=contract['identity']['session_id'],pid=pid,
            process_start_ticks=(proc/'stat').read_text().rsplit(')',1)[1].split()[19],scope=str(root),
            script=script,script_sha256=sha(script),argv=[v.decode() for v in (proc/'cmdline').read_bytes().split(b'\0') if v],
            max_memory_mib=details['plant_max_memory_mib']))
    if value['max_processes']==5:value['roles']=json.loads((root/'preview/roles.json').read_text())
    return value


def prepare_preview_roles(contract,output,calibration_config):
    """Freeze private paths and inputs only when five roles were explicitly declared."""
    if contract['resources']['max_workers']!=5:raise ValueError('preview requires an explicit five-role contract')
    details=contract['preview_setup'];root=Path(output).resolve();directory=root/'preview'
    directory.mkdir(mode=0o700,exist_ok=False)
    config=deepcopy(calibration_config);config.update(display_only=True,mesh_rendering_enabled=False)
    # This renderer is display geometry, never another observation producer/controller.
    for key in ('hardware_control','hardware_camera_port','hardware_wrist_camera_ports','live_backend','wrist_simulation_cameras'):
        config.pop(key,None)
    config_path=directory/'config.json';write(config_path,config)
    token=directory/'token'
    with os.fdopen(os.open(token,os.O_CREAT|os.O_EXCL|os.O_WRONLY,0o600),'w') as stream:stream.write(secrets.token_urlsafe(32))
    port=details['port']
    if port==0:
        with socket.socket() as probe:probe.bind(('127.0.0.1',0));port=probe.getsockname()[1]
    script=ROOT/'calibration/examples/scripts/debug/joint_head_workbench_live.py';wb=ROOT/'rule_based_workbench'
    entries={str(wb/name):sha(wb/name) for name in ('a1_isaac_assets.py','playground_sim.py','playground_geometry.py','motiongen_plan.py','isaac_lane.py')}
    roles=dict(preview=dict(root=str(directory),registration=str(root/'gpu-roles/preview.json'),script=str(script),script_sha256=sha(script),
        config_path=str(config_path),config_sha256=sha(config_path),token_file=str(token),port=port,max_memory_mib=details['max_memory_mib']),
        auxiliary=dict(root=str(root),registration=str(root/'gpu-roles/auxiliary.json'),entrypoints=entries,max_memory_mib=details['auxiliary_max_memory_mib']))
    write(directory/'roles.json',roles);return roles


class PreviewService:
    """One explicit start/recover at a time, owned child cleanup, cached UI state."""
    def __init__(self,*,execution,deadline_wall_ns,deps,mesh_inputs,validate_pose,admission_config=None):
        from local_execution import validate
        validate({'execution':execution})
        if execution['max_processes']!=5:raise ValueError('dedicated preview requires the five-role contract')
        self.execution=deepcopy(execution);self.spec=execution['roles']['preview'];self.deadline=deadline_wall_ns
        self.deps=deps;self.mesh_inputs=mesh_inputs;self.validate_pose=validate_pose
        self.admission_config=admission_config or (lambda:{'execution':self.execution})
        self.lock=threading.Lock();self.closed=threading.Event();self.thread=None;self.process=None;self.adapter=None
        self.status='configured';self.error=None;self.log=None
        self.requested={}
        self.display_context=None

    def start(self):
        with self.lock:
            if self.thread and self.thread.is_alive():raise ValueError('renderer start/recovery already running')
            if self.closed.is_set():raise ValueError('renderer lifetime is closed')
            self.status='starting';self.error=None
            self.thread=threading.Thread(target=self._start,daemon=True,name='preview-service');self.thread.start()
        return {'queued':True,'message':'仅恢复显示渲染器；设备控制与实际相机保持独立'}

    def recover(self):
        if not self.state()['recovery']['allowed']:raise ValueError('renderer recovery is not currently allowed')
        return self.start()

    def _stop_owned(self):
        adapter=self.adapter;self.adapter=None
        if adapter:
            with adapter.lock:
                self.requested=deepcopy(dict(view=adapter.view,mesh=adapter.mesh_selection))
            adapter.close()
        process=self.process
        if process and process.poll() is None:
            process.terminate()
            try:process.wait(timeout=30)
            except subprocess.TimeoutExpired:process.kill();process.wait(timeout=5)
        self.process=None
        if self.log:self.log.close();self.log=None

    def launch(self,budget,output):
        spec=self.spec
        args=['/isaac-sim/python.sh','-u','-B',spec['script'],'--config',spec['config_path'],
            '--output',str(output),'--token-file',spec['token_file'],'--port',str(spec['port']),
            '--render-gpu','1','--duration-s',str(budget),'--display-only','--execution-json',json.dumps(self.execution)]
        from launch_environment import owned_command
        return owned_command(ROOT/'rule_based_workbench',args,budget)

    def _start(self):
        try:
            self._stop_owned()
            if self.closed.is_set():return
            budget=min(14400.,(self.deadline-time.time_ns())/1e9-45)
            if budget<=0:raise TimeoutError('resident deadline leaves no renderer/cleanup budget')
            from local_execution import gpu_check
            configuration=self.admission_config()
            if configuration.get('execution')!=self.execution:raise ValueError('renderer and collection resource contracts differ')
            gpu_check(configuration,exclude='preview')
            if sha(self.spec['script'])!=self.spec['script_sha256'] or sha(self.spec['config_path'])!=self.spec['config_sha256']:
                raise ValueError('frozen renderer inputs changed')
            output=Path(self.spec['root'])/('frames-'+secrets.token_hex(8))
            self.log=(output.parent/(output.name+'.log')).open('xb')
            from launch_environment import clean_environment,clean_command,owned_command
            env={'CUDA_VISIBLE_DEVICES':'1','ROBOT_WORKBENCH_SESSION_ID':self.execution['session_id'],
                'PYTHONPATH':':'.join((str(ROOT/'rule_based_workbench'),str(ROOT/'calibration/infra'),str(ROOT/'calibration'),self.deps)),
                'PYTHONNOUSERSITE':'1','OMP_NUM_THREADS':'4','MKL_NUM_THREADS':'4','OPENBLAS_NUM_THREADS':'1'}
            command=self.launch(budget,output)
            cut=command.index('--')
            command=owned_command(ROOT/'rule_based_workbench',clean_command(command[cut+1:],env),budget)
            self.process=subprocess.Popen(command,env=clean_environment(),stdout=self.log,stderr=subprocess.STDOUT)
            ready_deadline=min(time.monotonic()+120,time.monotonic()+budget)
            while time.monotonic()<ready_deadline and not self.closed.wait(.1):
                if self.process.poll() is not None:raise RuntimeError('owned renderer exited before readiness')
                path=Path(self.spec['registration'])
                if not path.is_file():continue
                registered=json.loads(path.read_text())
                if str(output) not in registered.get('argv',[]) or registered.get('status')!='ready':continue
                gpu_check(self.admission_config(),require='preview')
                client=RendererClient('http://127.0.0.1:'+str(self.spec['port']),self.spec['token_file'])
                state=client.request('/state')
                if state.get('renderer_instance_id')!=registered.get('renderer_instance_id'):
                    raise ValueError('HTTP renderer differs from the registered GPU process')
                if state.get('capabilities',{}).get('execute') or state.get('capabilities',{}).get('capture'):
                    raise ValueError('dedicated renderer unexpectedly exposes execution')
                adapter=TargetPreview(client,state['renderer_instance_id'],validate_pose=self.validate_pose,mesh_inputs=self.mesh_inputs)
                requested=self.requested
                if requested.get('view'):adapter.overview(**requested['view'])
                if requested.get('mesh'):adapter.mesh(requested['mesh']['enabled'],camera=requested['mesh']['camera'] or 'head',candidate_id=requested['mesh']['candidate_id'])
                with self.lock:
                    if self.display_context:adapter.bind_context(self.display_context)
                    self.adapter=adapter;self.status='ready'
                adapter.start();return
            raise TimeoutError('renderer startup cancelled or exceeded readiness budget')
        except Exception as error:
            self._stop_owned()
            with self.lock:self.status='failed';self.error=str(error)

    def state(self):
        with self.lock:adapter=self.adapter;status=self.status;error=self.error;process=self.process
        value=adapter.state() if adapter else dict(connected=False,frames={},revisions={'target':0,'view':0,'mesh':0},
            mesh={'enabled':False,'candidate_id':None,'camera':'head'})
        if process and process.poll() is not None:status='failed';error=error or '显示渲染进程已退出';value['connected']=False
        busy=bool(self.thread and self.thread.is_alive())
        allowed=not self.closed.is_set() and not busy and (self.deadline-time.time_ns())/1e9>45
        value.update(status=status,error=error or value.get('error'),recovery=dict(allowed=allowed,
            action='preview_recover',message='可重新连接浏览器画面；若渲染服务异常，可单独恢复显示渲染器。设备控制与实际相机不重启。'))
        return value

    def current(self):
        with self.lock:adapter=self.adapter
        if adapter is None or self.process and self.process.poll() is not None:raise PendingFrame('显示渲染器尚未就绪，请查看恢复指引')
        return adapter

    def bind_context(self,display_context):
        with self.lock:
            self.display_context=display_context
            if self.adapter:self.adapter.bind_context(display_context)
    def target_pose(self,*args,**kwargs):return self.current().target_pose(*args,**kwargs)
    def overview(self,*args):return self.current().overview(*args)
    def mesh(self,*args,**kwargs):return self.current().mesh(*args,**kwargs)
    def media(self,*args,**kwargs):return self.current().media(*args,**kwargs)
    def mirror_feedback(self,*args):
        with self.lock:adapter=self.adapter
        if adapter:adapter.mirror_feedback(*args)

    def close(self):
        self.closed.set()
        if self.thread:self.thread.join(timeout=35)
        if self.thread and self.thread.is_alive():raise TimeoutError('owned preview startup cleanup still running')
        self._stop_owned()
        with self.lock:self.status='stopped'


def calibration_mesh_inputs(workbench,producer,camera,candidate_id):
    """Current producer packet + existing C candidate math; no solver/acceptance gate."""
    import numpy as np
    from arx_env.joint_head_isaac_fit import rigid
    name={'head':'base_0','left':'left_wrist_0','right':'right_wrist_0'}[camera]
    packet=producer.frames[name].snapshot();meta=packet['status']
    if not meta.get('ready') or meta.get('data_valid') is not True:raise ValueError('实际相机没有有效新帧')
    feedback=meta.get('frame_readback')
    if not isinstance(feedback,dict):raise ValueError('实际帧缺少同一物理步的关节读回')
    if camera=='head':
        row=next((item for item in workbench.calibration_results() if item['id']==candidate_id),None)
        if row is None:raise ValueError('所选头部候选不存在')
        candidate=deepcopy(row['parameters'])
    else:
        row=next((item for item in workbench.wrist_results() if item['id']==candidate_id),None)
        if row is None or workbench.wrist_overlay_error(row):raise ValueError('所选腕候选缺少完整参数')
        snapshot=workbench.wrist_contract(row)['head_snapshot'];q=feedback['qpos']
        offsets=np.concatenate([snapshot['joint_offsets_rad'][side] for side in ('left','right')])
        pose=workbench.geometry.frames(np.asarray(q[:6]+q[7:13])+offsets)[camera]['link6']@rigid(row[camera]['T_link6_camera'])
        candidate=dict(snapshot,id=candidate_id,camera_T_world_optical=pose.tolist())
    return packet['jpeg'],meta,feedback,deepcopy(producer.intrinsics[name]),candidate
