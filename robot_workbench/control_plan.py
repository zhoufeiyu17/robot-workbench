"""Private CPU plan compiler. Only sealed fixed rows and small metadata reach Owner.

Raw plan/source/timeline files are audit inputs for non-control workers. Loading
those artifacts is deliberately separate from the per-tick read-only row view.
"""
from collections.abc import Sequence
from copy import deepcopy
import fcntl
import hashlib
import json
import math
import mmap
import multiprocessing
from multiprocessing.connection import Connection
from multiprocessing.reduction import recv_handle, send_handle
import os
from pathlib import Path
import re
import secrets
import signal
import stat
import struct
import subprocess
import sys
import tempfile
import time

MAX_INPUT=16*1024*1024
MAX_METADATA=256*1024
HEADER=struct.Struct('<8s16sQ')
ROW=struct.Struct('<15dqBBII')
MAGIC=b'RWPLAN01'
SEALS=(getattr(fcntl,'F_SEAL_WRITE',8)|getattr(fcntl,'F_SEAL_GROW',4)|
       getattr(fcntl,'F_SEAL_SHRINK',2)|getattr(fcntl,'F_SEAL_SEAL',1))


class RawPreview(dict):
    def __init__(self, raw):
        if not isinstance(raw,bytes) or not 0<len(raw)<=MAX_INPUT:raise ValueError('bounded raw preview required')
        super().__init__(action='preview_frozen');self.raw=raw


class ArtifactPreview(dict):
    def __init__(self, *, plan_path, source_path):
        super().__init__(action='preview_frozen')
        self.paths={name:str(Path(path).absolute()) for name,path in (('plan',plan_path),('source',source_path))}


def preview_request(raw):
    """Small operator messages stay independent; large preview JSON is decoded in spawn."""
    if not isinstance(raw,bytes) or not 0<len(raw)<=MAX_INPUT:raise ValueError('bounded operator JSON required')
    if len(raw)<=64*1024:return json.loads(raw)
    if not re.match(rb'\s*\{\s*"action"\s*:\s*"preview_frozen"\s*,',raw):
        # JSON key order/whitespace cannot change an operator action. Unusual
        # large envelopes are classified off-process without returning the plan.
        with tempfile.TemporaryDirectory(prefix='control-json-') as root:
            value=_run('decode',raw,dict(owner_pid=os.getpid(),runtime_nonce=None,epoch=None),root,current=lambda:True)
        if value.get('action')!='preview_frozen':return value
    return RawPreview(raw)


def validate_python_input(value):
    # Compatibility is bounded before serialization. Production uses raw/ref.
    if (not isinstance(value,dict) or set(value)!={'action','plan','source'}
            or value.get('action')!='preview_frozen' or not isinstance(value.get('plan'),dict)
            or len(value['plan'].get('ticks',[]))>512):
        raise ValueError('large Python plans require raw preview bytes or artifact input')
    pending=[value];nodes=size=0
    while pending:
        item=pending.pop();nodes+=1
        if nodes>50000:raise ValueError('large Python plans require raw preview bytes or artifact input')
        if isinstance(item,dict):
            if nodes+len(pending)+2*len(item)>50000:raise ValueError('large Python plans require raw preview bytes or artifact input')
            pending.extend(item.keys());pending.extend(item.values())
        elif isinstance(item,(list,tuple)):
            if nodes+len(pending)+len(item)>50000:raise ValueError('large Python plans require raw preview bytes or artifact input')
            pending.extend(item)
        elif isinstance(item,str):size+=len(item)
        else:size+=16
        if size>512*1024:raise ValueError('large Python plans require raw preview bytes or artifact input')


def _python_input(value):
    validate_python_input(value)
    return json.dumps(value,allow_nan=False).encode()


def _read_file(path, maximum=MAX_INPUT):
    if not Path(path).is_absolute():raise ValueError('absolute artifact path required')
    fd=os.open(path,os.O_RDONLY|os.O_NOFOLLOW)
    with os.fdopen(fd,'rb') as stream:
        before=os.fstat(stream.fileno())
        if not stat.S_ISREG(before.st_mode) or not 0<before.st_size<=maximum:raise ValueError('bounded regular artifact required')
        raw=stream.read(maximum+1);after=os.fstat(stream.fileno())
    if (len(raw)!=before.st_size or (before.st_ino,before.st_size,before.st_mtime_ns)!=
            (after.st_ino,after.st_size,after.st_mtime_ns)):raise ValueError('artifact changed during read')
    return raw


def load_artifact(reference):
    """Full decode/hash belongs ONLY in non-realtime audit/recording/compiler workers."""
    if not isinstance(reference,dict) or set(reference)!={'path','sha256','size_bytes'}:
        raise ValueError('exact immutable artifact reference required')
    raw=_read_file(reference['path'],128*1024*1024)
    if len(raw)!=reference['size_bytes'] or hashlib.sha256(raw).hexdigest()!=reference['sha256']:
        raise ValueError('artifact size or SHA256 differs from validated reference')
    return json.loads(raw)


def _save(root,name,value):
    raw=json.dumps(value,allow_nan=False,separators=(',',':')).encode();path=Path(root)/name
    with path.open('xb') as stream:stream.write(raw)
    path.chmod(0o400)
    return dict(path=str(path),sha256=hashlib.sha256(raw).hexdigest(),size_bytes=len(raw))


class Column(Sequence):
    def __init__(self,rows,name):self.rows,self.name=rows,name
    def __len__(self):return len(self.rows)
    def __getitem__(self,index):
        if isinstance(index,slice):return [self[i] for i in range(*index.indices(len(self))) ]
        if self.name=='times':return self.rows.values(index)[0]
        return bool(self.rows.values(index)[16 if self.name=='holding_ticks' else 17])
    def __deepcopy__(self,memo):return self


class Rows(Sequence):
    def __init__(self,fd,metadata,nonce):
        self.fd=None;self.memory=None
        try:
            if (not isinstance(metadata,dict) or type(metadata.get('row_count')) is not int
                    or not isinstance(metadata.get('names'),list) or not metadata['names']
                    or not all(isinstance(name,str) for name in metadata['names'])):
                raise ValueError('invalid control array metadata')
            size=os.fstat(fd).st_size
            if sys.platform.startswith('linux'):
                if fcntl.fcntl(fd,fcntl.F_GET_SEALS)&SEALS!=SEALS:raise ValueError('control array is not fully sealed')
            elif fcntl.fcntl(fd,fcntl.F_GETFL)&os.O_ACCMODE!=os.O_RDONLY:
                raise ValueError('CPU fallback requires a read-only anonymous descriptor')
            if not HEADER.size<size<=128*1024*1024:raise ValueError('invalid control array size')
            # Some container kernels reject shared mappings of F_SEAL_WRITE
            # memfds even with PROT_READ. Private read-only mappings retain
            # the same sealed pages and never permit copy-on-write mutation.
            memory=mmap.mmap(fd,size,flags=mmap.MAP_PRIVATE,prot=mmap.PROT_READ)
            magic,identity,count=HEADER.unpack_from(memory)
            if magic!=MAGIC or identity.hex()!=nonce or size!=HEADER.size+count*ROW.size or count!=metadata['row_count']:
                memory.close();raise ValueError('control array identity or dimensions differ')
            self.fd,self.memory,self.count,self.names=fd,memory,count,tuple(metadata['names'])
            self.immutability='linux_sealed_memfd' if sys.platform.startswith('linux') else 'cpu_only_readonly_anonymous_fd'
        except BaseException:os.close(fd);raise
    def __len__(self):return self.count
    def values(self,index):
        if index<0:index+=len(self)
        if not 0<=index<len(self):raise IndexError(index)
        values=ROW.unpack_from(self.memory,HEADER.size+index*ROW.size)
        if (values[18]>=len(self.names) or values[19]>=len(self.names)
                or not all(math.isfinite(value) for value in values[:15])):
            raise ValueError('invalid sealed control row or stage index')
        return values
    def __getitem__(self,index):
        if isinstance(index,slice):return [self[i] for i in range(*index.indices(len(self)))]
        if index<0:index+=len(self)
        values=self.values(index);q=values[1:15]
        target=dict(schema_version='physical14.v1',encoding='absolute',units={'joints':'rad','grippers':'mm'},
            left_arm_joints=list(q[:6]),left_gripper=q[6],right_arm_joints=list(q[7:13]),right_gripper=q[13])
        return dict(tick=index,time_s=values[0],target=target,source_tick=None if values[15]<0 else values[15],
            holding=bool(values[16]),unloading=bool(values[17]),stage=self.names[values[18]],substage=self.names[values[19]])
    def __deepcopy__(self,memo):return self
    def __del__(self):
        if self.memory is not None:self.memory.close()
        if self.fd is not None:os.close(self.fd)


def _rows(commands,nonce):
    linux=sys.platform.startswith('linux');temporary=None
    if linux:
        fd=os.memfd_create('workbench-validated-plan',os.MFD_ALLOW_SEALING|os.MFD_CLOEXEC)
    else:
        fd,temporary=tempfile.mkstemp(prefix='control-plan-cpu-')
    names=[];indices={}
    def name(value):
        if not isinstance(value,str):raise ValueError('command stage must be text')
        if value not in indices:indices[value]=len(names);names.append(value)
        return indices[value]
    try:
        with os.fdopen(os.dup(fd),'wb') as stream:
            stream.write(HEADER.pack(MAGIC,bytes.fromhex(nonce),len(commands)))
            for row in commands:
                q=row['target'];values=[*q['left_arm_joints'],q['left_gripper'],*q['right_arm_joints'],q['right_gripper']]
                stream.write(ROW.pack(row['time_s'],*values,-1 if row['source_tick'] is None else row['source_tick'],
                    row['holding'],row['unloading'],name(row['stage']),name(row['substage'])))
        if linux:fcntl.fcntl(fd,fcntl.F_ADD_SEALS,SEALS)
        else:
            os.close(fd);fd=os.open(temporary,os.O_RDONLY);os.unlink(temporary);temporary=None
        return fd,dict(row_count=len(commands),names=names,immutability='linux_sealed_memfd' if linux else 'cpu_only_readonly_anonymous_fd')
    except BaseException:
        os.close(fd)
        if temporary is not None:os.unlink(temporary)
        raise


def _timeline_metadata(timeline):
    return {key:deepcopy(value) for key,value in timeline.items() if key not in ('commands','times','holding_ticks','unloading_ticks')}


def _worker(channel,parent_pid,operation,request,context,root,nonce):
    if sys.platform.startswith('linux'):
        import ctypes
        libc=ctypes.CDLL(None,use_errno=True)
        if libc.prctl(1,signal.SIGKILL,0,0,0)!=0:raise OSError(ctypes.get_errno(),'compiler parent lifetime guard failed')
    if os.getppid()!=parent_pid:return
    fd=None
    try:
        if operation=='decode':
            value=json.loads(request)
            if not isinstance(value,dict):raise ValueError('operator command object required')
            metadata=dict(decoded={'action':'preview_frozen'} if value.get('action')=='preview_frozen' else value)
        elif operation=='preview':
            import real_control
            if isinstance(request,bytes):command=json.loads(request)
            else:command=dict(action='preview_frozen',**{key:json.loads(_read_file(path)) for key,path in request.items()})
            if not isinstance(command,dict) or set(command)!={'action','plan','source'} or command['action']!='preview_frozen':
                raise ValueError('raw preview must contain exactly action, plan and source')
            ref=real_control.load_calibration(context['calibration_root'])
            owner_type,_,_=real_control.classes(ref,jaw_mapping_file=context['jaw_mapping_file'])
            owner=object.__new__(owner_type)
            for key in ('hardware','environment','hardware_binding','sample_clock','execution_error_settings'):setattr(owner,key,context[key])
            binding_ref=None
            if owner.hardware_binding:
                binding_ref=_save(root,'binding.json',json.loads(_read_file(owner.hardware_binding)))
                owner.hardware_binding=binding_ref['path']
            from types import SimpleNamespace
            owner.backend=SimpleNamespace(passive_jaws=tuple(context['disabled_grippers']))
            from arx_env import joint_head_jaw_mapping as jaw
            if jaw.mapping_identity()!=context['mapping_identity']:raise ValueError('compiler jaw mapping differs from live Owner')
            payload,frozen,base=owner._compile_frozen_local(command,context['limits'])
            from cup_return_replay import is_replay
            reference=dict(schema='collection_plan_ref.v1',environment=context['environment'],
                plan=_save(root,'plan.json',payload['plan']),source=_save(root,'source.json',payload['source']),
                binding=binding_ref,
                base_timeline=_save(root,'base-timeline.json',base),timeline_digest=frozen.plan['timeline_digest'],
                validation_worker_pid=os.getpid(),maintenance_replay=is_replay(payload['plan']),
                origin=dict(owner_pid=context['owner_pid'],runtime_nonce=context['runtime_nonce'],epoch=context['epoch']))
            omitted={'plan','ticks','times','holding_ticks','unloading_ticks','sink','gate','stop'}
            state={key:value for key,value in vars(frozen).items() if key not in omitted}
            plan={key:payload['plan'][key] for key in ('timeline_digest','simulation','planner_id') if key in payload['plan']}
            if 'replay_derivation' in payload['plan']:plan['replay_derivation']={'method':payload['plan']['replay_derivation']['method']}
            metadata=dict(reference=reference,frozen_state=state,plan=plan,timeline=_timeline_metadata(base))
            commands=base['commands']
        else:
            import real_control
            from execution_timeline import initialize
            if request.get('origin')!=dict(owner_pid=context['owner_pid'],runtime_nonce=context['runtime_nonce'],epoch=context['epoch']):
                raise ValueError('validated plan belongs to another Owner or epoch')
            base=load_artifact(request['base_timeline'])
            if base.get('parent_timeline_digest')!=request.get('timeline_digest'):raise ValueError('base timeline differs from validated plan')
            timeline=initialize(base,context['current_state'],context['physical_limits'],environment=context['environment'] or 'physical',
                disabled_grippers=context['disabled_grippers'],held_arms=context['held_arms'])
            ref=real_control.load_calibration(context['calibration_root'])
            _,sdk_target,_=real_control.classes(ref,jaw_mapping_file=context['jaw_mapping_file'])
            from arx_env import joint_head_jaw_mapping as jaw
            if jaw.mapping_identity()!=context['mapping_identity']:raise ValueError('compiler jaw mapping differs from live Owner')
            for row in timeline['commands']:
                if row['time_s']>timeline['initialization_duration_s']+1e-9:break
                if any(not lo<=q<=hi for q,(lo,hi) in zip(sdk_target(row['target']),context['limits'])):
                    raise ValueError('initialization command exceeds SDK joint limits')
            timeline_ref=_save(root,'execution-'+nonce+'.json',timeline)
            metadata=dict(timeline=_timeline_metadata(timeline),execution_timeline_ref=timeline_ref)
            if context.get('native_task_fixture') is not None:
                from integration import _digest
                source=load_artifact(request['source']);source_digest=_digest(source)
                try:
                    from robot_workbench.physx_task import prepare_isolated_task
                    task=prepare_isolated_task(load_artifact(request['plan']),source,context['native_task_fixture'],timeline)
                    if len(json.dumps(task,allow_nan=False).encode())>64*1024:
                        raise ValueError('native task policy exceeds bounded control metadata')
                except Exception as error:task=dict(error=str(error),source_digest=source_digest)
                task['preparation_worker_pid']=os.getpid()
                metadata['native_task']=task
            commands=timeline['commands']
        if operation!='decode':
            fd,arrays=_rows(commands,nonce);metadata['arrays']=arrays
        metadata['origin']=dict(owner_pid=context['owner_pid'],runtime_nonce=context['runtime_nonce'],epoch=context['epoch'])
        raw=json.dumps(metadata,allow_nan=False).encode()
        if len(raw)>MAX_METADATA:raise ValueError('compiled metadata exceeds bounded control envelope')
        channel.send(dict(pid=os.getpid(),parent_pid=parent_pid,nonce=nonce,metadata=raw))
        if fd is not None:send_handle(channel,fd,parent_pid)
        if not channel.poll(10) or channel.recv()!='accepted':raise ValueError('compiler handle was not accepted')
    except BaseException as error:
        try:channel.send(dict(error=str(error),type=type(error).__name__,nonce=nonce))
        except (OSError,EOFError):pass
    finally:
        if fd is not None:os.close(fd)
        channel.close()


def _verified_metadata(message,context,nonce,pid):
    if not isinstance(message,dict) or message.get('nonce')!=nonce:raise ValueError('compiler result identity mismatch')
    if 'error' in message:raise ValueError(message['error'])
    if (message.get('pid')!=pid or message.get('parent_pid')!=os.getpid()
            or not isinstance(message.get('metadata'),bytes) or len(message['metadata'])>MAX_METADATA):
        raise ValueError('compiler process or metadata identity differs')
    metadata=json.loads(message['metadata'])
    if not isinstance(metadata,dict):raise ValueError('compiler metadata object required')
    if metadata.get('origin')!=dict(owner_pid=context['owner_pid'],runtime_nonce=context['runtime_nonce'],epoch=context['epoch']):
        raise ValueError('compiler result belongs to another Owner or epoch')
    return metadata


def _run(operation,request,context,root,*,current):
    from launch_environment import clean_environment
    cpu_python=os.environ.get('WORKBENCH_CPU_PYTHON',sys.executable)
    if not Path(cpu_python).is_absolute():raise ValueError('absolute CPU compiler interpreter required')
    code=Path(__file__).resolve().parents[1]
    environment=clean_environment(dict(PYTHONPATH=os.pathsep.join(map(str,(code,code/'rule_based_workbench',code/'calibration/infra'))),
        CUDA_VISIBLE_DEVICES='',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1',NUMEXPR_NUM_THREADS='1'))
    parent,child=multiprocessing.Pipe();nonce=secrets.token_hex(16);process=None
    try:
        if not current():raise ValueError('preview revoked: owner epoch or control context changed during preview')
        with (Path(root)/('compiler-'+nonce+'.log')).open('xb') as log:
            process=subprocess.Popen([cpu_python,'-u','-B','-m','robot_workbench.control_plan','--worker-fd',str(child.fileno())],
                stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,pass_fds=(child.fileno(),),
                env=environment,start_new_session=True)
        child.close()
        if not current():raise ValueError('preview revoked: owner epoch or control context changed during preview')
        parent.send((os.getpid(),operation,request,context,str(root),nonce))
        deadline=time.monotonic()+120
        while not parent.poll(.02):
            if not current():raise ValueError('preview revoked: owner epoch or control context changed during preview')
            if process.poll() is not None:raise RuntimeError('plan compiler exited without a validated handle; inspect compiler log')
            if time.monotonic()>deadline:raise TimeoutError('plan compiler exceeded preparation deadline')
        metadata=_verified_metadata(parent.recv(),context,nonce,process.pid)
        if operation=='decode':
            parent.send('accepted');process.wait(timeout=3)
            if process.returncode!=0:raise RuntimeError('operator decoder failed')
            return metadata['decoded']
        while not parent.poll(.02):
            if not current():raise ValueError('preview revoked before control array transfer')
            if process.poll() is not None or time.monotonic()>deadline:raise RuntimeError('compiler control array transfer failed')
        fd=recv_handle(parent)
        if not current():
            os.close(fd);raise ValueError('preview revoked: owner epoch or control context changed during preview')
        rows=Rows(fd,metadata['arrays'],nonce)
        parent.send('accepted');process.wait(timeout=3)
        if process.returncode!=0:raise RuntimeError('compiler did not close after handle transfer')
        timeline=metadata['timeline'];timeline.update(commands=rows,times=Column(rows,'times'),
            holding_ticks=Column(rows,'holding_ticks'),unloading_ticks=Column(rows,'unloading_ticks'))
        if operation=='initialize':
            timeline['execution_timeline_ref']=metadata['execution_timeline_ref']
            if 'native_task' in metadata:timeline['native_task']=metadata['native_task']
            return timeline
        from frozen_executor import FrozenExecutor
        frozen=object.__new__(FrozenExecutor);frozen.__dict__.update(metadata['frozen_state'])
        for key in ('disabled_grippers','held_arms','enabled_gripper_keys'):setattr(frozen,key,tuple(getattr(frozen,key)))
        frozen.plan=dict(metadata['plan'],ticks=rows);frozen.ticks=rows;frozen.times=timeline['times']
        frozen.holding_ticks=timeline['holding_ticks'];frozen.unloading_ticks=timeline['unloading_ticks']
        frozen.sink=lambda _:None;frozen.gate=lambda:True;frozen.stop=lambda _:None
        return dict(action='preview_frozen',plan_ref=metadata['reference']),frozen,timeline
    finally:
        child.close();parent.close()
        if process is not None and process.poll() is None:
            process.terminate()
            try:process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                # This fixed CPU-only compiler has no SDK, GPU or robot transport.
                process.kill();process.wait(timeout=2)


def compile_preview(owner,command,context,*,current):
    if owner.execution_root is None:raise ValueError('isolated plans require an owned execution evidence directory')
    root=owner.execution_root.resolve()/('validated-plan-'+secrets.token_hex(12));root.mkdir(mode=0o700)
    request=command.raw if isinstance(command,RawPreview) else command.paths if isinstance(command,ArtifactPreview) else _python_input(command)
    return _run('preview',request,context,root,current=current)


def initialize(reference,context,*,current):
    if reference.get('origin')!=dict(owner_pid=context['owner_pid'],runtime_nonce=context['runtime_nonce'],epoch=context['epoch']):
        raise ValueError('validated plan belongs to another Owner or epoch')
    root=Path(reference['base_timeline']['path']).parent
    return _run('initialize',reference,context,root,current=current)


if __name__=='__main__':
    if len(sys.argv)!=3 or sys.argv[1]!='--worker-fd':raise SystemExit('private compiler pipe required')
    channel=Connection(int(sys.argv[2]))
    _worker(channel,*channel.recv())
