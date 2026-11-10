"""Pure CPU recording child; the controller keeps only bounded events and refs.

Private stdin/stdout pipes carry JSON, never camera pixels or compiled arrays.
The existing recorder still owns validation, codecs, manifests and failure rules.
"""
from copy import deepcopy
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import re
import secrets
import select
import shutil
import signal
import socket
import socketserver
import stat
import struct
import subprocess
import sys
import threading
import time
from .inference_trace import CaptureRun
import weakref
from .execution_settings import CAPTURE_QUEUE_BYTES,CAPTURE_MESSAGE_BYTES,recording_parameters
from .ros_data import bounded_value as _capture_value, BoundedQueue as _CaptureEvents

ROOT=Path(__file__).resolve().parents[1]
LIMIT=4*1024*1024
RECORDING_SERVICE_SCHEMA='workbench.recording-service.v6'


def _capture_spec(request_id, sources, purpose, resource_budget):
    if not isinstance(request_id,str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,128}',request_id):
        raise ValueError('bounded existing request_id required')
    if purpose not in ('data_required','infer_debug'):raise ValueError('known capture purpose required')
    if not isinstance(sources,(tuple,list)) or not 1<=len(sources)<=64:raise ValueError('1 to64 capture sources required')
    rows=[];seen=set();cameras=set()
    fields={'stream_id','kind','source_instance_id','source_clock_id','units','coordinate_frame','nominal_hz','required'}
    for source in sources:
        if not isinstance(source,dict) or not fields<=source.keys() or source.keys()-fields-{'source_ref'}:
            raise ValueError('complete source descriptor required')
        stream=source['stream_id']
        if not isinstance(stream,str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,64}',stream) or stream in seen:
            raise ValueError('unique bounded stream_id required')
        seen.add(stream)
        if source['kind'] not in ('control','feedback','state','event','video','model') or type(source['required']) is not bool:
            raise ValueError('known stream kind and required flag required')
        for key in ('source_instance_id','source_clock_id','coordinate_frame'):
            if source[key] is not None and (not isinstance(source[key],str) or not 0<len(source[key])<=200):
                raise ValueError('bounded source identity and coordinate frame required')
        rate=source['nominal_hz']
        if rate is not None and (type(rate) not in (int,float) or not math.isfinite(rate) or rate<=0):
            raise ValueError('positive nominal rate or explicit null required')
        if not isinstance(source['units'],(str,dict)):raise ValueError('explicit stream units required')
        if 'source_ref' in source:
            ref=source['source_ref']
            if isinstance(ref,dict) and ref.get('kind')=='ros':
                from .ros_recording import capture_source
                if set(ref)!={'kind','source','namespace','domain_id'}:
                    raise ValueError('exact ROS source binding required')
                expected=capture_source(ref['source'],namespace=ref['namespace'],domain_id=ref['domain_id'])
                if expected!=source:raise ValueError('ROS capture source differs from its declared stream')
                if type(ref['domain_id'])is not int or not 0<=ref['domain_id']<=232:
                    raise ValueError('explicit ROS domain required')
                rows.append(_capture_value(source)[0]);continue
            if (source['kind']!='video' or not isinstance(ref,dict)
                    or set(ref)!={'kind','descriptor_path','camera','service_instance_id'}
                    or ref['kind']!='common_camera' or ref['camera'] not in ('base_0','left_wrist_0','right_wrist_0')
                    or not isinstance(ref['descriptor_path'],str) or not Path(ref['descriptor_path']).is_absolute()
                    or not isinstance(ref['service_instance_id'],str) or not ref['service_instance_id']):
                raise ValueError('registered common-camera source reference required')
            key=(ref['descriptor_path'],ref['camera'])
            if key in cameras:raise ValueError('one capture consumer per camera source required')
            if cameras and key[0]!=next(iter(cameras))[0]:raise ValueError('one registered camera service per Run required')
            cameras.add(key)
        rows.append(_capture_value(source)[0])
    ros_sources=[r for r in rows if r.get('source_ref',{}).get('kind')=='ros']
    if ros_sources and (len(ros_sources)!=len(rows) or len({(r['source_ref']['namespace'],r['source_ref']['domain_id'])for r in rows})!=1):
        raise ValueError('ROS recording cannot mix legacy data sources or ROS domains')
    budget=dict(event_queue_bytes=CAPTURE_QUEUE_BYTES,message_bytes=CAPTURE_MESSAGE_BYTES)
    if resource_budget is not None:
        if not isinstance(resource_budget,dict) or set(resource_budget)-set(budget):raise ValueError('unknown capture resource budget')
        budget.update(resource_budget)
    budget=recording_parameters(value=dict(purpose_profile=purpose,resource_budget=budget))['resource_budget']
    return dict(schema='workbench.capture-session.v1',request_id=request_id,capture_session_id=request_id,
                source_descriptors=sorted(rows,key=lambda row:row['stream_id']),purpose_profile=purpose,resource_budget=budget)


def _continuous_source(row):
    ref=row.get('source_ref')
    return bool(ref and (ref.get('kind')!='ros' or ref['source']['kind']in('image','state')
        or ref['source']['metadata'].get('continuous_window')is True))


class _CaptureWriter:
    """Generic recording control plane; no SDK, camera ownership or control actions."""
    _sessions=weakref.WeakValueDictionary()
    _registry_lock=threading.Lock()

    @classmethod
    def create(cls,root,*,request_id,source_descriptors,purpose_profile,resource_budget=None,python=None):
        spec=_capture_spec(request_id,source_descriptors,purpose_profile,resource_budget)
        root=Path(root)
        if not root.is_absolute() or root.is_symlink():raise ValueError('absolute owned capture root required')
        root=root.resolve();root.mkdir(parents=True,exist_ok=True)
        directory=root/('capture-'+hashlib.sha256(request_id.encode()).hexdigest())
        with cls._registry_lock:
            existing=cls._sessions.get(str(directory))
            if existing is not None:
                if existing.spec!=spec:raise ValueError('capture request_id already binds a different request')
                return existing
            session=cls(directory,spec,python=python);cls._sessions[str(directory)]=session
            return session

    def __init__(self,directory,spec,*,python=None):
        from .inference_trace import write_json
        self.directory=directory;self.spec=spec;self.capture_session_id=spec['capture_session_id']
        self.python=python or os.environ.get('WORKBENCH_CPU_PYTHON') or sys.executable
        if not Path(self.python).is_absolute():raise ValueError('absolute CPU Python interpreter required')
        if directory.is_symlink():raise ValueError('capture directory must not be a symlink')
        directory.mkdir(exist_ok=True)
        self.lock=threading.RLock();self.control_lock=threading.Lock()
        self.events=_CaptureEvents(spec['resource_budget']['event_queue_bytes']);self.commands=queue.Queue()
        self.done=threading.Event();self.changed=threading.Condition(self.lock)
        self.cache=dict(state='preparing',capture_session_id=self.capture_session_id,ready=False)
        self.binding=None;self.terminal=None;self.process=None;self.thread=None
        self.camera_sources={row['stream_id']:row for row in spec['source_descriptors'] if _continuous_source(row)}
        self.last_sequence={row['stream_id']:0 for row in spec['source_descriptors'] if not _continuous_source(row)}
        self.last_time={};self.dropped={key:0 for key in self.last_sequence};self.publish_error=None
        self.publish_calls=self.publish_total_ns=self.publish_max_ns=0
        self._fd=os.open(directory/'writer.lock',os.O_CREAT|os.O_RDWR|os.O_NOFOLLOW,0o600)
        try:fcntl.flock(self._fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:
            os.close(self._fd);self._fd=None
            if json.loads((directory/'capture.json').read_text())!=spec:raise ValueError('capture request identity conflict')
            self.cache=self._read_status();self.cache['read_only']=True
            return
        header=directory/'capture.json'
        try:
            if header.exists():
                if header.is_symlink() or json.loads(header.read_text())!=spec:raise ValueError('capture request identity conflict')
                self.cache=self._read_status();self.cache['read_only']=True
                if self.cache.get('state') in ('complete','incomplete','failed'):self.done.set()
                os.close(self._fd);self._fd=None
            else:
                write_json(header,spec);self._launch()
        except BaseException:
            if self._fd is not None:os.close(self._fd);self._fd=None
            raise

    def _read_status(self):
        path=self.directory/'status.json'
        if path.exists() and not path.is_symlink():return json.loads(path.read_text())
        return dict(state='unknown',ready=False,capture_session_id=self.capture_session_id,error='writer state not available')

    def _launch(self,*,recover=False):
        self.done.clear()
        self.thread=threading.Thread(target=self._bridge,args=(recover,),daemon=True,name='capture-bridge')
        self.thread.start()

    def _error(self,error):
        with self.changed:
            self.cache.update(state='failed',ready=False,error=self.cache.get('error') or str(error));self.changed.notify_all()

    def _wait(self,predicate,timeout):
        with self.changed:
            if not self.changed.wait_for(lambda:predicate(self.cache) or self.done.is_set() or self.cache.get('error'),timeout):
                raise TimeoutError('capture response not confirmed; inspect original session')
            if not predicate(self.cache):raise ValueError(self.cache.get('error') or 'capture response not confirmed')
            return deepcopy(self.cache)

    def prepare(self,timeout=10):
        if self.cache.get('read_only'):return self.snapshot()
        return self._wait(lambda state:state.get('prepared') is True,timeout)

    def bind(self,*,run_id,business_kind,policy_hash,execution_owner=None,control_policy=None,execution_context=None,health_source=None,timeout=10):
        if (not isinstance(run_id,str) or not 0<len(run_id)<=200
                or business_kind not in ('inference','calibration','collection','research')
                or not isinstance(policy_hash,str) or not re.fullmatch('[a-f0-9]{64}',policy_hash)):
            raise ValueError('existing Run, business and policy identity required')
        binding=dict(run_id=run_id,business_kind=business_kind,policy_hash=policy_hash)
        if all(s.get('source_ref',{}).get('kind')=='ros'for s in self.spec['source_descriptors']):
            if health_source is None or health_source!=self.cache.get('health_source'):
                raise ValueError('original ROS recording health source must be confirmed at bind')
            binding['health_source']=deepcopy(health_source)
        elif health_source is not None:raise ValueError('ROS recording health source requires ROS capture')
        if control_policy is not None or execution_context is not None:
            from .execution_settings import validate_control_parameters,policy_hash as digest_policy
            if (not isinstance(control_policy,dict) or not isinstance(execution_context,dict)
                    or control_policy.get('policy_hash')!=policy_hash
                    or digest_policy(validate_control_parameters(control_policy['parameters'],
                        environment=execution_context.get('environment'),business=business_kind))!=policy_hash
                    or control_policy['parameters']['recording']!=dict(purpose_profile=self.spec['purpose_profile'],resource_budget=self.spec['resource_budget'])):
                raise ValueError('frozen Run policy/context differs from capture binding')
            expected_directory=('inference-'if business_kind=='inference'else'')+run_id
            if self.directory.parent.name!='capture' or self.directory.parent.parent.name!=expected_directory:
                raise ValueError('Run receipt requires its bound Run artifact directory')
            _capture_value(control_policy);_capture_value(execution_context)
            binding.update(json.loads(_encode(dict(control_policy=control_policy,execution_context=execution_context),CAPTURE_MESSAGE_BYTES)))
        if self.camera_sources or execution_owner is not None:
            from .sample_clock import owner_identity
            if not isinstance(execution_owner,dict) or owner_identity(execution_owner)!=execution_owner:
                raise ValueError('existing execution_owner required for camera recording')
            binding['execution_owner']=dict(execution_owner)
        with self.control_lock:
            with self.lock:
                if self.cache.get('read_only') or self.terminal is not None or self.done.is_set():raise ValueError('capture session is not writable')
                if self.binding is not None and self.binding!=binding:raise ValueError('capture binding changed')
                if self.cache.get('ready') is not True:raise ValueError('capture sources/resources not ready')
                if self.cache.get('binding')==binding:return self.snapshot()
                self.binding=binding;self.commands.put(dict(action='bind',binding=binding))
            return self._wait(lambda state:state.get('binding')==binding,timeout)

    def start(self,timeout=10):
        with self.control_lock:
            with self.lock:
                if self.cache.get('read_only') or self.terminal is not None or self.done.is_set():raise ValueError('capture session is not writable')
                if self.binding is None or self.cache.get('binding')!=self.binding:raise ValueError('capture binding not confirmed')
                if self.cache.get('state')=='recording':return self.snapshot()
                self.commands.put(dict(action='start'))
            return self._wait(lambda state:state.get('state')=='recording',timeout)

    def try_publish(self,stream_id,sequence,source_time,payload):
        started=time.monotonic_ns()
        try:return self._try_publish(stream_id,sequence,source_time,payload)
        finally:
            elapsed=time.monotonic_ns()-started
            with self.lock:
                self.publish_calls+=1;self.publish_total_ns+=elapsed;self.publish_max_ns=max(self.publish_max_ns,elapsed)

    def _try_publish(self,stream_id,sequence,source_time,payload):
        if any(row.get('source_ref',{}).get('kind')=='ros'for row in self.spec['source_descriptors']):
            raise ValueError('ROS source data cannot be injected over recording lifecycle IPC')
        with self.lock:
            if stream_id in self.camera_sources:raise ValueError('camera stream is produced by its bound source')
            if stream_id not in self.last_sequence:raise ValueError('undeclared capture stream')
            if type(sequence) is not int or sequence<=self.last_sequence[stream_id]:raise ValueError('capture sequence must increase from1')
            if type(source_time) not in (int,float) or not math.isfinite(source_time):raise ValueError('finite source timestamp required')
            if source_time<self.last_time.get(stream_id,float('-inf')):raise ValueError('source time moved backwards')
            if self.terminal is not None or self.done.is_set() or self.cache.get('state')!='recording':
                return dict(accepted=False,reason='capture_not_recording')
            self.last_sequence[stream_id]=sequence;self.last_time[stream_id]=source_time
            if stream_id not in self.cache.get('available_streams',()):
                self.dropped[stream_id]+=1;return dict(accepted=False,reason='source_unavailable')
            if self.cache.get('error'):
                self.dropped[stream_id]+=1;return dict(accepted=False,reason='writer_failed')
            try:
                value,size=_capture_value(payload,self.spec['resource_budget']['message_bytes']-256)
                message=dict(action='event',stream_id=stream_id,sequence=sequence,source_time=source_time,payload=value)
                self.events.put_nowait((size+256,message))
            except (ValueError,queue.Full) as error:
                self.dropped[stream_id]+=1;self.publish_error=str(error) or 'capture queue full'
                return dict(accepted=False,reason='gap',stream_id=stream_id,sequence=sequence)
            return dict(accepted=True)

    def finish(self,final_seq_by_stream,control_terminal,*,source_endpoints=None):
        finals=dict(final_seq_by_stream)
        if set(finals)!=set(self.last_sequence) or any(type(v) is not int or v<self.last_sequence[k] for k,v in finals.items()):
            raise ValueError('all final stream sequences must cover submitted events')
        if not isinstance(control_terminal,dict):raise ValueError('explicit control terminal object required')
        terminal,_=_capture_value(control_terminal)
        request=dict(action='finish',final_seq_by_stream=finals,control_terminal=terminal)
        if self.camera_sources or source_endpoints is not None:
            if not isinstance(source_endpoints,dict) or set(source_endpoints)!=set(self.camera_sources):
                raise ValueError('original source endpoints required for every bound camera')
            for stream,endpoint in source_endpoints.items():
                if (not isinstance(endpoint,dict) or set(endpoint)!={'source_clock_id','source_time'}
                        or endpoint['source_clock_id']!=self.camera_sources[stream]['source_clock_id']
                        or type(endpoint['source_time']) not in (int,float) or not math.isfinite(endpoint['source_time'])
                        or endpoint['source_time']<0):
                    raise ValueError('camera endpoint must use its original source clock and time')
            request['source_endpoints']=deepcopy(source_endpoints)
        with self.lock:
            if self.cache.get('read_only'):raise ValueError('existing capture requires inspection or recovery')
            if self.terminal is not None and self.terminal!=request:raise ValueError('capture finish request changed')
            self.terminal=request
            return dict(state='sealing',capture_session_id=self.capture_session_id)

    def snapshot(self):
        with self.lock:
            if self.cache.get('read_only') and self.process is None:
                self.cache=dict(self._read_status(),read_only=True)
                if self.cache.get('done'):self.done.set()
            state=deepcopy(self.cache)
            state.update(directory=str(self.directory),parent_queue=self.events.snapshot(),
                         rejected_by_stream=dict(self.dropped),publish_error=self.publish_error,
                         producer_publish_cost=dict(calls=self.publish_calls,total_s=self.publish_total_ns/1e9,
                            maximum_s=self.publish_max_ns/1e9,mean_s=self.publish_total_ns/max(1,self.publish_calls)/1e9))
            return state

    def _sender(self):
        try:
            while self.process.poll() is None:
                try:message=self.commands.get_nowait()
                except queue.Empty:
                    try:message=self.events.get(timeout=.02)
                    except queue.Empty:
                        with self.lock:message=deepcopy(self.terminal)
                        if message is None:continue
                        self._write(message);return
                self._write(message)
        except (OSError,ValueError,TypeError) as error:self._error(error)
        finally:
            try:self.process.stdin.close()
            except (OSError,ValueError):pass

    def _write(self,message):
        data=_encode(message,self.spec['resource_budget']['message_bytes']).encode()
        fd=self.process.stdin.fileno();deadline=time.monotonic()+5
        while data:
            if time.monotonic()>deadline:raise TimeoutError('capture writer pipe stalled')
            if not select.select([],[fd],[],.1)[1]:continue
            try:count=os.write(fd,data)
            except BlockingIOError:continue
            data=data[count:]

    def _bridge(self,recover):
        sender=None
        try:
            with (self.directory/'worker.log').open('ab') as log:
                self.process=subprocess.Popen([self.python,'-u','-B','-m',__name__,'capture-recover' if recover else 'capture'],
                    stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=log,env=_environment(ros=all(s.get('source_ref',{}).get('kind')=='ros'for s in self.spec['source_descriptors'])),start_new_session=True,
                    pass_fds=(self._fd,))
            os.set_blocking(self.process.stdin.fileno(),False)
            self._write(dict(directory=str(self.directory),writer_lock_fd=self._fd))
            if recover:self.process.stdin.close()
            else:
                sender=threading.Thread(target=self._sender,daemon=True,name='capture-sender');sender.start()
            for line in iter(self.process.stdout.readline,b''):
                if len(line)>CAPTURE_MESSAGE_BYTES:raise ValueError('capture status exceeds bound')
                state=json.loads(line)
                if state.get('capture_session_id')!=self.capture_session_id:raise ValueError('capture reply identity changed')
                with self.changed:self.cache=state;self.changed.notify_all()
                if state.get('done'):break
            try:self.process.stdin.close()
            except (OSError,ValueError):pass
            code=self.process.wait(timeout=5)
            if code or not self.cache.get('done'):raise ValueError('capture worker exited without terminal receipt')
        except Exception as error:self._error(error)
        finally:
            if sender is not None:sender.join(1)
            if self.process is not None and self.process.poll() is not None:
                self.process.stdout.close()
            if self._fd is not None and (self.process is None or self.process.poll() is not None):
                os.close(self._fd);self._fd=None
            with self.changed:self.done.set();self.changed.notify_all()

    def recover(self):
        if self.process is not None and self.process.poll() is None:raise ValueError('capture writer is still alive')
        if self.thread is not None and self.thread.is_alive():raise ValueError('capture writer exit not confirmed')
        if self._fd is not None:os.close(self._fd)
        self._fd=os.open(self.directory/'writer.lock',os.O_RDWR|os.O_NOFOLLOW)
        try:fcntl.flock(self._fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:
            os.close(self._fd);self._fd=None;raise ValueError('capture writer ownership is still held')
        if self.terminal is not None and not (self.directory/'finish-request.json').exists():
            from .inference_trace import write_json
            write_json(self.directory/'finish-request.json',self.terminal)
        self.cache=dict(state='recovering',ready=False,capture_session_id=self.capture_session_id)
        self._launch(recover=True)
        return self.snapshot()

    def close(self,timeout=10):
        if self.cache.get('read_only'):return
        if not self.done.is_set():
            if self.terminal is None:self.finish(dict(self.last_sequence),dict(status='stopped',reason='capture closed'),
                **({'source_endpoints':{key:dict(source_clock_id=row['source_clock_id'],source_time=0.) for key,row in self.camera_sources.items()}}
                   if self.camera_sources else {}))
            self.done.wait(timeout)
        if self.process is not None and self.process.poll() is None:
            os.killpg(self.process.pid,signal.SIGTERM)
            try:self.process.wait(5)
            except subprocess.TimeoutExpired:os.killpg(self.process.pid,signal.SIGKILL);self.process.wait(5)
            self.done.wait(5)
        if self.thread is not None:self.thread.join(2)
        if self._fd is not None and (self.process is None or self.process.poll() is not None):
            os.close(self._fd);self._fd=None


def _recording_descriptor(descriptor):
    fd=os.open(descriptor,os.O_RDONLY|os.O_NOFOLLOW|os.O_NONBLOCK)
    with os.fdopen(fd) as stream:
        info=os.fstat(stream.fileno())
        if not stat.S_ISREG(info.st_mode) or info.st_uid!=os.getuid() or info.st_mode&0o077:
            raise ValueError('private owned recording descriptor required')
        raw=stream.read(CAPTURE_MESSAGE_BYTES+1)
    if len(raw)>CAPTURE_MESSAGE_BYTES:raise ValueError('recording descriptor exceeds bound')
    value=json.loads(raw)
    if not isinstance(value,dict) or value.get('schema')!=RECORDING_SERVICE_SCHEMA:
        raise ValueError('recording service protocol version mismatch; expected '+RECORDING_SERVICE_SCHEMA)
    return value


def require_recording_protocol():
    """Check the registered version at configuration, not on a control tick."""
    descriptor=os.environ.get('WORKBENCH_RECORDING_SERVICE')
    if not descriptor:raise ValueError('registered resident recording service is required')
    _recording_descriptor(descriptor)


class _RecordingRejected(ValueError):
    """A complete, identity-checked service reply rejected the request."""


def recording_call(descriptor, action, *, timeout=10, **fields):
    """Private local recording IPC. Never called from a control tick."""
    return _recording_request(_recording_descriptor(descriptor),action,timeout=timeout,**fields)


def _recording_request(value, action, *, timeout=10, **fields):
    address=value.get('socket')
    if not isinstance(address,str) or not Path(address).is_absolute():raise ValueError('absolute recording socket required')
    info=Path(address).lstat()
    if not stat.S_ISSOCK(info.st_mode) or info.st_uid!=os.getuid() or info.st_mode&0o077:
        raise ValueError('private owned recording socket required')
    with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as channel:
        channel.settimeout(timeout);channel.connect(address)
        channel.sendall(_encode(dict(action=action,token=value['token'],**fields),CAPTURE_MESSAGE_BYTES).encode())
        with channel.makefile('rb') as stream:raw=stream.readline(CAPTURE_MESSAGE_BYTES+1)
    if not raw or len(raw)>CAPTURE_MESSAGE_BYTES:raise ValueError('bounded recording reply required')
    result=json.loads(raw)
    if result.get('service_instance_id')!=value.get('service_instance_id'):
        raise ValueError('recording service instance changed')
    if not result.get('ok'):raise _RecordingRejected(result.get('error') or 'recording service rejected request')
    return result['result']


class RecordingService:
    """Resident CPU supervisor; existing writers retain codecs and final barriers."""
    def __init__(self,root,*,data_roots,python=None):
        from .inference_trace import write_json
        from .control_owner import _private_json
        self.root=Path(root).absolute()
        if self.root.is_symlink():raise ValueError('owned recording service root required')
        self.root.mkdir(mode=0o700,parents=True,exist_ok=True)
        if self.root.stat().st_uid!=os.getuid() or self.root.stat().st_mode&0o077:
            raise ValueError('private owned recording service root required')
        self.data_roots=tuple(Path(p).resolve(strict=True) for p in data_roots)
        if not self.data_roots or any(not p.is_dir() for p in self.data_roots):raise ValueError('registered data roots required')
        self.python=python;self.sessions={};self.lock=threading.RLock();self.stopping=False
        self.instance=secrets.token_hex(16);self.token=secrets.token_hex(32)
        self.descriptor_path=self.root/'service.json';self.socket_path=self.root/'ipc.sock'
        self._fd=os.open(self.root/'service.lock',os.O_CREAT|os.O_RDWR|os.O_NOFOLLOW,0o600)
        try:fcntl.flock(self._fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BaseException:os.close(self._fd);raise
        self.registry=self.root/'sessions';self.registry.mkdir(mode=0o700,exist_ok=True)
        self.socket_path.unlink(missing_ok=True)
        service=self
        class Handler(socketserver.StreamRequestHandler):
            def handle(self):
                try:
                    self.request.settimeout(10)
                    raw=self.rfile.readline(CAPTURE_MESSAGE_BYTES+1)
                    if not raw or len(raw)>CAPTURE_MESSAGE_BYTES:raise ValueError('bounded recording request required')
                    request=json.loads(raw)
                    if not secrets.compare_digest(str(request.pop('token','')),service.token):raise ValueError('recording registration differs')
                    peer=None
                    if hasattr(socket,'SO_PEERCRED'):
                        pid,uid,_=struct.unpack('3i',self.request.getsockopt(socket.SOL_SOCKET,socket.SO_PEERCRED,12))
                        if uid!=os.getuid():raise ValueError('recording peer owner differs')
                        from .managed_services import process_identity
                        peer=process_identity(pid)
                    result=service.dispatch(request,peer=peer)
                    reply=dict(ok=True,result=result,service_instance_id=service.instance)
                except Exception as error:reply=dict(ok=False,error=str(error),service_instance_id=service.instance)
                try:self.wfile.write(_encode(reply,CAPTURE_MESSAGE_BYTES).encode())
                except (OSError,ValueError):pass
        class Server(socketserver.ThreadingUnixStreamServer):
            daemon_threads=True
            request_queue_size=32
            def service_actions(self):service._sweep()
        try:
            self.server=Server(str(self.socket_path),Handler);os.chmod(self.socket_path,0o600)
            _private_json(self.descriptor_path,dict(schema=RECORDING_SERVICE_SCHEMA,socket=str(self.socket_path),
                token=self.token,service_instance_id=self.instance,pid=os.getpid()))
            self.descriptor_path.chmod(0o600)
            # Reload only interrupted sessions. Sealed history remains on disk.
            for entry in os.scandir(self.registry):
                if not entry.is_file(follow_symlinks=False):continue
                saved=json.loads(Path(entry.path).read_text());directory=self._directory(saved['directory'])
                if saved.get('sealed')is True:continue
                writer=_CaptureWriter(directory,saved['spec'],python=self.python)
                writer.events.max_bytes=saved['spec']['resource_budget']['event_queue_bytes']//2
                writer.producer_identity=saved.get('producer_identity')
                previous=self._state(writer)
                if previous['sealed'] and previous['writer_exited'] and previous.get('run_receipt_ready')is not False:continue
                self.sessions[str(directory)]=writer
                if writer.cache.get('read_only') and not previous['sealed']:
                    try:writer.recover();writer.cache.pop('read_only',None)
                    except (OSError,ValueError):pass  # An old writer still holding its lock is never replaced.
        except BaseException:
            if hasattr(self,'server'):self.server.server_close()
            os.close(self._fd);raise

    def _directory(self,value):
        path=Path(value)
        if not path.is_absolute() or path.is_symlink() or '..' in path.parts:
            raise ValueError('absolute owned capture directory required')
        resolved=path.resolve()
        if not any(resolved.is_relative_to(root) and resolved!=root for root in self.data_roots):
            raise ValueError('capture directory outside registered data roots')
        return resolved

    def _state(self,writer):
        state=writer.snapshot();process=writer.process
        if not state.get('binding') and (writer.directory/'binding.json').is_file():
            state['binding']=json.loads((writer.directory/'binding.json').read_text())
        producer=getattr(writer,'producer_identity',None)
        if producer and writer.terminal is None and state.get('state')=='recording':
            from .managed_services import process_identity,_matches,_live
            current=process_identity(producer['pid'])
            if not _live(current) or not _matches(current,producer):
                gap=writer.directory/'producer-gap.json'
                if not gap.exists():
                    from .inference_trace import write_json
                    write_json(gap,dict(reason='recording producer exited before final sequence barrier; unsubmitted coverage unknown',
                        producer_identity=producer,checked_wall_ns=time.time_ns()))
        if (writer.directory/'producer-gap.json').exists():
            state.update(diagnostics_error='recording producer exited; coverage is incomplete',degraded=True)
            if writer.spec['purpose_profile']=='data_required':
                state.update(ready=False,error=state['diagnostics_error'])
        if writer.publish_error:
            state.update(diagnostics_error=writer.publish_error,degraded=True)
            if writer.spec['purpose_profile']=='data_required':state.update(ready=False,error=state.get('error') or writer.publish_error)
        state['writer_queue']=state['parent_queue']
        exited=bool(writer.done.is_set() and (process is None or process.poll() is not None)
                    and (writer.thread is None or not writer.thread.is_alive()))
        if process is None and state.get('worker_pid'):
            if Path('/proc/self/stat').exists():
                from .managed_services import process_identity,_matches,_live
                observed=process_identity(state['worker_pid']);recorded=state.get('writer_identity')
                exited=exited and (not _live(observed) or recorded is not None and not _matches(observed,recorded))
            else:
                try:os.kill(state['worker_pid'],0);exited=False
                except ProcessLookupError:pass
        sealed=bool(state.get('done') and state.get('manifest_ref')=='manifest.json'
                    and (writer.directory/'manifest.json').is_file())
        state.update(service_instance_id=self.instance,service_pid=os.getpid(),sealed=sealed,
            writer_exited=exited,
            writer_exit_code=process.poll() if process is not None else (0 if state.get('done') else None),
            submitted_sequences=dict(writer.last_sequence),submitted_times=dict(writer.last_time))
        if 'control_policy'in(state.get('binding')or{}):
            revision=state.get('seal_revision',0)
            state['run_receipt_ready']=bool(sealed and (writer.directory.parent.parent/f'receipt-r{revision:06d}.json').is_file()
                                           and not getattr(writer,'receipt_error',None))
        if getattr(writer,'receipt_error',None):state.update(error=writer.receipt_error,ready=False)
        if process is not None:
            state['worker_pid']=process.pid
            if Path('/proc/self/stat').exists():
                from .managed_services import process_identity
                state['writer_identity']=process_identity(process.pid)
        return state

    def snapshot(self):
        with self.lock:
            rows=[self._state(writer) for writer in self.sessions.values()]
            active=[row for row in rows if not row['writer_exited'] or not row['sealed'] or row.get('run_receipt_ready')is False]
            public=[]
            for row in active:
                value={key:row.get(key)for key in ('capture_session_id','directory','state','worker_pid','writer_identity')}
                writer=self.sessions[row['directory']]
                if not all(s.get('source_ref',{}).get('kind')=='ros'for s in writer.spec['source_descriptors']):
                    value.update(ready=row.get('ready'),error=row.get('error')or row.get('diagnostics_error'))
                public.append(value)
            return dict(service_instance_id=self.instance,pid=os.getpid(),ready=not self.stopping,
                active_sessions=public,can_stop=not active)

    def dispatch(self,request,*,peer=None):
        from .inference_trace import write_json
        action=request.pop('action')
        if action=='inspect':self._sweep();return self.snapshot()
        if action=='shutdown':
            with self.lock:
                self._sweep()
                state=self.snapshot()
                if not state['can_stop']:raise ValueError('recording sessions must seal and writers exit before service stop')
                self.stopping=True
                threading.Thread(target=self.server.shutdown,daemon=True).start()
                return state
        directory=self._directory(request.pop('directory'));key=str(directory)
        with self.lock:
            if self.stopping:raise ValueError('recording service is stopping')
            writer=self.sessions.get(key)
            if writer is None:
                # ponytail: at most 64 active Runs; use indexed admission if concurrent devices ever need more.
                for old in list(self.sessions):
                    old_state=self._state(self.sessions[old])
                    if old_state['writer_exited'] and old_state['sealed'] and old_state.get('run_receipt_ready')is not False:
                        del self.sessions[old]
                if len(self.sessions)>=64:raise ValueError('recording service active session bound reached')
                if action=='create':spec=request['spec']
                else:spec=json.loads((directory/'capture.json').read_text())
                if _capture_spec(spec['request_id'],spec['source_descriptors'],spec['purpose_profile'],spec['resource_budget'])!=spec:
                    raise ValueError('capture registration changed')
                expected='capture-'+hashlib.sha256(spec['request_id'].encode()).hexdigest()
                if directory.name!=expected:raise ValueError('capture request directory differs')
                directory.parent.mkdir(parents=True,exist_ok=True)
                pin=self.registry/(hashlib.sha256(key.encode()).hexdigest()+'.json')
                if pin.exists() and json.loads(pin.read_text()).get('sealed') and not (directory/'capture.json').is_file():
                    raise ValueError('sealed capture was removed; request_id cannot be reused')
                write_json(pin,dict(directory=key,spec=spec,producer_identity=peer))
                writer=_CaptureWriter(directory,spec,python=self.python);self.sessions[key]=writer
                writer.events.max_bytes=spec['resource_budget']['event_queue_bytes']//2
                writer.producer_identity=peer
            if action=='create':
                if request['spec']!=writer.spec:raise ValueError('capture request_id already binds a different request')
                self._state(writer)  # Preserve a previous producer's loss before attaching its replacement.
                if peer is not None:writer.producer_identity=peer
            elif action=='bind':writer.bind(**request['binding'])
            elif action=='start':writer.start()
            elif action=='event':return writer.try_publish(**request)
            elif action=='finish':writer.finish(**request)
            elif action=='recover':writer.recover();writer.cache.pop('read_only',None)
            elif action!='status':raise ValueError('unknown recording action')
            state=self._state(writer);self._publish_run_receipt(writer,state)
            self._retire(writer,state)
            if all(s.get('source_ref',{}).get('kind')=='ros'for s in writer.spec['source_descriptors'])and not state['sealed']:
                from .ros_data import RECORDING_HEALTH_FIELDS
                # Health comes from the original ROS publisher. This reply is
                # the lifecycle/identity proof, including supervisor failures.
                result={key:value for key,value in state.items()if key not in RECORDING_HEALTH_FIELDS
                    and key not in ('parent_queue','writer_queue','producer_publish_cost','rejected_by_stream','publish_error')}
                result['state']=state['state']
                if state['writer_exited']or state.get('diagnostics_error'):
                    if state.get('error'):result['error']=state['error']
                if action not in ('create','bind','recover'):
                    result.pop('binding',None);result.pop('source_qos',None)
                    if state.get('binding'):result.pop('health_source',None)
                return result
            return state

    def _publish_run_receipt(self,writer,state):
        """Produce the existing immutable Run envelope outside every business."""
        from .inference_trace import write_json
        binding=state.get('binding') or {}
        if 'control_policy'not in binding:return
        state['run_receipt_ready']=False
        if not state['sealed'] or not state['writer_exited']:return
        manifest_path=writer.directory/'manifest.json';manifest=json.loads(manifest_path.read_text())
        revision=manifest['seal_revision'];directory=writer.directory.parent.parent
        if not any(directory.is_relative_to(root) for root in self.data_roots):
            raise ValueError('Run receipt outside registered data roots')
        path=directory/f'receipt-r{revision:06d}.json'
        if path.exists():
            existing=json.loads(path.read_text())
            if (existing.get('schema')!='workbench.capture-run.v1' or existing.get('business_kind')!=binding['business_kind']
                    or existing.get('execution_instance_id')!=binding['run_id'] or existing.get('seal_revision')!=revision
                    or existing.get('control_policy')!=binding['control_policy'] or existing.get('execution_context')!=binding['execution_context']
                    or (existing.get('recording')or{}).get('manifest')!=manifest):
                raise ValueError('sealed Run receipt identity differs; original preserved')
        else:
            from .execution_settings import Guard
            counts=dict(G33=len(manifest['issues']),
                G36=sum(row.get('coverage_failed_conditions',0)for row in manifest['streams'].values()),
                G37=int(manifest.get('control_terminal')is None)+sum(row.get('final_sequence')is None for row in manifest['streams'].values()))
            checked=time.monotonic();guard=Guard.bind(binding['control_policy'],{})
            source_range=manifest['streams'].get('control',{}).get('source_time_range',[])
            source=source_range[-1]if source_range else checked
            decisions=[guard.evaluate(dict(rule_id=rule,signal_id=binding['run_id'],run_id=binding['run_id'],
                observed=dict(failed_conditions=count,receipt_ref=f'manifest-r{revision:06d}.json'),
                source_time=source,checked_time=checked),'seal')for rule,count in counts.items()]
            recorded=dict(state=manifest['recording_outcome'],done=True,ready=False,directory=str(writer.directory),
                recording_outcome=manifest['recording_outcome'],manifest=manifest,controller_result=manifest.get('control_terminal'),
                guard_decisions=decisions,error='; '.join(manifest['issues'])or None)
            receipt=dict(schema='workbench.capture-run.v1',seal_revision=revision,execution_instance_id=binding['run_id'],
                business_kind=binding['business_kind'],execution_context=binding['execution_context'],
                control_policy=binding['control_policy'],recording=recorded,controller=manifest.get('control_terminal'))
            write_json(path,receipt);write_json(directory/'receipt.json',receipt)
        state.update(run_receipt_ready=True,receipt_ref=dict(path=str(path),**_capture_file(path)),
            manifest_ref=dict(path=str(writer.directory/f'manifest-r{revision:06d}.json'),
                              **_capture_file(writer.directory/f'manifest-r{revision:06d}.json')))
        writer.receipt_error=None

    def _sweep(self):
        with self.lock:
            for writer in list(self.sessions.values()):
                try:
                    state=self._state(writer);self._publish_run_receipt(writer,state);self._retire(writer,state)
                except (OSError,ValueError,KeyError) as error:
                    writer.receipt_error=str(error)

    def _retire(self,writer,state):
        if not state['sealed'] or not state['writer_exited'] or state.get('run_receipt_ready')is False:return
        from .inference_trace import write_json
        key=str(writer.directory);pin=self.registry/(hashlib.sha256(key.encode()).hexdigest()+'.json')
        write_json(pin,dict(directory=key,spec=writer.spec,sealed=True,seal_revision=state.get('seal_revision')))
        self.sessions.pop(key,None)

    def serve_forever(self):self.server.serve_forever(poll_interval=.05)

    def close(self):
        self._sweep()
        if not self.snapshot()['can_stop']:raise ValueError('active recording service cannot close')
        self.server.server_close();self.socket_path.unlink(missing_ok=True)
        self.descriptor_path.unlink(missing_ok=True);os.close(self._fd)


class _ResidentWriterView:
    def __init__(self,session):self.session=weakref.ref(session)
    @property
    def pid(self):return self.session().cache.get('worker_pid')
    def poll(self):
        state=self.session().cache
        return state.get('writer_exit_code') if state.get('writer_exited') else None


class CaptureSession(_CaptureWriter):
    """Memory-only producer backed by the registered resident recording service."""
    _sessions=weakref.WeakValueDictionary()

    def __init__(self,directory,spec,*,python=None):
        descriptor=os.environ.get('WORKBENCH_RECORDING_SERVICE')
        if not descriptor:raise ValueError('WORKBENCH_RECORDING_SERVICE must register a resident recording service')
        self.descriptor=descriptor;self._service_descriptor=_recording_descriptor(descriptor)
        self.directory=directory;self.spec=spec;self.capture_session_id=spec['capture_session_id']
        self.lock=threading.RLock();self.control_lock=threading.Lock();self.changed=threading.Condition(self.lock)
        self.done=threading.Event();self.disconnected=threading.Event()
        # Split the existing total event budget across producer and service queues.
        self.events=_CaptureEvents(spec['resource_budget']['event_queue_bytes']//2);self.commands=queue.Queue()
        self.cache=dict(state='preparing',capture_session_id=self.capture_session_id,ready=False)
        self.camera_sources={r['stream_id']:r for r in spec['source_descriptors'] if _continuous_source(r)}
        self.last_sequence={r['stream_id']:0 for r in spec['source_descriptors'] if not _continuous_source(r)}
        self.last_time={};self.dropped={key:0 for key in self.last_sequence};self.publish_error=None
        self.publish_calls=self.publish_total_ns=self.publish_max_ns=0
        self.binding=None;self.terminal=None;self.process=_ResidentWriterView(self)
        self.ros_health=all(r.get('source_ref',{}).get('kind')=='ros'for r in spec['source_descriptors'])
        self.health_ros=self.health_source=self._health_reference=None
        self._health_values={};self._control_prepared=False;self._control_error=self._health_error=None
        self.thread=threading.Thread(target=self._bridge,daemon=True,name='capture-service-client');self.thread.start()

    def _call(self,action,**fields):
        state=_recording_request(self._service_descriptor,action,directory=str(self.directory),**fields)
        if action=='event':return state
        with self.changed:
            if not self.ros_health:self.cache=state
            else:
                if self.health_source is not None and not state.get('sealed'):
                    expected=self.health_source['source']['metadata']['writer_identity']
                    actual=state.get('writer_identity')or{}
                    if actual and any(actual.get(key)!=value for key,value in expected.items()):
                        raise ValueError('recording health writer changed; explicit recovery/rebind required')
                if not(self.cache.get('sealed')and not state.get('sealed')):
                    phases={'preparing':0,'recovering':0,'prepared':1,'bound':2,'recording':3,'sealing':4,'complete':5,'incomplete':5,'failed':5}
                    phase=state.get('state')
                    if phases.get(phase,0)<phases.get(self.cache.get('state'),0):state={k:v for k,v in state.items()if k!='state'}
                    self.cache.update(state)
                    if state.get('error'):self._control_error=state['error']
                    self._control_prepared=self.cache.get('state')in('prepared','bound','recording','sealing')
                    self._merge_health()
            self.changed.notify_all();return deepcopy(self.cache)

    def _error(self,error):
        self._control_error=self._control_error or str(error)
        super()._error(error)

    def _merge_health(self):
        if self.cache.get('sealed') or self.cache.get('ingress_closed'):
            self.cache['ready']=False
            if self._control_error:self.cache['error']=self._control_error
            return
        self.cache.update({key:value for key,value in self._health_values.items()if key!='state'})
        error=self._control_error or self._health_error or self._health_values.get('error')
        self.cache['prepared']=bool(self._control_prepared and self._health_values.get('prepared'))
        self.cache['ready']=bool(self._control_prepared and self.cache.get('state')!='sealing'
            and self._health_values.get('ready')and not error)
        if error:self.cache.update(error=error,degraded=True)
        else:self.cache.pop('error',None)

    def _bind_health(self,description):
        from .ros_data import RecordingHealth
        plane=self.spec['source_descriptors'][0]['source_ref']
        with self.changed:self.health_source=deepcopy(description)
        self.health_ros=RecordingHealth(description,capture_session_id=self.capture_session_id,
            device_id=plane['source']['device_id'],writer_identity=self.cache.get('writer_identity')or{},plane=plane,
            run_id=(self.binding or{}).get('run_id'),on_value=self._receive_health,on_error=self._health_fault)

    def _health_fault(self,error):
        with self.changed:
            self._health_error=self._health_error or str(error);self._merge_health();self.changed.notify_all()

    def _receive_health(self,reference,value):
        if set(value['streams'])!={s['stream_id']for s in self.spec['source_descriptors']}:
            self._health_fault('recording health stream binding differs');return
        if value.get('data_queue',{}).get('max_bytes',self.spec['resource_budget']['event_queue_bytes'])!=self.spec['resource_budget']['event_queue_bytes']:
            self._health_fault('recording health resource budget differs');return
        with self.changed:
            if self.cache.get('sealed'):return
            self._health_reference=reference;self._health_values=value
            self._merge_health();self.changed.notify_all()

    def bind(self,**fields):
        if self.ros_health:
            with self.lock:
                if self.health_source is None or not self.cache.get('prepared'):raise ValueError('ROS recording health binding is not confirmed')
                if fields.get('health_source',self.health_source)!=self.health_source:raise ValueError('recording health binding changed')
                fields['health_source']=deepcopy(self.health_source)
        result=super().bind(**fields)
        if self.health_ros is not None:self.health_ros.bind_run(fields['run_id'])
        return result

    def _bridge(self):
        sender=None
        try:
            try:state=self._call('create',spec=self.spec)
            except _RecordingRejected:
                # A rejected create is not a live writer. Confirm absence on
                # the same service before dropping its process placeholder;
                # transport failures retain the unknown process state.
                observed=_recording_request(self._service_descriptor,'inspect')
                if not any(row.get('capture_session_id')==self.capture_session_id
                           or row.get('directory')==str(self.directory)
                           for row in observed['active_sessions']):self.process=None
                raise
            with self.lock:
                self.last_sequence.update(state.get('submitted_sequences',{}));self.last_time.update(state.get('submitted_times',{}))
                self.binding=state.get('binding')
            sender=threading.Thread(target=self._sender,daemon=True,name='capture-service-sender');sender.start()
            while not self.disconnected.wait(.05):
                state=self._call('status')
                if state.get('writer_exited'):break
                if self.ros_health and self.health_source is None:
                    description=state.get('health_source')
                    if description is not None:self._bind_health(description)
                    elif state.get('state')in('prepared','bound','recording','sealing'):
                        raise ValueError('recording service does not provide the required ROS health_source')
        except Exception as error:self._error(error)
        finally:
            self.disconnected.set()
            if sender is not None:sender.join(6)
            if self.health_ros is not None:self.health_ros.close();self.health_ros=None
            with self.changed:self.done.set();self.changed.notify_all()

    def _sender(self):
        try:
            while not self.disconnected.is_set():
                try:message=self.commands.get_nowait()
                except queue.Empty:
                    try:message=self.events.get(timeout=.02)
                    except queue.Empty:
                        with self.lock:message=deepcopy(self.terminal)
                        if message is None:continue
                        self._write(message);return
                self._write(message)
        except Exception as error:self._error(error);self.disconnected.set()

    def _write(self,message):
        fields=dict(message);action=fields.pop('action')
        result=self._call(action,**fields)
        if action=='event' and not result['accepted']:
            with self.lock:
                self.dropped[fields['stream_id']]+=1;self.publish_error=result['reason']

    def recover(self):
        if self.process.poll() is None:raise ValueError('capture writer exit not confirmed')
        if self.thread.is_alive():self.thread.join(2)
        if self.thread.is_alive():raise ValueError('capture client exit not confirmed')
        with self.changed:
            self.health_source=self._health_reference=None;self._health_values={}
            self._control_prepared=False;self._control_error=self._health_error=None
            self.cache=dict(state='recovering',capture_session_id=self.capture_session_id,ready=False)
        self._call('recover');self.done.clear();self.disconnected.clear()
        self.thread=threading.Thread(target=self._bridge,daemon=True,name='capture-service-recovery');self.thread.start()
        return self.snapshot()

    def close(self,timeout=10):
        if not self.done.is_set() and self.terminal is None:
            self.finish(dict(self.last_sequence),dict(status='stopped',reason='capture closed'),
                **({'source_endpoints':{key:dict(source_clock_id=row['source_clock_id'],source_time=0.)
                   for key,row in self.camera_sources.items()}} if self.camera_sources else {}))
        if not self.done.wait(timeout):raise TimeoutError('resident recording seal is still pending; service preserved')
        self.thread.join(2)


class _CaptureEnding(ValueError):
    """A terminal request already closed producer admission for this session."""


class CaptureControlSession:
    """Existing control-recorder lifecycle backed by the common CPU capture Run."""
    recording_backend='capture_session_v1'
    pixel_formats=('rgb8','rgb8-depth16le')

    def __init__(self,root,*,execution_instance_id,business_kind,control_policy,cameras,
                 execution_owner,environment,clock,source_time,on_fault=lambda reason:None,
                 preparation_active=lambda:True,preparation_deadline_monotonic=None,artifact_refs=None,execution_context=None,
                 ros_sources=None,ros_publish=None,ros_watermarks=None):
        if not callable(source_time)or not callable(preparation_active):raise ValueError('trusted source clock and context verifier required')
        if not Path(root).is_absolute():raise ValueError('absolute owned control capture root required')
        self.execution_instance_id=execution_instance_id;self.execution_owner=deepcopy(execution_owner)
        self.source_time,self.preparation_active,self.on_fault=source_time,preparation_active,on_fault
        from .execution_settings import CAPTURE_PREPARATION_TIMEOUT_S
        self.preparation_deadline_monotonic=preparation_deadline_monotonic or time.monotonic()+CAPTURE_PREPARATION_TIMEOUT_S
        self.cancelled=threading.Event();self.execution_bound=threading.Event();self.ready=threading.Event()
        self._camera_tokens=frozenset()
        self._artifact_refs=deepcopy(artifact_refs or{})
        self.ros_publish=ros_publish
        self.capture=CaptureRun(execution_instance_id,Path(root).resolve(),dict(clock=clock),control_policy,cameras,environment,None,
            self.preparation_current,self._publish_receipt,business_kind=business_kind,
            ros_sources=ros_sources,ros_watermarks=ros_watermarks)
        if execution_context is not None:self.capture.config['execution_context']=deepcopy(execution_context)
        self.prepared,self.done=self.capture.prepared,self.capture.done;self.thread=self.capture.thread
        self._lock=threading.RLock();self._sequence=0;self._binding=None;self._fault_notified=False;self._last_source=None
        self.on_sealed=lambda value:None
        from .execution_settings import recording_parameters
        purpose=recording_parameters(business_kind,control_policy['parameters']['recording'])['purpose_profile']
        if purpose is None:raise ValueError('research recording purpose must be selected before execution')
        self._prepare_thread=threading.Thread(target=self._prepare,daemon=True,name='control-capture-prepare')
        self._prepare_thread.start()

    @property
    def root(self):return self.capture.root
    @property
    def manifest(self):return self.capture.manifest
    @property
    def controller_result(self):return self.capture.controller_result
    @property
    def ingress_closed(self):return self.capture.ingress_closed
    @property
    def diagnostics_error(self):
        session=self.capture.session
        with self.capture.lock:error=self.capture.error
        if session is not None:
            with session.lock:error=(error or session.publish_error or session.cache.get('error')
                or session.cache.get('diagnostics_error') or next(iter(session.cache.get('source_errors',{}).values()),None))
        return error
    @property
    def error(self):
        error=self.diagnostics_error
        if not error:return None
        from .execution_settings import Guard
        now=time.monotonic()
        decision=Guard.evaluate(dict(rule_id='G38',signal_id=self.execution_instance_id,
            observed=1,source_time=now,checked_time=now),'recording',self.capture.policy)
        return error if decision['verdict']=='reject'else None

    def is_alive(self):return self.thread.is_alive()

    def preparation_current(self):
        return (not self.cancelled.is_set() and not self.capture.done.is_set() and self.preparation_active()
            and (self.capture.prepared.is_set() or time.monotonic()<self.preparation_deadline_monotonic))

    def lease_active(self,lease):
        session=self.capture.session
        bound=session is not None and session.cache.get('binding')==session.binding and session.binding is not None
        # Revoking control does not revoke this already-bound recording lease.
        current=(self.preparation_current() or bound or self.capture._finishing) and not self.done.is_set()
        return (current and lease.get('run_id')in self._camera_tokens
            and lease.get('execution_instance_id')==self.execution_instance_id and lease.get('execution_owner')==self.execution_owner)

    def _prepare(self):
        try:
            if self.ros_publish is None:
                if callable(self.capture.cameras):self.capture.cameras=self.capture.cameras()
                directory=self.capture.directory.resolve()/'capture'/('capture-'+hashlib.sha256(self.execution_instance_id.encode()).hexdigest())
                self._camera_tokens=frozenset(_camera_token(directory,dict(stream_id='video-'+camera))for camera in self.capture.cameras.channels)
            self.capture.prepare(dict(execution_owner=self.execution_owner))
            if self._artifact_refs and self.capture.session is not None and not self.capture.error:
                self.submit('execution_binding',source_artifacts={name:dict(payload_ref=ref)for name,ref in self._artifact_refs.items()})
        except _CaptureEnding:
            return
        except Exception as error:
            self.capture._fail(error);self.prepared.set()
        if self.error:self._notify_fault(self.error)
        else:
            with self._lock:
                if not self.capture._finishing and self.preparation_current():self.ready.set()

    def _notify_fault(self,reason):
        if self.done.is_set()or not self.preparation_active():return
        with self._lock:
            if self._fault_notified:return
            self._fault_notified=True
        self.on_fault(str(reason))

    def bind_execution(self,timeline,*,execution_instance_id,execution_owner):
        if (execution_instance_id!=self.execution_instance_id or execution_owner!=self.execution_owner
                or self.capture._finishing or not self.prepared.is_set()or not self.preparation_current()):raise ValueError('current prepared capture binding required')
        identity=timeline['timeline_digest']
        if not isinstance(identity,str)or len(identity)!=64 or any(c not in '0123456789abcdef'for c in identity):
            raise ValueError('existing execution timeline digest required')
        with self._lock:
            if self._binding is not None:
                if self._binding!=identity:raise ValueError('capture execution timeline changed')
                return
            self._binding=identity;self.execution_bound.set()
        value=dict(timeline_digest=identity)
        if timeline.get('execution_timeline_ref'):value['artifact']=dict(payload_ref=timeline['execution_timeline_ref'])
        self.submit('execution_binding',**value)

    def submit(self,kind,**fields):
        if kind not in ('command','feedback','native_window','execution_binding','guard_decision','owner_event'):
            raise ValueError('declared control event required; camera pixels use their source stream')
        with self._lock:
            if self.capture._finishing or self.done.is_set():raise _CaptureEnding('capture already ending')
            self._sequence+=1
            try:
                stamp=self.source_time()
                if type(stamp)not in(int,float)or not math.isfinite(stamp):raise ValueError('finite control event source time required')
                value=dict(fields,capture_sequence=self._sequence,capture_source_time=stamp)
                if self.ros_publish is None:self.capture.record(kind,value)
                elif not self.ros_publish(kind,value):raise ValueError('ROS control event publication rejected')
                self._last_source=stamp
            except Exception as error:self.capture._fail(error)
        if self.error:self._notify_fault(self.error)

    def finish(self,status,reason=None):
        with self._lock:
            if self.capture._finishing:return
            self.ready.clear()
            value=dict(final_capture_sequence=self._sequence,finished_wall_ns=time.time_ns())
            if self.ros_publish is not None:value['ros_final_seq']={}
            try:
                stamp=self.source_time()
                if type(stamp)not in(int,float)or not math.isfinite(stamp):raise ValueError('control source clock unavailable at completion')
                value['source_time']=stamp
            except Exception as error:
                self.capture._fail(error)
                if self._last_source is not None:value['source_time']=self._last_source
            self.capture.finish(status,reason,value)

    def fail(self,reason):
        self.capture._fail(reason);self._notify_fault(reason)

    def snapshot(self):
        state=self.capture.snapshot()
        return dict(state,ready=bool(self.ready.is_set()and state.get('ready')and not self.error))

    def _publish_receipt(self,capture):
        from .inference_trace import write_json
        revision=(capture.manifest or{}).get('seal_revision',0)
        receipt=dict(schema='workbench.capture-run.v1',seal_revision=revision,
            execution_instance_id=self.execution_instance_id,business_kind=capture.business_kind,
            execution_context=deepcopy(capture.config.get('execution_context',{})),
            control_policy=capture.policy,recording=self.snapshot(),controller=capture.controller_result)
        path=capture.directory/f'receipt-r{revision:06d}.json'
        if capture.session is not None and 'control_policy'in(capture.session.binding or{}):
            if not capture.session.snapshot().get('run_receipt_ready'):raise ValueError('resident Run receipt is not sealed yet')
            receipt=json.loads(path.read_text())
        else:
            if path.exists():
                if json.loads(path.read_text())!=receipt:raise ValueError('sealed Run receipt differs; original revision preserved')
            else:write_json(path,receipt)
            write_json(capture.directory/'receipt.json',receipt)
        manifest=capture.root/f'manifest-r{revision:06d}.json'
        self.on_sealed(dict(execution_instance_id=self.execution_instance_id,business_kind=capture.business_kind,
            seal_revision=revision,receipt_ref=dict(path=str(path),**_capture_file(path)),
            manifest_ref=dict(path=str(manifest),**_capture_file(manifest))if manifest.is_file()else None))

    def close(self):self.capture.close()


def capture_control(root,payload,*,cameras,environment,clock,source_time,on_fault,ros_runtime=None):
    """Project the approved once-per-Run factory context, never SDK window data."""
    context=payload.get('recording_preparation')or{}
    required={'business_kind','control_policy','execution_instance_id','preparation_active'}
    if not required<=context.keys()or not re.fullmatch('[A-Za-z0-9_-]{1,128}',str(context['execution_instance_id'])):
        raise ValueError('frozen control recording context required before output')
    ros_options={}
    if ros_runtime is not None:
        from .ros_recording import capture_source
        from .inference_trace import clean
        def sources():
            expected={stream:dict(source_instance_id=ros_runtime.nonce)for stream in('state','control')}
            expected.update({name:dict(kind='image')for name in('base_0','left_wrist_0','right_wrist_0')})
            rows=ros_runtime.ros_directory.resolve(expected)
            try:rows+=ros_runtime.ros_directory.resolve({'pedal':dict(kind='event')},timeout=.1)
            except TimeoutError:pass  # Pedal is optional; motion/state/video requirements are unchanged.
            return [capture_source(row,namespace=ros_runtime.data_plane['namespace'],
                domain_id=ros_runtime.data_plane['domain_id'])for row in rows]
        def publish(kind,value):
            return ros_runtime.ros_publisher.event(kind,clean(value),run_id=context['execution_instance_id'],
                epoch=payload['execution_owner']['command_epoch'],source_time_ns=round(source_time()*1e9))
        ros_options=dict(ros_sources=sources,ros_publish=publish,ros_watermarks=ros_runtime.ros_publisher.flush)
    name=('inference-'if context['business_kind']=='inference'else'')+context['execution_instance_id']
    return CaptureControlSession(Path(root)/name,
        execution_instance_id=context['execution_instance_id'],business_kind=context['business_kind'],
        control_policy=context['control_policy'],cameras=cameras,execution_owner=payload['execution_owner'],
        environment=environment,clock=clock,source_time=source_time,on_fault=on_fault,
        preparation_active=context['preparation_active'],
        preparation_deadline_monotonic=context.get('preparation_deadline_monotonic'),
        execution_context=context.get('execution_context'),
        artifact_refs={name:ref for name,ref in (payload.get('plan_ref')or{}).items()
            if name in('plan','source','binding','base_timeline')and ref is not None},**ros_options)


def _encode(value,limit=LIMIT):
    raw=json.dumps(value,allow_nan=False,separators=(',',':'))+'\n'
    if len(raw.encode())>limit:raise ValueError('bounded recording IPC message required')
    return raw


def _timeline(value):
    if value is None:return None
    ref=value.get('execution_timeline_ref')
    if not isinstance(ref,dict):raise ValueError('isolated recording requires execution_timeline_ref, not control arrays')
    return dict(ref)


def _environment(*,ros=False):
    from shutil import which
    from launch_environment import clean_environment, HOST_PATH
    paths=[ROOT,ROOT/'rule_based_workbench',ROOT/'calibration',ROOT/'calibration/infra']
    # Keep the deployment-selected codecs, not its Python/CUDA runtime paths.
    # Falling back to the system FFmpeg can silently select an older encoder.
    codec_dirs=[str(Path(found).absolute().parent) for name in ('ffmpeg','ffprobe') if (found:=which(name))]
    result=clean_environment(dict(PYTHONPATH=os.pathsep.join(map(str,paths)),CUDA_VISIBLE_DEVICES='',
        PATH=os.pathsep.join(dict.fromkeys([*codec_dirs,*HOST_PATH.split(os.pathsep)])),
        OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1',NUMEXPR_NUM_THREADS='1'))
    if ros:
        for key in ('AMENT_PREFIX_PATH','CMAKE_PREFIX_PATH','COLCON_PREFIX_PATH','LD_LIBRARY_PATH','RMW_IMPLEMENTATION','ROS_DISTRO'):
            if key in os.environ:result[key]=os.environ[key]
        result['PYTHONPATH']=os.pathsep.join([result['PYTHONPATH'],os.environ.get('PYTHONPATH','')])
        result['ROS_LOCALHOST_ONLY']=os.environ.get('ROS_LOCALHOST_ONLY','1')
    return result


def _export_process(action,request,*,python=None):
    """Export/verify in a CPU child so codecs and raw indexes stay out of the Owner."""
    executable=python or os.environ.get('WORKBENCH_CPU_PYTHON') or sys.executable
    if not Path(executable).is_absolute():raise ValueError('absolute CPU Python interpreter required')
    with (Path(request['raw_root']).parent/(action+'-worker-'+secrets.token_hex(6)+'.log')).open('xb') as log:
        child=subprocess.Popen([executable,'-u','-B','-m',__name__,action],stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,stderr=log,text=True,env=_environment(),start_new_session=True)
        try:
            result,_=child.communicate(_encode(request),timeout=600)
            if child.returncode or len(result)>LIMIT:raise ValueError('isolated '+action+' failed; inspect '+log.name)
            return json.loads(result)
        except BaseException:
            if child.poll() is None:
                os.killpg(child.pid,signal.SIGTERM)
                try:child.wait(5)
                except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait(5)
            raise


def export_playback_process(raw_root,output,*,environment,python=None):
    return _export_process('export',dict(raw_root=str(raw_root),output=str(output),environment=environment),python=python)


def export_lerobot_process(raw_root,output,*,task,repo_id='workbench/arx6-coffee-collection',python=None):
    return _export_process('export-lerobot',dict(raw_root=str(raw_root),output=str(output),task=task,repo_id=repo_id),python=python)


class ProcessRecordingSession:
    def __init__(self,root,*,plan_ref=None,source=None,binding=None,session_kind='frozen',
                 camera_channels=None,camera_directories_path=None,camera_directories=None,
                 on_fault=lambda reason:None,preparation_active=lambda:True,python=None,
                 execution_timeline=None,execution_owner=None,execution_instance_id=None,
                 defer_execution=True,preparation_deadline_monotonic=None,environment=None,
                 storage_format='segments',persistent_service=False,max_wall_time_seconds=None,
                 resident_deadline_wall_ns=None):
        from real_recording import RecordingEventQueue
        self.root=Path(root)
        if not self.root.is_absolute():raise ValueError('absolute recording output required')
        if session_kind not in ('frozen','streaming'):raise ValueError('known recording session kind required')
        if session_kind=='frozen' and not isinstance(plan_ref,dict):raise ValueError('verified plan_ref required')
        self.execution_owner=deepcopy(execution_owner)
        self.execution_instance_id=execution_instance_id or (source or {}).get('execution_instance_id')
        self.environment=environment or (source or {}).get('environment') or (plan_ref or {}).get('environment')
        self.python=python or os.environ.get('WORKBENCH_CPU_PYTHON') or sys.executable
        if not Path(self.python).is_absolute():raise ValueError('absolute CPU Python interpreter required')
        self.camera_channels=camera_channels;self.camera_service=None
        descriptor=None;expected_instance=None
        if camera_channels is not None:
            services={id(channel.service):channel.service for channel in camera_channels.values()}
            if len(services)!=1:raise ValueError('one shared camera service required')
            self.camera_service=next(iter(services.values()))
            descriptor=str(self.camera_service.descriptor_path or Path(next(iter(self.camera_service.directories.values()))).parent/'camera-service.json')
            expected_instance=self.camera_service.descriptor['instance_id']
        elif camera_directories_path is None and camera_directories is None:raise ValueError('recording camera source required')
        options=dict(environment=self.environment,execution_owner=self.execution_owner,
            execution_instance_id=self.execution_instance_id,defer_execution=defer_execution,
            preparation_deadline_monotonic=preparation_deadline_monotonic or time.monotonic()+30,
            storage_format=storage_format,persistent_service=persistent_service,
            max_wall_time_seconds=max_wall_time_seconds,resident_deadline_wall_ns=resident_deadline_wall_ns)
        self.setup=dict(schema='recording_process.v1',root=str(self.root),kind=session_kind,plan_ref=deepcopy(plan_ref),
            source=deepcopy(source),binding=deepcopy(binding),camera_descriptor=descriptor,
            expected_camera_instance=expected_instance,camera_directories_path=str(camera_directories_path) if camera_directories_path else None,
            camera_directories=camera_directories,execution_timeline_ref=_timeline(execution_timeline),options=options)
        self.preparation_active=preparation_active;self.on_fault=on_fault;self.defer_execution=defer_execution
        self.preparation_deadline_monotonic=options['preparation_deadline_monotonic']
        self.prepared,self.ready,self.done,self.cancelled,self.execution_bound=(threading.Event() for _ in range(5))
        self.events=RecordingEventQueue(max_bytes=64*1024**2)
        self.lock=threading.RLock();self.error=self.seal_error=self.manifest=self.controller_result=None
        self.terminal=None;self.lease_tokens=set();self.channel_tokens={};self.process=None;self.worker_pid=None
        self.clock=None;self.cache=dict(ready=False,done=False,error=None,storage_format=storage_format)
        self._finished=threading.Event();self._forced=None;self._fault_notified=False
        self.thread=threading.Thread(target=self._bridge,name='recording-process-bridge',daemon=True);self.thread.start()

    def _error(self,reason):
        with self.lock:
            self.error=self.error or str(reason);self.cancelled.set()
            first=not self._fault_notified;self._fault_notified=True
        if first:self.on_fault(self.error)

    def _enqueue(self,value):
        try:self.events.put_nowait(value)
        except queue.Full:
            self._error('bounded recording process event queue overflow')
            self._forced=dict(action='fail',reason=self.error)
            raise ValueError(self.error)

    def preparation_current(self):
        return (not self.done.is_set() and not self.cancelled.is_set() and self.preparation_active()
                and (self.prepared.is_set() or time.monotonic()<self.preparation_deadline_monotonic))

    def lease_active(self,lease):
        return (self.preparation_current() and lease.get('run_id') in self.lease_tokens
            and lease.get('execution_instance_id')==self.execution_instance_id and lease.get('execution_owner')==self.execution_owner)

    def bind_execution(self,timeline,*,execution_instance_id,execution_owner):
        with self.lock:
            if (not self.preparation_current() or not self.prepared.is_set() or self.execution_bound.is_set()
                    or execution_instance_id!=self.execution_instance_id or execution_owner!=self.execution_owner):
                raise ValueError('one current recording timeline binding required')
            self._enqueue(dict(action='bind',reference=_timeline(timeline),execution_instance_id=execution_instance_id,execution_owner=execution_owner))
            self.execution_bound.set()

    def submit(self,kind,**fields):
        with self.lock:
            if self.error or self.terminal or self.done.is_set():raise ValueError('recording is closing or failed')
            if kind not in ('command','feedback','native_window'):raise ValueError('camera pixels must go directly to the recording child')
            self._enqueue(dict(action='event',kind=kind,fields=fields))

    def append_command(self,**fields):self.execution_bound.set();return self.submit('command',**fields)
    def append_feedback(self,**fields):return self.submit('feedback',**fields)
    def start(self):return self
    def status(self):return self.snapshot()

    def finish(self,status,reason=None):
        if status not in ('completed','failed','stopped'):raise ValueError('explicit terminal recording status required')
        with self.lock:
            if self.terminal is not None:return
            self.terminal=(status,reason,time.monotonic());self.controller_result=dict(status=status,reason=reason)
            if status!='completed':self.cancelled.set()
            self._enqueue(dict(action='finish',status=status,reason=reason))

    def fail(self,reason):
        self._error(reason);self._forced=dict(action='fail',reason=str(reason))

    def snapshot(self):
        with self.lock:value=deepcopy(self.cache)
        value.update(error=self.error or value.get('error'),done=self.done.is_set(),worker_pid=self.worker_pid,
            controller_result=self.controller_result or value.get('controller_result'),directory=str(self.root),
            transport='isolated_cpu_recording_process',parent_event_queue=self.events.snapshot())
        return value

    def _attach_tokens(self,tokens):
        if self.camera_channels is None or tokens==self.channel_tokens:return True
        for name,token in tokens.items():
            lease=self.camera_channels[name].current_lease()
            if lease is None:return False  # Child may already have sealed after this status sample.
            if (lease.get('run_id')!=token or lease.get('execution_owner')!=self.execution_owner
                    or lease.get('execution_instance_id')!=self.execution_instance_id):
                raise ValueError('recording child lease differs from current Owner')
        with self.camera_service.binding_lock:
            if self.camera_service.descriptor['instance_id']!=self.setup['expected_camera_instance']:raise ValueError('camera binding changed during recording preparation')
            for name,token in tokens.items():
                channel=self.camera_channels[name]
                if channel.cached_token not in (None,token):raise ValueError('camera client belongs to another recording')
                channel.cached_token=token;self.camera_service.known_tokens.add(token)
        self.channel_tokens=dict(tokens);self.lease_tokens=set(tokens.values())
        return True

    def _sender(self):
        try:
            while not self._finished.is_set():
                if self._forced is not None:
                    value=self._forced;self._forced=None
                else:
                    try:value=self.events.get(timeout=.05)
                    except queue.Empty:
                        if self.terminal is None and not self.preparation_current():
                            if self.prepared.is_set():self.finish('stopped','recording Owner revoked')
                            else:self.fail('recording preparation expired or Owner revoked')
                        continue
                self._write(value,512*1024)
        except (OSError,ValueError,TypeError) as error:
            if not self._finished.is_set():self._error(error)
        finally:
            try:self.process.stdin.close()
            except OSError:pass

    def _write(self,value,limit=LIMIT):
        payload=_encode(value,limit).encode();deadline=time.monotonic()+5;fd=self.process.stdin.fileno()
        while payload:
            if time.monotonic()>=deadline:raise TimeoutError('recording child command pipe stalled')
            if not select.select([], [fd], [], .1)[1]:continue
            try:count=os.write(fd,payload)
            except BlockingIOError:continue
            payload=payload[count:]

    def _bridge(self):
        sender=None
        try:
            self.root.parent.mkdir(parents=True,exist_ok=True)
            log=self.root.parent/('recording-worker-'+secrets.token_hex(6)+'.log')
            with log.open('xb') as stream:
                self.process=subprocess.Popen([self.python,'-u','-B','-m','robot_workbench.recording_process'],
                    stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=stream,text=True,bufsize=1,env=_environment(),start_new_session=True)
            self.worker_pid=self.process.pid
            os.set_blocking(self.process.stdin.fileno(),False)
            self._write(self.setup)
            sender=threading.Thread(target=self._sender,name='recording-process-events',daemon=True);sender.start()
            while True:
                line=self.process.stdout.readline(LIMIT+1)
                if not line:break
                if len(line)>LIMIT or not line.endswith('\n'):raise ValueError('recording status exceeds IPC bound')
                value=json.loads(line)
                if value.get('worker_pid')!=self.worker_pid:raise ValueError('recording worker identity changed')
                state=value['status']
                attached=True
                if value.get('prepared') and not state.get('done'):attached=self._attach_tokens(value['channel_tokens'])
                with self.lock:
                    self.cache=state;self.clock=value.get('clock');self.manifest=value.get('manifest')
                    self.seal_error=state.get('seal_error')
                if self.camera_channels is None:self.lease_tokens=set(value.get('lease_tokens',[]))
                if value.get('prepared') and attached:self.prepared.set()
                if state.get('ready'):self.ready.set()
                if state.get('error'):self._error(state['error']);self._finished.set()
                if state.get('done'):break
            self._finished.set()
            if sender is not None:sender.join(1)
            code=self.process.wait(timeout=5)
            if code or not self.cache.get('done'):self._error('recording child exited before a complete terminal receipt; inspect '+str(log))
        except Exception as error:
            self._error(error)
        finally:
            self._finished.set()
            if self.process is not None:
                try:self.process.stdin.close()
                except (OSError,ValueError):pass
                if sender is not None:sender.join(1)
                if self.process.poll() is None:
                    # Only this pure CPU recording session and its codecs; never
                    # the shared camera service, controller or another job.
                    try:self.process.wait(timeout=30)
                    except subprocess.TimeoutExpired:
                        self._error('recording CPU child did not seal after EOF; retained incomplete prefix')
                        try:os.killpg(self.process.pid,signal.SIGTERM)
                        except ProcessLookupError:pass
                        try:self.process.wait(timeout=5)
                        except subprocess.TimeoutExpired:
                            os.killpg(self.process.pid,signal.SIGKILL);self.process.wait(timeout=5)
                self.process.stdout.close()
            if self.camera_channels is not None:
                for name,channel in self.camera_channels.items():
                    try:
                        if self.camera_service.descriptor['instance_id']!=self.setup['expected_camera_instance']:
                            raise ValueError('camera service instance changed before recording cleanup')
                        token=self.channel_tokens.get(name);lease=channel.current_lease()
                        # A crash after begin but before its status reply may have
                        # left a partial lease. Recover only this exact execution.
                        if lease and lease.get('execution_owner')==self.execution_owner and lease.get('execution_instance_id')==self.execution_instance_id:
                            token=lease['run_id']
                        if token is not None:channel.end(token)
                    except (OSError,ValueError) as error:self._error('recording lease cleanup unknown: '+str(error))
            self.done.set()


def _capture_file(path):
    digest=hashlib.sha256();size=0
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):
            size+=len(block);digest.update(block)
    return dict(size_bytes=size,sha256=digest.hexdigest())


def _archive_capture_payload(value,root,index):
    if isinstance(value,list):return [_archive_capture_payload(item,root,index) for item in value]
    if not isinstance(value,dict):return value
    result={}
    for key,item in value.items():
        if key!='payload_ref':result[key]=_archive_capture_payload(item,root,index);continue
        if (not isinstance(item,dict) or set(item)!={'path','size_bytes','sha256'}
                or type(item['size_bytes']) is not int or item['size_bytes']<0
                or not isinstance(item['sha256'],str) or not re.fullmatch('[a-f0-9]{64}',item['sha256'])
                or not isinstance(item['path'],str) or not Path(item['path']).is_absolute()):
            raise ValueError('immutable payload_ref requires absolute path, byte size and SHA256')
        target=root/'artifacts'/item['sha256'];target.parent.mkdir(exist_ok=True)
        expected={name:item[name] for name in ('size_bytes','sha256')}
        if target.exists():
            if target.is_symlink() or _capture_file(target)!=expected:raise ValueError('archived payload changed')
        else:
            fd=os.open(item['path'],os.O_RDONLY|os.O_NOFOLLOW)
            temporary=target.with_suffix('.'+secrets.token_hex(4)+'.tmp')
            try:
                info=os.fstat(fd)
                if not stat.S_ISREG(info.st_mode) or info.st_size!=item['size_bytes']:raise ValueError('payload source identity changed')
                digest=hashlib.sha256();size=0
                with os.fdopen(fd,'rb',closefd=False) as source,temporary.open('xb') as output:
                    for block in iter(lambda:source.read(1024*1024),b''):
                        size+=len(block)
                        if size>item['size_bytes']:raise ValueError('payload grew during copy')
                        output.write(block);digest.update(block)
                    output.flush();os.fsync(output.fileno())
                if dict(size_bytes=size,sha256=digest.hexdigest())!=expected:raise ValueError('payload checksum changed')
                temporary.replace(target)
            finally:
                os.close(fd);temporary.unlink(missing_ok=True)
        result[key]=dict(path=str(target.relative_to(root)),**expected)
        index.write(json.dumps(result[key],separators=(',',':'))+'\n')
    return result


def _camera_token(root,source):
    return hashlib.sha256((str(root)+'\0'+source['stream_id']).encode()).hexdigest()[:32]


def _camera_stamp(source,meta):
    if source.get('source_ref',{}).get('kind')=='ros':
        from .ros_data import StreamRef
        ref=StreamRef(**meta['ros_reference'])
        if ref.source_instance_id!=source['source_instance_id']or ref.source_clock_id!=source['source_clock_id']:
            raise ValueError('ROS camera source identity changed')
        return ref.source_time_ns
    from .sample_clock import clock_spec,sample_stamp
    if meta.get('capture_instance_id')!=source['source_instance_id']:raise ValueError('camera capture instance changed')
    clock=clock_spec(meta['environment'],clock_domain=meta.get('clock_domain'),source_clock_id=meta.get('source_clock_id'))
    if (clock['source_clock_id'] or clock['clock_domain'])!=source['source_clock_id']:
        raise ValueError('camera source clock changed')
    return sample_stamp(meta,clock)[0]


def _release_camera_lease(root,source,binding):
    """Only release this dead/finished writer's proven lease, never another one."""
    from .rgbd_service import CommonCameraService
    ref=source['source_ref'];service=CommonCameraService.connect(ref['descriptor_path'])
    if service.descriptor['instance_id']!=ref['service_instance_id']:raise ValueError('camera service changed during release')
    channel=service.channels[ref['camera']];lease=channel.current_lease()
    if lease is None:return dict(released=True,already_absent=True)
    if (lease.get('run_id')!=_camera_token(root,source) or lease.get('execution_instance_id')!=binding.get('run_id')
            or lease.get('execution_owner')!=binding.get('execution_owner')):
        return dict(released=False,other_lease_preserved=True)
    before=channel.snapshot(lease['run_id'])
    if not channel.end(lease['run_id']):raise ValueError('camera lease release not confirmed')
    return dict(released=True,published_frames=before['published_frames'],queued_frames=before['queued_frames'])


class _CaptureCamera:
    """One existing camera channel and encoder, entirely inside the CPU writer."""
    def __init__(self,root,source):
        from .rgbd_service import CommonCameraService
        self.root,self.source=root,source;ref=source['source_ref']
        self.service=CommonCameraService.connect(ref['descriptor_path'])
        if self.service.descriptor['instance_id']!=ref['service_instance_id']:raise ValueError('camera service instance changed')
        capabilities=self.service.call('identity').get('capabilities',[])
        if 'recording_run_lease_v1' not in capabilities:
            raise ValueError('camera source lacks recording_run_lease_v1')
        self.camera=ref['camera'];self.channel=self.service.channels[self.camera]
        self.pixel_format=self.service.descriptor.get('pixel_formats',{}).get(self.camera,'rgb8-depth16le')
        if self.pixel_format=='rgb8'and ('rgb8_capture_v1'not in capabilities or source['units']!='rgb8'):
            raise ValueError('declared RGB-only recording capability required')
        self._check_source(self.service.latest_rgbd(self.camera)['metadata'])
        if self.channel.current_lease() is not None:raise ValueError('camera recording channel is already occupied')
        self.ffmpeg,self.ffprobe=shutil.which('ffmpeg'),shutil.which('ffprobe')
        if not self.ffmpeg or not self.ffprobe:raise ValueError('registered CPU video codecs are unavailable')
        self.lock=threading.Lock();self.done=threading.Event();self.ending=threading.Event()
        self.endpoint=None;self.binding=None;self.token=_camera_token(root,source);self.writer=None;self.thread=None
        self.state=dict(received=0,committed=0,last_sequence=0,last_source_time=None,endpoint_reached=False,
                        error=None,cleanup_error=None,outside_range_frames=0,unrecorded_queued_frames=0)

    def _check_source(self,meta):
        return _camera_stamp(self.source,meta)

    def begin(self,binding):
        from .sample_clock import owner_identity
        owner=owner_identity(binding['execution_owner'])
        if self.channel.current_lease() is not None:raise ValueError('camera recording channel became occupied')
        self.binding=binding;now=time.time_ns()
        self.lease=dict(run_id=self.token,start_wall_ns=now,deadline_wall_ns=now+600_000_000_000,
            execution_instance_id=binding['run_id'],execution_owner=owner,environment=self.service.descriptor['environment'],
            identity_scope='recording_run')
        try:self.channel.begin(self.lease)
        except Exception:
            _release_camera_lease(self.root,self.source,binding)
            raise

    def start(self):
        self.thread=threading.Thread(target=self._run,daemon=True,name='capture-'+self.source['stream_id']);self.thread.start()

    def finish(self,endpoint):
        if (not isinstance(endpoint,dict) or set(endpoint)!={'source_clock_id','source_time'}
                or endpoint['source_clock_id']!=self.source['source_clock_id']
                or type(endpoint['source_time']) not in (int,float) or not math.isfinite(endpoint['source_time'])
                or endpoint['source_time']<0):
            raise ValueError('camera endpoint differs from its bound source clock')
        with self.lock:
            if self.endpoint is not None and self.endpoint!=endpoint:raise ValueError('camera endpoint changed')
            self.endpoint=dict(endpoint);self.ending.set()

    def snapshot(self):
        with self.lock:return deepcopy(self.state)

    def _run(self):
        from rgbd_segments import SegmentWriter
        from .inference_trace import write_json
        index=None;cutoff_set=False;tail_started=None;last_ns=None
        try:
            index=(self.root/(self.source['stream_id']+'.jsonl')).open('x')
            while True:
                if self.ending.is_set() and not cutoff_set:
                    with self.lock:endpoint=dict(self.endpoint)
                    end_ns=round(endpoint['source_time']*1e9)
                    self.channel.stop_after(self.token,end_ns);cutoff_set=True;tail_started=time.monotonic()
                    if end_ns==0:
                        with self.lock:self.state['endpoint_reached']=True
                    elif last_ns is not None and last_ns>=end_ns:
                        with self.lock:self.state['endpoint_reached']=True
                        break
                now=time.time_ns()
                if now>self.lease['deadline_wall_ns']-300_000_000_000:
                    new=now+600_000_000_000
                    if not self.channel.renew(self.token,self.lease['deadline_wall_ns'],new):raise ValueError('camera lease renewal failed')
                    self.lease['deadline_wall_ns']=new
                frame=self.channel.peek(self.token)
                if frame is None:
                    if cutoff_set:
                        state=self.channel.snapshot(self.token)
                        if state['admission_closed'] and not state['queued_frames']:break
                        if time.monotonic()-tail_started>=5:raise TimeoutError('camera tail did not cover source endpoint within existing 5s budget')
                    time.sleep(.01);continue
                meta=frame['metadata'];stamp=self._check_source(meta)
                if last_ns is not None and stamp<=last_ns:raise ValueError('camera source time did not advance')
                if meta['environment']=='fake':
                    from .sample_clock import owner_identity
                    observed=owner_identity(meta);bound=self.binding['execution_owner']
                    if (any(observed[key]!=bound[key] for key in ('owner_pid','runtime_nonce'))
                            or observed['command_epoch']<bound['command_epoch']):
                        raise ValueError('camera frame Owner identity changed')
                if (meta.get('recording_execution_owner')!=self.binding['execution_owner']
                        or meta.get('recording_identity_scope')!='recording_run'
                        or meta.get('recording_run_id')!=self.token):
                    raise ValueError('camera frame lease association changed')
                if frame.get('pixel_format',self.pixel_format)!=self.pixel_format:raise ValueError('camera pixel format changed')
                if self.writer is None:self.writer=SegmentWriter(self.root/'camera',self.source['stream_id'],ffmpeg=self.ffmpeg,ffprobe=self.ffprobe,pixel_format=self.pixel_format)
                received=time.monotonic()
                if frame.get('pixel_format')in('rgb8','rgb8-depth16le'):
                    ref=self.writer.add_pixels(meta,frame['rgb'],frame['depth'],frame['width'],frame['height'])
                else:ref=self.writer.add(meta,frame['color_png'],frame['depth_png'])
                sequence=meta['recorded_frames']
                value=dict(sequence=sequence,source_time=stamp/1e9,received_monotonic_s=received,recorded_wall_ns=time.time_ns(),
                    payload=dict(frame_ref=dict(segment='camera/'+ref['segment'],frame=ref['frame']),metadata=meta))
                index.write(_encode(value));index.flush();self.channel.ack(self.token,meta['pair_id']);last_ns=stamp
                with self.lock:self.state.update(received=self.state['received']+1,last_sequence=sequence,last_source_time=stamp/1e9)
                if cutoff_set and end_ns>0 and stamp>=end_ns:
                    with self.lock:self.state['endpoint_reached']=True
                    break
        except Exception as error:
            with self.lock:self.state['error']=str(error)
        finally:
            try:
                release=_release_camera_lease(self.root,self.source,self.binding or {})
                with self.lock:
                    if self.state['error'] or self.endpoint is None or self.endpoint['source_time']==0:
                        self.state['unrecorded_queued_frames']=release.get('queued_frames',0)
                    else:self.state['outside_range_frames']=release.get('queued_frames',0)
                    if not release.get('released'):self.state['cleanup_error']='original camera lease no longer owned'
            except Exception as error:
                with self.lock:self.state['cleanup_error']=str(error)
            try:
                if self.writer is not None:
                    segments=self.writer.finish()
                    with self.lock:self.state['committed']=sum(segment['frame_count'] for segment in segments)
                if index is not None:index.flush();os.fsync(index.fileno());index.close()
            except Exception as error:
                with self.lock:self.state['error']=self.state['error'] or str(error)
                if self.writer is not None:self.writer.abort()
                if index is not None and not index.closed:index.close()
            receipt=self.snapshot();receipt['endpoint']=self.endpoint
            try:write_json(self.root/(self.source['stream_id']+'-source.json'),receipt)
            finally:self.done.set()


def _seal_capture(root,spec,extra_issue=None,*,probe_cache=None):
    """Verify the on-disk prefix and sequence barrier; never alter event files."""
    from .inference_trace import write_json
    from .pipeline_timing import timing
    seal_started=time.monotonic_ns()
    def phase(name,started=None):
        if started is not None:timing.record('recording.seal.'+name,started,capture_session_id=spec['capture_session_id'])
        try:write_json(root/'pipeline-timing.json',dict(stage=name,seal_started_ns=seal_started,timing=timing.snapshot(limit=64)))
        except OSError:timing.record('diagnostics.write_failed',time.monotonic_ns(),outcome='error')
    phase('verify_streams')
    finish_path=root/'finish.json';binding_path=root/'binding.json'
    finish=json.loads(finish_path.read_text()) if finish_path.exists() else None
    binding=json.loads(binding_path.read_text()) if binding_path.exists() else None
    revisions=[int(path.stem.rsplit('r',1)[-1]) for pattern in ('manifest-r[0-9]*.json','gaps-r[0-9]*.jsonl') for path in root.glob(pattern)]
    revision=max(revisions,default=0)+1;gap_name=f'gaps-r{revision:06d}.jsonl';segment_name=f'segments-r{revision:06d}.jsonl'
    issues=[extra_issue] if extra_issue else [];streams={};files={}
    model_indexes={'model_input':{},'model_result':{}}
    # Reuse the live writer's codec proofs when available; recovery starts cold.
    # Each reuse still hashes the current bytes; no stat-only authorization.
    if probe_cache is None:probe_cache={}
    ros_capture=all(s.get('source_ref',{}).get('kind')=='ros'for s in spec['source_descriptors'])
    producer_gap=root/'producer-gap.json'
    if producer_gap.exists():
        issues.append(json.loads(producer_gap.read_text())['reason']);files[producer_gap.name]=_capture_file(producer_gap)
    if finish is None:issues.append('no final sequence barrier received')
    if binding is None:issues.append('capture was not bound to a Run')
    activity=None;activity_policy=((binding or{}).get('execution_context')or{}).get('recording_activity')
    if activity_policy is not None:
        from .recording_activity import ActiveTimeAxis,POLICY
        try:
            if activity_policy!=POLICY:raise ValueError('unknown recording activity policy')
            activity=ActiveTimeAxis((finish or{}).get('recording_activity'))
        except ValueError as error:issues.append(str(error))
    with (root/gap_name).open('x') as gap_file,(root/segment_name).open('x') as segment_index:
        for descriptor in spec['source_descriptors']:
            stream_started=time.monotonic_ns()
            issue_start=len(issues)
            identity=descriptor['stream_id'];path=root/(identity+'.jsonl')
            last=count=valid_bytes=missing=0;digest=hashlib.sha256();tail_error=None;times=[]
            effective_count=0;effective_times=[]
            effective_name=f'effective-{identity}-r{revision:06d}.jsonl'
            effective_file=(root/effective_name).open('x')if activity is not None else None
            final=finish['final_seq_by_stream'][identity] if finish else None
            active_segment=None;segment_manifest=None;segment_frames=None;frame_index=0
            if descriptor['kind']=='video' and 'source_ref' in descriptor:
                receipt_path=root/(identity+'-source.json')
                if receipt_path.exists():
                    source_receipt=json.loads(receipt_path.read_text());files[receipt_path.name]=_capture_file(receipt_path)
                    for key in ('error','cleanup_error'):
                        if source_receipt.get(key):issues.append(identity+': '+source_receipt[key])
                    if source_receipt.get('unrecorded_queued_frames'):issues.append(identity+': accepted camera frames were not recorded')
                    endpoint=(finish or {}).get('source_endpoints',{}).get(identity)
                    if endpoint and endpoint['source_time']>0 and not source_receipt.get('endpoint_reached'):
                        issues.append(identity+': camera endpoint not covered')
                else:issues.append(identity+': camera source receipt missing')
            def gap(first,last,reason):
                gap_file.write(json.dumps(dict(stream_id=identity,first_sequence=first,last_sequence=last,reason=reason))+'\n')
            if path.exists():
                with path.open('rb') as stream:
                    for line in stream:
                        try:
                            if not line.endswith(b'\n') or len(line)>spec['resource_budget']['message_bytes']:
                                raise ValueError('incomplete or oversized event tail')
                            row=json.loads(line);sequence=row['sequence'];stamp=row['source_time']
                            if set(row)!={'sequence','source_time','received_monotonic_s','recorded_wall_ns','payload'}:
                                raise ValueError('event fields changed')
                            if type(sequence) is not int or sequence<=last or type(stamp) not in (int,float) or not math.isfinite(stamp):
                                raise ValueError('invalid event sequence or time')
                            _capture_value(row['payload'],spec['resource_budget']['message_bytes']-256)
                            if descriptor['kind']=='video' and 'source_ref' in descriptor:
                                from rgbd_segments import verify_segment
                                ref=row['payload']['frame_ref'];segment=ref['segment']
                                relative=Path(segment)
                                if relative.parts[:2]!=('camera',identity) or '..' in relative.parts:
                                    raise ValueError('camera segment belongs to another stream')
                                if segment!=active_segment:
                                    if segment_frames is not None:segment_frames.close()
                                    segment_manifest=verify_segment(root/'camera',str(relative.relative_to('camera')),ffprobe=shutil.which('ffprobe') or 'ffprobe',probe_cache=probe_cache)
                                    segment_frames=(root/'camera'/segment_manifest['files']['frames']['path']).open()
                                    frame_index=0;active_segment=segment
                                    segment_index.write(json.dumps(dict(stream_id=identity,segment=segment,**_capture_file(root/segment)))+'\n')
                                if ref['frame']!=frame_index or json.loads(segment_frames.readline())!=row['payload']['metadata']:
                                    raise ValueError('camera frame reference and segment metadata differ')
                                if stamp!=_camera_stamp(descriptor,row['payload']['metadata'])/1e9 or times and stamp<=times[-1]:
                                    raise ValueError('camera source timestamp differs or repeats')
                                frame_index+=1
                            if times and stamp<times[-1]:raise ValueError('event source time moved backwards')
                            model_kind=descriptor.get('source_ref',{}).get('source',{}).get('kind')
                            if ros_capture and model_kind in model_indexes:
                                payload=row['payload'];request=payload['request_id']
                                if not isinstance(request,str)or not re.fullmatch(r'[A-Za-z0-9_-]{1,128}',request):
                                    raise ValueError('invalid recorded model request identity')
                                if request in model_indexes[model_kind]:raise ValueError('duplicate model request identity')
                                if model_kind=='model_result':
                                    ref=payload['output_ref'];relative=Path(ref['path'])
                                    if relative!=Path('model')/request/'output.npz':raise ValueError('invalid model output path')
                                    path_out=root/relative
                                    if any(p.is_symlink()for p in (path_out,path_out.parent,path_out.parent.parent)):
                                        raise ValueError('model output symlink rejected')
                                    pin={key:ref[key]for key in ('size_bytes','sha256')}
                                    if _capture_file(path_out)!=pin:raise ValueError('model output bytes differ')
                                    files[str(relative)]=pin
                                model_indexes[model_kind][request]=payload
                        except (OSError,ValueError,KeyError,TypeError,RuntimeError) as error:
                            tail_error=str(error);issues.append(identity+': '+tail_error);break
                        if sequence>last+1:gap(last+1,sequence-1,'sequence_gap');missing+=sequence-last-1
                        count+=1;last=sequence;valid_bytes+=len(line);digest.update(line)
                        times=[times[0],stamp] if times else [stamp]
                        if activity is not None:
                            reference=row['payload'].get('metadata',{}).get('ros_reference')or row['payload'].get('reference')
                            if reference is not None:
                                try:effective=activity.offset_ns(reference['source_clock_id'],reference['source_time_ns'])
                                except (ValueError,KeyError,TypeError)as error:
                                    issues.append(identity+': effective time mapping: '+str(error));effective=None
                                if effective is not None:
                                    effective_file.write(json.dumps(dict(sequence=sequence,effective_time_ns=effective))+'\n')
                                    effective_count+=1;effective_times=[effective_times[0],effective]if effective_times else[effective]
            if final is not None:
                if final<last:issues.append(identity+': final sequence precedes committed data')
                elif final>last:gap(last+1,final,'missing_tail');missing+=final-last
            if missing:issues.append(identity+': sequence coverage incomplete')
            if descriptor['required'] and (not descriptor['source_instance_id'] or not descriptor['source_clock_id']
                    or not count and not(activity is not None and activity.duration_ns==0)):
                issues.append(identity+': required source unavailable or empty')
            if descriptor['required']and activity is not None and activity.duration_ns and not effective_count:
                issues.append(identity+': required active source unavailable or empty')
            streams[identity]=dict(committed=count,last_sequence=last,final_sequence=final,missing=missing,
                source_time_range=times,verified_prefix_bytes=valid_bytes,verified_prefix_sha256=digest.hexdigest(),tail_error=tail_error,
                coverage_failed_conditions=len(issues)-issue_start)
            if effective_file is not None:
                effective_file.flush();os.fsync(effective_file.fileno());effective_file.close()
                streams[identity].update(effective_count=effective_count,effective_time_range_ns=effective_times,
                    effective_timestamps_ref=effective_name)
                files[effective_name]=_capture_file(root/effective_name)
            if path.exists():files[path.name]=_capture_file(path)
            if segment_frames is not None:
                if frame_index!=segment_manifest['frame_count']:issues.append(identity+': unindexed segment frames')
                segment_frames.close()
            timing.record('recording.seal.stream.'+identity,stream_started)
        gap_file.flush();os.fsync(gap_file.fileno())
        segment_index.flush();os.fsync(segment_index.fileno())
    phase('verify_streams',seal_started);proof_started=time.monotonic_ns()
    index=root/'artifacts.jsonl'
    if index.exists():
        with index.open() as stream:
            for line in stream:
                try:
                    ref=json.loads(line);path=root/ref['path']
                    if path.parent!=root/'artifacts' or path.is_symlink() or _capture_file(path)!={key:ref[key] for key in ('size_bytes','sha256')}:
                        raise ValueError('artifact bytes differ')
                except (OSError,ValueError,KeyError,TypeError):issues.append('artifact index or payload incomplete');break
    for name in ('capture.json','binding.json','finish.json','finish-request.json','camera-recovery.json',gap_name,segment_name,'artifacts.jsonl'):
        if (root/name).exists():files[name]=_capture_file(root/name)
    if ros_capture and set(model_indexes['model_input'])!=set(model_indexes['model_result']):
        issues.append('model input/result coverage differs')
    if ros_capture:
        from .ros_data import StreamRef
        decisions=[]
        for request,payload in model_indexes['model_input'].items():
            try:
                metadata=json.loads(payload['metadata_json'])
                if 'stall_recovery'in metadata:
                    decision=metadata['stall_recovery']
                    if decision['actual_seed']!=metadata['seed']:raise ValueError('actual stall seed differs from model input')
                    decisions.append(dict(request_id=request,run_id=payload['run_id'],epoch=payload['epoch'],
                        source_time_ns=payload['reference']['source_time_ns'],**decision))
            except (KeyError,TypeError,ValueError)as error:
                issues.append('model input '+request+' stall metadata invalid: '+str(error))
        if decisions:
            path=root/'stall_recovery.jsonl';temporary=path.with_suffix('.tmp')
            with temporary.open('w')as stream:
                for row in decisions:stream.write(json.dumps(row,allow_nan=False)+'\n')
                stream.flush();os.fsync(stream.fileno())
            temporary.replace(path);files[path.name]=_capture_file(path)
        needed={};source_kinds={s['stream_id']:s['source_ref']['source']['kind']for s in spec['source_descriptors']}
        for request,value in model_indexes['model_input'].items():
            try:
                observation=value['observation']
                for raw,kind in [(observation['state'],'state'),*((r,'image')for r in observation['frames'])]:
                    ref=StreamRef(**raw);key=tuple(ref.__dict__.values())
                    if source_kinds.get(ref.stream_id)!=kind:raise ValueError('model observation source kind differs')
                    needed.setdefault(ref.stream_id,{}).setdefault(key,set()).add(request)
            except (KeyError,TypeError,ValueError):issues.append('model input '+request+' has invalid original source references')
        # Validate against the verified on-disk prefixes, not the recorder's
        # transient ROS caches. A frame could have been available to infer but
        # absent from the recording's admitted range.
        for stream,wanted in needed.items():
            path=root/(stream+'.jsonl');count=streams.get(stream,{}).get('committed',0)
            if count and path.is_file():
                with path.open()as source:
                    for index,line in enumerate(source):
                        if index>=count or not wanted:break
                        payload=json.loads(line)['payload']
                        raw=payload.get('metadata',{}).get('ros_reference')or payload.get('reference')
                        if raw is not None:
                            try:key=tuple(StreamRef(**raw).__dict__.values())
                            except (TypeError,ValueError):continue
                            wanted.pop(key,None)
            for requests in wanted.values():
                for request in sorted(requests):issues.append('model input '+request+' references missing original '+stream+' sample')
        if not issues and model_indexes['model_input']:
            from .ros_recording import verify_model_inputs
            try:verify_model_inputs(root,model_indexes['model_input'],streams,probe_cache=probe_cache,reconstruct_pixels=False)
            except (OSError,ValueError,KeyError,TypeError,RuntimeError)as error:
                issues.append('model input validation: '+str(error))
    phase('verify_model_inputs',proof_started)
    manifest=dict(schema='workbench.capture-manifest.v1',capture_session_id=spec['capture_session_id'],
        binding=binding,purpose_profile=spec['purpose_profile'],recording_outcome='incomplete' if issues else 'complete',
        control_terminal=finish['control_terminal'] if finish else None,streams=streams,issues=issues,files=files,
        gaps_ref=gap_name,segments_ref=segment_name,seal_revision=revision,coverage_scope='declared producer sequences; upstream sensor completeness is not inferred')
    if ros_capture:
        manifest.update(model_inputs=model_indexes['model_input'],model_results=model_indexes['model_result'])
        manifest['model_input_validation']=dict(
            metadata_and_references=('failed'if issues else'passed')if model_indexes['model_input']else'not_applicable',
            pixel_reconstruction='not_requested',scope='recorded structure, source references and input/result coverage')
    if activity is not None:manifest['recording_activity']=deepcopy(finish['recording_activity'])
    if activity_policy is not None and activity is None:manifest['playback_error']='recording activity timeline unavailable; unfiltered replay prohibited'
    elif any('source_ref' in source for source in spec['source_descriptors']):
        export_started=time.monotonic_ns();phase('export_playback')
        try:
            from segmented_playback import export_capture
            playback=export_capture(root,manifest,root/f'replay-r{revision:06d}',probe_cache=probe_cache)
            manifest['playback']=playback
            files.update({path:{key:pin[key] for key in ('size_bytes','sha256')} for path,pin in playback['files'].items()})
        except Exception as error:manifest['playback_error']=str(error)
        finally:phase('export_playback',export_started)
    timing.record('recording.seal.total',seal_started,capture_session_id=spec['capture_session_id'])
    manifest['pipeline_timing']=timing.snapshot(limit=0)
    write_json(root/f'manifest-r{revision:06d}.json',manifest)
    write_json(root/'manifest.json',manifest)
    phase('complete')
    return manifest


def _capture_worker(*,recover=False):
    from .inference_trace import write_json
    fd=sys.stdin.fileno();pending=bytearray()
    while b'\n' not in pending:
        block=os.read(fd,65536)
        if not block:raise ValueError('capture setup missing')
        pending.extend(block)
        if len(pending)>CAPTURE_MESSAGE_BYTES:raise ValueError('capture setup exceeds bound')
    raw,_,remaining=pending.partition(b'\n');setup=json.loads(raw);root=Path(setup['directory'])
    writer_fd=setup.get('writer_lock_fd')
    if writer_fd is None:writer_fd=os.open(root/'writer.lock',os.O_CREAT|os.O_RDWR|os.O_NOFOLLOW,0o600)
    actual=os.fstat(writer_fd);expected=os.stat(root/'writer.lock',follow_symlinks=False)
    if (actual.st_dev,actual.st_ino)!=(expected.st_dev,expected.st_ino) or not stat.S_ISREG(actual.st_mode):
        raise ValueError('capture writer lock identity differs')
    fcntl.flock(writer_fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
    spec=json.loads((root/'capture.json').read_text())
    if _capture_spec(spec['request_id'],spec['source_descriptors'],spec['purpose_profile'],spec['resource_budget'])!=spec:
        raise ValueError('capture header identity changed')
    if all(s.get('source_ref',{}).get('kind')=='ros'for s in spec['source_descriptors']):
        from .ros_recording import capture_worker
        return capture_worker(root,spec,remaining,recover=recover)
    available=[row['stream_id'] for row in spec['source_descriptors'] if row['source_instance_id'] and row['source_clock_id']]
    cameras={};source_errors={}
    if not recover:
        for source in spec['source_descriptors']:
            if 'source_ref' not in source:continue
            try:cameras[source['stream_id']]=_CaptureCamera(root,source)
            except Exception as error:
                source_errors[source['stream_id']]=str(error)
                if source['stream_id'] in available:available.remove(source['stream_id'])
    missing=[row['stream_id'] for row in spec['source_descriptors'] if row['required'] and row['stream_id'] not in available]
    state=dict(state='prepared',prepared=True,ready=not missing or spec['purpose_profile']=='infer_debug',
        degraded=bool(missing),missing_streams=missing,available_streams=available,capture_session_id=spec['capture_session_id'],
        worker_pid=os.getpid(),disk_free_bytes=shutil.disk_usage(root).free,streams={},binding=None,done=False,
        source_errors=source_errors,ingress_closed=False)
    if Path('/proc/self/stat').exists():
        from .managed_services import process_identity
        state['writer_identity']=process_identity(os.getpid())
    opened={};index=None;pending=bytearray(remaining);next_status=0.;terminal=False;output_alive=True
    def emit():
        nonlocal output_alive
        for identity,camera in cameras.items():
            camera_state=camera.snapshot()
            if not state.get('done'):state['streams'][identity]=camera_state
            if camera_state.get('error'):source_errors[identity]=camera_state['error']
            if camera_state.get('cleanup_error'):source_errors[identity]=camera_state['cleanup_error']
        state['missing_streams']=[row['stream_id'] for row in spec['source_descriptors'] if row['required']
                                 and (row['stream_id'] not in available or row['stream_id'] in source_errors)]
        state['degraded']=bool(source_errors or state['missing_streams'])
        if state['state'] in ('prepared','bound','recording'):
            state['ready']=not state['missing_streams'] or spec['purpose_profile']=='infer_debug'
        state['heartbeat_monotonic_s']=time.monotonic()
        try:write_json(root/'status.json',state)
        except OSError as error:
            state.update(ready=False,persistence_error=str(error))
            if not state.get('done'):raise
        if output_alive:
            try:sys.stdout.write(_encode(state,CAPTURE_MESSAGE_BYTES));sys.stdout.flush()
            except BrokenPipeError:output_alive=False
    def flush():
        for identity,stream in opened.items():
            stream.flush();os.fsync(stream.fileno());state['streams'][identity]['committed']=state['streams'][identity]['received']
        if index is not None:index.flush();os.fsync(index.fileno())
    try:
        if recover:
            state.update(state='recovering',ready=False);emit()
            binding_path=root/'binding.json';binding=json.loads(binding_path.read_text()) if binding_path.exists() else {}
            cleanup={}
            for source in spec['source_descriptors']:
                if 'source_ref' not in source:continue
                try:cleanup[source['stream_id']]=_release_camera_lease(root,source,binding)
                except Exception as error:cleanup[source['stream_id']]=dict(error=str(error))
            if cleanup:write_json(root/'camera-recovery.json',cleanup)
            request_path=root/'finish-request.json';recovered_boundary=False
            if not (root/'finish.json').exists() and request_path.exists():
                request=json.loads(request_path.read_text())
                for identity,result in cleanup.items():
                    source_path=root/(identity+'-source.json');old=json.loads(source_path.read_text()) if source_path.exists() else {}
                    request['final_seq_by_stream'][identity]=max(old.get('last_sequence',0),result.get('published_frames',0))
                write_json(root/'finish.json',request);recovered_boundary=bool(cleanup)
            manifest=_seal_capture(root,spec,'camera source boundary recovered after writer interruption' if recovered_boundary else None)
            state.update(state=manifest['recording_outcome'],manifest_ref='manifest.json',seal_revision=manifest['seal_revision'],
                         streams=manifest['streams'],ready=False,done=True);emit();return
        emit()
        while not terminal:
            if time.monotonic()>=next_status:
                flush();emit();next_status=time.monotonic()+.25
            if b'\n' not in pending:
                if not select.select([fd],[],[],.05)[0]:continue
                block=os.read(fd,65536)
                if not block:break
                pending.extend(block)
                if b'\n' not in pending and len(pending)>spec['resource_budget']['message_bytes']:
                    raise ValueError('capture command exceeds message bound')
                continue
            raw,_,remaining=pending.partition(b'\n');pending=bytearray(remaining)
            if len(raw)+1>spec['resource_budget']['message_bytes']:raise ValueError('capture command exceeds message bound')
            command=json.loads(raw);action=command['action']
            if action=='bind':
                binding=command['binding']
                if not state['ready'] or state['binding'] not in (None,binding):raise ValueError('capture binding is not available')
                if state['binding']==binding:emit();continue
                if any('source_ref' in row for row in spec['source_descriptors']):
                    from .sample_clock import owner_identity
                    if owner_identity(binding.get('execution_owner',{}))!=binding.get('execution_owner'):
                        raise ValueError('camera binding requires the existing execution_owner')
                write_json(root/'binding.json',binding);state.update(binding=binding,state='bound');emit()
            elif action=='start':
                if state['state']=='recording':emit();continue
                if state['state']!='bound':raise ValueError('capture must be bound before start')
                for identity,camera in list(cameras.items()):
                    try:camera.begin(state['binding'])
                    except Exception as error:
                        source_errors[identity]=str(error);available.remove(identity);del cameras[identity]
                if spec['purpose_profile']=='data_required' and any(row['required'] and row['stream_id'] in source_errors for row in spec['source_descriptors']):
                    raise ValueError('required camera lease could not be acquired')
                for descriptor in spec['source_descriptors']:
                    if 'source_ref' in descriptor:continue
                    identity=descriptor['stream_id'];opened[identity]=(root/(identity+'.jsonl')).open('x')
                    state['streams'][identity]=dict(received=0,committed=0,last_sequence=0)
                for camera in cameras.values():camera.start()
                index=(root/'artifacts.jsonl').open('x');state['state']='recording';emit()
            elif action=='event':
                identity=command['stream_id']
                if state['state']!='recording' or identity not in available:raise ValueError('event is outside the bound stream')
                row=state['streams'][identity];sequence=command['sequence']
                if type(sequence) is not int or sequence<=row['last_sequence']:raise ValueError('event sequence did not increase')
                stamp=command['source_time']
                if type(stamp) not in (int,float) or not math.isfinite(stamp):raise ValueError('invalid event source time')
                received=time.monotonic();payload=_archive_capture_payload(command['payload'],root,index)
                value=dict(sequence=sequence,source_time=stamp,received_monotonic_s=received,
                    recorded_wall_ns=time.time_ns(),payload=payload)
                opened[identity].write(_encode(value,spec['resource_budget']['message_bytes']))
                row.update(received=row['received']+1,last_sequence=sequence)
            elif action=='finish':
                state.update(state='sealing',ready=False);write_json(root/'finish-request.json',command);emit();terminal=command
            else:raise ValueError('unknown capture action')
        flush()
        for stream in opened.values():stream.close()
        if index is not None:index.close()
        for identity,camera in cameras.items():
            endpoint=(terminal or {}).get('source_endpoints',{}).get(identity,
                dict(source_clock_id=camera.source['source_clock_id'],source_time=0.))
            camera.finish(endpoint)
        deadline=time.monotonic()+35
        while any(camera.thread is not None and not camera.done.is_set() for camera in cameras.values()):
            if time.monotonic()>deadline:raise TimeoutError('camera writer did not seal within existing codec/RPC budgets')
            emit();time.sleep(.05)
        if terminal:
            for source in spec['source_descriptors']:
                if 'source_ref' not in source:continue
                identity=source['stream_id'];camera=cameras.get(identity)
                terminal['final_seq_by_stream'][identity]=camera.snapshot()['last_sequence'] if camera else 0
                if camera is None:write_json(root/(identity+'-source.json'),dict(error=source_errors.get(identity,'source unavailable'),last_sequence=0))
            write_json(root/'finish.json',terminal)
            state.update(ingress_closed=True,ready=False);emit()
        manifest=_seal_capture(root,spec)
        state.update(state=manifest['recording_outcome'],manifest_ref='manifest.json',seal_revision=manifest['seal_revision'],
                     streams=manifest['streams'],ready=False,done=True);emit()
    except BaseException as error:
        import traceback
        traceback.print_exc(file=sys.stderr)
        for stream in opened.values():
            try:stream.close()
            except OSError:pass
        if index is not None:
            try:index.close()
            except OSError:pass
        for identity,camera in cameras.items():
            try:
                if camera.thread is not None:
                    if not camera.ending.is_set():camera.finish(dict(source_clock_id=camera.source['source_clock_id'],source_time=0.))
                    camera.done.wait(35)
                elif camera.binding is not None:_release_camera_lease(root,camera.source,camera.binding)
            except Exception as cleanup_error:source_errors[identity]=str(cleanup_error)
        state.update(state='failed',ready=False,done=True,error=str(error))
        try:
            manifest=_seal_capture(root,spec,'writer failed: '+str(error))
            state.update(manifest_ref='manifest.json',seal_revision=manifest['seal_revision'],streams=manifest['streams'])
        except (OSError,ValueError,KeyError):pass
        try:emit()
        except OSError:pass


def _worker():
    sys.path[:0]=[str(ROOT/'rule_based_workbench'),str(ROOT/'calibration'),str(ROOT/'calibration/infra')]
    from contextlib import redirect_stdout
    from real_recording import RecordingSession,StreamingRecordingSession
    from robot_workbench.control_plan import load_artifact
    from robot_workbench.rgbd_service import CommonCameraService
    setup=json.loads(sys.stdin.readline(LIMIT+1));output=sys.stdout;session=None;active=threading.Event();active.set()
    if setup.get('schema')!='recording_process.v1':raise ValueError('known recording IPC setup required')
    options=setup['options'];owner=options['execution_owner'];ref=setup.get('plan_ref')
    try:
        if ref:
            if ref.get('origin')!=dict(owner_pid=owner['owner_pid'],runtime_nonce=owner['runtime_nonce'],epoch=owner['command_epoch']):
                raise ValueError('recording plan reference belongs to another Owner/epoch')
            source=load_artifact(ref['source']);plan=load_artifact(ref['plan']);binding=load_artifact(ref['binding']) if ref.get('binding') else {}
        else:source=setup['source'];plan=None;binding=setup['binding']
        if setup.get('camera_descriptor'):
            cameras=CommonCameraService.connect(setup['camera_descriptor'])
            if cameras.descriptor['instance_id']!=setup['expected_camera_instance']:raise ValueError('recording camera service changed')
            camera_options=dict(camera_channels=cameras.channels)
        else:
            directories=json.loads(Path(setup['camera_directories_path']).read_text()) if setup.get('camera_directories_path') else setup['camera_directories']
            camera_options=dict(camera_directories=directories)
        with redirect_stdout(sys.stderr):
            if setup['kind']=='streaming':
                session=StreamingRecordingSession(setup['root'],source,binding,execution_owner=owner,
                    preparation_active=active.is_set,max_wall_time_seconds=options.get('max_wall_time_seconds') or 600,
                    storage_format=options['storage_format'],**camera_options)
            else:
                timeline=load_artifact(setup['execution_timeline_ref']) if setup.get('execution_timeline_ref') else None
                session=RecordingSession(setup['root'],source,plan,binding,preparation_active=active.is_set,
                    execution_timeline=timeline,**options,**camera_options)
        def commands():
            try:
                while not session.done.is_set():
                    line=sys.stdin.readline(512*1024+1)
                    if not line:session.finish('stopped','recording parent pipe closed');active.clear();return
                    if len(line)>512*1024 or not line.endswith('\n'):raise ValueError('recording command exceeds bound')
                    value=json.loads(line)
                    with redirect_stdout(sys.stderr):
                        if value['action']=='event':
                            if setup['kind']=='streaming':getattr(session,'append_'+value['kind'])(**value['fields'])
                            else:session.submit(value['kind'],**value['fields'])
                        elif value['action']=='bind':session.bind_execution(load_artifact(value['reference']),execution_instance_id=value['execution_instance_id'],execution_owner=value['execution_owner'])
                        elif value['action']=='finish':session.finish(value['status'],value.get('reason'))
                        elif value['action']=='fail':session.fail(value['reason']);active.clear()
                        else:raise ValueError('unknown recording command')
            except Exception as error:session.fail(error);active.clear()
        command_thread=threading.Thread(target=commands,daemon=True);command_thread.start()
        for signum in (signal.SIGTERM,signal.SIGINT):signal.signal(signum,lambda *_:session.finish('stopped','recording worker termination requested'))
        while True:
            state=session.snapshot();manifest=session.manifest
            value=dict(worker_pid=os.getpid(),status=state,prepared=session.prepared.is_set(),clock=session.clock,
                channel_tokens=dict(session.channel_tokens),lease_tokens=sorted(session.lease_tokens),
                manifest={key:item for key,item in manifest.items() if key!='files'} if manifest else None)
            output.write(_encode(value));output.flush()
            if session.done.wait(.05):
                if state.get('done'):break
        session.thread.join(2)
        # Parent closes its command pipe after observing done. Let the input
        # reader release Python's buffered stdin lock before interpreter exit.
        command_thread.join(2)
        if command_thread.is_alive():os._exit(1)
    except Exception as error:
        if session is not None:
            session.fail(error)
            if not session.done.wait(30):
                # If the parent vanished, no supervisor remains to reap our
                # codec children. Abort only this recorder's owned writers.
                if session.recorder is not None:session.recorder.abort()
                session.done.wait(5)
        try:
            output.write(_encode(dict(worker_pid=os.getpid(),status=dict(done=True,ready=False,error=str(error)),prepared=False,channel_tokens={})))
            output.flush()
        except BrokenPipeError:pass


if __name__=='__main__':
    if sys.argv[1:] in (['capture'],['capture-recover']):
        _capture_worker(recover=sys.argv[1:]==['capture-recover'])
    elif sys.argv[1:] in (['export'],['export-lerobot']):
        from contextlib import redirect_stdout
        request=json.loads(sys.stdin.readline(LIMIT+1));output=sys.stdout
        with redirect_stdout(sys.stderr):
            if sys.argv[1]=='export':
                from real_recording import export_playback
                result=export_playback(request['raw_root'],None,request['output'],environment=request['environment'])
            else:
                from .lerobot_export import export
                result=export([request['raw_root']],request['output'],task=request['task'],repo_id=request['repo_id'])
        output.write(_encode(result));output.flush()
    else:_worker()
