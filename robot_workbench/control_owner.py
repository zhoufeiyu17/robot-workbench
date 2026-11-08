"""Private control service: one Runtime/Owner, independent of business lifetime.

The protocol transports declared control operations and cached facts, never
Python callables, arbitrary attributes, camera pixels or SDK objects.
"""
from collections import OrderedDict,deque
from concurrent.futures import ProcessPoolExecutor
from copy import deepcopy
import hashlib
import json
import math
import multiprocessing
import os
from pathlib import Path
import secrets
import socket
import socketserver
import stat
import threading
import time

LIMIT=4*1024*1024
SCHEMA='workbench.control_owner.v8'
PROTOCOL_FEATURES=('action_chunk_v1','normal_modes_v1','normal_finish_v1','recording_context_v1',
                   'infer_reference_policy_v1','finite_sync_chunks_v1','sdk_execution_context_v1','normal_complete_segment_v1','control_policy_v1','capture_session_v1','observation_policy_v1')
READS={'inspect','snapshot','inputs','receipt','events','inference_state','settings_snapshot','control_policy_snapshot'}


def collection_recording_root(owner_root):return Path(owner_root)/'collection-raw'


PREVIEW_FILES='frozen_preview_files.v1'


def _stage_preview(command,root):
    """Only opaque bytes cross this boundary; full JSON decoding stays in spawn."""
    from .control_plan import RawPreview,ArtifactPreview,_read_file,MAX_INPUT
    if not isinstance(command,(RawPreview,ArtifactPreview)):return command,None
    spool=Path(root)/'preview-inputs';spool.mkdir(mode=0o700,exist_ok=True)
    info=spool.stat()
    if spool.is_symlink()or info.st_uid!=os.getuid()or info.st_mode&0o077:
        raise ValueError('private owned preview spool required')
    if len(list(spool.iterdir()))>=16:raise ValueError('unresolved preview inputs require receipt inspection')
    blobs={'raw':command.raw}if isinstance(command,RawPreview)else{n:_read_file(p)for n,p in command.paths.items()}
    if any(not isinstance(raw,bytes)or not 0<len(raw)<=MAX_INPUT for raw in blobs.values()):
        raise ValueError('bounded opaque preview bytes required')
    directory=spool/secrets.token_hex(16);directory.mkdir(mode=0o700);refs={}
    try:
        for name,raw in blobs.items():
            path=directory/(name+'.json')
            with os.fdopen(os.open(path,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o400),'wb')as f:f.write(raw)
            refs[name]=dict(path=str(path),sha256=hashlib.sha256(raw).hexdigest(),size_bytes=len(raw))
    except BaseException:
        for path in directory.iterdir():path.unlink()
        directory.rmdir();raise
    return dict(action='preview_frozen',preview_files=dict(schema=PREVIEW_FILES,files=refs)),directory


def _restore_preview(command,root):
    if 'preview_files'not in command:return command
    from .control_plan import RawPreview,ArtifactPreview,_read_file,MAX_INPUT
    value=command['preview_files'];spool=Path(root)/'preview-inputs'
    if (set(command)!={'action','preview_files'}or command['action']!='preview_frozen'
            or not isinstance(value,dict)or set(value)!={'schema','files'}or value['schema']!=PREVIEW_FILES
            or not isinstance(value['files'],dict)or set(value['files'])not in ({'raw'},{'plan','source'})):
        raise ValueError('exact frozen preview file reference required')
    paths={};raw=None
    for name,ref in value['files'].items():
        if not isinstance(ref,dict)or set(ref)!={'path','sha256','size_bytes'}:raise ValueError('exact preview pin required')
        if (not isinstance(ref['path'],str)or type(ref['size_bytes'])is not int or not 0<ref['size_bytes']<=MAX_INPUT
                or not isinstance(ref['sha256'],str)or len(ref['sha256'])!=64
                or any(c not in '0123456789abcdef'for c in ref['sha256'])):raise ValueError('bounded preview path, size and SHA256 required')
        path=Path(ref['path']);parent=path.parent
        if (not path.is_absolute()or parent.parent!=spool or path.name!=name+'.json'
                or len(parent.name)!=32 or any(c not in '0123456789abcdef'for c in parent.name)
                or any(p.is_symlink()for p in (spool,parent,path))):raise ValueError('preview path outside private spool')
        for p in (spool,parent,path):
            info=p.stat()
            if info.st_uid!=os.getuid()or info.st_mode&0o077:raise ValueError('private owned preview files required')
        if path.stat().st_mode&0o222:raise ValueError('read-only preview input required')
        data=_read_file(path)
        if type(ref['size_bytes'])is not int or len(data)!=ref['size_bytes']or hashlib.sha256(data).hexdigest()!=ref['sha256']:
            raise ValueError('preview input size or SHA256 changed')
        paths[name]=path
        if name=='raw':raw=data
    return RawPreview(raw)if raw is not None else ArtifactPreview(plan_path=paths['plan'],source_path=paths['source'])


def _encode(value):
    raw=json.dumps(value,allow_nan=False,separators=(',',':')).encode()+b'\n'
    if len(raw)>LIMIT:raise ValueError('control IPC message exceeds bounded size')
    return raw


def _event_encoder_start(owner_pid):
    import sys
    if sys.platform=='linux':
        from .arx_output_guard import parent_death
        parent_death(owner_pid)


def _shutdown_encoder(executor):
    if getattr(executor, '_broken', False):
        # CPython 3.10 can leave a large queued write blocked on the parent's
        # unused reader after its only worker dies. Closing it releases shutdown.
        call_queue = getattr(executor, '_call_queue', None)
        if call_queue is not None:
            call_queue._reader.close()
    executor.shutdown(wait=True, cancel_futures=True)


def _encode_event_response(response):
    """Runs only in the CPU child, preserving the existing event byte budget."""
    result=response['result'];rows=[];size=0
    for row in result['events']:
        count=len(_encode(row))
        if rows and size+count>LIMIT//2:break
        rows.append(row);size+=count
    return _encode(dict(response,result=dict(result,events=rows)))


def _encode_read_response(response,started_s):
    return _encode(dict(response,read_snapshot_monotonic_s=started_s))


def _private_json(path,value):
    temporary=path.with_name(path.name+'.tmp-'+secrets.token_hex(4))
    with os.fdopen(os.open(temporary,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600),'wb')as stream:stream.write(_encode(value))
    os.replace(temporary,path)


def _identity():
    from .managed_services import process_identity
    value=process_identity(os.getpid())
    if value is None:raise RuntimeError('control service process identity unavailable')
    return value


class ControlCallUnknown(RuntimeError):
    """The caller must inspect this request ID, never resend a control operation."""
    def __init__(self,request_id,error):
        self.request_id=request_id
        super().__init__('control request result unknown; inspect '+request_id+': '+str(error))


class ControlChannel:
    def __init__(self,descriptor,*,timeout=8.):
        path=Path(descriptor)
        info=path.stat()
        if (not path.is_absolute()or path.is_symlink()or not stat.S_ISREG(info.st_mode)
                or info.st_uid!=os.getuid()or info.st_mode&0o077 or info.st_size>16384):
            raise ValueError('owned control descriptor required')
        value=json.loads(path.read_text())
        if value.get('schema')!=SCHEMA:
            raise ValueError('control Owner protocol version mismatch; expected '+SCHEMA)
        if value.get('environment')not in('physical','fake'):raise ValueError('explicit control service identity required')
        self.descriptor=value;self.timeout=timeout
        token_path=Path(value['token_file']);token_info=token_path.stat()
        if (token_path.is_symlink()or not stat.S_ISREG(token_info.st_mode)or token_info.st_uid!=os.getuid()
                or token_info.st_mode&0o077 or token_info.st_size>1024):raise ValueError('private control credential required')
        self.token=token_path.read_text().strip()
        if len(self.token)<32:raise ValueError('control credential too short')
        self.instance=value['instance_id']
        socket_path=Path(value['socket_path']);socket_info=socket_path.stat()
        if (not socket_path.is_absolute()or socket_path.is_symlink()or not stat.S_ISSOCK(socket_info.st_mode)
                or socket_info.st_uid!=os.getuid()or socket_info.st_mode&0o077):raise ValueError('private owned control socket required')

    def call(self,operation,*,expected_nonce=None,timeout=None,**arguments):
        request_id=secrets.token_hex(16)
        body=dict(schema=SCHEMA,instance_id=self.instance,token=self.token,request_id=request_id,
            operation=operation,expected_nonce=expected_nonce,arguments=arguments)
        connection=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM);connection.settimeout(timeout or self.timeout)
        submitted=False
        try:
            connection.connect(self.descriptor['socket_path']);raw=_encode(body)
            submitted=True;connection.sendall(raw)
            with connection.makefile('rb')as stream:reply=stream.readline(LIMIT+1)
            if not reply or len(reply)>LIMIT:raise OSError('bounded control reply missing')
            result=json.loads(reply)
            if not isinstance(result,dict)or result.get('request_id')!=request_id:raise OSError('control reply request identity differs')
            if result.get('ok')is not True:
                if result.get('error_type')=='PermissionError':raise PermissionError(result['error'])
                raise ValueError(result.get('error','control operation rejected'))
            value=result['result']
            if operation in {'inspect','snapshot'}and 'read_snapshot_monotonic_s'in result:
                value['control_owner']['reply_age_s']=max(0.,time.monotonic()-result['read_snapshot_monotonic_s'])
            return value
        except PermissionError:raise
        except (OSError,EOFError,json.JSONDecodeError)as error:
            if submitted and operation not in READS:
                raise ControlCallUnknown(request_id,error)from None
            raise ConnectionError('control service unavailable: '+str(error))from None
        finally:connection.close()


class _ControlActor:
    """A service-local capability, bound to a private maintenance lease and Stop."""
    def __init__(self,service,action,token):
        self.service,self.action,self.token=service,action,token
    def require(self,runtime):
        lease=self.service.maintenance
        if (runtime is not self.service.device.runtime or lease is None or self.token!=lease['token']
                or lease['stop_serial']!=self.service.stop_serial):raise ValueError('control maintenance lease revoked')
    def current(self):
        try:self.require(self.service.device.runtime);return True
        except ValueError:return False


class ControlOwnerService:
    def __init__(self,device,root,*,profile_sha256=None):
        self.device=device;self.root=Path(root).absolute()
        self.root.mkdir(mode=0o700,parents=True,exist_ok=True)
        info=self.root.stat()
        if self.root.is_symlink()or info.st_uid!=os.getuid()or info.st_mode&0o077:
            raise ValueError('private owned control service directory required')
        self.instance=secrets.token_hex(16);self.token=secrets.token_urlsafe(48)
        self.lock=threading.Lock();self.requests=OrderedDict();self.maintenance=None;self.stop_serial=0
        self.event_lock=threading.Lock();self.events=deque(maxlen=8192);self.event_sequence=0
        self.profile_sha256=profile_sha256;self.inference=None;self.last_runtime_nonce=None
        self.identity=_identity()
        self.descriptor_path=self.root/'control-owner.json';socket_path=self.root/'control.sock'
        if socket_path.exists():raise ValueError('control socket already exists; inspect its owner before replacement')
        if device.environment=='physical'and device.runtime is not None:
            raise ValueError('control service and encoder must start inert before SDK acquisition')
        self.event_encoder=ProcessPoolExecutor(max_workers=1,mp_context=multiprocessing.get_context('spawn'),
            initializer=_event_encoder_start,initargs=(os.getpid(),))
        # Eager creation binds Linux PDEATHSIG to this long-lived main thread,
        # never to a temporary request handler. No SDK objects enter the child.
        self.event_encoder_pid=self.event_encoder.submit(os.getpid).result(timeout=10.)
        self.event_encoding_slots=threading.BoundedSemaphore(2)
        # Status JSON competes for this process's GIL even after Runtime.lock
        # is released. Keep it separate from both control and the event codec.
        self.read_encoder=ProcessPoolExecutor(max_workers=1,mp_context=multiprocessing.get_context('spawn'),
            initializer=_event_encoder_start,initargs=(os.getpid(),))
        self.read_encoder_pid=self.read_encoder.submit(os.getpid).result(timeout=10.)
        self.read_encoding_slots=threading.BoundedSemaphore(2)
        service=self
        class Handler(socketserver.StreamRequestHandler):
            def handle(self):
                self.connection.settimeout(10.)
                request=None;dispatched=False;read_slot=False;read_started=None
                try:
                    raw=self.rfile.readline(LIMIT+1)
                    if not raw or len(raw)>LIMIT:raise ValueError('bounded control request required')
                    request=json.loads(raw)
                    if isinstance(request,dict)and request.get('operation')in {'inspect','snapshot'}:
                        # Backpressure waits before snapshot/Runtime locks; normal
                        # concurrent readers must not turn into false state errors.
                        if not service.read_encoding_slots.acquire(timeout=.2):
                            raise ValueError('control facts encoder busy; retry this read')
                        read_slot=True
                        read_started=time.monotonic()
                    result=service.dispatch(request);dispatched=True
                    response=dict(ok=True,request_id=request['request_id'],result=result)
                    if request['operation']=='events':
                        if not service.event_encoding_slots.acquire(blocking=False):
                            raise ValueError('event encoder busy; retry this read cursor')
                        try:future=service.event_encoder.submit(_encode_event_response,response)
                        except BaseException:
                            service.event_encoding_slots.release();raise
                        # A timed-out client must not free a still-running job's
                        # slot and allow unbounded work into the process queue.
                        future.add_done_callback(lambda _:service.event_encoding_slots.release())
                        encoded=future.result(timeout=8.)
                    elif read_slot:
                        future=service.read_encoder.submit(_encode_read_response,response,read_started)
                        future.add_done_callback(lambda _:service.read_encoding_slots.release())
                        read_slot=False  # The callback owns this slot, including on timeout.
                        encoded=future.result(timeout=8.)
                    else:encoded=_encode(response)
                except Exception as error:
                    if read_slot:service.read_encoding_slots.release()
                    # The mutation already happened. Close without a reply so
                    # ControlChannel reports unknown, never a false rejection.
                    if dispatched and request['operation']not in READS:return
                    # An events codec failure is a read failure; Stop, status and
                    # maintenance remain available without that child process.
                    encoded=_encode(dict(ok=False,request_id=request.get('request_id')if isinstance(request,dict)else None,error_type=type(error).__name__,error=str(error)[:2000]))
                try:self.wfile.write(encoded)
                except (BrokenPipeError,ConnectionResetError):pass
        class Server(socketserver.ThreadingUnixStreamServer):
            daemon_threads=True
        self.server=Server(str(socket_path),Handler);os.chmod(socket_path,0o600)
        token_path=self.root/'control-token'
        with os.fdopen(os.open(token_path,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600),'w')as stream:stream.write(self.token)
        descriptor=dict(schema=SCHEMA,instance_id=self.instance,socket_path=str(socket_path),token_file=str(token_path),
            environment=device.environment,device_id=device.id,identity=self.identity,profile_sha256=profile_sha256,
            frozen_preview_transport=PREVIEW_FILES)
        _private_json(self.descriptor_path,descriptor)

    def event(self,event_name,**value):
        with self.event_lock:
            self.event_sequence+=1
            self.events.append(dict(sequence=self.event_sequence,event=event_name,value=value))

    def inspect(self,business=None):
        state=self.device.snapshot(business)
        features=list(PROTOCOL_FEATURES)
        plane=getattr(self.device.runtime,'data_plane',None)
        if plane is not None:
            features.append('ros_data_plane_v1')
            raw=state.pop('calibration_feedback',None)
            state['feedback_reference']=(self.device.runtime.ros_publisher.reference('state',
                source_time_ns=raw['source_time_ns']if raw.get('source_clock_id')==self.device.runtime.ros_publisher.clock_id
                else round(raw['sample_time_s']*1e9))if raw else None)
        factory=getattr(getattr(self.device.runtime,'owner',None),'recording_factory',None)
        if callable(factory)and getattr(factory,'recording_backend',None)=='capture_session_v1':
            features.append('control_capture_v1')
            from .recording_process import CaptureControlSession
            if 'rgb8'in getattr(CaptureControlSession,'pixel_formats',()):features.append('rgb8_capture_v1')
        state['control_owner']=dict(schema=SCHEMA,instance_id=self.instance,identity=self.identity,
            protocol_features=features,
            runtime_nonce=self.device.runtime.nonce if self.device.runtime else None,profile_sha256=self.profile_sha256,
            tick_owned_by_service=True,business_lifetime_independent=True,
            event_encoding=dict(mode='spawned_cpu',pid=self.event_encoder_pid),
            read_encoding=dict(mode='spawned_cpu',pid=self.read_encoder_pid))
        if plane is not None:state['control_owner']['data_plane']=deepcopy(plane)
        if plane is not None and state.get('recording')is not None:
            recording=self.device.runtime.owner.recording
            session=getattr(getattr(recording,'capture',None),'session',None)
            description=getattr(session,'health_source',None)or getattr(recording,'health_source',None)
            if description is not None:
                info=state['recording']
                info.update(health_source=deepcopy(description),
                    writer_identity=deepcopy(description['source']['metadata']['writer_identity']),
                    execution_instance_id=recording.execution_instance_id)
                if not info['done']:
                    info.pop('ready',None);info.pop('error',None)
        return state

    def dispatch(self,request):
        if (not isinstance(request,dict)or set(request)!={'schema','instance_id','token','request_id','operation','expected_nonce','arguments'}
                or request['schema']!=SCHEMA or request['instance_id']!=self.instance
                or not isinstance(request['token'],str)or not secrets.compare_digest(request['token'],self.token)):
            raise PermissionError('private current control service credential required')
        operation=request['operation'];args=request['arguments'];request_id=request['request_id']
        if not isinstance(args,dict)or not isinstance(request_id,str)or not 1<=len(request_id)<=128:
            raise ValueError('explicit bounded control request identity required')
        runtime=self.device.runtime
        if runtime is not None:self.last_runtime_nonce=runtime.nonce
        if request['expected_nonce']is not None:
            current=runtime.nonce if runtime is not None else self.last_runtime_nonce
            if (request['expected_nonce']!=current or runtime is None and request['operation']not in
                    ('inspect','snapshot','receipt','events','maintenance_end')):
                raise ValueError('control Owner lifetime changed; no automatic retarget')
        if operation in READS:return self._read(operation,args)
        fingerprint=hashlib.sha256(_encode({k:v for k,v in request.items()if k!='token'})).hexdigest()
        with self.lock:
            old=self.requests.get(request_id)
            if old is not None:
                if old['fingerprint']!=fingerprint:raise ValueError('request ID reused for different control operation')
                if not old.get('done'):raise ValueError('control request is still in flight; inspect its receipt')
                if 'error'in old:raise ValueError(old['error'])
                return deepcopy(old['result'])
            if len(self.requests)>=256:
                first=next((key for key,row in self.requests.items()if row.get('done')),None)
                if first is None:raise ValueError('control receipt retention is full')
                del self.requests[first]
            entry=dict(fingerprint=fingerprint,operation=operation,started_wall_ns=time.time_ns(),done=False)
            self.requests[request_id]=entry
        try:
            result=self._write(operation,args)
            with self.lock:entry.update(done=True,result=deepcopy(result),finished_wall_ns=time.time_ns())
            return result
        except Exception as error:
            with self.lock:entry.update(done=True,error=str(error),error_type=type(error).__name__,finished_wall_ns=time.time_ns())
            raise

    def _read(self,operation,args):
        runtime=self.device.runtime
        if operation=='inspect':return self.inspect()
        if operation=='snapshot':return self.inspect(args.get('business'))
        if operation=='inputs':
            if runtime is None:raise ValueError('control Owner not acquired')
            if set(args)-{'include_feedback'}or type(args.get('include_feedback',True))is not bool:
                raise ValueError('explicit feedback projection flag required')
            if getattr(runtime,'data_plane',None) is not None:
                return runtime.control_inputs(include_feedback=False)
            return runtime.control_inputs(**args)
        if operation=='settings_snapshot':
            if runtime is None:raise ValueError('control Owner not acquired')
            return runtime.execution_settings_snapshot(args['business'])
        if operation=='control_policy_snapshot':
            if runtime is None:raise ValueError('control Owner not acquired')
            return runtime.control_policy_snapshot(args['business'])
        if operation=='inference_state':
            runner=runtime.owner.runner if runtime else None
            return dict(execution_instance_id=getattr(runtime.owner,'execution_instance_id',None)if runtime else None,
                snapshot=runner.snapshot()if getattr(runner,'is_inference',False)else None)
        if operation=='receipt':
            with self.lock:
                value=self.requests.get(args.get('request_id'))
                if value is None:raise ValueError('control receipt is not retained; do not replay')
                return deepcopy(value)
        if operation=='events':
            after=args.get('after',0)
            if type(after)is not int or after<0:raise ValueError('event cursor required')
            with self.event_lock:
                pending=[row for row in self.events if row['sequence']>after][:32]
                latest=self.event_sequence
            # These rows come from the bounded SDK/window interfaces. Unknown
            # or large diagnostic events travel singly, bounding parent pickling.
            small={'control_study_feedback','control_study_native_window','sdk_call_latency',
                'owner_inference','inference_guard_decision'}
            if pending and pending[0]['event']not in small:pending=pending[:1]
            else:
                pending=pending[:next((i for i,row in enumerate(pending)if row['event']not in small),len(pending))]
            # Append-only rows are snapshotted by the process pipe. JSON work
            # and byte-budget trimming belong to the CPU child, not this GIL.
            rows=pending
            return dict(events=rows,latest=latest,
                incomplete=bool(rows and rows[0]['sequence']>after+1))
        raise ValueError('unknown read operation')

    def _write(self,operation,args):
        runtime=self.device.runtime
        if operation=='acquire':
            if args!={'arms_supported':True}:raise ValueError('current supported-arms fact required')
            if runtime is not None:raise ValueError('Owner already acquired; attach without reconnecting')
            if self.maintenance is not None:raise ValueError('end the prior control maintenance lease before acquiring SDK')
            self.device.connect(args)
            return self.inspect()
        if operation=='maintenance_end':
            if not self.maintenance or args.get('token')!=self.maintenance['token']:raise PermissionError('maintenance capability required')
            if runtime is not None:runtime.end_maintenance()
            self.maintenance=None;return dict(ended=True)
        if runtime is None:raise ValueError('control Owner not acquired')
        if operation=='stop':
            if args:raise ValueError('Stop accepts no task payload')
            self.stop_serial+=1;return self.device.request_stop()
        if operation=='inference_configure':
            from .owner_inference import OwnerInference
            candidate=OwnerInference(self,args['configuration'],args['specification'])
            try:result=runtime.configure_inference(args['configuration'],candidate.factory)
            except BaseException:
                candidate.close();raise
            previous=self.inference
            self.inference=candidate
            if previous is not None:previous.close()
            return dict(result,session_id=candidate.session_id)
        if operation in('inference_observation','inference_result','inference_recording'):
            if self.inference is None or args.get('session_id')!=self.inference.session_id:
                raise ValueError('inference configuration changed')
            return self.inference.update(operation,args)
        if operation=='submit':
            business=args.get('business');command=args.get('command')
            if business not in self.device.credentials or not isinstance(command,dict):raise ValueError('registered business command required')
            command=_restore_preview(command,self.root)
            if self.device.environment=='physical'and command.get('action')in('release','recover'):
                raise ValueError('control lifecycle requires maintenance capability')
            return self.device.submit(business,command)
        if operation=='handoff':
            return runtime.handoff(self.device.credentials[args['business']],args['target'])
        if operation=='execution_settings':
            return runtime.update_execution_settings(self.device.credentials[args['business']],args['payload'],
                                                     expected_revision=args.get('expected_revision'))
        if operation=='control_policy':
            return runtime.update_control_policy(self.device.credentials[args['business']],args['payload'])
        if operation=='calibration_installed':return runtime.notify_calibration_installed(args['snapshot_id'])
        if operation=='revoke_inference':return runtime.revoke_inference()
        if operation=='maintenance_begin':
            if set(args)-{'client_identity'}:raise ValueError('declared maintenance client identity required')
            if self.maintenance is not None:
                old_identity=self.maintenance.get('client_identity')
                if not isinstance(old_identity,dict):raise ValueError('control maintenance already active; prior client identity unavailable')
                from .managed_services import process_identity
                current=process_identity(old_identity['pid'])
                if current is not None and all(current.get(k)==old_identity.get(k)for k in('pid','start_ticks','boot_id')):
                    raise ValueError('control maintenance already active in a live business process')
                with self.lock:
                    if any(not row.get('done')and row['operation']!='maintenance_begin'for row in self.requests.values()):
                        raise ValueError('previous control operation is still in flight; inspect receipt')
                # Orphan adoption is explicit maintenance, never part of read-only
                # startup. Preserve Stop generation and normal idle admission.
                actor=_ControlActor(self,'recover_control',self.maintenance['token'])
                with runtime._preview_boundary(),runtime.owner.lock:
                    reason=runtime._idle_reason(maintenance_actor=actor,include_in_flight=False)
                    if reason:raise ValueError(reason)
            else:runtime.begin_maintenance()
            self.maintenance=dict(token=secrets.token_urlsafe(32),stop_serial=self.stop_serial,client_identity=args.get('client_identity'))
            return dict(self.maintenance)
        if operation=='retire_protected':
            actor=_ControlActor(self,'recover_control',args.get('token'));actor.require(runtime)
            result=runtime.retire_protected();self.device.closed.set()
            if self.device.thread is not None:self.device.thread.join(2)
            self.device.runtime=None;self.device.credentials={};self.maintenance=None
            return result
        if operation=='hardware_binding':
            path=Path(args['path'])
            if (not path.is_absolute()or '..'in path.parts or path.suffix!='.json'
                    or path.parent.resolve()!=path.parent or path.parent.stat().st_uid!=os.getuid()):
                raise ValueError('owned absolute hardware binding location required')
            with runtime._preview_boundary(),runtime.owner.lock:
                reason=runtime._idle_reason()
                if reason:raise ValueError(reason)
                if str(getattr(runtime.owner,'hardware_binding',''))!=str(path):
                    runtime.owner.hardware_binding=str(path);runtime.preview_revision+=1
                    runtime.owner.pending.clear();runtime.reviewed_plans.clear()
            return dict(path=str(path))
        if operation in('release','recover','verify_hold','admit_collection'):
            action={'release':'release_control','recover':'recover_control','verify_hold':'verify_hold','admit_collection':'ops_prepare_collection'}[operation]
            actor=_ControlActor(self,action,args.get('token'));actor.require(runtime)
            if operation in('release','verify_hold')and args.get('arms_supported')is not True:raise ValueError('current supported-arms fact required')
            if operation=='release':return runtime.control_release(actor)
            if operation=='recover':return runtime.control_recover(actor)
            if operation=='verify_hold':return runtime.verify_hold(actor)
            return runtime.admit_business('collection',maintenance_actor=actor)
        raise ValueError('unknown control operation')

    def serve_forever(self):self.server.serve_forever(poll_interval=.05)

    def close(self):
        if self.device.runtime is not None:raise ValueError('normal supported release or verified protected retirement required before Owner service exit')
        if self.inference is not None:self.inference.close();self.inference=None
        self.server.shutdown();self.server.server_close()
        _shutdown_encoder(self.event_encoder)
        _shutdown_encoder(self.read_encoder)
        Path(self.server.server_address).unlink(missing_ok=True)
        token_path=self.root/'control-token'
        if not token_path.is_symlink() and token_path.is_file() and token_path.read_text()==self.token:
            token_path.unlink()


class _BackendFacts:
    """Read-only backend policy/protection projection, never an SDK proxy."""
    def __init__(self,client):self.client=client
    @property
    def passive_jaws(self):return tuple(self.client.state.get('passive_jaws',()))
    @property
    def terminal_failure(self):return (self.client.state.get('control_protection')or{}).get('reason')
    @property
    def is_connected(self):return self.client.state.get('connected')is True
    def protection_snapshot(self):return deepcopy(self.client.state.get('control_protection')or{})


class OwnerFacts:
    """Declared local fact projection for legacy consumers; no remote attributes."""
    def __init__(self,client):
        self.client=client;self.lock=threading.RLock();self.backend=_BackendFacts(client)
    @property
    def cache(self):return self.client.state
    @property
    def epoch(self):return self.client.state.get('command_epoch')
    @property
    def runtime_nonce(self):return self.client.nonce
    @property
    def feedback_healthy(self):return self.client.state.get('feedback_healthy')is True
    @property
    def shared_target_profile(self):return (self.client.state.get('target_interpreter')or{}).get('controller_profile')
    @property
    def hardware_binding(self):return self.client.inputs.get('hardware_binding')
    @property
    def execution_instance_id(self):return self.client.state.get('execution_instance_id')
    @property
    def runner(self):return self.client.runners.get(self.execution_instance_id)
    @property
    def recording(self):return self.client.recording
    @recording.setter
    def recording(self,value):self.client.attach_recording(value)
    def control_time(self):
        if self.client.environment=='fake':
            current=self.client.state['control_time_s']
            if type(current)not in(int,float)or not math.isfinite(current)or current<0:
                raise ValueError('current simulation source clock is unavailable')
            view=self.client.ros_state
            if view is not None and view.inputs is not None:
                try:current=max(current,view.inputs.latest('state')[0].source_time_ns/1e9)
                except LookupError:pass
            return current
        # Physical Owner and this client share host monotonic time. This never
        # refreshes the recorded feedback timestamp or substitutes a heartbeat.
        return time.monotonic()
    def recording_failed(self,error):self.client.recording_error=str(error)


class ControlRuntimeClient:
    """Business-side finite control API. It owns no tick thread or native SDK."""
    remote_control_owner=True
    def __init__(self,descriptor,*,hardware_binding=None,recording_backend=None):
        if recording_backend not in (None,'capture_session_v1'):raise ValueError('known control recording backend required')
        self.control_recording_backend=recording_backend
        self.review_action=None
        self.channel=ControlChannel(descriptor);self.lock=threading.RLock();self.state={};self.inputs={}
        self.nonce=None;self.fd=None;self.clients={};self.runners={};self.recording=None;self.recording_error=None;self.observation_error=None
        self.lease=None;self.protected_retirement=None;self.descriptor=Path(descriptor)
        self.inference_lock=threading.RLock();self.inference_session=None;self.inference_callbacks=None;self.event_cursor=0
        self.capture_backend=False
        self.observations=OrderedDict();self.event_closed=threading.Event();self.event_thread=None
        self.device_id=self.channel.descriptor['device_id'];self.environment=self.channel.descriptor['environment']
        self.owner=OwnerFacts(self);self.hardware_binding=hardware_binding;self.closed=threading.Event()
        self.data_plane=None;self.ros_state=None;self.recording_health=None;self.ros_control_source=None;self.ros_control_sequence=None
        self._recording_ack=None;self._recording_health_sent=None
        self.refresh()
        if self.nonce is not None:self._require_capture_capabilities()

    def _require_capture_capabilities(self):
        if self.control_recording_backend!='capture_session_v1':return
        features=set(self.state['control_owner']['protocol_features'])
        if not {'control_capture_v1','rgb8_capture_v1'}<=features:
            self.detach()
            raise ValueError('Owner recording capability is not configured; attachment refused before control')

    def _call(self,operation,**arguments):
        return self.channel.call(operation,expected_nonce=self.nonce,**arguments)

    def refresh(self,business=None):
        with self.lock:
            # Order response installation as well as reads: a slower old poll
            # must not overwrite a newer handoff/epoch in the business cache.
            value=self.channel.call('snapshot',expected_nonce=self.nonce,business=business)
            if (value.get('control_owner')or{}).get('schema')!=SCHEMA:
                raise ValueError('control Owner protocol version mismatch; attachment refused')
            nonce=value.get('runtime_nonce')
            if self.nonce is not None and nonce is not None and nonce!=self.nonce:raise ValueError('control Owner changed; explicit attachment required')
            if nonce is not None:self.nonce=nonce
            plane=(value.get('control_owner')or{}).get('data_plane')
            if plane is not None:
                if self.data_plane is not None and self.data_plane!=plane:raise ValueError('Owner ROS data plane changed; explicit reattachment required')
                if self.ros_state is None:
                    from .ros_inputs import RosStateView
                    self.data_plane=deepcopy(plane)
                    self.ros_state=RosStateView(plane,self.device_id,nonce,self.environment)
                self._project_feedback(value)
                self._project_recording(value)
            self.state=value
            self.fd=1 if nonce is not None and not(value.get('release_receipt')or{}).get('status')=='released'else None
        return deepcopy(value)

    def _project_feedback(self,value):
        try:
            value['calibration_feedback']=self.ros_state.resolve(value.get('feedback_reference'))
            value.pop('feedback_source_error',None)
        except (LookupError,ValueError,TimeoutError)as error:
            value['calibration_feedback']=None;value['feedback_source_error']=str(error)

    def _project_recording(self,value):
        from .ros_data import RecordingHealth
        info=value.get('recording')or{};description=info.get('health_source')
        health=self.recording_health
        if description is None or info.get('done') or info.get('ingress_closed'):
            if health is not None:health.close();self.recording_health=None
            return
        if health is not None and health.description!=description:
            health.close();health=self.recording_health=None
        if health is None:
            try:
                self.recording_health=health=RecordingHealth(description,
                    capture_session_id=description['source']['metadata']['capture_session_id'],
                    run_id=info['execution_instance_id'],device_id=self.device_id,plane=self.data_plane,
                    writer_identity=info['writer_identity'],ros=self.ros_state.ros)
            except (ValueError,RuntimeError)as error:
                info.update(ready=False,error=str(error));return
        info.update(health.snapshot())

    def acquire(self):
        if not self.state.get('connected'):
            self.state=self.channel.call('acquire',arms_supported=True);self.nonce=self.state.get('runtime_nonce');self.fd=1
        deadline=time.monotonic()+4
        while time.monotonic()<deadline:
            self.refresh()
            if (self.state.get('feedback_healthy')and self.state.get('stationary')
                    and (self.ros_state is None or self.state.get('calibration_feedback')))or 'FAULT'in str(self.state.get('state')):break
            time.sleep(.02)
        if self.hardware_binding is not None and self.state.get('feedback_healthy')and self.state.get('stationary'):
            self._call('hardware_binding',path=str(self.hardware_binding))
        self.inputs=self.control_inputs()
        self._require_capture_capabilities()
        return self

    def register_business(self,business):
        if business not in('calibration','collection','inference','research')or business in self.clients:raise ValueError('one registered business identity required')
        value=dict(business=business,token=secrets.token_urlsafe(32),runtime_nonce=self.nonce,environment=self.environment,device_id=self.device_id)
        self.clients[business]=value;return deepcopy(value)

    def _business(self,credential):
        if not isinstance(credential,dict)or self.clients.get(credential.get('business'))!=credential:raise PermissionError('registered local business credential required')
        return credential['business']

    @property
    def holder(self):return self.state.get('holder')
    @property
    def release_receipt(self):return deepcopy(self.state.get('release_receipt'))
    @property
    def maintenance_active(self):return bool(self.state.get('maintenance_active'))
    @property
    def calibration_revision(self):return self.state.get('calibration_revision')
    @property
    def calibration_snapshot_id(self):return self.state.get('calibration_snapshot_id')
    @property
    def initialization_error(self):return self.state.get('initialization_error')
    @property
    def _hold_verification(self):
        op=self.state.get('control_operation')or{}
        return op if op.get('status')in('queued','verifying_feedback','holding','stopping')else None

    def control_inputs(self,*,include_feedback=True):
        value=self._call('inputs',**({}if include_feedback else dict(include_feedback=False)))
        if getattr(self,'ros_state',None) is not None and include_feedback:self._project_feedback(value)
        self.inputs=value;return value

    def set_capture_result_handler(self,callback):
        if not callable(callback):raise ValueError('capture result callback required')
        self.capture_result_handler=callback
        if self.ros_state is not None:
            self._subscribe_capture_results()
            return
        with self.inference_lock:
            if self.event_thread is None:
                self.event_thread=threading.Thread(target=self._events,name='owner-business-events',daemon=True)
                self.event_thread.start()

    def _subscribe_capture_results(self):
        if self.ros_control_source is not None:return
        from .ros_data import SourceDirectory
        ros=self.ros_state.ros
        source=SourceDirectory(ros,self.device_id).resolve({'control':dict(source_instance_id=self.nonce,kind='event')})[0]
        publishers=ros.node.get_publishers_info_by_topic(source['topic'])
        if len(publishers)!=1:raise ValueError('one original control event publisher required')
        self.ros_control_source=source;self.ros_control_gid=bytes(publishers[0].endpoint_gid)
        ros.subscribe(source['topic'],ros.types.DataEvent,self._capture_result_event,depth=128)

    def _capture_result_event(self,message):
        from .ros_data import StreamRef,json_value
        source=self.ros_control_source;reference=StreamRef.from_message(message.reference)
        publishers=self.ros_state.ros.node.get_publishers_info_by_topic(source['topic'])
        if (len(publishers)!=1 or bytes(publishers[0].endpoint_gid)!=self.ros_control_gid
                or any(getattr(reference,k)!=source[k]for k in ('stream_id','source_instance_id','source_clock_id'))):
            raise ValueError('control event source changed; result consumer reattachment required')
        if self.ros_control_sequence is not None:
            if reference.sequence<=self.ros_control_sequence:return
            if reference.sequence>self.ros_control_sequence+1:self.capture_result_handler(None)
        self.ros_control_sequence=reference.sequence
        if message.event!='control_capture_sealed':return
        value=json_value(message.payload_json)
        if value.get('execution_instance_id')!=message.run_id:raise ValueError('sealed result belongs to another Run')
        self.capture_result_handler(value)
    def configure_remote_inference(self,configuration,specification,*,created,metric,record,predict,predict_chunk=None):
        owner=self.state.get('control_owner')or{}
        if owner.get('schema')!=SCHEMA:raise ValueError('control Owner protocol version mismatch; configuration refused')
        if configuration.get('data_plane')!=owner.get('data_plane'):
            raise ValueError('inference and Owner data plane differ; configuration refused')
        if configuration.get('action_chunk_binding')is not None and not callable(predict_chunk):
            raise ValueError('trusted action chunk configuration requires the context-aware model callback')
        with self.inference_lock:
            batch=self._call('events',after=self.event_cursor);boundary=batch['latest']
            callback=getattr(self,'capture_result_handler',None)
            if callback is not None:
                for row in batch['events']:
                    if row['event']=='control_capture_sealed':callback(row['value'])
                if batch['incomplete']:callback(None)
            result=self._call('inference_configure',configuration=configuration,specification=specification)
            self.event_cursor=boundary
            self.inference_session=result['session_id']
            self.capture_backend=configuration.get('recording_backend')=='capture_session_v1'
            self.recording_error=None;self.recording=None
            self.inference_callbacks=dict(created=created,metric=metric,record=record,predict=predict,predict_chunk=predict_chunk)
            if self.event_thread is None:
                self.event_thread=threading.Thread(target=self._events,name='owner-business-events',daemon=True)
                self.event_thread.start()
            return result
    def push_observation(self,value,support,epoch):
        if self.inference_session is None:return
        from .inference_owner import digest
        key=digest(value['frame_ids'])
        with self.lock:
            self.observations[key]=value
            while len(self.observations)>32:self.observations.popitem(last=False)
        metadata={k:deepcopy(value[k])for k in('source_time','device_state14','frame_ids','feedback_sample_time_s')}
        self._call('inference_observation',session_id=self.inference_session,observation=metadata,support=support,command_epoch=epoch)
    def _events(self):
        from .inference_owner import digest
        from .owner_inference import RemoteRunner,plain
        while not self.event_closed.is_set():
            with self.inference_lock:
                try:
                    batch=self._call('events',after=self.event_cursor)
                    if batch['incomplete']:self.recording_error='control event cursor overflow; evidence incomplete'
                    if batch['incomplete']and getattr(self,'capture_result_handler',None):self.capture_result_handler(None)
                    for row in batch['events']:
                        self.event_cursor=row['sequence']
                        data=row['value'];callbacks=self.inference_callbacks
                        if row['event']=='control_capture_sealed':
                            callback=getattr(self,'capture_result_handler',None)
                            if callback is not None:callback(data)
                            continue
                        if row['event']!='owner_inference'or data['session_id']!=self.inference_session or callbacks is None:continue
                        kind=data['kind']
                        if kind=='created':
                            identity=data['execution_instance_id'];runner=RemoteRunner(self,identity)
                            self.runners={identity:runner};self.refresh()
                            if getattr(self,'capture_backend',False):
                                callbacks['created'](self.owner,runner,data['configuration'],data['control_policy'])
                            else:callbacks['created'](self.owner,runner,data['configuration'])
                        elif kind=='metric':callbacks['metric'](data['name'],data['value'])
                        elif kind=='record':
                            callbacks['record'](data['name'],data['value'])
                        elif kind=='recording_stop':
                            if self.recording and not getattr(self,'capture_backend',False):
                                self.recording.finish(data['status'],data.get('reason'))
                        elif kind=='query':
                            request=data['request'];identity=request['request_id'];session=self.inference_session
                            def query(request=request,identity=identity,session=session,callbacks=callbacks,context=deepcopy(data.get('consumer_context'))):
                                try:
                                    with self.lock:observation=self.observations.get(digest(request['observation']['frame_ids']))
                                    if observation is None:raise ValueError('original observation images are no longer retained')
                                    request['observation']['images']=observation['images']
                                    request['applied_history']=__import__('numpy').asarray(request['applied_history'],dtype='float32').reshape(-1,14)
                                    if callbacks.get('predict_chunk')is not None:
                                        if not isinstance(context,dict):raise ValueError('Owner query context missing')
                                        response=plain(callbacks['predict_chunk'](request,context))
                                    else:response=plain(callbacks['predict'](request))
                                    error=None
                                except Exception as exc:response=None;error=str(exc)
                                try:self._call('inference_result',session_id=session,request_id=identity,response=response,error=error)
                                except (ValueError,ConnectionError):pass  # Revoked generation is never retried.
                            threading.Thread(target=query,name='business-model-query',daemon=True).start()
                    value=self._call('inference_state');runner=self.runners.get(value['execution_instance_id'])
                    if runner is not None and value['snapshot']is not None:runner.value=value['snapshot']
                    if self.recording is not None:self._recording_facts()
                    # Refresh state and recording health once per batch even
                    # under continuous traffic, then drain without sleeping.
                    if self.event_cursor<batch['latest']:continue
                except Exception as error:self.recording_error=str(error)
            self.event_closed.wait(.02)
    def _recording_facts(self):
        recording=self.recording
        prepared=bool(recording is not None and recording.prepared.is_set())
        done=bool(recording is not None and recording.done.is_set())
        thread=getattr(recording,'thread',None)
        identity=({'execution_instance_id':recording.execution_instance_id} if getattr(self,'capture_backend',False) and recording is not None else {})
        error=self.recording_error or getattr(recording,'error',None)
        state=recording.snapshot()if getattr(self,'capture_backend',False)and recording is not None else {}
        ros=getattr(self,'data_plane',None)is not None
        if not ros:error=error or state.get('error')
        elif state.get('writer_exited')and not state.get('sealed'):error=error or state.get('error')or'recording writer exited without seal'
        fields=dict(session_id=self.inference_session,prepared=prepared,done=done,
            alive=thread is None or thread.is_alive(),error=error,ingress_closed=state.get('ingress_closed')is True,**identity)
        description=state.get('health_source')
        if ros and description is not None and getattr(self,'_recording_health_sent',None)!=identity:
            fields.update(health_source=description,writer_identity=state['writer_identity'])
        if fields==getattr(self,'_recording_ack',None):return
        self._call('inference_recording',**fields);self._recording_ack=fields
        if description is not None:self._recording_health_sent=identity
    def snapshot(self,credential):return self.refresh(self._business(credential))
    def fact_snapshot(self):return self.refresh()
    def submit(self,credential,command):
        from .control_plan import RawPreview,ArtifactPreview
        business=self._business(credential)
        if isinstance(command,(RawPreview,ArtifactPreview))and self.channel.descriptor.get('frozen_preview_transport')!=PREVIEW_FILES:
            raise ValueError('independent Owner lacks frozen-preview file transport; no motion submitted')
        wire,directory=_stage_preview(command,self.descriptor.parent)
        unknown=False
        try:
            result=self._call('submit',business=business,command=wire)
            if isinstance(result,dict)and isinstance(result.get('token'),str):self.review_action=(result['token'],command.get('action'))
            return result
        except ControlCallUnknown as error:
            unknown=True
            if directory is not None:
                try:_private_json(directory/'unknown-receipt.json',dict(request_id=error.request_id,wall_ns=time.time_ns(),do_not_replay=True))
                except OSError:pass  # Retain inputs and the original unknown-result exception even if journaling fails.
            raise
        finally:
            # The compiler copied and pinned its own audit inputs before a known
            # reply. Unknown IPC completion retains the files and is never retried.
            if directory is not None and not unknown:
                for path in directory.iterdir():path.unlink()
                directory.rmdir()
    def request_stop(self):return self._call('stop')
    def handoff(self,credential,target,*,active=None):
        if active is not None and not active():raise ValueError('handoff revoked')
        result=self._call('handoff',business=self._business(credential),target=target);self.refresh();return result
    def update_execution_settings(self,credential,payload,*,expected_revision=None):
        arguments=dict(business=self._business(credential),payload=payload)
        if expected_revision is not None:arguments['expected_revision']=expected_revision
        result=self._call('execution_settings',**arguments);self.refresh();return result
    def execution_settings_snapshot(self,business):return self._call('settings_snapshot',business=business)
    def control_policy_snapshot(self,business):return self._call('control_policy_snapshot',business=business)
    @property
    def supports_control_policy(self):
        return True  # Required by the exact Owner protocol at attachment.
    def update_control_policy(self,credential,payload):
        result=self._call('control_policy',business=self._business(credential),payload=payload)
        self.refresh()
        return result
    def request_release(self,credential,command=None,*,maintenance=False,actor=None):
        if command not in(None,{'action':'release','confirmation':'ARMS_SUPPORTED_RELEASE'}):raise ValueError('supported release required')
        if actor is not None:return self.control_release(actor)
        if self.environment=='physical':raise PermissionError('physical release requires maintenance capability')
        return self._call('submit',business=self._business(credential),command=dict(action='release',confirmation='ARMS_SUPPORTED_RELEASE'))
    def notify_calibration_installed(self,snapshot_id):
        result=self._call('calibration_installed',snapshot_id=snapshot_id);self.refresh();return result
    def revoke_inference(self):return self._call('revoke_inference')
    def begin_maintenance(self):
        import sys
        identity=_identity()if sys.platform=='linux'else None
        self.lease=self._call('maintenance_begin',client_identity=identity);self.refresh()
    def end_maintenance(self):
        if self.lease is not None:self._call('maintenance_end',token=self.lease['token']);self.lease=None
        self.refresh()
    def _actor_call(self,operation,actor,**arguments):
        actor.require(self)
        expected={'release':'release_control','recover':'recover_control','verify_hold':'verify_hold','admit_collection':'ops_prepare_collection'}
        if actor.action!=expected[operation]:raise PermissionError('maintenance action does not authorize this control operation')
        if self.lease is None:raise PermissionError('Owner maintenance lease required')
        result=self._call(operation,token=self.lease['token'],**arguments);self.refresh();return result
    def control_release(self,actor):return self._actor_call('release',actor,arms_supported=True)
    def control_recover(self,actor):return self._actor_call('recover',actor)
    def verify_hold(self,actor):return self._actor_call('verify_hold',actor,arms_supported=True)
    def admit_business(self,target,*,active=None,expected_holder=None,maintenance_actor=None):
        if active is not None and not active():raise ValueError('business admission revoked')
        if expected_holder is not None and expected_holder!=self.holder:raise ValueError('holder changed')
        if maintenance_actor is not None:
            if target!='collection':raise ValueError('only collection maintenance admission is declared')
            return self._actor_call('admit_collection',maintenance_actor)
        return self.handoff(self.clients[self.holder],target,active=active)
    def retire_protected(self):
        if self.lease is None:raise PermissionError('Owner maintenance lease required')
        result=self._call('retire_protected',token=self.lease['token'])
        # Successful retirement consumes the service-side lease as well.
        self.protected_retirement=result;self.fd=None;self.lease=None
        return result
    def close(self):
        if self.state.get('connected'):raise ValueError('Owner still acquired; business detach is separate from release')
        self.detach();self.fd=None
    def detach(self):
        self.closed.set();self.event_closed.set()
        if getattr(self,'recording_health',None)is not None:self.recording_health.close();self.recording_health=None
        if getattr(self,'ros_state',None) is not None:self.ros_state.close()
    def monitor(self,closed):
        while not closed.is_set()and not self.closed.is_set():
            try:self.refresh();self.observation_error=None
            except (OSError,ValueError,ConnectionError)as error:self.observation_error=str(error)
            if self.release_receipt and self.release_receipt.get('status')=='released':break
            closed.wait(.05)
    def attach_recording(self,recording):
        self.recording=recording
        if recording is not None and self.inference_session is None:
            raise ValueError('recording requires a current declared inference lease')
