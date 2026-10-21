"""Private control service: one Runtime/Owner, independent of business lifetime.

The protocol transports declared control operations and cached facts, never
Python callables, arbitrary attributes, camera pixels or SDK objects.
"""
from collections import OrderedDict,deque
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import secrets
import socket
import socketserver
import stat
import threading
import time

LIMIT=4*1024*1024
SCHEMA='workbench.control_owner.v1'
READS={'inspect','snapshot','inputs','receipt','events','inference_state','settings_snapshot'}


def _encode(value):
    raw=json.dumps(value,allow_nan=False,separators=(',',':')).encode()+b'\n'
    if len(raw)>LIMIT:raise ValueError('control IPC message exceeds bounded size')
    return raw


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
        if value.get('schema')!=SCHEMA or value.get('environment')not in('physical','fake'):
            raise ValueError('explicit control service identity required')
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
            return result['result']
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
        service=self
        class Handler(socketserver.StreamRequestHandler):
            def handle(self):
                self.connection.settimeout(10.)
                request=None
                try:
                    raw=self.rfile.readline(LIMIT+1)
                    if not raw or len(raw)>LIMIT:raise ValueError('bounded control request required')
                    request=json.loads(raw);result=service.dispatch(request)
                    response=dict(ok=True,request_id=request['request_id'],result=result)
                except Exception as error:
                    response=dict(ok=False,request_id=(request or{}).get('request_id'),error_type=type(error).__name__,error=str(error))
                try:self.wfile.write(_encode(response))
                except (BrokenPipeError,ConnectionResetError):pass
        class Server(socketserver.ThreadingUnixStreamServer):
            daemon_threads=True
        self.server=Server(str(socket_path),Handler);os.chmod(socket_path,0o600)
        token_path=self.root/'control-token'
        with os.fdopen(os.open(token_path,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600),'w')as stream:stream.write(self.token)
        descriptor=dict(schema=SCHEMA,instance_id=self.instance,socket_path=str(socket_path),token_file=str(token_path),
            environment=device.environment,device_id=device.id,identity=self.identity,profile_sha256=profile_sha256)
        _private_json(self.descriptor_path,descriptor)

    def event(self,event_name,**value):
        with self.event_lock:
            self.event_sequence+=1
            self.events.append(dict(sequence=self.event_sequence,event=event_name,value=value))

    def inspect(self,business=None):
        state=self.device.snapshot(business)
        state['control_owner']=dict(schema=SCHEMA,instance_id=self.instance,identity=self.identity,
            runtime_nonce=self.device.runtime.nonce if self.device.runtime else None,profile_sha256=self.profile_sha256,
            tick_owned_by_service=True,business_lifetime_independent=True)
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
            return runtime.control_inputs()
        if operation=='settings_snapshot':
            if runtime is None:raise ValueError('control Owner not acquired')
            return runtime.execution_settings_snapshot(args['business'])
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
                rows=[];size=0
                for row in self.events:
                    if row['sequence']<=after:continue
                    count=len(_encode(row))
                    if rows and(size+count>LIMIT//2 or len(rows)>=128):break
                    rows.append(deepcopy(row));size+=count
                return dict(events=rows,latest=self.event_sequence,
                    incomplete=bool(rows and rows[0]['sequence']>after+1))
        raise ValueError('unknown read operation')

    def _write(self,operation,args):
        runtime=self.device.runtime
        if operation=='acquire':
            if args!={'arms_supported':True}:raise ValueError('current supported-arms fact required')
            if runtime is not None:raise ValueError('Owner already acquired; attach without reconnecting')
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
            result=runtime.configure_inference(args['configuration'],candidate.factory)
            self.inference=candidate
            return dict(result,session_id=candidate.session_id)
        if operation in('inference_observation','inference_result','inference_recording'):
            if self.inference is None or args.get('session_id')!=self.inference.session_id:
                raise ValueError('inference configuration changed')
            return self.inference.update(operation,args)
        if operation=='submit':
            business=args.get('business');command=args.get('command')
            if business not in self.device.credentials or not isinstance(command,dict):raise ValueError('registered business command required')
            if self.device.environment=='physical'and command.get('action')in('release','recover'):
                raise ValueError('control lifecycle requires maintenance capability')
            return self.device.submit(business,command)
        if operation=='handoff':
            return runtime.handoff(self.device.credentials[args['business']],args['target'])
        if operation=='execution_settings':
            return runtime.update_execution_settings(self.device.credentials[args['business']],args['payload'])
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
        self.server.shutdown();self.server.server_close()
        Path(self.server.server_address).unlink(missing_ok=True)


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
        # Physical Owner and this client share host monotonic time. This never
        # refreshes the recorded feedback timestamp or substitutes a heartbeat.
        return time.monotonic()
    def recording_failed(self,error):self.client.recording_error=str(error)


class ControlRuntimeClient:
    """Business-side finite control API. It owns no tick thread or native SDK."""
    remote_control_owner=True
    def __init__(self,descriptor,*,hardware_binding=None):
        self.channel=ControlChannel(descriptor);self.lock=threading.RLock();self.state={};self.inputs={}
        self.nonce=None;self.fd=None;self.clients={};self.runners={};self.recording=None;self.recording_error=None;self.observation_error=None
        self.lease=None;self.protected_retirement=None;self.descriptor=Path(descriptor)
        self.inference_lock=threading.RLock();self.inference_session=None;self.inference_callbacks=None;self.event_cursor=0
        self.observations=OrderedDict();self.event_closed=threading.Event();self.event_thread=None
        self.device_id=self.channel.descriptor['device_id'];self.environment=self.channel.descriptor['environment']
        self.owner=OwnerFacts(self);self.hardware_binding=hardware_binding;self.closed=threading.Event()
        self.refresh()

    def _call(self,operation,**arguments):
        return self.channel.call(operation,expected_nonce=self.nonce,**arguments)

    def refresh(self,business=None):
        with self.lock:
            # Order response installation as well as reads: a slower old poll
            # must not overwrite a newer handoff/epoch in the business cache.
            value=self.channel.call('snapshot',expected_nonce=self.nonce,business=business)
            nonce=value.get('runtime_nonce')
            if self.nonce is not None and nonce is not None and nonce!=self.nonce:raise ValueError('control Owner changed; explicit attachment required')
            if nonce is not None:self.nonce=nonce
            self.state=value
            self.fd=1 if nonce is not None and not(value.get('release_receipt')or{}).get('status')=='released'else None
        return deepcopy(value)

    def acquire(self):
        if not self.state.get('connected'):
            self.state=self.channel.call('acquire',arms_supported=True);self.nonce=self.state.get('runtime_nonce');self.fd=1
        deadline=time.monotonic()+4
        while time.monotonic()<deadline:
            self.refresh()
            if (self.state.get('feedback_healthy')and self.state.get('stationary'))or 'FAULT'in str(self.state.get('state')):break
            time.sleep(.02)
        if self.hardware_binding is not None and self.state.get('feedback_healthy')and self.state.get('stationary'):
            self._call('hardware_binding',path=str(self.hardware_binding))
        self.inputs=self._call('inputs')
        return self

    def register_business(self,business):
        if business not in('calibration','collection','inference')or business in self.clients:raise ValueError('one registered business identity required')
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

    def control_inputs(self):
        value=self._call('inputs');self.inputs=value;return value
    def configure_remote_inference(self,configuration,specification,*,created,metric,record,predict):
        with self.inference_lock:
            result=self._call('inference_configure',configuration=configuration,specification=specification)
            self.inference_session=result['session_id']
            self.recording_error=None;self.recording=None
            self.inference_callbacks=dict(created=created,metric=metric,record=record,predict=predict)
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
                    for row in batch['events']:
                        self.event_cursor=row['sequence']
                        data=row['value'];callbacks=self.inference_callbacks
                        if row['event']!='owner_inference'or data['session_id']!=self.inference_session or callbacks is None:continue
                        kind=data['kind']
                        if kind=='created':
                            identity=data['execution_instance_id'];runner=RemoteRunner(self,identity)
                            self.runners={identity:runner};self.refresh()
                            callbacks['created'](self.owner,runner,data['configuration'])
                        elif kind=='metric':callbacks['metric'](data['name'],data['value'])
                        elif kind=='record':
                            callbacks['record'](data['name'],data['value'])
                            self._recording_facts()
                        elif kind=='recording_stop':
                            if self.recording:self.recording.finish(data['status'],data.get('reason'))
                        elif kind=='query':
                            request=data['request'];identity=request['request_id'];session=self.inference_session
                            def query(request=request,identity=identity,session=session,callbacks=callbacks):
                                try:
                                    with self.lock:observation=self.observations.get(digest(request['observation']['frame_ids']))
                                    if observation is None:raise ValueError('original observation images are no longer retained')
                                    request['observation']['images']=observation['images']
                                    request['applied_history']=__import__('numpy').asarray(request['applied_history'],dtype='float32').reshape(-1,14)
                                    response=plain(callbacks['predict'](request));error=None
                                except Exception as exc:response=None;error=str(exc)
                                try:self._call('inference_result',session_id=session,request_id=identity,response=response,error=error)
                                except (ValueError,ConnectionError):pass  # Revoked generation is never retried.
                            threading.Thread(target=query,name='business-model-query',daemon=True).start()
                    value=self._call('inference_state');runner=self.runners.get(value['execution_instance_id'])
                    if runner is not None and value['snapshot']is not None:runner.value=value['snapshot']
                    if self.recording is not None:self._recording_facts()
                except Exception as error:self.recording_error=str(error)
            self.event_closed.wait(.02)
    def _recording_facts(self):
        recording=self.recording
        prepared=bool(recording is not None and recording.prepared.is_set())
        done=bool(recording is not None and recording.done.is_set())
        thread=getattr(recording,'thread',None)
        self._call('inference_recording',session_id=self.inference_session,prepared=prepared,done=done,
            alive=thread is None or thread.is_alive(),error=self.recording_error or getattr(recording,'error',None))
    def snapshot(self,credential):return self.refresh(self._business(credential))
    def fact_snapshot(self):return self.refresh()
    def submit(self,credential,command):return self._call('submit',business=self._business(credential),command=command)
    def request_stop(self):return self._call('stop')
    def handoff(self,credential,target,*,active=None):
        if active is not None and not active():raise ValueError('handoff revoked')
        result=self._call('handoff',business=self._business(credential),target=target);self.refresh();return result
    def update_execution_settings(self,credential,payload):
        result=self._call('execution_settings',business=self._business(credential),payload=payload);self.refresh();return result
    def execution_settings_snapshot(self,business):return self._call('settings_snapshot',business=business)
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
        result=self._call('retire_protected',token=self.lease['token']);self.protected_retirement=result;self.fd=None;return result
    def close(self):
        if self.state.get('connected'):raise ValueError('Owner still acquired; business detach is separate from release')
        self.detach();self.fd=None
    def detach(self):self.closed.set();self.event_closed.set()
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
