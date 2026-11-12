"""Private inherited-socket SDK worker. Business HTTP never owns native controllers."""
from copy import deepcopy
import json
import math
import mmap
import os
from pathlib import Path
import secrets
import socket
import stat
import subprocess
import sys
import threading
import time
from .release_trace import memory_file, archive_memory_file
from .execution_settings import NATIVE_WATCHDOG_TIMEOUT_S

LIMIT = 65536
EXECUTION_CONTEXT_SCHEMA = 'workbench.sdk-execution-context.v2'
METHODS = {'connect', 'close', 'begin', 'send', 'set_damping_hold',
           'read_calibration_state', 'get_bimanual_joint_position_limits', 'read_gains',
           'enable_collection_arm','queue_reference','configure_motion_gains','read_execution_output','set_execution_gains',
           'bind_execution_context','start_native_home'}


class SDKCommandRevoked(RuntimeError):
    """An expected control-generation barrier, not a broken SDK transport."""


class ReferenceWindowRejected(ValueError):
    """An explicit adapter admission rejection; no SDK/transport failure is implied."""


class SDKCommandRejected(RuntimeError):
    """Keep the channel usable so the Owner can revoke motion and request its stop."""


class SDKNativeFault(RuntimeError):
    """Retained native fault; never eligible for software in-place recovery."""
    def __init__(self, faults):
        self.faults=validate_native_faults(faults,required=True)
        super().__init__('native emergency latched: '+', '.join(side+': '+r['reason'] for side,r in self.faults.items()))


def validate_native_faults(value, *, required=False):
    fields={'library_build_id','controller_instance_id','reason','latched','feedback_valid',
            'damping_committed','joint','observed_value','latched_monotonic_ns'}
    reasons={'joint_state_invalid','interpolated_command_invalid','joint_torque_invalid',
             'gripper_state_invalid','over_current'}
    if not isinstance(value,dict) or set(value)-{'left','right'} or required and not value:
        raise ValueError('bounded native fault map required')
    for row in value.values():
        if (not isinstance(row,dict) or set(row)!=fields or row['latched']is not True
                or not isinstance(row['reason'],str)or row['reason']not in reasons or type(row['feedback_valid'])is not bool
                or type(row['damping_committed'])is not bool
                or any(not isinstance(row[k],str)or not 0<len(row[k])<=128 for k in ('library_build_id','controller_instance_id'))
                or type(row['joint'])is not int or not -1<=row['joint']<6
                or type(row['observed_value'])not in(int,float)or not math.isfinite(row['observed_value'])
                or type(row['latched_monotonic_ns'])is not int or row['latched_monotonic_ns']<=0):
            raise ValueError('invalid native fault report')
    return deepcopy(value)


def receive(connection, pending):
    while b'\n' not in pending:
        part = connection.recv(8192)
        if not part:raise ConnectionError('SDK control worker disconnected')
        pending += part
        if len(pending)>LIMIT:raise ValueError('SDK message exceeds size limit')
    line, pending = pending.split(b'\n', 1)
    return json.loads(line), pending


def send(connection, value):
    raw = json.dumps(value, allow_nan=False, separators=(',', ':')).encode()+b'\n'
    if len(raw)>LIMIT:raise ValueError('SDK message exceeds size limit')
    connection.sendall(raw)


def serve_worker(connection, backend, epoch=None):
    """No autonomous heartbeat: only an Owner feedback request renews the guard."""
    from .gc_timing import install,recent
    install()
    pending = b'';previous = 0;deadline = 0.;generation=None
    context_generation = None
    bound_context = None
    def gate():
        if time.monotonic()>deadline:raise ValueError('expired SDK command')
        if epoch is not None and epoch[:]!=generation.encode():raise SDKCommandRevoked('SDK command epoch revoked')
    backend.command_guard = gate
    while True:
        request, pending = receive(connection, pending)
        if (not isinstance(request, dict) or set(request)!={'id','method','args','kwargs','deadline','generation'}
                or type(request['id']) is not int or request['id']!=previous+1
                or request['method'] not in METHODS or not isinstance(request['args'],list)
                or not isinstance(request['kwargs'],dict)
                or not isinstance(request['generation'],str) or len(request['generation'])!=32
                or type(request['deadline']) not in (int,float)
                or not 0<=request['deadline']-time.monotonic()<=12):
            raise ValueError('invalid or expired SDK request')
        previous=request['id'];deadline=request['deadline'];generation=request['generation']
        cancel_before=getattr(backend,'reference_cancel_sequence',0)
        try:
            dispatch_started=time.monotonic();dispatch_cpu=time.thread_time_ns()
            capabilities=getattr(backend,'execution_context_capabilities',None)
            if capabilities is not None and capabilities!={'schema':EXECUTION_CONTEXT_SCHEMA}:
                raise ValueError('SDK execution context protocol version mismatch; backend operation refused')
            context_supported = capabilities == {'schema': EXECUTION_CONTEXT_SCHEMA}
            if request['method'] == 'bind_execution_context':
                from .execution_settings import validate_execution_context
                gate()
                if not context_supported or len(request['args']) != 1 or request['kwargs']:
                    raise ReferenceWindowRejected('SDK execution context binding is unavailable or malformed')
                try: context = validate_execution_context(request['args'][0])
                except ValueError as error: raise ReferenceWindowRejected(str(error)) from None
                if context_generation == generation and context != bound_context:
                    raise ReferenceWindowRejected('execution context change requires a new generation')
            if request['method'] == 'queue_reference' and context_supported:
                try: gate()
                except BaseException:
                    backend.set_damping_hold(manual=False)
                    backend.reference_cancel_sequence = getattr(backend, 'reference_cancel_sequence', 0) + 1
                    raise
                if context_generation != generation:
                    raise ReferenceWindowRejected('current generation requires execution context binding')
                required = {'reference_id', 'active_arm', 'host_times_s', 'q_sdk14', 'dq_sdk14'}
                if (len(request['args']) != 1 or request['kwargs'] or not isinstance(request['args'][0], dict)
                        or set(request['args'][0]) not in (required, required | {'torque_bias_sdk14'})):
                    raise ReferenceWindowRejected('minimal bound reference window required')
            if request['method'] == 'bind_execution_context' and context_generation == generation:
                value = dict(bound=True)
            else:
                value=getattr(backend,request['method'])(*request['args'],**request['kwargs'])
            if request['method'] == 'bind_execution_context':
                gate()
                context_generation, bound_context = generation, context
            if request['method'] == 'connect' and context_supported:
                value = dict(value or {}, execution_context={'schema': EXECUTION_CONTEXT_SCHEMA})
            if request['method']=='read_calibration_state'and isinstance(value,dict):
                value['sdk_worker_read_timing']=dict(dispatch_started_s=dispatch_started,
                    execute_s=time.monotonic()-dispatch_started,cpu_s=(time.thread_time_ns()-dispatch_cpu)/1e9,
                    gc=recent(),pid=os.getpid(),thread_id=threading.get_native_id())
            if hasattr(value,'tolist'):value=value.tolist()
            response=dict(id=previous,result=value,connected=bool(backend.is_connected),
                          release_steps=list(getattr(backend,'release_steps',())))
        except Exception as error:
            response=dict(id=previous,error=f'{type(error).__name__}: {error}'[:1000])
            native_type=getattr(getattr(backend,'_arx5',None),'NativeEmergencyLatched',())
            if isinstance(error,native_type) and request['method']not in('connect','close') and backend.is_connected:
                # Native latching is not a dead transport. Confirm fault output and
                # stop the other arm before returning a retained-fault response.
                backend.set_damping_hold(manual=False)
                faults=validate_native_faults(backend.native_faults(),required=True)
                if not all(r['feedback_valid'] and r['damping_committed'] for r in faults.values()):
                    raise RuntimeError('native fault has no valid retained damping') from error
                response.update(code='native_fault',faults=faults,connected=True,
                    release_steps=list(getattr(backend,'release_steps',())))
            guarded_reference=request['method']in('queue_reference','set_execution_gains','bind_execution_context','start_native_home')or(request['method']=='configure_motion_gains'and request['kwargs'].get('preserve_hold')is True)or(request['method']in('begin','set_damping_hold')and request['kwargs'].get('preserve_jaws')is True)
            if isinstance(error,ReferenceWindowRejected)and (guarded_reference or request['method']=='read_execution_output'):
                response.update(code='reference_rejected',connected=bool(backend.is_connected),
                    release_steps=list(getattr(backend,'release_steps',())))
            if isinstance(error,SDKCommandRevoked)and guarded_reference:
                response.update(code='reference_revoked',connected=bool(backend.is_connected),
                    release_steps=list(getattr(backend,'release_steps',())),
                    native_future_cancelled=getattr(backend,'reference_cancel_sequence',0)>cancel_before)
        if request['method']=='queue_reference':
            returned=time.monotonic()
            response['reference_timing']=dict(worker=dict(started_s=dispatch_started,returned_s=returned,
                wall_s=returned-dispatch_started,cpu_s=(time.thread_time_ns()-dispatch_cpu)/1e9),
                native=getattr(backend,'last_reference_timing',None))
        send(connection,response)
        if request['method']=='close' and 'error' not in response:return


class SDKProcess:
    """One explicitly acquired backend, with bounded RPCs and cached health reads."""
    def __init__(self, profile):
        from .execution_settings import Guard,control_parameters,defaults,policy_hash
        parameters=control_parameters(defaults('inference'),business='inference')
        self._watchdog_guard=Guard.bind(dict(revision=None,policy_hash=policy_hash(parameters),parameters=parameters),{})
        self.profile=deepcopy(profile);self.passive_jaws=tuple(profile.get('disabled_grippers',()))
        self.target_interpreter_profile=profile.get('target_interpreter_profile')
        self.motion_gain_profile=profile.get('motion_gain_profile') or self.target_interpreter_profile or 'normal_X5'
        self.reference_start_allowance=deepcopy(profile.get('reference_start_allowance'))
        self.native_reference_api=profile.get('native_reference_api','legacy_recompute')
        self.native_gripper_calibration_id=(profile.get('native_gripper_calibration')or{}).get('sha256')
        self.execution_context_capabilities = None
        self._bound_execution_context = None
        from .sdk_quiesce import ec_torque_units,dm_torque_units
        self.torque_unit_contract=(dm_torque_units(profile.get('dm_torque_contract'),can_interfaces=profile.get('can_interfaces'),robot_id=profile.get('robot_id'))
                                  or ec_torque_units(profile.get('ec_torque_contract')) or 'legacy_sdk_v1')
        from .gravity_calibration import load_gravity_calibration
        gravity=load_gravity_calibration(profile.get('sdk_gravity_calibration'))
        self.gravity_calibration_id=None if gravity is None else gravity['id']
        self.command_guard=lambda:None
        self.lock=threading.Lock();self.process=None;self.connection=None
        self.sequence=0;self.pending=b'';self.connected=False;self.release_steps=[]
        self.heartbeat=None;self.failure=None;self.guard_record=None;self.attempt=None
        self.generation=secrets.token_hex(16);self.epoch=None;self.epoch_file=None
        self.supervisor_owner=None
        self._joint_limits=None
        self.last_reference_timing=None
        self.output_capture=None
        self.worker_pid=None

    def revoke_output(self):
        generation = secrets.token_hex(16)
        self._bound_execution_context = None
        if self.epoch is not None:self.epoch[:]=generation.encode()
        self.generation = generation

    def dispose(self):
        if self.process is not None and self.process.poll() is None:raise RuntimeError('SDK worker exit not confirmed')
        if self.connection is not None:self.connection.close()
        if self.epoch is not None:self.epoch.close();self.epoch=None
        if self.epoch_file is not None:self.epoch_file.close();self.epoch_file=None

    def _start(self):
        if self.process is not None:raise RuntimeError('failed SDK worker requires explicit retirement; no automatic restart')
        self.attempt=secrets.token_hex(16)
        root=Path(self.profile['storage_root'])/'sdk-workers'/self.attempt
        root.mkdir(mode=0o700,parents=True)
        profile=root/'profile.json'
        with os.fdopen(os.open(profile,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600),'w') as stream:
            json.dump(self.profile,stream,allow_nan=False)
        self.guard_record=root/'guard.json'
        self.epoch_file=memory_file('workbench-sdk-command-epoch',32)
        self.epoch=mmap.mmap(self.epoch_file.fileno(),32);self.epoch[:]=self.generation.encode()
        self.connection,peer=socket.socketpair()
        owner=os.getpid()
        command=[sys.executable,'-B',str(Path(__file__).resolve().parents[1]/'scripts/serve_sdk_control.py'),
                 '--profile',str(profile),'--record',str(self.guard_record),'--fd',str(peer.fileno()),'--owner',str(owner),
                 '--epoch-fd',str(self.epoch_file.fileno())]
        launched=threading.Event();errors=[]
        def own_supervisor():
            # Linux PDEATHSIG follows the creating *thread*. Keep that thread
            # alive until the supervisor exits; HTTP request threads are transient.
            # This waiter never sends heartbeats or touches the SDK command stream.
            log=None
            try:
                log=memory_file('workbench-sdk-output',4*1024**2)
                self.process=subprocess.Popen(command,pass_fds=(peer.fileno(),self.epoch_file.fileno()),stdout=log,stderr=log)
            except BaseException as error:errors.append(error)
            finally:launched.set()
            try:
                if self.process is not None:self.process.wait()
            finally:
                from .gc_timing import release_control
                release_control(self)
            if log is not None:
                try:self.output_capture=archive_memory_file(log,root/'worker.log')
                except OSError as error:self.output_capture=dict(archive_error=str(error))
                finally:log.close()
        self.supervisor_owner=threading.Thread(target=own_supervisor,daemon=True,name='sdk-supervisor-owner')
        from .gc_timing import acquire_control,release_control
        acquire_control(self)
        try:
            self.supervisor_owner.start();launched.wait()
            if errors:raise errors[0]
        except BaseException:
            if self.process is None:release_control(self)
            self.connection.close();self.connection=None;raise
        finally:peer.close()

    @property
    def is_connected(self):
        return self.connected and self.process is not None and self.process.poll() is None and self.failure is None

    @property
    def terminal_failure(self):
        return self.failure or ('SDK 控制进程已退出' if self.connected and self.process.poll() is not None else None)

    def call(self, method, *args, _generation=None, **kwargs):
        if method not in METHODS:raise ValueError('unknown SDK method')
        generation=self.generation if _generation is None else _generation
        guarded_reference=method in('queue_reference','set_execution_gains','bind_execution_context','start_native_home')or(method=='configure_motion_gains'and kwargs.get('preserve_hold')is True)or(method in('begin','set_damping_hold')and kwargs.get('preserve_jaws')is True)
        if method in ('send','begin','enable_collection_arm','queue_reference')or guarded_reference:self.command_guard()
        requested=time.monotonic();requested_cpu=time.thread_time_ns()
        with self.lock:
            acquired=time.monotonic()
            if method=='queue_reference':self.last_reference_timing=None
            if self.failure:raise RuntimeError(self.failure)
            if self.process is None:
                if method!='connect':raise RuntimeError('SDK not connected')
                self._start()
            if method in('connect','close'):self._joint_limits=None
            if method=='get_bimanual_joint_position_limits' and self._joint_limits is not None:
                if not self.is_connected:raise RuntimeError('SDK not connected; cached limits cannot establish health')
                return deepcopy(self._joint_limits)
            budget=11. if method in ('connect','close') else .1
            self.sequence+=1
            try:
                self.connection.settimeout(budget+.05)
                send(self.connection,dict(id=self.sequence,method=method,args=args,kwargs=kwargs,
                                         deadline=time.monotonic()+budget,generation=generation))
                sent=time.monotonic()
                reply,self.pending=receive(self.connection,self.pending)
                received=time.monotonic()
                if not isinstance(reply,dict) or reply.get('id')!=self.sequence:raise ValueError('SDK response identity differs')
                reference_timing=reply.pop('reference_timing',None)
                if reference_timing is not None:
                    if (method!='queue_reference'or not isinstance(reference_timing,dict)
                            or set(reference_timing)!={'worker','native'}
                            or not isinstance(reference_timing['worker'],dict)
                            or set(reference_timing['worker'])!={'started_s','returned_s','wall_s','cpu_s'}
                            or any(type(v)not in(int,float)or not math.isfinite(v)or v<0 for v in reference_timing['worker'].values())
                            or reference_timing['native']is not None and(not isinstance(reference_timing['native'],dict)
                                or any(not isinstance(reference_timing['native'].get(k),list)or len(reference_timing['native'][k])>32 for k in('calls','stages')))
                            or len(json.dumps(reference_timing,allow_nan=False))>16384):
                        raise ValueError('bounded SDK reference timing required')
                    self.last_reference_timing=dict(reference_timing,client=dict(lock_wait_s=acquired-requested,
                        request_encode_send_s=sent-acquired,response_wait_decode_s=received-sent,
                        total_s=received-requested,request_sent_s=sent,response_received_s=received,
                        cpu_s=(time.thread_time_ns()-requested_cpu)/1e9))
                if 'error' in reply:
                    if reply.get('code')=='native_fault':
                        if (method in('connect','close')or set(reply)!={'id','error','code','faults','connected','release_steps'}
                                or reply['connected']is not True or not isinstance(reply['error'],str)
                                or not isinstance(reply['release_steps'],list)or len(reply['release_steps'])>32):
                            raise ValueError('invalid retained native fault response')
                        faults=validate_native_faults(reply['faults'],required=True)
                        if not all(r['feedback_valid'] and r['damping_committed'] for r in faults.values()):
                            raise ValueError('native fault output not confirmed')
                        self.connected=True;self.release_steps=reply['release_steps']
                        raise SDKNativeFault(faults)
                    if (guarded_reference or method=='read_execution_output')and reply.get('code')=='reference_rejected':
                        if (set(reply)!={'id','error','code','connected','release_steps'}or reply['connected']is not True
                                or not isinstance(reply['error'],str)or not isinstance(reply['release_steps'],list)
                                or len(reply['release_steps'])>32):
                            raise ValueError('invalid SDK admission rejection response')
                        self.connected=True;self.release_steps=reply['release_steps']
                        error=SDKCommandRejected(reply['error']);error.timing=self.last_reference_timing if method=='queue_reference'else None
                        raise error
                    if guarded_reference and reply.get('code')=='reference_revoked'and (reply.get('native_future_cancelled')is True or method=='bind_execution_context'):
                        self.connected=reply['connected'];self.release_steps=reply['release_steps']
                        error=SDKCommandRevoked(reply['error']);error.timing=self.last_reference_timing
                        raise error
                    raise ValueError(reply['error'])
                if (set(reply)!={'id','result','connected','release_steps'} or type(reply['connected']) is not bool
                        or not isinstance(reply['release_steps'],list) or len(reply['release_steps'])>32):
                    raise ValueError('invalid SDK response')
                self.connected=reply['connected'];self.release_steps=reply['release_steps']
                result=reply['result']
                if method=='queue_reference'and isinstance(result,dict)and self.last_reference_timing is not None:
                    result['sdk_reference_timing']=self.last_reference_timing
                if method=='get_bimanual_joint_position_limits':
                    # No RPC method changes limits within this fixed-profile SDK
                    # connection. Keep live feedback/heartbeats on their real path.
                    if (not isinstance(result,list) or len(result)!=14 or any(
                            not isinstance(row,list) or len(row)!=2 or any(type(v)not in(int,float)or not math.isfinite(v)for v in row)
                            or row[0]>row[1] for row in result)):
                        raise ValueError('SDK limits require 14 ordered finite pairs')
                    self._joint_limits=deepcopy(result)
                if method=='connect' and isinstance(result,dict):
                    self.heartbeat=result.get('heartbeat_monotonic_s')
                    capabilities=result.get('execution_context')
                    if capabilities is not None and capabilities!={'schema':EXECUTION_CONTEXT_SCHEMA}:
                        raise ValueError('SDK execution context protocol version mismatch; connection refused')
                    self.execution_context_capabilities = deepcopy(capabilities)
                    self._bound_execution_context = None
                if method=='read_calibration_state':
                    self.heartbeat=(result.get('can_output_guard') or {}).get('heartbeat_monotonic_s')
                    worker_pid=(result.get('sdk_worker_read_timing')or{}).get('pid')
                    if type(worker_pid)is int and worker_pid>0:self.worker_pid=worker_pid
                    result['sdk_client_read_timing']=dict(lock_wait_s=acquired-requested,
                        request_encode_send_s=sent-acquired,response_wait_decode_s=received-sent,
                        total_s=received-requested,request_sent_s=sent,response_received_s=received,
                        cpu_s=(time.thread_time_ns()-requested_cpu)/1e9,pid=os.getpid(),thread_id=threading.get_native_id())
                if method=='close':
                    if self.process.wait(timeout=2)!=0:raise ValueError('SDK supervisor failed during normal close')
                    self.dispose()
                return result
            except (OSError,ValueError,subprocess.SubprocessError) as error:
                # Closing this inherited socket makes the worker exit; its independent
                # guard still owns termination. Never close native SDK in this process.
                self.failure='SDK 控制通信失败：'+str(error);self.connected=False
                self.connection.close()
                failure=RuntimeError(self.failure)
                if method=='queue_reference':failure.timing=self.last_reference_timing
                raise failure from error

    def protection_snapshot(self):
        record={}
        exited=self.process is not None and self.process.poll() is not None
        # Live health comes from the heartbeat RPC and process identity. Regular
        # file reads can block despite O_NONBLOCK; read final receipts only after
        # this supervisor has exited, never under live Owner snapshot locks.
        if self.guard_record is not None and exited:
            try:
                fd=os.open(self.guard_record,os.O_RDONLY|os.O_NOFOLLOW|os.O_NONBLOCK)
                with os.fdopen(fd) as stream:
                    info=os.fstat(stream.fileno())
                    if not stat.S_ISREG(info.st_mode) or info.st_size>LIMIT:raise ValueError('invalid guard record')
                    record=json.load(stream)
            except (OSError,ValueError):pass
        latched=record.get('state')=='latched'
        process_evidence=record.get('process_evidence') or {}
        worker_exited=bool(exited and record.get('supervisor_pid')==self.process.pid
            and type(record.get('child_pid')) is int and record['child_pid']>0 and record['child_pid']!=self.process.pid
            and (process_evidence.get('wait') or {}).get('status')=='returned'
            and type(process_evidence.get('returncode')) is int
            and type(process_evidence.get('reaped_monotonic_ns')) is int)
        active=self.is_connected
        age=None if self.heartbeat is None else max(0.,time.monotonic()-self.heartbeat)
        failed=latched or bool(self.failure) or exited and self.connected
        reason=record.get('reason') or self.failure or ('SDK 控制进程已退出' if failed else None)
        client_failure=record.get('guard_client_failure')
        if (failed and isinstance(client_failure,dict) and client_failure.get('schema')=='guard_ipc_failure.v1'
                and client_failure.get('source_pid')==record.get('child_pid')
                and client_failure.get('self_sigkill_requested') is True and not process_evidence.get('terminations')
                and all(type(client_failure.get(key))is int for key in ('failed_monotonic_ns','started_monotonic_ns'))
                and client_failure['failed_monotonic_ns']>=client_failure['started_monotonic_ns']):
            elapsed=(client_failure['failed_monotonic_ns']-client_failure['started_monotonic_ns'])/1e6
            reason=f"SDK Guard IPC {client_failure.get('action')} {client_failure.get('stage')} · {client_failure.get('exception')} · 等待{elapsed:.1f}ms；已记录SDK请求fail-stop"
        heartbeat_fault=record.get('reason')=='SDK owner heartbeat lost'
        checked=time.monotonic()
        self.watchdog_decision=self._watchdog_guard.evaluate(dict(rule_id='G30',signal_id=self.attempt or 'sdk_unconnected',
            observed=dict(monitored=active or heartbeat_fault,age_s=age,fault_latched=heartbeat_fault),
            source_time=self.heartbeat if self.heartbeat is not None else checked,checked_time=checked),'native_watchdog')
        heartbeat_lost=self.watchdog_decision['verdict']=='reject'
        if exited and self.process.returncode and not heartbeat_fault:
            reason=f'SDK监督进程异常退出（退出码 {self.process.returncode}）；'+(reason or '控制通道断开')
        # A reaped worker no longer owes heartbeats. Its fault still requires
        # explicit recovery; an unconfirmed exit remains unavailable, not healthy.
        heartbeat_status=('not_monitored' if worker_exited else 'unavailable' if exited else
            'lost' if heartbeat_lost else
            'unavailable' if failed else 'healthy' if active else 'not_monitored')
        return dict(schema='workbench.control_protection.v1',attempt_id=self.attempt,timeout_ms=NATIVE_WATCHDOG_TIMEOUT_S*1000,
            heartbeat=dict(status=heartbeat_status,age_s=age if heartbeat_status in ('healthy','lost') else None,
                           monitored=not worker_exited and (active or failed)),
            active_anomaly=not worker_exited and (failed or heartbeat_lost),
            recovery_required=bool(failed),reason=reason,
            guard_client_failure=client_failure,
            event=(dict(id=self.attempt,wall_ns=record.get('wall_ns'),reason=reason,
                        stop_action='sdk_worker_terminated' if worker_exited else 'sdk_output_stop_requested',stop_confirmed=worker_exited,
                        sdk_close_confirmed=False,mechanical_state='not_verified',
                        evidence=str(self.guard_record),diagnostics=record.get('diagnostics'),guard_client_failure=client_failure) if failed else None),
            worker_exit_confirmed=worker_exited,supervisor_exit_confirmed=exited,
            supervisor_pid=self.process.pid if self.process else None,worker_pid=record.get('child_pid',self.worker_pid),
            supervisor_exit_code=self.process.returncode if self.process else None)

    def connect(self):return self.call('connect')
    def close(self):return self.call('close')
    def read_calibration_state(self):return self.call('read_calibration_state')
    def get_bimanual_joint_position_limits(self):return self.call('get_bimanual_joint_position_limits')
    def read_gains(self):return self.call('read_gains')
    def begin(self,*args,**kwargs):return self.call('begin',*args,**kwargs)
    def read_execution_output(self):return self.call('read_execution_output')
    def set_execution_gains(self,rows):return self.call('set_execution_gains',rows)
    def send(self,*args):
        if self.target_interpreter_profile:
            raise ReferenceWindowRejected('ordinary motion requires the shared target interpreter; point-send is disabled')
        return self.call('send',*args)
    def set_damping_hold(self,**kwargs):return self.call('set_damping_hold',**kwargs)
    def start_native_home(self,side):return self.call('start_native_home',side)
    def enable_collection_arm(self,*args):return self.call('enable_collection_arm',*args)
    def bind_execution_context(self, context):
        from .execution_settings import validate_execution_context
        generation = self.generation
        value = validate_execution_context(context)
        if self.execution_context_capabilities != {'schema': EXECUTION_CONTEXT_SCHEMA}:
            raise ReferenceWindowRejected('SDK worker lacks execution context capability')
        self.command_guard()
        if self._bound_execution_context == (generation, value):
            return dict(bound=True)
        if self._bound_execution_context is not None and self._bound_execution_context[0] == generation:
            self.revoke_output()
            generation = self.generation
        result = self.call('bind_execution_context', value, _generation=generation)
        if self.generation != generation:
            raise SDKCommandRevoked('execution context revoked during binding')
        if not isinstance(result, dict) or set(result) != {'bound'} or result['bound'] is not True:
            raise ReferenceWindowRejected('execution context binding was not confirmed')
        self._bound_execution_context = (generation, value)
        return result

    def queue_reference(self, packet):
        generation = self.generation
        if self.execution_context_capabilities is not None:
            if self._bound_execution_context is None or self._bound_execution_context[0] != generation:
                raise SDKCommandRevoked('current generation has no confirmed execution context')
            from .target_interpreter import SCHEMA
            context = self._bound_execution_context[1]
            if (not isinstance(packet, dict) or packet.get('schema') != SCHEMA or packet.get('controller_profile') != context['controller_profile']
                    or packet.get('velocity_source') != context.get('velocity_source')):
                raise ReferenceWindowRejected('reference window differs from bound execution configuration')
            fields = {'schema', 'controller_profile', 'reference_id', 'active_arm', 'host_times_s', 'q_sdk14', 'dq_sdk14', 'torque_bias_sdk14'}
            if set(packet) not in (fields, fields | {'velocity_source'}):
                raise ReferenceWindowRejected('exact source window required')
            bias = packet['torque_bias_sdk14']
            if (not isinstance(bias, list) or len(bias) != len(packet['host_times_s'])
                    or any(not isinstance(row, list) or len(row) != 14
                           or any(type(torque) not in (int, float) or not math.isfinite(torque) for torque in row) for row in bias)):
                raise ReferenceWindowRejected('finite torque bias matching the source window required')
            value = {key: packet[key] for key in ('reference_id', 'active_arm', 'host_times_s', 'q_sdk14', 'dq_sdk14')}
            if any(torque != 0 for row in bias for torque in row):
                value['torque_bias_sdk14'] = packet['torque_bias_sdk14']
            return self.call('queue_reference', value, _generation=generation)
        return self.call('queue_reference', packet)
    def configure_motion_gains(self,value,**kwargs):return self.call('configure_motion_gains',value,**kwargs)
