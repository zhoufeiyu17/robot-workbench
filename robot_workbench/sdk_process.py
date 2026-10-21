"""Private inherited-socket SDK worker. Business HTTP never owns native controllers."""
from copy import deepcopy
import json
import mmap
import os
from pathlib import Path
import secrets
import socket
import stat
import subprocess
import sys
import threading
import tempfile
import time

LIMIT = 65536
METHODS = {'connect', 'close', 'begin', 'send', 'set_damping_hold',
           'read_calibration_state', 'get_bimanual_joint_position_limits', 'read_gains',
           'enable_collection_arm','queue_reference'}


class SDKCommandRevoked(RuntimeError):
    """An expected control-generation barrier, not a broken SDK transport."""


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
    pending = b'';previous = 0;deadline = 0.;generation=None
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
            value=getattr(backend,request['method'])(*request['args'],**request['kwargs'])
            if hasattr(value,'tolist'):value=value.tolist()
            response=dict(id=previous,result=value,connected=bool(backend.is_connected),
                          release_steps=list(getattr(backend,'release_steps',())))
        except Exception as error:
            response=dict(id=previous,error=f'{type(error).__name__}: {error}'[:1000])
            if isinstance(error,SDKCommandRevoked)and request['method']=='queue_reference':
                response.update(code='reference_revoked',connected=bool(backend.is_connected),
                    release_steps=list(getattr(backend,'release_steps',())),
                    native_future_cancelled=getattr(backend,'reference_cancel_sequence',0)>cancel_before)
        send(connection,response)
        if request['method']=='close' and 'error' not in response:return


class SDKProcess:
    """One explicitly acquired backend, with bounded RPCs and cached health reads."""
    def __init__(self, profile):
        self.profile=deepcopy(profile);self.passive_jaws=tuple(profile.get('disabled_grippers',()))
        self.target_interpreter_profile=profile.get('target_interpreter_profile')
        self.command_guard=lambda:None
        self.lock=threading.Lock();self.process=None;self.connection=None
        self.sequence=0;self.pending=b'';self.connected=False;self.release_steps=[]
        self.heartbeat=None;self.failure=None;self.guard_record=None;self.attempt=None
        self.generation=secrets.token_hex(16);self.epoch=None;self.epoch_file=None
        self.supervisor_owner=None

    def revoke_output(self):
        self.generation=secrets.token_hex(16)
        if self.epoch is not None:self.epoch[:]=self.generation.encode()

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
        self.epoch_file=tempfile.TemporaryFile(dir=root);self.epoch_file.truncate(32)
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
            try:
                with (root/'worker.log').open('xb') as log:
                    self.process=subprocess.Popen(command,pass_fds=(peer.fileno(),self.epoch_file.fileno()),stdout=log,stderr=log)
            except BaseException as error:errors.append(error)
            finally:launched.set()
            try:
                if self.process is not None:self.process.wait()
            finally:
                from .gc_timing import release_control
                release_control(self)
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

    def call(self, method, *args, **kwargs):
        if method not in METHODS:raise ValueError('unknown SDK method')
        generation=self.generation
        if method in ('send','begin','enable_collection_arm','queue_reference'):self.command_guard()
        with self.lock:
            if self.failure:raise RuntimeError(self.failure)
            if self.process is None:
                if method!='connect':raise RuntimeError('SDK not connected')
                self._start()
            budget=11. if method in ('connect','close') else .1
            self.sequence+=1
            try:
                self.connection.settimeout(budget+.05)
                send(self.connection,dict(id=self.sequence,method=method,args=args,kwargs=kwargs,
                                         deadline=time.monotonic()+budget,generation=generation))
                reply,self.pending=receive(self.connection,self.pending)
                if not isinstance(reply,dict) or reply.get('id')!=self.sequence:raise ValueError('SDK response identity differs')
                if 'error' in reply:
                    if method=='queue_reference'and reply.get('code')=='reference_revoked'and reply.get('native_future_cancelled')is True:
                        self.connected=reply['connected'];self.release_steps=reply['release_steps']
                        raise SDKCommandRevoked(reply['error'])
                    raise ValueError(reply['error'])
                if (set(reply)!={'id','result','connected','release_steps'} or type(reply['connected']) is not bool
                        or not isinstance(reply['release_steps'],list) or len(reply['release_steps'])>32):
                    raise ValueError('invalid SDK response')
                self.connected=reply['connected'];self.release_steps=reply['release_steps']
                result=reply['result']
                if method=='connect' and isinstance(result,dict):self.heartbeat=result.get('heartbeat_monotonic_s')
                if method=='read_calibration_state':
                    self.heartbeat=(result.get('can_output_guard') or {}).get('heartbeat_monotonic_s')
                if method=='close':
                    if self.process.wait(timeout=2)!=0:raise ValueError('SDK supervisor failed during normal close')
                    self.dispose()
                return result
            except (OSError,ValueError,subprocess.SubprocessError) as error:
                # Closing this inherited socket makes the worker exit; its independent
                # guard still owns termination. Never close native SDK in this process.
                self.failure='SDK 控制通信失败：'+str(error);self.connected=False
                self.connection.close()
                raise RuntimeError(self.failure) from error

    def protection_snapshot(self):
        record={}
        if self.guard_record is not None:
            try:
                fd=os.open(self.guard_record,os.O_RDONLY|os.O_NOFOLLOW|os.O_NONBLOCK)
                with os.fdopen(fd) as stream:
                    info=os.fstat(stream.fileno())
                    if not stat.S_ISREG(info.st_mode) or info.st_size>LIMIT:raise ValueError('invalid guard record')
                    record=json.load(stream)
            except (OSError,ValueError):pass
        latched=record.get('state')=='latched'
        exited=self.process is not None and self.process.poll() is not None
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
        heartbeat_fault=record.get('reason')=='SDK owner heartbeat lost'
        if exited and self.process.returncode and not heartbeat_fault:
            reason=f'SDK监督进程异常退出（退出码 {self.process.returncode}）；'+(reason or '控制通道断开')
        # A reaped worker no longer owes heartbeats. Its fault still requires
        # explicit recovery; an unconfirmed exit remains unavailable, not healthy.
        heartbeat_status=('not_monitored' if worker_exited else 'unavailable' if exited else
            'lost' if heartbeat_fault or active and (age is None or age>.1) else
            'unavailable' if failed else 'healthy' if active else 'not_monitored')
        return dict(schema='workbench.control_protection.v1',attempt_id=self.attempt,timeout_ms=100,
            heartbeat=dict(status=heartbeat_status,age_s=age if heartbeat_status in ('healthy','lost') else None,
                           monitored=not worker_exited and (active or failed)),
            active_anomaly=not worker_exited and (failed or active and (age is None or age>.1)),
            recovery_required=bool(failed),reason=reason,
            event=(dict(id=self.attempt,wall_ns=record.get('wall_ns'),reason=reason,
                        stop_action='sdk_worker_terminated' if worker_exited else 'sdk_output_stop_requested',stop_confirmed=worker_exited,
                        sdk_close_confirmed=False,mechanical_state='not_verified',
                        evidence=str(self.guard_record),diagnostics=record.get('diagnostics')) if failed else None),
            worker_exit_confirmed=worker_exited,supervisor_exit_confirmed=exited,
            supervisor_pid=self.process.pid if self.process else None,worker_pid=record.get('child_pid'),
            supervisor_exit_code=self.process.returncode if self.process else None)

    def connect(self):return self.call('connect')
    def close(self):return self.call('close')
    def read_calibration_state(self):return self.call('read_calibration_state')
    def get_bimanual_joint_position_limits(self):return self.call('get_bimanual_joint_position_limits')
    def read_gains(self):return self.call('read_gains')
    def begin(self,*args):return self.call('begin',*args)
    def send(self,*args):return self.call('send',*args)
    def set_damping_hold(self,**kwargs):return self.call('set_damping_hold',**kwargs)
    def enable_collection_arm(self,*args):return self.call('enable_collection_arm',*args)
    def queue_reference(self,*args):return self.call('queue_reference',*args)
