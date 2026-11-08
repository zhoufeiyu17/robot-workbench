"""Independent CAN receive watchdog for the process that owns the native SDK.

No trajectory, gains, enable, disable or zero-setting commands are sent here.
The only transmitted CAN packet is the vendor's DM read-feedback query.
Loss is latched: restored feedback never restarts the SDK process.
"""
import ctypes
from collections import deque
from copy import deepcopy
import faulthandler
import json
import mmap
import os
from pathlib import Path
import resource
import select
import secrets
import signal
import socket
import stat
import struct
import subprocess
import sys
import threading
import time

from .release_trace import ReleaseTrace, memory_file, archive_memory_file

from .execution_settings import NATIVE_WATCHDOG_TIMEOUT_S

TIMEOUT = NATIVE_WATCHDOG_TIMEOUT_S  # Measured from actual CAN RX.
STARTUP_TIMEOUT = 10.
RELEASE_TIMEOUT = 10.
MOTORS = (1, 2, 4, 5, 6, 7, 8)
DM = (5, 6, 7, 8)
_client = None


def feedback_motor(can_id, data, flags=0):
    # SocketCAN marks locally generated echo with MSG_DONTROUTE/MSG_CONFIRM.
    if flags & (socket.MSG_DONTROUTE | getattr(socket, 'MSG_CONFIRM', 0x800)):
        return None
    if can_id & 0xE0000000 or len(data) != 8:
        return None
    if can_id == 0 and data[0] & 15 in DM:
        return data[0] & 15, data[0] >> 4
    if can_id in (1, 2, 4) and data[0] >> 5 == 1:
        return can_id, None
    return None


class FeedbackLatch:
    def __init__(self, buses, clock=time.monotonic, *, disabled_grippers=(), sdk_contract=None):
        self.buses = tuple(buses);self.clock = clock
        self.expected = {(b,m) for index,b in enumerate(self.buses) for m in MOTORS
                         if not (m==8 and ('left','right')[index] in disabled_grippers)}
        self.phase = 'idle';self.seen = {};self.enabled = set()
        self.output_started = {};self.statuses = {}
        self.retired = set()
        self.reason = None;self.started = self.heartbeat = clock()
        self.trip_time = self.trip_phase = None
        self.stack_requests = [];self.stack_heartbeat = None
        self.release_events=deque(maxlen=16);self.release_events_truncated=0
        self.sdk_contract=sdk_contract
        self.sdk_instances={};self.quiesce_reports={};self.generation=None
        self.rx_evidence={bus:dict(frames=0,valid_feedback_frames=0,stale_frames=0,future_frames=0,
            socket_drops=0,last_rx_age_s=None,maximum_rx_age_s=0.,last_kernel_rx_ns=None,last_read_monotonic_s=None)
            for bus in self.buses}

    def begin(self):
        if self.phase != 'idle' or self.reason:
            raise ValueError('output guard cannot automatically re-arm')
        self.phase = 'probe';self.seen.clear();self.enabled.clear()
        self.output_started.clear();self.statuses.clear()
        self.retired.clear()
        self.sdk_instances.clear();self.quiesce_reports.clear()
        self.started = self.heartbeat = self.clock()
        self.generation=secrets.token_hex(16)

    def register_sdk(self, side, identity):
        if (not self.sdk_contract or self.phase!='starting' or side not in ('left','right')
                or side in self.sdk_instances or not isinstance(identity,dict)
                or set(identity)!={'library_build_id','controller_instance_id'}
                or identity['library_build_id']!=self.sdk_contract['library_build_id']
                or not isinstance(identity['controller_instance_id'],str)
                or not 1<=len(identity['controller_instance_id'])<=128
                or identity['controller_instance_id'] in self.sdk_instances.values()):
            raise ValueError('invalid SDK instance registration')
        self.sdk_instances[side]=identity['controller_instance_id']

    def retire_sdk(self, side, report):
        bus=self.buses[('left','right').index(side)]
        if not self.sdk_contract:
            if report is not None:raise ValueError('quiesce report without pinned SDK contract')
        else:
            fields={'library_build_id','controller_instance_id','caller_generation','send_thread_joined',
                    'last_send_attempt_monotonic_ns','quiesce_returned_monotonic_ns','receiver_state','transport_basis','verification_scope','idempotent_hit'}
            if (not isinstance(report,dict) or set(report)!=fields
                    or report['library_build_id']!=self.sdk_contract['library_build_id']
                    or side not in self.sdk_instances
                    or report['controller_instance_id']!=self.sdk_instances[side]
                    or report['caller_generation']!=self.generation
                    or report['send_thread_joined'] is not True
                    or type(report['idempotent_hit']) is not bool
                    or report['receiver_state']!='vendor_owned_unverified'
                    or report['transport_basis']!='trusted_vendor_binary'
                    or report['verification_scope']!='controller_application'
                    or type(report['last_send_attempt_monotonic_ns']) is not int
                    or type(report['quiesce_returned_monotonic_ns']) is not int
                    or not int(self.started*1e9)<=report['last_send_attempt_monotonic_ns']<=report['quiesce_returned_monotonic_ns']<=int(self.clock()*1e9)
                    or self.clock()-report['last_send_attempt_monotonic_ns']/1e9>TIMEOUT):
                raise ValueError('invalid or stale SDK quiesce report')
        if self.phase!='releasing' or bus in self.retired or self.check():
            raise ValueError('SDK retirement outside healthy release')
        if self.sdk_contract:self.quiesce_reports[side]=dict(report)
        self.retired.add(bus);self.note_release('retire_'+side+'_received')
        return bus

    def receive(self, bus, can_id, data, received_at, flags=0):
        if bus in self.retired:return
        # TX echo is never feedback. During native construction it only marks
        # when a motor's actual output stream starts, after clear/enable setup.
        # This avoids interpreting the SDK's intentional disabled-clear delay
        # as the loss of an already running output stream.
        if self.phase=='starting' and flags & (socket.MSG_DONTROUTE | getattr(socket,'MSG_CONFIRM',0x800)):
            key=(bus,can_id)
            if (key in self.expected and len(data)==8 and not data.startswith(b'\xff'*7)
                    and (can_id in DM or can_id in (1,2,4) and data[0]>>5==0)):
                if received_at>self.clock():return
                self.output_started.setdefault(key,received_at)
            return
        update = self.validated_feedback(bus, can_id, data, received_at, flags)
        if update is not None:self.apply_feedback(*update)

    def validated_feedback(self, bus, can_id, data, received_at, flags=0):
        """Validate at reception time, before optional normal-state coalescing."""
        if bus in self.retired:return None
        decoded = feedback_motor(can_id, data, flags)
        if self.phase == 'idle' or self.reason or decoded is None or bus not in self.buses:
            return
        now = self.clock();motor, status = decoded
        if not 0 <= now-received_at <= TIMEOUT:
            return
        key = (bus, motor)
        if key not in self.expected:return
        # Native initialization clears a disabled DM motor, then deliberately
        # waits 100 ms before enabling it. That clear reply is not evidence that
        # an output stream has started. Once enabled, the 100 ms latch applies.
        if self.phase=='starting' and key not in self.output_started:return
        self.rx_evidence[bus]['valid_feedback_frames']+=1
        return bus,motor,status,received_at

    def apply_feedback(self, bus, motor, status, received_at):
        key=(bus,motor)
        if status is not None:
            self.statuses[key]=status
            if status not in (0,1):self.trip(f'{bus} motor {motor} fault status {status}')
            elif status == 1:self.enabled.add(key)
            elif self.phase == 'active' or self.phase in ('starting','releasing') and key in self.enabled:
                self.trip(f'{bus} motor {motor} disabled/fault status {status}')
        self.seen[key] = max(received_at, self.seen.get(key, 0))

    def missing(self, motors):
        now = self.clock()
        return [f'{bus}:{motor}' for bus in self.buses for motor in motors
                if (bus,motor) in self.expected
                if now-self.seen.get((bus, motor), float('-inf')) > TIMEOUT]

    def activate(self):
        if self.phase != 'starting' or self.reason:
            raise ValueError('output guard was not armed before SDK construction')
        missing = self.missing(MOTORS)
        if missing:self.trip('missing actual motor feedback: '+', '.join(missing))
        if any(self.statuses.get((b,m))!=1 for b,m in self.expected if m in DM):
            self.trip('DM motor enable was not confirmed by actual feedback')
        if self.reason:raise ValueError(self.reason)
        self.phase = 'active';self.heartbeat = self.clock()

    def trip(self, reason):
        if self.reason is None:
            self.reason = str(reason)
            self.trip_time,self.trip_phase = self.clock(),self.phase
            if self.phase=='releasing':self.note_release('latched')
        self.phase = 'latched'

    def note_release(self, action):
        """Existing IPC observations only; no IO, commands or deadline changes."""
        now=self.clock()
        if len(self.release_events)==self.release_events.maxlen:self.release_events_truncated+=1
        self.release_events.append(dict(action=action,monotonic_s=now,phase=self.phase,
            retired_buses=sorted(self.retired),reason=self.reason,
            motor_rx_age_s={f'{bus}:{motor}':None if (bus,motor) not in self.seen else max(0.,now-self.seen[bus,motor])
                for bus,motor in sorted(self.expected)}))

    def diagnostic_snapshot(self):
        # Called for persistence only after termination or while outputs are idle.
        now = self.trip_time if self.trip_time is not None else self.clock()
        return dict(sample_monotonic_s=now,phase=self.phase,phase_before_latch=self.trip_phase,
            receive_evidence=deepcopy(self.rx_evidence),
            last_heartbeat_monotonic_s=self.heartbeat,last_heartbeat_age_s=max(0.,now-self.heartbeat),
            motor_rx_age_s={f'{bus}:{motor}':None if (bus,motor) not in self.seen else max(0.,now-self.seen[bus,motor])
                            for bus,motor in sorted(self.expected)},retired_buses=sorted(self.retired),
            release_events=deepcopy(list(self.release_events)),release_events_truncated=self.release_events_truncated,
            last_confirmed_release_step=next((row['action'] for row in reversed(self.release_events) if row['action']!='latched'),None))

    def stack_request(self):
        # Never renew heartbeat or wait for the child. A new heartbeat ends a
        # stall; a single child has at most three diagnostic signal attempts.
        now = self.clock();age = now-self.heartbeat
        if (self.phase!='active' or self.reason or not .05<age<=TIMEOUT or self.missing(MOTORS)
                or self.stack_heartbeat==self.heartbeat or len(self.stack_requests)>=3):
            return None
        self.stack_heartbeat = self.heartbeat
        request=dict(requested_monotonic_s=now,wall_ns=time.time_ns(),last_heartbeat_age_s=age,
                     heartbeat_monotonic_s=self.heartbeat,signal='SIGUSR1')
        self.stack_requests.append(request)
        return request

    def check(self):
        now = self.clock()
        if self.phase in ('starting', 'active', 'releasing'):
            # Startup checks every motor already seen; uninitialized motors have
            # only the bounded constructor allowance. DM probes cover both arms.
            keys = self.output_started if self.phase == 'starting' else self.expected
            keys = [key for key in keys if key[0] not in self.retired]
            missing = [f'{b}:{m}' for b,m in keys if now-self.seen.get((b,m),self.output_started.get((b,m),float('-inf'))) > TIMEOUT]
            if missing:self.trip('CAN feedback lost: '+', '.join(missing))
            elif self.phase == 'active' and now-self.heartbeat > TIMEOUT:
                self.trip('SDK owner heartbeat lost')
            elif self.phase == 'starting' and now-self.started > STARTUP_TIMEOUT:
                self.trip('SDK initialization deadline exceeded')
            elif self.phase == 'releasing' and now-self.started > RELEASE_TIMEOUT:
                self.trip('SDK release deadline exceeded')
        return self.reason


def parent_death(parent_pid):
    if sys.platform != 'linux':raise RuntimeError('physical SDK protection requires Linux')
    libc = ctypes.CDLL(None, use_errno=True)
    if libc.prctl(1, signal.SIGKILL, 0, 0, 0) != 0:
        raise OSError(ctypes.get_errno(), 'PR_SET_PDEATHSIG failed')
    if os.getppid() != parent_pid:os._exit(125)


class CANBatchTiming:
    """Wall spans for all batches; CPU detail at pending IPC; no hot-path IO."""
    def __init__(self,detailed=True):
        self.stages={}
        self.detailed=detailed
        self.usage=self.read_usage()if detailed else None
        self.started=self.last=time.monotonic_ns()
        self.cpu=self.last_cpu=time.thread_time_ns()if detailed else None

    @staticmethod
    def read_usage():
        kind=getattr(resource,'RUSAGE_THREAD',None)
        if kind is None:return None
        try:return resource.getrusage(kind)
        except (OSError,ValueError):return None

    def mark(self,phase):
        now=time.monotonic_ns();cpu=time.thread_time_ns()if self.detailed else None
        row=self.stages.setdefault(phase,[0,0,0])
        row[0]+=now-self.last
        if cpu is not None:row[1]+=cpu-self.last_cpu
        row[2]+=1
        self.last,self.last_cpu=now,cpu

    def finish(self):
        usage=self.read_usage()if self.detailed else None
        fields=('ru_utime','ru_stime','ru_nvcsw','ru_nivcsw','ru_minflt','ru_majflt')
        return dict(schema='guard_can_batch_timing.v1',wall_s=(self.last-self.started)/1e9,
            cpu_s=(self.last_cpu-self.cpu)/1e9 if self.detailed else None,
            stages={k:dict(wall_s=v[0]/1e9,cpu_s=v[1]/1e9 if self.detailed else None,segments=v[2])for k,v in self.stages.items()},
            thread_usage_delta=None if usage is None or self.usage is None else
                {k:getattr(usage,k)-getattr(self.usage,k)for k in fields},
            scope='Instrumented batch includes probe overhead; thread usage separates user/kernel CPU, switches and page faults')


def receive_ready_can(ready,readers,latch,timing=None):
    """Bound connected CAN work before servicing IPC; retain queued RX stamps."""
    frames=0;pending={}
    coalesce=latch.phase in ('active','releasing')
    def flush():
        for update in pending.values():latch.apply_feedback(*update)
        pending.clear()
        if timing is not None:timing.mark('apply_coalesced')
    for sock in ready:
        if sock not in readers:continue
        started=time.monotonic()
        for index in range(128):
            # Probe/activation keep their original acquisition ordering. Once
            # connected, neither busy bus may monopolize a heartbeat roundtrip.
            if index and latch.phase in ('active','releasing')and time.monotonic()-started>=.0005:break
            if timing is not None:timing.mark('loop')
            try:raw,ancillary,flags,_addr=sock.recvmsg(16,128)
            except BlockingIOError:
                if timing is not None:timing.mark('recvmsg')
                break
            if timing is not None:timing.mark('recvmsg')
            frames+=1
            if len(raw)!=16 or flags & (socket.MSG_TRUNC | socket.MSG_CTRUNC):continue
            ident,size,data=struct.unpack('=IB3x8s',raw)
            if size!=8:continue
            stamp=next((v for level,kind,v in ancillary if level==socket.SOL_SOCKET and kind==35),None)
            if stamp is None or len(stamp)<struct.calcsize('@ll'):continue
            seconds,nanos=struct.unpack('@ll',stamp[:struct.calcsize('@ll')])
            if timing is not None:timing.mark('header_timestamp')
            kernel_ns=seconds*1_000_000_000+nanos
            age=(time.time_ns()-kernel_ns)/1e9
            read_at=time.monotonic();received=read_at-age
            row=latch.rx_evidence[readers[sock]];row['frames']+=1
            row['last_rx_age_s']=age;row['maximum_rx_age_s']=max(row['maximum_rx_age_s'],age)
            row['last_kernel_rx_ns']=kernel_ns;row['last_read_monotonic_s']=read_at
            if age>TIMEOUT:row['stale_frames']+=1
            elif age<0:row['future_frames']+=1
            for level,kind,value in ancillary:
                if level==socket.SOL_SOCKET and kind==40 and len(value)>=4:
                    row['socket_drops']=struct.unpack('=I',value[:4])[0]  # Linux SO_RXQ_OVFL.
            if timing is not None:timing.mark('clock_mapping')
            if received>=latch.started:
                if not coalesce:latch.receive(readers[sock],ident,data[:size],received,flags)
                else:
                    update=latch.validated_feedback(readers[sock],ident,data[:size],received,flags)
                    if timing is not None:timing.mark('validate_feedback')
                    if update is not None:
                        bus,motor,status,stamp=update
                        if status not in (None,1):
                            # Preserve the complete normal prefix before an
                            # abnormal/disabled transition; never merge it away.
                            flush();latch.apply_feedback(*update)
                        elif (bus,motor)not in pending or stamp>pending[bus,motor][3]:
                            pending[bus,motor]=update
            if timing is not None:timing.mark('merge_or_transition')
            if latch.reason:return frames
    flush()
    return frames


class GuardClient:
    def __init__(self, connection, failure_trace=None):
        self.connection = connection;self.lock = threading.Lock();self.pending = b''
        self.release_trace=None;self.failure_trace=failure_trace

    def call(self, action, **payload):
        requested=time.monotonic_ns();requested_cpu=time.thread_time_ns()
        with self.lock:
            started=time.monotonic_ns();stage='timeout_setup'
            budget=.5 if action in ('hello','begin') else TIMEOUT if action=='heartbeat' else .05
            try:
                self.connection.settimeout(budget);stage='send'
                self.connection.sendall(json.dumps({'action': action,**payload}).encode()+b'\n')
                sent=time.monotonic_ns();stage='receive'
                while b'\n' not in self.pending:
                    part = self.connection.recv(4096)
                    if not part:raise ConnectionError('CAN output supervisor disconnected')
                    self.pending += part
                    if len(self.pending) > 8192:raise ValueError('oversized guard response')
                line,self.pending = self.pending.split(b'\n',1);stage='decode';reply = json.loads(line)
            except (OSError, ValueError) as error:
                if self.failure_trace is not None:
                    try:self.failure_trace.write(dict(schema='guard_ipc_failure.v1',source_pid=os.getpid(),source_tid=threading.get_native_id(),
                        action=action,stage=stage,started_monotonic_ns=started,sent_monotonic_ns=locals().get('sent'),
                        failed_monotonic_ns=time.monotonic_ns(),budget_s=budget,exception=type(error).__name__,errno=getattr(error,'errno',None),
                        message=str(error)[:160],pending_bytes=len(self.pending),self_sigkill_requested=True))
                    except Exception:pass  # Diagnostic failure never delays the existing fail-stop.
                # The supervisor also owns and terminates us on IPC failure. Do
                # not call SDK close(), whose destructor can resume sending.
                os.kill(os.getpid(), signal.SIGKILL)
                raise
            if reply.get('error'):raise ValueError(reply['error'])
            returned=time.monotonic_ns()
            self.last_call_timing=dict(action=action,lock_wait_s=(started-requested)/1e9,
                encode_send_s=(sent-started)/1e9,response_wait_decode_s=(returned-sent)/1e9,
                total_s=(returned-requested)/1e9,sent_monotonic_ns=sent,returned_monotonic_ns=returned,
                cpu_s=(time.thread_time_ns()-requested_cpu)/1e9,
                server=reply.get('server_timing'))
            return reply


class IPCFailureTrace:
    """One bounded failure-only shared-memory slot, prefaulted before SDK startup."""
    SIZE=4096
    def __init__(self,fd,file=None):
        info=os.fstat(fd)
        if not stat.S_ISREG(info.st_mode)or info.st_uid!=os.getuid()or stat.S_IMODE(info.st_mode)!=0o600 or info.st_size!=self.SIZE:
            raise ValueError('owned private IPC failure memory required')
        self.fd,self.file=fd,file;self.memory=mmap.mmap(fd,self.SIZE)
    @classmethod
    def create(cls):
        file=memory_file('workbench-ipc-failure',cls.SIZE);result=cls(file.fileno(),file)
        result.memory[:]=b'\0'*cls.SIZE
        return result
    def write(self,value):
        raw=json.dumps(value,separators=(',',':'),allow_nan=False).encode()
        if len(raw)>self.SIZE-8:raise ValueError('bounded failure receipt required')
        self.memory[:8]=struct.pack('=II',1,len(raw));self.memory[8:8+len(raw)]=raw
        self.memory[:8]=struct.pack('=II',2,len(raw))
    def snapshot(self):
        header=self.memory[:8];sequence,size=struct.unpack('=II',header)
        if sequence==0:return None
        if sequence!=2 or not 0<size<=self.SIZE-8:return dict(status='partial')
        raw=self.memory[8:8+size]
        if self.memory[:8]!=header:return dict(status='partial')
        try:return json.loads(raw)
        except (ValueError,UnicodeDecodeError):return dict(status='partial')
    def close(self):
        self.memory.close()
        if self.file is not None:self.file.close()
        else:os.close(self.fd)


def bind_child():
    global _client
    parent = int(os.environ['ROBOT_WORKBENCH_GUARD_PARENT'])
    parent_death(parent)
    connection = socket.socket(fileno=int(os.environ.pop('ROBOT_WORKBENCH_GUARD_FD')))
    connection.settimeout(2.)
    peer = struct.unpack('3i',connection.getsockopt(socket.SOL_SOCKET,socket.SO_PEERCRED,12))
    if peer[0] != parent or peer[1] != os.getuid():raise ValueError('guard process identity differs')
    trace_fd = int(os.environ.pop('ROBOT_WORKBENCH_GUARD_TRACE_FD'))
    info = os.fstat(trace_fd)
    if not stat.S_ISREG(info.st_mode) or info.st_uid!=os.getuid() or stat.S_IMODE(info.st_mode)!=0o600:
        raise ValueError('guard traceback descriptor must be an owned private regular file')
    os.set_inheritable(trace_fd,False)
    faulthandler.enable(file=trace_fd,all_threads=True)
    faulthandler.register(signal.SIGUSR1,file=trace_fd,all_threads=True,chain=False)
    failure_fd=os.environ.pop('ROBOT_WORKBENCH_GUARD_FAILURE_FD',None)
    failure_trace=IPCFailureTrace(int(failure_fd)) if failure_fd is not None else None
    if failure_fd is not None:os.set_inheritable(int(failure_fd),False)
    _client = GuardClient(connection,failure_trace)
    _client.trace_fd = trace_fd  # Keep open for the entire owned SDK process.
    diagnostic_fd=os.environ.pop('ROBOT_WORKBENCH_GUARD_RELEASE_FD',None)
    if diagnostic_fd is not None:
        try:_client.release_trace=ReleaseTrace.inherit(int(diagnostic_fd))
        except (OSError,ValueError,BufferError,MemoryError):pass  # Diagnostics cannot change control admission.
    hello=_client.call('hello')
    _client.disabled_grippers = hello['disabled_grippers']
    _client.sdk_contract=hello.get('sdk_contract')


def require_guard():
    if _client is None:
        raise RuntimeError('真机 SDK 必须由独立 CAN 失联保护进程启动；请使用 serve_physical_device.py')
    return _client


def terminate_sdk(child):
    # A child PID cannot be reused until reaped. Never signal a searched PID.
    evidence=dict(polled_monotonic_ns=time.monotonic_ns(),returncode_before_signals=child.poll(),signals=[])
    if evidence['returncode_before_signals'] is None:
        for sig in (signal.SIGSTOP,signal.SIGKILL):
            row=dict(signal=sig.name,attempted_monotonic_ns=time.monotonic_ns())
            try:os.kill(child.pid,sig);row['sent']=True
            except ProcessLookupError:row.update(sent=False,result='process_not_present_at_signal')
            evidence['signals'].append(row)
    return evidence


def open_can(bus):
    sock=socket.socket(socket.PF_CAN,socket.SOCK_RAW,socket.CAN_RAW)
    try:
        sock.setsockopt(socket.SOL_SOCKET,35,1)  # Linux SO_TIMESTAMPNS.
        sock.setsockopt(socket.SOL_SOCKET,40,1)  # Linux SO_RXQ_OVFL; diagnostic only.
        sock.bind((bus,));sock.setblocking(False)
        return sock
    except BaseException:
        sock.close();raise


def supervise(profile, command, *, record_path=None, inherited_fds=()):
    if sys.platform != 'linux':raise RuntimeError('physical SDK protection requires Linux')
    from .gc_timing import install,recent
    install()
    buses = tuple(profile['can_interfaces'][side] for side in ('left','right'))
    disabled=profile.get('disabled_grippers',[])
    if disabled not in ([],['left'],['right'],['left','right']):
        raise ValueError('disabled_grippers must list left and/or right explicitly')
    sdk_contract=profile.get('sdk_quiesce')
    if sdk_contract is not None:
        from .sdk_quiesce import validate_contract
        validate_contract(sdk_contract)
    latch = FeedbackLatch(buses,disabled_grippers=disabled,sdk_contract=sdk_contract);readers = {};parent = peer = child = None
    trace_fd = trace_path = trace_memory = release_trace = failure_trace = None
    stack_capture=dict(storage='bounded_memory',limit_bytes=4*1024**2,archived=False)
    path = Path(record_path or Path(profile['storage_root'])/'can-output-guard.json')
    path.parent.mkdir(parents=True,exist_ok=True)
    previous = json.loads(path.read_text()) if path.exists() else {}
    process_evidence=dict(schema='guard_process_exit.v1',last_ipc=None,terminations=[],
                          first_failure=None,returncode=None,signal_sender='not_inferred')
    def observe_failure(reason):
        if process_evidence['first_failure'] is None:
            process_evidence['first_failure']=dict(reason=reason,monotonic_ns=time.monotonic_ns(),
                                                   child_returncode=child.poll() if child else None)
    def terminate():
        evidence=terminate_sdk(child)
        if evidence is not None:process_evidence['terminations'].append(evidence)
    def persist(state, *, include_release_trace=False, **extra):
        value = dict(schema='arx_can_output_guard.v1',state=state,supervisor_pid=os.getpid(),
            child_pid=child.pid if child else None,wall_ns=time.time_ns(),timeout_s=TIMEOUT,
            last_latch=previous.get('last_latch'),diagnostics=latch.diagnostic_snapshot(),
            stack_log=str(trace_path) if trace_path is not None else None,stack_requests=list(latch.stack_requests),**extra)
        value['stack_capture']=dict(stack_capture)
        value['process_evidence']=deepcopy(process_evidence)
        value['guard_client_failure']=failure_trace.snapshot() if failure_trace is not None else None
        value['sdk_quiesce_reports']=dict(latch.quiesce_reports)
        if latch.reason:value['last_latch']={'reason':latch.reason,'wall_ns':time.time_ns(),**value['diagnostics']}
        if include_release_trace:
            started=next((row['monotonic_s'] for row in reversed(latch.release_events)
                          if row['action']=='releasing_received'),None)
            value['release_trace']=(release_trace.snapshot(child.pid if child else None,
                not_before_ns=int(started*1e9) if started is not None else None) if release_trace is not None
                else dict(schema='workbench.release_trace.v1',supported=False,status='unsupported'))
            observed=value['release_trace']
            if observed.get('status') in ('observed','partial'):
                received={row['action'] for row in latch.release_events}
                expected={'guard_releasing':'releasing_received','retire_left':'retire_left_received',
                          'retire_right':'retire_right_received','guard_disarm':'disarm_received'}
                missing=sorted({expected[row['step']] for row in observed['events']
                    if row['status']=='returned' and row['step'] in expected and expected[row['step']] not in received})
                observed['ipc_order_consistency']=('unknown_truncated' if latch.release_events_truncated else
                    'inconsistent' if missing else 'consistent')
                observed['missing_guard_receipts']=missing
            if latch.reason:value['last_latch']['release_trace']=deepcopy(value['release_trace'])
        temporary=path.with_suffix('.tmp');temporary.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n');temporary.replace(path)
    def reply(value):
        process_evidence['last_reply']=dict(action=(process_evidence.get('last_ipc')or{}).get('action'),started_monotonic_ns=time.monotonic_ns())
        parent.sendall(json.dumps(value).encode()+b'\n')
        process_evidence['last_reply']['returned_monotonic_ns']=time.monotonic_ns()
    pending = b'';probe_request = False;next_probe = 0.;stop_signal = None
    def stop(signum, _):
        nonlocal stop_signal
        stop_signal = signum
    old_signals = {sig:signal.signal(sig,stop) for sig in (signal.SIGTERM,signal.SIGINT)}
    try:
        for bus in buses:
            readers[open_can(bus)]=bus
        parent,peer=socket.socketpair();parent.settimeout(.005)
        trace_path=path.parent/f'can-output-guard-stack-{os.getpid()}-{time.time_ns()}.log'
        trace_memory=memory_file('workbench-sdk-stack',stack_capture['limit_bytes'])
        trace_fd=trace_memory.fileno()
        env=dict(os.environ,ROBOT_WORKBENCH_GUARD_FD=str(peer.fileno()),ROBOT_WORKBENCH_GUARD_PARENT=str(os.getpid()),
                 ROBOT_WORKBENCH_GUARD_TRACE_FD=str(trace_fd))
        env.pop('ROBOT_WORKBENCH_GUARD_RELEASE_FD',None)
        env.pop('ROBOT_WORKBENCH_GUARD_FAILURE_FD',None)
        try:failure_trace=IPCFailureTrace.create()
        except (OSError,ValueError,BufferError,MemoryError):pass
        try:release_trace=ReleaseTrace.create()
        except (OSError,ValueError,BufferError,MemoryError):pass
        descriptors=(peer.fileno(),trace_fd,*inherited_fds)
        if failure_trace is not None:
            descriptors+=(failure_trace.fd,);env['ROBOT_WORKBENCH_GUARD_FAILURE_FD']=str(failure_trace.fd)
        if release_trace is not None:
            descriptors+=(release_trace.fd,);env['ROBOT_WORKBENCH_GUARD_RELEASE_FD']=str(release_trace.fd)
        parent_pid=os.getpid()
        child=subprocess.Popen(command,env=env,pass_fds=descriptors,preexec_fn=lambda:parent_death(parent_pid))
        peer.close();peer=None;persist('disconnected')
        cycle_done_ns=time.monotonic_ns();previous_batch=None;reported_slow_ns=None
        while child.poll() is None:
            if stop_signal:
                # Preserve existing supported-release semantics: ordinary service
                # stop cannot silently drop a powered robot's hold.
                child.send_signal(stop_signal);stop_signal=None
            if latch.check():
                observe_failure(latch.reason)
                terminate()  # Terminate before any log or filesystem IO.
                try:persist('latched',include_release_trace=True,reason=latch.reason)
                except OSError:pass
                break
            if latch.stack_request() is not None:
                try:os.kill(child.pid,signal.SIGUSR1)
                except ProcessLookupError:pass
            now=time.monotonic()
            # The legacy SDK decoder cannot accept another process's 0x7FF
            # query echo. Query only before any SDK constructor; thereafter RX only.
            if latch.phase == 'probe' and now>=next_probe:
                for sock,bus in readers.items():
                    for motor in DM:
                        if (bus,motor) not in latch.expected:continue
                        sock.send(struct.pack('=IB3x8s',0x7ff,4,bytes([motor,0,0xcc,0,0,0,0,0])))
                next_probe=now+.025
            select_started_ns=time.monotonic_ns();select_cpu=time.thread_time_ns()
            ready,_,_=select.select([parent,*readers],[],[],.005)
            selected_ns=time.monotonic_ns();selected_cpu=time.thread_time_ns()
            loop_timing=dict(select_started_monotonic_ns=select_started_ns,
                before_select_s=(select_started_ns-cycle_done_ns)/1e9,
                select_wall_s=(selected_ns-select_started_ns)/1e9,
                select_cpu_s=(selected_cpu-select_cpu)/1e9,previous_batch=previous_batch)
            # Always process CAN before activate/heartbeat, preserving acquisition
            # order and refusing queued pre-arm packets by their kernel RX stamp.
            can_probe=CANBatchTiming(detailed=parent in ready)
            can_frames=receive_ready_can(ready,readers,latch,can_probe)
            can_done_ns=time.monotonic_ns()
            can_cpu_ns=time.thread_time_ns()-selected_cpu
            can_detail=can_probe.finish()if parent in ready or can_done_ns-selected_ns>=5_000_000 else None
            if can_done_ns-selected_ns>=5_000_000:
                process_evidence['last_slow_can_batch']=dict(selected_monotonic_ns=selected_ns,
                    can_batch_s=(can_done_ns-selected_ns)/1e9,can_batch_cpu_s=can_cpu_ns/1e9,
                    can_frames=can_frames,detail=can_detail,loop=loop_timing,gc=recent())
            if latch.reason:continue
            if parent in ready:
                data=parent.recv(4096)
                if not data:
                    if latch.phase!='idle':
                        observe_failure('guard IPC closed with SDK ownership active')
                        latch.trip('guard IPC closed with SDK ownership active')
                    else:break
                    continue
                pending+=data
                if len(pending)>8192:raise ValueError('oversized guard request')
                while b'\n' in pending:
                    line,pending=pending.split(b'\n',1);request=json.loads(line)
                    action=request['action']
                    process_evidence['last_ipc']=dict(action=action,received_monotonic_ns=time.monotonic_ns())
                    fields=({'action','side','identity'} if action=='register_sdk' else
                            {'action','report'} if action in ('retire_left','retire_right') and sdk_contract else
                            {'action','detail'} if action=='trip' and 'detail' in request else {'action'})
                    if set(request)!=fields:raise ValueError('invalid guard request')
                    if action=='hello':reply({'ready':True,'disabled_grippers':disabled,'sdk_contract':sdk_contract})
                    elif action=='begin':
                        from .can_feedback_filter import install
                        for sock in readers:install(sock,active=False)
                        latch.begin();probe_request=True;next_probe=0.
                    elif action=='activate':
                        if sdk_contract and set(latch.sdk_instances)!={'left','right'}:
                            raise ValueError('SDK instances not registered')
                        try:latch.activate()
                        except ValueError:continue
                        from .can_feedback_filter import install
                        for sock,bus in readers.items():
                            install(sock,active=True,include_gripper=(bus,8)in latch.expected)
                        reply({'armed':True,'heartbeat_monotonic_s':latch.heartbeat})  # No filesystem IO while outputs are armed.
                    elif action=='heartbeat' and latch.phase in ('starting','active','releasing'):
                        slow=process_evidence.get('last_slow_can_batch')
                        new_slow=slow if slow and slow['selected_monotonic_ns']!=reported_slow_ns else None
                        latch.heartbeat=time.monotonic();reply({'healthy':True,'heartbeat_monotonic_s':latch.heartbeat,
                            'server_timing':dict(selected_monotonic_ns=selected_ns,can_batch_s=(can_done_ns-selected_ns)/1e9,
                                can_batch_cpu_s=can_cpu_ns/1e9,can_frames=can_frames,can_detail=can_detail,
                                loop=loop_timing,new_slow_can_batch=new_slow,gc=recent(),
                                request_decoded_monotonic_ns=process_evidence['last_ipc']['received_monotonic_ns'])})
                        if new_slow:reported_slow_ns=new_slow['selected_monotonic_ns']
                    elif action=='releasing' and latch.phase in ('starting','active'):
                        latch.phase='releasing';latch.started=time.monotonic();latch.note_release('releasing_received');reply({'protected':True})
                    elif action=='register_sdk':
                        try:latch.register_sdk(request['side'],request['identity'])
                        except ValueError as error:
                            latch.trip(str(error));break
                        reply({'registered':True})
                    elif action in ('retire_left','retire_right') and latch.phase=='releasing':
                        try:bus=latch.retire_sdk(action.removeprefix('retire_'),request.get('report'))
                        except ValueError as error:
                            latch.trip(str(error));break
                        reply({'retired':bus})
                    elif action=='disarm' and latch.phase=='releasing' and latch.retired==set(buses):
                        latch.note_release('disarm_received');latch.phase='idle';latch.seen.clear();latch.enabled.clear();reply({'disconnected':True});persist('disconnected',include_release_trace=True)
                    elif action=='trip':
                        detail=request.get('detail','')
                        if not isinstance(detail,str) or len(detail)>512:raise ValueError('invalid SDK error detail')
                        latch.trip('SDK acquisition or native call failed'+(': '+detail if detail else ''))
                    else:raise ValueError('invalid guard transition: '+str(action))
            if probe_request:
                if not latch.missing(DM):
                    latch.phase='starting';latch.started=time.monotonic();probe_request=False
                    latch.seen.clear();latch.enabled.clear();latch.output_started.clear();latch.statuses.clear()
                    reply({'protected_before_sdk':True,'generation':latch.generation})
                elif time.monotonic()-latch.started>.3:
                    missing=latch.missing(DM);latch.phase='idle';probe_request=False
                    reply({'error':'电机只读反馈预检失败，未启动 SDK：'+', '.join(missing)})
                    persist('connection_rejected',missing=missing)
            cycle_done_ns=time.monotonic_ns()
            previous_batch=dict(selected_monotonic_ns=selected_ns,can_frames=can_frames,
                can_batch_s=(can_done_ns-selected_ns)/1e9,can_batch_cpu_s=can_cpu_ns/1e9,
                after_can_s=(cycle_done_ns-can_done_ns)/1e9)
        process_evidence['wait']=dict(status='started',started_monotonic_ns=time.monotonic_ns())
        try:result=child.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process_evidence['wait'].update(status='timeout',observed_monotonic_ns=time.monotonic_ns())
            raise
        process_evidence['wait'].update(status='returned',returned_monotonic_ns=time.monotonic_ns())
        process_evidence['returncode']=result
        process_evidence['reaped_monotonic_ns']=time.monotonic_ns()
        try:stack_capture.update(archive_memory_file(trace_memory,trace_path),archived=True)
        except OSError as error:stack_capture['archive_error']=str(error)
        if latch.phase!='idle' and not latch.reason:
            observe_failure('SDK process exited before protected release completed')
            latch.trip('SDK process exited before protected release completed')
        if latch.reason:
            try:persist('latched',include_release_trace=True,reason=latch.reason)
            except OSError:pass
        else:persist('disconnected',include_release_trace=True)
        return result
    except BaseException as error:
        observe_failure(type(error).__name__+': '+str(error))
        if child is not None:terminate()
        latch.trip('supervisor failed: '+str(error))
        try:persist('latched',include_release_trace=True,reason=latch.reason)
        except OSError:pass
        raise
    finally:
        if child is not None and child.poll() is None:terminate()
        for sock in [*readers,parent,peer]:
            if sock is not None:sock.close()
        if trace_memory is not None:trace_memory.close()
        if release_trace is not None:release_trace.close()
        if failure_trace is not None:failure_trace.close()
        for sig,handler in old_signals.items():signal.signal(sig,handler)
