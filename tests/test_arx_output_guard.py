"""CPU fault injection only: no SDK import, PF_CAN socket, or robot commands.

Linux integration substitutes Unix datagrams for both CAN buses, preserving
kernel receive timestamps and the real supervisor/child IPC and signal paths.
"""
import json
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from robot_workbench import arx_output_guard as guard


def frame(motor, status=1):
    # DM sample layout matches the retained read-only motor-8 query receipt;
    # EC frames are protocol-shaped CPU inputs, not physical acceptance proof.
    if motor in (1, 2, 4):
        return motor, bytes([0x20, 0x7f, 0xff, 0x7f, 0xff, 0, 0, 0])
    return 0, bytes([(status << 4) | motor, 0x68, 0x2c, 0x7f, 0xf7, 0xff, 39, 33])


def wait_until(predicate, timeout=3):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.01)
    raise AssertionError('test child did not reach the expected state')


class FeedbackTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.
        self.latch = guard.FeedbackLatch(('can3', 'can1'), lambda: self.now)
        self.latch.begin()
        self.latch.phase = 'starting'

    def feed(self, omitted=()):
        for bus, motor in self.latch.expected:
            if (bus, motor) not in omitted:
                ident, data = frame(motor)
                self.latch.receive(bus,motor,b'\x00'*8,self.now,socket.MSG_DONTROUTE)
                self.latch.receive(bus, ident, data, self.now)

    def activate(self):
        self.feed()
        self.latch.activate()

    def test_rejects_echo_stale_future_and_invalid_packets(self):
        ident, data = frame(8)
        for flags in (socket.MSG_DONTROUTE, getattr(socket, 'MSG_CONFIRM', 0x800)):
            self.latch.receive('can3', ident, data, self.now, flags)
        for stamp in (self.now-.101, self.now+.01):
            self.latch.receive('can3', ident, data, stamp)
        for ident, data in ((0, b'\x18'), (0x80000000, data),
                            (0x40000000, data), (0x20000000, data),
                            (3, frame(1)[1]), (1, b'\x00'*8), (0, b'\x13'+b'\x00'*7)):
            self.latch.receive('can3', ident, data, self.now)
        self.assertEqual(self.latch.seen, {})
        ident, data = frame(8)
        self.latch.receive('can3',8,b'\x00'*8,self.now,socket.MSG_DONTROUTE)
        self.latch.receive('can3', ident, data, self.now)
        self.assertEqual(self.latch.seen, {('can3', 8): self.now})

    def test_one_motor_loss_latches_even_after_feedback_returns(self):
        self.activate()
        self.now += .101
        self.feed(omitted={('can3', 2)})
        self.latch.heartbeat = self.now
        reason = self.latch.check()
        self.assertIn('can3:2', reason)
        self.now += .01
        self.feed()
        self.assertEqual(self.latch.check(), reason)
        self.assertEqual(self.latch.phase, 'latched')
        with self.assertRaisesRegex(ValueError, 're-arm'):
            self.latch.begin()

    def test_stale_transmit_echo_cannot_reset_output_deadline(self):
        self.latch.receive('can3',5,b'\x00'*8,self.now-.2,socket.MSG_DONTROUTE)
        self.assertEqual(self.latch.seen,{})
        self.assertIn('can3:5',self.latch.check())

    def test_retired_arm_cannot_mask_loss_on_remaining_arm(self):
        self.activate();self.latch.phase='releasing';self.latch.retired.add('can3')
        self.now += .101;self.feed(omitted={(b,m) for b,m in self.latch.expected if b=='can3'})
        self.assertIsNone(self.latch.check())
        self.now += .101
        self.assertIn('can1:',self.latch.check())

    def test_owner_heartbeat_is_independent_of_healthy_motor_traffic(self):
        self.activate()
        self.now += .101
        self.feed()
        self.assertIn('owner heartbeat lost', self.latch.check())

    def test_stack_requests_once_per_stall_and_at_most_three_per_child(self):
        self.activate()
        for index in range(5):
            self.now+=.051;self.feed()
            request=self.latch.stack_request()
            self.assertEqual(request is not None,index<3)
            if request:self.assertEqual(request['signal'],'SIGUSR1')
            self.assertIsNone(self.latch.stack_request(),'same stall cannot flood the traceback file')
            self.now+=.01;self.feed();self.latch.heartbeat=self.now
            self.assertIsNone(self.latch.stack_request())
        self.assertEqual(len(self.latch.stack_requests),3)
        self.assertIsNone(self.latch.check())

    def test_diagnostic_does_not_renew_or_extend_100ms_heartbeat_deadline(self):
        self.activate();heartbeat=self.latch.heartbeat
        self.now=heartbeat+.051;self.feed()
        self.assertIsNotNone(self.latch.stack_request())
        self.assertEqual(self.latch.heartbeat,heartbeat)
        self.now=heartbeat+.100001;self.feed()
        self.assertIn('owner heartbeat lost',self.latch.check())
        self.assertIsNone(self.latch.stack_request())
        snapshot=self.latch.diagnostic_snapshot()
        self.assertEqual(snapshot['phase'],'latched');self.assertEqual(snapshot['phase_before_latch'],'active')
        self.assertGreater(snapshot['last_heartbeat_age_s'],.1)
        self.assertEqual(set(snapshot['motor_rx_age_s'].values()),{0.})
        self.now+=5
        self.assertEqual(self.latch.diagnostic_snapshot(),snapshot,'post-kill IO cannot change trigger-time ages')

    def test_diagnostics_require_active_phase_healthy_can_and_delayed_heartbeat(self):
        self.activate()
        self.now+=.04;self.feed();self.assertIsNone(self.latch.stack_request())
        self.now+=.02;self.feed();self.latch.seen.pop(('can3',2))
        self.assertIsNone(self.latch.stack_request(),'unseen motor is not healthy CAN')
        self.feed()
        for phase in ('idle','probe','starting','releasing','latched'):
            self.latch.phase=phase
            self.assertIsNone(self.latch.stack_request())
        self.assertEqual(self.latch.stack_requests,[])

    def test_diagnostic_snapshot_only_contains_expected_motor_ages(self):
        self.latch=guard.FeedbackLatch(('can3','can1'),lambda:self.now,disabled_grippers=['right'])
        empty=self.latch.diagnostic_snapshot()
        self.assertNotIn('can1:8',empty['motor_rx_age_s'])
        self.assertIsNone(empty['motor_rx_age_s']['can3:8'])
        self.latch.begin();self.latch.phase='starting';self.activate()
        self.now+=.02
        snapshot=self.latch.diagnostic_snapshot()
        self.assertAlmostEqual(snapshot['motor_rx_age_s']['can3:8'],.02)

    def test_native_disabled_clear_wait_is_not_an_enabled_output_stream(self):
        self.latch.output_started.clear()
        self.latch.receive('can3', *frame(5,0), self.now)
        self.now += .12
        self.assertIsNone(self.latch.check())
        self.latch.receive('can3',5,b'\x7f'*8,self.now,socket.MSG_DONTROUTE)
        self.latch.receive('can3', *frame(5,1), self.now)
        self.now += .101
        self.assertIn('can3:5', self.latch.check())

    def test_startup_checks_seen_motors_and_has_a_constructor_deadline(self):
        self.latch.receive('can3',5,b'\x00'*8,self.now,socket.MSG_DONTROUTE)
        self.latch.receive('can3', *frame(5), self.now)
        self.now += .101
        self.assertIn('can3:5', self.latch.check())
        self.setUp()
        self.now += guard.STARTUP_TIMEOUT + .01
        self.assertIn('initialization deadline', self.latch.check())

    def test_release_timeline_is_bounded_and_does_not_relax_feedback_deadline(self):
        self.activate();self.latch.phase='releasing'
        self.latch.note_release('releasing_received')
        self.now+=2.03;self.feed()
        self.now+=.101;self.feed(omitted={('can3',1)})
        self.assertIn('can3:1',self.latch.check())
        report=self.latch.diagnostic_snapshot()
        self.assertEqual(report['last_confirmed_release_step'],'releasing_received')
        self.assertEqual(report['release_events'][-1]['action'],'latched')
        self.assertEqual(report['retired_buses'],[])
        self.assertGreater(report['motor_rx_age_s']['can3:1'],.1)
        self.assertLess(report['motor_rx_age_s']['can1:1'],.01)
        for _ in range(20):self.latch.note_release('diagnostic_fixture')
        self.assertEqual(len(self.latch.release_events),16)
        self.assertGreater(self.latch.release_events_truncated,0)

    def test_release_keeps_joint_and_motor_reset_protection(self):
        self.activate()
        self.latch.phase = 'releasing'
        self.now += .101
        self.feed(omitted={('can1', 4)})
        self.assertIn('can1:4', self.latch.check())
        self.setUp()
        self.activate()
        self.latch.phase = 'releasing'
        self.latch.receive('can3', *frame(6, status=0), self.now)
        self.assertIn('disabled/fault', self.latch.check())

    def test_disabled_left_jaw_does_not_exempt_left_arm_or_right_jaw(self):
        self.latch = guard.FeedbackLatch(('can3', 'can1'), lambda: self.now,
                                        disabled_grippers=['left'])
        self.latch.begin()
        self.latch.phase = 'starting'
        self.activate()
        self.assertNotIn(('can3', 8), self.latch.expected)
        self.assertIn(('can1', 8), self.latch.expected)
        self.latch.receive('can3', *frame(8, status=15), self.now)
        self.assertIsNone(self.latch.check())
        self.now += .101
        self.feed(omitted={('can3', 1)})
        self.latch.heartbeat = self.now
        self.assertIn('can3:1', self.latch.check())

    def test_disabled_right_jaw_keeps_both_arms_and_left_gripper_required(self):
        self.latch = guard.FeedbackLatch(('can3','can1'),lambda:self.now,disabled_grippers=['right'])
        self.latch.begin();self.latch.phase='starting';self.activate()
        self.assertNotIn(('can1',8),self.latch.expected)
        self.assertIn(('can3',8),self.latch.expected)
        self.now += .101
        self.feed(omitted={('can3',8)})
        self.latch.heartbeat=self.now
        self.assertIn('can3:8',self.latch.check())


WRITER = """
import atexit, pathlib, sys, time
out, cleanup = map(pathlib.Path, sys.argv[1:])
atexit.register(lambda: cleanup.write_text('native destructor would run'))
with out.open('ab', buffering=0) as stream:
    while True:
        stream.write(b'x')
        time.sleep(.005)
"""


class ProcessTests(unittest.TestCase):
    def test_expired_supervisor_kills_before_latch_persistence_and_skips_diagnostic_signal(self):
        latch_type=guard.FeedbackLatch;order=[];peers=[]
        class Child:
            pid=123456
            alive=True
            def poll(self):return None if self.alive else -signal.SIGKILL
            def wait(self,timeout):
                self.persisted_before_wait=self.record.exists()
                return -signal.SIGKILL
        child=Child()
        def make_latch(buses,**kwargs):
            latch=latch_type(buses,lambda:100.,**kwargs)
            latch.phase='active';latch.heartbeat=99.89
            latch.seen={key:100. for key in latch.expected}
            return latch
        def open_bus(_):
            reader,peer=socket.socketpair();peers.append(peer);return reader
        def kill(value):self.assertIs(value,child);order.append('kill');child.alive=False
        write=Path.write_text
        def persist(path,text,*args,**kwargs):
            state=json.loads(text)['state']
            if state=='latched':self.assertEqual(order[0],'kill');order.append('latched')
            return write(path,text,*args,**kwargs)
        with tempfile.TemporaryDirectory() as directory:
            child.record=Path(directory)/'can-output-guard.json'
            profile=dict(can_interfaces=dict(left='can3',right='can1'),storage_root=directory)
            try:
                with patch.object(guard.sys,'platform','linux'),patch.object(guard,'FeedbackLatch',side_effect=make_latch), \
                     patch.object(guard,'open_can',side_effect=open_bus),patch.object(guard.subprocess,'Popen',return_value=child), \
                     patch.object(guard,'terminate_sdk',side_effect=kill),patch.object(Path,'write_text',persist), \
                     patch.object(latch_type,'stack_request',side_effect=AssertionError('expired guard must kill first')):
                    self.assertEqual(guard.supervise(profile,['no-process-executed']),-signal.SIGKILL)
                self.assertEqual(order,['kill','latched','latched'])
                self.assertTrue(child.persisted_before_wait)
                receipt=json.loads((Path(directory)/'can-output-guard.json').read_text())
                self.assertAlmostEqual(receipt['diagnostics']['last_heartbeat_age_s'],.11)
            finally:
                for peer in peers:peer.close()

    def test_signal_race_preserves_process_not_present_evidence(self):
        child=SimpleNamespace(pid=123456,poll=lambda:None)
        with patch.object(guard.os,'kill',side_effect=ProcessLookupError):
            evidence=guard.terminate_sdk(child)
        self.assertEqual(evidence['returncode_before_signals'],None)
        self.assertTrue(all(row['result']=='process_not_present_at_signal' and not row['sent'] for row in evidence['signals']))

    def test_termination_stops_background_output_without_destructor(self):
        with tempfile.TemporaryDirectory() as directory:
            out, cleanup = Path(directory)/'output', Path(directory)/'destructor'
            child = subprocess.Popen([sys.executable, '-c', WRITER, str(out), str(cleanup)])
            try:
                wait_until(lambda: out.exists() and out.stat().st_size >= 4)
                guard.terminate_sdk(child)
                self.assertEqual(child.wait(timeout=2), -signal.SIGKILL)
                size = out.stat().st_size
                time.sleep(.05)
                self.assertEqual(out.stat().st_size, size)
                self.assertFalse(cleanup.exists())
            finally:
                if child.poll() is None:
                    child.kill()
                child.wait(timeout=2)

    @unittest.skipUnless(sys.platform == 'linux', 'Linux parent-death signal')
    def test_parent_death_kills_output_child(self):
        with tempfile.TemporaryDirectory() as directory:
            out, ready = Path(directory)/'output', Path(directory)/'ready'
            worker = ("import os, pathlib, sys, time\n"
                      f"sys.path.insert(0, {str(ROOT)!r})\n"
                      "from robot_workbench.arx_output_guard import parent_death\n"
                      "parent_death(int(sys.argv[1]))\n"
                      "pathlib.Path(sys.argv[3]).write_text(str(os.getpid()))\n"
                      "with open(sys.argv[2], 'ab', buffering=0) as stream:\n"
                      "    while True:\n"
                      "        stream.write(b'x'); time.sleep(.005)\n")
            parent_code = ("import os, subprocess, sys, time\n"
                           f"child=subprocess.Popen([sys.executable,'-c',{worker!r},str(os.getpid()),sys.argv[1],sys.argv[2]])\n"
                           "while True: time.sleep(1)\n")
            parent = subprocess.Popen([sys.executable, '-c', parent_code, str(out), str(ready)])
            child_pid = child_start = None
            try:
                wait_until(lambda: ready.exists() and out.exists() and out.stat().st_size >= 4)
                child_pid = int(ready.read_text())
                child_start = Path(f'/proc/{child_pid}/stat').read_text().split(') ', 1)[1].split()[19]
                parent.kill()
                parent.wait(timeout=2)
                def stopped():
                    stat = Path(f'/proc/{child_pid}/stat')
                    return not stat.exists() or stat.read_text().split(') ', 1)[1].startswith('Z ')
                wait_until(stopped)
                size = out.stat().st_size
                time.sleep(.05)
                self.assertEqual(out.stat().st_size, size)
            finally:
                if parent.poll() is None:
                    parent.kill()
                parent.wait(timeout=2)
                if child_pid is not None and Path(f'/proc/{child_pid}/stat').exists():
                    # The grandchild may already have been reaped; never signal
                    # a reused PID even when cleaning up a failed test.
                    try:
                        fields = Path(f'/proc/{child_pid}/stat').read_text().split(') ', 1)[1].split()
                        if fields[19] == child_start and fields[0] != 'Z':
                            os.kill(child_pid, signal.SIGKILL)
                    except (FileNotFoundError, ProcessLookupError):pass


# The driver runs outside the supervisor and exchanges only Unix datagrams.
BUS_DRIVER = """
import json, pathlib, select, socket, struct, sys, time
mode, directory, *fds = sys.argv[1:]
sockets = [socket.socket(fileno=int(fd)) for fd in fds]
for sock in sockets: sock.setblocking(False)
active = pathlib.Path(directory)/'active'
activated = None
deadline = time.monotonic()+5
while time.monotonic() < deadline:
    for sock in sockets:
        while True:
            try:
                packet=sock.recv(1024)
                if active.exists():raise AssertionError('read query sent while SDK is active')
            except BlockingIOError: break
            except OSError: sys.exit(0)
    if active.exists() and activated is None: activated = time.monotonic()
    age = 0 if activated is None else time.monotonic()-activated
    for index, sock in enumerate(sockets):
        for motor in (1, 2, 4, 5, 6, 7, 8):
            if index == 0 and motor == 8: continue
            if mode == 'loss' and index == 0 and motor == 2 and .08 < age < .35: continue
            if mode.startswith('release_loss') and index == 0 and age > .08: continue
            if motor in (1, 2, 4): ident, data = motor, bytes([32,127,255,127,255,0,0,0])
            else: ident, data = 0, bytes([16|motor,104,44,127,247,255,39,33])
            try:
                sock.send(struct.pack('=IB3x8s',motor|0x40000000,8,bytes([0]*8)))
                sock.send(struct.pack('=IB3x8s',ident,8,data))
            except OSError: pass
    time.sleep(.005)
"""

SDK_STANDIN = """
import pathlib, sys, threading, time
sys.path.insert(0, sys.argv[1])
from robot_workbench.arx_output_guard import bind_child, require_guard
mode, directory = sys.argv[2], pathlib.Path(sys.argv[3])
bind_child()
client = require_guard()
assert client.disabled_grippers == ['left']
admission=client.call('begin')
done = threading.Event()
def native_output():
    with (directory/'output').open('ab', buffering=0) as stream:
        while not done.wait(.005): stream.write(b'x')
thread = threading.Thread(target=native_output)
thread.start()
time.sleep(.03)
if mode.startswith('quiesce'):
    for side in ('left','right'):
        client.call('register_sdk',side=side,identity=dict(library_build_id='a'*64,controller_instance_id=side))
client.call('activate')
(directory/'active').write_text('explicit connect once')
for _ in range(100):
    if mode=='exit23':
        import os
        os._exit(23)
    if mode=='segfault':
        import os, resource, signal
        resource.setrlimit(resource.RLIMIT_CORE,(0,0))
        os.kill(os.getpid(),signal.SIGSEGV)
    if mode=='ipc_closed':
        client.connection.close()
        time.sleep(1)
    if mode=='trip_detail':
        client.call('trip',detail='RuntimeError: CPU construction sentinel')
        time.sleep(1)
    client.call('heartbeat')
    if mode == 'heartbeat_future':
        threading.Timer(.3,lambda:(directory/'future-reference-applied').write_text('must not survive guard')).start()
    if mode.startswith('heartbeat'): time.sleep(.5)
    time.sleep(.01)
    if mode.startswith('quiesce'):
        client.call('releasing')
        trace=client.release_trace;trace.begin();trace.mark('guard_releasing','returned',time.monotonic_ns())
        done.set();thread.join();stamp=time.monotonic_ns()
        if mode=='quiesce_late':time.sleep(.12)
        for side in ('left','right'):
            report=dict(library_build_id=('b' if mode=='quiesce_bad' else 'a')*64,
                controller_instance_id=side,caller_generation=admission['generation'],
                send_thread_joined=True,idempotent_hit=False,last_send_attempt_monotonic_ns=stamp,
                quiesce_returned_monotonic_ns=stamp,receiver_state='vendor_owned_unverified',
                transport_basis='trusted_vendor_binary',verification_scope='controller_application')
            client.call('retire_'+side,report=report)
        trace.mark('guard_disarm','started',time.monotonic_ns());client.call('disarm')
        trace.mark('guard_disarm','returned',time.monotonic_ns())
        (directory/'released').write_text('candidate private IPC release');break
    if mode in ('release','release_loss','release_loss_lie'):
        client.call('releasing')
        trace=client.release_trace
        assert trace is not None
        trace.begin();trace.mark('guard_releasing','returned',time.monotonic_ns())
        trace.mark('destroy_left','started',time.monotonic_ns())
        if mode=='release_loss_lie':trace.mark('retire_left','returned',time.monotonic_ns())
        if mode.startswith('release_loss'):time.sleep(1)
        time.sleep(.03)
        done.set(); thread.join()
        trace.mark('destroy_left','returned',time.monotonic_ns())
        trace.mark('retire_left','started',time.monotonic_ns())
        client.call('retire_left')
        trace.mark('retire_left','returned',time.monotonic_ns())
        client.call('retire_right')
        trace.mark('guard_disarm','started',time.monotonic_ns())
        client.call('disarm')
        trace.mark('guard_disarm','returned',time.monotonic_ns())
        (directory/'released').write_text('native output stopped before disarm')
        break
else:
    raise AssertionError('missing feedback should have terminated this child')
"""


@unittest.skipUnless(sys.platform == 'linux', 'Linux timestamp and supervisor integration')
class SupervisorTests(unittest.TestCase):
    def test_real_supervisor_child_and_kernel_rx_with_disabled_left_jaw(self):
        for mode in ('release', 'loss', 'heartbeat','heartbeat_future','release_loss','release_loss_lie','quiesce','quiesce_bad','quiesce_late','trip_detail','exit23','segfault','ipc_closed'):
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as directory:
                pairs = [socket.socketpair(type=socket.SOCK_DGRAM) for _ in range(2)]
                class FakeCan:
                    def __init__(self,sock):self.sock=sock
                    def __getattr__(self,name):return getattr(self.sock,name)
                    def fileno(self):return self.sock.fileno()
                    def recvmsg(self,*args):
                        raw,anc,flags,address=self.sock.recvmsg(*args)
                        ident,n,data=struct.unpack('=IB3x8s',raw)
                        if ident&0x40000000:
                            raw=struct.pack('=IB3x8s',ident&~0x40000000,n,data);flags|=socket.MSG_DONTROUTE
                        return raw,anc,flags,address
                readers = dict(zip(('can3', 'can1'), (FakeCan(pair[0]) for pair in pairs)))
                for sock in readers.values():
                    sock.setsockopt(socket.SOL_SOCKET, 35, 1)
                    sock.setblocking(False)
                driver_fds = [pair[1].fileno() for pair in pairs]
                driver = subprocess.Popen([sys.executable, '-c', BUS_DRIVER, mode, directory,
                                           *map(str, driver_fds)], pass_fds=driver_fds)
                old_alarm = signal.signal(signal.SIGALRM,
                    lambda *_: (_ for _ in ()).throw(TimeoutError('CPU supervisor test exceeded 5 seconds')))
                signal.setitimer(signal.ITIMER_REAL, 5)
                try:
                    profile = dict(can_interfaces=dict(left='can3', right='can1'),
                                   disabled_grippers=['left'], storage_root=directory)
                    if mode.startswith('quiesce'):
                        import hashlib
                        artifact=Path(directory)/'cpu-sdk';artifact.write_bytes(b'inert CPU artifact')
                        row=dict(path=str(artifact),sha256=hashlib.sha256(artifact.read_bytes()).hexdigest())
                        profile['sdk_quiesce']=dict(library_build_id='a'*64,module=row,vendor_libraries=[row],
                                                   vendor_trust='human-approved-unmodified-binary')
                    latches=[];terminated=[];latch_type=guard.FeedbackLatch
                    snapshot=guard.ReleaseTrace.snapshot;terminate=guard.terminate_sdk
                    def make_latch(*args,**kwargs):
                        value=latch_type(*args,**kwargs);latches.append(value);return value
                    def kill(child):terminated.append(child.pid);return terminate(child)
                    def read_trace(buffer,pid,**kwargs):
                        latch=latches[-1]
                        if latch.reason:
                            if mode not in ('exit23','segfault'):self.assertIn(pid,terminated)
                        else:
                            self.assertEqual(latch.phase,'idle')
                            self.assertEqual(latch.release_events[-1]['action'],'disarm_received')
                        return snapshot(buffer,pid,**kwargs)
                    with patch.object(guard, 'open_can', side_effect=lambda bus: readers[bus]),\
                            patch.object(guard,'FeedbackLatch',side_effect=make_latch),\
                            patch.object(guard,'terminate_sdk',side_effect=kill),\
                            patch.object(guard.ReleaseTrace,'snapshot',read_trace):
                        result = guard.supervise(profile,
                            [sys.executable, '-c', SDK_STANDIN, str(ROOT), mode, directory])
                    receipt = json.loads((Path(directory)/'can-output-guard.json').read_text())
                    trace=Path(receipt['stack_log'])
                    self.assertTrue(trace.is_file());self.assertEqual(trace.stat().st_mode&0o777,0o600)
                    self.assertTrue((Path(directory)/'active').exists())
                    if mode in ('release','quiesce'):
                        self.assertEqual(result, 0)
                        self.assertEqual(receipt['state'], 'disconnected')
                        self.assertTrue((Path(directory)/'released').exists())
                        self.assertEqual(receipt['stack_requests'],[]);self.assertEqual(trace.read_text(),'')
                        evidence=receipt['release_trace']
                        self.assertEqual(evidence['status'],'observed');self.assertEqual(evidence['source_pid'],receipt['child_pid'])
                        self.assertEqual(evidence['events'][0]['step'],'guard_releasing')
                        self.assertEqual(evidence['last_confirmed_step']['step'],'guard_disarm')
                        self.assertEqual(evidence['ipc_order_consistency'],'consistent')
                    else:
                        if mode=='segfault':
                            # Fatal-signal teardown can outlast the watchdog window.
                            # Both exits require the real fatal stack and reap evidence below.
                            self.assertIn(result,(-signal.SIGSEGV,-signal.SIGKILL))
                        else:self.assertEqual(result,23 if mode=='exit23' else -signal.SIGKILL)
                        self.assertEqual(receipt['state'], 'latched')
                        process=receipt['process_evidence']
                        self.assertEqual(process['returncode'],result)
                        self.assertEqual(process['signal_sender'],'not_inferred')
                        self.assertIsNotNone(process['first_failure'])
                        self.assertIn(process['last_ipc']['action'],('activate','heartbeat','trip','releasing','retire_left','retire_right'))
                        if mode in ('exit23','segfault'):
                            if mode=='segfault':
                                self.assertIn('Fatal Python error: Segmentation fault',trace.read_text())
                                if result==-signal.SIGKILL:
                                    self.assertEqual(receipt['last_latch']['reason'],'SDK owner heartbeat lost')
                                    self.assertEqual([r['signal'] for r in process['terminations'][0]['signals']],['SIGSTOP','SIGKILL'])
                        elif mode=='ipc_closed':
                            self.assertIn('guard IPC closed',receipt['last_latch']['reason'])
                            self.assertEqual([r['signal'] for r in process['terminations'][0]['signals']],['SIGSTOP','SIGKILL'])
                        elif mode=='loss':
                            self.assertIn('can3:2', receipt['last_latch']['reason'])
                            self.assertEqual(receipt['stack_requests'],[])
                            self.assertGreater(receipt['diagnostics']['motor_rx_age_s']['can3:2'],.1)
                        elif mode.startswith('release_loss'):
                            self.assertIn('CAN feedback lost: can3:',receipt['last_latch']['reason'])
                            self.assertEqual(receipt['diagnostics']['retired_buses'],[])
                            evidence=receipt['release_trace'];self.assertEqual(evidence['status'],'observed')
                            if mode=='release_loss':
                                self.assertEqual(evidence['last_confirmed_step']['step'],'destroy_left')
                                self.assertEqual(evidence['last_confirmed_step']['status'],'started')
                                self.assertEqual(evidence['ipc_order_consistency'],'consistent')
                            else:
                                self.assertEqual(evidence['ipc_order_consistency'],'inconsistent')
                                self.assertEqual(evidence['missing_guard_receipts'],['retire_left_received'])
                            self.assertEqual(receipt['stack_requests'],[])
                        elif mode.startswith('quiesce'):
                            self.assertIn('invalid or stale SDK quiesce report',receipt['last_latch']['reason'])
                            self.assertEqual(receipt['diagnostics']['retired_buses'],[])
                        elif mode=='trip_detail':
                            self.assertEqual(receipt['last_latch']['reason'],
                                'SDK acquisition or native call failed: RuntimeError: CPU construction sentinel')
                        else:
                            self.assertEqual(receipt['last_latch']['reason'],'SDK owner heartbeat lost')
                            self.assertEqual(len(receipt['stack_requests']),1)
                            self.assertGreater(receipt['diagnostics']['last_heartbeat_age_s'],.1)
                            self.assertLess(receipt['diagnostics']['last_heartbeat_age_s'],.2)
                            self.assertTrue(all(age<.1 for age in receipt['diagnostics']['motor_rx_age_s'].values()))
                            self.assertIn('native_output',trace.read_text())
                        size = (Path(directory)/'output').stat().st_size
                        # Driver restores all feedback; the old process stays dead.
                        time.sleep(.4)
                        self.assertEqual((Path(directory)/'output').stat().st_size, size)
                        self.assertFalse((Path(directory)/'released').exists())
                        if mode=='heartbeat_future':self.assertFalse((Path(directory)/'future-reference-applied').exists())
                finally:
                    signal.setitimer(signal.ITIMER_REAL, 0)
                    signal.signal(signal.SIGALRM, old_alarm)
                    if driver.poll() is None:driver.kill()
                    driver.wait(timeout=2)
                    for pair in pairs:
                        for sock in pair:sock.close()


if __name__ == '__main__':
    unittest.main()
