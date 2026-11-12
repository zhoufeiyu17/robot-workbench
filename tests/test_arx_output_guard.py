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


class GuardCANBudgetTests(unittest.TestCase):
    def test_non_ipc_batch_keeps_wall_stages_without_extra_thread_usage_reads(self):
        with patch.object(guard.CANBatchTiming,'read_usage',side_effect=AssertionError('no per-loop rusage')):
            probe=guard.CANBatchTiming(detailed=False)
            probe.mark('recvmsg');probe.mark('validate_feedback');timing=probe.finish()
        self.assertIn('recvmsg',timing['stages'])
        self.assertIsNone(timing['stages']['recvmsg']['cpu_s'])
        self.assertIsNone(timing['thread_usage_delta'])

    def test_pending_ipc_probe_attributes_wait_to_receive_without_changing_feedback(self):
        class Reader:
            done=False
            def recvmsg(self,*_):
                if self.done:raise BlockingIOError
                self.done=True;time.sleep(.004);ns=time.time_ns();ident,data=frame(6)
                return struct.pack('=IB3x8s',ident,8,data),[(socket.SOL_SOCKET,35,
                    struct.pack('@ll',ns//10**9,ns%10**9))],0,None
        reader=Reader();latch=guard.FeedbackLatch(('can3','can1'))
        latch.phase='active';latch.started=time.monotonic()-.1
        probe=guard.CANBatchTiming()
        self.assertEqual(guard.receive_ready_can([reader],{reader:'can3'},latch,probe),1)
        timing=probe.finish()
        self.assertIn(('can3',6),latch.seen);self.assertIsNone(latch.reason)
        self.assertGreaterEqual(timing['stages']['recvmsg']['wall_s'],.003)
        self.assertLess(timing['stages']['recvmsg']['cpu_s'],timing['stages']['recvmsg']['wall_s'])
        self.assertIn('validate_feedback',timing['stages'])
        self.assertIn('apply_coalesced',timing['stages'])

    def test_coalescing_matches_ordered_active_and_release_health_updates(self):
        def packet(motor,status=1,stamp=.999,flags=0):
            ident,data=frame(motor,status)
            return ident,data,stamp,flags
        normal=[packet(m)for _ in range(8)for m in (1,2,4,5,6,7,8)]
        cases=[normal, normal+[packet(6,2),packet(6,1)],
               normal+[packet(6,0),packet(6,1)],
               normal+[packet(6,1,.8),packet(6,1,1.1)],
               [packet(6,2,.999,socket.MSG_DONTROUTE)]+normal]
        for phase in ('active','releasing'):
            for retired in (set(),{'can3'}):
                for values in cases:
                    baseline=guard.FeedbackLatch(('can3','can1'),clock=lambda:1.,disabled_grippers=['right'])
                    candidate=guard.FeedbackLatch(('can3','can1'),clock=lambda:1.,disabled_grippers=['right'])
                    for latch in (baseline,candidate):
                        latch.phase=phase;latch.started=.9;latch.retired=set(retired)
                        latch.enabled=set(latch.expected)
                    for bus in ('can3','can1'):
                        for ident,data,stamp,flags in values:
                            baseline.receive(bus,ident,data,stamp,flags)
                            if baseline.reason:break
                        if baseline.reason:break
                    class Reader:
                        def __init__(self):self.rows=iter(values)
                        def recvmsg(self,*_):
                            try:ident,data,stamp,flags=next(self.rows)
                            except StopIteration:raise BlockingIOError
                            ns=round(stamp*1e9)
                            return struct.pack('=IB3x8s',ident,8,data),[(socket.SOL_SOCKET,35,
                                struct.pack('@ll',ns//10**9,ns%10**9))],flags,None
                    left,right=Reader(),Reader();calls=[];apply=candidate.apply_feedback
                    def observe(*row):calls.append(row);apply(*row)
                    candidate.apply_feedback=observe
                    with patch.object(guard.time,'monotonic',return_value=1.),patch.object(guard.time,'time_ns',return_value=10**9):
                        guard.receive_ready_can([left,right],{left:'can3',right:'can1'},candidate)
                    self.assertEqual(candidate.seen,baseline.seen)
                    self.assertEqual(candidate.statuses,baseline.statuses)
                    self.assertEqual(candidate.enabled,baseline.enabled)
                    self.assertEqual(candidate.reason,baseline.reason)
                    if not baseline.reason:self.assertLessEqual(len(calls),13)
                    # The original timeout/retirement gate still sees the same
                    # last actual RX time after normal updates are compressed.
                    for latch in (baseline,candidate):latch.clock=lambda:1.101
                    self.assertEqual(candidate.check(),baseline.check())

    def test_connected_backlog_is_time_bounded_fair_and_keeps_original_rx_time(self):
        for phase in ('active','releasing','starting'):
            now=[1.];seen=[]
            class Reader:
                def __init__(self):self.remaining=128
                def recvmsg(self,*_):
                    if not self.remaining:raise BlockingIOError
                    self.remaining-=1;now[0]+=.0002
                    ident,data=frame(3)
                    return struct.pack('=IB3x8s',ident,8,data),[(socket.SOL_SOCKET,35,struct.pack('@ll',0,999000000))],0,None
            left,right=Reader(),Reader();parent=object()
            latch=SimpleNamespace(phase=phase,started=.9,reason=None,receive=lambda *v:seen.append(v),
                                  rx_evidence=guard.FeedbackLatch(('left','right')).rx_evidence)
            latch.validated_feedback=lambda bus,ident,data,stamp,flags:(bus,ident,None,stamp)
            latch.apply_feedback=lambda *v:seen.append(v)
            with patch.object(guard.time,'monotonic',side_effect=lambda:now[0]),patch.object(guard.time,'time_ns',side_effect=lambda:round(now[0]*1e9)):
                count=guard.receive_ready_can([left,parent,right],{left:'left',right:'right'},latch)
            if phase=='starting':self.assertEqual(count,256,'activation keeps the prior acquisition order')
            else:
                self.assertLessEqual(now[0]-1.,.0013);self.assertEqual(count,6)
                self.assertEqual(left.remaining,125);self.assertEqual(right.remaining,125)
            self.assertEqual(left.remaining+right.remaining+count,256,'unprocessed frames remain queued')
            self.assertEqual({r[0]for r in seen},{'left','right'})
            self.assertTrue(all(abs(r[3]-.999)<1e-8 for r in seen),'backlog must not become freshly timestamped feedback')

    def test_known_can_fault_does_not_wait_for_another_batch_or_heartbeat(self):
        class Reader:
            def recvmsg(self,*_):
                ident,data=frame(3,2)
                return struct.pack('=IB3x8s',ident,8,data),[(socket.SOL_SOCKET,35,struct.pack('@ll',1,0))],0,None
        reader=Reader();latch=SimpleNamespace(phase='active',started=0.,reason=None,
                                            rx_evidence=guard.FeedbackLatch(('left',)).rx_evidence)
        latch.receive=lambda *_:setattr(latch,'reason','motor fault')
        latch.validated_feedback=lambda bus,ident,data,stamp,flags:(bus,3,2,stamp)
        latch.apply_feedback=latch.receive
        with patch.object(guard.time,'monotonic',return_value=1.),patch.object(guard.time,'time_ns',return_value=1_000_000_000):
            self.assertEqual(guard.receive_ready_can([reader],{reader:'left'},latch),1)
        self.assertEqual(latch.reason,'motor fault')


class GuardClientIPCTests(unittest.TestCase):
    def test_heartbeat_reply_after_60ms_does_not_trigger_the_old_50ms_selfkill(self):
        trace=guard.IPCFailureTrace.create();parent,peer=socket.socketpair()
        code=("import socket,sys,json;sys.path.insert(0,sys.argv[1]);"
              "from robot_workbench.arx_output_guard import GuardClient,IPCFailureTrace;"
              "c=GuardClient(socket.socket(fileno=int(sys.argv[2])),IPCFailureTrace(int(sys.argv[3])));"
              "c.call('heartbeat');print('returned',flush=True);print(json.dumps(c.last_call_timing),flush=True)")
        child=subprocess.Popen([sys.executable,'-B','-c',code,str(ROOT),str(peer.fileno()),str(trace.fd)],
            pass_fds=(peer.fileno(),trace.fd),stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
        peer.close()
        try:
            parent.settimeout(2);request=parent.recv(4096);self.assertEqual(json.loads(request)['action'],'heartbeat')
            time.sleep(.06);parent.sendall(b'{"healthy":true}\n')
            output,error=child.communicate(timeout=2)
            self.assertEqual(child.returncode,0,error);self.assertIn('returned',output);self.assertIsNone(trace.snapshot())
            timing=json.loads(output.splitlines()[-1]);self.assertEqual(timing['action'],'heartbeat')
            self.assertGreaterEqual(timing['response_wait_decode_s'],.055)
            self.assertGreaterEqual(timing['total_s'],timing['response_wait_decode_s'])
            self.assertLess(timing['cpu_s'],timing['response_wait_decode_s']/2.,'waiting must not be labeled as computation')
        finally:
            if child.poll()is None:child.kill();child.wait()
            parent.close();trace.close()

    def test_no_reply_still_fails_stop_and_preserves_failure_memory_without_disk_io(self):
        trace=guard.IPCFailureTrace.create();parent,peer=socket.socketpair()
        code=("import socket,sys;sys.path.insert(0,sys.argv[1]);"
              "from robot_workbench.arx_output_guard import GuardClient,IPCFailureTrace;"
              "GuardClient(socket.socket(fileno=int(sys.argv[2])),IPCFailureTrace(int(sys.argv[3]))).call('heartbeat')")
        child=subprocess.Popen([sys.executable,'-B','-c',code,str(ROOT),str(peer.fileno()),str(trace.fd)],
            pass_fds=(peer.fileno(),trace.fd),stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
        peer.close()
        try:
            parent.settimeout(2);parent.recv(4096);child.communicate(timeout=2)
            self.assertEqual(child.returncode,-signal.SIGKILL)
            receipt=trace.snapshot();self.assertEqual(receipt['action'],'heartbeat');self.assertEqual(receipt['stage'],'receive')
            self.assertTrue(receipt['self_sigkill_requested']);self.assertEqual(receipt['source_pid'],child.pid)
            self.assertEqual(receipt['budget_s'],guard.TIMEOUT)
            self.assertGreaterEqual(receipt['failed_monotonic_ns']-receipt['started_monotonic_ns'],90_000_000)
        finally:
            if child.poll()is None:child.kill();child.wait()
            parent.close();trace.close()


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
    def test_receive_evidence_distinguishes_stale_queue_future_clock_and_socket_loss(self):
        from collections import deque
        latch=guard.FeedbackLatch(('can3','can1'),clock=lambda:10.)
        latch.started=9.;latch.phase='active'
        raw=struct.pack('=IB3x8s',1,8,b'\x20'+b'\0'*7)
        frames=deque((raw,[(socket.SOL_SOCKET,35,struct.pack('@ll',sec,nano)),
                          (socket.SOL_SOCKET,40,struct.pack('=I',4))],0,None)
                     for sec,nano in ((19,800_000_000),(20,10_000_000),(19,999_000_000)))
        class Reader:
            def recvmsg(self,*_):
                if frames:return frames.popleft()
                raise BlockingIOError()
        reader=Reader()
        with patch.object(guard.time,'monotonic',return_value=10.),patch.object(guard.time,'time_ns',return_value=20_000_000_000):
            self.assertEqual(guard.receive_ready_can([reader],{reader:'can3'},latch),3)
        row=latch.diagnostic_snapshot()['receive_evidence']['can3']
        self.assertEqual((row['frames'],row['valid_feedback_frames'],row['stale_frames'],row['future_frames'],row['socket_drops']),(3,1,1,1,4))
        self.assertAlmostEqual(row['maximum_rx_age_s'],.2)
        self.assertAlmostEqual(latch.seen['can3',1],9.999)
        self.assertEqual(latch.heartbeat,10.)

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
                    try:return stat.read_text().split(') ', 1)[1].startswith('Z ')
                    except (FileNotFoundError,ProcessLookupError):return True
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
    heartbeat=client.call('heartbeat')
    loop=heartbeat['server_timing']['loop']
    assert loop['select_started_monotonic_ns']<=heartbeat['server_timing']['selected_monotonic_ns']
    assert all(loop[k]>=0 for k in ('before_select_s','select_wall_s','select_cpu_s'))
    assert loop['previous_batch']['can_batch_s']>=0
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
        for mode in ('release', 'loss', 'heartbeat','heartbeat_future','release_loss','release_loss_lie','quiesce','quiesce_bad','quiesce_late','trip_detail','exit23','segfault','ipc_closed','network_down'):
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
                    receive_can=guard.receive_ready_can
                    def receive(*args,**kwargs):
                        if mode=='network_down'and latches[-1].phase=='active'and (Path(directory)/'active').exists():
                            raise OSError(100,'Network is down')
                        return receive_can(*args,**kwargs)
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
                            patch.object(guard.ReleaseTrace,'snapshot',read_trace),\
                            patch.object(guard,'receive_ready_can',side_effect=receive):
                        if mode=='network_down':
                            with self.assertRaisesRegex(OSError,'Network is down'):
                                guard.supervise(profile,[sys.executable,'-c',SDK_STANDIN,str(ROOT),mode,directory])
                            result=json.loads((Path(directory)/'can-output-guard.json').read_text())['process_evidence']['returncode']
                        else:
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
                        if mode=='network_down':
                            self.assertEqual(receipt['last_latch']['reason'],'supervisor failed: [Errno 100] Network is down')
                            self.assertEqual(process['wait']['status'],'returned')
                            self.assertEqual(process['returncode'],-signal.SIGKILL)
                            self.assertTrue(receipt['stack_capture']['archived'])
                        elif mode in ('exit23','segfault'):
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
