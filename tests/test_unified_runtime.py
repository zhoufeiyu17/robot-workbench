"""CPU protocol/race qualification of the actual Owner strategies; no real SDK."""
from copy import deepcopy
from itertools import islice, permutations
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT/'tests'), str(ROOT/'calibration'),
               str(ROOT/'calibration/infra'), str(ROOT/'calibration/tests'), str(ROOT/'rule_based_workbench')]
from robot_workbench.runtime import Runtime
from test_owner_clock import ClockBackend, Safety, calibration, jaw
from test_frozen_executor import fixture


class UnifiedRuntimeTest(unittest.TestCase):
    def test_pre_backend_lock_wait_is_reported_separately_from_fast_backend(self):
        runtime,_=self.create();events=[];started=threading.Event();results=[]
        def invoke():
            started.set()
            results.append(runtime._backend_call('read_gains',lambda:{'cpu_fixture':True}))
        with patch.object(runtime.owner,'event',side_effect=lambda name,**value:events.append((name,value))):
            with runtime.lock:
                worker=threading.Thread(target=invoke);worker.start()
                self.assertTrue(started.wait(1));time.sleep(.025)
            worker.join(2)
        self.assertFalse(worker.is_alive());self.assertEqual(results,[{'cpu_fixture':True}])
        records=[value for name,value in events if name=='sdk_call_latency']
        self.assertEqual(len(records),1);call=records[0]['call']
        self.assertGreaterEqual(call['admission_lock_wait_s'],.02)
        self.assertLess(call['backend_wall_s'],call['admission_lock_wait_s'])
        self.assertGreaterEqual(call['total_wall_s'],call['admission_lock_wait_s'])
        self.assertLess(call['admission_lock_cpu_s'],call['admission_lock_wait_s'])
        self.assertLess(call['entered_monotonic_s'],call['started_monotonic_s'])
        self.assertGreaterEqual(call['return_bookkeeping_s'],0.)

    def test_slow_sdk_call_is_recorded_at_source_even_without_status_polling(self):
        runtime,creds=self.create();events=[]
        with patch.object(runtime.owner,'event',side_effect=lambda name,**value:events.append((name,value))):
            result=runtime._backend_call('read_gains',lambda:(time.sleep(.012),{'cpu_fixture':True})[1])
        self.assertEqual(result,{'cpu_fixture':True})
        records=[value for name,value in events if name=='sdk_call_latency']
        self.assertEqual(len(records),1);self.assertEqual(records[0]['call']['operation'],'read_gains')
        call=records[0]['call'];self.assertGreaterEqual(call['returned_monotonic_s']-call['started_monotonic_s'],.01)
        self.assertLess(call['cpu_s'],.01);self.assertEqual(records[0]['runtime_nonce'],runtime.nonce)

    def test_release_waits_for_feedback_cycle_and_keeps_stop_independent(self):
        runtime,creds=self.create();entered=threading.Event();release=threading.Event();result=[]
        runtime.owner.backend.read_delay=lambda:(entered.set(),release.wait(1))
        tick=threading.Thread(target=lambda:self.advance(runtime));tick.start();self.assertTrue(entered.wait(1))
        self.assertTrue(runtime.snapshot(creds['calibration'])['allowed_operations']['release']['allowed'])
        def request():
            try:result.append(runtime.request_release(creds['calibration']))
            except Exception as error:result.append(error)
        worker=threading.Thread(target=request);worker.start()
        time.sleep(.01);self.assertTrue(worker.is_alive())
        runtime.request_stop();release.set();tick.join(1);worker.join(1)
        self.assertFalse(worker.is_alive());self.assertEqual(len(result),1)
        self.assertIsInstance(result[0],dict);self.assertEqual(runtime.release_receipt['status'],'requested')

    def test_protected_worker_retirement_keeps_failed_close_evidence_and_revokes_old_commands(self):
        runtime,creds=self.create()
        backend=runtime.owner.backend._backend
        backend.terminal_failure='CPU SDK worker heartbeat lost'
        backend.is_connected=False
        backend.protection_snapshot=lambda:dict(event=dict(id='cpu-protection',reason=backend.terminal_failure),worker_exit_confirmed=True)
        disposed=[];backend.dispose=lambda:disposed.append(True)
        runtime.ros_publisher=Mock();runtime.ros_node=Mock()
        with patch.object(backend,'close',side_effect=AssertionError('fault retirement is not normal SDK close')):
            runtime.owner.commands.put(('cpu-stale-command',runtime.owner.epoch,None))
            self.assertFalse(runtime.tick())
            self.assertTrue(runtime.owner.commands.empty())
            runtime.begin_maintenance()
            result=runtime.retire_protected()
        self.assertIsNone(runtime.fd);self.assertEqual(disposed,[True])
        runtime.ros_publisher.close.assert_called_once()
        runtime.ros_node.close.assert_called_once()
        self.assertTrue(result['worker_exit_confirmed']);self.assertFalse(result['sdk_close_confirmed'])
        self.assertEqual(result['mechanical_state'],'not_verified');self.assertFalse(result['motion_resumed'])
        with self.assertRaises(PermissionError):runtime.submit(creds['calibration'],dict(action='stop'))

    def test_idle_readiness_ignores_normal_tick_but_keeps_feedback_and_motion_guards(self):
        runtime,creds=self.create()
        runtime.tick_lock.acquire()
        try:
            state=runtime.snapshot(creds['calibration'])
            self.assertTrue(state['allowed_operations']['handoff']['allowed'])
            self.assertTrue(state['allowed_operations']['preview']['allowed'])
            self.assertTrue(state['execution_settings']['update_allowed'])
            self.assertTrue(state['idle_readiness']['allowed']);self.assertTrue(state['idle_readiness']['control_cycle_in_flight'])
            runtime.owner.feedback_healthy=False
            self.assertFalse(runtime.snapshot(creds['calibration'])['idle_readiness']['allowed'])
            runtime.owner.feedback_healthy=True
            runtime.owner.commands.put(('fixture',0,None))
            self.assertIn('pending',runtime.snapshot(creds['calibration'])['idle_readiness']['reason'])
            runtime.owner.commands.get_nowait()
            runtime.owner.control.state=runtime.owner.states.FAULT
            self.assertFalse(runtime.snapshot(creds['calibration'])['idle_readiness']['allowed'])
            runtime.owner.control.state=runtime.owner.states.READY
        finally:runtime.tick_lock.release()

    def test_handoff_waits_for_one_cycle_and_checks_cancellation_at_boundary(self):
        runtime,creds=self.create();entered=threading.Event();release=threading.Event();result=[]
        runtime.owner.backend.read_delay=lambda:(entered.set(),release.wait(1))
        tick=threading.Thread(target=lambda:self.advance(runtime));tick.start();self.assertTrue(entered.wait(1))
        active=[True]
        def handoff():
            try:result.append(runtime.handoff(creds['calibration'],'collection',active=lambda:active[0]))
            except Exception as error:result.append(error)
        worker=threading.Thread(target=handoff);worker.start()
        time.sleep(.01);self.assertTrue(worker.is_alive());active[0]=False;release.set()
        tick.join(1);worker.join(1);self.assertFalse(worker.is_alive())
        self.assertIsInstance(result[0],ValueError);self.assertIn('revoked',str(result[0]));self.assertEqual(runtime.holder,'calibration')
        runtime.owner.backend.read_delay=lambda:None
        runtime.handoff(creds['calibration'],'collection',active=lambda:True)
        self.assertEqual(runtime.holder,'collection')

    def setUp(self):
        jaw.configure_mapping(None)
        self.directory = TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.sim = 10.
        self.runtimes = []
        self.addCleanup(self.cleanup)

    def cleanup(self):
        for runtime, credentials in self.runtimes:
            if runtime.fd is not None:
                runtime.owner.recording = None
                if runtime.owner.backend.is_connected:
                    # Test-owned backend teardown may end an intentionally unfinished synthetic motion.
                    runtime.owner.submit(dict(action='release', confirmation='ARMS_SUPPORTED_RELEASE'))
                    runtime.tick()
                runtime.close()

    def create(self, *, device='robot', environment='fake', initialize=True, q=None, mapping=None, binding=None):
        runtime = Runtime(calibration, Safety, lambda:ClockBackend(lambda:self.sim, q,
            source_clock_id='cpu-fake-device-clock-v1' if environment=='fake' else None),
            device_id=device, environment=environment, lock_root=self.directory.name,
            initialize_backend=(lambda b:b.set_damping_hold(manual=True)) if initialize else None,
            control_clock=lambda:self.sim,jaw_mapping_file=mapping,hardware_binding=binding,data_plane=getattr(self,'ros_plane',None))
        credentials = {business:runtime.register_business(business) for business in ('calibration','collection')}
        self.runtimes.append((runtime, credentials))
        self.advance(runtime, 42)
        return runtime, credentials

    def advance(self, runtime, count=1):
        for _ in range(count):
            self.sim += .01
            runtime.tick()
            runner = runtime.owner.runner
            if runner and getattr(runner, 'runtime_started', False):
                self.assertTrue(runner.runtime_ready.wait(1.))

    def preview(self, runtime, credential):
        q = runtime.owner.cache['calibration_feedback']['qpos'][:]; q[0] += .01
        return runtime.submit(credential, dict(action='preview', active_arm='left', q_target=q))

    def confirm(self, runtime, credential, review):
        return runtime.submit(credential, dict(action='confirm', token=review['token'],
            confirmation='START_REVIEWED_SINGLE_ARM', physical_clearance=True))

    def test_actual_calibration_then_collection_and_back_preserve_position_hold(self):
        from test_collection_physx_protocol import protocol_fixture
        _,source,plan,_,mapping=protocol_fixture(Path(self.directory.name))
        first = plan['ticks'][0]['target']
        q = [*first['left_arm_joints'], jaw.feedback_sdk('left', first['left_gripper']),
             *first['right_arm_joints'], jaw.feedback_sdk('right', first['right_gripper'])]
        jaw.configure_mapping(mapping)
        q[6],q[13]=jaw.feedback_sdk('left',first['left_gripper']),jaw.feedback_sdk('right',first['right_gripper'])
        runtime, creds = self.create(q=q,mapping=mapping,binding=Path(self.directory.name)/'binding.json')
        self.confirm(runtime, creds['calibration'], self.preview(runtime, creds['calibration']))
        self.advance(runtime, 200)
        self.assertEqual(runtime.owner.runner.status, 'READY')
        self.assertTrue(runtime.snapshot(creds['calibration'])['position_hold_active'])
        old_runner, events = runtime.owner.runner, runtime.owner.backend.events
        before = len(events)
        runtime.handoff(creds['calibration'], 'collection')
        self.assertIsNone(runtime.owner.runner)
        with patch.object(old_runner, 'tick', side_effect=AssertionError('old motion policy resumed')):
            self.advance(runtime, 50)
        self.assertEqual(len(events), before, 'handoff sent targets, damping or release')
        self.assertEqual(runtime.snapshot(creds['collection'])['hold']['left'], 'position')
        self.assertFalse(runtime.snapshot(creds['collection'])['position_hold_active'])
        review = runtime.submit(creds['collection'], dict(action='preview_frozen', source=source, plan=plan))
        self.confirm(runtime, creds['collection'], review)
        for _ in range(2000):
            self.advance(runtime)
            if runtime.owner.runner.status == 'READY':
                break
        self.assertEqual(runtime.owner.runner.status, 'READY', runtime.owner.runner.reason)
        self.assertTrue(hasattr(runtime.owner.runner, 'frozen'))
        self.assertEqual(runtime.owner.runner.frozen.status, 'completed')
        state=runtime.snapshot(creds['collection'])
        self.assertTrue(state['position_hold_active']);self.assertGreater(state['maximum_control_gap_s'],0)
        self.assertEqual(state['physical_feedback']['left_arm_joints'],state['calibration_feedback']['qpos'][:6])
        self.assertEqual(state['execution_timeline']['timeline_digest'],runtime.owner.runner.runtime['timeline_digest'])
        from jaw_mapping_v2 import MAPPING_ID
        self.assertEqual(state['collection_jaw_mapping'],MAPPING_ID)
        self.advance(runtime, 40)
        before = len(events)
        runtime.handoff(creds['collection'], 'calibration')
        self.advance(runtime, 40)
        self.assertEqual(len(events), before)
        self.assertEqual(runtime.snapshot(creds['calibration'])['hold'], dict(left='position', right='position'))

    def test_same_trajectory_reapproval_has_a_new_execution_instance(self):
        runtime,creds=self.create()
        q=runtime.owner.cache['calibration_feedback']['qpos'][:]
        ids=[]
        for _ in range(2):
            review=runtime.submit(creds['calibration'],dict(action='preview',active_arm='left',q_target=q))
            self.confirm(runtime,creds['calibration'],review)
            self.advance(runtime,150)
            self.assertEqual(runtime.owner.runner.status,'READY')
            bound=runtime.snapshot(creds['calibration'])['execution_calibration']
            ids.append(bound['execution_instance_id'])
            self.assertEqual(runtime.owner.execution_instance_id,ids[-1])
            self.assertEqual(runtime.snapshot(creds['calibration'])['execution_instance_id'],ids[-1])
        self.assertNotEqual(ids[0],ids[1])
        self.assertEqual(runtime.owner.epoch,0)

    def test_native_task_status_is_a_small_independent_snapshot(self):
        runtime,creds=self.create()
        runtime.owner.native_task_status=dict(execution_timeline_digest='current',source_clock_id='plant',
            task_outcome='unverifiable',task_qualified=False,receipt_path=None,receipt_sha256=None,
            large_observations=['not part of lightweight status'])
        state=runtime.snapshot(creds['calibration'])
        self.assertFalse(state['native_task']['task_qualified'])
        self.assertNotIn('large_observations',state['native_task'])
        state['native_task']['task_outcome']='changed'
        self.assertEqual(runtime.owner.native_task_status['task_outcome'],'unverifiable')
        self.assertEqual(runtime.owner.control.state,runtime.owner.states.READY)

    def test_shared_capabilities_survive_collection_cache_publication_window(self):
        from robot_workbench.bridges import calibration_gateway
        runtime,creds=self.create()
        device=SimpleNamespace(runtime=runtime,snapshot=lambda business:runtime.snapshot(creds[business]))
        gateway=calibration_gateway(device)
        entered,release=threading.Event(),threading.Event()
        original=runtime.collection._cache_state
        def pause(owner,*args):
            result=original(owner,*args)
            entered.set();self.assertTrue(release.wait(2.))
            return result
        with patch.object(runtime.collection,'_cache_state',pause):
            worker=threading.Thread(target=lambda:self.advance(runtime));worker.start()
            try:
                self.assertTrue(entered.wait(1.))
                self.assertEqual(runtime.owner.cache['capabilities'],['collection_execution_v2'])
                status=gateway.status()
                self.assertTrue(status['motion_enabled'])
                for state in (status['owner'],runtime.owner.snapshot()):
                    self.assertIn('collection_execution_v2',state['capabilities'])
                    self.assertIn('workbench_trajectory_v1',state['capabilities'])
                    self.assertIn('calibration_capture_ack_v1',state['capabilities'])
            finally:release.set();worker.join(2.)
        self.assertFalse(worker.is_alive())

    def test_identity_lock_namespace_and_stale_runtime_credential(self):
        runtime, creds = self.create()
        factory = lambda: (_ for _ in ()).throw(AssertionError('backend constructed without lock'))
        with self.assertRaises(BlockingIOError):
            Runtime(calibration, Safety, factory, device_id='robot', environment='fake', lock_root=self.directory.name)
        other, other_creds = self.create(environment='physical')
        with self.assertRaises(PermissionError):
            other.submit(creds['calibration'], dict(action='stop'))
        with self.assertRaises(PermissionError):
            self.preview(runtime, creds['collection'])
        state = runtime.snapshot(creds['collection'])
        self.assertFalse(state['allowed_operations']['preview']['allowed'])
        self.assertIn('交接', state['allowed_operations']['preview']['reason'])
        self.assertTrue(state['allowed_operations']['stop']['allowed'])
        runtime.submit(creds['calibration'], dict(action='release', confirmation='ARMS_SUPPORTED_RELEASE'))
        runtime.tick(); runtime.close()
        replacement, _ = self.create()
        with self.assertRaises(PermissionError):
            replacement.submit(creds['calibration'], dict(action='stop'))

    def test_handoff_rejects_unknown_hold_stale_feedback_active_motion_and_writer(self):
        runtime, creds = self.create(initialize=False)
        with self.assertRaisesRegex(ValueError, 'holding-mode return'):
            runtime.handoff(creds['calibration'], 'collection')
        runtime.owner.backend.set_damping_hold(manual=True)
        runtime.owner.feedback_healthy = False
        with self.assertRaisesRegex(ValueError, 'fresh healthy'):
            runtime.handoff(creds['calibration'], 'collection')
        self.advance(runtime)
        self.sim += .101
        with self.assertRaisesRegex(ValueError, 'fresh healthy'):
            runtime.handoff(creds['calibration'], 'collection')
        self.advance(runtime, 42)
        done = threading.Event()
        runtime.owner.recording = SimpleNamespace(done=done, error='disk full', thread=SimpleNamespace(is_alive=lambda:True))
        with self.assertRaisesRegex(ValueError, 'writer has not exited'):
            runtime.handoff(creds['calibration'], 'collection')
        done.set()
        with self.assertRaisesRegex(ValueError, 'writer has not exited'):
            runtime.handoff(creds['calibration'], 'collection')
        runtime.owner.recording.thread.is_alive = lambda:False
        runtime.handoff(creds['calibration'], 'collection')
        runtime.handoff(creds['collection'], 'calibration')
        runtime.owner.recording = None
        self.confirm(runtime, creds['calibration'], self.preview(runtime, creds['calibration']))
        with self.assertRaisesRegex(ValueError, 'pending'):
            runtime.handoff(creds['calibration'], 'collection')
        self.advance(runtime)
        with self.assertRaisesRegex(ValueError, 'active'):
            runtime.handoff(creds['calibration'], 'collection')

    def test_stop_does_not_wait_for_calibration_compilation_or_restore_late_token(self):
        runtime, creds = self.create()
        entered, resume = threading.Event(), threading.Event()
        original, result = calibration.make_plan, []
        def slow(*args, **kwargs):
            entered.set(); self.assertTrue(resume.wait(2.)); return original(*args, **kwargs)
        def preview():
            try:result.append(self.preview(runtime, creds['calibration']))
            except Exception as error:result.append(error)
        with patch.object(calibration, 'make_plan', slow):
            worker = threading.Thread(target=preview); worker.start()
            self.assertTrue(entered.wait(1.))
            before = time.monotonic()
            receipt = runtime.submit(creds['collection'], dict(action='stop'))
            self.assertLess(time.monotonic()-before, .1)
            self.assertEqual(receipt['queued'], 'stop')
            self.assertFalse(receipt['stop']['stop_hold_return_confirmed'])
            resume.set(); worker.join(2.)
        self.assertFalse(worker.is_alive())
        self.assertIsInstance(result[0], ValueError)
        self.assertFalse(runtime.owner.pending)
        self.advance(runtime)
        self.assertFalse(runtime.owner.pending)
        self.assertTrue(runtime.snapshot(creds['calibration'])['stop_receipt']['stop_hold_return_confirmed'])

    def test_blocked_sdk_remains_unconfirmed_and_cannot_handoff(self):
        runtime, creds = self.create()
        self.confirm(runtime, creds['calibration'], self.preview(runtime, creds['calibration']))
        backend = runtime.owner.backend._backend
        entered, resume = threading.Event(), threading.Event()
        original = backend.send
        def blocked(*args):
            entered.set(); self.assertTrue(resume.wait(2.)); return original(*args)
        def run_until_send():
            for _ in range(60):
                self.advance(runtime)
                if entered.is_set():break
        with patch.object(backend, 'send', blocked):
            worker = threading.Thread(target=run_until_send); worker.start()
            self.assertTrue(entered.wait(1.))
            started = time.monotonic()
            receipt = runtime.submit(creds['collection'], dict(action='stop'))
            self.assertLess(time.monotonic()-started, .1)
            self.assertFalse(receipt['stop']['prior_calls_complete'])
            self.assertFalse(receipt['stop']['stop_hold_return_confirmed'])
            self.assertEqual(receipt['stop']['in_flight_calls'][0]['operation'], 'send')
            with self.assertRaisesRegex(ValueError, 'in flight'):
                runtime.handoff(creds['calibration'], 'collection')
            self.assertTrue(runtime.snapshot(creds['calibration'])['sdk_calls_in_flight'])
            resume.set(); worker.join(2.)
        self.assertFalse(worker.is_alive())
        self.advance(runtime)
        before = len([e for e in backend.events if e[0]=='send'])
        self.advance(runtime, 40)
        self.assertEqual(len([e for e in backend.events if e[0]=='send']), before)
        self.assertEqual(runtime.snapshot(creds['calibration'])['hold'], dict(left='damping', right='damping'))

    def test_collection_preview_cannot_restore_token_after_handoff(self):
        from test_collection_physx_protocol import protocol_fixture
        _,source,plan,_,mapping=protocol_fixture(Path(self.directory.name))
        runtime, creds = self.create(mapping=mapping,binding=Path(self.directory.name)/'binding.json')
        runtime.owner.source_validator(plan,source)
        runtime.handoff(creds['calibration'], 'collection')
        entered, resume = threading.Event(), threading.Event()
        original, result = runtime.collection.compile_frozen, []
        def slow(*args):
            entered.set(); self.assertTrue(resume.wait(2.)); return original(*args)
        def preview():
            try:result.append(runtime.submit(creds['collection'], dict(action='preview_frozen', source=source, plan=plan)))
            except Exception as error:result.append(error)
        with patch.object(runtime.collection, 'compile_frozen', slow):
            worker = threading.Thread(target=preview); worker.start()
            self.assertTrue(entered.wait(1.))
            runtime.handoff(creds['collection'], 'calibration')
            resume.set(); worker.join(2.)
        self.assertFalse(worker.is_alive())
        self.assertIsInstance(result[0], ValueError)
        self.assertIn('changed during preview',str(result[0]))
        self.assertFalse(runtime.owner.pending)
        self.assertFalse(runtime.owner.frozen_payloads)

    def collection_case(self, name):
        from test_collection_physx_protocol import protocol_fixture
        directory = Path(self.directory.name)/name
        directory.mkdir()
        _, source, plan, _, mapping = protocol_fixture(directory)
        jaw.configure_mapping(mapping)
        first = plan['ticks'][0]['target']
        q = [*first['left_arm_joints'], jaw.feedback_sdk('left', first['left_gripper']),
             *first['right_arm_joints'], jaw.feedback_sdk('right', first['right_gripper'])]
        runtime, creds = self.create(device=name, q=q, mapping=mapping, binding=directory/'binding.json')
        runtime.handoff(creds['calibration'], 'collection')
        return runtime, creds, dict(action='preview_frozen', source=source, plan=plan)

    def test_collection_compilation_uses_live_guard_and_publishes_only_after_final_recheck(self):
        for case in ('source','wall','stale','stop','release','epoch','limits','pedal','revision'):
            with self.subTest(case=case):
                runtime, creds, command = self.collection_case('live-guard-'+case)
                if case == 'wall':
                    runtime.owner.control_clock = None
                    runtime.owner.backend.clock = time.monotonic
                    for _ in range(42):
                        runtime.tick(); time.sleep(.01)
                entered, resume, compiled, publish = (threading.Event() for _ in range(4))
                original, submit, result = runtime.collection.compile_frozen, runtime.collection.submit, []
                events = list(runtime.owner.backend.events)
                stamp = runtime.owner.cache['calibration_feedback']['sample_time_s']
                def slow(*args):
                    entered.set(); self.assertTrue(resume.wait(3.)); return original(*args)
                def hold_compiled(compiler, payload):
                    review = submit(compiler, payload)
                    if payload['action'] != 'preview_frozen':return review
                    self.assertTrue(compiler.pending)
                    compiled.set(); self.assertTrue(publish.wait(3.))
                    return review
                def preview():
                    try:result.append(runtime.submit(creds['collection'], command))
                    except Exception as error:result.append(error)
                with patch.object(runtime.collection, 'compile_frozen', slow), \
                     patch.object(runtime.collection, 'submit', hold_compiled):
                    worker = threading.Thread(target=preview); worker.start()
                    try:
                        self.assertTrue(entered.wait(1.))
                        for _ in range(20):
                            if case != 'wall':self.sim += .01
                            runtime.tick(); time.sleep(.01)
                        if case == 'wall':runtime.tick()
                        live = runtime.owner.cache['calibration_feedback']['sample_time_s']
                        self.assertGreater(runtime.owner.control_time()-stamp, .1)
                        self.assertLess(runtime.owner.control_time()-live, .1)
                        self.assertIsNone(runtime._idle_reason())
                        self.assertFalse(runtime.owner.pending)
                        self.assertFalse(runtime.owner.frozen_payloads)
                        if case == 'stale':self.sim += .101
                        elif case in ('stop','release'):
                            runtime.submit(creds['collection'], dict(action=case, **(
                                {'confirmation':'ARMS_SUPPORTED_RELEASE'} if case=='release' else {})))
                        elif case == 'epoch':runtime.owner.epoch += 1
                        elif case == 'limits':runtime.owner.cache['joint_limits'][0][0] += .01
                        elif case == 'pedal':runtime.owner.pedal_snapshot = lambda:(1, False, True)
                        elif case == 'revision':runtime.preview_revision += 1
                        resume.set()
                        if case in ('source','wall','pedal'):
                            deadline=time.monotonic()+3
                            while not compiled.wait(.005) and time.monotonic()<deadline:
                                if case=='wall':runtime.tick()
                            self.assertTrue(compiled.is_set())
                            live=runtime.owner.cache['calibration_feedback']['sample_time_s']
                            self.assertFalse(runtime.owner.pending, 'compiler published before the final runtime gate')
                            self.assertFalse(runtime.owner.frozen_payloads)
                    finally:
                        resume.set(); publish.set(); worker.join(3.)
                self.assertFalse(worker.is_alive())
                if case in ('source','wall'):
                    self.assertIn(result[0]['token'], runtime.owner.pending)
                    self.assertEqual(runtime.owner.cache['calibration_feedback']['sample_time_s'], live)
                else:
                    self.assertIsInstance(result[0], ValueError)
                    self.assertFalse(runtime.owner.pending)
                    self.assertFalse(runtime.owner.frozen_payloads)
                    if case == 'stale':self.assertIn('feedback cache expired', str(result[0]))
                    if case in ('pedal','revision'):self.assertIn('revoked', str(result[0]))
                self.assertEqual(runtime.owner.backend.events, events, 'preview compilation touched the backend')

    def test_preview_tick_boundary_keeps_normal_feedback_and_stop_independent(self):
        from test_l1_fault_matrix import Harness
        for phase, blocked in (('final',False),('initial',True),('final',True)):
            with self.subTest(phase=phase, blocked=blocked):
                h = Harness()
                armed, entered, release = (threading.Event() for _ in range(3))
                try:
                    runtime = h.runtime
                    runtime.handoff(h.credentials['calibration'], 'collection')
                    original, result, waits = runtime.collection.submit, [], []
                    wait_for = runtime.tick_idle.wait_for
                    def read_delay():
                        if armed.is_set():
                            armed.clear(); entered.set()
                            if blocked:self.assertTrue(release.wait(2.))
                            else:time.sleep(.008)
                    def compiled(compiler, command):
                        review = original(compiler, command)
                        if phase == 'final' and command['action'] == 'preview_frozen':
                            armed.set(); self.assertTrue(entered.wait(1.))
                        return review
                    def wait(predicate, timeout):
                        waits.append(not predicate())
                        return wait_for(predicate, timeout)
                    def preview():
                        try:result.append(h.preview('collection'))
                        except Exception as error:result.append(error)
                    h.sdk.read_delay = read_delay
                    with patch.object(runtime.collection, 'submit', compiled), \
                         patch.object(runtime.tick_idle, 'wait_for', wait):
                        if phase == 'initial':
                            armed.set(); self.assertTrue(entered.wait(1.))
                        worker = threading.Thread(target=preview); worker.start()
                        try:
                            self.assertTrue(entered.wait(1.))
                            if blocked:
                                started = time.monotonic()
                                receipt = runtime.submit(h.credentials['collection'], dict(action='stop'))
                                self.assertLess(time.monotonic()-started, .1)
                                self.assertFalse(receipt['stop']['prior_calls_complete'])
                                with self.assertRaisesRegex(ValueError, 'in flight'):
                                    runtime.handoff(h.credentials['collection'], 'calibration')
                            worker.join(.5)
                            self.assertFalse(worker.is_alive(), 'preview boundary wait is unbounded')
                            self.assertIn(True, waits, 'normal running feedback was not overlapped')
                            if blocked:
                                self.assertIsInstance(result[0], ValueError)
                                self.assertIn('preview boundary wait', str(result[0]))
                                self.assertFalse(runtime.owner.pending)
                                self.assertFalse(runtime.owner.frozen_payloads)
                            else:self.assertIn(result[0]['token'], runtime.owner.pending)
                            self.assertTrue(h.allow_tick.is_set(), 'normal feedback clock was paused')
                            self.assertFalse(h.sdk.trace, 'preview submitted motion')
                        finally:release.set(); worker.join(2.)
                finally:release.set(); h.close()

    def test_install_revokes_unconfirmed_preview_without_changing_epoch_or_hold(self):
        runtime, creds = self.create()
        runtime.notify_calibration_installed('fixture-installed-v1')
        review = self.preview(runtime, creds['calibration'])
        epoch, hold, events = runtime.owner.epoch, deepcopy(runtime.hold), list(runtime.owner.backend.events)
        receipt = runtime.notify_calibration_installed('fixture-installed-v2')
        self.assertTrue(receipt['revoked'])
        self.assertFalse(runtime.owner.pending)
        with self.assertRaisesRegex(ValueError, 'unknown or consumed'):
            self.confirm(runtime, creds['calibration'], review)
        self.assertEqual(runtime.owner.epoch, epoch)
        self.assertEqual(runtime.hold, hold)
        self.assertEqual(runtime.owner.backend.events, events)
        state = runtime.snapshot(creds['calibration'])
        self.assertEqual(state['calibration_snapshot_id'], 'fixture-installed-v2')
        self.assertIsNone(state['execution_calibration'])
        fresh = self.preview(runtime, creds['calibration'])
        self.assertFalse(runtime.notify_calibration_installed('fixture-installed-v2')['revoked'])
        self.assertIn(fresh['token'], runtime.owner.pending, 'same installed identity is idempotent')

    def test_install_fences_queued_confirm_for_both_motion_strategies(self):
        for business in ('calibration','collection'):
            with self.subTest(business=business):
                if business == 'collection':
                    runtime, creds, command = self.collection_case('queued-collection')
                else:
                    runtime, creds = self.create(device='queued-calibration')
                runtime.notify_calibration_installed('fixture-installed-v1')
                review = (runtime.submit(creds[business], command) if business == 'collection'
                          else self.preview(runtime, creds[business]))
                self.confirm(runtime, creds[business], review)
                self.assertFalse(runtime.owner.commands.empty())
                epoch, events = runtime.owner.epoch, list(runtime.owner.backend.events)
                runtime.notify_calibration_installed('fixture-installed-v2')
                self.advance(runtime, 50)
                self.assertIsNone(runtime.owner.runner)
                self.assertIsNone(runtime.execution_calibration)
                self.assertEqual(runtime.owner.epoch, epoch)
                self.assertEqual(runtime.owner.backend.events, events, 'queued old plan reached the SDK')

    def test_install_preserves_running_calibration_sequence(self):
        runtime, creds = self.create()
        runtime.notify_calibration_installed('fixture-installed-v1')
        review = runtime.submit(creds['calibration'], dict(action='preview_workbench', waypoints=[
            dict(q=[q]+[0.]*11, speeds=[.1]*12) for q in (.01,.02)]))
        self.confirm(runtime, creds['calibration'], review)
        self.advance(runtime, 45)
        self.assertEqual(runtime.owner.runner.status, 'RUNNING')
        first, remaining = runtime.owner.runner.plan, tuple(runtime.owner.sequence_remaining)
        epoch, holding = runtime.owner.epoch, deepcopy(runtime.hold)
        accepted = deepcopy(runtime.execution_calibration)
        runtime.notify_calibration_installed('fixture-installed-v2')
        self.assertIs(runtime.owner.runner.plan, first)
        self.assertEqual(tuple(runtime.owner.sequence_remaining), remaining)
        self.assertEqual(runtime.owner.epoch, epoch)
        self.assertEqual(runtime.hold, holding)
        self.advance(runtime, 500)
        self.assertEqual(runtime.owner.runner.status, 'READY', runtime.owner.runner.reason)
        self.assertEqual(runtime.owner.runner.plan.sha256, review['plans'][-1]['sha256'])
        self.assertAlmostEqual(runtime.owner.backend.q[0], .02)
        self.assertEqual(runtime.execution_calibration, accepted)
        self.assertEqual(accepted['snapshot_id'], 'fixture-installed-v1')

    def test_install_after_owner_checks_but_before_acceptance_rejects_execution(self):
        runtime, creds = self.create()
        runtime.notify_calibration_installed('fixture-installed-v1')
        self.confirm(runtime, creds['calibration'], self.preview(runtime, creds['calibration']))
        entered, resume = threading.Event(), threading.Event()
        original = runtime.owner.accept_review
        def pause(plans):
            entered.set(); self.assertTrue(resume.wait(2.)); return original(plans)
        events = list(runtime.owner.backend.events)
        with patch.object(runtime.owner, 'accept_review', pause):
            worker = threading.Thread(target=lambda:self.advance(runtime)); worker.start()
            self.assertTrue(entered.wait(1.))
            runtime.notify_calibration_installed('fixture-installed-v2')
            resume.set(); worker.join(2.)
        self.assertFalse(worker.is_alive())
        self.assertIsNone(runtime.owner.runner)
        self.assertIsNone(runtime.execution_calibration)
        self.assertEqual(runtime.owner.backend.events, events)

    def test_install_between_acceptance_and_collection_runner_keeps_frozen_payload(self):
        runtime, creds, command = self.collection_case('accepted-collection')
        runtime.notify_calibration_installed('fixture-installed-v1')
        review = runtime.submit(creds['collection'], command)
        self.confirm(runtime, creds['collection'], review)
        entered, resume = threading.Event(), threading.Event()
        original = runtime.owner.start_motion
        def pause(*args, **kwargs):
            entered.set(); self.assertTrue(resume.wait(2.)); return original(*args, **kwargs)
        with patch.object(runtime.owner, 'start_motion', pause):
            worker = threading.Thread(target=lambda:self.advance(runtime)); worker.start()
            self.assertTrue(entered.wait(1.))
            accepted = deepcopy(runtime.execution_calibration)
            epoch = runtime.owner.epoch
            self.assertEqual(accepted['snapshot_id'], 'fixture-installed-v1')
            before = time.monotonic()
            runtime.notify_calibration_installed('fixture-installed-v2')
            self.assertLess(time.monotonic()-before, .1, 'install waited for the owner tick')
            resume.set(); worker.join(2.)
        self.assertFalse(worker.is_alive())
        runner = runtime.owner.runner
        self.assertTrue(hasattr(runner, 'frozen'), 'accepted collection degraded to a calibration runner')
        payload = deepcopy(runner.payload)
        from robot_workbench.control_plan import load_artifact
        self.assertEqual(load_artifact(payload['plan_ref']['source']),command['source'])
        self.assertEqual(load_artifact(payload['plan_ref']['plan']),command['plan'])
        for _ in range(2000):
            self.advance(runtime)
            if runner.status == 'READY':break
        self.assertEqual(runner.status, 'READY', runner.reason)
        self.assertEqual(runner.frozen.status, 'completed')
        self.assertEqual(runner.payload, payload)
        self.assertEqual(runtime.owner.epoch, epoch)
        self.assertEqual(runtime.execution_calibration, accepted)

    def test_install_rejects_late_preview_without_waiting_for_compilation(self):
        runtime, creds = self.create()
        runtime.notify_calibration_installed('fixture-installed-v1')
        entered, resume = threading.Event(), threading.Event()
        original, result = calibration.make_plan, []
        def slow(*args, **kwargs):
            entered.set(); self.assertTrue(resume.wait(2.)); return original(*args, **kwargs)
        def preview():
            try:result.append(self.preview(runtime, creds['calibration']))
            except Exception as error:result.append(error)
        with patch.object(calibration, 'make_plan', slow):
            worker = threading.Thread(target=preview); worker.start()
            self.assertTrue(entered.wait(1.))
            epoch = runtime.owner.epoch
            before = time.monotonic()
            runtime.notify_calibration_installed('fixture-installed-v2')
            self.assertLess(time.monotonic()-before, .1)
            resume.set(); worker.join(2.)
        self.assertFalse(worker.is_alive())
        self.assertIsInstance(result[0], ValueError)
        self.assertIn('revoked', str(result[0]))
        self.assertFalse(runtime.owner.pending)
        self.assertFalse(runtime.reviewed_plans)
        self.assertEqual(runtime.owner.epoch, epoch)

    def test_install_does_not_wait_for_sdk_and_stop_stays_independent(self):
        runtime, creds = self.create()
        runtime.notify_calibration_installed('fixture-installed-v1')
        self.confirm(runtime, creds['calibration'], self.preview(runtime, creds['calibration']))
        backend = runtime.owner.backend._backend
        entered, resume = threading.Event(), threading.Event()
        original = backend.send
        def blocked(*args):
            entered.set(); self.assertTrue(resume.wait(2.)); return original(*args)
        def run_until_send():
            for _ in range(60):
                self.advance(runtime)
                if entered.is_set():break
        with patch.object(backend, 'send', blocked):
            worker = threading.Thread(target=run_until_send); worker.start()
            self.assertTrue(entered.wait(1.))
            epoch = runtime.owner.epoch
            before = time.monotonic()
            runtime.notify_calibration_installed('fixture-installed-v2')
            self.assertLess(time.monotonic()-before, .1)
            self.assertEqual(runtime.owner.epoch, epoch)
            receipt = runtime.submit(creds['collection'], dict(action='stop'))
            self.assertFalse(receipt['stop']['stop_hold_return_confirmed'])
            self.assertEqual(receipt['stop']['in_flight_calls'][0]['operation'], 'send')
            resume.set(); worker.join(2.)
        self.assertFalse(worker.is_alive())
        self.advance(runtime)
        self.assertTrue(runtime.snapshot(creds['calibration'])['stop_receipt']['stop_hold_return_confirmed'])
        self.assertEqual(runtime.hold, dict(left='damping', right='damping'))

    def test_one_hundred_fixed_request_tick_handoff_schedules(self):
        # Replay the first 100 lexicographic orderings; two blocking-thread tests
        # above separately prove compilation and native IO concurrency behavior.
        for index, schedule in enumerate(islice(permutations('P C S T H'.split()), 100)):
            with self.subTest(schedule=''.join(schedule)):
                runtime, creds = self.create(device='schedule-'+str(index))
                review = {'token':'never-generated'}
                revoked = False
                for operation in schedule:
                    try:
                        if operation == 'P':review = self.preview(runtime, creds['calibration'])
                        elif operation == 'C':self.confirm(runtime, creds['calibration'], review)
                        elif operation == 'S':
                            runtime.submit(creds['collection'], dict(action='stop')); revoked = True
                        elif operation == 'T':self.advance(runtime)
                        else:
                            runtime.handoff(creds['calibration'], 'collection'); revoked = True
                    except (ValueError, PermissionError):
                        pass
                before = len([e for e in runtime.owner.backend.events if e[0]=='send'])
                self.advance(runtime, 4)
                if revoked:
                    self.assertEqual(len([e for e in runtime.owner.backend.events if e[0]=='send']), before)
                self.assertFalse(runtime.owner.pending if runtime.holder == 'collection' else {})


if __name__ == '__main__':
    unittest.main()
