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
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT/'tests'), str(ROOT/'calibration'),
               str(ROOT/'calibration/infra'), str(ROOT/'calibration/tests'), str(ROOT/'rule_based_workbench')]
from robot_workbench.runtime import Runtime
from test_owner_clock import ClockBackend, Safety, calibration, jaw
from test_frozen_executor import fixture


class UnifiedRuntimeTest(unittest.TestCase):
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
                    runtime.submit(credentials[runtime.holder], dict(action='release', confirmation='ARMS_SUPPORTED_RELEASE'))
                    runtime.tick()
                runtime.close()

    def create(self, *, device='robot', environment='fake', initialize=True, q=None):
        runtime = Runtime(calibration, Safety, lambda:ClockBackend(lambda:self.sim, q),
            device_id=device, environment=environment, lock_root=self.directory.name,
            initialize_backend=(lambda b:b.set_damping_hold(manual=True)) if initialize else None,
            control_clock=lambda:self.sim)
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
        source, plan = fixture()
        plan['stages'] = [dict(name='return_home', start_tick=0, end_tick=3)]
        first = plan['ticks'][0]['target']
        q = [*first['left_arm_joints'], jaw.feedback_sdk('left', first['left_gripper']),
             *first['right_arm_joints'], jaw.feedback_sdk('right', first['right_gripper'])]
        runtime, creds = self.create(q=q)
        self.confirm(runtime, creds['calibration'], self.preview(runtime, creds['calibration']))
        self.advance(runtime, 200)
        self.assertEqual(runtime.owner.runner.status, 'READY')
        old_runner, events = runtime.owner.runner, runtime.owner.backend.events
        before = len(events)
        runtime.handoff(creds['calibration'], 'collection')
        self.assertIsNone(runtime.owner.runner)
        with patch.object(old_runner, 'tick', side_effect=AssertionError('old motion policy resumed')):
            self.advance(runtime, 50)
        self.assertEqual(len(events), before, 'handoff sent targets, damping or release')
        self.assertEqual(runtime.snapshot(creds['collection'])['hold']['left'], 'position')
        review = runtime.submit(creds['collection'], dict(action='preview_frozen', source=source, plan=plan))
        self.confirm(runtime, creds['collection'], review)
        for _ in range(2000):
            self.advance(runtime)
            if runtime.owner.runner.status == 'READY':
                break
        self.assertEqual(runtime.owner.runner.status, 'READY', runtime.owner.runner.reason)
        self.assertTrue(hasattr(runtime.owner.runner, 'frozen'))
        self.assertEqual(runtime.owner.runner.frozen.status, 'completed')
        self.advance(runtime, 40)
        before = len(events)
        runtime.handoff(creds['collection'], 'calibration')
        self.advance(runtime, 40)
        self.assertEqual(len(events), before)
        self.assertEqual(runtime.snapshot(creds['calibration'])['hold'], dict(left='position', right='position'))

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
        source, plan = fixture()
        runtime, creds = self.create()
        runtime.handoff(creds['calibration'], 'collection')
        entered, resume = threading.Event(), threading.Event()
        original, result = runtime.owner.source_validator, []
        def slow(*args):
            entered.set(); self.assertTrue(resume.wait(2.)); return original(*args)
        def preview():
            try:result.append(runtime.submit(creds['collection'], dict(action='preview_frozen', source=source, plan=plan)))
            except Exception as error:result.append(error)
        with patch.object(runtime.owner, 'source_validator', slow):
            worker = threading.Thread(target=preview); worker.start()
            self.assertTrue(entered.wait(1.))
            runtime.handoff(creds['collection'], 'calibration')
            resume.set(); worker.join(2.)
        self.assertFalse(worker.is_alive())
        self.assertIsInstance(result[0], ValueError)
        self.assertFalse(runtime.owner.pending)
        self.assertFalse(runtime.owner.frozen_payloads)

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
