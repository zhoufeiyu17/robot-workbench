"""Preview network waits and optional overlays must not block renderer health."""
import sys
import threading
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'examples/scripts/debug'))
from joint_head_workbench import Workbench
from examples.scripts.debug.joint_head_workbench_live import Session, RTXRenderer
from test_joint_head_workbench_live import FakeGeometry


class StopLoop(BaseException):
    """Deterministically finish one iteration of an otherwise permanent worker."""


class PreviewResponsivenessTests(unittest.TestCase):
    def setUp(self):
        a = self.app = Workbench.__new__(Workbench)
        a.live = True
        a.control_mode = 'hardware_sync'
        a.lock = threading.Lock()
        a.mode_lock = threading.Lock()
        a.remote = {'mode': 'simulation', 'q': [0.] * 12, 'timestamp_s': 1.,
                    'capabilities': {'jaw_gap_mm': True}}
        a.remote_received = time.monotonic()
        a.remote_error = None
        a.remote_advanced = time.monotonic()
        a.last_remote_timestamp = None
        a.jaw_mm = np.array([80., 80.])
        a.tokens = {'existing': {'sentinel': True}}
        a.calibration = {'id': 'head', 'parameters': {'id': 'head'}}
        a.restore_wrist_display = Mock()

    def test_health_poll_does_not_restore_display_or_wait_for_page_locks(self):
        a = self.app
        a.remote_request = Mock(return_value=a.remote.copy())
        a.restore_display_once = Mock(side_effect=AssertionError('must run independently'))
        # Fail immediately on lock use rather than deadlocking this regression test.
        a.lock = Mock()
        a.mode_lock = Mock()
        with patch('joint_head_workbench.time.sleep', side_effect=StopLoop):
            with self.assertRaises(StopLoop):
                a.monitor_remote()
        a.remote_request.assert_called_once_with('/state')
        a.restore_display_once.assert_not_called()
        a.restore_wrist_display.assert_not_called()
        self.assertIsNone(a.remote_error)
        self.assertEqual(a.require_remote()['timestamp_s'], 1.)
        self.assertIn('existing', a.tokens)

    def test_display_restore_failures_preserve_healthy_feedback_and_tokens(self):
        a = self.app
        for failed_stage in ('head', 'wrist'):
            with self.subTest(stage=failed_stage):
                a.remote_request = Mock(side_effect=ValueError('head restore failed')
                                        if failed_stage == 'head' else None)
                a.restore_wrist_display = Mock(side_effect=ValueError('wrist restore failed'))
                with patch('joint_head_workbench.time.sleep', side_effect=StopLoop):
                    with self.assertRaises(StopLoop):
                        a.monitor_display_restore()
                self.assertEqual(a.display_restore_error, failed_stage + ' restore failed')
                self.assertIsNone(a.remote_error)
                self.assertIs(a.require_remote(), a.remote)
                self.assertIn('existing', a.tokens)
        a.remote_request = Mock()
        a.restore_wrist_display = Mock()
        with patch('joint_head_workbench.time.sleep', side_effect=StopLoop):
            with self.assertRaises(StopLoop):
                a.monitor_display_restore()
        self.assertIsNone(a.display_restore_error)

    def test_display_only_sender_leaves_locks_free_and_cannot_mint_motion_token(self):
        a = self.app
        a.geometry = SimpleNamespace(_q=lambda q: np.asarray(q, float))
        a.velocity_limits = np.ones(12)
        a.hardware_sync_active = Mock(return_value=True)
        a.preview_revision = 0
        a.preview_pending = {'q': [.1] * 12, 'revision': 7}
        a.preview_condition = threading.Condition()
        a.action = Mock(side_effect=AssertionError('display must bypass action lock'))

        def submit(path, body):
            self.assertEqual(path, '/preview')
            for lock in (a.lock, a.mode_lock):
                acquired = lock.acquire(blocking=False)
                self.assertTrue(acquired, 'network call holds a page/mode lock')
                if acquired:
                    lock.release()
            # A mode switch during the request must not turn display into authority.
            a.control_mode = 'simulation'
            return {'revision': body['revision']}

        a.remote_request = Mock(side_effect=submit)
        with patch.object(a.preview_condition, 'wait_for', side_effect=[True, StopLoop]):
            a.preview = Mock(wraps=a.preview)
            with self.assertRaises(StopLoop):
                a.send_previews()
        a.preview.assert_called_once_with({'q': [.1] * 12, 'revision': 7}, display_only=True)
        a.action.assert_not_called()
        self.assertEqual(a.preview_result['client_revision'], 7)
        self.assertTrue(a.preview_result['display_only'])
        self.assertIsNone(a.preview_result['token'])
        self.assertEqual(a.tokens, {'existing': {'sentinel': True}})

    def test_late_worker_success_or_failure_cannot_replace_newer_completed_preview(self):
        for old_fails in (False, True):
            with self.subTest(old_fails=old_fails):
                a = self.app
                a.geometry = SimpleNamespace(_q=lambda q: np.asarray(q, float))
                a.preview_condition = threading.Condition()
                a.preview_completed_dispatch = -1
                started, release = threading.Event(), threading.Event()
                errors = []
                waits = threading.local()

                def wait_once(predicate):
                    if getattr(waits, 'used', False):
                        raise StopLoop
                    waits.used = True
                    self.assertTrue(predicate())
                    return True

                def preview(body, **kwargs):
                    if body['revision'] == 1:
                        started.set()
                        if not release.wait(2):
                            raise AssertionError('newer worker did not complete')
                        if old_fails:
                            raise ValueError('older request failed')
                    return {'token': None, 'revision': body['revision']}

                def worker():
                    try:
                        a.send_previews()
                    except StopLoop:
                        pass
                    except BaseException as exc:
                        errors.append(exc)

                a.preview = Mock(side_effect=preview)
                with patch.object(a.preview_condition, 'wait_for', side_effect=wait_once):
                    a.queue_preview({'q': [.1] * 12, 'revision': 1})
                    old = threading.Thread(target=worker)
                    old.start()
                    try:
                        self.assertTrue(started.wait(2))
                        a.queue_preview({'q': [.2] * 12, 'revision': 2})
                        new = threading.Thread(target=worker)
                        new.start()
                        new.join(2)
                        self.assertFalse(new.is_alive())
                        self.assertEqual(a.preview_result['client_revision'], 2)
                    finally:
                        release.set()
                        old.join(2)
                self.assertFalse(old.is_alive())
                self.assertEqual(errors, [])
                self.assertEqual(a.preview_result, {'client_revision': 2, 'token': None, 'revision': 2})

    def test_dragging_throttles_actual_frames_then_restores_rate_without_pose_pollution(self):
        with tempfile.TemporaryDirectory() as output:
            s = Session(FakeGeometry(), output, {'initial_q': [0.] * 12})
            render = Mock(side_effect=lambda q, gaps=None: (np.asarray(q).tobytes(), b'actual-head'))
            renderer = SimpleNamespace(render=render)
            clock = 'examples.scripts.debug.joint_head_workbench_live.time.monotonic'
            with patch(clock, return_value=10.):
                s.tick(renderer, .04)
            original = s.jpeg['actual']
            for rev, now in enumerate((10.04, 10.08, 10.12, 10.16), 1):
                with patch(clock, return_value=now):
                    s.command('/preview', {'q': [.1 * rev] * 12, 'revision': rev})
                    s.tick(renderer, .04)
                self.assertEqual(s.jpeg['actual'], original)
                np.testing.assert_array_equal(s.q, np.zeros(12))
                np.testing.assert_array_equal(s.actual_q, np.zeros(12))
                self.assertNotEqual(s.jpeg['preview'], original)
            self.assertEqual(s.frame_sequence, 2, 'four drag ticks should update actual only once')
            self.assertEqual(len(s.render_times['preview']), 4)
            for now in (10.50, 10.54, 10.58):
                with patch(clock, return_value=now):
                    s.tick(renderer, .04)
            self.assertEqual(s.frame_sequence, 5, 'after drag window, actual resumes every 40 ms tick')
            self.assertEqual(s.jpeg['actual'], original)
            np.testing.assert_array_equal(s.actual_q, np.zeros(12))

    def test_pose_updates_only_changed_arm_usd_transforms(self):
        r = RTXRenderer.__new__(RTXRenderer)
        r.simulation_joint_offsets = np.zeros(12)
        r.pose_cache = {}
        r.g = SimpleNamespace(hand=None, mounts={side: np.eye(4) for side in ('left', 'right')})
        r.wrist_geometry = Mock()
        r.wrist_geometry.with_gaps.return_value = r.wrist_geometry
        r.wrist_geometry.frames.return_value = {side: {'link6': np.eye(4)} for side in ('left', 'right')}
        r.Gf = SimpleNamespace(Matrix4d=Mock(return_value='matrix'))
        parts = {side: Mock() for side in ('left', 'right')}
        tags = {side: Mock() for side in ('left', 'right')}
        r.parts = [(side, 'link6', op) for side, op in parts.items()]
        r.tags = list(tags.items())
        r.wrist_contract = None
        q = np.zeros(12)
        r.pose(q, [80., 80.])
        for op in (*parts.values(), *tags.values()):
            op.Set.assert_called_once()
            op.reset_mock()
        q[6] = .1
        r.pose(q, [80., 80.])
        for group in (parts, tags):
            group['left'].Set.assert_not_called()
            group['right'].Set.assert_called_once()
            group['right'].reset_mock()
        r.pose(q, [80., 80.])
        for op in (*parts.values(), *tags.values()):
            op.Set.assert_not_called()
        self.assertEqual(r.wrist_geometry.frames.call_count, 2)

    def test_mesh_switch_gates_jobs_rejects_stale_results_and_preserves_raw_preview(self):
        with tempfile.TemporaryDirectory() as output:
            s = Session(FakeGeometry(), output, {})
            self.assertFalse(s.mesh_rendering_enabled)
            for bad in (None, 0, 1, 'true', [], {}):
                with self.subTest(invalid=bad), self.assertRaises(ValueError):
                    s.command('/overlay-rendering', {'enabled': bad})
            s.calibration = {'id': 'head'}
            s.wrist_contract = {'head_snapshot': {'snapshot_sha256': 'snapshot'}}
            s.wrist_calibrations = {side: {'id': side} for side in ('left', 'right')}
            renderer = SimpleNamespace(
                render=Mock(return_value=(b'actual', b'head')),
                last_camera_images={'left': b'left', 'right': b'right'},
                encode_head=lambda image: image,
                calibration_overlay=Mock(return_value=b'head-overlay'),
                wrist_calibration_overlay=Mock(return_value=b'wrist-overlay'))
            module = 'examples.scripts.debug.joint_head_workbench_live'
            with patch(module + '.threading.Thread') as worker, patch(module + '.time.monotonic', return_value=10.):
                s.command('/preview', {'q': [.1] * 12, 'revision': 1})
                s.tick(renderer, .04)
                worker.assert_not_called()
                self.assertIn('actual', s.jpeg)
                self.assertEqual(s.preview_revision, 1)
                s.command('/overlay-rendering', {'enabled': True})
                with patch(module + '.time.monotonic', return_value=11.):
                    s.tick(renderer, .04)
                self.assertEqual(worker.call_count, 2)
                targets = {call.kwargs['target'].__name__ for call in worker.call_args_list}
                self.assertEqual(targets, {'update_calibration', 'update_wrist_calibration'})
                old_jobs = [call.kwargs for call in worker.call_args_list]
                s.command('/overlay-rendering', {'enabled': False})
                for job in old_jobs:
                    job['target'](*job['args'])
                self.assertNotIn('calibration', s.jpeg)
                self.assertNotIn('left-calibration', s.jpeg)
                self.assertNotIn('right-calibration', s.jpeg)
                worker.reset_mock()
                with patch(module + '.time.monotonic', return_value=12.):
                    s.command('/preview', {'q': [.2] * 12, 'revision': 2})
                    s.tick(renderer, .04)
                worker.assert_not_called()
                self.assertEqual(s.preview_revision, 2)
                self.assertEqual(s.jpeg['actual'], b'actual')
                self.assertEqual(s.jpeg['head'], b'head')
                self.assertEqual(s.jpeg['left'], b'left')
                np.testing.assert_array_equal(s.actual_q, np.zeros(12))


if __name__ == '__main__':
    unittest.main()
