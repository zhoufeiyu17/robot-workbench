"""A delayed manual preflight must never delay stop or revive execution authority."""
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'examples/scripts/debug'))
from joint_head_workbench import PipelineStopped, Workbench


class EndMirror(BaseException):
    """End the production mirror worker after one iteration."""


class PreflightRaceTests(unittest.TestCase):
    def app(self):
        a = Workbench.__new__(Workbench)
        root = tempfile.TemporaryDirectory()
        self.addCleanup(root.cleanup)
        a.output = Path(root.name)
        a.config = {'simulation_only': True, 'model_family': 'ac_one'}
        a.live = True
        a.lock = threading.RLock()
        a.mode_lock = threading.RLock()
        a.review_lock = threading.RLock()
        a.review_generation = 0
        a.control_mode = 'hardware_sync'
        a.geometry_id = a.plan_geometry_id = 'test-geometry'
        a.geometry = SimpleNamespace(_q=lambda q: np.asarray(q, float),
                                     limits=np.tile([-3., 3.], (12, 1)), mount_chain={})
        a.geometry.with_gaps = lambda *args: a.geometry
        a.geometry.path_check = Mock(return_value={'ok': True})
        a.geometry.support_path_check = Mock(return_value={'ok': True})
        a.tokens = {}
        a.hardware_tokens = set()
        a.calibration_previews = {}
        a.stop = threading.Event()
        a.q = np.zeros(12)
        a.jaw_mm = np.array([80., 80.])
        a.velocity_limits = np.ones(12)
        a.status = 'READY'
        a.importing = False
        a.pending_capture = None
        a.queue = []
        a.plan = None
        a.workflow = {'kind': 'head', 'head_result_id': None}
        a.selected_batch = None
        a.wrist_selected = {}
        a.calibration = None
        a.preview_revision = 0
        a.preview_condition = threading.Condition()
        a.frames = []
        a.fit_result = a.evaluation = a.batch = None
        a.reason = a.import_error = a.remote_error = a.hardware_error = a.mirror_error = None
        a.remote = {'mode': 'simulation', 'status': 'READY',
                    'capabilities': {'jaw_gap_mm': True}}
        a.remote_received = a.hardware_received = time.monotonic()
        a.hardware_feedback = {'owner': {'pid': 123, 'calibration_feedback': {'qpos': [0.] * 14}},
                               'source_age_s': 0.}
        a.control_status = {'owner': {'pid': 123, 'command_epoch': 4}}
        a.control_status['motion_enabled'] = True
        a.hardware_poll_lock = threading.Lock();a.hardware_history_lock = threading.Lock()
        a.hardware_history = []
        a.hardware_progress = dict(source=(123,None),sdk=[0.,0.],advanced=[time.monotonic()]*2)
        def read_hardware():
            now=time.monotonic()
            owner=dict(a.control_status['owner'],feedback_healthy=True,
                calibration_feedback=dict(qpos=[0.]*14,sdk_timestamps=[now,now],sample_wall_ns=time.time_ns()))
            return dict(reachable=True,feedback=dict(valid=True,host_sample_age_s=0.),raw_owner=owner)
        a.hardware_reader = SimpleNamespace(read_state=Mock(side_effect=read_hardware))
        a.hardware_connected = lambda: True
        a.control_available = lambda: True
        a.hardware_camera = None
        a.wrist_cameras = {}
        a.actual = {'parts': [], 'camera': None}
        a.csrf = 'local-test-only'
        a.persist = Mock()
        a.calibration_state = a.arrival_state = a.counts = a.library_state = a.diagnostics = lambda: {}
        a.wrist_results = a.history = lambda: []
        a.marker_prior = lambda *args: None
        a.control_state = lambda: {}
        a.execution_blocker = lambda: {'code': 'ready'}
        a.remote_request = Mock(return_value={'status': 'READY', 'revision': 1})
        a._control_request = Mock(side_effect=lambda action, body=None:
            a.control_status if action == 'state' else
            {'token': 'late-owner-token', 'expires_in_s': 30,
             'plans': [{'q_start': [0.] * 14, 'q_target': [0.] * 14}]}
            if action.endswith('preview') else {'ok': True})
        return a

    def start(self, callback):
        result, errors, done = [], [], threading.Event()

        def run():
            try:
                result.append(callback())
            except EndMirror:
                pass
            except BaseException as exc:
                errors.append(exc)
            finally:
                done.set()

        thread = threading.Thread(target=run, daemon=True)
        thread.start()
        return SimpleNamespace(thread=thread, done=done, result=result, errors=errors)

    def finish(self, *jobs):
        for job in jobs:
            job.thread.join(2)
            self.assertFalse(job.thread.is_alive(), 'test left a blocked worker')

    def block(self, result):
        entered, release = threading.Event(), threading.Event()

        def wait(*args, **kwargs):
            entered.set()
            if not release.wait(5):
                raise AssertionError('test did not release blocked preflight')
            return result

        return entered, release, wait

    def assert_readers_advance(self, a):
        def mirror_once(path, body=None):
            if path == '/mirror':
                raise EndMirror
            return {'status': 'READY', 'revision': 1}
        a.remote_request.side_effect = mirror_once
        state, mirror = self.start(a.state), self.start(a.mirror_hardware)
        try:
            self.assertTrue(state.done.wait(.5), 'state waited for preflight')
            self.assertTrue(mirror.done.wait(.5), 'mirror waited for preflight')
            self.assertEqual(state.errors, [])
            self.assertEqual(mirror.errors, [])
            self.assertEqual(state.result[0]['status'], 'READY')
            self.assertIn('/mirror', [call.args[0] for call in a.remote_request.call_args_list])
        finally:
            # The caller releases the preflight gate on assertion failure, so do
            # not join blocked readers here; they will finish on that release.
            self.addCleanup(self.finish, state, mirror)

    def test_each_stop_route_bypasses_blocked_manual_preflight_and_revokes_late_token(self):
        for blocked in ('path', 'support', 'gateway'):
            for stop_path in ('/api/stop', '/api/control/stop', '/api/calibration-pipeline'):
                with self.subTest(blocked=blocked, stop_path=stop_path):
                    a = self.app()
                    preview = {'token': 'late-owner-token', 'expires_in_s': 30,
                               'plans': [{'q_start': [0.] * 14, 'q_target': [0.] * 14}]}
                    entered, release, wait = self.block(preview if blocked == 'gateway' else {'ok': True})
                    if blocked == 'path':
                        a.geometry.path_check = wait
                    elif blocked == 'support':
                        a.geometry.support_path_check = wait
                    else:
                        a._control_request.side_effect = lambda action, body=None: wait() if action.endswith('preview') else {'ok': True}
                    planning = self.start(lambda: a.action('/api/hardware-trajectory-preview',
                        {'waypoints': [{'q': [.1] * 12}]}))
                    stopped = None
                    try:
                        self.assertTrue(entered.wait(1), planning.errors)
                        self.assert_readers_advance(a)
                        stopped = self.start(lambda: a.action(stop_path,
                            {'stop': True} if stop_path.endswith('pipeline') else {}))
                        self.assertTrue(stopped.done.wait(.5), 'stop waited for slow preflight')
                        self.assertEqual(stopped.errors, [])
                        self.assertTrue(a.stop.is_set())
                        self.assertIn('stop', [call.args[0] for call in a._control_request.call_args_list])
                    finally:
                        release.set()
                        self.finish(planning, *([stopped] if stopped else []))
                    self.assertEqual(planning.result, [], 'cancelled preflight returned a usable result')
                    self.assertTrue(planning.errors and isinstance(planning.errors[0], ValueError), planning.errors)
                    self.assertFalse(a.hardware_tokens)
                    with self.assertRaises(ValueError):
                        a.action('/api/control/motion-confirm', {'token': 'late-owner-token'})
                    self.assertNotIn('motion-confirm', [call.args[0] for call in a._control_request.call_args_list])

    def test_queue_check_leaves_state_and_mirror_live_and_stop_prevents_append(self):
        a = self.app()
        entered, release, a.geometry.path_check = self.block({'ok': True})
        planning = self.start(lambda: a.action('/api/queue', {'q': [.1] * 12}))
        jobs = [planning]
        try:
            self.assertTrue(entered.wait(1), planning.errors)
            self.assert_readers_advance(a)
            stopped = self.start(lambda: a.action('/api/stop', {}))
            jobs.append(stopped)
            self.assertTrue(stopped.done.wait(.5), 'stop waited for queue geometry')
            self.assertEqual(stopped.errors, [])
        finally:
            release.set()
            self.finish(*jobs)
        self.assertEqual(a.queue, [], 'a cancelled queue request appended its late target')
        self.assertTrue(planning.errors and isinstance(planning.errors[0], ValueError), planning.errors)

    def test_mode_aba_cannot_publish_inflight_hardware_review(self):
        a = self.app()
        entered, release, a.geometry.support_path_check = self.block({'ok': True})
        planning = self.start(lambda: a.action('/api/control/home-preview', {}))
        switching = None

        def switch_twice():
            a.action('/api/control-mode', {'mode': 'simulation'})
            a.action('/api/control-mode', {'mode': 'hardware_sync'})

        try:
            self.assertTrue(entered.wait(1), planning.errors)
            switching = self.start(switch_twice)
            self.assertTrue(switching.done.wait(.5), 'mode change waited for preflight')
            self.assertEqual(switching.errors, [])
            self.assertEqual(a.control_mode, 'hardware_sync')
        finally:
            release.set()
            self.finish(planning, *([switching] if switching else []))
        self.assertEqual(planning.result, [])
        self.assertTrue(planning.errors and isinstance(planning.errors[0], ValueError), planning.errors)
        self.assertFalse(a.hardware_tokens)

    def test_fake_restart_revokes_review_already_waiting_on_gateway(self):
        a = self.app()
        a.fake_restart_lock = threading.Lock()
        a.fake_restart_in_progress = False
        a.fake_restart_pending = {'restart': (time.monotonic() + 30, 123, 4)}
        a.fake_restart_owner = lambda: {'pid': 123, 'command_epoch': 4}
        fresh = {'motion_enabled': True, 'host_sample_recent': True, 'owner': {
            'pid': 456, 'state': 'READY', 'simulation_only': True,
            'physical_hardware_connected': False, 'connected': True,
            'feedback_healthy': True, 'automatic_enabled': False}}
        entered, release, wait = self.block({'token': 'late-owner-token', 'expires_in_s': 30,
            'plans': [{'q_start': [0.] * 14, 'q_target': [0.] * 14}]})
        a._control_request.side_effect = lambda action, body=None: fresh if action == 'state' else wait()
        planning = self.start(lambda: a.action('/api/control/home-preview', {}))
        recovery = None
        with patch('joint_head_workbench.subprocess.run') as systemctl:
            try:
                self.assertTrue(entered.wait(1), planning.errors)
                recovery = self.start(lambda: a.fake_controller_restart_confirm(
                    {'token': 'restart', 'owner_pid': 123, 'command_epoch': 4}))
                self.assertTrue(recovery.done.wait(.5), 'fake recovery waited for preflight')
                self.assertEqual(recovery.errors, [])
                self.assertEqual(recovery.result[0]['owner_pid'], 456)
                self.assertEqual(systemctl.call_count, 2)
            finally:
                release.set()
                self.finish(planning, *([recovery] if recovery else []))
        self.assertEqual(planning.result, [], 'old owner preview revived after fake restart')
        self.assertTrue(planning.errors and isinstance(planning.errors[0], ValueError), planning.errors)
        self.assertFalse(a.hardware_tokens)

    def test_simulation_preview_late_path_result_cannot_revive_after_stop(self):
        a = self.app()
        a.control_mode = 'simulation'
        entered, release, a.geometry.path_check = self.block({'ok': True})
        planning = self.start(lambda: a.action('/api/preview', {'q': [.1] * 12}))
        stopped = None
        try:
            self.assertTrue(entered.wait(1), planning.errors)
            stopped = self.start(lambda: a.action('/api/stop', {}))
            self.assertTrue(stopped.done.wait(.5), 'simulation stop waited for path check')
            self.assertEqual(stopped.errors, [])
            self.assertIn('/stop', [call.args[0] for call in a.remote_request.call_args_list])
        finally:
            release.set()
            self.finish(planning, *([stopped] if stopped else []))
        self.assertEqual(planning.result, [])
        self.assertTrue(planning.errors and isinstance(planning.errors[0], ValueError), planning.errors)
        self.assertFalse(a.tokens)

    def test_trusted_calibration_seed_still_skips_all_collision_checks(self):
        a = self.app()
        self.calibration_queue(a)
        a.geometry.path_check.side_effect = AssertionError('frozen seed was screened again')
        a.geometry.support_path_check.side_effect = AssertionError('frozen seed support was screened again')
        result = a.action('/api/calibration-preview', {})
        self.assertEqual(result['token'], 'late-owner-token')
        self.assertFalse(result['collision_checked'])
        self.assertIn(result['token'], a.hardware_tokens)
        a.geometry.path_check.assert_not_called()
        a.geometry.support_path_check.assert_not_called()

    def calibration_queue(self, a, kind='head'):
        a.control_status['owner']['capabilities'] = ['calibration_capture_ack_v1']
        a.queue = [{'q': [0.] * 12, 'jaw_gap_mm': [80., 80.], 'capture': True}]
        a.plan = {'motion_seed': a.STANDARD_MOTION_SEED}
        camera = SimpleNamespace(read_calibration_frame=lambda: (b'image', {'serial': 'fake'}))
        names = ('head', 'left', 'right') if kind == 'wrist' else ('head',)
        a.hardware_calibration_cameras = lambda: ({name: camera for name in names},
                                                {name: {'serial': 'fake'} for name in names})
        if kind == 'wrist':
            a.workflow = {'kind': 'wrist', 'head_result_id': 'test-head'}
            a.plan['wrist_contract'] = {'environment': 'hardware', 'head_snapshot': {'id': 'test-head'}}
            a.require_joint_head_dependency = Mock()

    def test_stop_revokes_late_calibration_preview_too(self):
        a = self.app()
        self.calibration_queue(a)
        entered, release, wait = self.block({'token': 'late-owner-token', 'expires_in_s': 30})
        a._control_request.side_effect = lambda action, body=None: a.control_status if action=='state' else wait() if action.endswith('preview') else {'ok': True}
        planning = self.start(lambda: a.action('/api/calibration-preview', {}))
        stopped = None
        try:
            self.assertTrue(entered.wait(1), planning.errors)
            stopped = self.start(lambda: a.action('/api/calibration-pipeline', {'stop': True}))
            self.assertTrue(stopped.done.wait(.5), 'stop waited for calibration preview gateway')
            self.assertEqual(stopped.errors, [])
        finally:
            release.set()
            self.finish(planning, *([stopped] if stopped else []))
        self.assertEqual(planning.result, [])
        self.assertTrue(planning.errors and isinstance(planning.errors[0], ValueError), planning.errors)
        self.assertFalse(a.hardware_tokens)
        self.assertFalse(a.calibration_previews)

    def test_stop_during_confirm_identity_check_prevents_late_motion_request(self):
        a = self.app()
        a.hardware_tokens.add('reviewed-before-stop')
        entered, release, a.check_owner_mapping = self.block(None)
        confirming = self.start(lambda: a.action('/api/control/motion-confirm',
            {'token': 'reviewed-before-stop'}))
        stopped = None
        try:
            self.assertTrue(entered.wait(1), confirming.errors)
            stopped = self.start(lambda: a.action('/api/stop', {}))
            self.assertTrue(stopped.done.wait(.5), 'stop waited for confirm identity lookup')
            self.assertEqual(stopped.errors, [])
        finally:
            release.set()
            self.finish(confirming, *([stopped] if stopped else []))
        self.assertNotIn('motion-confirm', [call.args[0] for call in a._control_request.call_args_list])
        self.assertTrue(confirming.errors and isinstance(confirming.errors[0], ValueError), confirming.errors)

    def test_fresh_review_after_stop_is_usable_and_each_confirmation_is_single_use(self):
        a = self.app()
        tokens = iter(('first-review', 'fresh-after-stop'))

        def gateway(action, body=None):
            if action.endswith('preview'):
                return {'token': next(tokens), 'expires_in_s': 30,
                        'plans': [{'q_start': [0.] * 14, 'q_target': [0.] * 14}]}
            return {'ok': True}

        a._control_request.side_effect = gateway
        first = a.action('/api/hardware-trajectory-preview', {'waypoints': [{'q': [.1] * 12}]})
        self.assertIn(first['token'], a.hardware_tokens)
        self.assertEqual(a.action('/api/control/motion-confirm', {'token': first['token']}), {'ok': True})
        with self.assertRaises(ValueError):
            a.action('/api/control/motion-confirm', {'token': first['token']})
        a.action('/api/stop', {})
        fresh = a.action('/api/hardware-trajectory-preview', {'waypoints': [{'q': [.2] * 12}]})
        self.assertNotEqual(first['token'], fresh['token'])
        self.assertEqual(a.hardware_tokens, {fresh['token']})
        self.assertEqual(a.action('/api/control/motion-confirm', {'token': fresh['token']}), {'ok': True})
        with self.assertRaises(ValueError):
            a.action('/api/control/motion-confirm', {'token': fresh['token']})
        self.assertEqual([call.args[1]['token'] for call in a._control_request.call_args_list
                          if call.args[0] == 'motion-confirm'], [first['token'], fresh['token']])

    def finish_background_action(self, a, path, body=None):
        # Keep the production background() lifecycle, joining its worker instead
        # of polling status or replacing the invalidation under test.
        threads = []
        real_thread = threading.Thread

        def record_thread(*args, **kwargs):
            thread = real_thread(*args, **kwargs)
            threads.append(thread)
            return thread

        with patch('joint_head_workbench.threading.Thread', side_effect=record_thread):
            result = a.action(path, body or {})
        self.assertEqual(len(threads), 1)
        threads[0].join(1)
        self.assertFalse(threads[0].is_alive(), 'background planner did not finish')
        self.assertEqual(a.state()['status'], 'READY', a.reason)
        return result

    def new_plan(self, a, kind='head'):
        waypoint = {'q': [.2] * 12, 'arm': 'left', 'kind': 'grid', 'cell': [1, 1]}
        if kind == 'head':
            a.geometry.auto_plan = Mock(return_value={'plan_complete': True, 'waypoints': [waypoint]})
            return self.finish_background_action(a, '/api/auto-plan')

        def plan_wrists():
            with a.lock:
                a.queue = [waypoint]
                a.plan = {'plan_complete': True}
        a.plan_wrists = Mock(side_effect=plan_wrists)
        return self.finish_background_action(a, '/api/wrist-plan')

    def test_new_head_plan_rejects_calibration_preview_still_waiting_for_gateway(self):
        a = self.app()
        self.calibration_queue(a)
        entered, release, wait = self.block({'token': 'late-owner-token', 'expires_in_s': 30})
        a._control_request.side_effect = lambda action, body=None: a.control_status if action=='state' else wait()
        preview = self.start(lambda: a.action('/api/calibration-preview', {}))
        try:
            self.assertTrue(entered.wait(1), preview.errors)
            self.new_plan(a)
            self.assertEqual(a.queue[0]['q'], [.2] * 12)
        finally:
            release.set()
            self.finish(preview)
        self.assertEqual(preview.result, [])
        self.assertTrue(preview.errors and isinstance(preview.errors[0], ValueError), preview.errors)
        self.assertFalse(a.hardware_tokens)
        self.assertFalse(a.calibration_previews)
        a.remote_request.assert_not_called()

    def test_new_head_or_wrist_plan_revokes_completed_calibration_review(self):
        for kind in ('head', 'wrist'):
            with self.subTest(kind=kind):
                a = self.app()
                self.calibration_queue(a, kind)
                preview = a.action('/api/calibration-preview', {})
                self.assertIn(preview['token'], a.hardware_tokens)
                self.assertIn(preview['token'], a.calibration_previews)
                self.new_plan(a, kind)
                self.assertEqual(a.queue[0]['q'], [.2] * 12)
                self.assertFalse(a.hardware_tokens)
                self.assertFalse(a.calibration_previews)
                with self.assertRaises(ValueError):
                    a.action('/api/calibration-confirm', {'token': preview['token'],
                        'confirmation': 'START_REVIEWED_SINGLE_ARM', 'physical_clearance': True})
                self.assertNotIn('motion-confirm', [call.args[0] for call in a._control_request.call_args_list])
                a.remote_request.assert_not_called()

    def test_pipeline_start_uses_shared_background_review_invalidation(self):
        a = self.app()
        self.calibration_queue(a)
        a.action('/api/calibration-preview', {})
        self.assertTrue(a.hardware_tokens)
        a.pipeline_stop = Mock(side_effect=PipelineStopped('test stops before planning or capture'))
        self.finish_background_action(a, '/api/calibration-pipeline', {'mode': 'full'})
        self.assertTrue(a.pipeline_report['stopped'])
        self.assertFalse(a.hardware_tokens)
        self.assertFalse(a.calibration_previews)
        a.remote_request.assert_not_called()


if __name__ == '__main__':
    unittest.main()
