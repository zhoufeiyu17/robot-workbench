"""CPU checks for separate execution and wall clocks; no SDK or PhysX imports."""
from pathlib import Path
import sys
import time
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'calibration'), str(ROOT / 'calibration/infra'),
               str(ROOT / 'calibration/tests'), str(ROOT / 'rule_based_workbench')]
from examples.scripts.debug import joint_head_real_owner as calibration
from arx_env import joint_head_jaw_mapping as jaw
from arx_env.joint_head_real_motion import make_workbench_plans
from test_joint_head_real_owner import Safety
from real_control import classes
from test_frozen_executor import fixture


class ClockBackend(calibration.SimulatedBackend):
    """Protocol fake only: advancing feedback is explicitly supplied by the test."""
    def __init__(self, clock, q=None):
        super().__init__(q)
        self.clock = clock
        self.read_delay = lambda: None
        self.frozen_sample = None

    def read_calibration_state(self):
        self.read_delay()
        now = self.clock() if self.frozen_sample is None else self.frozen_sample
        return dict(qpos=self.q[:], qvel=[0.] * 14, sample_time_s=now,
                    sample_wall_ns=time.time_ns(), sdk_timestamps=[now, now + 41.])

    def send(self, side, q):
        super().send(side, q)
        slot = 6 if side == 'left' else 13
        self.q[slot] = jaw.feedback_sdk(side, jaw.command_gap_mm(side, q[slot]))


class OwnerClockTest(unittest.TestCase):
    def setUp(self):
        jaw.configure_mapping(None)
        self.sim, self.wall = 10., 10000.
        self.wall_patch = patch('time.monotonic', side_effect=lambda: self.wall)
        self.wall_patch.start()
        self.addCleanup(self.wall_patch.stop)

    def ready(self, owner_type=calibration.Owner, q=None, events=None):
        backend = ClockBackend(lambda: self.sim, q)
        owner = owner_type(backend, Safety, control_clock=lambda: self.sim,
                           event=lambda kind, **fields: events.append((kind, fields)) if events is not None else None)
        for _ in range(42):
            owner.tick()
            self.sim += .01
        self.assertTrue(owner.snapshot()['stationary'])
        return owner

    def preview(self, owner):
        target = owner.snapshot()['calibration_feedback']['qpos'][:]
        target[0] += .01
        return owner.submit(dict(action='preview', active_arm='left', q_target=target))

    def confirm(self, owner, review):
        return owner.submit(dict(action='confirm', token=review['token'],
                                 confirmation='START_REVIEWED_SINGLE_ARM', physical_clearance=True))

    def test_paused_sim_review_expires_by_wall_time(self):
        owner = self.ready()
        review = self.preview(owner)
        self.assertEqual(review['plan']['created_at_s'], self.sim)
        self.assertEqual(len(owner.pending[review['token']]), 5)
        self.wall += 30.001
        with self.assertRaisesRegex(ValueError, 'expired'):
            self.confirm(owner, review)
        self.assertFalse(any(event[0] == 'begin' for event in owner.backend.events))

    def test_unexpired_review_and_execution_use_injected_time(self):
        owner = self.ready()
        review = self.preview(owner)
        self.wall += 29.
        self.confirm(owner, review)
        owner.tick()
        for _ in range(200):
            self.sim += .01
            self.wall += 1.
            owner.tick()
        self.assertEqual(owner.runner.status, 'READY', owner.runner.reason)
        self.assertAlmostEqual(owner.backend.q[0], .01)
        diagnostic = owner.diagnostic_sample(owner.current_feedback)
        self.assertEqual(diagnostic['monotonic_s'], self.wall)
        self.assertEqual(diagnostic['control_time_s'], self.sim)
        self.assertEqual(owner.snapshot()['control_clock'], 'injected')

    def test_sim_advancement_does_not_expire_review_before_wall_deadline(self):
        owner = self.ready()
        review = self.preview(owner)
        self.sim += 31.
        owner.tick()
        self.confirm(owner, review)
        owner.tick()
        self.assertEqual(owner.runner.status, 'VERIFYING', owner.runner.reason)

    def test_queued_review_still_expires_on_wall_clock(self):
        owner = self.ready()
        review = self.preview(owner)
        self.confirm(owner, review)
        self.wall += 31.
        owner.tick()
        self.assertEqual(owner.runner.status, 'FAULT')
        self.assertIn('expired', owner.runner.reason)

    def test_capture_window_and_recovery_use_control_time(self):
        owner = self.ready()
        review = owner.submit(dict(action='preview_workbench', waypoints=[
            dict(q=[.01] * 12, speeds=[.1] * 12, capture=True, sample_id='clock-sample')]))
        self.assertEqual(review['plans'][0]['created_at_s'], self.sim)
        self.confirm(owner, review)
        for _ in range(250):
            self.sim += .01
            self.wall += 1.
            owner.tick()
            if owner.capture_wait:
                break
        self.assertIsNotNone(owner.capture_wait, owner.snapshot())
        with patch.object(owner.stationary_window, 'update', return_value=False):
            self.sim += .01
            owner.tick()
        self.assertAlmostEqual(owner.capture_recovery_deadline, self.sim + 2.)
        self.wall += 100.
        self.sim += .01
        owner.tick()
        self.assertNotEqual(owner.runner.status, 'FAULT', owner.runner.reason)

    def test_feedback_freshness_uses_control_but_read_duration_uses_wall(self):
        owner = self.ready()
        def delayed_read():
            self.wall += .025
        owner.backend.read_delay = delayed_read
        owner.read_feedback()
        self.assertAlmostEqual(owner.feedback_read_duration_s, .025)
        owner.backend.frozen_sample = self.sim
        self.sim += .101
        with self.assertRaisesRegex(ValueError, 'stale'):
            owner.read_feedback()

    def test_default_clock_is_dynamic_and_plan_helper_accepts_explicit_time(self):
        owner = calibration.Owner(calibration.SimulatedBackend(), Safety)
        self.assertEqual(owner.control_time(), self.wall)
        self.wall += .01
        self.assertEqual(owner.control_time(), self.wall)
        q = [0.] * 14
        rows = [dict(q=[0.] * 12, speeds=[.1] * 12)]
        limits = owner.backend.get_bimanual_joint_position_limits()
        self.assertEqual(make_workbench_plans(q, rows, limits)[0].created_at_s, self.wall)
        self.assertEqual(make_workbench_plans(q, rows, limits, created_at_s=0.)[0].created_at_s, 0.)

    def test_collection_review_ttl_and_command_clock(self):
        collection, _, _ = classes(calibration)
        source, plan = fixture()
        plan['stages'] = [dict(name='return_home', start_tick=0, end_tick=3)]
        target = plan['ticks'][0]['target']
        q = [*target['left_arm_joints'], jaw.feedback_sdk('left', target['left_gripper']),
             *target['right_arm_joints'], jaw.feedback_sdk('right', target['right_gripper'])]
        events = []
        owner = self.ready(collection, q, events)
        review = owner.submit(dict(action='preview_frozen', plan=plan, source=source))
        self.assertEqual(owner.pending[review['token']][0][0].created_at_s, self.sim)
        self.wall += 31.
        with self.assertRaisesRegex(ValueError, 'expired'):
            self.confirm(owner, review)
        review = owner.submit(dict(action='preview_frozen', plan=plan, source=source))
        self.confirm(owner, review)
        for _ in range(250):
            self.sim += .01
            self.wall += 1.
            owner.tick()
            if owner.runner.runtime_started:
                self.assertTrue(owner.runner.runtime_ready.wait(1.))
            if owner.runner.status == 'READY':
                break
        self.assertEqual(owner.runner.status, 'READY', owner.runner.reason)
        commands = [fields for kind, fields in events if kind == 'command']
        self.assertEqual([row['tick'] for row in commands], [0, 1, 2])
        self.assertTrue(all(10. <= row['send_monotonic_s'] <= self.sim for row in commands))
        self.assertAlmostEqual(owner.snapshot()['sample_age_s'], 0.)


if __name__ == '__main__':
    unittest.main()
