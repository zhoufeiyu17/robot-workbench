"""CPU policies exercise the real shared Owner and both runner strategies."""
from copy import deepcopy
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT/'tests'), str(ROOT/'calibration'),
               str(ROOT/'calibration/infra'), str(ROOT/'calibration/tests'), str(ROOT/'rule_based_workbench')]
import test_unified_runtime as fixture
from robot_workbench.execution_settings import defaults, validate
from arx_env.joint_head_real_motion import MotionRunner, make_plan
from test_joint_head_real_motion import limits, feedback
from frozen_executor import FrozenExecutor
from test_frozen_executor import fixture as frozen_fixture


class SettingsTest(unittest.TestCase):
    setUp = fixture.UnifiedRuntimeTest.setUp
    cleanup = fixture.UnifiedRuntimeTest.cleanup
    create = fixture.UnifiedRuntimeTest.create
    advance = fixture.UnifiedRuntimeTest.advance
    preview = fixture.UnifiedRuntimeTest.preview
    confirm = fixture.UnifiedRuntimeTest.confirm

    def test_update_preserves_defaults_revokes_reviews_and_persists_without_motion(self):
        runtime, creds = self.create()
        runtime.owner.arm_tolerances = [.007+i*.001 for i in range(12)]
        settings = runtime.execution_settings_snapshot('calibration')['settings']
        self.assertEqual(settings['arrival_tolerances_rad'], runtime.owner.arm_tolerances)
        self.assertFalse(runtime.execution_settings_snapshot('collection')['settings']['tracking_enabled'])
        preview = self.preview(runtime, creds['calibration'])
        before = deepcopy(runtime.owner.backend.events)
        settings.update(arrival_enabled=False, tracking_enabled=False)
        result = runtime.update_execution_settings(creds['calibration'], settings)
        self.assertEqual(result['revision'], 1)
        self.assertEqual(runtime.owner.backend.events, before)
        self.assertFalse(runtime.reviewed_plans)
        self.assertEqual(json.loads(runtime.execution_settings_path.read_text()), runtime.export_execution_settings())
        with self.assertRaisesRegex(ValueError, 'unknown or consumed'):
            self.confirm(runtime, creds['calibration'], preview)
        self.confirm(runtime, creds['calibration'], self.preview(runtime, creds['calibration']))
        self.advance(runtime)
        self.assertEqual(runtime.owner.runner.error_settings, settings)
        self.assertEqual(runtime.execution_calibration['execution_settings']['settings'], settings)
        with self.assertRaisesRegex(ValueError, 'motion or capture'):
            runtime.update_execution_settings(creds['calibration'], settings)
        runtime.submit(creds['collection'], {'action':'stop'})
        self.advance(runtime)
        self.assertEqual(runtime.owner.runner.status, 'STOPPED')

    def test_authentication_backend_support_and_bad_values(self):
        runtime, creds = self.create()
        settings = defaults()
        with self.assertRaises(PermissionError):
            runtime.update_execution_settings(creds['collection'], settings)
        settings['fake_error']['enabled'] = True
        with self.assertRaisesRegex(ValueError, 'does not support'):
            runtime.update_execution_settings(creds['calibration'], settings)
        self.assertEqual(runtime.execution_settings_revision, 0)
        with self.assertRaisesRegex(ValueError, 'only in fake'):
            validate(settings, environment='physical')
        for changes in ({'arrival_enabled':1}, {'tracking_joint_rad':float('nan')},
                        {'arrival_tolerances_rad':[.01]*11}, {'tracking_duration_s':-1}):
            with self.assertRaises(ValueError):
                validate(dict(defaults(), **changes))

    def test_persistent_policy_restores_and_business_handoff_keeps_other_default(self):
        runtime,creds=self.create()
        settings=defaults();settings.update(arrival_enabled=False,tracking_enabled=False,arrival_jaw_mm=7.)
        runtime.update_execution_settings(creds['calibration'],settings)
        runtime.handoff(creds['calibration'],'collection')
        self.assertEqual(runtime.execution_settings_snapshot('collection')['settings'], defaults('collection'))
        runtime.owner.submit(dict(action='release',confirmation='ARMS_SUPPORTED_RELEASE'))
        runtime.tick();runtime.close()
        restored,creds=self.create()
        self.assertEqual(restored.execution_settings_snapshot('calibration')['settings'],settings)
        self.assertEqual(restored.execution_settings_revision,1)
        with self.assertRaisesRegex(ValueError,'startup-only'):
            restored.restore_execution_settings(restored.export_execution_settings())

    def test_untouched_calibration_keeps_legacy_passive_guards_until_explicit_save(self):
        for custom in (False, True):
            runtime, creds = self.create(device='custom' if custom else 'untouched')
            if custom:
                runtime.update_execution_settings(creds['calibration'], defaults())
            state = runtime.execution_settings_snapshot('calibration')
            self.assertEqual(state['legacy_passive_guards'], not custom)
            self.confirm(runtime, creds['calibration'], self.preview(runtime, creds['calibration']))
            self.advance(runtime, 35)
            self.assertEqual(runtime.owner.runner.configured_error_policy, custom)
            self.assertEqual(runtime.execution_calibration['execution_settings']['legacy_passive_guards'], not custom)
            runtime.owner.backend.q[7] = .02
            self.advance(runtime)
            if custom:
                self.assertNotEqual(runtime.owner.runner.status, 'FAULT')
            else:
                self.assertEqual(runtime.owner.runner.status, 'FAULT')
                self.assertIn('inactive arm drift', runtime.owner.runner.reason)

    def test_manual_no_threshold_finishes_with_real_residual_but_stale_feedback_still_faults(self):
        q = [0.]*14
        target = q[:]; target[0] = .2
        plan = make_plan('both', q, target, limits(), kind='workbench', created_at_s=8500., duration_s=2.)
        settings = defaults(); settings.update(arrival_enabled=False, tracking_enabled=False)
        sent=[]
        runner = MotionRunner(plan, begin=lambda q:q, send=sent.append, stop=lambda reason:None,
                              execution_settings=settings)
        runner.start(8500., feedback(8500., q))
        for i in range(1, 50):
            now = 8500.+i*.05
            runner.tick(now, feedback(now, q))
        self.assertEqual(runner.status, 'READY', runner.reason)
        self.assertAlmostEqual(sent[-1][0], .2)
        self.assertGreater(runner.tracking_exceeded_samples, 0)
        self.assertAlmostEqual(runner.target_error['arm_rad']['left_J1'], -.2)
        self.assertEqual(runner.tick(now+.05, feedback(now-.2, q)), 'FAULT')
        self.assertIn('older than', runner.reason)

    def test_configured_tracking_and_arrival_are_independent(self):
        for arrival, tracking, expected in ((False,False,'READY'), (True,False,'FAULT'), (False,True,'FAULT')):
            q=[0.]*14; target=q[:];target[0]=.15
            plan=make_plan('both',q,target,limits(),kind='workbench',created_at_s=8500.,duration_s=2.)
            settings=defaults(); settings.update(arrival_enabled=arrival,tracking_enabled=tracking,
                                                tracking_joint_rad=.02,tracking_duration_s=.1)
            runner=MotionRunner(plan,begin=lambda q:q,send=lambda q:None,stop=lambda why:None,execution_settings=settings)
            runner.start(8500.,feedback(8500.,q))
            for n in range(1,120):runner.tick(8500.+n*.05,feedback(8500.+n*.05,q))
            self.assertEqual(runner.status,expected,runner.reason)

    def test_capture_waits_for_stationarity_without_requiring_arrival_and_queue_continues(self):
        runtime,creds=self.create()
        settings=defaults();settings.update(arrival_enabled=False,tracking_enabled=False)
        runtime.update_execution_settings(creds['calibration'],settings)
        waypoints=[dict(q=[.03]*12,speeds=[.1]*12,capture=True,sample_id='first'),
                   dict(q=[.06]*12,speeds=[.1]*12,capture=False)]
        review=runtime.submit(creds['calibration'],dict(action='preview_workbench',waypoints=waypoints))
        self.confirm(runtime,creds['calibration'],review)
        backend=runtime.owner.backend._backend
        def lag(side,q):
            # Real feedback remains far from command; hold the actual zero pose.
            backend.events.append(('send',side,list(q)))
        with patch.object(backend,'send',side_effect=lag):
            for _ in range(300):
                self.advance(runtime)
                if runtime.owner.capture_wait:break
            self.assertIsNotNone(runtime.owner.capture_wait,runtime.owner.snapshot())
            self.assertAlmostEqual(runtime.owner.runner.target_error['arm_rad']['left_J1'],-.03)
            old=runtime.owner.capture_wait['ready_wall_ns']
            # Motion after READY revokes the capture window despite observation mode.
            for n in range(3):
                backend.q[0]+=.004
                self.advance(runtime)
            self.assertIsNone(runtime.owner.capture_wait)
            for _ in range(100):
                self.advance(runtime)
                if runtime.owner.capture_wait:break
            self.assertGreater(runtime.owner.capture_wait['ready_wall_ns'],old)
            runtime.submit(creds['calibration'],dict(action='capture_ack',owner_pid=runtime.owner.cache['pid'],
                command_epoch=runtime.owner.epoch,sample_id='first',capture_ready_wall_ns=runtime.owner.capture_wait['ready_wall_ns']))
            for _ in range(300):self.advance(runtime)
            self.assertEqual(runtime.owner.runner.status,'READY',runtime.owner.runner.reason)
            self.assertFalse(runtime.owner.sequence_remaining)
            self.assertIsNone(runtime.owner.capture_sample)

    def test_frozen_tracking_duration_and_endpoint_policy(self):
        source,plan=frozen_fixture()
        for arrival,tracking,expected in ((False,False,'completed'),(True,False,'failed'),(False,True,'failed')):
            settings=defaults('collection');settings.update(arrival_enabled=arrival,tracking_enabled=tracking,
                tracking_joint_rad=.001,tracking_duration_s=.02,arrival_tolerances_rad=[.001]*12)
            follower=FrozenExecutor(plan,source,sink=lambda q:None,gate=lambda:True,stop=lambda why:None,
                                    execution_policy='observation',execution_settings=settings)
            measured=deepcopy(plan['ticks'][0]['target'])
            state=lambda t:dict(sample_monotonic_s=t,state=measured)
            follower.start(10.,state(10.))
            for n in range(1,400):
                result=follower.tick(10.+n*.01,state(10.+n*.01))
                if result['status'] in ('completed','failed'):break
            self.assertEqual(result['status'],expected,result)
            self.assertGreater(result['tracking_exceeded_samples'],0)
            self.assertEqual(result['execution_settings'],settings)


if __name__ == '__main__':
    unittest.main()
