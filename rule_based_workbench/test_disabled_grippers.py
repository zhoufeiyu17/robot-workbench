"""CPU-only collection regression for an explicitly disconnected gripper."""
from copy import deepcopy
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT/'rule_based_workbench')]
from execution_timeline import initialize, prepare, validate
from frozen_executor import FrozenExecutor
from integration import _digest
from robot_workbench.execution_settings import defaults, DEFAULT_JOINT_TOLERANCE_RAD
from test_frozen_executor import fixture


class DisabledGripperTests(unittest.TestCase):
    def follower(self, disabled=('right',)):
        source, plan = fixture(.001)
        sent, stopped = [], []
        settings = defaults()
        settings['tracking_duration_s'] = 0.
        follower = FrozenExecutor(plan, source, sink=lambda q: sent.append(q),
            gate=lambda: True, stop=stopped.append, execution_policy='observation',
            execution_settings=settings, disabled_grippers=disabled)
        def feedback(now):
            state = deepcopy(sent[-1] if sent else plan['ticks'][0]['target'])
            state['right_gripper'] = 0.  # Unavailable raw placeholder, not a measured gap.
            return dict(sample_monotonic_s=now, state=state)
        return follower, plan, sent, stopped, feedback

    def test_disabled_right_zero_does_not_block_tracking_or_arrival(self):
        follower, plan, sent, stopped, feedback = self.follower()
        before = deepcopy(plan)
        follower.start(0., feedback(0.))
        for step in range(1, 10):
            result = follower.tick(step/100, feedback(step/100))
            self.assertNotEqual(result['status'], 'failed', result)
        self.assertEqual(result['status'], 'completed')
        self.assertTrue(result['arrival_qualified'])
        self.assertTrue(result['final_feedback_qualified'])
        self.assertIsNone(result['arrival_error']['jaw_mm'][1])
        self.assertEqual(result['disabled_grippers'], ['right'])
        self.assertEqual(plan, before)
        self.assertFalse(stopped)
        self.assertTrue(all(row['right_gripper'] == 38. for row in sent))

    def test_left_jaw_and_both_arm_guards_remain_active(self):
        for key in ('left_gripper', 'left_arm_joints', 'right_arm_joints'):
            with self.subTest(key=key):
                follower, _, _, stopped, feedback = self.follower()
                follower.start(0., feedback(0.))
                sample = feedback(.01)
                if key == 'left_gripper':sample['state'][key] += 10.
                else:sample['state'][key][0] += .2
                result = follower.tick(.01, sample)
                self.assertEqual(result['status'], 'failed', result)
                self.assertIn('tracking', result['reason'])
                self.assertEqual(len(stopped), 1)
        follower, _, _, _, feedback = self.follower(disabled=())
        follower.start(0., feedback(0.))
        self.assertEqual(follower.tick(.01, feedback(.01))['status'], 'failed')
        follower, _, _, _, feedback = self.follower()
        follower.start(0., feedback(0.))
        sample = feedback(.01);sample['state']['left_gripper'] += 3.
        result = follower.tick(.01, sample)
        self.assertEqual(result['status'], 'running')
        self.assertFalse(result['arrival_qualified'])

    def test_disabled_jaw_commands_are_rejected_not_silently_dropped(self):
        source, plan = fixture()
        for disabled in (('left',), ('right',)):
            candidate = deepcopy(plan)
            if disabled == ('right',):
                candidate['ticks'][1]['target']['right_gripper'] += 1.
                candidate['timeline_digest'] = _digest(candidate['ticks'])
            with self.assertRaisesRegex(ValueError, 'disabled .* gripper'):
                FrozenExecutor(candidate, source, sink=lambda _: None, gate=lambda: True,
                    stop=lambda _: None, source_validator=lambda *_: None,
                    disabled_grippers=disabled)

    def test_initialization_keeps_raw_evidence_without_phantom_jaw_return(self):
        _, plan = fixture()
        plan['stages'] = [dict(name='left_operation', start_tick=0)]
        base = prepare(plan)
        current = deepcopy(base['commands'][0]['target'])
        current['right_gripper'] = 0.
        limits = dict(left_arm_joints=[[-3., 3.]]*6, right_arm_joints=[[-3., 3.]]*6,
                      left_gripper=[0., 83.], right_gripper=[0., 83.])
        ordinary = initialize(base, current, limits)
        self.assertGreater(ordinary['initialization_duration_s'], 10.)
        timeline = initialize(base, current, limits, disabled_grippers=('right',))
        self.assertEqual(timeline['initialization_duration_s'], 0.)
        self.assertEqual(timeline['initialization']['actual_start']['right_gripper'], 0.)
        self.assertEqual(timeline['initialization']['legal_start']['right_gripper'], 38.)
        self.assertEqual(timeline['initialization']['disabled_grippers'], ['right'])
        self.assertEqual(timeline['commands'], base['commands'])
        self.assertEqual(validate(timeline, plan), timeline)
        bad = deepcopy(timeline)
        del bad['initialization']['disabled_grippers']
        with self.assertRaisesRegex(ValueError, 'sealed parent'):
            validate(bad, plan)

    def test_inactive_arm_is_fixed_before_review_and_drift_requires_reobservation(self):
        _,plan=fixture()
        plan['stages']=[dict(name='left_operation',start_tick=0)]
        for row in plan['ticks']:row['target']['right_arm_joints']=[.2]*6
        plan['timeline_digest']=_digest(plan['ticks'])
        base=prepare(plan);current=deepcopy(base['commands'][0]['target'])
        current['left_arm_joints'][0]=.05
        limits=dict(left_arm_joints=[[-3.,3.]]*6,right_arm_joints=[[-3.,3.]]*6,
                    left_gripper=[0.,83.],right_gripper=[0.,83.])
        timeline=initialize(base,current,limits,held_arms=('right',))
        self.assertGreater(timeline['initialization_duration_s'],0.)
        self.assertTrue(all(row['target']['right_arm_joints']==[.2]*6 for row in timeline['commands']))
        self.assertEqual(timeline['commands'][-3:],[dict(row,tick=row['tick']+len(timeline['commands'])-3,
            time_s=row['time_s']+timeline['initialization_duration_s']) for row in base['commands']])
        self.assertEqual(validate(timeline,plan),timeline)
        current['right_arm_joints'][0]+=.011
        with self.assertRaisesRegex(ValueError,'reobserve and replan'):
            initialize(base,current,limits,held_arms=('right',))
        current['right_arm_joints'][0]-=.011
        moved=deepcopy(base);moved['commands'][1]['target']['right_arm_joints'][0]+=.001
        with self.assertRaisesRegex(ValueError,'reobserve and replan'):
            initialize(moved,current,limits,held_arms=('right',))

    def test_inactive_arm_execution_respects_user_tracking_policy_without_changing_targets(self):
        source,plan=fixture(.001)
        source['initial_state']={'held_arms':['right']}
        for enforced in (False,True):
            sent=[];stopped=[];settings=defaults('collection')
            settings.update(tracking_enabled=enforced,tracking_duration_s=0.)
            follower=FrozenExecutor(plan,source,sink=sent.append,gate=lambda:True,stop=stopped.append,
                execution_policy='observation',execution_settings=settings)
            sample=dict(sample_monotonic_s=0.,state=deepcopy(plan['ticks'][0]['target']))
            self.assertEqual(follower.start(0.,sample)['status'],'running')
            sample=deepcopy(sample);sample['sample_monotonic_s']=.01
            sample['state']['right_arm_joints'][0]+=.2
            result=follower.tick(.01,sample)
            self.assertEqual(result['status'],'failed' if enforced else 'running')
            self.assertEqual(result['tracking_error']['joint_rad'],.2)
            self.assertEqual(len(sent),1 if enforced else 2)
            self.assertEqual(len(stopped),1 if enforced else 0)
            self.assertTrue(all(row['right_arm_joints']==plan['ticks'][0]['target']['right_arm_joints'] for row in sent))

    def test_owner_propagates_disabled_channel_through_real_collection_runner(self):
        from real_control import load_calibration, classes
        from test_real_control import Takeover, State
        ref = load_calibration(ROOT/'calibration')
        from arx_env import joint_head_jaw_mapping as jaw
        jaw.configure_mapping(None)
        Owner, sdk, _ = classes(ref)
        source, plan = fixture(.001)
        plan['stages'] = [dict(name='initial_open', start_tick=0)]
        # This synthetic source exercises the runtime hold policy; the actual
        # observed-state/source binding is checked in test_initial_state.py.
        source['initial_state']={'held_arms':['right']}
        for row in plan['ticks']:row['target']['right_arm_joints']=[.2]*6
        plan['timeline_digest']=_digest(plan['ticks'])
        def raw(target):
            values = sdk(target)
            values[6] = jaw.interpolate(target['left_gripper'], jaw.GAP_MM['left'], jaw.FEEDBACK['left'])
            values[13] = 0.
            return values
        class Backend(ref.SimulatedBackend):
            passive_jaws = ('right',)
            def send(self, side, values):
                super().send(side, values)
                self.q[13] = 0.
        clock = [100.]
        with patch('time.monotonic', lambda: clock[0]):
            backend = Backend(raw(plan['ticks'][0]['target']))
            owner = Owner(backend, SimpleNamespace(PedalTakeover=Takeover, TakeoverState=State))
            owner.execution_error_settings = defaults()
            owner.execution_error_settings['tracking_duration_s'] = 0.
            for _ in range(42):
                owner.tick();clock[0] += .01
            preview = owner.submit(dict(action='preview_frozen', plan=plan, source=source))
            self.assertEqual(preview['disabled_grippers'], ['right'])
            self.assertEqual(preview['held_arms'], ['right'])
            owner.submit(dict(action='confirm', token=preview['token'],
                confirmation='START_REVIEWED_SINGLE_ARM', physical_clearance=True))
            for _ in range(150):
                runner = owner.runner
                if runner and runner.runtime_started:
                    self.assertTrue(runner.runtime_ready.wait(2))
                if runner and runner.last_sent_index >= 0:
                    backend.q = raw(runner.frozen.ticks[runner.last_sent_index]['target'])
                owner.tick();clock[0] += .01
                if owner.runner and owner.runner.status in ('READY', 'FAULT'):break
            self.assertEqual(owner.runner.status, 'READY', owner.snapshot())
            self.assertEqual(owner.runner.disabled_grippers, ('right',))
            self.assertEqual(owner.runner.frozen.disabled_grippers, ('right',))
            self.assertEqual(owner.runner.runtime['initialization_duration_s'], 0.)
            self.assertEqual(owner.runner.runtime['initialization']['actual_start']['right_gripper'], 0.)
            self.assertIsNone(owner.runner.frozen.arrival_error['jaw_mm'][1])
            self.assertEqual(owner.runner.runtime['initialization']['held_arms'],['right'])
            self.assertTrue(all(row['target']['right_arm_joints']==[.2]*6 for row in owner.runner.frozen.ticks))
            backend.q[7]+=DEFAULT_JOINT_TOLERANCE_RAD+.001
            with self.assertRaisesRegex(ValueError,'reobserve and replan'):
                owner.runner._check_start_drift(backend.q)


if __name__ == '__main__':
    unittest.main()
