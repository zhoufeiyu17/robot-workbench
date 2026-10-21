"""CPU contract fixtures only: no GPU, scene modification, SDK or motion."""
from copy import deepcopy
import math
import unittest

from cup_return_replay import build_replay_source, build_return_plan, validate_evidence, replay_model_plan
from frozen_executor import FrozenExecutor
from integration import _digest
from physical_motiongen import freeze_physical
from real_contract import validate_real_plan
from test_fake_device_pipeline import fake_inputs
from test_physical_motiongen import bind_contact_fixture


def fixture(*, jaw_mapping_sha256=None):
    source, model, binding = fake_inputs()
    if jaw_mapping_sha256 is not None:
        source['calibration_profile']['model_binding']['jaw_mapping_sha256']=jaw_mapping_sha256
        profile_digest=_digest(source['calibration_profile'])
        source['initial_state']['calibration_profile_digest']=profile_digest
        model['calibration_profile_digest']=binding['calibration_profile_digest']=profile_digest
        model['planning_validation']['source_digest']=binding['source_digest']=_digest(source)
    original = deepcopy(source['initial_state']['model'])
    opening, closed = original['left_gripper'], 64.992
    spans = [('initial_hold', 1), ('initial_open', 2), ('approach_pre', 16), ('approach', 16),
        ('preclose_settle', 3), ('close_ramp', 11), ('close_and_hold', 18), ('lift', 16),
        ('transport', 4), ('transport', 4), ('lower', 16), ('support_settle', 3),
        ('open', 45), ('release_hold', 6), ('retreat', 16), ('home', 16),
        ('home_gripper_restore', 2), ('home_hold', 9)]
    stages, targets = [], []
    from gripper_distance_plan import HOLDING_STAGES
    for name, count in spans:
        stages.append(dict(name=name, start_tick=len(targets), motiongen_receipt_index=0))
        for i in range(count):
            row = deepcopy(original)
            u=i/max(1,count-1);u=10*u**3-15*u**4+6*u**5
            heights={'approach_pre':(0.,.06),'approach':(.06,0.),'lift':(0.,.04),
                'transport':(.04,.04),'lower':(.04,.02),'support_settle':(.02,.02),
                'open':(.02,.02),'release_hold':(.02,.02),'retreat':(.02,.06),'home':(.06,0.)}
            lo,hi=heights.get(name,(0.,0.));row['left_arm_joints'][2]+=lo+(hi-lo)*u
            if name in HOLDING_STAGES: row['left_gripper'] = closed
            if name == 'close_ramp': row['left_gripper'] = opening-(opening-closed)*i/(count-1)
            if name == 'open': row['left_gripper'] = closed+(opening-closed)*(1-math.cos(math.pi*i/44))/2
            if name == 'approach_pre': row['left_arm_joints'][1] += .001*math.sin(math.pi*i/(count-1))
            targets.append(row)
    model.update(stages=stages, ticks=[dict(tick=i, time_s=i/30, target=t) for i,t in enumerate(targets)],
        native_actions=[[0.]*6+[t['left_gripper']]+[0.]*6+[t['right_gripper']] for t in targets],
        native_substeps=[8]*len(targets), duration_s=len(targets)/30,
        native_identity=dict(acceptance=dict(landing_speed_m_s=.03,stable_window_s=.1,hand_clearance_m=.01)),
        gripper_opening_mm=opening, normal_recipe=dict(requested_speed_scale=.97),
        gripper_release=dict(schema_version='commanded_gap_release.v3', duration_s=1.5,
                             maximum_gap_speed_mm_s=20., initial_model_gap_mm=closed, arm_motion=False))
    model['timeline_digest'] = _digest(model['ticks'])
    model['planning_validation']['timeline_digest'] = model['timeline_digest']
    from control_motion import validate_control
    model['control_validation'] = validate_control(model)
    bind_contact_fixture(model, source)
    parent = freeze_physical(model, source, binding, environment='fake')
    cup = deepcopy(source['scene']['environment_from_cup'])
    cup['position'][0] += .1
    receipt = dict(schema_version='owner_physx_native_task.v1', environment='fake', simulation_only=True,
        source_digest=_digest(source), parent_timeline_digest=parent['timeline_digest'],
        source_clock_id=source['clock']['source_clock_id'], motion_outcome='completed',
        task_outcome='success', task_qualified=True, execution_instance_id='completed-parent',
        physical_configuration_sha256='a'*64, last_observation=dict(sample=dict(world_from_cup=cup)))
    current = deepcopy(source['initial_state'])
    current['clock']['source_clock_id'] = 'new-plant-clock'
    for row in current['owner_samples']: row['source_clock_id'] = 'new-plant-clock'
    context = dict(schema_version='cup_return_context.v1', clock=current['clock'],
        parent_execution_instance_id=receipt['execution_instance_id'], physical_configuration_sha256='a'*64,
        current_cup_pose=cup, current_machine_pose=deepcopy(source['scene']['environment_from_machine_root']),
        parent_fixture_machine_pose=deepcopy(source['scene']['environment_from_machine_root']),
        parent_fixture_sha256='c'*64, pose_basis='parent_plant_fixture')
    replay_source = build_replay_source(source, current, context, run_id='return-cup')
    replay_binding = deepcopy(binding)
    replay_binding.update(source_digest=_digest(replay_source), initial_state=current, clock=current['clock'],
        schema_version='fake_current_replay_binding.v1',evidence_source='fake_current_replay_context',
        observation_scope='historical_parent_only')
    replay = build_return_plan(parent, source, receipt, replay_source)
    return source, parent, receipt, replay_source, replay_binding, replay


class CupReturnReplayTest(unittest.TestCase):
    def test_exact_arm_reverse_and_honest_new_context(self):
        original, parent, receipt, source, binding, plan = fixture()
        validate_real_plan(plan, source, binding, environment='fake')
        self.assertNotEqual(source['clock'], original['clock'])
        self.assertEqual(source['scene'], original['scene'])
        self.assertEqual(source['observation_scope'], 'historical_parent_only')
        self.assertEqual(plan['simulation'], dict(validated=False, outcome='not_run'))
        self.assertNotIn('physical_gpu_conversion', plan)
        self.assertNotIn('gpu_final_acceptance', plan['planning_validation'])
        proof = plan['replay_derivation']
        self.assertEqual(proof['parent_native_receipt'], receipt)
        self.assertEqual(proof['return_goal'], original['scene']['environment_from_cup'])
        self.assertEqual(proof['collision_check']['status'], 'not_run')
        for tick, index in zip(plan['ticks'], proof['parent_tick_indices']):
            for side in ('left', 'right'):
                self.assertEqual(tick['target'][side+'_arm_joints'], parent['ticks'][index]['target'][side+'_arm_joints'])
        self.assertEqual(proof['parent_tick_indices'], sorted(proof['parent_tick_indices'], reverse=True))
        follower = FrozenExecutor(plan, source, sink=lambda _:None, gate=lambda:False, stop=lambda _:None,
            source_validator=lambda p,s:validate_real_plan(p,s,binding,environment='fake'))
        self.assertEqual(sum(follower.unloading_ticks), 45)
        self.assertEqual(follower.snapshot()['validation_scope'], 'exact_parent_reverse_without_new_collision_check')
        from execution_timeline import build, validate
        validate(build(plan), plan)
        model = replay_model_plan(plan, source)
        self.assertNotIn('joint_space', model)
        self.assertEqual(model['simulation']['outcome'], 'not_run')

    def test_altered_targets_or_provenance_cannot_claim_reverse(self):
        *_, source, binding, plan = fixture()
        changes = [lambda p:p['ticks'][25]['target']['left_arm_joints'].__setitem__(0, .3),
            lambda p:p['native_actions'][25].__setitem__(6, 64.),
            lambda p:p['stages'][3].__setitem__('name', 'transport'),
            lambda p:p['replay_derivation']['parent_native_receipt'].__setitem__('task_outcome', 'failed'),
            lambda p:p['replay_derivation']['parent_native_receipt'].__setitem__('source_clock_id', 'wrong-plant'),
            lambda p:p['replay_derivation']['parent_plan']['physical_gpu_conversion']['model_validation']['gpu_final_acceptance']['support'].__setitem__('passed', False),
            lambda p:p['control'].__setitem__('max_target_velocity_rad_s', 20.),
            lambda p:p['simulation'].update(validated=True, outcome='success')]
        for mutate in changes:
            bad = deepcopy(plan); mutate(bad)
            with self.assertRaises(ValueError): validate_evidence(bad, source)
        with self.assertRaises(ValueError): validate_real_plan(plan, source, binding, environment='physical')

    def test_changed_scene_calibration_or_unqualified_parent_rejected(self):
        original, parent, receipt, source, binding, plan = fixture()
        for field in ('current_cup_pose', 'current_machine_pose'):
            bad = deepcopy(source); bad['replay_context'][field]['position'][0] += .007
            with self.assertRaises(ValueError): build_return_plan(parent, original, receipt, bad)
        for field in ('physical_configuration_sha256', 'parent_execution_instance_id'):
            bad = deepcopy(source); bad['replay_context'][field] = 'wrong'
            with self.assertRaises(ValueError): build_return_plan(parent, original, receipt, bad)
        bad = deepcopy(source); bad['calibration_profile']['joints']['zero_offsets_rad'][0] += .001
        with self.assertRaises(ValueError): build_return_plan(parent, original, receipt, bad)
        bad = deepcopy(receipt); bad['motion_outcome'] = 'interrupted'
        with self.assertRaises(ValueError): build_return_plan(parent, original, bad, source)

    def test_machine_continuity_uses_the_actual_parent_fixture_not_vision_estimate(self):
        original, parent, receipt, source, _, _ = fixture()
        current = deepcopy(source)
        actual = deepcopy(original['scene']['environment_from_machine_root'])
        actual['position'][0] += .0071
        current['replay_context'].update(current_machine_pose=deepcopy(actual),parent_fixture_machine_pose=deepcopy(actual))
        plan = build_return_plan(parent, original, receipt, current)
        validate_evidence(plan, current)
        self.assertEqual(current['scene'], original['scene'])
        moved = deepcopy(current); moved['replay_context']['current_machine_pose']['position'][0] += .006
        with self.assertRaises(ValueError): build_return_plan(parent, original, receipt, moved)
        for patch in ({'pose_basis':'historical_vision_estimate'},{'parent_fixture_sha256':None},
                      {'parent_fixture_sha256':'not-a-file-digest'}):
            bad = deepcopy(current); bad['replay_context'].update(patch)
            with self.assertRaises(ValueError): build_return_plan(parent, original, receipt, bad)


if __name__ == '__main__':
    unittest.main()
