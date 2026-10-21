"""Single-request fixed-right static waiver; no GPU, SDK, motion or geometry edits."""
from copy import deepcopy
import unittest
from unittest.mock import patch

from control_motion import validate_control
from frozen_executor import FrozenExecutor
from integration import _digest
from joint_space import convert_state
from physical_motiongen import freeze_physical,validate_model_evidence,validate_evidence
from real_contract import (fixed_right_collision_policy,skipped_fixed_right_acceptance,
                           validate_fixed_right_hold,validate_real_source,validate_real_plan,fixed_right_hold_reference)
from test_operator_assisted_source import receipt_fixture
from test_assisted_gripper import assisted_fixture


def fixture(gap=61.):
    receipt,profile,binding=receipt_fixture();initial=binding['initial_state']
    initial.update(disabled_grippers=['right'],held_arms=['right'],
        disabled_gripper_model='profile_max_opening_mm_assumption_not_measured')
    initial['planned_reported']['right_gripper']=profile['gripper']['max_opening_mm']
    initial['model']=convert_state(initial['planned_reported'],profile,to_reported=False)
    for sample in initial['owner_samples']:sample['disabled_grippers']=['right']
    receipt['initial_state_digest']=_digest(initial)
    policy=dict(schema_version='operator_fixed_right_collision_policy.v1',mode='skip_fixed_static_checks',
        approved_by='user',approval_id='explicit-current-request-fixture',request_id=receipt['request_id'],
        initial_state_digest=_digest(initial),right_arm_model_joints=deepcopy(initial['model']['right_arm_joints']))
    receipt['fixed_right_collision_policy']=policy
    with patch('test_assisted_gripper.receipt_fixture',return_value=(receipt,profile,binding)):
        source,model,binding=assisted_fixture(gap)
    for tick,action in zip(model['ticks'],model['native_actions']):
        tick['target']['right_gripper']=source['initial_state']['model']['right_gripper']
        action[13]=tick['target']['right_gripper']
    model['timeline_digest']=_digest(model['ticks']);model['control_validation']=validate_control(model)
    validation=model['planning_validation'];validation.update(timeline_digest=model['timeline_digest'],
        static_collision_checked=False,fixed_right_collision_policy_digest=_digest(policy))
    model['fixed_right_collision_policy']=deepcopy(policy)
    validation['gpu_final_acceptance'].update(
        fixed=skipped_fixed_right_acceptance(source,initial['model']['right_arm_joints']),
        static_collision_checked=False,fixed_right_collision_policy_digest=_digest(policy),
        validation_scope='moving_left_and_cup; fixed_right_static_checks_skipped_by_operator')
    return source,model,binding


class FixedRightPolicyTest(unittest.TestCase):
    def test_existing_initial_boundary_projection_is_fixed_not_a_motion_tolerance(self):
        source,_,_=fixture(65.);initial=source['initial_state']
        initial['model']['right_arm_joints'][2]=-.0004525095250023214
        receipt=source['scene']['source']['assistance_receipt'];policy=receipt['fixed_right_collision_policy']
        policy.update(initial_state_digest=_digest(initial),right_arm_model_joints=deepcopy(initial['model']['right_arm_joints']))
        receipt['initial_state_digest']=_digest(initial)
        q=deepcopy(policy['right_arm_model_joints']);q[2]=0.
        proof=dict(simulation_initial_command=dict(schema_version='physical_model_initial_command.v1',
            evidence_source='physical_source.initial_state.model',source_digest=_digest(source),
            initial_state_digest=_digest(initial),target=deepcopy(initial['model'])),
            position_noise_environment='physical',arm_allowance_rad=.01,
            planned_joint_state={f'right_joint1{i+1}':v for i,v in enumerate(q)},
            soft_limit_projection=[dict(joint='right_joint13',measured_rad=policy['right_arm_model_joints'][2],
                projected_rad=0.,lower_rad=0.,upper_rad=3.14)])
        reference=fixed_right_hold_reference(source,proof)
        self.assertEqual(reference['held_model_joints'],q)
        self.assertEqual(reference['measured_model_joints'][2],-.0004525095250023214)
        validate_fixed_right_hold(source,q,proof)
        records=skipped_fixed_right_acceptance(source,q,proof)
        self.assertEqual(records['self']['right_hold_reference'],reference)
        changed=list(q);changed[2]=.0001
        with self.assertRaises(ValueError):validate_fixed_right_hold(source,changed,proof)
        for change in (lambda p:p['soft_limit_projection'][0].update(projected_rad=.0001),
                       lambda p:p['soft_limit_projection'].clear(),
                       lambda p:p.update(arm_allowance_rad=.02),
                       lambda p:p['simulation_initial_command'].update(source_digest='f'*64)):
            bad=deepcopy(proof);change(bad)
            with self.assertRaises(ValueError):fixed_right_hold_reference(source,bad)

    def test_bound_plan_explicitly_skips_and_runtime_requires_disabled_right(self):
        for gap in (61.,65.):
            with self.subTest(gap=gap):
                source,model,binding=fixture(gap);validate_real_source(source,binding)
                validate_model_evidence(model,source)
                physical=freeze_physical(model,source,binding)
                validate_real_plan(physical,source,binding);validate_evidence(physical,source)
                self.assertEqual(physical['ticks'][1]['target']['left_gripper'],gap)
                static=physical['physical_gpu_conversion']['model_validation']['gpu_final_acceptance']['fixed']
                self.assertTrue(all(row['passed'] is None and row['status']=='skipped_by_operator' for row in static.values()))
                self.assertIs(physical['planning_validation']['static_collision_checked'],False)
                options=dict(sink=lambda _:None,gate=lambda:False,stop=lambda _:None,
                             source_validator=lambda p,s:validate_real_plan(p,s,binding))
                with self.assertRaisesRegex(ValueError,'remain disabled'):
                    FrozenExecutor(physical,source,**options)
                FrozenExecutor(physical,source,disabled_grippers=('right',),**options)

    def test_scope_cannot_expand_or_change_the_right_hold(self):
        source,model,binding=fixture(65.);self.assertIsNone(fixed_right_collision_policy({}))
        for edit in (lambda s:s.update(adapter='foundationpose'),lambda s:s.update(environment='fake'),
                     lambda s:s['initial_state'].update(held_arms=[]),
                     lambda s:s['initial_state'].update(disabled_grippers=[]),
                     lambda s:s['grasp_command'].update(metal_gap_mm=61.),
                     lambda s:s['grasp_command'].update(metal_gap_mm=62.),
                     lambda s:s['scene']['source']['assistance_receipt']['fixed_right_collision_policy'].update(request_id='other'),
                     lambda s:s['scene']['source']['assistance_receipt']['fixed_right_collision_policy'].update(initial_state_digest='0'*64),
                     lambda s:s['scene']['source']['assistance_receipt']['fixed_right_collision_policy'].update(approval_id=''),
                     lambda s:s['scene']['source']['assistance_receipt']['fixed_right_collision_policy'].update(approved_by='agent')):
            changed=deepcopy(source);edit(changed)
            with self.subTest(edit=edit),self.assertRaises(ValueError):fixed_right_collision_policy(changed)
        q=deepcopy(source['initial_state']['model']['right_arm_joints']);q[0]+=.001
        with self.assertRaisesRegex(ValueError,'unchanged measured'):validate_fixed_right_hold(source,q)
        changed=deepcopy(model);changed['ticks'][1]['target']['right_arm_joints'][0]+=.001
        changed['timeline_digest']=_digest(changed['ticks']);changed['control_validation']=validate_control(changed)
        changed['planning_validation']['timeline_digest']=changed['timeline_digest']
        with self.assertRaisesRegex(ValueError,'unchanged measured'):validate_model_evidence(changed,source)
        changed=deepcopy(source);changed['scene']['source']['assistance_receipt']['fixed_right_collision_policy']['approval_id']='changed'
        with self.assertRaises(ValueError):validate_real_source(changed,binding)

    def test_active_checks_and_honest_skip_records_are_mandatory(self):
        source,model,_=fixture(65.)
        for edit in (lambda p:p['planning_validation']['gpu_final_acceptance']['fixed']['world'].update(passed=True),
                     lambda p:p['planning_validation']['gpu_final_acceptance']['fixed']['self'].update(performed=True),
                     lambda p:p['planning_validation']['gpu_final_acceptance']['support'].update(passed=False),
                     lambda p:p['planning_validation']['gpu_final_acceptance']['closing'].update(passed=False),
                     lambda p:p['planning_validation']['gpu_final_acceptance']['jaw_sweeps'][0].update(passed=False),
                     lambda p:p['planning_validation']['gpu_final_acceptance']['task_semantics'].update(passed=False),
                     lambda p:p.pop('fixed_right_collision_policy'),
                     lambda p:p['planning_validation'].update(static_collision_checked=True)):
            changed=deepcopy(model);edit(changed)
            with self.subTest(edit=edit),self.assertRaises(ValueError):validate_model_evidence(changed,source)
        ordinary=deepcopy(source);ordinary['scene']['source']['assistance_receipt'].pop('fixed_right_collision_policy')
        with self.assertRaises(ValueError):validate_model_evidence(model,ordinary)


if __name__=='__main__':unittest.main()
