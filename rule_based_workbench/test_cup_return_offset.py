"""Synthetic Cartesian solver contracts; no physical or collision qualification."""
from copy import deepcopy
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

from cup_return_replay import build_return_plan,validate_evidence,OFFSET_PLANNER_ID,REPLAY_PLANNER_ID
from cup_return_offset import NativeReturnKinematics
from integration import _digest
from test_cup_return_replay import fixture


class LinearKinematics(NativeReturnKinematics):
    solve_calls=0
    def __init__(self,path,source,world):
        self.spec=NS();self.limits={side:[(-2.,2.)]*6 for side in ('left','right')}
        self.identity=dict(urdf_path=path,urdf_sha256=source['calibration_profile']['model_binding']['urdf_sha256'],
            world_from_robot=world,implementation='CPU linear fixture',implementation_sha256='c'*64,collision_checked=False)
    def pose(self,target):return dict(position=list(target['left_arm_joints'][:3]),orientation_wxyz=[1.,0.,0.,0.])
    def solve(self,goal,target,seed):
        type(self).solve_calls+=1
        return list(goal['position'])+target['left_arm_joints'][3:]


def encode(stage,spec,opening,cache,**kwargs):
    return dict(stage,native_actions=[[*row['left_arm_joints'],row['left_gripper'],*row['right_arm_joints'],row['right_gripper']]
                                    for row in stage['rows']])


def decode(action,spec,opening,*,distance):
    return dict(schema_version='physical14.v1',encoding='absolute',units=dict(joints='rad',grippers='mm'),
        left_arm_joints=action[:6],left_gripper=action[6],right_arm_joints=action[7:13],right_gripper=action[13])


class OffsetTest(unittest.TestCase):
    def setUp(self):
        self.original,self.parent,self.receipt,self.source,self.binding,self.base=fixture()
        self.source['replay_context'].update(return_origin=deepcopy(self.original['scene']['environment_from_cup']),
            offset_xy_mm=[40.,-25.],world_from_robot=dict(position=[0.,0.,0.],orientation_wxyz=[1.,0.,0.,0.]))
        for name,value in (('cup_return_offset.NativeReturnKinematics',LinearKinematics),
                           ('motiongen_plan.encode_stage',encode),('isaac_lane.native_target',decode)):
            patcher=patch(name,value);patcher.start();self.addCleanup(patcher.stop)
        LinearKinematics.solve_calls=0

    def build(self):
        kine=LinearKinematics('fixture.urdf',self.source,self.source['replay_context']['world_from_robot'])
        return build_return_plan(self.parent,self.original,self.receipt,self.source,kinematics=kine)

    def test_local_offset_keeps_other_stages_and_validation_never_reruns_ik(self):
        plan=self.build();count=LinearKinematics.solve_calls
        self.assertGreater(count,0)
        with patch.object(LinearKinematics,'solve',side_effect=AssertionError('Owner must not rerun IK')):
            validate_evidence(plan,self.source)
        self.assertEqual(LinearKinematics.solve_calls,count)
        self.assertEqual(plan['planner_id'],OFFSET_PLANNER_ID)
        proof=plan['replay_derivation'];offset=proof['offset_derivation']
        self.assertEqual(proof['return_origin'],self.original['scene']['environment_from_cup'])
        self.assertEqual(proof['offset_xy_mm'],[40.,-25.])
        self.assertEqual(proof['collision_check']['status'],'not_run')
        self.assertEqual(offset['schema_version'],'cup_return_local_offset.v2')
        self.assertTrue(plan['control_validation']['passed'])
        for index,stage in enumerate(plan['stages']):
            end=plan['stages'][index+1]['start_tick'] if index+1<len(plan['stages']) else len(plan['ticks'])
            parent=stage['base_reverse_stage_index'];old=self.base['stages'][parent]
            old_end=self.base['stages'][parent+1]['start_tick'] if parent+1<len(self.base['stages']) else len(self.base['ticks'])
            rows=plan['ticks'][stage['start_tick']:end];prior=self.base['ticks'][old['start_tick']:old_end]
            if index not in offset['local_stage_indices']:
                self.assertEqual([r['target'] for r in rows],[r['target'] for r in prior])
            elif stage['name'] in ('support_settle','open','release_hold'):
                self.assertEqual(len(rows),len(prior))
                for row,previous in zip(rows,prior):
                    self.assertEqual(row['target']['left_gripper'],previous['target']['left_gripper'])
                    self.assertAlmostEqual(row['target']['left_arm_joints'][0]-previous['target']['left_arm_joints'][0],.04)
                    self.assertAlmostEqual(row['target']['left_arm_joints'][1]-previous['target']['left_arm_joints'][1],-.025)
            if stage['substage']=='return_vertical_lower':
                xy=[row['target']['left_arm_joints'][:2] for row in rows]
                self.assertTrue(all(point==xy[0] for point in xy),'table lowering must have no horizontal motion')
            if stage['substage'] in ('return_hover_alignment','return_unshift_at_clearance'):
                height=[row['target']['left_arm_joints'][2] for row in rows]
                self.assertTrue(all(z==height[0] for z in height),'horizontal transfer must retain clearance')

    def test_zero_effective_offset_preserves_exact_targets_and_clock(self):
        self.source['replay_context']['offset_xy_mm']=[0.,0.]
        plan=build_return_plan(self.parent,self.original,self.receipt,self.source)
        self.assertEqual(plan['planner_id'],REPLAY_PLANNER_ID)
        for key in ('ticks','native_actions','native_substeps','stages','duration_s'):
            self.assertEqual(plan[key],self.base[key])
        self.assertEqual(LinearKinematics.solve_calls,0)
        validate_evidence(plan,self.source)

    def test_zero_user_offset_returns_to_fixed_origin_after_previous_offset_cycle(self):
        self.source['replay_context']['offset_xy_mm']=[0.,0.]
        self.source['replay_context']['return_origin']['position'][0]-=.03
        plan=self.build()
        self.assertEqual(plan['planner_id'],OFFSET_PLANNER_ID)
        self.assertAlmostEqual(plan['replay_derivation']['effective_delta_m'][0],-.03)
        self.assertEqual(plan['replay_derivation']['offset_xy_mm'],[0.,0.])

    def test_changed_fixed_solution_clock_model_and_out_of_range_offset_rejected(self):
        plan=self.build()
        for mutate in (lambda p:p['replay_derivation']['offset_derivation']['model_joint_solutions'][5].__setitem__(0,.4),
                       lambda p:p['replay_derivation']['offset_derivation']['kinematics'].__setitem__('urdf_sha256','wrong'),
                       lambda p:p['ticks'][700 if len(p['ticks'])>700 else 40].__setitem__('time_s',99.),
                       lambda p:p['replay_derivation']['offset_derivation'].__setitem__('hover_settle_s',0.)):
            bad=deepcopy(plan);mutate(bad)
            with self.assertRaises(ValueError):validate_evidence(bad,self.source)
        for invalid in ([50.01,0.],[0.,float('nan')],['5',0.],None):
            self.source['replay_context']['offset_xy_mm']=invalid
            with self.assertRaises(ValueError):self.build()

    def test_successful_faster_parent_uses_its_existing_dynamic_limits(self):
        self.source['replay_context']['offset_xy_mm']=[0.,0.]
        self.parent['normal_recipe']['requested_speed_scale']=1.7
        plan=build_return_plan(self.parent,self.original,self.receipt,self.source)
        self.assertTrue(plan['control_validation']['passed'])
        self.assertEqual(plan['control'],self.parent['control'])


if __name__=='__main__':unittest.main()
