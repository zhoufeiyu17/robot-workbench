"""Position projection has one policy; raw values and motion gates stay separate."""
from copy import deepcopy
from pathlib import Path
import json
import sys
from tempfile import TemporaryDirectory
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'rule_based_workbench')]
from robot_workbench.joint_bounds import project_joint_positions
from joint_space import project_model_state, reported_command
from execution_timeline import build, validate
from test_frozen_executor import fixture


class PositionBoundsTests(unittest.TestCase):
    def test_policy_boundary_and_raw_evidence(self):
        for environment, allowance in (('fake',.005),('physical',.01)):
            raw=[-allowance,1+allowance]
            bounded,changes=project_joint_positions(raw,[(0,1)]*2,environment=environment)
            self.assertEqual(bounded,[0,1]);self.assertEqual(raw,[-allowance,1+allowance])
            self.assertEqual([row['original_rad'] for row in changes],raw)
            for excess in (-allowance-.000001,1+allowance+.000001,float('nan'),float('inf'),True):
                with self.assertRaises(ValueError):project_joint_positions([excess],[(0,1)],environment=environment)
        with self.assertRaises(ValueError):project_joint_positions([0],[(0,1)],environment='unknown')

    def test_conversion_projects_arms_only(self):
        arm=NS(soft_lower_limits_rad=[0.]*6,soft_upper_limits_rad=[1.]*6)
        spec=NS(left=NS(arm=arm),right=NS(arm=arm))
        raw=dict(left_arm_joints=[-.007]*6,right_arm_joints=[1.007]*6,left_gripper=83.74,right_gripper=-1.)
        before=deepcopy(raw)
        result,changes=project_model_state(raw,spec,environment='physical')
        self.assertEqual(raw,before);self.assertEqual(len(changes),12)
        self.assertEqual(result['left_gripper'],83.74);self.assertEqual(result['right_gripper'],-1.)
        with self.assertRaises(ValueError):project_model_state(raw,spec,environment='fake')

    def test_execution_prefix_is_bounded_and_suffix_immutable(self):
        _,plan=fixture();plan['stages']=[dict(name='home',start_tick=0)];original=deepcopy(plan)
        current=deepcopy(plan['ticks'][0]['target']);current['left_arm_joints'][0]=-.007
        limits={side+'_arm_joints':[[-3.,3.]]*6 for side in ('left','right')}
        limits['left_arm_joints'][0]=[0.,3.]
        limits.update(left_gripper=[0.,83.],right_gripper=[0.,83.])
        result=build(plan,current_state=current,limits=limits,environment='physical')
        validate(result,plan)
        self.assertEqual(result['initialization']['actual_start'],current)
        self.assertEqual(result['initialization']['legal_start']['left_arm_joints'][0],0.)
        suffix=[row for row in result['commands'] if row['source_tick'] is not None]
        self.assertEqual([row['target'] for row in suffix],[row['target'] for row in plan['ticks']])
        self.assertEqual(plan,original)
        with self.assertRaises(ValueError):build(plan,current_state=current,limits=limits,environment='fake')
        result['initialization']['position_corrections'][0]['delta_rad']=0.
        with self.assertRaises(ValueError):validate(result,plan)

    def test_model_and_owner_domains_are_normalized_before_encoding(self):
        _,plan=fixture()
        source,_=fixture()
        source['calibration_profile']['joints']['zero_offsets_rad']=[.002]*12
        source['initial_state']={'reported_joint_limits':{side+'_arm_joints':[[0.,1.]]*6 for side in ('left','right')}}
        arm=NS(soft_lower_limits_rad=[0.]*6,soft_upper_limits_rad=[1.]*6)
        spec=NS(left=NS(arm=arm),right=NS(arm=arm))
        raw=deepcopy(plan['ticks'][0]['target'])
        raw.update(left_arm_joints=[0.]*6,right_arm_joints=[0.]*6)
        normalized,changes=project_model_state(raw,spec,environment='fake',source=source)
        self.assertEqual(normalized['left_arm_joints'],[.002]*6)
        self.assertEqual(len(changes),12)
        self.assertEqual(reported_command(normalized,source)['left_arm_joints'],[0.]*6)
        with self.assertRaises(ValueError):reported_command(raw,source)

    def test_remote_reconstruction_preserves_fake_policy_and_digest(self):
        import playground_pipeline as pipeline
        _,plan=fixture();plan['stages']=[dict(name='home',start_tick=0)]
        current=deepcopy(plan['ticks'][0]['target']);current['left_arm_joints'][0]=-.004
        limits={side+'_arm_joints':[[-3.,3.]]*6 for side in ('left','right')}
        limits['left_arm_joints'][0]=[0.,3.]
        limits.update(left_gripper=[0.,83.],right_gripper=[0.,83.])
        timeline=build(plan,current_state=current,limits=limits,environment='fake')
        with TemporaryDirectory() as folder:
            parent=Path(folder)/'plan.json';parent.write_text(json.dumps(plan))
            config=dict(workbench_code=str(ROOT/'rule_based_workbench'),remote_run_root=folder)
            with patch.object(pipeline,'python_command',side_effect=lambda script,*args:[sys.executable,'-B','-c',script,*args]), \
                    patch.object(pipeline,'transport',side_effect=lambda cfg,side,argv:argv):
                for _ in range(2):
                    path=pipeline.prepare_remote_runtime(config,str(parent),timeline)
                    self.assertEqual(json.loads(Path(path).read_text()),timeline)


if __name__=='__main__':unittest.main()
