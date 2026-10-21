"""CPU-only 61 mm assistance contract and one-time model/metal conversion."""
from copy import deepcopy
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'rule_based_workbench')]
from gripper_distance_plan import requested_metal_gap_mm, distance_plan, validate_distance_control
from physical_motiongen import freeze_physical, validate_evidence, reported_target, validate_model_evidence
from source_from_detection import build_assisted_source
from integration import _digest
from joint_space import gripper_offset_mm
from control_motion import validate_control
from real_contract import validate_real_plan
from test_operator_assisted_source import receipt_fixture
from test_physical_motiongen import fake_inputs, bind_contact_fixture


def assisted_fixture(gap=61.):
    receipt,profile,binding=receipt_fixture()
    command=dict(metal_gap_mm=gap,units='metal_inner_gap_mm',controlled_gripper='left_gripper')
    receipt['grasp_command']=deepcopy(command)
    source,_,binding=build_assisted_source(receipt,profile,'assisted-gap-fixture',binding)
    _,model,_=fake_inputs()
    model.update(source_run_id=source['run_id'],source_sequence=source['sequence'],scene_digest=source['scene']['scene_digest'])
    closed=gap-gripper_offset_mm(profile)
    model['ticks'][1]['target']['left_gripper']=closed;model['native_actions'][1][6]=closed
    model['gripper_distance_control'].update(closed_metal_gap_mm=gap,rubber_gap_mm=gap-3.)
    model['timeline_digest']=_digest(model['ticks']);model['control_validation']=validate_control(model)
    model['planning_validation'].update(source_digest=_digest(source),timeline_digest=model['timeline_digest'])
    bind_contact_fixture(model,source)
    certificate=model['grasp_contact_certificate'];region=certificate['grasp_contact_region']
    region['commanded_closed_jaw_m']=closed/2000;region['jaw_breakpoints_m'][-1]=closed/2000
    for row in region['regions']:row['jaw_interval_m'][0]=closed/2000
    region['geometry_digest']=_digest({k:v for k,v in region.items() if k!='geometry_digest'})
    certificate['certificate_digest']=_digest({k:v for k,v in certificate.items() if k!='certificate_digest'})
    model['task_targets']['grasp']['contact_certificate_digest']=certificate['certificate_digest']
    model['planning_validation']['gpu_final_acceptance']['closing']['contact_certificate_digest']=certificate['certificate_digest']
    return source,model,binding


class AssistedGripperTest(unittest.TestCase):
    def test_only_the_same_explicit_physical_receipt_changes_the_default(self):
        source,_,_=assisted_fixture()
        self.assertEqual(requested_metal_gap_mm(source),61.)
        self.assertEqual(requested_metal_gap_mm(assisted_fixture(65.)[0]),65.)
        self.assertEqual(requested_metal_gap_mm({}),65.)
        self.assertEqual(requested_metal_gap_mm(dict(source,adapter='foundationpose')),65.)
        for change in ({'metal_gap_mm':True},{'metal_gap_mm':float('nan')},{'metal_gap_mm':float('inf')},
                       {'metal_gap_mm':62.},{'units':'motor_travel_mm'},{'controlled_gripper':'right_gripper'}):
            bad=deepcopy(source);bad['grasp_command'].update(change)
            bad['scene']['source']['assistance_receipt']['grasp_command']=deepcopy(bad['grasp_command'])
            with self.subTest(change=change),self.assertRaises(ValueError):requested_metal_gap_mm(bad)
        for bad in (dict(source,environment='fake'),dict(source,simulation_only=True),dict(source,grasp_command=None)):
            with self.assertRaises(ValueError):requested_metal_gap_mm(bad)
        bad=deepcopy(source);bad['grasp_command']['metal_gap_mm']=65.
        with self.assertRaises(ValueError):requested_metal_gap_mm(bad)

    def test_generated_61_distance_targets_are_receipt_bound_and_legacy_65_71_remain(self):
        source,base,_=assisted_fixture();base.pop('gripper_distance_control')
        for i,(tick,action) in enumerate(zip(base['ticks'],base['native_actions'])):
            tick['target'].update(left_gripper=83.,right_gripper=83.)
            action[6]=-1 if i==1 else 1;action[13]=1
        plan=distance_plan(base,source,close_metal_gap_mm=61.)
        self.assertEqual(plan['ticks'][1]['target']['left_gripper'],61.)
        self.assertAlmostEqual(plan['native_actions'][1][6],60.992)
        self.assertEqual(plan['gripper_distance_control']['rubber_gap_mm'],58.)
        self.assertEqual(validate_distance_control(plan,source),[False,True,False])
        for gap in (65.,71.,True):
            with self.assertRaises(ValueError):distance_plan(base,source,close_metal_gap_mm=gap)
        bad=deepcopy(plan);bad['gripper_distance_control'].update(closed_metal_gap_mm=65.,rubber_gap_mm=62.)
        with self.assertRaises(ValueError):validate_distance_control(bad,source)
        bad=deepcopy(plan);bad['gripper_distance_control']['units']='motor_travel_mm'
        with self.assertRaises(ValueError):validate_distance_control(bad,source)
        ordinary=dict(source,adapter='foundationpose')
        for gap in (65.,71.):
            legacy=distance_plan(base,ordinary,close_metal_gap_mm=gap)
            self.assertEqual(legacy['ticks'][1]['target']['left_gripper'],gap)
            validate_distance_control(legacy,ordinary)
        with self.assertRaises(ValueError):distance_plan(base,ordinary,close_metal_gap_mm=61.)
        with self.assertRaises(ValueError):validate_distance_control(plan,ordinary)

    def test_motiongen_freeze_exports_the_same_requested_gap_and_release_endpoint(self):
        from motiongen_plan import freeze
        from physical_motiongen import target_vector,vector_target
        source,model,_=assisted_fixture()
        grip=dict(open_travel_mm=82.992,closed_travel_mm=60.992,closed_metal_gap_mm=61.,rubber_gap_mm=58.,
            native_travel_offset_mm=.008,max_metal_gap_mm=83.)
        stages=[dict(name=stage['name'],rows=[tick['target']],native_actions=[target_vector(tick['target'])])
                for stage,tick in zip(model['stages'],model['ticks'])]
        with patch('isaac_lane.native_target',side_effect=lambda raw,*a,**kw:vector_target(raw)):
            frozen=freeze(stages,source,dict(native_identity=dict(control=model['control'])),grip,None,run_id='assisted-freeze')
        self.assertEqual(frozen['gripper_distance_control']['closed_metal_gap_mm'],61.)
        self.assertEqual(frozen['gripper_distance_control']['rubber_gap_mm'],58.)
        self.assertEqual(frozen['gripper_release']['initial_model_gap_mm'],60.992)
        self.assertEqual(frozen['gripper_closing']['target_model_gap_mm'],60.992)

    def test_model_conversion_and_import_keep_61_and_reject_changed_policy(self):
        source,model,binding=assisted_fixture()
        validate_model_evidence(model,source)
        physical=freeze_physical(model,source,binding)
        self.assertEqual(physical['ticks'][1]['target']['left_gripper'],61.)
        self.assertEqual(physical['gripper_distance_control']['closed_metal_gap_mm'],61.)
        self.assertEqual(physical['gripper_distance_control']['rubber_gap_mm'],58.)
        validate_real_plan(physical,source,binding);validate_evidence(physical,source)
        target=deepcopy(model['ticks'][1]['target']);target['left_gripper']+=1e-12
        self.assertEqual(reported_target(target,source['calibration_profile'],source)['left_gripper'],61.)
        bad=deepcopy(model);bad['gripper_distance_control'].update(closed_metal_gap_mm=65.,rubber_gap_mm=62.)
        with self.assertRaises(ValueError):validate_model_evidence(bad,source)
        bad=deepcopy(source);bad['grasp_command']['metal_gap_mm']=65.
        with self.assertRaises(ValueError):validate_evidence(physical,bad)
        ordinary,old_model,old_binding=fake_inputs()
        old=freeze_physical(old_model,ordinary,old_binding)
        self.assertEqual(old['ticks'][1]['target']['left_gripper'],65.)


if __name__=='__main__':unittest.main()
