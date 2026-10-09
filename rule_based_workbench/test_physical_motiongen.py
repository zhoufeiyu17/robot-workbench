"""Fake GPU receipts/physical inputs only; no inference, network, SDK or motion."""
from copy import deepcopy

from integration import CUROBO_PLANNER_ID, _digest
from joint_space import gripper_offset_mm
from physical_motiongen import freeze_physical, validate_evidence
from real_contract import validate_real_plan
from test_frozen_executor import fixture
from test_initial_state import initial_fixture
from test_joint_space import a1_profile
from gripper_distance_plan import HOLDING_STAGES,NATIVE_DISTANCE_ENCODING
from frozen_executor import FrozenExecutor


def fake_inputs():
    source,plan=fixture()
    for key in ('lane','adapter','instance_id','gpu'):source.pop(key,None)
    profile=a1_profile(source['calibration_profile'])
    profile['model_binding']['urdf_sha256']='a'*64
    profile['joints']['zero_offsets_rad']=[(i-6)*.001 for i in range(12)]
    initial=initial_fixture(profile)
    source.update(schema_version='physical_collection_source.v1',lane='physical-real',
                  evidence_source='physical_rgbd',robot_id='cpu-fixture',camera_serials=dict(head='h',left='l',right='r'),
                  calibration_profile=profile,initial_state=initial)
    start=initial['model'];closed=deepcopy(start);closed['left_gripper']=65.-gripper_offset_mm(profile)
    targets=[start,closed,start]
    plan['control'].update(max_target_velocity_rad_s=2.,max_target_acceleration_rad_s2=5.)
    plan.update(planner_id=CUROBO_PLANNER_ID,source_run_id=source['run_id'],source_sequence=source['sequence'],
        scene_digest=source['scene']['scene_digest'],calibration_profile_digest=_digest(profile),
        ticks=[dict(tick=i,time_s=i/30,target=deepcopy(t)) for i,t in enumerate(targets)],
        native_substeps=[8]*3,duration_s=.1,simulation=dict(validated=False,outcome='not_run'),
        stages=[dict(name=n,start_tick=i,motiongen_receipt_index=0) for i,n in enumerate(('initial_hold','close_and_hold','home_hold'))],
        native_actions=[[0.]*6+[t['left_gripper']]+[0.]*6+[t['right_gripper']] for t in targets],
        native_action_encoding=NATIVE_DISTANCE_ENCODING,
        gripper_distance_control=dict(schema_version='sim_gripper_distance_control.v1',closed_metal_gap_mm=65.,
            rubber_gap_mm=62.,native_travel_offset_mm=.008,holding_stages=sorted(HOLDING_STAGES)),
        planning_backend=dict(backend='curobo_motiongen',curobo_version='0.7.8',gpu_ik=True,
            gpu_trajectory_search=True,native_robot_sha256='a'*64),
        motiongen_receipts=[dict(success=True,path_sha256='b'*64)])
    plan['timeline_digest']=_digest(plan['ticks'])
    from control_motion import validate_control
    plan['control_validation']=validate_control(plan)
    passed=dict(passed=True)
    acceptance=dict(passed=True,fixed=dict(world=passed,self=passed),jaw_sweeps=[passed],
                    closing=passed,support=passed,task_semantics=passed,receiving_goal=passed)
    plan['planning_validation']=dict(schema_version='curobo_motiongen_path_validation.v1',passed=True,
        cpu_search_fallback=False,source_digest=_digest(source),timeline_digest=plan['timeline_digest'],
        gpu_final_acceptance=acceptance,motiongen_receipts_sha256=_digest(plan['motiongen_receipts']),
        selected_motiongen_receipt_indices=[0])
    binding=dict(calibration_accepted=True,evidence_source='physical_rgbd',robot_id=source['robot_id'],
        camera_serials=source['camera_serials'],source_digest=_digest(source),
        calibration_profile_digest=_digest(profile),initial_state=initial)
    return source,plan,binding


def main():
    source,model,binding=fake_inputs();before=deepcopy(model)
    physical=freeze_physical(model,source,binding)
    assert model==before and physical['native_actions']==model['native_actions']
    assert physical['ticks'][1]['target']['left_gripper']==65.
    assert physical['gripper_distance_control']['rubber_gap_mm']==62.
    assert physical['ticks'][0]['target']==source['initial_state']['planned_reported']
    assert physical['timeline_digest']!=model['timeline_digest']
    validate_real_plan(physical,source,binding)
    FrozenExecutor(physical,source,sink=lambda _:None,gate=lambda:False,stop=lambda _:None,
                   source_validator=lambda p,s:validate_real_plan(p,s,binding))
    def rejects(fn):
        try:fn()
        except ValueError:return
        raise AssertionError('invalid physical GPU evidence accepted')
    rejects(lambda:freeze_physical(physical,source,binding))
    for mutate in (lambda p:p['ticks'][1]['target']['left_arm_joints'].__setitem__(0,.2),
                   lambda p:p['native_actions'][1].__setitem__(6,0.),
                   lambda p:p.__setitem__('offline_candidate',True),
                   lambda p:p['physical_gpu_conversion']['model_validation']['gpu_final_acceptance']['support'].__setitem__('passed',False)):
        bad=deepcopy(physical);mutate(bad);rejects(lambda:validate_evidence(bad,source))
    # Even a self-consistent wrapper cannot turn failed geometry into success.
    bad=deepcopy(physical);proof=bad['physical_gpu_conversion'];proof['model_validation']['gpu_final_acceptance']['closing']['passed']=False
    bad['planning_validation']['conversion_digest']=_digest(proof)
    bad['planning_validation']['model_validation_digest']=_digest(proof['model_validation'])
    rejects(lambda:validate_evidence(bad,source))
    bad=deepcopy(source);bad['synthetic_fixture']=True
    rejects(lambda:freeze_physical(model,bad,binding))
    print('physical GPU source/one-time conversion/complete evidence/tamper guards PASS; fake inputs only')


if __name__=='__main__':main()
