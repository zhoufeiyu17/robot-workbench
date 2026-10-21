"""CPU release contracts: exact v3 targets/clock, legacy bounds and consumer windows."""
from copy import deepcopy
import math
import sys
from types import ModuleType
from unittest.mock import patch

from execution_timeline import prepare
from frozen_executor import FrozenExecutor
from gripper_distance_plan import HOLDING_STAGES, validate_distance_control, validate_release_control, release_targets
from half_speed_plan import resample_speed
from integration import _digest, validate_sim_distance_plan
from joint_space import contract
from test_frozen_executor import fixture
from test_joint_space import a1_profile, physical_fixture
from test_motiongen_pipeline import motiongen_fixture


def release_fixture():
    source, plan = motiongen_fixture()
    closed, opening = 64.992, 82.992
    gaps = [opening, closed]+[closed+(opening-closed)*(1-math.cos(math.pi*i/44))/2 for i in range(45)]+[opening]*7
    target = dict(plan['ticks'][0]['target'], right_gripper=opening)
    plan.update(ticks=[dict(tick=i, time_s=i/30, target=dict(deepcopy(target), left_gripper=gap)) for i,gap in enumerate(gaps)],
        native_actions=[[0.]*6+[gap]+[0.]*6+[opening] for gap in gaps], native_substeps=[8]*len(gaps), duration_s=len(gaps)/30,
        gripper_opening_mm=opening,
        stages=[dict(name=name, start_tick=i) for name,i in [('initial_hold',0),('close_and_hold',1),('open',2),('release_hold',47),('home',53)]],
        gripper_release=dict(schema_version='commanded_gap_release.v3', duration_s=1.5,
            maximum_gap_speed_mm_s=20., initial_model_gap_mm=closed, arm_motion=False))
    plan['timeline_digest'] = _digest(plan['ticks'])
    return source, plan


def check_assisted_release():
    from test_assisted_gripper import assisted_fixture
    from control_motion import validate_control
    from physical_motiongen import freeze_physical, validate_evidence, retime_physical, target_vector, vector_target
    source, model, binding = assisted_fixture()
    gaps, policy = release_targets(60.992, 82.992)
    assert len(gaps) == 53 and policy['schema_version'] == 'commanded_gap_release.v4'
    assert policy['duration_s'] == 53/30
    assert max(b-a for a,b in zip(gaps,gaps[1:]))*30 <= 20.
    old_gaps = [60.992+22*(1-math.cos(math.pi*i/44))/2 for i in range(45)]
    assert max(b-a for a,b in zip(old_gaps,old_gaps[1:]))*30 > 23.5
    reference = model['ticks'][1]['target']
    rows = [deepcopy(model['ticks'][0]['target']),deepcopy(reference)]
    rows += [dict(deepcopy(reference),left_gripper=gap) for gap in gaps]
    rows += [dict(deepcopy(reference),left_gripper=82.992) for _ in range(7)]
    model.update(ticks=[dict(tick=i,time_s=i/30,target=row) for i,row in enumerate(rows)],
        native_actions=[target_vector(row) for row in rows],native_substeps=[8]*len(rows),duration_s=len(rows)/30,
        gripper_opening_mm=82.992,gripper_release=policy,
        stages=[dict(name=name,start_tick=i,motiongen_receipt_index=0)
                for name,i in [('initial_hold',0),('close_and_hold',1),('open',2),('release_hold',55),('home_hold',61)]])
    model['gripper_distance_control']['geometry_derivation']=dict(open_travel_mm=82.992,closed_travel_mm=60.992,
        closed_metal_gap_mm=61.,rubber_gap_mm=58.,native_travel_offset_mm=.008,max_metal_gap_mm=83.)
    model['native_identity']=dict(control=deepcopy(model['control']))
    model['timeline_digest']=_digest(model['ticks']);model['control_validation']=validate_control(model)
    model['planning_validation']['timeline_digest']=model['timeline_digest']
    assert sum(validate_release_control(model)) == 53
    physical = freeze_physical(model,source,binding)
    assert physical['ticks'][2]['target']['left_gripper'] == 61.
    assert physical['ticks'][54]['target']['left_gripper'] == 83.
    assert physical['gripper_release']['initial_model_gap_mm'] == 60.992
    validate_evidence(physical,source)
    assert sum(prepare(physical)['unloading_ticks']) == 53
    follower=FrozenExecutor(physical,source,sink=lambda _:None,gate=lambda:False,stop=lambda _:None,
        source_validator=lambda p,s:None)
    assert sum(follower.unloading_ticks) == 53 and not follower.unloading_ticks[55]
    module=ModuleType('robot_sim.specs.action');module.ac_one_dual_arm_zero_home_action_spec=lambda:None
    with patch.dict(sys.modules,{'robot_sim.specs.action':module}),patch('isaac_lane.native_target',side_effect=lambda raw,*a,**kw:vector_target(raw)):
        for speed in (.3,1.,1.7):
            scaled=retime_physical(physical,source,binding,speed)
            assert sum(validate_release_control(scaled)) == 53
            assert scaled['gripper_release'] == physical['gripper_release']
            validate_evidence(scaled,source)
    # The legacy resampler accepts unsigned geometric candidates, not GPU conversion receipts.
    from integration import SIM_PLANNER_ID
    legacy=deepcopy(physical);legacy['planner_id']=SIM_PLANNER_ID
    legacy.pop('planning_validation');legacy.pop('physical_gpu_conversion')
    for speed in (.3,1.):
        assert sum(validate_release_control(resample_speed(legacy,source,binding,speed))) == 53
    for mutate in (lambda p:p['ticks'][20]['target']['left_arm_joints'].__setitem__(0,.5),
                   lambda p:p['ticks'][20]['target'].update(left_gripper=p['ticks'][20]['target']['left_gripper']+.001),
                   lambda p:p['gripper_release'].update(duration_s=1.5),
                   lambda p:p['gripper_release'].update(maximum_gap_speed_mm_s=24.),
                   lambda p:p['gripper_release'].update(schema_version='commanded_gap_release.v3'),
                   lambda p:p['ticks'][55]['target'].update(right_gripper=80.)):
        bad=deepcopy(model);mutate(bad)
        try:validate_release_control(bad)
        except ValueError:pass
        else:raise AssertionError('v4 release relaxed motion, curve, timing, speed or hold')


def main():
    source, plan = release_fixture()
    assert sum(validate_release_control(plan)) == 45
    assert sum(prepare(plan)['unloading_ticks']) == 45
    assert sum(validate_sim_distance_plan(plan, source)) == 1
    assert max((b['target']['left_gripper']-a['target']['left_gripper'])*30
               for a,b in zip(plan['ticks'][2:46],plan['ticks'][3:47])) < 20.
    mutations = {
        'unknown schema': lambda p: p['gripper_release'].update(schema_version='commanded_gap_release.v5'),
        'unknown field': lambda p: p['gripper_release'].update(curve='linear'),
        'model gap declaration': lambda p: p['gripper_release'].update(initial_model_gap_mm=65.),
        'binary encoding': lambda p: p.update(native_action_encoding='binary'),
        'v2 cannot relabel v3': lambda p: p['gripper_release'].update(schema_version='commanded_gap_release.v2'),
        'v2 cannot use fast curve': lambda p: p.update(gripper_release=dict(schema_version='commanded_gap_release.v2',maximum_step_mm=.2)),
        'duration declaration': lambda p: p['gripper_release'].update(duration_s=1.6),
        'speed declaration': lambda p: p['gripper_release'].update(maximum_gap_speed_mm_s=21.),
        'legacy step declaration': lambda p: p['gripper_release'].update(maximum_step_mm=.2),
        'clock': lambda p: p['ticks'][20].update(time_s=p['ticks'][20]['time_s']+.001),
        'physics clock': lambda p: p['control'].update(physics_hz=480),
        'short native tick': lambda p: p['native_substeps'].__setitem__(20,7),
        'duration': lambda p: p.update(duration_s=p['duration_s']+.001),
        'curve': lambda p: p['ticks'][20]['target'].update(left_gripper=p['ticks'][20]['target']['left_gripper']+.001),
        'nan gap': lambda p: p['ticks'][20]['target'].update(left_gripper=float('nan')),
        'moving arm': lambda p: p['ticks'][20]['target']['left_arm_joints'].__setitem__(0,.001),
        'moving right jaw': lambda p: p['ticks'][20]['target'].update(right_gripper=82.),
        'missing release hold': lambda p: p['stages'][3].update(name='home'),
        'short release hold': lambda p: p['stages'][4].update(start_tick=52),
        'moving release hold': lambda p: p['ticks'][49]['target']['left_arm_joints'].__setitem__(0,.001),
        'nonstationary seam': lambda p: p['ticks'][1]['target']['left_arm_joints'].__setitem__(0,.001),
    }
    for label, mutate in mutations.items():
        bad = deepcopy(plan); mutate(bad)
        if label != 'nan gap': bad['timeline_digest'] = _digest(bad['ticks'])
        for check in (validate_release_control, prepare, lambda p: validate_sim_distance_plan(p,source)):
            try: check(bad)
            except ValueError: pass
            else: raise AssertionError('invalid release accepted: '+label)
    # An exact cosine for a larger range still fails the commanded speed bound.
    bad = deepcopy(plan); bad['gripper_opening_mm'] = 100.
    for i,row in enumerate(bad['ticks'][2:47]):
        row['target']['left_gripper'] = 64.992+(100.-64.992)*(1-math.cos(math.pi*i/44))/2
    try: validate_release_control(bad)
    except ValueError: pass
    else: raise AssertionError('overspeed cosine accepted')

    # Physical conversion keeps metadata in model millimetres while targets use metal gap.
    source, physical = fixture(0.)
    for key in ('lane','adapter','instance_id','gpu'): source.pop(key,None)
    source.update(schema_version='physical_collection_source.v1', evidence_source='physical_rgbd',
        robot_id='fixture', camera_serials=dict(head='h',left='l',right='r'))
    source['calibration_profile'] = a1_profile(source['calibration_profile'])
    physical['calibration_profile_digest'] = _digest(source['calibration_profile'])
    for key in ('ticks','native_actions','native_substeps','duration_s','stages','gripper_release'):
        physical[key] = deepcopy(plan[key])
    physical.update(gripper_opening_mm=82.992, native_action_encoding=plan['native_action_encoding'],
        timeline_digest=_digest(physical['ticks']), native_report_sha256='a'*64, native_identity={},
        gripper_distance_control=dict(schema_version='gripper_distance_control.v2', closed_metal_gap_mm=65.,
            native_travel_offset_mm=.008, rubber_gap_mm=62., controlled_gripper='left_gripper', holding_stages=sorted(HOLDING_STAGES)))
    physical_fixture(physical,source)
    physical['joint_space'] = contract(source['calibration_profile'],physical['native_actions'])
    binding = dict(calibration_accepted=True,evidence_source='physical_rgbd',robot_id=source['robot_id'],
        camera_serials=source['camera_serials'],source_digest=_digest(source),
        calibration_profile_digest=_digest(source['calibration_profile']))
    assert sum(validate_distance_control(physical,source)) == 1
    assert sum(prepare(physical)['unloading_ticks']) == 45
    follower = FrozenExecutor(physical,source,sink=lambda _:None,gate=lambda:True,stop=lambda _:None,
        source_validator=lambda p,s:None)
    assert sum(follower.unloading_ticks) == 45 and not follower.unloading_ticks[47]
    for scale in (.1,.3,1.):
        slow = resample_speed(physical,source,binding,scale)
        assert sum(validate_release_control(slow)) == 45
        assert 'open' in slow['execution_derivation']['unscaled_wait_stages']
    check_assisted_release()
    print('Release v3: frozen cosine/30 Hz/speed, v2 separation, physical conversion, unloading and fixed retiming: PASS')


if __name__ == '__main__': main()
