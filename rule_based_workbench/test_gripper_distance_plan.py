"""No robot: continuous jaw targets, calibrated bounds and unchanged tracking guard."""
from copy import deepcopy
import math
from frozen_executor import FrozenExecutor
from gripper_distance_plan import distance_plan
from half_speed_plan import resample_speed
from integration import _digest
from joint_space import contract
from real_contract import validate_real_plan
from test_frozen_executor import fixture
from test_joint_space import physical_fixture


def main():
    from contract import validate_physical_feedback,validate_physical14
    source,plan=fixture(.001)
    for gap in (-1.,-.1,83.08613979079334,84.):
        state=dict(plan['ticks'][0]['target'],left_gripper=gap,right_gripper=gap)
        assert validate_physical_feedback(state)==state
    for gap in (-1.01,84.01):
        try:validate_physical_feedback(dict(state,left_gripper=gap))
        except ValueError:pass
        else:raise AssertionError('out-of-envelope feedback accepted')
    try:validate_physical14(dict(state,left_gripper=-.1))
    except ValueError:pass
    else:raise AssertionError('feedback allowance leaked to commands')
    for k in ('lane','adapter','instance_id','gpu'):source.pop(k,None)
    source.update(schema_version='physical_collection_source.v1',evidence_source='physical_rgbd',
                  robot_id='fixture',camera_serials=dict(head='h',left='l',right='r'))
    physical_fixture(plan,source)
    plan.update(native_report_sha256='a'*64,native_identity={'fixture':True})
    plan['native_actions']=[[0.]*6+[1]+[0.]*6+[1] for _ in plan['ticks']]
    plan['joint_space']=contract(source['calibration_profile'],plan['native_actions'])
    binding=dict(calibration_accepted=True,evidence_source='physical_rgbd',robot_id='fixture',
                 camera_serials=source['camera_serials'],source_digest=_digest(source),
                 calibration_profile_digest=_digest(source['calibration_profile']))
    trace=dict(native_report_sha256='a'*64,steps_sha256='b'*64,left_model_gap_mm=[83.,81.5,83.],
               stages=['close','hold','open'],grasp_confirmed=[False,True,False])
    old=deepcopy(plan);fixed=distance_plan(plan,source,trace);assert plan==old
    slow=resample_speed(fixed,source,binding,.3)
    assert abs(slow['ticks'][1]['target']['left_gripper']-82.625)<1e-9
    assert min(t['target']['left_gripper'] for t in slow['ticks'])>81
    sent=[]
    def follower():return FrozenExecutor(slow,source,sink=sent.append,gate=lambda:True,stop=lambda _:None,
                                        source_validator=lambda p,s:validate_real_plan(p,s,binding))
    f=follower()
    def feedback(now):return dict(sample_monotonic_s=now,state=deepcopy(sent[-1] if sent else slow['ticks'][0]['target']))
    f.start(0.,feedback(0.))
    for k in range(1,math.ceil(slow['duration_s']*100)+2):
        r=f.tick(k/100,feedback(k/100))
        if r['status']=='completed':break
        assert r['status']=='running',r
    assert r['status']=='completed'
    sent.clear();f=follower();bad=feedback(0.);bad['state']['left_gripper']=0.
    assert f.start(0.,bad)['status']=='running'
    bad['sample_monotonic_s']=.01
    assert f.tick(.01,bad)['status']=='failed', 'post-command jaw distance protection was bypassed'
    bad=deepcopy(trace);bad['left_model_gap_mm'][1]=float('nan')
    try:distance_plan(plan,source,bad)
    except ValueError:pass
    else:raise AssertionError('nonfinite jaw accepted')
    # The new fixed target is a command, never a measured-gap or force claim.
    source,base=fixture(.001)
    for key in ('lane','adapter','instance_id','gpu'):source.pop(key,None)
    source.update(schema_version='physical_collection_source.v1',evidence_source='physical_rgbd',
                  robot_id='fixture',camera_serials=dict(head='h',left='l',right='r'))
    target=deepcopy(base['ticks'][0]['target']);target['left_gripper']=target['right_gripper']=83.
    base.update(ticks=[dict(tick=i,time_s=i/30,target=deepcopy(target)) for i in range(7)],
                native_substeps=[8]*7,duration_s=7/30,native_report_sha256='a'*64,native_identity={},
                native_actions=[[0.]*6+[-1 if 1<=i<5 else 1]+[0.]*6+[1] for i in range(7)],
                stages=[dict(name='approach',start_tick=0),dict(name='close_and_hold',start_tick=1),
                        dict(name='lift',start_tick=3),dict(name='open',start_tick=5),dict(name='home',start_tick=6)])
    base['timeline_digest']=_digest(base['ticks']);physical_fixture(base,source)
    binding.update(source_digest=_digest(source),calibration_profile_digest=_digest(source['calibration_profile']))
    trace=dict(native_report_sha256='a'*64,steps_sha256='b'*64,left_model_gap_mm=[83.]*7,
               stages=['approach','close_and_hold','close_and_hold','lift','lift','open','home'],
               grasp_confirmed=[False,False,True,True,True,False,False])
    fixed=distance_plan(base,source,trace,close_metal_gap_mm=71)
    narrower=distance_plan(base,source,trace,close_metal_gap_mm=65)
    assert narrower['gripper_distance_control']['rubber_gap_mm']==62
    assert [t['target']['left_gripper'] for t in narrower['ticks']]==[83,65,65,65,65,83,83]
    assert fixed['native_action_encoding']=='normalized_joint_gripper_travel_mm.v1'
    assert [t['target']['left_gripper'] for t in fixed['ticks']]==[83,71,71,71,71,83,83]
    assert [a[6] for a in fixed['native_actions']]==[83,71,71,71,71,83,83]
    assert not fixed['gripper_distance_control']['contact_detection']
    from test_joint_space import a1_profile
    calibrated=deepcopy(source);calibrated['calibration_profile']=a1_profile(source['calibration_profile'])
    a1=distance_plan(base,calibrated,close_metal_gap_mm=71)
    assert abs(a1['native_actions'][1][6]-70.992)<1e-12
    assert abs(a1['native_actions'][1][13]-82.992)<1e-12
    assert a1['native_actions'][1][:6]==base['native_actions'][1][:6]

    from frozen_executor import validate_plan_evidence
    assert fixed['simulation']=={'validated':False,'outcome':'not_run'}
    try:validate_plan_evidence(fixed,source)
    except ValueError:pass
    else:raise AssertionError('unsigned geometry candidate accepted')
    # Synthetic receipt tests validation only; it is not robot/simulation evidence.
    fixed['planning_validation']=dict(schema_version='geometric_path_validation.v1',passed=True,
        timeline_digest=fixed['timeline_digest'],source_digest=_digest(source),
        collision_coverage=['base_link','left','right'],max_sample_step_rad=.01)
    validate_plan_evidence(fixed,source)
    for key,value in [('passed',False),('timeline_digest','wrong'),('source_digest','wrong'),
                      ('collision_coverage',['left','right']),('max_sample_step_rad',.011),
                      ('max_sample_step_rad',float('nan')),('max_sample_step_rad',True)]:
        bad=deepcopy(fixed);bad['planning_validation'][key]=value
        try:validate_plan_evidence(bad,source)
        except ValueError:pass
        else:raise AssertionError('invalid geometry receipt accepted: '+key)
    def executor():
        return FrozenExecutor(fixed,source,sink=lambda _:None,gate=lambda:True,stop=lambda _:None,
                              source_validator=lambda p,s:validate_real_plan(p,s,binding))
    def fb(t,state=None,stamp=None):
        return dict(sample_monotonic_s=t if stamp is None else stamp,state=deepcopy(target if state is None else state))
    def closed():
        f=executor();assert f.start(0.,fb(0.))['status']=='running'
        for t in (.01,.02,.03,.04):assert f.tick(t,fb(t))['status']=='running'
        return f
    measured=deepcopy(target);measured['left_gripper']=77.05
    f=closed();assert f.tick(.05,fb(.05,measured))['status']=='running'
    measured_open=dict(measured,left_gripper=source['calibration_profile']['gripper']['max_opening_mm']+.08613979079334)
    f=closed();assert f.tick(.05,fb(.05,measured_open))['status']=='running','normal open feedback at first close was rejected'
    for change in ({'left_gripper':68.0},{'left_gripper':source['calibration_profile']['gripper']['max_opening_mm']+1.01},{'right_gripper':77.0},
                   {'left_arm_joints':[q+.1 for q in target['left_arm_joints']]}):
        state=dict(measured,**change);f=closed()
        assert f.tick(.05,fb(.05,state))['status']=='failed',change
    f=executor();assert f.start(0.,fb(0.,measured))['status']=='running'
    assert f.tick(.01,fb(.01,measured))['status']=='failed','approach must retain post-command jaw tracking'
    f=closed();assert f.tick(.05,fb(.05,measured,stamp=.03))['status']=='failed','sample-time stage required'
    f=closed()
    for k in range(5,18):assert f.tick(k/100,fb(k/100,measured))['status']=='running'
    assert f.tick(.18,fb(.18,measured))['status']=='failed','open must restore two-sided jaw tracking'
    for scale in (.1,.3,1.):
        slow=resample_speed(fixed,source,binding,scale)
        validate_plan_evidence(slow,source)
        assert slow['planning_validation']['derivation']['collision_audit_rerun'] is False
        assert all(t['target']['left_gripper'] in (71,83) for t in slow['ticks'])
        assert all(a[6]==t['target']['left_gripper'] for a,t in zip(slow['native_actions'],slow['ticks']))
    ramp=deepcopy(fixed)
    ramp.pop('planning_validation')  # Mutated ramp is an unsigned candidate.
    ramp['stages'][-1]['name']='release_hold'
    ramp['ticks'][5]['target']['left_gripper']=77.
    ramp['native_actions'][5][6]=77.
    ramp['stages'].insert(-1,dict(name='open',start_tick=4))
    ramp['stages']=[s for s in ramp['stages'] if s['start_tick']!=5]
    ramp['ticks'][4]['target']['left_gripper']=76.8
    ramp['native_actions'][4][6]=76.8
    ramp['timeline_digest']=_digest(ramp['ticks'])
    ramp['joint_space']=contract(source['calibration_profile'],ramp['native_actions'])
    slow=resample_speed(ramp,source,binding,.5)
    opening=next(s['start_tick'] for s in slow['stages'] if s['name']=='open')
    assert abs(slow['ticks'][opening+1]['target']['left_gripper']-76.9)<1e-9
    assert abs(slow['native_actions'][opening+1][6]-76.9)<1e-9
    assert 'open' not in slow['execution_derivation']['unscaled_wait_stages']
    for mutate in ('encoding','binary','target','stage'):
        bad=deepcopy(fixed)
        if mutate=='encoding':bad['native_action_encoding']='binary'
        elif mutate=='binary':bad['native_actions'][1][6]=-1;bad['joint_space']=contract(source['calibration_profile'],bad['native_actions'])
        elif mutate=='target':bad['ticks'][1]['target']['left_gripper']=72;bad['timeline_digest']=_digest(bad['ticks'])
        else:bad['stages'][1]['start_tick']=0
        try:FrozenExecutor(bad,source,sink=lambda _:None,gate=lambda:True,stop=lambda _:None,
                           source_validator=lambda p,s:validate_real_plan(p,s,binding))
        except ValueError:pass
        else:raise AssertionError('inconsistent distance contract accepted: '+mutate)
    print('Distance gripper interpolation and retained tracking guard: PASS')


if __name__=='__main__':main()
