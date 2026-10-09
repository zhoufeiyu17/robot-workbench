"""CPU-only derivation + real contract + frozen follower integration fixture."""
from copy import deepcopy
import math

from frozen_executor import FrozenExecutor
from half_speed_plan import half_speed, resample_speed
from integration import _digest
from real_contract import validate_real_plan
from test_frozen_executor import fixture
from test_harness import rejects
from test_joint_space import physical_fixture


def main():
    for last in (1, 4, 5, 8):
        source, plan = fixture(.001)
        source = {k:deepcopy(v) for k,v in source.items() if k not in ('lane','adapter','instance_id','gpu')}
        source.update(schema_version='physical_collection_source.v1',evidence_source='physical_rgbd',
                      robot_id='cpu-fixture',camera_serials=dict(head='h',left='l',right='r'))
        binding = dict(calibration_accepted=True,evidence_source='physical_rgbd',robot_id=source['robot_id'],
                       camera_serials=source['camera_serials'],source_digest=_digest(source),
                       calibration_profile_digest=_digest(source['calibration_profile']))
        plan.update(native_substeps=[8,8,last],duration_s=(16+last)/240,
                    native_report='fixture-report.json',native_report_sha256='a'*64,
                    native_identity={'fixture_only':True},
                    stages=[dict(name='grasp',start_tick=0,start_s=0.),dict(name='return',start_tick=2,start_s=2/30)])
        physical_fixture(plan,source)
        plan['native_actions']=[[i*.02]*6+[1 if i<2 else -1]+[i*.03]*6+[1] for i in range(3)]
        from joint_space import contract
        plan['joint_space']=contract(source['calibration_profile'],plan['native_actions'])
        before=deepcopy(plan)
        derived=half_speed(plan,source,binding)
        assert plan==before
        assert -1e-14<=derived['duration_s']-2*plan['duration_s']<1/30+1e-14
        assert derived['native_substeps']==[8]*(5 if last<=4 else 6)
        for i in range(3):
            assert derived['ticks'][2*i]['target']==plan['ticks'][i]['target']
            assert derived['native_actions'][2*i]==plan['native_actions'][i]
        for i in range(2):
            middle=derived['ticks'][2*i+1]['target']
            for key in ('left_arm_joints','right_arm_joints'):
                a,b=plan['ticks'][i]['target'][key],plan['ticks'][i+1]['target'][key]
                assert middle[key]==[(x+y)/2 for x,y in zip(a,b)]
                assert all(abs(m-x)<=abs(y-x)/2+1e-14 for x,m,y in zip(a,middle[key],b))
            assert middle['left_gripper']==plan['ticks'][i]['target']['left_gripper']
            assert derived['native_actions'][2*i+1]==[x if j in (6,13) else (x+y)/2 for j,(x,y) in enumerate(zip(plan['native_actions'][i],plan['native_actions'][i+1]))]
        if last>4:
            assert derived['ticks'][-1]['target']==plan['ticks'][-1]['target']
            assert derived['native_actions'][-1]==plan['native_actions'][-1]
        assert derived['native_identity']==plan['native_identity']
        assert derived['stages'][1]['start_tick']==4 and derived['stages'][1]['start_s']==4/30
        assert derived['execution_derivation']['derived_timeline_simulation_validated'] is False
        validate_real_plan(derived,source,binding)
        sent=[]
        follower=FrozenExecutor(derived,source,sink=sent.append,gate=lambda:True,stop=lambda _:None,
                               source_validator=lambda p,s:validate_real_plan(p,s,binding))
        feedback=lambda t:dict(sample_monotonic_s=t,state=deepcopy(sent[-1] if sent else derived['ticks'][0]['target']))
        follower.start(0.,feedback(0.))
        for j in range(1,math.ceil(derived['duration_s']*100)+2):
            now=j/100
            receipt=follower.tick(now,feedback(now))
            if receipt['status']=='completed':break
            assert receipt['status']=='running',receipt
        assert receipt['status']=='completed',receipt
        assert len(receipt['tick_first_send_monotonic_s'])==len(derived['ticks'])
        rejects(lambda p:half_speed(p,source,binding),derived)
        bad=deepcopy(plan);bad['native_substeps']=[8,4,last]
        rejects(lambda p:half_speed(p,source,binding),bad)
        bad=deepcopy(plan);bad['timeline_digest']='wrong'
        rejects(lambda p:half_speed(p,source,binding),bad)
        for scale in (.1,.3,.4,.9,1.):
            slow=resample_speed(plan,source,binding,scale)
            assert slow['ticks'][0]['target']==plan['ticks'][0]['target']
            assert slow['ticks'][-1]['target']==plan['ticks'][-1]['target']
            assert -1e-14<=slow['duration_s']-plan['duration_s']/scale<1/30+1e-14
            assert all(n==8 for n in slow['native_substeps'][:-1]) and 1<=slow['native_substeps'][-1]<=8
            vertices=slow['execution_derivation']['parent_vertex_ticks']
            for i,vertex in enumerate(vertices):
                assert slow['ticks'][vertex]['target']==plan['ticks'][i]['target']
                assert slow['native_actions'][vertex]==plan['native_actions'][i]
            for k,tick in enumerate(slow['ticks']):
                i=max(i for i,v in enumerate(vertices) if v<=k);j=min(i+1,2)
                alpha=(k-vertices[i])/(vertices[j]-vertices[i]) if j!=i else 0
                for key in ('left_arm_joints','right_arm_joints'):
                    expected=[a+(b-a)*alpha for a,b in zip(plan['ticks'][i]['target'][key],plan['ticks'][j]['target'][key])]
                    assert all(abs(a-b)<1e-14 for a,b in zip(tick['target'][key],expected))
                assert tick['target']['left_gripper']==plan['ticks'][i]['target']['left_gripper']
                if k:
                    assert abs(tick['target']['left_arm_joints'][0]-slow['ticks'][k-1]['target']['left_arm_joints'][0])<=.001/math.floor(1/scale)+1e-14
            assert slow['stages'][1]['start_tick']==math.ceil(2/scale)
            validate_real_plan(slow,source,binding)
            sent=[]
            follower=FrozenExecutor(slow,source,sink=sent.append,gate=lambda:True,stop=lambda _:None,
                                   source_validator=lambda p,s:validate_real_plan(p,s,binding))
            fb=lambda t:dict(sample_monotonic_s=t,state=deepcopy(sent[-1] if sent else slow['ticks'][0]['target']))
            follower.start(0.,fb(0.))
            for k in range(1,math.ceil(slow['duration_s']*100)+2):
                now=k/100
                receipt=follower.tick(now,fb(now))
                if receipt['status']=='completed':break
                assert receipt['status']=='running',receipt
            assert receipt['status']=='completed'
        for invalid in (0,.09,1.01,True,float('nan')):
            rejects(lambda _:resample_speed(plan,source,binding,invalid),None)
    # An explicit 18-tick close wait stays 0.6 s at every selectable speed.
    source,base=fixture(.001)
    source={k:deepcopy(v) for k,v in source.items() if k not in ('lane','adapter','instance_id','gpu')}
    source.update(schema_version='physical_collection_source.v1',evidence_source='physical_rgbd',
                  robot_id='wait-fixture',camera_serials=dict(head='h',left='l',right='r'))
    binding=dict(calibration_accepted=True,evidence_source='physical_rgbd',robot_id=source['robot_id'],
                 camera_serials=source['camera_serials'],source_digest=_digest(source),
                 calibration_profile_digest=_digest(source['calibration_profile']))
    ticks=[]
    for i in range(22):
        target=deepcopy(base['ticks'][0]['target'])
        target['left_arm_joints'][0]=.001*(min(i,2) if i<20 else i-18)
        ticks.append(dict(tick=i,time_s=i/30,target=target))
    base.update(ticks=ticks,native_actions=[[0.]*14 for _ in ticks],native_substeps=[8]*22,
                duration_s=22/30,timeline_digest=_digest(ticks),native_report_sha256='a'*64,native_identity={},
                stages=[dict(name='approach',start_tick=0),dict(name='close_and_hold',start_tick=2),dict(name='lift',start_tick=20)])
    physical_fixture(base,source)
    for scale in (.1,.3,1.):
        slow=resample_speed(base,source,binding,scale)
        begin,end=[s['start_s'] for s in slow['stages'][1:]]
        assert abs(end-begin-.6)<1e-12,(scale,begin,end)
        assert abs(slow['execution_derivation']['speed_scale']-scale)<1e-12
    bad=deepcopy(base);bad['ticks'][3]['target']['left_arm_joints'][0]+=.01
    bad['timeline_digest']=_digest(bad['ticks'])
    rejects(lambda p:resample_speed(p,source,binding),bad)
    # A sharp corner must be visited, never cut by a 30% sample spanning two edges.
    corner=deepcopy(base)
    corner['stages']=[dict(name='motion',start_tick=0)]
    for i,tick in enumerate(corner['ticks']):
        tick['target']['left_arm_joints'][:2]=([.001,0.] if i==1 else [0.,.001] if i>=2 else [0.,0.])
    corner['timeline_digest']=_digest(corner['ticks'])
    curved=resample_speed(corner,source,binding,.3)
    vertices=curved['execution_derivation']['parent_vertex_ticks']
    assert vertices[:3]==[0,4,7]
    for i,vertex in enumerate(vertices):assert curved['ticks'][vertex]['target']==corner['ticks'][i]['target']
    assert curved['ticks'][4]['target']['left_arm_joints'][:2]==[.001,0.]
    assert abs(curved['execution_derivation']['maximum_local_speed_scale']-1/3)<1e-12
    # Geometry approval carries forward only from a valid bound parent receipt.
    geometric=deepcopy(corner);geometric['simulation']=dict(validated=False,outcome='not_run')
    unsigned=resample_speed(geometric,source,binding,.3)
    assert 'planning_validation' not in unsigned
    geometric['planning_validation']=dict(schema_version='geometric_path_validation.v1',passed=True,
        timeline_digest=geometric['timeline_digest'],source_digest=_digest(source),
        collision_coverage=['base_link','left','right'],max_sample_step_rad=.01)
    checked=resample_speed(geometric,source,binding,.3)
    from frozen_executor import validate_plan_evidence
    validate_plan_evidence(checked,source)
    receipt=checked['planning_validation']
    assert receipt['timeline_digest']==checked['timeline_digest']
    assert receipt['derivation']['parent_receipt_digest']==_digest(geometric['planning_validation'])
    assert receipt['derivation']['collision_audit_rerun'] is False
    assert checked['simulation']==dict(validated=False,outcome='not_run')
    for key,value in [('timeline_digest','wrong'),('source_digest','wrong'),('passed',False),('max_sample_step_rad',.02)]:
        bad=deepcopy(geometric);bad['planning_validation'][key]=value
        rejects(lambda p:resample_speed(p,source,binding,.3),bad)
    print('30/40/50 percent interpolation, partial ticks, physical contract and fake follower: PASS; no hardware/simulation')


if __name__=='__main__':main()
