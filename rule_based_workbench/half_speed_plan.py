"""CPU-only explicit execution resampling; never new simulation evidence."""
from bisect import bisect_right
from copy import deepcopy
from fractions import Fraction
import math

from frozen_executor import FrozenExecutor, validate_plan_evidence
from integration import _digest
from joint_space import contract
from real_contract import validate_real_plan


WAIT_STAGES = frozenset({"preclose_settle", "close_and_hold", "lift_hold", "support_settle", "release_hold", "home_hold"})


def resample_speed(plan, source, binding, speed_scale=.3):
    """Retain every parent vertex at 30 Hz, interpolating only within original edges.

    Explicit stationary wait stages retain their duration. Motion is slowed;
    binary grippers hold their floor source sample. Cumulative ceiling quantization changes local speed slightly; total
    duration padding stays below 1/30 s, including a short final native tick.
    """
    def check(value):
        return FrozenExecutor(value, source, sink=lambda _:None, gate=lambda:False,
                              stop=lambda _:None,
                              source_validator=lambda p,s:validate_real_plan(p,s,binding))
    geometric = plan.get('simulation') == {'validated':False,'outcome':'not_run'}
    if geometric:
        # Preparation may resample an unsigned geometric candidate, never execute it.
        validate_real_plan(plan,source,binding)
        from gripper_distance_plan import validate_distance_control
        validate_distance_control(plan,source)
        if 'planning_validation' in plan:
            validate_plan_evidence(plan,source)
    else:
        check(plan)
    if 'execution_derivation' in plan:
        raise ValueError('derive only once from the original validated plan')
    if type(speed_scale) not in (int,float) or not math.isfinite(speed_scale) or not .1 <= speed_scale <= 1:
        raise ValueError('speed scale must be finite and between 0.1 and 1.0')
    scale=Fraction(str(speed_scale))
    hz, nominal=plan['control']['physics_hz'],plan['control']['substeps']
    if type(hz) is not int or type(nominal) is not int or nominal <= 0 or hz != 30*nominal:
        raise ValueError('execution resampling requires the original 30 Hz cadence')
    if not geometric and (not plan.get('native_report_sha256') or not isinstance(plan.get('native_identity'),dict)):
        raise ValueError('original native report identity is required')
    if any(not isinstance(a,list) or len(a)!=14 for a in plan['native_actions']):
        raise ValueError('fourteen native action values required')
    steps=plan.get('native_substeps')
    if (not isinstance(steps,list) or len(steps)!=len(plan['ticks'])
            or any(type(n) is not int or not 1<=n<=nominal for n in steps)
            or any(n!=nominal for n in steps[:-1])
            or not math.isclose(plan['duration_s'],sum(steps)/hz,abs_tol=1e-9,rel_tol=0)
            or any(not math.isclose(t['time_s'],i*nominal/hz,abs_tol=1e-9,rel_tol=0) for i,t in enumerate(plan['ticks']))):
        raise ValueError('original native cadence and duration must be consistent')
    stages=deepcopy(plan.get('stages',[]))
    starts=[]
    for stage in stages:
        index=stage.get('start_tick')
        if type(index) is not int or not 0<=index<len(plan['ticks']) or starts and index<=starts[-1]:
            raise ValueError('stage starts must identify strictly increasing original ticks')
        starts.append(index)
    for index,stage in enumerate(stages):
        end=starts[index+1] if index+1<len(starts) else len(plan['ticks'])
        if stage['name'] in WAIT_STAGES:
            reference=plan['ticks'][starts[index]]['target']
            if any(any(t['target'][key]!=reference[key] for key in ('left_arm_joints','right_arm_joints'))
                   for t in plan['ticks'][starts[index]:end]):
                raise ValueError('declared stationary wait contains arm motion')
    boundaries=[Fraction(0)]
    rates=[]
    for i,n in enumerate(plan['native_substeps']):
        stage_index=bisect_right(starts,i)-1
        wait=stage_index>=0 and stages[stage_index]['name'] in WAIT_STAGES
        rate=Fraction(1) if wait else scale
        rates.append(rate)
        boundaries.append(boundaries[-1]+Fraction(n)/rate)
    exact_steps=boundaries[-1]
    count=math.ceil(exact_steps/nominal)
    total_steps=count*nominal
    vertices=[math.ceil(boundary/nominal) for boundary in boundaries[:-1]]
    if vertices[-1]>=count:
        # A short final native tick may end before its rounded-up vertex time.
        # Preserve that vertex with one physics step, still < one control tick late.
        count=vertices[-1]+1
        total_steps=vertices[-1]*nominal+1
    ticks,actions,counts=[],[],[]
    for k in range(count):
        i=min(bisect_right(vertices,k)-1,len(plan['ticks'])-1)
        j=min(i+1,len(plan['ticks'])-1)
        alpha=(k-vertices[i])/(vertices[j]-vertices[i]) if j!=i else 0.0
        a,b=plan['ticks'][i]['target'],plan['ticks'][j]['target']
        target=deepcopy(a)
        if alpha:
            for key in ('left_arm_joints','right_arm_joints'):
                target[key]=[x+(y-x)*alpha for x,y in zip(a[key],b[key])]
            if plan.get('gripper_distance_control',{}).get('schema_version') == 'gripper_distance_control.v1':
                for key in ('left_gripper','right_gripper'):
                    target[key]=a[key]+(b[key]-a[key])*alpha
        left,right=plan['native_actions'][i],plan['native_actions'][j]
        native=[x if slot in (6,13) or not alpha else x+(y-x)*alpha
                for slot,(x,y) in enumerate(zip(left,right))]
        # Distance ramps are motion too. Preserve discrete close/release events
        # at stage boundaries, then interpolate both lanes' physical distance.
        if (alpha and plan.get('native_action_encoding') and
                bisect_right(starts,i)==bisect_right(starts,j)):
            for key,slot in (('left_gripper',6),('right_gripper',13)):
                target[key]=a[key]+(b[key]-a[key])*alpha
                native[slot]=left[slot]+(right[slot]-left[slot])*alpha
        ticks.append(dict(tick=k,time_s=k*nominal/hz,target=target))
        actions.append(native);counts.append(min(nominal,total_steps-k*nominal))
    for stage in stages:
        index=stage.get('start_tick')
        if type(index) is not int or not 0<=index<len(plan['ticks']):
            raise ValueError('stage start must identify an original tick')
        start=vertices[index]
        stage.update(start_tick=start,start_s=ticks[start]['time_s'])
    result=deepcopy(plan)
    result.update(run_id=plan['run_id']+'-speed-'+str(speed_scale)+'-linear',ticks=ticks,
                  native_actions=actions,native_substeps=counts,duration_s=total_steps/hz,
                  timeline_digest=_digest(ticks),stages=stages,
                  joint_space=contract(source['calibration_profile'],actions),
                  execution_derivation=dict(schema_version='execution_derivation.v1',
                      method='vertex_preserving_linear_30hz',speed_scale=speed_scale,
                      parent_vertex_ticks=vertices,
                      maximum_local_speed_scale=max((1/(vertices[i+1]-vertices[i]) for i in range(len(vertices)-1) if rates[i]!=1),default=1.),
                      source_phase='cumulative retimed boundaries rounded up to control ticks; local speed quantized, stationary waits exact',
                      unscaled_wait_stages=sorted(WAIT_STAGES),
                      corner_sampling='every parent tick vertex preserved exactly; interpolation only within each parent edge',
                      gripper_interpolation=('distance_linear_within_stage; stage_boundary_previous_hold'
                                             if plan.get('native_action_encoding') else
                                             'physical_gap_linear; native_binary_intent_previous_hold'
                                             if plan.get('gripper_distance_control') else 'binary_gripper_previous_hold'),
                      duration_quantization_s=float((total_steps-exact_steps)/hz),
                      duration_quantization_bound_s=1/30,
                      endpoint_hold_padding='complete final control tick, or one physics step when needed to preserve a late final vertex',
                      parent_plan_digest=_digest(plan),parent_run_id=plan['run_id'],
                      parent_timeline_digest=plan['timeline_digest'],parent_duration_s=plan['duration_s'],
                      native_report_sha256=plan.get('native_report_sha256'),
                      simulation_evidence_scope='original_parent_plan_only',
                      derived_timeline_simulation_validated=False,hardware_validated=False))
    if geometric:
        result.pop('planning_validation', None)
        if 'planning_validation' in plan:
            receipt=deepcopy(plan['planning_validation'])
            receipt.update(timeline_digest=result['timeline_digest'],derivation=dict(
                method='parent_checked_polyline_subdivision',parent_timeline_digest=plan['timeline_digest'],
                parent_receipt_digest=_digest(plan['planning_validation']),collision_audit_rerun=False))
            result['planning_validation']=receipt
            validate_plan_evidence(result,source)
        result['execution_derivation']['geometric_validation_required']='planning_validation' not in result
        validate_real_plan(result,source,binding)
        validate_distance_control(result,source)
    else:
        check(result)
    return result


def half_speed(plan,source,binding):
    return resample_speed(plan,source,binding,.5)


def resume_initial_open(plan, source, binding, measured_gap_mm, *, minimum_tick=0):
    """Operator-approved resume only within stationary initial opening; keep the checked suffix."""
    validate_real_plan(plan,source,binding)
    validate_plan_evidence(plan,source)
    if 'execution_resume' in plan or type(measured_gap_mm) not in (int,float) or not math.isfinite(measured_gap_mm):
        raise ValueError('one finite measured initial-opening resume required')
    stages=plan['stages']; opening=next(i for i,s in enumerate(stages) if s['name']=='initial_open')
    start,end=stages[opening]['start_tick'],stages[opening+1]['start_tick']
    reference=plan['ticks'][0]['target']
    prefix=plan['ticks'][:end]
    if (any(s['name'] not in ('initial_hold','initial_open') for s in stages[:opening+1])
            or any(a['target']['left_gripper']>b['target']['left_gripper'] for a,b in zip(prefix,prefix[1:]))
            or not reference['left_gripper']<=measured_gap_mm<=prefix[-1]['target']['left_gripper']
            or type(minimum_tick) is not int or not 0<=minimum_tick<end):
        raise ValueError('resume is restricted to unchanged arms and monotonic initial opening')
    cut=next(i for i in range(max(start,minimum_tick),end) if plan['ticks'][i]['target']['left_gripper']>=measured_gap_mm)
    if any(any(t['target'][k]!=reference[k] for k in ('left_arm_joints','right_arm_joints','right_gripper')) for t in prefix[:cut+1]):
        raise ValueError('cannot trim a prefix containing arm motion')
    result=deepcopy(plan); hz=plan['control']['physics_hz']; nominal=plan['control']['substeps']
    result['ticks']=[dict(t,tick=i,time_s=i*nominal/hz) for i,t in enumerate(result['ticks'][cut:])]
    result['native_actions']=result['native_actions'][cut:];result['native_substeps']=result['native_substeps'][cut:]
    result['stages']=[dict(stages[opening],start_tick=0,start_s=0.)]+[
        dict(s,start_tick=s['start_tick']-cut,start_s=(s['start_tick']-cut)*nominal/hz) for s in stages[opening+1:]]
    result.update(run_id=plan['run_id']+'-resume-open-'+str(cut),duration_s=sum(result['native_substeps'])/hz,
                  timeline_digest=_digest(result['ticks']),joint_space=contract(source['calibration_profile'],result['native_actions']))
    result['execution_resume']=dict(schema_version='initial_open_resume.v1',parent_plan_digest=_digest(plan),
        parent_timeline_digest=plan['timeline_digest'],skipped_ticks=cut,measured_metal_gap_mm=measured_gap_mm,
        first_target_digest=_digest(result['ticks'][0]['target']),initial_state_digest=plan['initial_state_digest'])
    receipt=deepcopy(plan['planning_validation'])
    receipt.update(timeline_digest=result['timeline_digest'],execution_resume_digest=_digest(result['execution_resume']),
        derivation=dict(method='checked_stationary_initial_open_suffix',parent_timeline_digest=plan['timeline_digest'],
            parent_receipt_digest=_digest(plan['planning_validation']),collision_audit_rerun=False))
    result['planning_validation']=receipt
    validate_real_plan(result,source,binding);validate_plan_evidence(result,source)
    return result
