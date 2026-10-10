"""30 Hz execution and a measured-start prefix; planning evidence stays intact."""
from bisect import bisect_right
from copy import deepcopy
import math

from contract import ARM_KEYS, GRIPPER_KEYS, validate_physical14
from integration import _digest
from robot_workbench.joint_bounds import position_noise_rad, project_joint_positions

CONTROL_HZ = 30


def prepare(plan):
    """Run off the owner thread. Preserve every parent command and its duration."""
    rows = plan['ticks']
    times = [row['time_s'] for row in rows]
    duration = plan['duration_s']
    if (not rows or times[0] != 0 or any(b <= a for a,b in zip(times,times[1:]))
            or not math.isfinite(duration) or duration <= times[-1]
            or plan['timeline_digest'] != _digest(rows)):
        raise ValueError('sealed ordered parent timeline required')
    from gripper_distance_plan import holding_ticks, validate_release_control
    holding = holding_ticks(plan)
    unloading = validate_release_control(plan)
    starts = [s['start_tick'] for s in plan['stages']]
    commands=[]
    for i,row in enumerate(rows):
        tick=i;now=times[i];target=deepcopy(row['target'])
        validate_physical14(target)
        stage=plan['stages'][bisect_right(starts,i)-1]
        commands.append(dict(tick=tick,time_s=now,target=target,source_tick=i,
            stage=stage['name'],substage=stage.get('substage',stage['name']),holding=holding[i],unloading=unloading[i]))
    return dict(schema_version='execution_timeline.v1',control_hz=CONTROL_HZ,commands=commands,
        times=[row['time_s'] for row in commands],holding_ticks=[row['holding'] for row in commands],
        unloading_ticks=[row['unloading'] for row in commands],
        duration_s=duration,parent_timeline_digest=plan['timeline_digest'],
        base_timeline_digest=_digest(commands),initialization_duration_s=0.,
        derivation='unchanged parent physical targets and times, with an explicit initialization prefix',
        planning_evidence_unchanged=True,collision_audit_rerun=False)


def initialize(base,current_state,limits,*,environment='physical'):
    """Compile off-thread. Measured values remain raw; only issued targets are bounded."""
    from contract import validate_physical_feedback
    validate_physical_feedback(current_state, enforce_range=False)
    if base.get('schema_version')!='execution_timeline.v1' or base.get('control_hz')!=CONTROL_HZ:
        raise ValueError('prepared 30 Hz base required')
    goal=base['commands'][0]['target'];legal=deepcopy(current_state);corrections=[]
    for key in ARM_KEYS:
        if len(limits[key])!=6:raise ValueError('six physical joint limits required')
        legal[key], changes=project_joint_positions(current_state[key],limits[key],environment=environment)
        corrections.extend(dict(row,arm=key) for row in changes)
    for key in GRIPPER_KEYS:
        lo,hi=limits[key];legal[key]=min(hi,max(lo,current_state[key]))
    for key in (*ARM_KEYS,*GRIPPER_KEYS):
        values=goal[key] if key in ARM_KEYS else [goal[key]]
        bounds=limits[key] if key in ARM_KEYS else [limits[key]]
        if any(not math.isfinite(lo) or not math.isfinite(hi) or lo>hi or not lo-1e-12<=v<=hi+1e-12 for v,(lo,hi) in zip(values,bounds)):
            raise ValueError('preview initial target exceeds command limits')
    arm_delta=max(abs(a-b) for key in ARM_KEYS for a,b in zip(legal[key],goal[key]))
    jaw_delta=max(abs(legal[key]-goal[key]) for key in GRIPPER_KEYS)
    duration=max(1.875*arm_delta/.08,math.sqrt(5.773503*arm_delta/5.),1.875*jaw_delta/6.)
    intervals=math.ceil(duration*CONTROL_HZ-1e-9)
    duration=intervals/CONTROL_HZ
    prefix=[]
    for tick in range(intervals):
        t=tick/intervals;s=10*t**3-15*t**4+6*t**5;target=deepcopy(legal)
        for key in ARM_KEYS:target[key]=[a+(b-a)*s for a,b in zip(legal[key],goal[key])]
        for key in GRIPPER_KEYS:target[key]=legal[key]+(goal[key]-legal[key])*s
        prefix.append(dict(tick=tick,time_s=tick/CONTROL_HZ,target=target,source_tick=None,
            stage='initialize',substage='return_to_preview_start',holding=False,unloading=False))
    commands=prefix+[dict(row,tick=row['tick']+intervals,time_s=row['time_s']+duration) for row in base['commands']]
    initialization=dict(actual_start=deepcopy(current_state),legal_start=legal,target=deepcopy(goal),
        command_limits=deepcopy(limits),duration_s=duration,command_projection=legal!=current_state,
        environment=environment,position_noise_rad=position_noise_rad(environment),position_corrections=corrections,
        arm_speed_rad_s=.08,jaw_speed_mm_s=6.)
    return dict(base,commands=commands,duration_s=base['duration_s']+duration,
        times=[row['time_s'] for row in commands],holding_ticks=[row['holding'] for row in commands],
        unloading_ticks=[row['unloading'] for row in commands],
        initialization_duration_s=duration,initialization=initialization,
        timeline_digest=_digest(dict(base_timeline_digest=base['base_timeline_digest'],prefix=prefix,initialization=initialization)),
        digest_kind='base command digest plus initialization prefix and its measured provenance')


def build(plan,*,current_state=None,limits=None,environment='physical'):
    base=prepare(plan)
    if current_state is not None:return initialize(base,current_state,limits,environment=environment)
    return dict(base,timeline_digest=base['base_timeline_digest'],digest_kind='base commands')


def validate(timeline,plan):
    """At file/recording boundaries, reconstruct the declared interpolation off-thread."""
    expected=prepare(plan)
    if timeline.get('initialization'):
        init=timeline['initialization']
        expected=initialize(expected,init['actual_start'],init['command_limits'],environment=init['environment'])
    else:
        expected=dict(expected,timeline_digest=expected['base_timeline_digest'],digest_kind='base commands')
    if timeline!=expected:raise ValueError('execution timeline differs from its sealed parent/initialization')
    return timeline
