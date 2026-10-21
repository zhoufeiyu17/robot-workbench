"""Cheap complete-command motion check; no collision queries or device access."""
import math
from copy import deepcopy

from integration import _digest


class MotionLimitError(ValueError):
    """A valid speed selection can exceed the limits of one particular path."""
    def __init__(self, failure, technical=None):
        self.failure=deepcopy(failure)
        super().__init__(technical or failure['message'])


def motion_failure(plan, checks):
    stages=plan.get('stages') or []
    names={'approach_pre':'接近杯子的预备动作','approach':'接近杯子','lift_withdraw':'抬杯并退出',
           'cross':'移向咖啡机','insert':'进入接液区','lower':'放杯','retreat_insert':'退出接液区','home':'回位'}
    violations=[]
    for side,check in checks.items():
        if check['passed']:continue
        path=[row['target'][side+'_arm_joints'] for row in plan['ticks']]
        velocities=[[0.]*6]+[[(b-a)*30 for a,b in zip(x,y)] for x,y in zip(path,path[1:])]+[[0.]*6]
        accelerations=[[(b-a)*30 for a,b in zip(x,y)] for x,y in zip(velocities,velocities[1:])]
        for kind,values,limit,unit in (('velocity',velocities,check['velocity_limit_rad_s'],'rad/s'),
                                       ('acceleration',accelerations,check['acceleration_limit_rad_s2'],'rad/s²')):
            peak,tick,joint=max((abs(v),i,j) for i,row in enumerate(values) for j,v in enumerate(row))
            if peak<=limit+1e-9:continue
            tick=min(tick,len(path)-1)
            stage=next((s for s in reversed(stages) if s['start_tick']<=tick),{})
            name=stage.get('substage') or stage.get('name')
            violations.append(dict(kind=kind,side=side,joint=joint+1,value=peak,limit=limit,unit=unit,
                tick=tick,stage=name,stage_label=names.get(name,name or '完整轨迹')))
    worst=max(violations,key=lambda v:v['value']/v['limit'])
    metric='加速度' if worst['kind']=='acceleration' else '速度'
    speed=plan.get('speed_scale')
    message=(f"轨迹{metric}超限：{worst['stage_label']}，{'左' if worst['side']=='left' else '右'} J{worst['joint']} "
             f"{worst['value']:.3f} {worst['unit']}，上限 {worst['limit']:g} {worst['unit']}。")
    if speed is not None:message+=f'本次请求速度 {speed*100:g}%。'
    return dict(kind='motion_limit',message=message,
        recovery='请降低运动速度后重新生成计划；限速取决于本次路径。尚未下发这条轨迹，无需恢复控制器或重新预热。',
        details=dict(requested_speed_scale=speed,violations=violations,collision_failure=False))


def validate_control(plan):
    from motiongen_motion import discrete_motion
    control=plan.get('control',{})
    hz,substeps=control.get('physics_hz'),control.get('substeps')
    if type(hz) is not int or type(substeps) is not int or substeps<=0 or hz!=30*substeps:
        raise ValueError('complete command motion validation requires 30 Hz')
    rows=plan.get('ticks')
    if not isinstance(rows,list) or not rows:
        raise ValueError('complete nonempty command timeline required')
    for i,row in enumerate(rows):
        if row.get('tick')!=i or not math.isclose(row.get('time_s',-1),i/30,rel_tol=0,abs_tol=1e-9):
            raise ValueError('motion validation requires ordered 30 Hz targets')
    limits=(control.get('max_target_velocity_rad_s'),control.get('max_target_acceleration_rad_s2'))
    if any(type(v) not in (int,float) or not math.isfinite(v) or v<=0 for v in limits):
        raise ValueError('finite positive frozen velocity/acceleration limits required')
    checks={side:discrete_motion([row['target'][side+'_arm_joints'] for row in rows],*limits)
            for side in ('left','right')}
    result=dict(schema_version='discrete_control_validation.v1',passed=all(c['passed'] for c in checks.values()),
                timeline_digest=_digest(rows),control_digest=_digest(control),checks=checks,
                scope='all 12 planned arm targets, stage joins, initial and final zero velocity; not measured tracking',
                collision_audit_rerun=False)
    if not result['passed']:
        raise MotionLimitError(motion_failure(plan,checks),'complete command velocity/acceleration limit: '+str(checks))
    return result


def validate_receipt(plan):
    if plan.get('control_validation')!=validate_control(plan):
        raise ValueError('complete command motion receipt differs')
