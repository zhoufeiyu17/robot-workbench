"""Cheap complete-command motion check; no collision queries or device access."""
import math

from integration import _digest


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
        raise ValueError('complete command velocity/acceleration limit: '+str(checks))
    return result


def validate_receipt(plan):
    if plan.get('control_validation')!=validate_control(plan):
        raise ValueError('complete command motion receipt differs')
