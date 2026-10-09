"""Cross-stage finite differences, binding and no duplicate collision checks."""
from copy import deepcopy
from control_motion import validate_control,validate_receipt
from test_frozen_executor import fixture


def main():
    source,plan=fixture(.001)
    plan['control'].update(max_target_velocity_rad_s=2.,max_target_acceleration_rad_s2=5.)
    plan['control_validation']=validate_control(plan)
    validate_receipt(plan)
    assert plan['control_validation']['checks']['left']['max_acceleration_rad_s2']<5.
    bad=deepcopy(plan)
    # Each individual two-point segment is tame; this join exceeds acceleration.
    for tick,q in zip(bad['ticks'],(0.,.004,-.001)):tick['target']['left_arm_joints'][0]=q
    try:validate_control(bad)
    except ValueError as error:assert 'acceleration' in str(error)
    else:raise AssertionError('unacceptable join published')
    bad=deepcopy(plan);bad['control']['max_target_acceleration_rad_s2']=4.
    try:validate_receipt(bad)
    except ValueError:pass
    else:raise AssertionError('stale motion receipt accepted')
    assert plan['control_validation']['collision_audit_rerun'] is False
    print('complete 12-joint command dynamics/joins/receipt binding PASS; no GPU or collision queries')


if __name__=='__main__':main()
