"""Cross-stage finite differences, binding and no duplicate collision checks."""
from copy import deepcopy
from control_motion import validate_control,validate_receipt,MotionLimitError
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
    bad.update(stages=[dict(name='approach_pre',start_tick=0)],speed_scale=1.23)
    try:validate_control(bad)
    except MotionLimitError as error:
        assert 'acceleration' in str(error)
        assert '接近杯子的预备动作' in error.failure['message'] and '左 J1' in error.failure['message']
        assert '123%' in error.failure['message'] and '重新生成计划' in error.failure['recovery']
        violation=error.failure['details']['violations'][0]
        assert violation['kind']=='acceleration' and abs(violation['value']-8.1)<1e-10
        assert violation['limit']==5. and error.failure['details']['collision_failure'] is False
    else:raise AssertionError('unacceptable join published')
    bad=deepcopy(plan);bad['control']['max_target_acceleration_rad_s2']=4.
    try:validate_receipt(bad)
    except ValueError:pass
    else:raise AssertionError('stale motion receipt accepted')
    assert plan['control_validation']['collision_audit_rerun'] is False
    print('complete 12-joint command dynamics/joins/receipt binding PASS; no GPU or collision queries')


if __name__=='__main__':main()
