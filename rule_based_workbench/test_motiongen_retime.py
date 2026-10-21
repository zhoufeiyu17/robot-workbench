"""Regression for alternating 1/2-tick slowdown jitter; no robot dependencies."""
import math
from unittest.mock import patch
import numpy as np
from motiongen_plan import scaled_stages,check_requested_speed
from control_motion import validate_control,MotionLimitError


def decode(action, spec, opening, distance):
    return dict(left_arm_joints=action[:6],right_arm_joints=action[7:13],
                left_gripper=action[6],right_gripper=action[13])


def main():
    u=np.linspace(0,1,121)
    path=(10*u**3-15*u**4+6*u**5)[:,None]*np.array([[1.,-.5,.7,0.,.1,.3]])
    actions=[[*q,65.,*[0.]*6,83.] for q in path.tolist()]
    rows=[decode(a,None,None,True) for a in actions]
    motion=dict(name='transport',rows=rows,native_actions=actions)
    wait=dict(name='close_and_hold',rows=[rows[-1]]*18,native_actions=[actions[-1]]*18)
    parent_a=np.abs(np.diff(path,n=2,axis=0)*900).max()
    with patch('isaac_lane.native_target',side_effect=decode):
        for percent in range(10,171):
            speed=percent/100
            stages,times=scaled_stages([motion,wait],speed,None,83.)
            q=np.array([r['left_arm_joints'] for r in stages[0]['rows']])
            assert np.array_equal(q[0],path[0]) and np.array_equal(q[-1],path[-1])
            if speed<=1.:
                assert np.abs(np.diff(q,n=2,axis=0)*900).max()<=parent_a+1e-10
            assert stages[1]==wait
            assert np.allclose(np.diff(times[0]),times[0][1],rtol=0,atol=1e-12)
            assert len(q)==math.ceil((len(rows)-1)/speed)+1
            if speed==1.:assert stages[0]['rows']==rows
            plan=dict(control=dict(physics_hz=240,substeps=8,max_target_velocity_rad_s=2.,max_target_acceleration_rad_s2=5.),
                      ticks=[dict(tick=i,time_s=i/30,target=row) for i,row in enumerate(stages[0]['rows']+stages[1]['rows'])])
            assert validate_control(plan)['passed']
            if speed==1.7:
                plan['control']['max_target_velocity_rad_s']=.01
                try:validate_control(plan)
                except ValueError as error:assert 'velocity/acceleration limit' in str(error)
                else:raise AssertionError('speed selection must not relax the frozen hard limits')
        opening=dict(wait,name='open')
        assert scaled_stages([opening],1.7,None,83.,fixed_stages={'open'})[0][0]==opening
        singleton=dict(name='initial_hold',rows=rows[:1],native_actions=actions[:1])
        assert scaled_stages([singleton],.3,None,83.)[0][0]['rows']==rows[:1]
        # An allowed percentage is not a promise that this particular path can accelerate.
        u=np.linspace(0,1,31)
        fast_actions=[[0.,float(.8*(10*t**3-15*t**4+6*t**5)),0.,0.,0.,0.,83.,*[0.]*6,83.] for t in u]
        fast=dict(name='approach_pre',rows=[decode(a,None,None,True) for a in fast_actions],native_actions=fast_actions)
        limits=dict(physics_hz=240,substeps=8,max_target_velocity_rad_s=2.,max_target_acceleration_rad_s2=5.)
        check_requested_speed([singleton,fast],1.,limits,None,83.)
        try:check_requested_speed([singleton,fast],1.23,limits,None,83.)
        except MotionLimitError as error:
            assert error.failure['details']['requested_speed_scale']==1.23
            assert error.failure['details']['violations'][0]['joint']==2
        else:raise AssertionError('overspeed prefix allowed subsequent task planning')
    print('30-170% user range / 10% internal slowdown, exact endpoints/fixed waits and unchanged hard limits PASS')


if __name__=='__main__':main()
