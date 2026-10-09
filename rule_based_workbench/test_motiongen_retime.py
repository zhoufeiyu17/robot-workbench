"""Regression for alternating 1/2-tick slowdown jitter; no robot dependencies."""
import math
from unittest.mock import patch
import numpy as np
from motiongen_plan import scaled_stages


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
        for percent in range(10,101):
            stages,times=scaled_stages([motion,wait],percent/100,None,83.)
            q=np.array([r['left_arm_joints'] for r in stages[0]['rows']])
            assert np.array_equal(q[0],path[0]) and np.array_equal(q[-1],path[-1])
            assert np.abs(np.diff(q,n=2,axis=0)*900).max()<=parent_a+1e-10
            assert stages[1]==wait
            assert np.allclose(np.diff(times[0]),times[0][1],rtol=0,atol=1e-12)
        singleton=dict(name='initial_hold',rows=rows[:1],native_actions=actions[:1])
        assert scaled_stages([singleton],.3,None,83.)[0][0]['rows']==rows[:1]
    print('10-100% uniform clock, bounded acceleration, exact stage endpoints and fixed waits PASS')


if __name__=='__main__':main()
