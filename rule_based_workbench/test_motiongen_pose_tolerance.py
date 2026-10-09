"""Accepted task tolerances are metric distance/angle, not solver cost units."""
import math
from motiongen_motion import task_pose_error

identity = [1., 0., 0., 0.]
rotation = lambda degrees: [math.cos(math.radians(degrees)/2), 0., 0., math.sin(math.radians(degrees)/2)]
assert task_pose_error([.002,0,0], rotation(.5), [0,0,0], identity)[2]
assert not task_pose_error([.0022,0,0], identity, [0,0,0], identity)[2]
assert not task_pose_error([0,0,0], rotation(.6), [0,0,0], identity)[2]
assert task_pose_error([0,0,0], [-1.,0,0,0], [0,0,0], identity)[2]
print('2mm / 0.5deg task pose acceptance: PASS')

from motiongen_motion import normalized_fk_quaternion
q=normalized_fk_quaternion([.99999994,0.,0.,0.])
assert q==(1.,0.,0.,0.)
for bad in ([0.,0.,0.,0.],[2.,0.,0.,0.],[float('nan'),0.,0.,0.]):
    try:normalized_fk_quaternion(bad)
    except ValueError:pass
    else:raise AssertionError('invalid native rotation normalized silently')
print('float32 FK rotation normalization and invalid-rotation rejection PASS')

# Diagnostics must not replace the original planning failure.
from types import SimpleNamespace as NS
from motiongen_motion import MotionGenerator
engine=MotionGenerator.__new__(MotionGenerator)
engine.receipts=[]
engine.generator=NS(rollout_fn=NS(),get_all_rollout_instances=lambda:[])
failed=NS(success=NS(all=lambda:NS(item=lambda:False)),status='FINETUNE_TRAJOPT_FAIL',
          solve_time=0.,total_time=0.,debug_info={'trajopt_result':NS(interpolated_solution=NS(position=object()))})
try:engine._result('probe','plan_goalset',failed,0.)
except ValueError as error:assert 'FINETUNE_TRAJOPT_FAIL' in str(error)
else:raise AssertionError('failed plan accepted')
assert 'failure_diagnostic_error' in engine.receipts[0]
assert engine.receipts[0]['attempts'] is None
failed.attempts=0
failed.graph_time=.012
failed.used_graph=True
try:engine._result('joint-probe','plan_single_js',failed,0.)
except ValueError:pass
else:raise AssertionError('failed joint plan accepted')
assert engine.receipts[-1]['native_attempts']==0 and engine.receipts[-1]['attempts']==1
assert engine.receipts[-1]['used_graph'] and engine.receipts[-1]['graph_time_seconds']==.012
print('failure diagnostics preserve original MotionGen rejection PASS')

class Counter:
    def __init__(self):self.resets=0
    def zero_(self):self.resets+=1
counter,stats=Counter(),Counter()
cost=NS(_region_counters=counter,_region_stats=stats)
rollout=NS(robot_self_collision_cost=cost,robot_self_collision_constraint=cost)
engine.generator=NS(rollout_fn=rollout,get_all_rollout_instances=lambda:[rollout])
engine._reset_region_counters()
assert counter.resets==stats.resets==1
print('per-query capacity counters reset once across shared rollouts PASS')

from motiongen_motion import linear_hold_weights
assert linear_hold_weights(True,True)==(0,[1.,1.,1.,0.,1.,1.])
assert linear_hold_weights(True,linear_axis=1)==(1,[1.,1.,1.,1.,0.,1.])
assert linear_hold_weights(True)==(None,[1.,1.,1.,0.,0.,0.])
for options in [(False,False,1),(True,False,3),(True,False,True),(True,True,1)]:
    try:linear_hold_weights(*options)
    except ValueError:pass
    else:raise AssertionError('invalid linear constraint accepted')
print('native tool-X approach / tool-Y cross masks and invalid-axis rejection PASS')

import numpy as np
from motiongen_motion import segment_errors
origin=np.array([.00236,0.,0.]);goal=np.array([0.,.22,0.])
straight=origin+np.linspace(0,1,25)[:,None]*(goal-origin)
assert segment_errors(straight,origin,goal).max()<1e-12
curved=straight.copy();curved[12,2]+=.0021
assert segment_errors(curved,origin,goal).max()>.002
assert segment_errors(np.array([[.003,0.,0.]]),np.zeros(3),np.zeros(3))[0]==.003
print('rotated start-goal line, real 2mm deviation and degenerate segment PASS')
