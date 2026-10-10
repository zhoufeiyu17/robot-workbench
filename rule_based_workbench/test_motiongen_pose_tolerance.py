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

from motiongen_motion import diagonal_waypoints,discrete_motion
start_pose=dict(position=[.01,.02,.03],orientation_wxyz=identity)
goal_pose=dict(position=[.1047,.022477,.025],orientation_wxyz=[-1.,0.,0.,0.])
for ticks in (129,170,90):
    points,progress=diagonal_waypoints(start_pose,goal_pose,ticks/30)
    assert len(points)==len(progress)==ticks and 0<progress[0]<1
    assert progress[-3:]==[1.,1.,1.] and points[-3:]==[points[-1]]*3
    assert all(a<=b for a,b in zip(progress,progress[1:]))
    xyz=np.array([p['position'] for p in points]);begin=np.array(start_pose['position'])
    assert np.allclose((xyz-begin)/(np.array(goal_pose['position'])-begin),np.array(progress)[:,None])
    assert all(p['orientation_wxyz']==identity for p in points)
    assert all(b[2]<=a[2] for a,b in zip(xyz,xyz[1:]))  # No reverse 5mm descent.
for duration in (0.,.05,5.666667,float('inf'),True):
    try:diagonal_waypoints(start_pose,goal_pose,duration)
    except ValueError:pass
    else:raise AssertionError('noninteger or invalid command budget accepted')
print('shared Cartesian progress / signed quaternions / exact budgets / final two commands PASS')

# NumPy stand-ins exercise control flow only; this is not GPU qualification.
from unittest.mock import patch
from pathlib import Path
import hashlib
import tempfile
from integration import _digest
from curobo_planning import LEFT_JOINTS
class Tensor(np.ndarray):
    def __new__(cls,values):return np.asarray(values,dtype=float).view(cls)
    def detach(self):return self
    def cpu(self):return self
    def double(self):return self.astype(float)
    def abs(self):return np.abs(self)
    def clamp(self,low,high):return self.clip(low,high)

def cartesian_fixture(urdf):
    engine=MotionGenerator.__new__(MotionGenerator)
    engine.receipts=[];engine.motion_control_limits=dict(max_velocity_rad_s=2.,max_acceleration_rad_s2=5.)
    locked=dict(left_joint7=.03,left_joint8=.03,right_joint11=.17)
    engine.config=dict(kinematics=dict(urdf_path=str(urdf),lock_joints=locked,base_link='base_link',ee_link='left_link6'))
    engine.tensor_args=NS(to_device=Tensor)
    engine.cup_contact=NS(assert_ready=lambda engine:None)
    engine.torch=NS(acos=np.arccos,linalg=NS(vector_norm=lambda a,dim=None:Tensor(np.linalg.norm(a,axis=dim))),
        cuda=NS(synchronize=lambda:None))
    def fk(rows):
        return NS(ee_position=Tensor(rows)[:,:3],ee_quaternion=Tensor([rotation(math.degrees(row[3])) for row in rows]))
    engine.generator=NS(kinematics=NS(get_state=fk))
    engine._reset_region_counters=lambda:None
    engine.gpu_queries=[]
    def gpu(rows):
        engine.gpu_queries.append(rows)
        return dict(passed=True,samples=len(rows),path_sha256=_digest(rows),fixture=True)
    engine.validate_continuous_path=gpu
    return engine

class Solver:
    calls=[];states=[];fail_batch=0;offset=0.;branch=0.
    def __init__(self,urdf,movable,positions):
        self.states.append(dict(positions))
        assert set(movable)==set(positions)
        self.identity=dict(urdf_sha256=hashlib.sha256(Path(urdf).read_bytes()).hexdigest(),
            base_link='base_link',ee_link='left_link6',fixture=True)
    def batch_ik(self,positions,quaternions,seeds):
        self.calls.append((positions,quaternions,seeds))
        rows=Tensor([[*p,0.,0.,0.] for p in positions])
        if len(rows)>1:
            rows[len(rows)//2,0]+=self.offset
            rows[-1,0]+=self.branch
        return NS(success=np.array([len(self.calls)!=self.fail_batch]*len(rows)),solution=rows,
            position_error=Tensor([0.]*len(rows)),rotation_error=Tensor([0.]*len(rows)))

with tempfile.TemporaryDirectory() as temporary:
    urdf=Path(temporary)/'fixture.urdf';urdf.write_text('<robot name="fixture"/>')
    def solve(engine,goal=goal_pose,duration=3.,end_q=None):
        Solver.calls=[];Solver.states=[]
        return engine.plan_cartesian([*start_pose['position'],0.,0.,0.],goal,name='fixture/insert',
            urdf_path=urdf,movable=(*LEFT_JOINTS,*engine.config['kinematics']['lock_joints']),
            duration_s=duration,end_q=end_q)
    with patch('curobo_planning.A1Kinematics',Solver):
        engine=cartesian_fixture(urdf)
        rows,receipt=solve(engine)
        assert receipt['method']=='A1Kinematics.batch_ik' and receipt['success'] and engine.receipts==[receipt]
        assert len(rows)==90 and rows[-3:]==[[*goal_pose['position'],0.,0.,0.]]*3
        assert rows[0]!=engine.gpu_queries[0][0] and engine.gpu_queries==[[[*start_pose['position'],0.,0.,0.]]+rows]
        assert len(Solver.calls)==2 and len(Solver.calls[1][0])==88
        assert Solver.states[0]['left_joint7']==.03 and Solver.states[0]['left_joint1']==.01
        assert receipt['maximum_waypoint_error_m']<1e-12
        assert receipt['path_sha256']==_digest(rows) and receipt['gpu']['path_sha256']==_digest(engine.gpu_queries[0])
        assert receipt['timing']['control_validation']['passed'] and receipt['timing']['budget_extended'] is False
        assert receipt['task_corridor']['maximum_linear_transverse_error_m']<1e-12
        # A new scene is recomputed, including endpoint IK, with its current locked state.
        engine.config['kinematics']['lock_joints']['left_joint7']=.031
        revised=dict(position=[.055,-.01,.026],orientation_wxyz=identity)
        revised_rows,revised_receipt=solve(engine,revised)
        assert revised_rows[-1][:3]==revised['position'] and len(Solver.calls)==2
        assert receipt['goal_pose_sha256']!=revised_receipt['goal_pose_sha256']
        assert receipt['locked_positions_sha256']!=revised_receipt['locked_positions_sha256']
        rows,explicit=solve(engine,end_q=[*goal_pose['position'],0.,0.,0.])
        assert len(Solver.calls)==1 and explicit['endpoint_seed_source']=='explicit model joint endpoint'
        # Every rejection retains a failed receipt and cannot grow the command budget.
        cases=[('endpoint IK',dict(fail_batch=1)),('waypoint IK',dict(fail_batch=2)),
               ('10um',dict(offset=.000011)),('joint branch',dict(branch=.002))]
        for message,changes in cases:
            engine=cartesian_fixture(urdf)
            with patch.multiple(Solver,**changes):
                try:solve(engine)
                except ValueError as error:assert message in str(error),(message,str(error))
                else:raise AssertionError('Cartesian failure accepted: '+message)
            assert engine.receipts[-1]['success'] is False and not engine.gpu_queries
        for message,options in [('joint motion limits',dict(duration=.1)),
                                ('10um',dict(end_q=[goal_pose['position'][0]+.00002,*goal_pose['position'][1:],0.,0.,0.])),
                                ('0.5deg',dict(goal=dict(position=goal_pose['position'],orientation_wxyz=rotation(.6))))]:
            engine=cartesian_fixture(urdf)
            try:solve(engine,**options)
            except ValueError as error:assert message in str(error),(message,str(error))
            else:raise AssertionError('Cartesian safety gate accepted: '+message)
            assert engine.receipts[-1]['success'] is False and not engine.gpu_queries
        engine=cartesian_fixture(urdf)
        engine.validate_continuous_path=lambda rows:dict(passed=False)
        try:solve(engine)
        except ValueError as error:assert 'GPU path rejected' in str(error)
        else:raise AssertionError('collision failure accepted')
        assert engine.receipts[-1]['success'] is False
        engine=cartesian_fixture(urdf);engine.motion_control_limits=None
        try:solve(engine)
        except ValueError as error:assert 'motion limits' in str(error)
        else:raise AssertionError('unbound control limits accepted')
print('dynamic waypoint IK / model bindings / exact arrival / 10um and motion/collision failure gates PASS (CPU fixture)')
