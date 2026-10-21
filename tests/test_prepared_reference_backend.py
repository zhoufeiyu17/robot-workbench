"""Queue admission, clock mapping and cancellation; no SDK or CAN imports."""
import sys
import time
from pathlib import Path
from types import ModuleType,SimpleNamespace as NS
import unittest
from unittest.mock import patch
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'calibration'),str(ROOT/'calibration/infra')]
from examples.scripts.debug.joint_head_real_owner import hardware_backend


class State:
    def __init__(self,n):self.p=np.zeros(n);self.v=np.zeros(n);self.t=np.zeros(n);self.gripper_pos=0.
    def pos(self):return self.p
    def vel(self):return self.v
    def torque(self):return self.t


class Gain:
    def __init__(self,n):self.p=np.zeros(n);self.d=np.zeros(n)
    def kp(self):return self.p
    def kd(self):return self.d


class Controller:
    def __init__(self):self.future=[];self.events=[]
    def get_timestamp(self):return time.monotonic()-100.
    def set_joint_traj(self,frames):self.future=frames;self.events.append('queue')
    def set_joint_cmd(self,state):self.future=[];self.events.append('fixed')
    def set_gain(self,gain):self.events.append('gain')


class Base:
    def __init__(self,config):
        self.config=config;config.gripper_width_m=.082;config.teach_kd_scale=.1
        self.sides=('left','right');self._controllers={s:Controller()for s in self.sides}
        self._controller_configs={s:NS(default_kp=np.array([80,70,70,30,30,20]),default_kd=np.array([2,2,2,1,1,.7]),interpolation_method='linear')for s in self.sides}
        self._arx5=NS(JointState=State,Gain=Gain)
    def get_joint_position_limits(self,side):return np.asarray([[-3,3]]*6+[[0,.082]])
    def _require_connected(self):pass
    def set_damping_hold(self,**kw):
        for controller in self._controllers.values():
            controller.set_joint_cmd(State(6));controller.set_gain(Gain(6))


class PreparedBackendTests(unittest.TestCase):
    def test_diagnostic_gain_is_single_joint_once_only_and_stop_removes_it(self):
        class DiagnosticBase(Base):
            def __init__(self,config):
                super().__init__(config);config.gravity_compensation=True;self.gains={}
                self._robot_configs={side:NS(robot_model='X5')for side in self.sides}
                for side in self.sides:
                    cfg=self._controller_configs[side];cfg.default_gripper_kp=5.;cfg.default_gripper_kd=.2
                    state=State(6);state.gripper_pos=.04
                    self._controllers[side].get_joint_state=lambda state=state:state
                    self._controllers[side].set_gain=lambda gain,side=side:self.gains.__setitem__(side,gain)
            def get_bimanual_joint_position_limits(self):return np.vstack([self.get_joint_position_limits(s)for s in self.sides])
        module=ModuleType('arx_env.backends.arx5_backend');module.Arx5Backend=DiagnosticBase;module.Arx5BackendConfig=lambda **kw:NS(**kw)
        for options,side,index,kp,kd in [({'left_j6_diagnostic':True},'left',5,40.,1.),
                                       ({'right_j4_diagnostic_kp':60.},'right',3,60.,1.),
                                       ({'right_j4_diagnostic_kp':80.,'right_j4_diagnostic_kd':1.6},'right',3,80.,1.6)]:
            with patch.dict(sys.modules,{'arx_env.backends.arx5_backend':module}):b=hardware_backend(disabled_grippers=('right',),**options)
            b.command_guard=lambda:None;q=[0.]*14;q[6]=q[13]=.04
            with self.assertRaisesRegex(ValueError,'exactly one'):b.begin('right'if side=='left'else'left',q)
            b.begin(side,q);expected=np.array([80.,70.,70.,30.,30.,20.]);expected[index]=kp
            np.testing.assert_allclose(b.gains[side].kp(),expected);self.assertEqual(b.gains[side].kd()[index],kd)
            self.assertNotIn('right'if side=='left'else'left',b.gains)
            with self.assertRaisesRegex(ValueError,'exactly one'):b.begin(side,q)
            with self.assertRaisesRegex(ValueError,'does not permit'):b.enable_collection_arm(side,q)
            b.set_damping_hold(manual=False);self.assertFalse(b._joint_diagnostic_gain_active)
            np.testing.assert_allclose(b.gains[side].kp(),np.array([80.,70.,70.,30.,30.,20.])*.3)
            with self.assertRaisesRegex(ValueError,'exactly one'):b.begin(side,q)
        with patch.dict(sys.modules,{'arx_env.backends.arx5_backend':module}):
            for options in ({'left_j6_diagnostic':1},{'left_j6_diagnostic':True,'prepared_reference_enabled':True},
                            {'left_j6_diagnostic':True,'right_j4_diagnostic_kp':60.}):
                with self.assertRaises(ValueError):hardware_backend(disabled_grippers=('right',),**options)

    def test_transport_delay_preserves_torque_ramp_at_admission(self):
        from robot_workbench.motion_reference import ReferenceIntegral
        b=self.backend();integral=ReferenceIntegral()
        for _ in range(3):
            p=self.packet();now=time.monotonic()
            p['torque_bias_sdk6']=integral.packet(now,now,[0.,0.,1.,0.,0.,0.],p['host_times_s'])
            time.sleep(.005)  # Existing native RPC p99 is about 5ms.
            b.queue_reference(p)
            time.sleep(.04)
        b.set_damping_hold(manual=False)
        self.assertEqual(b._prepared_bias_windows,{});self.assertEqual(b._controllers['left'].future,[])

    def backend(self,enabled=True,profile=None):
        module=ModuleType('arx_env.backends.arx5_backend');module.Arx5Backend=Base;module.Arx5BackendConfig=lambda **kw:NS(**kw)
        with patch.dict(sys.modules,{'arx_env.backends.arx5_backend':module}):
            b=hardware_backend(disabled_grippers=('right',),prepared_reference_enabled=enabled,target_interpreter_profile=profile)
        b.command_guard=lambda:None
        return b

    def test_cross_arm_windows_retain_independent_torque_admission_and_fixed_reset(self):
        b=self.backend(profile='x5_shared_precision_v1')
        def packet(side,now,bias):
            values=np.zeros((2,14));offset=0 if side=='left'else 7;values[:,offset+1]=bias
            return dict(schema='workbench.target_window.v1',controller_profile='x5_shared_precision_v1',
                reference_id='a'*64,active_arm=side,host_times_s=[now+.03,now+.08],
                q_sdk14=np.zeros((2,14)).tolist(),dq_sdk14=np.zeros((2,14)).tolist(),torque_bias_sdk14=values.tolist())
        # Accumulate a real bounded ramp on right J2, then switch to left.
        for i in range(1,17):
            now=1000.+i*.1
            with patch('time.monotonic',return_value=now):b.queue_reference(packet('right',now,i*.05))
        with patch('time.monotonic',return_value=1001.7):b.queue_reference(packet('left',1001.7,0.))
        self.assertEqual(set(b._prepared_bias_windows),{'left','right'})
        self.assertAlmostEqual(b._controllers['right'].future[-1].torque()[1],.8)
        # Switching sides must not erase the actual right-arm torque history.
        with patch('time.monotonic',return_value=1001.8),self.assertRaisesRegex(ValueError,'admission slew'):
            b.queue_reference(packet('right',1001.8,0.))
        b._send_fixed('right',[0.]*14)
        self.assertNotIn('right',b._prepared_bias_windows)
        self.assertIn('left',b._prepared_bias_windows)
        with patch('time.monotonic',return_value=1001.81):b.queue_reference(packet('right',1001.81,0.))
        b.set_damping_hold(manual=False);self.assertEqual(b._prepared_bias_windows,{})

    def packet(self):
        now=time.monotonic()
        return dict(schema='prepared_motion_reference.v1',reference_id='a'*64,active_arm='left',
            host_times_s=[now+.03,now+.08],q_sdk14=[[0.]*14,[.01]*6+[0.]*8],dq_sdk14=[[.2]*6+[0.]*8]*2)

    def test_clock_bounded_admission_and_stop_replace_future_before_gains(self):
        b=self.backend();p=self.packet();p['torque_bias_sdk6']=[[0.,0.,.005,0.,0.,0.],[0.,0.,.02,0.,0.,0.]]
        receipt=b.queue_reference(p);controller=b._controllers['left']
        self.assertAlmostEqual(receipt['host_to_sdk_offset_s'],-100.,places=3)
        self.assertEqual(controller.future[1].timestamp, p['host_times_s'][1]+receipt['host_to_sdk_offset_s'])
        np.testing.assert_allclose(controller.future[1].vel(),.2)
        np.testing.assert_allclose(controller.future[1].torque(),[0.,0.,.02,0.,0.,0.])
        self.assertEqual(b._controllers['right'].events,[])
        b.set_damping_hold(manual=False)
        self.assertEqual(b._controllers['left'].future,[])
        self.assertIsNone(b._prepared_reference)
        self.assertLess(b._controllers['left'].events.index('fixed'),b._controllers['left'].events.index('gain'))
        with self.assertRaisesRegex(ValueError,'not enabled'):self.backend(False).queue_reference(self.packet())
        b=self.backend();b._controller_configs['left'].interpolation_method='cubic'
        with self.assertRaisesRegex(ValueError,'qualified native linear'):b.queue_reference(self.packet())

    def test_revocation_racing_native_admission_clears_already_queued_future(self):
        b=self.backend();calls=[]
        def gate():
            calls.append(1)
            if len(calls)==3:raise ValueError('epoch revoked after native admission')
        b.command_guard=gate
        with self.assertRaisesRegex(ValueError,'epoch revoked'):b.queue_reference(self.packet())
        self.assertEqual(b._controllers['left'].future,[])
        self.assertEqual(b._controllers['left'].events[:2],['queue','fixed'])
        self.assertIsNone(b._prepared_reference)

    def test_reject_expired_oversize_nonfinite_and_out_of_bounds_windows(self):
        for change in ('expired','oversize','nan','velocity','position','torque','torque_nan','torque_slew'):
            b=self.backend();p=self.packet()
            if change=='expired':p['host_times_s']=[time.monotonic()-1,time.monotonic()]
            elif change=='oversize':p['host_times_s']*=20;p['q_sdk14']*=20;p['dq_sdk14']*=20
            elif change=='nan':p['q_sdk14'][0][0]=float('nan')
            elif change=='velocity':p['dq_sdk14'][0][0]=2.1
            elif change=='position':p['q_sdk14'][0][0]=3.1
            else:p['torque_bias_sdk6']=[[0.,0.,float('nan')if change=='torque_nan'else .5 if change=='torque_slew'else 1.51,0.,0.,0.]]*2
            with self.subTest(change=change),self.assertRaises(ValueError):b.queue_reference(p)
            self.assertEqual(b._controllers['left'].events,[])


if __name__=='__main__':unittest.main()
