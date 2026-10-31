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
    def test_arx13_roles_reach_backend_without_renaming_host_interfaces(self):
        module=ModuleType('arx_env.backends.arx5_backend')
        module.Arx5Backend=Base;module.Arx5BackendConfig=lambda **kw:NS(**kw)
        with patch.dict(sys.modules,{'arx_env.backends.arx5_backend':module}):
            backend=hardware_backend(can_interfaces=dict(left='can13l',right='can13r'))
            self.assertEqual((backend.config.left_can,backend.config.right_can),('can13l','can13r'))
            default=hardware_backend()
            self.assertEqual((default.config.left_can,default.config.right_can),('can3','can1'))
            with self.assertRaisesRegex(ValueError,'CAN role'):
                hardware_backend(can_interfaces=dict(left='can13r',right='can13l'))

    def test_guard_delay_does_not_age_a_previously_captured_control_snapshot(self):
        now=[100.]
        class SnapshotBase(Base):
            raw_delay=0.
            def read_calibration_state(self):
                captured=now[0];now[0]+=self.raw_delay
                for controller in self._controllers.values():controller.get_joint_state()
                return dict(qpos=[captured]*14,qvel=[0.]*14,sdk_timestamps=[captured]*2,
                            sample_time_s=captured,sample_wall_ns=1)
        module=ModuleType('arx_env.backends.arx5_backend');module.Arx5Backend=SnapshotBase
        module.Arx5BackendConfig=lambda **kw:NS(**kw)
        with patch.dict(sys.modules,{'arx_env.backends.arx5_backend':module}):b=hardware_backend(disabled_grippers=('right',))
        def guard(action):
            now[0]+=.048
            return dict(heartbeat_monotonic_s=now[0])
        b.output_guard=NS(call=guard);b._telemetry_time=100.;b._controller_telemetry=dict(status='cached')
        for controller in b._controllers.values():controller.get_joint_state=lambda:NS(gripper_torque=0.)
        b.set_damping_hold(manual=False)
        with patch('time.monotonic',side_effect=lambda:now[0]):result=b.read_calibration_state()
        self.assertAlmostEqual(result['sample_time_s'],100.048)
        self.assertAlmostEqual(result['qpos'][0],100.048,'must take a new snapshot, never just retimestamp old data')
        self.assertAlmostEqual(result['feedback_read_timing']['guard_call_s'],.048)
        self.assertEqual(result['gripper_feedback_valid'],[True,False])
        self.assertEqual({r['side']for r in result['feedback_read_timing']['native_calls']},{'left','right'})
        self.assertEqual([r['phase']for r in result['feedback_read_timing']['native_calls']],
                         ['jaw_diagnostic','jaw_diagnostic','raw_snapshot','raw_snapshot'])
        self.assertTrue(all(c._read_probe is None for c in b._controllers.values()))
        b.raw_delay=.06;b._telemetry_time=now[0]
        with patch('time.monotonic',side_effect=lambda:now[0]):slow=b.read_calibration_state()
        self.assertAlmostEqual(now[0]-slow['sample_time_s'],.06,msg='actual slow raw reads must remain stale')
        self.assertAlmostEqual(slow['feedback_read_timing']['raw_snapshot_s'],.06)

    def test_study_gain_configuration_is_ram_only_and_stop_gains_are_independent(self):
        from robot_workbench.execution_settings import study_defaults
        b=self.backend(profile='x5_control50_damping_v1');value=study_defaults()['gains'];value['left']['kp'][3]=70.
        result=b.configure_motion_gains(value);self.assertFalse(result['hardware_write'])
        self.assertEqual(b.motion_gains('left')[0][3],70.)
        self.assertFalse(any(c.events for c in b._controllers.values()))
        captured={}
        for side,c in b._controllers.items():c.set_gain=lambda gain,side=side:captured.__setitem__(side,gain)
        b.set_damping_hold(manual=False)
        np.testing.assert_allclose(captured['left'].kp(),[24,21,21,9,9,6])
        b.configure_motion_gains(None);self.assertEqual(b.motion_gains('left')[0][3],90.)

    def test_terminal_gain_change_keeps_commands_and_bias_and_stops_on_revocation(self):
        from copy import deepcopy
        from robot_workbench.execution_settings import study_defaults
        from robot_workbench.sdk_process import SDKCommandRevoked
        b=self.backend(profile='x5_control50_damping_v1');old=study_defaults()['gains']
        b.configure_motion_gains(old);new=deepcopy(old);new['left']['kp'][2]=.5;new['left']['kd'][2]=.2
        windows={s:(np.array([time.monotonic(),time.monotonic()+1]),np.full((2,6),.3))for s in('left','right')}
        b._prepared_bias_windows=windows
        for c in b._controllers.values():c.future=['unchanged']
        report=b.configure_motion_gains(new,preserve_hold=True)
        self.assertEqual(report['changed_sides'],['left']);self.assertIs(b._prepared_bias_windows,windows)
        self.assertEqual(b._controllers['left'].events,['gain']);self.assertEqual(b._controllers['right'].events,[])
        for c in b._controllers.values():self.assertEqual(c.future,['unchanged'])
        def revoked():raise SDKCommandRevoked('test stop')
        b.command_guard=revoked
        with self.assertRaises(SDKCommandRevoked):b.configure_motion_gains(old,preserve_hold=True)
        self.assertEqual(b.reference_cancel_sequence,1)
        for c in b._controllers.values():self.assertEqual(c.future,[])

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

    def test_expanded_extra_bias_is_admitted_only_for_named_experiment_left_arm(self):
        from robot_workbench.motion_reference import STUDY_BIAS_PROFILE
        for profile,side,allowed in [(STUDY_BIAS_PROFILE,'left',True),(STUDY_BIAS_PROFILE,'right',False),
                                     ('x5_control50_damping_v1','left',True),('x5_control50_damping_v1','right',False),
                                     ('x5_control50_pd30_feedback50_v1','left',False)]:
            b=self.backend(profile=profile);offset=0 if side=='left'else 7
            values=np.zeros((2,14));values[:,offset+2]=2.
            b._prepared_bias_windows={side:(np.array([999.,1001.]),values[:,offset:offset+6])}
            packet=dict(schema='workbench.target_window.v1',controller_profile=profile,reference_id='a'*64,
                active_arm=side,host_times_s=[1000.03,1000.08],q_sdk14=np.zeros((2,14)).tolist(),
                dq_sdk14=np.zeros((2,14)).tolist(),torque_bias_sdk14=values.tolist())
            with patch('time.monotonic',return_value=1000.):
                if allowed:b.queue_reference(packet)
                else:
                    with self.assertRaisesRegex(ValueError,'torque bias exceeds'):b.queue_reference(packet)
            if allowed:
                values[:,offset+2]=2.31 if profile==STUDY_BIAS_PROFILE else 2.001;packet['torque_bias_sdk14']=values.tolist()
                with patch('time.monotonic',return_value=1000.),self.assertRaisesRegex(ValueError,'torque bias exceeds'):
                    b.queue_reference(packet)

    def backend(self,enabled=True,profile=None,native_api='legacy_recompute'):
        module=ModuleType('arx_env.backends.arx5_backend');module.Arx5Backend=Base;module.Arx5BackendConfig=lambda **kw:NS(**kw)
        with patch.dict(sys.modules,{'arx_env.backends.arx5_backend':module}):
            b=hardware_backend(disabled_grippers=('right',),prepared_reference_enabled=enabled,target_interpreter_profile=profile,native_reference_api=native_api)
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
        # This checks the adapter's submitted frames only. Native SDK velocity
        # rewriting and final output require the separate binary qualification.
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

    def test_preserved_reference_routes_explicitly_and_native_refusal_stops_both_arms(self):
        class NativeRejected(Exception):pass
        b=self.backend(profile='x5_control50_damping_v1',native_api='preserve_reference_v1')
        b._arx5.ReferenceAdmissionRejected=NativeRejected
        calls=[]
        for side,c in b._controllers.items():
            def preserve(frames,side=side,c=c):calls.append(side);c.future=frames
            c.set_joint_reference=preserve
            c.set_joint_traj=lambda frames:self.fail('preserved mode fell back to velocity-recomputing API')
        b.set_damping_hold(manual=False)
        packet=dict(schema='workbench.target_window.v1',controller_profile='x5_control50_damping_v1',reference_id='a'*64,
            active_arm='both',host_times_s=[1000.03,1000.08],q_sdk14=np.zeros((2,14)).tolist(),
            dq_sdk14=np.full((2,14),.123).tolist(),torque_bias_sdk14=np.zeros((2,14)).tolist())
        for row in packet['dq_sdk14']:row[6]=row[13]=0.
        with patch('time.monotonic',return_value=1000.):receipt=b.queue_reference(packet)
        self.assertEqual(calls,['left','right']);self.assertEqual(receipt['native_reference_api'],'preserve_reference_v1')
        for side in calls:np.testing.assert_allclose(b._controllers[side].future[0].vel(),.123)
        def reject(frames):raise NativeRejected('native queue capacity')
        b._controllers['right'].controller.set_joint_reference=reject
        from robot_workbench.sdk_process import ReferenceWindowRejected
        with patch('time.monotonic',return_value=1000.),self.assertRaisesRegex(ReferenceWindowRejected,'native queue capacity'):
            b.queue_reference(packet)
        self.assertTrue(all(not c.future for c in b._controllers.values()));self.assertIsNone(b._prepared_reference)

    def test_expired_window_is_a_typed_rejection_with_bounded_clock_diagnostics(self):
        from robot_workbench.sdk_process import ReferenceWindowRejected
        b=self.backend();packet=self.packet();packet['host_times_s']=[time.monotonic()-1,time.monotonic()]
        with self.assertRaises(ReferenceWindowRejected)as caught:b.queue_reference(packet)
        for field in('rows=2','first_lead_ms=','last_lead_ms=','admission_monotonic_s='):
            self.assertIn(field,str(caught.exception))
        self.assertEqual(b._controllers['left'].events,[])
        self.assertEqual(b._controllers['right'].events,[])

    def test_position_only_uses_sdk_recompute_and_retains_admission_and_stop(self):
        b=self.backend(profile='x5_control50_damping_v1',native_api='preserve_reference_v1')
        p=self.packet();p.update(schema='workbench.target_window.v1',active_arm='both',
            controller_profile='x5_control50_damping_v1',velocity_source='sdk_position_only',
            torque_bias_sdk14=np.zeros((2,14)).tolist())
        for c in b._controllers.values():c.set_joint_reference=lambda _:self.fail('wrong native API')
        receipt=b.queue_reference(p)
        self.assertEqual(receipt['native_reference_api'],'legacy_recompute')
        for c in b._controllers.values():
            np.testing.assert_array_equal(c.future[0].vel(),np.zeros(6))
        np.testing.assert_allclose(b._controllers['left'].future[-1].pos(),.01)
        b.set_damping_hold(manual=False)
        self.assertTrue(all(not c.future for c in b._controllers.values()))
        p['velocity_source']='unknown'
        with self.assertRaisesRegex(ValueError,'position-only contract'):b.queue_reference(p)
        p['velocity_source']='sdk_position_only';p['dq_sdk14'][0][0]=3.
        with self.assertRaisesRegex(ValueError,'command bounds'):b.queue_reference(p)
        p['dq_sdk14'][0][0]=.2;p['torque_bias_sdk14'][0][0]=.001
        with self.assertRaisesRegex(ValueError,'added torque bias'):b.queue_reference(p)

    def test_stage_timings_preserve_native_delay_and_stop_after_partial_admission(self):
        from robot_workbench.sdk_process import ReferenceWindowRejected
        b=self.backend(profile='x5_control50_damping_v1');b.set_damping_hold(manual=False)
        now=[1000.];packet=dict(schema='workbench.target_window.v1',controller_profile='x5_control50_damping_v1',
            reference_id='a'*64,active_arm='both',host_times_s=[1000.03,1000.08],
            q_sdk14=np.zeros((2,14)).tolist(),dq_sdk14=np.zeros((2,14)).tolist(),torque_bias_sdk14=np.zeros((2,14)).tolist())
        raw=b._controllers['left'].controller;original=raw.set_joint_traj
        def delayed(frames):original(frames);now[0]+=.04
        raw.set_joint_traj=delayed
        with patch('time.monotonic',side_effect=lambda:now[0]),self.assertRaisesRegex(ReferenceWindowRejected,'multi-arm admission'):
            b.queue_reference(packet)
        timing=b.last_reference_timing
        self.assertEqual(timing['phase'],'rejected_stop');self.assertAlmostEqual(timing['wall_s'],.04)
        self.assertAlmostEqual(next(r['wall_s']for r in timing['stages']if r['phase']=='commit_left'),.04)
        native=next(r for r in timing['calls']if r['method']=='set_joint_traj')
        self.assertEqual(native['side'],'left');self.assertAlmostEqual(native['wall_s'],.04)
        self.assertTrue(all(not c.future and c._read_probe is None for c in b._controllers.values()))
        self.assertIsNone(b._prepared_reference)

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
