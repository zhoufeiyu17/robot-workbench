"""Native-constructor boundary, with an inert SDK double; never opens CAN."""
from pathlib import Path
import os
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'tests')]
from test_owner_clock import calibration
from robot_workbench.release_trace import ReleaseTrace
from robot_workbench import arx_output_guard,sdk_quiesce


class BackendGuardTests(unittest.TestCase):
    def test_telemetry_reads_native_command_gains_and_effort_without_writes(self):
        backend,guard,events=self.fixture(['right'])
        backend.sides=('left','right')
        state=lambda:NS(pos=lambda:[.01]*6,torque=lambda:[.4]*6,gripper_torque=0.)
        command=lambda:NS(pos=lambda:[0.]*6,vel=lambda:[.2]*6,torque=lambda:[.3]*6)
        gain=lambda:NS(kp=lambda:[30.]*6,kd=lambda:[1.]*6)
        backend._controllers={side:NS(get_joint_state=state,get_joint_cmd=command,get_gain=gain)for side in ('left','right')}
        backend.output_guard=guard
        base=type(backend).__bases__[0]
        with patch.object(base,'read_calibration_state',return_value={},create=True),patch.object(calibration.time,'monotonic',return_value=10.):
            first=backend.read_calibration_state()['controller_telemetry']
            self.assertEqual(first['status'],'observed');self.assertEqual(first['arms']['right']['output_qpos'],[0.]*6)
            self.assertEqual(first['arms']['right']['qpos'],[.01]*6)
            self.assertEqual(first['arms']['right']['output_qvel'],[.2]*6)
            self.assertEqual(first['arms']['right']['effort'],[.4]*6)
            backend._controllers['right'].get_joint_cmd=lambda:(_ for _ in ()).throw(ValueError('unavailable'))
            self.assertIs(backend.read_calibration_state()['controller_telemetry'],first)
        with patch.object(base,'read_calibration_state',return_value={},create=True),patch.object(calibration.time,'monotonic',return_value=10.2):
            self.assertEqual(backend.read_calibration_state()['controller_telemetry']['status'],'unavailable')
        self.assertTrue(all(e==('guard','heartbeat')for e in events))

    def fixture(self,disabled):
        events=[]
        def create(robot,controller,interface):
            events.append(('native',interface,robot.gripper_motor_type))
            class Controller:
                def __del__(self):events.append(('native_closed',interface))
            return Controller()
        sdk=NS(MotorType=NS(NONE='none'),Arx5JointController=create,
            __file__=__file__,workbench_build_id='a'*64)
        class Base:
            def __init__(self,config):self.config=config;self._controllers={};self.is_connected=False;self._robot_configs={};self._controller_configs={}
            def _import_arx5(self):return sdk
            def connect(self):
                self._arx5=self._import_arx5()
                for side in ('left','right'):
                    robot=NS(gripper_motor_type='DM');interface=getattr(self.config,side+'_can')
                    self._controllers[side]=self._arx5.Arx5JointController(robot,None,interface)
                self.is_connected=True
            def set_to_damping(self):events.append(('damping',))
            def close(self):self._controllers.clear();self.is_connected=False
        module=NS(Arx5Backend=Base,Arx5BackendConfig=lambda **kw:NS(**kw))
        with patch.dict(sys.modules,{'arx_env.backends.arx5_backend':module}):
            sides=['left'] if disabled is True else [] if disabled is False else list(disabled)
            backend=calibration.hardware_backend(disabled_grippers=sides)
        guard=NS(disabled_grippers=sides,call=lambda action:events.append(('guard',action)))
        return backend,guard,events

    def test_guard_ack_precedes_native_construction_and_left_none_precedes_enable(self):
        backend,guard,events=self.fixture(True)
        with patch('robot_workbench.arx_output_guard.require_guard',return_value=guard):backend.connect()
        self.assertEqual(events,[('guard','begin'),('native','can3','none'),('native','can1','DM'),('guard','activate')])
        backend.close()
        self.assertEqual(events[-7:],[('guard','releasing'),('damping',),('native_closed','can3'),('guard','retire_left'),('native_closed','can1'),('guard','retire_right'),('guard','disarm')])

    def test_rejected_guard_cannot_construct_any_controller(self):
        backend,guard,events=self.fixture(False)
        def reject(action):raise ValueError('actual motor feedback missing')
        guard.call=reject
        with patch('robot_workbench.arx_output_guard.require_guard',return_value=guard),self.assertRaises(ValueError):backend.connect()
        self.assertEqual(events,[])
        with patch('robot_workbench.arx_output_guard._client',None),self.assertRaisesRegex(RuntimeError,'独立 CAN'):backend.connect()
        self.assertEqual(events,[])

    def test_normal_manual_hold_preserves_each_native_jaw_write(self):
        backend,_,_=self.fixture(False)
        import numpy as np
        writes=[]
        class Gain:
            def __init__(self,_):
                self.p=np.zeros(6);self.d=np.zeros(6)
                self.gripper_kp=self.gripper_kd=0.
            def kp(self):return self.p
            def kd(self):return self.d
        class Controller:
            def set_gain(self,gain):writes.append(('gain',gain.gripper_kp,gain.gripper_kd))
            def set_joint_cmd(self,command):writes.append(('command',command.gripper_pos))
        backend._arx5=NS(Gain=Gain)
        backend.sides=('left','right');backend._controllers={s:Controller()for s in backend.sides}
        backend._controller_configs={s:NS(default_kp=[80,70,70,30,30,20],default_kd=[2,2,2,1,1,.7])for s in backend.sides}
        backend.config.gripper_width_m=.083;backend.config.teach_kd_scale=.1
        backend.command_guard=lambda:None;backend.native_faults=lambda:{}
        backend.read_execution_output=lambda:dict(q_sdk14=[0.]*6+[.06]+[0.]*6+[.06],
            gains={s:dict(gripper_kp=4.,gripper_kd=.3)for s in backend.sides},
            observations={s:dict(sample_monotonic_ns=10_000_000_000)for s in backend.sides})
        def legacy_stop(self,**kwargs):
            for side,controller in self._controllers.items():
                controller.set_joint_cmd(NS(gripper_pos=self._stop_gripper_command(side,.02)))
                gain=Gain(6);gain.gripper_kp=.25;gain.gripper_kd=.1;controller.set_gain(gain)
        base=type(backend).__mro__[1]
        with patch.object(base,'set_damping_hold',legacy_stop,create=True),patch.object(calibration.time,'monotonic_ns',return_value=10_000_000_000):
            backend.set_damping_hold(manual=False,preserve_jaws=True)
        self.assertEqual([w for w in writes if w[0]=='command'],[('command',.06)]*2)
        self.assertTrue(all(w==('gain',4.,.3)for w in writes if w[0]=='gain'))
        self.assertEqual(backend._retained_stop_jaws,{})

    def test_right_disabled_keeps_left_native_gripper_active(self):
        backend,guard,events=self.fixture(['right'])
        with patch('robot_workbench.arx_output_guard.require_guard',return_value=guard):backend.connect()
        self.assertEqual(backend.passive_jaws, ('right',))
        self.assertEqual(events,[('guard','begin'),('native','can3','DM'),('native','can1','none'),('guard','activate')])
        backend.close()

    def test_watchdog_cannot_exclude_a_gripper_that_sdk_will_enable(self):
        backend,guard,events=self.fixture(False);guard.disabled_grippers=['left']
        with patch('robot_workbench.arx_output_guard.require_guard',return_value=guard),self.assertRaisesRegex(ValueError,'differ'):backend.connect()
        self.assertEqual(events,[])

    def test_memory_diagnostics_preserve_native_close_and_guard_message_order(self):
        baseline,guard,expected=self.fixture(['right'])
        with patch('robot_workbench.arx_output_guard.require_guard',return_value=guard):baseline.connect()
        baseline.close()
        reader=ReleaseTrace.create();writer=ReleaseTrace.inherit(os.dup(reader.fd))
        try:
            backend,guard,events=self.fixture(['right']);guard.release_trace=writer
            with patch('robot_workbench.arx_output_guard.require_guard',return_value=guard):backend.connect()
            self.assertEqual(reader.snapshot(os.getpid())['status'],'not_started')
            backend.close();self.assertEqual(events,expected)
            value=reader.snapshot(os.getpid());self.assertEqual(value['status'],'observed')
            self.assertEqual(value['events'][0]['step'],'guard_releasing')
            self.assertEqual(value['events'][0]['status'],'returned')
            for side in ('left','right'):
                marks=[(row['step'],row['status'])for row in value['events']]
                self.assertLess(marks.index(('destroy_'+side,'returned')),marks.index(('retire_'+side,'started')))
            self.assertEqual(value['last_confirmed_step']['step'],'guard_disarm')
            self.assertEqual(value['last_confirmed_step']['status'],'returned')
        finally:writer.close();reader.close()

    def test_candidate_quiesces_and_retires_before_resource_destructor(self):
        backend,guard,events=self.fixture(['right'])
        guard.sdk_contract={'library_build_id':'a'*64,'module':{'path':__file__}}
        def call(action,**payload):
            events.append(('guard',action))
            return {'generation':'current'} if action=='begin' else {}
        guard.call=call
        with patch('robot_workbench.arx_output_guard.require_guard',return_value=guard):
            # The fixture's SDK stays inert. Add only the candidate API under test.
            original=backend._import_arx5
            def load():
                sdk=original();create=sdk.Arx5JointController
                def controller(*args):
                    obj=create(*args)
                    interface=args[-1]
                    obj.controller_instance_id=lambda:interface
                    def quiesce(generation):
                        events.append(('quiesce',interface,generation))
                        return NS(library_build_id='a'*64,controller_instance_id=interface,
                            caller_generation=generation,send_thread_joined=True,idempotent_hit=False,
                            last_send_attempt_monotonic_ns=1,receiver_state='vendor_owned_unverified',
                            quiesce_returned_monotonic_ns=2,
                            transport_basis='trusted_vendor_binary',verification_scope='controller_application')
                    obj.quiesce=quiesce
                    return obj
                sdk.Arx5JointController=controller
                return sdk
            backend._import_arx5=load;backend.connect()
        backend.close()
        for interface,side in [('can3','left'),('can1','right')]:
            self.assertLess(events.index(('quiesce',interface,'current')),events.index(('guard','retire_'+side)))
            self.assertLess(events.index(('guard','retire_'+side)),events.index(('native_closed',interface)))
        self.assertEqual(events[-1],('guard','disarm'))

    def test_native_acquisition_error_uses_bounded_private_trip_evidence(self):
        backend,guard,events=self.fixture(False)
        def call(action,**payload):
            events.append((action,payload));return {}
        guard.call=call
        with patch('robot_workbench.arx_output_guard.require_guard',return_value=guard):
            sdk=backend._import_arx5()
        def reject(*_):raise RuntimeError('native construction failed '+'.'*900)
        sdk.Arx5JointController=reject
        with patch('robot_workbench.arx_output_guard.require_guard',return_value=guard),self.assertRaises(RuntimeError):
            backend.connect()
        self.assertEqual(events[-1][0],'trip')
        self.assertTrue(events[-1][1]['detail'].startswith('RuntimeError: native construction failed'))
        self.assertEqual(len(events[-1][1]['detail']),512)


if __name__=='__main__':unittest.main()
