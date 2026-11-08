"""Existing home primitives and begin behavior; fake native sink, no live IO."""
from copy import deepcopy
from pathlib import Path
from types import ModuleType, SimpleNamespace as NS
import sys
import time
import unittest
from unittest.mock import patch

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'tests'),str(ROOT/'calibration'),str(ROOT/'calibration/infra')]
from arx_env import joint_head_jaw_mapping as jaw
from arx_env.joint_head_real_motion import make_plan
from robot_workbench.target_interpreter import prepare_point,TargetInterpreter
from test_prepared_reference_backend import Base,State,Gain,hardware_backend


class HomeBootstrapTests(unittest.TestCase):
    def test_strict_begin_preserves_observed_command_without_relabelling_feedback(self):
        module=ModuleType('arx_env.backends.arx5_backend');module.Arx5Backend=Base;module.Arx5BackendConfig=lambda **kw:NS(**kw)
        with patch.dict(sys.modules,{'arx_env.backends.arx5_backend':module}):
            backend=hardware_backend(prepared_reference_enabled=True,target_interpreter_profile='normal_X5')
        backend._robot_configs={s:NS(robot_model='X5')for s in('left','right')}
        backend.config.gravity_compensation=True;backend.command_guard=lambda:None
        backend.get_bimanual_joint_position_limits=lambda:np.asarray(self.limits)
        commands={};observations={};events=[]
        for side,slot in [('left',6),('right',13)]:
            cfg=backend._controller_configs[side];cfg.default_gripper_kp=5.;cfg.default_gripper_kd=.2
            measured=State(6);measured.p[:]=.02;measured.gripper_pos=jaw.feedback_sdk(side,31.)
            cmd=State(6);cmd.p[:]=.02;cmd.gripper_pos=self.start[slot];cmd.gripper_vel=0.
            gain=Gain(6);gain.gripper_kp=5.;gain.gripper_kd=.2
            observations[side]=NS(valid=True,emergency=False,sequence=1,sample_monotonic_ns=time.monotonic_ns(),command=cmd,gain=gain)
            backend._controllers[side].get_control_observation=lambda side=side:observations[side]
            backend._controllers[side].controller_instance_id=lambda side=side:'native-'+side
            backend._controllers[side].get_joint_state=lambda measured=measured:measured
            backend._controllers[side].set_gain=lambda gain:events.append('gain')
        backend._send_fixed=lambda side,q:commands.__setitem__(side,list(q))
        initial=self.start[:];initial[6]=jaw.feedback_sdk('left',30.)
        result=backend.begin('left',initial,preserve_jaws=True)
        self.assertAlmostEqual(commands['left'][6],self.start[6])
        self.assertAlmostEqual(result['command_sdk14'][6],self.start[6])
        self.assertAlmostEqual(result['measured_qpos14'][6],jaw.feedback_sdk('left',31.))
        self.assertEqual(result['source_output']['observations']['left']['sequence'],1)
        self.assertFalse(result['source_output']['firmware_ack'])
        commands.clear();events.clear();observations['left'].sample_monotonic_ns-=200_000_000
        with self.assertRaisesRegex(ValueError,'fresh healthy'):backend.begin('left',initial,preserve_jaws=True)
        self.assertEqual(commands,{});self.assertEqual(events,[])
        del backend._controllers['left'].get_control_observation
        with self.assertRaisesRegex(ValueError,'no feedback fallback'):backend.begin('left',initial,preserve_jaws=True)
        self.assertEqual(commands,{})

    def setUp(self):
        jaw.configure_mapping(None);self.addCleanup(jaw.configure_mapping,None)
        self.limits=[[-3.,3.]if i not in(6,13)else[0.,.082]for i in range(14)]
        self.start=[.02]*14
        self.start[6]=jaw.command_sdk('left',30.)
        self.start[13]=jaw.command_sdk('right',40.)

    def test_reference_and_every_window_can_hold_jaws_but_does_not_qualify_begin(self):
        target=self.start[:]
        for i in(*range(6),*range(7,13)):target[i]=0.
        plan=make_plan('both',self.start,target,self.limits,kind='workbench',q_start_is_command=True,
                       max_velocity=.1,joint_velocity_limits=[.1]*12,created_at_s=100.)
        ref=prepare_point(plan,self.start,target)
        sampled=[ref.sample(part)for part in np.array_split(np.linspace(0,ref.duration_s,1001),40)]
        q,dq=(np.concatenate([pair[i]for pair in sampled])for i in(0,1))
        np.testing.assert_array_equal(q[:,[6,13]],np.tile(np.asarray(self.start)[[6,13]],(len(q),1)))
        self.assertLessEqual(np.max(abs(dq)),.1+1e-12)
        packets=[];now=[100.]
        backend=NS(queue_reference=lambda p:packets.append(deepcopy(p)),torque_unit_contract='legacy_sdk_v1')
        interp=TargetInterpreter(backend,ref,active_arm='both',start_s=100.,epoch=6,
                                 current_epoch=lambda:6,gate=lambda:None,clock=lambda:now[0],
                                 profile='normal_X5',integral=False)
        for elapsed in np.arange(0.,ref.duration_s+.21,.05):
            now[0]=100.+float(elapsed)
            feedback=dict(qpos=ref.sample([elapsed])[0][0].tolist(),qvel=[0.]*14,sample_time_s=now[0])
            interp.tick(now[0],feedback)
        self.assertTrue(packets)
        for packet in packets:
            for row in packet['q_sdk14']:self.assertEqual([row[6],row[13]],[self.start[6],self.start[13]])
        interp.revoke()
        count=len(packets)
        with self.assertRaises(ValueError):interp.tick(now[0],feedback)
        self.assertEqual(len(packets),count)

    def test_existing_begin_checks_feedback_drift_not_previous_command_and_rebinds_before_gain(self):
        module=ModuleType('arx_env.backends.arx5_backend');module.Arx5Backend=Base;module.Arx5BackendConfig=lambda **kw:NS(**kw)
        with patch.dict(sys.modules,{'arx_env.backends.arx5_backend':module}):
            backend=hardware_backend(prepared_reference_enabled=True,target_interpreter_profile='x5_control50_damping_v1',motion_gain_profile='normal_X5')
        backend._robot_configs={s:NS(robot_model='X5')for s in('left','right')}
        backend.config.gravity_compensation=True
        for cfg in backend._controller_configs.values():cfg.default_gripper_kp=5.;cfg.default_gripper_kd=.2
        backend.command_guard=lambda:None
        backend.get_bimanual_joint_position_limits=lambda:np.asarray(self.limits)
        measured=State(6);measured.p[:]=.02;measured.gripper_pos=jaw.feedback_sdk('left',31.)
        backend._controllers['left'].get_joint_state=lambda:measured
        events=[];backend._send_fixed=lambda side,q:events.append(('fixed',list(q)))
        backend._controllers['left'].set_gain=lambda gain:events.append(('gain',None))
        initial=self.start[:];initial[6]=jaw.feedback_sdk('left',30.)
        backend.begin('left',initial)
        self.assertEqual([x[0]for x in events],['fixed','gain'])
        self.assertAlmostEqual(events[0][1][6],jaw.command_sdk('left',31.))
        self.assertNotEqual(events[0][1][6],self.start[6])
        events.clear();measured.gripper_pos=jaw.feedback_sdk('left',33.)
        with self.assertRaisesRegex(ValueError,'measured begin pose changed'):backend.begin('left',initial)
        self.assertEqual(events,[])


if __name__=='__main__':unittest.main()
