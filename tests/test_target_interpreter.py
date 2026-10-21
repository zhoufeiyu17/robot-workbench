"""Shared reference/SDK-boundary qualification; CPU only, no hardware imports."""
from copy import deepcopy
from pathlib import Path
import sys
import json
import unittest
from unittest.mock import patch
from types import ModuleType, SimpleNamespace as NS

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'tests'),str(ROOT/'calibration'),str(ROOT/'calibration/infra')]
from robot_workbench.target_interpreter import (TargetInterpreter,prepare_point,prepare_samples,gains,SCHEMA)
from arx_env.joint_head_real_motion import make_plan
from test_prepared_reference_backend import Base,State,Gain
from examples.scripts.debug.joint_head_real_owner import hardware_backend


class TargetInterpreterTests(unittest.TestCase):
    def point(self,side='both'):
        limits=[[-3.,3.]]*14;limits[6]=limits[13]=[0.,.082]
        initial=np.zeros(14);initial[[6,13]]=.04
        target=initial.copy();target[0]=.1;target[10]=-.1
        if side=='left':target[10]=0.
        if side=='right':target[0]=0.
        plan=make_plan(side,initial.tolist(),target.tolist(),limits,kind='workbench'if side=='both'else'target',
                       duration_s=2.,max_velocity=.2,q_start_is_command=True)
        return prepare_point(plan,initial,target),limits

    def fixture(self,side='both',integral=False):
        reference,_=self.point(side);self.now=100.;self.epoch=4;self.packets=[]
        backend=NS(queue_reference=lambda packet:self.packets.append(deepcopy(packet))or{'firmware_ack':False})
        interpreter=TargetInterpreter(backend,reference,active_arm=side,start_s=self.now,epoch=self.epoch,
            current_epoch=lambda:self.epoch,gate=lambda:None,clock=lambda:self.now,integral=integral)
        return interpreter

    def feedback(self,interpreter,error=0.):
        q,_,_=interpreter.reference.sample([self.now-interpreter.start_s])
        q[0,[0,10]]-=error
        return dict(sample_time_s=self.now,qpos=q[0].tolist())

    def test_both_arms_use_one_window_and_zero_velocity_terminal(self):
        executor=self.fixture();executor.tick(self.now,self.feedback(executor))
        p=self.packets[-1];self.assertEqual(p['schema'],SCHEMA);self.assertEqual(p['active_arm'],'both')
        self.assertLessEqual(len(p['host_times_s']),32)
        self.assertTrue(any(row[0]>0. and row[10]<0. for row in p['dq_sdk14']))
        for _ in range(230):
            self.now+=.01;executor.tick(self.now,self.feedback(executor))
        np.testing.assert_allclose(self.packets[-1]['dq_sdk14'],0.,atol=1e-12)
        expected=executor.reference.target
        for row in self.packets[-1]['q_sdk14']:np.testing.assert_allclose(row,expected,atol=1e-12)

    def test_stop_epoch_revokes_future_admission_and_old_integral(self):
        executor=self.fixture(integral=True);executor.tick(self.now,self.feedback(executor,.01))
        self.epoch+=1;count=len(self.packets);self.now+=.01
        with self.assertRaisesRegex(ValueError,'epoch revoked'):executor.tick(self.now,self.feedback(executor))
        executor.revoke();self.assertFalse(executor.integrals);self.assertEqual(len(self.packets),count)

    def test_stale_feedback_and_starvation_never_append_another_window(self):
        executor=self.fixture(integral=True);executor.tick(self.now,self.feedback(executor));count=len(self.packets)
        self.now+=.02;state=self.feedback(executor);state['sample_time_s']-=.051
        with self.assertRaisesRegex(ValueError,'fresh feedback'):executor.tick(self.now,state)
        self.now+=.3
        with self.assertRaisesRegex(ValueError,'starved'):executor.tick(self.now,self.feedback(executor))
        self.assertEqual(len(self.packets),count)

    def test_new_point_and_late_reference_cannot_reuse_old_epoch(self):
        executor=self.fixture();executor.revoke()
        with self.assertRaises(ValueError):executor.tick(self.now,self.feedback(executor))
        with self.assertRaisesRegex(ValueError,'no point-send fallback'):
            TargetInterpreter(NS(send=lambda *_:None),executor.reference,active_arm='left',start_s=100.,
                epoch=0,current_epoch=lambda:0,gate=lambda:None,clock=lambda:100.)

    def test_frozen_path_cannot_silently_change_timing_or_space(self):
        reference,limits=self.point();q=[reference.initial.tolist(),reference.target.tolist()]
        with self.assertRaisesRegex(ValueError,'explicit prepared derivation'):
            prepare_samples([0.,.001],q,limits,source_sha256='a'*64)
        prepared=prepare_samples([0.,.001],q,limits,source_sha256='a'*64,timing_policy='retime_allowed')
        self.assertGreater(prepared.duration_s,.001)
        end,velocity,_=prepared.sample([prepared.duration_s]);np.testing.assert_allclose(end[0],q[-1])
        np.testing.assert_allclose(velocity,0.)
        self.assertEqual(prepared.metadata['path_policy'],'unchanged_piecewise_linear')

    def test_named_gain_policy_keeps_left_and_other_right_joints_unchanged(self):
        self.assertEqual(gains('normal_X5','left'),gains('x5_shared_precision_v1','left'))
        kp,kd=gains('x5_shared_precision_v1','right');self.assertEqual((kp[3],kd[3]),(80.,1.6))
        self.assertEqual(kp[:3]+kp[4:],[80.,70.,70.,30.,20.])

    def test_window_size_does_not_grow_with_source_history(self):
        reference,limits=self.point();targets=np.tile(reference.initial,(20001,1))
        reference=prepare_samples(np.linspace(0.,2.,len(targets)),targets,limits,source_sha256='a'*64)
        self.now=100.;packets=[]
        executor=TargetInterpreter(NS(queue_reference=lambda p:packets.append(p)or{}),reference,
            active_arm='both',start_s=100.,epoch=1,current_epoch=lambda:1,gate=lambda:None,clock=lambda:self.now)
        executor.tick(self.now,dict(sample_time_s=self.now,qpos=targets[0].tolist()))
        self.assertLessEqual(len(packets[0]['q_sdk14']),32)
        self.assertLess(len(json.dumps(packets[0]).encode()),40*1024)
        self.assertNotIn('source',packets[0]);self.assertNotIn('metadata',packets[0])

    def test_fresh_sensor_noise_retimes_without_changing_the_target_or_joint_caps(self):
        _,limits=self.point();start=[0.]*14;start[6]=start[13]=.04
        target=start[:];target[0]=.001
        plan=make_plan('left',start,target,limits,max_velocity=.03,q_start_is_command=True)
        measured=start[:];measured[0]=-.003
        reference=prepare_point(plan,measured,target)
        self.assertGreater(reference.duration_s,plan.duration_s)
        q,dq,ddq=reference.sample(np.linspace(0.,reference.duration_s,31))
        np.testing.assert_allclose(q[-1],target)
        self.assertLessEqual(np.max(np.abs(dq[:,0])),.03+1e-12)
        self.assertLessEqual(np.max(np.abs(ddq[:,0])),5.+1e-12)

    def test_frozen_jaw_transition_is_not_interpolated_before_its_command_stage(self):
        _,limits=self.point();opened=[0.]*14;opened[6]=opened[13]=.07
        closed=opened[:];closed[6]=.069
        reference=prepare_samples([0.,1/30,2/30],[opened,closed,closed],limits,
            source_sha256='a'*64,source_semantics='command_intervals',duration_s=20/240)
        packets=[];executor=TargetInterpreter(NS(queue_reference=lambda p:packets.append(p)or{}),reference,
            active_arm='both',start_s=100.,epoch=1,current_epoch=lambda:1,gate=lambda:None,clock=lambda:100.)
        executor.tick(100.,dict(sample_time_s=100.,qpos=opened))
        packet=packets[-1];jaw=np.asarray(packet['q_sdk14'])[:,6]
        self.assertAlmostEqual(np.interp(100.+1/30-1e-5,packet['host_times_s'],jaw),opened[6],places=12)
        self.assertLess(np.interp(100.+1/30+.01,packet['host_times_s'],jaw),opened[6])
        self.assertAlmostEqual(reference.duration_s,20/240)
        self.assertEqual(reference.metadata['transition_activation'],'not_before_source_command_due')

    def test_two_arm_native_clock_mapping_and_stop_clears_both_queues(self):
        module=ModuleType('arx_env.backends.arx5_backend');module.Arx5Backend=Base;module.Arx5BackendConfig=lambda **kw:NS(**kw)
        with patch.dict(sys.modules,{'arx_env.backends.arx5_backend':module}):
            backend=hardware_backend(disabled_grippers=('right',),prepared_reference_enabled=True,
                target_interpreter_profile='normal_X5')
        backend.command_guard=lambda:None
        executor=self.fixture();executor.tick(self.now,self.feedback(executor));p=self.packets[-1]
        # The SDK clock of each arm is independent; neither can borrow the other mapping.
        import time
        self.now=time.monotonic();executor.start_s=self.now
        executor.next_upload=executor.queued_until=None
        executor.tick(self.now,self.feedback(executor));p=self.packets[-1]
        backend._controllers['right'].get_timestamp=lambda:time.monotonic()-75.
        receipt=backend.queue_reference(p)
        self.assertAlmostEqual(receipt['clock_mappings']['left']['offset_s'],-100.,places=3)
        self.assertAlmostEqual(receipt['clock_mappings']['right']['offset_s'],-75.,places=3)
        for side in ('left','right'):self.assertTrue(backend._controllers[side].future)
        backend.set_damping_hold(manual=False)
        for side in ('left','right'):self.assertFalse(backend._controllers[side].future)

    def test_dual_admission_failure_cancels_first_arm_instead_of_partial_motion(self):
        module=ModuleType('arx_env.backends.arx5_backend');module.Arx5Backend=Base;module.Arx5BackendConfig=lambda **kw:NS(**kw)
        with patch.dict(sys.modules,{'arx_env.backends.arx5_backend':module}):
            backend=hardware_backend(prepared_reference_enabled=True,target_interpreter_profile='normal_X5')
        backend.command_guard=lambda:None
        executor=self.fixture();import time
        self.now=time.monotonic();executor.start_s=self.now;executor.tick(self.now,self.feedback(executor))
        def fail(_):raise RuntimeError('injected second-arm admission failure')
        backend._controllers['right'].set_joint_traj=fail
        with self.assertRaisesRegex(RuntimeError,'second-arm'):backend.queue_reference(self.packets[-1])
        for side in ('left','right'):self.assertFalse(backend._controllers[side].future)

    def test_same_gains_for_point_and_collection_with_no_legacy_output_escape(self):
        class ProfileBase(Base):
            def __init__(self,config):
                super().__init__(config);config.gravity_compensation=True;self.observed={}
                self._robot_configs={s:NS(robot_model='X5')for s in self.sides}
                for side in self.sides:
                    cfg=self._controller_configs[side];cfg.default_gripper_kp=5.;cfg.default_gripper_kd=.2
                    state=State(6);state.gripper_pos=.04
                    self._controllers[side].get_joint_state=lambda state=state:state
                    self._controllers[side].set_gain=lambda gain,side=side:self.observed.__setitem__(side,gain)
            def get_bimanual_joint_position_limits(self):
                return np.vstack([self.get_joint_position_limits(s)for s in self.sides])
        module=ModuleType('arx_env.backends.arx5_backend');module.Arx5Backend=ProfileBase;module.Arx5BackendConfig=lambda **kw:NS(**kw)
        with patch.dict(sys.modules,{'arx_env.backends.arx5_backend':module}):
            backend=hardware_backend(prepared_reference_enabled=True,target_interpreter_profile='x5_shared_precision_v1')
        backend.command_guard=lambda:None;q=[0.]*14;q[6]=q[13]=.04
        backend.begin('right',q)
        self.assertEqual((backend.observed['right'].kp()[3],backend.observed['right'].kd()[3]),(80.,1.6))
        backend.enable_collection_arm('right',q)
        self.assertEqual((backend.observed['right'].kp()[3],backend.observed['right'].kd()[3]),(80.,1.6))
        with self.assertRaisesRegex(ValueError,'point-send is disabled'):backend.send('right',q)
        with self.assertRaisesRegex(ValueError,'legacy reference packets'):
            backend.queue_reference(dict(schema='prepared_motion_reference.v1',reference_id='a'*64,active_arm='left',
                host_times_s=[1.,2.],q_sdk14=[q,q],dq_sdk14=[[0.]*14]*2))
        backend.set_damping_hold(manual=False)
        self.assertEqual(backend.observed['right'].kp()[3],9.)

    def test_continuous_planned_velocity_acceleration_and_bad_profile_rejected(self):
        module=ModuleType('arx_env.backends.arx5_backend');module.Arx5Backend=Base;module.Arx5BackendConfig=lambda **kw:NS(**kw)
        with patch.dict(sys.modules,{'arx_env.backends.arx5_backend':module}):
            backend=hardware_backend(prepared_reference_enabled=True,target_interpreter_profile='normal_X5')
        backend.command_guard=lambda:None;executor=self.fixture();import time
        self.now=time.monotonic();executor.start_s=self.now;executor.tick(self.now,self.feedback(executor));packet=self.packets[-1]
        packet['dq_sdk14'][1][0]=1.
        with self.assertRaisesRegex(ValueError,'acceleration'):backend.queue_reference(packet)
        self.assertFalse(backend._controllers['left'].future)
        packet['controller_profile']='x5_shared_precision_v1'
        with self.assertRaisesRegex(ValueError,'profile differs'):backend.queue_reference(packet)


if __name__=='__main__':unittest.main()
