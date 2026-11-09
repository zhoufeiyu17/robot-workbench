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
    def test_bound_position_guard_rejects_corrupt_reference_before_sdk(self):
        from robot_workbench.execution_settings import Guard,control_parameters,defaults,policy_hash
        reference,_=self.point();original=reference.sample
        def corrupt(times):
            q,v,a=original(times);q[:,0]=9.;return q,v,a
        broken=NS(metadata=reference.metadata,duration_s=reference.duration_s,sample=corrupt)
        parameters=control_parameters(defaults('inference'),business='inference')
        frozen=dict(revision=2,parameters=parameters,policy_hash=policy_hash(parameters))
        guard=Guard.bind(frozen,dict(joint_limits=[[-1.,1.]]*12,feedback_timeout_s=.1,starvation_margin_s=.015))
        backend=NS(queue_reference=lambda packet:self.fail('G18 must reject before SDK admission'))
        interpreter=TargetInterpreter(backend,broken,active_arm='left',start_s=100.,epoch=1,
            current_epoch=lambda:1,gate=lambda:None,clock=lambda:100.,guard=guard)
        with self.assertRaisesRegex(ValueError,'G18'):
            interpreter.tick(100.,dict(sample_time_s=100.,qpos=[0.]*14))
        decision=interpreter.last_timing['guard_decisions'][-1]
        self.assertEqual(decision['rule_id'],'G18');self.assertEqual(decision['policy_hash'],frozen['policy_hash'])
        self.assertEqual(decision['observed']['active_arm'],'left')

    def test_unknown_torque_units_are_not_accepted_as_legacy(self):
        from robot_workbench.sdk_process import ReferenceWindowRejected
        reference,_=self.point()
        backend=NS(queue_reference=lambda packet:self.fail('must reject before output'),torque_unit_contract='unknown')
        with self.assertRaisesRegex(ReferenceWindowRejected,'unknown torque unit contract'):
            TargetInterpreter(backend,reference,active_arm='both',start_s=100.,epoch=1,
                current_epoch=lambda:1,gate=lambda:None,clock=lambda:100.)

    def test_sdk_default_pd_is_independent_of_reference_profile_and_study_override(self):
        module=ModuleType('arx_env.backends.arx5_backend');module.Arx5Backend=Base;module.Arx5BackendConfig=lambda **kw:NS(**kw)
        with patch.dict(sys.modules,{'arx_env.backends.arx5_backend':module}):
            backend=hardware_backend(prepared_reference_enabled=True,target_interpreter_profile='x5_control50_damping_v1',motion_gain_profile='normal_X5')
            self.assertEqual(backend.target_interpreter_profile,'x5_control50_damping_v1')
            self.assertEqual(backend.motion_gain_profile,'normal_X5')
            for side in ('left','right'):self.assertEqual(backend.motion_gains(side),gains('normal_X5',side))
            custom={side:dict(zip(('kp','kd'),gains('x5_control50_damping_v1',side)))for side in ('left','right')}
            backend.configure_motion_gains(custom)
            self.assertEqual(backend.motion_gains('left'),gains('x5_control50_damping_v1','left'))
            backend.configure_motion_gains(None)
            self.assertEqual(backend.motion_gains('left'),gains('normal_X5','left'))
            with self.assertRaises(ValueError):hardware_backend(motion_gain_profile='unknown')

    def test_failed_translation_retains_its_own_phase_wall_and_cpu_without_retry(self):
        from robot_workbench.sdk_process import ReferenceWindowRejected
        for phase in ('reference_sample','left_integral','left_feedforward','native_admission'):
            with self.subTest(phase=phase):
                executor=self.fixture(integral=True)
                executor.feedforward=NS(packet=lambda host,values,**kwargs:values)
                executor.tick(self.now,self.feedback(executor));previous=executor.last_packet
                old_timing=executor.last_timing;self.now+=.1;feedback=self.feedback(executor)
                timer=NS(wall=200.,cpu=1_000_000);calls=[]
                error=ReferenceWindowRejected('injected timing-stage failure')
                def fail(*args,**kwargs):
                    calls.append(phase);timer.wall+=.029;timer.cpu+=200_000
                    raise error
                target,method={'reference_sample':(type(executor.reference),'sample'),
                    'left_integral':(executor.integrals['left'],'packet'),
                    'left_feedforward':(executor.feedforward,'packet'),
                    'native_admission':(executor.backend,'queue_reference')}[phase]
                with patch.object(target,method,side_effect=fail), \
                        patch('robot_workbench.target_interpreter.time.monotonic',side_effect=lambda:timer.wall), \
                        patch('robot_workbench.target_interpreter.time.thread_time_ns',side_effect=lambda:timer.cpu):
                    with self.assertRaises(ReferenceWindowRejected)as raised:
                        executor.tick(self.now,feedback)
                self.assertIs(raised.exception,error);self.assertEqual(calls,[phase])
                timing=executor.snapshot()['timing_s']
                self.assertIsNot(timing,old_timing);self.assertEqual(timing['status'],'failed')
                self.assertEqual(timing['phase'],phase);self.assertEqual(timing['error_type'],'ReferenceWindowRejected')
                stage=timing['stages'][-1];self.assertEqual(stage['phase'],phase)
                self.assertAlmostEqual(stage['wall_s'],.029);self.assertAlmostEqual(stage['cpu_s'],.0002)
                self.assertAlmostEqual(timing['wall_s'],.029)
                self.assertIs(executor.last_packet,previous);self.assertEqual(executor.submissions,1)

    def test_cadence_skip_keeps_last_admission_timing_and_exact_packet(self):
        executor=self.fixture();result=executor.tick(self.now,self.feedback(executor))
        timing=executor.last_timing
        self.assertEqual(timing['status'],'admitted');self.assertIs(result['timing_s'],timing)
        self.assertAlmostEqual(timing['first_point_source_s'],self.packets[-1]['host_times_s'][0])
        self.assertAlmostEqual(timing['first_point_lead_at_submit_s'],.03)
        self.now+=.001;self.assertIsNone(executor.tick(self.now,self.feedback(executor)))
        self.assertIs(executor.last_timing,timing);self.assertEqual(len(self.packets),1)

    def point(self,side='both'):
        limits=[[-3.,3.]]*14;limits[6]=limits[13]=[0.,.082]
        initial=np.zeros(14);initial[[6,13]]=.04
        target=initial.copy();target[0]=.1;target[10]=-.1
        if side=='left':target[10]=0.
        if side=='right':target[0]=0.
        plan=make_plan(side,initial.tolist(),target.tolist(),limits,kind='workbench'if side=='both'else'target',
                       duration_s=2.,max_velocity=.2,q_start_is_command=True)
        return prepare_point(plan,initial,target),limits

    def fixture(self,side='both',integral=False,profile='normal_X5',native_grid=None,velocity_source=None):
        reference,_=self.point(side);self.now=100.;self.epoch=4;self.packets=[]
        if native_grid is not None:reference.metadata['native_grid']=native_grid
        if velocity_source is not None:reference.metadata['velocity_source']=velocity_source
        backend=NS(queue_reference=lambda packet:self.packets.append(deepcopy(packet))or{'firmware_ack':False})
        interpreter=TargetInterpreter(backend,reference,active_arm=side,start_s=self.now,epoch=self.epoch,
            current_epoch=lambda:self.epoch,gate=lambda:None,clock=lambda:self.now,integral=integral,profile=profile)
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

    def test_position_only_policy_is_explicit_bounded_and_revocable(self):
        executor=self.fixture(profile='x5_control50_damping_v1',native_grid='reference_origin_30Hz',velocity_source='sdk_position_only')
        executor.tick(self.now,self.feedback(executor))
        self.assertEqual(self.packets[-1]['velocity_source'],'sdk_position_only')
        self.assertEqual(executor.snapshot()['velocity_source'],'sdk_position_only')
        self.assertTrue(np.any(np.asarray(self.packets[-1]['dq_sdk14'])!=0))
        executor.revoke();self.now+=.02
        with self.assertRaisesRegex(ValueError,'revoked'):executor.tick(self.now,self.feedback(executor))
        for grid,source in [('reference_origin_100Hz','sdk_position_only'),('reference_origin_30Hz','unknown')]:
            with self.assertRaisesRegex(ValueError,'position-only velocity'):
                self.fixture(profile='x5_control50_damping_v1',native_grid=grid,velocity_source=source)
        with self.assertRaisesRegex(ValueError,'zero integral'):
            self.fixture(profile='x5_control50_damping_v1',native_grid='reference_origin_30Hz',velocity_source='sdk_position_only',integral=True)

    def test_fixed_reference_grid_preserves_overlap_despite_upload_jitter(self):
        executor=self.fixture(profile='x5_control50_damping_v1')
        executor.reference.metadata['native_grid']='reference_origin_100Hz'
        executor.tick(self.now,self.feedback(executor));first=self.packets[-1]
        self.now+=.0237;executor.tick(self.now,self.feedback(executor));second=self.packets[-1]
        self.assertGreaterEqual(second['host_times_s'][0]-self.now,.03-1e-9)
        self.assertGreaterEqual(second['host_times_s'][-1]-second['host_times_s'][0],.1-1e-9)
        overlap=0
        for i,t in enumerate(first['host_times_s']):
            for j,u in enumerate(second['host_times_s']):
                if abs(t-u)<1e-10:
                    overlap+=1;np.testing.assert_allclose(first['q_sdk14'][i],second['q_sdk14'][j],atol=1e-12)
                    np.testing.assert_allclose(first['dq_sdk14'][i],second['dq_sdk14'][j],atol=1e-12)
        self.assertGreater(overlap,5)
        executor.revoke()
        with self.assertRaisesRegex(ValueError,'revoked'):executor.tick(self.now,self.feedback(executor))

    def test_200hz_keeps_horizon_cadence_overlap_and_terminal_stop(self):
        executor=self.fixture(profile='x5_control50_damping_v1',integral=True,native_grid='reference_origin_200Hz')
        executor.tick(self.now,self.feedback(executor));first=self.packets[-1]
        self.assertEqual(executor.snapshot()['reference_node_hz'],200)
        self.assertEqual(len(first['host_times_s']),21)
        np.testing.assert_allclose(np.diff(first['host_times_s']),.005,atol=1e-12)
        self.assertAlmostEqual(first['host_times_s'][-1]-first['host_times_s'][0],.1)
        self.assertAlmostEqual(executor.next_upload-self.now,.02)
        self.now+=.0237;executor.tick(self.now,self.feedback(executor));second=self.packets[-1]
        self.assertGreaterEqual(second['host_times_s'][0]-self.now,.03-1e-9)
        overlap=0
        for i,t in enumerate(first['host_times_s']):
            for j,u in enumerate(second['host_times_s']):
                if abs(t-u)<1e-10:
                    overlap+=1
                    for key in ('q_sdk14','dq_sdk14','torque_bias_sdk14'):
                        np.testing.assert_allclose(first[key][i],second[key][j],atol=1e-12)
        self.assertGreater(overlap,10)
        for _ in range(220):
            self.now+=.01;executor.tick(self.now,self.feedback(executor))
        np.testing.assert_allclose(self.packets[-1]['dq_sdk14'],0.,atol=1e-12)
        count=len(self.packets);executor.revoke();self.now+=.01
        with self.assertRaisesRegex(ValueError,'revoked'):executor.tick(self.now,self.feedback(executor))
        self.assertEqual(len(self.packets),count);self.assertFalse(executor.integrals)

    def test_grid_policy_rejects_unknown_rate_and_oversized_window_before_submission(self):
        for grid,profile,reason in [('reference_origin_200Hz','normal_X5','bounded 0.1 s'),
                ('reference_origin_30Hz','normal_X5','bounded 0.1 s'),
                ('reference_origin_201Hz','x5_control50_damping_v1','native_grid')]:
            with self.subTest(grid=grid,profile=profile),self.assertRaisesRegex(ValueError,reason):
                self.fixture(profile=profile,native_grid=grid)
            self.assertFalse(self.packets)

    def test_30hz_preserves_horizon_overlap_and_stop_with_fifty_hz_uploads(self):
        executor=self.fixture(profile='x5_control50_damping_v1',integral=True,native_grid='reference_origin_30Hz')
        executor.tick(self.now,self.feedback(executor));previous=self.packets[-1]
        self.assertEqual(executor.snapshot()['reference_node_hz'],30)
        self.assertEqual(len(previous['host_times_s']),4)
        np.testing.assert_allclose(np.diff(previous['host_times_s']),1/30,atol=1e-12)
        self.assertAlmostEqual(previous['host_times_s'][-1]-previous['host_times_s'][0],.1)
        repeated_first=False
        for delay in (.0237,.0208,.0202,.0241)*30:
            self.now+=delay;executor.tick(self.now,self.feedback(executor));current=self.packets[-1]
            self.assertAlmostEqual(executor.next_upload-self.now,.02)
            self.assertGreaterEqual(current['host_times_s'][0]-self.now,.03-1e-9)
            self.assertLess(current['host_times_s'][0]-self.now,.03+1/30+1e-9)
            repeated_first|=abs(previous['host_times_s'][0]-current['host_times_s'][0])<1e-10
            common=0
            for i,t in enumerate(previous['host_times_s']):
                for j,u in enumerate(current['host_times_s']):
                    if abs(t-u)<1e-10:
                        common+=1
                        for key in('q_sdk14','dq_sdk14','torque_bias_sdk14'):
                            np.testing.assert_allclose(previous[key][i],current[key][j],atol=1e-12)
            self.assertGreaterEqual(common,2);previous=current
        self.assertTrue(repeated_first)
        np.testing.assert_allclose(previous['dq_sdk14'],0,atol=1e-12)
        count=len(self.packets);executor.revoke();self.now+=.02
        with self.assertRaisesRegex(ValueError,'revoked'):executor.tick(self.now,self.feedback(executor))
        self.assertEqual(len(self.packets),count)

    def test_repeated_future_knot_does_not_double_integrate(self):
        from robot_workbench.motion_reference import ReferenceIntegral
        integral=ReferenceIntegral(gains_sdk6=[1.]*6)
        first=100.+2/30;times=first+np.arange(4)/30
        integral.packet(100.,100.,[.001]*6,times)
        second=np.asarray(integral.packet(100.02,100.02,[.002]*6,times))
        np.testing.assert_allclose(second[0],0.,atol=1e-12)
        np.testing.assert_allclose(second[1],.002/30,atol=1e-12)
        third=np.asarray(integral.packet(100.04,100.04,[.003]*6,times+1/30))
        np.testing.assert_allclose(third[0],.002/30,atol=1e-12)
        with self.assertRaisesRegex(ValueError,'clock gap or reversal'):
            integral.packet(100.04,100.04,[.003]*6,times+1/30)
        with self.assertRaisesRegex(ValueError,'clock gap or reversal'):
            integral.packet(100.05,100.05,[.003]*6,times)

    def test_upstream_wait_uses_actual_admission_clock_and_keeps_freshness_budget(self):
        executor=self.fixture(integral=True,profile='x5_control50_damping_v1')
        sample=self.feedback(executor);sampled_now=self.now;self.now+=.031
        executor.tick(sampled_now,sample)
        self.assertGreaterEqual(self.packets[-1]['host_times_s'][0]-self.now,.03-1e-9)
        self.now+=.021
        with self.assertRaisesRegex(ValueError,'observed_age_ms=52.000'):executor.tick(sampled_now,sample,force=True)

    def test_preserved_boundary_event_uses_chronological_first_knot(self):
        previous=self.fixture(profile='x5_control50_damping_v1',native_grid='reference_origin_100Hz')
        previous.tick(self.now,self.feedback(previous));prefix=deepcopy(previous.last_packet)
        boundary=prefix['host_times_s'][-1];original=previous.reference
        reference=NS(metadata=original.metadata,duration_s=original.duration_s,sample=original.sample,
                     event_times=np.asarray([boundary-previous.start_s]))
        def replacement():
            value=TargetInterpreter(previous.backend,reference,active_arm='both',start_s=previous.start_s,
                epoch=self.epoch,current_epoch=lambda:self.epoch,gate=lambda:None,clock=lambda:self.now,
                profile='x5_control50_damping_v1')
            value.preserved_prefix=deepcopy(prefix)
            return value
        self.now=boundary-.035;executor=replacement()
        packet=executor.tick(self.now,self.feedback(executor))['packet']
        self.assertAlmostEqual(packet['host_times_s'][0],boundary)
        self.assertTrue(all(b>a for a,b in zip(packet['host_times_s'],packet['host_times_s'][1:])))
        np.testing.assert_allclose(packet['q_sdk14'][0],prefix['q_sdk14'][-1],rtol=0,atol=1e-12)
        count=len(self.packets);self.now=boundary+.001;expired=replacement()
        with self.assertRaisesRegex(ValueError,'committed prefix expired'):
            expired.tick(self.now,self.feedback(expired))
        self.assertEqual(len(self.packets),count)

    def test_stop_epoch_revokes_future_admission_and_old_integral(self):
        executor=self.fixture(integral=True);executor.tick(self.now,self.feedback(executor,.01))
        self.epoch+=1;count=len(self.packets);self.now+=.01
        with self.assertRaisesRegex(ValueError,'epoch revoked'):executor.tick(self.now,self.feedback(executor))
        executor.revoke();self.assertFalse(executor.integrals);self.assertEqual(len(self.packets),count)

    def test_new_point_inherits_actual_total_bias_without_the_old_feedforward_profile(self):
        previous=self.fixture(integral=True)
        previous.feedforward=NS(packet=lambda times,values,**kwargs:np.asarray(values)+.1)
        previous.tick(self.now,self.feedback(previous))
        following=TargetInterpreter(previous.backend,previous.reference,active_arm='both',start_s=self.now,
            epoch=self.epoch,current_epoch=lambda:self.epoch,gate=lambda:None,clock=lambda:self.now,integral=True)
        following.inherit_hold(previous,self.now)
        np.testing.assert_allclose(following.integrals['left'].bias,.1)
        np.testing.assert_allclose(following.integrals['right'].bias,0.)
        self.assertIsNone(following.feedforward);self.assertTrue(previous.cancelled)

    def test_stale_feedback_and_starvation_never_append_another_window(self):
        executor=self.fixture(integral=True);executor.tick(self.now,self.feedback(executor));count=len(self.packets)
        self.now+=.02;state=self.feedback(executor);state['sample_time_s']-=.051
        with self.assertRaisesRegex(ValueError,'fresh feedback'):executor.tick(self.now,state)
        self.now+=.3
        with self.assertRaisesRegex(ValueError,'starved'):executor.tick(self.now,self.feedback(executor))
        self.assertEqual(len(self.packets),count)

    def test_feedforward_total_saturation_does_not_store_hidden_integral(self):
        from robot_workbench.motion_reference import ReferenceFeedforward,FEEDFORWARD_SLEW_LIMITS
        for sign in(-1.,1.):
            executor=self.fixture(integral=True,profile='x5_control50_damping_v1')
            profile=dict(phase=np.array([0.,1.]),bias=np.tile([sign*.3,0.,0.,0.,0.,0.],(2,1)))
            executor.feedforward=ReferenceFeedforward(profile,self.now,2.,self.now-2.)
            previous=None
            for i in range(200):
                self.now+=.02;executor.tick(self.now,self.feedback(executor,sign*.01),force=True)
                packet=self.packets[-1];times=np.asarray(packet['host_times_s']);values=np.asarray(packet['torque_bias_sdk14'])[:,:6]
                self.assertLessEqual(sign*executor.integrals['left'].bias[0],.1+1e-12)
                self.assertTrue(np.all(abs(values[:,0])<=.4+1e-12))
                self.assertTrue(np.all(abs(np.diff(values,axis=0))<=np.diff(times)[:,None]*FEEDFORWARD_SLEW_LIMITS+1e-9))
                if previous is not None:
                    np.testing.assert_allclose(values[0],[np.interp(times[0],previous[0],previous[1][:,j])for j in range(6)],atol=1e-10)
                previous=(times,values)
            self.assertAlmostEqual(sign*executor.integrals['left'].bias[0],.1)
            for _ in range(4):
                self.now+=.02;executor.tick(self.now,self.feedback(executor,-sign*.01),force=True)
            self.assertLess(sign*self.packets[-1]['torque_bias_sdk14'][0][0],.39,
                'reverse error must act promptly instead of unwinding hidden excess')
            executor.revoke();self.assertFalse(executor.integrals);self.assertIsNone(executor.feedforward)

    def test_feedforward_without_integral_reaches_sdk_and_revokes(self):
        from robot_workbench.motion_reference import JOINTS,ReferenceFeedforward
        executor=self.fixture(integral=False,profile='x5_control50_damping_v1')
        profile=dict(phase=np.array([0.,1.]),bias=np.tile([.1]*6,(2,1)))
        executor.feedforward=ReferenceFeedforward(profile,self.now,2.,self.now-2.)
        for _ in range(50):
            executor.tick(self.now,self.feedback(executor),force=True);self.now+=.02
        packet=self.packets[-1];bias=np.asarray(packet['torque_bias_sdk14'])
        np.testing.assert_allclose(bias[:,:6],.1,atol=1e-10)
        np.testing.assert_array_equal(bias[:,6:],0.)
        expected=executor.reference.sample(np.asarray(packet['host_times_s'])-executor.start_s)
        np.testing.assert_array_equal(packet['q_sdk14'],expected[0])
        np.testing.assert_array_equal(np.asarray(packet['dq_sdk14'])[:,JOINTS],expected[1][:,JOINTS])
        self.assertFalse(executor.integrals)
        executor.revoke();self.assertIsNone(executor.feedforward)
        with self.assertRaises(ValueError):executor.tick(self.now,self.feedback(executor))

    def test_stale_feedback_reports_true_age_and_hold_phase_without_widening_budget(self):
        from robot_workbench.target_interpreter import ReferenceFeedbackStaleError
        executor=self.fixture(integral=True);self.now=103.
        state=self.feedback(executor);state['sample_time_s']-=.052
        with self.assertRaises(ReferenceFeedbackStaleError)as raised:executor.tick(self.now,state)
        error=raised.exception;self.assertEqual(error.phase,'terminal_hold')
        self.assertAlmostEqual(error.maximum_age_s,.05);self.assertAlmostEqual(error.age_s,.052)
        self.assertIn('observed_age_ms=52.000',str(error));self.assertFalse(self.packets)
        state['sample_time_s']=self.now+.001
        with self.assertRaises(ValueError)as raised:executor.tick(self.now,state)
        self.assertNotIsInstance(raised.exception,ReferenceFeedbackStaleError,'clock reversal is not an acknowledgeable freshness event')

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

    def test_study_gain_profile_changes_only_left_pd_by_the_declared_factor(self):
        expected=gains('x5_shared_precision_v1','left')
        actual=gains('x5_control50_pd15_v1','left')
        for before,after in zip(expected,actual):np.testing.assert_allclose(after,np.asarray(before)*1.5)
        self.assertEqual(gains('x5_control50_pd15_v1','right'),gains('x5_shared_precision_v1','right'))
        for side in ('left','right'):
            self.assertEqual(gains('x5_control50_pd15_v1',side),gains('x5_control50_pd15_fast_i_v1',side))
            self.assertEqual(gains('x5_control50_pd15_v1',side),gains('x5_control50_feedback50_v1',side))

    def test_high_rate_feedback_windows_remain_bounded_and_stop_revokes_admission(self):
        executor=self.fixture(integral=True,profile='x5_control50_feedback50_v1')
        for i in range(51):
            self.now=100.+i*.01;executor.tick(self.now,self.feedback(executor,.002))
        self.assertGreaterEqual(len(self.packets),20)
        self.assertTrue(all(len(p['q_sdk14'])<=13 for p in self.packets))
        self.assertEqual(executor.integrals['left'].rapid_scale,8)
        self.assertFalse(executor.integrals['right'].rapid)
        count=len(self.packets);executor.revoke();self.now+=.01
        with self.assertRaises(ValueError):executor.tick(self.now,self.feedback(executor))
        self.assertEqual(len(self.packets),count)

    def test_pd30_contrast_only_changes_left_pd_preserving_damping_ratio_and_sdk_bounds(self):
        before=gains('x5_control50_feedback50_v1','left')
        after=gains('x5_control50_pd30_feedback50_v1','left')
        np.testing.assert_allclose(after[0],np.asarray(before[0])*2.)
        np.testing.assert_allclose(after[1],np.asarray(before[1])*np.sqrt(2.))
        self.assertTrue(all(0<x<=500 for x in after[0]))
        self.assertTrue(all(0<x<=5 for x in after[1]))
        self.assertEqual(gains('x5_control50_pd30_feedback50_v1','right'),gains('x5_control50_feedback50_v1','right'))
        executor=self.fixture(integral=True,profile='x5_control50_pd30_feedback50_v1')
        self.now=100.;executor.tick(self.now,self.feedback(executor,.002))
        self.assertAlmostEqual(executor.next_upload,100.02)
        self.assertLessEqual(len(self.packets[-1]['q_sdk14']),13)
        self.assertEqual(executor.integrals['left'].rapid_scale,8)
        self.assertFalse(executor.integrals['right'].rapid)

    def test_damping_contrast_keeps_gains_and_uses_approved_left_j3_budget(self):
        profile='x5_control50_damping_v1'
        self.assertEqual(gains(profile,'left')[0],gains('x5_control50_pd30_feedback50_v1','left')[0])
        self.assertEqual(gains(profile,'left')[1],[5.,5.,5.,3.,3.,2.1])
        self.assertEqual(gains(profile,'right'),gains('x5_control50_pd30_feedback50_v1','right'))
        executor=self.fixture(integral=True,profile=profile)
        np.testing.assert_allclose(executor.integrals['left'].limits,[.4,1.2,2.,.3,.15,.15])
        np.testing.assert_allclose(executor.integrals['right'].limits,[.4,1.2,1.5,.3,.15,.15])
        self.assertEqual(executor.integrals['left'].rapid_scale,8)
        executor.tick(self.now,self.feedback(executor,.002))
        self.assertAlmostEqual(executor.next_upload,self.now+.02)
        self.assertFalse(executor.integrals['right'].rapid)

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
