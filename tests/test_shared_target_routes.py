"""Actual Runtime/runners for all four sources, with point-send forbidden.

The plant is an ideal CPU protocol adapter, not a physical precision test.
"""
from copy import deepcopy
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'tests'),str(ROOT/'calibration'),str(ROOT/'calibration/infra'),
              str(ROOT/'calibration/tests'),str(ROOT/'rule_based_workbench')]
from robot_workbench.runtime import Runtime
from robot_workbench.target_interpreter import SCHEMA
from test_owner_clock import ClockBackend,Safety,calibration,jaw
import test_inference_owner as inference_fixture


class WindowBackend(ClockBackend):
    target_interpreter_profile='normal_X5'
    def __init__(self,clock,q=None):
        super().__init__(clock,q,source_clock_id='shared-interpreter-cpu-clock')
        self.packets=[];self.window=None;self.applied_command=None
        self.position_bias=np.zeros(14);self.control_thread=threading.get_ident()

    def configure_motion_gains(self,value,*,preserve_hold=False):
        self.study_gain_override=deepcopy(value)
        if preserve_hold:
            self.events.append(('hold_gains',deepcopy(value)))
            return dict(configured=True,hardware_write=True,preserved_hold=True)
        return dict(configured=value is not None,hardware_write=False)

    def queue_reference(self,packet):
        assert threading.get_ident()==self.control_thread,'model/data worker attempted device output'
        assert packet['schema']==SCHEMA,'source escaped shared interpreter'
        self.packets.append(deepcopy(packet));self.window=deepcopy(packet)
        command=self.applied_command[:]if self.applied_command is not None else self.q[:]
        if self.applied_command is None:
            command[6]=jaw.command_sdk('left',jaw.feedback_gaps(self.q)[0])
            command[13]=jaw.command_sdk('right',jaw.feedback_gaps(self.q)[1])
        self.window['host_times_s'].insert(0,self.clock())
        self.window['q_sdk14'].insert(0,command)
        return dict(firmware_ack=False,frames=len(packet['host_times_s']))

    def send(self,*_):
        raise AssertionError('ordinary motion bypassed the shared interpreter via old point-send')

    def read_calibration_state(self):
        if self.window is not None:
            times=self.window['host_times_s'];rows=np.asarray(self.window['q_sdk14'])
            command=[float(np.interp(self.clock(),times,rows[:,j]))for j in range(14)]
            self.applied_command=command[:]
            sides=('left','right')if self.window['active_arm']=='both'else(self.window['active_arm'],)
            for side in sides:
                start,end=(0,6)if side=='left'else(7,13)
                self.q[start:end]=[command[i]+self.position_bias[i]for i in range(start,end)]
                self.q[end]=jaw.feedback_sdk(side,jaw.command_gap_mm(side,command[end]))
        return super().read_calibration_state()

    def set_damping_hold(self,**kwargs):
        self.window=None
        return super().set_damping_hold(**kwargs)


class SharedTargetRoutesTest(unittest.TestCase):
    profile='normal_X5'
    def capture_factory(self):
        from types import SimpleNamespace
        made=[]
        class Recorder:
            recording_backend='capture_session_v1'
            def __init__(rec,payload,on_fault):
                context=payload['recording_preparation'];rec.context=context;rec.notify=on_fault
                rec.execution_instance_id=context['execution_instance_id'];rec.events=[];rec.diagnostics_error=None
                rec.ready=threading.Event();rec.prepared=threading.Event();rec.done=threading.Event()
                rec.capture=SimpleNamespace(config={},_finishing=False)
                rec.preparation_deadline_monotonic=time.monotonic()+10
                rec.root=Path(self.temp.name)/rec.execution_instance_id;rec.manifest=None;rec.controller_result=None
                rec.thread=SimpleNamespace(is_alive=lambda:not rec.done.is_set())
            @property
            def error(rec):return rec.diagnostics_error if rec.context['control_policy']['parameters']['recording']['purpose_profile']=='data_required'else None
            def submit(rec,kind,**fields):rec.events.append((kind,deepcopy(fields)))
            def finish(rec,status,reason=None):
                if rec.capture._finishing:return
                rec.capture._finishing=True;rec.controller_result=dict(status=status,reason=reason);rec.done.set()
            def bind_execution(rec,*args,**kwargs):pass
            def snapshot(rec):return dict(ready=rec.ready.is_set(),done=rec.done.is_set(),error=rec.diagnostics_error)
        def factory(payload,on_fault):
            value=Recorder(payload,on_fault);made.append(value);return value
        factory.recording_backend='capture_session_v1';factory.supports_deferred_preparation=True
        self.runtime.owner.recording_factory=factory
        return made

    def test_common_capture_prepares_without_output_then_records_actual_sdk_calls(self):
        self.create();made=self.capture_factory()
        review=self.runtime.submit(self.creds['calibration'],dict(action='preview_workbench',
            waypoints=[dict(q12=[.02]*12,speeds12=[.1]*12)]))
        self.confirm('calibration',review);self.advance(20)
        recording=made[0];self.assertEqual(self.runtime.owner.runner.status,'ARMED');self.assertEqual(self.raw.packets,[])
        self.assertTrue(self.runtime.owner.feedback_healthy)
        recording.prepared.set();recording.ready.set();self.advance(250)
        self.assertEqual(self.runtime.owner.runner.status,'READY',self.runtime.owner.runner.reason)
        self.assertTrue(recording.done.is_set());self.assertEqual(recording.controller_result['status'],'completed')
        self.assertEqual(recording.context['control_policy']['policy_hash'],self.runtime.execution_calibration['control_policy']['policy_hash'])
        self.assertTrue(any(kind=='feedback'and data.get('phase')=='sdk_read'for kind,data in recording.events))
        self.assertTrue(any(kind=='command'and data.get('operation')=='queue_reference'and data['phase']=='returned'for kind,data in recording.events))

    def test_stop_during_common_capture_preparation_never_starts_delayed_motion(self):
        self.create();made=self.capture_factory()
        review=self.runtime.submit(self.creds['calibration'],dict(action='preview_workbench',waypoints=[dict(q12=[.02]*12,speeds12=[.1]*12)]))
        self.confirm('calibration',review);self.advance(3);recording=made[0]
        self.runtime.request_stop();self.advance(2)
        self.assertEqual(recording.controller_result['status'],'stopped')
        recording.prepared.set();recording.ready.set();self.advance(100)
        self.assertEqual(self.raw.packets,[]);self.assertEqual(self.runtime.owner.runner.status,'STOPPED')

    def test_required_recording_failure_queues_one_stop_and_prevents_later_windows(self):
        self.create();made=self.capture_factory()
        review=self.runtime.submit(self.creds['calibration'],dict(action='preview_workbench',waypoints=[dict(q12=[.1]*12,speeds12=[.1]*12)]))
        self.confirm('calibration',review);self.advance(1);recording=made[0]
        recording.prepared.set();recording.ready.set();self.advance(40);self.assertTrue(self.raw.packets)
        recording.diagnostics_error='CPU writer exited';recording.notify(recording.diagnostics_error)
        epoch=self.runtime.owner.epoch;count=len(self.raw.packets)
        recording.notify(recording.diagnostics_error);self.assertEqual(self.runtime.owner.epoch,epoch)
        self.advance(5)
        self.assertEqual(self.runtime.owner.epoch,epoch+1);self.assertEqual(len(self.raw.packets),count)
        self.assertEqual(recording.controller_result['status'],'failed')

    def test_native_model_jaw_guard_uses_full_calibrated_opening(self):
        self.create();self.runtime.handoff(self.creds['calibration'],'inference')
        config=dict(model_id='cpu-fixture',duration_s=40.,execute_steps=30,
            binding={'schema':'workbench.wam-native-coordinates.v1'},
            conversion_identity={'mapping':{'profile_max_opening_mm':{'left':83.,'right':83.}}})
        self.runtime.configure_inference(config,lambda *args:None)
        self.runtime.submit(self.creds['inference'],dict(action='preview_inference'))
        guard=self.runtime._control_guards['inference']
        for gap,expected in [(81.,'pass'),(82.5,'pass'),(83.,'pass'),(83.001,'reject'),(-.001,'reject')]:
            decision=guard.evaluate(dict(rule_id='G17',signal_id='model-jaw-boundary',
                observed=dict(minimum=[gap,gap],maximum=[gap,gap]),
                source_time=self.now,checked_time=self.now,epoch=self.runtime.owner.epoch),'model_command')
            self.assertEqual(decision['verdict'],expected,decision)
            self.assertEqual(decision['limit']['bounds'],[[0.,83.],[0.,83.]])

    def test_preview_and_native_window_share_bound_position_decisions(self):
        self.create()
        review=self.runtime.submit(self.creds['calibration'],dict(action='preview_workbench',
            waypoints=[dict(q12=[.02]*12,speeds12=[.1]*12,jaw_gap_mm=[30.,30.])]))
        prepared=[row for name,row in self.events if name=='guard_decision'and row['rule_id']=='G19']
        self.assertTrue(prepared)
        self.confirm('calibration',review);self.advance(100)
        runner=self.runtime.owner.runner;frozen=self.runtime.execution_calibration['control_policy']
        self.assertEqual({row['policy_hash']for row in prepared},{frozen['policy_hash']})
        native=runner.target_interpreter.last_timing['guard_decisions']
        self.assertIn('G18',{row['rule_id']for row in native})
        self.assertEqual({row['policy_hash']for row in native},{frozen['policy_hash']})
        boundary=self.runtime.owner.cache['calibration_feedback']['position_boundary']['guard_decision']
        self.assertEqual((boundary['rule_id'],boundary['policy_hash']),('G20',frozen['policy_hash']))
        with self.assertRaisesRegex(ValueError,'G18'):
            self.runtime.owner.send('left',[99.]+[0.]*13)

    def test_strict_begin_command_is_carried_into_every_home_window(self):
        self.create();owner=self.runtime.owner;owner.preserve_begin_jaws=True
        measured=self.raw.q[:];held=measured[:]
        for side,slot in [('left',6),('right',13)]:
            measured[slot]=jaw.feedback_sdk(side,31.)
            held[slot]=jaw.command_sdk(side,30.)
        measured[0]=held[0]=.02;self.raw.q=measured[:];self.raw.applied_command=held[:]
        self.advance(40)
        calls=[]
        def begin(side,feedback,*,preserve_jaws=False):
            self.assertTrue(preserve_jaws);calls.append(side)
            return dict(measured_qpos14=feedback[:],command_sdk14=held[:],source_output={'firmware_ack':False})
        self.raw.begin=begin
        review=self.runtime.submit(self.creds['calibration'],dict(action='preview_workbench',
            waypoints=[dict(q12=[0.]*12,speeds12=[.1]*12,jaw_gap_mm=[30.,30.])]))
        self.confirm('calibration',review);self.advance(150)
        self.assertEqual(calls,['left','right'])
        self.assertTrue(self.raw.packets)
        for packet in self.raw.packets:
            for row in packet['q_sdk14']:
                self.assertEqual([row[6],row[13]],[held[6],held[13]])
        self.assertEqual(owner.last_begin_command['epoch'],owner.epoch)

    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.now=100.;self.events=[];self.instances=[];self.runtime=None
        jaw.configure_mapping(None);self.addCleanup(jaw.configure_mapping,None)
        self.addCleanup(self.cleanup)

    def create(self,q=None,mapping=None,binding=None):
        if mapping:jaw.configure_mapping(mapping)
        self.raw=WindowBackend(lambda:self.now,q)
        self.raw.target_interpreter_profile=self.profile
        self.runtime=Runtime(calibration,Safety,lambda:self.raw,device_id='shared-target-cpu',environment='fake',
            lock_root=self.temp.name,initialize_backend=lambda b:b.set_damping_hold(manual=True),
            control_clock=lambda:self.now,jaw_mapping_file=mapping,hardware_binding=binding,
            data_plane=getattr(self,'ros_plane',None),
            event=lambda name,**data:self.events.append((name,data)))
        self.creds={business:self.runtime.register_business(business)for business in ('calibration','collection','inference')}
        self.advance(42)

    def cleanup(self):
        if self.runtime is not None and self.runtime.fd is not None:
            self.runtime.owner.recording=None
            self.runtime.owner.submit(dict(action='release',confirmation='ARMS_SUPPORTED_RELEASE'))
            self.runtime.tick();self.runtime.close()

    def advance(self,count=1):
        for _ in range(count):
            runner=self.runtime.owner.runner
            if runner and getattr(runner,'runtime_started',False):runner.runtime_ready.wait(.05)
            if getattr(runner,'is_inference',False)and runner.loop._query_pending:time.sleep(.001)
            self.now+=.01;self.runtime.tick()

    def confirm(self,business,review):
        self.runtime.submit(self.creds[business],dict(action='confirm',token=review['token'],
            confirmation='START_REVIEWED_SINGLE_ARM',physical_clearance=True))

    def assert_shared(self):
        self.assertTrue(self.raw.packets)
        self.assertTrue(all(packet['schema']==SCHEMA for packet in self.raw.packets))
        self.assertIsNotNone(self.runtime.owner.runner.target_interpreter)
        state=self.runtime.snapshot(self.creds[self.runtime.holder])
        self.assertTrue(state['target_interpreter']['enabled'])
        self.assertEqual(state['target_interpreter']['controller_profile'],self.profile)

    def assert_stop_cancels(self):
        self.runtime.request_stop();self.advance(3);count=len(self.raw.packets)
        self.assertIsNone(self.raw.window);self.advance(100)
        self.assertEqual(len(self.raw.packets),count)

    def test_pedal_loss_keeps_feedback_after_fault_without_resuming_output(self):
        from types import SimpleNamespace
        from unittest.mock import patch
        self.create()
        listener=SimpleNamespace(_stop_lock=threading.Lock(),_last_press_ns=None,_key_down=False,
            input_healthy=True,fault_reason='pedal read failed: OSError: [Errno 19] No such device',
            take_presses=lambda:[],requested=threading.Event(),close=lambda:None)
        self.runtime.owner.listener=listener
        command=dict(action='preview_workbench',waypoints=[dict(q12=[.05]+[0.]*11,speeds12=[.1]*12)])
        review=self.runtime.submit(self.creds['calibration'],command)
        self.confirm('calibration',review);self.advance(40);self.assertIsNotNone(self.raw.window)
        listener.input_healthy=False;self.advance()
        owner=self.runtime.owner;reason=owner.control.reason
        self.assertIn('No such device',reason);self.assertEqual(owner.control.state,owner.states.FAULT)
        self.assertIsNone(self.raw.window);count=len(self.raw.packets)
        with patch.object(self.raw,'read_calibration_state',wraps=self.raw.read_calibration_state)as read:
            self.advance(50)
            self.assertEqual(read.call_count,50,'fault holding must keep the actual feedback/heartbeat path alive')
        state=self.runtime.snapshot(self.creds['calibration'])
        self.assertTrue(state['feedback_healthy']);self.assertFalse(state['automatic_enabled'])
        self.assertEqual(len(self.raw.packets),count)
        self.runtime.request_stop();self.advance(5)
        self.assertTrue(self.runtime.stop_receipt['stop_hold_return_confirmed'])
        listener.input_healthy=True;self.advance(20)
        self.assertEqual(owner.control.state,owner.states.FAULT);self.assertEqual(owner.control.reason,reason)
        self.assertEqual(len(self.raw.packets),count)
        with self.assertRaises(ValueError):self.runtime.submit(self.creds['calibration'],command)
        listener.input_healthy=False
        with patch.object(self.raw,'read_calibration_state',side_effect=RuntimeError('feedback unavailable')):
            self.advance()
            self.assertFalse(self.runtime.snapshot(self.creds['calibration'])['feedback_healthy'])

    def test_native_pd_defaults_do_not_enable_integral_for_ordinary_businesses(self):
        from robot_workbench.sdk_quiesce import DM_EQUIVALENT_UNITS
        self.profile='x5_control50_damping_v1';self.create()
        self.raw.motion_gain_profile='normal_X5';self.raw.torque_unit_contract=DM_EQUIVALENT_UNITS
        self.creds['research']=self.runtime.register_business('research')
        for business in ('calibration','collection','inference','research'):
            with self.subTest(business=business):
                if self.runtime.holder!=business:self.runtime.handoff(self.creds[self.runtime.holder],business)
                review=self.runtime.submit(self.creds[business],dict(action='preview_workbench',
                    waypoints=[dict(q12=[.02]+[0.]*11,speeds12=[.1]*12)]))
                self.confirm(business,review);self.advance(250)
                runner=self.runtime.owner.runner
                self.assertEqual(runner.status,'READY',runner.reason)
                self.assertFalse(runner.target_interpreter.integrals)
                self.assertTrue(self.raw.packets)
                np.testing.assert_array_equal(self.raw.packets[-1]['torque_bias_sdk14'],0.)
                self.assert_stop_cancels()

    def test_explicit_dm_integral_refusal_stops_and_recovers_without_resuming(self):
        import os
        from robot_workbench.sdk_quiesce import DM_EQUIVALENT_UNITS
        from robot_workbench.execution_settings import study_defaults
        self.profile='x5_control50_damping_v1';self.create()
        self.raw.motion_gain_profile='normal_X5';self.raw.torque_unit_contract=DM_EQUIVALENT_UNITS
        self.creds['research']=self.runtime.register_business('research')
        self.runtime.handoff(self.creds['calibration'],'research')
        settings=study_defaults()
        settings['integral_gains']={side:[1.,1.,1.,0.,0.,0.]for side in ('left','right')}
        command=dict(action='preview_workbench',waypoints=[dict(q12=[.02]+[0.]*11,speeds12=[.1]*12)],study_settings=settings)
        review=self.runtime.submit(self.creds['research'],command)
        self.confirm('research',review);self.advance(250)
        self.assertEqual(self.runtime.owner.runner.status,'READY',self.runtime.owner.runner.reason)
        for side in ('left','right'):
            np.testing.assert_array_equal(self.runtime.owner.runner.target_interpreter.integrals[side].gains,settings['integral_gains'][side])
        self.assert_stop_cancels();before=len(self.raw.packets)
        settings['integral_gains']['right'][3]=1.
        review=self.runtime.submit(self.creds['research'],command)
        self.confirm('research',review);self.advance(100)
        owner=self.runtime.owner
        self.assertIn('DM unit qualification requires J4/J5/J6 Ki0',owner.recoverable_fault or '')
        self.assertIsNone(self.raw.window);self.assertEqual(len(self.raw.packets),before)
        owner.submit(dict(action='recover',confirmation='ACKNOWLEDGE_MOTION_FAULT',
            owner_pid=os.getpid(),command_epoch=owner.epoch))
        self.advance(50)
        state=self.runtime.snapshot(self.creds['research'])
        self.assertEqual(state['state'],'STOPPED_DAMPING_HOLD');self.assertIsNone(state['motion'])
        self.assertEqual(len(self.raw.packets),before)

    def test_native_window_rejection_faults_and_stops_without_disconnecting(self):
        from robot_workbench.sdk_process import SDKCommandRejected
        self.create()
        review=self.runtime.submit(self.creds['calibration'],dict(action='preview_workbench',
            waypoints=[dict(q12=[.05]+[0.]*11,speeds12=[.1]*12,jaw_gap_mm=[40.,40.])]))
        self.confirm('calibration',review);self.advance(40)
        self.assertIsNotNone(self.raw.window)
        timing=dict(worker=dict(wall_s=.04,cpu_s=.001),native=dict(phase='commit_right'))
        def reject(packet):
            error=SDKCommandRejected('bounded, ordered future reference window required');error.timing=timing
            raise error
        self.raw.queue_reference=reject;self.advance(12)
        state=self.runtime.snapshot(self.creds['calibration'])
        self.assertTrue(state['connected']);self.assertFalse(state['automatic_enabled'])
        self.assertIn('FAULT',state['state']);self.assertIsNone(self.raw.window)
        recorded=[data['timing']['sdk_reference_timing']for name,data in self.events
                  if name=='sdk_call_latency'and 'sdk_reference_timing'in data['timing']]
        self.assertEqual(recorded,[timing],'even a fast typed refusal must preserve its stage diagnostics')
        diagnostic=next(data['diagnostic']for name,data in self.events if name=='fault_diagnostic')
        self.assertEqual(diagnostic['reference_admission']['timing'],timing)
        translation=diagnostic['target_translation_timing']
        self.assertEqual(translation['status'],'failed');self.assertEqual(translation['phase'],'native_admission')
        self.assertEqual(translation['error_type'],'SDKCommandRejected')
        self.assertEqual(translation,state['target_interpreter']['reference']['timing_s'])
        count=len(self.raw.packets);self.advance(50)
        self.assertEqual(len(self.raw.packets),count,'fault cannot auto-retry the rejected trajectory')

    def test_control_gap_stops_then_allows_only_explicit_healthy_stationary_ack(self):
        import os
        self.create()
        review=self.runtime.submit(self.creds['calibration'],dict(action='preview_workbench',
            waypoints=[dict(q12=[.05]+[0.]*11,speeds12=[.1]*12,jaw_gap_mm=[40.,40.])]))
        self.confirm('calibration',review);self.advance(40)
        self.now+=.11;self.runtime.tick();owner=self.runtime.owner
        self.assertIn('control tick gap exceeds 0.1 s',owner.recoverable_fault)
        self.assertIsNone(self.raw.window);count=len(self.raw.packets)
        self.advance(50);self.assertEqual(len(self.raw.packets),count)
        command=dict(action='recover',confirmation='ACKNOWLEDGE_MOTION_FAULT',owner_pid=os.getpid(),command_epoch=owner.epoch)
        owner.feedback_healthy=False
        with self.assertRaises(ValueError):owner.submit(command)
        self.advance(1);owner.submit(command);self.advance(2)
        state=self.runtime.snapshot(self.creds['calibration'])
        self.assertEqual(state['state'],'STOPPED_DAMPING_HOLD');self.assertIsNone(state['motion'])
        self.advance(50);self.assertEqual(len(self.raw.packets),count)

    def test_calibration_single_target_reaches_shared_executor_and_stop_cancels(self):
        self.create();q=self.runtime.owner.cache['calibration_feedback']['qpos'][:];q[0]=.02
        review=self.runtime.submit(self.creds['calibration'],dict(action='preview',active_arm='left',q_target=q))
        self.confirm('calibration',review);self.advance(230);self.assert_shared()
        self.assertEqual(self.runtime.owner.runner.status,'READY',self.runtime.owner.runner.reason)
        self.assert_stop_cancels()

    def test_manual_trajectory_is_shared_by_collection_and_inference_without_model(self):
        self.create()
        for business in ('collection','inference'):
            self.runtime.handoff(self.creds[self.runtime.holder],business)
            row=dict(q12=[.02]+[0.]*11,speeds12=[.1]*12)
            with self.assertRaises(PermissionError):
                self.runtime.submit(self.creds['calibration'],dict(action='preview_workbench',waypoints=[row]))
            with self.assertRaisesRegex(ValueError,'capture acknowledgement'):
                self.runtime.submit(self.creds[business],dict(action='preview_workbench',waypoints=[dict(row,capture=True)]))
            review=self.runtime.submit(self.creds[business],dict(action='preview_workbench',waypoints=[row]))
            self.assertEqual(review['business'],business)
            self.confirm(business,review);self.advance(230);self.assert_shared()
            self.assertEqual(self.runtime.owner.runner.status,'READY',self.runtime.owner.runner.reason)
            self.assert_stop_cancels()

    def test_manual_both_home_has_both_arms_in_the_same_window(self):
        q=[0.]*14;q[0]=.02;q[10]=-.02
        self.create(q=q)
        review=self.runtime.submit(self.creds['calibration'],dict(action='preview_both_home'))
        duration=review.get('total_motion_duration_s')or max(plan['duration_s']for plan in review['plans'])
        self.confirm('calibration',review);self.advance(round((duration+2)*100));self.assert_shared()
        self.assertEqual(self.runtime.owner.runner.status,'READY',self.runtime.owner.runner.reason)
        self.assertTrue(all(p['active_arm']=='both'for p in self.raw.packets))
        self.assertLess(abs(self.raw.q[0]),1e-5);self.assertLess(abs(self.raw.q[10]),1e-5)
        self.assert_stop_cancels()

    def test_continuous_start_retime_with_moving_jaw_completes_full_execution(self):
        q=[0.]*14;q[6]=jaw.feedback_sdk('left',1.)
        self.create(q=q);self.raw.passive_jaws=('right',)
        waypoint=dict(q12=[0.,0.,.2]+[0.]*9,speeds12=[.2]*12,jaw_gap_mm=[1.,0.])
        review=self.runtime.submit(self.creds['calibration'],dict(action='preview_workbench',waypoints=[waypoint]))
        self.confirm('calibration',review);self.advance(400)
        self.assertEqual(self.runtime.owner.runner.status,'READY',self.runtime.owner.runner.reason)
        previous=self.runtime.owner.runner.target_interpreter
        read=self.raw.read_calibration_state
        def noisy_feedback():
            state=read();state['qpos'][6]=jaw.feedback_sdk('left',jaw.feedback_gaps(state['qpos'])[0]+.2)
            return state
        self.raw.read_calibration_state=noisy_feedback;self.advance(40)
        waypoint.update(q12=[0.]*12,jaw_gap_mm=[83.,0.])
        review=self.runtime.submit(self.creds['calibration'],dict(action='preview_workbench',waypoints=[waypoint]))
        self.confirm('calibration',review);self.advance(50)
        runner=self.runtime.owner.runner
        self.assertIs(type(runner.execution_duration_s),float)
        self.assertGreater(runner.execution_duration_s,runner.plan.duration_s)
        self.assertEqual(runner.target_interpreter.compensation_parent,previous.reference.metadata['reference_id'])
        self.advance(round((runner.execution_duration_s+2)*100))
        self.assertEqual(runner.status,'READY',runner.reason)
        self.assertLess(abs(jaw.feedback_gaps(self.raw.q)[0]-83.),1e-6)
        self.assertLess(np.max(np.abs(np.asarray(self.raw.q)[[0,1,2,3,4,5,7,8,9,10,11,12]])),1e-6)
        self.assert_stop_cancels()

    def test_rule_collection_preserves_reference_knots_without_old_send(self):
        from test_collection_physx_protocol import protocol_fixture
        _,source,plan,_,mapping=protocol_fixture(Path(self.temp.name),source_clock_id='shared-interpreter-cpu-clock')
        jaw.configure_mapping(mapping);first=plan['ticks'][0]['target']
        q=[*first['left_arm_joints'],jaw.feedback_sdk('left',first['left_gripper']),
           *first['right_arm_joints'],jaw.feedback_sdk('right',first['right_gripper'])]
        self.create(q=q,mapping=mapping,binding=Path(self.temp.name)/'binding.json')
        self.runtime.handoff(self.creds['calibration'],'collection')
        review=self.runtime.submit(self.creds['collection'],dict(action='preview_frozen',source=source,plan=plan))
        self.confirm('collection',review)
        for _ in range(3000):
            self.advance()
            if self.runtime.owner.runner.status in ('READY','FAULT'):break
        runner=self.runtime.owner.runner;self.assert_shared()
        self.assertEqual(runner.status,'READY',runner.reason)
        self.assertEqual(runner.frozen.snapshot()['submission_semantics'],'source_reference_knot_evaluations')
        self.assertEqual(runner.shared_reference.metadata['timing_policy'],'strict_duration')
        self.assertFalse(any(name=='command'for name,_ in self.events))
        self.assertTrue(any(name=='source_target'for name,_ in self.events))
        self.assert_stop_cancels()

    def test_online_infer_uses_shared_chunks_and_only_completed_reference_history(self):
        self.create();self.runtime.handoff(self.creds['calibration'],'inference')
        inference_fixture.InferenceOwnerTests.configure(self,execute_steps=10)
        review=self.runtime.submit(self.creds['inference'],dict(action='preview_inference'))
        self.confirm('inference',review);self.advance(100)
        # The real preparation subprocess needs wall time; this accelerated
        # source-clock fixture must not consume the episode while it starts.
        runner=self.runtime.owner.runner;deadline=time.monotonic()+5
        while runner.loop._result.empty()and not runner.loop._commands and not runner.loop.completed and time.monotonic()<deadline:time.sleep(.005)
        self.assertTrue(not runner.loop._result.empty()or runner.loop._commands or runner.loop.completed,
            'isolated CPU preparation must finish; a prediction request count is not a prepared result')
        self.advance(400);self.assert_shared()
        runner=self.runtime.owner.runner
        self.assertEqual(runner.status,'RUNNING',runner.reason)
        self.assertGreater(runner.loop.completed,0)
        self.assertGreater(len(runner.loop.history),0)
        self.assertEqual(runner.target_interpreter.reference.metadata['kind'],'online_prediction_chunk')
        self.assertTrue(any(name=='inference_prepared_chunk'for name,_ in self.events))
        self.assert_stop_cancels()

    def test_half_second_live_infer_stops_both_arms_and_revokes_future_windows(self):
        from types import SimpleNamespace
        from robot_workbench.inference_owner import InferenceRunner,device_state
        from robot_workbench.inference_model import MIXED366_ID
        self.create();self.runtime.handoff(self.creds['calibration'],'inference')
        requests=[]
        def predict(request):
            requests.append(request);rows=np.zeros((60,14));rows[:,0]=np.arange(1,61)*.0001;rows[:,7]=-np.arange(1,61)*.0001
            return dict(request_id=request['request_id'],action14=rows)
        def factory(owner,proxy,feedback,config):
            adapter=SimpleNamespace(device_to_model=lambda q:q,decode=lambda q,held:dict(final_device14=q.tolist()))
            return InferenceRunner(owner,proxy,feedback,config,predict=predict,adapter=adapter,
                observe=lambda now,fb:dict(source_time=now,device_state14=device_state(fb['qpos']),images={}))
        config=dict(model_id=MIXED366_ID,task='coffee',duration_s=.5,execute_steps=15,
            bounded_live_prediction=True,target_execution='shared_reference_windows',maximum_observation_age_s=.5)
        self.runtime.configure_inference(config,factory)
        self.confirm('inference',self.runtime.submit(self.creds['inference'],dict(action='preview_inference',duration_s=.5)))
        for _ in range(100):
            self.advance()
            if self.runtime.owner.runner.bounded_result is not None:break
        runner=self.runtime.owner.runner;deadline=time.monotonic()+5
        while runner.bounded_result.empty()and time.monotonic()<deadline:time.sleep(.005)
        self.assertFalse(runner.bounded_result.empty());self.advance(70)
        self.assertEqual(runner.status,'STOPPED',runner.reason);self.assertIsNone(self.raw.window)
        self.assertEqual(len(requests),1);self.assertEqual(len(requests[0]['applied_history']),0)
        self.assertTrue(all(p['active_arm']=='both'for p in self.raw.packets))
        self.assertTrue(any(any(row[7]<0 for row in p['q_sdk14'])for p in self.raw.packets))
        elapsed=[v for name,v in self.events if name=='inference_bounded_execution_elapsed']
        self.assertEqual(len(elapsed),1);self.assertAlmostEqual(elapsed[0]['actual_interval_s'],.5,delta=.011)
        count=len(self.raw.packets);self.advance(100);self.assertEqual(len(self.raw.packets),count)

    def test_fixed_prepared_infer_uses_the_identical_executor(self):
        from robot_workbench.motion_reference import prepare_reference
        self.create();self.runtime.handoff(self.creds['calibration'],'inference')
        q=np.zeros((61,14));q[:,0]=np.linspace(0.,.02,61)
        source=dict(device_targets14=q.tolist(),model_id='cpu',calibration_profile_digest='c'*64)
        limits=np.asarray(self.raw.get_bimanual_joint_position_limits())
        compiled=prepare_reference(source,[0.]*14,limits,source_sha256='a'*64)
        inference_fixture.InferenceOwnerTests.configure(self,duration_s=2,prepared_reference=compiled)
        review=self.runtime.submit(self.creds['inference'],dict(action='preview_inference',duration_s=2))
        self.confirm('inference',review);self.advance(360);self.assert_shared()
        self.assertEqual(self.runtime.owner.runner.status,'READY',self.runtime.owner.runner.reason)
        self.assert_stop_cancels()

    def test_fixed_reference_completion_does_not_submit_after_the_source_boundary(self):
        from robot_workbench.motion_reference import prepare_reference
        self.create();self.runtime.handoff(self.creds['calibration'],'inference')
        q=np.zeros((61,14));q[:,0]=np.linspace(0.,.02,61)
        source=dict(device_targets14=q.tolist(),model_id='cpu',calibration_profile_digest='c'*64)
        compiled=prepare_reference(source,[0.]*14,np.asarray(self.raw.get_bimanual_joint_position_limits()),source_sha256='a'*64)
        inference_fixture.InferenceOwnerTests.configure(self,duration_s=2,prepared_reference=compiled)
        review=self.runtime.submit(self.creds['inference'],dict(action='preview_inference',duration_s=2))
        self.confirm('inference',review)
        for _ in range(200):
            self.advance()
            if self.runtime.owner.runner is not None and self.runtime.owner.runner.started is not None:break
        self.assertIsNotNone(self.runtime.owner.runner.started)
        runner=self.runtime.owner.runner;end=runner.started+runner.execution_duration_s
        while self.now+.01<end:self.advance()
        self.assertGreaterEqual(runner.native_queued_until,end)
        # Reproduce the old boundary: completion falls on an eligible upload tick.
        runner.target_interpreter.next_upload=end-.001
        submissions=len(self.raw.packets);self.advance()
        self.assertEqual(runner.status,'READY',runner.reason)
        self.assertEqual(len(self.raw.packets),submissions,'completion must not add a late actual command')

    def test_late_online_prediction_cannot_restart_the_stopped_common_executor(self):
        entered,release=threading.Event(),threading.Event();self.addCleanup(release.set)
        def predict(request):
            entered.set()
            if not release.wait(3):raise TimeoutError('CPU late-result fixture not released')
            return dict(request_id=request['request_id'],action14=np.tile([.02]*6+[.088]+[0.]*7,(60,1)))
        self.create();self.runtime.handoff(self.creds['calibration'],'inference')
        inference_fixture.InferenceOwnerTests.configure(self,predict=predict)
        review=self.runtime.submit(self.creds['inference'],dict(action='preview_inference'))
        self.confirm('inference',review);self.advance(50);self.assertTrue(entered.wait(1))
        self.runtime.request_stop();self.advance(3);count=len(self.raw.packets)
        release.set();self.advance(100)
        self.assertEqual(len(self.raw.packets),count);self.assertIsNone(self.raw.window)

    def test_repeated_point_motion_preserves_command_hold_and_integral_without_gain_reenable(self):
        self.profile='x5_shared_precision_v1';self.create();self.raw.position_bias[0]=-.003
        def move(target):
            q=self.runtime.owner.cache['calibration_feedback']['qpos'][:];q[0]=target
            review=self.runtime.submit(self.creds['calibration'],dict(action='preview',active_arm='left',q_target=q))
            self.confirm('calibration',review);self.advance(230)
            self.assertEqual(self.runtime.owner.runner.status,'READY',self.runtime.owner.runner.reason)
        move(.02);previous=self.runtime.owner.runner.target_interpreter
        before=sum(row[0]=='begin'for row in self.raw.events)
        self.assertGreater(previous.last_packet['torque_bias_sdk14'][0][0],.05)
        move(.03);current=self.runtime.owner.runner.target_interpreter
        self.assertEqual(sum(row[0]=='begin'for row in self.raw.events),before)
        self.assertEqual(current.compensation_parent,previous.reference.metadata['reference_id'])
        self.assertTrue(previous.cancelled)
        self.assertGreater(current.last_packet['torque_bias_sdk14'][0][0],.05)


class SharedPrecisionRoutesTest(SharedTargetRoutesTest):
    profile='x5_shared_precision_v1'


if __name__=='__main__':unittest.main()
