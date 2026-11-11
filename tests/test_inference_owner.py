"""Actual shared Owner/Runtime policy admission and control tests; CPU backend only."""
from copy import deepcopy
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import threading
import time
import unittest
from unittest.mock import patch

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'tests'),str(ROOT/'calibration'),str(ROOT/'calibration/infra'),
              str(ROOT/'calibration/tests'),str(ROOT/'rule_based_workbench')]
from robot_workbench.runtime import Runtime
from robot_workbench.inference_owner import InferenceRunner, device_limits, device_state
from robot_workbench.inference_model import LegacyBaseAdapter,ContinuousBaseAdapter
from test_owner_clock import ClockBackend, Safety, calibration, jaw


class InferenceOwnerTests(unittest.TestCase):
    def test_optional_clock_mapping_rejects_boot_change_and_clock_step(self):
        from robot_workbench.inference_owner import map_recording_cutoff
        first=dict(host_id='host',boot_id='boot',mapping_id='episode-clock',wall_ns=1000,
            monotonic_before_ns=100,monotonic_after_ns=102)
        now=dict(first,wall_ns=1100,monotonic_before_ns=200,monotonic_after_ns=202)
        value=map_recording_cutoff(1000,first,now,max_uncertainty_ns=4)
        self.assertEqual((value['earliest_ns'],value['latest_ns']),(100,102))
        for bad in [dict(now,boot_id='reboot'),dict(now,wall_ns=2100),
                    dict(now,monotonic_after_ns=220),dict(now,monotonic_before_ns=100)]:
            with self.assertRaises(ValueError):map_recording_cutoff(1000,first,bad,max_uncertainty_ns=4)
        with self.assertRaises(ValueError):map_recording_cutoff(999,first,now,max_uncertainty_ns=4)

    def test_normal_mode_admission_requires_gate_and_explicit_teach_hold_model(self):
        from robot_workbench.inference_owner import validate_mode_request
        identity=dict(session_id='episode',runtime_nonce='owner',epoch=4)
        request=dict(identity,request_id='pause',mode='HOLD')
        options=dict(identity=identity,current_mode='MODEL',clock_id='source-clock',now_ns=200,recorder_ids=['robot','rgb'])
        with self.assertRaisesRegex(ValueError,'closed Raw gate'):validate_mode_request(request,**options)
        self.assertEqual(validate_mode_request(request,**dict(options,recorder_ids=[],recording_required=False))['recording_policy'],
            'explicit_no_recording_consumers')
        with self.assertRaisesRegex(ValueError,'hide active'):
            validate_mode_request(request,**dict(options,recording_required=False))
        gate=dict(identity,request_id='pause',closed=True,clock_id='source-clock',cutoff_ns=150,gate_generation=2,
            recorders=[dict(record_id=name,closed=True,clock_id='source-clock',cutoff_ns=150,generation=2)for name in('robot','rgb')])
        request['recording_gate']=gate
        accepted=validate_mode_request(request,**options);self.assertEqual(accepted['raw_cutoff_ns'],150)
        for bad_gate in [dict(gate,request_id='old'),dict(gate,epoch=3),dict(gate,clock_id='wall-clock'),
                         dict(gate,cutoff_ns=201),dict(gate,recorders=gate['recorders'][:1]),
                         dict(gate,recorders=[gate['recorders'][0]]*2),dict(gate,closed=False)]:
            with self.subTest(gate=bad_gate),self.assertRaises(ValueError):
                validate_mode_request(dict(request,recording_gate=bad_gate),**options)
        with self.assertRaisesRegex(ValueError,'explicit HOLD'):
            validate_mode_request(dict(request,mode='MODEL'),**dict(options,current_mode='TEACH'))
        with self.assertRaisesRegex(ValueError,'never queued'):
            validate_mode_request(dict(request,mode='MODEL'),**dict(options,current_mode='BRAKE'))
        self.assertTrue(validate_mode_request(dict(request,request_id='new-model',mode='MODEL'),
            **dict(options,current_mode='HOLD'))['requires_new_observation_and_epoch'])
        self.assertTrue(validate_mode_request(dict(request,mode='TEACH'),
            **dict(options,current_mode='TEACH'))['no_op'])
        with self.assertRaises(ValueError):validate_mode_request(dict(request,epoch=True),**options)

    def setUp(self):
        jaw.configure_mapping(None)
        self.directory=TemporaryDirectory();self.addCleanup(self.directory.cleanup)
        self.now=100.;self.events=[];self.instances=[];self.release=threading.Event()
        self.addCleanup(self.cleanup)
        self.runtime=Runtime(calibration,Safety,
            lambda:ClockBackend(lambda:self.now, source_clock_id='cpu-policy-clock'),device_id='policy-fixture',
            environment='fake',lock_root=self.directory.name,initial_holder='inference',
            initialize_backend=lambda b:b.set_damping_hold(manual=True),control_clock=lambda:self.now,
            event=lambda name,**data:self.events.append((name,data)))
        self.creds={name:self.runtime.register_business(name) for name in ('calibration','collection','inference')}
        self.advance(42)

    def cleanup(self):
        self.release.set()
        if hasattr(self,'runtime') and self.runtime.fd is not None:
            self.runtime.owner.recording=None
            self.runtime.owner.submit(dict(action='release',confirmation='ARMS_SUPPORTED_RELEASE'))
            self.runtime.tick();self.runtime.close()

    def advance(self,count=1):
        for _ in range(count):
            self.now+=.01;self.runtime.tick()
            runner=self.runtime.owner.runner
            if getattr(runner,'is_inference',False) and runner.loop._query_pending:
                time.sleep(.0001)

    def configure(self,predict=None,record=None,record_prepared=None,duration_s=40,execute_steps=30,prepared_reference=None,integral=False,recording_backend=None):
        def to_model(q):
            result=np.asarray(q).copy();result[[6,13]]/=1000;return result
        def to_device(q):
            result=np.asarray(q).copy();result[[6,13]]*=1000;return result
        def factory(owner,proxy,feedback,configuration):
            limits=device_limits(proxy.joint_limits)
            adapter=LegacyBaseAdapter(model_to_device=to_device,device_to_model=to_model,position_limits=limits,
                closed_jaw_mm=65.,open_jaw_mm=min(83.,limits[6][1]),project_position_limits=True)
            def observe(now,feedback):
                return dict(source_time=feedback['sample_time_s'],device_state14=device_state(feedback['qpos']),images={})
            default=lambda request:dict(request_id=request['request_id'],action14=np.tile([.02]*6+[.088]+[0.]*7,(60,1)))
            runner=InferenceRunner(owner,proxy,feedback,configuration,predict=predict or default,observe=observe,
                adapter=adapter,record=record,record_prepared=record_prepared,prepared_reference=prepared_reference,support=lambda now,feedback:dict(source_time=feedback['sample_time_s'],
                    tray_supported=True,stable=True,evidence_source='cpu-test-only'))
            self.instances.append(runner)
            return runner
        self.factory=factory
        config=dict(model_identity='CPU contract fixture',seed=8,duration_s=duration_s,execute_steps=execute_steps)
        if recording_backend is not None:config['recording_backend']=recording_backend
        if prepared_reference is not None:config.update(prepared_reference=prepared_reference.metadata,
            trajectory_kind='prepared_native_reference',execution_duration_s=prepared_reference.duration_s+(4.04 if integral else .14),reference_integral_feedback=integral)
        return self.runtime.configure_inference(config,factory)

    def test_native_reference_uses_short_windows_and_stop_revokes_future_without_model(self):
        from robot_workbench.motion_reference import prepare_reference
        q=np.zeros((61,14));q[:,0]=np.linspace(0,.2,61)
        source=dict(device_targets14=q.tolist(),model_id='cpu',calibration_profile_digest='c'*64)
        limits=np.asarray([[-3,3]]*14,dtype=float);limits[[6,13]]=[0,.082]
        compiled=prepare_reference(source,np.zeros(14),limits,source_sha256='a'*64)
        packets=[];future=[];raw=self.runtime.owner.backend._backend;original_stop=raw.set_damping_hold
        def queue(packet):
            packets.append(deepcopy(packet));future[:]=packet['host_times_s']
            return dict(queued_until_host_s=future[-1],firmware_ack=False)
        def stop(**kwargs):future.clear();return original_stop(**kwargs)
        raw.queue_reference=queue;raw.set_damping_hold=stop
        def no_model(_):raise AssertionError('native reference must not query a model')
        self.configure(predict=no_model,duration_s=2,prepared_reference=compiled,integral=True)
        review=self.preview(duration_s=2);self.assertAlmostEqual(review['total_motion_duration_s'],compiled.duration_s+4.04)
        self.confirm(review);self.advance(70)
        runner=self.runtime.owner.runner;self.assertEqual(runner.status,'RUNNING',runner.reason)
        self.assertGreater(len(packets),1);self.assertTrue(future)
        self.assertTrue(all(len(p['host_times_s'])<=32 for p in packets))
        self.assertTrue(all(np.shape(p['torque_bias_sdk6'])==(len(p['host_times_s']),6)for p in packets))
        self.advance(int(compiled.duration_s*100)+30)
        self.assertEqual(runner.status,'RUNNING','feedback must continue during the terminal hold')
        np.testing.assert_allclose(packets[-1]['dq_sdk14'],0.,atol=1e-9)
        self.runtime.request_stop();self.advance(3)
        count=len(packets);self.assertFalse(future);self.advance(100)
        self.assertEqual(len(packets),count);self.assertEqual(runner.status,'STOPPED')
        # A fresh reviewed execution reaches its real retimed endpoint and
        # leaves zero-velocity future windows, without querying the model.
        self.configure(predict=no_model,duration_s=2,prepared_reference=compiled)
        self.confirm(self.preview(duration_s=2));self.advance(int((compiled.duration_s+.8)*100))
        runner=self.runtime.owner.runner
        self.assertEqual(runner.status,'READY',runner.reason)
        np.testing.assert_allclose(packets[-1]['dq_sdk14'],0.,atol=1e-9)
        count=len(packets);self.advance(30);self.assertEqual(len(packets),count)

    def test_execution_length_reaches_loop_and_review_identity(self):
        from robot_workbench.inference_owner import configuration,digest
        for steps in (1,30,60):
            self.assertEqual(configuration({'execute_steps':steps})['execute_steps'],steps)
        for value in (0,61,True,1.5,'30'):
            with self.assertRaises(ValueError):configuration({'execute_steps':value})
        self.assertNotEqual(digest(configuration({'execute_steps':10})),digest(configuration({'execute_steps':30})))
        self.configure(execute_steps=30);review=self.preview()
        self.assertEqual(review['configuration']['execute_steps'],30)
        self.confirm(review);self.advance(40)
        self.assertEqual(self.runtime.owner.runner.loop.execute_steps,30)

    def preview(self,**fields):
        return self.runtime.submit(self.creds['inference'],dict(action='preview_inference',**fields))

    def confirm(self,review):
        return self.runtime.submit(self.creds['inference'],dict(action='confirm',token=review['token'],
            confirmation='START_REVIEWED_SINGLE_ARM',physical_clearance=True))

    def test_confirmation_starts_shared_owner_and_40s_completion_has_only_final_support_goal(self):
        self.configure();before=len(self.runtime.owner.backend.events)
        review=self.preview(seed=13)
        self.assertEqual(len(self.runtime.owner.backend.events),before)
        self.assertEqual(review['plan']['kind'],'inference')
        self.assertEqual(review['duration_s'],40.)
        self.assertFalse(review['collision_checked'])
        self.confirm(review);self.advance(40)
        runner=self.runtime.owner.runner;started=runner.started
        self.assertIsInstance(runner,InferenceRunner)
        while self.now-started<40.+1e-8:self.advance()
        self.assertEqual(runner.status,'READY',runner.reason)
        self.assertTrue(runner.loop.outcome['success'])
        self.assertGreater(runner.loop.completed,10)
        self.assertLessEqual(len(runner.loop.history),60)
        self.assertEqual(runner.config['seed'],13)
        self.assertFalse(hasattr(runner,'frozen'),'native collection contact evaluator must not attach')
        state=self.runtime.snapshot(self.creds['inference'])
        self.assertEqual(state['inference']['runner_status'],'READY')
        self.assertFalse(state['automatic_enabled'])
        count=len(self.runtime.owner.backend.events)
        self.advance(40)
        self.assertEqual(len(self.runtime.owner.backend.events),count,'completed episode must not resume prediction commands')

    def test_stop_and_handoff_discard_late_prediction_without_waiting_for_model(self):
        entered=threading.Event()
        def predict(request):
            entered.set();self.release.wait(2.)
            return dict(request_id=request['request_id'],action14=np.ones((60,14)))
        self.configure(predict);self.confirm(self.preview());self.advance(40)
        self.assertTrue(entered.wait(1.))
        runner=self.runtime.owner.runner
        started=time.monotonic();self.runtime.submit(self.creds['calibration'],{'action':'stop'})
        self.assertLess(time.monotonic()-started,.1)
        self.assertEqual(runner.loop.status,'stopped')
        self.advance(40)
        self.runtime.handoff(self.creds['inference'],'calibration')
        count=len(self.runtime.owner.backend.events)
        self.release.set();self.advance(40)
        self.assertEqual(len(self.runtime.owner.backend.events),count)
        self.assertIsNone(self.runtime.owner.runner)

    def test_two_second_review_and_actual_deadline_are_frozen(self):
        from robot_workbench.inference_owner import configuration, digest
        rows=[]
        self.configure(record=lambda name,value:rows.append((name,value)),duration_s=2)
        with self.assertRaisesRegex(ValueError,'differs'):self.preview(duration_s=40)
        review=self.preview(duration_s=2,seed=8)
        self.assertEqual(review['duration_s'],2)
        self.assertEqual(review['plan']['duration_s'],2)
        self.assertEqual(review['configuration_digest'],digest(review['configuration']))
        self.assertNotEqual(digest(configuration({'duration_s':2})),digest(configuration({'duration_s':40})))
        self.confirm(review);self.advance(40);runner=self.runtime.owner.runner
        self.assertEqual(runner.loop.duration,2)
        end=runner.started+2.
        while self.now<end+1e-8:self.advance()
        self.assertEqual(runner.status,'READY',runner.reason)
        self.assertEqual(runner.reason,'2 source seconds completed')
        count=runner.actual_submission_count;predictions=runner.loop.predictions
        self.advance(100)
        self.assertEqual(runner.actual_submission_count,count)
        self.assertEqual(runner.loop.predictions,predictions)
        self.assertTrue(runner.data_done.wait(1.))
        commands=[value for name,value in rows if name=='command']
        self.assertTrue(commands)
        self.assertTrue(all(row['send_monotonic_s']<end for row in commands))
        config_event=next(value for name,value in rows if name=='configuration')
        self.assertEqual(config_event['source_end_s'],end)

    def test_preloaded_reference_submits_all_sixty_points_without_initial_hold_or_model_query(self):
        def no_model(_):raise AssertionError('fixed reference must not call a model')
        self.configure(predict=no_model,duration_s=2,execute_steps=60)
        original=self.factory
        def factory(*args):
            runner=original(*args)
            runner.preloaded_prediction=dict(request_id='fixed-reference',action14=np.tile([.02]*6+[.088]+[0.]*7,(60,1)),audit=dict(model_called=False))
            return runner
        self.runtime.configure_inference(dict(model_identity='CPU contract fixture',seed=8,duration_s=2,execute_steps=60),factory)
        self.confirm(self.preview(duration_s=2));self.advance(250)
        runner=self.runtime.owner.runner
        self.assertEqual(runner.status,'READY',runner.reason)
        self.assertEqual(runner.actual_submission_count,60)
        self.assertEqual(runner.loop.completed,60)
        self.assertFalse(runner.loop.external_completed)

    def test_one_second_scheduler_has_30hz_boundary_and_discards_late_response(self):
        from test_inference import InferenceTests, observation
        helper=InferenceTests()
        loop,_=helper.loop(duration_s=1)
        for chunk in range(3):
            now=chunk/3
            loop.tick(now,epoch=1,observation=observation(now));helper.await_result(loop)
            for index in range(10):
                tick=chunk*10+index
                command=loop.tick(tick/30,epoch=1)
                self.assertIsNotNone(command)
                loop.acknowledge(command['command_id'],applied_device14=command['final_device14'],source_time=(tick+1)/30)
        self.assertEqual(loop.completed,30)
        predictions=loop.predictions
        self.assertIsNone(loop.tick(1.,epoch=1,observation=observation(1.)))
        self.assertEqual(loop.status,'completed');self.assertEqual(loop.predictions,predictions)
        entered=threading.Event()
        def predict(request):
            entered.set();self.release.wait(2)
            return dict(request_id=request['request_id'],action14=np.zeros((60,14)))
        late,_=helper.loop(predict,duration_s=1)
        late.tick(0.,epoch=1,observation=observation());self.assertTrue(entered.wait(1))
        self.assertIsNone(late.tick(1.,epoch=1,observation=observation(1.)))
        self.release.set();helper.await_result(late)
        self.assertIsNone(late.tick(1.01,epoch=1,observation=observation(1.01)))
        self.assertEqual(late.discarded,1);self.assertEqual(late.submitted,0)

    def test_one_second_sdk_send_rechecks_clock_after_target_conversion(self):
        self.configure(duration_s=1);self.confirm(self.preview(duration_s=1));self.advance(40)
        runner=self.runtime.owner.runner;end=runner.started+1.;before=runner.actual_submission_count
        command=dict(command_id='deadline-test',due_source_time=end-.01,submitted_source_time=end-.01,
                     final_device14=runner.initial_device14)
        self.now=end-.01
        from robot_workbench.inference_owner import sdk_target
        def delayed_target(value):self.now=end;return sdk_target(value)
        with patch('robot_workbench.inference_owner.sdk_target',side_effect=delayed_target):
            runner._submit(command,self.runtime.owner.cache['calibration_feedback'])
        self.assertEqual(runner.actual_submission_count,before)
        self.assertEqual(runner.loop.status,'completed')

    def test_one_second_model_worker_cannot_start_query_after_deadline(self):
        requests=[]
        def predict(request):
            requests.append(request)
            return dict(request_id=request['request_id'],action14=np.zeros((60,14)))
        self.configure(predict,duration_s=1);self.confirm(self.preview(duration_s=1));self.advance(40)
        runner=self.runtime.owner.runner;self.now=runner.started+1.;before=len(requests)
        with self.assertRaisesRegex(ValueError,'deadline'):
            runner.loop.predict({'request_id':'late-worker'})
        self.assertEqual(len(requests),before)

    def test_configuration_calibration_and_expiry_revoke_reviews_without_motion(self):
        self.configure();review=self.preview();before=len(self.runtime.owner.backend.events)
        self.runtime.configure_inference(dict(model_identity='replacement',seed=9),self.factory)
        with self.assertRaisesRegex(ValueError,'unknown'):self.confirm(review)
        review=self.preview();self.confirm(review)
        self.runtime.notify_calibration_installed('new-installation')
        self.advance();self.assertIsNone(self.runtime.owner.runner)
        self.assertFalse(self.runtime.owner.last_confirm_rejection['motion_started'])
        review=self.preview();self.runtime.owner.review_wall_deadline=time.monotonic()-1
        with self.assertRaisesRegex(ValueError,'expired'):self.confirm(review)
        self.assertEqual(len(self.runtime.owner.backend.events),before)

    def test_invalid_model_and_feedback_fault_but_numeric_thresholds_do_not(self):
        self.configure(lambda request:dict(request_id=request['request_id'],action14=np.full((60,14),float('nan'))))
        self.confirm(self.preview());self.advance(45)
        self.assertEqual(self.runtime.owner.runner.status,'FAULT')
        self.assertIn('finite',self.runtime.owner.runner.reason)

    def test_position_projection_is_recorded_and_large_speed_does_not_retime_or_stop(self):
        self.configure(lambda request:dict(request_id=request['request_id'],action14=np.full((60,14),99.)))
        self.confirm(self.preview());self.advance(60)
        runner=self.runtime.owner.runner
        self.assertEqual(runner.status,'RUNNING',runner.reason)
        self.assertGreater(runner.loop.completed,1)
        receipt=next(data for name,data in self.events if name=='inference_prediction_received')
        self.assertTrue(any(row['kind']=='position' for row in receipt['audit']['problems']))
        self.assertTrue(any(row['kind']=='velocity' for row in receipt['final_device_audit']['problems']))
        self.assertEqual(receipt['final_device_audit']['unavailable_acceleration_axes'],[6,13])
        self.assertTrue(receipt['conversions'][0]['projections'])
        self.assertEqual(runner.last_device_target[0],runner.limits[0][1])
        self.assertEqual(runner.loop.period,1/30)

    def test_fake_applied_sequence_mismatch_is_never_fed_to_model_history(self):
        backend=self.runtime.owner.backend._backend
        backend.sequence=0
        read,send=backend.read_calibration_state,backend.send
        def observed():
            return dict(read(),applied_command_sequences={'left':None,'right':None})
        def issue(side,q):
            backend.sequence+=1;send(side,q)
        backend.read_calibration_state,backend.send=observed,issue
        self.advance(2)
        self.configure();self.confirm(self.preview());self.advance(55)
        runner=self.runtime.owner.runner
        self.assertEqual(runner.status,'FAULT')
        self.assertIn('application identity',runner.reason)
        self.assertEqual(runner.loop.completed,0)
        self.assertEqual(len(runner.loop.history),0)

    def test_fresh_feedback_loss_cannot_be_disabled_by_inference_policy(self):
        self.configure();self.confirm(self.preview());self.advance(40)
        self.runtime.owner.backend.frozen_sample=self.now
        self.advance(12)
        self.assertEqual(self.runtime.owner.runner.status,'FAULT')
        self.assertFalse(self.runtime.owner.cache['feedback_healthy'])

    def test_data_worker_never_blocks_stop_and_sealing_blocks_handoff(self):
        entered=threading.Event()
        def record(name,value):
            entered.set();self.release.wait(2.)
        self.configure(record=record);self.confirm(self.preview());self.advance()
        self.assertTrue(entered.wait(1.));runner=self.runtime.owner.runner
        start=time.monotonic();self.runtime.submit(self.creds['inference'],{'action':'stop'});self.advance(40)
        self.assertLess(time.monotonic()-start,.2)
        with self.assertRaisesRegex(ValueError,'recording'):self.runtime.handoff(self.creds['inference'],'collection')
        self.release.set();self.assertTrue(runner.data_done.wait(1.))
        self.runtime.handoff(self.creds['inference'],'collection')

    def test_prepare_has_bounded_wall_wait_then_new_stationary_window_and_actual_initial_hold(self):
        rows=[]
        self.configure(record=lambda name,value:rows.append((name,value)),record_prepared=lambda:False)
        self.confirm(self.preview());self.advance(10)
        runner=self.runtime.owner.runner
        self.assertEqual(runner.status,'VERIFYING')
        self.assertEqual(runner.actual_submission_count,0)
        with patch('time.monotonic',return_value=time.monotonic()+2.):self.advance()
        self.advance(29)
        self.assertIsNone(runner.started)
        self.advance(8)
        self.assertEqual(runner.status,'RUNNING',runner.reason)
        self.assertIn('1 wall second',runner.recording_error)
        self.assertGreaterEqual(runner.actual_submission_count,1)
        self.assertEqual(len(runner.loop.external_completed),1)
        self.assertEqual(runner.command_context['execution_tick'],runner.actual_submission_count-1)
        self.assertTrue(any(name=='prepare' for name,_ in rows))

    def test_resident_capture_waits_for_its_handshake_before_model_admission(self):
        ready=[False]
        self.configure(record=lambda *_:None,record_prepared=lambda:ready[0],recording_backend='capture_session_v1')
        self.confirm(self.preview());self.advance(10)
        runner=self.runtime.owner.runner
        with patch('time.monotonic',return_value=time.monotonic()+2.):self.advance()
        self.assertEqual(runner.status,'VERIFYING')
        self.assertIsNone(runner.recording_error)
        self.assertEqual(runner.actual_submission_count,0)
        ready[0]=True;self.advance(40)
        self.assertEqual(runner.status,'RUNNING',runner.reason)

    def test_resident_capture_handshake_still_has_a_finite_deadline(self):
        self.configure(record=lambda *_:None,record_prepared=lambda:False,recording_backend='capture_session_v1')
        self.confirm(self.preview());self.advance(10)
        runner=self.runtime.owner.runner
        with patch('time.monotonic',return_value=time.monotonic()+11.):self.advance()
        self.assertIn('10 wall seconds',runner.recording_error)

    def test_first_video_command_is_real_hold_with_owner_thread_timestamp(self):
        rows=[]
        self.configure(record=lambda name,value:rows.append((name,value)),record_prepared=lambda:True)
        self.confirm(self.preview());self.advance(40)
        runner=self.runtime.owner.runner
        self.runtime.submit(self.creds['inference'],{'action':'stop'})
        self.assertTrue(runner.data_done.wait(1.))
        commands=[value for name,value in rows if name=='command']
        self.assertEqual(commands[0]['kind'],'initial_hold')
        self.assertEqual(commands[0]['tick'],0)
        self.assertEqual(commands[0]['send_monotonic_s'],runner.started)
        self.assertGreater(commands[0]['send_wall_ns'],0)
        self.assertLessEqual(commands[0]['feedback_at_send']['sample_time_s'],commands[0]['send_monotonic_s'])
        self.assertEqual([value['tick'] for value in commands],list(range(len(commands))))

    def test_recording_finishes_after_measured_feedback_covers_endpoint(self):
        rows=[]
        self.runtime.owner.backend.clock=lambda:self.now-.0325
        self.now+=.04;self.advance(40)
        self.configure(record=lambda name,value:rows.append((name,value)),record_prepared=lambda:True,duration_s=1)
        self.confirm(self.preview(duration_s=1));self.advance(40);runner=self.runtime.owner.runner
        end=runner.started+1.
        while self.now+.01<end-1e-9:self.advance()
        self.now=end-.01;before=runner.actual_submission_count;self.advance()
        self.assertEqual(runner.status,'SETTLING',runner.reason)
        self.assertFalse(runner._data_finish_requested)
        self.assertEqual(runner.actual_submission_count,before)
        self.advance()
        self.assertEqual(runner.status,'READY',runner.reason)
        self.assertEqual(runner.actual_submission_count,before)
        self.assertTrue(runner.data_done.wait(1.))
        feedback=[value['raw']['sample_time_s'] for name,value in rows if name=='feedback']
        self.assertGreaterEqual(feedback[-1],end-.03)
        self.assertLess(feedback[-1],end+.02)
        self.assertEqual(rows[-1][0],'finish')

    def test_120_second_budget_is_bound_to_review_and_stops_at_deadline(self):
        self.configure(duration_s=120)
        review=self.preview(duration_s=120)
        self.assertEqual(review['duration_s'],120.)
        self.confirm(review);self.advance(40)
        runner=self.runtime.owner.runner
        self.now=runner.started+119.9
        self.advance()
        self.assertEqual(runner.status,'RUNNING')
        self.now=runner.started+120.01
        self.advance()
        self.assertEqual(runner.status,'READY')
        self.assertEqual(runner.reason,'120 source seconds completed')

    def test_duration_coverage_accepts_30ms_without_relabelling_feedback(self):
        from robot_workbench.sample_clock import policy_duration_coverage
        for duration in (1.,40.,120.):
            for shortfall in (.0025,.029,.03,.031):
                row=policy_duration_coverage(duration,duration-shortfall)
                self.assertEqual(row['passed'],shortfall<=.03)
                self.assertEqual(row['observed_duration_s'],duration-shortfall)
                self.assertFalse(row['timestamps_modified'])

    def test_wrong_holder_and_unprepared_model_cannot_get_review(self):
        with self.assertRaisesRegex(ValueError,'prepare'):self.preview()
        self.configure()
        for invalid in (60.,3.,0.,True,'1',float('nan')):
            with self.assertRaises(ValueError):self.preview(duration_s=invalid)
        self.runtime.handoff(self.creds['inference'],'collection')
        with self.assertRaises(PermissionError):self.preview()

    def configure_continuous_disabled_right(self):
        backend=self.runtime.owner.backend._backend;backend.passive_jaws=('right',)
        read=backend.read_calibration_state
        def feedback():
            value=read();value['gripper_feedback_valid']=[True,False]
            value['gripper_feedback_reason']=[None,'operator_disabled_no_feedback']
            return value
        backend.read_calibration_state=feedback;self.advance(2)
        rows=[];requests=[]
        def to_model(value):
            result=np.asarray(value).copy();result[[*range(6),*range(7,13)]]+=.01
            result[[6,13]]=(result[[6,13]]-.008)/1000.;return result
        def to_device(value):
            result=np.asarray(value).copy();result[[*range(6),*range(7,13)]]-=.01
            result[[6,13]]=result[[6,13]]*1000.+.008;return result
        def factory(owner,proxy,feedback,config):
            adapter=ContinuousBaseAdapter(model_to_device=to_device,device_to_model=to_model,
                position_limits=device_limits(proxy.joint_limits),disabled_right_gripper_model_m=.082992)
            def predict(request):
                requests.append(deepcopy(request))
                return dict(request_id=request['request_id'],action14=np.tile([.03]*6+[.052242]+[99.]*6+[.001],(60,1)))
            runner=InferenceRunner(owner,proxy,feedback,config,predict=predict,
                observe=lambda now,fb:dict(source_time=fb['sample_time_s'],device_state14=device_state(fb['qpos'],
                    disabled_grippers=('right',),right_gripper_device_mm=83.),images={}),adapter=adapter,
                record=lambda name,value:rows.append((name,value)),record_prepared=lambda:True)
            self.instances.append(runner);return runner
        config=dict(model_id='base566-epoch005-step42130',disabled_grippers=['right'],
            disabled_right_gripper_model_m=.082992,disabled_right_gripper_device_mm=83.,
            disabled_right_gripper_assumption='profile_max_opening_not_measured',
            conversion_identity={'mapping':{'model_gap_offset_mm':.008,'profile_max_opening_mm':83.}})
        self.runtime.configure_inference(config,factory)
        return backend,rows,requests

    def test_continuous_left_gap_and_disabled_right_history_preserve_raw_readback(self):
        backend,rows,requests=self.configure_continuous_disabled_right()
        raw_right=self.runtime.owner.cache['calibration_feedback']['qpos'][13]
        self.confirm(self.preview());self.advance(70)
        runner=self.runtime.owner.runner
        self.assertEqual(runner.status,'RUNNING',runner.reason)
        self.assertAlmostEqual(runner.last_device_target[6],52.25,places=8)
        self.assertAlmostEqual(runner.last_device_target[13],83.,places=8)
        self.assertGreater(len(runner.loop.history),1)
        self.assertTrue(all(abs(row[13]-.082992)<1e-12 for row in runner.loop.history))
        self.assertEqual(runner.last_device_target[7:13],runner.initial_device14[7:13])
        self.runtime.submit(self.creds['inference'],{'action':'stop'});self.assertTrue(runner.data_done.wait(1))
        feedback=[v for n,v in rows if n=='feedback']
        self.assertTrue(feedback)
        self.assertEqual(feedback[0]['raw']['qpos'][13],raw_right)
        self.assertEqual(feedback[0]['unavailable_axes'],[13])
        command=[v for n,v in rows if n=='command'][0]
        self.assertEqual(command['disabled_sdk_slots'],[13])

    def test_disabled_policy_is_bound_before_preview_and_checked_during_execution(self):
        backend,_,_=self.configure_continuous_disabled_right()
        backend.passive_jaws=()
        with self.assertRaisesRegex(ValueError,'disabled-gripper'):self.preview()
        backend.passive_jaws=('right',)
        self.confirm(self.preview());self.advance(60)
        runner=self.runtime.owner.runner;count=runner.actual_submission_count
        backend.passive_jaws=()
        self.advance()
        self.assertEqual(runner.status,'FAULT')
        self.assertIn('disabled-gripper',runner.reason)
        self.assertEqual(runner.actual_submission_count,count)


class CommonCapturePolicyTests(unittest.TestCase):
    def test_recording_fault_uses_frozen_purpose_and_does_not_weaken_motion_guards(self):
        from types import SimpleNamespace
        from unittest.mock import Mock
        from robot_workbench.execution_settings import control_parameters,defaults,policy_hash
        for purpose in ('infer_debug','data_required'):
            runner=object.__new__(InferenceRunner);runner.common_capture=True;runner.recording_error=None
            runner.execution_instance_id='run';runner.owner=SimpleNamespace(control_time=lambda:1.,event=Mock(),recording_failed=Mock())
            parameters=control_parameters(defaults('inference'),business='inference');parameters['recording']['purpose_profile']=purpose
            runner.control_policy=dict(revision=1,policy_hash=policy_hash(parameters),parameters=parameters)
            runner._recording_failure('writer unavailable')
            self.assertEqual(runner.owner.recording_failed.call_count,0 if purpose=='infer_debug' else 1)
            decision=runner.owner.event.call_args.kwargs['decision']
            self.assertEqual(decision['policy_hash'],runner.control_policy['policy_hash'])
            self.assertEqual(decision['action'],'CONTINUE' if purpose=='infer_debug' else 'BRAKE_TO_HOLD')

    def test_concurrent_capture_records_have_one_ordered_sequence(self):
        import queue
        from types import SimpleNamespace
        runner=object.__new__(InferenceRunner);runner.common_capture=True;runner.bounded_live=False
        runner.record=None;runner.recording_error=None;runner._data_finish_requested=False
        runner.data_queue=queue.Queue();runner.capture_lock=threading.Lock();runner.capture_sequence=0
        runner.owner=SimpleNamespace(control_time=time.monotonic)
        threads=[threading.Thread(target=lambda:[runner._record('event',{'value':i}) for i in range(100)]) for _ in range(4)]
        for thread in threads:thread.start()
        for thread in threads:thread.join()
        values=[runner.data_queue.get_nowait()[1] for _ in range(400)]
        self.assertEqual([v['capture_sequence'] for v in values],list(range(1,401)))
        self.assertEqual([v['capture_source_time'] for v in values],sorted(v['capture_source_time'] for v in values))


if __name__=='__main__':unittest.main()
