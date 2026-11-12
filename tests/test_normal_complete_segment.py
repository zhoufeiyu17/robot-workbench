"""Approved normal policy on real Runtime/Owner; mocked devices, no hardware."""
from copy import deepcopy
import unittest
import numpy as np
import test_normal_modes as normal
from test_infer_curve_policy import POLICY
from robot_workbench.motion_reference import prepare_reference,retime_for_brake_reserve,check_brake_reserve


class CompleteNormalTests(unittest.TestCase):
    profile='normal_X5'
    setUp=normal.NormalModeTests.setUp
    cleanup=normal.NormalModeTests.cleanup
    confirm=normal.NormalModeTests.confirm
    setup_modes=normal.NormalModeTests.setup_modes
    until=normal.NormalModeTests.until
    advance=normal.NormalModeTests.advance
    request=normal.NormalModeTests.request
    completed=normal.NormalModeTests.completed
    finish_request=normal.NormalModeTests.finish_request

    def setup_reviewed(self,duration=.5):
        self.setup_modes(recording=True,finish_kind='dagger_reset_v1',initial_mode='HOLD',reviewed=True,reviewed_duration_s=duration)

    def test_planning_update_preserves_current_segment_and_applies_next(self):
        self.setup_reviewed(40.);runner=self.runtime.owner.runner
        request,_=self.request('MODEL');self.completed(request)
        self.until(lambda:runner.normal_segment is not None and runner.normal_segment['status']=='executing')
        first=deepcopy(runner.normal_segment);old=runner.config['reference_planning']['velocity_limit_rad_s']
        policy=self.runtime.control_policy_snapshot('inference')
        changed=self.runtime.update_control_policy(self.creds['inference'],dict(expected_revision=policy['revision'],
            changes={'planning.velocity_limit_rad_s':old/2,'planning.jaw_velocity_limit_mm_s':25.}))
        self.assertEqual(changed['parameters']['planning']['velocity_limit_rad_s'],old/2)
        self.assertEqual(runner.normal_segment,first)
        self.until(lambda:runner.normal_segment['reference_id']!=first['reference_id'])
        self.assertEqual(runner.normal_segment['planning_velocity_limit_rad_s'],old/2)
        self.assertEqual(runner.normal_segment['planning_jaw_velocity_limit_mm_s'],25.)
        self.assertEqual(self.runtime.owner.runner,runner)
        self.assertTrue(self.runtime.execution_settings_path.is_file())

    def test_live_planning_rejects_stale_revision_and_preserves_failed_write(self):
        from unittest.mock import patch
        self.setup_reviewed(40.);runner=self.runtime.owner.runner
        policy=self.runtime.control_policy_snapshot('inference')
        update=dict(expected_revision=policy['revision'],changes={'planning.velocity_limit_rad_s':.2})
        with self.assertRaisesRegex(ValueError,'revision'):
            self.runtime.update_control_policy(self.creds['inference'],dict(update,expected_revision=policy['revision']+1))
        with patch('pathlib.Path.write_text',side_effect=OSError('disk failure')):
            with self.assertRaises(OSError):self.runtime.update_control_policy(self.creds['inference'],update)
        self.assertEqual(self.runtime.control_policy_snapshot('inference')['parameters'],policy['parameters'])
        self.assertIsNone(runner.pending_planning_update);self.assertFalse(self.runtime.settings_write_pending)
        for field,value in [('planning.velocity_limit_rad_s',3.),('planning.jaw_velocity_limit_mm_s',50.)]:
            with self.assertRaisesRegex(ValueError,'上限'):
                self.runtime.update_control_policy(self.creds['inference'],dict(expected_revision=policy['revision'],changes={field:value}))

    def check_endpoint_delta(self,changes,accepted):
        from unittest.mock import patch
        self.setup_reviewed();runner=self.runtime.owner.runner
        request,_=self.request('MODEL');self.completed(request)
        self.until(lambda:runner.normal_segment['status']=='verifying_endpoint')
        read=runner._normal_output
        def output():
            value=read()
            for field,axis,delta in changes:value[field][axis]+=delta
            return value
        with patch.object(runner,'_normal_output',side_effect=output):
            for _ in range(20):
                self.advance()
                if runner.normal_segment['status']=='completed'or runner.status=='FAULT':break
            else:self.fail('endpoint did not reach a decision')
        if accepted:
            self.assertEqual(runner.normal_segment['status'],'completed');self.assertNotEqual(runner.status,'FAULT')
        else:
            self.assertEqual(runner.status,'FAULT');self.assertIn('native endpoint mismatch',runner.reason)
            self.assertTrue(any(name=='inference_normal_model_endpoint_mismatch'for name,_ in self.events))
            diagnostic=self.runtime.owner.last_fault_diagnostic['native_command_mismatch']
            self.assertEqual(diagnostic['phase'],'complete_segment_endpoint')
            self.assertTrue(diagnostic['position_mismatch_axes']or diagnostic['velocity_mismatch_axes'])
            self.assertIsNotNone(diagnostic['last_submitted_window'])

    def test_native_endpoint_accepts_declared_joint_and_jaw_position_velocity_errors(self):
        self.check_endpoint_delta([(field,axis,9e-7 if axis in (6,13)else 9e-6)
            for field in ('q_sdk14','dq_sdk14')for axis in range(14)],True)

    def test_native_endpoint_rejects_joint_position_above_tolerance(self):
        self.check_endpoint_delta([('q_sdk14',0,1.1e-5)],False)

    def test_native_endpoint_rejects_joint_velocity_above_tolerance(self):
        self.check_endpoint_delta([('dq_sdk14',7,1.1e-5)],False)

    def test_native_endpoint_rejects_jaw_position_above_tolerance(self):
        self.check_endpoint_delta([('q_sdk14',6,1.1e-6)],False)

    def test_native_endpoint_rejects_jaw_velocity_above_tolerance(self):
        self.check_endpoint_delta([('dq_sdk14',13,1.1e-6)],False)

    def test_initial_hold_uses_declared_native_command_tolerances(self):
        from unittest.mock import patch
        from robot_workbench.execution_settings import NativeCommandMismatchError
        self.setup_reviewed();runner=self.runtime.owner.runner
        output=runner._normal_output()
        for axis in range(14):
            output['q_sdk14'][axis]+=9e-7 if axis in (6,13)else 9e-6
            output['dq_sdk14'][axis]+=9e-7 if axis in (6,13)else 9e-6
        with patch.object(runner,'_normal_output',return_value=output):
            runner._verify_initial_hold()
            output['q_sdk14'][0]+=2e-6
            with self.assertRaises(NativeCommandMismatchError)as failed:runner._verify_initial_hold()
        self.assertEqual(failed.exception.diagnostic['phase'],'initial_joint_hold')
        self.assertEqual(failed.exception.diagnostic['position_mismatch_axes'],['left_J1'])

    def test_release_receipt_retains_native_fault_after_sdk_close(self):
        self.check_endpoint_delta([('q_sdk14',7,1.1e-5)],False)
        expected=deepcopy(self.runtime.owner.last_fault_diagnostic)
        self.advance(50)
        reply=self.runtime.request_release(self.creds['inference'])
        self.assertEqual(reply['release_receipt']['fault_diagnostic'],expected)
        self.until(lambda: not self.runtime.owner.backend.is_connected)
        self.runtime.close()
        self.assertEqual(self.runtime.release_receipt['status'],'released')
        self.assertEqual(self.runtime.release_receipt['fault_diagnostic'],expected)

    def test_production_model_continues_fresh_segments_without_raw_gate_transitions(self):
        self.setup_reviewed(40.);runner=self.runtime.owner.runner
        request,_=self.request('MODEL');self.completed(request)
        def completed():return [data for name,data in self.events if name=='inference_normal_model_segment_completed']
        self.until(lambda:len(completed())>=2)
        rows=completed();self.assertEqual(runner.mode,'MODEL');self.assertIsNone(runner.normal_job)
        self.assertFalse(getattr(runner,'normal_supply_exhausted',False))
        self.assertGreater(rows[1]['observation_source_s'],rows[0]['completed_source_s'])
        self.assertTrue(all(row['complete_source_steps']==15 for row in rows))
        self.assertFalse(any(name=='inference_normal_pause_required'for name,_ in self.events))
        epoch=self.runtime.owner.epoch;self.request('MODEL')
        self.assertEqual(self.runtime.owner.epoch,epoch,'already-MODEL intent cannot restart the pipeline')
        request,_=self.request('HOLD',gate=True);self.completed(request)
        count=len(self.requests);self.advance(80);self.assertEqual(len(self.requests),count)

    def test_stall_recovery_consumes_actual_complete_segment_feedback_and_resets_on_hold(self):
        from types import SimpleNamespace
        from unittest.mock import Mock
        import threading,json
        from robot_workbench.wam import StallRecovery
        from robot_workbench.inference_workflow import InferenceWorkflow
        self.setup_reviewed(40.);runner=self.runtime.owner.runner
        recovery=StallRecovery({'left':80.,'right':89.});decisions=[];sequence=0
        flow=SimpleNamespace(client=None,lock=threading.RLock(),current={'id':runner.execution_instance_id,
            'owner':self.runtime.owner,'stall_recovery':recovery},_metric_event=Mock())
        original_record=runner._record
        def record(name,value):
            nonlocal sequence
            sequence+=1
            message=SimpleNamespace(run_id=runner.execution_instance_id,event='inference_'+name,
                reference=SimpleNamespace(source_instance_id=self.runtime.nonce,sequence=sequence),
                payload_json=json.dumps(value))
            InferenceWorkflow._ros_control_event(flow,message)
            original_record(name,value)
        runner._record=record
        predict=runner.predict
        def prediction(request):
            decisions.append(recovery.choose(request['seed']))
            return predict(request)
        runner.predict=prediction
        request,_=self.request('MODEL');self.completed(request)
        self.until(lambda:any(row['stalled']for row in decisions),count=2000)
        first=next(row for row in decisions if row['stalled'])
        self.assertEqual(first['observed_steps'],90)
        self.assertTrue(first['head_black']);self.assertNotEqual(first['actual_seed'],first['configured_seed'])
        request,_=self.request('HOLD',gate=True);self.completed(request)
        self.assertEqual(recovery.choose(1)['observed_steps'],0)

    def test_episode_budget_finishes_only_whole_segments_then_uses_original_gate(self):
        self.setup_reviewed(1.);runner=self.runtime.owner.runner
        request,_=self.request('MODEL');self.completed(request)
        self.until(lambda:runner.snapshot()['transition_stage']=='AWAITING_RECORDING_GATE')
        self.assertEqual(runner.normal_segment['status'],'completed')
        self.assertLessEqual(runner.normal_segment['end_source_s'],runner.started+1.)
        self.assertEqual(runner.normal_segment['complete_source_steps'],15)
        self.assertFalse(runner.normal_ended)

    def test_complete_source_segment_waits_for_raw_gate_and_does_not_end_episode(self):
        self.setup_reviewed();runner=self.runtime.owner.runner
        self.advance(80);self.assertEqual(runner.mode,'HOLD');self.assertEqual(self.requests,[])
        request,_=self.request('MODEL');self.completed(request)
        self.until(lambda:runner.snapshot()['transition_stage']=='AWAITING_RECORDING_GATE')
        segment=runner.normal_segment
        self.assertEqual(segment['status'],'completed');self.assertEqual(segment['complete_source_steps'],15)
        self.assertEqual(segment['source_duration_s'],.5);self.assertLessEqual(segment['execution_duration_s'],2.)
        self.assertFalse(runner.normal_ended);self.assertEqual(len(self.requests),1)
        self.advance(80);self.assertEqual(runner.status,'RUNNING');self.assertEqual(len(self.requests),1)
        request,_=self.request('HOLD',gate=True);self.completed(request)
        request,_=self.request('TEACH',gate=True);self.completed(request)
        with self.assertRaises(ValueError):self.request('MODEL')
        request,_=self.request('HOLD',gate=True);self.completed(request)
        request,_=self.request('END',gate=True);self.completed(request)
        self.assertTrue(runner.normal_ended)
        self.runtime.submit(self.creds['inference'],self.finish_request(request))
        self.until(lambda:runner.finish_completed,count=6000)
        self.assertTrue(self.raw.is_connected)

    def test_early_hold_marks_partial_and_fault_stop_does_not_wait_for_raw_gate(self):
        self.setup_reviewed();runner=self.runtime.owner.runner
        request,_=self.request('MODEL');self.completed(request)
        request,_=self.request('HOLD',gate=True);self.completed(request)
        self.assertEqual(runner.normal_segment['status'],'interrupted')
        self.assertEqual(runner.normal_segment['complete_source_steps'],0)
        request,_=self.request('MODEL');self.completed(request)
        before=len(self.raw.packets);self.runtime.request_stop();self.advance(5)
        self.assertEqual(runner.status,'STOPPED');self.assertIsNone(self.raw.window)
        self.advance(60);self.assertEqual(len(self.raw.packets),before)

    def test_approved_parameters_are_exact_and_old_normal_defaults_remain_bounded(self):
        self.setup_reviewed()
        from robot_workbench.inference_owner import configuration
        config=deepcopy(self.runtime.owner.runner.config)
        for field,value in [('brake_duration_s',.5),('gain_transition_s',.1),('teach_kd_scale',.2)]:
            bad=deepcopy(config);bad['normal_modes'][field]=value
            with self.assertRaises(ValueError):configuration(bad)
        bad=deepcopy(config);bad.pop('reference_planning')
        with self.assertRaises(ValueError):configuration(bad)
        bad=deepcopy(config);bad['finite_chunk_count']=4
        with self.assertRaises(ValueError):configuration(bad)

    def test_pending_model_stop_discards_late_result_without_any_model_window(self):
        import threading
        self.setup_reviewed();runner=self.runtime.owner.runner
        entered=threading.Event();release=threading.Event();original=runner.predict
        def predict(request):
            entered.set();release.wait(2);return original(request)
        runner.predict=predict;self.addCleanup(release.set)
        self.request('MODEL');self.until(entered.is_set)
        self.runtime.request_stop();self.advance(2);count=len(self.raw.packets)
        release.set();runner.bounded_worker.join(2);self.advance(30)
        self.assertEqual(runner.status,'STOPPED');self.assertEqual(len(self.raw.packets),count)
        self.assertIsNone(runner.normal_segment)

    def test_timed_out_model_cannot_overlap_or_contaminate_a_new_model_request(self):
        import threading
        self.setup_reviewed();runner=self.runtime.owner.runner
        entered=threading.Event();release=threading.Event();original=runner.predict;calls=[]
        def predict(request):
            calls.append(request['request_id'])
            if len(calls)==1:entered.set();release.wait(3)
            return original(request)
        runner.predict=predict;self.addCleanup(release.set)
        first,_=self.request('MODEL');self.until(entered.is_set)
        self.until(lambda:runner.mode_receipts[first['request_id']]['status']=='failed')
        self.assertTrue(runner.mode_receipts[first['request_id']]['retained_hold']['confirmed'])
        second,_=self.request('MODEL');self.advance(10);self.assertEqual(len(calls),1)
        old=runner.bounded_worker;release.set();old.join(2);self.completed(second)
        self.assertEqual(len(calls),2);self.assertEqual(runner.normal_segment['request_id'],calls[1])

    def test_real_adapter_accepts_normal_brake_above_legacy5_only_with_bound_authority(self):
        from unittest.mock import patch
        from test_prepared_reference_backend import PreparedBackendTests
        from robot_workbench.motion_reference import prepare_brake
        from robot_workbench.target_interpreter import TargetInterpreter
        from robot_workbench.sdk_process import ReferenceWindowRejected
        q=np.zeros(14);v=np.zeros(14);v[0]=.8
        limits=np.tile([-3.,3.],(14,1));limits[[6,13]]=[0,.082]
        brake=prepare_brake(q,v,np.zeros(14),limits,duration_s=.2,velocity_limit=1.,acceleration_limit=40.,source_sha256='b'*64)
        backend=PreparedBackendTests().context_backend();now=[1000.];origin=1000.02
        context=dict(controller_profile='normal_X5',jaw_calibration_id='c'*64,
            limits=dict(velocity_limit_rad_s=1.,acceleration_limit_rad_s2=40.,jaw_velocity_limit_mm_s=40.))
        def output():
            q,v,_=brake.sample([max(0.,now[0]-origin)])
            return dict(q_sdk14=q[0].tolist(),dq_sdk14=v[0].tolist(),
                observations={side:dict(sample_monotonic_ns=round(now[0]*1e9)) for side in ('left','right')})
        backend.read_execution_output=output
        backend.bind_execution_context(context)
        interpreter=TargetInterpreter(backend,brake,active_arm='both',start_s=origin,
            epoch=1,current_epoch=lambda:1,gate=lambda:None,clock=lambda:now[0])
        rejected=False
        with patch('time.monotonic',side_effect=lambda:now[0]):
            for elapsed in np.arange(0,.37,.02):
                now[0]=1000.+float(elapsed)
                result=interpreter.tick(now[0],dict(qpos=q.tolist(),sample_time_s=now[0]))
                if result and not rejected:
                    backend.bind_execution_context({k:v for k,v in context.items() if k!='limits'})
                    try:backend.queue_reference(result['packet'])
                    except ReferenceWindowRejected as error:
                        self.assertIn('acceleration admission',str(error));rejected=True
                    finally:backend.bind_execution_context(context)
        self.assertTrue(rejected)
        self.assertTrue(backend._controllers['left'].future)

    def test_brake_reserve_retime_preserves_every_target_or_rejects_full_segment(self):
        q=np.zeros((16,14));q[:,0]=np.linspace(0,.12,16)
        limits=np.tile([-3.,3.],(14,1));limits[[6,13]]=[0,.082]
        r=prepare_reference(dict(kind='online_prediction_chunk',device_targets14=q.tolist(),duration_s=.5,active_arms=['left','right']),
            q[0],limits,source_sha256='a'*64,sdk_targets14=q,jaw_scales={'left':1000.,'right':1000.},
            planning_policy=POLICY,initial_velocity_sdk14=[0.]*14,initial_acceleration_sdk14=[0.]*14)
        result=retime_for_brake_reserve(r,limits,duration_s=.2,velocity_limit=1.,acceleration_limit=40.,maximum_duration_s=2.)
        check_brake_reserve(result,limits,duration_s=.2,velocity_limit=1.,acceleration_limit=40.)
        np.testing.assert_allclose(result.sample(result.breaks)[0],q,atol=1e-10)
        with self.assertRaises(ValueError):retime_for_brake_reserve(r,limits,duration_s=.2,velocity_limit=1.,acceleration_limit=40.,maximum_duration_s=.01)


if __name__=='__main__':unittest.main()
