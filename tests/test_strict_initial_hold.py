"""Released-jaw output follows feedback until the existing begin fixes a hold."""
from copy import deepcopy
import unittest
from unittest.mock import patch
import test_normal_modes as normal


class ReleasedJawBackend(normal.NormalBackend):
    before_jaws=(.08142049533336265,.08174040495225869)
    later_jaws=(.08141083705906593,.08173079084172062)

    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.q[6],self.q[13]=self.before_jaws
        self.drift_at=float('inf');self.begin_calls=[]
        for value in self.mode_gains.values():value['gripper_kp']=value['gripper_kd']=0.

    def read_calibration_state(self):
        for side,i,index in(('left',6,0),('right',13,1)):
            if self.mode_gains[side]['gripper_kp']==0:
                self.q[i]=(self.later_jaws if self.clock()>=self.drift_at else self.before_jaws)[index]
            elif self.window is None and self.applied_command is not None:self.q[i]=self.applied_command[i]
        return super().read_calibration_state()

    def read_execution_output(self):
        result=super().read_execution_output()
        for side,i in(('left',6),('right',13)):
            if self.mode_gains[side]['gripper_kp']==0:result['q_sdk14'][i]=self.q[i]
        return result

    def begin(self,side,q,*,preserve_jaws=False):
        self.begin_calls.append((side,self.clock()))
        result=super().begin(side,q,preserve_jaws=preserve_jaws)
        self.mode_gains[side]['gripper_kp']=5.;self.mode_gains[side]['gripper_kd']=.2
        return result


class StrictInitialHoldTests(unittest.TestCase):
    profile='normal_X5'
    setUp=normal.NormalModeTests.setUp
    cleanup=normal.NormalModeTests.cleanup
    setup_modes=normal.NormalModeTests.setup_modes
    advance=normal.NormalModeTests.advance

    def until(self,predicate,count=500):
        if getattr(self,'setup_only',False):return
        return normal.NormalModeTests.until(self,predicate,count)

    def confirm(self,business,review):
        from robot_workbench.inference_model import MIXED366_ID
        config=deepcopy(self.runtime.inference_configuration);factory=self.runtime.inference_factory
        for name in ('normal_modes','normal_initial_mode','normal_finish_kind','normal_finish_policy','_normal_finish'):config.pop(name,None)
        config.update(bounded_live_prediction=True,duration_s=.5,maximum_observation_age_s=.5,execute_steps=15,
            model_id=MIXED366_ID,task='coffee',target_execution='shared_reference_windows')
        self.runtime.configure_inference(config,factory)
        review=self.runtime.submit(self.creds['inference'],dict(action='preview_inference',duration_s=.5))
        return normal.NormalModeTests.confirm(self,business,review)

    def prepare(self):
        self.setup_only=True
        with patch.object(normal,'NormalBackend',ReleasedJawBackend):self.setup_modes(finish_kind='maintenance_home_v1')
        # Change passive feedback before the first hold, independent of startup latency.
        self.setup_only=False;self.raw.drift_at=self.now
        for gains in self.raw.mode_gains.values():
            gains['gripper_kp']=gains['gripper_kd']=0.
        self.advance()
        return self.runtime.owner.runner

    def test_native_passive_drift_binds_once_before_new_observation_and_model_budget(self):
        runner=self.prepare()
        self.until(lambda:runner.initialization_source_s is not None)
        self.assertIsNone(runner.started);self.assertEqual(runner.actual_submission_count,0)
        self.assertFalse(self.requests);self.assertEqual(len(self.raw.begin_calls),2)
        expected=runner.initialization['begin_command']['command_sdk14']
        self.assertEqual([expected[6],expected[13]],list(ReleasedJawBackend.later_jaws))
        observe=runner.observe
        runner.observe=lambda now,fb:dict(observe(now,fb),source_time=runner.initialization_source_s-.01)
        self.advance(40);self.assertFalse(self.requests,'pre-initialization images cannot seed a model request')
        runner.observe=observe
        self.until(lambda:runner.actual_submission_count>0)
        self.assertGreater(self.requests[0]['observation']['source_time'],runner.initialization_source_s)
        self.assertGreater(runner.started,runner.initialization_source_s)
        self.assertEqual(runner.execution_duration_s,.5)
        self.assertEqual(runner.prepared_reference.metadata['initial_sdk_command14'],expected)
        self.assertEqual(len(self.raw.begin_calls),2,'no second begin after model preparation')
        self.assertTrue(any(name=='inference_initial_hold_returned'for name,_ in self.events))

    def test_real_command_change_after_anchor_faults_without_reprepare_or_model_windows(self):
        runner=self.prepare();predict=runner.predict
        def changed(request):
            result=predict(request);self.raw.applied_command[13]-=9.61411053807e-6
            return result
        runner.predict=changed
        for _ in range(500):
            self.advance()
            if runner.status=='FAULT':break
        self.assertEqual(runner.status,'FAULT',runner.reason)
        self.assertIn('strict initialized jaw hold changed',runner.reason)
        self.assertEqual(runner.actual_submission_count,0);self.assertEqual(len(self.requests),1)
        self.assertEqual(len(self.raw.begin_calls),2)
        self.assertEqual(runner.initialization['stage'],'failed')

    def test_stop_during_initialization_never_queries_or_restores_old_reference(self):
        runner=self.prepare();self.until(lambda:runner.initialization_source_s is not None)
        self.runtime.request_stop();self.advance(45)
        self.assertEqual(runner.status,'STOPPED');self.assertFalse(self.requests)
        self.assertEqual(runner.actual_submission_count,0);self.assertIsNone(self.raw.window)
        self.assertTrue(self.runtime.stop_receipt['stop_hold_return_confirmed'])
        self.assertEqual(runner.initialization['stage'],'cancelled')

    def test_model_failure_after_initial_hold_stops_without_model_window(self):
        runner=self.prepare();self.model_error=True
        for _ in range(500):
            self.advance()
            if runner.status=='FAULT':break
        self.assertEqual(runner.status,'FAULT');self.assertIn('CPU model supply unavailable',runner.reason)
        self.assertEqual(runner.actual_submission_count,0);self.assertIsNone(self.raw.window)
        self.assertEqual(runner.initialization['stage'],'failed')

    def test_stop_between_arm_begins_cancels_remaining_initialization(self):
        runner=self.prepare();begin=self.raw.begin
        def interrupted(side,q,**kwargs):
            result=begin(side,q,**kwargs)
            if side=='left':self.runtime.request_stop()
            return result
        self.raw.begin=interrupted
        self.advance(80)
        self.assertEqual(runner.status,'STOPPED');self.assertEqual(len(self.raw.begin_calls),1)
        self.assertEqual(runner.actual_submission_count,0);self.assertFalse(self.requests)
        self.assertTrue(self.runtime.stop_receipt['stop_hold_return_confirmed'])


if __name__=='__main__':unittest.main()
