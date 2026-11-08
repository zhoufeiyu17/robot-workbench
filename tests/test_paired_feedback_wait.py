"""Exercise the real refresh/feedback/pairing/wait chain with CPU sample streams."""
from copy import deepcopy
import threading
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock,patch

from robot_workbench.inference_model import MIXED366_ID
from robot_workbench.inference_workflow import InferenceWorkflow,CAMERAS
from robot_workbench.observation_pairing import TimestampPairer,PairingPending


class PairedFeedbackWaitTests(unittest.TestCase):
    def frames(self,stamps):
        return {role:dict(width=640,height=480,rgb=bytes(640*480*3),metadata=dict(
            serial=role,capture_instance_id='capture',environment='physical',clock_domain='host_wall',
            host_receive_ns=10_000_000_000+round(stamp*1e9),frame_number=round(stamp*1000)))
            for role,stamp in zip(CAMERAS,stamps)}

    def feedback(self,stamp):
        return dict(sample_time_s=stamp,sample_wall_ns=10_000_000_000+round(stamp*1e9),
            qpos=[0.]*14,gripper_feedback_valid=[True,True])

    def setUp(self):
        self.now=.150;self.clock=0.;self.sleeps=[];self.healthy=True;self.current_age=0.
        self.owner=dict(runtime_nonce='runtime',command_epoch=1)
        flow=object.__new__(InferenceWorkflow);self.flow=flow
        flow.ros_plane=None  # This fixture explicitly exercises the retained legacy pairing path.
        flow.lock=threading.RLock();flow.observation_lock=threading.Lock();flow.generation=1
        flow.prepared=dict(binding={'camera_serials':{r:r for r in CAMERAS}},binding_digest='binding',
            profile={'gripper':{'max_opening_mm':83.}},capture_instances={c+'_rgb':'capture'for c in CAMERAS.values()})
        flow._binding_current=lambda prepared:True;flow.selected_model_id=MIXED366_ID
        flow.disabled_grippers=[];flow.support_provider=None;flow.cache=None
        self.current_frames=self.frames([.116,.143,.060])
        flow.rgb_source=NS(latest_rgb=lambda camera:self.current_frames[next(r for r,c in CAMERAS.items()if c==camera)])
        flow.device=NS(environment='physical',runtime=NS(remote_control_owner=True,push_observation=Mock(),
            control_inputs=lambda:dict(calibration_feedback=self.feedback(self.now-self.current_age),
                feedback_healthy=self.healthy,control_time_s=self.now,joint_limits=[[-3,3]]*14,
                disabled_grippers=[],**self.owner)))
        flow.observation_pairer=TimestampPairer()
        flow.observation_pairer.pair(self.frames([0.,0.,0.]),self.feedback(0.),self.owner)
        flow.observation_pairer.pair(self.frames([.033,.040,.030]),self.feedback(.035),self.owner)

    def wait(self,on_sleep=None):
        def sleep(delay):
            self.sleeps.append(delay);self.clock+=delay;self.now+=delay
            if on_sleep:on_sleep()
        with patch('robot_workbench.inference_workflow.time.monotonic',side_effect=lambda:self.clock),\
             patch('robot_workbench.inference_workflow.time.sleep',side_effect=sleep):
            return self.flow._wait_observation()

    def test_old_paired_feedback_waits_for_new_qualified_sample_without_restamping(self):
        self.wait(lambda:setattr(self,'current_frames',self.frames([self.now]*3)))
        self.assertEqual(self.sleeps,[.005])
        cached=self.flow.cache['observation']
        self.assertAlmostEqual(cached['feedback_sample_time_s'],.150)
        self.assertEqual(cached['feedback_sample_wall_ns'],10_150_000_000)
        self.assertLessEqual(self.now-cached['feedback_sample_time_s'],.1)
        self.assertAlmostEqual(cached['source_time'],.116)
        self.flow.device.runtime.push_observation.assert_called_once()

    def test_permanent_history_lag_still_fails_at_original_half_second_budget(self):
        with self.assertRaises(PairingPending):self.wait()
        self.assertGreaterEqual(self.clock,.5);self.assertLess(self.clock,.506)
        self.assertIsNone(self.flow.cache);self.flow.device.runtime.push_observation.assert_not_called()

    def test_current_hardware_fault_or_stale_feedback_is_not_retried(self):
        for healthy,age in ((False,0.),(True,.101)):
            with self.subTest(healthy=healthy,age=age):
                self.healthy,self.current_age=healthy,age
                with self.assertRaises(ValueError)as caught:self.wait()
                self.assertNotIsInstance(caught.exception,PairingPending);self.assertEqual(self.sleeps,[])
        self.flow.device.runtime.push_observation.assert_not_called()

    def test_fault_arriving_during_wait_stops_on_the_next_sample(self):
        with self.assertRaises(ValueError)as caught:self.wait(lambda:setattr(self,'healthy',False))
        self.assertNotIsInstance(caught.exception,PairingPending);self.assertEqual(self.sleeps,[.005])
        self.flow.device.runtime.push_observation.assert_not_called()

    def test_camera_identity_changes_reject_before_history_can_wait(self):
        for key,value in (('serial','different'),('environment','fake'),('capture_instance_id','restarted')):
            with self.subTest(key=key):
                original=deepcopy(self.current_frames['head']['metadata'])
                self.current_frames['head']['metadata'][key]=value
                with self.assertRaises(ValueError)as caught:self.wait()
                self.assertNotIsInstance(caught.exception,PairingPending);self.assertEqual(self.sleeps,[])
                self.assertEqual(caught.exception.guard_decision['rule_id'],'G01')
                self.assertEqual(caught.exception.guard_decision['action'],'INVALIDATE_RUN')
                self.current_frames['head']['metadata']=original

    def test_binding_change_is_not_retried(self):
        self.flow._binding_current=lambda prepared:False
        with self.assertRaises(ValueError)as caught:self.wait()
        self.assertNotIsInstance(caught.exception,PairingPending);self.assertEqual(self.sleeps,[])

    def test_future_paired_feedback_is_not_reclassified_as_pending(self):
        self.flow.observation_pairer=NS(pair=lambda frames,feedback,owner:(frames,self.feedback(self.now+.01),{}))
        with self.assertRaises(ValueError)as caught:self.wait()
        self.assertNotIsInstance(caught.exception,PairingPending);self.assertEqual(self.sleeps,[])

    def test_camera_half_second_age_limit_is_unchanged(self):
        self.current_frames['head']['metadata']['host_receive_ns']=10_000_000_000+round((self.now-.501)*1e9)
        self.flow.observation_pairer=NS(pair=lambda frames,feedback,owner:(frames,feedback,{}))
        with self.assertRaisesRegex(ValueError,'0.5 秒')as caught:self.wait()
        self.assertNotIsInstance(caught.exception,PairingPending);self.assertEqual(self.sleeps,[])


if __name__=='__main__':unittest.main()
