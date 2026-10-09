import copy
import unittest
from unittest.mock import Mock, patch
import cv2
import numpy as np
from arx_env.joint_head_workbench_capture import CaptureError, capture_sample, rectify_corners


class CaptureTest(unittest.TestCase):
    def setUp(self):
        self.k=dict(serial='camera-1',width=640,height=480,fx=400.,fy=400.,cx=320.,cy=240.,
                    coefficients=[0.]*5,distortion_model='distortion.inverse_brown_conrady',rectified=False)
        image=np.full((480,640),255,np.uint8)
        draw=getattr(cv2.aruco,'generateImageMarker',None) or cv2.aruco.drawMarker
        image[100:200,100:200]=draw(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),30,100)
        self.raw=cv2.imencode('.png',image)[1].tobytes()
        self.meta=dict(raw=True,annotated=False,image_domain='raw',metadata_matched=True,ready=True,
                       serial='camera-1',intrinsics=self.k,host_receive_ns=1_050_000_000,frame_number=12)
        self.camera=Mock();self.camera.read_calibration_frame.return_value=(self.raw,self.meta)
        self.history=[]
        for stamp,sdk in [(1_000_000_000,1.),(1_090_000_000,2.)]:
            self.history.append(dict(owner=dict(pid=123,command_epoch=4,feedback_healthy=True,
                calibration_feedback=dict(sample_wall_ns=stamp,qpos=[0.]*14,sdk_timestamps=[sdk,sdk]))))

    def capture(self,**kwargs):
        return capture_sample({'left':self.camera},{'left':self.k},lambda:copy.deepcopy(self.history),
                              owner_pid=123,command_epoch=4,**kwargs)

    def test_raw_bytes_detected_corners_and_stationary_bracket(self):
        result=self.capture(required_markers={'left':[30]})
        self.assertEqual(result['raw_images']['left'],self.raw)
        row=result['rows']['left']
        self.assertEqual(len(row['detections']['30']),4)
        self.assertEqual(row['q_reported12'],[0.]*12)
        self.assertEqual(row['bracket_gap_s'],.09)
        self.assertFalse(row['synchronized'])
        self.assertTrue(result['images']['left'].startswith(b'\x89PNG'))
        self.assertTrue(result['cameras']['left']['rectified'])
        self.assertEqual(result['row']['camera_detections']['left'],row['detections_rectified'])
        np.testing.assert_allclose(row['detections']['30'],row['detections_rectified']['30'])

    def test_each_camera_must_postdate_current_capture_readiness(self):
        ready=self.meta['host_receive_ns']
        self.assertTrue(self.capture(not_before_ns=ready)['rows'])
        with self.assertRaisesRegex(CaptureError,'predates'):self.capture(not_before_ns=ready+1)
        stale=Mock();stale.read_calibration_frame.return_value=(self.raw,{**self.meta,'host_receive_ns':ready-1})
        with self.assertRaisesRegex(CaptureError,'predates'):
            capture_sample({'left':self.camera,'right':stale},{'left':self.k,'right':self.k},lambda:self.history,
                           owner_pid=123,command_epoch=4,not_before_ns=ready)

    def test_waits_for_future_feedback_without_reading_new_frames(self):
        history=Mock(side_effect=[self.history[:1],self.history])
        with patch('arx_env.joint_head_workbench_capture.time.sleep') as sleep:
            capture_sample({'left':self.camera},{'left':self.k},history,owner_pid=123,command_epoch=4)
        self.assertEqual(self.camera.read_calibration_frame.call_count,1)
        sleep.assert_called_once()
        with self.assertRaises(CaptureError):
            capture_sample({'left':self.camera},{'left':self.k},lambda:[],owner_pid=123,command_epoch=4,feedback_wait_s=0)

    def test_failures_retain_image_and_do_not_accept_annotated_or_wrong_identity(self):
        for key,value in [('annotated',True),('raw',False),('metadata_matched',False),('serial','wrong')]:
            with self.subTest(key=key):
                meta={**self.meta,key:value};self.camera.read_calibration_frame.return_value=(self.raw,meta)
                with self.assertRaises(CaptureError) as caught:self.capture()
                self.assertEqual(caught.exception.evidence['raw_images']['left'],self.raw)
        self.camera.read_calibration_frame.return_value=(self.raw,self.meta)
        evidence=self.capture(required_markers={'left':[24]})
        self.assertEqual(evidence['rows']['left']['missing_markers'],[24])

    def test_bracket_gap_owner_motion_and_sdk_fail_closed(self):
        pristine=copy.deepcopy(self.history)
        mutations=[lambda: self.history.pop(),
                   lambda: self.history[1]['owner'].update(pid=999),
                   lambda: self.history[1]['owner'].update(command_epoch=5),
                   lambda: self.history[1]['owner']['calibration_feedback'].update(sample_wall_ns=1_200_000_000),
                   lambda: self.history[1]['owner']['calibration_feedback'].update(sdk_timestamps=[1.,2.]),
                   lambda: self.history[1]['owner']['calibration_feedback']['qpos'].__setitem__(0,.006)]
        for mutation in mutations:
            self.history=copy.deepcopy(pristine);mutation()
            with self.assertRaises(CaptureError):self.capture()

    def test_missing_bracket_preserves_timing_and_reader_failure(self):
        self.history.pop()
        with self.assertRaisesRegex(CaptureError,'bracket missing') as caught:
            self.capture(feedback_wait_s=0,feedback_diagnostics=lambda:{'read_error':'read timeout'})
        evidence=caught.exception.evidence
        self.assertEqual(evidence['camera_metadata']['left']['host_receive_ns'],1_050_000_000)
        self.assertEqual(evidence['feedback_history']['oldest_ns'],1_000_000_000)
        self.assertEqual(evidence['feedback_history']['newest_ns'],1_000_000_000)
        self.assertEqual(evidence['feedback_reader']['read_error'],'read timeout')

    def test_inverse_brown_corners_remain_finite(self):
        k={**self.k,'coefficients':[-.05,.06,.001,.0004,-.02]}
        uv=np.array([[100.,100.],[199.,100.],[199.,199.],[100.,199.]])
        corrected=rectify_corners(uv,k)
        self.assertTrue(np.isfinite(corrected).all())
        self.assertGreater(np.max(abs(corrected-uv)),.1)
        self.k.update(k)
        result=capture_sample({'left':self.camera},None,lambda:self.history,owner_pid=123,command_epoch=4)
        self.assertEqual(result['cameras']['left']['coefficients'],[0.]*5)
        self.assertEqual(cv2.imdecode(np.frombuffer(result['images']['left'],np.uint8),cv2.IMREAD_COLOR).shape,(480,640,3))

    def test_owner_failure_identifies_cause_and_keeps_compact_evidence(self):
        for change,label in [({'pid':999},'owner PID changed'),({'command_epoch':5},'command epoch changed'),
                             ({'feedback_healthy':False},'feedback unhealthy')]:
            with self.subTest(label=label):
                owner=self.history[1]['owner']
                owner.update(pid=123,command_epoch=4,feedback_healthy=True,state='FAULT',reason='stationary lost',
                             last_fault_diagnostic={'reasons':['position_span'],'ring':['do not duplicate']})
                owner.update(change)
                with self.assertRaisesRegex(CaptureError,label) as caught:self.capture()
                error=caught.exception
                self.assertIn('position_span',str(error));self.assertIn('state=FAULT',str(error))
                self.assertEqual(error.evidence['expected_owner'],{'pid':123,'command_epoch':4})
                self.assertEqual(error.evidence['failed_owner']['fault_reasons'],['position_span'])
                self.assertNotIn('ring',str(error.evidence['failed_owner']))
                self.assertEqual(error.evidence['raw_images']['left'],self.raw)

    def make_fake(self):
        self.meta.update(environment='fake',simulation_only=True,timestamp_kind='simulation_step',
            clock_domain='simulation',source_clock_id='fixture-plant',source_time_ns=50_000_000,physics_step=12,
            host_receive_ns=9_000_000_000)
        for row,source,step,wall in zip(self.history,(25_000_000,75_000_000),(6,18),(3_000_000_000,4_000_000_000)):
            row['owner'].update(environment='fake',mode='simulation',simulation_only=True)
            row['owner']['calibration_feedback'].update(clock_domain='simulation',source_clock_id='fixture-plant',
                source_time_ns=source,physics_step=step,sample_wall_ns=wall)
        return dict(environment='fake',source_clock_id='fixture-plant',not_before_source_ns=40_000_000)

    def test_fake_pairs_on_source_time_without_rewriting_real_wall_metadata(self):
        options=self.make_fake()
        result=self.capture(**options)
        row=result['rows']['left']
        self.assertEqual(row['host_receive_ns'],9_000_000_000)
        self.assertEqual(row['source_time_ns'],50_000_000)
        self.assertEqual(row['feedback_bracket']['before_ns'],25_000_000)
        self.assertEqual(row['feedback_bracket']['after_ns'],75_000_000)
        self.assertEqual(row['feedback_bracket']['physics_step_before'],6)
        self.assertEqual(row['feedback_bracket']['physics_step_after'],18)
        self.assertEqual(row['clock']['source_clock_id'],'fixture-plant')
        self.assertEqual(result['row']['runtime_environment'],'fake')
        with self.assertRaises(CaptureError):self.capture()  # No implicit physical fallback.
        with self.assertRaisesRegex(CaptureError,'source readiness'):self.capture(**options,not_before_ns=123)
        with self.assertRaisesRegex(CaptureError,'predates'):self.capture(**{**options,'not_before_source_ns':51_000_000})

    def test_fake_rejects_clock_step_identity_epoch_and_unbracketed_frames(self):
        options=self.make_fake();meta=copy.deepcopy(self.meta);history=copy.deepcopy(self.history)
        changes=[lambda:self.meta.update(source_clock_id='other-plant'),
            lambda:self.meta.update(clock_domain='host_wall'),lambda:self.meta.update(environment='physical'),
            lambda:self.meta.pop('source_time_ns'),lambda:self.meta.update(source_time_ns=100_000_000),
            lambda:self.meta.update(physics_step=99),
            lambda:self.history[1]['owner']['calibration_feedback'].update(physics_step=5),
            lambda:self.history[1]['owner']['calibration_feedback'].update(source_clock_id='restarted-plant'),
            lambda:self.history[1]['owner'].update(command_epoch=5),lambda:self.history[1]['owner'].update(pid=999),
            lambda:self.history.reverse()]
        for change in changes:
            self.meta=copy.deepcopy(meta);self.history=copy.deepcopy(history)
            change();self.camera.read_calibration_frame.return_value=(self.raw,self.meta)
            with self.subTest(change=change),self.assertRaises(CaptureError):self.capture(**options,feedback_wait_s=0)

    def test_exact_fake_source_step_still_uses_advancing_feedback_bracket(self):
        options=self.make_fake()
        middle=copy.deepcopy(self.history[0]);middle['owner']['calibration_feedback'].update(
            source_time_ns=50_000_000,physics_step=12,sdk_timestamps=[1.5,1.5])
        self.history.insert(1,middle)
        self.assertEqual(self.capture(**options)['rows']['left']['bracket_gap_s'],.05)
        self.meta['physics_step']=13
        with self.assertRaisesRegex(CaptureError,'different physics step'):self.capture(**options)


if __name__=='__main__':unittest.main()
