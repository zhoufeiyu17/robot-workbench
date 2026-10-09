"""Clock/identity corruption checks; CPU fixtures, no physical timing qualification."""
from copy import deepcopy
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from robot_workbench.sample_clock import clock_spec,clock_for_source,sample_stamp,assert_progress,bracket_samples
from physical_motiongen import freeze_physical
from real_observation import capture, decode_snapshot
from real_recording import RealRecorder, bundle
from test_fake_device_pipeline import fake_inputs, fake_metadata, FAKE_CLOCK_ID, FAKE_RUNTIME_NONCE
from test_real_observation import fixture as snapshot


class SourceClockTest(unittest.TestCase):
    def test_actual_wall_receipt_never_advances_sim_source(self):
        clock=clock_spec('fake',source_clock_id='plant-a')
        sample=dict(clock_domain='simulation',source_clock_id='plant-a',source_time_ns=0,physics_step=0,host_receive_ns=10_000)
        first=sample_stamp(sample,clock)
        latest=sample_stamp(dict(sample,host_receive_ns=90_000_000_000),clock)
        self.assertEqual(first,(0,0));self.assertEqual(first,latest)
        with self.assertRaises(ValueError):assert_progress(first,latest)
        assert_progress(first,latest,allow_same=True)
        self.assertEqual(sample_stamp({'host_receive_ns':100},clock_spec()),(100,None))

    def test_unmarked_wrong_domain_instance_time_and_step_are_rejected(self):
        clock=clock_spec('fake',source_clock_id='plant-a')
        good=dict(clock_domain='simulation',source_clock_id='plant-a',source_time_ns=1,physics_step=1,host_receive_ns=100)
        for patch in ({'clock_domain':None},{'clock_domain':'host_wall'},{'source_clock_id':'plant-b'},
                      {'source_time_ns':None},{'source_time_ns':-1},{'physics_step':True},{'environment':'physical'}):
            with self.subTest(patch=patch),self.assertRaises(ValueError):sample_stamp(good|patch,clock)
        with self.assertRaises(ValueError):clock_for_source({},'fake')
        with self.assertRaises(ValueError):clock_for_source({'clock':dict(clock,environment='physical')},'fake')
        with self.assertRaises(ValueError):sample_stamp(good,clock_spec())
        for current in ((0,2),(2,0),(2,1),(1,2)):
            with self.subTest(current=current),self.assertRaises(ValueError):assert_progress((1,1),current)

    def test_brackets_keep_real_left_right_steps_and_never_forge_exact(self):
        samples=[(0,0),(10,2),(20,4)]
        self.assertEqual(bracket_samples(samples,(10,2)),(1,1))
        self.assertEqual(bracket_samples(samples,(15,3)),(1,2))
        for target in ((10,3),(15,2),(15,5),(21,5),(-1,0)):
            with self.subTest(target=target),self.assertRaises(ValueError):bracket_samples(samples,target)

    def test_camera_owner_unavailable_changed_step_and_producer_fault_are_rejected(self):
        source,_,_=fake_inputs();profile=source['calibration_profile']
        changes=[dict(owner_available=False),dict(owner_pid=None),dict(runtime_nonce=None),
                 dict(timestamp_kind='host_receive'),dict(data_valid=False,recording_error='source queue overflow'),
                 dict(depth_physics_step=241),dict(depth_device_timestamp_ms=1001.),dict(timestamp_domain='hardware_clock'),
                 dict(frame_readback=dict(clock_domain='simulation',source_clock_id=FAKE_CLOCK_ID,source_time_ns=1_000_000_000,physics_step=241))]
        for change in changes:
            def metadata(meta):fake_metadata(meta);meta.update(change)
            body=snapshot(1,metadata)
            with self.subTest(change=change),self.assertRaises(ValueError):decode_snapshot(body,'h',profile,environment='fake')
            with self.subTest(change=change),self.assertRaises(ValueError):bundle(body,'h',environment='fake')

    def test_wall_span_cannot_satisfy_simulation_observation_window(self):
        source,_,binding=fake_inputs(); index=[0]; wall=[0.]
        def read():
            index[0]+=1
            def metadata(meta):
                fake_metadata(meta,index[0]*10_000_000)
                meta['host_receive_ns']=index[0]*1_000_000_000
            return snapshot(index[0],metadata)
        with TemporaryDirectory() as folder:
            path=Path(folder)/'capture'
            with self.assertRaises(TimeoutError):
                capture(read,source['calibration_profile'],binding,path,environment='fake',timeout_s=9.,
                    monotonic=lambda:wall[0],sleep=lambda seconds:wall.__setitem__(0,wall[0]+seconds))
            self.assertFalse((path/'rgbd-manifest.json').exists())

    def test_equal_time_wrong_frame_step_invalidates_raw_without_motion_fault(self):
        source,model,binding=fake_inputs();plan=freeze_physical(model,source,binding,environment='fake')
        identity=dict(owner_pid=1234,command_epoch=3,runtime_nonce=FAKE_RUNTIME_NONCE)
        fields=dict(clock_domain='simulation',source_clock_id=FAKE_CLOCK_ID,physics_step=24000,**identity)
        with TemporaryDirectory() as folder:
            recorder=RealRecorder(Path(folder)/'raw',source,plan,binding,environment='fake')
            recorder.record_command(0,plan['ticks'][0]['target'],100.,1_000_000_000,send_source_time_ns=100_000_000_000,**(fields|dict(physics_step=None,feedback_at_send={**fields,'source_time_ns':100_000_000_000,'sample_wall_ns':1_000_000_000})))
            recorder.record_feedback(0,plan['ticks'][0]['target'],100.,1_000_000_000,source_time_ns=100_000_000_000,**fields)
            body=snapshot(1,lambda meta:fake_metadata(meta,100_000_000_000,step=24001))
            recorder.record_rgbd('base_0',body)
            result=recorder.seal('completed')
            self.assertEqual(result['controller_outcome'],'completed')
            self.assertEqual(result['capture_status'],'incomplete')
            self.assertTrue(any('different physics step' in problem for problem in result['validation']['problems']))

    def test_command_between_physics_steps_keeps_send_and_feedback_times_separate(self):
        source,model,binding=fake_inputs();plan=freeze_physical(model,source,binding,environment='fake')
        identity=dict(owner_pid=1234,command_epoch=3,runtime_nonce=FAKE_RUNTIME_NONCE)
        sampled=dict(clock_domain='simulation',source_clock_id=FAKE_CLOCK_ID,
                     source_time_ns=100_000_000_000,physics_step=24000,sample_wall_ns=1_000_000_000)
        with TemporaryDirectory() as folder:
            recorder=RealRecorder(Path(folder)/'raw',source,plan,binding,environment='fake')
            recorder.record_command(0,plan['ticks'][0]['target'],100.002,1_002_000_000,
                clock_domain='simulation',source_clock_id=FAKE_CLOCK_ID,send_source_time_ns=100_002_000_000,
                feedback_at_send=sampled,**identity)
            row=recorder.commands[0]
            self.assertIsNone(row['physics_step'])
            self.assertEqual(row['send_source_time_ns']-row['feedback_at_send']['source_time_ns'],2_000_000)
            self.assertEqual(row['feedback_at_send']['physics_step'],24000)
            for sample in (sampled|{'source_time_ns':100_035_000_001},
                           sampled|{'source_time_ns':99_000_000_000},sampled|{'source_clock_id':'foreign'}):
                with self.subTest(sample=sample),self.assertRaises(ValueError):
                    recorder.record_command(1,plan['ticks'][1]['target'],100.035,1_035_000_000,
                        clock_domain='simulation',source_clock_id=FAKE_CLOCK_ID,send_source_time_ns=100_035_000_000,
                        feedback_at_send=sample,**identity)

    def test_recorder_rejects_unmarked_events_and_changed_plant_or_owner(self):
        source,model,binding=fake_inputs();plan=freeze_physical(model,source,binding,environment='fake')
        identity=dict(owner_pid=1234,command_epoch=3,runtime_nonce=FAKE_RUNTIME_NONCE)
        with TemporaryDirectory() as folder:
            root=Path(folder)
            with self.assertRaises(ValueError):RealRecorder(root/'other',source,plan,binding,environment='fake',source_clock_id='different')
            recorder=RealRecorder(root/'raw',source,plan,binding,environment='fake')
            with self.assertRaises(ValueError):recorder.record_command(0,plan['ticks'][0]['target'],100.,1_000_000_000)
            recorder.record_command(0,plan['ticks'][0]['target'],100.,1_000_000_000,clock_domain='simulation',
                source_clock_id=FAKE_CLOCK_ID,send_source_time_ns=100_000_000_000,physics_step=None,
                feedback_at_send=dict(clock_domain='simulation',source_clock_id=FAKE_CLOCK_ID,source_time_ns=100_000_000_000,physics_step=24000,sample_wall_ns=1_000_000_000),**identity)
            for change in ({'source_clock_id':'different'},{'command_epoch':4},{'runtime_nonce':'new-owner'}):
                values=dict(clock_domain='simulation',source_clock_id=FAKE_CLOCK_ID,source_time_ns=100_000_000_000,physics_step=24000,**identity)|change
                with self.subTest(change=change),self.assertRaises(ValueError):
                    recorder.record_feedback(0,plan['ticks'][0]['target'],100.,1_000_000_000,**values)


if __name__=='__main__':unittest.main()
