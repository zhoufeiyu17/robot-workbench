"""Clock/identity corruption checks; CPU fixtures, no physical timing qualification."""
from copy import deepcopy
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch
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
                    wall_deadline_ns=10_000_000_000,wall_ns=lambda:1_000_000_000+round(wall[0]*1e9),
                    monotonic=lambda:wall[0],sleep=lambda seconds:wall.__setitem__(0,wall[0]+seconds))
            self.assertFalse((path/'rgbd-manifest.json').exists())

    def test_slow_simulation_seals_nine_source_spaced_pairs_after_thirty_wall_seconds(self):
        source,_,binding=fake_inputs();wall=[0.];base=1_000_000_000_000;reads=[]
        def read():
            step=int(wall[0]*60)  # .25 source seconds per wall second at 240 Hz.
            stamp=step*1_000_000_000//240;reads.append(stamp)
            return snapshot(step,lambda meta:fake_metadata(meta,stamp,step=step))
        with TemporaryDirectory() as folder:
            path=Path(folder)/'capture'
            capture(read,source['calibration_profile'],binding,path,environment='fake',
                wall_deadline_ns=base+60_000_000_000,wall_ns=lambda:base+round(wall[0]*1e9),
                monotonic=lambda:wall[0],sleep=lambda seconds:wall.__setitem__(0,wall[0]+seconds))
            manifest=json.loads((path/'rgbd-manifest.json').read_text());rows=manifest['frames']
            self.assertEqual(wall[0],32.)
            self.assertEqual(len(rows),9);self.assertGreater(len(reads),len(rows))
            self.assertEqual([row['source_stamp_ns'] for row in rows],[i*1_000_000_000 for i in range(9)])
            self.assertEqual([row['physics_step'] for row in rows],[i*240 for i in range(9)])
            self.assertTrue(all(row['source_time_ns']==row['source_stamp_ns'] for row in rows))
            self.assertFalse((path/'capture-diagnostic.json').exists())

    def test_frozen_source_times_out_in_thirty_wall_seconds_and_retains_timestamped_partial(self):
        source,_,binding=fake_inputs();wall=[0.];base=1_000_000_000_000
        body=snapshot(1,lambda meta:fake_metadata(meta,1_000_000_000))
        with TemporaryDirectory() as folder:
            path=Path(folder)/'capture'
            with self.assertRaisesRegex(TimeoutError,'source did not advance for 30 wall seconds'):
                capture(lambda:body,source['calibration_profile'],binding,path,environment='fake',
                    wall_deadline_ns=base+1_200_000_000_000,wall_ns=lambda:base+round(wall[0]*1e9),
                    monotonic=lambda:wall[0],sleep=lambda seconds:wall.__setitem__(0,wall[0]+seconds))
            evidence=json.loads((path/'capture-diagnostic.json').read_text())
            self.assertEqual(evidence['elapsed_wall_s'],30.)
            self.assertEqual(evidence['retained_frame_count'],1)
            self.assertEqual(evidence['source_stamp_range_ns'],[1_000_000_000]*2)
            self.assertEqual(evidence['retained_source_span_ns'],0)
            self.assertEqual(evidence['frames'][0]['physics_step'],240)
            self.assertTrue((path/evidence['frames'][0]['path']).is_file())
            self.assertEqual(evidence['last_observed']['source_stamp_ns'],1_000_000_000)
            self.assertFalse((path/'rgbd-manifest.json').exists());self.assertFalse((path/'acquisition-binding.json').exists())

    def test_advancing_source_still_obeys_fixed_deadline_cancel_and_wall_clock_rollback(self):
        for kind in ('deadline','cancel','wall_rollback'):
            source,_,binding=fake_inputs();wall=[0.];base=1_000_000_000_000
            def read():
                step=int(wall[0]*60)
                return snapshot(step,lambda meta:fake_metadata(meta,step*1_000_000_000//240,step=step))
            def wall_ns():return base+round((wall[0] if kind!='wall_rollback' or wall[0]<5 else -100)*1e9)
            with self.subTest(kind=kind),TemporaryDirectory() as folder:
                path=Path(folder)/'capture'
                error=InterruptedError if kind=='cancel' else TimeoutError
                with self.assertRaises(error):
                    capture(read,source['calibration_profile'],binding,path,environment='fake',
                        wall_deadline_ns=base+10_000_000_000,wall_ns=wall_ns,
                        monotonic=lambda:wall[0],sleep=lambda seconds:wall.__setitem__(0,wall[0]+seconds),
                        cancel=lambda:kind=='cancel' and wall[0]>=2)
                evidence=json.loads((path/'capture-diagnostic.json').read_text())
                self.assertEqual(evidence['elapsed_wall_s'],2. if kind=='cancel' else 10.)
                self.assertLess(evidence['retained_source_span_ns'],8_000_000_000)
                self.assertEqual(len(evidence['frames']),len(list(path.glob('*.npz'))))
                self.assertFalse(evidence['manifest_sealed']);self.assertFalse((path/'rgbd-manifest.json').exists())

    def test_capture_rejects_regression_identity_change_and_new_pair_at_same_step(self):
        source,_,binding=fake_inputs()
        for kind in ('regression','clock','owner','same_step'):
            def change(meta):
                fake_metadata(meta,500_000_000 if kind=='regression' else 1_000_000_000)
                if kind=='clock':meta['source_clock_id']='another-plant'
                if kind=='owner':meta['runtime_nonce']='another-owner'
            frames=iter((snapshot(1,lambda meta:fake_metadata(meta,1_000_000_000)),snapshot(2,change)))
            wall=[0.];base=1_000_000_000_000
            with self.subTest(kind=kind),TemporaryDirectory() as folder:
                path=Path(folder)/'capture'
                with self.assertRaises(ValueError):
                    capture(lambda:next(frames),source['calibration_profile'],binding,path,environment='fake',
                        wall_deadline_ns=base+60_000_000_000,wall_ns=lambda:base+round(wall[0]*1e9),
                        monotonic=lambda:wall[0],sleep=lambda seconds:wall.__setitem__(0,wall[0]+seconds))
                self.assertFalse((path/'rgbd-manifest.json').exists())
                self.assertEqual(json.loads((path/'capture-diagnostic.json').read_text())['retained_frame_count'],1)

    def test_physical_default_timeout_remains_total_thirty_wall_seconds(self):
        from integration import _digest
        from test_harness import PROFILE
        profile=deepcopy(PROFILE);binding=dict(calibration_accepted=True,evidence_source='physical_rgbd',robot_id='fixture',
            camera_serials=dict(head='h',left='l',right='r'),calibration_profile_digest=_digest(profile))
        wall=[0.];index=[0]
        def read():
            index[0]+=1
            return snapshot(index[0],lambda meta:meta.update(host_receive_ns=index[0]*10_000_000))
        with TemporaryDirectory() as folder:
            path=Path(folder)/'capture'
            with self.assertRaisesRegex(TimeoutError,'physical observation'):
                capture(read,profile,binding,path,monotonic=lambda:wall[0],
                    sleep=lambda seconds:wall.__setitem__(0,wall[0]+seconds))
            evidence=json.loads((path/'capture-diagnostic.json').read_text())
            self.assertEqual(evidence['elapsed_wall_s'],30.)
            self.assertEqual(evidence['retained_frame_count'],30)
            self.assertEqual(evidence['timeout_meaning'],'total_wall_watchdog')
            self.assertFalse((path/'rgbd-manifest.json').exists())

    def test_same_step_cannot_hide_changed_metadata_or_pixels(self):
        source,_,binding=fake_inputs();profile=source['calibration_profile']
        decoded=decode_snapshot(snapshot(1,lambda meta:fake_metadata(meta,1_000_000_000)),
            'h',profile,environment='fake',clock=binding['clock'],expected_owner=binding['capture_owner'])
        for field in ('host_receive_ns','original_intrinsics','frame_readback','rgb','depth_m'):
            changed=deepcopy(decoded)
            if field=='host_receive_ns':changed[0][field]+=1
            elif field=='original_intrinsics':changed[0][field]['distortion_model']='changed'
            elif field=='frame_readback':changed[0][field]={'untrusted':'changed readback'}
            else:changed[-1][field].flat[0]+=1
            wall=[0.];base=1_000_000_000_000
            with self.subTest(field=field),TemporaryDirectory() as folder, \
                    patch('real_observation.decode_snapshot',side_effect=[decoded,changed]):
                path=Path(folder)/'capture'
                with self.assertRaisesRegex(ValueError,'same source step changed'):
                    capture(lambda:b'unused by decoded fixture',profile,binding,path,environment='fake',
                        wall_deadline_ns=base+60_000_000_000,wall_ns=lambda:base+round(wall[0]*1e9),
                        monotonic=lambda:wall[0],sleep=lambda seconds:wall.__setitem__(0,wall[0]+seconds))
                self.assertFalse((path/'rgbd-manifest.json').exists())

    def test_frames_snapshot_request_age_changes_but_same_sample_can_be_polled(self):
        import io
        import zipfile
        from real_rgbd import Frames,snapshot_bundle
        source,_,binding=fake_inputs();wall=[0.];base=1_000_000_000_000
        frames=Frames();calls=[0];ages=[];first_two=[]
        def read():
            index=calls[0];calls[0]+=1
            if index!=1:  # Second request polls the same producer frame.
                number=max(1,index)
                body=snapshot(number,lambda meta:fake_metadata(meta,number*1_000_000_000))
                with zipfile.ZipFile(io.BytesIO(body)) as archive:
                    frames.publish(status=json.loads(archive.read('metadata.json')),
                        color_png=archive.read('color.png'),depth_png=archive.read('depth.png'))
            frame=frames.snapshot();ages.append(frame['status']['source_age_s'])
            if index<2:first_two.append(deepcopy(frame))
            self.assertNotIn('source_age_s',frames.latest['status'])
            return snapshot_bundle(frame['status'],frame['color_png'],frame['depth_png'])
        def wall_ns():return base+round(wall[0]*1e9)
        with TemporaryDirectory() as folder,patch('real_rgbd.time.time_ns',wall_ns):
            path=Path(folder)/'capture'
            capture(read,source['calibration_profile'],binding,path,environment='fake',
                wall_deadline_ns=base+60_000_000_000,wall_ns=wall_ns,
                monotonic=lambda:wall[0],sleep=lambda seconds:wall.__setitem__(0,wall[0]+seconds))
            self.assertEqual(ages[:2],[0.,.25])
            self.assertEqual(first_two[0]['color_png'],first_two[1]['color_png'])
            self.assertEqual(first_two[0]['depth_png'],first_two[1]['depth_png'])
            self.assertEqual([key for key in first_two[0]['status']
                if first_two[0]['status'][key]!=first_two[1]['status'][key]],['source_age_s'])
            rows=json.loads((path/'rgbd-manifest.json').read_text())['frames']
            self.assertEqual(len(rows),9)
            self.assertEqual([row['source_stamp_ns'] for row in rows],[i*1_000_000_000 for i in range(1,10)])

    def test_diagnostic_disk_error_does_not_replace_original_capture_error(self):
        source,_,binding=fake_inputs();original=OSError('camera read failed')
        def fail():raise original
        with TemporaryDirectory() as folder,patch.object(Path,'write_text',side_effect=OSError('disk full')):
            with self.assertRaises(OSError) as caught:
                capture(fail,source['calibration_profile'],binding,Path(folder)/'capture',environment='fake',
                    wall_deadline_ns=60_000_000_000,wall_ns=lambda:1_000_000_000)
            self.assertIs(caught.exception,original)
            if hasattr(original,'__notes__'):self.assertIn('disk full',original.__notes__[0])

    def test_physical_final_storage_latency_keeps_existing_success_semantics(self):
        from integration import _digest
        from test_harness import PROFILE
        import numpy as np
        profile=deepcopy(PROFILE);binding=dict(calibration_accepted=True,evidence_source='physical_rgbd',robot_id='fixture',
            camera_serials=dict(head='h',left='l',right='r'),calibration_profile_digest=_digest(profile))
        wall=[0.];frames=iter(snapshot(i) for i in range(9));save=np.savez_compressed;count=[0]
        def slow_final_save(*args,**kwargs):
            save(*args,**kwargs);count[0]+=1
            if count[0]==9:wall[0]+=25
        with TemporaryDirectory() as folder,patch('numpy.savez_compressed',slow_final_save):
            path=Path(folder)/'capture'
            capture(lambda:next(frames),profile,binding,path,monotonic=lambda:wall[0],
                sleep=lambda seconds:wall.__setitem__(0,wall[0]+seconds))
            self.assertEqual(wall[0],33.)
            self.assertTrue((path/'acquisition-binding.json').exists())

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
