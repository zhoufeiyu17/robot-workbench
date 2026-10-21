"""Ingress decisions are unchanged; only bounded numeric evidence is retained."""
import json
import hashlib
import cv2
from pathlib import Path
import sys
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch,Mock

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from robot_workbench.capture_diagnostics import CaptureDiagnostics,BINS_NS,unsupported
from rule_based_workbench.real_rgbd import FrameProgress


def counters(rgb,depth):
    return dict(rgb_number=rgb,depth_number=depth,rgb_device_ms=rgb*33.,depth_device_ms=depth*33.)


class CaptureDiagnosticsTest(unittest.TestCase):
    def test_real_capture_loop_rejects_bad_pair_recovers_preview_and_expires_prolonged_loss(self):
        import numpy as np
        from rule_based_workbench.real_rgbd import capture,Frames
        from robot_workbench.rgbd_service import CAMERAS
        for prolonged in (False,True):
            with self.subTest(prolonged=prolonged),tempfile.TemporaryDirectory() as folder:
                camera=CAMERAS['left'];frames=Frames();clock=[0.];number=[0];published=[]
                intr=SimpleNamespace(width=4,height=2,fx=2.,fy=2.,ppx=2.,ppy=1.,model='none',coeffs=[0.]*5)
                def pair(index,bad):
                    def frame(depth):
                        return SimpleNamespace(get_frame_number=lambda:index,get_timestamp=lambda:index*33.+(100. if depth and bad else 0.),
                            get_frame_timestamp_domain=lambda:'global_time',
                            get_data=lambda:np.zeros((2,4) if depth else (2,4,2),dtype=np.uint16 if depth else np.uint8),
                            profile=SimpleNamespace(as_video_stream_profile=lambda:SimpleNamespace(get_intrinsics=lambda:intr)))
                    return SimpleNamespace(get_color_frame=lambda:frame(False),get_depth_frame=lambda:frame(True))
                def wait(_timeout):
                    number[0]+=1
                    if number[0]>20:clock[0]+=1.1 if prolonged else .04
                    return pair(number[0],number[0]==21 or prolonged and number[0]>20)
                pipeline=SimpleNamespace(wait_for_frames=wait,stop=Mock(),start=lambda _:SimpleNamespace(
                    get_device=lambda:SimpleNamespace(first_depth_sensor=lambda:SimpleNamespace(get_depth_scale=lambda:.001))))
                rs=SimpleNamespace(pipeline=lambda:pipeline,config=lambda:SimpleNamespace(enable_device=lambda _:None,enable_stream=lambda *a:None),
                    stream=SimpleNamespace(color=1,depth=2),format=SimpleNamespace(yuyv=1,z16=2),align=lambda _:SimpleNamespace(process=lambda p:p))
                channel=SimpleNamespace(token=lambda:'current-recording',fail=Mock())
                def publish(*args,**kwargs):
                    published.append(args)
                    if len(published)==2:frames.stop_requested.set()
                service=SimpleNamespace(descriptor=dict(environment='physical',intrinsics={camera:{'serial':'123'}}),
                    directories={camera:folder},channels={camera:channel},publish=publish)
                args=SimpleNamespace(service_descriptor=Path(folder)/'descriptor',camera_name='left',serial='123',output=Path(folder),duration_s=0)
                with patch.dict(sys.modules,{'pyrealsense2':rs}),patch('robot_workbench.rgbd_service.CommonCameraService.connect',return_value=service),\
                        patch('rule_based_workbench.real_rgbd.time.monotonic',side_effect=lambda:clock[0]):
                    if prolonged:
                        with self.assertRaisesRegex(RuntimeError,'3 seconds'):capture(args,frames)
                    else:capture(args,frames)
                pipeline.stop.assert_called_once()
                self.assertEqual(len(published),0 if prolonged else 2)
                self.assertGreater(channel.fail.call_count,0)
                diagnostics=json.loads((Path(folder)/'capture-diagnostics.json').read_text())
                self.assertEqual(diagnostics['accepted'],0 if prolonged else 2)
                self.assertGreater(diagnostics['rejected'],0)

    def test_bad_timestamp_pair_is_rejected_without_advancing_valid_frame_or_stall_budget(self):
        progress=FrameProgress(0.)
        self.assertFalse(progress.accept(1,1,.1,timestamps=(100.,134.,'global','global',30)))
        self.assertEqual(progress.last_rejection,'timestamp_mismatch');self.assertIsNone(progress.published)
        self.assertEqual(progress.last_fresh,0.)
        self.assertTrue(progress.accept(2,2,.2,timestamps=(200.,200.,'global','global',30)))
        self.assertEqual(progress.published,(2,2))
        with self.assertRaisesRegex(RuntimeError,'3 seconds'):
            progress.accept(3,3,3.3,timestamps=(300.,350.,'global','global',30))

    def test_journal_explains_an_event_free_capture_and_clean_end(self):
        with tempfile.TemporaryDirectory() as folder:
            d=CaptureDiagnostics(folder,'healthy')
            for i in range(3):d.attempt(counters(i,i),counters(i,i),True,'accepted',host_receive_ns=100+i)
            d.close(reason='stop_requested')
            rows=[json.loads(line) for line in d.journal_path.read_text().splitlines()]
            self.assertEqual(rows[0]['record_type'],'capture_started')
            self.assertTrue(any(row['record_type']=='capture_summary' for row in rows))
            self.assertEqual(rows[-1]['record_type'],'capture_ended')
            self.assertEqual(rows[-1]['accepted'],3);self.assertEqual(rows[-1]['reason'],'stop_requested')
            self.assertFalse(any(row['record_type']=='pair_decision' for row in rows))

    def test_journal_is_bounded_and_reports_dropped_events_without_changing_acceptance(self):
        with tempfile.TemporaryDirectory() as folder:
            d=CaptureDiagnostics(folder,'bounded');entered=threading.Event();release=threading.Event()
            original=d._write
            def slow(value):entered.set();release.wait(3);original(value)
            with patch.object(d,'_write',side_effect=slow),patch('robot_workbench.capture_diagnostics.JOURNAL_LIMIT',1200):
                d.attempt(counters(0,0),counters(0,0),False,'reused_pair',host_receive_ns=100)
                self.assertTrue(entered.wait(1))
                for i in range(1,200):d.attempt(counters(i*2,i*2),counters(i*2,i*2),True,'accepted',host_receive_ns=100+i)
                self.assertGreater(d.journal_queue_dropped,0)
                release.set();d.close()
            raw=d.journal_path.read_bytes();rows=[json.loads(line) for line in raw.splitlines()]
            journal=d.journal()
            self.assertLessEqual(len(raw),1200);self.assertTrue(journal['full'])
            self.assertGreater(journal['file_omitted'],0)
            self.assertEqual(journal['committed_prefix_sha256'],hashlib.sha256(raw).hexdigest())
            self.assertEqual(journal['submitted_records'],journal['committed_records']+journal['queue_dropped']+journal['file_omitted'])
            self.assertEqual(len({row['record_sequence'] for row in rows}),len(rows))
            self.assertEqual(d.accepted,199);self.assertEqual(d.rejected,1)

    def test_rejected_partial_pair_explains_a_gap_without_changing_pair_policy(self):
        with tempfile.TemporaryDirectory() as folder:
            d=CaptureDiagnostics(folder,'instance');p=FrameProgress(0)
            for i,(rgb,depth) in enumerate(((10,20),(11,20),(12,22))):
                accepted=p.accept(rgb,depth,.1+i*.1)
                d.attempt(counters(rgb,depth),counters(rgb,depth),accepted,
                    'accepted' if accepted else 'reused_pair',host_receive_ns=100+i)
            event=d.take_event()
            self.assertEqual(event['event']['previous_published'],[10,20])
            self.assertEqual(event['recent_attempts'][1]['raw']['rgb_number'],11)
            self.assertFalse(event['recent_attempts'][1]['accepted'])
            self.assertEqual(d.summary()['rejected'],1)
            self.assertEqual(p.published,(12,22))
            d.active_stage=None;d.close()
            saved=json.loads((Path(folder)/'capture-diagnostics.json').read_text())
            self.assertEqual(saved['attribution'],'not_determined')
            self.assertFalse(saved['retention']['pixels'])

    def test_slow_diagnostic_storage_cannot_queue_frames_or_block_capture(self):
        with tempfile.TemporaryDirectory() as folder:
            d=CaptureDiagnostics(folder,'instance');entered=threading.Event();release=threading.Event()
            original=d._write;threads=[]
            def slow(value):
                threads.append(threading.get_ident());entered.set();release.wait(3);original(value)
            with patch.object(d,'_write',side_effect=slow):
                for i in range(80):
                    row=counters(i*2,i*2)
                    d.attempt(row,row,True,'accepted',host_receive_ns=100+i)
                self.assertTrue(entered.wait(1));self.assertEqual(d.attempts,80)
                self.assertLessEqual(len(d.recent),32);self.assertLessEqual(len(d.events),4)
                self.assertGreater(d.skipped_writes,0)
                self.assertNotIn(threading.get_ident(),threads)
                release.set();d.close()

    def test_stage_histograms_and_storage_errors_are_diagnostic_only(self):
        with tempfile.TemporaryDirectory() as folder:
            d=CaptureDiagnostics(folder,'instance')
            for value in (100,2_000_000,200_000_000):d.stage('wait',value)
            self.assertEqual(sum(d.timings['wait']['histogram']),3)
            self.assertEqual(d.timings['wait']['maximum_ns'],200_000_000)
            with patch.object(d,'_write',side_effect=OSError('fixture disk failure')):
                d.attempt(counters(1,2),counters(1,2),False,'fixture_failure',host_receive_ns=100)
                d.future.exception(timeout=2);d.close()
            self.assertIn('fixture disk failure',d.write_error)
            self.assertFalse(unsupported()['supported'])
            self.assertEqual(unsupported()['evidence_type'],'simulation_sample')

    def test_actual_ingress_distinguishes_raw_frameset_from_align_and_pair_rejection(self):
        import numpy as np
        from rule_based_workbench.real_rgbd import capture
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);directory=root/'channel';directory.mkdir()
            intr=SimpleNamespace(width=2,height=2,fx=10.,fy=10.,ppx=1.,ppy=1.,model='none',coeffs=[])
            class Frame:
                def __init__(self,number,stamp,depth=False):
                    self.number,self.stamp,self.depth=number,stamp,depth
                    self.profile=SimpleNamespace(as_video_stream_profile=lambda:SimpleNamespace(get_intrinsics=lambda:intr))
                def get_frame_number(self):return self.number
                def get_timestamp(self):return self.stamp
                def get_frame_timestamp_domain(self):return 'hardware_clock'
                def get_data(self):return np.zeros((2,2),np.uint16) if self.depth else np.zeros((2,2,2),np.uint8)
            class Pair:
                def __init__(self,rgb,depth,stamp):self.rgb,self.depth=Frame(rgb,stamp),Frame(depth,stamp,True)
                def get_color_frame(self):return self.rgb
                def get_depth_frame(self):return self.depth
            raw=[Pair(10,20,100.),Pair(11,21,133.),Pair(12,22,166.)]
            aligned=[raw[0],Pair(11,20,133.),raw[2]]
            stopped=threading.Event();published=[];calls=[0]
            def wait(_):
                calls[0]+=1
                return raw[max(0,calls[0]-21)]
            pipeline=SimpleNamespace(wait_for_frames=wait,stop=lambda:None,
                start=lambda _:SimpleNamespace(get_device=lambda:SimpleNamespace(first_depth_sensor=lambda:SimpleNamespace(get_depth_scale=lambda:.001))))
            config=SimpleNamespace(enable_device=lambda _:None,enable_stream=lambda *a:None)
            sdk=SimpleNamespace(pipeline=lambda:pipeline,config=lambda:config,
                stream=SimpleNamespace(color='color',depth='depth'),format=SimpleNamespace(yuyv='yuyv',z16='z16'),
                align=lambda _:SimpleNamespace(process=lambda _:aligned.pop(0)))
            def publish(camera,metadata,*a,**kw):
                published.append(metadata)
                if len(published)==2:stopped.set()
            service=SimpleNamespace(descriptor={'environment':'physical','intrinsics':{'base_0':{'serial':'123'}}},
                directories={'base_0':str(directory)},channels={'base_0':SimpleNamespace(token=lambda:None)},publish=publish)
            frames=SimpleNamespace(stop_requested=stopped,publish=lambda **kw:None)
            cv=SimpleNamespace(COLOR_YUV2RGB_YUY2=1,cvtColor=lambda raw,mode:np.zeros((2,2,3),np.uint8))
            args=SimpleNamespace(output=root,service_descriptor=root/'descriptor',serial='123',camera_name='head',duration_s=0)
            with patch.dict(sys.modules,{'pyrealsense2':sdk,'cv2':cv}),patch('robot_workbench.rgbd_service.CommonCameraService.connect',return_value=service):
                capture(args,frames)
            self.assertEqual([row['frame_number'] for row in published],[10,12])
            evidence=published[-1]['capture_gap_evidence'];rejected=evidence['recent_attempts'][1]
            self.assertEqual(rejected['raw']['depth_number'],21)
            self.assertEqual(rejected['aligned']['depth_number'],20)
            self.assertFalse(rejected['accepted'])
            snapshot=json.loads((root/'capture-diagnostics.json').read_text())
            self.assertEqual(snapshot['accepted'],2);self.assertEqual(snapshot['rejected'],1)
            self.assertEqual(snapshot['stage_timings']['publish']['count'],2)


if __name__=='__main__':unittest.main()
