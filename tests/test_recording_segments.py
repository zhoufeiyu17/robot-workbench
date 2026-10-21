"""Real lossless codecs through recorder/seal/recovery; explicit skip without codecs."""
from copy import deepcopy
import hashlib
import io
import json
from pathlib import Path
import shutil
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

import numpy as np
from PIL import Image

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'tests'),str(ROOT/'rule_based_workbench')]
import real_recording
import rgbd_segments
from real_rgbd import pair_metadata
from test_fake_device_pipeline import fake_metadata, FAKE_CLOCK_ID, FAKE_RUNTIME_NONCE
from test_recording_wall_clock import plan_fixture
import test_recording_wall_clock as clock_fixture
from robot_workbench.rgbd_channel import MemoryRgbdBudget,MemoryRgbdChannel


def pixels(blob):
    with Image.open(io.BytesIO(blob)) as image:
        return np.asarray(image).copy()


def file_hashes(root):
    return {str(path.relative_to(root)):hashlib.sha256(path.read_bytes()).hexdigest()
            for path in root.rglob('*') if path.is_file()}


@unittest.skipUnless(shutil.which('ffmpeg') and shutil.which('ffprobe'),
                     'actual segmented recorder/recovery requires ffmpeg and ffprobe; no codec mock qualification')
class SegmentedRecorderTest(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name);self.raw=self.root/'raw'
        source,plan,binding,timeline=plan_fixture(4,'fake')
        from execution_timeline import build
        limits={side+'_arm_joints':[[-3.,3.]]*6 for side in ('left','right')}
        limits.update(left_gripper=[0.,83.],right_gripper=[0.,83.])
        timeline=build(plan,current_state=source['initial_state']['measured_reported'],limits=limits,
            environment='fake',held_arms=source['initial_state']['held_arms'])
        self.source,self.plan=source,plan
        self.wall=[1_000_000_000_000]
        owner=dict(owner_pid=1234,command_epoch=3,runtime_nonce=FAKE_RUNTIME_NONCE)
        self.recorder=real_recording.RealRecorder(self.raw,source,plan,binding,
            environment='fake',execution_timeline=timeline,execution_owner=owner,
            storage_format='segments',clock_ns=lambda:self.wall[0])
        self.addCleanup(self.recorder.abort)
        writer=rgbd_segments.SegmentWriter
        small=patch.object(rgbd_segments,'SegmentWriter',side_effect=lambda *a,**kw:writer(*a,**dict(kw,segment_frames=3)))
        small.start();self.addCleanup(small.stop)
        self.expected={camera:[] for camera in real_recording.CAMERAS}

    def populate(self,cameras=None):
        for index in range(5):
            self.wall[0]=1_000_000_000_000+index*1_000_000
            stamp=100_000_000_000+index*1_000_000_000//30
            clock=dict(clock_domain='simulation',source_clock_id=FAKE_CLOCK_ID,
                       source_time_ns=stamp,physics_step=24_000+index*8,sample_wall_ns=self.wall[0])
            owner=dict(owner_pid=1234,command_epoch=3,runtime_nonce=FAKE_RUNTIME_NONCE)
            target=self.plan['ticks'][min(index,3)]['target']
            if index<4:
                self.recorder.record_command(index,target,stamp/1e9,self.wall[0],
                    send_source_time_ns=stamp,physics_step=None,feedback_at_send=clock,
                    clock_domain='simulation',source_clock_id=FAKE_CLOCK_ID,**owner)
            self.recorder.record_feedback(min(index,3),target,stamp/1e9,self.wall[0],raw=dict(clock,**owner))
            for camera_index,camera in enumerate(cameras or real_recording.CAMERAS):
                side=real_recording.SIDES[camera]
                meta=pair_metadata(serial=self.source['camera_serials'][side],camera_name=side,
                    color_number=index,depth_number=index,color_ms=stamp/1e6,depth_ms=stamp/1e6,
                    color_domain='simulation',depth_domain='simulation',host_receive_ns=self.wall[0],
                    depth_scale_m=.001,intrinsics=dict(width=16,height=12,fx=100.,fy=100.,cx=8.,cy=6.))
                fake_metadata(meta,stamp,step=24_000+index*8)
                meta['host_receive_ns']=self.wall[0]
                color=(np.arange(16*12*3).reshape(12,16,3)+index*19+camera_index*31).astype(np.uint8)
                depth=(np.arange(16*12).reshape(12,16)*293+index*97+camera_index).astype(np.uint16)
                depth[0,:4]=[0,1,32768,65535]
                payload=[]
                for array in (color,depth):
                    out=io.BytesIO();Image.fromarray(array).save(out,format='PNG');payload.append(out.getvalue())
                self.recorder.record_rgbd_frame(camera,meta,*payload)
                self.expected[camera].append((deepcopy(meta),color,depth))

    def assert_decoded(self,rows,camera,count):
        self.assertEqual(len(rows),count)
        for index,(row,meta,color,depth) in enumerate(rows):
            expected,rgb,z=self.expected[camera][index]
            self.assertEqual(meta,expected)
            self.assertEqual({k:v for k,v in row.items() if k not in ('segment','segment_frame')},expected)
            self.assertEqual(row['segment_frame'],index%3)
            np.testing.assert_array_equal(pixels(color),rgb)
            np.testing.assert_array_equal(pixels(depth),z)

    def test_seal_verify_read_matches_original_pixels_depth_metadata_and_feedback(self):
        self.populate()
        sealed=self.recorder.seal('completed')
        self.assertEqual(sealed['capture_status'],'complete',sealed['validation'])
        self.assertEqual(sealed['rgbd_storage'],'lossless_segments_v1')
        self.assertEqual(sealed['camera_frames'],dict.fromkeys(real_recording.CAMERAS,5))
        self.assertEqual(real_recording.verify_raw(self.raw,environment='fake')[0],sealed)
        for camera in real_recording.CAMERAS:
            self.assert_decoded(list(real_recording.iter_rgbd(self.raw,camera,environment='fake')),camera,5)
        pairs=json.loads((self.raw/'frame-feedback-pairs.json').read_text())['pairs']
        self.assertEqual(len(pairs),15)
        self.assertTrue(all(row['association']=='exact_feedback_sample' for row in pairs))
        self.assertFalse(list(self.raw.rglob('*.zip')))
        self.assertFalse(list(self.raw.rglob('*.png')))
        self.assertFalse([path for path in self.raw.rglob('*') if '.partial.' in path.name])

    def test_abort_recovers_only_committed_prefix_without_modifying_raw(self):
        self.populate()
        for writer in self.recorder.segment_writers.values():writer.wait_committed()
        self.recorder.abort()
        before=file_hashes(self.raw)
        self.assertTrue(any('.partial.' in name for name in before))
        report_path=self.root/'recovery.json'
        report=real_recording.recover_raw(self.raw,report_path,environment='fake')
        self.assertFalse(report['episode_complete']);self.assertFalse(report['motion_resumed'])
        self.assertEqual(report['status'],'incomplete')
        for camera in real_recording.CAMERAS:
            self.assertEqual(report['cameras'][camera]['verified_frame_count'],3)
            self.assert_decoded(list(real_recording.iter_recovered_rgbd(report_path,camera)),camera,3)
        self.assertEqual(file_hashes(self.raw),before)
        self.assertFalse((self.raw/'manifest.json').exists())

    def test_missing_camera_is_incomplete_and_does_not_break_prefix_recovery(self):
        self.populate(real_recording.CAMERAS[:2])
        for writer in self.recorder.segment_writers.values():writer.wait_committed()
        self.recorder.abort()
        before=file_hashes(self.raw)
        report_path=self.root/'missing-camera.json'
        report=real_recording.recover_raw(self.raw,report_path,environment='fake')
        absent=real_recording.CAMERAS[-1]
        self.assertEqual(report['cameras'][absent]['verified_frame_count'],0)
        self.assertEqual(report['cameras'][absent]['status'],'incomplete')
        self.assertFalse(report['episode_complete'])
        self.assertEqual(list(real_recording.iter_recovered_rgbd(report_path,absent)),[])
        for camera in real_recording.CAMERAS[:2]:
            self.assert_decoded(list(real_recording.iter_recovered_rgbd(report_path,camera)),camera,3)
        self.assertEqual(file_hashes(self.raw),before)


@unittest.skipUnless(shutil.which('ffmpeg') and shutil.which('ffprobe'), 'actual CPU codecs required')
class MemorySegmentStartupTest(unittest.TestCase):
    def test_slow_first_codec_preserves_prebind_backlog_and_exact_rgbd(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);h=clock_fixture.RecordingWallClockTest()
            h.wall=[time.time_ns()];h.source,h.plan,binding,timeline=plan_fixture(4,'fake')
            owner=dict(owner_pid=1234,command_epoch=3,runtime_nonce=FAKE_RUNTIME_NONCE)
            budget=MemoryRgbdBudget();current=[]
            channels={camera:MemoryRgbdChannel(budget=budget,clock_ns=lambda:h.wall[0],
                lease_guard=lambda lease:bool(current) and current[0].lease_active(lease)) for camera in real_recording.CAMERAS}
            entered,release=threading.Event(),threading.Event()
            write=rgbd_segments._Child.write
            def slow_first(child,payload):
                if not entered.is_set():
                    entered.set()
                    if not release.wait(10):raise TimeoutError('test did not release first codec')
                return write(child,payload)
            expected={camera:[] for camera in channels}
            def publish(index):
                h.wall[0]+=1_000_000
                stamp=100_000_000_000+index*1_000_000_000//30
                for camera,channel in channels.items():
                    side=real_recording.SIDES[camera]
                    meta=pair_metadata(serial=h.source['camera_serials'][side],camera_name=side,
                        color_number=index+33,depth_number=index+33,color_ms=stamp/1e6,depth_ms=stamp/1e6,
                        color_domain='simulation',depth_domain='simulation',host_receive_ns=h.wall[0],
                        depth_scale_m=.001,intrinsics=dict(width=16,height=12,fx=100.,fy=100.,cx=8.,cy=6.))
                    fake_metadata(meta,stamp,step=24_000+index*8);meta['host_receive_ns']=h.wall[0]
                    rgb=(np.arange(16*12*3).reshape(12,16,3)+(index+33)*7).astype(np.uint8)
                    depth=(np.arange(16*12).reshape(12,16)*293+(index+33)*31).astype(np.uint16)
                    depth[0,:4]=[0,1,32768,65535]
                    payload=[]
                    for array in (rgb,depth):
                        stream=io.BytesIO();Image.fromarray(array).save(stream,format='PNG');payload.append(stream.getvalue())
                    self.assertTrue(channel.write(meta,*payload,sampled_token=channel.token()))
                    expected[camera].append((deepcopy(meta),rgb,depth))
            with patch.object(rgbd_segments._Child,'write',slow_first):
                session=real_recording.RecordingSession(root/'raw',h.source,h.plan,binding,
                    environment='fake',execution_owner=owner,camera_channels=channels,storage_format='segments',
                    clock_ns=lambda:h.wall[0],defer_execution=True,execution_instance_id='pre-roll-codec-test',
                    preparation_active=lambda:True,preparation_deadline_monotonic=time.monotonic()+5,
                    persistent_service=True,max_wall_time_seconds=60)
                h.session=session;current.append(session)
                try:
                    self.assertTrue(session.prepared.wait(3))
                    publish(-33)
                    self.assertTrue(entered.wait(3))
                    for index in range(-32,0):publish(index)
                    self.assertTrue(all(ch.snapshot()['queued_frames']==33 for ch in channels.values()))
                    self.assertFalse(session.execution_bound.is_set());self.assertFalse(session.ready.is_set())
                    release.set()
                    h.wait_for(lambda:session.recorder is not None and all(len(rows)==33 for rows in session.recorder.frames.values()),session,seconds=10)
                    self.assertEqual(budget.size,0)
                    self.assertFalse((session.root/'execution-timeline.json').exists())
                    session.bind_execution(timeline,execution_instance_id='pre-roll-codec-test',execution_owner=owner)
                    for index in range(5):
                        publish(index);h.submit_feedback(index)
                        if index<4:h.submit_command(index)
                    session.finish('completed')
                    self.assertTrue(session.done.wait(15),session.snapshot())
                    self.assertIsNone(session.error,session.snapshot())
                    self.assertEqual(session.manifest['capture_status'],'complete',session.manifest)
                    for camera in channels:
                        rows=list(real_recording.iter_rgbd(session.root,camera,environment='fake'))
                        self.assertEqual(len(rows),38)
                        for (_,meta,rgb,depth),(wanted,color,z) in zip(rows,expected[camera]):
                            self.assertEqual(meta,wanted)
                            np.testing.assert_array_equal(pixels(rgb),color)
                            np.testing.assert_array_equal(pixels(depth),z)
                    self.assertEqual(budget.size,0)
                    timings=json.loads((session.root/'recording-timings.json').read_text())
                    self.assertEqual(timings['segments']['base_0']['first_frame']['status'],'written')
                finally:
                    release.set()
                    if not session.done.is_set():session.finish('stopped','CPU test cleanup')
                    self.assertTrue(session.done.wait(15))


if __name__=='__main__':unittest.main()
