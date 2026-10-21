"""Policy event/IPC recording regression; codec pixel qualification has its own real-codec gate."""
from copy import deepcopy
import json
from pathlib import Path
import shutil
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'rule_based_workbench'),str(ROOT/'tests')]
import real_recording as recording
import test_rgbd_service as frames_fixture
from test_recording_wall_clock import plan_fixture
from integration import _digest
from robot_workbench.sample_clock import clock_spec


class StreamingRecordingTest(unittest.TestCase):
    def test_physical_liveness_refreshes_head_after_io_and_rechecks_terminal(self):
        for stop_during_write in (False,True):
            with self.subTest(stop_during_write=stop_during_write),tempfile.TemporaryDirectory() as temporary:
                source,plan,binding,timeline=plan_fixture(4,'physical')
                h=frames_fixture.SharedCameraTest();self.addCleanup(h.doCleanups)
                service=h.service('physical',camera_serials=source['camera_serials'])
                clock=[time.time_ns()];epoch=clock[0];entered=threading.Event();release=threading.Event();next_round=threading.Event()
                class BlockedWriter(recording.RealRecorder):
                    def record_rgbd_pixels(self,camera,*args,**kwargs):
                        if camera=='base_0' and not entered.is_set():
                            entered.set()
                            if not release.wait(5):raise TimeoutError('CPU fixture writer was not released')
                        return super().record_rgbd_pixels(camera,*args,**kwargs)
                class ObservedSession(recording.RecordingSession):
                    def _drain_events(self):
                        result=super()._drain_events()
                        if self.recorder is not None and all(len(rows)>=2 for rows in self.recorder.frames.values()):next_round.set()
                        return result
                session=ObservedSession(Path(temporary)/'raw',source,plan,binding,camera_channels=service.channels,
                    storage_format='zip',clock_ns=lambda:clock[0],recorder_factory=BlockedWriter,
                    environment='physical',execution_owner=h.owner,defer_execution=True,execution_instance_id='blocked-io',
                    persistent_service=True,max_wall_time_seconds=30,preparation_active=lambda:True,
                    preparation_deadline_monotonic=time.monotonic()+5)
                try:
                    self.assertTrue(session.prepared.wait(5),session.snapshot())
                    def publish(index,stamp):
                        for camera,channel in service.channels.items():
                            meta=h.metadata(camera,index);meta['host_receive_ns']=stamp
                            service.publish(camera,meta,*h.pixels(index),width=2,height=2,sampled_token=channel.token())
                    publish(0,epoch)
                    session.bind_execution(timeline,execution_instance_id='blocked-io',execution_owner=h.owner)
                    self.assertTrue(entered.wait(5),session.snapshot())
                    clock[0]=epoch+600_000_000
                    if stop_during_write:session.finish('stopped','CPU terminal arrived during write')
                    else:publish(1,epoch+400_000_000)
                    release.set()
                    if not stop_during_write:
                        self.assertTrue(next_round.wait(5),session.snapshot())
                        self.assertIsNone(session.error,session.snapshot())
                        session.finish('stopped','CPU fresh producer proved')
                    self.assertTrue(session.done.wait(10),session.snapshot());self.assertIsNone(session.error,session.snapshot())
                    rows=[json.loads(line) for line in (session.root/'base_0-pairs.jsonl').read_text().splitlines()]
                    self.assertEqual(int(rows[0]['host_receive_ns']),epoch)
                    if not stop_during_write:self.assertEqual(int(rows[1]['host_receive_ns']),epoch+400_000_000)
                finally:
                    release.set()
                    if not session.done.is_set():session.finish('stopped','CPU test cleanup')
                    self.assertTrue(session.done.wait(10),session.snapshot())

    def test_physical_frozen_session_uses_same_deferred_ipc_channels(self):
        source,plan,binding,timeline=plan_fixture(4,'physical')
        from physical_motiongen import validate_evidence,freeze_physical
        from test_physical_motiongen import bind_contact_fixture
        from execution_timeline import build
        model=validate_evidence(plan,source)
        source['initial_state'].update(disabled_grippers=['right'],
            disabled_gripper_model='profile_max_opening_mm_assumption_not_measured')
        for sample in source['initial_state']['owner_samples']:sample['disabled_grippers']=['right']
        binding.update(initial_state=deepcopy(source['initial_state']),source_digest=_digest(source))
        model['planning_validation']['source_digest']=_digest(source);bind_contact_fixture(model,source)
        plan=freeze_physical(model,source,binding)
        limits={side+'_arm_joints':[[-3.,3.]]*6 for side in ('left','right')}
        limits.update(left_gripper=[0.,83.],right_gripper=[0.,83.])
        timeline=build(plan,current_state=source['initial_state']['measured_reported'],limits=limits,
            environment='physical',disabled_grippers=['right'],held_arms=source['initial_state']['held_arms'])
        h=frames_fixture.SharedCameraTest();self.addCleanup(h.doCleanups)
        service=h.service('physical',camera_serials=source['camera_serials'])
        with tempfile.TemporaryDirectory() as temporary:
            session=recording.RecordingSession(Path(temporary)/'raw',source,plan,binding,
                camera_channels=service.channels,storage_format='segments' if shutil.which('ffmpeg') and shutil.which('ffprobe') else 'zip',
                environment='physical',execution_owner=h.owner,defer_execution=True,execution_instance_id='physical-frozen',
                persistent_service=True,max_wall_time_seconds=7200,
                preparation_active=lambda:True,preparation_deadline_monotonic=time.monotonic()+2)
            try:
                self.assertTrue(session.prepared.wait(3))
                epoch=time.time_ns();mono=time.monotonic();metadata=h.metadata
                def images(index):
                    for camera in service.channels:
                        value=metadata(camera,index);value['host_receive_ns']=epoch+index*1_000_000_000//30
                        service.publish(camera,value,*h.pixels(index),width=2,height=2,sampled_token=service.channels[camera].token())
                images(0)
                session.bind_execution(timeline,execution_instance_id='physical-frozen',execution_owner=h.owner)
                self.assertTrue(session.ready.wait(3),session.snapshot())
                self.assertEqual(timeline['initialization']['held_arms'],['right'])
                self.assertEqual(timeline['initialization']['disabled_grippers'],['right'])
                for index in range(5):
                    target=plan['ticks'][min(index,3)]['target'];wall=epoch+index*1_000_000_000//30
                    if index<4:session.submit('command',tick=index,target=target,send_monotonic_s=mono+index/30,send_wall_ns=wall)
                    session.submit('feedback',tick=min(index,3),state=target,sample_monotonic_s=mono+index/30,sample_wall_ns=wall,raw={})
                    if index:images(index)
                session.finish('completed');self.assertTrue(session.done.wait(10),session.snapshot())
                self.assertIsNone(session.error,session.snapshot());self.assertIsNone(session.seal_error)
                self.assertEqual(session.manifest['capture_status'],'complete',session.manifest)
                recording.verify_raw(session.root,environment='physical')
            finally:
                if not session.done.is_set():session.finish('stopped','CPU test cleanup')
                self.assertTrue(session.done.wait(10))

    def test_physical_and_fake_actual_events_and_pre_roll_share_session(self):
        for environment in ('fake','physical'):
            with self.subTest(environment=environment),tempfile.TemporaryDirectory() as temporary:
                h=frames_fixture.SharedCameraTest();self.addCleanup(h.doCleanups);service=h.service(environment)
                old,plan,binding,_=plan_fixture(4,environment)
                cameras={role:h.k[name]['serial'] for role,name in frames_fixture.CAMERAS.items()}
                source=dict(schema_version='policy_execution_source.v1',environment=environment,
                    robot_id=old['robot_id'],camera_serials=cameras,calibration_profile=old['calibration_profile'],
                    clock=clock_spec(environment,source_clock_id='cpu-source' if environment=='fake' else None),
                    execution_instance_id='stream-test-'+environment,runtime_nonce=h.owner['runtime_nonce'],
                    command_epoch=h.owner['command_epoch'],duration_s=.1)
                binding=dict(calibration_accepted=True,robot_id=source['robot_id'],camera_serials=cameras,
                             calibration_profile_digest=_digest(source['calibration_profile']))
                original=recording.StreamingRecorder
                class EventFixtureRecorder(original):
                    def __init__(self,*args,**kwargs):
                        super().__init__(*args,**kwargs)
                        if not shutil.which('ffmpeg') or not shutil.which('ffprobe'):
                            # Exercise actual event+IPC/storage on hosts lacking codecs;
                            # production remains segments, separately tested with real ffmpeg.
                            self.segment_executor.shutdown();self.segment_executor=None;self.storage_format='zip'
                with patch.object(recording,'StreamingRecorder',EventFixtureRecorder):
                    session=recording.StreamingRecordingSession(Path(temporary)/'raw',source,binding,
                        camera_channels=service.channels,execution_owner=h.owner,max_wall_time_seconds=30)
                    try:
                        self.assertTrue(session.prepared.wait(3),session.snapshot())
                        with patch.object(service,'call',side_effect=AssertionError('Owner snapshot must not perform IPC')):
                            session.snapshot()
                        epoch=time.time_ns();mono=time.monotonic();metadata=h.metadata
                        def stamped(camera,index):
                            value=metadata(camera,index);value['host_receive_ns']=epoch+index*1_000_000_000//30
                            return value
                        h.metadata=stamped
                        def images(index):
                            for camera in service.channels:h.publish(service,camera,index,service.channels[camera].token())
                        images(0)
                        for index in range(5):
                            self.assertIsNone(session.error,session.snapshot())
                            wall=epoch+index*1_000_000_000//30;source_ns=100_000_000_000+index*1_000_000_000//30
                            observed=dict(sample_wall_ns=wall,**h.owner,clock_domain=source['clock']['clock_domain'],
                                source_clock_id=source['clock']['source_clock_id'])
                            if environment=='fake':observed.update(source_time_ns=source_ns,physics_step=index*8)
                            now=source_ns/1e9 if environment=='fake' else mono+index/30
                            target=plan['ticks'][min(index,3)]['target']
                            session.append_feedback(tick=min(index,3),state=target,sample_monotonic_s=now,
                                sample_wall_ns=wall,raw=observed)
                            if index<4:
                                fields=dict(h.owner,clock_domain=source['clock']['clock_domain'],
                                    source_clock_id=source['clock']['source_clock_id'],feedback_at_send=observed)
                                if environment=='fake':fields.update(send_source_time_ns=source_ns,physics_step=None)
                                session.append_command(tick=index,target=target,send_monotonic_s=now,send_wall_ns=wall,**fields)
                            if index:images(index)
                        session.finish('completed')
                        self.assertTrue(session.done.wait(10),session.snapshot())
                        self.assertIsNone(session.error,session.snapshot())
                        self.assertIsNone(session.seal_error,session.snapshot())
                        self.assertEqual(session.manifest['capture_status'],'complete',session.manifest)
                        self.assertEqual(session.manifest['recording_kind'],'streaming_policy')
                        self.assertFalse((session.root/'plan.json').exists())
                        manifest,actual,absent=recording.verify_raw(session.root,environment=environment)
                        self.assertIsNone(absent);self.assertEqual(actual,source)
                        self.assertEqual(manifest['command_count'],4);self.assertEqual(manifest['feedback_count'],5)
                        self.assertEqual(manifest['actual_commands_sha256'],manifest['files']['commands.jsonl']['sha256'])
                        for camera in service.channels:
                            self.assertEqual(len(list(recording.iter_rgbd(session.root,camera,environment=environment))),5)
                        if session.recorder.storage_format=='segments':
                            playback=recording.export_playback(session.root,None,Path(temporary)/'playback',environment=environment)
                            self.assertEqual(playback['recording_kind'],'streaming_policy')
                            self.assertNotIn('plan_run_id',playback)
                            self.assertEqual(playback['actual_commands_sha256'],manifest['actual_commands_sha256'])
                            for timing in playback['source_timing'].values():
                                self.assertAlmostEqual(timing['encoded_duration_s'],.1,places=6)
                        self.assertEqual(service.snapshot()['recording_budget']['queued_bytes'],0)
                    finally:
                        if not session.done.is_set():session.finish('stopped','CPU test cleanup')
                        self.assertTrue(session.done.wait(10))


if __name__=='__main__':unittest.main()
