"""Real CPU subprocess, immutable artifact and shared-camera lease regression."""
from copy import deepcopy
import hashlib
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'rule_based_workbench'),str(ROOT/'tests')]
import test_rgbd_service as fixtures
from test_recording_wall_clock import plan_fixture
from robot_workbench.recording_process import ProcessRecordingSession
from real_recording import verify_raw,iter_rgbd
from integration import _digest
from robot_workbench.sample_clock import clock_spec


def artifact(root,name,value):
    path=root/name;raw=json.dumps(value,allow_nan=False).encode();path.write_bytes(raw);path.chmod(0o400)
    return dict(path=str(path),size_bytes=len(raw),sha256=hashlib.sha256(raw).hexdigest())


class RecordingProcessTest(unittest.TestCase):
    def test_clean_child_keeps_selected_codecs_without_inheriting_python_or_cuda_paths(self):
        from robot_workbench.recording_process import _environment
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            for name in ('ffmpeg','ffprobe'):
                tool=root/name;tool.write_text('#!/bin/sh\necho selected-'+name+'\n');tool.chmod(0o700)
            with patch.dict(os.environ,dict(PATH=str(root)+os.pathsep+'/unrelated/runtime/bin',
                    PYTHONHOME='/bad/python',LD_LIBRARY_PATH='/bad/cuda')):
                env=_environment()
            self.assertNotIn('/unrelated/runtime/bin',env['PATH'])
            self.assertNotIn('PYTHONHOME',env);self.assertNotIn('LD_LIBRARY_PATH',env)
            self.assertEqual(env['CUDA_VISIBLE_DEVICES'],'')
            for name in ('ffmpeg','ffprobe'):
                self.assertEqual(subprocess.check_output([name],env=env,text=True).strip(),'selected-'+name)

    def session(self,root,service,owner,environment,kind='frozen',bad=None):
        source,plan,binding,timeline=plan_fixture(4,environment)
        if kind=='streaming':
            source=dict(schema_version='policy_execution_source.v1',environment=environment,
                robot_id=source['robot_id'],camera_serials=source['camera_serials'],calibration_profile=source['calibration_profile'],
                clock=clock_spec(environment,source_clock_id='cpu-source' if environment=='fake' else None),
                execution_instance_id='process-test',runtime_nonce=owner['runtime_nonce'],command_epoch=owner['command_epoch'],duration_s=.1)
            binding=dict(calibration_accepted=True,robot_id=source['robot_id'],camera_serials=source['camera_serials'],
                calibration_profile_digest=_digest(source['calibration_profile']))
            options=dict(session_kind='streaming',source=source,binding=binding)
        else:
            ref=dict(schema='collection_plan_ref.v1',environment=environment,
                origin=dict(owner_pid=owner['owner_pid'],runtime_nonce=owner['runtime_nonce'],epoch=owner['command_epoch']),
                **{key:artifact(root,key+'.json',value) for key,value in [('source',source),('plan',plan),('binding',binding)]})
            if bad=='owner':ref['origin']['epoch']+=1
            if bad=='hash':ref['plan']['sha256']='0'*64
            options=dict(plan_ref=ref)
        timeline_ref=dict(execution_timeline_ref=artifact(root,'timeline.json',timeline))
        session=ProcessRecordingSession(root/'raw',camera_channels=service.channels,execution_owner=owner,
            execution_instance_id='process-test',environment=environment,storage_format='zip',persistent_service=True,
            max_wall_time_seconds=30,preparation_deadline_monotonic=time.monotonic()+10,**options)
        return session,source,plan,timeline_ref

    def close(self,session):
        if not session.done.is_set():session.finish('stopped','test cleanup')
        self.assertTrue(session.done.wait(15),session.snapshot());session.thread.join(2)
        self.assertFalse(session.thread.is_alive());self.assertIsNotNone(session.process.poll())

    def test_frozen_and_policy_both_clocks_actual_process_and_exact_pixels(self):
        for kind in ('frozen','streaming'):
            for environment in ('fake','physical'):
                with self.subTest(kind=kind,environment=environment),tempfile.TemporaryDirectory() as tmp:
                    root=Path(tmp);fixture=fixtures.SharedCameraTest();self.addCleanup(fixture.doCleanups)
                    service=fixture.service(environment,camera_serials={'head':'h','left':'l','right':'r'})
                    with patch('robot_workbench.control_plan.load_artifact',side_effect=AssertionError('parent must not load large artifacts')):
                        session,source,plan,timeline=self.session(root,service,fixture.owner,environment,kind)
                        try:
                            self.assertTrue(session.prepared.wait(10),session.snapshot())
                            self.assertNotEqual(session.worker_pid,os.getpid())
                            self.assertTrue(all(ch.cached_token for ch in service.channels.values()))
                            epoch=time.time_ns();mono=time.monotonic();clock=source.get('clock') or clock_spec(environment)
                            def publish(index):
                                for camera,channel in service.channels.items():
                                    meta=fixture.metadata(camera,index);meta.update(clock_domain=clock['clock_domain'],source_clock_id=clock['source_clock_id'],host_receive_ns=epoch+index*1_000_000_000//30)
                                    service.publish(camera,meta,*fixture.pixels(index),width=2,height=2,sampled_token=channel.cached_token)
                            publish(0)
                            if kind=='frozen':
                                session.bind_execution(timeline,execution_instance_id='process-test',execution_owner=fixture.owner)
                                if environment=='physical':self.assertTrue(session.ready.wait(10),session.snapshot())
                            for index in range(5):
                                wall=epoch+index*1_000_000_000//30;source_ns=100_000_000_000+index*1_000_000_000//30
                                observed=dict(fixture.owner,sample_wall_ns=wall,clock_domain=clock['clock_domain'],source_clock_id=clock['source_clock_id'])
                                if environment=='fake':observed.update(source_time_ns=source_ns,physics_step=index*8)
                                now=source_ns/1e9 if environment=='fake' else mono+index/30
                                target=plan['ticks'][min(index,3)]['target']
                                session.append_feedback(tick=min(index,3),state=target,sample_monotonic_s=now,sample_wall_ns=wall,raw=observed)
                                if index<4:
                                    fields=dict(fixture.owner,clock_domain=clock['clock_domain'],source_clock_id=clock['source_clock_id'],feedback_at_send=observed)
                                    if kind=='frozen' and environment=='physical':fields={}
                                    if environment=='fake':fields.update(send_source_time_ns=source_ns,physics_step=None)
                                    session.append_command(tick=index,target=target,send_monotonic_s=now,send_wall_ns=wall,**fields)
                                    if index==0:
                                        native=dict(schema='workbench.target_window.v1',reference_id='a'*64,active_arm='both',
                                            controller_profile='normal_X5',host_times_s=[now+.03,now+.23],
                                            q_sdk14=[[0.]*14]*2,dq_sdk14=[[0.]*14]*2,torque_bias_sdk14=[[0.]*14]*2)
                                        session.submit('native_window',packet=native,admission=dict(firmware_ack=False),
                                            send_monotonic_s=now,send_wall_ns=wall)
                                if index:publish(index)
                            session.finish('completed');self.assertTrue(session.done.wait(15),session.snapshot())
                            self.assertIsNone(session.error,session.snapshot());self.assertIsNone(session.seal_error,session.snapshot())
                            manifest,_,_=verify_raw(session.root,environment=environment)
                            self.assertEqual(manifest['capture_status'],'complete',manifest)
                            self.assertEqual(manifest['native_window_count'],1)
                            native_rows=[json.loads(row)for row in(session.root/'native-windows.jsonl').read_text().splitlines()]
                            self.assertEqual(native_rows[0]['packet'],native)
                            self.assertEqual(manifest['native_windows_sha256'],manifest['files']['native-windows.jsonl']['sha256'])
                            for camera in service.channels:
                                rows=list(iter_rgbd(session.root,camera,environment=environment));self.assertEqual(len(rows),5)
                                from PIL import Image
                                for index,(_,_,color,depth) in enumerate(rows):
                                    with Image.open(io.BytesIO(color)) as picture:self.assertEqual(picture.tobytes(),fixture.pixels(index)[0])
                                    with Image.open(io.BytesIO(depth)) as picture:self.assertEqual(list(picture.getdata()),[0,1,32768,65535])
                            self.assertEqual(service.snapshot()['recording_budget']['queued_bytes'],0)
                            self.assertTrue(all(ch.cached_token is None for ch in service.channels.values()))
                        finally:self.close(session)

    def test_owner_epoch_and_hash_fail_closed_before_camera_lease(self):
        for bad in ('owner','hash'):
            with self.subTest(bad=bad),tempfile.TemporaryDirectory() as tmp:
                fixture=fixtures.SharedCameraTest();self.addCleanup(fixture.doCleanups);service=fixture.service()
                session,*_=self.session(Path(tmp),service,fixture.owner,'fake',bad=bad)
                try:
                    self.assertTrue(session.done.wait(10),session.snapshot());self.assertTrue(session.error)
                    self.assertFalse(session.prepared.is_set())
                    self.assertTrue(all(ch.current_lease() is None for ch in service.channels.values()))
                finally:self.close(session)

    def test_stop_before_binding_releases_only_own_leases_and_exits(self):
        with tempfile.TemporaryDirectory() as tmp:
            fixture=fixtures.SharedCameraTest();self.addCleanup(fixture.doCleanups);service=fixture.service('physical',camera_serials={'head':'h','left':'l','right':'r'})
            session,*_=self.session(Path(tmp),service,fixture.owner,'physical')
            self.assertTrue(session.prepared.wait(10),session.snapshot());session.finish('stopped','revoked')
            self.close(session)
            self.assertTrue(all(ch.current_lease() is None for ch in service.channels.values()))
            self.assertIsNone(service.process.poll(),'shared camera service is not a recording child')

    def test_child_crash_closes_leases_retains_prefix_and_never_claims_complete(self):
        with tempfile.TemporaryDirectory() as tmp:
            fixture=fixtures.SharedCameraTest();self.addCleanup(fixture.doCleanups)
            service=fixture.service('physical',camera_serials={'head':'h','left':'l','right':'r'})
            session,*_=self.session(Path(tmp),service,fixture.owner,'physical')
            self.assertTrue(session.prepared.wait(10),session.snapshot())
            deadline=time.monotonic()+10
            while not (session.root/'source.json').is_file() and not session.done.is_set() and time.monotonic()<deadline:time.sleep(.01)
            self.assertTrue((session.root/'source.json').is_file(),'crash fixture requires an actual persisted prefix')
            session.process.kill()
            self.close(session)
            self.assertTrue(session.error);self.assertFalse((session.manifest or {}).get('capture_status')=='complete')
            self.assertTrue(session.root.is_dir(),'partial recording survives worker loss')
            self.assertTrue(all(ch.current_lease() is None for ch in service.channels.values()))
            self.assertIsNone(service.process.poll())


if __name__=='__main__':unittest.main()
