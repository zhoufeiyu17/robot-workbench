"""Real Owner/recorder/lease wiring on CPU plant fixtures, never GPU qualification."""
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import secrets
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'tests'),str(ROOT/'rule_based_workbench'),str(ROOT/'calibration'),str(ROOT/'calibration/infra')]
from test_collection_physx_protocol import protocol_fixture,LaggingPlant,ref,FakeSafety
from arx_env.joint_head_jaw_mapping import configure_mapping
from real_control import classes
from real_recording import RecordingSession,RealRecorder,CAMERAS,verify_raw
from robot_workbench.physx import PhysxBackend,SimulationClock
from robot_workbench.physx_camera import CameraProducer,camera_intrinsics
from robot_workbench.runtime import Runtime


class ImagePlant(LaggingPlant):
    def cameras(self,*,render):
        assert render is False
        return {name:dict(rgb=np.arange(36,dtype=np.uint8).reshape(3,4,3),
                         depth=np.full((3,4),.5,dtype=np.float32)) for name in CAMERAS}


class RecordingPreparationTest(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name);self.sessions=[];self.releases=[];self.events=[];self.runtime=None
        _,source,_,_,mapping=protocol_fixture(self.root)
        collection,_,_=classes(ref,jaw_mapping_file=mapping)
        self.addCleanup(configure_mapping,None)
        initial=deepcopy(source['initial_state']['planned_reported'])
        for side in ('left','right'):
            initial[side+'_arm_joints']=[q+.04 for q in initial[side+'_arm_joints']]
            initial[side+'_gripper']-=.008
        self.plant=ImagePlant(initial);self.clock=SimulationClock()
        self.backend=PhysxBackend(self.plant,joint_offsets=[.04]*12,metal_gap_offset_mm=.008)
        _,self.source,self.plan,self.binding,_=protocol_fixture(self.root,
            source_clock_id=self.backend.source_clock_id,runtime_nonce='preparation-cpu-runtime')
        self.factory_hook=None;self.timeline_writes=[]
        test=self
        class ObservedRecorder(RealRecorder):
            def write_json(self,name,value):
                if name=='execution-timeline.json':test.timeline_writes.append((str(self.root),deepcopy(value)))
                return super().write_json(name,value)
        def factory(payload,on_fault):
            if self.factory_hook:self.factory_hook(payload)
            # This legacy recorder fixture consumes the ref only in its dedicated
            # preparation thread; production uses ProcessRecordingSession.
            if 'plan_ref' in payload:
                from robot_workbench.control_plan import load_artifact
                source=load_artifact(payload['plan_ref']['source']);plan=load_artifact(payload['plan_ref']['plan'])
            else:source,plan=payload['source'],payload['plan']
            session=RecordingSession(self.root/secrets.token_hex(8)/'raw',source,plan,self.binding,
                camera_directories=self.producer.directories,execution_timeline=payload.get('execution_timeline'),
                environment='fake',execution_owner=payload['execution_owner'],on_fault=on_fault,
                resident_deadline_wall_ns=time.time_ns()+120_000_000_000,recorder_factory=ObservedRecorder,
                **payload['recording_preparation'])
            self.sessions.append(session)
            return session
        factory.supports_deferred_preparation=True
        self.factory=factory
        self.owner=collection(self.backend,FakeSafety,hardware=False,environment='fake',
            hardware_binding=self.root/'binding.json',control_clock=self.clock,runtime_nonce='preparation-cpu-runtime',
            recording_factory=factory,event=lambda kind,**fields:self.events.append(dict(kind=kind,wall_ns=time.time_ns(),**fields)))
        self.backend.owner=self.owner
        rig={'device_calibrations':{name:dict(serial=self.source['camera_serials'][side],width=4,height=3,
            intrinsics=dict(fx=100.,fy=100.,cx=2.,cy=1.),distortion=dict(coefficients=[0.]*5))
            for name,side in zip(CAMERAS,('head','left','right'))}}
        self.producer=CameraProducer(camera_intrinsics(rig),self.root/'spool',min_free_bytes=0,
            memory_recording=False,
            recording_guard=lambda lease:self.owner.recording is not None and self.owner.recording.lease_active(lease))
        self.sample_images=True
        self.addCleanup(self.cleanup)

    def cleanup(self):
        for release in self.releases:release.set()
        try:self.owner.submit(dict(action='stop'));self.owner.tick()
        except Exception:pass
        runner=self.owner.runner
        if runner and runner.preparation_thread:runner.preparation_thread.join(3)
        for session in self.sessions:
            if not session.done.is_set():session.finish('stopped','CPU fixture cleanup')
            session.done.wait(3)
        self.producer.close()
        if self.runtime and self.runtime.fd is not None:
            self.owner.submit(dict(action='release',confirmation='ARMS_SUPPORTED_RELEASE'))
            self.runtime.tick();self.runtime.close()

    def step(self):
        self.clock.advance(self.runtime or self.owner,self.backend)
        if self.sample_images and self.clock.index%40==0:
            self.producer.collect(self.plant,self.backend,dict(owner_available=self.owner.feedback_healthy,
                owner_pid=os.getpid(),command_epoch=self.owner.epoch,runtime_nonce=self.owner.runtime_nonce))
        if self.clock.index%12==0:time.sleep(.002)

    def until(self,predicate,seconds=5):
        deadline=time.monotonic()+seconds
        while not predicate():
            if time.monotonic()>=deadline:self.fail('CPU preparation fixture timed out: '+str(self.owner.snapshot()))
            self.step()

    def start(self):
        self.until(lambda:self.owner.snapshot().get('stationary') is True)
        submit=(lambda command:self.runtime.submit(self.credentials['collection'],command)) if self.runtime else self.owner.submit
        preview=submit(dict(action='preview_frozen',source=self.source,plan=self.plan))
        if self.runtime is None:self.owner.execution_instance_id=secrets.token_hex(16)
        prior=self.owner.runner
        submit(dict(action='confirm',token=preview['token'],confirmation='START_REVIEWED_SINGLE_ARM',physical_clearance=True))
        self.until(lambda:self.owner.runner is not prior)
        return self.owner.runner

    def test_valid_cup_return_executes_without_starting_a_recording_factory(self):
        from test_cup_return_replay import fixture as replay_fixture
        from cup_return_replay import build_return_plan
        from integration import _digest
        original,parent,receipt,source,binding,_=replay_fixture(
            jaw_mapping_sha256=hashlib.sha256((self.root/'jaw.json').read_bytes()).hexdigest())
        source['clock']['source_clock_id']=self.backend.source_clock_id
        source['initial_state']['clock']=deepcopy(source['clock'])
        source['replay_context']['clock']=deepcopy(source['clock'])
        for sample in source['initial_state']['owner_samples']:sample['source_clock_id']=self.backend.source_clock_id
        self.source=source;self.plan=build_return_plan(parent,original,receipt,source)
        binding.update(source_digest=_digest(source),clock=source['clock'],initial_state=source['initial_state'])
        (self.root/'binding.json').write_text(json.dumps(binding))
        self.factory_hook=lambda _:self.fail('maintenance replay must not start a recorder')
        runner=self.start();self.until(lambda:runner.status=='READY')
        self.assertTrue(runner.maintenance_replay)
        self.assertIsNone(runner.preparation_thread)
        self.assertIsNone(self.owner.recording)
        self.assertEqual(self.sessions,[])
        self.assertGreater(runner.last_sent_index,0)
        self.assertEqual(self.owner.snapshot()['frozen']['validation_scope'],'exact_parent_reverse_without_new_collision_check')
        self.assertFalse(any(row['kind']=='recording_preparation_started' for row in self.events))
        self.assertFalse(list((self.root/'spool').rglob('recording-request.json')))

    def test_filesystem_consumer_rejects_memory_only_camera_registration(self):
        for directory in self.producer.directories.values():
            (Path(directory)/'recording-service.json').write_text(json.dumps(
                dict(schema='rgbd_lease.v1',transport='bounded_memory',same_process_only=True)))
        self.start()
        self.until(lambda:self.owner.recording is not None and self.owner.recording.done.is_set())
        self.assertIn('on-demand recording service required',self.owner.recording.error)
        self.assertFalse(list((self.root/'spool').rglob('recording-request.json')))

    def test_actual_sampled_first_frames_precede_command_and_final_timeline_is_bound_once(self):
        runner=self.start()
        self.until(lambda:self.owner.recording is not None and self.owner.recording.prepared.is_set())
        session=self.owner.recording
        if session.camera_channels is None:self.assertIsNone(session.recorder)
        self.assertFalse((session.root/'execution-timeline.json').exists())
        for instance,identity in ((secrets.token_hex(16),runner.recording_owner),
                (runner.execution_instance_id,dict(runner.recording_owner,runtime_nonce='wrong-owner'))):
            with self.assertRaises(ValueError):session.bind_execution({'not':'a final timeline'},
                execution_instance_id=instance,execution_owner=identity)
            self.assertFalse(session.execution_bound.is_set())
        self.until(lambda:runner.status=='READY')
        self.until(lambda:self.owner.recording is not None and self.owner.recording.done.is_set())
        session=self.owner.recording
        self.assertIsNone(session.error,session.snapshot())
        self.assertEqual(session.manifest['capture_status'],'complete',session.manifest)
        self.assertTrue(session.prepared.is_set());self.assertTrue(session.execution_bound.is_set())
        command=session.recorder.commands[0]
        prepared=next(row for row in self.events if row['kind']=='recording_preparation_finished')
        self.assertGreaterEqual(command['send_source_time_ns']-round(prepared['source_time_s']*1e9),300_000_000)
        for camera,rows in session.recorder.frames.items():
            self.assertLessEqual(rows[0]['source_time_ns'],command['send_source_time_ns'])
            self.assertGreaterEqual(int(rows[0]['host_receive_ns']),session.preparation_started_wall_ns)
            self.assertEqual(rows[0]['command_epoch'],session.execution_owner['command_epoch'])
            self.assertEqual(rows[0]['runtime_nonce'],self.owner.runtime_nonce)
        self.assertEqual(len(self.timeline_writes),1)
        self.assertEqual(self.timeline_writes[0][1],runner.runtime)
        with self.assertRaises(ValueError):session.bind_execution(runner.runtime,
            execution_instance_id=runner.execution_instance_id,execution_owner=runner.recording_owner)
        verify_raw(session.root,environment='fake')

    def test_recording_waits_for_fresh_feedback_past_actual_reference_endpoint(self):
        # Host command submission may be later than the sampled SDK feedback.
        self.owner.control_time=lambda:self.clock()+.008
        runner=self.start()
        self.until(lambda:runner.status=='READY')
        self.assertTrue(runner.recording_waits_for_endpoint)
        self.until(lambda:self.owner.recording.done.is_set())
        session=self.owner.recording
        self.assertIsNone(session.error,session.snapshot())
        self.assertEqual(session.manifest['capture_status'],'complete',session.manifest)
        command=session.recorder.commands[0];feedback=session.recorder.feedback[-1]
        self.assertGreaterEqual(feedback['sample_monotonic_s'],command['send_monotonic_s']+runner.frozen.duration_s)
        verify_raw(session.root,environment='fake')

    def test_preparation_does_not_relabel_an_old_queued_camera_packet(self):
        self.until(lambda:self.owner.snapshot().get('stationary') is True)
        self.producer.queue.join()
        entered,release=threading.Event(),threading.Event();self.releases.append(release)
        original=self.producer._publish;old=[]
        def delayed(packet):
            if not entered.is_set():
                old.append(deepcopy({key:packet[key] for key in ('wall_ns','feedback')}));entered.set()
                if not release.wait(3):raise TimeoutError('old camera packet was not released')
            original(packet)
        self.producer._publish=delayed
        self.until(entered.is_set)
        runner=self.start()
        self.until(lambda:self.owner.recording is not None and self.owner.recording.prepared.is_set())
        session=self.owner.recording
        self.assertLess(old[0]['wall_ns'],session.preparation_started_wall_ns)
        self.assertFalse(session.ready.is_set())
        release.set();self.producer.queue.join()
        self.until(lambda:runner.status=='READY')
        self.until(session.done.is_set)
        self.assertIsNone(session.error,session.snapshot())
        self.assertEqual(session.manifest['capture_status'],'complete',session.manifest)
        for rows in session.recorder.frames.values():
            self.assertNotIn(old[0]['feedback']['source_time_ns'],[row['source_time_ns'] for row in rows])
            self.assertTrue(all(int(row['host_receive_ns'])>=session.preparation_started_wall_ns for row in rows))

    def test_factory_failure_and_absent_images_do_not_gate_motion(self):
        self.factory_hook=lambda _:(_ for _ in ()).throw(OSError('preparation storage failed'))
        runner=self.start();self.until(lambda:runner.status=='READY')
        self.assertIn('preparation storage failed',self.owner.recording_error)
        self.assertGreater(runner.last_sent_index,0)
        self.factory_hook=None;self.sample_images=False
        runner=self.start();self.until(lambda:runner.status=='READY')
        self.assertTrue(self.owner.recording.prepared.is_set())
        self.assertFalse(self.owner.recording.ready.is_set(),'image readiness must never gate motion')
        self.assertGreater(runner.last_sent_index,0)

    def test_one_second_timeout_then_same_epoch_new_execution_rejects_late_factory(self):
        entered,release=threading.Event(),threading.Event();self.releases.append(release)
        calls=[]
        def delayed(payload):
            calls.append(payload)
            if len(calls)==1:
                entered.set()
                if not release.wait(5):raise TimeoutError('CPU fixture did not release old factory')
        self.factory_hook=delayed
        old=self.start();self.until(entered.is_set)
        old_epoch=self.owner.epoch
        self.until(lambda:old.status=='READY',seconds=4)
        self.assertTrue(old.preparation_closed)
        self.assertIn('exceeded 1 wall second',self.owner.recording_error)
        first=next(row for row in self.events if row['kind']=='command')
        preparation=next(row for row in self.events if row['kind']=='recording_preparation_started')
        self.assertGreaterEqual(first['wall_ns']-preparation['wall_ns'],1_000_000_000)
        self.assertIsNone(self.owner.recording)
        new=self.start();self.until(lambda:self.owner.recording is not None and self.owner.recording.prepared.is_set())
        current=self.owner.recording
        self.assertEqual(self.owner.epoch,old_epoch)
        self.assertNotEqual(new.execution_instance_id,old.execution_instance_id)
        release.set();old.preparation_thread.join(3)
        self.until(lambda:len(self.sessions)==2 and all(s is current or s.done.is_set() for s in self.sessions))
        self.assertIs(self.owner.recording,current)
        self.assertIsNone(self.owner.recording_error)
        for value in self.producer.directories.values():
            request=json.loads((Path(value)/'recording-request.json').read_text())
            self.assertEqual(request['execution_instance_id'],new.execution_instance_id)

    def assert_late_link_revoked(self,*,stop):
        entered,release,visible,return_link=(threading.Event() for _ in range(4))
        self.releases.extend((release,return_link));actual_link=os.link
        def delayed_link(source,destination):
            entered.set()
            if not release.wait(5):raise TimeoutError('CPU fixture link not released')
            actual_link(source,destination);visible.set()
            if not return_link.wait(5):raise TimeoutError('CPU fixture post-link not released')
        with patch('real_recording.os.link',delayed_link):
            runner=self.start();self.until(entered.is_set)
            if stop:
                started=time.monotonic();self.owner.submit(dict(action='stop'))
                self.assertLess(time.monotonic()-started,.1)
                self.until(lambda:runner.status=='STOPPED')
            else:
                self.until(lambda:runner.status=='READY')
                self.assertIn('exceeded 1 wall second',self.owner.recording_error)
            release.set();self.assertTrue(visible.wait(2))
            for _ in range(100):self.step()
            self.producer.queue.join()
            self.assertTrue(list((self.root/'spool').rglob('recording-request.json')))
            self.assertFalse(list((self.root/'spool').rglob('*.zip')),'revoked visible lease must not authorize producer')
            return_link.set()
            self.until(lambda:all(session.done.is_set() for session in self.sessions))
        self.assertFalse(list((self.root/'spool').rglob('recording-request.json')))
        if stop:self.assertEqual(runner.last_sent_index,-1)
        else:self.assertGreater(runner.last_sent_index,0)

    def test_stop_during_link_rejects_late_visible_lease_without_blocking_stop(self):
        self.assert_late_link_revoked(stop=True)

    def test_link_timeout_does_not_stop_motion_or_admit_late_visible_lease(self):
        self.assert_late_link_revoked(stop=False)

    def assert_prepared_revoked(self,command):
        runner=self.start();self.until(lambda:self.owner.recording is not None and self.owner.recording.prepared.is_set())
        session=self.owner.recording
        leases=[json.loads((Path(path)/'recording-request.json').read_text()) for path in self.producer.directories.values()]
        self.owner.submit(command)
        self.assertTrue(all(not session.lease_active(lease) for lease in leases))
        self.assertTrue(session.done.wait(3),session.snapshot())
        self.assertFalse(list((self.root/'spool').rglob('recording-request.json')))
        self.assertEqual(runner.last_sent_index,-1)

    def test_release_revokes_prepared_lease_before_a_control_tick(self):
        self.assert_prepared_revoked(dict(action='release',confirmation='ARMS_SUPPORTED_RELEASE'))

    def test_pedal_revokes_prepared_lease_before_a_control_tick(self):
        self.assert_prepared_revoked(dict(action='simulate_pedal'))

    def test_handoff_revokes_a_timed_out_factory_before_its_late_return(self):
        self.runtime=Runtime(ref,FakeSafety,lambda:self.backend,device_id='cpu-preparation-handoff',environment='fake',
            lock_root=self.root/'owner-lock',initial_holder='collection',control_clock=self.clock,
            initialize_backend=lambda backend:backend.set_damping_hold(manual=True),
            jaw_mapping_file=self.root/'jaw.json',hardware_binding=self.root/'binding.json',recording_factory=self.factory)
        self.credentials={name:self.runtime.register_business(name) for name in ('collection','calibration')}
        self.owner=self.runtime.owner;self.backend.owner=self.owner
        entered,release=threading.Event(),threading.Event();self.releases.append(release)
        def blocked(payload):
            entered.set()
            if not release.wait(5):raise TimeoutError('handoff fixture did not release factory')
        self.factory_hook=blocked
        runner=self.start();self.until(entered.is_set);self.until(lambda:runner.status=='READY')
        self.until(lambda:self.owner.snapshot().get('stationary') is True)
        receipt=self.runtime.handoff(self.credentials['collection'],'calibration')
        self.assertEqual(receipt['holder'],'calibration')
        release.set();runner.preparation_thread.join(3)
        self.assertFalse(runner.preparation_thread.is_alive())
        self.until(lambda:self.sessions and all(session.done.is_set() for session in self.sessions))
        self.assertIsNone(self.owner.recording)
        self.assertFalse(list((self.root/'spool').rglob('recording-request.json')))


if __name__=='__main__':unittest.main()
