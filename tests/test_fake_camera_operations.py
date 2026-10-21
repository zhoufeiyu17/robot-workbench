"""Fake camera maintenance with real CPU service PIDs and a main-thread plant fixture."""
from copy import deepcopy
import io
import json
from pathlib import Path
import sys
import threading
import time
import unittest
import zipfile
from unittest.mock import patch
from urllib.request import build_opener,ProxyHandler

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'tests')]
import test_physx_camera as fixtures
from robot_workbench.physx_camera import CAMERAS


class FakeCameraOperationsTest(unittest.TestCase):
    setUp=fixtures.PhysxCameraTest.setUp
    producer=fixtures.PhysxCameraTest.producer
    collect=fixtures.PhysxCameraTest.collect
    wait=fixtures.PhysxCameraTest.wait
    memory_lease=fixtures.PhysxCameraTest.memory_lease

    def start_producer(self):
        producer=self.producer(memory_recording=True,recording_guard=lambda _:True)
        self.collect(producer);self.wait(producer)
        return producer

    def operation(self,producer,callback=None,*,pump=True,timeout_s=5.):
        result=[];errors=[];phases=[]
        def execute():
            try:result.append((callback or producer.restart)(lambda phase,*args:phases.append(phase)))
            except Exception as error:errors.append(error)
        thread=threading.Thread(target=execute);thread.start()
        deadline=time.monotonic()+timeout_s
        while thread.is_alive() and time.monotonic()<deadline:
            if pump:self.collect(producer)
            time.sleep(.01)
        thread.join(timeout=.1)
        self.assertFalse(thread.is_alive(),'camera maintenance did not complete')
        return result,errors,phases

    def test_restart_preserves_plant_clients_clock_and_ports_with_fresh_pixels(self):
        import numpy as np
        from PIL import Image
        producer=self.start_producer();ports=producer.start_servers()
        old=producer.service_owner;client=producer.common
        frames,channels,directories=producer.frames,producer.channels,producer.directories
        plant_state=deepcopy(self.plant.read());clock=self.backend.source_clock_id
        started=producer.start();self.assertTrue(started['already_running'])
        self.assertEqual(started['instance_id'],old.descriptor['instance_id'])
        self.assertEqual(set(started['cameras']),set(CAMERAS.values()))
        for camera,row in started['cameras'].items():
            self.assertEqual(row['host_receive_ns'],client.latest_rgbd(camera)['metadata']['host_receive_ns'])
            self.assertEqual(row['rgb_bytes'],36);self.assertEqual(row['depth_bytes'],24)
            self.assertEqual(row['serial'],producer.intrinsics[camera]['serial']);self.assertTrue(row['pair_id'])
        self.assertIs(producer.service_owner,old)
        sample_threads=[]
        def cameras(*,render):
            self.assertFalse(render);sample_threads.append(threading.get_ident())
            return {name:dict(rgb=np.full((3,4,3),211,dtype=np.uint8),
                             depth=np.full((3,4),.75,dtype=np.float32)) for name in CAMERAS.values()}
        self.plant.cameras=cameras
        reconnect=client.reconnect;calls=[]
        def busy_once(**kwargs):
            calls.append(True)
            if len(calls)==1:raise ValueError('camera operations are active; reconnect requires idle clients')
            return reconnect(**kwargs)
        with patch.object(client,'reconnect',side_effect=busy_once):
            result,errors,phases=self.operation(producer)
        self.assertEqual(errors,[]);self.assertEqual(len(calls),2)
        self.wait(producer)
        receipt=result[0];self.assertNotEqual(receipt['old']['instance_id'],receipt['new']['instance_id'])
        self.assertEqual(receipt['new']['service_pid'],producer.service_owner.process.pid)
        self.assertIsNotNone(old.process.poll())
        self.assertIs(producer.common,client);self.assertIs(producer.frames,frames)
        self.assertIs(producer.channels,channels);self.assertIs(producer.directories,directories)
        self.assertEqual(receipt['source_clock_id'],clock);self.assertEqual(self.plant.read(),plant_state)
        self.assertEqual(set(sample_threads),{threading.get_ident()})
        self.assertEqual(receipt['preview_ports'],ports)
        self.assertEqual(json.loads(producer.descriptor_path.read_text())['instance_id'],receipt['new']['instance_id'])
        self.assertEqual(producer.descriptor_path.stat().st_mode&0o777,0o600)
        self.assertEqual(phases,['pausing_camera_samples','starting_camera_service',
                                 'waiting_for_fresh_camera_frames','rebinding_camera_clients'])
        for role,camera in CAMERAS.items():
            frame=client.latest_rgbd(camera)
            self.assertEqual(frame['rgb'],bytes([211])*36)
            self.assertEqual(frame['depth'],(750).to_bytes(2,'little')*12)
            self.assertIsNone(channels[camera].current_lease())
            with build_opener(ProxyHandler({})).open(f'http://127.0.0.1:{ports[role]}/snapshot.zip',timeout=2) as response:
                with zipfile.ZipFile(io.BytesIO(response.read())) as archive:
                    with Image.open(io.BytesIO(archive.read('color.png'))) as image:self.assertEqual(image.tobytes(),frame['rgb'])

    def test_pending_old_packet_drains_before_old_service_is_stopped(self):
        producer=self.start_producer();old=producer.service_owner
        entered=threading.Event();release=threading.Event();published=producer._publish;packets=[]
        def hold(packet):
            packets.append(packet);entered.set();release.wait(3)
            return published(packet)
        producer._publish=hold
        self.collect(producer);self.assertTrue(entered.wait(1))
        done=threading.Event();errors=[]
        def restart():
            try:producer.restart(timeout_s=3)
            except Exception as error:errors.append(error)
            finally:done.set()
        thread=threading.Thread(target=restart);thread.start()
        deadline=time.monotonic()+1
        while not producer.paused and time.monotonic()<deadline:time.sleep(.005)
        self.assertTrue(producer.paused);self.assertIsNone(old.process.poll())
        self.assertFalse(self.collect(producer));self.assertFalse(done.is_set())
        producer._publish=published;release.set()
        while not done.is_set() and time.monotonic()<deadline+4:
            self.collect(producer);time.sleep(.01)
        thread.join(1);self.assertFalse(thread.is_alive());self.assertEqual(errors,[])
        self.wait(producer)
        with self.assertRaisesRegex(ValueError,'previous source service'):producer._publish(packets[0])
        for camera in CAMERAS.values():
            meta=producer.common.latest_rgbd(camera)['metadata']
            self.assertGreater(meta['physics_step'],packets[0]['feedback']['physics_step'])
            self.assertEqual(meta['capture_instance_id'],producer.service_owner.descriptor['instance_id'])

    def test_lease_and_main_thread_restart_are_rejected_without_replacement(self):
        producer=self.start_producer();old=producer.service_owner
        with self.assertRaisesRegex(RuntimeError,'main thread'):producer.restart()
        lease=self.memory_lease(producer)
        result,errors,_=self.operation(producer,pump=False)
        self.assertEqual(result,[]);self.assertIn('recording lease',str(errors[0]))
        self.assertIs(producer.service_owner,old);self.assertIsNone(old.process.poll())
        for channel in producer.channels.values():channel.end(lease['run_id'])

    def test_failed_first_frames_preserve_old_binding_and_explicit_start_can_recover(self):
        producer=self.start_producer();old_id=producer.common.descriptor['instance_id']
        result,errors,_=self.operation(producer,lambda progress:producer.restart(progress,timeout_s=.05),pump=False)
        self.assertEqual(result,[]);self.assertIsInstance(errors[0],TimeoutError)
        self.assertTrue(producer.paused)
        self.assertEqual(producer.common.descriptor['instance_id'],old_id)
        failed_owner=producer.service_owner
        lease=self.memory_lease(failed_owner)
        _,blocked,_=self.operation(producer,producer.start,pump=False)
        self.assertIn('recording lease',str(blocked[0]))
        self.assertTrue(producer.paused);self.assertIs(producer.service_owner,failed_owner)
        for channel in failed_owner.channels.values():channel.end(lease['run_id'])
        result,errors,_=self.operation(producer,producer.start)
        self.assertEqual(errors,[]);self.assertTrue(result[0]['reconnected'])
        self.assertIsNotNone(failed_owner.process.poll());self.assertFalse(producer.paused)
        self.wait(producer)

    def test_dead_service_keeps_encoder_alive_for_explicit_restart(self):
        producer=self.start_producer();old=producer.service_owner;old.close()
        self.collect(producer)
        deadline=time.monotonic()+2
        while producer.queue.unfinished_tasks and time.monotonic()<deadline:time.sleep(.01)
        self.assertTrue(producer.worker.is_alive())
        result,errors,_=self.operation(producer,producer.start)
        self.assertEqual(errors,[]);self.assertTrue(result[0]['reconnected'])
        self.wait(producer)

    def test_start_waits_for_actual_first_frames_without_restarting_warming_service(self):
        producer=self.producer(memory_recording=True,recording_guard=lambda _:True)
        owner=producer.service_owner;client=producer.common
        result,errors,_=self.operation(producer,lambda progress:producer.start(progress,timeout_s=.04),pump=False)
        self.assertEqual(result,[]);self.assertIsInstance(errors[0],TimeoutError)
        self.assertIs(producer.service_owner,owner);self.assertIsNone(owner.process.poll())
        result,errors,_=self.operation(producer,producer.start)
        self.assertEqual(errors,[]);self.assertTrue(result[0]['already_running'])
        self.assertIs(producer.common,client);self.assertEqual(result[0]['instance_id'],owner.descriptor['instance_id'])
        self.assertEqual(set(result[0]['cameras']),set(CAMERAS.values()))
        self.wait(producer)

    def test_start_does_not_retry_identity_or_lease_errors_as_warmup(self):
        producer=self.start_producer()
        for message in ('camera identity changed','recording lease remains','ValueError: raw camera frame stale or host clock inconsistent'):
            with patch.object(producer.common,'latest_rgbd',side_effect=ValueError(message)) as read:
                began=time.monotonic()
                with self.assertRaisesRegex(ValueError,message):producer.start(timeout_s=1)
                self.assertLess(time.monotonic()-began,.1);read.assert_called_once()
        frame=producer.common.latest_rgbd('base_0');frame['metadata']['source_clock_id']='different-plant'
        with patch.object(producer.common,'latest_rgbd',return_value=frame):
            with self.assertRaisesRegex(ValueError,'another plant clock'):producer.start(timeout_s=1)

    def test_stop_is_idempotent_preserves_other_service_and_start_restores_fresh_frames(self):
        from robot_workbench.rgbd_service import CommonCameraService
        producer=self.start_producer();ports=producer.start_servers();owner=producer.service_owner
        other=CommonCameraService.start(self.k,self.root/'unrelated-camera','fake')
        self.addCleanup(other.close)
        client=producer.common;frames=producer.frames;channels=producer.channels
        clock=self.backend.source_clock_id;plant=deepcopy(self.plant.read());actor=deepcopy(self.actor)
        descriptor=producer.descriptor_path.read_bytes();before=client.latest_rgbd('base_0')['metadata']
        result,errors,phases=self.operation(producer,producer.stop,pump=False)
        self.assertEqual(errors,[]);receipt=result[0]
        self.assertTrue(receipt['stopped']);self.assertTrue(receipt['exit_verified'])
        self.assertFalse(receipt['already_stopped']);self.assertEqual(receipt['service_pid'],owner.process.pid)
        self.assertEqual(phases,['pausing_camera_samples','stopping_camera_service'])
        self.assertIsNotNone(owner.process.poll());self.assertIsNone(other.process.poll())
        self.assertTrue(producer.paused);self.assertTrue(producer.worker.is_alive())
        self.assertFalse(producer.closed.is_set());self.assertEqual(producer.descriptor_path.read_bytes(),descriptor)
        self.assertFalse(self.collect(producer));self.assertEqual(self.plant.read(),plant);self.assertEqual(self.actor,actor)
        self.assertEqual(producer.clock_id,clock);self.assertEqual(self.backend.source_clock_id,clock)
        result,errors,_=self.operation(producer,producer.stop,pump=False)
        self.assertEqual(errors,[]);self.assertTrue(result[0]['already_stopped'])
        self.assertEqual(result[0]['service_pid'],owner.process.pid)
        result,errors,_=self.operation(producer,producer.start)
        self.assertEqual(errors,[]);self.assertTrue(result[0]['reconnected']);self.wait(producer)
        self.assertFalse(producer.paused);self.assertIsNone(other.process.poll())
        self.assertIs(producer.common,client);self.assertIs(producer.frames,frames);self.assertIs(producer.channels,channels)
        self.assertEqual(result[0]['preview_ports'],ports)
        self.assertEqual(result[0]['source_clock_id'],clock)
        for name in CAMERAS.values():
            frame=client.latest_rgbd(name);meta=frame['metadata']
            self.assertGreater(meta['physics_step'],before['physics_step'])
            self.assertNotEqual(meta['capture_instance_id'],before['capture_instance_id'])
            self.assertEqual(meta['source_clock_id'],clock)
            self.assertEqual(len(frame['rgb']),36);self.assertEqual(len(frame['depth']),24)

    def test_stop_rejects_local_or_other_client_lease_without_stopping_service(self):
        producer=self.start_producer();owner=producer.service_owner
        with self.assertRaisesRegex(RuntimeError,'main thread'):producer.stop()
        for holder in (producer,owner):
            lease=self.memory_lease(holder)
            try:
                result,errors,_=self.operation(producer,producer.stop,pump=False)
                self.assertEqual(result,[]);self.assertIn('recording lease',str(errors[0]))
                self.assertIsNone(owner.process.poll());self.assertFalse(producer.paused)
            finally:
                for channel in holder.channels.values():channel.end(lease['run_id'])

    def test_stop_drains_pending_packet_and_retains_service_on_drain_timeout(self):
        producer=self.start_producer();owner=producer.service_owner
        entered=threading.Event();release=threading.Event();publish=producer._publish
        self.addCleanup(release.set)
        def hold(packet):
            entered.set();release.wait(3);return publish(packet)
        producer._publish=hold;self.collect(producer);self.assertTrue(entered.wait(1))
        result,errors,_=self.operation(producer,lambda progress:producer.stop(progress,timeout_s=.03),pump=False)
        self.assertEqual(result,[]);self.assertIsInstance(errors[0],TimeoutError)
        self.assertIn('did not become idle',str(errors[0]));self.assertIsNone(owner.process.poll())
        self.assertFalse(producer.paused)
        result=[];errors=[]
        def stop():
            try:result.append(producer.stop(timeout_s=2))
            except Exception as error:errors.append(error)
        worker=threading.Thread(target=stop);worker.start()
        deadline=time.monotonic()+1
        while not producer.paused and time.monotonic()<deadline:time.sleep(.005)
        self.assertTrue(producer.paused);self.assertTrue(worker.is_alive());self.assertIsNone(owner.process.poll())
        self.assertFalse(self.collect(producer));release.set();worker.join(3)
        self.assertFalse(worker.is_alive());self.assertEqual(errors,[]);self.assertTrue(result[0]['exit_verified'])
        self.assertEqual(producer.queue.unfinished_tasks,0);self.assertIsNotNone(owner.process.poll())

    def test_unknown_stop_exit_remains_paused_and_requires_verified_retry(self):
        producer=self.start_producer();owner=producer.service_owner
        with patch.object(owner,'close',return_value=None):
            result,errors,_=self.operation(producer,producer.stop,pump=False)
        self.assertEqual(result,[]);self.assertIsInstance(errors[0],TimeoutError)
        self.assertIn('exit remains unknown',str(errors[0]));self.assertIsNone(owner.process.poll())
        self.assertTrue(producer.paused);self.assertFalse(self.collect(producer))
        result,errors,_=self.operation(producer,producer.stop,pump=False)
        self.assertEqual(errors,[]);self.assertTrue(result[0]['exit_verified'])
        self.assertIsNotNone(owner.process.poll())


if __name__=='__main__':unittest.main()
