"""CPU image/transport/lease tests; no Isaac, RealSense, GPU or hardware access."""
from copy import deepcopy
import io
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
from types import SimpleNamespace
from urllib.request import build_opener,ProxyHandler
import zipfile
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'calibration/infra'),str(ROOT/'rule_based_workbench'),str(ROOT/'tests')]
from robot_workbench.physx import PhysxBackend
from robot_workbench.physx_camera import CameraProducer,CAMERAS,camera_intrinsics
from test_physx_boundary import LaggingPlant
from real_recording import bundle


class CameraPlant(LaggingPlant):
    def cameras(self,*,render):
        if render:raise AssertionError('producer must not request extra render/step')
        return {name:dict(rgb=np.arange(36,dtype=np.uint8).reshape(3,4,3),
                          depth=np.full((3,4),.5,dtype=np.float32)) for name in CAMERAS.values()}


class PhysxCameraTest(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name)
        rig={'device_calibrations':{name:dict(serial=str(i+1),width=4,height=3,
             intrinsics=dict(fx=100.,fy=100.,cx=2.,cy=1.),distortion=dict(coefficients=[0.]*5))
             for i,name in enumerate(CAMERAS.values())}}
        self.k=camera_intrinsics(rig)
        self.plant=CameraPlant();self.backend=PhysxBackend(self.plant,joint_offsets=[0.]*12)
        self.actor=dict(owner_available=True,owner_pid=1234,command_epoch=7,runtime_nonce='runtime-at-sample')

    def producer(self,**kwargs):
        producer=CameraProducer(self.k,self.root/'spool',min_free_bytes=0,**kwargs)
        self.addCleanup(producer.close)
        return producer

    def collect(self,producer,actor=None):
        self.backend.advance((self.backend.physics_step+1)/240)
        return producer.collect(self.plant,self.backend,actor or self.actor)

    def wait(self,producer,expected_error=None):
        deadline=time.monotonic()+3
        while producer.queue.unfinished_tasks and time.monotonic()<deadline:time.sleep(.005)
        self.assertEqual(producer.queue.unfinished_tasks,0)
        self.assertEqual(producer.error,expected_error)

    def lease(self,producer):
        now=time.time_ns();request=dict(run_id='a'*32,start_wall_ns=now-1_000_000,
                                       deadline_wall_ns=now+30_000_000_000)
        for path in producer.directories.values():
            (Path(path)/'recording-request.json').write_text(json.dumps(request))

    def test_http_is_cached_same_plant_pair_with_sampling_identity_and_no_idle_spool(self):
        producer=self.producer();ports=producer.start_servers()
        self.assertTrue(self.collect(producer))
        self.actor.update(command_epoch=8,runtime_nonce='later-runtime')
        self.wait(producer)
        http=build_opener(ProxyHandler({}))
        steps=self.plant.steps
        for role,name in CAMERAS.items():
            with http.open(f'http://127.0.0.1:{ports[role]}/snapshot.zip',timeout=2) as response:
                raw=response.read();meta,_,_=bundle(raw,self.k[name]['serial'],environment='fake')
            self.assertEqual(meta['owner_pid'],1234)
            self.assertEqual(meta['command_epoch'],7)
            self.assertEqual(meta['runtime_nonce'],'runtime-at-sample')
            self.assertEqual(meta['source_clock_id'],self.backend.source_clock_id)
            self.assertEqual(meta['source_time_ns'],1_000_000_000//240)
            self.assertEqual(meta['physics_step'],1)
            self.assertEqual(meta['frame_number'],0)
            self.assertEqual(meta['frame_readback']['qpos'],self.backend.read_calibration_state()['qpos'])
            self.assertFalse(meta['frame_readback']['owner_feedback_event'])
            self.assertEqual(type(meta['host_receive_ns']),int)
            for endpoint in ('/status','/frame.jpg','/raw-frame.jpg','/depth.png'):
                with http.open(f'http://127.0.0.1:{ports[role]}'+endpoint,timeout=2) as response:
                    self.assertEqual(response.status,200);self.assertTrue(response.read())
        self.assertEqual(self.plant.steps,steps)
        self.assertFalse(list(self.root.rglob('*.zip')))
        self.assertFalse(producer.collect(self.plant,self.backend,self.actor))
        self.lease(producer);self.collect(producer);self.wait(producer)
        self.assertEqual(len(list(self.root.rglob('*.zip'))),3)
        timings=producer.snapshot()['last_publish']
        self.assertEqual(set(timings['cameras']),set(CAMERAS.values()))
        self.assertGreaterEqual(timings['queue_wait_s'],0)
        for row in timings['cameras'].values():
            self.assertGreaterEqual(row['total_wall_s'],row['encode_wall_s']+row['spool_write_wall_s'])

    def test_overflow_invalidates_recording_but_does_not_change_plant(self):
        producer=self.producer(queue_capacity=1)
        entered,release=threading.Event(),threading.Event()
        publish=producer._publish
        def blocked(packet):
            entered.set();release.wait(3);return publish(packet)
        producer._publish=blocked
        self.addCleanup(release.set)
        self.lease(producer)
        self.collect(producer);self.assertTrue(entered.wait(1))
        self.assertTrue(self.collect(producer))
        started=time.monotonic()
        self.assertFalse(self.collect(producer))
        self.assertLess(time.monotonic()-started,.2,'full camera queue must not wait for encoding or storage')
        self.assertEqual(self.backend.physics_step,3)
        self.assertEqual(self.backend.sequence,0)
        release.set();self.wait(producer)
        self.assertEqual(producer.snapshot()['dropped_batches'],1)
        drop=producer.snapshot()['last_drop']
        self.assertEqual(drop['reason'],'camera source queue overflow')
        self.assertEqual(drop['physics_step'],3)
        self.assertEqual(drop['source_clock_id'],self.backend.source_clock_id)
        self.assertEqual(drop['queue_capacity'],1)
        self.assertEqual(drop['queue_size'],1)
        self.assertGreaterEqual(drop['detected_wall_ns'],drop['sample_wall_ns'])
        for name,frame in producer.frames.items():
            status=frame.snapshot()['status']
            self.assertTrue(status['ready'])
            self.assertFalse(status['data_valid'])
            self.assertIn('overflow',status['recording_error'])
        for path in self.root.rglob('*.zip'):
            with zipfile.ZipFile(io.BytesIO(path.read_bytes())) as archive:
                meta=json.loads(archive.read('metadata.json'))
                self.assertFalse(meta['data_valid'])
                self.assertEqual(meta['dropped_preview_batches'],1)
                self.assertIn('camera source queue overflow',meta['recording_error'])
                with self.assertRaisesRegex(ValueError,'camera source queue overflow'):
                    bundle(path.read_bytes(),meta['serial'],environment='fake')

    def test_encoder_failure_keeps_its_reason_without_poisoning_a_later_lease(self):
        producer=self.producer();self.lease(producer)
        publish=producer._publish
        def fail_once(packet):
            producer._publish=publish
            raise ValueError('encoder fixture failure')
        producer._publish=fail_once
        self.collect(producer);self.wait(producer,'ValueError: encoder fixture failure')
        self.collect(producer);self.wait(producer,'ValueError: encoder fixture failure')
        for path in self.root.rglob('*.zip'):
            with zipfile.ZipFile(path) as archive:
                status=json.loads(archive.read('metadata.json'))
            self.assertFalse(status['data_valid'])
            self.assertIn('encoder fixture failure',status['recording_error'])
            self.assertNotIn('overflow',status['recording_error'])
        now=time.time_ns()
        for path in producer.directories.values():
            (Path(path)/'recording-request.json').write_text(json.dumps(dict(run_id='b'*32,
                start_wall_ns=now,deadline_wall_ns=now+30_000_000_000)))
        self.collect(producer);self.wait(producer,'ValueError: encoder fixture failure')
        for frame in producer.frames.values():
            status=frame.snapshot()['status']
            self.assertTrue(status['data_valid']);self.assertTrue(status['recording_enabled'])
            self.assertIsNone(status['recording_error'])

    def test_reported_spool_failure_invalidates_data_and_secondary_io_keeps_first_error(self):
        producer=self.producer();self.lease(producer)
        with patch('real_rgbd.shutil.disk_usage',return_value=SimpleNamespace(free=0)):
            self.collect(producer);self.wait(producer)
        for frame in producer.frames.values():
            status=frame.snapshot()['status']
            self.assertFalse(status['data_valid']);self.assertFalse(status['recording_enabled'])
            self.assertIn('disk reserve',status['recording_error'])
        def fail(*args):raise OSError('secondary IO failure')
        for spool in producer.spools.values():spool.write=fail
        producer._lease_drop_error=lambda spool:'original camera queue overflow'
        self.collect(producer);self.wait(producer)
        for frame in producer.frames.values():
            self.assertEqual(frame.snapshot()['status']['recording_error'],'original camera queue overflow')

    def test_unconnected_actor_is_explicit_and_other_threads_cannot_sample(self):
        producer=self.producer()
        self.collect(producer,dict(owner_available=False))
        self.wait(producer)
        meta=producer.frames['base_0'].snapshot()['status']
        self.assertFalse(meta['owner_available']);self.assertIsNone(meta['owner_pid'])
        errors=[]
        def other_thread():
            try:producer.collect(self.plant,self.backend,self.actor)
            except RuntimeError as error:errors.append(str(error))
        thread=threading.Thread(target=other_thread);thread.start();thread.join(1)
        self.assertEqual(len(errors),1)


if __name__=='__main__':unittest.main()
