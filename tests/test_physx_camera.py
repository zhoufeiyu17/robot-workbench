"""CPU image/transport/lease tests; no Isaac, RealSense, GPU or hardware access."""
from copy import deepcopy
import io
import json
import os
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
    @unittest.skipUnless(os.environ.get('WORKBENCH_NATIVE_ROS_TEST')=='1','isolated native ROS required')
    def test_ros_source_preserves_rgbd_and_restarts_only_publication(self):
        from robot_workbench.ros_data import RosNode,SourceDirectory
        from robot_workbench.ros_inputs import RosInputs
        plane=dict(schema='workbench.ros-data-plane.v1',namespace='/wb_physx',domain_id=126)
        with patch('robot_workbench.rgbd_service.CommonCameraService.start',side_effect=AssertionError('legacy IPC source must not start')):
            producer=self.producer(ros_plane=plane,device_id='cpu_fake')
        packets={};publish=producer._publish
        def observed(packet):
            packets[packet['feedback']['physics_step']]=dict(wall=packet['wall_ns'],frame=packet['frame_sequence'],actor=dict(packet['actor']))
            return publish(packet)
        producer._publish=observed
        receiver=RosNode('fake_model_input',namespace='/wb_physx',domain_id=126);self.addCleanup(receiver.close)
        deadline=time.monotonic()+8
        while True:
            self.collect(producer);self.wait(producer)
            try:
                frames={name:producer.common.latest_rgbd(name)for name in CAMERAS.values()}
                break
            except (LookupError,ValueError):
                self.assertLess(time.monotonic(),deadline);time.sleep(.02)
        rows=SourceDirectory(receiver,'cpu_fake').resolve({name:dict(kind='image')for name in CAMERAS.values()})
        model=RosInputs(receiver,rows)
        self.assertEqual(set(model.sources),set(CAMERAS.values()))
        self.assertEqual(producer.channels,{})
        self.assertEqual(producer.spools,{})
        self.assertIsNone(producer.service_owner)
        for name,frame in frames.items():
            self.assertEqual(frame['rgb'],np.arange(36,dtype=np.uint8).tobytes())
            self.assertEqual(frame['depth'],(500).to_bytes(2,'little')*12)
            self.assertEqual(frame['metadata']['source_time_ns'],frame['metadata']['depth_reference']['source_time_ns'])
            self.assertNotIn('frame_readback',frame['metadata'],'state must stay on its unique ROS source')
            original_sample=packets[frame['metadata']['physics_step']]
            self.assertEqual(frame['metadata']['host_receive_ns'],original_sample['wall'])
            self.assertEqual(frame['metadata']['frame_number'],original_sample['frame'])
            for key in ('owner_pid','command_epoch','runtime_nonce'):
                self.assertEqual(frame['metadata'][key],original_sample['actor'][key])
            self.assertEqual(producer.ros.node.count_publishers('/wb_physx/'+name),1)
            self.assertEqual(producer.ros.node.count_publishers('/wb_physx/'+name+'_depth'),1)
        from real_rgbd import snapshot_bundle
        from real_observation import decode_snapshot
        values=producer.frames['base_0'].snapshot(formats=('color_png','depth_png'))
        encoded=snapshot_bundle(values['status'],values['color_png'],values['depth_png'])
        decoded=decode_snapshot(encoded,self.k['base_0']['serial'],{'frames':{'environment_from_camera':
            {'position':[0,0,0],'orientation_wxyz':[1,0,0,0]}}},environment='fake')
        self.assertEqual(decoded[0]['host_receive_ns'],values['status']['host_receive_ns'])
        self.assertTrue(np.all(decoded[-1]['depth_m']==.5))
        from real_bridge import RealBridge
        token=self.root/'bridge-token';token.write_text('test-only-'+40*'x')
        bridge=RealBridge(dict(environment='fake',robot_id='cpu_fake',data_plane=plane,
            control_owner_descriptor='/unused-for-camera',token_file=str(token),control_url='http://127.0.0.1:9',
            cameras={name:dict(url='http://127.0.0.1:9',serial=self.k[name]['serial'])for name in CAMERAS.values()}))
        self.addCleanup(bridge.close)
        bridge.request=lambda *a,**k: self.fail('ROS input must not request HTTP camera/state payloads')
        deadline=time.monotonic()+8
        while True:
            self.collect(producer);self.wait(producer)
            try:body,kind=bridge.camera('base_0','snapshot.zip');break
            except ValueError:
                self.assertLess(time.monotonic(),deadline);time.sleep(.01)
        self.assertEqual(kind,'application/zip')
        observed=decode_snapshot(body,self.k['base_0']['serial'],{'frames':{'environment_from_camera':
            {'position':[0,0,0],'orientation_wxyz':[1,0,0,0]}}},environment='fake')
        self.assertEqual(observed[0]['source_clock_id'],self.backend.source_clock_id)
        original=producer.descriptor['instance_id'];actual=self.plant.read();clock=self.backend.source_clock_id
        self.backend.owner=SimpleNamespace(recording=SimpleNamespace(done=threading.Event()))
        errors=[]
        def blocked_stop():
            try:producer.stop()
            except ValueError as error:errors.append(str(error))
        blocked=threading.Thread(target=blocked_stop);blocked.start();blocked.join(2)
        self.assertEqual(len(errors),1);self.assertIn('recording is active',errors[0]);self.assertFalse(producer.paused)
        self.backend.owner=None
        def maintenance(callback,pump=False):
            results=[];errors=[]
            def call():
                try:results.append(callback(timeout_s=5))
                except Exception as error:errors.append(error)
            worker=threading.Thread(target=call);worker.start();deadline=time.monotonic()+8
            while worker.is_alive()and time.monotonic()<deadline:
                if pump:self.collect(producer)
                time.sleep(.01)
            worker.join(.1);self.assertFalse(worker.is_alive());self.assertEqual(errors,[])
            return results[0]
        stopped=maintenance(producer.stop)
        self.assertFalse(stopped['exit_verified']);self.assertTrue(stopped['process_preserved'])
        self.assertFalse(stopped['source_available']);self.assertEqual(stopped['service_pid'],os.getpid())
        self.assertFalse(producer.collect(self.plant,self.backend,self.actor))
        result=maintenance(producer.start,pump=True)
        self.assertTrue(result['reconnected']);self.assertNotEqual(original,result['instance_id'])
        self.assertEqual(self.backend.source_clock_id,clock);self.assertEqual(self.plant.read(),actual)
        deadline=time.monotonic()+3
        while not model.caches['base_0'].error and time.monotonic()<deadline:time.sleep(.01)
        with self.assertRaises(ValueError):model.latest('base_0')

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
        kwargs.setdefault('memory_recording', False)
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

    def memory_lease(self, producer, token='a'*32,duration_ns=30_000_000_000):
        now=time.time_ns()
        lease=dict(run_id=token,start_wall_ns=now,admission_start_wall_ns=now,
            deadline_wall_ns=now+duration_ns,execution_instance_id='execution-'+token,
            execution_owner={key:self.actor[key] for key in ('owner_pid','command_epoch','runtime_nonce')})
        for channel in producer.channels.values():channel.begin(lease)
        return lease

    def test_memory_delivery_is_exact_bounded_and_creates_no_frame_files(self):
        producer=self.producer(memory_recording=True,recording_guard=lambda _:True)
        self.collect(producer);self.wait(producer)
        lease=self.memory_lease(producer)
        self.collect(producer);self.wait(producer)
        for name,channel in producer.channels.items():
            self.assertEqual(channel.snapshot()['queued_frames'],1)
            peek=channel.peek(lease['run_id'])
            self.assertEqual(peek['metadata']['source_time_ns'],producer.frames[name].snapshot()['status']['source_time_ns'])
            self.assertEqual(peek['rgb'],np.arange(36,dtype=np.uint8).tobytes())
            self.assertEqual(peek['depth'],np.full((3,4),500,dtype='<u2').tobytes())
            peek['metadata']['runtime_nonce']='consumer mutation'
            frame=channel.pop(lease['run_id'])
            self.assertEqual(frame['metadata']['runtime_nonce'],self.actor['runtime_nonce'])
            self.assertEqual(channel.latest(lease['run_id']),frame['metadata'])
            self.assertEqual(channel.snapshot()['queued_bytes'],0)
            with self.assertRaisesRegex(ValueError,'already leased'):channel.begin(lease)
            self.assertFalse(channel.end('b'*32))
            self.assertEqual(channel.token(),lease['run_id'])
        files=[path for path in self.root.rglob('*') if path.is_file()]
        self.assertEqual(len([path for path in files if path.name=='recording-service.json']),3)
        self.assertFalse([path for path in files if path.suffix in ('.png','.zip')])
        for path in files:
            if path.name=='recording-service.json':
                self.assertEqual(json.loads(path.read_text())['transport'],'unix_rgbd')

    def test_late_encoded_packet_and_revoked_owner_do_not_enter_next_memory_run(self):
        live=[True]
        producer=self.producer(memory_recording=True,recording_guard=lambda _:live[0])
        lease=self.memory_lease(producer)
        entered,release=threading.Event(),threading.Event();self.addCleanup(release.set)
        publish=producer._publish
        def delayed(packet):
            entered.set();release.wait(3);return publish(packet)
        producer._publish=delayed
        self.collect(producer);self.assertTrue(entered.wait(1))
        for channel in producer.channels.values():channel.end(lease['run_id'])
        current=self.memory_lease(producer,'b'*32)
        release.set();self.wait(producer)
        for channel in producer.channels.values():self.assertIsNone(channel.peek(current['run_id']))
        live[0]=False
        self.collect(producer);self.wait(producer)
        for channel in producer.channels.values():self.assertIsNone(channel.peek(current['run_id']))
        live[0]=True
        self.collect(producer);self.wait(producer)
        for channel in producer.channels.values():
            self.assertEqual(channel.pop(current['run_id'])['metadata']['frame_number'],2)

    def test_memory_queue_overflow_fails_data_without_blocking_physics_and_new_lease_recovers(self):
        producer=self.producer(memory_recording=True,recording_guard=lambda _:True)
        lease=self.memory_lease(producer)
        self.collect(producer);self.wait(producer)
        for channel in producer.channels.values():
            channel.fail('injected camera recording memory queue overflow',sampled_token=lease['run_id'])
        start=time.monotonic();self.collect(producer)
        self.assertLess(time.monotonic()-start,.2)
        self.wait(producer)
        self.assertEqual(self.backend.sequence,0)
        for channel in producer.channels.values():
            with self.assertRaisesRegex(ValueError,'memory queue overflow'):channel.peek(lease['run_id'])
            self.assertEqual(channel.snapshot()['queued_frames'],1)
            channel.end(lease['run_id'])
        current=self.memory_lease(producer,'b'*32)
        self.collect(producer);self.wait(producer)
        for channel in producer.channels.values():
            self.assertTrue(channel.pop(current['run_id'])['metadata']['data_valid'])
            self.assertIsNone(channel.snapshot()['error'])

    def test_memory_source_drop_is_immediately_visible_and_lease_renewal_keeps_admission(self):
        producer=self.producer(memory_recording=True,recording_guard=lambda _:True)
        lease=self.memory_lease(producer)
        channel=producer.channels['base_0']
        channel.renew(lease['run_id'],lease['deadline_wall_ns'],lease['deadline_wall_ns']+1_000_000_000)
        self.assertEqual(channel.current_lease()['admission_start_wall_ns'],lease['admission_start_wall_ns'])
        with self.assertRaisesRegex(ValueError,'ownership/window changed'):
            channel.renew(lease['run_id'],lease['deadline_wall_ns'],lease['deadline_wall_ns']+2_000_000_000)
        publish=producer._publish
        def broken(packet):
            producer._publish=publish
            raise ValueError('memory encode fixture failed')
        producer._publish=broken
        self.collect(producer);self.wait(producer,'ValueError: memory encode fixture failed')
        for channel in producer.channels.values():
            with self.assertRaisesRegex(ValueError,'memory encode fixture failed'):channel.latest(lease['run_id'])
        self.assertFalse(list(self.root.rglob('*.png'))+list(self.root.rglob('*.zip')))
        self.assertEqual(len(list(self.root.rglob('recording-service.json'))),3)

    def test_memory_byte_limit_and_lease_expiry_are_enforced(self):
        producer=self.producer(memory_recording=True,recording_guard=lambda _:True,recording_budget_bytes=1)
        lease=self.memory_lease(producer)
        self.collect(producer);self.wait(producer)
        for channel in producer.channels.values():
            self.assertEqual(channel.snapshot()['queued_bytes'],0)
            with self.assertRaisesRegex(ValueError,'memory queue overflow'):channel.pop(lease['run_id'])
            channel.end(lease['run_id'])
        current=self.memory_lease(producer,'b'*32,duration_ns=100_000_000)
        time.sleep(.12)
        self.collect(producer);self.wait(producer)
        for channel in producer.channels.values():
            self.assertIsNone(channel.peek(current['run_id']))
            with self.assertRaisesRegex(ValueError,'expired leases cannot resume'):
                channel.renew(current['run_id'],current['deadline_wall_ns'],current['deadline_wall_ns']+1_000_000_000)


if __name__=='__main__':unittest.main()
