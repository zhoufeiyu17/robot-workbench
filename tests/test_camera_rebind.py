"""Explicit reconnect against real CPU child services in both camera environments."""
from copy import deepcopy
import io
import json
from pathlib import Path
import sys
import threading
import time
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'rule_based_workbench'),str(ROOT/'tests')]
import test_rgbd_service as fixtures
from robot_workbench.rgbd_service import CommonCameraService,CAMERAS


class CameraReconnectTest(unittest.TestCase):
    service=fixtures.SharedCameraTest.service
    lease=fixtures.SharedCameraTest.lease
    pixels=fixtures.SharedCameraTest.pixels
    metadata=fixtures.SharedCameraTest.metadata
    publish=fixtures.SharedCameraTest.publish

    def source(self,environment,*,capture='replacement',clock='cpu-source',stale=False,intrinsics_drift=False):
        service=self.service(environment)
        for camera in CAMERAS.values():
            meta=self.metadata(camera,3);meta['capture_instance_id']=capture+'-'+camera
            if environment=='fake':meta['source_clock_id']=clock
            if stale:meta['host_receive_ns']-=10_000_000_000
            if intrinsics_drift:meta['intrinsics']['coefficients']=[.01]*5
            service.publish(camera,meta,*self.pixels(77),width=2,height=2,sampled_token=None)
        return service

    def attached(self,environment):
        old=self.source(environment,capture='original')
        path=Path(old.directories['base_0']).parent.parent/'trusted-camera.json'
        self.replace(path,old)
        return old,CommonCameraService.connect(path),path

    def replace(self,path,service,**changes):
        path.write_text(json.dumps(dict(service.descriptor,**changes)))

    def test_replacement_preserves_clients_and_reads_three_real_pixel_pairs(self):
        from PIL import Image
        for environment in ('fake','physical'):
            with self.subTest(environment=environment):
                old,client,path=self.attached(environment)
                frames,channels,directories=client.frames,client.channels,client.directories
                old_frames=dict(frames);old_channels=dict(channels)
                token=self.lease(client)
                for channel in channels.values():channel.end(token)
                new=self.source(environment)
                old_pid=old.process.pid;old.close();self.replace(path,new)
                receipt=client.reconnect()
                self.assertEqual(receipt['old']['service_pid'],old_pid)
                self.assertEqual(receipt['new']['service_pid'],new.process.pid)
                self.assertNotEqual(receipt['old']['instance_id'],receipt['new']['instance_id'])
                self.assertNotIn('secret',json.dumps(receipt))
                self.assertFalse(receipt['recording_leases_restored'])
                self.assertIs(client.frames,frames);self.assertIs(client.channels,channels)
                self.assertIs(client.directories,directories)
                for camera in CAMERAS.values():
                    self.assertIs(frames[camera],old_frames[camera]);self.assertIs(channels[camera],old_channels[camera])
                    self.assertEqual(directories[camera],new.directories[camera])
                    self.assertEqual(receipt['cameras'][camera]['capture_instance_id'],'replacement-'+camera)
                    self.assertTrue(receipt['cameras'][camera]['healthy'])
                    self.assertLess(receipt['cameras'][camera]['source_age_s'],2)
                    latest=client.latest_rgbd(camera)
                    self.assertEqual((latest['rgb'],latest['depth']),self.pixels(77))
                    with Image.open(io.BytesIO(frames[camera].snapshot(['color_png'])['color_png'])) as image:
                        self.assertEqual(image.tobytes(),self.pixels(77)[0])
                    self.assertIsNone(channels[camera].current_lease())
                    with self.assertRaisesRegex(ValueError,'another execution'):channels[camera].pop(token)
                with self.assertRaisesRegex(ValueError,'cannot resume'):self.lease(client,token)
                fresh=self.lease(client,'b'*32)
                for camera in CAMERAS.values():
                    metadata=self.metadata(camera,4);metadata['capture_instance_id']='replacement-'+camera
                    client.publish(camera,metadata,*self.pixels(88),width=2,height=2,sampled_token=fresh)
                    self.assertEqual(channels[camera].pop(fresh)['rgb'],self.pixels(88)[0])
                    late=self.metadata(camera,5);late['capture_instance_id']='original-'+camera
                    with self.assertRaisesRegex(ValueError,'capture instance changed'):
                        client.publish(camera,late,*self.pixels(99),width=2,height=2,sampled_token=token)
                    with self.assertRaisesRegex(ValueError,'previous service binding'):
                        client.call('channel',camera=camera,method='fail',args=['late failure'],
                                    kwargs={'sampled_token':fresh},_generation=0)
                    self.assertIsNone(channels[camera].snapshot()['error'])
                    self.assertEqual(client.latest_rgbd(camera)['rgb'],self.pixels(88)[0])
                    channels[camera].end(fresh)

    def test_live_local_and_other_client_leases_block_reconnect(self):
        for environment in ('fake','physical'):
            old,client,path=self.attached(environment);new=self.source(environment);self.replace(path,new)
            for holder in (client,old,new):
                with self.subTest(environment=environment,holder=holder is client):
                    token=self.lease(holder)
                    with self.assertRaisesRegex(ValueError,'recording lease'):client.reconnect()
                    self.assertEqual(client.descriptor['instance_id'],old.descriptor['instance_id'])
                    for channel in holder.channels.values():channel.end(token)
            self.assertTrue(client.reconnect()['reconnected'])

    def test_missing_stale_or_changed_descriptor_leaves_old_binding_diagnosable(self):
        for environment in ('fake','physical'):
            old,client,path=self.attached(environment)
            new=self.source(environment)
            saved=deepcopy(client.descriptor)
            for change in ('environment','serial','intrinsics','unavailable','invalid','unchanged'):
                with self.subTest(environment=environment,change=change):
                    altered=deepcopy(new.descriptor)
                    if change=='environment':altered['environment']='physical' if environment=='fake' else 'fake'
                    if change=='serial':altered['intrinsics']['base_0']['serial']='different-device'
                    if change=='intrinsics':altered['intrinsics']['base_0']['fx']+=1
                    if change=='unavailable':altered['endpoint']='/tmp/no-camera-reconnect.sock'
                    if change=='unchanged':altered=deepcopy(old.descriptor)
                    path.write_text('broken json' if change=='invalid' else json.dumps(altered))
                    with self.assertRaises(ValueError):client.reconnect()
                    self.assertEqual(client.descriptor,saved)
                    self.assertFalse(client.reconnect_state()['reconnecting'])
                    self.assertEqual(client.latest_rgbd('base_0')['metadata']['capture_instance_id'],'original-base_0')
            for replacement in (self.service(environment),self.source(environment,stale=True),
                                self.source(environment,intrinsics_drift=True)):
                self.replace(path,replacement)
                with self.assertRaises(ValueError):client.reconnect()
                self.assertEqual(client.descriptor,saved)
            if environment=='fake':
                other_clock=self.source(environment,clock='other-plant');self.replace(path,other_clock)
                with self.assertRaisesRegex(ValueError,'source clock changed'):client.reconnect()
                self.assertEqual(client.descriptor,saved)

    def test_active_call_and_concurrent_operations_are_not_moved_or_replayed(self):
        for environment in ('fake','physical'):
            old,client,path=self.attached(environment);new=self.source(environment);self.replace(path,new)
            entered=threading.Event();release=threading.Event();result=[]
            original=client._request
            def held(descriptor,action,**fields):
                entered.set();release.wait(3)
                return original(descriptor,action,**fields)
            with patch.object(client,'_request',side_effect=held):
                worker=threading.Thread(target=lambda:result.append(client.latest_rgbd('base_0')))
                worker.start();self.assertTrue(entered.wait(1))
                try:
                    with self.assertRaisesRegex(ValueError,'active'):client.reconnect()
                finally:release.set();worker.join(3)
            self.assertEqual(result[0]['metadata']['capture_instance_id'],'original-base_0')
            entered.clear();release.clear();errors=[]
            def reconnect():
                try:client.reconnect()
                except Exception as error:errors.append(error)
            with patch.object(client,'_request',side_effect=held):
                worker=threading.Thread(target=reconnect);worker.start();self.assertTrue(entered.wait(1))
                try:
                    with self.assertRaisesRegex(ValueError,'in progress'):client.latest_rgbd('base_0')
                    with self.assertRaisesRegex(ValueError,'active'):client.reconnect()
                finally:release.set();worker.join(3)
            self.assertEqual(errors,[])
            self.assertEqual(client.latest_rgbd('base_0')['metadata']['capture_instance_id'],'replacement-base_0')

    def test_live_unreachable_old_service_cannot_claim_lease_absence(self):
        old,client,path=self.attached('physical');new=self.source('physical');self.replace(path,new)
        with patch.object(client,'_request',side_effect=ConnectionRefusedError('offline')):
            with self.assertRaisesRegex(ValueError,'cannot establish idle lease state'):client.reconnect()
        self.assertEqual(client.descriptor['instance_id'],old.descriptor['instance_id'])

    def test_lost_begin_or_end_reply_retains_lease_and_never_replays(self):
        for environment in ('fake','physical'):
            old,client,path=self.attached(environment);new=self.source(environment);self.replace(path,new)
            now=time.time_ns();token='c'*32
            lease=dict(run_id=token,start_wall_ns=now-1,deadline_wall_ns=now+60_000_000_000,
                execution_instance_id='lost-reply',execution_owner=self.owner,environment=environment)
            channel=client.channels['base_0'];original=client._request
            def lost(descriptor,action,**fields):
                original(descriptor,action,**fields)
                raise ConnectionError('reply lost after mutation')
            with patch.object(client,'_request',side_effect=lost) as request:
                with self.assertRaises(ConnectionError):channel.begin(lease)
                request.assert_called_once()
            self.assertEqual(channel.cached_token,token)
            with self.assertRaisesRegex(ValueError,'recording lease'):client.reconnect()
            with patch.object(client,'_request',side_effect=lost) as request:
                with self.assertRaises(ConnectionError):channel.end(token)
                request.assert_called_once()
            self.assertEqual(channel.cached_token,token)
            with self.assertRaisesRegex(ValueError,'recording lease'):client.reconnect()
            self.assertFalse(channel.end(token))  # Explicit end confirms the lease is already absent.
            self.assertIsNone(channel.cached_token)
            self.assertTrue(client.reconnect()['reconnected'])

    def test_allow_current_validates_pixels_and_leases_without_mutating_binding(self):
        for environment in ('fake','physical'):
            old,client,path=self.attached(environment)
            descriptor=client.descriptor;frames=client.frames;channels=client.channels
            receipt=client.reconnect(allow_current=True)
            self.assertTrue(receipt['already_bound']);self.assertFalse(receipt['reconnected'])
            self.assertEqual(receipt['generation'],0);self.assertIs(client.descriptor,descriptor)
            self.assertIs(client.frames,frames);self.assertIs(client.channels,channels)
            self.assertEqual(receipt['old']['instance_id'],receipt['new']['instance_id'])
            for camera,row in receipt['cameras'].items():
                self.assertEqual(row['rgb_bytes'],12);self.assertEqual(row['depth_bytes'],8)
                self.assertEqual(row['host_receive_ns'],old.latest_rgbd(camera)['metadata']['host_receive_ns'])
            token=self.lease(client)
            with self.assertRaisesRegex(ValueError,'recording lease'):client.reconnect(allow_current=True)
            for channel in channels.values():channel.end(token)
            with self.assertRaises(ValueError):client.reconnect(allow_current='true')
            with patch('robot_workbench.rgbd_service.time.time_ns',return_value=time.time_ns()+10_000_000_000):
                with self.assertRaisesRegex(ValueError,'stale'):client.reconnect(allow_current=True)
            self.assertEqual(client.generation,0)


if __name__=='__main__':unittest.main()
