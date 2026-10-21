"""Real child-process camera IPC tests; CPU pixels only, no SDK or simulator."""
from copy import deepcopy
import errno
import io
from pathlib import Path
import socket
import struct
import sys
import tempfile
import time
import unittest
from unittest.mock import MagicMock,patch

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'rule_based_workbench')]
from robot_workbench.rgbd_service import CommonCameraService,CAMERAS,_address,_receive,MAX_HEADER
from real_rgbd import pair_metadata


class SharedCameraTest(unittest.TestCase):
    def service(self,environment='fake',budget=1024**3,camera_serials=None):
        temp=tempfile.TemporaryDirectory(prefix='rgbd-');self.addCleanup(temp.cleanup)
        self.k={name:dict(serial=str(index+1),width=2,height=2,fx=100.,fy=100.,cx=1.,cy=1.,
                         coefficients=[0.]*5,distortion_model='none',rectified=False)
                for index,name in enumerate(CAMERAS.values())}
        if camera_serials:
            for role,name in CAMERAS.items():self.k[name]['serial']=camera_serials[role]
        service=CommonCameraService.start(self.k,Path(temp.name)/'camera',environment,budget_bytes=budget)
        self.addCleanup(service.close)
        self.owner=dict(owner_pid=1234,command_epoch=3,runtime_nonce='test-owner')
        self.environment=environment
        return service

    def lease(self,service,token='a'*32):
        now=time.time_ns()
        lease=dict(run_id=token,start_wall_ns=now-1,deadline_wall_ns=now+60_000_000_000,
                   execution_instance_id='ipc-test',execution_owner=self.owner,environment=self.environment)
        for channel in service.channels.values():channel.begin(lease)
        return token

    def pixels(self,index=0):
        rgb=bytes((i+index)%256 for i in range(12))
        depth=b''.join(x.to_bytes(2,'little') for x in (0,1,32768,65535))
        return rgb,depth

    def metadata(self,camera,index):
        ns=100_000_000_000+index*1_000_000_000//30
        domain='simulation' if self.environment=='fake' else 'hardware_clock'
        meta=pair_metadata(serial=self.k[camera]['serial'],camera_name=next(k for k,v in CAMERAS.items() if v==camera),
            color_number=index,depth_number=index,color_ms=ns/1e6,depth_ms=ns/1e6,
            color_domain=domain,depth_domain=domain,host_receive_ns=time.time_ns(),depth_scale_m=.001,
            intrinsics=self.k[camera])
        meta.update(capture_instance_id='same-source',environment=self.environment,
            simulation_only=self.environment=='fake',evidence_source='fake_device_rgbd' if self.environment=='fake' else 'physical_rgbd',
            host_receive_ns=int(meta['host_receive_ns']),data_valid=True)
        if self.environment=='fake':meta.update(clock_domain='simulation',source_clock_id='cpu-source',source_time_ns=ns,
            physics_step=index*8,timestamp_kind='simulation_step',owner_available=True,**self.owner)
        else:meta.update(clock_domain='host_wall',source_clock_id=None)
        return meta

    def publish(self,service,camera,index,token):
        return service.publish(camera,self.metadata(camera,index),*self.pixels(index),width=2,height=2,sampled_token=token)

    def test_both_ingress_modes_use_same_raw_channel_and_preview_codec(self):
        from PIL import Image
        for environment in ('fake','physical'):
            service=self.service(environment);token=self.lease(service)
            for name in CAMERAS.values():
                response=self.publish(service,name,0,token)
                self.assertTrue(response['recorded'])
                frame=service.channels[name].peek(token)
                self.assertEqual((frame['rgb'],frame['depth']),self.pixels())
                self.assertEqual(frame['pixel_format'],'rgb8-depth16le')
                self.assertNotIn('color_png',frame)
                latest=service.latest_rgbd(name)
                self.assertEqual(latest['metadata'],frame['metadata'])
                preview=service.frames[name].snapshot()
                with Image.open(io.BytesIO(preview['color_png'])) as image:
                    self.assertEqual(image.tobytes(),frame['rgb'])
                with Image.open(io.BytesIO(preview['depth_png'])) as image:
                    self.assertEqual(list(image.getdata()),[0,1,32768,65535])
                if environment=='physical':
                    self.assertNotIn('owner_pid',frame['metadata'])
                    self.assertEqual(frame['metadata']['recording_execution_owner'],self.owner)
            self.assertEqual(service.snapshot()['recording_budget']['max_bytes'],1024**3)
            for ch in service.channels.values():ch.end(token)
            self.assertEqual(service.snapshot()['recording_budget']['queued_bytes'],0)
            service.close();self.assertIsNotNone(service.process.poll())

    def test_old_lease_wrong_source_and_wrong_environment_are_rejected(self):
        service=self.service();token=self.lease(service)
        self.publish(service,'base_0',0,token)
        meta=self.metadata('base_0',1);meta['capture_instance_id']='other-process'
        with self.assertRaisesRegex(ValueError,'capture instance changed'):
            service.publish('base_0',meta,*self.pixels(),width=2,height=2,sampled_token=token)
        meta=self.metadata('base_0',1);meta['environment']='physical'
        with self.assertRaisesRegex(ValueError,'identity'):
            service.publish('base_0',meta,*self.pixels(),width=2,height=2,sampled_token=token)
        for ch in service.channels.values():ch.end(token)
        current=self.lease(service,'b'*32)
        self.assertFalse(self.publish(service,'base_0',1,token)['recorded'])
        self.assertIsNone(service.channels['base_0'].peek(current))
        self.assertTrue(self.publish(service,'base_0',2,current)['recorded'])
        with self.assertRaisesRegex(ValueError,'during recording'):
            service.rebind_camera('base_0','same-source')
        for ch in service.channels.values():ch.end(current)
        service.rebind_camera('base_0','same-source')
        meta=self.metadata('base_0',0);meta['capture_instance_id']='replacement-camera'
        self.assertFalse(service.publish('base_0',meta,*self.pixels(),width=2,height=2,sampled_token=None)['recorded'])
        self.assertEqual(service.latest_rgbd('base_0')['metadata']['capture_instance_id'],'replacement-camera')

    def test_ipc_frame_bound_shared_budget_and_disconnect(self):
        service=self.service(budget=5000);token=self.lease(service)
        for index in range(20):self.publish(service,'base_0',index,token)
        snapshot=service.snapshot()
        self.assertLessEqual(snapshot['recording_budget']['queued_bytes'],5000)
        self.assertIn('shared byte budget',snapshot['recording_channels']['base_0']['error'])
        latest=service.latest_rgbd('base_0')
        self.assertTrue(latest['metadata']['data_valid'])
        self.assertIn('shared byte budget',latest['metadata']['recording_error'])
        with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as connection:
            connection.settimeout(2);connection.connect(_address(service.descriptor['endpoint']))
            connection.sendall(struct.pack('!I',MAX_HEADER+1))
            self.assertIn('header exceeds',_receive(connection)['error'])
        for ch in service.channels.values():ch.end(token)
        self.assertEqual(service.snapshot()['recording_budget']['queued_bytes'],0)
        service.close()
        with self.assertRaises((OSError,ConnectionError)):service.latest_rgbd('base_0')

    def test_connect_eagain_retries_only_before_sending_with_one_shared_timeout(self):
        service=CommonCameraService(dict(endpoint='/cpu/camera.sock',secret='private-fixture',intrinsics={},directories={}))
        sockets=[];clock=[10.]
        def connection(*args):
            sock=MagicMock();sock.__enter__.return_value=sock
            if len(sockets)<2:sock.connect.side_effect=BlockingIOError(errno.EAGAIN,'temporary backlog')
            sockets.append(sock);return sock
        with patch('robot_workbench.rgbd_service.socket.socket',side_effect=connection),\
                patch('robot_workbench.rgbd_service.time.monotonic',side_effect=lambda:clock[0]),\
                patch('robot_workbench.rgbd_service.time.sleep',side_effect=lambda value:clock.__setitem__(0,clock[0]+value)),\
                patch('robot_workbench.rgbd_service._send') as send,\
                patch('robot_workbench.rgbd_service._receive',return_value={'recorded':True}) as receive:
            metadata={'host_receive_ns':123,'pair_id':'original-pair'}
            self.assertTrue(service.call('publish',metadata=metadata,rgb=b'raw')['recorded'])
            self.assertEqual(len(sockets),3);send.assert_called_once();receive.assert_called_once_with(sockets[-1])
            self.assertEqual(send.call_args.args[1]['metadata'],metadata)
            self.assertEqual(send.call_args.args[1]['rgb'],b'raw')
            self.assertAlmostEqual(sockets[-1].settimeout.call_args.args[0],4.99)
            self.assertTrue(all(sock.__exit__.call_count==1 for sock in sockets))
        # Exhaustion never sends, and every attempt consumes the original budget.
        clock[0]=0.;sockets.clear()
        def blocked(*args):
            sock=MagicMock();sock.__enter__.return_value=sock
            sock.connect.side_effect=BlockingIOError(errno.EWOULDBLOCK,'temporary backlog')
            sockets.append(sock);return sock
        sleeps=[]
        def delay(value):sleeps.append(value);clock[0]+=2.
        with patch('robot_workbench.rgbd_service.socket.socket',side_effect=blocked),\
                patch('robot_workbench.rgbd_service.time.monotonic',side_effect=lambda:clock[0]),\
                patch('robot_workbench.rgbd_service.time.sleep',side_effect=delay),\
                patch('robot_workbench.rgbd_service._send') as send:
            with self.assertRaises(TimeoutError):service.call('publish',metadata={'pair_id':'unchanged'})
            send.assert_not_called();self.assertEqual(len(sockets),4)
            self.assertTrue(all(0<value<=.005 for value in sleeps))
        # Errors after a request starts, and permanent connect errors, are never replayed.
        for phase in ('connect','send','receive'):
            sock=MagicMock();sock.__enter__.return_value=sock
            if phase=='connect':sock.connect.side_effect=ConnectionRefusedError(errno.ECONNREFUSED,'closed')
            with patch('robot_workbench.rgbd_service.socket.socket',return_value=sock) as create,\
                    patch('robot_workbench.rgbd_service.time.sleep') as sleep,\
                    patch('robot_workbench.rgbd_service._send',side_effect=BlockingIOError(errno.EAGAIN,'send failed') if phase=='send' else None) as send,\
                    patch('robot_workbench.rgbd_service._receive',side_effect=BlockingIOError(errno.EAGAIN,'read failed') if phase=='receive' else None):
                with self.subTest(phase=phase),self.assertRaises(OSError):service.call('channel',method='pop')
                create.assert_called_once();sleep.assert_not_called()
                self.assertEqual(send.call_count,0 if phase=='connect' else 1)


if __name__=='__main__':unittest.main()
