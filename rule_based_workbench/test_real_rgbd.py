"""Hardware-free check of atomic RGB-D transport and calibration compatibility."""
import io
import json
from pathlib import Path
import signal
from types import SimpleNamespace
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
from http.server import ThreadingHTTPServer
from urllib.error import HTTPError
from urllib.request import urlopen
import zipfile

from rule_based_workbench.real_rgbd import RawSpool, FrameProgress, Frames, main, make_handler, pair_metadata, snapshot_bundle


class RGBDTest(unittest.TestCase):
    def test_native_fifo_preserves_device_frames_during_slow_publish(self):
        """Exercise capture with a 30Hz SDK producer and an 80ms publisher."""
        from collections import deque
        import numpy as np
        from rule_based_workbench.real_rgbd import capture
        class Queue:
            def __init__(self,capacity=1,keep_frames=False):
                self.rows=deque(maxlen=capacity);self.condition=threading.Condition()
            def put(self,frame):
                with self.condition:self.rows.append(frame);self.condition.notify()
            def wait_for_frame(self,timeout):
                with self.condition:
                    if not self.rows:self.condition.wait(timeout/1000)
                    if not self.rows:raise TimeoutError('synthetic SDK stopped')
                    return self.rows.popleft()
        intrinsic=SimpleNamespace(width=4,height=3,fx=4.,fy=4.,ppx=2.,ppy=1.5,model='none',coeffs=[0.]*5)
        class Frame:
            profile=SimpleNamespace(as_video_stream_profile=lambda:SimpleNamespace(get_intrinsics=lambda:intrinsic))
            def __init__(self,n,depth=False):self.n,self.depth=n,depth
            def get_frame_number(self):return self.n
            def get_timestamp(self):return self.n*1000/30
            def get_frame_timestamp_domain(self):return 'timestamp_domain.hardware_clock'
            def get_data(self):return np.ones((3,4),dtype=np.uint16)if self.depth else np.zeros((3,4,2),dtype=np.uint8)
        class Pair:
            def __init__(self,n):self.n=n
            def as_frameset(self):return self
            def get_color_frame(self):return Frame(self.n)
            def get_depth_frame(self):return Frame(self.n,True)
        class Pipeline:
            def start(self,config,queue=None):
                self.queue=queue or Queue();self.closed=threading.Event()
                def produce():
                    n=0
                    while not self.closed.wait(1/30):n+=1;self.queue.put(Pair(n))
                self.thread=threading.Thread(target=produce);self.thread.start()
                return SimpleNamespace(get_device=lambda:SimpleNamespace(first_depth_sensor=lambda:SimpleNamespace(get_depth_scale=lambda:.001)))
            def wait_for_frames(self,timeout):
                frame=self.queue.wait_for_frame(timeout)
                with self.queue.condition:
                    while self.queue.rows:frame=self.queue.rows.pop()
                return frame
            def stop(self):self.closed.set();self.thread.join(2)
        frames=SimpleNamespace(stop_requested=threading.Event(),publish=lambda **kw:None);published=[]
        with tempfile.TemporaryDirectory()as temporary:
            service=SimpleNamespace(descriptor={'environment':'physical','intrinsics':{'base_0':{'serial':'123'}}},
                directories={'base_0':temporary},channels={'base_0':SimpleNamespace(token=lambda:None)})
            def publish(camera,status,*args,**kwargs):
                time.sleep(.08);published.append(status)
                if len(published)==5:frames.stop_requested.set()
            service.publish=publish
            sdk=SimpleNamespace(pipeline=Pipeline,config=lambda:SimpleNamespace(enable_device=lambda *a:None,enable_stream=lambda *a:None),
                stream=SimpleNamespace(color=1,depth=2),format=SimpleNamespace(yuyv=1,z16=2),frame_queue=Queue,
                align=lambda _:SimpleNamespace(process=lambda value:value))
            cv=SimpleNamespace(COLOR_YUV2RGB_YUY2=1,cvtColor=lambda image,mode:np.zeros((3,4,3),dtype=np.uint8))
            args=SimpleNamespace(service_descriptor='synthetic',serial='123',camera_name='head',output=Path(temporary),duration_s=0)
            with patch.dict('sys.modules',{'pyrealsense2':sdk,'cv2':cv}),patch(
                    'robot_workbench.rgbd_service.CommonCameraService.connect',return_value=service):capture(args,frames)
        self.assertEqual(len(published),5)
        self.assertEqual([r['frame_number']for r in published],list(range(21,26)))
        self.assertEqual(published[-1]['color_missing_frames'],0)
        self.assertEqual(published[-1]['depth_missing_frames'],0)

    def test_first_invalid_spooled_frame_preserves_producer_failure(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);now=1_000_000_000
            spool=RawSpool(root,clock_ns=lambda:now,min_free_bytes=0)
            (root/'recording-request.json').write_text(json.dumps(dict(run_id='a'*32,start_wall_ns=now,
                deadline_wall_ns=now+1_000_000_000)))
            status={'data_valid':False,'recording_error':'camera source queue overflow during recording lease'}
            spool.write(status,b'color',b'depth')
            path=next(root.rglob('*.zip'))
            with zipfile.ZipFile(path) as archive:
                raw=json.loads(archive.read('metadata.json'))
            self.assertFalse(raw['data_valid'])
            self.assertEqual(raw['recording_error'],'camera source queue overflow during recording lease')
            self.assertEqual(status['recording_error'],raw['recording_error'])
            with patch('rule_based_workbench.real_rgbd.shutil.disk_usage',return_value=SimpleNamespace(free=0)):
                spool.write(status,b'color',b'depth')
            self.assertFalse(status['recording_enabled'])
            self.assertEqual(status['recording_error'],raw['recording_error'])
            self.assertIn('disk reserve',spool.error,'a later storage failure must not erase the first producer error')

    def test_raw_lease_idle_expiry_and_disk_reserve(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary)
            now=[1_000_000_000]
            spool=RawSpool(root,clock_ns=lambda:now[0],min_free_bytes=100)
            status={}
            spool.write(status,b'color',b'depth')
            self.assertFalse(status['recording_enabled'])
            self.assertEqual(list(root.rglob('*.zip')),[])
            request=root/'recording-request.json'
            request.write_text(json.dumps(dict(run_id='a'*32,start_wall_ns=now[0],deadline_wall_ns=now[0]+1_000_000_000)))
            spool.write(status,b'color',b'depth')
            self.assertEqual(len(list(root.rglob('*.zip'))),1)
            now[0]+=1_000_000_000
            spool.write(status,b'color',b'depth')
            self.assertFalse(status['recording_enabled'])
            self.assertEqual(len(list(root.rglob('*.zip'))),1)
            request.write_text(json.dumps(dict(run_id='b'*32,start_wall_ns=now[0],deadline_wall_ns=now[0]+1_000_000_000)))
            with patch('rule_based_workbench.real_rgbd.shutil.disk_usage',return_value=SimpleNamespace(free=0)):
                spool.write(status,b'color',b'depth')
            self.assertFalse(status['recording_enabled'])
            self.assertIn('disk reserve',status['recording_error'])
            spool.write(status,b'color',b'depth')
            self.assertFalse(status['recording_enabled'])
            self.assertEqual(len(list(root.rglob('*.zip'))),1)

    def test_reused_frameset_rollback_and_stall(self):
        progress = FrameProgress(0)
        self.assertTrue(progress.accept(10, 20, .1))
        self.assertFalse(progress.accept(10, 20, .2))
        self.assertFalse(progress.accept(11, 20, .3))
        self.assertTrue(progress.accept(11, 21, .4))
        with self.assertRaisesRegex(RuntimeError, 'regressed'):
            progress.accept(10, 22, .5)
        with self.assertRaisesRegex(RuntimeError, '3 seconds'):
            progress.accept(11, 21, 3.4)
        progress = FrameProgress(0)
        self.assertTrue(progress.accept(10, 20, .1))
        self.assertFalse(progress.accept(12, 20, .2))
        with self.assertRaisesRegex(RuntimeError, 'regressed'):
            progress.accept(11, 21, .3)

    def test_sigterm_graceful_shutdown(self):
        previous = signal.getsignal(signal.SIGTERM)
        def fake_capture(args, frames):
            signal.raise_signal(signal.SIGTERM)
            self.assertTrue(frames.stop_requested.is_set())
            frames.publish(status={'ready': True, 'recorded_frames': 7})
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)/'new'
            service=SimpleNamespace(frames={'base_0':Frames()},start_servers=lambda _:dict(head=18499,left=18500,right=18501))
            with patch('sys.argv', ['real_rgbd', '--serial', '123', '--camera-name', 'head',
                                    '--port', '18499', '--output', str(output),'--service-descriptor','unused.json']), \
                    patch('robot_workbench.rgbd_service.CommonCameraService.connect',return_value=service), \
                    patch('rule_based_workbench.real_rgbd.capture', fake_capture):
                main()
            final = json.loads((output/'status.json').read_text())
            self.assertEqual(final['stop_signal'], signal.SIGTERM)
            self.assertEqual(final['recorded_frames'], 7)
            self.assertFalse(final['ready'])
        self.assertEqual(signal.getsignal(signal.SIGTERM), previous)

    def test_atomic_pair_and_http(self):
        values = dict(serial='123', camera_name='head', color_number=17, depth_number=19,
                      color_ms=100, depth_ms=103, color_domain='hardware_clock',
                      depth_domain='hardware_clock', host_receive_ns=time.time_ns(),
                      depth_scale_m=.001, intrinsics={'fx': 600})
        metadata = pair_metadata(**values)
        for update in ({'depth_ms': 200}, {'depth_domain': 'system_time'}, {'depth_scale_m': 0}):
            with self.assertRaises(ValueError):
                pair_metadata(**{**values, **update})
        color, depth = b'lossless-color-png', b'uint16-depth-png'
        with zipfile.ZipFile(io.BytesIO(snapshot_bundle(metadata, color, depth))) as archive:
            self.assertEqual(archive.read('depth.png'), depth)
            self.assertEqual(json.loads(archive.read('metadata.json'))['pair_id'], metadata['pair_id'])
        frames = Frames()
        server = ThreadingHTTPServer(('127.0.0.1', 0), make_handler(frames))
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        base = f'http://127.0.0.1:{server.server_port}'
        try:
            with self.assertRaises(HTTPError) as error:
                urlopen(base+'/frame.jpg')
            self.assertEqual(error.exception.code, 503)
            frames.publish(status=metadata, jpeg=b'\xff\xd8test\xff\xd9', color_png=color, depth_png=depth)
            with urlopen(base+'/snapshot.zip') as response:
                header = json.loads(response.headers['X-Camera-Metadata'])
                with zipfile.ZipFile(io.BytesIO(response.read())) as archive:
                    self.assertEqual(header, json.loads(archive.read('metadata.json')))
                    self.assertEqual(archive.read('color.png'), color)
            with urlopen(base+'/raw-frame.jpg') as response:
                status = json.loads(response.headers['X-Camera-Metadata'])
                self.assertTrue(status['raw'])
                self.assertFalse(status['annotated'])
            frames.publish(status={**metadata, 'host_receive_ns': str(time.time_ns()-3_000_000_000)}, jpeg=b'x')
            with self.assertRaises(HTTPError):
                urlopen(base+'/depth.png')
        finally:
            server.shutdown()
            server.server_close()
            thread.join()


if __name__ == '__main__':
    unittest.main()
