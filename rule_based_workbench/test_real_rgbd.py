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
            with patch('sys.argv', ['real_rgbd', '--serial', '123', '--camera-name', 'head',
                                    '--port', '18499', '--output', str(output)]), \
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
