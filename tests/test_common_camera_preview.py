"""CPU HTTP checks for the shared camera monitor; no camera or robot opened."""
from http.client import HTTPConnection
import json
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from tempfile import TemporaryDirectory

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'scripts')]
from serve_common_camera_preview import SOURCES, serve
from robot_workbench.site_cameras import read_camera


class Frame:
    def __init__(self, name):
        self.calls = []
        self.error = None
        self.value = dict(jpeg=b'\xff\xd8' + name.encode() + b'\xff\xd9', status=dict(
            ready=True, host_receive_ns=time.time_ns(), capture_fps=30,
            source_time_ns=123456, frame_number=17, capture_instance_id=name,
            secret='must-not-leak'))

    def snapshot(self, formats):
        self.calls.append(formats)
        if self.error:
            raise self.error
        return self.value


class PreviewTest(unittest.TestCase):
    def setUp(self):
        self.client = SimpleNamespace(frames={name: Frame(name) for name in SOURCES.values()})
        self.server = serve('/unused/private-descriptor.json', port=0, client=self.client)
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.addCleanup(self.server.server_close)
        self.addCleanup(self.server.shutdown)

    def request(self, path='/streams', method='GET', headers=None):
        connection = HTTPConnection('127.0.0.1', self.server.server_port, timeout=3)
        connection.request(method, path, headers=headers or {})
        response = connection.getresponse()
        result = response.status, dict(response.headers), response.read()
        connection.close()
        return result

    def test_three_sources_preserve_pixels_and_timestamp_with_bridge_compatibility(self):
        code, _, raw = self.request()
        self.assertEqual(code, 200)
        status = json.loads(raw)
        self.assertTrue(status['readonly'])
        self.assertNotIn(b'must-not-leak', raw)
        for row in status['streams']:
            self.assertEqual(row['status'], 'live')
            self.assertEqual(row['source_time_ns'], 123456)
            frame = self.client.frames[SOURCES[row['key']]]
            self.assertEqual(row['host_receive_ns'], frame.value['status']['host_receive_ns'])
            data = read_camera('127.0.0.1', self.server.server_port, '/frame/' + row['key'] + '.jpg')
            self.assertEqual(data, frame.value['jpeg'])
            self.assertTrue(all(formats == ['jpeg'] for formats in frame.calls))
        self.assertEqual([r['status'] for r in read_camera('127.0.0.1', self.server.server_port, '/streams')], ['live'] * 3)

    def test_stale_initializing_and_source_errors_are_explicit_without_secret_leaks(self):
        head = self.client.frames['base_0']
        head.value['status']['host_receive_ns'] -= 5_000_000_000
        self.client.frames['left_wrist_0'].value = dict(status=dict(ready=False, phase='warming_up'))
        self.client.frames['right_wrist_0'].error = ValueError('secret=must-not-leak')
        raw = self.request()[2]
        self.assertNotIn(b'must-not-leak', raw)
        rows = json.loads(raw)['streams']
        self.assertGreaterEqual(rows[0]['last_frame_age_s'], 5)
        self.assertEqual(rows[0]['status'], 'stale')
        self.assertEqual(rows[1]['source_state'], 'starting')
        self.assertEqual(rows[2]['source_state'], 'error')
        for key in SOURCES:
            code, _, raw = self.request('/frame/' + key + '.jpg')
            self.assertEqual(code, 503)
            self.assertNotIn(b'must-not-leak', raw)
        head.value['status']['host_receive_ns'] = time.time_ns() + 10_000_000_000
        self.assertIsNone(json.loads(self.request()[2])['streams'][0]['last_frame_age_s'])
        self.assertEqual(self.request('/frame/base_0_rgb.jpg')[0], 503)

    def test_read_only_routes_host_and_descriptor_constraints(self):
        for method in ('POST', 'PUT', 'PATCH', 'DELETE'):
            self.assertEqual(self.request('/streams', method)[0], 405)
        for path in ('/operator', '/frame/unknown.jpg', '/camera-service.json'):
            self.assertEqual(self.request(path)[0], 404)
        self.assertEqual(self.request(headers={'Host': 'evil.example'})[0], 403)
        self.assertEqual(self.request('/streams?secret=x')[0], 400)
        self.assertEqual(self.request('http://evil.example/streams')[0], 403)
        with self.assertRaises(ValueError):
            serve('relative.json', client=self.client)
        with self.assertRaises(ValueError):
            serve('/private.json', listen='0.0.0.0', client=self.client)
        self.assertEqual(sum(len(frame.calls) for frame in self.client.frames.values()), 0)

    def test_camera_only_start_is_fixed_idempotent_and_keeps_no_source_monitor_online(self):
        with TemporaryDirectory() as root:
            server=serve(str(Path(root)/'not-started.json'),port=0,
                         source_unit='robot-workbench-rgbd.service',source_state=Path(root)/'state.json')
            threading.Thread(target=server.serve_forever,daemon=True).start()
            self.addCleanup(server.server_close);self.addCleanup(server.shutdown)
            def request(path,body=b'{}',headers=None):
                c=HTTPConnection('127.0.0.1',server.server_port,timeout=3)
                c.request('POST',path,body,headers or {'X-Camera-Start':'1'})
                r=c.getresponse();result=r.status,r.read();c.close();return result
            with patch('serve_common_camera_preview.subprocess.run',return_value=SimpleNamespace(returncode=0,stdout='inactive',stderr='')) as command:
                for _ in range(2):self.assertEqual(request('/start')[0],202)
                self.assertEqual(command.call_count,2)
                self.assertTrue(all(c.args[0]==['systemctl','--user','start','--no-block','robot-workbench-rgbd.service'] for c in command.call_args_list))
                for path in ('/operator','/start?unit=robot-workbench-physical.service'):
                    self.assertEqual(request(path)[0],405)
                self.assertEqual(request('/start',headers={'X-Camera-Start':'1','Origin':'http://other.test'})[0],403)
                self.assertEqual(request('/start',body=b'{"command":"start"}')[0],403)
                self.assertEqual(command.call_count,2)
            with self.assertRaises(ValueError):serve('/unused.json',source_unit='robot-workbench-physical.service')


if __name__ == '__main__':
    unittest.main()
