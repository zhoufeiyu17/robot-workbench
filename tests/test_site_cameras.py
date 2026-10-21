"""Real CPU HTTP tests for multi-device camera isolation, persistence and failures."""
from http.client import HTTPConnection
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import threading
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'scripts')]
import serve_lan_gateway as gateway
from serve_site_camera_bridge import serve as camera_bridge
from robot_workbench.site_cameras import CameraDirectory, inject_tab


class CameraTest(unittest.TestCase):
    def setUp(self):
        self.temp = TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.config = Path(self.temp.name) / 'cameras.json'
        self.calls = []
        self.cameras = []
        for n in (1, 2):
            calls = self.calls
            class Camera(BaseHTTPRequestHandler):
                def log_message(self, *_): pass
                def do_GET(self):
                    calls.append((self.server.number, self.path))
                    if self.path == '/streams':
                        data = json.dumps(dict(streams=[dict(key=k, status='live', last_frame_age_s=.1, fps=8.)
                            for k in ('base_0_rgb', 'left_wrist_0_rgb', 'right_wrist_0_rgb')])).encode(); kind = 'application/json'
                    elif self.path.startswith('/frame/'):
                        data = b'\xff\xd8camera-' + str(self.server.number).encode() + b'\xff\xd9'; kind = 'image/jpeg'
                    else:
                        data = b'<!doctype html><html><body><nav class="tabs"><button data-tab="calibration">C</button></nav></body></html>'; kind = 'text/html'
                    self.send_response(200); self.send_header('Content-Type', kind); self.send_header('Content-Length', str(len(data))); self.end_headers(); self.wfile.write(data)
            server = ThreadingHTTPServer(('127.0.0.1', 0), Camera); server.daemon_threads = True; server.number = n
            thread = threading.Thread(target=server.serve_forever, daemon=True); thread.start(); self.cameras.append(server)
            self.addCleanup(server.server_close); self.addCleanup(server.shutdown)
        self.config.write_text(json.dumps({'version': 1, 'devices': [dict(id='arx1', label='ARX 1', address='127.0.0.1', port=self.cameras[0].server_port)]}))
        self.server = gateway.serve(listen='127.0.0.1', port=0, upstream_port=self.cameras[0].server_port,
            camera_config=self.config, camera_assets=ROOT / 'web')
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.addCleanup(self.server.server_close); self.addCleanup(self.server.shutdown)
        self.authority = self.server.public_authority

    def request(self, path='/api/site-cameras', body=None, *, token=None, origin=None):
        connection = HTTPConnection('127.0.0.1', self.server.server_port, timeout=5)
        headers = {'Host': self.authority}
        if token: headers['X-Site-Camera-Token'] = token
        if origin: headers['Origin'] = origin
        if body is not None: headers['Content-Type'] = 'application/json'
        connection.request('POST' if body is not None else 'GET', path, None if body is None else json.dumps(body), headers)
        r = connection.getresponse(); result = (r.status, dict(r.headers), r.read()); connection.close(); return result

    def test_two_devices_cannot_mix_frames_and_configuration_survives_restart(self):
        code, _, raw = self.request(); self.assertEqual(code, 200); first = json.loads(raw); source1 = first['devices'][0]['source']
        body = {'operation': 'save', 'device': dict(id='arx2', label='ARX 2', address='127.0.0.1', port=self.cameras[1].server_port)}
        code, _, raw = self.request(body=body, token=first['csrf']); self.assertEqual(code, 200); second = json.loads(raw)['devices'][1]
        for ident, source, expected in [('arx1', source1, b'camera-1'), ('arx2', second['source'], b'camera-2')]:
            status = json.loads(self.request('/api/site-cameras/' + ident + '/streams')[2]); self.assertTrue(status['reachable'])
            code, headers, data = self.request('/api/site-cameras/' + ident + '/frame/base_0_rgb.jpg?source=' + source)
            self.assertEqual(code, 200); self.assertIn(expected, data); self.assertEqual(headers['X-Camera-Source'], source)
        self.assertEqual(self.request('/api/site-cameras/arx2/frame/base_0_rgb.jpg?source=' + source1)[0], 409)
        restored = CameraDirectory(self.config, ROOT / 'web', allow_loopback=True)
        self.assertEqual([r['id'] for r in restored.devices], ['arx1', 'arx2'])

    def test_no_arbitrary_proxy_controls_or_unauthorized_configuration(self):
        first = json.loads(self.request()[2]); body = dict(operation='save', device=dict(label='test', address='127.0.0.1', port=8081))
        self.assertEqual(self.request(body=body)[0], 403)
        self.assertEqual(self.request(body=body, token=first['csrf'], origin='http://evil.test')[0], 403)
        for address in ('169.254.169.254', '8.8.8.8', 'example.com', 'http://127.0.0.1'):
            body['device']['address'] = address
            self.assertEqual(self.request(body=body, token=first['csrf'])[0], 400)
        before = len(self.calls)
        self.assertEqual(self.request('/api/site-cameras/arx1/operator', body={'action': 'stop'})[0], 405)
        self.assertEqual(self.request('/api/site-cameras/arx1/frame/unknown.jpg?source=x')[0], 404)
        self.assertEqual(len(self.calls), before)

    def test_offline_is_explicit_and_monitor_works_without_workbench_upstream(self):
        self.cameras[0].shutdown(); self.cameras[0].server_close()
        status = json.loads(self.request('/api/site-cameras/arx1/streams')[2]); self.assertFalse(status['reachable']); self.assertIn('不可用', status['error'])
        self.assertEqual(self.request('/site-cameras')[0], 200)
        self.assertEqual(self.request('/web/site_cameras.js')[0], 200)
        self.assertEqual(self.request()[0], 200)

    def test_tab_is_injected_once_and_other_workbench_routes_still_work(self):
        code, _, html = self.request('/'); self.assertEqual(code, 200); self.assertIn('现场相机'.encode(), html)
        self.assertEqual(inject_tab(html), html)
        self.assertEqual(self.request('/other-existing-route')[0], 200)

    def test_source_bridge_only_reads_existing_camera_service(self):
        server = camera_bridge('127.0.0.1', 0, self.cameras[0].server_port)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        self.addCleanup(server.server_close); self.addCleanup(server.shutdown)
        client = HTTPConnection('127.0.0.1', server.server_port, timeout=3)
        client.request('GET', '/streams'); r = client.getresponse(); self.assertEqual(r.status, 200); self.assertTrue(json.loads(r.read())['readonly'])
        client.close(); client = HTTPConnection('127.0.0.1', server.server_port, timeout=3)
        before = len(self.calls); client.request('POST', '/operator', b'{}'); r = client.getresponse(); self.assertEqual(r.status, 405); r.read(); client.close()
        self.assertEqual(len(self.calls), before)

    def test_source_stopped_is_distinct_from_network_failure(self):
        owner = Path(self.temp.name) / 'owner.json'
        import time
        owner.write_text(json.dumps(dict(state='stopped', heartbeat=time.monotonic())))
        server = camera_bridge('127.0.0.1', 0, self.cameras[0].server_port, owner)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        self.addCleanup(server.server_close); self.addCleanup(server.shutdown)
        first = json.loads(self.request()[2])
        body = dict(operation='save', device=dict(id='arx1', label='ARX 1', address='127.0.0.1', port=server.server_port))
        self.assertEqual(self.request(body=body, token=first['csrf'])[0], 200)
        status = json.loads(self.request('/api/site-cameras/arx1/streams')[2])
        self.assertTrue(status['reachable'])
        self.assertEqual({r['source_state'] for r in status['streams']}, {'stopped'})
        server.shutdown(); server.server_close()
        status = json.loads(self.request('/api/site-cameras/arx1/streams')[2])
        self.assertFalse(status['reachable'])

    def test_start_requires_current_device_source_csrf_and_only_calls_camera_lifecycle(self):
        first=json.loads(self.request()[2]);source=first['devices'][0]['source']
        path='/api/site-cameras/arx1/start';body={'source':source}
        with patch('robot_workbench.site_cameras.camera_lifecycle',return_value=dict(available=True,phase='starting')) as start:
            self.assertEqual(self.request(path,body=body)[0],403)
            self.assertEqual(self.request(path,body={'source':'old-address'},token=first['csrf'])[0],409)
            self.assertEqual(self.request(path,body=body,token=first['csrf'],origin='http://other.test')[0],403)
            self.assertEqual(self.request(path,body={**body,'command':'connect'},token=first['csrf'])[0],409)
            self.assertFalse(start.called)
            self.assertEqual(self.request(path,body=body,token=first['csrf'])[0],202)
            start.assert_called_once_with('127.0.0.1',self.cameras[0].server_port,start=True)


if __name__ == '__main__': unittest.main()
