"""Harmless real CPU subprocess lifecycle and same-origin proxy regression."""
from http.client import HTTPConnection
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from robot_workbench.fake_session import FakeSession, serve_session, stop_owned, PublicSessionError

CHILD = r'''
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
import json, pathlib, sys, os
class Handler(BaseHTTPRequestHandler):
    protocol_version='HTTP/1.1'
    def log_message(self,*args): pass
    def do_GET(self):
        raw=json.dumps(dict(csrf='backend-token',host=self.headers['Host'],path=self.path,server_instance_id='cpu-backend',device=dict(id='cpu-fixture',environment='fake'))).encode()
        self.send_response(200)
        if 'unframed=1' in self.path:
            self.close_connection=True
        else:
            self.send_header('Content-Length',str(len(raw)))
        self.end_headers(); self.wfile.write(raw)
    def do_POST(self):
        raw=self.rfile.read(int(self.headers['Content-Length']))
        self.send_response(200 if self.headers.get('X-CSRF-Token')=='backend-token' else 403)
        self.send_header('Content-Length',str(len(raw)))
        self.end_headers(); self.wfile.write(raw)
server=ThreadingHTTPServer(('127.0.0.1',0), Handler)
pathlib.Path(sys.argv[1]).write_text(json.dumps(dict(pid=os.getpid(),ui_port=server.server_port,environment='fake')))
server.serve_forever()
'''


class FakeSessionTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name); self.calls = []
        for name in ('session.html', 'session.js', 'style.css'):
            (self.root/name).write_text('offline management')
        def prepare(generation, remaining_s):
            self.calls.append((generation, remaining_s))
            run = self.root/str(generation); run.mkdir()
            return dict(argv=[sys.executable, '-u', '-c', CHILD, str(run/'service.json')],
                cwd=str(ROOT), run_path=str(run), service_path=str(run/'service.json'), log_path=str(run/'child.log'))
        self.prepare = prepare
        self.manager = FakeSession(prepare, state_dir=self.root/'state', deadline_wall_ns=time.time_ns()+int(30e9), grace_s=.2)
        self.server = serve_session(self.manager, web_dir=self.root, browser_ports=[18866])
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True); self.thread.start()
        self.addCleanup(self.cleanup)
    def cleanup(self):
        self.manager.close(); self.server.shutdown(); self.server.server_close(); self.thread.join(1)
    def http(self, path, body=None, **headers):
        connection = HTTPConnection('127.0.0.1', self.server.server_port, timeout=3)
        raw = None if body is None else json.dumps(body).encode()
        headers.setdefault('Host', '127.0.0.1:18866')
        if raw is not None:
            headers.setdefault('Origin', 'http://127.0.0.1:18866'); headers.setdefault('X-CSRF-Token', self.manager.csrf)
        connection.request('GET' if body is None else 'POST', path, raw, headers)
        response = connection.getresponse(); code = response.status; raw = response.read(); connection.close()
        return code, json.loads(raw) if raw.startswith(b'{') else raw
    def body(self, ident, **kwargs):
        return dict(manager_instance_id=self.manager.manager_instance_id, generation=self.manager.generation,
                    request_id=ident, **kwargs)
    def wait_status(self, status):
        until = time.monotonic()+6
        while time.monotonic() < until:
            value = self.manager.snapshot()
            if value['status'] == status:
                return value
            time.sleep(.025)
        self.fail(str(value))
    def start(self, ident='start'):
        body = self.body(ident); begin = time.monotonic()
        self.assertEqual(self.http('/api/session/start', body)[0], 202)
        self.assertLess(time.monotonic()-begin, .5)
        return body, self.wait_status('ready')
    def test_start_proxy_stop_restart_and_stale_generation(self):
        self.assertEqual(self.http('/')[1], b'offline management')
        self.assertEqual(self.http('/api/state')[0], 503)
        start, ready = self.start()
        self.assertEqual(self.http('/api/session/start', start)[0], 202)
        self.assertEqual(len(self.calls), 1)
        code, state = self.http('/api/state?environment=fake&business=collection')
        self.assertEqual(code, 200); self.assertEqual(state['host'], '127.0.0.1:18866')
        code, _ = self.http('/api/actions/stop', {'request_id': 'business'}, **{'X-CSRF-Token': 'backend-token'})
        self.assertEqual(code, 200)
        self.assertEqual(self.http('/api/actions/stop', {'request_id': 'wrong'})[0], 403)
        stop = self.body('stop', confirm=True)
        self.assertEqual(self.http('/api/session/stop', stop)[0], 202)
        self.wait_status('stopped'); self.assertEqual(self.http('/api/session')[0], 200)
        self.assertTrue(json.loads((self.root/'state/cleanup-1.json').read_text())['all_owned_exited'])
        self.start('again')
        self.assertEqual(self.http('/api/session/stop', {**stop, 'request_id': 'stale-stop'})[0], 409)
        self.assertEqual(self.manager.snapshot()['status'], 'ready')
    def test_proxy_keeps_one_socket_for_state_post_state_but_closes_unframed_or_rejected(self):
        self.start()
        connection = HTTPConnection('127.0.0.1', self.server.server_port, timeout=3)
        self.addCleanup(connection.close)
        headers = {'Host': '127.0.0.1:18866', 'Origin': 'http://127.0.0.1:18866', 'X-CSRF-Token': 'backend-token'}
        sock = None
        for method, path, body in [('GET', '/api/state', None), ('GET', '/api/state', None),
                                  ('POST', '/api/actions/stop', b'{"request_id":"business"}'),
                                  ('GET', '/api/state', None)]:
            connection.request(method, path, body, headers)
            response = connection.getresponse(); self.assertEqual(response.status, 200)
            self.assertFalse(response.will_close); self.assertTrue(response.read())
            if sock is None: sock = connection.sock
            self.assertIs(connection.sock, sock)
        connection.request('GET', '/api/state?unframed=1', headers=headers)
        response = connection.getresponse(); self.assertTrue(response.will_close); response.read()
        self.assertIsNone(connection.sock)
        connection.request('POST', '/api/actions/stop', b'{}', {**headers, 'X-CSRF-Token': 'wrong'})
        response = connection.getresponse(); self.assertEqual(response.status, 403)
        self.assertTrue(response.will_close); response.read(); self.assertIsNone(connection.sock)
        connection.request('GET', '/api/state', headers={**headers, 'Connection': 'close'})
        response = connection.getresponse(); self.assertTrue(response.will_close); response.read()

    def test_origin_token_instance_generation_and_body_boundaries(self):
        body = self.body('go')
        for headers in ({'Host': 'evil:18866'}, {'Origin': 'http://evil:18866'}, {'X-CSRF-Token': 'stale'}):
            self.assertEqual(self.http('/api/session/start', body, **headers)[0], 403)
        for change in ({'manager_instance_id': 'old'}, {'generation': True}, {'environment': 'physical'}, {'argv': ['bad']}):
            self.assertEqual(self.http('/api/session/start', {**body, **change})[0], 409)
        self.assertEqual(self.http('http://evil.example/api/state')[0], 404)
        self.assertEqual(self.http('/api/session/stop', body)[0], 409)
        self.assertEqual(self.http('/api/session/start', body, **{'Transfer-Encoding': 'chunked'})[0], 409)
        self.assertEqual(self.calls, [])
    def test_deadline_stops_child_but_keeps_management_entry(self):
        self.start(); self.manager.deadline_wall_ns = time.time_ns()+int(.2e9)
        self.wait_status('expired')
        until = time.monotonic()+5
        while self.manager.worker is not None and time.monotonic() < until:
            time.sleep(.03)
        self.assertIsNone(self.manager.worker)
        self.assertEqual(self.http('/api/session')[1]['status'], 'expired')
        self.assertTrue(self.http('/api/session')[1]['cleanup_verified'])
        self.assertEqual(self.http('/')[1], b'offline management')
        self.assertEqual(self.http('/api/session/start', self.body('late'))[0], 409)
    def test_cancel_preparation_and_failure_allow_no_late_spawn(self):
        entered = threading.Event(); release = threading.Event()
        def slow(generation, remaining):
            entered.set(); release.wait(2); return self.prepare(generation, remaining)
        self.manager.prepare = slow
        self.http('/api/session/start', self.body('slow')); self.assertTrue(entered.wait(1))
        self.http('/api/session/stop', self.body('stop', confirm=True)); release.set()
        self.wait_status('stopped'); self.assertFalse((self.root/'1/service.json').exists())
        def fail(*args):
            raise RuntimeError('private-api-token-must-not-leak')
        self.manager.prepare = fail
        self.http('/api/session/start', self.body('fail')); failed = self.wait_status('failed')
        self.assertNotIn('private-api-token', json.dumps(failed))
        self.assertTrue(failed['allowed']['start']['allowed'])
        self.assertIn('private-api-token', (self.root/'state/manager-errors.log').read_text())
    def test_full_profile_requires_explicit_admission_and_physical_rejected(self):
        with self.assertRaises(ValueError):
            FakeSession(self.prepare, state_dir=self.root/'bad', deadline_wall_ns=time.time_ns(), profile='physical')
        self.manager.profile = 'full_fake'
        self.assertFalse(self.manager.snapshot()['allowed']['start']['allowed'])
        self.assertEqual(self.http('/api/session/start', self.body('no-guard'))[0], 409)
        self.manager.start_guard = lambda generation, remaining: False
        self.http('/api/session/start', self.body('denied')); self.wait_status('failed')
        self.assertEqual(self.calls, [])
    def test_fake_state_health_and_owned_service_pid_are_required(self):
        self.start()
        service = self.root/'1/service.json'; value = json.loads(service.read_text())
        value['pid'] = 1; service.write_text(json.dumps(value))
        self.wait_status('failed')
        self.assertEqual(self.http('/api/state')[0], 503)
        value['pid'] = self.manager.child.pid; service.write_text(json.dumps(value))
        self.wait_status('ready')
        self.assertEqual(self.http('/api/state')[0], 200)
        original = self.manager._health
        with patch('robot_workbench.fake_session.HTTPConnection') as connection:
            response = connection.return_value.getresponse.return_value; response.status = 200
            for state in ({}, {'server_instance_id': 'old', 'device': {'environment': 'physical'}},
                          {'server_instance_id': '', 'device': {'environment': 'fake'}}):
                response.read.return_value = json.dumps(state).encode()
                self.assertFalse(original(1234))

    def test_slow_health_never_blocks_stop_and_guard_failure_records_cleanup(self):
        entered = threading.Event(); release = threading.Event()
        def slow_health(port):
            entered.set(); release.wait(2); return 'old-ready'
        self.manager._health = slow_health
        self.http('/api/session/start', self.body('slow-health')); self.assertTrue(entered.wait(3))
        begin = time.monotonic(); self.http('/api/session/stop', self.body('stop', confirm=True))
        self.assertLess(time.monotonic()-begin, .5); release.set(); self.wait_status('stopped')
        callbacks = []; self.manager.on_stopped = lambda spec, receipt: callbacks.append((spec, receipt))
        self.manager.start_guard = lambda *args: False
        self.http('/api/session/start', self.body('guard-fail')); self.wait_status('failed')
        self.assertEqual(callbacks[0][0], None); self.assertTrue(callbacks[0][1]['all_owned_exited'])

    def test_pid_reuse_never_signals_unknown_process(self):
        child = unittest.mock.Mock(pid=456); child.poll.return_value=0
        with patch('robot_workbench.fake_session.process_table', return_value={456: ('new-identity', 1, 456, 'S')}), \
             patch('robot_workbench.fake_session.os.kill') as kill, \
             patch('robot_workbench.fake_session.group_alive', return_value=True):
            receipt = stop_owned(child, {456: 'original-identity'}, .01)
        kill.assert_not_called(); self.assertFalse(receipt['all_owned_exited'])

    def test_only_reviewed_failure_messages_are_public(self):
        def refused(*args):raise PublicSessionError('GPU仍被占用，请等待资源核验')
        self.manager.start_guard=refused
        self.http('/api/session/start',self.body('public'))
        self.assertEqual(self.wait_status('failed')['error'],'GPU仍被占用，请等待资源核验')
        def private(*args):raise OSError('private command environment must not appear')
        self.manager.start_guard=private
        self.http('/api/session/start',self.body('private'))
        self.assertNotIn('private command',self.wait_status('failed')['error'])


if __name__ == '__main__':
    unittest.main()
