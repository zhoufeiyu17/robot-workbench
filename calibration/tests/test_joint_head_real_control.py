"""Local mock owner only. Never accesses a robot or SDK."""
import importlib.util
import json
import shutil
import subprocess
from pathlib import Path
import threading
import time
import unittest
from unittest.mock import patch, MagicMock
import tempfile
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.error import HTTPError
from urllib.request import Request, urlopen

path = Path(__file__).resolve().parents[1] / 'examples/scripts/debug/joint_head_real_control.py'
spec = importlib.util.spec_from_file_location('real_control', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class ControlTest(unittest.TestCase):
    def test_explicit_acquisition_only_and_duplicate_guard(self):
        with tempfile.TemporaryDirectory() as tmp:
            config = dict(argv=['/usr/bin/python3', 'owner.py', '--output', '{output}'], cwd=tmp,
                          log_dir=tmp, process_names=['joint_head_real_owner.py'])
            bridge = module.Bridge(start_config=config)
            with patch.object(module.subprocess, 'Popen') as launch:
                self.assertIsNone(bridge.child)
                with self.assertRaises(ValueError): bridge.command('/api/acquire', {})
                launch.assert_not_called()
                proc = MagicMock(); proc.poll.return_value = None; proc.pid = 2468
                launch.return_value = proc
                with patch.object(module.socket, 'socket') as sock, patch.object(module.Path, 'is_dir', return_value=True), patch.object(module.Path, 'glob', return_value=[]):
                    sock.return_value.__enter__.return_value.connect_ex.return_value = 1
                    result = bridge.command('/api/acquire', {'confirmation':'ARMS_SUPPORTED_CONNECT'})
                self.assertEqual(result['starting_pid'], 2468)
                self.assertNotIn('{output}', launch.call_args.args[0][-1])
                with self.assertRaises(ValueError): bridge.command('/api/acquire', {'confirmation':'ARMS_SUPPORTED_CONNECT'})
                self.assertEqual(launch.call_count, 1)

    @unittest.skipUnless(shutil.which('node'), 'Node required for browser-script check')
    def test_browser_review_invalidation_and_both_home_summary(self):
        script = path.with_suffix('.html').read_text().split('<script>', 1)[1].split('</script>', 1)[0]
        script = script.replace('poll();imageTick();setInterval(imageTick,1200);setInterval(stale,400);', '')
        check = r"""
const vm=require('node:vm'),assert=require('node:assert/strict');
const elements=new Map();
const context=vm.createContext({Date,console,document:{getElementById(id){if(!elements.has(id))elements.set(id,{disabled:false,checked:false,textContent:'',close(){}});return elements.get(id);}},setTimeout(){}});
vm.runInContext(SCRIPT,context);
vm.runInContext(`
currentState={motion_enabled:true,owner:{pid:123,command_epoch:7}};
lastSeen=Date.now();motionToken='review';motionReview={pid:123,epoch:7,expires:Date.now()+30000};
`,context);
assert.equal(vm.runInContext('reviewValid()',context),true);
vm.runInContext('currentState.owner.command_epoch++;stale()',context);
assert.equal(vm.runInContext('motionToken',context),null);
assert.equal(elements.get('motionExecute').disabled,true);
vm.runInContext(`motionToken='review';motionReview={pid:123,epoch:8,expires:Date.now()+30000};lastSeen=Date.now()-2000;stale()`,context);
assert.equal(vm.runInContext('motionToken',context),null);
assert.equal(elements.get('bothHomePreview').disabled,true);
vm.runInContext(`motionToken='review';motionReview={pid:123,epoch:8,expires:Date.now()-1};lastSeen=Date.now();stale()`,context);
assert.equal(vm.runInContext('motionToken',context),null);
const summary=vm.runInContext(`motionSummary({plan:{active_arm:'both',kind:'both_home',duration_s:10,max_velocity:.2,max_jaw_velocity:.005,q_start:Array(14).fill(0),q_target:Array(14).fill(.082)}})`,context);
assert.match(summary,/左右臂＋夹爪同时回零/);assert.match(summary,/任一故障停止全部/);
assert.match(summary,/10.0 秒/);assert.match(summary,/左爪 SDK 开度/);assert.match(summary,/右爪 SDK 开度/);assert.match(summary,/82.0 mm/);
""".replace('SCRIPT', json.dumps(script))
        subprocess.run([shutil.which('node'), '-e', check], check=True, capture_output=True, text=True)

    def test_mock_owner_and_browser_boundaries(self):
        received = []
        state = dict(pid=1234, mode='hardware', state='FAULT_STOP_UNCONFIRMED', command_epoch=0,
                     robot={'qpos': [0.] * 14},
                     calibration_feedback=dict(qpos=[0.] * 13 + [-.075], qvel=[0.] * 14,
                                               sdk_timestamps=[1., 1.], sample_wall_ns=time.time_ns()))
        class MockOwner(BaseHTTPRequestHandler):
            def log_message(self, *_):
                pass
            def do_GET(self):
                self.send_response(200)
                self.end_headers()
                self.wfile.write(json.dumps(state).encode())
            def do_POST(self):
                received.append(json.loads(self.rfile.read(int(self.headers['Content-Length']))))
                self.send_response(202)
                self.end_headers()
                self.wfile.write(b'{"queued":true}')
        owner = ThreadingHTTPServer(('127.0.0.1', 0), MockOwner)
        threading.Thread(target=owner.serve_forever, daemon=True).start()
        app = module.Bridge(owner.server_port)
        server = ThreadingHTTPServer(('127.0.0.1', 0), module.handler(app))
        threading.Thread(target=server.serve_forever, daemon=True).start()
        base = f'http://127.0.0.1:{server.server_port}'
        def request(path, body=None, **headers):
            data = None if body is None else json.dumps(body).encode()
            h = {'Origin': base, 'Content-Type': 'application/json', 'X-CSRF-Token': app.csrf}
            h.update(headers)
            try:
                with urlopen(Request(base + path, data, headers=h), timeout=2) as result:
                    return result.status, result.read()
            except HTTPError as exc:
                return exc.code, exc.read()
        try:
            code, raw = request('/api/state')
            data = json.loads(raw)
            self.assertEqual(code, 200)
            self.assertFalse(data['motion_enabled'])
            self.assertEqual(data['owner']['calibration_feedback']['qpos'][13], -.075)
            self.assertEqual(request('/api/state', Host='evil.test')[0], 403)
            self.assertEqual(request('/api/stop', {}, Origin='https://evil.test')[0], 403)
            self.assertEqual(request('/api/stop', {}, **{'X-CSRF-Token': 'wrong'})[0], 403)
            self.assertEqual(request('/api/home', {})[0], 400)
            self.assertEqual(request('/api/motion-preview', {'active_arm': 'left', 'q_target': [0.]*14})[0], 400)
            self.assertEqual(request('/api/home-preview', {'active_arm': 'left'})[0], 400)
            self.assertEqual(request('/api/both-home-preview', {})[0], 400)
            self.assertEqual(request('/api/stop', {'action': 'home'})[0], 400)
            self.assertEqual(received, [])
            state['motion_protocol'] = 'reviewed-single-arm-v1'
            state['feedback_healthy'] = True
            self.assertFalse(json.loads(request('/api/state')[1])['motion_enabled'])
            recover = dict(confirmation='ACKNOWLEDGE_MOTION_FAULT', owner_pid=1234, command_epoch=0)
            self.assertEqual(request('/api/recover', dict(recover, owner_pid=True))[0], 400)
            self.assertEqual(request('/api/recover', dict(recover, confirmation=''))[0], 400)
            self.assertEqual(request('/api/recover', recover)[0], 202)
            self.assertEqual(received.pop(), dict(action='recover', **recover))
            state['state'] = 'READY'
            state['calibration_feedback']['sample_wall_ns'] = time.time_ns()
            self.assertEqual(request('/api/motion-preview', {'active_arm': 'left', 'q_target': [0.]*14})[0], 202)
            self.assertEqual(received.pop()['action'], 'preview')
            self.assertEqual(request('/api/home-preview', {'active_arm': 'left'})[0], 202)
            self.assertEqual(received.pop(), {'action': 'preview_home', 'active_arm': 'left'})
            self.assertEqual(request('/api/home-preview', {'active_arm': 'both'})[0], 400)
            self.assertEqual(request('/api/both-home-preview', {})[0], 202)
            self.assertEqual(received.pop(), {'action': 'preview_both_home'})
            self.assertEqual(request('/api/both-home-preview', {'active_arm': 'both'})[0], 400)
            for route, action in [('arm-home', 'preview_arm_home'), ('jaw-probe', 'preview_jaw_probe')]:
                self.assertEqual(request('/api/'+route+'-preview', {'active_arm': 'right'})[0], 202)
                self.assertEqual(received.pop(), {'action': action, 'active_arm': 'right'})
            self.assertEqual(request('/api/motion-preview', {'active_arm': 'left', 'q_target': [True]*14})[0], 400)
            confirm = dict(token='owner-token', confirmation='START_REVIEWED_SINGLE_ARM', physical_clearance=True)
            self.assertEqual(request('/api/motion-confirm', dict(confirm, physical_clearance=False))[0], 400)
            self.assertEqual(request('/api/motion-confirm', confirm)[0], 202)
            self.assertEqual(received.pop()['action'], 'confirm')
            # Stop remains available despite fault or stale feedback.
            state['calibration_feedback']['sample_wall_ns'] = 1
            self.assertFalse(json.loads(request('/api/state')[1])['host_sample_recent'])
            self.assertEqual(request('/api/stop', {})[0], 202)
            self.assertEqual(received, [{'action': 'stop'}])
            p = json.loads(request('/api/release-preview', {})[1])
            body = dict(token=p['token'], confirmation='ARMS_SUPPORTED_RELEASE')
            state['pid'] = 1235
            self.assertEqual(request('/api/release-confirm', body)[0], 400)
            p = json.loads(request('/api/release-preview', {})[1])
            body = dict(token=p['token'], confirmation='ARMS_SUPPORTED_RELEASE')
            self.assertEqual(request('/api/release-confirm', body)[0], 202)
            self.assertEqual(request('/api/release-confirm', body)[0], 400)
            self.assertEqual(received[-1], {'action': 'release', 'confirmation': 'ARMS_SUPPORTED_RELEASE'})
            p = json.loads(request('/api/release-preview', {})[1])
            expiry, identity = app.pending[p['token']]
            app.pending[p['token']] = (0., identity)
            self.assertEqual(request('/api/release-confirm', dict(token=p['token'], confirmation='ARMS_SUPPORTED_RELEASE'))[0], 400)
            p = json.loads(request('/api/release-preview', {})[1])
            request('/api/stop', {})
            self.assertEqual(request('/api/release-confirm', dict(token=p['token'], confirmation='ARMS_SUPPORTED_RELEASE'))[0], 400)
            code, page = request('/')
            self.assertEqual(code, 200)
            self.assertIn('确认故障并重新开放预览', page.decode())
            self.assertIn('左右臂＋夹爪一键回零', page.decode())
            self.assertIn('最高 0.2 rad/s', page.decode())
            self.assertIn('同时驱动左右臂＋夹爪', page.decode())
            self.assertNotIn('依次执行', page.decode())
            self.assertNotIn('整臂含爪回零会被拒绝', page.decode())
        finally:
            server.shutdown()
            server.server_close()
            owner.shutdown()
            owner.server_close()
        offline = app.status()
        self.assertFalse(offline['reachable'])
        self.assertFalse(offline['motion_enabled'])


if __name__ == '__main__':
    unittest.main()
