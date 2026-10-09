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

path = Path(__file__).resolve().parents[1] / 'infra/arx_env/joint_head_workbench_control.py'
spec = importlib.util.spec_from_file_location('real_control', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class ControlTest(unittest.TestCase):
    def test_conditional_stop_preserves_identity_for_atomic_owner_check(self):
        bridge=module.Bridge()
        with patch.object(bridge,'request') as request:
            bridge.command('/api/stop',{'owner_pid':123,'command_epoch':4})
            request.assert_called_with('/operator',{'action':'stop','owner_pid':123,'command_epoch':4})
            bridge.command('/api/stop',{})
            request.assert_called_with('/operator',{'action':'stop'})
            with self.assertRaises(ValueError):bridge.command('/api/stop',{'owner_pid':123})
            self.assertEqual(request.call_count,2)
    def test_capture_window_ack_routes_exact_timestamp(self):
        bridge=module.Bridge()
        body={'owner_pid':123,'command_epoch':1,'sample_id':'frame','capture_ready_wall_ns':123456}
        with patch.object(bridge,'request',return_value={'queued':'capture_ack'}) as request:
            bridge.command('/api/capture-ack',body)
            request.assert_called_once_with('/operator',{'action':'capture_ack',**body})
            for value in (0,-1,True,'123'):
                with self.assertRaises(ValueError):bridge.command('/api/capture-ack',{**body,'capture_ready_wall_ns':value})

    def test_capture_ack_routes_without_changing_motion_confirmation(self):
        bridge=module.Bridge();body={'owner_pid':123,'command_epoch':4,'sample_id':'frame-0'}
        with patch.object(bridge,'request',return_value={'queued':'capture_ack'}) as request:
            self.assertEqual(bridge.command('/api/capture-ack',body),{'queued':'capture_ack'})
            request.assert_called_once_with('/operator',{'action':'capture_ack',**body})
            for bad in ({**body,'owner_pid':True},{**body,'sample_id':''},{**body,'extra':1}):
                with self.assertRaises(ValueError):bridge.command('/api/capture-ack',bad)
            self.assertEqual(request.call_count,1)
        body={'waypoints':[{'q':[0.]*12,'speeds':[.1]*12,'capture':True,'sample_id':'frame-0'}]}
        status={'motion_enabled':True,'owner':{'capabilities':['workbench_trajectory_v1']}}
        with patch.object(bridge,'status',return_value=status),patch.object(bridge,'request') as request:
            with self.assertRaisesRegex(ValueError,'capture ACK'):bridge.command('/api/trajectory-preview',body)
            request.assert_not_called()
            status['owner']['capabilities'].append('calibration_capture_ack_v1')
            bridge.command('/api/trajectory-preview',body)
            request.assert_called_once_with('/operator',{'action':'preview_workbench',**body})

    def test_controller_scan_excludes_filename_mentions_and_binds_starttime(self):
        bridge=module.Bridge(start_config={'process_names':['owner.py']})
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            for pid,cmd in ((111,b'python3\0-s\0/x/owner.py\0'),(112,b'cat\0/x/owner.py\0'),(113,b'python3\0-c\0print(1)\0/x/owner.py\0')):
                proc=root/str(pid);proc.mkdir()
                (proc/'cmdline').write_bytes(cmd)
                (proc/'stat').write_text(str(pid)+' (python) '+' '.join(['S']+['0']*18+['23456']))
            with patch.object(module.Path,'is_dir',return_value=True), patch.object(module.Path,'glob',return_value=list(root.iterdir())):
                found=bridge.controller_processes()
            self.assertEqual([p['pid'] for p in found],[111])
            self.assertEqual(found[0]['starttime'],'23456')

    def test_takeover_confirmation_is_single_use_and_binds_process_snapshot(self):
        bridge = module.Bridge(start_config={'process_names': ['owner.py']})
        processes = [dict(pid=123, starttime='321', uid=module.os.getuid(), name='owner.py')]
        with patch.object(bridge, 'controller_processes', return_value=processes), patch.object(bridge, 'stop_reviewed_controllers') as stop, patch.object(bridge, '_acquire_ready', return_value={'status':'INITIALIZING'}) as acquire:
            preview = bridge.command('/api/takeover-preview', {})
            body = dict(token=preview['token'], confirmation='TAKE_OVER_CONTROL')
            with patch.object(bridge, 'controller_processes', side_effect=[processes, []]):
                self.assertEqual(bridge.command('/api/takeover-confirm', body)['status'], 'INITIALIZING')
            stop.assert_called_once_with(processes)
            acquire.assert_called_once_with({'confirmation':'ARMS_SUPPORTED_CONNECT'})
            with self.assertRaises(ValueError):bridge.command('/api/takeover-confirm', body)
            preview = bridge.command('/api/takeover-preview', {})
            changed = [dict(processes[0], starttime='999')]
            with patch.object(bridge, 'controller_processes', return_value=changed):
                with self.assertRaisesRegex(ValueError, 'changed'):
                    bridge.command('/api/takeover-confirm', dict(body, token=preview['token']))
            self.assertEqual(stop.call_count, 1)
            preview = bridge.command('/api/takeover-preview', {})
            bridge.takeover_pending[preview['token']] = (0, processes)
            with self.assertRaisesRegex(ValueError, 'expired'):
                bridge.command('/api/takeover-confirm', dict(body, token=preview['token']))
            self.assertEqual(acquire.call_count, 1)

    def test_takeover_signals_bound_handles_and_escalates_only_survivors(self):
        bridge = module.Bridge()
        processes = [dict(pid=123), dict(pid=124)]
        with patch.object(module.os, 'pidfd_open', side_effect=[11,12], create=True), patch.object(module.signal, 'pidfd_send_signal', create=True) as send, patch.object(module.os, 'close') as close, patch.object(bridge, 'controller_processes', return_value=processes), patch.object(module.select, 'select', side_effect=[([11],[],[]),([12],[],[])]), patch.object(module.time, 'monotonic', side_effect=[0,0,0,3,3,3,3]):
            bridge.stop_reviewed_controllers(processes)
        self.assertEqual(send.call_args_list, [unittest.mock.call(11,module.signal.SIGTERM),unittest.mock.call(12,module.signal.SIGTERM),unittest.mock.call(12,module.signal.SIGKILL)])
        self.assertEqual(close.call_count, 2)

    def test_connect_reuses_owner_after_camera_ready(self):
        bridge=module.Bridge(start_config={'camera':{'port':8114}})
        with patch.object(bridge,'camera_status',return_value={'ready':True}), patch.object(bridge,'status',return_value={'reachable':True,'owner':{'state':'READY'}}), patch.object(bridge,'acquire_owner') as start:
            result=bridge.acquire({'confirmation':'ARMS_SUPPORTED_CONNECT'})
            self.assertEqual(result['status'],'EXISTING_OWNER')
            start.assert_not_called()
        with self.assertRaises(ValueError):bridge.acquire({})

    def test_camera_autostarts_before_owner(self):
        with tempfile.TemporaryDirectory() as tmp:
            bridge=module.Bridge(start_config={'camera':{'port':8114,'argv':['camera-python','camera.py','--output','{output}']},'cwd':tmp,'log_dir':tmp})
            proc=MagicMock();proc.poll.return_value=None
            with patch.object(bridge,'camera_status',side_effect=[{'ready':False},{'ready':True}]), patch.object(module.socket,'socket') as sock, patch.object(module.subprocess,'Popen',return_value=proc) as launch, patch.object(bridge,'status',return_value={'reachable':False}), patch.object(bridge,'acquire_owner',return_value={'status':'INITIALIZING'}) as owner:
                sock.return_value.__enter__.return_value.connect_ex.return_value=1
                self.assertEqual(bridge.acquire({'confirmation':'ARMS_SUPPORTED_CONNECT'})['status'],'INITIALIZING')
                self.assertEqual(launch.call_args.args[0][:2],['camera-python','camera.py'])
                owner.assert_called_once()

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

    def test_browser_has_one_takeover_confirmation_and_unified_execution(self):
        html = (path.parents[2]/'examples/scripts/debug/joint_head_workbench.html').read_text()
        acquire = html.split("$('acquire').onclick=",1)[1].split("\n",1)[0]
        self.assertEqual(acquire.count('window.confirm('),1)
        self.assertIn("post('takeover-preview')", acquire)
        self.assertIn("post('takeover-confirm'", acquire)
        self.assertNotIn('ARMS_SUPPORTED_CONNECT', acquire)
        self.assertIn("api('/api/hardware-trajectory-preview'",html)
        self.assertIn("action('/api/control/motion-confirm'",html)
        sliders = html.split('function buildJoints()',1)[1].split('async function executeHardware',1)[0]
        self.assertNotIn('/motion-confirm',sliders)
        self.assertNotIn('executeHardware(',sliders)

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
        server = ThreadingHTTPServer(('127.0.0.1', 0), module.handler(app, 'test-token'))
        threading.Thread(target=server.serve_forever, daemon=True).start()
        base = f'http://127.0.0.1:{server.server_port}'
        def request(path, body=None, **headers):
            data = None if body is None else json.dumps(body).encode()
            h = {'Authorization':'Bearer test-token', 'Origin': base, 'Content-Type': 'application/json', 'X-CSRF-Token': app.csrf}
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
            self.assertEqual(request('/api/stop', {}, Authorization='Bearer wrong')[0], 403)

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
            self.assertEqual(request('/api/recover', recover)[0], 200)
            self.assertEqual(received.pop(), dict(action='recover', **recover))
            state['state'] = 'READY'
            state['calibration_feedback']['sample_wall_ns'] = time.time_ns()
            self.assertEqual(request('/api/motion-preview', {'active_arm': 'left', 'q_target': [0.]*14})[0], 200)
            self.assertEqual(received.pop()['action'], 'preview')
            self.assertEqual(request('/api/home-preview', {'active_arm': 'left'})[0], 200)
            self.assertEqual(received.pop(), {'action': 'preview_home', 'active_arm': 'left'})
            self.assertEqual(request('/api/home-preview', {'active_arm': 'both'})[0], 400)
            self.assertEqual(request('/api/both-home-preview', {})[0], 200)
            self.assertEqual(received.pop(), {'action': 'preview_both_home'})
            self.assertEqual(request('/api/both-home-preview', {'active_arm': 'both'})[0], 400)
            for route, action in [('arm-home', 'preview_arm_home')]:
                self.assertEqual(request('/api/'+route+'-preview', {'active_arm': 'right'})[0], 200)
                self.assertEqual(received.pop(), {'action': action, 'active_arm': 'right'})
            self.assertEqual(request('/api/motion-preview', {'active_arm': 'left', 'q_target': [True]*14})[0], 400)
            confirm = dict(token='owner-token', confirmation='START_REVIEWED_SINGLE_ARM', physical_clearance=True)
            self.assertEqual(request('/api/motion-confirm', dict(confirm, physical_clearance=False))[0], 400)
            self.assertEqual(request('/api/motion-confirm', confirm)[0], 200)
            self.assertEqual(received.pop()['action'], 'confirm')
            # Stop remains available despite fault or stale feedback.
            state['calibration_feedback']['sample_wall_ns'] = 1
            self.assertFalse(json.loads(request('/api/state')[1])['host_sample_recent'])
            self.assertEqual(request('/api/stop', {})[0], 200)
            self.assertEqual(received, [{'action': 'stop'}])
            p = json.loads(request('/api/release-preview', {})[1])
            body = dict(token=p['token'], confirmation='ARMS_SUPPORTED_RELEASE')
            state['pid'] = 1235
            self.assertEqual(request('/api/release-confirm', body)[0], 400)
            p = json.loads(request('/api/release-preview', {})[1])
            body = dict(token=p['token'], confirmation='ARMS_SUPPORTED_RELEASE')
            self.assertEqual(request('/api/release-confirm', body)[0], 200)
            self.assertEqual(request('/api/release-confirm', body)[0], 400)
            self.assertEqual(received[-1], {'action': 'release', 'confirmation': 'ARMS_SUPPORTED_RELEASE'})
            p = json.loads(request('/api/release-preview', {})[1])
            expiry, identity = app.pending[p['token']]
            app.pending[p['token']] = (0., identity)
            self.assertEqual(request('/api/release-confirm', dict(token=p['token'], confirmation='ARMS_SUPPORTED_RELEASE'))[0], 400)
            p = json.loads(request('/api/release-preview', {})[1])
            request('/api/stop', {})
            self.assertEqual(request('/api/release-confirm', dict(token=p['token'], confirmation='ARMS_SUPPORTED_RELEASE'))[0], 400)
            self.assertEqual(request('/')[0],404)
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
