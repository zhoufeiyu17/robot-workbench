"""Workbench hardware gateway: explicit acquisition and reviewed owner commands."""
import json
import math
import os
import socket
import subprocess
import uuid
from pathlib import Path
import secrets
import signal
import select
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.error import HTTPError
from urllib.request import Request, build_opener, ProxyHandler


class Bridge:
    def __init__(self, owner_port=8098, fault_file=None, start_config=None):
        self.owner = f'http://127.0.0.1:{owner_port}'
        self.owner_port = owner_port
        self.start_config = start_config
        self.child = None
        self.camera_child = None
        self.setup_lock = threading.Lock()
        self.start_error = None
        self.fault_file = fault_file
        self.csrf = secrets.token_urlsafe(32)
        self.pending = {}
        self.takeover_pending = {}
        self.lock = threading.Lock()
        self.http = build_opener(ProxyHandler({}))

    def request(self, path, body=None):
        payload = None if body is None else json.dumps(body).encode()
        request = Request(self.owner + path, payload, headers={
            'Content-Type': 'application/json', 'Origin': self.owner})
        try:
            with self.http.open(request, timeout=.8) as response:
                data = response.read(4*1024*1024+1)
        except HTTPError as exc:
            detail=exc.read(65536).decode(errors='replace')
            raise ValueError(f'owner rejected request ({exc.code}): {detail}') from None
        if len(data) > 4*1024*1024:
            raise ValueError('owner reply too large')
        value = json.loads(data)
        if not isinstance(value, dict):
            raise ValueError('owner reply must be an object')
        return value

    def feedback(self):
        owner = self.request('/state')
        feedback = owner.get('calibration_feedback') or {}
        stamp = feedback.get('sample_wall_ns')
        age = (time.time_ns() - stamp) / 1e9 if type(stamp) is int else None
        return dict(owner=owner, source_age_s=age, reachable=True)

    def status(self):
        try:
            owner = self.request('/state')
            reachable, error = True, None
        except Exception as exc:
            owner, reachable, error = {}, False, f'{type(exc).__name__}: {exc}'
        feedback = owner.get('calibration_feedback')
        fresh = False
        if isinstance(feedback, dict):
            stamp = feedback.get('sample_wall_ns')
            stamps = feedback.get('sdk_timestamps')
            q = feedback.get('qpos')
            fresh = (isinstance(stamp, int) and not isinstance(stamp, bool)
                     and 0 <= time.time_ns() - stamp <= 500_000_000
                     and isinstance(stamps, list) and len(stamps) == 2
                     and all(isinstance(x, (int, float)) and not isinstance(x, bool)
                             and math.isfinite(x) and x >= 0 for x in stamps)
                     and isinstance(q, list) and len(q) == 14
                     and all(isinstance(x, (int, float)) and not isinstance(x, bool)
                             and math.isfinite(x) for x in q))
        fault = None
        if self.fault_file:
            try:
                fault = json.loads(self.fault_file.read_text())
            except Exception as exc:
                fault = {'read_error': str(exc)}
        return dict(reachable=reachable, error=error, owner=owner,
                    acquire_available=bool(self.start_config and not self.setup_lock.locked()),
                    camera=self.camera_status(),
                    starting_pid=self.child.pid if self.child and self.child.poll() is None else None,
                    start_error=self.start_error or (f'owner exited with code {self.child.poll()}; inspect startup log before reconnecting' if self.child and self.child.poll() is not None else None),
                    raw_feedback_available=isinstance(feedback, dict),
                    host_sample_recent=fresh,
                    feedback_note='Host sample age only; independent SDK progress must be checked by owner. Legacy robot.qpos may clip jaws.',
                    motion_enabled=bool(reachable and fresh and owner.get('feedback_healthy')
                                        and owner.get('state') in ('READY', 'STOPPED_DAMPING_HOLD', 'MANUAL_TAKEOVER')
                                        and owner.get('motion_protocol') == 'reviewed-single-arm-v1'),
                    motion_disabled_reason='Reviewed motion requires a compatible fresh owner; Cartesian commands unavailable.',
                    recorded_fault=fault)

    def camera_status(self):
        port=(self.start_config or {}).get('camera',{}).get('port')
        if not port:return {'ready':False,'error':'camera not configured'}
        try:
            with self.http.open(f'http://127.0.0.1:{port}/status',timeout=.8) as response:
                status=json.loads(response.read(65536))
            age=(time.time_ns()-int(status.get('host_receive_ns',0)))/1e9
            return {**status,'ready':status.get('ready') is True and 0<=age<2}
        except Exception as exc:return {'ready':False,'error':str(exc)}

    def acquire(self, body):
        if body != {'confirmation':'ARMS_SUPPORTED_CONNECT'}:raise ValueError('confirm current physical support before connecting')
        if not self.setup_lock.acquire(blocking=False):raise ValueError('connection setup already running')
        try:
            return self._acquire_ready(body)
        finally:self.setup_lock.release()

    def _acquire_ready(self, body):
        cfg=(self.start_config or {}).get('camera')
        if cfg and not self.camera_status()['ready']:
            with socket.socket() as probe:
                if probe.connect_ex(('127.0.0.1',cfg['port']))==0:raise ValueError('camera service occupied but not ready; inspect it without restarting')
            if self.camera_child and self.camera_child.poll() is None:raise ValueError('camera still starting; wait for status')
            output=Path(self.start_config['log_dir'])/('camera-'+uuid.uuid4().hex)
            output.parent.mkdir(parents=True,exist_ok=True)
            with output.with_suffix('.log').open('xb') as log:
                self.camera_child=subprocess.Popen([v.replace('{output}',str(output)) for v in cfg['argv']],cwd=self.start_config['cwd'],stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            for _ in range(40):
                if self.camera_status()['ready']:break
                if self.camera_child.poll() is not None:raise ValueError('camera failed; inspect '+str(output.with_suffix('.log')))
                time.sleep(.25)
            else:raise ValueError('camera not ready yet; owner not started')
        status=self.status()
        if status['reachable']:
            return {'status':'EXISTING_OWNER','owner_state':status['owner'].get('state'),'camera':self.camera_status(),'note':'Existing controller preserved; no reinitialization'}
        return self.acquire_owner(body)

    def acquire_owner(self, body):
        if body != {'confirmation': 'ARMS_SUPPORTED_CONNECT'}:
            raise ValueError('confirm current physical support before connecting')
        with self.lock:
            if not self.start_config:
                raise ValueError('owner start config not installed on this workstation')
            if self.child and self.child.poll() is None:
                raise ValueError('owner startup or existing process still active; inspect state')
            with socket.socket() as probe:
                if probe.connect_ex(('127.0.0.1', self.owner_port)) == 0:
                    raise ValueError('owner port occupied; do not start another controller')
            conflicts = self.controller_processes()
            if conflicts:
                raise ValueError(f"controller still running: PID {conflicts[0]['pid']}; preview takeover")
            cfg = self.start_config
            output = Path(cfg['log_dir']) / ('owner-' + uuid.uuid4().hex)
            output.parent.mkdir(parents=True, exist_ok=True)
            argv = [arg.replace('{output}', str(output)) for arg in cfg['argv']]
            self.pending.clear()
            try:
                with output.with_suffix('.startup.log').open('xb') as log:
                    self.child = subprocess.Popen(argv, cwd=cfg['cwd'], env={**os.environ, **cfg.get('env', {})},
                                                  stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                                                  start_new_session=True)
            except Exception as exc:
                self.start_error = str(exc)
                raise
            self.start_error = None
            return {'starting_pid': self.child.pid, 'status': 'INITIALIZING',
                    'note': 'Explicit startup only; no home or trajectory command. Check feedback and pedal state.'}

    def controller_processes(self):
        """Only exact configured controller script names; never substring-kill."""
        if not self.start_config or not Path('/proc').is_dir():
            raise ValueError('hardware acquisition requires Linux process preflight and start config')
        processes = []
        for proc in Path('/proc').glob('[0-9]*'):
            if int(proc.name) == os.getpid():
                continue
            try:
                before = (proc / 'stat').read_text().rsplit(')', 1)[1].split()
                argv = (proc / 'cmdline').read_bytes().split(b'\0')
                args = [arg.decode(errors='replace') for arg in argv if arg]
                names = [Path(args[0]).name] if args else []
                # A filename merely mentioned by cat/grep is not a controller.
                if names and (names[0].startswith('python') or names[0] in ('bash', 'sh')):
                    for arg in args[1:]:
                        if arg in ('-c', '-m'):
                            break
                        if not arg.startswith('-'):
                            names.append(Path(arg).name)
                            break
                matches = sorted(set(names) & set(self.start_config['process_names']))
                if not matches:
                    continue
                uid = proc.stat().st_uid
                after = (proc / 'stat').read_text().rsplit(')', 1)[1].split()
                if before[19] != after[19]:
                    raise ValueError('controller changed during scan; retry preview')
                if after[0] == 'Z':
                    continue
                processes.append(dict(pid=int(proc.name), starttime=after[19], uid=uid, name=matches[0]))
            except FileNotFoundError:
                continue
            except PermissionError:
                raise ValueError('cannot inspect process ownership; acquisition blocked')
        return sorted(processes, key=lambda item: item['pid'])

    def takeover_preview(self, body):
        if body:
            raise ValueError('takeover preview takes no parameters')
        processes = self.controller_processes()
        if any(p['uid'] != os.getuid() for p in processes):
            raise ValueError('controller belongs to another user; cannot take over with this account')
        token = secrets.token_urlsafe(24)
        with self.lock:
            self.takeover_pending.clear()
            self.takeover_pending[token] = (time.monotonic() + 30, processes)
        return dict(token=token, expires_in_s=30, processes=processes,
                    effect='Stop every listed robot controller, force remaining controllers to exit, then start camera and control. Confirmation asserts the environment is safe; no further support prompt.')

    def stop_reviewed_controllers(self, processes):
        # pidfds bind signals to the inspected process even if its PID is reused.
        if processes and (not hasattr(os, 'pidfd_open') or not hasattr(signal, 'pidfd_send_signal')):
            raise ValueError('Linux pidfd support required for takeover')
        handles = []
        try:
            for process in processes:
                handles.append(os.pidfd_open(process['pid']))
            if self.controller_processes() != processes:
                raise ValueError('controllers changed; preview takeover again')
            for fd in handles:
                try:
                    signal.pidfd_send_signal(fd, signal.SIGTERM)
                except ProcessLookupError:
                    pass
            deadline = time.monotonic() + 2
            remaining = list(handles)
            while remaining and time.monotonic() < deadline:
                exited, _, _ = select.select(remaining, [], [], max(0, deadline-time.monotonic()))
                remaining = [fd for fd in remaining if fd not in exited]
            for fd in remaining:
                try:
                    signal.pidfd_send_signal(fd, signal.SIGKILL)
                except ProcessLookupError:
                    pass
            deadline = time.monotonic() + 2
            while remaining and time.monotonic() < deadline:
                exited, _, _ = select.select(remaining, [], [], max(0, deadline-time.monotonic()))
                remaining = [fd for fd in remaining if fd not in exited]
            if remaining:
                raise ValueError('controller exit not confirmed; owner not started')
        finally:
            for fd in handles:
                os.close(fd)

    def takeover_confirm(self, body):
        if (set(body) != {'token', 'confirmation'} or not isinstance(body['token'], str)
                or body['confirmation'] != 'TAKE_OVER_CONTROL'):
            raise ValueError('explicit takeover confirmation required')
        if not self.setup_lock.acquire(blocking=False):
            raise ValueError('connection setup already running')
        try:
            with self.lock:
                preview = self.takeover_pending.pop(body['token'], None)
            if preview is None or time.monotonic() > preview[0]:
                raise ValueError('takeover preview expired or consumed; preview again')
            if self.controller_processes() != preview[1]:
                raise ValueError('controllers changed; preview takeover again')
            self.stop_reviewed_controllers(preview[1])
            if self.controller_processes():
                raise ValueError('new controller appeared; preview takeover again')
            with self.lock:
                self.pending.clear()
            return self._acquire_ready({'confirmation': 'ARMS_SUPPORTED_CONNECT'})
        finally:
            self.setup_lock.release()

    @staticmethod
    def identity(status):
        owner = status['owner']
        return [owner.get(key) for key in ('pid', 'mode', 'state', 'command_epoch')]

    def command(self, path, body):
        if not isinstance(body, dict):
            raise ValueError('object required')
        if path == '/api/takeover-preview':
            return self.takeover_preview(body)
        if path == '/api/takeover-confirm':
            return self.takeover_confirm(body)
        if path == '/api/acquire':
            return self.acquire(body)
        if path == '/api/trajectory-preview':
            status = self.status()
            if (not status['motion_enabled'] or 'workbench_trajectory_v1' not in status['owner'].get('capabilities', [])):
                raise ValueError('workbench trajectory owner required; connect updated owner')
            if set(body) != {'waypoints'}:
                raise ValueError('waypoints required')
            if (isinstance(body['waypoints'],list) and any(isinstance(w,dict) and w.get('capture') for w in body['waypoints'])
                    and 'calibration_capture_ack_v1' not in status['owner'].get('capabilities', [])):
                raise ValueError('capture ACK capable owner required')
            return self.request('/operator', dict(action='preview_workbench', **body))
        if path == '/api/capture-ack':
            if (set(body) not in ({'owner_pid','command_epoch','sample_id'}, {'owner_pid','command_epoch','sample_id','capture_ready_wall_ns'})
                    or ('capture_ready_wall_ns' in body and (type(body['capture_ready_wall_ns']) is not int or body['capture_ready_wall_ns']<=0))
                    or any(type(body[k]) is not int for k in ('owner_pid','command_epoch'))
                    or not isinstance(body['sample_id'],str) or not body['sample_id'].strip()):
                raise ValueError('capture ACK requires owner_pid, command_epoch and sample_id')
            return self.request('/operator',dict(action='capture_ack',**body))
        if path == '/api/jaw-preview':
            if set(body) != {'active_arm', 'sdk_gap_mm'} or body['active_arm'] not in ('left', 'right', 'both'):
                raise ValueError('arm and SDK gaps required')
            gap = body['sdk_gap_mm']
            gaps = gap if isinstance(gap, dict) and body['active_arm'] == 'both' else dict(left=gap, right=gap)
            if set(gaps) != {'left', 'right'} or any(type(v) not in (int, float) or not math.isfinite(v) for v in gaps.values()):
                raise ValueError('finite SDK gaps required')
            if not self.status()['motion_enabled']:
                raise ValueError('compatible fresh motion owner required')
            return self.request('/operator', dict(action='preview_jaw_target', **body))
        if path in ('/api/motion-preview', '/api/both-home-preview', '/api/home-preview', '/api/arm-home-preview',  '/api/motion-confirm'):
            status = self.status()
            if not status['motion_enabled']:
                raise ValueError('compatible fresh motion owner required')
            if path == '/api/both-home-preview':
                if body:
                    raise ValueError('both-arm home takes no parameters')
                return self.request('/operator', {'action': 'preview_both_home'})
            if path in ('/api/home-preview', '/api/arm-home-preview'):
                if set(body) != {'active_arm'} or body['active_arm'] not in ('left', 'right'):
                    raise ValueError('home requires exactly one active arm')
                return self.request('/operator', dict(action={'/api/home-preview':'preview_home', '/api/arm-home-preview':'preview_arm_home'}[path], **body))
            if path.endswith('preview'):
                if set(body) != {'active_arm', 'q_target'} or body['active_arm'] not in ('left', 'right'):
                    raise ValueError('single arm and full target required')
                q = body['q_target']
                if (not isinstance(q, list) or len(q) != 14
                        or any(isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) for x in q)):
                    raise ValueError('14 finite target numbers required')
                return self.request('/operator', dict(action='preview', **body))
            if (set(body) != {'token', 'confirmation', 'physical_clearance'}
                    or not isinstance(body['token'], str)
                    or body['confirmation'] != 'START_REVIEWED_SINGLE_ARM'
                    or body['physical_clearance'] is not True):
                raise ValueError('explicit reviewed trajectory and clearance confirmation required')
            return self.request('/operator', dict(action='confirm', **body))
        if path == '/api/recover':
            if (set(body) != {'confirmation', 'owner_pid', 'command_epoch'}
                    or body['confirmation'] != 'ACKNOWLEDGE_MOTION_FAULT'
                    or any(type(body[key]) is not int for key in ('owner_pid', 'command_epoch'))):
                raise ValueError('explicit fault acknowledgement and owner identity required')
            return self.request('/operator', dict(action='recover', **body))
        if path == '/api/stop':
            if body and (set(body)!={'owner_pid','command_epoch'} or any(type(body[k]) is not int for k in ('owner_pid','command_epoch'))):
                raise ValueError('conditional stop requires owner_pid and command_epoch')
            with self.lock:
                self.pending.clear()
            return self.request('/operator', {'action': 'stop', **body})
        if path == '/api/release-preview':
            if body:
                raise ValueError('release preview takes no parameters')
            status = self.status()
            if not status['reachable']:
                raise ValueError('owner unreachable; release cannot be sent')
            token = secrets.token_urlsafe(24)
            with self.lock:
                self.pending.clear()
                self.pending[token] = (time.monotonic() + 30, self.identity(status))
            return dict(token=token, expires_in_s=30, owner_identity=self.identity(status),
                        effect='Close both controllers and remove software holding/compensation. No automatic restart. Arms must be physically supported.')
        if path == '/api/release-confirm':
            if set(body) != {'token', 'confirmation'} or body['confirmation'] != 'ARMS_SUPPORTED_RELEASE':
                raise ValueError('explicit supported-arm confirmation required')
            if not isinstance(body['token'], str):
                raise ValueError('preview token required')
            with self.lock:
                preview = self.pending.pop(body['token'], None)
            if preview is None or time.monotonic() > preview[0]:
                raise ValueError('preview expired or consumed; preview again')
            current = self.status()
            if not current['reachable'] or self.identity(current) != preview[1]:
                raise ValueError('owner changed; preview again')
            return self.request('/operator', {'action': 'release', 'confirmation': 'ARMS_SUPPORTED_RELEASE'})
        raise ValueError('unsupported action; no arbitrary command forwarding')



def handler(app, token):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_): pass
        def send(self, value, code=200):
            raw=json.dumps(value,allow_nan=False).encode()
            self.send_response(code);self.send_header('Content-Type','application/json')
            self.send_header('Content-Length',str(len(raw)));self.end_headers();self.wfile.write(raw)
        def authorized(self):
            return self.headers.get('Host') in (f'127.0.0.1:{self.server.server_port}',f'localhost:{self.server.server_port}') and secrets.compare_digest(self.headers.get('Authorization',''),'Bearer '+token)
        def do_GET(self):
            if not self.authorized():return self.send({'error':'unauthorized'},403)
            if self.path=='/api/feedback':
                try:return self.send(app.feedback())
                except Exception as exc:return self.send({'reachable':False,'error':str(exc)},503)
            if self.path!='/api/state':return self.send({'error':'not found'},404)
            self.send(app.status())
        def do_POST(self):
            if not self.authorized():return self.send({'error':'unauthorized'},403)
            try:
                size=int(self.headers.get('Content-Length','0'))
                if not 0<size<=4*1024*1024:raise ValueError('small JSON required')
                self.send(app.command(self.path,json.loads(self.rfile.read(size))))
            except (ValueError,TypeError) as exc:self.send({'error':str(exc)},400)
            except Exception as exc:self.send({'error':str(exc),'outcome':'unknown; inspect owner, never retry automatically'},502)
    return Handler


def main():
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--port',type=int,default=8119);p.add_argument('--owner-port',type=int,default=8098)
    p.add_argument('--start-config',type=Path,required=True);p.add_argument('--token-file',type=Path,required=True)
    a=p.parse_args();token=a.token_file.read_text().strip()
    if len(token)<32 or not 1024<=a.port<=65535 or not 1024<=a.owner_port<=65535 or a.port==a.owner_port:raise ValueError('invalid gateway settings')
    start_config=json.loads(a.start_config.read_text())
    if start_config.get('jaw_mapping_file'):
        from .joint_head_jaw_mapping import configure_mapping
        configure_mapping(start_config['jaw_mapping_file'])
    app=Bridge(a.owner_port,start_config=start_config)
    ThreadingHTTPServer(('127.0.0.1',a.port),handler(app,token)).serve_forever()

if __name__=='__main__':main()
