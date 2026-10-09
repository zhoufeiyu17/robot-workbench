#!/usr/bin/env python3
"""Loopback console; optional trusted owner launch only after explicit operator click."""
import argparse
import json
import math
import os
import socket
import subprocess
import uuid
from pathlib import Path
import secrets
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
        self.start_error = None
        self.fault_file = fault_file
        self.csrf = secrets.token_urlsafe(32)
        self.pending = {}
        self.lock = threading.Lock()
        self.http = build_opener(ProxyHandler({}))

    def request(self, path, body=None):
        payload = None if body is None else json.dumps(body).encode()
        request = Request(self.owner + path, payload, headers={
            'Content-Type': 'application/json', 'Origin': self.owner})
        with self.http.open(request, timeout=.8) as response:
            data = response.read(65537)
        if len(data) > 65536:
            raise ValueError('owner reply too large')
        value = json.loads(data)
        if not isinstance(value, dict):
            raise ValueError('owner reply must be an object')
        return value

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
                    acquire_available=bool(self.start_config and not reachable and not (self.child and self.child.poll() is None)),
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

    def acquire(self, body):
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
            # Inspect command lines without executing shell commands. This catches known
            # controllers; the owner port bind is a second guard before SDK connect.
            if not Path('/proc').is_dir():
                raise ValueError('hardware acquisition requires Linux process preflight')
            for proc in Path('/proc').glob('[0-9]*'):
                if proc.name == str(os.getpid()):
                    continue
                try:
                    argv = (proc / 'cmdline').read_bytes().split(b'\0')
                except FileNotFoundError:
                    continue
                except PermissionError:
                    raise ValueError('cannot inspect process ownership; acquisition blocked')
                if any(Path(arg.decode(errors='replace')).name in self.start_config['process_names'] for arg in argv):
                    raise ValueError(f'possible controller still running: PID {proc.name}')
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

    @staticmethod
    def identity(status):
        owner = status['owner']
        return [owner.get(key) for key in ('pid', 'mode', 'state', 'command_epoch')]

    def command(self, path, body):
        if not isinstance(body, dict):
            raise ValueError('object required')
        if path == '/api/acquire':
            return self.acquire(body)
        if path == '/api/jaw-preview':
            if set(body) != {'active_arm', 'sdk_gap_mm'} or body['active_arm'] not in ('left', 'right') or type(body['sdk_gap_mm']) not in (int, float) or not math.isfinite(body['sdk_gap_mm']):
                raise ValueError('one arm and finite SDK gap in mm required')
            if not self.status()['motion_enabled']:
                raise ValueError('compatible fresh motion owner required')
            return self.request('/operator', dict(action='preview_jaw_target', **body))
        if path in ('/api/motion-preview', '/api/both-home-preview', '/api/home-preview', '/api/arm-home-preview', '/api/jaw-probe-preview', '/api/motion-confirm'):
            status = self.status()
            if not status['motion_enabled']:
                raise ValueError('compatible fresh motion owner required')
            if path == '/api/both-home-preview':
                if body:
                    raise ValueError('both-arm home takes no parameters')
                return self.request('/operator', {'action': 'preview_both_home'})
            if path in ('/api/home-preview', '/api/arm-home-preview', '/api/jaw-probe-preview'):
                if set(body) != {'active_arm'} or body['active_arm'] not in ('left', 'right'):
                    raise ValueError('home requires exactly one active arm')
                return self.request('/operator', dict(action={'/api/home-preview':'preview_home', '/api/arm-home-preview':'preview_arm_home', '/api/jaw-probe-preview':'preview_jaw_probe'}[path], **body))
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
            if body:
                raise ValueError('stop takes no parameters')
            with self.lock:
                self.pending.clear()
            return self.request('/operator', {'action': 'stop'})
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


def handler(app):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def send(self, value, code=200, mime='application/json'):
            data = value if isinstance(value, bytes) else json.dumps(value, allow_nan=False).encode()
            self.send_response(code)
            self.send_header('Content-Type', mime)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Content-Security-Policy', "default-src 'self'; script-src 'unsafe-inline'; style-src 'unsafe-inline'; img-src 'self' http://127.0.0.1:8114; connect-src 'self'; frame-ancestors 'none'; base-uri 'none'")
            self.end_headers()
            try:
                self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def host_valid(self):
            return self.headers.get('Host') in (f'127.0.0.1:{self.server.server_port}', f'localhost:{self.server.server_port}')

        def do_GET(self):
            if not self.host_valid():
                return self.send({'error': 'loopback Host required'}, 403)
            if self.path == '/':
                page = Path(__file__).with_suffix('.html').read_text().replace('__CSRF__', app.csrf)
                return self.send(page.encode(), mime='text/html; charset=utf-8')
            if self.path == '/api/state':
                return self.send(app.status())
            self.send({'error': 'not found'}, 404)

        def do_POST(self):
            if (not self.host_valid() or self.headers.get('Origin') != 'http://' + self.headers.get('Host', '')
                    or not secrets.compare_digest(self.headers.get('X-CSRF-Token', ''), app.csrf)):
                return self.send({'error': 'same-origin request with CSRF token required'}, 403)
            try:
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length <= 1024 or self.headers.get_content_type() != 'application/json':
                    raise ValueError('small JSON request required')
                body = json.loads(self.rfile.read(length))
                value = app.command(self.path, body)
                return self.send(value, 202)
            except (ValueError, TypeError) as exc:
                return self.send({'error': str(exc)}, 400)
            except Exception as exc:
                return self.send({'error': str(exc), 'outcome': 'unknown; inspect owner state, never automatically retry'}, 502)
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8115)
    parser.add_argument('--owner-port', type=int, default=8098)
    parser.add_argument('--fault-file', type=Path, help='Fixed local JSON history shown explicitly as recorded evidence')
    parser.add_argument('--owner-start-config', type=Path, help='Trusted local JSON argv/cwd/env/log_dir/process_names; starts only on explicit click')
    args = parser.parse_args()
    if any(not 1024 <= p <= 65535 for p in (args.port, args.owner_port)) or args.port == args.owner_port:
        parser.error('distinct unprivileged loopback ports required')
    cfg = None
    if args.owner_start_config:
        cfg = json.loads(args.owner_start_config.read_text())
        if (not isinstance(cfg, dict) or set(cfg) - {'argv', 'cwd', 'env', 'log_dir', 'process_names'}
                or not all(k in cfg for k in ('argv', 'cwd', 'log_dir', 'process_names'))
                or not isinstance(cfg['argv'], list) or not cfg['argv']
                or not all(isinstance(x, str) and x for x in cfg['argv'])
                or not Path(cfg['argv'][0]).is_absolute()
                or sum('{output}' in x for x in cfg['argv']) != 1
                or not isinstance(cfg['process_names'], list) or 'joint_head_real_owner.py' not in cfg['process_names']
                or not all(isinstance(x, str) and x for x in cfg['process_names'])
                or not isinstance(cfg.get('env', {}), dict)
                or not all(isinstance(k, str) and isinstance(v, str) for k,v in cfg.get('env', {}).items())
                or not all(isinstance(cfg[k], str) and Path(cfg[k]).is_absolute() for k in ('cwd', 'log_dir'))):
            parser.error('invalid trusted owner start config')
    app = Bridge(args.owner_port, args.fault_file, cfg)
    server = ThreadingHTTPServer(('127.0.0.1', args.port), handler(app))
    print(f'Explicit-acquisition bridge: http://127.0.0.1:{args.port}/ ; does not initialize hardware', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
