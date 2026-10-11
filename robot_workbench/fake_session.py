"""A loopback, CPU-only entry point for explicitly started bounded fake sessions."""
from copy import deepcopy
from http.client import HTTPConnection, HTTPException
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import re
import secrets
import signal
import subprocess
import threading
import time
import traceback
from urllib.parse import urlsplit

from robot_workbench.service import loopback_browser_hosts
from rule_based_workbench.owned_process import group_alive

ROOT = Path(__file__).resolve().parents[1]


class PublicSessionError(ValueError):
    """Only reviewed user guidance; subprocess arguments and logs stay private."""


def process_table():
    """Identity plus ancestry. Never infer ownership from a command name."""
    rows = {}
    if Path('/proc/self/stat').exists():
        for entry in Path('/proc').iterdir():
            if not entry.name.isdigit():
                continue
            try:
                fields = (entry/'stat').read_text().rsplit(')', 1)[1].split()
                rows[int(entry.name)] = (fields[19], int(fields[1]), int(fields[2]), fields[0])
            except (OSError, ValueError, IndexError):
                pass
    else:
        output = subprocess.check_output(['ps', '-axo', 'pid=,ppid=,pgid=,lstart=,stat='], text=True)
        for line in output.splitlines():
            fields = line.split()
            if len(fields) >= 9:
                rows[int(fields[0])] = (' '.join(fields[3:8]), int(fields[1]), int(fields[2]), fields[8])
    return rows


def remember_owned(child, owned):
    rows = process_table()
    # A new session's group can survive its leader. A live recorded identity or
    # that same non-reusable group establishes descent, even after reparenting.
    parents = {pid for pid, identity in owned.items() if pid in rows and rows[pid][0] == identity}
    group_proven = child.pid in parents or any(rows[pid][2] == child.pid for pid in parents)
    if not owned and child.poll() is None and child.pid in rows:
        owned[child.pid] = rows[child.pid][0]; parents.add(child.pid); group_proven = True
    changed = True
    while changed:
        changed = False
        for pid, (identity, parent, group, _) in rows.items():
            if parent in parents or (group_proven and group == child.pid):
                if pid not in parents:
                    parents.add(pid); changed = True
                owned.setdefault(pid, identity)
    return rows


def stop_owned(child, owned, grace_s):
    """Only signal identities observed in this child's session/descendant tree."""
    remember_owned(child, owned)
    for sig, duration in ((signal.SIGTERM, grace_s), (signal.SIGKILL, 2.)):
        rows = remember_owned(child, owned)
        for pid, identity in tuple(owned.items()):
            if pid in rows and rows[pid][0] == identity and not rows[pid][3].startswith('Z'):
                # Re-read immediately before each signal to reject PID reuse.
                current = process_table().get(pid)
                if current and current[0] == identity:
                    try:
                        os.kill(pid, sig)
                    except ProcessLookupError:
                        pass
        end = time.monotonic() + duration
        while time.monotonic() < end:
            child.poll()
            rows = remember_owned(child, owned)
            if not any(pid in rows and rows[pid][0] == identity and not rows[pid][3].startswith('Z')
                       for pid, identity in owned.items()):
                break
            time.sleep(.05)
    rows = process_table()
    survivors = [pid for pid, identity in owned.items()
                 if pid in rows and rows[pid][0] == identity and not rows[pid][3].startswith('Z')]
    group_remaining = group_alive(child.pid) and any(row[2] == child.pid and not row[3].startswith('Z') for row in rows.values())
    return dict(identities=owned, survivors=survivors, group_remaining=group_remaining,
                all_owned_exited=not survivors and not group_remaining)


class FakeSession:
    def __init__(self, prepare, *, state_dir, deadline_wall_ns, profile='protocol_fake',
                 start_guard=None, on_stopped=None, poll_s=.2, grace_s=5.):
        if profile not in ('protocol_fake', 'full_fake'):
            raise ValueError('only explicit fake profiles can be managed')
        if type(deadline_wall_ns) is not int or deadline_wall_ns <= 0:
            raise ValueError('an explicit finite wall-clock deadline is required')
        self.prepare, self.start_guard, self.on_stopped = prepare, start_guard, on_stopped
        self.profile, self.deadline_wall_ns = profile, deadline_wall_ns
        self.state_dir = Path(state_dir); self.state_dir.mkdir(parents=True, exist_ok=True)
        self.poll_s, self.grace_s = poll_s, grace_s
        self.lock = threading.Lock(); self.closed = threading.Event(); self.cancel = threading.Event()
        self.manager_instance_id = secrets.token_hex(16); self.csrf = secrets.token_urlsafe(32)
        self.generation = 0; self.status = 'stopped'; self.message = '尚未启动 fake 服务'; self.error = None
        self.stage = 'stopped'; self.stage_started = time.monotonic()
        self.current_run = None; self.backend_port = None; self.worker = None; self.child = None
        self.requests = {}; self.cleanup_ok = True
        self.timer = threading.Thread(target=self._deadline, daemon=True); self.timer.start()

    def _set(self, status, message):
        if self.status != status:
            self.stage_started = time.monotonic()
        self.status = self.stage = status; self.message = message

    def remaining(self):
        return max(0., (self.deadline_wall_ns-time.time_ns())/1e9)

    def snapshot(self):
        with self.lock:
            start = self.status in ('stopped', 'failed') and self.cleanup_ok and self.child is None and self.worker is None and self.remaining() > 0 and not self.closed.is_set()
            if self.profile == 'full_fake' and self.start_guard is None:
                start = False
            stop = self.status in ('starting', 'ready', 'failed') and (self.child is not None or self.worker is not None)
            return dict(schema='fake-session.v1', managed=True, profile=self.profile, manager_instance_id=self.manager_instance_id,
                csrf=self.csrf, generation=self.generation, status=self.status, message=self.message, error=self.error,
                allowed=dict(start=dict(allowed=start, reason='' if start else '当前状态或已授权时段不允许启动'),
                             stop=dict(allowed=stop, reason='' if stop else '没有可停止的自有 fake 服务')),
                deadline_wall_ns=self.deadline_wall_ns, remaining_wall_s=self.remaining(),
                cleanup_verified=self.worker is None and self.child is None and self.cleanup_ok,
                stage=dict(id=self.stage, label=self.message, elapsed_s=time.monotonic()-self.stage_started),
                current_run=deepcopy(self.current_run), workbench_url='/')

    def request(self, action, body):
        if not isinstance(body, dict) or body.get('manager_instance_id') != self.manager_instance_id:
            raise ValueError('管理器已变化，请刷新状态')
        ident = body.get('request_id')
        if not isinstance(ident, str) or not 1 <= len(ident) <= 128:
            raise ValueError('request_id required')
        if set(body) - {'manager_instance_id', 'request_id', 'generation', 'confirm'}:
            raise ValueError('unsupported lifecycle parameters')
        encoded = json.dumps([action, body], sort_keys=True)
        with self.lock:
            if ident in self.requests:
                if self.requests[ident] != encoded:
                    raise ValueError('request_id reused with different parameters')
            else:
                if type(body.get('generation')) is not int or body['generation'] != self.generation:
                    raise ValueError('fake 服务已变化，请刷新后操作')
                if self.remaining() <= 0 or self.closed.is_set():
                    raise ValueError('本次已授权时段已结束')
                if action == 'start':
                    if (self.status not in ('stopped', 'failed') or self.child is not None or self.worker is not None
                            or not self.cleanup_ok or (self.profile == 'full_fake' and self.start_guard is None)):
                        raise ValueError('当前不能启动 fake 服务')
                    self.generation += 1; self.cancel = threading.Event(); self.error = None
                    self.backend_port = None; self.current_run = None; self._set('starting', '正在准备 fake 服务')
                    self.worker = threading.Thread(target=self._run, args=(self.generation, self.cancel), daemon=True)
                    self.worker.start()
                elif action == 'stop':
                    if body.get('confirm') is not True:
                        raise ValueError('停止会中断当前 fake 作业，必须明确确认')
                    if self.worker is None:
                        raise ValueError('没有可停止的自有 fake 服务')
                    self.backend_port = None; self.cancel.set(); self._set('stopping', '正在停止本次 fake 服务')
                else:
                    raise ValueError('unknown lifecycle action')
                self.requests[ident] = encoded
                if len(self.requests) > 128:
                    self.requests.pop(next(iter(self.requests)))
        return self.snapshot()

    def _log_failure(self):
        with (self.state_dir/'manager-errors.log').open('a') as stream:
            os.chmod(stream.name, 0o600); traceback.print_exc(file=stream)

    def _health(self, port):
        identity = None
        for business in ('calibration', 'collection'):
            connection = HTTPConnection('127.0.0.1', port, timeout=.6)
            try:
                connection.request('GET', '/api/state?environment=fake&business='+business)
                response = connection.getresponse(); raw = response.read(4*1024*1024+1)
                value = json.loads(raw)
                if (response.status != 200 or len(raw) > 4*1024*1024 or not isinstance(value, dict)
                        or value.get('device', {}).get('environment') != 'fake'
                        or not isinstance(value.get('server_instance_id'), str) or not value['server_instance_id']):
                    return False
                current = (value['server_instance_id'], value['device'].get('id'))
                if identity is not None and identity != current:
                    return False
                identity = current
            except (OSError, HTTPException, ValueError):
                return False
            finally:
                connection.close()
        return identity[0]

    def _run(self, generation, cancel):
        spec = None; child = None; owned = {}; failed = False; public_error = None
        try:
            if self.start_guard is not None and self.start_guard(generation, self.remaining()) is False:
                raise PermissionError('fake resource admission refused')
            spec = self.prepare(generation, self.remaining())
            if cancel.is_set() or self.remaining() <= 0:
                return
            if (not isinstance(spec, dict) or not isinstance(spec.get('argv'), list)
                    or not spec['argv'] or not all(isinstance(x, str) and x for x in spec['argv'])):
                raise ValueError('invalid prepared child specification')
            run = Path(spec['run_path']); service = Path(spec['service_path']); log = Path(spec['log_path'])
            if not all(path.is_absolute() for path in (run, service, log, Path(spec['cwd']))) or service.exists():
                raise ValueError('fresh absolute run/service paths required')
            run.mkdir(parents=True, exist_ok=True); log.parent.mkdir(parents=True, exist_ok=True)
            with log.open('ab') as stream:
                # Spawn is the only small local operation inside this lock: a
                # stop/deadline cannot slip between the final gate and creation.
                with self.lock:
                    if cancel.is_set() or self.remaining() <= 0 or self.closed.is_set():
                        return
                    child = subprocess.Popen(spec['argv'], cwd=spec['cwd'], env=spec.get('env'),
                        stdin=subprocess.DEVNULL, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
            remember_owned(child, owned)
            with self.lock:
                self.child = child; self.current_run = dict(id=run.name, generation=generation, pid=child.pid, process_start_ticks=owned.get(child.pid))
            last_health = 0.; misses = 0
            while not cancel.is_set() and self.remaining() > 0 and not self.closed.is_set():
                remember_owned(child, owned)
                if child.poll() is not None:
                    raise RuntimeError('child exited')
                if time.monotonic()-last_health >= 1.:
                    last_health = time.monotonic(); healthy = False
                    try:
                        value = json.loads(service.read_text())
                        port = value.get('ui_port')
                        service_pid = value.get('pid'); current = process_table().get(service_pid)
                        if (type(port) is int and 1 <= port <= 65535 and value.get('environment') == 'fake'
                                and type(service_pid) is int and service_pid in owned
                                and current and current[0] == owned[service_pid] and not current[3].startswith('Z')):
                            healthy = self._health(port)
                    except (OSError, ValueError, TypeError):
                        pass
                    progress = next(((name, label) for name, label in (
                        ('service.json', '正在核验工作台接口'), ('calibration-config.json', '正在接入标定与数采服务'),
                        ('plant-fixture.json', '正在准备物理相机'), ('registration.json', '正在准备物理场景'),
                        ('run.json', '正在加载 fake 运行环境')) if (service.parent/name).exists()), None)
                    with self.lock:
                        if not cancel.is_set() and self.remaining() > 0:
                            if healthy:
                                misses = 0; self.backend_port = port; self.error = None
                                self.current_run['server_instance_id'] = healthy
                                self._set('ready', '工作台已可访问')
                            elif self.status == 'ready':
                                misses += 1
                                if misses >= 3:
                                    self.backend_port = None; self.error = 'fake 服务暂时无法读取，可停止后重试'
                                    self._set('failed', self.error)
                            elif self.status == 'starting' and progress is not None:
                                if self.stage != progress[0]:
                                    self.stage_started = time.monotonic()
                                self.stage, self.message = progress
                cancel.wait(self.poll_s)
        except Exception as error:
            if isinstance(error, PublicSessionError):public_error = str(error)
            failed = True; self._log_failure()
        finally:
            receipt = dict(identities={}, survivors=[], group_remaining=False, all_owned_exited=True)
            if child is not None:
                try:
                    receipt = stop_owned(child, owned, self.grace_s)
                except Exception:
                    self._log_failure(); receipt['all_owned_exited'] = False
            receipt.update(generation=generation, checked_wall_ns=time.time_ns())
            if self.on_stopped is not None:
                try:
                    self.on_stopped(spec, receipt)
                except Exception:
                    self._log_failure(); receipt['callback_failed'] = True
            (self.state_dir/f'cleanup-{generation}.json').write_text(json.dumps(receipt, sort_keys=True))
            with self.lock:
                self.backend_port = None; self.child = None; self.worker = None
                self.cleanup_ok = receipt['all_owned_exited'] and not receipt.get('callback_failed')
                self.error = ('自有进程清退尚未核验，禁止重新启动' if not self.cleanup_ok
                              else public_error or 'fake 启动或运行失败，诊断已保存在本任务目录' if failed else None)
                if self.remaining() <= 0:
                    self._set('expired', '本次已授权时段已结束')
                elif self.error:
                    self._set('failed', self.error)
                else:
                    self._set('stopped', 'fake 服务已停止；结果保留')

    def _deadline(self):
        while not self.closed.wait(min(.2, max(.01, self.remaining()))):
            if self.remaining() <= 0:
                with self.lock:
                    self.backend_port = None; self.cancel.set(); self._set('expired', '本次已授权时段已结束')
                return

    def close(self):
        self.closed.set(); self.cancel.set()
        worker = self.worker
        if worker is not None:
            worker.join(self.grace_s+4)


def serve_session(manager, *, port=0, browser_ports=(), web_dir=ROOT/'web'):
    allowed_hosts = loopback_browser_hosts(browser_ports)
    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'; timeout = 5; disable_nagle_algorithm = True
        def log_message(self, *args):
            pass
        def send(self, code, value, kind='application/json'):
            raw = json.dumps(value, ensure_ascii=False, allow_nan=False).encode() if kind == 'application/json' else value
            self.send_response(code); self.send_header('Content-Type', kind)
            self.send_header('Content-Length', str(len(raw))); self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff'); self.send_header('Connection', 'close')
            self.end_headers(); self.close_connection = True
            try:
                self.wfile.write(raw)
            except (BrokenPipeError, ConnectionResetError):
                pass
        def valid(self):
            return len(self.headers.get_all('Host', [])) == 1 and self.headers['Host'] in self.server.allowed_hosts
        def route(self, body=None):
            path = urlsplit(self.path)
            if path.scheme or path.netloc or path.fragment:
                return self.send(404, {'error': 'not found'})
            if path.path == '/api/session' and body is None:
                return self.send(200, manager.snapshot())
            if path.path in ('/api/session/start', '/api/session/stop') and body is not None:
                if not secrets.compare_digest(self.headers.get('X-CSRF-Token', ''), manager.csrf):
                    return self.send(403, {'error': 'invalid manager token'})
                try:
                    return self.send(202, manager.request(path.path.rsplit('/', 1)[1], json.loads(body)))
                except (ValueError, TypeError) as error:
                    return self.send(409, {'error': str(error)})
            with manager.lock:
                backend = manager.backend_port if manager.status == 'ready' and manager.remaining() > 0 else None
            static = {'/session': ('session.html', 'text/html; charset=utf-8'),
                      '/web/session.js': ('session.js', 'text/javascript'), '/web/style.css': ('style.css', 'text/css')}
            if path.path in static or (path.path == '/' and not backend):
                if body is not None:
                    return self.send(405, {'error': 'GET required'})
                name, kind = static.get(path.path, static['/session'])
                try:
                    return self.send(200, (Path(web_dir)/name).read_bytes(), kind)
                except OSError:
                    return self.send(503, {'error': 'manager page unavailable'})
            allowed = path.path in ('/', '/index.html', '/app.js', '/style.css', '/web/app.js',
                '/web/index.html', '/api/state', '/api/preview-media', '/api/calibration-file', '/api/result-media')
            allowed |= bool(re.fullmatch(r'/api/(?:actions/[a-z_]+|media/[A-Za-z0-9_-]+)', path.path))
            if not allowed:
                return self.send(404, {'error': 'not found'})
            if not backend:
                return self.send(503, {'error': 'fake 服务未就绪', 'session_url': '/session'})
            headers = {key: value for key, value in self.headers.items() if key.lower() in
                ('host', 'origin', 'x-csrf-token', 'content-type', 'content-length', 'range', 'if-none-match', 'if-modified-since')}
            connection = HTTPConnection('127.0.0.1', backend, timeout=15)
            started = False
            try:
                connection.request(self.command, self.path, body=body, headers=headers)
                response = connection.getresponse(); self.send_response(response.status); started = True
                lengths = response.headers.get_all('Content-Length', [])
                framed = (len(lengths) == 1 and lengths[0].isascii() and lengths[0].isdigit()
                          and response.headers.get('Transfer-Encoding') is None)
                self.close_connection = self.close_connection or not framed or response.status >= 400
                for key, value in response.getheaders():
                    if key.lower() not in ('connection', 'transfer-encoding', 'server', 'date'):
                        if key.lower() != 'content-length' or response.headers.get('Transfer-Encoding') is None:
                            self.send_header(key, value)
                if self.close_connection:
                    self.send_header('Connection', 'close')
                self.end_headers(); sent = 0
                while data := response.read(65536):
                    self.wfile.write(data); sent += len(data)
                if framed and sent != int(lengths[0]):
                    self.close_connection = True
            except (OSError, HTTPException, ValueError):
                self.close_connection = True
                if not started:
                    self.send(503, {'error': 'fake 服务暂时无法访问', 'session_url': '/session'})
            finally:
                connection.close()
        def do_GET(self):
            if not self.valid():
                return self.send(403, {'error': 'invalid Host'})
            if self.headers.get('Transfer-Encoding') is not None or self.headers.get_all('Content-Length', []) not in ([], ['0']):
                return self.send(409, {'error': 'GET request body is not supported'})
            self.route()
        def do_POST(self):
            keep_alive = not self.close_connection
            self.close_connection = True
            if not self.valid() or self.headers.get('Origin') not in (None, 'http://'+self.headers['Host']):
                return self.send(403, {'error': 'invalid Host or Origin'})
            lengths = self.headers.get_all('Content-Length', [])
            if (self.headers.get('Transfer-Encoding') is not None or len(lengths) != 1
                    or not lengths[0].isascii() or not lengths[0].isdigit()):
                return self.send(409, {'error': 'one Content-Length required; no Transfer-Encoding'})
            size = int(lengths[0])
            if not 0 < size <= 4*1024*1024:
                return self.send(413, {'error': 'small JSON required'})
            body = self.rfile.read(size)
            if len(body) != size:
                return self.send(409, {'error': 'incomplete request body'})
            self.close_connection = not keep_alive
            self.route(body)
    server = ThreadingHTTPServer(('127.0.0.1', port), Handler); server.daemon_threads = True
    server.allowed_hosts = allowed_hosts | loopback_browser_hosts([server.server_port])
    return server
