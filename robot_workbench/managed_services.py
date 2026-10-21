"""Manage only trusted deployment processes and explicitly named user units.

Linux /proc identities are mandatory; another OS is reported as unknown. This
module does not connect SDKs, assess robot safety, discover services by name, or
kill on a timeout. The caller must release and verify control before stop. A
process service should keep its workers in its original session. Observed
descendants are checked after stop even if they detach; the caller can also pin
reported SDK/model children with track_child before releasing application state.
"""
from __future__ import annotations

from contextlib import contextmanager
from dataclasses import dataclass
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import tempfile
import threading
import time


PROC_ROOT = Path('/proc')
CGROUP_ROOT = Path('/sys/fs/cgroup')


@dataclass(frozen=True)
class ServiceSpec:
    name: str
    kind: str
    expected_script: str
    identity_path: str
    command: list[str] | tuple[str, ...] | None = None
    cwd: str | None = None
    unit: str | None = None
    start_timeout_s: float = 5.
    stop_timeout_s: float = 10.
    poll_s: float = .05
    env: dict[str, str] | None = None

    def __post_init__(self):
        if not isinstance(self.name, str) or not re.fullmatch(r'[a-z][a-z0-9_-]{0,63}', self.name):
            raise ValueError('fixed service name required')
        for path in (self.expected_script, self.identity_path):
            if not isinstance(path, str) or not Path(path).is_absolute() or '..' in Path(path).parts:
                raise ValueError('absolute deployment script and identity paths required')
        if self.expected_script == self.identity_path:
            raise ValueError('identity receipt cannot overwrite the deployment script')
        for value in (self.start_timeout_s, self.stop_timeout_s, self.poll_s):
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError('finite positive lifecycle timeouts required')
        if self.env is not None:
            if (not isinstance(self.env, dict) or any(not isinstance(key, str)
                    or not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', key)
                    or not isinstance(value, str) or '\0' in value for key, value in self.env.items())):
                raise ValueError('trusted process env must map valid variable names to NUL-free strings')
            object.__setattr__(self, 'env', dict(self.env))
        if self.kind == 'process':
            if (not isinstance(self.command, (list, tuple)) or not self.command
                    or not all(isinstance(arg, str) and arg and '\0' not in arg for arg in self.command)
                    or not Path(self.command[0]).is_absolute()
                    or not isinstance(self.cwd, str) or not Path(self.cwd).is_absolute()
                    or '..' in Path(self.cwd).parts or self.unit is not None):
                raise ValueError('process requires trusted argv and absolute cwd; no unit or shell')
            executable = Path(self.command[0]).name
            if executable in ('sh', 'bash', 'dash', 'zsh', 'ksh', 'fish', 'env', 'sudo'):
                raise ValueError('shells and command wrappers are not managed service commands')
            if _script_argument(self.command) != self.expected_script:
                raise ValueError('command must execute the exact expected_script, not inline code or a module')
            object.__setattr__(self, 'command', tuple(self.command))
        elif self.kind == 'user_unit':
            if (not isinstance(self.unit, str) or not re.fullmatch(r'[A-Za-z0-9_][A-Za-z0-9_.@-]*\.service', self.unit)
                    or self.command is not None or self.cwd is not None or self.env is not None):
                raise ValueError('user_unit requires an exact service unit; command/cwd/env overrides are forbidden')
        else:
            raise ValueError('service kind must be process or user_unit')


def _script_argument(argv):
    """An exact executable or Python script, never an argument to -c or a shell."""
    if not argv:
        return None
    if re.fullmatch(r'python(?:\d+(?:\.\d+)*)?', Path(argv[0]).name):
        index = 1
        while index < len(argv) and argv[index] in ('-B', '-u', '-E', '-I', '-s', '-S', '-O', '-OO'):
            index += 1
        if index < len(argv) and not argv[index].startswith('-'):
            return argv[index]
        return None
    return argv[0]


def _boot_id():
    if not (PROC_ROOT/'self/stat').exists():
        raise OSError('Linux /proc identity verification is unavailable; no lifecycle action was taken')
    value = (PROC_ROOT/'sys/kernel/random/boot_id').read_text().strip()
    if not value:
        raise OSError('kernel boot identity is unavailable')
    return value


def _stat(pid):
    directory = PROC_ROOT/str(pid)
    fields = (directory/'stat').read_text().rsplit(')', 1)[1].split()
    return dict(pid=pid, start_ticks=fields[19], ppid=int(fields[1]), pgid=int(fields[2]), sid=int(fields[3]),
                process_state=fields[0], uid=directory.stat().st_uid)


def process_identity(pid):
    """Return a real Linux identity, None for a missing PID, or raise if unknown."""
    if type(pid) is not int or pid <= 0:
        raise ValueError('positive integer PID required')
    boot = _boot_id()
    try:
        before = _stat(pid)
        argv = [os.fsdecode(part) for part in (PROC_ROOT/str(pid)/'cmdline').read_bytes().split(b'\0') if part]
        after = _stat(pid)
    except FileNotFoundError:
        # A command line can disappear while a process exits. Confirm the PID
        # itself vanished; missing/inaccessible cmdline is otherwise unknown.
        if not (PROC_ROOT/str(pid)).exists():
            return None
        raise
    if any(before[key] != after[key] for key in ('start_ticks', 'pgid', 'sid', 'uid')):
        raise OSError('process identity changed during verification')
    return dict(after, boot_id=boot, script=_script_argument(argv))


def _live(identity):
    return identity is not None and identity['process_state'] not in ('Z', 'X', 'x')


def _matches(current, recorded):
    return current is not None and all(current.get(key) == recorded.get(key)
        for key in ('pid', 'start_ticks', 'pgid', 'sid', 'uid', 'boot_id', 'script'))


class ManagedService:
    def __init__(self, spec: ServiceSpec):
        if not isinstance(spec, ServiceSpec):
            raise TypeError('a validated ServiceSpec is required')
        self.spec = spec
        self.path = Path(spec.identity_path)
        self.child = None
        self._mutation = threading.Lock()
        self.fingerprint = hashlib.sha256(json.dumps(dict(name=spec.name, kind=spec.kind,
            expected_script=spec.expected_script, command=spec.command, cwd=spec.cwd, unit=spec.unit, env=spec.env),
            sort_keys=True).encode()).hexdigest()

    def _view(self, state, reason, *, identity=None, owned=False, running=None, **extra):
        return dict(name=self.spec.name, kind=self.spec.kind, state=state, running=running,
                    identity=identity, owned=owned, reason=reason, **extra)

    def _unknown(self, reason, **extra):
        return self._view('unknown', str(reason), **extra)

    def _load(self):
        try:
            fd = os.open(self.path, os.O_RDONLY | os.O_NOFOLLOW)
        except FileNotFoundError:
            return None
        with os.fdopen(fd) as stream:
            raw = stream.read(65537)
        if len(raw) > 65536:
            raise ValueError('service identity receipt exceeds its size limit')
        value = json.loads(raw)
        if (not isinstance(value, dict) or value.get('schema') != 'robot_workbench.managed_service.v1'
                or value.get('spec_sha256') != self.fingerprint):
            raise ValueError('service receipt does not match this deployment; it was not overwritten')
        identity = value.get('identity')
        if identity is not None and (not isinstance(identity, dict)
                or type(identity.get('pid')) is not int or identity['pid'] <= 0
                or not isinstance(identity.get('start_ticks'), str) or not identity['start_ticks'].isdigit()
                or any(type(identity.get(key)) is not int for key in ('pgid', 'sid', 'uid'))
                or identity.get('script') != self.spec.expected_script or not identity.get('boot_id')):
            raise ValueError('invalid saved process identity')
        children=value.get('children',[])
        if (not isinstance(children,list) or any(not isinstance(row,dict)
                or type(row.get('pid')) is not int or row['pid']<=0
                or not isinstance(row.get('start_ticks'),str) or not row['start_ticks'].isdigit()
                or any(type(row.get(key)) is not int for key in ('pgid','sid','uid'))
                or not row.get('boot_id') for row in children)):
            raise ValueError('invalid saved child process identities')
        return value

    def _save(self, phase, identity, reason='', *, children=None):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        if self.path.is_symlink():
            raise ValueError('service identity receipt must not be a symlink')
        if children is None:children=(self._load() or {}).get('children',[])
        value = dict(schema='robot_workbench.managed_service.v1', spec_sha256=self.fingerprint,
                     phase=phase, identity=identity,children=children, reason=reason, updated_at_ns=time.time_ns())
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(mode='w', dir=self.path.parent, prefix='.service-',
                                             delete=False, encoding='utf-8') as stream:
                temporary = Path(stream.name)
                json.dump(value, stream, allow_nan=False, indent=2)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, self.path)
            directory_fd = os.open(self.path.parent, os.O_RDONLY)
            try:
                os.fsync(directory_fd)
            finally:
                os.close(directory_fd)
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)

    @contextmanager
    def _locked(self):
        if not self._mutation.acquire(blocking=False):
            raise OSError('another lifecycle operation is in progress')
        fd = None
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            fd = os.open(str(self.path)+'.lock', os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            yield
        finally:
            if fd is not None:
                os.close(fd)
            self._mutation.release()

    def _members(self, identity):
        """Observe the recorded session/group; never discover ownership by name."""
        members = []
        if _boot_id() != identity['boot_id']:
            return members
        for entry in PROC_ROOT.iterdir():
            if not entry.name.isdigit():
                continue
            try:
                row = _stat(int(entry.name))
            except FileNotFoundError:
                continue
            if _live(row) and (row['pgid'] == identity['pgid'] or row['sid'] == identity['sid']):
                members.append(row)
        return members

    def _tracked_live(self,receipt):
        live=[]
        for row in (receipt or {}).get('children',[]):
            if self._historical_live(row):live.append(row)
        return live

    def _descendants(self,identity):
        rows={}
        for entry in PROC_ROOT.iterdir():
            if not entry.name.isdigit():continue
            try:rows[int(entry.name)]=_stat(int(entry.name))
            except FileNotFoundError:continue
        parents={identity['pid']}
        while True:
            children={pid for pid,row in rows.items() if row['ppid'] in parents
                      or row['sid']==identity['sid'] or row['pgid']==identity['pgid']}
            expanded=parents|children
            if expanded==parents:break
            parents=expanded
        result=[]
        for pid in parents-{identity['pid']}:
            current=process_identity(pid)
            if _live(current):
                if current['uid']!=os.getuid():raise OSError('owned descendant changed user identity')
                result.append(current)
        return result

    def _unit_members(self,unit):
        group=unit.get('ControlGroup')
        if not group:
            if not unit['MainPID'] and unit['ActiveState'] in ('inactive','failed'):return []
            raise OSError('unit ControlGroup is unavailable; descendant exit cannot be verified')
        if not group.startswith('/') or group=='/' or '..' in Path(group).parts:raise OSError('invalid unit ControlGroup')
        path=CGROUP_ROOT/group.lstrip('/')/'cgroup.procs'
        try:
            pids=set(path.read_text().split())
            for nested in path.parent.rglob('cgroup.procs'):
                if nested!=path:pids.update(nested.read_text().split())
        except FileNotFoundError:
            if not unit['MainPID'] and unit['ActiveState'] in ('inactive','failed'):return []
            raise
        result=[]
        for raw in pids:
            if not raw.isdigit():raise OSError('invalid unit cgroup process identity')
            row=process_identity(int(raw))
            if _live(row):
                if row['uid']!=os.getuid():raise OSError('unit cgroup includes a different user')
                result.append(row)
        return result

    def track_child(self,pid):
        """Record a reported SDK/model child only after proving current ownership."""
        def action():
            state=self.snapshot()
            if state['state']!='running' or not state['owned']:return self._unknown('service identity unavailable for child tracking')
            identity=state['identity']
            if pid==identity['pid']:return state
            candidates=(self._unit_members(state['unit_state']) if self.spec.kind=='user_unit' else self._descendants(identity))
            child=next((row for row in candidates if row['pid']==pid),None)
            if child is None:return self._unknown('reported child is not in this owned process tree or unit cgroup')
            children=(self._load() or {}).get('children',[])
            for row in children:
                if row['pid']==pid and not _matches(row,child):return self._unknown('recorded child PID changed identity')
            if not any(_matches(row,child) for row in children):children.append(child)
            self._save('running',identity,children=children)
            return dict(state,tracked_child=child)
        return self._operate(action)

    def _process_snapshot(self, receipt):
        if receipt is None:
            return self._view('stopped', 'No process has been registered by this service manager.', running=False)
        identity = receipt.get('identity')
        if identity is None:
            return self._unknown(receipt.get('reason') or 'A prior start has no verified PID identity; inspect it before retrying.')
        if identity['boot_id'] != _boot_id():
            return self._view('stopped', 'The recorded process belongs to a previous kernel boot.', identity=identity, running=False)
        current = process_identity(identity['pid'])
        if current is not None and not _matches(current, identity):
            return self._unknown('Recorded PID identity changed; the live process was not adopted or signalled.', identity=current, running=_live(current))
        members = self._members(identity)
        children=self._tracked_live(receipt)
        if not _live(current):
            if members or children:
                return self._unknown('Recorded leader exited but its group/session or tracked children are still alive.', identity=identity, running=True,children=children)
            return self._view('stopped', 'Recorded process and session have exited.', identity=identity, running=False, owned=True)
        if (current['uid'] != os.getuid() or current['script'] != self.spec.expected_script
                or current['pgid'] != current['pid'] or current['sid'] != current['pid']):
            return self._unknown('Process is not the registered user-owned session leader.', identity=current, running=True)
        if any(row['uid'] != current['uid'] or row['sid'] != current['sid'] for row in members):
            return self._unknown('Process group contains an identity outside the registered user/session.', identity=current, running=True)
        if receipt.get('phase') in ('stopping', 'unknown'):
            return self._unknown(receipt.get('reason') or 'The previous shutdown has not completed.', identity=current, owned=True, running=True)
        return self._view('running', 'Registered process identity is live; application health is checked separately.', identity=current, owned=True, running=True)

    def _systemctl(self, *args):
        result = subprocess.run(['systemctl', '--user', *args], stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=min(10., self.spec.start_timeout_s), check=False)
        if result.returncode:
            raise OSError('systemctl could not verify or operate the configured user unit (exit '+str(result.returncode)+')')
        return result.stdout

    def _unit_state(self):
        properties = 'MainPID,ControlPID,ActiveState,SubState,LoadState,SendSIGKILL,KillSignal,ControlGroup,KillMode'
        text = self._systemctl('show', '--no-pager', '--property='+properties, '--', self.spec.unit)
        values = dict(line.split('=', 1) for line in text.splitlines() if '=' in line)
        if (values.get('LoadState') != 'loaded' or not values.get('MainPID', '').isdigit()
                or not values.get('ControlPID', '').isdigit() or 'ActiveState' not in values):
            raise OSError('configured user unit identity/status is unavailable')
        values['MainPID'] = int(values['MainPID'])
        values['ControlPID'] = int(values['ControlPID'])
        return values

    def _historical_live(self, identity):
        if identity is None or identity['boot_id'] != _boot_id():
            return False
        current = process_identity(identity['pid'])
        if current is not None and not _matches(current, identity):
            raise OSError('historical unit PID changed identity; no process was signalled')
        return _live(current)

    def _unit_snapshot(self, receipt):
        unit = self._unit_state()
        historical = (receipt or {}).get('identity')
        members=self._unit_members(unit);children=self._tracked_live(receipt)
        if unit['MainPID'] == 0:
            if self._historical_live(historical) or members or children:
                return self._unknown('Unit has no MainPID but a recorded process or cgroup child is still alive.', identity=historical, running=True, unit_state=unit,children=children)
            if unit['ControlPID'] or unit['ActiveState'] not in ('inactive', 'failed'):
                return self._unknown('Unit is transitioning or has an unverified control process.', unit_state=unit)
            if (receipt or {}).get('phase') == 'starting':
                return self._unknown('A prior unit start has no verified outcome; inspect the unit job before retrying.', unit_state=unit)
            return self._view('stopped', 'Unit is inactive and its recorded process has exited.', identity=historical, running=False, unit_state=unit)
        current = process_identity(unit['MainPID'])
        if (not _live(current) or current['uid'] != os.getuid()
                or current['script'] != self.spec.expected_script):
            return self._unknown('Unit MainPID does not match the configured user and script.', identity=current, running=_live(current), unit_state=unit)
        if historical is not None and not _matches(current, historical) and (self._historical_live(historical) or children):
            return self._unknown('Previous recorded unit process is still live; replacement identity was not adopted.', identity=current, running=True, unit_state=unit)
        if unit['ActiveState'] != 'active' or unit['ControlPID']:
            return self._unknown('Unit transition has not completed.', identity=current, owned=True, running=True, unit_state=unit)
        return self._view('running', 'Explicit user unit and MainPID script verified; application health is separate.',
                          identity=current, owned=True, running=True, unit_state=unit)

    def snapshot(self):
        try:
            _boot_id()
            if self.child is not None:
                self.child.poll()
            receipt = self._load()
            return self._process_snapshot(receipt) if self.spec.kind == 'process' else self._unit_snapshot(receipt)
        except (OSError, ValueError, IndexError, KeyError, subprocess.SubprocessError) as exc:
            return self._unknown(str(exc))

    def _start(self):
        before = self.snapshot()
        if before['state'] == 'running' and before['owned']:
            self._save('running', before['identity'])
            return dict(before, reused=True)
        if before['state'] != 'stopped':
            return before
        if not Path(self.spec.expected_script).is_file():
            return self._unknown('Configured deployment script is missing; no process was started.')
        if self.spec.kind == 'process' and not Path(self.spec.cwd).is_dir():
            return self._unknown('Configured working directory is unavailable; no process was started.')
        # Clear an old exited PID before spawning: a crash between spawn and
        # identity capture must never make a new child look like the old exit.
        self._save('starting', None, 'Start admitted; a verified running identity has not been recorded yet.',children=[])
        if self.spec.kind == 'user_unit':
            self._systemctl('start', '--no-block', '--', self.spec.unit)
        else:
            log_fd = os.open(str(self.path)+'.log', os.O_CREAT | os.O_APPEND | os.O_WRONLY | os.O_NOFOLLOW, 0o600)
            try:
                self.child = subprocess.Popen(self.spec.command, cwd=self.spec.cwd, shell=False,
                    env=dict(os.environ, **self.spec.env) if self.spec.env is not None else None,
                    stdin=subprocess.DEVNULL, stdout=log_fd, stderr=subprocess.STDOUT, start_new_session=True)
            finally:
                os.close(log_fd)
        deadline = time.monotonic()+self.spec.start_timeout_s
        while time.monotonic() < deadline:
            if self.spec.kind == 'process':
                self.child.poll()
                identity = process_identity(self.child.pid)
                if (_live(identity) and identity['script'] == self.spec.expected_script
                        and identity['uid'] == os.getuid() and identity['pgid'] == self.child.pid and identity['sid'] == self.child.pid):
                    self._save('running', identity)
                    return self.snapshot()
                if self.child.returncode is not None:
                    return self._unknown('Started process exited before its identity could be verified.')
            else:
                observed = self.snapshot()
                if observed['state'] == 'running' and observed['owned']:
                    self._save('running', observed['identity'])
                    return observed
            time.sleep(self.spec.poll_s)
        return self._unknown('Start deadline passed without a verified identity; no process was killed and no retry was started.')

    def _stop(self):
        before = self.snapshot()
        if before['state'] == 'stopped':
            return dict(before,exit_verified=True,children_exited=(self._load() or {}).get('children',[]))
        if before['state'] != 'running' or not before['owned']:
            return before
        identity = before['identity']
        if self.spec.kind == 'user_unit':
            unit = before['unit_state']
            if unit.get('SendSIGKILL') != 'no' or unit.get('KillSignal') not in ('SIGTERM', '15'):
                return self._unknown('Unit stop policy must explicitly use SIGTERM and SendSIGKILL=no; unit configuration was not changed.', identity=identity, owned=True, running=True)
        observed=(self._unit_members(before['unit_state']) if self.spec.kind=='user_unit' else self._descendants(identity))
        children=(self._load() or {}).get('children',[])
        for child in observed:
            if child['pid']==identity['pid']:continue
            if any(row['pid']==child['pid'] and not _matches(row,child) for row in children):
                return self._unknown('recorded child PID changed identity before stop')
            if not any(_matches(row,child) for row in children):children.append(child)
        self._save('stopping', identity, 'TERM requested; parent and child exits must still be verified.',children=children)
        current = process_identity(identity['pid'])
        if not _matches(current, identity):
            return self._unknown('Process identity changed immediately before stop; nothing was signalled.', identity=current, running=_live(current))
        if self.spec.kind == 'process':
            try:
                os.killpg(identity['pgid'], signal.SIGTERM)
            except ProcessLookupError:
                pass
        else:
            latest = self._unit_state()
            if (latest['MainPID'] != identity['pid'] or latest.get('SendSIGKILL') != 'no'
                    or latest.get('KillSignal') not in ('SIGTERM', '15')
                    or latest.get('ControlGroup')!=before['unit_state'].get('ControlGroup')):
                return self._unknown('Unit identity or stop policy changed; no stop was submitted.', identity=identity)
            self._systemctl('stop', '--no-block', '--', self.spec.unit)
        deadline = time.monotonic()+self.spec.stop_timeout_s
        while time.monotonic() < deadline:
            observed = self.snapshot()
            if observed['state'] == 'stopped':
                self._save('stopped', identity, 'Recorded process exit verified.')
                return dict(observed,exit_verified=True,children_exited=children)
            time.sleep(self.spec.poll_s)
        self._save('unknown', identity, 'TERM/stop deadline passed; process exit is not verified. No SIGKILL was sent.')
        return self._unknown('TERM/stop deadline passed; process exit is not verified. No SIGKILL was sent.', identity=identity,
                             owned=True, running=self.snapshot()['running'])

    def _operate(self, action):
        try:
            with self._locked():
                return action()
        except (OSError, ValueError, IndexError, KeyError, subprocess.SubprocessError) as exc:
            return self._unknown(str(exc))

    def start(self):
        return self._operate(self._start)

    def stop(self):
        return self._operate(self._stop)

    def restart(self, progress=None):
        def action():
            if progress:
                progress('stopping_service', .1, {'service': self.spec.name})
            stopped = self._stop()
            if stopped['state'] != 'stopped':
                return stopped
            if progress:
                progress('starting_service', .6, {'service': self.spec.name})
            return dict(self._start(),stop_receipt=stopped)
        return self._operate(action)
