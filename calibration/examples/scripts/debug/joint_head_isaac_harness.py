#!/usr/bin/env python3
"""Loopback simulator experiment harness. The fixed worker never controls hardware."""
import argparse
from copy import deepcopy
import hashlib
from http.server import ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import secrets
import subprocess
import threading
from urllib.parse import urlsplit
import uuid
import xml.etree.ElementTree as ET

from joint_head_isaac_review import handler as review_handler, read_json, review


def encoded(value):
    return (json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False) + '\n').encode()


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def close_values(a, b, tolerance):
    if isinstance(a, list) and isinstance(b, list):
        return len(a) == len(b) and all(close_values(x, y, tolerance) for x, y in zip(a, b))
    return (type(a) in (int, float) and type(b) in (int, float)
            and math.isfinite(a) and math.isfinite(b) and abs(a-b) <= tolerance)


class App:
    def __init__(self, dataset, runner, jobs, template, fit_command=None):
        self.baseline = dataset.resolve(strict=True)
        self.active_root = self.baseline
        review(self.baseline)
        self.template_path = template.resolve(strict=True)
        self.runner_path = runner.resolve(strict=True)
        self.argv = read_json(self.runner_path)
        if not isinstance(self.argv, list) or not self.argv or not all(isinstance(a, str) and a for a in self.argv):
            raise ValueError('runner-command must be a JSON array of trusted argv strings')
        if not Path(self.argv[0]).is_absolute() or not Path(self.argv[0]).is_file():
            raise ValueError('worker executable must be an existing absolute path')
        self.fit_path = fit_command.resolve(strict=True) if fit_command else None
        self.fit_argv = read_json(self.fit_path) if self.fit_path else None
        if self.fit_argv is not None and (not isinstance(self.fit_argv, list) or not self.fit_argv
                or not all(isinstance(a, str) and a for a in self.fit_argv)
                or not Path(self.fit_argv[0]).is_absolute() or not Path(self.fit_argv[0]).is_file()):
            raise ValueError('fit-command requires trusted argv with an absolute executable')
        self.fit = None
        self.jobs = jobs.resolve()
        self.jobs.mkdir(parents=True, exist_ok=True)
        self.session = self.jobs / ('session-' + uuid.uuid4().hex[:12])
        self.session.mkdir(mode=0o700)
        self.csrf = secrets.token_urlsafe(32)
        self.lock = threading.RLock()
        self.plan = self.preview = self.execution = self.process = None
        self.record = self.log = None
        self.consumed = set()

    def fit_ready(self):
        if self.fit_argv is None:
            return False, 'CPU fit command is not configured'
        if self.process is not None:
            return False, 'a job is running'
        try:
            if read_json(self.active_root / 'completion.json').get('passed') is not True:
                return False, 'capture completion has not passed'
            frames = read_json(self.active_root / 'dataset.json')['frames']
            if not {'fit', 'holdout'} <= {f.get('split') for f in frames}:
                return False, 'both fit and holdout frames are required'
        except (OSError, ValueError, KeyError, TypeError) as exc:
            return False, str(exc)
        return True, None

    def start_fit(self, request):
        if request:
            raise ValueError('fit accepts no paths or command overrides')
        ready, reason = self.fit_ready()
        if not ready:
            raise ValueError(reason)
        job_id = 'fit-' + uuid.uuid4().hex[:12]
        output = self.active_root / (job_id + '.json')
        log = (self.session / (job_id + '.log')).open('wb')
        repo = Path(__file__).resolve().parents[3]
        environment = {**os.environ, 'CUDA_VISIBLE_DEVICES': '',
                       'PYTHONPATH': str(repo / 'infra') + os.pathsep + os.environ.get('PYTHONPATH', '')}
        try:
            process = subprocess.Popen([*self.fit_argv, str(self.active_root), '--output', str(output)],
                cwd=repo, env=environment, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                start_new_session=True)
        except BaseException:
            log.close()
            raise
        record = {'id': job_id, 'kind': 'fit', 'status': 'running', 'output': output,
                  'dataset': self.active_root, 'dataset_sha256': sha(self.active_root / 'dataset.json'), 'error': None}
        self.process, self.log, self.record, self.fit = process, log, record, record
        return self.state()

    def identities(self):
        return {str(p): sha(p) for p in [self.template_path, self.runner_path,
                self.baseline / 'dataset.json', self.baseline / 'initial-state.json', self.baseline / 'robot.urdf',
                *[Path(arg) for arg in self.argv if Path(arg).is_absolute() and Path(arg).is_file()]]}

    def create_plan(self, request):
        if self.process is not None:
            raise ValueError('simulation job running')
        plan = deepcopy(read_json(self.template_path))
        if request.get('template_batch') is True and set(request) == {'template_batch'}:
            kind = 'template_batch'
        elif set(request) == {'target_deg'}:
            kind = 'target'
            plan['stations'] = [{'id': 'requested-target', 'split': 'fit', 'q_deg': request['target_deg']}]
        else:
            raise ValueError('provide target_deg[12] or template_batch:true')
        data = read_json(self.baseline / 'dataset.json')
        names = data['joint_names']
        limits = {j.get('name'): j.find('limit') for j in ET.parse(self.baseline / 'robot.urdf').getroot().findall('joint')}
        current = data['frames'][0]['q_reported_rad']
        margin = math.radians(5)
        stations = plan.get('stations')
        if not isinstance(stations, list) or not stations or len(stations) > 60:
            raise ValueError('plan needs 1..60 stations')
        for row in stations:
            q = row.get('q_deg')
            if not isinstance(q, list) or len(q) != 12 or any(type(v) not in (int, float) or not math.isfinite(v) for v in q):
                raise ValueError('each target requires 12 finite numeric degrees')
            for i, value in enumerate(q):
                limit = limits[names[i]]
                lower, upper = float(limit.get('lower')), float(limit.get('upper'))
                target = math.radians(value)
                if not lower-margin <= current[i] <= upper+margin:
                    raise ValueError(f'reported start outside nominal allowance: {names[i]}')
                if abs(target-current[i]) > 1e-4 and not lower+margin <= target <= upper-margin:
                    raise ValueError(f'changed target needs 5 degree nominal limit margin: {names[i]}')
            current = [math.radians(v) for v in q]
        plan['save_preview'] = True
        raw = encoded(plan)
        identities = self.identities()
        digest = hashlib.sha256(raw + encoded(identities)).hexdigest()
        path = self.session / (digest + '.json')
        if path.exists():
            if path.read_bytes() != raw:
                raise ValueError('frozen plan artifact was modified')
        else:
            with path.open('xb') as stream:
                stream.write(raw)
            path.chmod(0o600)
        self.plan = {'hash': digest, 'path': path, 'file_sha256': sha(path), 'identities': identities,
                     'public': {'hash': digest, 'kind': kind, 'stations': select_stations(stations),
                                'start_q_reported_rad': data['frames'][0]['q_reported_rad'],
                                'start_mode': 'fresh simulation reset', 'joint_names': names,
                                'physical_limit_check_required': True,
                                'capture_runner_sha256': data.get('runner_sha256')}}
        self.preview = self.execution = None
        return self.state()

    def validate_plan(self, value):
        if self.plan is None or value != self.plan['hash']:
            raise ValueError('plan missing or stale')
        if self.identities() != self.plan['identities'] or sha(self.plan['path']) != self.plan['file_sha256']:
            raise ValueError('baseline, scene or frozen plan changed; plan again')

    def start(self, request, confirmed=False):
        if self.process is not None:
            raise ValueError('simulation job running')
        self.validate_plan(request.get('plan_hash'))
        if confirmed:
            if not self.preview or self.preview['status'] != 'passed' or request.get('preview_hash') != self.preview.get('preview_hash'):
                raise ValueError('exact completed preview confirmation required')
            if self.plan['hash'] in self.consumed:
                raise ValueError('confirmation already consumed; create a new plan')
            if self.verify(self.preview['output']) != self.preview['preview_hash']:
                raise ValueError('preview evidence changed')
        elif self.preview is not None:
            raise ValueError('preview already started; plan again to retry')
        role = 'execution' if confirmed else 'preview'
        parent = self.session / (role + '-' + uuid.uuid4().hex[:12])
        parent.mkdir()
        output = parent / 'output'
        log = (parent / 'worker.log').open('wb')
        try:
            process = subprocess.Popen([*self.argv, '--plan', str(self.plan['path']), '--output', str(output)],
                                       stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        except BaseException:
            log.close()
            raise
        record = {'id': parent.name, 'status': 'running', 'output': output, 'error': None}
        self.process, self.log, self.record = process, log, record
        setattr(self, role, record)
        if confirmed:
            self.consumed.add(self.plan['hash'])
        return self.state()

    def verify(self, root):
        if read_json(root / 'completion.json').get('passed') is not True:
            raise ValueError('simulator completion gate failed')
        if read_json(root / 'run.json').get('plan_sha256') != self.plan['file_sha256']:
            raise ValueError('worker did not run the exact frozen plan')
        baseline, actual = read_json(self.baseline / 'dataset.json'), read_json(root / 'dataset.json')
        if baseline['joint_names'] != actual['joint_names'] or sha(self.baseline / 'robot.urdf') != sha(root / 'robot.urdf'):
            raise ValueError('simulator model changed')
        if baseline.get('runner_sha256') != actual.get('runner_sha256'):
            raise ValueError('capture runner changed from preview baseline')
        if not close_values(baseline['frames'][0]['q_reported_rad'], actual['frames'][0]['q_reported_rad'], .01):
            raise ValueError('fresh-reset feedback differs from preview origin')
        a, b = read_json(self.baseline / 'initial-state.json'), read_json(root / 'initial-state.json')
        if a['joint_names'] != b['joint_names'] or not all(close_values(a[k], b[k], 1e-4) for k in ('joint_pos', 'camera_world')):
            raise ValueError('fresh-reset joints or camera changed')
        if not close_values(a['bodies_world']['base_link'], b['bodies_world']['base_link'], 1e-4):
            raise ValueError('fresh-reset base pose changed')
        frames = actual['frames'][1:]
        if not frames:
            raise ValueError('no captured target frame')
        for f in frames:
            error = f.get('tracking_error_rad')
            velocity = f.get('velocity_rad_s')
            if (type(error) not in (int, float) or not math.isfinite(error) or not 0 <= error < .01
                    or not isinstance(velocity, list) or len(velocity) != 12
                    or any(type(v) not in (int, float) or not math.isfinite(v) or abs(v) >= .01 for v in velocity)):
                raise ValueError('captured pose tracking/velocity gate failed')
        evidence = review(root)
        if not evidence['preview']['frames']:
            raise ValueError('missing rendered trajectory preview')
        files = ['dataset.json', 'preview.json', 'initial-state.json', 'completion.json', 'run.json', 'robot.urdf']
        files += [f['image'] for f in actual['frames']]
        files += read_json(root / 'preview.json')['frames']
        return hashlib.sha256(encoded({'plan': self.plan['hash'], 'files': {f: sha(root / f) for f in files}})).hexdigest()

    def poll(self):
        if self.process is None or self.process.poll() is None:
            return
        code = self.process.returncode
        self.log.close()
        try:
            if code != 0:
                raise ValueError(f'worker exited {code}; see local worker.log')
            if self.record.get('kind') == 'fit':
                result = read_json(self.record['output'])
                if (result.get('dataset_sha256') != self.record['dataset_sha256']
                        or sha(self.record['dataset'] / 'dataset.json') != self.record['dataset_sha256']
                        or result.get('used_truth_in_fit') is not False):
                    raise ValueError('fit report provenance check failed')
                self.record.update(status='completed', fit_status=result.get('status'), report_path=self.record['output'])
            else:
                self.record['preview_hash'] = self.verify(self.record['output'])
                self.record['status'] = 'passed'
                self.active_root = self.record['output']
        except (ValueError, OSError, KeyError, TypeError) as exc:
            self.record.update(status='failed', error=str(exc))
        self.process = self.log = self.record = None

    def state(self):
        self.poll()
        can_fit, fit_reason = self.fit_ready()
        return {'csrf_token': self.csrf, 'simulation_only': True, 'hardware_connected': False,
                'stop_available': False, 'busy': self.process is not None,
                'plan': self.plan['public'] if self.plan else None,
                'preview': public_job(self.preview), 'execution': public_job(self.execution),
                'fit': public_job(self.fit), 'can_fit': can_fit, 'fit_unavailable_reason': fit_reason,
                'initial_q_reported_rad': read_json(self.baseline / 'dataset.json')['frames'][0]['q_reported_rad'],
                'active_dataset': str(self.active_root)}


def select_stations(stations):
    return [{k: row[k] for k in ('id', 'split', 'q_deg') if k in row} for row in stations]


def public_job(job):
    return {k: str(v) if isinstance(v, Path) else v for k, v in job.items()} if job else None


def handler(app):
    class Handler(review_handler(lambda: app.active_root)):
        def do_GET(self):
            if urlsplit(self.path).path != '/api/state':
                return super().do_GET()
            if self.headers.get('Host') not in (f'127.0.0.1:{self.server.server_port}', f'localhost:{self.server.server_port}'):
                return self.send({'error': 'loopback Host required'}, 403)
            with app.lock:
                return self.send(app.state())

        def do_POST(self):
            try:
                host = self.headers.get('Host')
                if host not in (f'127.0.0.1:{self.server.server_port}', f'localhost:{self.server.server_port}') or self.headers.get('Origin') != 'http://' + host:
                    raise ValueError('same loopback origin required')
                if not secrets.compare_digest(self.headers.get('X-CSRF-Token', ''), app.csrf):
                    raise ValueError('CSRF token required')
                if self.headers.get('Content-Type', '').split(';')[0] != 'application/json':
                    raise ValueError('JSON required')
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length <= 8192:
                    raise ValueError('invalid request size')
                body = json.loads(self.rfile.read(length))
                if not isinstance(body, dict):
                    raise ValueError('JSON object required')
                with app.lock:
                    app.poll()
                    path = urlsplit(self.path).path
                    if path == '/api/plan':
                        return self.send(app.create_plan(body))
                    if path == '/api/fit':
                        return self.send(app.start_fit(body))
                    if path in ('/api/preview', '/api/confirm'):
                        return self.send(app.start(body, confirmed=path == '/api/confirm'))
                    if path == '/api/stop':
                        return self.send({'error': 'remote cancellation not integrated; local kill would not certify remote stop'}, 409)
                    return self.send({'error': 'not found'}, 404)
            except (ValueError, OSError, KeyError, TypeError) as exc:
                return self.send({'error': str(exc)}, 400)
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('dataset', 'runner-command', 'jobs', 'plan-template'):
        parser.add_argument('--' + name, type=Path, required=True)
    parser.add_argument('--port', type=int, default=8111)
    parser.add_argument('--fit-command', type=Path, help='trusted JSON argv; dataset and --output are appended')
    args = parser.parse_args()
    if not 1024 <= args.port <= 65535:
        parser.error('invalid port')
    app = App(args.dataset, args.runner_command, args.jobs, args.plan_template, args.fit_command)
    with ThreadingHTTPServer(('127.0.0.1', args.port), handler(app)) as server:
        print(f'SIMULATION ONLY: http://127.0.0.1:{args.port}', flush=True)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            print('UI closed. Active remote simulation is not stopped by closing this UI.', flush=True)


if __name__ == '__main__':
    main()
