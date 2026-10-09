"""Local contract checks only; these bytes are not simulator evidence."""
import http.client
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
from http.server import ThreadingHTTPServer

SCRIPTS = Path(__file__).resolve().parents[1] / 'examples/scripts/debug'
sys.path.insert(0, str(SCRIPTS))
from joint_head_isaac_harness import App, handler


def test_frozen_simulator_jobs():
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp).resolve()
        base = root / 'baseline'
        base.mkdir()
        names = [f'joint{i}' for i in range(12)]
        def write(path, value):
            path.write_text(json.dumps(value))
        matrix = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
        write(base / 'dataset.json', {'schema': 'joint-head-isaac-v1', 'joint_names': names,
            'runner_sha256': 'unit-test-runner', 'frames': [{'id': 'initial', 'image': 'initial.png', 'q_reported_rad': [0]*12}]})
        write(base / 'initial-state.json', {'joint_names': names, 'joint_pos': [0]*12, 'camera_world': matrix,
                                           'bodies_world': {'base_link': matrix}})
        (base / 'robot.urdf').write_text('<robot>' + ''.join(f'<joint name="{n}"><limit lower="-3" upper="3"/></joint>' for n in names) + '</robot>')
        (base / 'initial.png').write_bytes(b'not-an-image-test-only')
        template = root / 'template.json'
        write(template, {'joint_offsets_deg': [3]*12, 'stations': [{'id': 'target', 'q_deg': [10]*12}]})
        worker = root / 'worker.py'
        worker.write_text('''import argparse,hashlib,json,shutil
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--baseline');p.add_argument('--plan');p.add_argument('--output');a=p.parse_args()
out=Path(a.output);shutil.copytree(a.baseline,out)
d=json.loads((out/'dataset.json').read_text());d['frames'].append({'id':'target','image':'initial.png','q_reported_rad':[0]*12,'tracking_error_rad':0.,'velocity_rad_s':[0]*12});(out/'dataset.json').write_text(json.dumps(d))
(out/'preview.json').write_text(json.dumps({'frames':['initial.png'],'fps':2}))
(out/'completion.json').write_text('{"passed":true}')
(out/'run.json').write_text(json.dumps({'plan_sha256':hashlib.sha256(Path(a.plan).read_bytes()).hexdigest()}))
''')
        runner = root / 'argv.json'
        write(runner, [sys.executable, str(worker), '--baseline', str(base)])
        fitter = root / 'fitter.py'
        fitter.write_text('''import argparse,hashlib,json,os
from pathlib import Path
assert os.environ['CUDA_VISIBLE_DEVICES']==''
p=argparse.ArgumentParser();p.add_argument('dataset');p.add_argument('--output');a=p.parse_args()
Path(a.output).write_text(json.dumps({'status':'IMAGE_FIT_FAIL','used_truth_in_fit':False,'dataset_sha256':hashlib.sha256((Path(a.dataset)/'dataset.json').read_bytes()).hexdigest()}))
''')
        fit_command = root / 'fit-argv.json'
        write(fit_command, [sys.executable, str(fitter)])
        app = App(base, runner, root / 'jobs', template, fit_command)
        assert not app.fit_ready()[0]
        state = app.create_plan({'target_deg': [10]*12})
        assert 'joint_offsets_deg' not in json.dumps(state)
        seal = state['plan']['hash']
        try:
            app.start({'plan_hash': seal, 'preview_hash': 'bad'}, confirmed=True)
            assert False, 'confirm before preview accepted'
        except ValueError:
            pass
        app.start({'plan_hash': seal})
        for _ in range(100):
            state = app.state()
            if not state['busy']:
                break
            time.sleep(.02)
        assert state['preview']['status'] == 'passed', state
        app.start({'plan_hash': seal, 'preview_hash': state['preview']['preview_hash']}, confirmed=True)
        for _ in range(100):
            state = app.state()
            if not state['busy']:
                break
            time.sleep(.02)
        assert state['execution']['status'] == 'passed', state
        try:
            app.start({'plan_hash': seal, 'preview_hash': state['preview']['preview_hash']}, confirmed=True)
            assert False, 'reused confirmation accepted'
        except ValueError:
            pass
        (base / 'initial-state.json').write_text('{}')
        try:
            app.validate_plan(seal)
            assert False, 'baseline drift accepted'
        except ValueError:
            pass
        server = ThreadingHTTPServer(('127.0.0.1', 0), handler(app))
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            conn = http.client.HTTPConnection('127.0.0.1', server.server_port)
            conn.request('GET', '/api/state')
            response = conn.getresponse()
            assert response.status == 200
            token = json.loads(response.read())['csrf_token']
            body = json.dumps({'target_deg': [10]*12})
            origin = f'http://127.0.0.1:{server.server_port}'
            for headers in [{'Content-Type': 'application/json', 'Origin': origin},
                            {'Content-Type': 'application/json', 'Origin': 'http://evil.example', 'X-CSRF-Token': token}]:
                conn.request('POST', '/api/plan', body, headers)
                response = conn.getresponse()
                assert response.status == 400
                response.read()
            fit_data = json.loads((app.active_root / 'dataset.json').read_text())
            fit_data['frames'][0]['split'] = 'fit'
            fit_data['frames'][1]['split'] = 'holdout'
            write(app.active_root / 'dataset.json', fit_data)
            headers = {'Content-Type': 'application/json', 'Origin': origin, 'X-CSRF-Token': token}
            conn.request('POST', '/api/fit', json.dumps({'dataset': '/tmp/untrusted'}), headers)
            response = conn.getresponse()
            assert response.status == 400
            response.read()
            conn.request('POST', '/api/fit', '{}', headers)
            response = conn.getresponse()
            assert response.status == 200
            response.read()
            for _ in range(100):
                state = app.state()
                if not state['busy']:
                    break
                time.sleep(.02)
            assert state['fit']['status'] == 'completed', state
            assert state['fit']['fit_status'] == 'IMAGE_FIT_FAIL'
            assert Path(state['fit']['report_path']).is_file()
            conn.close()
        finally:
            server.shutdown()
            server.server_close()
            thread.join()


if __name__ == '__main__':
    test_frozen_simulator_jobs()
    print('PASS: frozen plan, two subprocess stages, truth isolation, drift, replay, CSRF and CPU fit HTTP contract')
