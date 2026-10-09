"""Bring the DSW wrist views up from the fixed nominal fixture.

Why this exists
---------------
The renderer creates its `left`/`right` views only when a wrist contract arrives,
and the contract's *cameras* are what those views serve. The workbench only ever
pushes a contract that came from a solved wrist result, so after a renderer restart
with no usable result there is no way back: the hardware wrist cameras have no
pixel source (HTTP 503) and the wrist half of the pipeline is dead.

This pushes a **nominal fixture** contract instead - the same shape the workbench
builds at plan time (frozen head snapshot + rectified wrist intrinsics + nominal
mounts + the seed's planned frames). The wrist camera poses come from the renderer's
own fixed `wrist_nominal_mounts` fixture, so nothing about a real calibration is
invented here; this only restores what the renderer needs to render at all.

It is the missing cold-start step of the bootstrap chain (architecture §7.6), and it
is an operator action, not part of any automatic flow.

Usage (on the machine that runs the workbench):

  PYTHONPATH=<repo>/infra:<repo> python3 joint_head_wrist_fixture_bringup.py \
      --config <run>/config.json --head <run>/calibrations/hardware/v000012.json \
      --seed <run>/configs/wrist-motion-seed-v5.json
"""
import argparse
import json
import sys
import time
import urllib.error
import urllib.request
from importlib import util
from pathlib import Path


def workbench_api(host):
    def get(path):
        with urllib.request.urlopen(host + path, timeout=30) as response:
            return json.loads(response.read())

    def post(path, body):
        state = get('/api/state')
        request = urllib.request.Request(host + path, data=json.dumps(body).encode(),
                                         headers={'Content-Type': 'application/json',
                                                  'X-CSRF-Token': state['csrf_token'],
                                                  'Origin': host})
        try:
            with urllib.request.urlopen(request, timeout=180) as response:
                return response.status, json.loads(response.read())
        except urllib.error.HTTPError as error:
            return error.code, json.loads(error.read())

    return get, post


def renderer_post(port, token_file, path, body):
    token = Path(token_file).read_text().strip()
    request = urllib.request.Request('http://127.0.0.1:%d%s' % (port, path), data=json.dumps(body).encode(),
                                     headers={'Host': '127.0.0.1:%d' % port, 'Origin': 'http://127.0.0.1:%d' % port,
                                              'Authorization': 'Bearer ' + token,
                                              'Content-Type': 'application/json'})
    with urllib.request.urlopen(request, timeout=60) as response:
        return response.status, response.read()


def load_workbench(path):
    spec = util.spec_from_file_location('joint_head_workbench', path)
    module = util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def nominal_fixture_contract(config):
    """Cold-start fake camera pixels from CAD; never registers a calibration."""
    from joint_head_workbench import Workbench
    candidate=json.loads(Path(config['candidate']).read_text())
    snapshot=Workbench.calibration_snapshot({
        'id':'nominal-fixture-'+config.get('asset_geometry_id','legacy'),
        'camera_T_world_optical':candidate['T_world_optical'],
        'joint_offsets_rad':{s:[0.]*6 for s in ('left','right')},
        'T_link6_marker':candidate['T_link6_marker']})
    seed=json.loads(Path(config['wrist_seed_file']).read_text())
    return {'head_snapshot':snapshot,'solver':'hybrid','display_only':True,
            'cameras':{s:{**config['wrist_simulation_cameras'][s],'rectified':True,'coefficients':[0.]*5} for s in ('left','right')},
            'nominal_mounts':config['wrist_nominal_mounts'],
            'planned_frames':[{k:w[k] for k in ('id','observer','target','split')} for w in seed['waypoints'] if w.get('capture') is not False],
            'table_z_m':0.,'table_clearance_m':.005,'marker_size_m':config.get('marker_size_m',.015),
            'geometry_id':'nominal-fixture','asset_geometry_id':config.get('asset_geometry_id'),
            'environment':'hardware','robot_source':'simulated','simulation_only':True}


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--config', required=True, help='run config.json')
    parser.add_argument('--head', required=True, help='registered head version JSON to freeze into the contract')
    parser.add_argument('--seed', required=True, help='wrist motion seed whose capture poses become planned_frames')
    parser.add_argument('--workbench', default='http://127.0.0.1:8117')
    parser.add_argument('--renderer-port', type=int, default=8118)
    parser.add_argument('--token-file', help='renderer bearer token (defaults to the config token file)')
    parser.add_argument('--workbench-script', help='path to joint_head_workbench.py (defaults to this directory)')
    args = parser.parse_args()

    config = json.loads(Path(args.config).read_text())
    record = json.loads(Path(args.head).read_text())
    row = (record.get('payload') or {}).get('row') or {}
    parameters = row.get('parameters')
    if not parameters:
        raise SystemExit('no parameters in ' + args.head)
    seed = json.loads(Path(args.seed).read_text())
    script = Path(args.workbench_script) if args.workbench_script else Path(__file__).with_name('joint_head_workbench.py')
    snapshot = load_workbench(script).Workbench.calibration_snapshot(parameters)
    print('head snapshot %s complete=%s sha=%s' % (snapshot['id'][:12], snapshot['snapshot_complete'],
                                                   snapshot['snapshot_sha256'][:12]))
    if not snapshot['snapshot_complete']:
        raise SystemExit('the frozen head snapshot is incomplete; the renderer will reject the contract')

    cameras = {}
    for side in ('left', 'right'):
        camera = dict(config['wrist_simulation_cameras'][side])
        camera.pop('serial', None)
        cameras[side] = {**camera, 'rectified': True, 'coefficients': [0.] * 5,
                         'distortion_model': 'distortion.inverse_brown_conrady'}
    planned = [{key: w[key] for key in ('id', 'observer', 'target', 'split')}
               for w in seed['waypoints'] if w.get('capture') is not False]
    contract = {'head_snapshot': snapshot, 'solver': 'hybrid', 'cameras': cameras,
                'nominal_mounts': config['wrist_nominal_mounts'], 'planned_frames': planned,
                'table_z_m': 0., 'table_clearance_m': .005, 'marker_size_m': config.get('marker_size_m', .015),
                'geometry_id': 'nominal-fixture', 'environment': 'hardware',
                'robot_source': 'simulated', 'simulation_only': True}

    get, post = workbench_api(args.workbench)
    state = get('/api/state')
    previous = state['control_mode']
    print('workbench control_mode=%s dsw=%s' % (previous, state.get('dsw_connection')))
    if previous != 'simulation':
        # The renderer accepts a wrist contract only while it is idle with no mirror.
        print('switch to simulation:', post('/api/control-mode', {'mode': 'simulation'})[0])
        time.sleep(4)
    try:
        token_file = args.token_file or config['live_backend']['token_file']
        code, body = renderer_post(args.renderer_port, token_file, '/wrist-config', {'wrist_contract': contract})
        print('wrist-config http=%d %s' % (code, body.decode('utf-8', 'replace')[:160]))
    finally:
        if previous != 'simulation':
            time.sleep(2)
            print('switch back to %s: %s' % (previous, post('/api/control-mode', {'mode': previous})[0]))
    for _ in range(8):
        state = get('/api/state')
        remote = state.get('remote') or {}
        print('  mode=%s status=%s wrist_contract=%s' % (state['control_mode'], remote.get('status'),
                                                         str(remote.get('wrist_contract_sha256'))[:12]))
        if remote.get('wrist_contract_sha256'):
            break
        time.sleep(3)
    return 0


if __name__ == '__main__':
    sys.exit(main())
