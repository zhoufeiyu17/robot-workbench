"""Print a pinned same-device fake calibration source; read-only, no GPU.

Run on the host containing the source run. Pass the JSON to
serve_physx_device.py --calibration-resume-source. Startup validates it again.
"""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from robot_workbench.calibration_profile import _json, _resume_files, _sha


def source_spec(root, base):
    root, base = Path(root), Path(base)
    if not root.is_absolute() or not base.is_absolute() or root.is_symlink():
        raise ValueError('absolute, independent source paths required')
    config_path = root / 'calibration-config.json'
    config = _json(config_path)
    if (config.get('environment') != 'fake' or config.get('simulation_only') is not True
            or config.get('robot_source') != 'simulated'
            or Path(config['calibration_root']) != root / 'calibrations'):
        raise ValueError('resident fake calibration source required')
    active = root / 'calibrations/hardware/active'
    records = {kind: _json(active / (kind + '.json')) for kind in ('head', 'wrist', 'manifest')}
    if (records['wrist']['dependency_id'] != records['head']['id']
            or any(records[k]['id'] != records['manifest'][k]['id'] for k in ('head', 'wrist'))):
        raise ValueError('source installed group differs')
    paths = [config_path, base, base.with_name('nominal-config.json'),
             Path(config['urdf']).parent / 'SHA256SUMS.json',
             *[root / name for name in ('run.json', 'plant-fixture.json', 'truth-fixture.json', 'registration.json')],
             Path(config['camera']), *map(Path, config['hardware_wrist_camera_files'].values())]
    for key in ('urdf', 'mesh_manifest', 'scene_seed', 'candidate', 'camera_seed', 'jaw_mapping_file', 'head_motion_seed'):
        if config.get(key): paths.append(Path(config[key]))
    if config.get('chassis_mesh'): paths.append(Path(config['mesh_manifest']).parent / config['chassis_mesh'])
    if config.get('calibration_resume'): paths.append(Path(config['calibration_resume']['path']))
    paths.extend(root / relative for relative in _resume_files(root))
    pins = {}
    for path in sorted(set(paths)):
        if not path.is_absolute() or path.is_symlink() or not path.is_file():
            raise ValueError('regular absolute source file required: ' + str(path))
        pins[str(path)] = _sha(path)
    return dict(mode='dev_resume', source_run=str(root), source_base_manifest=str(base),
                expected_install=dict(head_id=records['head']['id'], wrist_id=records['wrist']['id'],
                                      manifest_sha256=pins[str(active / 'manifest.json')]), input_sha256=pins)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-run', type=Path, required=True)
    parser.add_argument('--source-base', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(source_spec(args.source_run, args.source_base), indent=2))
