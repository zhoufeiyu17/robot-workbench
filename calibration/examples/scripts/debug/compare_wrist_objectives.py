"""Offline comparison only: fixed seed/head/splits, no publication or installation."""
import argparse
import hashlib
import json
from pathlib import Path

from arx_env.joint_head_workbench_geometry import Geometry
from arx_env.joint_head_wrist_calibration import solve_wrist_dataset, rigid_difference


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--dataset', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--baseline-report', type=Path)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError('output must be a new directory')
    args.output.mkdir(parents=True)
    root = args.dataset.resolve().parent
    dataset = json.loads(args.dataset.read_text())
    contract = dataset['wrist_contract']
    if contract.get('solver') != 'hybrid' or contract.get('estimate_marker_mount', False):
        raise ValueError('comparison requires existing hybrid plan with frozen head marker mounts')
    config = json.loads(args.config.read_text())
    geometry = Geometry(config)
    files = [args.dataset.resolve(), root/'completion.json', args.config.resolve()]
    files += [root/(row['camera_images'].get(row['observer']) or row['image']) for row in dataset['frames']]
    files += [Path(config[key]) for key in ('urdf','scene_seed','candidate','camera','mesh_manifest')]
    manifest_path = Path(config['mesh_manifest'])
    files += [manifest_path.parent/path for path in json.loads(manifest_path.read_text())['model']['meshes'].values()]
    files += [Path(__file__), Path(__import__('arx_env.joint_head_wrist_calibration',fromlist=['']).__file__)]
    def fingerprints():
        return {str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in files}
    if args.baseline_report:
        files.append(args.baseline_report.resolve())
    before = fingerprints()
    reports = solve_wrist_dataset(dataset, geometry, contract['nominal_mounts'], dataset_root=root,
        allow_partial=True, source_completion=json.loads((root/'completion.json').read_text()), mesh_scope='arm',
        progress=lambda row: print(json.dumps(row), flush=True))
    if fingerprints() != before:
        raise ValueError('source changed during offline comparison')
    output = dict(schema='joint-head-wrist-objective-comparison-v1', source_sha256=before,
        source_dataset=str(args.dataset), frozen_head_id=contract['head_snapshot']['id'],
        nominal_mounts=contract['nominal_mounts'], original_acceptance=contract.get('acceptance',{}),
        installed=False, physical_validated=False, arm_mesh=reports,
        corners_only={side:report['corner_only_comparison'] for side, report in reports.items()},
        interpretation='Same fit/holdout, seed, head markers and offsets. Mesh metrics for both objectives use the same arm-only evidence; gripper remains an occluder, never a residual sample. Corner-only mesh metrics are evaluation only. Original acceptance thresholds unchanged; no result is installed.')
    if args.baseline_report:
        baseline = json.loads(args.baseline_report.read_text())
        output['full_mesh_baseline'] = baseline
        output['delta_from_installed'] = {mode:{side:rigid_difference(baseline[side]['T_link6_camera'], output[mode][side]['T_link6_camera'])
            for side in reports} for mode in ('arm_mesh','corners_only')}
    (args.output/'comparison.json').write_text(json.dumps(output,indent=2,allow_nan=False)+'\n')
    print(json.dumps({side:{mode:{key:output[mode][side][key] for key in ('fit','holdout','optimizer_success','jacobian_rank')}
                           for mode in ('arm_mesh','corners_only')} for side in reports}), flush=True)


if __name__ == '__main__':
    main()
