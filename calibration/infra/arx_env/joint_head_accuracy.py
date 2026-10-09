"""Post-fit physical scoring against separately loaded truth; never used by fitting.

Head truth uses camera_T_world_optical and joint_offsets_rad {left: [6], right: [6]}.
Wrist truth uses {left: T_link6_camera, right: T_link6_camera}, matching the simulator.
All transforms are parent-from-child, in metres; joint offsets are radians.
Pixel errors and Jacobian rank are deliberately not physical accuracy evidence.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from .joint_head_isaac_fit import rigid, score_truth


def _result(metrics, thresholds):
    passed = all(metrics[key] <= limit + 1e-10 for key, limit in thresholds.items())
    return {'status': 'verified_pass' if passed else 'verified_fail',
            'physical_passed': bool(passed), 'truth_available': True,
            'metrics': metrics, 'thresholds': thresholds}


def _unverified(missing):
    return {'status': 'unverified', 'physical_passed': False, 'truth_available': False,
            'reason': 'independent_truth_or_metrology_required', 'missing_truth': missing}


def score_head_accuracy(report, truth=None):
    """Score all 12 offsets, including any joints held fixed by the solver."""
    keys = ('camera_T_world_optical', 'joint_offsets_rad')
    missing = [key for key in keys if truth is None or key not in truth]
    if truth is not None and isinstance(truth.get('joint_offsets_rad'), dict):
        missing.extend(f'joint_offsets_rad.{side}' for side in ('left', 'right')
                       if side not in truth['joint_offsets_rad'])
    if missing:
        return _unverified(missing)

    def legacy(value):
        offsets = np.asarray([value['joint_offsets_rad'][side]
                              for side in ('left', 'right')], dtype=float)
        if offsets.shape != (2, 6) or not np.isfinite(offsets).all():
            raise ValueError('expected twelve finite joint offsets, six per side')
        return {'camera_T_base_optical': value['camera_T_world_optical'],
                'joint_offsets_rad': {f'{side}_j{joint + 1}': float(offsets[i, joint])
                                      for i, side in enumerate(('left', 'right'))
                                      for joint in range(6)}}

    metrics = score_truth(legacy(report), legacy(truth))
    return _result(metrics, {'camera_translation_error_mm': 2.,
                             'camera_rotation_error_deg': .3,
                             'max_joint_offset_error_deg': .5})


def score_wrist_accuracy(report, truth=None):
    """Score both sides; missing truth for either side cannot pass the pair."""
    results = {}
    for side in ('left', 'right'):
        if truth is None or side not in truth:
            results[side] = _unverified([side])
            continue
        expected_value = truth[side]
        if isinstance(expected_value, dict):
            if 'T_link6_camera' not in expected_value:
                results[side] = _unverified([f'{side}.T_link6_camera'])
                continue
            expected_value = expected_value['T_link6_camera']
        candidate, expected = rigid(report[side]['T_link6_camera']), rigid(expected_value)
        metrics = {
            'camera_translation_error_mm': float(np.linalg.norm(candidate[:3, 3] - expected[:3, 3]) * 1000),
            'camera_rotation_error_deg': float(np.rad2deg(Rotation.from_matrix(
                candidate[:3, :3] @ expected[:3, :3].T).magnitude())),
        }
        results[side] = _result(metrics, {'camera_translation_error_mm': 2.,
                                         'camera_rotation_error_deg': .3})
    available = all(value['truth_available'] for value in results.values())
    passed = all(value['physical_passed'] for value in results.values())
    return {'status': ('verified_pass' if passed else 'verified_fail') if available else 'unverified',
            'physical_passed': passed, 'truth_available': available, 'sides': results}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('kind', choices=('head', 'wrist'))
    parser.add_argument('report', type=Path)
    parser.add_argument('--truth', type=Path, help='separate post-fit truth, never a solver input')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    report = json.loads(args.report.read_text())
    truth = json.loads(args.truth.read_text()) if args.truth else None
    result = (score_head_accuracy if args.kind == 'head' else score_wrist_accuracy)(report, truth)
    text = json.dumps(result, indent=2, allow_nan=False) + '\n'
    if args.output:
        with args.output.open('x') as stream:
            stream.write(text)
    print(text, end='')
    return 0 if result['physical_passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
