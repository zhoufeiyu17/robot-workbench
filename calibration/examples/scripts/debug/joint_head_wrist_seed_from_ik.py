"""Freeze a wrist seed from the planner's own IK search instead of random sampling.

Running plan_wrist without a seed makes the planner solve the poses itself: least_squares
onto a marker position a short distance in front of the observing camera, then every leg
proved with path_check. Freezing that route - anchored at the same start pose the planner
used - yields a seed that needs no entry leg (start == anchor) and whose legs were already
checked on this machine. plan_from_seed then re-verifies all of it from scratch.

Run:
  PYTHONPATH=infra:. python3 examples/scripts/debug/joint_head_wrist_seed_from_ik.py \
      --config <run>/config.json --head calibrations/hardware/v000012.json \
      --cameras /tmp/wrist-cameras.json --start-q "$(cat /tmp/start-q.json)" \
      --out /tmp/wrist-motion-seed-v4.json
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'infra'))
from arx_env.joint_head_isaac_fit import rigid  # noqa: E402
from arx_env.joint_head_workbench_geometry import Geometry  # noqa: E402
from examples.scripts.debug.joint_head_wrist_plan import plan_wrist  # noqa: E402
from examples.scripts.debug.joint_head_wrist_clearance import MastClearance  # noqa: E402
from examples.scripts.debug.joint_head_wrist_seed_search import (  # noqa: E402
    ARM, MARKER, frame_room, head_snapshot)

ARM_NAMES = ('left', 'right')


POLE_NAMES = ('top_camera', 'mast', 'pole', 'stand')


def mast(geometry):
    """The camera pole: what makes a folded posture unsafe to bring close."""
    for name, vertices, _ in getattr(geometry, 'static_meshes', []):
        if any(token in name for token in POLE_NAMES):
            return np.asarray(vertices, float)
    return None


def clearance(geometry, q, vertices):
    """Nearest approach of any arm link to the pole, in metres."""
    if vertices is None:
        return float('nan')
    frames = geometry.frames(q)
    worst = float('inf')
    for side in ARM_NAMES:
        for name, T in frames[side].items():
            gap = np.linalg.norm(vertices - T[:3, 3], axis=1).min()
            worst = min(worst, gap)
    return float(worst)


def posture(geometry, q):
    """Elbow flexion per arm: near zero is an extended arm, large is folded."""
    return {side: float(abs(q[ARM[side][0] + 2])) for side in ARM_NAMES}


def report(geometry, offsets, cameras, seed, mounts, marker_mounts_by_side):
    """Posture metrics for an existing seed: how folded, how close to the pole."""
    pole = mast(geometry)
    rows = []
    for row in seed['waypoints']:
        if row.get('capture') is False:
            continue
        q = np.asarray(row['q_nominal_rad'], float) - offsets
        frames = geometry.frames(q)
        optical = frames[row['observer']]['link6'] @ mounts[row['observer']]
        marker = frames[row['target']]['link6'] @ marker_mounts_by_side[row['target']]
        rows.append({'id': row['id'], 'observer': row['observer'], 'split': row['split'],
                     'elbow_deg': {side: np.degrees(abs(q[ARM[side][0] + 2])) for side in ARM_NAMES},
                     'pole': clearance(geometry, q, pole),
                     'distance': float(np.linalg.norm(marker[:3, 3] - optical[:3, 3]))})
    print('%-10s %-6s %-8s %8s %8s %8s %8s' % ('id', 'side', 'split', 'L-elbow', 'R-elbow', 'pole m', 'dist m'))
    for row in rows:
        print('%-10s %-6s %-8s %8.1f %8.1f %8.3f %8.3f' % (
            row['id'], row['observer'], row['split'], row['elbow_deg']['left'],
            row['elbow_deg']['right'], row['pole'], row['distance']))
    poles = [r['pole'] for r in rows if np.isfinite(r['pole'])]
    elbows = [v for r in rows for v in r['elbow_deg'].values()]
    print('SUMMARY poses %d | pole min %.3f m mean %.3f m | elbow max %.0f deg mean %.0f deg' % (
        len(rows), min(poles), float(np.mean(poles)), max(elbows), float(np.mean(elbows))))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--config', required=True)
    ap.add_argument('--head', required=True)
    ap.add_argument('--cameras')
    ap.add_argument('--start-q', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--poses-per-side', type=int, default=16)
    ap.add_argument('--seed-id', default='x5-wrist-visible-seed-v5')
    ap.add_argument('--min-clearance-m', type=float, default=0.02,
                    help='search-time margin from either arm to the camera mast; recorded in the seed')
    ap.add_argument('--seed-report', help='report posture metrics for an existing seed and exit')
    ap.add_argument('--verify', action='store_true', help='with --seed-report, also run plan_from_seed')
    args = ap.parse_args()

    config = json.loads(Path(args.config).read_text())
    snapshot = head_snapshot(args.head)
    offsets = np.concatenate([snapshot['joint_offsets_rad'][s] for s in ARM_NAMES])
    home = np.asarray(json.loads(args.start_q), float)
    if home.shape != (12,):
        raise ValueError('start pose must have twelve joints')
    mounts = config.get('wrist_nominal_mounts')
    if not mounts:
        raise ValueError('no wrist_nominal_mounts in the configuration')
    cameras = (json.loads(Path(args.cameras).read_text()) if args.cameras
               else config['wrist_simulation_cameras'])
    cameras = {side: {**cameras[side], 'rectified': True, 'coefficients': [0.] * 5,
                      'distortion_model': 'distortion.inverse_brown_conrady'} for side in ARM_NAMES}

    base = Geometry(config)
    geometry = base.with_gaps(83., 83.)
    geometry.config = {**geometry.config, 'table_z_m': 0., 'table_clearance_m': .005}
    pole = mast(base)
    optical_mount = {side: rigid(mounts[side]) for side in ARM_NAMES}
    marker_mount = {side: rigid(snapshot['T_link6_marker'][str(MARKER[side])]) for side in ARM_NAMES}

    mast_clearance = MastClearance(geometry)
    print('=== planner IK, no seed (this is the path the workbench takes when no seed is set) ===')
    print('margin to the mast: %.0f mm (mast = rod + original mount seat + extension + head camera box)'
          % (args.min_clearance_m * 1000))
    result = plan_wrist(geometry, mounts, poses_per_side=args.poses_per_side, seed=20,
                        start_q=home, joint_offsets=offsets, cameras=cameras,
                        marker_mounts=snapshot['T_link6_marker'], min_clearance_m=args.min_clearance_m)
    print('waypoints %d | planned frames %d' % (len(result['waypoints']), len(result['planned_frames'])))
    print('rejection counts:', json.dumps(result['rejection_counts'], ensure_ascii=False, sort_keys=True))

    print('\n=== posture of every capture pose ===')
    print('%-14s %-8s %-8s %7s %7s %7s %8s' % ('id', 'observer', 'split', 'room px', 'dist m', 'pole m', 'mast m'))
    for row in result['waypoints']:
        if row.get('capture') is False:
            continue
        q = np.asarray(row['q'], float)
        room = frame_room(row['visibility'], cameras[row['observer']], 18)
        frames = geometry.frames(q)
        optical = frames[row['observer']]['link6'] @ optical_mount[row['observer']]
        marker = frames[row['target']]['link6'] @ marker_mount[row['target']]
        print('%-14s %-8s %-8s %7.1f %7.3f %7.3f %8.3f' % (
            row['id'], row['observer'], row['split'], room,
            float(np.linalg.norm(marker[:3, 3] - optical[:3, 3])), clearance(geometry, q, pole),
            mast_clearance.clearance(geometry, q)))

    if args.seed_report:
        seed = json.loads(Path(args.seed_report).read_text())
        print('=== posture report for %s (%s) ===' % (args.seed_report, seed.get('id')))
        report(geometry, offsets, cameras, seed, optical_mount, marker_mount)
        if args.verify:
            checked = plan_wrist(geometry, mounts, poses_per_side=args.poses_per_side, seed=20,
                                 start_q=home, joint_offsets=offsets, cameras=cameras,
                                 marker_mounts=snapshot['T_link6_marker'], motion_seed=seed)
            print('plan_from_seed ACCEPTED: %d waypoints, %d frames' % (
                len(checked['waypoints']), len(checked['planned_frames'])))
        return 0
    seed = {'schema': 'x5-wrist-motion-seed-v1', 'id': args.seed_id,
            'min_clearance_m': args.min_clearance_m,
            'note': ('poses solved by the planner on this machine and frozen: arm extended, '
                     'gripper rotated so each wrist camera looks at the other wrist marker; '
                     'every leg proved collision-free and every pose proved unoccluded, and every '
                     'pose keeps min_clearance_m from the camera mast (rod + original mount seat + '
                     'extension rod + head camera box). plan_from_seed re-checks that margin.'),
            'joint_order': [f'{side}/joint{i}' for side in ARM_NAMES for i in range(1, 7)],
            'start_q_nominal_rad': [float(v) for v in home + offsets],
            'waypoints': []}
    for row in result['waypoints']:
        item = {'id': row['id'], 'q_nominal_rad': [float(v) for v in np.asarray(row['q'], float) + offsets],
                'jaw_gap_mm': [83.0, 83.0]}
        if row.get('capture') is False:
            item['capture'] = False
        else:
            item.update(observer=row['observer'], target=row['target'], split=row['split'])
        seed['waypoints'].append(item)
    print('\nfroze %d waypoints (%d capture, %d transit)' % (
        len(seed['waypoints']), sum(1 for w in seed['waypoints'] if w.get('capture') is not False),
        sum(1 for w in seed['waypoints'] if w.get('capture') is False)))

    print('\n=== re-verify through plan_from_seed (the path the workbench will take) ===')
    verified = plan_wrist(geometry, mounts, poses_per_side=args.poses_per_side, seed=20,
                          start_q=home, joint_offsets=offsets, cameras=cameras,
                          marker_mounts=snapshot['T_link6_marker'], motion_seed=seed)
    print('ACCEPTED: waypoints %d frames %d seed %s' % (
        len(verified['waypoints']), len(verified['planned_frames']), verified['motion_seed']))
    gaps = [mast_clearance.clearance(geometry, np.asarray(w['q_nominal_rad'], float) - offsets)
            for w in seed['waypoints']]
    print('seed mast clearance: min %.3f m mean %.3f m over %d waypoints (margin %.0f mm)'
          % (min(gaps), float(np.mean(gaps)), len(gaps), args.min_clearance_m * 1000))
    Path(args.out).write_text(json.dumps(seed, indent=1, allow_nan=False))
    print('wrote', args.out)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
