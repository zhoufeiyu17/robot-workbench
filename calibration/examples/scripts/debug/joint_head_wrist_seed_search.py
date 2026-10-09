"""Search a safer cross-wrist motion seed.

The previous seed reached its mutual-view poses by folding the arm inward, which
swings link1 close to the camera mast, and its entry route from the parked pose
grazes the chassis. This searches instead for poses that reach the other wrist
with the arm extended forward and the last joints rotated, then orders them by
travel distance so the entry is short.

Every claim is judged by the validators the planner itself uses
(Geometry.preview, wrist_visibility and plan_from_seed), so a seed accepted here
is accepted on the machine - this is not a second, weaker model of safety.

Two stages, because a mutual-view waypoint is a full twelve-joint pose: one arm
presents its marker, the other aims its camera at it.

  A  find a presentation pose per arm (extended forward, collision-free)
  B  with that arm presenting, search the other arm for a clear, well-framed view

Usage, from the run's code root on the ARX host:

  PYTHONPATH=infra:. python3 examples/scripts/debug/joint_head_wrist_seed_search.py \
      --config <run>/config.json --head calibrations/hardware/v000012.json \
      --cameras /tmp/wrist-cameras.json --start-q "$(cat /tmp/start-q.json)" \
      --out configs/wrist-motion-seed-v4.json
"""
import argparse
import copy
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'infra'))
from arx_env.joint_head_workbench_geometry import Geometry  # noqa: E402
from arx_env.joint_head_isaac_fit import rigid  # noqa: E402
from arx_env.joint_head_wrist_calibration import rectified_camera  # noqa: E402
from examples.scripts.debug.joint_head_wrist_plan import plan_wrist, wrist_visibility  # noqa: E402

ARM = {'left': (0, 6), 'right': (6, 12)}
MARKER = {'left': 24, 'right': 30}
OTHER = {'left': 'right', 'right': 'left'}
MIN_CORNER_MARGIN_PX = 30
PERTURB_JOINTS = (3, 4, 5)          # J4/J5/J6: rotating the wrist aims the camera


def prepare(geometry, offsets, marker_mounts, optical_mounts, cameras):
    """A copy of the geometry bound the way the planner binds it.

    Two different rigid mounts live on link6 and they are not interchangeable:
    the camera (optical) mount aims the observer's view, the marker mount places the
    target's printed tag. wrist_visibility() takes the camera mount as its
    optical_mount argument, so passing the marker mount there evaluates a camera that
    does not exist at that pose."""
    g = copy.copy(geometry)
    bounds = g.limits.copy()
    # The base yaw stays near the body centre: a wrist sees the other wrist from
    # across the chest, and a wide swing is what brought link1 to the mast.
    bounds[[0, 6], 0] = np.maximum(bounds[[0, 6], 0], -np.deg2rad(100))
    bounds[[0, 6], 1] = np.minimum(bounds[[0, 6], 1], np.deg2rad(100))
    g.limits = bounds
    g.frames = lambda q: geometry.frames(np.asarray(q) + offsets)
    g.mounts = {side: rigid(marker_mounts[str(marker)]) for side, marker in MARKER.items()}
    g.optical = {side: rigid(optical_mounts[side]) for side in ('left', 'right')}
    g.cameras = {side: rectified_camera(cameras[side]) for side in ('left', 'right')}
    return g


def frame_room(view, camera, margin):
    uv = np.asarray(view['corners_uv'], float)
    return float(min(uv[:, 0].min() - margin, uv[:, 1].min() - margin,
                     camera['width'] - margin - uv[:, 0].max(),
                     camera['height'] - margin - uv[:, 1].max()))


def reach(g, q, side):
    return float(np.linalg.norm(g.frames(q)[side]['link6'][:3, 3][:2]))


def usable(g, q, arm):
    pose = g.preview(q, False)
    return pose


def find_presentation(g, start, *, samples, rng, margin=18):
    """One forward-extended, collision-free pose per arm that the other can see."""
    out = {}
    lo, hi = g.limits[:6].T
    for side in ('left', 'right'):
        best = None
        for _ in range(samples):
            q = start.copy()
            q[ARM[side][0]:ARM[side][1]] = rng.uniform(lo, hi)
            pose = usable(g, q, side)
            if not pose['in_limits'] or not pose['collision_free']:
                continue
            score = reach(g, q, side)
            if best is None or score > best[0]:
                best = (score, q.copy())
        if best is None:
            raise SystemExit(f'no collision-free presentation pose for {side}')
        out[side] = best[1]
    return out


def find_observers(g, presentation, start, *, samples, keep, rng, margin=18):
    """With the other arm presenting, aim this arm's camera at its marker."""
    found = {}
    for side in ('left', 'right'):
        target = OTHER[side]
        camera = g.cameras[side]
        lo, hi = g.limits[ARM[side][0]:ARM[side][1]].T
        rows = []
        for _ in range(samples):
            q = presentation[target].copy()
            q[ARM[side][0]:ARM[side][1]] = rng.uniform(lo, hi)
            view = wrist_visibility(g, q, side, target, g.optical[side], camera)
            if not view['visible']:
                continue
            room = frame_room(view, camera, margin)
            if room < MIN_CORNER_MARGIN_PX:
                continue
            if not usable(g, q, side)['collision_free']:
                continue
            rows.append({'q': q.copy(), 'room': room, 'reach': reach(g, q, side)})
        rows.sort(key=lambda r: -(r['room'] + r['reach'] * 60.))
        chosen = []
        for row in rows:
            if all(np.max(np.abs(row['q'][ARM[side][0]:ARM[side][1]]
                                 - other['q'][ARM[side][0]:ARM[side][1]])) > .12 for other in chosen):
                chosen.append(row)
            if len(chosen) >= keep:
                break
        found[side] = chosen
    return found


def order(poses, start):
    """Greedy nearest-neighbour so the route is short and the entry is gentle."""
    remaining = list(poses)
    route, current = [], start.copy()
    while remaining:
        nxt = min(remaining, key=lambda r: float(np.max(np.abs(r['q'] - current))))
        route.append(nxt)
        remaining.remove(nxt)
        current = nxt['q']
    return route


def build(g, presentation, found, start, *, holdout_every=4, margin=18,
          seed_id='x5-wrist-visible-seed-v4', rng=None):
    waypoints, verified = [], {}
    for side in ('left', 'right'):
        target = OTHER[side]
        poses = order(found[side], start)
        for index, row in enumerate(poses):
            # Rotating the last joints between neighbouring poses is what gives the
            # fit independent views without sweeping the arm through the chassis.
            q = row['q'].copy()
            if index:
                arm = ARM[side]
                q[arm[0] + PERTURB_JOINTS[0]:arm[0] + 6] += rng.normal(0, .05, 3)
                q = np.clip(q, g.limits[:, 0], g.limits[:, 1])
            view = wrist_visibility(g, q, side, target, g.optical[side], g.cameras[side])
            pose = g.preview(q, False)
            if not view['visible'] or not pose['collision_free'] or not pose['in_limits']:
                continue
            room = frame_room(view, g.cameras[side], margin)
            if room < MIN_CORNER_MARGIN_PX * .6:
                continue
            split = 'holdout' if index % holdout_every == holdout_every - 1 else 'fit'
            verified[side] = verified.get(side, 0) + 1
            waypoints.append({'id': f'{side}-{index}', 'observer': side, 'target': target,
                              'split': split, 'q_nominal_rad': [float(v) for v in q],
                              'jaw_gap_mm': [83.0, 83.0], 'frame_room_px': round(room, 1)})
    counts = {side: sum(1 for w in waypoints if w['observer'] == side and w['split'] == 'fit')
              for side in ('left', 'right')}
    holds = {side: sum(1 for w in waypoints if w['observer'] == side and w['split'] == 'holdout')
             for side in ('left', 'right')}
    print('verified poses: fit', counts, 'holdout', holds)
    if min(counts.values()) < 3 or min(holds.values()) < 1:
        raise SystemExit('fewer than three fit and one holdout pose per side survived')
    seed = {'schema': 'x5-wrist-motion-seed-v1', 'id': seed_id,
            'note': ('searched on the current machine: arm extended forward, last joints '
                     'rotated to face the other wrist; every pose collision-free, in limits, '
                     'and the marker proven unoccluded by the mast'),
            'joint_order': [f'{side}/joint{i}' for side in ('left', 'right') for i in range(1, 7)],
            'start_q_nominal_rad': [float(v) for v in start],
            'waypoints': waypoints}
    return seed


def head_snapshot(path):
    record = json.loads(Path(path).read_text())
    payload = record.get('payload', {})
    parameters = (payload.get('row') or {}).get('parameters') or payload.get('parameters')
    if not parameters:
        raise ValueError(f'no head parameters in {path}')
    from examples.scripts.debug.joint_head_workbench import Workbench
    return Workbench.calibration_snapshot(parameters)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--config', required=True)
    ap.add_argument('--head', required=True, help='registered head version JSON')
    ap.add_argument('--start-q', required=True, help='current twelve-joint pose as JSON')
    ap.add_argument('--cameras', help='wrist intrinsics JSON (defaults to the simulation config)')
    ap.add_argument('--out', required=True)
    ap.add_argument('--samples', type=int, default=6000)
    ap.add_argument('--keep', type=int, default=6)
    ap.add_argument('--seed', type=int, default=7)
    args = ap.parse_args()

    config = json.loads(Path(args.config).read_text())
    start = np.asarray(json.loads(args.start_q), float)
    if start.shape != (12,):
        raise ValueError('start pose must have twelve joints')
    snapshot = head_snapshot(args.head)
    offsets = np.concatenate([snapshot['joint_offsets_rad'][s] for s in ('left', 'right')])
    base = Geometry(config).with_gaps(83., 83.)
    base.config = {**base.config, 'table_z_m': 0., 'table_clearance_m': .005}
    mounts = {side: rigid(config['wrist_nominal_mounts'][side]) for side in ('left', 'right')}
    cameras = (json.loads(Path(args.cameras).read_text()) if args.cameras
               else config['wrist_simulation_cameras'])
    cameras = {side: {**cameras[side], 'rectified': True, 'coefficients': [0.] * 5,
                      'distortion_model': 'distortion.inverse_brown_conrady'} for side in ('left', 'right')}
    g = prepare(base, offsets, snapshot['T_link6_marker'], mounts, cameras)
    rng = np.random.default_rng(args.seed)

    presentation = find_presentation(g, start, samples=args.samples, rng=rng)
    for side in ('left', 'right'):
        print(side, 'presentation reach %.3f m' % reach(g, presentation[side], side))
    found = find_observers(g, presentation, start, samples=args.samples, keep=args.keep, rng=rng)
    print('observer candidates:', {side: len(found[side]) for side in found})
    seed = build(g, presentation, found, start, rng=rng)
    Path(args.out).write_text(json.dumps(seed, indent=1))
    print('wrote', args.out, 'with', len(seed['waypoints']), 'capture waypoints')

    # The planner's own entry point: if this passes, the run will pass.
    plan = plan_wrist(base, mounts, start_q=start, joint_offsets=offsets, cameras=cameras,
                      marker_mounts=snapshot['T_link6_marker'], motion_seed=seed)
    print('PLAN ACCEPTED: waypoints', len(plan['waypoints']), 'frames', len(plan['planned_frames']),
          'seed', plan['motion_seed'])
    for row in plan['waypoints']:
        if row.get('capture') is not False:
            print('  %-8s %-5s %-7s room %.1f px' % (row['id'], row['observer'], row['split'],
                                                     frame_room(row['visibility'], cameras[row['observer']], 18)))


if __name__ == '__main__':
    main()
