"""Geometry first: can "extend forward + rotate the gripper" give the wrists a mutual view?

Answers the posture question with the planner's own judge before any search spends time.
For each direction (left camera -> right marker, and back) a waypoint needs:

  facing  the target marker's normal points back at the observer camera
  framed  the marker corners land inside the image with an 18 px margin
  clear   nine sightlines from the optical centre reach the marker through no mesh

The first two are cheap orientation conditions; only the third needs the full triangle
ray test. So this sweeps the last joints on a grid using the cheap half, then confirms the
survivors with wrist_visibility and the collision check - the same pair the planner uses.

Run:
  PYTHONPATH=infra:. python3 examples/scripts/debug/joint_head_wrist_seed_geometry.py \
      --config <run>/config.json --head calibrations/hardware/v000012.json \
      --cameras /tmp/wrist-cameras.json --start-q "$(cat /tmp/start-q.json)"
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'infra'))
from arx_env.joint_head_workbench_geometry import Geometry  # noqa: E402
from arx_env.joint_head_isaac_fit import rigid  # noqa: E402
from arx_env.joint_head_wrist_calibration import project_marker  # noqa: E402
from examples.scripts.debug.joint_head_wrist_plan import wrist_visibility  # noqa: E402
from examples.scripts.debug.joint_head_wrist_seed_search import (  # noqa: E402
    ARM, OTHER, frame_room, find_presentation, head_snapshot, prepare, reach)

# Per-arm joint offsets inside the twelve-joint vector: J4, J5, J6.
LAST = {'J4': 3, 'J5': 4, 'J6': 5}
STAGES = [('J6', {'J6': 25}), ('J5+J6', {'J5': 7, 'J6': 21}), ('J4+J5+J6', {'J4': 3, 'J5': 5, 'J6': 11})]


def screen(g, q, observer, target, camera):
    """The cheap half of wrist_visibility: aim and framing, no mesh ray test."""
    frames = g.frames(q)
    optical = frames[observer]['link6'] @ g.optical[observer]
    marker = frames[target]['link6'] @ g.mounts[target]
    try:
        uv, depth = project_marker(g.optical[observer], frames[observer]['link6'], marker, camera, g.marker_size)
    except ValueError:
        return {'facing': False, 'framed': False, 'room': -1e9, 'reason': 'marker behind camera'}
    margin = float(g.config.get('margin_px', 18))
    facing = float(marker[:3, 2] @ (optical[:3, 3] - marker[:3, 3])) < 0
    framed = bool(np.all(depth > .005) and np.all(uv >= margin) and
                  np.all(uv < np.array([camera['width'], camera['height']]) - margin))
    room = float(min(uv[:, 0].min() - margin, uv[:, 1].min() - margin,
                     camera['width'] - margin - uv[:, 0].max(),
                     camera['height'] - margin - uv[:, 1].max()))
    in_cam = optical[:3, :3].T @ (marker[:3, 3] - optical[:3, 3])
    return {'facing': facing, 'framed': framed, 'room': room, 'uv': uv, 'in_cam': in_cam,
            'distance': float(np.linalg.norm(marker[:3, 3] - optical[:3, 3])),
            'aim_deg': np.degrees([np.arctan2(in_cam[0], in_cam[2]), np.arctan2(in_cam[1], in_cam[2])])}


def assignments(g, arm, spec):
    """Grid values for one arm's named joints, inside that arm's joint limits."""
    axes = []
    for name, count in spec.items():
        index = ARM[arm][0] + LAST[name]
        lo, hi = g.limits[index]
        axes.append((name, np.linspace(lo, hi, count)))
    out = [{}]
    for name, values in axes:
        out = [{**base, name: float(v)} for base in out for v in values]
    return out


def apply(g, q, arm, values):
    q = q.copy()
    for name, value in values.items():
        q[ARM[arm][0] + LAST[name]] = value
    return q


def sweep(g, base, cameras, spec):
    """Evaluate both directions over the grid; return summary plus the best rows."""
    left, right = assignments(g, 'left', spec), assignments(g, 'right', spec)
    rows, counts = [], {'pairs': 0, 'both_facing': 0, 'both_framed': 0, 'both_ok': 0}
    for lv in left:
        for rv in right:
            q = apply(g, apply(g, base, 'left', lv), 'right', rv)
            counts['pairs'] += 1
            views = {}
            for observer in ('left', 'right'):
                views[observer] = screen(g, q, observer, OTHER[observer], cameras[observer])
            if all(views[o]['facing'] for o in views):
                counts['both_facing'] += 1
            if all(views[o]['facing'] and views[o]['framed'] for o in views):
                counts['both_framed'] += 1
            if all(views[o]['facing'] and views[o]['framed'] for o in views):
                row = {'q': q, 'left': views['left'], 'right': views['right'],
                       'room': min(views['left']['room'], views['right']['room'])}
                rows.append(row)
    rows.sort(key=lambda r: -r['room'])
    return counts, rows


def confirm(g, rows, cameras, limit=40):
    """Full judge on the best screened rows: mesh occlusion plus collision and limits."""
    out, blockers = [], {}
    for row in rows[:limit]:
        verdict = {}
        for observer in ('left', 'right'):
            verdict[observer] = wrist_visibility(g, row['q'], observer, OTHER[observer],
                                                 g.optical[observer], cameras[observer])
        pose = g.preview(row['q'], False)
        ok = (all(verdict[o]['visible'] for o in verdict) and pose['collision_free'] and pose['in_limits'])
        if ok:
            out.append({**row, 'verdict': verdict, 'room': min(frame_room(verdict[o], cameras[o], 18) for o in verdict)})
        else:
            for observer in ('left', 'right'):
                if not verdict[observer]['visible']:
                    key = verdict[observer]['occluder'] or verdict[observer]['reason'] or 'unclear'
                    blockers[key] = blockers.get(key, 0) + 1
            collisions = pose['collisions'] or []
            for pair in collisions:
                blockers['collision: ' + ' / '.join(map(str, pair))] = blockers.get('collision: ' + ' / '.join(map(str, pair)), 0) + 1
            if not pose['in_limits']:
                blockers['out of limits'] = blockers.get('out of limits', 0) + 1
    return out, blockers


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--config', required=True)
    ap.add_argument('--head', required=True)
    ap.add_argument('--start-q', required=True)
    ap.add_argument('--cameras')
    ap.add_argument('--presentation-samples', type=int, default=250)
    ap.add_argument('--seed', type=int, default=7)
    args = ap.parse_args()

    config = json.loads(Path(args.config).read_text())
    start = np.asarray(json.loads(args.start_q), float)
    snapshot = head_snapshot(args.head)
    offsets = np.concatenate([snapshot['joint_offsets_rad'][s] for s in ('left', 'right')])
    cameras = (json.loads(Path(args.cameras).read_text()) if args.cameras
               else config['wrist_simulation_cameras'])
    cameras = {side: {**cameras[side], 'rectified': True, 'coefficients': [0.] * 5,
                      'distortion_model': 'distortion.inverse_brown_conrady'} for side in ('left', 'right')}
    base = Geometry(config).with_gaps(83., 83.)
    base.config = {**base.config, 'table_z_m': 0., 'table_clearance_m': .005}
    mounts = {side: rigid(config['wrist_nominal_mounts'][side]) for side in ('left', 'right')}
    g = prepare(base, offsets, snapshot['T_link6_marker'], mounts, cameras)

    rng = np.random.default_rng(args.seed)
    presentation = find_presentation(g, start, samples=args.presentation_samples, rng=rng)
    q0 = start.copy()
    for side in ('left', 'right'):
        q0[ARM[side][0]:ARM[side][1]] = presentation[side][ARM[side][0]:ARM[side][1]]
        print('%s presentation: reach %.3f m  link6 %s' % (
            side, reach(g, q0, side), np.round(g.frames(q0)[side]['link6'][:3, 3], 3)))
    pose = g.preview(q0, False)
    print('base pose: in_limits %s collision_free %s marker_size %.3f m camera %dx%d margin %.0f' % (
        pose['in_limits'], pose['collision_free'], g.marker_size,
        cameras['left']['width'], cameras['left']['height'], float(g.config.get('margin_px', 18))))
    cam = g.frames(q0)['left']['link6'] @ g.optical['left']
    tgt = g.frames(q0)['right']['link6'] @ g.mounts['right']
    print('left camera to right marker: %.3f m  (the planner IK targets 0.13-0.22 m ahead of'
          ' the camera, so the wrists have to come close)' % float(np.linalg.norm(tgt[:3, 3] - cam[:3, 3])))
    print('J6 axis in the observing camera frame: %s (a rotation this axis cannot sweep is'
          ' unreachable by turning the gripper alone)' % np.round(
              cam[:3, :3].T @ g.frames(q0)['left']['link6'][:3, 2], 3))
    print()

    for name, spec in STAGES:
        counts, rows = sweep(g, q0, cameras, spec)
        print('%s: %d pose pairs | both facing %d | both facing+framed %d' % (
            name, counts['pairs'], counts['both_facing'], counts['both_framed']))
        if not rows:
            continue
        print('   best screened room %.1f px at %s' % (
            rows[0]['room'], {k: round(v, 3) for k, v in
                              {n: apply(g, q0, 'left', {})[ARM['left'][0] + LAST[n]] for n in spec}.items()}))
        good, blockers = confirm(g, rows, cameras)
        print('   confirmed visible + collision-free: %d' % len(good))
        if good:
            best = good[0]
            print('   BEST room %.1f px | %s' % (
                best['room'], ' '.join('%s-%s' % (o, round(best['q'][ARM[o][0] + LAST[n]], 3))
                                       for o in ('left', 'right') for n in spec)))
            print('   q = %s' % np.round(best['q'], 4).tolist())
            for observer in ('left', 'right'):
                print('   %s sees %s: room %.1f px distance %.3f m aim %s deg' % (
                    observer, OTHER[observer], frame_room(best['verdict'][observer], cameras[observer], 18),
                    best[observer]['distance'], np.round(best[observer]['aim_deg'], 1)))
            return 0
        print('   blocked by: %s' % json.dumps(blockers, ensure_ascii=False))
    print('\nNo last-joint combination at this base pose gives a mutual view.')
    return 1


if __name__ == '__main__':
    raise SystemExit(main())
