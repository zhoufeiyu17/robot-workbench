"""Is the wrist half actually able to measure the sticker mount, or only to slide along it?

Both halves estimate the same `T_link6_marker`, and a two-round pipeline was built on the
assumption that the wrist half - six parameters instead of the head half's twenty-six -
measures that installation better and can hand it back. Measurements on the real capture
say otherwise, so this tool exists to keep the question answerable rather than a matter of
argument. It answers three separate things, all read-only:

1. **Same quantity?** A camera-frame mistake would show up as huge, pose-dependent
   residuals. The wrist capture carries head-camera detections of the same stickers at the
   same poses, so the head camera's extrinsic can be scored on pixels it was never fitted
   on, with either candidate mount. Consistent numbers mean the two halves do frame the
   sticker the same way (`T_link6_marker` lives in the arm's link6 frame, which is
   camera-independent, so no view conversion is involved).

2. **How strongly does each camera pin the mount?** Hold the mount at points along an SE(3)
   path from one half's value to the other's, refit *only that camera*, and read the curve.
   A steep curve means the pixels determine the mount; a flat one means camera and mount
   trade off and the value is a basin, not a measurement.

3. **How much viewpoint diversity does each half have?** The sticker's orientation spread in
   the camera frame is what decides (2): the head half sweeps the arm past a fixed camera,
   the wrist half watches a sticker that its own motion keeps nearly facing it.

usage: joint_head_marker_identifiability.py --config <run>/config.json [--steps N] [--json]
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation, Slerp

OTHER = {'left': 'right', 'right': 'left'}
ORDER = ('left', 'right')

# The checkout keeps `infra/` next to `examples/`; a deployed run keeps it next to the
# copied scripts (`<root>/code/infra`). Both are `parents[3]` of this file's directory.
for candidate in (Path(__file__).resolve().parents[3] / 'infra',
                  Path(__file__).resolve().parents[3] / 'code' / 'infra'):
    if candidate.is_dir():
        sys.path.insert(0, str(candidate))

from arx_env.joint_head_wrist_calibration import (marker_corners, rigid, rigid_difference,
                                                  solve_wrist_calibration)


def interpolate(a, b, t):
    """SE(3) path from a to b: slerp on rotation, linear on translation."""
    rotations = Slerp([0., 1.], Rotation.concatenate([Rotation.from_matrix(np.asarray(a, float)[:3, :3]),
                                                      Rotation.from_matrix(np.asarray(b, float)[:3, :3])]))
    T = np.eye(4)
    T[:3, :3] = rotations([float(t)]).as_matrix()[0]
    T[:3, 3] = (1 - float(t)) * np.asarray(a, float)[:3, 3] + float(t) * np.asarray(b, float)[:3, 3]
    return T


def spread(transforms):
    """How much the poses differ: mean translation std (mm) and rotation std (deg)."""
    transforms = [np.asarray(T, float) for T in transforms]
    positions = np.asarray([T[:3, 3] for T in transforms])
    mean_rotation = Rotation.from_matrix(np.asarray([T[:3, :3] for T in transforms])).mean()
    angles = []
    for T in transforms:
        delta = T[:3, :3].T @ mean_rotation.as_matrix()
        angles.append(np.degrees(np.arccos(np.clip((np.trace(delta) - 1) / 2, -1, 1))))
    return {'poses': len(transforms),
            'translation_std_mm': float(np.linalg.norm(positions.std(0)) * 1000),
            'translation_max_mm': float(np.abs(positions - positions.mean(0)).max() * 1000),
            'rotation_std_deg': float(np.std(angles)),
            'rotation_max_deg': float(np.max(angles))}


def project(camera, intrinsics, T_world_marker, size):
    """Marker corners in pixels; the same convention both halves use."""
    T = np.linalg.inv(rigid(camera)) @ rigid(T_world_marker)
    points = marker_corners(size) @ T[:3, :3].T + T[:3, 3]
    if np.any(points[:, 2] <= 0):
        raise ValueError('marker behind camera')
    return points[:, :2] / points[:, 2:3] * np.asarray([intrinsics['fx'], intrinsics['fy']]) \
        + np.asarray([intrinsics['cx'], intrinsics['cy']])


def corner_rms(rows, mounts_by_marker, camera, intrinsics, size):
    """Corner RMS of `rows` under one candidate installation per marker.

    A candidate far enough out to push the sticker behind the camera is reported as
    unusable rather than allowed to abort the whole report: that is a result about the
    candidate, not an error in the measurement.
    """
    values = []
    try:
        for row in rows:
            uv = project(camera, intrinsics, row['link6'] @ rigid(mounts_by_marker[str(row['marker'])]), size)
            values.append(np.linalg.norm(uv - row['corners'], axis=1))
    except ValueError as exc:
        return {'error': str(exc)}
    if not values:
        return None
    values = np.concatenate(values)
    return dict(rms=float(np.sqrt(np.mean(values ** 2))), corners=len(values), poses=len(rows))


def newest_batch(run, kind):
    """Newest hardware capture of the requested kind whose completion says it passed."""
    best = None
    for capture in sorted((run / 'data').glob('batch-*/capture')):
        dataset = capture / 'dataset.json'
        completion = capture / 'completion.json'
        if not dataset.exists() or not completion.exists():
            continue
        if not json.loads(completion.read_text()).get('passed'):
            continue
        # A present contract marks a wrist capture; test the key, not its truthiness,
        # so an oddly empty contract is still classified as one.
        if isinstance(json.loads(dataset.read_text()).get('wrist_contract'), dict) != (kind == 'wrist'):
            continue
        stamp = capture.stat().st_mtime
        if best is None or stamp > best[0]:
            best = (stamp, capture)
    return best[1] if best else None


def load_wrist(capture):
    record = json.loads((capture / 'dataset.json').read_text())
    contract = record['wrist_contract']
    snapshot = contract['head_snapshot']
    ids = snapshot['marker_ids']
    offsets = np.asarray([snapshot['joint_offsets_rad'][s] for s in ORDER], float).reshape(12)
    nominal = {side: rigid(snapshot['T_link6_marker'][str(ids[side])]) for side in ORDER}
    return record, contract, snapshot, ids, offsets, nominal


def wrist_observations(record, geometry, ids, offsets, nominal, side):
    """Rows for solve_wrist_calibration: the sticker carries the snapshot's mount."""
    rows = []
    for row in record['frames']:
        if row['observer'] != side:
            continue
        target = row['target']
        q = np.asarray(row.get('camera_feedback', {}).get(side, {}).get(
            'q_reported12', row['q_reported_rad']), float)
        frames = geometry.frames(q + offsets)
        corners = (row.get('camera_detections') or {}).get(side, {}).get(str(ids[target]))
        if corners is None:
            continue
        rows.append(dict(id=row['planned_id'], split=row['split'],
                         T_world_observer_link6=frames[side]['link6'].tolist(),
                         T_world_marker=(frames[target]['link6'] @ nominal[target]).tolist(),
                         T_world_target_link6=frames[target]['link6'].tolist(),
                         corners_uv=np.asarray(corners, float).tolist()))
    return rows


def wrist_estimate(rows, camera, camera_seed, size, snapshot_id, initial_marker):
    """Fit the camera alone, or camera plus the other arm's sticker when asked."""
    report = solve_wrist_calibration(rows, camera, size, camera_seed,
                                     source_calibration_id=snapshot_id,
                                     initial_T_link6_marker=initial_marker)
    return report


def camera_rows(capture, camera_name, geometry, offsets, ids, arm_of):
    """Rows for one camera of a capture: the arm's link6 pose and the pixels decoded."""
    record = json.loads((capture / 'dataset.json').read_text())
    rows = []
    for row in record['frames']:
        detected = (row.get('camera_detections') or {}).get(camera_name) or {}
        if not detected:
            continue
        observer = row.get('observer')
        q = np.asarray(row.get('camera_feedback', {}).get(observer, {}).get(
            'q_reported12', row['q_reported_rad']), float)
        frames = geometry.frames(q + offsets)
        for marker_id, corners in detected.items():
            marker = int(marker_id)
            rows.append(dict(split=row['split'], marker=marker, arm=arm_of[marker],
                             link6=frames[arm_of[marker]]['link6'],
                             observer_link6=frames[observer]['link6'] if observer else None,
                             corners=np.asarray(corners, float)))
    return rows


def refit_camera(rows, camera_seed, mounts_by_marker, intrinsics, size):
    """Six-parameter camera fit with the sticker mounts held fixed."""
    seed = np.asarray(camera_seed, float)

    def residual(x):
        camera = seed.copy()
        camera[:3, 3] += x[:3]
        camera[:3, :3] = Rotation.from_rotvec(x[3:]).as_matrix() @ camera[:3, :3]
        values = []
        for row in rows:
            # Clamp depth exactly as the head half's own predict() does, so a trial pose
            # that dips behind the camera stays finite instead of aborting the fit.
            T = np.linalg.inv(rigid(camera)) @ rigid(row['link6'] @ rigid(mounts_by_marker[str(row['marker'])]))
            points = marker_corners(size) @ T[:3, :3].T + T[:3, 3]
            z = np.maximum(points[:, 2], 1e-5)
            uv = points[:, :2] / z[:, None] * np.asarray([intrinsics['fx'], intrinsics['fy']]) \
                + np.asarray([intrinsics['cx'], intrinsics['cy']])
            values.append((uv - row['corners']).ravel())
        return np.concatenate(values)

    result = least_squares(residual, np.zeros(6), loss='soft_l1', max_nfev=200)
    camera = seed.copy()
    camera[:3, 3] += result.x[:3]
    camera[:3, :3] = Rotation.from_rotvec(result.x[3:]).as_matrix() @ camera[:3, :3]
    return camera


def split_rms(rows, mounts_by_marker, camera, intrinsics, size):
    out = {}
    for split in ('fit', 'holdout'):
        selected = [row for row in rows if row['split'] == split]
        out[split] = corner_rms(selected, mounts_by_marker, camera, intrinsics, size)
    return out


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Measure whether the wrist half can identify the sticker mount (read-only)')
    parser.add_argument('--config', required=True, type=Path, help='workbench config.json used by the run')
    parser.add_argument('--wrist-batch', help='capture directory or batch name; default: newest passing wrist capture')
    parser.add_argument('--head-batch', help='capture directory or batch name; default: newest passing head capture')
    parser.add_argument('--steps', type=int, default=9, help='profile resolution along the SE(3) path')
    parser.add_argument('--json', action='store_true', help='print the measurements as JSON')
    args = parser.parse_args(argv)

    repo_root = Path(__file__).resolve().parents[3]
    for candidate in (repo_root / 'infra', repo_root / 'code' / 'infra'):
        if candidate.is_dir():
            sys.path.insert(0, str(candidate))
    from arx_env.joint_head_workbench_geometry import Geometry

    run = args.config.resolve().parent
    geometry = Geometry(json.loads(args.config.read_text()))

    def resolve(value, kind):
        if not value:
            found = newest_batch(run, kind)
            if found is None:
                raise SystemExit('no passing %s capture under %s' % (kind, run / 'data'))
            return found
        path = Path(value)
        if path.is_dir():
            return path
        return run / 'data' / value / 'capture'

    wrist_capture = resolve(args.wrist_batch, 'wrist')
    head_capture = resolve(args.head_batch, 'head')
    record, contract, snapshot, ids, offsets, nominal = load_wrist(wrist_capture)
    arm_of = {ids[side]: side for side in ORDER}
    size = geometry.marker_size
    head_intrinsics = json.loads((wrist_capture / 'dataset.json').read_text())['cameras']['head']
    head_camera = rigid(snapshot['camera_T_world_optical'])

    wrist_rows = {side: wrist_observations(record, geometry, ids, offsets, nominal, side)
                  for side in ORDER}
    fits = {}
    estimate = {side: nominal[side].copy() for side in ORDER}
    for free in (False, True):
        cameras = {}
        for side in ORDER:
            report = wrist_estimate(wrist_rows[side], contract['cameras'][side],
                                    rigid(contract['nominal_mounts'][side]), size, snapshot['id'],
                                    nominal[OTHER[side]] if free else None)
            cameras[side] = rigid(report['T_link6_camera'])
            if free:
                estimate[OTHER[side]] = rigid(report['T_link6_marker'])
        fits['freed' if free else 'frozen'] = cameras

    # The head half's own solution for this capture: its camera, its offsets, its mounts.
    head_fit = max((head_capture.glob('fit-*.json')), key=lambda path: path.stat().st_mtime)
    head_report = json.loads(head_fit.read_text())
    head_offsets = np.asarray([head_report['joint_offsets_rad'][s] for s in ORDER], float).reshape(12)
    head_mounts = {str(marker): rigid(value) for marker, value in
                   head_report['T_link6_marker'].items() if marker in ('24', '30')}
    head_seed = rigid(head_report['camera_T_world_optical'])
    head_rows = camera_rows(head_capture, 'head', geometry, head_offsets, ids, arm_of)
    cross_rows = camera_rows(wrist_capture, 'head', geometry, offsets, ids, arm_of)

    candidates = {'头部半场（快照/自身解）': nominal, '设计名义值': None, '腕半场（放开安装）': estimate}
    design = json.loads(Path(json.loads(args.config.read_text())['candidate']).read_text())['T_link6_marker']
    candidates['设计名义值'] = {side: rigid(design[str(ids[side])]) for side in ORDER}

    result = {'wrist_capture': str(wrist_capture), 'head_capture': str(head_capture),
              'marker_ids': ids, 'candidates': {}, 'cross_table': {}, 'profile': {},
              'viewpoint': {}, 'wrist_fits': {}}

    for label, mounts in candidates.items():
        result['candidates'][label] = {side: rigid_difference(nominal[side], mounts[side]) for side in ORDER}
    for label, cameras in fits.items():
        result['wrist_fits'][label] = {side: {'camera': cameras[side].tolist()} for side in ORDER}

    for camera_label, cameras in fits.items():
        for side in ORDER:
            result['cross_table']['%s/%s' % (side, camera_label)] = {
                label: corner_rms(camera_rows(wrist_capture, side, geometry, offsets, ids, arm_of),
                                  {str(ids[s]): mounts[s] for s in ORDER},
                                  cameras[side], contract['cameras'][side], size)
                for label, mounts in candidates.items()}

    def head_camera_case(rows, seed):
        out = {}
        for label, mounts in candidates.items():
            keyed = {str(ids[side]): mounts[side] for side in ORDER}
            camera = refit_camera([row for row in rows if row['split'] == 'fit'], seed, keyed,
                                  head_intrinsics, size)
            out[label] = split_rms(rows, keyed, camera, head_intrinsics, size)
        return out

    result['cross_table']['head/own-capture'] = head_camera_case(head_rows, head_seed)
    result['cross_table']['head/wrist-capture'] = head_camera_case(cross_rows, head_camera)

    path = np.linspace(0., 1., max(2, args.steps))
    for side in ORDER:
        target = OTHER[side]
        curve = []
        for t in path:
            mount = interpolate(nominal[target], estimate[target], t)
            rows = [dict(id=row['id'], split=row['split'],
                         T_world_observer_link6=row['T_world_observer_link6'],
                         T_world_marker=(np.asarray(row['T_world_target_link6'], float) @ mount).tolist(),
                         corners_uv=row['corners_uv']) for row in wrist_rows[side]]
            report = wrist_estimate(rows, contract['cameras'][side],
                                    rigid(contract['nominal_mounts'][side]), size, snapshot['id'], None)
            curve.append({'t': float(t), 'fit_px': report['fit']['rms_px'],
                          'holdout_px': report['holdout']['rms_px'],
                          'mount_from_head_mm': rigid_difference(nominal[target], mount)['translation_mm'],
                          'camera_from_seed_mm': rigid_difference(np.asarray(contract['nominal_mounts'][side]),
                                                                  report['T_link6_camera'])['translation_mm']})
        result['profile']['wrist/%s' % side] = curve
    for label, rows, seed in (('head/own-capture', head_rows, head_seed),
                              ('head/wrist-capture', cross_rows, head_camera)):
        curve = []
        for t in path:
            keyed = {str(ids[side]): interpolate(nominal[side], estimate[side], t) for side in ORDER}
            camera = refit_camera([row for row in rows if row['split'] == 'fit'], seed, keyed,
                                  head_intrinsics, size)
            scored = split_rms(rows, keyed, camera, head_intrinsics, size)
            curve.append({'t': float(t), 'fit_px': (scored['fit'] or {}).get('rms'),
                          'holdout_px': (scored['holdout'] or {}).get('rms')})
        result['profile'][label] = curve

    wrist_camera = fits['frozen']
    for side in ORDER:
        rows = camera_rows(wrist_capture, side, geometry, offsets, ids, arm_of)
        result['viewpoint']['wrist/%s relative pose' % side] = spread([
            np.linalg.inv(np.asarray(row['observer_link6'], float)) @ np.asarray(row['link6'], float)
            for row in rows])
        result['viewpoint']['wrist/%s sticker in camera' % side] = spread([
            np.linalg.inv(np.asarray(row['observer_link6'], float) @ wrist_camera[side])
            @ np.asarray(row['link6'], float) for row in rows])
    for label, rows, seed in (('head/own-capture', head_rows, head_seed),
                              ('head/wrist-capture', cross_rows, head_camera)):
        result['viewpoint'][label] = spread([np.linalg.inv(seed) @ np.asarray(row['link6'], float)
                                             for row in rows])

    if args.json:
        print(json.dumps(result, ensure_ascii=False, indent=2, default=float))
        return 0

    print('腕批次 %s | 头批次 %s' % (wrist_capture.parent.name, head_capture.parent.name))
    print()
    print('== 同一张纸？两个候选安装在各相机像素上的残差（rms, px）')
    for key, table in result['cross_table'].items():
        print('   %s' % key)
        for label, value in table.items():
            if value is None:
                print('      %-24s —' % label)
            elif 'error' in (value if isinstance(value, dict) else {}):
                print('      %-24s 不可用：%s' % (label, value['error']))
            elif 'fit' in value:
                both = [value['fit'], value['holdout']]
                print('      %-24s fit %s | holdout %s' % (label, *[
                    '—' if part is None else ('不可用' if 'error' in part else '%.4f' % part['rms'])
                    for part in both]))
            else:
                print('      %-24s %s' % (label, '%.4f' % value['rms']))
    print()
    print('== 安装沿路径移动时，只放开该相机重解（t=0 头部值, t=1 腕半场估计值）')
    for key, curve in result['profile'].items():
        print('   %s' % key)
        for row in curve:
            print('      t=%.2f fit %s px | holdout %s px' % (
                row['t'], '—' if row['fit_px'] is None else '%.4f' % row['fit_px'],
                '—' if row['holdout_px'] is None else '%.4f' % row['holdout_px']))
    print()
    print('== 视角多样性（贴纸在相机坐标下的姿态散布）')
    for key, value in result['viewpoint'].items():
        if value is None:
            continue
        print('   %-32s 平移 %6.2f mm（最大 %6.2f）| 旋转 %6.2f 度（最大 %6.2f）'
              % (key, value['translation_std_mm'], value['translation_max_mm'],
                 value['rotation_std_deg'], value['rotation_max_deg']))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
