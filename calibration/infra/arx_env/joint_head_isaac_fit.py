"""Fit rectified Isaac RGB detections and encoder feedback; never read hidden truth.

Transforms are T_parent_child; optical coordinates are x right, y down, z forward.
Joint correction convention: physical q = reported q + estimated offset.
"""
import argparse
import hashlib
import json
from pathlib import Path
import uuid
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from .urdf_mesh import origin, relative_file

SCALE = np.r_[[.01] * 3, [np.deg2rad(1)] * 15]
BOUNDS = np.r_[[.05] * 3, [np.deg2rad(10)] * 3, [np.deg2rad(5)] * 12]


def rigid(value):
    T = np.asarray(value, float)
    if (T.shape != (4, 4) or not np.isfinite(T).all()
            or not np.allclose(T[3], [0, 0, 0, 1], atol=1e-7)
            or not np.allclose(T[:3, :3].T @ T[:3, :3], np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(T[:3, :3]), 1, atol=1e-6)):
        raise ValueError('expected finite rigid transform')
    return T


class Model:
    """Full URDF tree FK, expressed relative to base_link rather than world."""
    def __init__(self, path, joint_names, base='base_link'):
        tree = ET.parse(path).getroot()
        links = {row.get('name') for row in tree.findall('link')}
        rows = tree.findall('joint')
        roots = links - {row.find('child').get('link') for row in rows}
        if len(roots) != 1 or base not in links:
            raise ValueError('URDF must be a single tree containing base_link')
        self.root, self.base, self.names = roots.pop(), base, list(joint_names)
        self.links, self.joints, self.mimics = links, [], {}
        for row in rows:
            name, kind = row.get('name'), row.get('type')
            if kind not in ('fixed', 'revolute', 'continuous', 'prismatic'):
                raise ValueError(f'unsupported joint type {kind}')
            axis = row.find('axis')
            axis = np.fromstring('1 0 0' if axis is None else axis.get('xyz'), sep=' ')
            if axis.shape != (3,) or not np.isfinite(axis).all() or np.linalg.norm(axis) < 1e-9:
                raise ValueError('invalid joint axis')
            mimic = row.find('mimic')
            if mimic is not None:
                self.mimics[name] = (mimic.get('joint'), float(mimic.get('multiplier', 1)), float(mimic.get('offset', 0)))
            self.joints.append((name, row.find('parent').get('link'), row.find('child').get('link'),
                                origin(row.find('origin')), axis / np.linalg.norm(axis), kind))
        known = {j[0]: j[-1] for j in self.joints}
        if len(known) != len(self.joints) or len(set(self.names)) != len(self.names):
            raise ValueError('duplicate joint names')
        if any(known.get(name) not in ('revolute', 'continuous') or name in self.mimics for name in self.names):
            raise ValueError('fit joints must be independent angular joints')
        # Topologically sort once; fitting evaluates this tree thousands of times.
        pending, self.joints = self.joints, []
        reached = {self.root}
        while pending:
            ready = [j for j in pending if j[1] in reached]
            if not ready or any(j[2] in reached for j in ready) or len({j[2] for j in ready}) != len(ready):
                raise ValueError('URDF is cyclic, disconnected, or has multiple parents')
            for row in ready:
                self.joints.append(row)
                reached.add(row[2])
                pending.remove(row)
        if reached != links:
            raise ValueError('disconnected URDF links')

    def frames(self, q, joint_values=None):
        q = np.asarray(q, float)
        if q.shape != (len(self.names),) or not np.isfinite(q).all():
            raise ValueError('invalid joint feedback vector')
        values = {row[0]: 0. for row in self.joints}
        if joint_values:
            kinds = {row[0]: row[-1] for row in self.joints}
            if any(kinds.get(name) != 'prismatic' or name in self.mimics for name in joint_values):
                raise ValueError('explicit joint values require independent prismatic joints')
            if not all(np.isfinite(value) for value in joint_values.values()):
                raise ValueError('nonfinite explicit joint value')
            values.update(joint_values)
        values.update(zip(self.names, q))
        pending = dict(self.mimics)
        while pending:
            ready = [name for name, (parent, _, _) in pending.items() if parent in values and parent not in pending]
            if not ready:
                raise ValueError('invalid mimic joint dependency')
            for name in ready:
                parent, multiplier, offset = pending.pop(name)
                values[name] = multiplier * values[parent] + offset
        frames = {self.root: np.eye(4)}
        for name, parent, child, T, axis, kind in self.joints:
            delta = np.eye(4)
            if kind in ('revolute', 'continuous'):
                delta[:3, :3] = Rotation.from_rotvec(axis * values[name]).as_matrix()
            elif kind == 'prismatic':
                delta[:3, 3] = axis * values[name]
            frames[child] = frames[parent] @ T @ delta
        base_inverse = np.linalg.inv(frames[self.base])
        return {name: base_inverse @ T for name, T in frames.items()}


def camera_candidate(nominal, x):
    T = np.array(nominal, float).copy()
    T[:3, 3] += x[:3]
    T[:3, :3] = Rotation.from_rotvec(x[3:6]).as_matrix() @ T[:3, :3]
    return T


def predict(model, manifest, row, parameters):
    frames = model.frames(np.asarray(row['q_reported_rad']) + parameters[6:])
    camera = camera_candidate(manifest['camera_nominal_T_base_optical'], parameters)
    points = []
    markers = {m['id']: m for m in manifest['markers']}
    for marker_id in row['marker_ids']:
        marker = markers[marker_id]
        s = marker['size_m'] / 2
        corners = np.array([[-s, -s, 0], [s, -s, 0], [s, s, 0], [-s, s, 0]])
        T = frames[marker['link']] @ np.asarray(marker['T_link_marker'])
        points.extend(corners @ T[:3, :3].T + T[:3, 3])
    pc = (np.asarray(points).reshape(-1, 3) - camera[:3, 3]) @ camera[:3, :3]
    k = manifest['intrinsics']
    # Keep the objective finite when an optimizer trial passes behind the camera.
    z = np.maximum(pc[:, 2], 1e-5)
    uv = pc[:, :2] / z[:, None] * [k['fx'], k['fy']] + [k['cx'], k['cy']]
    return uv, pc[:, 2]


def refine_border(gray, corners):
    """Intersect outer border lines, excluding rounded/blurred corner neighborhoods.

    Locate the 50% local black/white intensity crossing, without assuming a camera
    response curve. Reject unreliable edges instead of inventing invisible corners.
    """
    import cv2
    image = np.asarray(gray, np.float32) / 255
    corners = np.asarray(corners, float).reshape(4, 2)
    center, lines = corners.mean(axis=0), []
    offsets = np.linspace(-3, 3, 61)
    for a, b in zip(corners, np.roll(corners, -1, axis=0)):
        length = np.linalg.norm(b - a)
        if length < 24:
            return None  # A six-cell marker smaller than this is poorly sampled.
        tangent = (b - a) / length
        normal = np.array([-tangent[1], tangent[0]])
        if normal @ ((a + b) / 2 - center) < 0:
            normal = -normal
        sites = a + (b - a) * np.linspace(.15, .85, 30)[:, None]
        samples = sites[:, None, :] + normal * offsets[None, :, None]
        if (np.any(samples[..., 0] < 1) or np.any(samples[..., 0] >= image.shape[1] - 1)
                or np.any(samples[..., 1] < 1) or np.any(samples[..., 1] >= image.shape[0] - 1)):
            return None
        intensity = cv2.remap(image, samples[..., 0].astype(np.float32),
                              samples[..., 1].astype(np.float32), cv2.INTER_LINEAR)
        black, white = intensity[:, :5].mean(axis=1), intensity[:, -5:].mean(axis=1)
        midpoint = (black + white) / 2
        crossing = np.argmax(intensity >= midpoint[:, None], axis=1)
        good = (white - black > .2) & (crossing > 2) & (crossing < 59)
        if good.sum() < 20:
            return None
        rows, indices = np.flatnonzero(good), crossing[good]
        low, high = intensity[rows, indices - 1], intensity[rows, indices]
        shifts = offsets[indices - 1] + .1 * (midpoint[good] - low) / (high - low)
        hits = sites[good] + shifts[:, None] * normal
        vx, vy, px, py = cv2.fitLine(hits.astype(np.float32), cv2.DIST_HUBER, 0, .01, .01).ravel()
        n = np.array([-vy, vx])
        distance = float(n @ [px, py])
        if np.percentile(np.abs(hits @ n - distance), 90) > .45:
            return None
        lines.append((n, distance))
    refined = []
    for i in range(4):
        normals = np.array([lines[i - 1][0], lines[i][0]])
        if abs(np.linalg.det(normals)) < .2:
            return None
        refined.append(np.linalg.solve(normals, [lines[i - 1][1], lines[i][1]]))
    refined = np.asarray(refined)
    return refined if np.max(np.linalg.norm(refined - corners, axis=1)) < 3 else None


def detect(path):
    import cv2
    image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise ValueError(f'unreadable image {path}')
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), params)
    corners, ids, _ = detector.detectMarkers(image)
    if ids is None:
        return {}
    ids = ids.ravel().tolist()
    if len(set(ids)) != len(ids):
        raise ValueError('duplicate ArUco IDs in one image')
    found = {}
    for key, value in zip(ids, corners):
        refined = refine_border(image, value)
        if refined is not None:
            found[key] = refined
    return found


def load_dataset(path):
    path = Path(path).resolve(strict=True)
    if path.is_dir():
        path = path / 'dataset.json'
    manifest = json.loads(path.read_text())
    if manifest.get('schema') != 'joint-head-isaac-v1' or len(manifest['joint_names']) != 12:
        raise ValueError('expected joint-head-isaac-v1 and 12 ordered joints')
    model = Model(relative_file(path.parent, manifest['urdf_path']), manifest['joint_names'])
    rigid(manifest['camera_nominal_T_base_optical'])
    k = manifest['intrinsics']
    if not np.isfinite([k[key] for key in ('fx', 'fy', 'cx', 'cy')]).all() or min(k['fx'], k['fy']) <= 0:
        raise ValueError('invalid pinhole intrinsics')
    ids = set()
    for marker in manifest['markers']:
        if (not isinstance(marker['id'], int) or marker['id'] not in range(50) or marker['id'] in ids
                or marker['link'] not in model.links or not np.isfinite(marker['size_m']) or marker['size_m'] <= 0):
            raise ValueError('invalid marker ID, mount, or size')
        ids.add(marker['id'])
        rigid(marker['T_link_marker'])
    rows, frame_ids, paths = [], set(), set()
    for frame in manifest['frames']:
        q = np.asarray(frame['q_reported_rad'], float)
        if q.shape != (12,) or not np.isfinite(q).all() or frame['split'] not in ('fit', 'holdout'):
            raise ValueError('invalid frame feedback or split')
        image = relative_file(path.parent, frame['image'])
        if frame['id'] in frame_ids or image in paths:
            raise ValueError('duplicate frame ID or reused image')
        frame_ids.add(frame['id'])
        paths.add(image)
        found = detect(image)
        selected = sorted(ids.intersection(found))
        rows.append({**frame, 'marker_ids': selected,
                     'uv': np.concatenate([found[key] for key in selected]) if selected else np.empty((0, 2)),
                     'image_sha256': hashlib.sha256(image.read_bytes()).hexdigest()})
    return path, model, manifest, rows


def residuals(model, manifest, rows, x):
    return np.concatenate([(predict(model, manifest, row, x)[0] - row['uv']).ravel() for row in rows])


def statistics(errors):
    return {'corners': len(errors), 'mean_px': float(np.mean(errors)),
            'rms_px': float(np.sqrt(np.mean(np.square(errors)))), 'p95_px': float(np.percentile(errors, 95))}


def coverage_report(manifest, rows):
    expected = sorted(m['id'] for m in manifest['markers'])
    per_frame = [{'id': r['id'], 'split': r['split'], 'expected_marker_ids': expected,
                  'detected_marker_ids': r['marker_ids'],
                  'missing_marker_ids': sorted(set(expected) - set(r['marker_ids']))} for r in rows]
    totals = {}
    planned = manifest.get('planned_frames')
    for split in ('fit', 'holdout'):
        subset = [r for r in rows if r['split'] == split]
        planned_subset = [r for r in planned if r['split'] == split] if isinstance(planned, list) else None
        missing_capture = sorted({r['id'] for r in planned_subset} - {r['id'] for r in subset}) if planned_subset is not None else None
        totals[split] = {'planned_frames': len(planned_subset) if planned_subset is not None else None,
                         'captured_frames': len(subset), 'effective_frames': sum(bool(r['marker_ids']) for r in subset),
                         'missing_capture_ids': missing_capture,
                         'zero_detection_ids': [r['id'] for r in subset if not r['marker_ids']],
                         'marker_visibility': {str(key): {'detected_frames': sum(key in r['marker_ids'] for r in subset),
                                                         'captured_frames': len(subset)} for key in expected}}
    complete = not totals['holdout']['zero_detection_ids'] and not totals['holdout']['missing_capture_ids']
    return {'splits': totals, 'per_frame': per_frame, 'holdout_coverage_pass': complete,
            'expected_marker_semantics': 'all configured markers; partial per-frame visibility is allowed',
            'planned_coverage_known': planned is not None,
            'gate': 'Every captured holdout frame must have detections; every explicitly planned holdout must be captured. Unknown plan counts remain unknown.'}


def solve(model, manifest, rows):
    train = [r for r in rows if r['split'] == 'fit' and r['marker_ids']]
    holdout = [r for r in rows if r['split'] == 'holdout' and r['marker_ids']]
    if sum(len(r['uv']) for r in train) < 18 or not holdout:
        raise ValueError('insufficient pixel detections or missing detected holdout frames')
    result = least_squares(lambda y: residuals(model, manifest, train, y * SCALE), np.zeros(18),
                           bounds=(-BOUNDS / SCALE, BOUNDS / SCALE), loss='soft_l1', f_scale=1.,
                           max_nfev=150, ftol=1e-10, xtol=1e-10, gtol=1e-10)
    x = result.x * SCALE
    # Data-only Jacobian in scaled coordinates; robust-loss weights/prior rows cannot create rank.
    eps = 1e-4
    jac = np.column_stack([(residuals(model, manifest, train, x + np.eye(18)[i] * SCALE[i] * eps)
                           - residuals(model, manifest, train, x - np.eye(18)[i] * SCALE[i] * eps)) / (2 * eps)
                          for i in range(18)])
    _, singular, right_vectors = np.linalg.svd(jac, full_matrices=False)
    rank = int(np.count_nonzero(singular > max(singular[0] * 1e-5, 1e-10)))
    condition = float(singular[0] / singular[-1]) if singular[-1] > 1e-12 else None
    metrics, per_link, per_frame, macro = {}, {}, [], {}
    marker_links = {m['id']: m['link'] for m in manifest['markers']}
    parameter_names = ['camera_tx_m', 'camera_ty_m', 'camera_tz_m', 'camera_rx_rad', 'camera_ry_rad', 'camera_rz_rad'] + [name + '_offset_rad' for name in manifest['joint_names']]
    for split, subset in (('fit', train), ('holdout', holdout)):
        errors = np.linalg.norm(residuals(model, manifest, subset, x).reshape(-1, 2), axis=1)
        metrics[split] = {**statistics(errors), 'frames': len(subset),
                          'nominal_mean_px': float(np.linalg.norm(residuals(model, manifest, subset, np.zeros(18)).reshape(-1, 2), axis=1).mean())}
        grouped, link_frame_means, side_frame_means, frame_means = {}, {}, {}, []
        for row in subset:
            error = np.linalg.norm(predict(model, manifest, row, x)[0] - row['uv'], axis=1)
            per_frame.append({'id': row['id'], 'split': split, **statistics(error), 'marker_ids': row['marker_ids']})
            frame_means.append(float(error.mean()))
            frame_links, frame_sides = {}, {}
            for i, marker_id in enumerate(row['marker_ids']):
                link = marker_links[marker_id]
                side = 'left' if link.startswith('left_') else 'right' if link.startswith('right_') else 'body'
                values = error[4*i:4*i+4]
                grouped.setdefault(link, []).extend(values)
                frame_links.setdefault(link, []).extend(values)
                frame_sides.setdefault(side, []).extend(values)
            for link, values in frame_links.items():
                link_frame_means.setdefault(link, []).append(float(np.mean(values)))
            for side, values in frame_sides.items():
                side_frame_means.setdefault(side, []).append(float(np.mean(values)))
        per_link[split] = {link: statistics(error) for link, error in grouped.items()}
        links = {link: {'mean_px': float(np.mean(values)), 'effective_frames': len(values)} for link, values in link_frame_means.items()}
        sides = {side: {'mean_px': float(np.mean(values)), 'effective_frames': len(values)} for side, values in side_frame_means.items()}
        macro[split] = {'frame_mean_px': float(np.mean(frame_means)), 'link_macro_mean_px': float(np.mean([r['mean_px'] for r in links.values()])),
                        'side_macro_mean_px': float(np.mean([r['mean_px'] for r in sides.values()])), 'per_link': links, 'per_side': sides,
                        'weighting': 'equal frames within each link/side, then equal observed links/sides; missing evidence reported in coverage'}
    coverage = coverage_report(manifest, rows)
    per_frame.extend({'id': r['id'], 'split': r['split'], 'corners': 0, 'mean_px': None, 'rms_px': None, 'p95_px': None, 'marker_ids': []} for r in rows if not r['marker_ids'])
    positive_depth = all(np.all(predict(model, manifest, r, x)[1] > .01) for r in train + holdout)
    at_bound = bool(np.any(np.abs(x) > BOUNDS * .999))
    sufficient = rank == 18 and condition is not None and condition < 10000
    passed = bool(result.success and sufficient and positive_depth and not at_bound and metrics['holdout']['mean_px'] < 1. and coverage['holdout_coverage_pass'])
    return {'status': 'IMAGE_FIT_PASS' if passed else 'IMAGE_FIT_FAIL',
            'parameters': x.tolist(), 'camera_T_base_optical': camera_candidate(manifest['camera_nominal_T_base_optical'], x).tolist(),
            'joint_offsets_rad': dict(zip(manifest['joint_names'], x[6:].tolist())),
            'metrics': metrics, 'per_link': per_link, 'per_frame': per_frame, 'macro_metrics': macro, 'coverage': coverage,
            'observability': {'rank': rank, 'parameters': 18, 'scaled_singular_values': singular.tolist(),
                              'parameter_scale': SCALE.tolist(), 'condition_number': condition, 'sufficient': sufficient,
                              'weakest_direction_scaled': dict(zip(parameter_names, right_vectors[-1].tolist())),
                              'weakest_direction_physical': dict(zip(parameter_names, (right_vectors[-1] * SCALE).tolist())),
                              'weakest_direction_definition': 'unit right singular vector in scaled coordinates; physical delta = parameter_scale * vector, metres/radians; sign arbitrary'},
            'optimizer_success': bool(result.success), 'optimizer_message': result.message,
            'optimizer_nfev': result.nfev, 'at_parameter_bound': at_bound, 'positive_depth': positive_depth,
            'used_truth_in_fit': False, 'physical_validated': False, 'applied': False,
            'acceptance': 'image-only gate: rank 18, scaled condition <10000, holdout mean <1 px, positive depth, interior bounds, holdout coverage pass; truth recovery and simulation realism evaluated separately',
            'frames_without_detections': [r['id'] for r in rows if not r['marker_ids']]}


def fit_dataset(path, output=None):
    path = Path(path).resolve(strict=True)
    if path.is_dir():
        path = path / 'dataset.json'
    initial_hash = hashlib.sha256(path.read_bytes()).hexdigest()
    completion = path.parent / 'completion.json'
    completion_bytes = completion.read_bytes() if completion.exists() else None
    if completion_bytes is not None and json.loads(completion_bytes).get('passed') is not True:
        raise ValueError('capture completion must pass before fitting')
    path, model, manifest, rows = load_dataset(path)
    urdf_path = relative_file(path.parent, manifest['urdf_path'])
    urdf_hash = hashlib.sha256(urdf_path.read_bytes()).hexdigest()
    report = solve(model, manifest, rows)
    if hashlib.sha256(path.read_bytes()).hexdigest() != initial_hash:
        raise ValueError('dataset changed during fitting; refuse a running capture')
    if hashlib.sha256(urdf_path.read_bytes()).hexdigest() != urdf_hash or any(hashlib.sha256(relative_file(path.parent, r['image']).read_bytes()).hexdigest() != r['image_sha256'] for r in rows):
        raise ValueError('dataset input changed during fitting')
    if (completion.read_bytes() if completion.exists() else None) != completion_bytes:
        raise ValueError('capture completion changed during fitting')
    report['dataset_sha256'] = initial_hash
    report['input_snapshot_verified'] = True
    report['capture_completion_verified'] = completion_bytes is not None
    import cv2
    report['detector'] = {'dictionary': 'DICT_4X4_50', 'opencv_version': cv2.__version__,
                          'method': 'ArUco SUBPIX seed; local raw-intensity black/white midpoint border lines; robust intersections',
                          'rejection': 'edge under24px; low contrast; clipping; incoherent line; near-parallel edges; excessive correction',
                          'radiometry': 'No inverse response assumed; remaining tone-mapping bias is not corrected using truth.'}
    report['urdf_sha256'] = urdf_hash
    report['observations'] = [{**row, 'uv': row['uv'].tolist()} for row in rows]
    output = Path(output) if output else path.parent / ('fit-' + uuid.uuid4().hex[:12] + '.json')
    with output.open('x') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write('\n')
    return report, output


def score_truth(report, truth):
    """Explicit post-fit scorer. Caller loads truth; fitting never sees it."""
    candidate = rigid(report['camera_T_base_optical'])
    expected = rigid(truth['camera_T_base_optical'])
    error = np.array([value - truth['joint_offsets_rad'][name] for name, value in report['joint_offsets_rad'].items()])
    return {'camera_translation_error_mm': float(np.linalg.norm(candidate[:3, 3] - expected[:3, 3]) * 1000),
            'camera_rotation_error_deg': float(np.rad2deg(Rotation.from_matrix(candidate[:3, :3] @ expected[:3, :3].T).magnitude())),
            'joint_offset_errors_deg': np.rad2deg(error).tolist(),
            'max_joint_offset_error_deg': float(np.max(np.abs(np.rad2deg(error))))}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('dataset', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    report, output = fit_dataset(args.dataset, args.output)
    print(json.dumps({'report': str(output), 'status': report['status'], 'metrics': report['metrics'],
                      'observability': report['observability']}, indent=2))
