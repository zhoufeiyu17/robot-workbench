"""Fit one constant link6-to-optical transform from opposite-wrist markers.

All transforms are T_parent_child. FK, joint corrections and marker attachment
are frozen by the caller. Images must be rectified; corner order is TL, TR, BR,
BL in marker coordinates (-x,-y), (+x,-y), (+x,+y), (-x,+y).
A numerical fit is not physical calibration or collision-clearance acceptance.
"""
import hashlib
import json
from pathlib import Path

import numpy as np
import cv2
from scipy.spatial import cKDTree
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from .joint_head_isaac_fit import rigid


def _mesh_observation(value):
    """Validate the small, camera-independent mesh/RGB evidence record."""
    if not isinstance(value, dict):
        raise ValueError('wrist mesh evidence must be an object')
    points = np.asarray(value.get('points_link6'), float)
    target = np.asarray(value.get('observed_uv'), float)
    normals = np.asarray(value.get('normals'), float)
    if (points.ndim != 2 or points.shape[1] != 3 or target.shape != (len(points), 2)
            or normals.shape != target.shape or len(points) < 8
            or not np.isfinite(points).all() or not np.isfinite(target).all()
            or not np.isfinite(normals).all()):
        raise ValueError('invalid wrist mesh evidence')
    norm = np.linalg.norm(normals, axis=1)
    if np.any(norm <= 1e-8):
        raise ValueError('zero wrist mesh evidence normal')
    valid = np.asarray(value.get('valid', np.ones(len(points), dtype=bool)))
    if valid.shape != (len(points),) or valid.dtype != np.bool_:
        raise ValueError('wrist mesh validity must contain one boolean per point')
    value = dict(value)
    value['points_link6'] = points.tolist()
    value['observed_uv'] = target.tolist()
    value['normals'] = (normals / norm[:, None]).tolist()
    value['valid'] = valid.tolist()
    return value


def _wrist_mesh_evidence(geometry, row, observer, target, camera, mount, offsets,
                         image_path, *, samples=240, mesh_scope="full"):
    """Extract target-wrist RGB contour evidence from the nominal full X5 mesh.

    The RGB edge map is built once from the nominal mesh.  The returned private
    Arm scope samples only opposite link1..link5; base, gripper and all other
    meshes still participate in visibility but never supply arm residuals.
    The context is used only to refresh projected correspondences during fitting;
    it is never serialized into the dataset or used as simulator truth.
    """
    if mesh_scope not in ("full", "arm"):
        raise ValueError("wrist mesh_scope must be full or arm")
    from . import joint_head_markerless as mesh
    from .joint_head_x5_hybrid import X5MeshModel, indexed_mesh

    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image is None or image.shape[:2] != (camera['height'], camera['width']):
        raise ValueError('unreadable wrist RGB image or camera resolution mismatch')
    adapter = X5MeshModel(geometry)
    manifest = {'intrinsics': camera, 'camera_nominal_T_base_optical': rigid(mount).tolist()}
    frames = adapter.frames_with_feedback(row, manifest, offsets)
    observer_frame = frames[f'{observer}/link6']
    inverse_observer = np.linalg.inv(observer_frame)
    meshes, frame_map = [], {}
    prefix = f'{target}/'
    for source in adapter.meshes:
        world_frame = frames[source['name']]
        relative = inverse_observer @ world_frame
        vertices = source['v'] @ relative[:3, :3].T + relative[:3, 3]
        name = f'target/{source["name"][len(prefix):]}' if source['name'].startswith(prefix) else f'occluder/{source["name"]}'
        local = indexed_mesh(name, vertices, source['f'])
        meshes.append(local); frame_map[local['name']] = np.eye(4)
    if not any(item['name'].startswith('target/') for item in meshes):
        raise ValueError(f'no target mesh for {target}')

    class _RelativeModel:
        def frames_with_feedback(self, _row, _manifest, _offsets):
            return frame_map

    shape = image.shape[:2]
    # RGB segmentation and contour visibility share the full-scene silhouette;
    # the observer arm and chassis occlude targets but never supply target samples.
    parts = mesh.camera_mesh_parts(_RelativeModel(), manifest, row, meshes, np.zeros(18))
    mask = np.isfinite(mesh.raster_depth(parts, camera, shape)).astype(np.uint8)
    edge = mesh.foreground_edges(image, mask)
    if len(edge['points']) < 20:
        raise ValueError('wrist RGB foreground evidence insufficient')
    contour = mesh.contours(_RelativeModel(), manifest, row, meshes, np.zeros(18), shape,
                            r'^target/' if mesh_scope == 'full' else r'^target/link[1-5]$', samples,
                            occlusion_pattern=r'^(target|occluder)/')
    target_uv, valid = mesh.match(contour, edge)
    if int(valid.sum()) < 8:
        raise ValueError('wrist RGB mesh correspondence coverage insufficient')
    context = {'points_link6': contour['points'], 'normals': contour['normals'],
               'edge_points': edge['points'], 'edge_normals': edge['normals'],
               'tree': edge['tree'], 'camera': camera}
    evidence = _refresh_wrist_mesh_evidence(rigid(mount), context)
    evidence.update(mesh_scope=mesh_scope, image=str(image_path), image_sha256=hashlib.sha256(Path(image_path).read_bytes()).hexdigest())
    return evidence, context



def _refresh_wrist_mesh_evidence(T_link6_camera, context):
    """Reproject fixed mesh samples and refresh nearest RGB edge correspondences."""
    points = np.asarray(context['points_link6'], float)
    normals = np.asarray(context['normals'], float)
    optical = (points - T_link6_camera[:3, 3]) @ T_link6_camera[:3, :3]
    if np.any(optical[:, 2] <= 0):
        raise ValueError('wrist mesh point behind optical camera')
    k = _intrinsics(context['camera'])
    uv = optical[:, :2] / optical[:, 2, None] * k[:2] + k[2:]
    distances, indices = context['tree'].query(uv, k=1)
    valid = np.isfinite(distances) & (distances <= 20.)
    target = np.asarray(context['edge_points'])[indices].copy()
    return _mesh_observation({'points_link6': points, 'observed_uv': target,
                              'normals': normals, 'valid': valid, 'coverage': float(valid.mean()),
                              'source': 'nominal_x5_mesh_rgb_edges'})


def _intrinsics(camera):
    values = np.array([camera[key] for key in ('fx', 'fy', 'cx', 'cy')], float)
    if not np.isfinite(values).all() or np.any(values[:2] <= 0):
        raise ValueError('finite rectified pinhole intrinsics with positive focal lengths required')
    coefficients=np.asarray(camera.get('coefficients',[0.]*5),float)
    if not np.isfinite(coefficients).all() or np.any(coefficients):
        raise ValueError('wrist images must have zero distortion coefficients after rectification')
    return values


def rectified_camera(camera):
    """Validate the explicit image contract used by planning and dataset fitting."""
    _intrinsics(camera)
    if camera.get('rectified') is not True:
        raise ValueError('explicit rectified wrist camera required')
    if any(type(camera.get(key)) is not int or camera[key]<=0 for key in ('width','height')):
        raise ValueError('positive integer wrist image dimensions required')
    return camera


def marker_corners(marker_size_m):
    size = float(marker_size_m)
    if not np.isfinite(size) or size <= 0:
        raise ValueError('positive finite marker size required')
    return np.array([[-1, -1, 0], [1, -1, 0], [1, 1, 0], [-1, 1, 0]]) * size / 2


def mesh_observation_residual(T_link6_camera, row, camera):
    """Normal image residual for fixed mesh↔RGB correspondences in link6 coordinates."""
    observation = row.get('mesh_observation') if isinstance(row, dict) else None
    if not isinstance(observation, dict):
        raise ValueError('hybrid wrist rows require mesh_observation')
    points = np.asarray(observation.get('points_link6'), float)
    target = np.asarray(observation.get('observed_uv'), float)
    normals = np.asarray(observation.get('normals'), float)
    if (points.ndim != 2 or points.shape[1] != 3 or target.shape != (len(points), 2)
            or normals.shape != target.shape or len(points) < 4
            or not np.isfinite(points).all() or not np.isfinite(target).all()
            or not np.isfinite(normals).all()):
        raise ValueError('invalid wrist mesh observation')
    norm = np.linalg.norm(normals, axis=1)
    if np.any(norm <= 1e-8):
        raise ValueError('zero wrist mesh observation normal')
    valid = np.asarray(observation.get('valid', np.ones(len(points), dtype=bool)))
    if valid.shape != (len(points),) or valid.dtype != np.bool_:
        raise ValueError('wrist mesh validity must contain one boolean per point')
    normals = normals / norm[:, None]
    optical = (points - T_link6_camera[:3, 3]) @ T_link6_camera[:3, :3]
    if np.any(optical[:, 2] <= 0):
        raise ValueError('wrist mesh point behind optical camera')
    k = _intrinsics(camera)
    projected = optical[:, :2] / optical[:, 2, None] * k[:2] + k[2:]
    # Invalid correspondences remain constant penalties even while T moves.
    return np.where(valid, np.einsum('ij,ij->i', projected - target, normals), 20.)


def project_marker(T_link6_camera, T_world_observer_link6, T_world_marker,
                   camera, marker_size_m):
    """Return (4x2 pixels, 4 depths in metres), rejecting points behind camera."""
    T = np.linalg.inv(rigid(T_world_observer_link6) @ rigid(T_link6_camera)) @ rigid(T_world_marker)
    points = marker_corners(marker_size_m) @ T[:3, :3].T + T[:3, 3]
    if np.any(points[:, 2] <= 0):
        raise ValueError('marker is behind optical camera')
    k = _intrinsics(camera)
    return points[:, :2] / points[:, 2:3] * k[:2] + k[2:], points[:, 2]


def rigid_difference(reference, candidate):
    """Translation (mm) and rotation (deg) from one rigid transform to another."""
    a, b = rigid(reference), rigid(candidate)
    angle = np.degrees(np.arccos(np.clip((np.trace(b[:3, :3] @ a[:3, :3].T) - 1) / 2, -1, 1)))
    return {'translation_mm': float(np.linalg.norm(b[:3, 3] - a[:3, 3]) * 1000.),
            'rotation_deg': float(angle)}


def solve_wrist_calibration(observations, camera, marker_size_m, initial_T_link6_camera,
                            *, source_calibration_id, max_nfev=500,
                            mesh_residual=None, mesh_sigma_px=2., mesh_observation_builder=None,
                            initial_T_link6_marker=None, progress=None):
    """Fit only fit rows, evaluate untouched holdout rows, and report observability.

    Rows contain id, split (fit/holdout), T_world_observer_link6,
    T_world_marker, corners_uv. Split IDs and a complete input digest are returned
    for provenance. No automatic repartition, outlier deletion or joint fitting.

    Passing `initial_T_link6_marker` also estimates the *target* arm's sticker
    installation (six more degrees of freedom) jointly with the observer camera. The
    two are identifiable from opposite-wrist observations - the camera and the marker
    sit on different arms whose relative pose the encoders already give - which is the
    point: the head half solves the mount next to a camera, twelve offsets and a mesh,
    so its mount is the weakest of its twenty-six parameters, while the wrist half can
    spend 32 close-range views of that same sticker on the mount alone.
    """
    if not isinstance(source_calibration_id, str) or not source_calibration_id.strip():
        raise ValueError('source calibration identity required')
    k = _intrinsics(camera)
    corners = marker_corners(marker_size_m)
    initial = rigid(initial_T_link6_camera).copy()
    free_marker = initial_T_link6_marker is not None
    initial_marker = rigid(initial_T_link6_marker).copy() if free_marker else None
    if mesh_residual is not None and (not callable(mesh_residual) or not np.isfinite(mesh_sigma_px) or mesh_sigma_px <= 0):
        raise ValueError('mesh residual callback and positive finite mesh sigma required')
    if mesh_observation_builder is not None and not callable(mesh_observation_builder):
        raise ValueError('mesh observation builder must be callable')
    rows, ids = [], set()
    for row in observations:
        identity, split = row['id'], row['split']
        if not isinstance(identity, str) or not identity or identity in ids:
            raise ValueError('unique nonempty observation IDs required')
        if split not in ('fit', 'holdout'):
            raise ValueError('split must be explicitly fit or holdout')
        ids.add(identity)
        observer = rigid(row['T_world_observer_link6']).copy()
        marker = rigid(row['T_world_marker']).copy()
        uv = np.asarray(row['corners_uv'], float).copy()
        if uv.shape != (4, 2) or not np.isfinite(uv).all():
            raise ValueError('four finite measured marker corners required')
        relative = np.linalg.inv(observer) @ marker
        points = corners @ relative[:3, :3].T + relative[:3, 3]
        rows.append((identity, split, points, uv, observer, marker, row, relative))
    fit = [row for row in rows if row[1] == 'fit']
    holdout = [row for row in rows if row[1] == 'holdout']
    if len(fit) < 3 or not holdout:
        raise ValueError('at least three fit poses and one independent holdout pose required')
    payload = {'source_calibration_id': source_calibration_id, 'camera': k.tolist(),
               'marker_size_m': float(marker_size_m), 'initial_T_link6_camera': initial.tolist(),
               **({'initial_T_link6_marker': initial_marker.tolist()} if free_marker else {}),
               'observations': [{'id': r[0], 'split': r[1], 'corners_uv': r[3].tolist(),
                                 'T_world_observer_link6': r[4].tolist(),
                                 'T_world_marker': r[5].tolist(),
                                 **({'mesh_observation': r[6].get('mesh_observation')} if 'mesh_observation' in r[6] else {})} for r in rows]}
    digest = hashlib.sha256(json.dumps(payload, sort_keys=True, allow_nan=False).encode()).hexdigest()
    # Scale numerical Jacobian coordinates: centimetres and 0.1 radians.
    scale = np.array([.01, .01, .01, .1, .1, .1])

    def delta_of(values):
        delta = np.eye(4)
        delta[:3, :3] = Rotation.from_rotvec(values[3:] * scale[3:]).as_matrix()
        delta[:3, 3] = values[:3] * scale[:3]
        return delta

    def transform(x):
        return initial @ delta_of(x[:6])

    def marker_perturbation(x):
        """The mount correction itself, applied on the right of the frozen mount."""
        return delta_of(x[6:]) if free_marker else None

    def marker_transform(x):
        return initial_marker @ delta_of(x[6:])

    def marker_points(row, marker_delta):
        """Sticker corners in the observer's link6 frame under a mount perturbation.

        The mount is perturbed in the target arm's link6 frame (`T_link6_marker @ D`),
        so every row's frozen `T_world_marker` picks up the same right-multiplied
        delta and `inv(observer) @ T_world_marker` becomes `relative @ D`.
        """
        if marker_delta is None:
            return row[2]
        relative = row[7] @ marker_delta
        return corners @ relative[:3, :3].T + relative[:3, 3]

    def marker_errors(T, selected, marker_delta=None):
        points = np.concatenate([marker_points(row, marker_delta) for row in selected])
        optical = (points - T[:3, 3]) @ T[:3, :3]
        depth = optical[:, 2]
        uv = optical[:, :2] / np.maximum(depth[:, None], 1e-6) * k[:2] + k[2:]
        measured = np.concatenate([row[3] for row in selected])
        return uv - measured, depth

    def mesh_errors(T, selected):
        if mesh_residual is None and mesh_observation_builder is None:
            return np.empty(0, float)
        values = []
        for row in selected:
            source = row[6]
            value = np.asarray(mesh_residual(T, source), float).reshape(-1) if mesh_residual else np.empty(0)
            if value.size and not np.isfinite(value).all():
                raise ValueError('mesh residual must be finite')
            values.extend(value.tolist())
        return np.asarray(values, float)

    def errors(T, selected, marker_delta=None):
        marker, depth = marker_errors(T, selected, marker_delta)
        return marker, depth, mesh_errors(T, selected)

    def fixed_mesh_rows(selected, candidate):
        if mesh_observation_builder is None:
            return selected
        return [(*row[:6], dict(row[6], mesh_observation=_mesh_observation(
                    mesh_observation_builder(candidate, row[6]))), row[7]) for row in selected]

    def objective(x, selected):
        marker, _, mesh = errors(transform(x), selected, marker_perturbation(x))
        return np.r_[marker.ravel(), mesh / float(mesh_sigma_px)]

    x = np.zeros(12 if free_marker else 6); total_nfev = 0
    # Refresh fit-only correspondences between solves, never inside numerical
    # differentiation. Eight bounded ICP steps match the head solver budget.
    iteration_budget = 8 if mesh_observation_builder is not None else 1
    for iteration in range(iteration_budget):
        fixed_fit = fixed_mesh_rows(fit, transform(x))
        result = least_squares(lambda value: objective(value, fixed_fit),
                               x, max_nfev=max_nfev, x_scale='jac',
                               ftol=1e-11, xtol=1e-11, gtol=1e-11)
        total_nfev += result.nfev
        if progress is not None:
            progress(dict(fraction=.8*(iteration+1)/iteration_budget, label="wrist_optimizer",
                          completed=iteration+1, total=iteration_budget))
        change = np.max(np.abs(result.x-x)); x = result.x
        if change <= 1e-8:
            break
    if progress is not None:
        progress(dict(fraction=.8, label="wrist_validation", completed=0, total=1))
    T = rigid(transform(result.x))
    correction = marker_perturbation(result.x)
    marker = rigid(marker_transform(result.x)) if free_marker else None
    # result.jac uses the same fixed fit correspondences as the final solve.
    singular = np.linalg.svd(result.jac, compute_uv=False)
    rank = int(np.sum(singular > singular[0] * 1e-8)) if singular[0] > 0 else 0
    expected_rank = 12 if free_marker else 6

    def metrics(selected, candidate=T, marker_delta=None):
        selected = fixed_mesh_rows(selected, candidate)
        residual, depths, mesh = errors(candidate, selected, marker_delta)
        distance = np.linalg.norm(residual, axis=1)
        coverage = [float(row[6]['mesh_observation'].get('coverage',1.))
                    for row in selected if isinstance(row[6].get('mesh_observation'),dict)]
        return {'poses': len(selected), 'corners': len(distance),
                'rms_px': float(np.sqrt(np.mean(distance ** 2))),
                'mean_px': float(distance.mean()), 'p95_px': float(np.percentile(distance, 95)),
                'max_px': float(distance.max()), 'positive_depth': bool(np.all(depths > 0)),
                'min_depth_m': float(depths.min()),
                'mesh_residual_count': int(mesh.size),
                'mesh_rms_px': float(np.sqrt(np.mean(mesh ** 2))) if mesh.size else None,
                'mesh_mean_abs_px': float(np.mean(np.abs(mesh))) if mesh.size else None,
                'mesh_coverage': float(np.mean(coverage)) if coverage else None}

    fit_metrics, holdout_metrics = metrics(fit, T, correction), metrics(holdout, T, correction)
    report = {'T_link6_camera': T.tolist(), 'source_calibration_id': source_calibration_id,
            'input_sha256': digest, 'optimizer_success': bool(result.success),
            'optimizer_message': result.message, 'nfev': total_nfev,
            'numerically_valid': bool(result.success and rank == expected_rank and fit_metrics['positive_depth']
                                     and holdout_metrics['positive_depth']),
            'physical_validated': False, 'fit': fit_metrics, 'holdout': holdout_metrics,
            'objective': 'sticker_corners_plus_mesh' if (mesh_residual is not None or mesh_observation_builder is not None) else 'sticker_corners',
            'mesh_sigma_px': float(mesh_sigma_px) if (mesh_residual is not None or mesh_observation_builder is not None) else None,
            'jacobian_rank': rank, 'jacobian_singular_values': singular.tolist(),
            'jacobian_condition_number': float(singular[0]/singular[-1]) if singular[-1]>1e-12 else None,
            'parameter_count': expected_rank, 'marker_mount_estimated': free_marker,
            'fit_ids': [r[0] for r in fit], 'holdout_ids': [r[0] for r in holdout]}
    if free_marker:
        # Both values are in the same frame, so the difference is the correction the
        # wrist found in the mount the head half had handed it.
        report.update(initial_T_link6_marker=initial_marker.tolist(), T_link6_marker=marker.tolist(),
                      marker_mount_delta=rigid_difference(initial_marker, marker))
    if mesh_observation_builder is not None:
        report.update(correspondence_iterations=iteration+1,
                      jacobian_scope='fit data only, fixed correspondences from final ICP solve')
    if mesh_residual is not None or mesh_observation_builder is not None:
        baseline = solve_wrist_calibration([r[6] for r in rows],camera,marker_size_m,initial,
                                          source_calibration_id=source_calibration_id,max_nfev=max_nfev,
                                          initial_T_link6_marker=initial_T_link6_marker)
        frozen = rigid(baseline['T_link6_camera'])
        baseline.update(fit=metrics(fit,frozen),holdout=metrics(holdout,frozen),
                        mesh_evidence='same frozen RGB edge maps and selected mesh samples as hybrid')
        report['corner_only_comparison'] = baseline
    if progress is not None:
        progress(dict(fraction=1., label="wrist_side_complete", completed=1, total=1))
    return report


def wrist_capture_coverage(dataset, source_completion=None):
    """Describe original capture coverage without changing its frozen contract."""
    planned = [row['id'] for row in dataset['wrist_contract']['planned_frames']]
    captured = [row['planned_id'] for row in dataset['frames']]
    return {'planned_ids': planned, 'captured_ids': captured,
            'planned_count': len(planned), 'captured_count': len(captured),
            'missing_ids': [identity for identity in planned if identity not in set(captured)],
            'partial': set(planned) != set(captured),
            'source_completion_passed': (source_completion or {}).get('passed'),
            'dataset_sha256': hashlib.sha256(json.dumps(dataset, sort_keys=True, allow_nan=False).encode()).hexdigest()}


def solve_wrist_dataset(dataset, geometry, initial_mounts, *, dataset_root=None, progress=None,
                        allow_partial=False, source_completion=None, mesh_scope="full"):
    """Adapt measured pixels using one frozen head snapshot; never read simulation truth.

    Missing planned frames reject the dataset unless explicitly authorized.
    Missing detections and insufficient fit/holdout coverage always reject it.
    The default 2 px gate is fixed before fitting.
    """
    if mesh_scope not in ('full', 'arm'):
        raise ValueError('wrist mesh_scope must be full or arm')
    if progress is not None:
        progress(dict(fraction=0., label='wrist_rgb_evidence', completed=0, total=len(dataset['frames'])))
    if not isinstance(allow_partial, bool):raise ValueError('allow_partial must be boolean')
    coverage = wrist_capture_coverage(dataset, source_completion)
    contract=dataset['wrist_contract'];snapshot=contract['head_snapshot']
    solver = contract.get('solver', 'corners')
    if solver not in ('corners', 'hybrid'):
        raise ValueError('unsupported wrist solver')
    keys=('id','camera_T_world_optical','joint_offsets_rad','marker_ids','T_link6_marker')
    payload={key:snapshot[key] for key in keys}
    digest=hashlib.sha256(json.dumps(payload,sort_keys=True,allow_nan=False).encode()).hexdigest()
    if not isinstance(snapshot['id'],str) or not snapshot['id'] or digest!=snapshot.get('snapshot_sha256'):
        raise ValueError('complete frozen head snapshot with matching digest required')
    if snapshot.get('snapshot_complete') is False or snapshot['marker_ids']!={'left':24,'right':30}:
        raise ValueError('incomplete snapshot or unexpected marker mapping')
    rigid(snapshot['camera_T_world_optical'])
    offsets=np.asarray([snapshot['joint_offsets_rad'][side] for side in ('left','right')],float)
    if offsets.shape!=(2,6) or not np.isfinite(offsets).all():raise ValueError('twelve finite frozen offsets required')
    offsets=offsets.reshape(12)
    mounts={side:rigid(snapshot['T_link6_marker'][str(marker)]) for side,marker in snapshot['marker_ids'].items()}
    estimate_marker=bool(contract.get('estimate_marker_mount',False))
    cameras={side:rectified_camera(contract['cameras'][side]) for side in ('left','right')}
    for side in cameras:
        if not np.array_equal(rigid(initial_mounts[side]),rigid(contract['nominal_mounts'][side])):
            raise ValueError('initial wrist mount differs from frozen plan')
    acceptance = contract.get('acceptance', {})
    threshold=float(acceptance.get('max_holdout_rms_px',2.))
    mesh_threshold=float(acceptance.get('max_holdout_mesh_rms_px',2.))
    if not np.isfinite(threshold) or threshold<=0:raise ValueError('positive predeclared holdout RMS limit required')
    planned={}
    for row in contract['planned_frames']:
        identity=row['id'];observer=row['observer'];target=row['target']
        if not isinstance(identity,str) or not identity or identity in planned:
            raise ValueError('unique nonempty planned IDs required')
        if (observer,target) not in (('left','right'),('right','left')) or row['split'] not in ('fit','holdout'):
            raise ValueError('invalid planned wrist observation')
        planned[identity]=row
    observations={side:[] for side in cameras};mesh_context={side:{} for side in cameras}
    seen=set();frame_ids=set()
    for row in dataset['frames']:
        if row.get('valid_for_fit') is False:raise ValueError('invalid captured wrist frame')
        identity=row['planned_id'];frame_id=row['id']
        if identity not in planned or identity in seen or not isinstance(frame_id,str) or not frame_id or frame_id in frame_ids:
            raise ValueError('unexpected or duplicate captured frame')
        expected=planned[identity]
        if any(row[key]!=expected[key] for key in ('observer','target','split')):
            raise ValueError('captured wrist identity/split differs from frozen plan')
        q=np.asarray(row.get('camera_feedback',{}).get(row['observer'],{}).get('q_reported12',row['q_reported_rad']),float)
        if q.shape!=(12,) or not np.isfinite(q).all():raise ValueError('twelve finite reported joints required')
        side,target=row['observer'],row['target'];marker=str(snapshot['marker_ids'][target])
        detected=row.get('camera_detections',{}).get(side,{})
        if marker not in detected:raise ValueError('planned opposite marker missing: '+identity)
        corners=np.asarray(detected[marker],float)
        if corners.shape!=(4,2) or not np.isfinite(corners).all():raise ValueError('four finite measured corners required')
        if np.any(corners<0) or np.any(corners>=[cameras[side]['width'],cameras[side]['height']]):
            raise ValueError('measured wrist corners outside image')
        frames=geometry.frames(q+offsets)
        captured = {'id':identity,'split':row['split'],
            'T_world_observer_link6':frames[side]['link6'].tolist(),
            'T_world_marker':(frames[target]['link6']@mounts[target]).tolist(),'corners_uv':corners.tolist()}
        if solver == 'hybrid':
            if dataset_root is None:
                raise ValueError('dataset_root required for hybrid wrist RGB evidence')
            images = row.get('camera_images', {})
            image_name = images.get(side) or row.get('image')
            if not isinstance(image_name, str) or not image_name:
                raise ValueError('hybrid wrist frame missing observer camera image')
            image_path = (Path(dataset_root) / image_name).resolve()
            root = Path(dataset_root).resolve()
            if not image_path.is_file() or not image_path.is_relative_to(root):
                raise ValueError('hybrid wrist image missing or outside dataset_root')
            evidence, context = _wrist_mesh_evidence(
                geometry, {**row, 'q_reported_rad': q.tolist()}, side, target,
                cameras[side], initial_mounts[side], offsets, image_path, mesh_scope=mesh_scope)
            captured['mesh_observation'] = evidence
            mesh_context[side][identity] = context
        observations[side].append(captured)
        seen.add(identity);frame_ids.add(frame_id)
        if progress is not None:
            progress(dict(fraction=.2*len(seen)/len(dataset["frames"]), label="wrist_rgb_evidence",
                          completed=len(seen), total=len(dataset["frames"])))
    if seen!=set(planned) and not allow_partial:raise ValueError('incomplete planned frame coverage: '+', '.join(sorted(set(planned)-seen)))
    contract_digest=hashlib.sha256(json.dumps(contract,sort_keys=True,allow_nan=False).encode()).hexdigest()
    marker_size=float(contract.get('marker_size_m',geometry.marker_size))
    marker_corners(marker_size)
    mesh_residuals = None
    mesh_builders = None
    if solver == 'hybrid':
        if not all('mesh_observation' in row for side_rows in observations.values() for row in side_rows):
            raise ValueError('hybrid wrist dataset missing mesh observations')
        mesh_residuals = {side: (lambda T, row, camera=cameras[side]: mesh_observation_residual(T, row, camera))
                          for side in cameras}
        mesh_builders = {
            side: (lambda T, row, contexts=mesh_context[side]:
                   _refresh_wrist_mesh_evidence(T, contexts[row['id']]))
            for side in cameras}
    reports={}
    for side_index, (side,rows) in enumerate(observations.items()):
        target='right' if side=='left' else 'left'
        report=solve_wrist_calibration(rows,cameras[side],marker_size,initial_mounts[side],
                                       source_calibration_id=snapshot['id'],
                                       mesh_residual=None if mesh_residuals is None else mesh_residuals[side],
                                       mesh_observation_builder=None if mesh_builders is None else mesh_builders[side],
                                       initial_T_link6_marker=mounts[target] if estimate_marker else None,
                                       progress=(None if progress is None else
                                           lambda value: progress({**value, "fraction": .2+.4*(side_index+value["fraction"]),
                                                                   "label": side+"_"+value["label"]})))
        hybrid = solver == 'hybrid'
        mesh_valid = (not hybrid or (
            report['holdout']['mesh_residual_count'] > 0 and
            np.isfinite(report['holdout']['mesh_rms_px']) and
            report['holdout']['mesh_rms_px'] <= mesh_threshold and
            float(report['holdout'].get('mesh_coverage') or 0.) >= .7))
        report.update(mesh_scope=mesh_scope if hybrid else None, observer=side,target='right' if side=='left' else 'left',
            head_snapshot_sha256=digest,contract_sha256=contract_digest,coverage_complete=not coverage['partial'],
            capture_coverage=coverage, partial_capture_authorized=allow_partial, marker_size_m=marker_size,
            acceptance={'max_holdout_rms_px':threshold, **({'max_holdout_mesh_rms_px':mesh_threshold} if hybrid else {})},
            accepted=bool(report['numerically_valid'] and report['holdout']['rms_px']<=threshold and mesh_valid),
            # The mount this solve attributes to the *target* arm's sticker. Every side
            # estimates exactly one of the two, so a complete pair carries both and the
            # head half can re-fit with them held fixed.
            marker_mount=({'marker_id':snapshot['marker_ids'][target],'side':target,
                           'T_link6_marker':report['T_link6_marker'],
                           'delta_from_head_snapshot':report['marker_mount_delta']}
                          if estimate_marker else None),
            environment=contract.get('environment','simulation'),
            # Inherit the recorded robot source. Deriving this from `environment`
            # would mark every hardware-link result as "not simulation", including
            # runs against a simulated robot (legacy fallback kept for old contracts).
            simulation_only=bool(contract.get('simulation_only',contract.get('environment','simulation')!='hardware')),
            robot_source=contract.get('robot_source','unknown'),physical_validated=False,camera=json.loads(json.dumps(cameras[side])))
        reports[side]=report
    return reports
