"""Two 15 mm ArUco stickers plus RGB mesh silhouette; simulation candidate only.

Known mounts and known installation planes are explicit metrology assumptions.
Free mounts add twelve nuisance parameters; plane mounts add six (u/v/yaw).
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import time

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from . import joint_head_markerless as mesh
from .joint_head_isaac_fit import (BOUNDS, SCALE, Model, camera_candidate,
                                   coverage_report, refine_border, rigid, statistics)
from .urdf_mesh import relative_file


def detect(image):
    """Decode real pixels and refine sampled border lines; reject undersampled tags."""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
    detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), params)
    corners, ids, _ = detector.detectMarkers(gray)
    found, diagnostics = {}, []
    if ids is None:
        return found, diagnostics
    if len(set(ids.ravel().tolist())) != len(ids):
        raise ValueError('duplicate decoded marker ID')
    for marker_id, value in zip(ids.ravel().tolist(), corners):
        xy = value.reshape(4, 2).astype(float)
        side = float(np.linalg.norm(np.roll(xy, -1, axis=0)-xy, axis=1).min())
        entry = {'id': marker_id, 'minimum_side_px': side, 'accepted': False}
        diagnostics.append(entry)
        if side < 24:
            entry['reason'] = 'under24px'
            continue
        if (xy < 4).any() or (xy >= np.array(gray.shape[::-1])-4).any():
            entry['reason'] = 'clipped'
            continue
        refined = refine_border(gray, xy)
        if refined is None:
            entry['reason'] = 'border_refinement_rejected'
            continue
        if not cv2.isContourConvex(refined.astype(np.float32)):
            entry['reason'] = 'nonconvex_refinement'
            continue
        found[marker_id] = np.asarray(refined, float)
        entry.update(accepted=True, method='border_midpoint')
    return found, diagnostics


MOUNT_DOF = {'known': 0, 'plane': 3, 'free': 6}
MOUNT_NAMES = {'known': (), 'plane': ('u_m', 'v_m', 'yaw_rad'),
               'free': ('dx_m', 'dy_m', 'dz_m', 'rx_rad', 'ry_rad', 'rz_rad')}


def resolve_mount_mode(x, mount_mode=None):
    modes = {18+2*dof: mode for mode, dof in MOUNT_DOF.items()}
    mode = modes.get(len(x)) if mount_mode is None else mount_mode
    if mode not in MOUNT_DOF or len(x) != 18+2*MOUNT_DOF[mode] or not np.isfinite(x).all():
        raise ValueError('invalid mount mode or parameter vector')
    return mode


def marker_indices(markers):
    """Installation parameter blocks follow manifest order, never decoded ID order."""
    ids = [m['id'] for m in markers]
    if (len(ids) != 2 or any(type(i) is not int or i not in range(50) for i in ids)
            or len(set(ids)) != 2):
        raise ValueError('exactly two distinct integer DICT_4X4_50 IDs (0..49) required')
    return {marker_id: index for index, marker_id in enumerate(ids)}


def marker_mount(marker, x, mount_mode=None, *, marker_index=None):
    mode = resolve_mount_mode(x, mount_mode)
    nominal = rigid(marker['T_link_marker'])
    dof = MOUNT_DOF[mode]
    if mode == 'known':
        return nominal.copy()
    # Compatibility for standalone legacy ID0/ID1 calls; dataset paths pass a slot.
    if marker_index is None:
        marker_index = marker['id']
    if type(marker_index) is not int or marker_index not in (0, 1):
        raise ValueError('explicit manifest marker_index (0 or 1) required for this ID')
    delta = np.asarray(x[18+dof*marker_index:18+dof*(marker_index+1)])
    if mode == 'free':
        return camera_candidate(nominal, delta)
    motion = np.eye(4)
    motion[:2, 3] = delta[:2]
    motion[:3, :3] = Rotation.from_rotvec([0., 0., delta[2]]).as_matrix()
    return nominal @ motion


def marker_predictions(model, manifest, row, x):
    mode = resolve_mount_mode(x)
    fk = mesh.frames_with_feedback(model, row, manifest, x[6:18])
    camera = camera_candidate(manifest['camera_nominal_T_base_optical'], x[:18])
    markers = {m['id']: m for m in manifest['markers']}
    indices = marker_indices(manifest['markers'])
    points = []
    for marker_id in row['marker_ids']:
        marker = markers[marker_id]
        s = marker['size_m']/2
        corners = np.array([[-s, -s, 0], [s, -s, 0], [s, s, 0], [-s, s, 0]])
        mount = marker_mount(marker, x, mode, marker_index=indices[marker_id])
        T = fk[marker['link']] @ mount
        points.extend(corners @ T[:3, :3].T + T[:3, 3])
    pc = (np.asarray(points).reshape(-1, 3)-camera[:3, 3]) @ camera[:3, :3]
    return mesh.project(pc, manifest['intrinsics']), pc[:, 2]


def marker_residual(model, manifest, rows, x):
    return np.concatenate([(marker_predictions(model, manifest, r, x)[0]-r['uv']).ravel() for r in rows])


def parameter_settings(mount_mode):
    if isinstance(mount_mode, bool):  # Existing callers used unknown_mounts=True/False.
        mount_mode = 'free' if mount_mode else 'known'
    if mount_mode not in MOUNT_DOF:
        raise ValueError('invalid mount mode')
    ntranslation, nrotation = {'known': (0, 0), 'plane': (2, 1), 'free': (3, 3)}[mount_mode]
    mount_scale = np.tile([.005]*ntranslation + [np.deg2rad(2)]*nrotation, 2)
    mount_bounds = np.tile([.01]*ntranslation + [np.deg2rad(15)]*nrotation, 2)
    return np.r_[SCALE, mount_scale], np.r_[BOUNDS, mount_bounds]


def weighted_soft_l1(z, weights):
    """SciPy rho, rho', rho'': count weights outside rho preserve noise scales."""
    t = 1 + z
    return np.asarray([2*(np.sqrt(t)-1), 1/np.sqrt(t), -.5/t**1.5])*weights


def solve_markers(model, manifest, rows, x, scale, bounds, marker_sigma=.5):
    return least_squares(lambda z: marker_residual(model, manifest, rows, z*scale)/marker_sigma, x/scale,
                         bounds=(-bounds/scale, bounds/scale), loss='soft_l1', f_scale=1.,
                         max_nfev=150, ftol=1e-9, xtol=1e-9, gtol=1e-9)


def jacobian_report(residual, x, scale, names, mask=None):
    """Rank of the fit Jacobian, optionally over a subset of the parameters.

    `mask` names the parameters that are actually optimized: a frozen mount still
    moves the residual when it is perturbed, so without the mask a fit that froze it
    would keep reporting the rank of a fit that did not.
    """
    indices = np.arange(len(x)) if mask is None else np.asarray(mask, int)
    if indices.ndim != 1 or not len(indices) or indices.min() < 0 or indices.max() >= len(x):
        raise ValueError('invalid parameter mask')
    basis = np.zeros((len(indices), len(x)))
    basis[np.arange(len(indices)), indices] = 1e-3
    jac = np.column_stack([(residual(x+e*scale)-residual(x-e*scale))/.002 for e in basis])
    _, singular, vectors = np.linalg.svd(jac, full_matrices=False)
    return {'rank': int(np.sum(singular > max(singular[0]*1e-6, 1e-9))),
            'parameters': len(x), 'optimized_parameters': len(indices),
            'free_indices': indices.tolist(), 'singular_values': singular.tolist(),
            'condition_number': float(singular[0]/singular[-1]) if singular[-1] > 1e-12 else None,
            'parameter_scale': scale.tolist(),
            'weakest_directions_scaled': [dict(zip(names, v.tolist())) for v in vectors[-3:]],
            'scope': 'fit data only, fixed final mesh correspondences, no priors; rank does not prove accuracy'}


def fit_dataset(dataset, mesh_path, output, *, unknown_mounts=False, mount_mode=None, iterations=12,
                samples=400, marker_sigma=.5, mesh_sigma=2., mesh_effective_count=32.):
    started = time.monotonic()
    if mount_mode is not None and unknown_mounts:
        raise ValueError('use mount_mode or legacy unknown_mounts, not both')
    mount_mode = mount_mode or ('free' if unknown_mounts else 'known')
    scale, bounds = parameter_settings(mount_mode)
    estimate_mounts = mount_mode != 'known'
    path = Path(dataset).resolve(strict=True)
    if path.is_dir():
        path = path/'dataset.json'
    output = Path(output).resolve()
    if output.exists():
        raise ValueError('output directory must be new')
    if not 1 <= iterations <= 20 or not 40 <= samples <= 2000:
        raise ValueError('invalid bounded iteration/sample budget')
    if not np.isfinite([marker_sigma, mesh_sigma, mesh_effective_count]).all() or min(marker_sigma, mesh_sigma, mesh_effective_count) <= 0:
        raise ValueError('positive finite residual scales required')
    manifest = json.loads(path.read_text())
    if manifest.get('schema') != 'joint-head-isaac-v1' or len(manifest['joint_names']) != 12:
        raise ValueError('expected joint-head-isaac-v1 and twelve arm joints')
    rigid(manifest['camera_nominal_T_base_optical'])
    k = manifest['intrinsics']
    if not np.isfinite([k[n] for n in ('fx', 'fy', 'cx', 'cy')]).all() or min(k['fx'], k['fy']) <= 0:
        raise ValueError('invalid intrinsics')
    urdf = relative_file(path.parent, manifest['urdf_path'])
    model = Model(urdf, manifest['joint_names'])
    markers = manifest['markers']
    indices = marker_indices(markers)
    for m in markers:
        if m['link'] not in model.links or not np.isclose(m['size_m'], .015, atol=1e-9, rtol=0):
            raise ValueError('expected valid rigid host link and 0.015 m black-border size')
        rigid(m['T_link_marker'])
    rows = [dict(r) for r in manifest['frames']]
    if not rows or len(set(r['id'] for r in rows)) != len(rows) or any(r['split'] not in ('fit', 'holdout') for r in rows):
        raise ValueError('invalid frame IDs or split')
    images = [relative_file(path.parent, r['image']) for r in rows]
    if len(set(images)) != len(images):
        raise ValueError('reused image')
    completion = path.parent/'completion.json'
    if not completion.exists() or json.loads(completion.read_text()).get('passed') is not True:
        raise ValueError('passed capture completion required')
    sources = [path, urdf, Path(mesh_path).resolve(strict=True), completion,
               Path(__file__).resolve(), Path(mesh.__file__).resolve(),
               Path(__file__).with_name('joint_head_isaac_fit.py').resolve(), *images]
    snapshots = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    meshes = mesh.load_meshes(mesh_path, model.links)
    pattern, edges = 'left|right', []
    if not any(re.search(pattern, m['name']) for m in meshes):
        raise ValueError('no left/right arm meshes')
    for row, image_path in zip(rows, images):
        independent = {j[0] for j in model.joints if j[-1] != 'fixed'}-set(model.names)-set(model.mimics)
        if not independent.issubset(row.get('q_all_reported', {})):
            raise ValueError('complete independent non-arm q_all_reported required')
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError('unreadable image')
        found, row['detector'] = detect(image)
        row['marker_ids'] = sorted(indices.keys() & found.keys())
        row['uv'] = np.concatenate([found[i] for i in row['marker_ids']]) if row['marker_ids'] else np.empty((0, 2))
        parts = mesh.camera_mesh_parts(model, manifest, row, meshes, np.zeros(18))
        selected = [p for m, p in zip(meshes, parts) if re.search(pattern, m['name'])]
        mask = np.isfinite(mesh.raster_depth(selected, k, image.shape[:2])).astype(np.uint8)
        edges.append(mesh.foreground_edges(image, mask))
    coverage = coverage_report(manifest, rows)
    # Missing detections are recorded, never silently converted to an easier dataset.
    adequate = all(sum(marker_id in r['marker_ids'] for r in rows if r['split'] == split) >= count
                   for marker_id in indices for split, count in (('fit', 4), ('holdout', 2)))
    output.mkdir(parents=True)
    observations = [{**r, 'uv': r['uv'].tolist()} for r in rows]
    (output/'observations.json').write_text(json.dumps({'coverage': coverage, 'frames': observations}, indent=2))
    if not adequate:
        raise ValueError('insufficient per-marker evidence: need >=4 fit and >=2 holdout each; see observations.json')
    for i, edge in enumerate(edges):
        if not cv2.imwrite(str(output/f'observed-mask-{i:03d}.png'), edge['mask']*255):
            raise IOError('could not save segmentation evidence')
    training = [i for i, r in enumerate(rows) if r['split'] == 'fit']
    fit_rows, fit_edges = [rows[i] for i in training], [edges[i] for i in training]
    x = np.zeros(len(scale))
    nominal, nominal_rows, corr = mesh.evaluate(model, manifest, rows, meshes, edges, x[:18], pattern, samples, output, 'nominal')
    trace = []
    if not estimate_mounts:
        result = solve_markers(model, manifest, fit_rows, x, scale, bounds, marker_sigma)
        x = result.x*scale
        trace.append({'phase': 'known_mount_marker_warmstart', 'success': bool(result.success), 'parameters': x.tolist()})
    initialization = None
    loss_weights = np.r_[np.full(len(fit_rows)*samples, mesh_effective_count/samples),
                         np.ones(sum(r['uv'].size for r in fit_rows))]
    current = [corr[i] for i in training]
    def residual(trial, corrs, marker_weight=1.):
        mr = mesh.objective(model, manifest, fit_rows, meshes, corrs, trial[:18], samples)/mesh_sigma
        return np.r_[mr, marker_residual(model, manifest, fit_rows, trial)/marker_sigma*marker_weight]
    # Unknown installation cannot safely initialize joints from a fixed-mount solve.
    for iteration in range(iterations + (4 if estimate_mounts else 0)):
        warm = estimate_mounts and iteration < 4
        active = np.arange(6 if warm and iteration < 2 else 18 if warm else len(x))
        _, _, current = mesh.evaluate(model, manifest, fit_rows, meshes, fit_edges, x[:18], pattern, samples)
        base = x.copy()
        def objective(z):
            trial = base.copy()
            trial[active] = z*scale[active]
            return residual(trial, current, 0. if warm else 1.)
        result = least_squares(objective, x[active]/scale[active],
                               bounds=(-bounds[active]/scale[active], bounds[active]/scale[active]),
                               loss=lambda z: weighted_soft_l1(z, loss_weights), f_scale=1.,
                               max_nfev=40, diff_step=1e-3, ftol=1e-6, xtol=1e-6)
        x[active] = result.x*scale[active]
        trace.append({'iteration': iteration, 'phase': 'mesh_warmstart' if warm else 'joint_hybrid',
                      'nfev': result.nfev, 'optimizer_success': bool(result.success), 'parameters': x.tolist()})
        print(json.dumps(trace[-1]), flush=True)
        if (estimate_mounts and iteration == 3) or (not estimate_mounts and iteration == 0):
            initial_x = x if estimate_mounts else np.asarray(trace[0]['parameters'])
            initialization = {'method': 'mesh_only' if estimate_mounts else 'known_mount_marker_only',
                              'camera_T_base_optical': camera_candidate(manifest['camera_nominal_T_base_optical'], initial_x[:18]).tolist(),
                              'joint_offsets_rad': dict(zip(model.names, initial_x[6:18].tolist())),
                              'parameters': initial_x.tolist()}
    metrics, frame_reports, corrs = mesh.evaluate(model, manifest, rows, meshes, edges, x[:18], pattern, samples, output, 'fitted')
    marker_metrics, marker_frames = {}, []
    for i, (row, edge) in enumerate(zip(rows, edges)):
        uv, depth = marker_predictions(model, manifest, row, x)
        errors = np.linalg.norm(uv-row['uv'], axis=1)
        marker_frames.append({'id': row['id'], 'split': row['split'], 'marker_ids': row['marker_ids'],
                              'positive_depth': bool(np.all(depth > .01)),
                              **(statistics(errors) if len(errors) else {'corners': 0, 'mean_px': None})})
        image = cv2.imread(str(output/f'fitted-{i:03d}.png'))
        for p, q in zip(uv, row['uv']):
            if np.max(np.abs(np.r_[p, q])) < 1e6:
                cv2.circle(image, tuple(np.rint(q).astype(int)), 3, (255, 0, 255), 1)
                cv2.drawMarker(image, tuple(np.rint(p).astype(int)), (0, 255, 255), cv2.MARKER_CROSS, 7, 1)
        cv2.imwrite(str(output/f'fitted-{i:03d}.png'), image)
    for split in ('fit', 'holdout'):
        subset = [r for r in rows if r['split'] == split]
        errors = np.linalg.norm(marker_residual(model, manifest, subset, x).reshape(-1, 2), axis=1)
        marker_metrics[split] = statistics(errors)
        marker_metrics[split]['frame_macro_mean_px'] = float(np.mean([r['mean_px'] for r in marker_frames if r['split'] == split and r['corners']]))
    names = ['camera_dx_m', 'camera_dy_m', 'camera_dz_m', 'camera_rx_rad', 'camera_ry_rad', 'camera_rz_rad', *model.names]
    names += [f'marker{i}_{n}' for i in indices for n in MOUNT_NAMES[mount_mode]]
    current = [corrs[i] for i in training]
    mounts = {str(m['id']): marker_mount(m, x, mount_mode, marker_index=indices[m['id']]).tolist() for m in markers}
    report = {'schema': 'joint-head-hybrid-fit-v1', 'status': 'EXPERIMENTAL_CANDIDATE',
              'camera_T_base_optical': camera_candidate(manifest['camera_nominal_T_base_optical'], x[:18]).tolist(),
              'joint_offsets_rad': dict(zip(model.names, x[6:18].tolist())), 'parameters': x.tolist(),
              'mount_mode': {'known': 'known_exact_mount_assumption', 'free': 'unknown_6dof_no_prior',
                             'plane': 'known_plane_uv_yaw_no_prior'}[mount_mode],
              'mount_parameterization': mount_mode, 'parameter_names': names,
              'mount_marker_ids_in_parameter_order': list(indices),
              'mount_assumption': {'known': 'Full nominal T_link_marker is exact.',
                                   'free': 'Each marker translates/rotates freely within bounds; corrections in host link.',
                                   'plane': 'Nominal marker plane normal and normal displacement in host link are exactly known; T=T_nominal@Trans(u,v,0)@Rz(yaw). In-plane u/v/yaw are estimated; no out-of-plane error is permitted.'}[mount_mode],
              'estimated_T_link_marker': mounts, 'mount_prior': None,
              'initialization': initialization,
              'marker_metrics': marker_metrics, 'marker_frames': marker_frames, 'coverage': coverage,
              'positive_depth': all(r['positive_depth'] for r in marker_frames),
              'metrics': metrics, 'frames': frame_reports, 'nominal_metrics': nominal, 'nominal_frames': nominal_rows,
              'observability': jacobian_report(lambda t: residual(t, current)*np.sqrt(loss_weights), x, scale, names),
              'marker_only_observability': jacobian_report(lambda t: marker_residual(model, manifest, fit_rows, t)/marker_sigma, x, scale, names),
              'at_parameter_bound': bool(np.any(np.abs(x) > bounds*.999)),
              'bound_parameters': [n for n, v, b in zip(names, x, bounds) if abs(v) > b*.999],
              'bounds_physical': dict(zip(names, bounds.tolist())), 'trace': trace,
              'method': {'dictionary': 'DICT_4X4_50', 'black_border_size_m': .015,
                         'opencv_version': cv2.__version__,
                         'detector': 'decoded pixels; reject minimum side under24px; raw-decoded border midpoint only',
                         'mesh': 'unchanged fit02 nominal-seeded fixed RGB GrabCut exterior and full robot self-occlusion',
                         'marker_sigma_px': marker_sigma, 'mesh_sigma_px': mesh_sigma,
                         'mesh_effective_count_per_frame': mesh_effective_count,
                         'weighting': 'marker rho((coordinate/marker_sigma)^2); mesh (effective_count/samples)*rho((normal/mesh_sigma)^2); rho(z)=2*(sqrt(1+z)-1)',
                         'robust_transition_px': {'marker': marker_sigma, 'mesh': mesh_sigma},
                         'observability_weighting': 'data Jacobian uses sqrt(count_weight)/sigma, without robust influence weights or priors',
                         'samples_per_frame': samples, 'holdout': 'never optimized; missing frames/detections retained in coverage'},
              'used_truth_in_fit': False, 'used_segmentation_in_fit': True, 'used_segmentation_truth_in_fit': False,
              'physical_validated': False, 'applied': False, 'input_sha256': snapshots,
              'limitations': ['RGB local segmentation and nominal geometry assumptions inherited from fit02',
                              'unknown installation may remain weakly observable; full rank is not an accuracy certificate',
                              'sub24px markers rejected; image blur still requires fresh rendered/physical validation',
                              'known mounts require measured geometry; not established by attaching a sticker',
                              'no confidence intervals or collision/hardware certification'],
              'elapsed_seconds': time.monotonic()-started}
    if any(hashlib.sha256(Path(p).read_bytes()).hexdigest() != digest for p, digest in snapshots.items()):
        raise ValueError('input changed during fitting')
    report['input_snapshot_verified'] = True
    with (output/'fit.json').open('x') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('dataset', type=Path)
    parser.add_argument('--meshes', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument('--unknown-mounts', action='store_true', help='legacy alias for --mount-mode free')
    modes.add_argument('--mount-mode', choices=tuple(MOUNT_DOF), help='known exact mount, free 6DOF, or known-plane u/v/yaw')
    parser.add_argument('--iterations', type=int, default=12)
    parser.add_argument('--samples', type=int, default=400)
    args = parser.parse_args()
    report = fit_dataset(args.dataset, args.meshes, args.output, unknown_mounts=args.unknown_mounts,
                         mount_mode=args.mount_mode,
                         iterations=args.iterations, samples=args.samples)
    print(json.dumps({'status': report['status'], 'mount_mode': report['mount_mode'],
                      'marker_metrics': report['marker_metrics'], 'observability': report['observability']}, indent=2))
