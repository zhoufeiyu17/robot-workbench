"""Markerless RGB/mesh experiment, not a certified physical calibration.

NPZ: names (unicode link names), vertices_i (link-local metres), faces_i (indices).
Only RGB edges, nominal meshes and reported joint feedback enter the fit. Full
robot depth is predicted from the candidate, never read from simulator truth.
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
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from .joint_head_isaac_fit import Model, SCALE, BOUNDS, camera_candidate, rigid
from .urdf_mesh import relative_file


def frames_with_feedback(model, row, manifest, offsets):
    """Retain all measured joints; selected arm joints receive offsets exactly once."""
    if hasattr(model, 'frames_with_feedback'):
        return model.frames_with_feedback(row, manifest, offsets)
    values = {j[0]: 0. for j in model.joints}
    extra = row.get('q_all_reported', {})
    names = row.get('all_joint_names', manifest.get('all_joint_names'))
    positions = row.get('all_joint_positions_rad')
    if names is not None or positions is not None:
        if names is None or positions is None or len(names) != len(positions) or len(set(names)) != len(names):
            raise ValueError('invalid full joint feedback')
        extra = {**extra, **dict(zip(names, positions))}
    if not isinstance(extra, dict) or set(extra) - set(values) or not np.isfinite(list(extra.values())).all():
        raise ValueError('invalid additional joint feedback')
    values.update(extra)
    q = np.asarray(row['q_reported_rad'], float)
    if q.shape != (len(model.names),) or not np.isfinite(q).all():
        raise ValueError('invalid arm joint feedback')
    values.update(zip(model.names, q + offsets))
    pending = dict(model.mimics)
    while pending:
        ready = [n for n, (p, _, _) in pending.items() if p not in pending]
        if not ready:
            raise ValueError('invalid mimic dependency')
        for n in ready:
            parent, scale, offset = pending.pop(n)
            values[n] = values[parent] * scale + offset
    frames = {model.root: np.eye(4)}
    for name, parent, child, T, axis, kind in model.joints:
        delta = np.eye(4)
        if kind in ('revolute', 'continuous'):
            delta[:3, :3] = Rotation.from_rotvec(axis * values[name]).as_matrix()
        elif kind == 'prismatic':
            delta[:3, 3] = axis * values[name]
        frames[child] = frames[parent] @ T @ delta
    inverse = np.linalg.inv(frames[model.base])
    return {name: inverse @ T for name, T in frames.items()}


def load_meshes(path, links):
    meshes = []
    with np.load(path, allow_pickle=False) as data:
        for i, name in enumerate(data['names'].tolist()):
            v, f = np.asarray(data[f'vertices_{i}'], float), np.asarray(data[f'faces_{i}'])
            if (name not in links or v.ndim != 2 or v.shape[1] != 3 or f.ndim != 2 or f.shape[1] != 3
                    or not np.issubdtype(f.dtype, np.integer) or not np.isfinite(v).all()
                    or len(f) == 0 or f.min() < 0 or f.max() >= len(v)):
                raise ValueError('invalid link-local mesh')
            # STL often duplicates every vertex; exact welding restores adjacency.
            v, inverse = np.unique(v, axis=0, return_inverse=True)
            f = inverse[f]
            edges = np.sort(np.concatenate([f[:, [0, 1]], f[:, [1, 2]], f[:, [2, 0]]]), axis=1)
            owners = np.tile(np.arange(len(f)), 3)
            unique, edge_ids, count = np.unique(edges, axis=0, return_inverse=True, return_counts=True)
            first, last = np.full(len(unique), -1), np.full(len(unique), -1)
            first[edge_ids] = owners
            last[edge_ids[::-1]] = owners[::-1]
            meshes.append({'name': name, 'v': v, 'f': f, 'edges': unique,
                           'first': first, 'last': last, 'count': count})
    if not meshes:
        raise ValueError('empty mesh bundle')
    return meshes


def project(pc, k):
    return pc[:, :2] / np.maximum(pc[:, 2, None], 1e-8) * [k['fx'], k['fy']] + [k['cx'], k['cy']]


def raster_depth(parts, k, shape):
    """Perspective-correct nearest depth; clipped/behind-near-plane faces skipped."""
    h, w = shape
    depth = np.full(h*w, np.inf)
    for pc, faces in parts:
        uv = project(pc, k)
        tri, screen = pc[faces], uv[faces]
        good = np.all(tri[:, :, 2] > .01, axis=1) & np.isfinite(screen).all(axis=(1, 2))
        tri, screen = tri[good], screen[good]
        lo = np.maximum(np.ceil(screen.min(1)), [0, 0]).astype(int)
        hi = np.minimum(np.floor(screen.max(1)), [w-1, h-1]).astype(int)
        den = ((screen[:, 1, 1]-screen[:, 2, 1])*(screen[:, 0, 0]-screen[:, 2, 0])
               +(screen[:, 2, 0]-screen[:, 1, 0])*(screen[:, 0, 1]-screen[:, 2, 1]))
        good = (hi >= lo).all(1) & (np.abs(den) > 1e-10)
        tri, screen, lo, hi, den = [a[good] for a in (tri, screen, lo, hi, den)]
        count = np.prod(hi-lo+1, axis=1)
        start = 0
        while start < len(tri):
            stop = min(len(tri), start + max(1, int(np.searchsorted(np.cumsum(count[start:]), 250000))))
            c = count[start:stop]
            owner = np.repeat(np.arange(stop-start), c)
            offset = np.arange(c.sum()) - np.repeat(np.cumsum(c)-c, c)
            widths = hi[start:stop, 0]-lo[start:stop, 0]+1
            xx = lo[start:stop, 0][owner]+offset % widths[owner]
            yy = lo[start:stop, 1][owner]+offset // widths[owner]
            s = screen[start:stop][owner]
            a = ((s[:, 1, 1]-s[:, 2, 1])*(xx-s[:, 2, 0])+(s[:, 2, 0]-s[:, 1, 0])*(yy-s[:, 2, 1]))/den[start:stop][owner]
            b = ((s[:, 2, 1]-s[:, 0, 1])*(xx-s[:, 2, 0])+(s[:, 0, 0]-s[:, 2, 0])*(yy-s[:, 2, 1]))/den[start:stop][owner]
            bary = np.column_stack([a, b, 1-a-b])
            good = (bary >= -1e-8).all(1)
            z = 1/(bary[good]/tri[start:stop][owner[good], :, 2]).sum(1)
            np.minimum.at(depth, yy[good]*w+xx[good], z)
            start = stop
    return depth.reshape(h, w)


def camera_mesh_parts(model, manifest, row, meshes, x):
    fk = frames_with_feedback(model, row, manifest, x[6:])
    camera = camera_candidate(manifest['camera_nominal_T_base_optical'], x)
    k, parts = manifest['intrinsics'], []
    for mesh in meshes:
        T = fk[mesh['name']]
        world = mesh['v'] @ T[:3, :3].T + T[:3, 3]
        parts.append(((world-camera[:3, 3]) @ camera[:3, :3], mesh['f']))
    return parts


def exterior_distance(mask):
    boundary = np.zeros(mask.shape, np.uint8)
    curves, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    cv2.drawContours(boundary, curves, -1, 1, 1)
    return cv2.distanceTransform(1-boundary, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)


def contours(model, manifest, row, meshes, x, shape, pattern, samples, occlusion_pattern=None):
    k = manifest['intrinsics']
    parts = camera_mesh_parts(model, manifest, row, meshes, x)
    # By default only calibrated targets occlude each other; camera housings
    # and chassis can surround the optical origin without appearing in RGB.
    depth = raster_depth([part for item, part in zip(meshes, parts)
                          if re.search(occlusion_pattern or pattern, item['name'])], k, shape)
    exterior = exterior_distance(np.isfinite(depth))
    # A one-pixel raster halo avoids rejecting the analytic silhouette itself.
    near = cv2.erode(depth.astype(np.float32), np.ones((3, 3), np.uint8))
    point_rows, normal_rows, link_rows, uv_rows = [], [], [], []
    for mesh_id, (mesh, (pc, faces)) in enumerate(zip(meshes, parts)):
        if not re.search(pattern, mesh['name']):
            continue
        tri = pc[faces]
        facing = np.einsum('ij,ij->i', np.cross(tri[:, 1]-tri[:, 0], tri[:, 2]-tri[:, 0]), tri[:, 0]) < 0
        selected = ((facing[mesh['first']] != facing[mesh['last']]) | (mesh['count'] == 1))
        edge = mesh['edges'][selected]
        if not len(edge):
            continue
        uv = project(pc, k)
        length = np.linalg.norm(uv[edge[:, 1]]-uv[edge[:, 0]], axis=1)
        good = (pc[edge, 2] > .01).all(1) & (length > .2)
        edge, length = edge[good], length[good]
        count = np.minimum(np.ceil(length/2).astype(int), 1000)
        owners = np.repeat(np.arange(len(edge)), count)
        if not len(owners):
            continue
        t = (np.arange(len(owners))-np.repeat(np.cumsum(count)-count, count)+.5)/count[owners]
        a, b = edge[owners, 0], edge[owners, 1]
        points = mesh['v'][a]*(1-t[:, None])+mesh['v'][b]*t[:, None]
        p = pc[a]*(1-t[:, None])+pc[b]*t[:, None]
        projected = project(p, k)
        tangent = uv[b]-uv[a]
        normal = np.column_stack([-tangent[:, 1], tangent[:, 0]]) / np.linalg.norm(tangent, axis=1)[:, None]
        pix = np.rint(projected).astype(int)
        good = ((pix >= 2) & (pix < np.array(shape[::-1])-2)).all(1)
        index = np.flatnonzero(good)
        # Occlusion test tolerance covers depth slope across the raster halo.
        visible = p[index, 2] <= near[pix[index, 1], pix[index, 0]] + .003
        index = index[visible & (exterior[pix[index, 1], pix[index, 0]] <= 2)]
        point_rows.extend(points[index]); normal_rows.extend(normal[index])
        uv_rows.extend(projected[index]); link_rows.extend([mesh_id]*len(index))
    if not point_rows:
        return {'points': np.empty((0, 3)), 'normals': np.empty((0, 2)), 'uv': np.empty((0, 2)), 'links': np.empty(0, int)}
    # Equal-length edge sampling, deterministic cap; no special fingertip weight.
    keep = np.linspace(0, len(point_rows)-1, min(samples, len(point_rows))).astype(int)
    return {'points': np.asarray(point_rows)[keep], 'normals': np.asarray(normal_rows)[keep],
            'uv': np.asarray(uv_rows)[keep], 'links': np.asarray(link_rows)[keep]}


def image_edges(path):
    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError('unreadable RGB image')
    gray = cv2.GaussianBlur(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), (3, 3), .7)
    edges = cv2.Canny(gray, 50, 120)
    y, x = np.nonzero(edges)
    gx, gy = cv2.Sobel(gray, cv2.CV_32F, 1, 0), cv2.Sobel(gray, cv2.CV_32F, 0, 1)
    normal = np.column_stack([gx[y, x], gy[y, x]])
    normal /= np.maximum(np.linalg.norm(normal, axis=1)[:, None], 1e-8)
    points = np.column_stack([x, y]).astype(float)
    return {'image': image, 'points': points, 'normals': normal,
            'tree': cKDTree(points) if len(points) else None}


def foreground_edges(image, nominal_mask):
    """RGB GrabCut, seeded once from nominal geometry, never from fitted parameters."""
    # ponytail: fixed-camera local initialization; large pose errors need motion-derived seeds.
    seed = np.zeros(nominal_mask.shape, np.uint8)
    seed[cv2.dilate(nominal_mask, np.ones((71, 71), np.uint8)) > 0] = cv2.GC_PR_BGD
    seed[nominal_mask > 0] = cv2.GC_PR_FGD
    seed[cv2.erode(nominal_mask, np.ones((35, 35), np.uint8)) > 0] = cv2.GC_FGD
    if not np.any(seed == cv2.GC_FGD) or not np.any(seed == cv2.GC_BGD):
        raise ValueError('nominal foreground seeds insufficient')
    cv2.setRNGSeed(0)
    cv2.grabCut(image, seed, None, np.zeros((1, 65)), np.zeros((1, 65)), 5, cv2.GC_INIT_WITH_MASK)
    mask = np.isin(seed, [cv2.GC_FGD, cv2.GC_PR_FGD]).astype(np.uint8)
    return edges_from_mask(image, mask)


def edges_from_mask(image, mask):
    """Reuse a frozen observed RGB mask when scoring another candidate."""
    curves, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    points, normals = [], []
    h, w = mask.shape
    for curve in curves:
        xy = curve[:, 0].astype(float)
        if len(xy) < 20:
            continue
        tangent = np.roll(xy, -3, axis=0)-np.roll(xy, 3, axis=0)
        norm = np.linalg.norm(tangent, axis=1)
        good = (norm > 0) & (xy[:, 0] > 2) & (xy[:, 0] < w-3) & (xy[:, 1] > 2) & (xy[:, 1] < h-3)
        points.extend(xy[good])
        normals.extend(np.column_stack([-tangent[good, 1], tangent[good, 0]])/norm[good, None])
    points = np.asarray(points).reshape(-1, 2)
    return {'image': image, 'mask': mask, 'points': points, 'normals': np.asarray(normals).reshape(-1, 2),
            'tree': cKDTree(points) if len(points) else None}


def match(contour, edges, radius=20):
    n = len(contour['uv'])
    target, valid = np.zeros((n, 2)), np.zeros(n, bool)
    if not n or edges['tree'] is None:
        return target, valid
    distance, index = edges['tree'].query(contour['uv'], k=min(24, len(edges['points'])))
    if distance.ndim == 1:
        distance, index = distance[:, None], index[:, None]
    delta = edges['points'][index]-contour['uv'][:, None]
    normal = contour['normals']
    tangent = np.column_stack([-normal[:, 1], normal[:, 0]])
    tangential = np.abs(np.einsum('nki,ni->nk', delta, tangent))
    alignment = np.abs(np.einsum('nki,ni->nk', edges['normals'][index], normal))
    score = np.where((distance <= radius) & (alignment >= .65) & (tangential <= 3),
                     distance + tangential, np.inf)
    best = np.argmin(score, axis=1)
    valid = np.isfinite(score[np.arange(n), best])
    target[:] = edges['points'][index[np.arange(n), best]]
    return target, valid


def projected_points(model, manifest, row, meshes, x, corr):
    frames = frames_with_feedback(model, row, manifest, x[6:])
    camera = camera_candidate(manifest['camera_nominal_T_base_optical'], x)
    world = np.empty_like(corr['points'])
    for link_id in np.unique(corr['links']):
        selected = corr['links'] == link_id
        T = frames[meshes[link_id]['name']]
        world[selected] = corr['points'][selected] @ T[:3, :3].T + T[:3, 3]
    return project((world-camera[:3, 3]) @ camera[:3, :3], manifest['intrinsics'])


def objective(model, manifest, rows, meshes, correspondences, x, samples):
    result = []
    for row, corr in zip(rows, correspondences):
        uv = projected_points(model, manifest, row, meshes, x, corr)
        r = np.full(samples, 20.)
        error = np.einsum('ij,ij->i', uv-corr['target'], corr['normals'])
        r[:len(error)] = np.where(corr['valid'], error, 20.)
        result.append(r)
    return np.concatenate(result)


def evaluate(model, manifest, rows, meshes, edges, x, pattern, samples, output=None, prefix='fit'):
    reports, corrs = [], []
    for i, (row, edge) in enumerate(zip(rows, edges)):
        corr = contours(model, manifest, row, meshes, x, edge['image'].shape[:2], pattern, samples)
        corr['target'], corr['valid'] = match(corr, edge)
        penalty = np.full(samples, 20.)
        distance = np.linalg.norm(corr['uv']-corr['target'], axis=1)
        penalty[:len(distance)] = np.where(corr['valid'], distance, 20.)
        reports.append({'id': row['id'], 'split': row['split'], 'contour_samples': len(distance),
                        'matched': int(corr['valid'].sum()), 'denominator': samples,
                        'coverage': float(corr['valid'].sum()/samples), 'mean_penalized_px': float(penalty.mean()),
                        'matched_mean_px': float(distance[corr['valid']].mean()) if corr['valid'].any() else None})
        if output is not None:
            image = edge['image'].copy()
            for p, q, ok in zip(corr['uv'], corr['target'], corr['valid']):
                a, b = tuple(np.rint(p).astype(int)), tuple(np.rint(q).astype(int))
                cv2.circle(image, a, 1, (0, 220, 0) if ok else (0, 0, 255), -1)
                if ok:
                    cv2.line(image, a, b, (255, 255, 0), 1)
            cv2.putText(image, f'{prefix} {row["id"]} {reports[-1]["mean_penalized_px"]:.2f}px cov {reports[-1]["coverage"]:.0%}',
                        (8, 20), cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 255, 255), 1)
            cv2.imwrite(str(output / f'{prefix}-{i:03d}.png'), image)
        corrs.append(corr)
    metrics = {}
    for split in ('fit', 'holdout'):
        subset = [r for r in reports if r['split'] == split]
        metrics[split] = {'frames': len(subset), 'macro_mean_penalized_px': float(np.mean([r['mean_penalized_px'] for r in subset])) if subset else None,
                          'macro_coverage': float(np.mean([r['coverage'] for r in subset])) if subset else 0.,
                          'frames_without_matches': [r['id'] for r in subset if r['matched'] == 0]}
    return metrics, reports, corrs


def fit_dataset(dataset, mesh_path, output, *, iterations=6, samples=400, pattern='left|right'):
    start = time.monotonic()
    path = Path(dataset).resolve(strict=True)
    if path.is_dir():
        path = path/'dataset.json'
    output = Path(output).resolve()
    if output.exists():
        raise ValueError('output directory must be new')
    manifest = json.loads(path.read_text())
    if len(manifest['joint_names']) != 12 or not 1 <= iterations <= 20 or not 40 <= samples <= 2000:
        raise ValueError('expected 12 arm joints and bounded fit budget')
    if manifest.get('markers'):
        raise ValueError('markerless fit requires capture with markers=[]')
    completion = path.parent/'completion.json'
    if completion.exists() and json.loads(completion.read_text()).get('passed') is not True:
        raise ValueError('capture did not complete')
    rigid(manifest['camera_nominal_T_base_optical'])
    k = manifest['intrinsics']
    if not np.isfinite([k[n] for n in ('fx', 'fy', 'cx', 'cy')]).all() or min(k['fx'], k['fy']) <= 0:
        raise ValueError('invalid intrinsics')
    urdf = relative_file(path.parent, manifest['urdf_path'])
    model = Model(urdf, manifest['joint_names'])
    meshes = load_meshes(mesh_path, model.links)
    if not any(re.search(pattern, m['name']) for m in meshes):
        raise ValueError('no selected contour links')
    rows = manifest['frames']
    if not rows or len({r['id'] for r in rows}) != len(rows) or any(r['split'] not in ('fit', 'holdout') for r in rows):
        raise ValueError('invalid frame IDs or splits')
    images = [relative_file(path.parent, r['image']) for r in rows]
    if len(set(images)) != len(images):
        raise ValueError('reused image')
    sources = [path, urdf, Path(mesh_path).resolve(), *images]
    snapshots = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    edges = []
    for row, image_path in zip(rows, images):
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError('unreadable RGB image')
        parts = camera_mesh_parts(model, manifest, row, meshes, np.zeros(18))
        selected = [part for mesh, part in zip(meshes, parts) if re.search(pattern, mesh['name'])]
        mask = np.isfinite(raster_depth(selected, k, image.shape[:2])).astype(np.uint8)
        edges.append(foreground_edges(image, mask))
    training = [i for i, r in enumerate(rows) if r['split'] == 'fit']
    if not training:
        raise ValueError('no fit frames')
    output.mkdir(parents=True)
    for i, edge in enumerate(edges):
        cv2.imwrite(str(output/f'observed-mask-{i:03d}.png'), edge['mask']*255)
    x = np.zeros(18)
    nominal, nominal_rows, corrs = evaluate(model, manifest, rows, meshes, edges, x, pattern, samples, output, 'nominal')
    trace = []
    fit_rows = [rows[i] for i in training]
    fit_edges = [edges[i] for i in training]
    current = [corrs[i] for i in training]
    for iteration in range(iterations):
        active = np.arange(6 if iteration < 2 else 18)
        base = x.copy()
        def residual(z):
            trial = base.copy()
            trial[active] = z*SCALE[active]
            return objective(model, manifest, fit_rows, meshes, current, trial, samples)
        result = least_squares(residual, x[active]/SCALE[active], bounds=(-BOUNDS[active]/SCALE[active], BOUNDS[active]/SCALE[active]),
                               loss='soft_l1', f_scale=2., max_nfev=30, diff_step=1e-3, ftol=1e-5, xtol=1e-5)
        x[active] = result.x*SCALE[active]
        metrics, _, current = evaluate(model, manifest, fit_rows, meshes, fit_edges, x, pattern, samples)
        trace.append({'iteration': iteration, 'active_parameters': len(active), 'nfev': result.nfev,
                      'optimizer_success': bool(result.success), 'metrics': metrics, 'parameters': x.tolist()})
        print(json.dumps(trace[-1]), flush=True)
    metrics, frame_reports, corrs = evaluate(model, manifest, rows, meshes, edges, x, pattern, samples, output, 'fitted')
    current = [corrs[i] for i in training]
    def scaled_residual(z):
        return objective(model, manifest, fit_rows, meshes, current, z*SCALE, samples)
    z = x/SCALE
    jac = np.column_stack([(scaled_residual(z+np.eye(18)[j]*1e-3)-scaled_residual(z-np.eye(18)[j]*1e-3))/.002 for j in range(18)])
    _, singular, vectors = np.linalg.svd(jac, full_matrices=False)
    rank = int(np.sum(singular > max(singular[0]*1e-6, 1e-9)))
    parameter_names = ['camera_dx', 'camera_dy', 'camera_dz', 'camera_rx', 'camera_ry', 'camera_rz', *model.names]
    planned = manifest.get('planned_frames')
    missing = sorted({r['id'] for r in planned}-{r['id'] for r in rows}) if planned else None
    report = {'schema': 'joint-head-markerless-fit-v1', 'status': 'EXPERIMENTAL_CANDIDATE',
              'camera_T_base_optical': camera_candidate(manifest['camera_nominal_T_base_optical'], x).tolist(),
              'joint_offsets_rad': dict(zip(model.names, x[6:].tolist())), 'parameters': x.tolist(),
              'nominal_metrics': nominal, 'metrics': metrics, 'frames': frame_reports, 'nominal_frames': nominal_rows,
              'trace': trace, 'missing_planned_frames': missing,
              'at_parameter_bound': bool(np.any(np.abs(x) > BOUNDS*.999)),
              'observability': {'rank': rank, 'singular_values': singular.tolist(),
                                'condition_number': float(singular[0]/singular[-1]) if singular[-1] > 1e-12 else None,
                                'weakest_direction_scaled': dict(zip(parameter_names, vectors[-1].tolist())),
                                'parameter_scale': SCALE.tolist(), 'scope': 'local frozen-correspondence data Jacobian; rank is not accuracy or uniqueness proof'},
              'method': {'observations': 'RGB GrabCut 5 iterations, nominal arm mesh seeds fixed before fitting, exterior boundary',
                         'geometry': 'analytic mesh silhouette within 2px of full robot exterior; candidate depth occlusion',
                         'correspondences': 'nearest RGB edge; absolute normal cosine >=.65; tangent <=3px; radius20px',
                         'objective': 'normal-distance robust ICP, refreshed candidate contour every iteration',
                         'score': 'Euclidean match distance; missing/rejected samples penalized20px; fixed per-frame denominator; frame-macro averages',
                         'selected_link_pattern': pattern, 'samples_per_frame': samples},
              'used_truth_in_fit': False, 'used_segmentation_in_fit': True, 'used_segmentation_truth_in_fit': False,
              'physical_validated': False, 'applied': False,
              'elapsed_seconds': time.monotonic()-start, 'input_sha256': snapshots,
              'limitations': ['local ICP can lock onto background/texture edges; no learned segmentation',
                             'scene objects absent from robot mesh can occlude robot without model explanation',
                             'same nominal mesh accuracy assumed; near-plane-crossing triangles skipped',
                             'fixed head pose and known intrinsics; no physical control or safety validation',
                             'no global convergence or confidence interval claim; fresh pose holdout only',
                             'unknown unreported non-arm joints default0; measured mimic joints recomputed from driver']}
    if any(hashlib.sha256(Path(p).read_bytes()).hexdigest() != digest for p, digest in snapshots.items()):
        raise ValueError('input changed during fit')
    report['input_snapshot_verified'] = True
    with (output/'fit.json').open('x') as f:
        json.dump(report, f, indent=2, allow_nan=False)
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('dataset', type=Path)
    parser.add_argument('--meshes', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--iterations', type=int, default=6)
    parser.add_argument('--samples', type=int, default=400)
    parser.add_argument('--links', default='left|right')
    args = parser.parse_args()
    result = fit_dataset(args.dataset, args.meshes, args.output, iterations=args.iterations, samples=args.samples, pattern=args.links)
    print(json.dumps({'status': result['status'], 'metrics': result['metrics'], 'observability': result['observability']}))
