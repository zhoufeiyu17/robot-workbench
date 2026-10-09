"""X5 full-mesh adapter and free-sticker head fit; no hardware or truth access."""
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares

from . import joint_head_markerless as mesh
from .joint_head_hybrid import (marker_predictions, marker_residual, marker_mount,
                                weighted_soft_l1, jacobian_report)
from .joint_head_isaac_fit import camera_candidate, rigid, statistics
from .joint_head_jaw_mapping import measured_jaw_gaps
from .joint_head_workbench_capture import _rectification_map

SIDES = ('left', 'right')
IDS = (24, 30)
SOLVER = 'head_mesh_marker_joint_v1'


def marker_statistics(errors):
    return statistics(errors) if len(errors) else dict(corners=0, mean_px=None, rms_px=None, p95_px=None)


def indexed_mesh(name, vertices, faces):
    vertices, inverse = np.unique(vertices, axis=0, return_inverse=True)
    faces = inverse[faces]
    edges = np.sort(np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]]), axis=1)
    owners = np.tile(np.arange(len(faces)), 3)
    edges, ids, count = np.unique(edges, axis=0, return_inverse=True, return_counts=True)
    first, last = np.full(len(edges), -1), np.full(len(edges), -1)
    first[ids], last[ids[::-1]] = owners, owners[::-1]
    return dict(name=name, v=vertices, f=faces, edges=edges, first=first, last=last, count=count)


class X5MeshModel:
    """Expose full triangles, independently posed fingers, and world FK to mesh solver.

    Estimated camera/support proxies are deliberately excluded: their geometry
    was inferred from sticker fits and is not an independent image constraint.
    """
    def __init__(self, geometry):
        self.geometry = geometry
        self.names = [f'{side}_J{i}' for side in SIDES for i in range(1, 7)]
        self.meshes = []
        for side in SIDES:
            parts = geometry.full_side_meshes[side] if geometry.hand is None else geometry.meshes
            for name, vertices, faces in parts:
                if name != 'link6' or geometry.hand is None:
                    self.meshes.append(indexed_mesh(f'{side}/{name}', vertices, faces))
                    continue
                signs = geometry.hand['signs']
                if not np.all(signs[faces] == signs[faces[:, :1]]):
                    raise ValueError('finger triangles cross rigid components')
                for sign in (-1, 0, 1):
                    selected = faces[signs[faces[:, 0]] == sign]
                    if len(selected):
                        self.meshes.append(indexed_mesh(f'{side}/link6/{sign}', vertices, selected))
        for name, vertices, faces in geometry.static_meshes:
            self.meshes.append(indexed_mesh(f'static/{name}', vertices, faces))
        self.links = {m['name'] for m in self.meshes} | {f'{s}/link6' for s in SIDES}

    def frames_with_feedback(self, row, manifest, offsets):
        q, offsets = np.asarray(row['q_reported_rad'], float), np.asarray(offsets, float)
        if q.shape != (12,) or offsets.shape != (12,) or not np.isfinite(np.r_[q, offsets]).all():
            raise ValueError('finite twelve-joint feedback and offsets required')
        gaps = measured_jaw_gaps(row['jaw_gap_mm']) / 1000
        if self.geometry.hand is None:
            posed=self.geometry.with_feedback_gaps(*(gaps*1000)).frames(q+offsets)
            return {**{f'{side}/{name}':T for side in SIDES for name,T in posed[side].items()},
                    **{f'static/{name}':np.eye(4) for name,_,_ in self.geometry.static_meshes}}
        frames = {}
        for i, side in enumerate(SIDES):
            fk = self.geometry.model.frames(q[i*6:i*6+6] + offsets[i*6:i*6+6])
            frames.update({f'{side}/{name}': self.geometry.bases[side] @ T for name, T in fk.items()})
            for sign in (-1, 0, 1):
                delta = np.eye(4)
                delta[1, 3] = sign * (gaps[i] - self.geometry.hand['nominal_gap_m']) / 2
                frames[f'{side}/link6/{sign}'] = frames[f'{side}/link6'] @ delta
        frames.update({f'static/{name}': np.eye(4) for name, _, _ in self.geometry.static_meshes})
        return frames


def score_head_candidate(geometry, rows, image_root, camera, candidate,
                         observation_directory, *, samples=200, prefix='legacy'):
    """Evaluate a frozen candidate against the hybrid's exact frozen RGB masks.

    No fitting or pixel selection. This permits the historical 26-DOF corner
    result to be scored alongside the 30-DOF corner and hybrid comparison.
    """
    root, output = Path(image_root), Path(observation_directory)
    model = X5MeshModel(geometry)
    offsets = np.concatenate([candidate['joint_offsets_rad'][s] for s in SIDES])
    manifest = dict(intrinsics=camera, camera_nominal_T_base_optical=rigid(candidate['camera_T_world_optical']),
                    markers=[dict(id=marker,link=f'{side}/link6',size_m=geometry.marker_size,
                                  T_link_marker=candidate['T_link6_marker'][str(marker)])
                             for side,marker in zip(SIDES,IDS)])
    prepared, edges = [], []
    for i, source in enumerate(rows):
        row = dict(source)
        row['q_reported_rad'] = np.asarray(row['q'],float).tolist()
        row['marker_ids'] = [m for m in IDS if m in row['uv']]
        row['uv'] = np.concatenate([row['uv'][m] for m in row['marker_ids']]) if row['marker_ids'] else np.empty((0,2))
        image = cv2.imread(str(root/row['image']))
        mask = cv2.imread(str(output/f'observed-mask-{i:03d}.png'),cv2.IMREAD_GRAYSCALE)
        if image is None or mask is None:
            raise ValueError('original image and saved observed mask required')
        if np.any(camera.get('coefficients',[0.]*5)):
            image=cv2.remap(image,_rectification_map(json.dumps(camera,sort_keys=True)),None,cv2.INTER_LINEAR)
        edges.append(mesh.edges_from_mask(image,(mask>0).astype(np.uint8)))
        prepared.append(row)
    parameters=np.r_[np.zeros(6),offsets,np.zeros(12)]
    metrics,frames,_=mesh.evaluate(model,manifest,prepared,model.meshes,edges,parameters[:18],
                                  r'^(left|right)/',samples,output,prefix)
    markers={split:statistics(np.linalg.norm(marker_residual(model,manifest,[r for r in prepared if r['split']==split],parameters).reshape(-1,2),axis=1))
             for split in ('fit','holdout')}
    return dict(mesh_metrics=metrics,marker_metrics=markers,mesh_frames=frames,optimization_performed=False,
                mask_source=str(output),candidate_observability=candidate.get('observability'))


def independent_pose_coverage(rows):
    """Count new views of each marker's own arm; incidental other-arm motion is not a new view."""
    counts = {}
    tolerance = 1e-4  # Equality tolerance only, not a sufficient excitation threshold.
    for i, marker in enumerate(IDS):
        fit, heldout = [], []
        for split, collection in [('fit',fit),('holdout',heldout)]:
            for row in rows:
                if row['split'] != split or marker not in row['marker_ids']:
                    continue
                q=np.asarray(row['q_reported_rad'])[i*6:i*6+6]
                if not any(np.max(np.abs(q-other)) <= tolerance for other in collection):
                    collection.append(q)
        novel=[q for q in heldout if not any(np.max(np.abs(q-other)) <= tolerance for other in fit)]
        counts[str(marker)]={'distinct_fit':len(fit),'distinct_holdout':len(heldout),'holdout_not_in_fit':len(novel)}
    return {'markers':counts,'passed':all(v['distinct_fit']>=4 and v['holdout_not_in_fit']>=2 for v in counts.values()),
            'equality_tolerance_rad':tolerance,'scope':'detect repeated poses only; rank/conditioning still required for excitation'}


def expand_active(z, indices, scale):
    """Optimizer coordinates back on the full parameter vector; frozen entries stay zero."""
    values = np.zeros(30)
    values[indices] = np.asarray(z)*scale[indices]
    return values


def head_mesh_thresholds(source=None, recorded=None):
    """Only explicitly physical capture provenance enables the hardware gate."""
    source = source or {}
    physical = (source.get('environment', 'hardware') == 'hardware' and source.get('robot_source') == 'physical'
                and source.get('simulation_only') is False)
    thresholds = {'holdout_mesh_max_px': 10. if physical else 7.,
                  'holdout_mesh_min_coverage': .6 if physical else .7}
    for key, choose in [('holdout_mesh_max_px', min), ('holdout_mesh_min_coverage', max)]:
        if recorded is not None and key in recorded:
            value = recorded[key]
            if type(value) not in (int, float) or not np.isfinite(value) or value < 0 or (key.endswith('coverage') and value > 1):
                raise ValueError('invalid recorded head mesh threshold: '+key)
            thresholds[key] = choose(thresholds[key], value)
    return thresholds


def fit_head(geometry, rows, image_root, camera, nominal_camera, mounts, output,
             *, iterations=8, samples=200, max_nfev=80, capture_coverage=None,
             frozen_mounts=None, frozen_mount_provenance=None, source_metadata=None, progress=None):
    """Compare corner-only and mesh+corner fits on identical splits.

    `frozen_mounts` ({marker id -> 4x4}) holds one or both sticker installations fixed
    at a value measured elsewhere - normally the wrist half, which sees the same
    sticker from six degrees of freedom instead of the twenty-six the head half
    solves at once. A frozen mount is still a parameter of the manifest (the model
    needs it to predict anything), it is simply not optimized here, and the
    observability report says so rather than reporting the rank of a fit that moved it.
    """
    if not 1 <= iterations <= 20 or not 40 <= samples <= 2000 or not 1 <= max_nfev <= 1000:
        raise ValueError('invalid bounded solver budget')
    def emit(fraction, label, completed=0, total=1):
        if progress is not None:
            progress(dict(fraction=fraction, label=label, completed=completed, total=total))
    emit(0., "head_initialization")
    root, output = Path(image_root), Path(output)
    if output.exists():
        raise ValueError('output directory must be new')
    if (not np.isfinite([camera[key] for key in ('fx','fy','cx','cy')]).all()
            or min(camera['fx'], camera['fy']) <= 0):
        raise ValueError('positive finite pinhole intrinsics required')
    model = X5MeshModel(geometry)
    frozen = {}
    for index, (side, marker) in enumerate(zip(SIDES, IDS)):
        if frozen_mounts is None or str(marker) not in frozen_mounts:
            continue
        value = rigid(frozen_mounts[str(marker)])
        if not np.isfinite(value).all():
            raise ValueError('frozen sticker mount must be a finite rigid transform')
        frozen[index] = value
    manifest = {'intrinsics': camera, 'camera_nominal_T_base_optical': rigid(nominal_camera).tolist(),
                'markers': [dict(id=marker, link=f'{side}/link6', size_m=geometry.marker_size,
                                 T_link_marker=rigid(frozen.get(i, rigid(mounts[i]))).tolist(),
                                 mount_frozen=i in frozen)
                            for i, (side, marker) in enumerate(zip(SIDES, IDS))]}
    prepared = []
    paths = set()
    for source in rows:
        row = dict(source)
        row['q_reported_rad'] = np.asarray(row['q'], float).tolist()
        row['marker_ids'] = [key for key in IDS if key in row['uv']]
        row['uv'] = np.concatenate([row['uv'][key] for key in row['marker_ids']]) if row['marker_ids'] else np.empty((0, 2))
        path = (root / row['image']).resolve(strict=True)
        if path in paths or not path.is_relative_to(root.resolve()):
            raise ValueError('reused or out-of-root image')
        paths.add(path)
        model.frames_with_feedback(row, manifest, np.zeros(12))
        prepared.append(row)
    rows = prepared
    if (not rows or len({r['id'] for r in rows}) != len(rows)
            or any(r['split'] not in ('fit', 'holdout') for r in rows)):
        raise ValueError('unique fit/holdout observations required')
    if not all(any(r['split'] == split for r in rows) for split in ('fit', 'holdout')):
        raise ValueError('mesh fitting requires fit images and independent holdout images')
    marker_counts = {str(marker): {split: sum(marker in r['marker_ids'] for r in rows if r['split'] == split)
                                  for split in ('fit', 'holdout')} for marker in IDS}
    observed_mounts = [i for i, marker in enumerate(IDS)
                       if marker_counts[str(marker)]['fit'] and i not in frozen]
    active_parameters = np.array(list(range(18)) + [j for i in observed_mounts for j in range(18+6*i,24+6*i)])
    hashes = {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in paths}
    if len(set(hashes.values())) != len(hashes):
        raise ValueError('duplicate image bytes cannot count as independent observations')
    pose_coverage=independent_pose_coverage(rows)
    scale = np.r_[[.01]*3, [.02]*15, np.tile([.01]*3 + [.05]*3, 2)]
    bounds = np.r_[[.15]*3, [.4]*3, [.15]*12, np.tile([.15]*3 + [1.5]*3, 2)]
    names = [f'camera_{s}' for s in ('tx', 'ty', 'tz', 'rx', 'ry', 'rz')] + model.names
    names += [f'{side}_mount_{s}' for side in SIDES for s in ('tx', 'ty', 'tz', 'rx', 'ry', 'rz')]
    fit_rows = [r for r in rows if r['split'] == 'fit']
    marker_sigma, mesh_sigma, effective_count = .5, 2., 32.
    def marker_objective(x):
        return marker_residual(model, manifest, fit_rows, x) / marker_sigma
    corner = np.zeros(30)
    has_fit_markers = any(counts['fit'] for counts in marker_counts.values())
    if has_fit_markers:
        # The corner baseline starts from the same nominal geometry, so a frozen mount
        # must stay frozen here too or the comparison would silently move it.
        corner_result = least_squares(lambda z: marker_objective(expand_active(z, active_parameters, scale)),
                                      np.zeros(len(active_parameters)),
                                      bounds=(-bounds[active_parameters]/scale[active_parameters],
                                              bounds[active_parameters]/scale[active_parameters]),
                                      loss='soft_l1', max_nfev=max_nfev)
        corner = expand_active(corner_result.x, active_parameters, scale)
    emit(.05, "head_rgb_masks", 0, len(rows))
    edges = []
    output.mkdir(parents=True)
    for i, row in enumerate(rows):
        image = cv2.imread(str(root / row['image']))
        if image is None or image.shape[:2] != (camera['height'], camera['width']):
            raise ValueError('unreadable image or camera resolution mismatch')
        if np.any(camera.get('coefficients', [0.]*5)):
            image = cv2.remap(image, _rectification_map(json.dumps(camera, sort_keys=True)), None, cv2.INTER_LINEAR)
        # Freeze observations once from the fit-only sticker solution. Holdout
        # corners never affect this seed; hybrid iterations never resegment RGB.
        parts = mesh.camera_mesh_parts(model, manifest, row, model.meshes, corner[:18])
        mask = np.isfinite(mesh.raster_depth([part for part, m in zip(parts, model.meshes)
                                              if m['name'].startswith(('left/', 'right/'))], camera, image.shape[:2]))
        edge = mesh.foreground_edges(image, mask.astype(np.uint8))
        edges.append(edge)
        if not cv2.imwrite(str(output/f'observed-mask-{i:03d}.png'), edge['mask']*255):
            raise IOError('cannot save mask evidence')
        print(json.dumps(dict(phase='freeze_rgb_mask', id=row['id'], points=len(edge['points']))), flush=True)
        emit(.05 + .2*(i+1)/len(rows), 'head_rgb_masks', i+1, len(rows))
    fit_edges = [e for e, r in zip(edges, rows) if r['split'] == 'fit']
    pattern = r'^(left|right)/'
    weights = np.r_[np.full(len(fit_rows)*samples, effective_count/samples),
                    np.ones(sum(r['uv'].size for r in fit_rows))]
    def joint_objective(x, correspondences):
        return np.r_[mesh.objective(model, manifest, fit_rows, model.meshes, correspondences, x[:18], samples)/mesh_sigma,
                     marker_objective(x)]
    # Preserve the sticker solution through initialization instead of discarding
    # it in a mesh-only warmup. Nominal warmup is only for markerless captures.
    x, trace = corner.copy(), []
    warmup_iterations = 0 if has_fit_markers else 4
    for iteration in range(iterations+warmup_iterations):
        warm = iteration < warmup_iterations
        active = np.arange(6 if iteration < 2 else 18) if warm else active_parameters
        _, _, corrs = mesh.evaluate(model, manifest, fit_rows, model.meshes, fit_edges, x[:18], pattern, samples)
        base = x.copy()
        def expand(z):
            trial = base.copy(); trial[active] = z*scale[active]
            return trial
        def objective(z):
            residual = joint_objective(expand(z), corrs)
            if warm:
                residual[len(fit_rows)*samples:] = 0.
            return residual
        result = least_squares(objective, x[active]/scale[active],
                               bounds=(-bounds[active]/scale[active], bounds[active]/scale[active]),
                               loss=lambda z: weighted_soft_l1(z, weights), max_nfev=max_nfev,
                               diff_step=1e-3, ftol=1e-6, xtol=1e-6)
        x = expand(result.x)
        trace.append(dict(iteration=iteration, phase='mesh_warmup' if warm else 'mesh_and_marker',
                          success=bool(result.success), nfev=result.nfev, cost=float(result.cost)))
        print(json.dumps(trace[-1]), flush=True)
        emit(.25 + .6*(iteration+1)/(iterations+warmup_iterations), "head_optimizer",
             iteration+1, iterations+warmup_iterations)
    comparison = {}
    for label, parameters in [('corner_only', corner), ('hybrid', x)]:
        metrics, frames, corrs = mesh.evaluate(model, manifest, rows, model.meshes, edges,
                                             parameters[:18], pattern, samples, output, label)
        marker_metrics = {}
        for split in ('fit', 'holdout'):
            selected = [r for r in rows if r['split'] == split]
            errors = np.linalg.norm(marker_residual(model, manifest, selected, parameters).reshape(-1, 2), axis=1)
            marker_metrics[split] = marker_statistics(errors)
        frozen_corrs = [c for c, r in zip(corrs, rows) if r['split'] == 'fit']
        residual = marker_objective if label == 'corner_only' else lambda p: joint_objective(p, frozen_corrs)*np.sqrt(weights)
        observability = (jacobian_report(residual, parameters, scale, names, mask=active_parameters)
                         if label != 'corner_only' or has_fit_markers
                         else dict(rank=0, parameters=30, optimized_parameters=0, condition_number=None,
                                   scope='no fit sticker observations'))
        comparison[label] = dict(mesh_metrics=metrics, marker_metrics=marker_metrics, mesh_frames=frames,
                                 available=label != 'corner_only' or has_fit_markers, observability=observability,
                                 camera_T_world_optical=camera_candidate(nominal_camera, parameters[:6]).tolist(),
                                 joint_offsets_rad={s:parameters[6+i*6:12+i*6].tolist() for i,s in enumerate(SIDES)},
                                 T_link6_marker={str(m['id']):marker_mount(m,parameters,'free',marker_index=i).tolist()
                                                for i,m in enumerate(manifest['markers'])})
        for i, row in enumerate(rows):
            image = cv2.imread(str(output/f'{label}-{i:03d}.png'))
            predicted, _ = marker_predictions(model, manifest, row, parameters)
            for observed, projected in zip(row['uv'], predicted):
                if np.max(np.abs(np.r_[observed, projected])) < 1e6:
                    cv2.circle(image, tuple(np.rint(observed).astype(int)), 3, (255, 0, 255), 1)
                    cv2.drawMarker(image, tuple(np.rint(projected).astype(int)), (0, 255, 255), cv2.MARKER_CROSS, 7, 1)
            cv2.imwrite(str(output/f'{label}-{i:03d}.png'), image)
            if label == 'hybrid':
                cv2.imwrite(str(output.parent/(output.name+'-'+Path(row['image']).stem+'-overlay.png')), image)
        emit(.85 + .1*(1 if label == 'corner_only' else 2)/2, 'head_validation',
             1 if label == 'corner_only' else 2, 2)
    selected = comparison['hybrid']
    # A frozen mount is a promise to the caller that it was not fitted here; check the
    # parameter vector against that promise instead of trusting the active set.
    for index in sorted(frozen):
        reported = np.asarray(selected['T_link6_marker'][str(IDS[index])])
        if not np.allclose(reported, frozen[index], rtol=0, atol=1e-12):
            raise ValueError('frozen sticker mount moved during the fit: '+str(IDS[index]))
        if any(j in active_parameters for j in range(18+6*index, 24+6*index)):
            raise ValueError('frozen sticker mount left in the optimized set: '+str(IDS[index]))
    positive = all(np.all(marker_predictions(model, manifest, r, x)[1] > .01) for r in rows)
    at_bound = bool(np.any(np.abs(x) > .999*bounds))
    mesh_thresholds = head_mesh_thresholds(source_metadata)
    image_checks = dict(optimizer_converged=bool(result.success), positive_depth=positive,
                        not_at_bound=not at_bound,
                        full_rank=selected['observability']['rank'] == len(active_parameters),
                        independent_poses=pose_coverage['passed'],
                        all_captured_valid=bool(capture_coverage and capture_coverage.get('all_captured_valid')),
                        capture_counts_match=bool(capture_coverage and capture_coverage.get('completion_counts_match')),
                        no_missing_planned_frames=bool(capture_coverage and capture_coverage.get('planned_frames') is not None
                                                      and not capture_coverage.get('missing_planned_ids')
                                                      and not capture_coverage.get('unexpected_planned_ids')),
                        holdout_mesh_coverage=selected['mesh_metrics']['holdout']['macro_coverage'] >= mesh_thresholds['holdout_mesh_min_coverage'],
                        holdout_mesh_px=selected['mesh_metrics']['holdout']['macro_mean_penalized_px'] <= mesh_thresholds['holdout_mesh_max_px'],
                        holdout_marker_px=(selected['marker_metrics']['holdout']['rms_px'] is not None
                                           and selected['marker_metrics']['holdout']['rms_px'] <= 2.))
    for path, digest in hashes.items():
        if hashlib.sha256(Path(path).read_bytes()).hexdigest() != digest:
            raise ValueError('input image changed during fit')
    emit(1., 'head_solver_complete', 1, 1)
    return dict(schema='joint-head-x5-hybrid-v1', solver=SOLVER, objective='sticker_corners_plus_mesh', status='HYBRID_IMAGE_CANDIDATE',
                selected_candidate='hybrid', **{k:selected[k] for k in ('camera_T_world_optical','joint_offsets_rad','T_link6_marker','observability')},
                comparison=comparison, metrics_rectified_px=selected['marker_metrics'], mesh_metrics=selected['mesh_metrics'],
                optimizer=dict(success=bool(result.success), at_bound=at_bound, positive_depth=positive),
                acceptance=dict(image_checks=image_checks, image_passed=all(image_checks.values()),
                                physical_passed=False, physical_status='independent_truth_or_metrology_required',
                                thresholds=dict(camera_position_mm=2.,camera_rotation_deg=.3,max_joint_offset_deg=.5,**mesh_thresholds)),
                physical_accuracy_validated=False, applied=False, used_truth_in_fit=False,
                mount_mode=('frozen_%d_of_2' % len(frozen)) if frozen else 'unknown_6dof_no_prior',
                frozen_mounts={str(IDS[i]): frozen[i].tolist() for i in sorted(frozen)},
                frozen_mount_provenance=frozen_mount_provenance,
                optimized_parameters=active_parameters.tolist(),
                parameter_names=names, parameters=x.tolist(), trace=trace,
                initialization=dict(source='fit_sticker_solution' if has_fit_markers else 'nominal',
                                    parameters=corner.tolist(), masks_frozen=True,
                                    holdout_used=False, mesh_warmup_iterations=warmup_iterations),
                sticker_evidence={'counts':marker_counts,
                    'unobserved_fit_mounts':[str(marker) for i,marker in enumerate(IDS) if i not in observed_mounts],
                    'frozen_mounts':[str(IDS[i]) for i in sorted(frozen)],
                    'unobserved_mount_policy':'retain initialization only; not measured, not an exact-placement constraint'},
                fit_ids=[r['id'] for r in fit_rows], holdout_ids=[r['id'] for r in rows if r['split']=='holdout'],
                heldout_overlays=[str(output/f'hybrid-{i:03d}.png') for i,r in enumerate(rows) if r['split']=='holdout'],
                source=dict(camera=camera, images=hashes),
                coverage=dict(capture=capture_coverage,independent_poses=pose_coverage),
                assumptions=['Fixed nominal link geometry, arm bases, intrinsics and marker size.',
                             'Sticker mounts independently estimated in six DOF; no exact placement assumption.'
                             if not frozen else
                             'Sticker mounts %s held at a value measured by the wrist half (%s); the other is free in six DOF.'
                             % (sorted(str(IDS[i]) for i in frozen), frozen_mount_provenance or 'unspecified source'),
                             'Fixed RGB foreground masks; estimated camera/support proxies excluded.',
                             'Full rank and pixel thresholds do not certify physical accuracy.'])
