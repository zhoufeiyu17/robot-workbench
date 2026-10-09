"""CPU-only X5 two-sticker fit; never connects to hardware or applies parameters."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import sys

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'infra'))
from arx_env.joint_head_isaac_fit import Model, camera_candidate, rigid, statistics

OLD = Path('/Users/wujie/Documents/ChatGPT/first_work/arx-rule-harness-20260916')
SIDES = ('left', 'right')
IDS = (24, 30)
SCALE = np.r_[[.01]*3, [.02]*3, [.02]*8, np.tile([.01]*3 + [.05]*3, 2)]
BOUNDS = np.r_[[.15]*3, [.4]*3, [.15]*8, np.tile([.15]*3 + [1.5]*3, 2)]


def rectify(uv, k):
    """librealsense inverse-Brown deprojection, then virtual pinhole pixels."""
    if k['distortion_model'].split('.')[-1] != 'inverse_brown_conrady':
        raise ValueError('only verified inverse_brown_conrady input supported')
    original = (np.asarray(uv, float) - [k['cx'], k['cy']]) / [k['fx'], k['fy']]
    x, y = original[:, 0].copy(), original[:, 1].copy()
    k1, k2, p1, p2, k3 = k['coefficients']
    for _ in range(10):
        r2 = x*x + y*y
        ir = 1 / (1 + ((k3*r2 + k2)*r2 + k1)*r2)
        xq, yq = x/ir, y/ir
        dx = 2*p1*xq*yq + p2*(r2 + 2*xq*xq)
        dy = 2*p2*xq*yq + p1*(r2 + 2*yq*yq)
        x, y = (original[:, 0]-dx)*ir, (original[:, 1]-dy)*ir
    return np.c_[x, y] * [k['fx'], k['fy']] + [k['cx'], k['cy']]


def load_rows(path, k, *, return_coverage=False):
    path = Path(path).resolve()
    manifest_path = path/'dataset.json'
    manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() else None
    if manifest is not None:
        frames = manifest.get('frames')
        if not isinstance(frames,list) or not frames:
            raise ValueError('dataset manifest must enumerate captured frames')
        ids = [frame.get('id') for frame in frames]
        if (any(not isinstance(i,str) or not re.fullmatch(r'[A-Za-z0-9_-]+',i) or 'truth' in i.lower() for i in ids)
                or len(set(ids)) != len(ids)):
            raise ValueError('invalid/duplicate manifest frame IDs')
        sources = [path/(identity+'.json') for identity in ids]
    else:
        # Historical sidecars only; never parse arbitrary report/config/truth JSON.
        sources = sorted(p for p in path.iterdir() if re.fullmatch(r'(?:frame|sample|test)-\d{3,6}\.json',p.name))
    rows, rejected, captured_ids, recovered = [], [], [], []
    for source in sources:
        if source.is_symlink() or not source.is_file():
            raise ValueError(f'missing or symlink frame sidecar: {source}')
        row = json.loads(source.read_text())
        if not isinstance(row, dict) or not {'qpos', 'split', 'image', 'id'} <= row.keys():
            raise ValueError(f'invalid capture sidecar: {source}')
        if manifest is not None and row['id'] != source.stem:
            raise ValueError('manifest/sidecar identity mismatch')
        captured_ids.append(row.get('planned_id',row['id']))
        legacy_marker_only = (manifest is not None and manifest.get('schema') == 'x5-rtx-kinematic-v1'
            and row.get('source') == 'Isaac RTX X5 persistent kinematic rendered pixels'
            and row.get('feedback_kind') == 'commanded kinematic state; not motor feedback'
            and row.get('expected_marker_id') in IDS
            and str(row['expected_marker_id']) not in row.get('detections', {})
            and 'marker_detected' not in row and 'mesh_valid' not in row
            and not row.get('reason') and not row.get('error'))
        if row.get('mesh_valid') is False or (row.get('valid_for_fit') is False and row.get('mesh_valid') is not True and not legacy_marker_only):
            rejected.append({'id':row['id'],'split':row['split'],'reason':row.get('reason',row.get('error','capture marked invalid_for_fit'))})
            continue
        q = np.asarray(row['qpos'], float)
        if q.shape != (14,) or not np.isfinite(q).all() or row['split'] not in ('fit', 'holdout', 'test'):
            raise ValueError(f'invalid feedback/split: {source}')
        bracket=float(row.get('bracket_gap_s',1));span=np.asarray(row.get('pose_span_rad',1),float)
        if not np.isfinite(bracket) or bracket < 0 or bracket > .1 or not np.isfinite(span).all() or np.any(span<0) or np.max(span) > .005:
            raise ValueError(f'nonstationary/stale sample: {source}')
        image = (path / row['image']).resolve()
        if not image.is_relative_to(path) or not image.is_file():
            raise ValueError(f'missing image {image}')
        if row.get('valid_for_fit') is False and legacy_marker_only:
            pixels = cv2.imread(str(image))
            if pixels is None or pixels.shape[:2] != (k['height'], k['width']):
                raise ValueError(f'unreadable legacy image or resolution mismatch: {image}')
            recovered.append(row['id'])
        row['q'] = np.r_[q[:6], q[7:13]]
        row['uv'] = {}
        for marker_id in IDS:
            uv = row.get('detections', {}).get(str(marker_id))
            if uv is not None:
                uv = np.asarray(uv, float)
                if uv.shape != (4, 2) or not np.isfinite(uv).all():
                    raise ValueError('invalid marker corners')
                row['uv'][marker_id] = rectify(uv, k)
        row['source_sha256'] = hashlib.sha256(source.read_bytes()).hexdigest()
        row['source_json'] = str(source)
        row['image_sha256'] = hashlib.sha256(image.read_bytes()).hexdigest()
        rows.append(row)
    if len({r['id'] for r in rows}) != len(rows):
        raise ValueError('duplicate sample ID')
    coverage={'captured_frames':len(sources),'used_frames':len(rows),'rejected_frames':rejected,
              'legacy_marker_only_recovered_ids':recovered,
              'all_captured_valid':not rejected,'manifest_enumerated':manifest is not None,
              'source_files':[str(p) for p in sources],
              'by_split':{split:{'used':sum(r['split']==split for r in rows),'rejected':sum(r['split']==split for r in rejected)}
                          for split in ('fit','holdout','test')}}
    planned=manifest.get('planned_frames') if manifest is not None else None
    expected={r.get('id',r.get('sample_id')) for r in planned if r.get('capture') is not False} if isinstance(planned,list) else None
    if expected is not None and (None in expected or any(not isinstance(i,str) or not i for i in expected)):
        raise ValueError('planned samples require id or sample_id')
    coverage['planned_frames']=len(expected) if expected is not None else None
    coverage['missing_planned_ids']=sorted(expected-set(captured_ids)) if expected is not None else None
    coverage['unexpected_planned_ids']=sorted(set(captured_ids)-expected) if expected is not None else None
    return (rows,coverage) if return_coverage else rows


class Fit:
    def __init__(self, urdf, scene, camera, k, size, geometry=None):
        self.geometry = geometry
        self.model = None if geometry is not None else Model(urdf, [f'joint{i}' for i in range(1, 7)])
        self.bases = [rigid(scene['T_world_arm_base'][s]) for s in SIDES]
        self.camera = rigid(camera['T_world_camera'])
        self.k = k
        s = size/2
        self.corners = np.array([[-s, -s, 0], [s, -s, 0], [s, s, 0], [-s, s, 0]])
        self.mounts = []

    def terminal(self, row, side, offsets):
        if self.geometry is not None:
            q = row['q'].copy(); q[side*6+1:side*6+5] += offsets
            return self.geometry.frames(q)[SIDES[side]]['link6']
        q = row['q'][side*6:side*6+6].copy()
        q[1:5] += offsets
        return self.bases[side] @ self.model.frames(q)['link6']

    def initialize_mounts(self, rows, fallback_mounts=None):
        k = self.k
        K = np.array([[k['fx'], 0, k['cx']], [0, k['fy'], k['cy']], [0, 0, 1.]])
        for side, marker in enumerate(IDS):
            # Largest observed marker minimizes single-view planar PnP noise.
            choices = [r for r in rows if r['split'] == 'fit' and marker in r['uv']]
            if not choices:
                if not fallback_mounts or str(marker) not in fallback_mounts:
                    raise ValueError(f'no fit detections or seed mount for {marker}')
                self.mounts.append(rigid(fallback_mounts[str(marker)]))
                continue
            row = max(choices, key=lambda r: cv2.contourArea(r['uv'][marker].astype(np.float32)))
            ok, rv, tv = cv2.solvePnP(self.corners, row['uv'][marker], K, None, flags=cv2.SOLVEPNP_ITERATIVE)
            if not ok or tv[2, 0] <= 0:
                raise ValueError('PnP mount initialization failed')
            T = np.eye(4)
            T[:3, :3], T[:3, 3] = Rotation.from_rotvec(rv.ravel()).as_matrix(), tv.ravel()
            self.mounts.append(np.linalg.inv(self.terminal(row, side, np.zeros(4))) @ self.camera @ T)

    def predict(self, row, x):
        camera = camera_candidate(self.camera, x[:6])
        found, depths = {}, []
        for side, marker in enumerate(IDS):
            if marker not in row['uv']:
                continue
            mount = camera_candidate(self.mounts[side], x[14+side*6:20+side*6])
            T = self.terminal(row, side, x[6+side*4:10+side*4]) @ mount
            world = self.corners @ T[:3, :3].T + T[:3, 3]
            pc = (world - camera[:3, 3]) @ camera[:3, :3]
            depths.extend(pc[:, 2])
            found[marker] = pc[:, :2] / np.maximum(pc[:, 2:3], 1e-4) * [self.k['fx'], self.k['fy']] + [self.k['cx'], self.k['cy']]
        return found, np.asarray(depths)

    def residual(self, rows, x):
        values = []
        for row in rows:
            found, _ = self.predict(row, x)
            values.extend((found[key] - uv).ravel() for key, uv in row['uv'].items())
        return np.concatenate(values)


def run_corner_only(args):
    root = args.dataset.resolve()
    k = json.loads((root/'camera.json').read_text())
    rows = load_rows(root, k)
    if any(r['split'] == 'test' for r in rows):
        raise ValueError('reserved test observations require --evaluate-report; never refit them')
    train = [r for r in rows if r['split'] == 'fit' and r['uv']]
    holdout = [r for r in rows if r['split'] == 'holdout' and r['uv']]
    if len(train) < 8 or not holdout:
        raise ValueError('need at least 8 fit poses and independent holdout poses')
    fit = Fit(args.urdf, json.loads(args.scene.read_text()), json.loads(args.camera_seed.read_text()), k, args.size)
    fit.initialize_mounts(train)
    fixed_indices = np.r_[0:6, 14:26]
    def optimize(start, indices):
        def expand(v):
            x = start.copy(); x[indices] = v*SCALE[indices]
            return x
        result = least_squares(lambda v: fit.residual(train, expand(v)), start[indices]/SCALE[indices],
                               bounds=(-BOUNDS[indices]/SCALE[indices], BOUNDS[indices]/SCALE[indices]),
                               loss='soft_l1', f_scale=1., max_nfev=args.max_nfev)
        return expand(result.x), result
    baseline, _ = optimize(np.zeros(26), fixed_indices)
    x, result = optimize(baseline, np.arange(26))
    names = ['camera_'+s for s in ('tx_m', 'ty_m', 'tz_m', 'rx_rad', 'ry_rad', 'rz_rad')]
    names += [f'{side}_J{i}_offset_rad' for side in SIDES for i in range(2, 6)]
    names += [f'{side}_mount_{s}' for side in SIDES for s in ('tx_m', 'ty_m', 'tz_m', 'rx_rad', 'ry_rad', 'rz_rad')]
    eps = 1e-4
    J = np.column_stack([(fit.residual(train, x + np.eye(26)[i]*SCALE[i]*eps) - fit.residual(train, x - np.eye(26)[i]*SCALE[i]*eps))/(2*eps) for i in range(26)])
    _, sv, vt = np.linalg.svd(J, full_matrices=False)
    rank = int(np.sum(sv > sv[0]*1e-5))
    # Conditional linearized uncertainty: omitted geometry/pixel systematics are not covered.
    sigma2 = float(fit.residual(train, x) @ fit.residual(train, x) / max(1, J.shape[0]-26))
    std = np.sqrt(np.maximum(0, np.diag(np.linalg.pinv(J.T@J, rcond=1e-12))*sigma2))*SCALE if rank == 26 else None
    per_frame = []
    metrics = {}
    for split in ('fit', 'holdout'):
        selected = [r for r in rows if r['split'] == split and r['uv']]
        metrics[split] = {**statistics(np.linalg.norm(fit.residual(selected, x).reshape(-1, 2), axis=1)),
                          'fixed_joint_baseline': statistics(np.linalg.norm(fit.residual(selected, baseline).reshape(-1, 2), axis=1)),
                          'captured_frames': sum(r['split'] == split for r in rows), 'detected_frames': len(selected)}
        for row in selected:
            pred, _ = fit.predict(row, x)
            per_frame.append({'id': row['id'], 'split': split, 'markers': {str(key): {**statistics(np.linalg.norm(pred[key]-uv, axis=1)), 'observed_rectified_uv': uv.tolist(), 'predicted_rectified_uv': pred[key].tolist()} for key, uv in row['uv'].items()}})
    report = {'status': 'REAL_IMAGE_FIT_CANDIDATE', 'applied': False, 'physical_accuracy_validated': False,
              'camera_T_world_optical': camera_candidate(fit.camera, x[:6]).tolist(),
              'joint_offsets_rad': {side: [0., *x[6+i*4:10+i*4].tolist(), 0.] for i, side in enumerate(SIDES)},
              'T_link6_marker': {str(marker): camera_candidate(fit.mounts[i], x[14+i*6:20+i*6]).tolist() for i, marker in enumerate(IDS)},
              'fixed_joint_baseline': {
                  'camera_T_world_optical': camera_candidate(fit.camera, baseline[:6]).tolist(),
                  'T_link6_marker': {str(marker): camera_candidate(fit.mounts[i], baseline[14+i*6:20+i*6]).tolist() for i, marker in enumerate(IDS)},
                  'joint_offsets_rad': {side: [0.]*6 for side in SIDES}},
              'metrics_rectified_px': metrics, 'per_frame': per_frame,
              'observability': {'parameter_count': 26, 'data_rank': rank, 'scaled_condition': float(sv[0]/sv[-1]) if sv[-1] > 1e-12 else None,
                                'singular_values': sv.tolist(), 'weakest_direction_scaled': dict(zip(names, vt[-1].tolist())),
                                'conditional_1sigma': dict(zip(names, std.tolist())) if std is not None else None,
                                'caveat': 'Linearized pixel noise only; nominal base geometry, tag size and fixed gauges are assumptions, not independently measured truth.'},
              'assumptions': ['X5 URDF and nominal CAD arm bases fixed; head world frame inherited from scene seed.',
                              'J1 and J6 offsets fixed zero; J6 exactly couples with free terminal sticker mount. J1 defines nominal base yaw convention.',
                              'Two free rigid sticker mounts; no camera-shell plane constraint or assumed measured installation.',
                              'Black marker side length assumed %.6f m, not independently measured.' % args.size,
                              'Stationary host bracket pairing, not hardware synchronization; left jaw excluded.',
                              '17px markers accepted by user; subpixel accuracy does not imply mm accuracy.'],
              'optimizer': {'success': bool(result.success), 'message': result.message, 'nfev': result.nfev,
                            'at_bound': bool(np.any(abs(x) > .999*BOUNDS)), 'positive_depth': all(np.all(fit.predict(r, x)[1] > .01) for r in rows)},
              'source': {'urdf': str(args.urdf), 'urdf_sha256': hashlib.sha256(args.urdf.read_bytes()).hexdigest(),
                         'camera': k, 'images': {r['id']: r['image_sha256'] for r in rows}},
              'missing_detections': [{'id': r['id'], 'split': r['split'], 'missing_ids': [m for m in IDS if m not in r['uv']]} for r in rows if len(r['uv']) != 2]}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    selected = 'fixed_joint_baseline' if metrics['holdout']['fixed_joint_baseline']['mean_px'] <= metrics['holdout']['mean_px'] else 'joint_candidate'
    report['selected_candidate'] = selected
    overlay_x = baseline if selected == 'fixed_joint_baseline' else x
    overlays = []
    for row in holdout:
        image = cv2.imread(str(root / row['image']))
        pred, _ = fit.predict(row, overlay_x)
        for marker, rectified in pred.items():
            if marker not in row['uv']:
                continue
            raw = rectified.copy()
            for _ in range(15):
                raw += rectified - rectify(raw, k)
            if np.max(abs(rectify(raw, k) - rectified)) > 1e-5:
                raise ValueError('inverse projection failed')
            observed = np.asarray(row['detections'][str(marker)])
            cv2.polylines(image, [np.rint(observed).astype(np.int32)], True, (0, 220, 0), 1)
            cv2.polylines(image, [np.rint(raw).astype(np.int32)], True, (0, 0, 255), 1)
        cv2.putText(image, 'green observed / red fitted; heldout', (8, 20), cv2.FONT_HERSHEY_SIMPLEX, .45, (255,255,255), 1)
        overlay = args.output.parent / (args.output.stem + '-' + Path(row['image']).stem + '-overlay.png')
        if not cv2.imwrite(str(overlay), image):
            raise ValueError('overlay write failed')
        overlays.append(str(overlay))
    report['heldout_overlays'] = overlays
    with args.output.open('x') as out:
        json.dump(report, out, indent=2, allow_nan=False)
    print(json.dumps({'output': str(args.output), 'metrics': metrics, 'rank': rank, 'optimizer': report['optimizer']}, indent=2))


def write_progress(output, value):
    path = output.with_suffix(".progress.json")
    temporary = path.with_suffix(".json.tmp")
    temporary.write_text(json.dumps(value, allow_nan=False))
    temporary.replace(path)


def run(args):
    """Workbench defaults to joint mesh+sticker; legacy baseline remains explicit."""
    if args.solver == 'corner-only':
        return run_corner_only(args)
    from arx_env.joint_head_workbench_geometry import Geometry
    from arx_env.joint_head_x5_hybrid import fit_head
    if args.workbench_config is None:
        raise ValueError('hybrid solver requires --workbench-config with full X5 mesh assets')
    write_progress(args.output, dict(fraction=0., label="head_loading", completed=0, total=1))
    root = args.dataset.resolve()
    completion = root/'completion.json'
    if not completion.exists() or json.loads(completion.read_text()).get('passed') is not True:
        raise ValueError('passed capture completion required')
    if args.output.exists():
        raise ValueError('output must be new')
    k = json.loads((root/'camera.json').read_text())
    rows, coverage = load_rows(root, k, return_coverage=True)
    completion_data = json.loads(completion.read_text())
    recovered_count = len(coverage.get('legacy_marker_only_recovered_ids', []))
    normal_counts_match = (completion_data.get('captured') == coverage['captured_frames']
                           and completion_data.get('accepted') == coverage['used_frames']
                           and completion_data.get('failed',0) == len(coverage['rejected_frames']))
    legacy_recovery_match = (recovered_count > 0
                             and completion_data.get('captured') == coverage['captured_frames']
                             and completion_data.get('accepted', 0) + recovered_count == coverage['used_frames']
                             and completion_data.get('failed', 0) == recovered_count)
    coverage['completion_counts_match'] = bool(normal_counts_match or legacy_recovery_match)
    config = json.loads(args.workbench_config.read_text())
    if Path(config['urdf']).resolve() != args.urdf.resolve() or Path(config['scene_seed']).resolve() != args.scene.resolve():
        raise ValueError('solver and mesh geometry must use identical URDF and arm-base scene')
    snapshots = {p.resolve():hashlib.sha256(p.read_bytes()).hexdigest()
                 for p in [args.workbench_config, args.urdf, args.scene, args.camera_seed,
                           completion, root/'camera.json', *[Path(p) for p in coverage['source_files']],
                           *([root/'dataset.json'] if (root/'dataset.json').exists() else [])]}
    mesh_manifest = Path(config['mesh_manifest'])
    manifest = json.loads(mesh_manifest.read_text())
    paths = ([part['path'] for part in manifest['parts']] if config.get('model_family')=='ac_one'
             else manifest['model']['meshes'].values())
    mesh_paths = [mesh_manifest, *[mesh_manifest.parent/str(p) for p in paths]]
    if config.get('chassis_mesh'):
        mesh_paths.append(mesh_manifest.parent/config['chassis_mesh'])
    snapshots.update({p.resolve():hashlib.sha256(p.read_bytes()).hexdigest() for p in mesh_paths})
    if config.get('jaw_mapping_file'):
        from arx_env.joint_head_jaw_mapping import configure_mapping
        configure_mapping(config['jaw_mapping_file'])
    geometry = Geometry(config)
    if not np.isclose(geometry.marker_size, args.size, rtol=0, atol=1e-9):
        raise ValueError('solver and mesh geometry marker dimensions differ')
    seed = Fit(args.urdf, json.loads(args.scene.read_text()), json.loads(args.camera_seed.read_text()), k, args.size,
               geometry=geometry if config.get('model_family')=='ac_one' else None)
    fallback_mounts = None
    if args.workbench_config is not None:
        cfg = json.loads(args.workbench_config.read_text())
        fallback_mounts = json.loads(Path(cfg['candidate']).read_text()).get('T_link6_marker')
    seed.initialize_mounts(rows, fallback_mounts=fallback_mounts)
    frozen_mounts = json.loads(args.marker_mounts.read_text()) if args.marker_mounts else None
    if frozen_mounts is not None:
        # Seed from the frozen values as well, so the manifest, the corner baseline and
        # the stored parameters all describe the same sticker placement.
        for index, marker in enumerate(IDS):
            if str(marker) in frozen_mounts:
                seed.mounts[index] = rigid(frozen_mounts[str(marker)])
    dataset_metadata = json.loads((root/'dataset.json').read_text()) if (root/'dataset.json').exists() else {}
    report = fit_head(geometry, rows, root, k, seed.camera, seed.mounts,
                      args.output.with_suffix(''), iterations=args.iterations,
                      samples=args.samples, max_nfev=args.max_nfev, capture_coverage=coverage,
                      frozen_mounts=frozen_mounts, frozen_mount_provenance=args.marker_mounts_provenance,
                      source_metadata=dataset_metadata,
                      progress=lambda value: write_progress(args.output, {**value, "fraction": .98*value["fraction"]}))
    report['source'].update(urdf=str(args.urdf), urdf_sha256=hashlib.sha256(args.urdf.read_bytes()).hexdigest(),
                            geometry_config_sha256=hashlib.sha256(args.workbench_config.read_bytes()).hexdigest())
    if any(hashlib.sha256(path.read_bytes()).hexdigest() != digest for path,digest in snapshots.items()):
        raise ValueError('capture or geometry input changed during fit')
    report['input_sha256'] = {str(path):digest for path,digest in snapshots.items()}
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
    write_progress(args.output, dict(fraction=1., label='head_result_saved', completed=1, total=1))
    print(json.dumps({'output': str(args.output), 'solver': report['solver'],
                      'acceptance': report['acceptance'], 'comparison': report['comparison']}, indent=2))


def evaluate_saved(args):
    """Evaluate frozen transforms only. Test observations cannot enter optimization."""
    root = args.dataset.resolve()
    k = json.loads((root/'camera.json').read_text())
    rows = [r for r in load_rows(root, k) if r['split'] == 'test']
    if not rows:
        raise ValueError('no reserved test rows')
    saved_bytes = args.evaluate_report.read_bytes()
    saved = json.loads(saved_bytes)
    if saved.get('solver') == 'head_mesh_marker_joint_v1':
        raise ValueError('legacy --evaluate-report cannot score hybrid results; use score_head_candidate with frozen RGB masks (no optimization)')
    if hashlib.sha256(args.urdf.read_bytes()).hexdigest() != saved['source']['urdf_sha256']:
        raise ValueError('URDF differs from frozen solver input')
    for key in ('fx', 'fy', 'cx', 'cy', 'distortion_model', 'coefficients'):
        if k[key] != saved['source']['camera'][key]:
            raise ValueError('camera differs from frozen solver input')
    f = Fit(args.urdf, json.loads(args.scene.read_text()),
            {'T_world_camera': saved['camera_T_world_optical']}, k, args.size)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    report = {'status': 'FROZEN_CANDIDATE_TEST', 'optimization_performed': False,
              'applied': False, 'physical_accuracy_validated': False,
              'frozen_report_sha256': hashlib.sha256(saved_bytes).hexdigest(),
              'selected_candidate': 'fixed_joint_baseline',
              'selected_T_world_optical': saved['fixed_joint_baseline']['camera_T_world_optical'],
              'transform_convention': 'T_world_optical maps optical camera points to nominal CAD body/world frame; world is not an independently calibrated physical table frame.',
              'selected_joint_offsets_rad': saved['fixed_joint_baseline']['joint_offsets_rad'],
              'selected_T_link6_marker': saved['fixed_joint_baseline']['T_link6_marker'],
              'candidates': {}, 'overlays': [],
              'assumptions': saved['assumptions'],
              'coverage': [{'id': r['id'], 'detected_ids': list(r['uv']), 'missing_ids': [m for m in IDS if m not in r['uv']],
                            'image_sha256': r['image_sha256']} for r in rows]}
    for name, candidate in (('fixed_joint_baseline', saved['fixed_joint_baseline']), ('joint_candidate', saved)):
        f.camera = rigid(candidate['camera_T_world_optical'])
        f.mounts = [rigid(candidate['T_link6_marker'][str(m)]) for m in IDS]
        x = np.zeros(26)
        for i, side in enumerate(SIDES):
            offsets = candidate['joint_offsets_rad'][side]
            if offsets[0] != 0 or offsets[5] != 0:
                raise ValueError('unexpected nonzero fixed gauge')
            x[6+i*4:10+i*4] = offsets[1:5]
        errors, frames = [], []
        for row in rows:
            pred, depth = f.predict(row, x)
            if np.any(depth <= .01):
                raise ValueError('nonpositive test depth')
            error = np.concatenate([np.linalg.norm(pred[m]-uv, axis=1) for m, uv in row['uv'].items()]) if pred else np.array([])
            errors.extend(error)
            frames.append({'id': row['id'], **(statistics(error) if len(error) else {'corners': 0, 'mean_px': None})})
            if name != 'fixed_joint_baseline':
                continue
            image = cv2.imread(str(root/row['image']))
            for marker, rectified in pred.items():
                if marker not in row['uv']:
                    continue
                raw = rectified.copy()
                for _ in range(15):
                    raw += rectified - rectify(raw, k)
                if np.max(abs(rectify(raw, k)-rectified)) > 1e-5:
                    raise ValueError('inverse projection failed')
                observed = np.asarray(row['detections'][str(marker)])
                cv2.polylines(image, [np.rint(observed).astype(np.int32)], True, (0, 220, 0), 1)
                cv2.polylines(image, [np.rint(raw).astype(np.int32)], True, (0, 0, 255), 1)
            cv2.putText(image, 'fresh test: green observed / red frozen baseline', (8, 20), cv2.FONT_HERSHEY_SIMPLEX, .4, (255,255,255), 1)
            overlay = args.output.parent/(args.output.stem+'-'+Path(row['image']).stem+'-overlay.png')
            if not cv2.imwrite(str(overlay), image):
                raise ValueError('overlay write failed')
            report['overlays'].append(str(overlay))
        report['candidates'][name] = {'metrics_rectified_px': statistics(errors) if errors else None, 'frames': frames}
    if args.evaluate_report.read_bytes() != saved_bytes:
        raise ValueError('frozen report changed during evaluation')
    with args.output.open('x') as out:
        json.dump(report, out, indent=2, allow_nan=False)
    print(json.dumps({'output': str(args.output), 'candidates': report['candidates']}, indent=2))


def self_test():
    k = dict(fx=392., fy=392., cx=315., cy=235., coefficients=[0]*5,
             distortion_model='distortion.inverse_brown_conrady')
    uv = np.array([[30., 40.], [600., 450.]])
    assert np.allclose(rectify(uv, k), uv)
    f = Fit(OLD/'calibration-base-xyz-round1/dataset-v2/model/model.urdf',
            json.loads((OLD/'calibration-motion-design/scene-seed.json').read_text()),
            json.loads((OLD/'calibration-base-xyz-home-v6/outer-mean-fit-v2/result.json').read_text()), k, .015)
    row = {'q': np.zeros(12), 'uv': {24: np.zeros((4, 2)), 30: np.zeros((4, 2))}, 'split': 'fit'}
    for i in range(2):
        T = np.eye(4); T[:3, 3] = [(-1 if i == 0 else 1)*.1, 0, .5]
        f.mounts.append(np.linalg.inv(f.terminal(row, i, np.zeros(4))) @ f.camera @ T)
    row['uv'] = f.predict(row, np.zeros(26))[0]
    f.mounts = []; f.initialize_mounts([row])
    assert np.max(abs(f.residual([row], np.zeros(26)))) < 1e-4
    x = np.zeros(26); x[6] = .01
    assert np.linalg.norm(f.residual([row], x)) > .1
    print('PASS rectification, X5 FK/PnP roundtrip, joint-offset projection')


if __name__ == '__main__':
    if sys.argv[1:] == ['--self-test']:
        self_test()
        raise SystemExit(0)

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--urdf', type=Path, default=OLD/'calibration-base-xyz-round1/dataset-v2/model/model.urdf')
    parser.add_argument('--scene', type=Path, default=OLD/'calibration-motion-design/scene-seed.json')
    parser.add_argument('--camera-seed', type=Path, default=OLD/'calibration-base-xyz-home-v6/outer-mean-fit-v2/result.json')
    parser.add_argument('--size', type=float, default=.015)
    parser.add_argument('--max-nfev', type=int, default=180)
    parser.add_argument('--solver', choices=('hybrid', 'corner-only'), default='hybrid')
    parser.add_argument('--workbench-config', type=Path)
    parser.add_argument('--iterations', type=int, default=8)
    parser.add_argument('--samples', type=int, default=200)
    parser.add_argument('--evaluate-report', type=Path, help='evaluate reserved test split using frozen transforms; no fitting')
    parser.add_argument('--marker-mounts', type=Path,
                        help='json {marker id: 4x4} to hold those sticker installations fixed (e.g. the wrist half estimate)')
    parser.add_argument('--marker-mounts-provenance', help='free text naming where the frozen mounts came from')
    args = parser.parse_args()
    evaluate_saved(args) if args.evaluate_report else run(args)
