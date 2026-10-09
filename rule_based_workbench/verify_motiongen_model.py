"""Real-GPU validation of the fixed-mesh/moving-sphere robot; no executable plan.

Run only on the authorized DSW. Synthetic overlap probes preserve the generated
radii and collision masks, but deliberately move sphere centers: they verify
cuRobo world/self semantics. Separate recorded-pose checks require native mast
collision positives and nearby native-clear negatives using real FK spheres.
"""
import argparse
from copy import deepcopy
import json
import hashlib
import math
from pathlib import Path
import sys
import time


def recorded_cases(args, original):
    """Reuse recorded feedback and the nearest historically clear plan targets."""
    import numpy as np
    from curobo_planning import LEFT_JOINTS
    paths = {name: getattr(args, name) for name in
             ('mast_state', 'historical_plan', 'historical_profile', 'historical_regression')}
    raw = {name: path.read_bytes() for name, path in paths.items()}
    values = {name: json.loads(blob) for name, blob in raw.items()}
    historical = values['mast_state']
    if historical.get('diagnostic_only_not_command') is not True:
        raise ValueError('recorded mast feedback must be labelled diagnostic only')
    reference = historical['positions']
    if set(reference) != set(original) or not all(math.isfinite(v) for v in reference.values()):
        raise ValueError('complete finite recorded mast model positions required')
    plan, profile, regression = (values[name] for name in ('historical_plan', 'historical_profile', 'historical_regression'))
    offset = profile['joints']['zero_offsets_rad'][:6]
    if len(offset) != 6:
        raise ValueError('historical reported-to-model zero offsets required')
    candidates = []
    for row in regression['rows']:
        if row['kind'] != 'original-plan' or row['free'] is not True:
            continue
        tick = row['tick']
        q = [a+b for a, b in zip(plan['ticks'][tick]['target']['left_arm_joints'], offset, strict=True)]
        if len(q) != 6 or not all(math.isfinite(v) for v in q):
            raise ValueError('invalid historical model target')
        distance = float(np.linalg.norm(np.asarray(q)-[reference[name] for name in LEFT_JOINTS]))
        candidates.append(dict(case='historical_plan_tick_'+str(tick), tick=tick, left_arm_distance_rad=distance,
            positions={**original, **dict(zip(LEFT_JOINTS, q))}))
    candidates.sort(key=lambda row: (row['left_arm_distance_rad'], row['tick']))
    # Bounded verification, not another trajectory search. Keep nearby distinct
    # arm states; the current native checker still decides whether they are clear.
    selected = []
    for row in candidates:
        if all(np.linalg.norm(np.asarray([row['positions'][n] for n in LEFT_JOINTS])-
                              [other['positions'][n] for n in LEFT_JOINTS]) >= .02 for other in selected):
            selected.append(row)
        if len(selected) == 24:
            break
    return historical, selected, {name: dict(path=str(paths[name]), sha256=hashlib.sha256(blob).hexdigest())
                                  for name, blob in raw.items()}


def run(args):
    from benchmark_planning import write_new
    args.output.mkdir(parents=True, exist_ok=False)
    report = dict(passed=False, hardware_access=False, executable_plan=False, offline_candidate=True,
                  scope='all source-part representation, real FK, world/self masks and recorded mast cases; not full path or solid-volume certification')
    model_report = None
    started = time.perf_counter()
    try:
        if sys.platform == 'darwin':
            raise RuntimeError('official sphere fitting/GPU checks must run on the authorized DSW, never this Mac')
        report['phase'] = 'load_frozen_geometry'
        from playground_geometry import load
        source = json.loads(args.source.read_text())
        planner, observation, exported = load(args.input, source, args.native_code, args.assets)
        from robot_sim.asset_cfg.robots.ac_one_spec import ac_one_asset_spec
        from curobo_robot_model import build_robot_model, fixed_world_mesh_descriptors, digest
        from curobo_planning import LEFT_JOINTS
        report['phase'] = 'moving_sphere_fitting_and_fixed_mesh_validation'
        fitting = time.perf_counter()
        config, model_report = build_robot_model(exported, ac_one_asset_spec(args.assets).urdf_path,
            planner.fk.movable, planner.home_state, ignored_link_pairs=planner.geo.whole.ignores)
        report['fitting_seconds'] = time.perf_counter()-fitting
        if (model_report.get('numeric_quality_gate_passed') is not True or model_report.get('fixture_fitter_used')
                or model_report['part_coverage']['complete'] is not True
                or model_report['part_coverage']['source_parts'] != len(exported['parts'])):
            raise ValueError('current full model has not passed its sphere quality and source-part coverage gates')
        fixed_meshes = fixed_world_mesh_descriptors(exported, model_report, planner.fk, planner.home_state)
        sphere_parts = {row['name'] for row in model_report['part_reports']}
        fixed_parts = {row['source_part_name'] for row in fixed_meshes}
        if sphere_parts & fixed_parts or sphere_parts | fixed_parts != {row['name'] for row in exported['parts']}:
            raise AssertionError('source parts are missing, duplicated, or represented by both schemes')
        report['fixed_meshes'] = dict(part_count=len(fixed_meshes), triangles=sum(len(row['faces']) for row in fixed_meshes),
            descriptors_sha256=digest(fixed_meshes), locked_positions=model_report['fixed_world_geometry']['locked_positions'],
            source_part_coverage=model_report['part_coverage'], winding_repaired_without_triangle_geometry_changes=True)
        write_new(args.output/'robot-config.json', config)

        report['phase'] = 'gpu_model_creation'
        import numpy as np
        import torch
        import curobo
        from curobo.types.base import TensorDeviceType
        from curobo.types.robot import RobotConfig
        from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel
        from curobo.geom.types import Cuboid, Mesh, WorldConfig
        from curobo.geom.sdf.world import CollisionQueryBuffer, WorldCollisionConfig, WorldPrimitiveCollision
        from curobo.geom.sdf.world_mesh import WorldMeshCollision
        from curobo.rollout.cost.self_collision_cost import SelfCollisionCost, SelfCollisionCostConfig
        from robot_sim.asset_cfg.objects.coffee_machine import RigidTransform
        from robot_sim.tasks.manager_based.coffee_make.curobo_collision_cache import refresh_slow_self_collision_outputs
        if curobo.__version__ != '0.7.8' or not torch.cuda.is_available():
            raise RuntimeError('actual cuRobo 0.7.8 and CUDA are required')
        tensor_args = TensorDeviceType(device=torch.device('cuda:0'))
        # Collision geometry is unchanged. Explicitly store aliases for FK inspection;
        # a MotionGen goal configuration must not freeze all these inspection links.
        inspected = deepcopy(config)
        extra = deepcopy(inspected['kinematics']['extra_links'])
        inspected['kinematics']['link_names'] = list(dict.fromkeys([
            *config['kinematics']['link_names'], *extra]))
        robot = RobotConfig.from_dict(deepcopy(inspected), tensor_args)
        model = CudaRobotModel(robot.kinematics)
        assert tuple(model.joint_names) == LEFT_JOINTS, model.joint_names
        report.update(curobo_version=curobo.__version__, torch_version=torch.__version__,
                      gpu=torch.cuda.get_device_name(0), fk_inspection_links=inspected['kinematics']['link_names'])

        report['phase'] = 'all_link_fk_parity'
        original = dict(planner.home_state)
        q = np.asarray([[original[name] for name in LEFT_JOINTS]], dtype=float)
        bounds = model_report['kinematics']['joints']
        lower, upper = ([bounds[name][key] for name in LEFT_JOINTS] for key in ('lower', 'upper'))
        rng = np.random.default_rng(29)
        q = np.concatenate((q, np.clip(q+rng.uniform(-.08, .08, (8, 6)), lower, upper)))
        q_gpu = tensor_args.to_device(q)
        sphere_indices = {alias: model.kinematics_config.get_sphere_index_from_link_name(alias)
                          .detach().cpu().tolist() for alias in config['kinematics']['collision_link_names']}

        def fk_check(positions):
            state = model.get_state(q_gpu)
            torch.cuda.synchronize()
            gpu_xyz = state.links_position.detach().cpu().numpy().copy()
            gpu_quat = state.links_quaternion.detach().cpu().numpy().copy()
            gpu_spheres = state.get_link_spheres().detach().cpu().numpy().copy()
            names = list(state.link_names)
            if set(names) != set(inspected['kinematics']['link_names']):
                raise AssertionError('GPU FK omitted actual links or fixed aliases')
            position_errors, rotation_errors, sphere_errors, radius_errors = [], [], [], []
            fixed_position_errors, fixed_rotation_errors = [], []
            for index, row in enumerate(q):
                frames = {name: planner.fk.pose(frame) for name, frame in planner.fk.forward(
                    {**positions, **dict(zip(LEFT_JOINTS, row))}).items()}
                for alias, values in extra.items():
                    transform = values['fixed_transform']
                    frames[alias] = frames[values['parent_link_name']].compose(
                        RigidTransform(tuple(transform[:3]), tuple(transform[3:])))
                for link, xyz, quat in zip(names, gpu_xyz[index], gpu_quat[index]):
                    expected = frames[link]
                    position_errors.append(float(np.linalg.norm(xyz-expected.position)))
                    rotation_errors.append(float(min(np.linalg.norm(quat-expected.orientation_wxyz),
                                                     np.linalg.norm(quat+expected.orientation_wxyz))))
                for item in fixed_meshes:
                    link_index = names.index(item['link'])
                    expected = RigidTransform(**item['base_from_shape'])
                    fixed_position_errors.append(float(np.linalg.norm(gpu_xyz[index, link_index]-expected.position)))
                    fixed_rotation_errors.append(float(min(
                        np.linalg.norm(gpu_quat[index, link_index]-expected.orientation_wxyz),
                        np.linalg.norm(gpu_quat[index, link_index]+expected.orientation_wxyz))))
                for alias, indices in sphere_indices.items():
                    definitions = config['kinematics']['collision_spheres'][alias]
                    if len(indices) != len(definitions):
                        raise AssertionError('actual GPU sphere count differs: '+alias)
                    padding = config['kinematics']['collision_sphere_buffer'].get(alias, 0.)
                    for sphere_index, sphere in zip(indices, definitions):
                        expected = frames[alias].transform_point(tuple(sphere['center']))
                        sphere_errors.append(float(np.linalg.norm(gpu_spheres[index, sphere_index, :3]-expected)))
                        radius_errors.append(abs(float(gpu_spheres[index, sphere_index, 3])-(sphere['radius']+padding)))
            if (not np.isfinite(position_errors+rotation_errors+sphere_errors+radius_errors).all()
                    or max(position_errors) > 2e-5 or max(rotation_errors) > 2e-5
                    or max(sphere_errors) > 2e-5 or max(radius_errors) > 1e-6
                    or max(fixed_position_errors) > 2e-5 or max(fixed_rotation_errors) > 2e-5):
                raise AssertionError(f'full-model FK mismatch: position={max(position_errors)}, quaternion={max(rotation_errors)}')
            return dict(configurations=len(q), real_links=len(config['kinematics']['link_names']), fixed_aliases=len(extra),
                        max_position_error_m=max(position_errors), max_quaternion_chord_error=max(rotation_errors),
                        max_sphere_center_error_m=max(sphere_errors), max_sphere_radius_error_m=max(radius_errors),
                        fixed_mesh_aliases=len(fixed_meshes), fixed_mesh_links_checked=sorted({item['link'] for item in fixed_meshes}),
                        max_fixed_mesh_position_error_m=max(fixed_position_errors), max_fixed_mesh_quaternion_error=max(fixed_rotation_errors),
                        position_tolerance_m=2e-5, quaternion_chord_tolerance=2e-5)

        report['fk_initial'] = fk_check(original)
        native_spheres = model.get_state(q_gpu[:1]).get_link_spheres().detach().clone()
        torch.cuda.synchronize()

        report['phase'] = 'locked_jaw_update'
        changed = dict(original)
        locks = deepcopy(inspected['kinematics']['lock_joints'])
        for names in (('left_joint7', 'left_joint8'),):
            lo, hi = max(bounds[name]['lower'] for name in names), min(bounds[name]['upper'] for name in names)
            old = original[names[0]]
            delta = min(.004, (hi-lo)/4)
            new = old-delta if old-delta >= lo else old+delta
            if not lo <= new <= hi or abs(new-old) < 1e-6:
                raise ValueError('cannot form a legal, nontrivial symmetric locked-jaw update')
            for name in names:
                changed[name] = locks[name] = new
        updated = deepcopy(inspected)
        updated['kinematics']['lock_joints'] = locks
        updated_robot = RobotConfig.from_dict(deepcopy(updated), tensor_args)
        # This is the same native kinematics update invoked by MotionGen.update_locked_joints.
        model.update_kinematics_config(updated_robot.kinematics.kinematics_config)
        from motiongen_motion import sync_locked_joint_metadata
        sync_locked_joint_metadata(model, locks)
        report['fk_locked_jaws'] = fk_check(changed)
        locked_values = dict(zip(model.lock_jointstate.joint_names,
                                model.lock_jointstate.position.detach().cpu().reshape(-1).tolist()))
        assert all(abs(locked_values[name]-value) <= 1e-7 for name, value in locks.items()), locked_values
        report['locked_jaw_updates'] = {name: dict(before=original[name], after=changed[name])
                                       for name in locks if changed[name] != original[name]}
        assert all(changed[name] == value for name, value in model_report['fixed_world_geometry']['locked_positions'].items())
        model.update_kinematics_config(robot.kinematics.kinematics_config)
        sync_locked_joint_metadata(model, inspected['kinematics']['lock_joints'])

        report['phase'] = 'gpu_world_and_self_exemption_probes'
        kin = model.kinematics_config
        self_config = model.get_self_collision_config()
        base_aliases = model_report['base_collision_aliases']
        base_ids = [int(index) for alias in base_aliases
                    for index in kin.get_sphere_index_from_link_name(alias).detach().cpu().tolist()]
        moving_aliases = [row['alias'] for row in model_report['part_map'].values()
                          if row['link'] == 'left_link3' and row['representation'] == 'robot_spheres']
        if not base_ids or not moving_aliases:
            raise ValueError('actual base spheres and moving link3 spheres are required')
        moving_ids = [int(index) for alias in moving_aliases
                      for index in kin.get_sphere_index_from_link_name(alias).detach().cpu().tolist()]
        if self_config.experimental_kernel:
            enabled_pairs = {tuple(sorted(pair)) for pair in self_config.thread_location[:self_config.thread_max]
                             .detach().cpu().reshape(-1, 2).tolist()}
            is_enabled = lambda a, b: tuple(sorted((a, b))) in enabled_pairs
        else:
            matrix = self_config.collision_matrix.detach().cpu()
            is_enabled = lambda a, b: bool(matrix[a, b].item())
        chosen = next(((a, b) for a in base_ids for b in moving_ids if is_enabled(a, b)), None)
        if chosen is None:
            raise AssertionError('no nonexempt base/link3 sphere pair exists in actual GPU self config')
        base_id, moving_id = chosen
        radii = native_spheres[0, :, 3]
        if not torch.all(radii[base_ids] < -1.).item() or radii[moving_id].item() <= 0:
            raise AssertionError('actual GPU sphere radii do not preserve base/world-only exclusion')
        # Preserve every generated radius and exclusion. Only artificial positions
        # isolate an enabled pair; neither meshes nor fit spheres are replaced.
        separated = native_spheres.unsqueeze(1).clone()
        offsets = self_config.offset.detach()
        stride = 10.*(1.+float(torch.max(torch.abs(radii+offsets)).item()))
        separated[0, 0, :, 0] = 100.+stride*torch.arange(len(radii), device=tensor_args.device)
        separated[0, 0, :, 1:3] = 0.
        separated[0, 0, base_id, :3] = 0.

        world = WorldPrimitiveCollision(WorldCollisionConfig(tensor_args=tensor_args,
            world_model=WorldConfig(cuboid=[Cuboid(name='overlap_probe', dims=[.2, .2, .2], pose=[0., 0., 0., 1., 0., 0., 0.])]),
            cache={'obb': 1}))
        world_buffer = CollisionQueryBuffer.initialize_from_shape(separated.shape, tensor_args, world.collision_types)
        weight, activation = tensor_args.to_device([1.]), tensor_args.to_device([0.])

        def world_cost(spheres):
            value = world.get_sphere_distance(spheres, world_buffer, weight, activation)
            torch.cuda.synchronize()
            return value.detach().cpu().clone()

        inside = separated.clone()
        inside[0, 0, base_ids, :3] = 0.
        base_cost = world_cost(inside)
        assert torch.count_nonzero(base_cost).item() == 0, 'disabled base/world spheres generated world cost'
        inside[0, 0, moving_id, :3] = 0.
        positive_control = world_cost(inside)
        assert positive_control[0, 0, moving_id].item() > 0, 'world collision kernel positive control failed'
        assert torch.count_nonzero(positive_control[0, 0, base_ids]).item() == 0
        inside[0, 0, moving_id, :3] = separated[0, 0, moving_id, :3]
        assert torch.count_nonzero(world_cost(inside)).item() == 0, 'world collision output remained stale'
        report['base_world_probe'] = dict(passed=True, base_spheres_tested=len(base_ids),
            base_cost_max=float(base_cost.max()), positive_control_cost=float(positive_control[0, 0, moving_id]),
            library='WorldPrimitiveCollision.get_sphere_distance', radii_preserved=True)
        model_report['base_world_exclusion_gpu_verified'] = True

        self_cost = SelfCollisionCost(SelfCollisionCostConfig(weight=1., tensor_args=tensor_args,
                                                              self_collision_kin_config=self_config))
        cache_refresh = refresh_slow_self_collision_outputs(self_cost)
        from motiongen_self_collision import split_self_collision_cost
        self_partition = split_self_collision_cost(self_cost)
        def collision_cost(spheres):
            value = self_cost(spheres)
            torch.cuda.synchronize()
            return float(value.detach().cpu().max())
        far_before = collision_cost(separated)
        overlap = separated.clone()
        overlap[0, 0, moving_id, :3] = overlap[0, 0, base_id, :3]
        close = collision_cost(overlap)
        far_after = collision_cost(separated)
        if not all(math.isfinite(value) for value in (far_before, close, far_after)) or far_before != 0. or close <= 0. or far_after != 0.:
            raise AssertionError(f'actual self kernel failed far/overlap/far: {[far_before, close, far_after]}')
        report['base_self_probe'] = dict(passed=True, base_sphere_index=base_id, moving_sphere_index=moving_id,
            moving_link='left_link3', kernel_pair_enabled=True, far_before=far_before, overlap=close, far_after=far_after,
            actual_world_radius_m=float(radii[base_id]), actual_self_radius_offset_m=float(offsets[base_id]),
            library='SelfCollisionCost', experimental_kernel=self_config.experimental_kernel,
            slow_output_refresh_applied=cache_refresh, runtime_fast_partition=self_partition,
            radii_and_collision_masks_preserved=True)
        model_report['base_self_collision_gpu_verified'] = True

        report['phase'] = 'retained_mount_local_exemption'
        mount_probes = []
        for exemption in model_report['exemptions']:
            if exemption['reason'] != 'pinned_mount_component':
                continue
            aliases = [model_report['part_map'][name]['alias'] for name in exemption['parts']]
            first, second = [sphere_indices[alias] for alias in aliases]
            if any(is_enabled(a, b) for a in first for b in second):
                raise AssertionError('pinned mounting component exemption absent from GPU pair mask')
            probe = separated.clone()
            probe[0, 0, first[0], :3] = probe[0, 0, second[0], :3]
            cost = collision_cost(probe)
            if cost != 0.:
                raise AssertionError('exempt mount/link1 overlap generated a self collision cost')
            mount_probes.append(dict(parts=exemption['parts'], tested_sphere_pairs=len(first)*len(second), overlap_cost=cost))
        if not mount_probes:
            raise AssertionError('no retained left mounting component exemptions were tested')
        report['mount_component_probes'] = dict(passed=True, cases=mount_probes,
            nonexempt_same_mount_vs_link3_positive=report['base_self_probe']['overlap'])

        report['phase'] = 'fixed_mesh_world_and_recorded_mast_cases'
        mesh_world = WorldMeshCollision(WorldCollisionConfig(tensor_args=tensor_args,
            world_model=WorldConfig(mesh=[Mesh(name=item['name'], vertices=item['vertices'], faces=item['faces'],
                pose=[*item['base_from_shape']['position'], *item['base_from_shape']['orientation_wxyz']]) for item in fixed_meshes]),
            cache={'mesh': len(fixed_meshes)}))
        # Check what Warp actually loaded, not just the configuration we supplied.
        mesh_pose_errors = []
        for item in fixed_meshes:
            cached = mesh_world._wp_mesh_cache[item['name']]
            if (not np.array_equal(cached.vertices.numpy(), np.asarray(item['vertices'], dtype=np.float32))
                    or not np.array_equal(cached.faces.numpy(), np.asarray(item['faces']).reshape(-1))):
                raise AssertionError('GPU fixed mesh vertices/faces differ from the original component descriptor')
            index = mesh_world.get_mesh_idx(item['name'])
            actual_pose = mesh_world._mesh_tensor_list[1][0, index, :7].detach().cpu().numpy()
            expected = RigidTransform(**item['base_from_shape']).inverse()
            mesh_pose_errors.extend((float(np.linalg.norm(actual_pose[:3]-expected.position)),
                float(min(np.linalg.norm(actual_pose[3:]-expected.orientation_wxyz),
                          np.linalg.norm(actual_pose[3:]+expected.orientation_wxyz)))))
        if max(mesh_pose_errors) > 2e-5:
            raise AssertionError('installed GPU fixed mesh inverse poses differ from native FK')
        report['fixed_meshes'].update(gpu_vertices_faces_equal=True, gpu_loaded_components=len(fixed_meshes),
                                     max_gpu_inverse_pose_error=max(mesh_pose_errors))
        mesh_buffer = CollisionQueryBuffer.initialize_from_shape(separated.shape, tensor_args, mesh_world.collision_types)
        def mesh_cost(spheres, only=None):
            for item in fixed_meshes:
                mesh_world.enable_mesh(only is None or item['name'] in only, name=item['name'])
            value = mesh_world.get_sphere_distance(spheres, mesh_buffer, weight, activation)
            torch.cuda.synchronize()
            values = value.detach().cpu().clone()
            if not torch.isfinite(values).all():
                raise AssertionError('nonfinite fixed robot mesh collision query')
            return values

        def real_spheres(positions):
            values = tensor_args.to_device([[positions[name] for name in LEFT_JOINTS]])
            spheres = model.get_state(values).get_link_spheres().detach().clone().unsqueeze(1)
            torch.cuda.synchronize()
            return spheres

        def native_audit(positions):
            frames = {name: planner.fk.pose(matrix) for name, matrix in planner.fk.forward(positions).items()}
            return planner.geo.whole.audit(frames, margin=.0001,
                world_margin_paths=planner.geo.machine_shape_paths, world_margin=.003)

        historical, nearby, provenance = recorded_cases(args, original)
        historical_native = native_audit(historical['positions'])
        mast_cases = dict(provenance=provenance, historical_feedback_case=historical['case'],
            historical_source_sha256=historical['source_sha256'], raw_recorded_native=historical_native,
            current_lock_basis='model configuration fixed right-arm/jaws; historical feedback is diagnostic, not commanded',
            positives=[], negatives=[], current_native_rejected_candidates=[],
            scope='current whole native geometry for negatives; recorded left-link3/mast pair for positives; no robot execution')
        report['recorded_mast_cases'] = mast_cases
        fragments = historical['expected_pair_fragments']
        recorded_pairs = [pair for pair in historical_native['violations'] if all(token in str(pair) for token in fragments)]
        if not recorded_pairs:
            raise AssertionError('raw recorded mast collision no longer reproduced by native geometry')
        current = {**original, **{name: historical['positions'][name] for name in LEFT_JOINTS}}
        current_native = native_audit(current)
        mast_cases['current_lock_native'] = current_native
        pairs = [pair for pair in current_native['violations'] if all(token in str(pair) for token in fragments)]
        by_source = {item['source_part_name']: item for item in fixed_meshes}
        mast_names = sorted({name for pair in pairs for name in pair[:2] if name in by_source and name.startswith('base_link/')})
        if not mast_names:
            raise AssertionError('mast collision did not survive the explicit current-lock diagnostic rebind')
        actual = real_spheres(current)
        for name in mast_names:
            values = mesh_cost(actual, only={by_source[name]['name']})
            cost = float(values[0, 0, moving_ids].max())
            mast_cases['positives'].append(dict(component=name, left_link='left_link3', gpu_cost=cost,
                positions=current, radii_and_fk_centers_preserved=True))
        for candidate in nearby:
            result = native_audit(candidate['positions'])
            if not result['free']:
                mast_cases['current_native_rejected_candidates'].append(dict(case=candidate['case'], violations=result['violations']))
                continue
            spheres = real_spheres(candidate['positions'])
            world_value, self_value = float(mesh_cost(spheres).max()), collision_cost(spheres)
            mast_cases['negatives'].append(dict(**candidate, current_native=result,
                gpu_fixed_mesh_world_cost=world_value, gpu_self_cost=self_value,
                radii_and_fk_centers_preserved=True))
            if len(mast_cases['negatives']) == 3:
                break
        if len(mast_cases['negatives']) < 3:
            raise AssertionError('fewer than three nearby recorded targets remain native-clear in the current scene')
        if any(row['gpu_cost'] <= 0 for row in mast_cases['positives']):
            raise AssertionError('GPU fixed mesh query missed a native historical mast collision pair')
        if any(row['gpu_fixed_mesh_world_cost'] != 0. or row['gpu_self_cost'] != 0. for row in mast_cases['negatives']):
            raise AssertionError('GPU robot proxy produced false positives on nearby current-native-clear poses')
        mast_cases['passed'] = True
        model_report['historical_mast_gpu_collision_verified'] = True
        model_report['nearby_safe_gpu_cases_verified'] = True
        report.update(passed=True, phase='complete', allocated_spheres=model_report['allocated_spheres'],
                      robot_config_sha256=model_report['robot_config_sha256'],
                      optimization_proxy_only=True, conservative_coverage_proven=False)
    except Exception as error:
        import traceback
        report.update(error=f'{type(error).__name__}: {error}', traceback=traceback.format_exc())
    if model_report is not None:
        write_new(args.output/'model-report.json', model_report)
    else:
        write_new(args.output/'model-report.json', dict(generated=False, passed=False,
            phase=report.get('phase'), error=report.get('error'), fallback_used=False))
    report['total_seconds'] = time.perf_counter()-started
    write_new(args.output/'result.json', report)
    print(json.dumps(report, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('input', 'source', 'native-code', 'assets', 'output', 'mast-state',
                 'historical-plan', 'historical-profile', 'historical-regression'):
        parser.add_argument('--'+name, type=Path, required=True)
    raise SystemExit(run(parser.parse_args()))
