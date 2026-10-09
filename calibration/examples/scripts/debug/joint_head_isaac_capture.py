#!/usr/bin/env python3
"""Task-isolated Isaac/PhysX calibration capture. No real robot control.

Uses the deployed robot_sim source and assets without modifying them. Images
come from RTX, state from PhysX, and hidden truth is a separate artifact.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import time
import traceback


def marker_mounts(plan):
    """Keep nominal mounts in fit inputs; injected mounting error is scorer-only."""
    import copy
    import numpy as np
    from scipy.spatial.transform import Rotation
    markers = copy.deepcopy(plan.get('markers', []))
    errors = plan.get('marker_mount_errors', {})
    ids = [m['id'] for m in markers]
    if len(set(ids)) != len(ids) or set(errors) - {str(i) for i in ids}:
        raise ValueError('duplicate marker or unknown mounting-error ID')
    for marker in markers:
        error = errors.get(str(marker['id']), {})
        xyz = np.asarray(error.get('xyz_m', [0, 0, 0]), float)
        rpy = np.asarray(error.get('rpy_deg', [0, 0, 0]), float)
        if (xyz.shape != (3,) or rpy.shape != (3,) or not np.isfinite(xyz).all()
                or not np.isfinite(rpy).all() or abs(xyz).max() > .005 or abs(rpy).max() > 5):
            raise ValueError('invalid bounded marker mounting perturbation')
        mount = np.asarray(marker['T_link_marker'], float).copy()
        mount[:3, 3] += xyz
        mount[:3, :3] = Rotation.from_euler('xyz', rpy, degrees=True).as_matrix() @ mount[:3, :3]
        marker['T_link_marker'] = mount.tolist()
    return markers


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source', type=Path, required=True)
    p.add_argument('--assets', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--plan', type=Path)
    p.add_argument('--renderer-gpu-index', type=int, choices=(0, 1), default=0,
                   help='physical Vulkan device; CUDA device remains logical cuda:0')
    p.add_argument('--dry-run', action='store_true')
    args = p.parse_args()
    urdf = args.assets / 'robots/arx_ac_one/v1/source/acone/urdf/acone_no_handle_d405.urdf'
    for path in (urdf, args.source / 'source/robot_sim/robot_sim/tasks/manager_based/coffee_make/env.py'):
        if not path.is_file():
            raise FileNotFoundError(path)
    if args.output.exists():
        raise FileExistsError(args.output)
    if args.dry_run:
        print(json.dumps({'dry_run': True, 'urdf_sha256': hashlib.sha256(urdf.read_bytes()).hexdigest()}))
        return
    os.environ['ROBOT_SIM_ASSET_ROOT'] = str(args.assets.resolve())
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / 'run.json').write_text(json.dumps({'pid': os.getpid(), 'started_unix': time.time(),
        'source': str(args.source), 'assets': str(args.assets), 'plan': str(args.plan),
        'runner_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'plan_sha256': hashlib.sha256(args.plan.read_bytes()).hexdigest() if args.plan else None}, indent=2))
    from isaaclab.app import AppLauncher
    app = AppLauncher(headless=True, enable_cameras=True, device='cuda:0', multi_gpu=False,
                      kit_args=f'--/renderer/gpuEnumeration/glInterop/enabled=false --/renderer/activeGpu={args.renderer_gpu_index} --/renderer/multiGpu/enabled=false').app
    passed = False
    try:
        run(args, urdf)
        passed = True
    except BaseException:
        (args.output / 'failure.txt').write_text(traceback.format_exc())
        traceback.print_exc()
    finally:
        (args.output / 'completion.json').write_text(json.dumps({'passed': passed}) + '\n')
        print(f'CALIBRATION_CAPTURE_PASSED={passed}', flush=True)
        app.close(skip_cleanup=True)
    if not passed:
        raise SystemExit(1)


def run(args, urdf):
    import numpy as np
    import torch
    from PIL import Image
    from scipy.spatial.transform import Rotation
    import isaaclab.sim as sim_utils
    from robot_sim.asset_cfg.d405 import remap_rgb, sampling_map
    from robot_sim.tasks.manager_based.coffee_make.d405_camera import color_observation
    from robot_sim.tasks.manager_based.coffee_make.env import CoffeeMakeEnv
    from robot_sim.tasks.manager_based.coffee_make.episode_scene import EpisodeScene
    from robot_sim.tasks.manager_based.coffee_make.generalization_env_cfg import make_generalization_env_cfg
    from robot_sim.tasks.manager_based.coffee_make.generalization_execution import ControlSpec
    from robot_sim.tasks.manager_based.coffee_make.interaction_evaluation import AcceptanceSpec

    plan = json.loads(args.plan.read_text()) if args.plan else {}
    names = [*(f'left_joint{i}' for i in range(1, 7)), *(f'right_joint1{i}' for i in range(1, 7))]
    offsets = np.deg2rad(plan.get('joint_offsets_deg', [0.0] * 12))
    if offsets.shape != (12,) or not np.isfinite(offsets).all() or abs(offsets).max() > np.deg2rad(5):
        raise ValueError('Invalid injected joint offsets')
    cfg = make_generalization_env_cfg(control=ControlSpec(240, 8, 1.0, 2.0, 300.0),
        acceptance=AcceptanceSpec.development(), asset_root=args.assets, camera_rig='d405',
        episode=EpisodeScene.nominal(), physics_device='cuda:0', diagnostic_observe_only=True)
    def transform(pos, quat_wxyz):
        t = np.eye(4)
        t[:3, :3] = Rotation.from_quat(np.roll(quat_wxyz, -1)).as_matrix()
        t[:3, 3] = pos
        return t
    camera_cfg = cfg.scene.d405_base_0
    nominal_camera = transform(camera_cfg.offset.pos, camera_cfg.offset.rot)
    if 'camera_nominal_T_base_optical' in plan:
        nominal_camera = np.asarray(plan['camera_nominal_T_base_optical'], dtype=float)
    if 'camera_calibration' in plan:
        cfg.d405_rig['device_calibrations']['base_0'] = plan['camera_calibration']
    delta = np.eye(4)
    delta[:3, :3] = Rotation.from_euler('xyz', plan.get('camera_delta_rpy_deg', [0, 0, 0]), degrees=True).as_matrix()
    delta[:3, 3] = plan.get('camera_delta_xyz_m', [0, 0, 0])
    true_camera = delta @ nominal_camera
    true_camera[:3, 3] = nominal_camera[:3, 3] + delta[:3, 3]
    camera_cfg.offset.pos = tuple(true_camera[:3, 3])
    camera_cfg.offset.rot = tuple(np.roll(Rotation.from_matrix(true_camera[:3, :3]).as_quat(), 1))
    env = CoffeeMakeEnv(cfg=cfg)
    env.reset()
    robot = env.scene['robot']
    jids, actual = robot.find_joints(names, preserve_order=True)
    assert actual == names, actual
    sensor = env.scene['d405_base_0']
    calibration = cfg.d405_rig['device_calibrations']['base_0']
    shutil.copyfile(urdf, args.output / 'robot.urdf')
    dataset = {'schema': 'joint-head-isaac-v1', 'urdf_path': 'robot.urdf', 'joint_names': names,
        'camera_nominal_T_base_optical': nominal_camera.tolist(), 'intrinsics': calibration['intrinsics'],
        'markers': plan.get('markers', []), 'frames': [], 'source': 'Isaac5.1 RTX + PhysX AC-One',
        'geometry_equivalence': 'not_identical_to_x5', 'calibration': calibration,
        'asset_root': str(args.assets), 'model_bundle': 'FK-only URDF; rendered meshes remain at asset_root',
        'runner_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    actual_markers = marker_mounts(plan)
    truth = {'camera_T_base_optical': true_camera.tolist(), 'joint_offsets_rad': dict(zip(names, offsets.tolist())),
             'markers': actual_markers, 'frames': []}
    stage = sim_utils.get_current_stage()
    if dataset['markers']:
        create_markers(stage, actual_markers)
        from pxr import UsdGeom
        marker_debug = []
        for m in dataset['markers']:
            path = f"/World/envs/env_0/Robot/{m['link']}/CalibrationMarker{m['id']}"
            prim = stage.GetPrimAtPath(path + '/cell_0_0')
            marker_debug.append({'id': m['id'], 'world': str(UsdGeom.XformCache().GetLocalToWorldTransform(prim)),
                'visibility': str(UsdGeom.Imageable(prim).ComputeVisibility()),
                'purpose': str(UsdGeom.Imageable(prim).ComputePurpose())})
        (args.output / 'marker-debug.json').write_text(json.dumps(marker_debug, indent=2))
    for _ in range(8):
        env.sim.render()

    base_id = robot.find_bodies(['base_link'], preserve_order=True)[0][0]
    def poses():
        return {name: transform(robot.data.body_pos_w[0, i].tolist(), robot.data.body_quat_w[0, i].tolist()).tolist()
                for i, name in enumerate(robot.body_names)}
    (args.output / 'initial-state.json').write_text(json.dumps({'bodies_world': poses(),
        'joint_names': robot.joint_names, 'joint_pos': robot.data.joint_pos[0].tolist(),
        'camera_world': transform(sensor.data.pos_w[0].tolist(), sensor.data.quat_w_ros[0].tolist()).tolist()}, indent=2))

    absolute_step = env._m2_runtime.absolute_step
    preview_files = []
    def capture(frame_id, split, target=None, motion=None):
        env._m2_runtime._render()
        sensor.update(0.0, force_recompute=True)
        raw = sensor.data.output['rgb'][0, ..., :3].cpu().numpy().copy()
        ideal = remap_rgb(raw, sampling_map(calibration, distorted=False))
        Image.fromarray(ideal).save(args.output / f'{frame_id}-pinhole.png')
        Image.fromarray(color_observation(env, 'base_0')[0].cpu().numpy()).save(args.output / f'{frame_id}-color.png')
        actual_q = robot.data.joint_pos[0, jids].cpu().numpy().astype(float)
        velocity = robot.data.joint_vel[0, jids].cpu().numpy().astype(float)
        frame = {'id': frame_id, 'split': split, 'image': f'{frame_id}-pinhole.png',
                 'q_reported_rad': (actual_q - offsets).tolist(), 'velocity_rad_s': velocity.tolist(),
                 'physics_step': absolute_step, 'sim_time_s': absolute_step / 240}
        feedback = dict(zip(robot.joint_names, robot.data.joint_pos[0].cpu().numpy().astype(float).tolist()))
        feedback.update(zip(names, (actual_q - offsets).tolist()))
        frame['q_all_reported'] = feedback
        if target is not None:
            frame.update(target_reported_rad=target.tolist(), motion=motion,
                         tracking_error_rad=float(abs(actual_q - offsets - target).max()))
        dataset['frames'].append(frame)
        truth['frames'].append({'id': frame_id, 'q_true': actual_q.tolist(), 'bodies_world': poses()})
        (args.output / 'dataset.json').write_text(json.dumps(dataset, indent=2))
        (args.output / 'truth.json').write_text(json.dumps(truth, indent=2))
        print(json.dumps({'captured': frame_id, 'velocity_max': float(abs(velocity).max()),
                          'tracking_error_rad': frame.get('tracking_error_rad')}), flush=True)

    capture('initial', 'fit')
    stations = plan.get('stations', [
        {'id': 'left-j6', 'q_deg': [0, 0, 0, 0, 0, 5, 0, 0, 0, 0, 0, 0]},
        {'id': 'right-j6', 'q_deg': [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5]}])
    expanded = []
    previous = (robot.data.joint_pos[0, jids].cpu().numpy() - offsets).copy()
    for station in stations:
        goal = np.deg2rad(station['q_deg'])
        # Move one arm at a time, including the initial physical-zero to
        # biased-feedback transition; intermediate stations are diagnostic.
        changed = [np.max(abs(goal[i:i+6] - previous[i:i+6])) > 1e-5 for i in (0, 6)]
        if all(changed):
            intermediate = previous.copy()
            intermediate[:6] = goal[:6]
            expanded.append({'id': station['id'] + '-left-transition', 'q_deg': np.rad2deg(intermediate).tolist(), 'split': station.get('split', 'fit')})
        expanded.append(station)
        previous = goal
    for station in expanded:
        q_goal = np.deg2rad(station['q_deg'])
        if q_goal.shape != (12,) or not np.isfinite(q_goal).all():
            raise ValueError('Invalid station target')
        physical_goal = q_goal + offsets
        limits = robot.data.soft_joint_pos_limits[0, jids].cpu().numpy()
        # Reported rad -> deg -> rad can put an unchanged zero joint ~1e-18
        # beyond its exact lower bound. Clamp round-off only, never real motion.
        invalid = (physical_goal < limits[:, 0] - 1e-12) | (physical_goal > limits[:, 1] + 1e-12)
        if invalid.any():
            details = [{'joint': names[i], 'target': float(physical_goal[i]),
                        'lower': float(limits[i, 0]), 'upper': float(limits[i, 1])}
                       for i in np.flatnonzero(invalid)]
            raise ValueError(f'Station exceeds physical joint limits: {station["id"]}: {details}')
        physical_goal = np.clip(physical_goal, limits[:, 0], limits[:, 1])
        q_start = robot.data.joint_pos[0, jids].cpu().numpy().copy()
        seconds = max(1.0, float(abs(physical_goal - q_start).max()) / np.deg2rad(8) * 1.6)
        move_steps = int(seconds * 240)
        contact_count = 0
        target = robot.data.joint_pos.clone()
        velocity_samples = []
        settled_window = []
        for tick in range(move_steps + 480):
            fraction = min(1.0, (tick + 1) / move_steps)
            fraction = fraction * fraction * (3 - 2 * fraction)
            q = q_start + fraction * (physical_goal - q_start)
            target[:, jids] = torch.tensor(q, device=env.device, dtype=target.dtype)
            robot.set_joint_position_target(target)
            env.scene.write_data_to_sim()
            absolute_step += 1
            env._m2_contact_reader.begin_step(absolute_step)
            env.sim.step(render=False)
            contacts = env._m2_contact_reader.read(absolute_step)
            contact_count += len(contacts)
            env.scene.update(1 / 240)
            if tick >= move_steps + 456:
                measured = robot.data.joint_pos[0, jids].cpu().numpy()
                speed = float(robot.data.joint_vel[0, jids].abs().max())
                settled_window.append(speed < .01 and float(abs(measured - physical_goal).max()) < .01)
            if tick % 120 == 0:
                velocity_samples.append(float(robot.data.joint_vel[0, jids].abs().max()))
            if plan.get('save_preview', True) and tick % 120 == 0:
                env._m2_runtime._render()
                sensor.update(0., force_recompute=True)
                raw = sensor.data.output['rgb'][0, ..., :3].cpu().numpy().copy()
                image = Image.fromarray(remap_rgb(raw, sampling_map(calibration, distorted=False)))
                filename = f'preview-{len(preview_files):04d}.jpg'
                image.save(args.output / filename, quality=85)
                preview_files.append(filename)
        if not all(settled_window) or len(settled_window) != 24:
            raise RuntimeError(f"Station did not settle: {station['id']}")
        capture(station['id'], station.get('split', 'fit'), q_goal,
                {'steps': move_steps + 480, 'contact_record_count': contact_count,
                 'max_sampled_velocity_rad_s': max(velocity_samples),
                 'collision_clearance_certified': False})
    (args.output / 'preview.json').write_text(json.dumps({'frames': preview_files, 'fps': 2}))
    if preview_files:
        images = [Image.open(args.output / p).resize((640, 480)) for p in preview_files]
        images[0].save(args.output / 'trajectory.gif', save_all=True, append_images=images[1:], duration=500, loop=0)
    (args.output / 'summary.json').write_text(json.dumps({'frames': len(dataset['frames']),
        'max_tracking_error_rad': max(f.get('tracking_error_rad', 0) for f in dataset['frames']),
        'motion': 'servo targets executed by PhysX; no state teleport',
        'collision_clearance_certified': False, 'simulation_only': True}, indent=2))
    env.close()


def create_markers(stage, markers):
    """Opaque finite printed plates rigidly mounted in link coordinates."""
    import cv2
    import numpy as np
    from pxr import Gf, UsdGeom, UsdShade, Sdf
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    materials = []
    for label, value in [('black', .005), ('white', .9)]:
        path = '/World/CalibrationMaterials/' + label
        mat = UsdShade.Material.Define(stage, path)
        shader = UsdShade.Shader.Define(stage, path + '/Shader')
        shader.CreateIdAttr('UsdPreviewSurface')
        shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(value))
        shader.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(.8)
        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
        materials.append(mat)
    for marker in markers:
        path = f"/World/envs/env_0/Robot/{marker['link']}/CalibrationMarker{marker['id']}"
        parent = stage.GetPrimAtPath(str(Path(path).parent))
        if not parent.IsValid():
            raise ValueError(f'Missing marker parent {parent}')
        board = UsdGeom.Xform.Define(stage, path)
        matrix = np.asarray(marker['T_link_marker'])
        board.AddTransformOp().Set(Gf.Matrix4d(*matrix.T.ravel().tolist()))
        side = float(marker['size_m'])
        if marker.get('support_plate_thickness_m') is not None:
            # Explicit rigid backing; the printed face points along marker -Z.
            # Visual mounting proxy only: no fastener strength/collision certification.
            thickness = float(marker['support_plate_thickness_m'])
            if not 0 < thickness <= .005:
                raise ValueError('invalid support plate thickness')
            plate = UsdGeom.Cube.Define(stage, path + '/SupportPlate')
            plate.CreateSizeAttr(1.)
            plate.AddTranslateOp().Set(Gf.Vec3d(0, 0, thickness/2 + .0001))
            plate.AddScaleOp().Set(Gf.Vec3d(side*8/6, side*8/6, thickness))
            UsdShade.MaterialBindingAPI.Apply(plate.GetPrim()).Bind(materials[1])
            if marker.get('support_saddle_proxy', False):
                saddle = UsdGeom.Cube.Define(stage, path + '/SupportSaddleProxy')
                saddle.CreateSizeAttr(1.)
                saddle.AddTranslateOp().Set(Gf.Vec3d(0, 0, thickness + .0001 + .0029/2))
                saddle.AddScaleOp().Set(Gf.Vec3d(.015, .010, .0029))
                UsdShade.MaterialBindingAPI.Apply(saddle.GetPrim()).Bind(materials[1])
        pattern = cv2.aruco.generateImageMarker(dictionary, marker['id'], 6)
        # 8x8 cells includes the white quiet zone. Separate opaque quads avoid
        # texture filtering changing physical target dimensions.
        pattern = np.pad(pattern, 1, constant_values=255)
        for y in range(8):
            for x in range(8):
                cell = side / 6
                x0, y0 = (x - 4) * cell, (y - 4) * cell
                mesh = UsdGeom.Mesh.Define(stage, path + f'/cell_{x}_{y}')
                mesh.CreatePointsAttr([(x0,y0,0),(x0+cell,y0,0),(x0+cell,y0+cell,0),(x0,y0+cell,0)])
                mesh.CreateFaceVertexCountsAttr([4])
                mesh.CreateFaceVertexIndicesAttr([3,2,1,0])
                mesh.CreateDoubleSidedAttr(True)
                mesh.CreateSubdivisionSchemeAttr('none')
                UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(materials[int(pattern[y,x] > 0)])


if __name__ == '__main__':
    main()
