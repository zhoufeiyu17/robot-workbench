"""Offline bridge from installed calibration to collection and camera configuration.

The caller supplies environment and sensor-parent frame registrations explicitly.
This module never connects a device, authorizes motion, or moves camera housings.
"""
from copy import deepcopy
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
from integration import _digest, validate_profile

CAMERAS = {'head': 'base_0', 'left': 'left_wrist_0', 'right': 'right_wrist_0'}


def rigid(value):
    matrix = np.asarray(value, float)
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
            or not np.allclose(matrix[3], [0,0,0,1], atol=1e-7)
            or not np.allclose(matrix[:3,:3].T @ matrix[:3,:3], np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(matrix[:3,:3]), 1, atol=1e-6)):
        raise ValueError('finite rigid transform required')
    return matrix


def pose(matrix):
    matrix = rigid(matrix)
    q = Rotation.from_matrix(matrix[:3, :3]).as_quat()
    return {'position': matrix[:3, 3].tolist(), 'orientation_wxyz': q[[3, 0, 1, 2]].tolist()}


def build_profile(snapshot, template, environment_from_calibration_world):
    """Preserve explicit gripper/tolerance policy; do not invent physical thresholds."""
    from calibrated_mesh import CalibratedMesh, load_snapshot
    validate_profile(template)
    config, head, wrist, active = load_snapshot(snapshot)
    model = CalibratedMesh(snapshot)
    intrinsics = {'head': json.loads(Path(config['camera']).read_text()),
                  **wrist['payload']['contract']['cameras']}
    return profile_from_calibration(model.calibration(), intrinsics, active, template,
                                    environment_from_calibration_world)


def profile_from_calibration(calibration, intrinsics, active, template, environment_from_calibration_world, *, environment='physical'):
    """Common profile boundary for legacy snapshots and sealed native A1-cal."""
    validate_profile(template)
    if environment not in ('physical','fake'):
        raise ValueError('explicit physical/fake profile environment required')
    simulated=environment=='fake'
    if (active.get('robot_source') != ('simulated' if simulated else 'physical')
            or active.get('simulation_only') is not simulated
            or not simulated and calibration.get('simulation_only') is True):
        raise ValueError('installed calibration differs from trusted profile environment')
    registration = rigid(environment_from_calibration_world)
    camera = registration @ rigid(calibration['T_world_head_optical'])
    profile = deepcopy(template)
    profile['joints'] = {'zero_offsets_rad': calibration['zero_offsets_rad'], 'signs': [1]*12}
    profile['camera'] = {'xyz_m': camera[:3, 3].tolist(),
                         'rpy_rad': Rotation.from_matrix(camera[:3, :3]).as_euler('xyz').tolist()}
    profile['frames']['environment_from_camera'] = pose(camera)
    profile['physical_calibration'] = {
        **deepcopy(calibration), 'robot_id': active['robot_id'],
        'T_environment_calibration_world': registration.tolist(),
        'intrinsics': {k: {field: v[field] for field in
                          ('serial','width','height','fx','fy','cx','cy','coefficients','distortion_model','rectified')}
                       for k, v in intrinsics.items()}}
    serials = [v['serial'] for v in profile['physical_calibration']['intrinsics'].values()]
    if len(set(serials)) != 3 or any(not s for s in serials):
        raise ValueError('three distinct camera serials required')
    profile['source'] = {'kind': 'installed_fake_calibration' if simulated else 'installed_physical_calibration',
                         'simulation_only':simulated,'runtime_environment':environment,
                         'robot_source':'simulated' if simulated else 'physical','head_id': active['head']['id'],
                         'wrist_id': active['wrist']['id'], 'acceptance': deepcopy(active['acceptance'])}
    profile.pop('profile_id', None)
    profile['profile_id'] = environment+'-' + _digest(profile)[:16]
    validate_profile(profile)
    return profile


def camera_offsets(profile, parents):
    """Parent registration maps world (head) or link6 (wrists) into USD parent.

    Explicit parent names and transforms prevent silently assuming that the
    calibration world equals the simulation world or that a USD link is link6.
    """
    calibration = profile['physical_calibration']
    if set(parents) != set(CAMERAS):
        raise ValueError('head/left/right sensor parent registrations required')
    result = {}
    for name, device in CAMERAS.items():
        binding = parents[name]
        if set(binding) != {'parent', 'T_parent_reference'} or not isinstance(binding['parent'], str) or not binding['parent']:
            raise ValueError('explicit parent name and reference transform required')
        local = calibration['T_world_head_optical'] if name == 'head' else calibration['T_link6_wrist_optical'][name]
        result[device] = {'parent': binding['parent'], **pose(rigid(binding['T_parent_reference']) @ rigid(local))}
    return result


def configure_cameras(cfg, profile, parents):
    """Apply optical poses only, before Isaac environment construction."""
    offsets = camera_offsets(profile, parents)
    rig = deepcopy(cfg.d405_rig)
    cameras = {c['device_calibration_ref']: c for c in rig['cameras']}
    if set(cameras) != set(offsets):
        raise ValueError('renderer must expose exactly the three calibrated cameras')
    pending = []
    for name, device in CAMERAS.items():
        offset = offsets[device]
        sensor = getattr(cfg.scene, 'd405_' + device)
        if cameras[device]['parent'] != offset['parent'] or sensor.prim_path != f"{{ENV_REGEX_NS}}/Robot/{offset['parent']}/D405RGB_{device}":
            raise ValueError('renderer sensor parent differs from explicit registration')
        k = profile['physical_calibration']['intrinsics'][name]
        if k['distortion_model'] not in ('inverse_brown_conrady', 'distortion.inverse_brown_conrady'):
            raise ValueError('D405 renderer requires inverse Brown Conrady calibration')
        rig['device_calibrations'][device] = {
            'serial': k['serial'], 'width': k['width'], 'height': k['height'], 'collection_fps': 30,
            'intrinsics': {key: k[key] for key in ('fx','fy','cx','cy')},
            'distortion': {'model': 'inverse_brown_conrady', 'coefficient_order': ['k1','k2','p1','p2','k3'],
                           'coefficients': k['coefficients']}}
        pending.append((sensor, offset))
    for sensor, offset in pending:
        sensor.offset.pos = tuple(offset['position'])
        sensor.offset.rot = tuple(offset['orientation_wxyz'])
        sensor.offset.convention = 'ros'
    cfg.d405_rig = rig
    return offsets
