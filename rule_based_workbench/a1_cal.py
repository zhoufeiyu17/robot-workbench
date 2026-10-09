"""Build a sealed native A1-cal from calibration-store active files, offline only.

Joint offsets stay in profile.json (model-space URDF); optical children alone move.
A fixture build is permanently labelled simulation-only and never authorizes motion.
"""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import shutil
import tempfile
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
from integration import _digest
from joint_space import offsets
from jaw_mapping_v2 import MAPPING_ID
from model_adapt import digest, seal, verify
from physical_profile import profile_from_calibration, rigid


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), ensure_ascii=False, allow_nan=False).encode()


def read_active(active, geometry_id, *, fixture=False, asset_seal=None, environment='physical'):
    active = Path(active)
    if environment not in ('physical','fake') or fixture and environment!='physical':
        raise ValueError('explicit physical/fake environment; fixture is a separate legacy test mode')
    if active.is_symlink() or any((active/(name+'.json')).is_symlink() for name in ('head','wrist','manifest')):
        raise ValueError('installed calibration must use regular files')
    manifest_bytes=(active/'manifest.json').read_bytes()
    records = {name: json.loads((active/(name+'.json')).read_bytes()) for name in ('head', 'wrist')}
    if (active/'manifest.json').read_bytes()!=manifest_bytes:
        raise ValueError('installed calibration changed during group read')
    records['manifest']=json.loads(manifest_bytes)
    manifest = records['manifest']
    if manifest.get('schema') != 'joint-head-calibration-active-v1':
        raise ValueError('installed calibration schema required')
    expected = ('hardware','simulated',True) if environment=='fake' else ('simulation', 'simulated', True) if fixture else ('hardware', 'physical', False)
    if tuple(manifest.get(k) for k in ('environment', 'robot_source', 'simulation_only')) != expected:
        raise ValueError('calibration provenance differs from requested physical/fixture mode')
    for kind in ('head', 'wrist'):
        record = deepcopy(records[kind])
        sha = record.pop('sha256')
        if hashlib.sha256(canonical(record)).hexdigest() != sha:
            raise ValueError(kind+' calibration content digest mismatch')
        if (record.get('schema') != 'joint-head-calibration-store-v1' or record.get('kind') != kind
                or record.get('robot_id') != manifest.get('robot_id') or record.get('environment') != expected[0]
                or any(records[kind].get(k) != manifest[kind].get(k) for k in ('id', 'version', 'sha256'))):
            raise ValueError(kind+' calibration identity mismatch')
        row = record['payload']['row']
        if (row.get('robot_source'), row.get('simulation_only')) != expected[1:]:
            raise ValueError(kind+' calibration source mismatch')
    if records['wrist'].get('dependency_id') != records['head']['id']:
        raise ValueError('wrist calibration must depend on this head result')
    if environment=='fake' and manifest['wrist'].get('dependency_id')!=records['head']['id']:
        raise ValueError('fake active manifest wrist dependency differs from this head')
    # Workbench geometry_id also hashes configuration/support policy; it is not
    # the asset ID. Both head and wrist must independently pin the same A1 asset.
    head = records['head']['payload']['row']['parameters']
    wrist = records['wrist']['payload']
    identities = (head, wrist['row'], wrist['contract'])
    if (any(row.get('asset_geometry_id') != geometry_id for row in identities)
            or any(row.get('asset_seal_sha256') != asset_seal for row in identities)
            or wrist['contract'].get('geometry_id') != manifest.get('geometry_id')
            or not manifest.get('geometry_id')):
        raise ValueError('installed calibration must match sealed native A1 geometry')
    return records


def build(a1, active, head_camera, template, registration, output, *, fixture=False, environment='physical'):
    a1, active, output = map(Path, (a1, active, output))
    verify(a1)
    contract = json.loads((a1/'calibration-input.json').read_text())
    files = contract['geometry_files']
    if any(not (a1/name).resolve().is_relative_to(a1.resolve()) for name in files):
        raise ValueError('A1 geometry file escapes bundle')
    if (hashlib.sha256(canonical(files)).hexdigest() != contract['geometry_id']
            or any(digest(a1/name) != sha for name, sha in files.items())):
        raise ValueError('A1 geometry identity mismatch')
    records = read_active(active, contract['geometry_id'], fixture=fixture, asset_seal=digest(a1/'SHA256SUMS.json'),environment=environment)
    simulated=fixture or environment=='fake'
    head, wrist, manifest = (records[k] for k in ('head', 'wrist', 'manifest'))
    parameters = head['payload']['row']['parameters']
    if any(len(parameters['joint_offsets_rad'][side]) != 6 for side in ('left', 'right')):
        raise ValueError('six joint offsets per arm required')
    joint_offsets = [v for side in ('left', 'right') for v in parameters['joint_offsets_rad'][side]]
    offsets({'joints': {'zero_offsets_rad': joint_offsets, 'signs': contract['joint_signs']}})
    cameras = {'head': rigid(parameters['camera_T_world_optical']),
               **{side: rigid(wrist['payload']['row'][side]['T_link6_camera']) for side in ('left', 'right')}}
    intrinsics = {'head': head_camera, **wrist['payload']['contract']['cameras']}
    for camera in intrinsics.values():
        for field in ('width', 'height', 'fx', 'fy'):
            if not np.isfinite(camera[field]) or camera[field] <= 0:
                raise ValueError('positive camera dimensions/focal lengths required')
        if not np.isfinite([camera['cx'], camera['cy'], *camera['coefficients']]).all():
            raise ValueError('finite camera intrinsics required')
    calibration = dict(schema='calibrated_robot_geometry.v1', source_geometry_id=contract['geometry_id'],
        zero_offsets_rad=joint_offsets, joint_order=contract['joint_order'], length_unit='m', angle_unit='rad',
        reported_to_model='q_model = q_reported + zero_offsets_rad',
        model_to_reported='q_reported = q_model - zero_offsets_rad',
        T_world_head_optical=cameras['head'].tolist(),
        T_link6_wrist_optical={s: cameras[s].tolist() for s in ('left', 'right')},
        joint_zero_offsets_baked_into_urdf=False, head_id=head['id'], wrist_id=wrist['id'],
        acceptance=manifest['acceptance'], simulation_only=simulated, runtime_allowed=False)
    calibration.update(calibration_geometry_id=manifest['geometry_id'],
                       head_acceptance=deepcopy(parameters.get('acceptance', {})))
    template = deepcopy(template)
    template['gripper'].update(mapping=MAPPING_ID, max_opening_mm=83.)
    profile = profile_from_calibration(calibration, intrinsics, manifest, template, registration,
                                       environment='fake' if simulated else 'physical')
    if fixture:profile['source']['kind']='fixture_calibration'
    profile['model_binding'] = dict(a1_sha256=digest(a1/'SHA256SUMS.json'),
        geometry_id=contract['geometry_id'], jaw_mapping_sha256=digest(a1/'jaw-mapping.json'))
    tree = ET.parse(a1/'robot.urdf')
    joints = {j.get('name'): j for j in tree.getroot().findall('joint')}
    for camera, prefix in (('head', 'top'), ('left', 'left_wrist'), ('right', 'right_wrist')):
        body = joints[prefix+'_camera_link_joint']
        if body.find('parent').get('link') != contract['camera_parents'][camera]:
            raise ValueError('camera parent differs from A1 calibration contract')
        o = body.find('origin')
        T = np.eye(4)
        if o is not None:
            T[:3, 3] = np.fromstring(o.get('xyz', '0 0 0'), sep=' ')
            T[:3, :3] = Rotation.from_euler('xyz', np.fromstring(o.get('rpy', '0 0 0'), sep=' ')).as_matrix()
        local = np.linalg.inv(T) @ cameras[camera]
        optical = joints[prefix+'_camera_rgb_optical_frame_joint']
        old = optical.find('origin')
        if old is not None: optical.remove(old)
        ET.SubElement(optical, 'origin', xyz=' '.join(map(str, local[:3, 3])),
                      rpy=' '.join(map(str, Rotation.from_matrix(local[:3, :3]).as_euler('xyz'))))
    if output.exists(): raise FileExistsError(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.a1-cal-', dir=output.parent) as temp:
        stage = Path(temp)/'bundle'
        shutil.copytree(a1, stage)
        (stage/'SHA256SUMS.json').unlink()
        ET.indent(tree); tree.write(stage/'robot.urdf', encoding='utf-8', xml_declaration=True)
        profile['model_binding']['urdf_sha256'] = digest(stage/'robot.urdf')
        profile.pop('profile_id')
        profile['profile_id'] = ('fake-' if environment=='fake' else '')+'a1-cal-'+_digest(profile)[:16]
        for name, value in (('profile', profile), ('adaptation', calibration)):
            (stage/(name+'.json')).write_bytes(canonical(value)+b'\n')
        (stage/'calibration_sources').mkdir()
        for name, record in records.items():
            (stage/'calibration_sources'/(name+'.json')).write_bytes(canonical(record)+b'\n')
        (stage/'head-camera.json').write_bytes(canonical(head_camera)+b'\n')
        model = json.loads((stage/'manifest.json').read_text())
        model.update(id=output.name, schema='robot_model_calibrated.v1', calibration_status='fixture' if fixture else 'candidate',
                     runtime_allowed=False, simulation_only=simulated, runtime_environment='fake' if simulated else 'physical',
                     joint_zero_offsets_baked_into_urdf=False,urdf_sha256=digest(stage/'robot.urdf'),
                     a1_sha256=digest(a1/'SHA256SUMS.json'), profile_digest=_digest(profile))
        (stage/'manifest.json').write_bytes(canonical(model)+b'\n')
        seal(stage); verify(stage); verify(a1)
        stage.rename(output)
    return {'output': str(output), 'seal': digest(output/'SHA256SUMS.json'), 'profile_id': profile['profile_id'],
            'runtime_allowed': False, 'simulation_only': simulated}


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('a1', 'active', 'head-camera', 'template', 'registration', 'output'):
        p.add_argument('--'+name, type=Path, required=True)
    p.add_argument('--fixture', action='store_true')
    p.add_argument('--environment',choices=('physical','fake'),default='physical')
    args = p.parse_args()
    load = lambda path: json.loads(path.read_text())
    print(json.dumps(build(args.a1, args.active, load(args.head_camera), load(args.template),
                           load(args.registration), args.output, fixture=args.fixture,environment=args.environment), indent=2))
