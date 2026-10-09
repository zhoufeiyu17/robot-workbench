"""Freeze A1 from baseline plus pinned installed calibration; no hardware IO."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import numpy as np


def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def seal(root):
    root=Path(root)
    files={str(p.relative_to(root)):digest(p) for p in sorted(root.rglob('*')) if p.is_file() and p.name!='SHA256SUMS.json'}
    target=root/'SHA256SUMS.json'
    with target.open('x') as f:json.dump(files,f,indent=2);f.write('\n')
    return digest(target)


def verify(root):
    root=Path(root);entries=json.loads((root/'SHA256SUMS.json').read_text())
    actual={str(p.relative_to(root)) for p in root.rglob('*') if p.is_file() and p.name!='SHA256SUMS.json'}
    if set(entries)!=actual:raise ValueError('frozen model file set changed')
    for name,sha in entries.items():
        path=(root/name).resolve()
        if not path.is_relative_to(root.resolve()) or digest(path)!=sha:raise ValueError('frozen model digest mismatch: '+name)


def build_pending(baseline,mapping,output):
    """AC-One A1 geometry, deliberately without any old X5 calibration values."""
    baseline,output=map(Path,(baseline,output));verify(baseline)
    shutil.copytree(baseline,output)
    (output/'SHA256SUMS.json').unlink()  # New derivative needs its own identity.
    tree=ET.parse(output/'robot.urdf');root=tree.getroot();root.set('name',output.name)
    removed=[]
    for link in root.findall('link'):
        for e in list(link):
            if e.tag in ('visual','collision') and e.get('name') in ('left_handle','right_handle'):
                removed.append(e.get('name'));link.remove(e)
    if sorted(removed)!=['left_handle','left_handle','right_handle','right_handle']:
        raise ValueError('expected two baseline handle visual/collision pairs')
    ET.indent(root);tree.write(output/'robot.urdf',encoding='utf-8',xml_declaration=True)
    manifest=json.loads((baseline/'manifest.json').read_text())
    manifest.update(schema='robot_model_adapt.v1',id=output.name,
        baseline_sha256=digest(baseline/'SHA256SUMS.json'),urdf_sha256=digest(output/'robot.urdf'),
        handles='removed; original source retained',calibration_status='required_head_then_wrists',
        old_x5_calibration_applied=False,deployment_status='offline_not_deployed',runtime_allowed=False)
    manifest['parts']=[p for p in manifest['parts'] if p['name'] not in ('left_handle','right_handle')]
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    shutil.copy2(mapping,output/'jaw-mapping.json')
    geometry_files={'robot.urdf':digest(output/'robot.urdf'),**{p['path']:digest(output/p['path']) for p in manifest['parts']}}
    geometry_id=hashlib.sha256(json.dumps(geometry_files,sort_keys=True,separators=(',',':')).encode()).hexdigest()
    contract=dict(schema='robot_calibration_input.v1',geometry_id=geometry_id,
        geometry_id_basis='SHA256 of sorted compact JSON of URDF and active mesh path/digest pairs',geometry_files=geometry_files,
        baseline_sha256=digest(baseline/'SHA256SUMS.json'),model='AC-One native kinematics',
        joint_order=[f'{s}_joint{i+(10 if s=="right" else 0)}' for s in ('left','right') for i in range(1,7)],
        units=dict(length='m',joint_angle='rad',public_jaw_gap='mm'),
        sdk_order='left6,left_gripper,right6,right_gripper',
        joint_signs=[1]*12,joint_signs_basis='existing AC-One/SDK convention; verify motion direction before calibration',
        zero_offsets_rad=None,reported_to_model='q_model = q_reported + fitted_offset; offset not yet fitted',
        camera_parents=dict(head='base_link',left='left_link6',right='right_link16'),
        camera_poses='nominal optical frames in URDF; replace only after new AC-One calibration',
        markers=dict(dictionary='DICT_4X4_50',left_id=24,right_id=30,black_size_mm=15,paper_size_mm=20,
            recommended_surface='rigid wrist housing top face',nominal_body_from_marker_xyz_m=[0,0,.0225],
            actual_mounts='to be fitted; demonstration positions are not measured extrinsics'),
        gripper=manifest['gripper'],sdk_gripper_mapping='jaw-mapping.json; reinterpretation, not physical revalidation',
        required_order=['AC-One geometry adapter offline verification','head and joint offsets','wrist optical extrinsics'],
        hardware_access_authorized=False)
    (output/'calibration-input.json').write_text(json.dumps(contract,indent=2)+'\n')
    identity=seal(output);verify(baseline);verify(output);return identity


def build(baseline,snapshot,mapping,output):
    from calibrated_mesh import CalibratedMesh, load_snapshot
    from calibrated_urdf import set_origin
    from arx_env.urdf_mesh import origin
    baseline,snapshot,output=map(Path,(baseline,snapshot,output));verify(baseline)
    model=CalibratedMesh(snapshot);config,head,wrist,active=load_snapshot(snapshot)
    manifest=json.loads((baseline/'manifest.json').read_text())
    # Baseline must really use the X5 geometry that the fitted offsets describe.
    if digest(baseline/'sources/x5.urdf')!=digest(Path(config['urdf'])):raise ValueError('calibration arm source differs')
    for side in ('left','right'):
        if not np.allclose(manifest['T_world_arm_base'][side],model.geometry.bases[side],atol=1e-9,rtol=0):
            raise ValueError('calibration world registration differs: '+side)
    output.mkdir(parents=True,exist_ok=False)
    for p in baseline.iterdir():
        if p.name=='SHA256SUMS.json':continue
        if p.is_dir():shutil.copytree(p,output/p.name)
        else:shutil.copy2(p,output/p.name)
    tree=ET.parse(output/'robot.urdf');root=tree.getroot();root.set('name','ac_one_x5_A1')
    removed=[]
    for link in root.findall('link'):
        for e in list(link):
            if e.tag in ('visual','collision') and e.get('name') in ('left_handle','right_handle'):
                removed.append(e.get('name'));link.remove(e)
    if sorted(removed)!=['left_handle','left_handle','right_handle','right_handle']:raise ValueError('expected two handle visual/collision pairs')
    joints={j.get('name'):j for j in root.findall('joint')}
    for name,T in [('top',model.head_camera),('left_wrist',model.wrist_cameras['left']),('right_wrist',model.wrist_cameras['right'])]:
        body=origin(joints[name+'_camera_link_joint'].find('origin'))
        # Preserve visual housing. Move only its optical child relative to it.
        set_origin(joints[name+'_camera_rgb_optical_frame_joint'],np.linalg.inv(body)@T)
    ET.indent(root);tree.write(output/'robot.urdf',encoding='utf-8',xml_declaration=True)
    calibration=model.calibration()
    calibration.update(id='ac_one_x5_A1',baseline_sha256=digest(baseline/'SHA256SUMS.json'),
        joint_zero_offsets_baked_into_urdf=False,calibration_world_equals_baseline_base_link=True,
        acceptance_state='user_accepted_provisional',runtime_allowed=False,
        intrinsics={'head':json.loads(Path(config['camera']).read_text()),**wrist['payload']['contract']['cameras']})
    (output/'adaptation.json').write_text(json.dumps(calibration,indent=2)+'\n')
    (output/'calibration_sources').mkdir()
    for name in ('head','wrist','manifest'):shutil.copy2(snapshot/'active'/(name+'.json'),output/'calibration_sources'/(name+'.json'))
    shutil.copy2(mapping,output/'jaw-mapping.json')
    sources=deepcopy(manifest)
    sources.update(id='ac_one_x5_A1',schema='robot_model_adapt.v1',baseline_sha256=digest(baseline/'SHA256SUMS.json'),
                   urdf_sha256=digest(output/'robot.urdf'),handles='removed from visual and collision; source archived',
                   cameras='calibrated optical children; nominal housings unchanged',
                   acceptance=deepcopy(active['acceptance']),deployment_status='offline_not_deployed',runtime_allowed=False)
    sources['parts']=[p for p in sources['parts'] if p['name'] not in ('left_handle','right_handle')]
    # The unused handle meshes remain as provenance; only URDF-active parts render.
    (output/'manifest.json').write_text(json.dumps(sources,indent=2)+'\n')
    identity=seal(output);verify(baseline);verify(output)
    return identity


if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser()
    for name in ('baseline','snapshot','mapping','output'):p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args();print(build(a.baseline,a.snapshot,a.mapping,a.output))
