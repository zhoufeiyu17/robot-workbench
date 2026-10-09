"""Prepare nominal A1 workbench inputs from a sealed asset; no robot or service IO."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from arx_env.joint_head_isaac_fit import Model


def prepare(assets, seal, previous, output):
    assets=Path(assets).resolve();output=Path(output).resolve()
    checksums=assets/'SHA256SUMS.json'
    if hashlib.sha256(checksums.read_bytes()).hexdigest()!=seal:
        raise ValueError('A1 asset seal differs from reviewed version')
    for name,digest in json.loads(checksums.read_text()).items():
        path=(assets/name).resolve()
        if not path.is_relative_to(assets) or not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest()!=digest:
            raise ValueError('A1 sealed asset mismatch: '+name)
    contract=json.loads((assets/'calibration-input.json').read_text())
    model=Model(assets/'robot.urdf',contract['joint_order'])
    poses=model.frames(np.zeros(12))
    camera=poses['top_camera_rgb_optical_frame']
    mounts={};wrist={}
    marker=np.diag([1.,-1.,-1.,1.]);marker[:3,3]=contract['markers']['nominal_body_from_marker_xyz_m']
    for side,terminal,ident in [('left','left_link6','24'),('right','right_link16','30')]:
        mounts[ident]=(np.linalg.inv(poses[terminal])@poses[side+'_wrist_camera_link']@marker).tolist()
        wrist[side]=(np.linalg.inv(poses[terminal])@poses[side+'_wrist_camera_rgb_optical_frame']).tolist()
    config=json.loads(Path(previous).read_text())
    for key in ('wrist_support_proxy','wrist_camera_housings','chassis_mesh','chassis_scale','chassis_T_world_mesh','static_boxes',
                'simulation_joint_offsets_rad','simulation_marker_mounts','simulation_head_calibration','wrist_simulation_mounts'):
        config.pop(key,None)
    config.update(model_family='ac_one',asset_geometry_id=contract['geometry_id'],asset_seal_sha256=seal,
                  urdf=str(assets/'robot.urdf'),mesh_manifest=str(assets/'manifest.json'),
                  jaw_mapping_file=str(assets/'jaw-mapping.json'),wrist_nominal_mounts=wrist,
                  gap_m={'left':.080,'right':.080},initial_q=[0.]*12,marker_size_m=.015)
    support=json.loads((Path(__file__).resolve().parents[3]/'configs/ac-one-a1-parking-support.json').read_text())
    if support['asset_geometry_id']!=contract['geometry_id']:
        raise ValueError('new A1 geometry requires review of the parking support contact')
    config['parking_support_contact']=support
    output.mkdir(parents=True,exist_ok=False)
    payloads={'scene_seed':{'T_world_arm_base':{s:np.eye(4).tolist() for s in ('left','right')},'source':'native A1 full robot tree'},
              'candidate':{'T_world_optical':camera.tolist(),'T_link6_marker':mounts,'source':'A1 nominal; uncalibrated'},
              'camera_seed':{'T_world_camera':camera.tolist(),'source':'A1 nominal; uncalibrated'}}
    for key,value in payloads.items():
        target=output/(key+'.json');target.write_text(json.dumps(value,indent=2));config[key]=str(target)
    (output/'config.json').write_text(json.dumps(config,indent=2))
    return config


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--assets',required=True);p.add_argument('--seal',required=True)
    p.add_argument('--previous-config',required=True);p.add_argument('--output',required=True)
    a=p.parse_args();prepare(a.assets,a.seal,a.previous_config,a.output)
