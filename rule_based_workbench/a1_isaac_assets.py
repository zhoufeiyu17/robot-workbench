"""Build an isolated native asset root for A1-cal; shared source assets stay read-only."""
import argparse
from copy import deepcopy
import json
import os
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
from model_adapt import digest, verify
from physical_profile import rigid

ROBOT = 'robots/arx_ac_one/v1/'


def conversion_device(gpu):
    if type(gpu) is not int or gpu not in (0,1):raise ValueError('explicit physical GPU index 0 or 1 required')
    return dict(headless=True,device='cuda:0',active_gpu=gpu,multi_gpu=False,
        kit_args=f'--/renderer/activeGpu={gpu} --/renderer/multiGpu/enabled=false --/renderer/multiGpu/autoEnable=false --/renderer/gpuEnumeration/glInterop/enabled=false')


def calibrated_rig(rig, profile):
    rig = deepcopy(rig)
    cal = profile['physical_calibration']
    refs = {'base_0': ('head', 'base_link'), 'left_wrist_0': ('left', 'left_link6'),
            'right_wrist_0': ('right', 'right_link16')}
    optical_axes = np.array([[0,0,1],[-1,0,0],[0,-1,0]])
    if {c['device_calibration_ref'] for c in rig['cameras']} != set(refs):
        raise ValueError('three known camera bindings required')
    for camera in rig['cameras']:
        device = camera['device_calibration_ref']; name, parent = refs[device]
        if camera['parent'] != parent: raise ValueError('unexpected camera parent')
        T = rigid(cal['T_world_head_optical'] if name == 'head' else cal['T_link6_wrist_optical'][name])
        body = T[:3,:3] @ optical_axes.T
        camera['rpy'] = Rotation.from_matrix(body).as_euler('xyz').tolist()
        camera['xyz'] = (T[:3,3] - body @ [rig['camera_proxy_size_m'][0]/2,0,0]).tolist()
        k = cal['intrinsics'][name]
        if k['distortion_model'] not in ('inverse_brown_conrady','distortion.inverse_brown_conrady'):
            raise ValueError('unsupported calibrated D405 distortion')
        rig['device_calibrations'][device] = dict(serial=k['serial'], width=k['width'], height=k['height'], collection_fps=30,
            intrinsics={x:k[x] for x in ('fx','fy','cx','cy')},
            distortion={'model':'inverse_brown_conrady','coefficient_order':['k1','k2','p1','p2','k3'],'coefficients':k['coefficients']})
    rig.update(id=profile['profile_id'], status='calibrated_optical_frames',
               optical_origin_note='xyz/rpy are inverse encoding for camera_offset; housing mesh is unchanged',
               calibration_binding=profile['model_binding'])
    return rig


def writable(root, relative):
    """Replace a copied overlay entry; all base symlinks were dereferenced on copy."""
    target = root/relative
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.is_symlink() or target.is_file(): target.unlink()
    elif target.is_dir(): shutil.rmtree(target)
    return target


def prepare(bundle, base, output):
    bundle, base, output = (Path(p).resolve() for p in (bundle,base,output))
    verify(bundle)
    profile = json.loads((bundle/'profile.json').read_text())
    if digest(bundle/'robot.urdf') != profile['model_binding']['urdf_sha256']:
        raise ValueError('A1-cal URDF/profile mismatch')
    if output.is_relative_to(base) or output.is_relative_to(bundle):
        raise ValueError('output must stay outside base assets and calibration bundle')
    # Native asset resolution confines paths to this root; external symlinks are forbidden.
    shutil.copytree(base, output, symlinks=False)
    model = writable(output, ROBOT+'source/acone/urdf/a1-cal.urdf')
    shutil.copyfile(bundle/'robot.urdf',model)
    meshdir = writable(output, ROBOT+'source/acone/urdf/meshes')
    shutil.copytree(bundle/'meshes',meshdir)
    for name in ('acone.urdf','acone_no_handle.urdf','acone_no_handle_d405.urdf'):
        writable(output,ROBOT+'source/acone/urdf/'+name).symlink_to(model.name)
    rig = calibrated_rig(json.loads((base/(ROBOT+'d405_rig.json')).read_text()),profile)
    writable(output,ROBOT+'d405_rig.json').write_text(json.dumps(rig,indent=2)+'\n')
    usd = writable(output,ROBOT+'sim/a1-cal.usd')
    for name in ('ac_one.usd','ac_one_no_handle.usd','ac_one_no_handle_d405.usd'):
        writable(output,ROBOT+'sim/'+name).symlink_to(usd.name)
    binding = dict(schema='a1_isaac_assets.v1',bundle_seal=digest(bundle/'SHA256SUMS.json'),
                   profile_id=profile['profile_id'],model_binding=profile['model_binding'],
                   base_assets=str(base),urdf=str(model),usd=str(usd),conversion_complete=False,
                   physics_validated=False,runtime_allowed=False)
    (output/'a1-binding.json').write_text(json.dumps(binding,indent=2)+'\n')
    return binding


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('bundle','base-assets','output'):p.add_argument('--'+key,type=Path,required=True)
    p.add_argument('--prepare-only',action='store_true')
    p.add_argument('--gpu',type=int,choices=(0,1),default=0,help='physical CUDA mask and Vulkan adapter; logical CUDA device stays 0')
    args=p.parse_args()
    binding=prepare(args.bundle,args.base_assets,args.output)
    if args.prepare_only:
        print(json.dumps(binding,indent=2));return
    os.environ['CUDA_VISIBLE_DEVICES']=str(args.gpu)
    from isaaclab.app import AppLauncher
    app=AppLauncher(**conversion_device(args.gpu)).app
    try:
        from isaaclab.sim.converters import UrdfConverter,UrdfConverterCfg
        target=Path(binding['usd'])
        converter=UrdfConverter(UrdfConverterCfg(asset_path=binding['urdf'],usd_dir=str(target.parent),
            usd_file_name=target.name,force_usd_conversion=True,make_instanceable=True,fix_base=True,
            # Fixed camera/optical links without inertia must not become default 1 kg bodies.
            merge_fixed_joints=True,self_collision=False,collision_from_visuals=False,
            joint_drive=UrdfConverterCfg.JointDriveCfg(drive_type='force',target_type='position',
                gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0.,damping=0.))))
        if Path(converter.usd_path).resolve()!=target.resolve() or not target.is_file():
            raise ValueError('converter output mismatch')
        from pxr import Usd,UsdPhysics
        stage=Usd.Stage.Open(str(target))
        joints={p.GetName() for p in stage.Traverse() if p.IsA(UsdPhysics.RevoluteJoint) or p.IsA(UsdPhysics.PrismaticJoint)}
        expected={j.get('name') for j in ET.parse(binding['urdf']).getroot().findall('joint') if j.get('type')!='fixed'}
        if joints!=expected:raise ValueError('converted articulation joint set mismatch: '+str(joints^expected))
        binding.update(conversion_complete=True,usd_sha256=digest(target),joint_names=sorted(joints))
        (args.output/'a1-binding.json').write_text(json.dumps(binding,indent=2)+'\n')
        print(json.dumps(binding,indent=2),flush=True)
    finally:app.close(skip_cleanup=True)


if __name__=='__main__':main()
