"""Rectified auxiliary output reuses one native overscan sampling pass."""
from copy import deepcopy
from dataclasses import dataclass
from types import SimpleNamespace as NS
from unittest.mock import patch
import inspect
import json
from pathlib import Path
import sys
from playground_sim import camera_frame, camera_metadata, main, first_frame, replay


class Tensor:
    def __init__(self,value):self.value=value
    def __getitem__(self,index):return self
    def cpu(self):return self
    def numpy(self):return self.value
    def tolist(self):return self.value

@dataclass
class Pose:
    position:tuple=(0,0,0)
    orientation_wxyz:tuple=(1,0,0,0)
    def inverse(self):return Pose(tuple(-v for v in self.position),self.orientation_wxyz)
    def compose(self,other):return Pose(tuple(a+b for a,b in zip(self.position,other.position)),other.orientation_wxyz)


def test():
    calibration=dict(serial='cpu',width=640,height=480,intrinsics=dict(fx=392,fy=391,cx=315,cy=234),distortion=dict(coefficients=[0.]*5))
    names=('base_0','left_wrist_0','right_wrist_0')
    env=NS(cfg=NS(d405_rig=dict(device_calibrations={name:deepcopy(calibration) for name in names})),scene={})
    image=NS(shape=(480,640,3),dtype='uint8')
    calls=[]
    views=[]
    def calibrated(view,camera):
        calls.append(camera);views.append(view)
        assert view.scene is env.scene and view.cfg is not env.cfg
        assert view.cfg.d405_rig['device_calibrations'][camera]['distortion']['coefficients']==[0.]*5
        return Tensor(image)
    modules={'robot_sim.tasks.manager_based.coffee_make.d405_camera':NS(color_observation=calibrated),
             'robot_sim.asset_cfg.objects.coffee_machine':NS(RigidTransform=Pose)}
    # Only the native sampling function sees the scene; no overscan is returned directly.
    sensors={name:NS(data=NS(pos_w=Tensor([1,2,4]),quat_w_ros=Tensor([1,0,0,0]),
        intrinsic_matrices=Tensor([[400,0,400],[0,400,320],[0,0,1]]))) for name in names}
    source=dict(calibration_profile=dict(frames=dict(environment_from_camera=dict(position=(1,2,3),orientation_wxyz=(1,0,0,0)))))
    with patch.dict(sys.modules,modules):
        for name in names:assert camera_frame(env,name) is image
        assert calls==list(names)
        meta=camera_metadata(env,sensors,source,Pose((0,0,1)))
        assert meta['image_domain']=='rectified' and meta['head_source_pose_check']['passed']
        assert meta['cameras']['base_0']['K'][0][0]==392
        assert meta['cameras']['base_0']['overscan_K'][0][0]==400
        assert meta['cameras']['base_0']['source_distortion']['coefficients']==[0.]*5
        sensors['base_0'].data.pos_w=Tensor([1,2,4.001])
        try:camera_metadata(env,sensors,source,Pose((0,0,1)))
        except ValueError:pass
        else:raise AssertionError('wrong actual head extrinsic accepted')
        sensors['base_0'].data.pos_w=Tensor([1,2,4])
        env.cfg.d405_rig['device_calibrations']['base_0']['distortion']['coefficients'][0]=.1
        original=deepcopy(env.cfg.d405_rig)
        env._d405_grid_cache={'base_0':'existing-distorted-grid'}
        assert camera_frame(env,'base_0') is image
        meta=camera_metadata(env,sensors,source,Pose((0,0,1)))
        assert meta['cameras']['base_0']['distortion_coefficients']==[0.]*5
        assert meta['cameras']['base_0']['source_distortion']['coefficients'][0]==.1
        assert meta['cameras']['base_0']['rectification']['resampling_passes']==1
        assert env.cfg.d405_rig==original and env._d405_grid_cache=={'base_0':'existing-distorted-grid'}
        grid=views[-1]._d405_grid_cache
        assert camera_frame(env,'base_0') is image and views[-1]._d405_grid_cache is grid
        env.cfg.d405_rig['device_calibrations']['base_0']['intrinsics']['fx']+=1
        assert camera_frame(env,'base_0') is image and views[-1]._d405_grid_cache is not grid
        env.cfg.d405_rig['device_calibrations']['base_0']['distortion']['coefficients'][0]=0
        image.shape=(640,800,3)
        try:camera_frame(env,'base_0')
        except ValueError:pass
        else:raise AssertionError('overscan dimensions accepted')
    for function in (first_frame,replay):
        assert 'camera_frame(' in inspect.getsource(function)
        assert '.data.output[' not in inspect.getsource(function)
    assert 'first_frame(' in inspect.getsource(main)
    print('Playground camera calibrated output/domain and measured pose CPU checks: PASS')


def native_cpu_ramp(native_code,rig_path):
    """Use the installed nonzero rig and actual native Torch sampling on CPU."""
    import hashlib
    import numpy as np
    import torch
    from playground_geometry import configure_native
    configure_native(Path(native_code))
    from robot_sim.asset_cfg.d405 import sampling_map,RENDER_WIDTH,RENDER_HEIGHT
    from robot_sim.tasks.manager_based.coffee_make.d405_camera import color_observation
    from robot_sim.asset_cfg.objects.coffee_machine import RigidTransform
    raw_rig=Path(rig_path).read_bytes();rig=json.loads(raw_rig)
    original=deepcopy(rig);names=('base_0','left_wrist_0','right_wrist_0')
    assert any(abs(x)>1e-12 for name in names for x in rig['device_calibrations'][name]['distortion']['coefficients'])
    yy,xx=np.meshgrid(np.arange(RENDER_HEIGHT),np.arange(RENDER_WIDTH),indexing='ij')
    ramp=np.stack((xx*.2,yy*.25,(xx+yy)*.1),axis=-1).astype(np.float32)
    raw=torch.tensor(ramp,device='cpu').unsqueeze(0)
    sensors={name:NS(data=NS(output={'rgb':raw},pos_w=torch.tensor([[1.,2.,4.]]),
        quat_w_ros=torch.tensor([[1.,0.,0.,0.]]),
        intrinsic_matrices=torch.tensor([[[400.,0.,400.],[0.,400.,320.],[0.,0.,1.]]]))) for name in names}
    env=NS(cfg=NS(d405_rig=rig),scene={'d405_'+name:sensor for name,sensor in sensors.items()})
    source={'calibration_profile':{'frames':{'environment_from_camera':dict(position=[1.,2.,3.],orientation_wxyz=[1.,0.,0.,0.])}}}
    results={}
    for name in names:
        # Populate the real native distorted cache first; it must remain separate.
        distorted=color_observation(env,name)[0].numpy()
        old_grid=env._d405_grid_cache[name].clone()
        actual=camera_frame(env,name)
        calibration=deepcopy(rig['device_calibrations'][name])
        calibration['distortion']['coefficients']=[0.]*5
        uv=sampling_map(calibration)
        expected=np.rint(np.stack((uv[...,0]*.2,uv[...,1]*.25,(uv[...,0]+uv[...,1])*.1),axis=-1)).clip(0,255).astype(np.uint8)
        error=np.abs(actual.astype(int)-expected.astype(int))
        assert actual.shape==(calibration['height'],calibration['width'],3) and actual.dtype==np.uint8
        assert error.max()<=1 and np.mean(error) < .001
        assert torch.equal(env._d405_grid_cache[name],old_grid)
        if any(abs(v)>1e-12 for v in rig['device_calibrations'][name]['distortion']['coefficients']):
            assert np.any(actual!=distorted),'distorted native output was mislabeled rectified'
        results[name]=dict(shape=list(actual.shape),max_pixel_error=int(error.max()),mean_pixel_error=float(error.mean()),
                           distorted_output_differs=bool(np.any(actual!=distorted)))
    metadata=camera_metadata(env,sensors,source,RigidTransform((0.,0.,1.)))
    for name in names:
        row=metadata['cameras'][name];calibration=rig['device_calibrations'][name];k=calibration['intrinsics']
        assert row['distortion_coefficients']==[0.]*5 and row['source_distortion']==calibration['distortion']
        assert row['K']==[[k['fx'],0,k['cx']],[0,k['fy'],k['cy']],[0,0,1]]
        assert row['rectification']['resampling_passes']==1
    assert rig==original and Path(rig_path).read_bytes()==raw_rig
    assert metadata['head_source_pose_check']['passed']
    assert not torch.cuda.is_initialized()
    print(json.dumps(dict(native_cpu_ramp_passed=True,source_rig_sha256=hashlib.sha256(raw_rig).hexdigest(),
        original_rig_unchanged=True,cuda_initialized=False,cameras=results,metadata=metadata),indent=2))


if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--native-code');parser.add_argument('--rig')
    args=parser.parse_args()
    test()
    if args.native_code or args.rig:
        if not args.native_code or not args.rig:parser.error('--native-code and --rig must be supplied together')
        native_cpu_ramp(args.native_code,args.rig)
