"""CPU adapter checks: overscan must pass through native calibrated observation."""
from copy import deepcopy
from dataclasses import dataclass
from types import SimpleNamespace as NS
from unittest.mock import patch
import inspect
import sys
from playground_sim import camera_frame, camera_metadata, main, replay


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
    env=NS(cfg=NS(d405_rig=dict(device_calibrations={name:deepcopy(calibration) for name in names})))
    image=NS(shape=(480,640,3),dtype='uint8')
    calls=[]
    def calibrated(env,camera):calls.append(camera);return Tensor(image)
    modules={'robot_sim.tasks.manager_based.coffee_make.d405_camera':NS(color_observation=calibrated),
             'robot_sim.asset_cfg.objects.coffee_machine':NS(RigidTransform=Pose)}
    # No raw RGB field is supplied: output cannot accidentally read overscan.
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
        sensors['base_0'].data.pos_w=Tensor([1,2,4.001])
        try:camera_metadata(env,sensors,source,Pose((0,0,1)))
        except ValueError:pass
        else:raise AssertionError('wrong actual head extrinsic accepted')
        env.cfg.d405_rig['device_calibrations']['base_0']['distortion']['coefficients'][0]=.1
        try:camera_frame(env,'base_0')
        except ValueError:pass
        else:raise AssertionError('distorted frame mislabeled rectified')
        env.cfg.d405_rig['device_calibrations']['base_0']['distortion']['coefficients'][0]=0
        image.shape=(640,800,3)
        try:camera_frame(env,'base_0')
        except ValueError:pass
        else:raise AssertionError('overscan dimensions accepted')
    for function in (main,replay):
        assert 'camera_frame(' in inspect.getsource(function)
        assert '.data.output[' not in inspect.getsource(function)
    print('Playground camera calibrated output/domain and measured pose CPU checks: PASS')

if __name__=='__main__':test()
