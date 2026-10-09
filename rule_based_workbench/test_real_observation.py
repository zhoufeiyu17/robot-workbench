"""CPU synthetic PNG/ZIP fixtures; no camera, robot, detector or network calls."""
from copy import deepcopy
import io
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch
from types import SimpleNamespace
import numpy as np
from PIL import Image

from integration import _digest
from real_observation import capture, decode_snapshot, inverse_brown_map, inverse_brown_rectified_pixels, remap_rgbd
from real_rgbd import pair_metadata, snapshot_bundle
from real_pipeline import validate_capture
from test_harness import PROFILE, rejects


def fixture(i, update=None):
    intr=dict(width=4,height=3,fx=100.,fy=101.,cx=2.,cy=1.,coefficients=[0.]*5,
              distortion_model='distortion.brown_conrady',rectified=False)
    meta=pair_metadata(serial='h',camera_name='head',color_number=i,depth_number=i+10,
         color_ms=i*1000.,depth_ms=i*1000.+1,color_domain='hardware_clock',depth_domain='hardware_clock',
         host_receive_ns=(i+1)*1_000_000_000,depth_scale_m=.001,intrinsics=intr)
    if update:update(meta)
    color=io.BytesIO();depth=io.BytesIO()
    Image.fromarray(np.full((3,4,3),[5,10,15],dtype=np.uint8)).save(color,format='PNG')
    Image.fromarray(np.full((3,4),1234,dtype=np.uint16)).save(depth,format='PNG')
    return snapshot_bundle(meta,color.getvalue(),depth.getvalue())


def main():
    profile=deepcopy(PROFILE)
    binding=dict(calibration_accepted=True,evidence_source='physical_rgbd',robot_id='fixture',
                 camera_serials=dict(head='h',left='l',right='r'),calibration_profile_digest=_digest(profile))
    with TemporaryDirectory() as directory:
        output=Path(directory)/'observation';frames=iter(fixture(i) for i in range(9));clock=[0.]
        result=capture(lambda:next(frames),profile,binding,output,monotonic=lambda:clock[0],sleep=lambda t:clock.__setitem__(0,clock[0]+t))
        manifest=validate_capture(output,result)
        assert len(manifest['frames'])==9 and manifest['source_clock']=='host_receive_wall_ns'
        with np.load(output/'rgbd-0000.npz',allow_pickle=False) as data:
            assert set(data.files)=={'rgb','depth_m','K','position','quaternion_wxyz'}
            assert np.allclose(data['depth_m'],1.234)
            assert np.array_equal(data['rgb'][0,0],[5,10,15])
            assert np.allclose(data['position'],profile['frames']['environment_from_camera']['position'])
        rejects(lambda _:capture(lambda:fixture(0),profile,binding,Path(directory)/'duplicate',sleep=lambda _:None),None)
        assert not (Path(directory)/'duplicate/rgbd-manifest.json').exists()
        bad=deepcopy(binding);bad['calibration_accepted']=False
        rejects(lambda _:capture(lambda:fixture(0),profile,bad,Path(directory)/'unaccepted'),None)
        assert not (Path(directory)/'unaccepted').exists()
    rejects(lambda b:decode_snapshot(b,'h',profile),fixture(0,lambda m:m.update(serial='wrong')))
    rejects(lambda b:decode_snapshot(b,'h',profile),fixture(0,lambda m:m['intrinsics'].update(coefficients=[.1,0,0,0,0],distortion_model='unsupported')))
    # Nonzero radial AND tangential terms: verify inverse mapping numerically.
    intr=dict(width=64,height=48,fx=53.,fy=54.,cx=31.2,cy=23.7,
              coefficients=[.11,-.05,.003,-.004,.012],distortion_model='inverse_brown_conrady')
    mapping=inverse_brown_map(json.dumps(intr,sort_keys=True))
    yy,xx=np.indices((48,64),dtype=float); target=np.stack((xx,yy),axis=-1)
    assert np.max(np.abs(inverse_brown_rectified_pixels(mapping,intr)-target)) < .001
    assert np.max(np.abs(mapping-target)) > .1
    # Compare with the authoritative calibration equations if the repo is supplied.
    import sys, importlib.util
    if len(sys.argv)>1:
        path=Path(sys.argv[1])/'infra/arx_env/joint_head_workbench_capture.py'
        spec=importlib.util.spec_from_file_location('calibration_capture_reference',path)
        ref=importlib.util.module_from_spec(spec);spec.loader.exec_module(ref)
        assert np.allclose(ref.rectify_corners(mapping.reshape(-1,2),intr),target.reshape(-1,2),atol=.001)
    rgb=np.stack((xx,yy,xx+yy),axis=-1).astype(np.uint8)
    depth=(1000+xx+64*yy).astype(np.uint16);depth[20,20]=0
    sample=target.copy();sample[10,10]=[10.25,11.5];sample[0,0]=[-.1,0]
    sample[1,1]=[20,20]
    color,z=remap_rgbd(rgb,depth,sample)
    assert np.array_equal(color[10,10],np.rint([10.25,11.5,21.75]).astype(np.uint8))
    assert z[10,10]==depth[12,10] and z[1,1]==0
    assert not color[0,0].any() and z[0,0]==0
    assert z[2,2]==depth[2,2] and np.array_equal(color[2,2],rgb[2,2])
    meta,*_,arrays=decode_snapshot(fixture(0,lambda m:m['intrinsics'].update(
        coefficients=[.1,-.02,.003,-.004,.01],distortion_model='inverse_brown_conrady')),'h',profile)
    assert meta['rectification']=='inverse_brown_conrady_to_same_K'
    assert meta['original_intrinsics']['coefficients'][0]==.1 and meta['intrinsics']['rectified']
    assert arrays['depth_m'].shape==(3,4)
    assert np.all((arrays['depth_m']==0)|np.isclose(arrays['depth_m'],1.234))
    calls=[]
    cv2=SimpleNamespace(CV_32FC1=5,INTER_LINEAR=1,INTER_NEAREST=0,BORDER_CONSTANT=0,
        initUndistortRectifyMap=lambda *args:(np.zeros((3,4)),np.zeros((3,4))),
        remap=lambda image,mx,my,method,**kwargs:(calls.append(method) or image))
    with patch.dict('sys.modules',{'cv2':cv2}):
        meta,*_=decode_snapshot(fixture(0,lambda m:m['intrinsics'].update(coefficients=[.1,0,0,0,0])),'h',profile)
        assert calls==[cv2.INTER_LINEAR,cv2.INTER_NEAREST]
        assert meta['intrinsics']['rectified'] and meta['original_intrinsics']['coefficients'][0]==.1
    print('Physical observation: nine-pair sealing, units, transforms, provenance and distortion routing PASS (CPU fixtures)')


if __name__=='__main__':main()
