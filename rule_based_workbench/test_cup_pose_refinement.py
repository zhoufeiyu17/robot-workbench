"""Independent projection fixtures check3D response and fail-closed image evidence."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

import cv2
import numpy as np
from cup_pose_refinement import (METHOD, fit_translation, mesh_rim, refine_observation,
                                 validate_observation, validate_spec)
from robot_workbench.observation_window import ContinuousObservationWindow


def spec():
    return dict(schema='cup_pose_refinement.v1',method=METHOD,cup_profile_sha256='a'*64,
        perception_asset_digest='b'*64,mesh_sha256='c'*64,rim_height_m=.10925,radius_m=.0395,
        max_fit_rms_px=.75,max_position_delta_m=.03)


K=np.array([[700.,0.,320.],[0.,700.,240.],[0.,0.,1.]])


def projected_rim(position, *, tilt=0.):
    theta=np.linspace(0,2*np.pi,256,endpoint=False)
    points=np.c_[.0395*np.cos(theta),.0395*np.sin(theta),np.full(len(theta),.10925)]
    pixels,_=cv2.projectPoints(points,np.array([tilt,0.,0.]),np.asarray(position),K,np.zeros(5))
    return pixels[:,0].astype(np.float32)


def observation(position,stamp=1):
    identity=np.eye(4).ravel().tolist()
    return dict(pair_valid=True,provider_status='OK',source_stamp_ns=stamp,tracking_session_id='session',
        runtime_config_id='runtime',frame_id='camera',tracks=[
        dict(object_type=kind,track_id=i,pose_current=True,world={'xyz_m':list(position)},
             canonical_world={'xyz_m':list(position)},xyz_m=list(position),
             provenance=dict(asset=dict(asset_id=kind,revision='v1',digest='b'*64),
                 canonical_from_simulation_mesh_root=identity))for i,kind in enumerate(('coffee_machine','cup'),1)])


class RimTest(unittest.TestCase):
    def test_full_xyz_inverse_and_real_z_change_reaches_original_guard(self):
        p=np.array([.02,-.01,.65]);initial=p+[.003,-.002,.005]
        a=fit_translation(cv2.fitEllipse(projected_rim(p)),K,[0.,0.,0.],[1.,0.,0.,0.],initial,spec())
        moved=p+[0.,0.,.01]
        b=fit_translation(cv2.fitEllipse(projected_rim(moved)),K,[0.,0.,0.],[1.,0.,0.,0.],initial,spec())
        np.testing.assert_allclose(a['world_m'],p,atol=2e-6)
        np.testing.assert_allclose(b['world_m'],moved,atol=2e-6)
        window=ContinuousObservationWindow()
        for i in range(9):window.add(observation(a['world_m'],1+i*400_000_000))
        self.assertTrue(window.status()['ready'])
        window.add(observation(b['world_m'],1+9*400_000_000))
        self.assertFalse(window.status()['ready'])
        self.assertEqual(window.status()['last_reset']['reason'],'cup_position_span')
        self.assertGreater(window.status()['last_reset']['displacement_m'],.009)

    def test_wrong_scale_or_incompatible_tilt_is_rejected(self):
        p=[.02,-.01,.65];ellipse=cv2.fitEllipse(projected_rim(p))
        bad=spec();bad['radius_m']=.06
        with self.assertRaises(ValueError):fit_translation(ellipse,K,[0.,0.,0.],[1.,0.,0.,0.],p,bad)
        with self.assertRaises(ValueError):fit_translation(cv2.fitEllipse(projected_rim(p,tilt=.7)),K,[0.,0.,0.],[1.,0.,0.,0.],p,spec())

    def test_image_receipt_raw_preservation_and_no_missing_rim_fallback(self):
        with TemporaryDirectory()as tmp:
            path=Path(tmp)/'frame.npz';p=[.02,-.01,.65];pixels=projected_rim(p)
            rgb=np.full((480,640,3),[90,55,30],np.uint8)
            cv2.fillConvexPoly(rgb,np.rint(pixels).astype(np.int32),(255,255,255),lineType=cv2.LINE_AA)
            np.savez(path,rgb=rgb,K=K,position=np.zeros(3),quaternion_wxyz=np.array([1.,0.,0.,0.]))
            cup=dict(asset_id='cup',revision='v1',interaction_profile_sha256='a'*64,pose_refinement=spec())
            raw=observation(p);raw['tracks'][1]['image_bbox']=dict(valid=True,freshness='CURRENT',frame_id='camera',
                source_stamp_ns=1,xyxy=[float(pixels[:,0].min()-3),float(pixels[:,1].min()-3),
                                      float(pixels[:,0].max()+3),float(pixels[:,1].max()+3)])
            result=refine_observation(raw,path,cup);self.assertTrue(result['pair_valid'])
            receipt=result['tracks'][1]['provenance']['pose_refinement']
            self.assertEqual(receipt['raw_world_m'],p)
            self.assertEqual(raw['tracks'][1]['world']['xyz_m'],p)
            validate_observation(result,cup,input_sha256=hashlib.sha256(path.read_bytes()).hexdigest())
            json.dumps(result,allow_nan=False)  # Native scalars are part of the saved contract.
            bad=deepcopy(result);bad['tracks'][1]['world']['xyz_m'][0]+=.001
            with self.assertRaises(ValueError):validate_observation(bad,cup)
            with self.assertRaises(ValueError):validate_observation(result,cup,input_sha256='d'*64)
            np.savez(path,rgb=np.zeros_like(rgb),K=K,position=np.zeros(3),quaternion_wxyz=np.array([1.,0.,0.,0.]))
            rejected=refine_observation(raw,path,cup)
            self.assertFalse(rejected['pair_valid']);self.assertFalse(rejected['tracks'][1]['pose_current'])
            self.assertEqual(rejected['tracks'][1]['provenance']['pose_refinement']['status'],'rejected')
            bad=deepcopy(raw);bad['tracks'][1]['provenance']['asset']['asset_id']='another'
            with self.assertRaisesRegex(ValueError,'another detected asset'):refine_observation(bad,path,cup)

    def test_profile_and_finite_limits(self):
        validate_spec(spec(),'a'*64)
        with self.assertRaises(ValueError):validate_spec(spec(),'d'*64)
        for key,value in [('radius_m',float('nan')),('max_fit_rms_px',2.),('max_position_delta_m',.031)]:
            bad=spec();bad[key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):validate_spec(bad,'a'*64)

    def test_visual_mesh_outer_ring_is_distinct_from_inner_lip_and_collision_margin(self):
        with TemporaryDirectory()as tmp:
            path=Path(tmp)/'visual.obj';lines=[]
            for radius,height in ((.028,0.),(.0395,.10925),(.037,.10925),(.0391,.1105)):
                for t in np.linspace(0,2*np.pi,128,endpoint=False):lines.append(f'v {radius*np.cos(t)} {radius*np.sin(t)} {height}\n')
            path.write_text(''.join(lines));value=mesh_rim(path)
            self.assertAlmostEqual(value['radius_m'],.0395,places=8)
            self.assertAlmostEqual(value['rim_height_m'],.10925,places=8)


if __name__=='__main__':unittest.main()
