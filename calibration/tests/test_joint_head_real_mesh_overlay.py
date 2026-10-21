import unittest
from types import SimpleNamespace
import numpy as np
from arx_env.joint_head_workbench_overlay import pair_real_feedback, real_mesh_overlay

class OverlayTests(unittest.TestCase):
    def test_nearest_sample_and_rejection(self):
        def sample(t,pid=7):return {'owner':{'pid':pid,'calibration_feedback':{'sample_wall_ns':t,'qpos':[t]*14}}}
        h=[sample(980_000_000),sample(1_008_000_000),sample(1_000_000_000,8)]
        f,m=pair_real_feedback({'frame_timestamp_ns':1_000_000_000},h,7)
        self.assertEqual(m['image_joint_delta_ms'],8)
        self.assertEqual(f['sample_wall_ns'],1_008_000_000)
        with self.assertRaises(ValueError):pair_real_feedback({'frame_timestamp_ns':2_000_000_000},h,7)
    def test_real_pixels_actual_joints_and_distortion(self):
        seen=[]
        def frames(q):seen.append(q.copy());return {s:{'link':np.eye(4)} for s in ('left','right')}
        v=np.array([[-.3,-.3,1],[.3,-.3,1],[0,.3,1]])
        g=SimpleNamespace(frames=frames,side_meshes={'left':[('link',v,None)],'right':[]})
        camera=dict(width=64,height=48,fx=50,fy=50,cx=32,cy=24,coefficients=[-.05,.05,.001,.001,-.01],distortion_model='distortion.inverse_brown_conrady')
        candidate=dict(camera_T_world_optical=np.eye(4).tolist(),joint_offsets_rad={'left':[.1]*6,'right':[.2]*6})
        original=np.full((48,64,3),90,np.uint8);q=np.arange(14)/100;q[6]=q[13]=.07
        result=real_mesh_overlay(g,original,q,candidate,camera)
        np.testing.assert_allclose(seen[0],np.r_[q[:6]+.1,q[7:13]+.2])
        self.assertTrue(np.any(result!=original));np.testing.assert_array_equal(result[0,0],original[0,0])
        self.assertTrue(np.all(original==90))

    def test_overlay_retains_endpoint_feedback_without_mutating_geometry(self):
        from unittest.mock import patch
        from arx_env.joint_head_workbench_overlay import mesh_overlay
        from arx_env.joint_head_jaw_mapping import interpolate, GAP_MM, FEEDBACK
        v=np.array([[-.1,-.1,1],[.1,-.1,1],[0,.1,1]])
        camera=dict(width=64,height=48,fx=1000,fy=1000,cx=32,cy=24)
        g=SimpleNamespace(frames=lambda q:{s:{'link6':np.eye(4)} for s in ('left','right')},
            side_meshes={s:[('link6',v,None)] for s in ('left','right')},
            gaps={'left':.083,'right':.083},hand={'signs':np.ones(3)},k=camera)
        img=np.zeros((48,64,3),np.uint8)
        candidate=dict(camera_T_world_optical=np.eye(4), offsets=np.zeros(12),jaw_gap_mm=[-2.,87.])
        with patch('cv2.fillConvexPoly') as draw:
            mesh_overlay(g,img,np.zeros(12),candidate)
            polygons=[call.args[1].copy() for call in draw.call_args_list]
        expected=v.copy();expected[:,1]+=(.087-.083)/2
        np.testing.assert_array_equal(polygons[1], (expected[:,:2]*1000+[32,24]).astype(np.int32))
        raw=np.zeros(14)
        raw[6]=interpolate(-2.,GAP_MM['left'],FEEDBACK['left'])
        raw[13]=interpolate(87.,GAP_MM['right'],FEEDBACK['right'])
        with patch('cv2.fillConvexPoly') as draw:
            real_mesh_overlay(g,img,raw,dict(camera_T_world_optical=np.eye(4),
                joint_offsets_rad={'left':[0.]*6,'right':[0.]*6}),camera)
            np.testing.assert_allclose(draw.call_args_list[1].args[1],polygons[1],atol=1)
        self.assertEqual(g.gaps,{'left':.083,'right':.083})
        np.testing.assert_array_equal(v,np.array([[-.1,-.1,1],[.1,-.1,1],[0,.1,1]]))
        with self.assertRaises(ValueError):
            mesh_overlay(g,img,np.zeros(12),{**candidate,'jaw_gap_mm':[0,89.000001]})

    def test_articulated_fingers_use_gap_frames(self):
        from arx_env.joint_head_workbench_overlay import mesh_overlay
        from arx_env.joint_head_jaw_mapping import feedback_gaps
        seen=[]
        vertices=np.array([[-.1,-.1,1],[.1,-.1,1],[0,.1,1]])
        camera=dict(width=64,height=48,fx=50,fy=50,cx=32,cy=24)
        class Geometry:
            hand=None
            k=camera
            side_meshes={'left':[('finger',vertices,None)],'right':[]}
            def with_feedback_gaps(self,*gaps):
                seen.append(gaps)
                transform=np.eye(4);transform[0,3]=gaps[0]/100
                return SimpleNamespace(frames=lambda q:{'left':{'finger':transform}})
        g=Geometry();image=np.zeros((48,64,3),np.uint8)
        candidate=dict(camera_T_world_optical=np.eye(4),offsets=np.zeros(12),jaw_gap_mm=[0,0])
        closed=mesh_overlay(g,image,np.zeros(12),candidate)
        opened=mesh_overlay(g,image,np.zeros(12),{**candidate,'jaw_gap_mm':[40,40]})
        self.assertFalse(np.array_equal(closed,opened))
        q=np.zeros(14);q[6]=q[13]=.04
        real_mesh_overlay(g,image,q,dict(camera_T_world_optical=np.eye(4),joint_offsets_rad={s:[0]*6 for s in ('left','right')}),camera)
        np.testing.assert_allclose(seen[-1],feedback_gaps(q))
        np.testing.assert_array_equal(vertices,[[-.1,-.1,1],[.1,-.1,1],[0,.1,1]])
