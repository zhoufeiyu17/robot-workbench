"""CPU-only support-plane proof and diagnostic stop conditions."""
from copy import deepcopy
from dataclasses import dataclass
from itertools import product
from types import SimpleNamespace as NS
from unittest.mock import patch
import math,sys,unittest
from integration import _qrotate
from pickup_pregrasp import derive,validate,original_target_diagnosis

def inputs():
    return dict(hand=[dict(name='original hand',vertices=[list(v)for v in product((-.02,.14),(-.01,.01),(-.01,.01))])],
        bands=[dict(name='cup',vertices=[list(v)for v in product((-.04,.04),(-.04,.04),(0.,.13))])],
        band_profiles=[dict(name='cup',z_min_m=0.,z_max_m=.13,radius_m=.04)],
        open_jaw_m=.0415,max_band_radius_m=.04,source_parts_sha256='a'*64,geometry_export_sha256='b'*64,
        coordinate_frame='cup_bottom_center',units='m',hand_representation='original_convex_body_vertices')

@dataclass
class Pose:
    position:tuple=(0.,0.,0.)
    orientation_wxyz:tuple=(1.,0.,0.,0.)
    def compose(self,other):return other

class PregraspTest(unittest.TestCase):
    def test_whole_body_plane_recomputes_in_both_modes_and_rotated_frames(self):
        for environment in ('fake','physical'):
            for q in ([1.,0.,0.,0.],[math.sqrt(.5),0.,0.,math.sqrt(.5)]):
                n=_qrotate(q,[1,0,0]);pose=dict(position=[-.1*v for v in n],orientation_wxyz=q)
                cup=dict(position=[1.,2.,3.],orientation_wxyz=[1.,0.,0.,0.]);data=inputs();proof=derive(data,pose,cup)
                self.assertAlmostEqual(proof['retreat_m'],.0801);self.assertTrue(proof['within_old_bound'])
                self.assertAlmostEqual(proof['separation_m'],.0001)
                self.assertEqual(derive(data,{k:tuple(v)for k,v in pose.items()},{k:tuple(v)for k,v in cup.items()}),proof)
                source=dict(environment=environment,maintenance_context=dict(current_geometry=dict(export_sha256='b'*64),current_scene=dict(cup_pose=cup)))
                parent=dict(planning_backend=dict(source_parts_sha256='a'*64),gripper_opening_mm=83.)
                self.assertEqual(validate(proof,data,source,parent),proof)
                for field in ('retreat_m','inputs_sha256','world_from_pregrasp'):
                    changed=deepcopy(proof);changed[field]=0
                    with self.assertRaises(ValueError):validate(changed,data,source,parent)
                changed=deepcopy(data);changed['geometry_export_sha256']='c'*64
                with self.assertRaises(ValueError):validate(proof,changed,source,parent)
        data=inputs();pose=dict(position=[-1.,0.,0.],orientation_wxyz=[1.,0.,0.,0.])
        with self.assertRaises(ValueError):derive(data,pose,cup)
        self.assertFalse(derive(data,dict(pose,position=[.1,0.,0.]),cup)['within_old_bound'])
        shifted=deepcopy(data)
        for point in shifted['bands'][0]['vertices']:point[2]+=.065
        with self.assertRaises(ValueError):derive(shifted,dict(pose,position=[-.1,0.,0.]),cup)
        data['hand'][0]['vertices'][0][0]=float('nan')
        with self.assertRaises(ValueError):derive(data,dict(pose,position=[-.1,0.,0.]),cup)

    def test_phase_zero_requires_actual_matching_finite_witness_and_never_executes(self):
        candidates=[dict(candidate_id=str(i),world_from_pregrasp=dict(position=[0,0,0],orientation_wxyz=[1,0,0,0]))for i in range(2)]
        prepared=dict(proxy=dict(part_reports=[dict(alias='body',link='base_link',name='base_link/chassis_0/component_1'),dict(alias='arm',link='left_link3',name='left3')]))
        for failure in (None,'world','missing','nan','other','unexpected','no_pair'):
            saved={}
            class Engine:
                receipts=[]
                config=dict(kinematics=dict(base_link='base_link',cspace=dict(joint_names=[f'left_joint{i}'for i in range(1,7)])))
                generator=NS(joint_names=[f'left_joint{i}'for i in range(1,7)])
                cup_contact=NS(allow_claw_contact=lambda enabled:None)
                def plan_pose(self,*args,**kwargs):
                    record=dict(failed_ik_constraints=dict(self=[[1.]],world=[[0.]]),failed_ik_joints=[[0.]*6],
                        proxy_overlap_diagnostic=[dict(aliases=['body','arm'],proxy_overlap_m=.001)])
                    if failure=='missing':record.pop('failed_ik_constraints')
                    if failure=='nan':record['failed_ik_constraints']['self']=[[float('nan')]]
                    if failure=='no_pair':record['proxy_overlap_diagnostic']=[]
                    if failure=='world':record['proxy_overlap_diagnostic']=[];record['failed_ik_constraints']['world']=[[1.]]
                    self.receipts.append(record)
                    if failure=='unexpected':return [],{}
                    raise ValueError('other diagnostic failure'if failure=='other'else'MotionGen pickup_approach_pre failed: MotionGenStatus.IK_FAIL')
            module='robot_sim.asset_cfg.objects.coffee_machine'
            with patch.dict(sys.modules,{module:NS(RigidTransform=Pose)}),patch('pickup_pregrasp.fixed_body_witnesses',return_value=[dict(backend='CPU mock of same-body GPU world witness')]if failure=='world'else[]):
                call=lambda:original_target_diagnosis(Engine(),prepared,candidates,[0.]*6,Pose(),lambda n,v:saved.update({n:deepcopy(v)}))
                if failure in(None,'world'):self.assertTrue(call()['passed'])
                else:
                    with self.assertRaises(ValueError):call()
                    self.assertFalse(saved['original-target-diagnosis.json']['passed'])
                self.assertIn('pickup-motiongen-receipts.json',saved)

if __name__=='__main__':unittest.main()
