"""Pickup provenance and actual-joint-before-closing checks; CPU mocks are not GPU qualification."""
from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace as NS, ModuleType
from unittest.mock import patch
import hashlib,sys,tempfile,unittest
from integration import _digest
import physical_cup_maintenance as maintenance
from test_physical_cup_maintenance import current_fixture

def pickup_fixture():
    parent,old,execution,source,binding,offset,geometry,events,current=current_fixture()
    source['maintenance_context']['pickup_policy']=maintenance.PICKUP_POLICY
    index=next(i for i,s in enumerate(offset['stages'])if s['name']=='approach')
    stage=offset['stages'][index];stage['derived_from_parent_stage']=True
    proof=dict(schema_version='physical_cup_pickup_derivation.v1',binding=maintenance.pickup_binding(source),
        affected_parent_indices=[index],actual_fk_qualified=True,metal_gap_mm=source['grasp_command']['metal_gap_mm'],
        candidate_order=['fixture'],attempts=[dict(candidate_id='fixture',status='qualified_pickup')],
        selected_candidate=dict(candidate_id='fixture'),
        contact_certificate_digest='e'*64,stages={str(index):dict(rows_sha256=_digest(stage['rows']),
            native_actions_sha256=_digest(stage['native_actions']))})
    proof['derivation_digest']=_digest(proof);offset['pickup_derivation']=proof
    geometry.update(source_digest=_digest(source),pickup_derivation_digest=_digest(proof))
    geometry['checks']['grasp_contact']['certificate_digest']='e'*64
    return parent,old,execution,source,offset,geometry,index

@dataclass
class Pose:
    position:tuple=(0.,0.,0.)
    orientation_wxyz:tuple=(1.,0.,0.,0.)
    def inverse(self):return self
    def compose(self,other):return other

class PickupTest(unittest.TestCase):
    def test_right_codec_roundoff_preserves_exact_native_commands(self):
        from physical_cup_return_geometry import verify_rebound_right_hold
        reference=dict(right_arm_joints=[-.012828236407027533,0.,0.,0.,0.,0.],right_gripper=82.992)
        row=deepcopy(reference);row['right_arm_joints'][0]+=2e-16
        native=[.1,.2,.3,.4,.5,.6,82.992]
        stage=dict(rows=[row],native_actions=[[0.]*7+native])
        verify_rebound_right_hold(stage,reference,native)
        for field in ('native','decoded'):
            changed=deepcopy(stage)
            if field=='native':changed['native_actions'][0][7]+=.001
            else:changed['rows'][0]['right_arm_joints'][0]+=.001
            with self.assertRaises(ValueError):verify_rebound_right_hold(changed,reference,native)

    def test_carry_transition_binds_the_existing_joint_goal_without_new_waypoints(self):
        from joint_space import convert_state
        parent,old,execution,source,offset,geometry,index=pickup_fixture()
        source['maintenance_context']['pickup_transition_policy']=maintenance.PICKUP_TRANSITION_POLICY
        base=maintenance.reverse_stages(parent,old);carry=next(i for i,s in enumerate(base)if s['name']=='lift')
        q=convert_state(base[carry]['rows'][-1],source['calibration_profile'],to_reported=False)['left_arm_joints']
        proof=offset['pickup_derivation'];proof['binding']=maintenance.pickup_binding(source)
        proof['actual_fk']=dict(model_joints=q)
        if carry not in proof['affected_parent_indices']:
            proof['affected_parent_indices'].append(carry)
            proof['stages'][str(carry)]=dict(rows_sha256='a'*64,native_actions_sha256='b'*64)
        transition=dict(policy=maintenance.PICKUP_TRANSITION_POLICY,inherited_stage_index=carry,
            inherited_model_joints=q,actual_grasp_model_joints=q,
            urdf_sha256=source['calibration_profile']['model_binding']['urdf_sha256'],
            fk_implementation_sha256='c'*64,
            contact_certificate_digest=proof['contact_certificate_digest'],tray_world_obstacle_retained=True,
            free_xyz=True,world_from_tool_goal=dict(position=[0.,0.,0.],orientation_wxyz=[1.,0.,0.,0.]),
            join_length_l2_rad=0.,native_fk_goal_check=dict(passed=True,position_error_m=0.,angle_error_rad=0.),actual_held_cup_goal_check=dict(passed=True))
        proof['carry_entry_derivation']=transition
        proof['derivation_digest']=_digest({k:v for k,v in proof.items()if k!='derivation_digest'})
        maintenance.validate_pickup_derivation(proof,source,parent)
        for key,value in [('inherited_model_joints',[99.]*6),('tray_world_obstacle_retained',False),('join_length_l2_rad',-1.)]:
            changed=deepcopy(proof);changed['carry_entry_derivation'][key]=value
            changed['derivation_digest']=_digest({k:v for k,v in changed.items()if k!='derivation_digest'})
            with self.assertRaises(ValueError):maintenance.validate_pickup_derivation(changed,source,parent)
        changed=deepcopy(proof);changed['pickup_lift_height_m']=.02
        changed['derivation_digest']=_digest({k:v for k,v in changed.items()if k!='derivation_digest'})
        with self.assertRaises(ValueError):maintenance.validate_pickup_derivation(changed,source,parent)

    def test_pickup_policy_proof_and_complete_geometry_are_required(self):
        parent,old,execution,source,offset,geometry,index=pickup_fixture()
        plan=maintenance.derive_return(parent,old,execution,source,offset,geometry)
        maintenance.validate_evidence(plan,source)
        self.assertEqual(plan['motion_profile'],'physical-maintenance-current-pickup-30hz')
        for change in ('missing','binding','index','rows','geometry','order'):
            s,o,g=deepcopy(source),deepcopy(offset),deepcopy(geometry)
            if change=='missing':o.pop('pickup_derivation')
            elif change=='binding':s['maintenance_context']['current_geometry']['revision']='changed'
            elif change=='index':
                p=o['pickup_derivation'];p['affected_parent_indices']=[0];p['derivation_digest']=_digest({k:v for k,v in p.items()if k!='derivation_digest'})
            elif change=='rows':o['stages'][index]['rows'][0]['left_arm_joints'][0]+=.001
            elif change=='order':
                p=o['pickup_derivation'];p['candidate_order']=['earlier','fixture'];p['derivation_digest']=_digest({k:v for k,v in p.items()if k!='derivation_digest'})
            else:g['pickup_derivation_digest']='0'*64
            with self.subTest(change=change),self.assertRaises(ValueError):
                maintenance.derive_return(parent,old,execution,s,o,g)
        legacy=deepcopy(source);legacy['maintenance_context'].pop('pickup_policy')
        with self.assertRaises(ValueError):maintenance._stages(parent,old,legacy,offset)

    def test_ik_and_actual_fk_precede_closing_and_candidate_order_is_preserved_both_modes(self):
        from physical_cup_return_geometry import retarget_pickup
        import motiongen_plan,motiongen_targets,motiongen_motion,gpu_acceptance
        parent,old,execution,source,offset,geometry,index=pickup_fixture()
        modules={}
        for name in ['robot_sim','robot_sim.asset_cfg','robot_sim.asset_cfg.objects','robot_sim.asset_cfg.robots']:
            modules[name]=ModuleType(name);modules[name].__path__=[]
        coffee=ModuleType('robot_sim.asset_cfg.objects.coffee_machine');coffee.RigidTransform=Pose
        finger=ModuleType('robot_sim.asset_cfg.robots.ac_one_finger_contact')
        finger.ac_one_finger_contact_spec=lambda:NS(rubber_thickness_m=.0015)
        modules.update({coffee.__name__:coffee,finger.__name__:finger,'trimesh':NS(load=lambda *a,**k:object())})
        stages=deepcopy(offset['stages'])
        pre_index=next(i for i,stage in enumerate(stages)if stage['name']=='approach_pre')
        fixed_right_native=stages[pre_index-1]['native_actions'][-1][7:]
        candidate=dict(world_from_pregrasp={'position':[0,0,0],'orientation_wxyz':[1,0,0,0]},
            world_from_tool={'position':[0,0,0],'orientation_wxyz':[1,0,0,0]})
        targets=dict(grasp_candidates=[dict(candidate,candidate_id='ranked-b'),dict(candidate,candidate_id='ranked-a')],
                     grasp_candidate_selection={'method':'existing-ranked-order'})
        for environment in ('fake','physical'):
            trace=[];saved={};phases=[]
            class Engine:
                config={'kinematics':{'lock_joints':{'left_joint7':0.,'left_joint8':0.}}}
                receipts=[]
                cup_contact=NS(allow_claw_contact=lambda value:phases.append(value))
                def plan_pose(self,*a,**k):
                    assert phases[-1] is True,'pre/approach must use the canonical proposal policy'
                    trace.append('ik');return [[0.]*6,[.001]*6],{}
                def update_locked_joints(self,*a,**k):pass
                def update_world(self,*a,**k):pass
            kine=NS(identity={'urdf_path':'CPU fixture'},spec=None,names=lambda side:[f'left_joint{i}'for i in range(1,7)])
            kine.positions=lambda row:dict(left_joint7=0.,left_joint8=0.)
            def pose(*a):trace.append('actual_fk');return candidate['world_from_tool']
            kine.pose=pose
            def close(*a,**k):
                assert phases[-1] is False,'source closing must restore the strict phase'
                trace.append('closing:'+k['candidate_id'].split('/')[-1]);raise ValueError('actual MotionGen pose lacks 2 mm contact-band reserve: contact_geometry_unproven: open pad already contacts cup: left_link7')
            def encode(stage,*a,**k):return dict(stage,native_actions=[[0.]*7+fixed_right_native for row in stage['rows']])
            with tempfile.TemporaryDirectory()as directory:
                p=Path(directory)/'cup.obj';p.write_bytes(b'CPU fixture; not physical mesh')
                s=deepcopy(source);s['environment']=environment
                s['maintenance_context']['current_geometry']['object_mesh_sha256']=hashlib.sha256(p.read_bytes()).hexdigest()
                with patch.dict(sys.modules,modules),patch.object(motiongen_targets,'generate_targets',return_value=targets), \
                    patch.object(motiongen_plan,'encode_stage',side_effect=encode), \
                    patch.object(motiongen_plan,'world_config',return_value=({},)), \
                    patch.object(motiongen_motion,'continuous_path',side_effect=lambda path,*a,**k:(path,{})), \
                    patch.object(gpu_acceptance,'closing_geometry',side_effect=close):
                    planner=NS(geo=NS(mount=Pose(),audit=lambda *a,**k:None))
                    with self.assertRaisesRegex(ValueError,'bounded pickup candidates exhausted'):
                        retarget_pickup(planner,NS(sample=NS(world_from_cup=Pose())),{},Engine(),{'fixed_meshes':[]},kine,
                            stages,parent,s,p,lambda name,value:saved.update({name:deepcopy(value)}))
                    known_trace=list(trace);known_attempts=deepcopy(saved['pickup-attempts.json']);trace.clear()
                    with patch.object(gpu_acceptance,'closing_geometry',side_effect=ValueError('source identity changed')):
                        with self.assertRaisesRegex(ValueError,'source identity changed'):
                            retarget_pickup(planner,NS(sample=NS(world_from_cup=Pose())),{},Engine(),{'fixed_meshes':[]},kine,
                                stages,parent,s,p,lambda name,value:saved.update({name:deepcopy(value)}))
                    self.assertEqual(trace,['ik','ik','actual_fk'])
                    self.assertEqual(saved['pickup-attempts.json'],[dict(candidate_id='ranked-b',status='aborted',error='source identity changed')])
                    with patch.object(gpu_acceptance,'closing_geometry',side_effect=ValueError('MotionGen pickup_held exceeds 0.5deg along the held-orientation path')):
                        with self.assertRaisesRegex(ValueError,'bounded pickup candidates exhausted'):
                            retarget_pickup(planner,NS(sample=NS(world_from_cup=Pose())),{},Engine(),{'fixed_meshes':[]},kine,
                                stages,parent,s,p,lambda name,value:saved.update({name:deepcopy(value)}))
                    self.assertEqual([a['status']for a in saved['pickup-attempts.json']],['rejected','rejected'])
                    self.assertIn('pickup-motiongen-receipts.json',saved)
            self.assertEqual(known_trace,['ik','ik','actual_fk','closing:ranked-b','ik','ik','actual_fk','closing:ranked-a'])
            self.assertEqual([x['candidate_id']for x in known_attempts],['ranked-b','ranked-a'])

    def test_original_obj_width_is_only_a_geometric_comparison(self):
        import trimesh
        from grasp_contact_geometry import original_mesh_sections
        mesh=trimesh.creation.box(extents=[.06,.065,.13]);mesh.apply_translation([0.,0.,.065])
        region=dict(contact_height_interval_m=[.04,.07],centering_axis_cup=[0.,1.,0.])
        args=dict(mesh_sha256='a'*64,metal_gap_mm=65.,opening_mm=83.,rubber_thickness_m=.0015)
        result=original_mesh_sections(mesh,region,**args)
        self.assertTrue(result['passed']);self.assertEqual(len(result['sections']),3)
        self.assertFalse(original_mesh_sections(mesh,region,**dict(args,metal_gap_mm=72.))['passed'])
        for axis in ([0.,0.,0.],[float('nan'),1.,0.],[1.,0.]):
            with self.assertRaises(ValueError):original_mesh_sections(mesh,dict(region,centering_axis_cup=axis),**args)

if __name__=='__main__':unittest.main()
