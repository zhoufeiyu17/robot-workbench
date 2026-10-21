"""CPU checks for persistent model reuse and dynamic-world binding; no CUDA."""
from copy import deepcopy
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

import numpy as np

from a1_asset_derivation import CERTIFICATES
from curobo_planning import robot_config
import motiongen_plan as planning
from motiongen_motion import checked_locked_transforms, current_link_goals, locked_transform_config


@dataclass
class Pose:
    position: tuple
    orientation_wxyz: tuple=(1.,0.,0.,0.)


class CacheTest(unittest.TestCase):
    def setUp(self):
        self.directory=TemporaryDirectory();self.addCleanup(self.directory.cleanup)
        root=Path(self.directory.name);self.urdf=root/'robot.urdf'
        self.names=(*planning.LEFT,*planning.RIGHT,'left_joint7','left_joint8','right_joint17','right_joint18')
        joints=''.join('<joint name="'+n+'" type="'+('prismatic' if n in self.names[-4:] else 'revolute')+'">'
            '<axis xyz="0 0 1"/><limit lower="0" upper="3" velocity="5.5"/></joint>' for n in self.names)
        self.urdf.write_text('<robot name="fixture"><link name="base_link"/><link name="left_link6"/>'+joints+'</robot>')
        for name in CERTIFICATES:(root/name).write_text(json.dumps(dict(terminal_jaw_position_m=.03,version=1)))
        self.opened=dict.fromkeys(self.names,.1)
        self.planner=NS(identity={'model':'A1'},fk=NS(movable=self.names,
            forward=lambda q:{'right_link11':Pose((q['right_joint11'],q['right_joint17'],0.))},pose=lambda p:p),
            geo=NS(whole=NS(ignores=[])))
        self.observation=NS(sample=NS(world_from_cup=Pose((.3,.2,.1))),machine=Pose((.5,.4,.1)))
        self.data=dict(urdf_sha256=hashlib.sha256(self.urdf.read_bytes()).hexdigest(),parts=[{'geometry':1}],
            cup={'geometry':1},observation={'scene':{'shapes':[]}},table={'geometry':1},obstacles=[],
            native_identity={'control':dict(max_target_velocity_rad_s=2.,max_target_acceleration_rad_s2=5.)})
        self.cache={};self.builds=0;self.engines=[];self.now=0.
        self.envelope={'fixture':'selected-cup','version':1};self.envelope_paths=[]
        def prepare_cup(world,*,path=None):
            self.envelope_paths.append(path);return deepcopy(self.envelope)
        def advance(seconds):self.now+=seconds
        def build(data,urdf,movable,opened,**kwargs):
            self.builds+=1;advance(5)
            config,identity=robot_config(urdf,movable,opened)
            return config,dict(kinematics=identity,fixed_world_geometry={'locked_positions':{}},model=data['parts'])
        def meshes(*args):
            advance(2)
            return [dict(name='right',link='right_link11',vertices=[[0.,0.,0.]],faces=[[0,0,0]],
                         source_part_sha256='fixed',base_from_shape={})]
        def world(planner,cup,fixed):
            advance(1)
            poses=[dict(name='cup',geometry=self.data['cup'],base_from_shape={'position':cup.position}),
                   dict(name='machine',geometry=self.data['table'],base_from_shape={'position':self.observation.machine.position}),
                   *deepcopy(fixed)]
            return poses,['cup'],poses
        owner=self
        class Engine:
            def __init__(self,config,world,model_report,prepared_cup_envelope=None):
                advance(10);self.config=deepcopy(config);self.world=deepcopy(world);self.warmups=0
                self.receipts=[];self.locked_joint_updates=[];self.grasp_contact_certificate=None
                self.cup_contact=NS(model_report=model_report,inward_envelope=deepcopy(prepared_cup_envelope))
                self.detaches=self.seed_resets=0
                def detach(**kw):self.detaches+=1
                def reset():self.seed_resets+=1
                self.generator=NS(detach_object_from_robot=detach,reset_seed=reset)
                owner.engines.append(self)
            def warmup(self):advance(7);self.warmups+=1
            def update_world(self,world):advance(1);self.world=deepcopy(world)
            def update_locked_joints(self,values,*,update_right=False):
                advance(1);self.config['kinematics']['lock_joints']=dict(values)
                self.locked_joint_updates.append(dict(update_right=update_right,values=dict(values)))
        for replacement in (
            patch.object(planning.time,'perf_counter',lambda:self.now),
            patch.object(planning,'certificate_path',lambda name,urdf:root/name),
            patch('curobo_robot_model.build_conservative_robot_model',side_effect=build),
            patch('curobo_robot_model.fixed_world_mesh_descriptors',side_effect=meshes),
            patch.object(planning,'world_config',side_effect=world),
            patch('cup_envelope.prepare',side_effect=prepare_cup),
            patch('motiongen_motion.MotionGenerator',Engine)):
            replacement.start();self.addCleanup(replacement.stop)

    def prepare(self):
        return planning.prepare_model(self.planner,self.observation,self.data,self.urdf,self.opened,self.cache)

    def test_selected_envelope_preflight_and_cache_identity(self):
        self.planner.operator_assisted_cup_envelope=Path('/explicit/assisted-envelope.json')
        cold=self.prepare();engine=cold['engine']
        self.assertEqual(engine.cup_contact.inward_envelope,self.envelope)
        self.assertEqual(cold['cup_envelope'],self.envelope)
        self.assertEqual(self.envelope_paths[-1],self.planner.operator_assisted_cup_envelope)
        self.assertTrue(self.prepare()['reused'])
        self.envelope['version']=2
        changed=self.prepare()
        self.assertFalse(changed['reused']);self.assertIsNot(changed['engine'],engine)
        self.assertEqual(changed['engine'].cup_contact.inward_envelope,self.envelope)
        count=len(self.engines)
        with patch('cup_envelope.prepare',side_effect=ValueError('cup proof mismatch')):
            with self.assertRaisesRegex(ValueError,'cup proof mismatch'):self.prepare()
        self.assertEqual(len(self.engines),count)

    def test_scene_start_and_locked_pose_reuse_refreshes_all_bindings(self):
        cold=self.prepare();engine=cold['engine']
        self.assertEqual(cold['model_build_seconds'],7.)
        self.assertEqual(cold['cold_preparation_seconds'],26.)
        engine.grasp_contact_certificate={'stale':True};engine.receipts=[{'stale':True}]
        self.opened['left_joint1']+=.000001
        self.opened['right_joint11']+=.25
        self.opened['right_joint17']=self.opened['right_joint18']=.04
        self.opened['left_joint7']=self.opened['left_joint8']=.04
        self.observation.sample.world_from_cup=Pose((.1,.2,.3))
        self.observation.machine=Pose((.7,.4,.3))
        hot=self.prepare()
        self.assertTrue(hot['reused'] and hot['model_reused'])
        self.assertIs(hot['engine'],engine);self.assertEqual((self.builds,engine.warmups),(1,1))
        self.assertEqual(hot['cold_preparation_seconds'],0.)
        self.assertEqual(hot['model_build_seconds'],0.)
        self.assertEqual(hot['dynamic_update_seconds'],2.)
        self.assertEqual(engine.world[0]['base_from_shape']['position'],(.1,.2,.3))
        self.assertEqual(engine.world[1]['base_from_shape']['position'],(.7,.4,.3))
        self.assertEqual(hot['fixed_meshes'][0]['base_from_shape']['position'],(.35,.04,0.))
        self.assertEqual(hot['proxy']['fixed_world_geometry']['locked_positions']['right_joint11'],.35)
        self.assertEqual(engine.config['kinematics']['lock_joints']['left_joint7'],.04)
        self.assertEqual(engine.config['kinematics']['cspace']['retract_config'],[0.]*6)
        self.assertIsNone(engine.grasp_contact_certificate);self.assertEqual(engine.receipts,[])
        self.assertEqual((engine.detaches,engine.seed_resets),(1,1))
        self.assertTrue(engine.locked_joint_updates[-1]['update_right'])

    def test_asset_rules_control_and_topology_changes_invalidate(self):
        self.prepare()
        changes=[lambda:self.data['parts'][0].update(geometry=2),
            lambda:self.planner.identity.update(calibration='new'),
            lambda:(self.urdf.parent/CERTIFICATES[0]).write_text('{"version":2}'),
            lambda:self.data['native_identity']['control'].update(max_target_velocity_rad_s=1.),
            lambda:self.data['cup'].update(geometry=2),
            lambda:self.data['table'].update(geometry=2)]
        for change in changes:
            previous=self.cache['engine'];change();value=self.prepare()
            self.assertFalse(value['reused']);self.assertIsNot(previous,value['engine'])
        self.urdf.write_text(self.urdf.read_text().replace('upper="3"','upper="2"'))
        with self.assertRaisesRegex(ValueError,'URDF differs'):self.prepare()
        self.data['urdf_sha256']=hashlib.sha256(self.urdf.read_bytes()).hexdigest()
        self.assertFalse(self.prepare()['reused'])

    def test_cached_model_still_validates_joint_input(self):
        self.prepare();self.opened['right_joint17']+=.01
        with self.assertRaisesRegex(ValueError,'symmetric'):self.prepare()
        self.opened['right_joint18']=self.opened['right_joint17']
        self.opened['left_joint1']=float('nan')
        with self.assertRaisesRegex(ValueError,'finite'):self.prepare()


class LockedTransformsTest(unittest.TestCase):
    def test_pose_goals_use_current_fixed_links_and_actual_ee_target(self):
        observed={'left_link6':'actual start','right_link17':'new right tip','right_link18':'new other tip'}
        generator=NS(compute_kinematics=lambda state:NS(link_pose=observed))
        goals=current_link_goals(generator,object(),'left_link6','requested target')
        self.assertEqual(goals,dict(observed,left_link6='requested target'))
        self.assertEqual(observed['left_link6'],'actual start')

    def test_native_fk_only_update_preserves_topology_and_collision_inputs(self):
        original={'kinematics':dict(lock_joints={'right':.1},collision_spheres={'a':[1]},
            collision_link_names=['a'],extra_links={'a':{'parent_link_name':'left'}},cspace={'retract_config':[0.]})}
        light=locked_transform_config(original,{'right':.2})
        self.assertIsNone(light['kinematics']['collision_spheres'])
        self.assertEqual(original['kinematics']['collision_spheres'],{'a':[1]})
        self.assertEqual(light['kinematics']['extra_links'],original['kinematics']['extra_links'])
        current=NS(link_name_to_idx_map={'root':0,'right':1},joint_names=['left'],
            fixed_transforms=np.tile(np.eye(4),(2,1,1)),link_map=np.array([0,0]),joint_map=np.array([-1,0]),
            joint_map_type=np.array([-1,0]),joint_offset_map=np.array([1.,0.,1.,0.]))
        updated=deepcopy(current);updated.fixed_transforms[1,0,3]=.2
        torch=NS(equal=np.array_equal,isfinite=np.isfinite)
        self.assertIs(checked_locked_transforms(current,updated,torch),updated.fixed_transforms)
        updated.joint_map[1]=1
        with self.assertRaisesRegex(ValueError,'topology'):checked_locked_transforms(current,updated,torch)
        updated=deepcopy(current);updated.fixed_transforms[1,0,0]=float('nan')
        with self.assertRaisesRegex(ValueError,'topology'):checked_locked_transforms(current,updated,torch)


if __name__=='__main__':unittest.main()
