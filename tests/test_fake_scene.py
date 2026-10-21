"""CPU contracts for native scene edits; no simulator, GPU or motion commands."""
from copy import deepcopy
from dataclasses import asdict,dataclass
import hashlib
import json
from pathlib import Path
import sys
import threading
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock,patch
import numpy as np

ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'rule_based_workbench')]
from robot_workbench.fake_scene import FakeScene


@dataclass(frozen=True)
class Pose:
    """Axis-aligned CPU fixture; native RigidTransform owns rotation math."""
    position:tuple
    orientation_wxyz:tuple=(1.,0.,0.,0.)
    def compose(self,other):return Pose(tuple(a+b for a,b in zip(self.position,other.position)))
    def inverse(self):return Pose(tuple(-v for v in self.position))


@dataclass(frozen=True)
class Episode:
    environment_from_machine_root:Pose
    environment_from_cup:Pose
    episode_id:str='fixture'
    seed:int=123
    machine_asset:str='unchanged-machine'
    def digest(self):return hashlib.sha256(json.dumps(asdict(self),sort_keys=True).encode()).hexdigest()


class FakeSceneTest(unittest.TestCase):
    def test_restore_accepts_quaternion_norm_noise_before_native_parsing(self):
        import math
        from robot_workbench.fake_scene import normalize_restored_scene,normalize_restored_orientations
        for scale in (1.0000000298600251,.999,1.001):
            raw=asdict(self.episode);raw['environment_from_machine_root']['orientation_wxyz']=[scale,0.,0.,0.]
            result,corrections=normalize_restored_scene(raw)
            self.assertEqual(result['environment_from_machine_root']['orientation_wxyz'],[1.,0.,0.,0.])
            self.assertEqual(raw['environment_from_machine_root']['orientation_wxyz'][0],scale)
            self.assertEqual(corrections[0]['original_norm'],scale)
        for quaternion in ([1.00101,0.,0.,0.],[0.,0.,0.,0.],[float('nan'),0.,0.,0.]):
            raw=asdict(self.episode);raw['environment_from_cup']['orientation_wxyz']=quaternion
            with self.assertRaises(ValueError):normalize_restored_scene(raw)
        raw=Episode(Pose((.4,.3,.2),(0.7109540700912476,0.,0.,-0.7032384872436523)),self.episode.environment_from_cup)
        fixed,corrections=normalize_restored_orientations(raw)
        self.assertAlmostEqual(math.sqrt(sum(x*x for x in fixed.environment_from_machine_root.orientation_wxyz)),1.)
        self.assertEqual(fixed.environment_from_machine_root.position,raw.environment_from_machine_root.position)
        self.assertEqual(len(corrections),1)

    def test_snapshot_preserves_scene_and_uses_canonical_pose_arrays(self):
        self.native.physical_identity={'unchanged':'physics'}
        with patch.dict(sys.modules,{'robot_sim.asset_cfg.objects.coffee_machine':NS(RigidTransform=Pose)}):
            value=self.edit.snapshot()
        self.assertIsInstance(value['cup']['position'],list)
        self.assertIsInstance(value['environment_from_machine_root']['orientation_wxyz'],list)
        self.assertEqual(len(value['physical_configuration_sha256']),64)
        self.assertEqual(value['world_from_robot']['position'],[0.,0.,.5])
        self.reset.assert_not_called();self.env.sim.forward.assert_not_called()
        self.native._render.assert_not_called()

    def setUp(self):
        self.episode=Episode(Pose((.4,.3,.2)),Pose((.2,.1,.8)))
        def data(pose):return NS(root_pos_w=np.array([pose.position]),root_quat_w=np.array([pose.orientation_wxyz]))
        self.machine=data(self.episode.environment_from_machine_root);self.cup=data(self.episode.environment_from_cup)
        self.machine.default_root_state=np.array([[.4,.3,.2,1.,0.,0.,0.,0.,0.,0.,0.,0.,0.]])
        self.robot=data(Pose((0.,0.,.5)))
        self.robot.joint_pos=np.array([[.01]*16]);self.robot.joint_vel=np.zeros((1,16));self.robot.joint_pos_target=np.array([[.02]*16])
        class Scene(dict):
            def update(self,*,dt):self.last_update_dt=dt
        scene=Scene(robot=NS(data=self.robot),cup=NS(data=self.cup),coffee_machine=NS(data=self.machine))
        self.native=NS(episode=self.episode,absolute_step=240,_completed_readback='old',latest_contacts=('old',),
            instance=NS(),layout=NS(),_render=Mock())
        @dataclass(frozen=True)
        class Instance:
            environment_from_machine_root:Pose
            world_from_environment:Pose
            environment_from_robot:Pose=Pose((0.,0.,.5))
        @dataclass(frozen=True)
        class Layout:
            coffee_machine:Pose
            cup_start:Pose
        self.native.instance=Instance(self.episode.environment_from_machine_root,Pose((0.,0.,0.)))
        self.native.layout=Layout(self.episode.environment_from_machine_root,self.episode.environment_from_cup)
        self.native.read=Mock(side_effect=lambda step:(NS(world_from_cup=Pose(tuple(self.cup.root_pos_w[0]),tuple(self.cup.root_quat_w[0]))),
            NS(geometry_state='runtime_readback',digest=lambda:hashlib.sha256(self.machine.root_pos_w.tobytes()).hexdigest())))
        self.env=NS(scene=scene,_m2_runtime=self.native,_sim_step_counter=240,physics_dt=1/240,
            sim=NS(current_time=1.,current_time_step_index=240,forward=Mock()),extras={})
        self.state=dict(connected=True,automatic_enabled=False,motion_status='no_automatic_task',recording={'done':True},
            allowed_operations={'preview':{'allowed':True}})
        self.device=NS(environment='fake',runtime=NS(holder='collection',lock=threading.RLock(),owner=NS(lock=threading.RLock())),snapshot=lambda business:deepcopy(self.state),
            workflows={'collection':NS(snapshot=lambda:{'busy':False})},
            collection_preparation=NS(snapshot=lambda:{'busy':False,'status':'ready'},workers=object(),installed='unchanged-install'))
        self.application=NS(lock=threading.RLock(),jobs={('fake','collection'):{'action':'fake_scene_update','status':'running'}},
            invalidate_collection_preparation=Mock())
        self.events=[];self.event=lambda kind,**fields:self.events.append((kind,deepcopy(fields)))
        self.reset=Mock(side_effect=self.move)
        module='robot_sim.tasks.manager_based.coffee_make.episode_reset'
        modules=patch.dict(sys.modules,{module:NS(reset_episode_poses=self.reset)})
        modules.start();self.addCleanup(modules.stop)
        self.edit=FakeScene(self.env,self.episode,self.device,self.application,self.event)
        self.payload={'cup_xy_m':[.03,-.02],'machine_xy_m':[-.01,.04]}

    def move(self,env,episode,configured_episode):
        self.assertIs(env,self.env);self.assertEqual(configured_episode,self.episode)
        self.assertGreater(self.application.invalidate_collection_preparation.call_count,0,'plans are revoked before native writes')
        self.assertTrue(self.device.runtime.lock._is_owned());self.assertTrue(self.device.runtime.owner.lock._is_owned())
        self.machine.root_pos_w[0]=episode.environment_from_machine_root.position
        self.cup.root_pos_w[0]=episode.environment_from_cup.position

    def test_relative_to_initial_episode_keeps_robot_clock_workers_and_installation(self):
        worker=self.device.collection_preparation.workers
        for payload in (self.payload,{'cup_xy_m':[-.02,.01],'machine_xy_m':[.05,-.05]}, {'cup_xy_m':[0.,0.],'machine_xy_m':[0.,0.]}):
            before_robot=self.robot.joint_pos.copy()
            receipt=self.edit.apply(payload)
            self.assertEqual(receipt['status'],'applied');self.assertFalse(receipt['planner_input'])
            self.assertTrue(receipt['requires_new_observation'])
            self.assertEqual(receipt['before']['clock'],receipt['after']['clock']);self.assertEqual(receipt['before']['robot'],receipt['after']['robot'])
            self.assertEqual(self.env._sim_step_counter,240);np.testing.assert_array_equal(self.robot.joint_pos,before_robot)
            for name,field in (('cup','environment_from_cup'),('machine','environment_from_machine_root')):
                baseline=getattr(self.episode,field);offset=payload[name+'_xy_m']
                np.testing.assert_allclose(receipt['after'][name]['position'],[baseline.position[0]+offset[0],baseline.position[1]+offset[1],baseline.position[2]])
            self.assertEqual(self.native.instance.environment_from_machine_root,self.native.episode.environment_from_machine_root)
            self.assertEqual(self.env.extras['requested_scene_sha256'],self.native.episode.digest())
            self.assertIsNone(self.native._completed_readback);self.assertEqual(self.native.latest_contacts,())
            json.dumps(receipt,allow_nan=False)
        self.assertEqual(self.native.episode,self.episode,'zero offsets restore only the scene to its original definition')
        self.assertIs(self.device.collection_preparation.workers,worker);self.assertEqual(self.device.collection_preparation.installed,'unchanged-install')
        self.assertEqual(self.reset.call_count,3);self.assertEqual(self.native._render.call_count,3)
        self.assertEqual([event[0] for event in self.events],['fake_scene_update_requested','fake_scene_update_applied']*3)

    def test_invalid_offsets_and_busy_contexts_have_no_side_effects(self):
        for payload in ({},[],dict(self.payload,extra=1),dict(self.payload,cup_xy_m=[.051,0]),dict(self.payload,machine_xy_m=[0,float('nan')]),
                        dict(self.payload,cup_xy_m=[True,0]),dict(self.payload,cup_xy_m=[0,0,0]),dict(self.payload,cup_xy_m=[float('inf'),0])):
            with self.subTest(payload=payload),self.assertRaises(ValueError):self.edit.apply(payload)
        contexts=[patch.object(self.device,'environment','physical'),patch.dict(self.state,automatic_enabled=True),
            patch.dict(self.state,motion_status='RUNNING'),patch.dict(self.state,recording={'done':False}),
            patch.dict(self.state,allowed_operations={'preview':{'allowed':False,'reason':'control command still pending'}}),
            patch.object(self.device.workflows['collection'],'snapshot',lambda:{'busy':True}),
            patch.object(self.device.collection_preparation,'snapshot',lambda:{'busy':True}),
            patch.dict(self.application.jobs,{('fake','calibration'):{'action':'manual_preview','status':'running'}})]
        for context in contexts:
            with context,self.assertRaises(ValueError):self.edit.apply(self.payload)
        errors=[]
        def outside_main():
            try:self.edit.apply(self.payload)
            except Exception as error:errors.append(error)
        thread=threading.Thread(target=outside_main);thread.start();thread.join(1)
        self.assertEqual(len(errors),1);self.assertIsInstance(errors[0],RuntimeError)
        self.reset.assert_not_called();self.application.invalidate_collection_preparation.assert_not_called();self.assertEqual(self.events,[])

    def test_confirmation_queued_after_precheck_prevents_pose_write_and_render_is_unlocked(self):
        def queued(device):self.state['allowed_operations']['preview']={'allowed':False,'reason':'control command still pending'}
        self.application.invalidate_collection_preparation.side_effect=queued
        with self.assertRaisesRegex(ValueError,'control command still pending'):self.edit.apply(self.payload)
        self.reset.assert_not_called();self.native._render.assert_not_called()
        self.application.invalidate_collection_preparation.side_effect=None
        self.state['allowed_operations']['preview']={'allowed':True}
        def unlocked():
            self.assertFalse(self.device.runtime.lock._is_owned());self.assertFalse(self.device.runtime.owner.lock._is_owned())
        self.env.sim.forward.side_effect=unlocked;self.native._render.side_effect=unlocked
        self.edit.apply(self.payload)

    def test_partial_native_failure_is_audited_without_rollback_or_reexecution(self):
        def partial(env,episode,configured_episode):
            self.machine.root_pos_w[0]=episode.environment_from_machine_root.position
            raise RuntimeError('fixed anchor write failed')
        self.reset.side_effect=partial
        with self.assertRaisesRegex(RuntimeError,'fixed anchor'):self.edit.apply(self.payload)
        self.assertEqual(self.reset.call_count,1);self.assertEqual(self.events[-1][0],'fake_scene_update_failed')
        failure=self.events[-1][1]
        self.assertNotEqual(failure['before']['machine'],failure['after']['machine'])
        self.assertEqual(failure['before']['robot'],failure['after']['robot']);self.assertNotIn('robot_preserved',failure)
        self.native._render.assert_not_called()

    def test_readback_mismatch_rejects_success(self):
        for kind in ('cup','robot','clock'):
            with self.subTest(kind=kind):
                def wrong(env,episode,configured_episode):
                    self.move(env,episode,configured_episode)
                    if kind=='cup':self.cup.root_pos_w[0,0]+=.001
                    elif kind=='robot':self.robot.joint_pos[0,0]+=.001
                    else:self.env._sim_step_counter+=1
                self.reset.side_effect=wrong
                with self.assertRaises(RuntimeError):self.edit.apply(self.payload)
                self.assertEqual(self.events[-1][0],'fake_scene_update_failed')
        self.assertNotIn('fake_scene_update_applied',[row[0] for row in self.events])


if __name__=='__main__':unittest.main()
