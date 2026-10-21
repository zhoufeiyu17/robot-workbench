"""Explicit idle fake-scene edits. Receipts are audit data, never planner input."""
from copy import deepcopy
from dataclasses import asdict, replace
import json
import math
import threading


def restore_episode_poses(env,episode,base):
    """Restore object poses without applying a new table-start task evaluation."""
    from robot_sim.tasks.manager_based.coffee_make.episode_reset import reset_episode_poses
    episode,corrections=normalize_restored_orientations(episode)
    native=env._m2_runtime
    reset_episode_poses(env,episode,base)
    native.episode=episode
    native.instance=replace(native.instance,environment_from_machine_root=episode.environment_from_machine_root)
    native.layout=replace(native.layout,
        coffee_machine=replace(native.layout.coffee_machine,position=episode.environment_from_machine_root.position,
            orientation_wxyz=episode.environment_from_machine_root.orientation_wxyz),
        cup_start=replace(native.layout.cup_start,position=episode.environment_from_cup.position,
            orientation_wxyz=episode.environment_from_cup.orientation_wxyz))
    native._completed_readback=None;native.latest_contacts=()
    return episode,corrections


def normalize_restored_scene(scene):
    """Accept <=0.1% quaternion norm error before strict native pose parsing."""
    scene=deepcopy(scene);corrections=[]
    for key in ('environment_from_cup','environment_from_machine_root'):
        pose=scene[key];q=pose['orientation_wxyz']
        if len(q)!=4 or any(type(v) not in (int,float) or not math.isfinite(v) for v in q):
            raise ValueError('restored orientation requires four finite quaternion values')
        norm=math.sqrt(sum(v*v for v in q))
        if not math.isfinite(norm) or abs(norm-1)>.001+1e-12:
            raise ValueError('restored quaternion norm differs from one by more than 0.1%')
        normalized=tuple(v/norm for v in q)
        if tuple(q)!=normalized:
            corrections.append(dict(object=key,original=list(q),normalized=list(normalized),original_norm=norm,
                accepted_norm_error_fraction=.001,
                reason='measured quaternion scale normalization; orientation unchanged'))
        pose['orientation_wxyz']=list(normalized)
    return scene,corrections


def normalize_restored_orientations(episode):
    scene,corrections=normalize_restored_scene(asdict(episode))
    poses={key:replace(getattr(episode,key),orientation_wxyz=tuple(scene[key]['orientation_wxyz']))
        for key in ('environment_from_cup','environment_from_machine_root')}
    return replace(episode,**poses),corrections


class FakeScene:
    def __init__(self,env,base_episode,device,application,event):
        self.env,self.base_episode=env,deepcopy(base_episode)
        self.device,self.application,self.event=device,application,event

    def _idle(self):
        if threading.current_thread() is not threading.main_thread():
            raise RuntimeError('fake scene edits must run on the Isaac main thread')
        if self.device.environment!='fake':raise ValueError('scene editing is available only for fake devices')
        for workflow in self.device.workflows.values():
            if workflow.snapshot().get('busy'):raise ValueError('a business task is still active')
        preparation=self.device.collection_preparation
        if preparation is not None and preparation.snapshot().get('busy'):
            raise ValueError('collection initialization is still active')
        with self.application.lock:
            for key,job in self.application.jobs.items():
                if key[0]!=self.device.environment or job.get('status')!='running':continue
                if key==(self.device.environment,'collection') and job.get('action') in ('fake_scene_update','return_cup'):continue
                raise ValueError('another device request is still active')
        runtime=self.device.runtime
        state=self.device.snapshot(runtime.holder if runtime else 'collection')
        if state.get('automatic_enabled') or state.get('motion_status') not in (None,'no_automatic_task','READY','STOPPED','FAULT'):
            raise ValueError('device motion is still active')
        if (state.get('recording') or {}).get('done') is False:raise ValueError('recording has not finished')
        gate=state.get('allowed_operations',{}).get('preview',{})
        if gate.get('allowed') is not True:raise ValueError(gate.get('reason') or 'Owner idle state is not confirmed')
        return runtime

    def _read(self):
        env=self.env;native=env._m2_runtime
        sample,scene=native.read(env._sim_step_counter)
        robot=env.scene['robot'].data
        value=dict(clock=dict(physics_step=env._sim_step_counter,absolute_step=native.absolute_step,
                        simulation_time_s=env.sim.current_time,simulation_step=env.sim.current_time_step_index),
            robot={key:getattr(robot,key).tolist() for key in
                ('root_pos_w','root_quat_w','joint_pos','joint_vel','joint_pos_target')},
            cup=asdict(sample.world_from_cup),
            machine=dict(position=env.scene['coffee_machine'].data.root_pos_w[0].tolist(),
                         orientation_wxyz=env.scene['coffee_machine'].data.root_quat_w[0].tolist()),
            observed_scene_sha256=scene.digest(),observed_machine_geometry=scene.geometry_state)
        json.dumps(value,allow_nan=False)
        return value

    def snapshot(self):
        """Read current fake poses on the physics thread, without reset or rendering."""
        self._idle()
        from integration import _digest
        from robot_sim.asset_cfg.objects.coffee_machine import RigidTransform
        value=self._read();native=self.env._m2_runtime
        value['cup']=asdict(native.instance.world_from_environment.inverse().compose(RigidTransform(**value['cup'])))
        value['environment_from_machine_root']=asdict(native.instance.environment_from_machine_root)
        value['world_from_robot']=asdict(native.instance.world_from_environment.compose(native.instance.environment_from_robot))
        value['physical_configuration_sha256']=_digest(native.physical_identity)
        return json.loads(json.dumps(value,allow_nan=False))

    def apply(self,payload):
        if not isinstance(payload,dict) or set(payload)!={'cup_xy_m','machine_xy_m'}:
            raise ValueError('explicit cup_xy_m and machine_xy_m offsets are required')
        for key,value in payload.items():
            if (not isinstance(value,(list,tuple)) or len(value)!=2 or
                    any(type(v) not in (int,float) or not math.isfinite(v) or abs(v)>.05 for v in value)):
                raise ValueError(key+' must contain two finite offsets within ±0.05 m')
        runtime=self._idle()
        env=self.env;native=env._m2_runtime;base=self.base_episode
        def shifted(pose,xy):return replace(pose,position=(pose.position[0]+xy[0],pose.position[1]+xy[1],pose.position[2]))
        episode=replace(base,environment_from_cup=shifted(base.environment_from_cup,payload['cup_xy_m']),
            environment_from_machine_root=shifted(base.environment_from_machine_root,payload['machine_xy_m']))
        before=self._read()
        receipt=dict(environment='fake',simulation_only=True,planner_input=False,offsets=deepcopy(payload),
            previous_episode=asdict(native.episode),requested_episode=asdict(episode),
            previous_episode_sha256=native.episode.digest(),requested_episode_sha256=episode.digest(),before=before)
        json.dumps(receipt,allow_nan=False)
        self.event('fake_scene_update_requested',**receipt)
        self.application.invalidate_collection_preparation(self.device)
        try:
            # Keep the Runtime -> Owner lock order; do not hold either during
            # forwarding, readback or rendering. Queued confirmations lose this race.
            with runtime.lock,runtime.owner.lock:
                if self.device.runtime is not runtime or runtime.holder!='collection':
                    raise ValueError('collection control context changed before the scene edit')
                gate=self.device.snapshot('collection').get('allowed_operations',{}).get('preview',{})
                if gate.get('allowed') is not True:raise ValueError(gate.get('reason') or 'Owner is no longer idle')
                restore_episode_poses(env,episode,base)
            env.sim.forward();env.scene.update(dt=env.physics_dt)
            after=self._read()
            if after['clock']!=before['clock'] or after['robot']!=before['robot']:
                raise RuntimeError('scene edit changed robot state or simulation clock')
            origin=native.instance.world_from_environment
            initial=origin.compose(base.environment_from_machine_root)
            target=origin.compose(episode.environment_from_machine_root)
            baseline=env.scene['coffee_machine'].data.default_root_state[0].tolist()
            machine_root=type(base.environment_from_machine_root)(tuple(baseline[:3]),tuple(baseline[3:7]))
            expected=dict(cup=origin.compose(episode.environment_from_cup),
                machine=target.compose(initial.inverse()).compose(origin.compose(machine_root)))
            for name,pose in expected.items():
                actual=after[name]
                position_error=max(abs(a-b) for a,b in zip(actual['position'],pose.position,strict=True))
                dot=abs(sum(a*b for a,b in zip(actual['orientation_wxyz'],pose.orientation_wxyz,strict=True)))
                dot/=math.sqrt(sum(a*a for a in actual['orientation_wxyz'])*sum(b*b for b in pose.orientation_wxyz))
                angle=2*math.acos(min(1.,dot))
                if position_error>1e-4 or angle>1e-3:raise RuntimeError(name+' pose readback differs from the scene request')
            native._render()
            env.extras.update(requested_scene=asdict(episode),requested_scene_sha256=episode.digest(),
                layout=asdict(native.layout),scene_sha256=after['observed_scene_sha256'])
            receipt.update(status='applied',after=after,robot_preserved=True,clock_preserved=True,
                requires_new_observation=True)
            json.dumps(receipt,allow_nan=False)
            self.event('fake_scene_update_applied',**receipt)
            return receipt
        except Exception as error:
            try:receipt['after']=self._read()
            except Exception as read_error:receipt['readback_error']=str(read_error)
            self.event('fake_scene_update_failed',**receipt,error=str(error))
            raise
