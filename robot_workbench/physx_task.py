"""Read-only native coffee acceptance for a resident Owner-driven fake plant.

Task failures latch evidence. This module has no actuator, stop or fault outlet.
It never resets the environment or borrows an earlier replay's success.
"""
from copy import deepcopy
from collections.abc import Sequence
from dataclasses import asdict, replace
import json
import math
from types import SimpleNamespace


def inference_tray_support(native, backend):
    """Main-thread sensor adapter: actual support only, no path/collision query.

    This diagnostic side channel is never part of the policy observation. A cup
    may be off-centre or still gripped; only current tray support and motion
    stability are measured here.
    """
    from robot_sim.tasks.manager_based.coffee_make.scene_instance import resolve_observed_machine_scene
    absolute=native.env._sim_step_counter
    transport=backend.plant.contact_status
    if transport.get('complete') is not True or transport.get('absolute_physics_step')!=absolute:
        raise ValueError('current tray contact transport is incomplete')
    machine=native.env.scene['coffee_machine'];cup=native.env.scene['cup']
    poses={path:native._pose(machine.data.body_pos_w[0,index].tolist(),machine.data.body_quat_w[0,index].tolist())
        for path,index in native._machine_body_indices}
    scene=resolve_observed_machine_scene(native.machine_profile,native.instance,poses,observation_id=str(absolute))
    spec=native.spec.acceptance
    return tray_support_sample(native.latest_contacts,cup_root=native.root+'/Cup',scene=scene,
        cup_velocity=cup.data.root_lin_vel_w[0].tolist(),cup_angular_velocity=cup.data.root_ang_vel_w[0].tolist(),
        dt=native.env.physics_dt,source_time=backend.physics_step/240.,source_clock_id=backend.source_clock_id,
        physics_step=backend.physics_step,absolute_physics_step=absolute,surface_tolerance_m=spec.surface_tolerance_m,
        normal_tilt_rad=spec.tilt_rad,minimum_force_n=spec.support_force_min_n,
        stable_speed_m_s=spec.stable_speed_m_s,stable_angular_speed_rad_s=spec.stable_angular_speed_rad_s)


def tray_support_sample(contacts, *, cup_root, scene, cup_velocity, cup_angular_velocity, dt,
                        source_time, source_clock_id, physics_step, absolute_physics_step,
                        surface_tolerance_m, normal_tilt_rad, minimum_force_n,
                        stable_speed_m_s, stable_angular_speed_rad_s):
    """Translate measured top-surface impulses to the common support evidence."""
    norm=lambda values:math.sqrt(sum(v*v for v in values))
    if not math.isfinite(dt) or dt<=0:raise ValueError('positive contact sample interval required')
    inverse=scene.world_from_support.inverse();force=0.;count=0
    for contact in contacts:
        if cup_root not in contact.actors:continue
        index=contact.actors.index(cup_root)
        if contact.shapes[1-index]!=scene.support_shape_path:continue
        normal=inverse.rotate(contact.normal);position=inverse.transform_point(contact.position)
        if abs(position[2])>surface_tolerance_m or abs(normal[2])<math.cos(normal_tilt_rad):continue
        force+=abs(inverse.rotate(contact.impulse)[2])/dt;count+=1
    speed,angular=norm(cup_velocity),norm(cup_angular_velocity)
    if not all(math.isfinite(v) for v in (force,speed,angular)):raise ValueError('nonfinite support readback')
    return dict(source_time=source_time,source_clock_id=source_clock_id,physics_step=physics_step,
        absolute_physics_step=absolute_physics_step,evidence_source='physx_actual_tray_contact_and_cup_velocity',
        tray_supported=count>0 and force>=minimum_force_n,stable=speed<=stable_speed_m_s and angular<=stable_angular_speed_rad_s,
        tray_force_n=force,tray_contacts=count,cup_speed_m_s=speed,cup_angular_speed_rad_s=angular,
        thresholds=dict(minimum_force_n=minimum_force_n,surface_tolerance_m=surface_tolerance_m,
            normal_tilt_rad=normal_tilt_rad,stable_speed_m_s=stable_speed_m_s,stable_angular_speed_rad_s=stable_angular_speed_rad_s))


def plant_target(target, fixture):
    """Reported command -> fixed plant coordinates, independent of fitted offsets."""
    result={}
    for i,side in enumerate(('left','right')):
        result[side]=[q+d for q,d in zip(target[side+'_arm_joints'],
            fixture['joint_offsets_rad'][i*6:i*6+6])]+[target[side+'_gripper']-fixture['metal_gap_offset_mm']]
    return result


def prepare_task(plan, source, plant_fixture):
    """Validate/freeze plan inputs off the realtime loop; no Isaac/CUDA import."""
    from integration import _digest
    from physical_motiongen import validate_evidence
    from robot_workbench.collection_physx import validate_plant_fixture
    if source.get('environment')!='fake' or source.get('simulation_only') is not True:
        raise ValueError('native task evidence is limited to explicit fake-device observations')
    fixture=validate_plant_fixture(plant_fixture,source)
    from cup_return_replay import is_replay, replay_model_plan
    model=replay_model_plan(plan,source) if is_replay(plan) else validate_evidence(plan,source)
    return dict(plan=deepcopy(plan),model_plan=model,fixture=fixture,
        source_digest=_digest(source),parent_timeline_digest=plan['timeline_digest'])


def prepare_task_execution(prepared,timeline):
    """Compile task-only evidence in the existing D compilation worker."""
    from execution_timeline import validate
    frozen=deepcopy(validate(timeline,prepared['plan']))
    return dict(source_timeline=timeline,timeline=frozen,
                targets=[plant_target(row['target'],prepared['fixture']) for row in frozen['commands']])


def prepare_isolated_task(plan,source,fixture,timeline):
    """Run every full task check in the compiler; return only bounded native policy."""
    from isaac_lane import prepare_motiongen_replay_policy
    prepared=prepare_task(plan,source,fixture)
    prepare_task_execution(prepared,timeline)
    model=prepared['model_plan']
    compact={key:deepcopy(model[key]) for key in ('timeline_digest','cup_contact_preset','gripper_opening_mm') if key in model}
    if model.get('cup_contact_preset') and 'replay_derivation' not in model:
        compact['planning_validation']={'gpu_final_acceptance':{'closing':{'end_m':model['planning_validation']['gpu_final_acceptance']['closing']['end_m']}}}
    if 'replay_derivation' in model:
        proof=model['replay_derivation']
        compact['replay_derivation']={key:deepcopy(proof[key]) for key in ('return_goal','goal_basis','parent_native_receipt_digest',
            'collision_check','historical_release_contact_model_gap_mm')}
        compact['replay_derivation']['parent_native_receipt']={'physical_configuration_sha256':proof['parent_native_receipt']['physical_configuration_sha256']}
    else:compact['prepared_policy']=prepare_motiongen_replay_policy(model)
    return dict(prepared=dict(model_plan=compact,fixture=prepared['fixture'],source_digest=prepared['source_digest'],
        parent_timeline_digest=prepared['parent_timeline_digest']),source_digest=prepared['source_digest'],error=None)


class PlantTargets(Sequence):
    """A fixed-size per-sample conversion over sealed Owner rows, with no full copy."""
    def __init__(self,rows,fixture):self.rows,self.fixture=rows,fixture
    def __len__(self):return len(self.rows)
    def __getitem__(self,index):return plant_target(self.rows[index]['target'],self.fixture)


def return_sample_audit(evaluator, sample, scene, contacts, goal, *, dt, opening):
    """Actual contacts and native tolerances; no mesh query or coffee outlet goal."""
    from isaac_lane import tray_placement_region
    a=evaluator.spec;pose=sample.world_from_cup;inverse=pose.inverse()
    norm=lambda values:math.sqrt(sum(v*v for v in values))
    bottom_z=evaluator._lowest_z(pose)
    upright=pose.rotate((0,0,1))[2]>=math.cos(a.tilt_rad)
    on_goal_plane=abs(bottom_z-goal['position'][2])<=a.surface_tolerance_m
    table_force=tray_force=0.;finger_forces=dict(left_link7=0.,left_link8=0.)
    finger_counts=dict.fromkeys(finger_forces,0);failures=[]
    machine_shapes={shape.shape_path for shape in scene.shapes}
    supported=on_goal_plane and upright
    opened=all(abs(q-a.open_position_m)<=a.open_tolerance_m for q in sample.jaw_positions)
    for contact in contacts:
        if contact.separation_m>0 and not any(contact.impulse):continue
        if evaluator.cup_root not in contact.actors:
            if any(actor.startswith(evaluator.robot_root+'/') for actor in contact.actors):
                failures.append('robot_forbidden_contact')
            continue
        ci=contact.actors.index(evaluator.cup_root);other=contact.actors[1-ci];shape=contact.shapes[1-ci]
        force=norm(contact.impulse)/dt;local=inverse.transform_point(contact.position);normal=inverse.rotate(contact.normal)
        bottom=abs(local[2])<=a.surface_tolerance_m and abs(normal[2])>=math.cos(a.tilt_rad)
        if shape==evaluator.table_shape:
            if not (bottom and supported and abs(contact.position[2]-goal['position'][2])<=a.surface_tolerance_m
                    and force<=a.support_force_max_n):
                failures.append('cup_table_invalid_support')
            table_force+=abs(contact.impulse[2])/dt
        elif shape in machine_shapes:
            support_inverse=scene.world_from_support.inverse()
            legal_tray=(shape==scene.support_shape_path and bottom and tray_placement_region(evaluator,sample,scene)
                and abs(support_inverse.transform_point(contact.position)[2])<=a.surface_tolerance_m
                and abs(support_inverse.rotate(contact.normal)[2])>=math.cos(a.tilt_rad))
            if legal_tray:tray_force+=force
            elif shape==scene.support_shape_path or max(0.,-contact.separation_m)>getattr(a,'cup_machine_contact_allowance_m',0.):
                failures.append('cup_machine_forbidden_contact')
        elif other in {evaluator.robot_root+'/'+link for link in finger_forces}:
            link=other.rsplit('/',1)[-1];finger_forces[link]+=force;finger_counts[link]+=1
            tilt=(evaluator.cup.sidewall_normal_limit_rad(local[2])
                  if any(b.radius_start_m is not None for b in evaluator.cup.bands) else a.tilt_rad)
            if (evaluator.history['release_confirmed'] or shape!=other+'/RealFingerRubberPad'
                    or not evaluator.cup.candidate_grasp_band_m[0]<=local[2]<=evaluator.cup.candidate_grasp_band_m[1]
                    or abs(normal[2])>math.sin(tilt)
                    or not a.finger_contact_open_state_allowed(grasp_confirmed=evaluator.history['grasp_confirmed'],
                        landing=supported,opened=opened,opening=opening,force=force)):
                failures.append('finger_cup_forbidden_contact')
        else:failures.append('cup_unknown_contact')
    if max(table_force,tray_force)>a.support_force_max_n or any(f>a.finger_force_max_n for f in finger_forces.values()):
        failures.append('contact_force_limit')
    if (opening or opened) and any(f>a.residual_force_max_n for f in finger_forces.values()):
        failures.append('release_force_limit')
    # Match the accepted supported-unload policy: transient landing speeds do
    # not reject legal support. Stable final speed is still independently required.
    if (not supported and finger_counts and any(finger_counts.values())
            and sample.finger_cup_relative_speed_m_s>a.finger_contact_speed_limit(evaluator.history['grasp_confirmed'])):
        failures.append('finger_contact_speed_limit')
    return dict(physics_step=sample.physics_step,failures=failures,table_force_n=table_force,tray_force_n=tray_force,
        finger_forces_n=finger_forces,finger_contact_counts=finger_counts,opened=opened,
        table_supported=supported and a.support_force_min_n<=table_force<=a.support_force_max_n,
        upright=upright,bottom_world_z_m=bottom_z,goal_xy_distance_m=math.dist(pose.position[:2],goal['position'][:2]),
        goal_tolerance_m=.01,cup_speed_m_s=norm(sample.cup_velocity),cup_angular_speed_rad_s=norm(sample.cup_angular_velocity),
        stable=norm(sample.cup_velocity)<=a.stable_speed_m_s and norm(sample.cup_angular_velocity)<=a.stable_angular_speed_rad_s,
        hand_clear=sample.hand_clearance_m>=a.hand_clearance_m and not any(finger_counts.values()),
        home=max(abs(q-home) for q,home in zip(sample.arm_positions,evaluator.home_positions,strict=True))<=a.home_position_rad
            and max(map(abs,sample.arm_velocities))<=a.home_speed_rad_s)


class CoffeeTaskMonitor:
    """Create before the first compiled Owner command reaches physics.

    Call observe_current once after every completed backend.advance, and finish
    immediately at the Owner's final feedback endpoint. Construction validates
    the compiled timeline once; it does not change the native loop/action manager.
    """
    def __init__(self,native,backend,prepared,execution_timeline,*,command_epoch,runtime_nonce,prepared_execution=None):
        from integration import _digest
        from isaac_lane import install_grasp_contact_adapter,install_motiongen_replay_policy,SimDistanceActionSpec
        from robot_workbench.collection_physx import validate_plant_fixture
        fixture=prepared['fixture']
        validate_plant_fixture(fixture,{'clock':{'source_clock_id':fixture['source_clock_id']}},backend)
        if getattr(backend.plant,'env',None) is not native.env or getattr(native.env,'_m2_runtime',None) is not native:
            raise ValueError('task readback and Owner backend must share the very same native plant')
        if backend.owner is None or getattr(backend.owner,'environment',None)!='fake' or backend.owner.hardware:
            raise ValueError('native task monitor requires the actual fake CollectionOwner')
        if (type(command_epoch) is not int or not isinstance(runtime_nonce,str) or not runtime_nonce
                or backend.owner.epoch!=command_epoch or backend.owner.runtime_nonce!=runtime_nonce):
            raise ValueError('current execution owner identity required')
        instance=getattr(backend.owner,'execution_instance_id',None)
        if not isinstance(instance,str) or not instance:
            raise ValueError('current Owner acceptance instance required')
        compiled=prepared_execution or prepare_task_execution(prepared,execution_timeline)
        if compiled['source_timeline'] is not execution_timeline:
            raise ValueError('task compilation does not belong to this exact Owner timeline')
        timeline=compiled['timeline']
        model=prepared['model_plan']
        self.return_goal=deepcopy(model.get('replay_derivation',{}).get('return_goal'))
        if native.physical_identity.get('cup_contact_preset')!=model.get('cup_contact_preset'):
            raise ValueError('native cup contact preset differs from the frozen plan')
        if timeline['commands'][-1]['stage']!='home_hold':
            raise ValueError('native task qualification requires the full frozen home endpoint')
        self.native,self.backend=native,backend
        self.plant=backend.plant
        self.clock_id=fixture['source_clock_id']
        self.fixture=deepcopy(fixture)
        self.timeline=timeline
        self.targets=compiled['targets']
        self.epoch,self.nonce=command_epoch,runtime_nonce
        self.instance=instance
        self.execution_error_settings=deepcopy(getattr(backend.owner,'execution_error_settings',None))
        self.execution_id=timeline['timeline_digest']
        self.start_step=self.last_step=backend.physics_step
        self.absolute_offset=native.env._sim_step_counter-backend.physics_step
        self.last_tick=-1;self.final_step=None;self.last_context=None
        self.finished=False;self.outcome='running';self.first_failure=None
        self.samples=0;self.last_evidence=None;self.initialization_samples=0
        self.physical_identity=deepcopy(native.physical_identity)
        sample,scene=native.read(native.env._sim_step_counter)
        self.initial_observation=asdict(sample)
        previous=native.evaluator
        evaluator=type(previous)(native.spec.acceptance,native.cup_profile,
            cup_root=previous.cup_root,robot_root=previous.robot_root,table_shape=previous.table_shape,
            machine_profile_sha256=previous.machine_profile_sha256,
            diagnostic_relax_guards=False,diagnostic_observe_only=False)
        evaluator.home_positions=tuple(self.targets[-1]['left'][:6]+self.targets[-1]['right'][:6])
        evaluator.reset(sample)
        self.runtime=SimpleNamespace(evaluator=evaluator,root=native.root,
            loop=SimpleNamespace(physics_steps=sample.physics_step,terminal=False),
            spec=native.spec,physical_identity=self.physical_identity,
            physical_sha256=_digest(self.physical_identity))
        if self.return_goal is not None:
            proof=model['replay_derivation']
            if self.runtime.physical_sha256!=proof['parent_native_receipt']['physical_configuration_sha256']:
                raise ValueError('return replay current plant configuration differs from completed parent')
            base=evaluator.spec
            evaluator.spec=replace(base,release_timeout_s=base.release_timeout_s+1.5)
            configuration=dict(schema_version='owner_cup_return_policy.v1',return_goal=deepcopy(self.return_goal),
                goal_basis=proof['goal_basis'],goal_xy_tolerance_m=.01,base_acceptance=asdict(base),
                effective_acceptance=asdict(evaluator.spec),parent_native_receipt_digest=proof['parent_native_receipt_digest'],
                collision_check=deepcopy(proof['collision_check']),final_goal='current table support, release, stability and home',
                historical_release_contact_model_gap_mm=proof['historical_release_contact_model_gap_mm'])
            self.policy=dict(configuration=configuration,configuration_sha256=_digest(configuration),
                progress=dict(last_sample_native_success=False,final_native_success=False))
            evaluator.observe=self._observe_return
            release_gap=proof['historical_release_contact_model_gap_mm'] if model.get('cup_contact_preset') else None
        else:
            install_grasp_contact_adapter(self.runtime,diagnostic_compression=bool(model.get('cup_contact_preset')))
            self.policy=install_motiongen_replay_policy(self.runtime,model,applied_context=self._policy_context,
                prepared_policy=model.get('prepared_policy'))
            release_gap=(model['planning_validation']['gpu_final_acceptance']['closing']['end_m']*2000
                         if model.get('cup_contact_preset') else None)
        adapter=install_grasp_contact_adapter(self.runtime,diagnostic_compression=bool(model.get('cup_contact_preset')))
        if self.return_goal is not None:
            self.policy['incidental_contacts']=adapter['incidental_contacts']
            self.policy['configuration']['rubber_pad_body_contact']={k:v for k,v in adapter.items() if k!='incidental_contacts'}
            self.policy['configuration_sha256']=_digest(self.policy['configuration'])
        self.intent=SimDistanceActionSpec(None,model['gripper_opening_mm'],release_contact_mm=release_gap)
        self.identity=dict(environment='fake',simulation_only=True,source_clock_id=self.clock_id,
            execution_instance_id=instance,
            source_digest=prepared['source_digest'],parent_timeline_digest=prepared['parent_timeline_digest'],
            execution_timeline_digest=self.execution_id,plant_fixture_digest=_digest(fixture),
            physical_configuration_sha256=self.runtime.physical_sha256,
            command_epoch=command_epoch,runtime_nonce=runtime_nonce,
            home_positions_model_rad=list(evaluator.home_positions),
            execution_geometry_collision_check=dict(status='not_run',reason='planning_phase_only'),
            initialization_scope='actual contacts and poses; no parent source tick or geometry certification')
        if getattr(native,'instance',None) is not None:
            self.identity['actual_environment_from_machine_root']=json.loads(json.dumps(asdict(native.instance.environment_from_machine_root)))
        if self.return_goal is not None:
            self.identity.update(task_kind='cup_return',execution_geometry_collision_check=deepcopy(proof['collision_check']))

    def _observe_return(self,sample,scene,contacts,*,dt,complete,commanded_gripper_open=None):
        e=self.runtime.evaluator;a=e.spec;e._current_physics_step=sample.physics_step
        if (not complete or not math.isfinite(dt) or dt<=0 or sample.physics_step!=e.previous.physics_step+1
                or sample.episode_id!=e.previous.episode_id or scene.instance.episode_id!=sample.episode_id
                or scene.observation_id!=str(sample.physics_step) or scene.profile_sha256!=e.machine_profile_sha256):
            return e.fail('stale_physics_or_geometry',status='unverifiable')
        opening=(all(v>=-1e-5 for v in sample.jaw_velocities) and any(v>1e-5 for v in sample.jaw_velocities)
                 if commanded_gripper_open is None else commanded_gripper_open)
        audit=return_sample_audit(e,sample,scene,contacts,self.return_goal,dt=dt,opening=opening)
        e.last_audit=audit;progress=self.policy['progress'];progress['last_observed_step']=sample.physics_step
        if audit['failures']:
            reason=audit['failures'][0]
            return e.fail(reason,status='unverifiable' if reason=='cup_unknown_contact' else 'completed')
        forces=audit['finger_forces_n'];counts=audit['finger_contact_counts']
        e.force_window.append((dt,forces.copy()));e.force_window_duration+=dt
        while e.force_window_duration>a.stable_window_s+1e-12:
            duration,values=e.force_window.popleft();removed=min(duration,e.force_window_duration-a.stable_window_s)
            e.force_window_duration-=removed
            if duration-removed>1e-12:e.force_window.appendleft((duration-removed,values))
        means={link:sum(duration*values[link] for duration,values in e.force_window)/e.force_window_duration for link in forces}
        grip=(e.force_window_duration+1e-12>=a.stable_window_s and all(counts.values())
            and all(a.finger_force_min_n<=force<=a.finger_force_max_n for force in means.values()))
        e.history['grasp_confirmed']|=grip and not audit['opened']
        on_support=audit['table_supported'] or audit['tray_force_n']>0
        if grip and (e.grasp_transform is None or on_support):
            e.grasp_transform=sample.world_from_tool.inverse().compose(sample.world_from_cup)
        if e.history['grasp_confirmed'] and not e.history['release_confirmed'] and not on_support:
            if not grip:return e.fail('grasp_lost')
            delta=e.grasp_transform.inverse().compose(sample.world_from_tool.inverse().compose(sample.world_from_cup))
            if math.sqrt(sum(v*v for v in delta.position))>a.slip_m or 2*math.acos(min(1.,abs(delta.orientation_wxyz[0])))>a.slip_rad:
                return e.fail('grasp_slip')
        at_goal=audit['table_supported'] and audit['stable'] and audit['goal_xy_distance_m']<=audit['goal_tolerance_m']
        release=at_goal and audit['opened'] and not any(counts.values()) and e.history['grasp_confirmed']
        e.history['release_confirmed']|=e._window('release',release,dt,a.release_window_s)
        if audit['table_supported'] and (opening or audit['opened']):e.release_elapsed+=dt
        if e.release_elapsed>a.release_timeout_s and not e.history['release_confirmed']:return e.fail('release_timeout')
        if e.history['release_confirmed'] and not at_goal:return e.fail('released_cup_unstable')
        qualified=at_goal and e.history['release_confirmed'] and audit['hand_clear'] and audit['home']
        progress['last_sample_native_success']=e._window('home',qualified,dt,a.stable_window_s)
        progress.update(grasp_observed=e.history['grasp_confirmed'],release_confirmed=e.history['release_confirmed'],
            return_goal=deepcopy(audit),last_observed_source_tick=self.last_context.get('source_tick'))
        e.previous=sample
        return None

    def _policy_context(self,sample):
        if self.last_context is None:raise ValueError('no actually applied Owner command')
        return dict(self.last_context,final_sample=sample.physics_step==self.final_step)

    def _applied(self):
        backend=self.backend
        left,right=backend.applied_contexts['left'],backend.applied_contexts['right']
        if not isinstance(left,dict) or left!=right:
            raise ValueError('both plant sides require the same actually applied Owner command')
        if (left.get('execution_id')!=self.execution_id or left.get('command_epoch')!=self.epoch
                or left.get('execution_instance_id')!=self.instance
                or getattr(backend.owner,'execution_instance_id',None)!=self.instance
                or left.get('runtime_nonce')!=self.nonce or backend.owner.epoch!=self.epoch
                or backend.owner.runtime_nonce!=self.nonce):
            raise ValueError('stale or mismatched applied Owner identity')
        tick=left.get('execution_tick')
        if type(tick) is not int or not 0<=tick<len(self.targets) or tick not in (self.last_tick,self.last_tick+1):
            raise ValueError('applied command skipped, regressed, or lies outside the compiled execution')
        row=self.timeline['commands'][tick]
        if (left.get('source_tick')!=row['source_tick'] or left.get('stage')!=row['stage']
                or row['source_tick'] is None and row['stage']!='initialize'):
            raise ValueError('applied stage differs from the actual compiled command')
        for side in ('left','right'):
            if any(abs(a-b)>1e-9 for a,b in zip(backend.active[side],self.targets[tick][side])):
                raise ValueError('applied plant target differs from fixed-truth command mapping')
        self.last_tick=tick;self.last_context=deepcopy(left)
        return row

    def _latch(self,reason,*,outcome='unverifiable',detail=None):
        if self.first_failure is None:
            self.outcome=outcome
            self.first_failure=dict(reason=str(reason),detail=detail,physics_step=self.backend.physics_step,
                absolute_physics_step=self.native.env._sim_step_counter,applied_context=deepcopy(self.last_context),
                contact_status=deepcopy(getattr(self.backend.plant,'contact_status',None)),last_observation=deepcopy(self.last_evidence))
            sample=(self.last_evidence or {}).get('sample',{})
            if 'finger_cup_relative_speed_m_s' in sample:
                e=self.runtime.evaluator
                self.first_failure['contact_speed_m_s']=sample['finger_cup_relative_speed_m_s']
                self.first_failure['contact_speed_limit_m_s']=e.spec.finger_contact_speed_limit(e.history.get('grasp_confirmed',False))

    def observe_current(self):
        """Errors invalidate task evidence, never propagate into the motion loop."""
        if self.finished or self.first_failure is not None:return self.outcome
        try:
            backend=self.backend;absolute=self.native.env._sim_step_counter
            if (backend.source_clock_id!=self.clock_id or backend.physics_step!=self.last_step+1
                    or absolute!=backend.physics_step+self.absolute_offset
                    or backend.plant is not self.plant or backend.plant.env is not self.native.env
                    or list(backend.offsets)!=self.fixture['joint_offsets_rad']
                    or backend.gap_offset!=self.fixture['metal_gap_offset_mm']
                    or self.native.physical_identity!=self.physical_identity):
                raise ValueError('plant identity, physical configuration or physics sequence changed')
            row=self._applied()
            contacts=self.backend.plant.contact_status
            if contacts.get('complete') is not True or contacts.get('absolute_physics_step')!=absolute:
                raise ValueError('current native contact transport incomplete')
            sample,scene=self.native.read(absolute)
            if sample.physics_step!=absolute:raise ValueError('native sample step differs from plant')
            self.runtime.loop.physics_steps=absolute
            self.last_step=backend.physics_step
            self.samples+=1;self.initialization_samples+=row['source_tick'] is None
            commanded_open=None
            if row['source_tick'] is not None:
                self.intent.set_stage(row['stage'])
                target=self.backend.active
                command=[*target['left'][:6],target['left'][6],*target['right'][:6],target['right'][6]]
                commanded_open=self.intent._binary(command)[6]>0
            self.last_evidence=dict(sample=asdict(sample),contacts=[asdict(c) for c in self.native.latest_contacts],
                folded_self_collision=deepcopy(self.identity['execution_geometry_collision_check']),
                commanded_gripper_open=commanded_open,
                command_intent_basis='measured_jaw_motion' if commanded_open is None else 'actually_applied_stage_and_gap')
            evaluator=self.runtime.evaluator
            # The frozen plan already checked geometry. Actual PhysX contacts
            # still qualify the grasp, support, release and task outcome.
            event=evaluator.observe(sample,scene,self.native.latest_contacts,dt=self.native.env.physics_dt,
                complete=True,commanded_gripper_open=commanded_open)
            self.last_evidence['native_audit']=deepcopy(evaluator.last_audit)
            if event is not None and event.reason!='success':
                self._latch(event.reason,outcome='unverifiable' if event.run_status=='unverifiable' else 'failed',
                    detail='native StepEvent.run_status='+event.run_status)
        except Exception as error:
            self._latch('task_observation_unverifiable',detail=f'{type(error).__name__}: {error}')
        return self.outcome

    def finish(self,motion_outcome,final_feedback,*,data_outcome='unknown'):
        """Final success is this Owner endpoint's current native success only."""
        if self.finished:return deepcopy(self.result)
        if self.first_failure is None:
            if motion_outcome!='completed':
                self.outcome='interrupted'
            else:
                try:
                    self._applied()
                    if (final_feedback.get('source_clock_id')!=self.clock_id
                            or final_feedback.get('physics_step')!=self.last_step
                            or final_feedback.get('source_time_ns')!=self.last_step*1_000_000_000//240
                            or self.backend.physics_step!=self.last_step
                            or self.last_tick!=len(self.targets)-1):
                        raise ValueError('motion completion is not this full execution native observation endpoint')
                    self.final_step=self.last_step+self.absolute_offset
                    if self.policy['progress'].get('last_sample_native_success') is not True:
                        self._latch('final_native_task_not_successful',outcome='failed')
                    else:
                        if self.return_goal is not None:
                            from integration import _digest
                            self.policy['configuration']['actual_final_physics_step']=self.final_step
                            self.policy['configuration_sha256']=_digest(self.policy['configuration'])
                            self.policy['progress']['final_native_success']=True
                        else:
                            from isaac_lane import finish_motiongen_replay_policy
                            finish_motiongen_replay_policy(self.runtime)
                        self.outcome='success'
                except Exception as error:
                    self._latch('final_task_not_qualified',detail=f'{type(error).__name__}: {error}')
        # A latched process failure must not make its old pose look like the
        # final placement. Read the actual endpoint once, without re-evaluating
        # or upgrading the failed task and without adding work per physics step.
        endpoint={'status':'unavailable'}
        try:
            step=self.backend.physics_step;absolute=self.native.env._sim_step_counter
            owner=self.backend.owner
            if (owner.execution_instance_id!=self.instance or owner.runtime_nonce!=self.nonce
                    or self.backend.source_clock_id!=self.clock_id
                    or absolute!=step+self.absolute_offset
                    or final_feedback.get('source_clock_id')!=self.clock_id
                    or final_feedback.get('physics_step')!=step
                    or final_feedback.get('source_time_ns')!=step*1_000_000_000//240):
                raise ValueError('final pose does not match this execution feedback endpoint')
            sample,_=self.native.read(absolute)
            if sample.physics_step!=absolute:raise ValueError('final native pose is stale')
            endpoint=dict(status='observed',physics_step=step,absolute_physics_step=absolute,
                sample=asdict(sample),contact_status=deepcopy(self.backend.plant.contact_status),
                world_displacement_from_start_m=[a-b for a,b in zip(sample.world_from_cup.position,
                    self.initial_observation['world_from_cup']['position'],strict=True)],
                task_qualification_unchanged=True)
            instance=getattr(self.native,'instance',None)
            if getattr(instance,'world_from_environment',None) is not None:
                pose=instance.world_from_environment.inverse().compose(sample.world_from_cup)
                endpoint['environment_from_cup']=asdict(pose)
                if self.return_goal is not None:
                    endpoint['return_goal_error_m']=[a-b for a,b in zip(pose.position,self.return_goal['position'],strict=True)]
                    endpoint['return_goal_xy_error_m']=math.dist(pose.position[:2],self.return_goal['position'][:2])
        except Exception as error:endpoint=dict(status='unavailable',reason=str(error))
        self.finished=True
        self.result=dict(schema_version='owner_physx_native_task.v1',**self.identity,
            motion_outcome=motion_outcome,data_outcome=data_outcome,task_outcome=self.outcome,
            task_qualified=self.outcome=='success',hardware_qualified=False,
            start_physics_step=self.start_step,last_physics_step=self.last_step,
            observed_physics_samples=self.samples,initialization_samples=self.initialization_samples,
            final_applied_execution_tick=self.last_tick,first_failure=deepcopy(self.first_failure),
            last_observation=deepcopy(self.last_evidence),policy=deepcopy(self.policy),
            initial_observation=deepcopy(self.initial_observation),endpoint_observation=endpoint,
            execution_error_settings=deepcopy(self.execution_error_settings),
            fake_actuation_error=(self.backend.error_model_evidence() if callable(getattr(self.backend,'error_model_evidence',None)) else None),
            motion_policy='task outcome is observed only; Owner control guards are unchanged')
        return deepcopy(self.result)
