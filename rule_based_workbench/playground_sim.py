"""Isolated Isaac playground: geometric planning, first frame, or unverified replay.

No robot SDK, physical service, or network client. Only replay advances the task
trajectory; reset still uses the existing native reset/settling implementation.
"""
import argparse
import faulthandler
from copy import deepcopy
from dataclasses import asdict, replace
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import time
import traceback
from types import SimpleNamespace

from integration import _digest, PIPELINE_LANES
from gripper_distance_plan import NATIVE_DISTANCE_ENCODING, validate_distance_control
from joint_space import convert_state, validate_initial_state, gripper_offset_mm
from isaac_lane import (apply_whitebox_materials, bind_effective_gripper_identity,
                        close_runtime, file_sha256, preview_sensors, write)


def reset_failure_context(env):
    """Preserve existing simulator evidence; never step, read sensors, or change acceptance."""
    runtime=getattr(env,'_m2_runtime',None)
    if runtime is None:return dict(status='unavailable',reason='runtime_not_created')
    try:
        evaluator=runtime.evaluator;contacts=runtime.latest_contacts;audit=deepcopy(evaluator.last_audit)
        value=dict(status='recorded',physics_step=audit.get('physics_step'),
            audit=audit,history=deepcopy(evaluator.history),
            contacts=[{key:getattr(contact,key) for key in
                ('actors','shapes','position','normal','impulse','separation_m')} for contact in contacts[:64]],
            contact_count=len(contacts),contacts_truncated=len(contacts)>64)
        value['maintenance_audit']=deepcopy(getattr(runtime,'maintenance_initial_audit',None))
        value['maintenance_audit_trace']=deepcopy(getattr(runtime,'maintenance_audit_trace',[]))
        event=getattr(evaluator,'event',None)
        value['task_event']=asdict(event) if event is not None else None
        json.dumps(value,allow_nan=False)
        return value
    except Exception as error:
        return dict(status='unavailable',reason='diagnostic_snapshot_failed',error=str(error))


def rectified_calibration(env,camera):
    calibration=deepcopy(env.cfg.d405_rig['device_calibrations'][camera])
    calibration['distortion']['coefficients']=[0.]*len(calibration['distortion']['coefficients'])
    return calibration


def camera_frame(env,camera):
    """Sample the raw overscan once at the original K, without distorting first."""
    from robot_sim.tasks.manager_based.coffee_make.d405_camera import color_observation
    calibration=rectified_calibration(env,camera)
    cache=getattr(env,'_playground_rectified_d405_cache',None)
    if cache is None:cache=env._playground_rectified_d405_cache={}
    identity=_digest(calibration)
    if cache.get(camera,{}).get('identity')!=identity:
        cache[camera]=dict(identity=identity,grids={})
    # Native color_observation owns the pixel-centre convention and grid_sample.
    # Give it a separate view/cache: never alter the real rig or distorted cache.
    view=SimpleNamespace(scene=env.scene,
        cfg=SimpleNamespace(d405_rig={'device_calibrations':{camera:calibration}}),
        _d405_grid_cache=cache[camera]['grids'])
    frame=color_observation(view,camera)[0].cpu().numpy()
    if frame.shape!=(calibration['height'],calibration['width'],3) or str(frame.dtype)!='uint8':
        raise ValueError('calibrated camera output shape or dtype differs')
    return frame


def sensor_pose(sensor):
    position=sensor.data.pos_w[0].cpu().tolist()
    quaternion=sensor.data.quat_w_ros[0].cpu().tolist()
    norm=math.sqrt(sum(v*v for v in quaternion))
    if not math.isfinite(norm) or abs(norm-1)>1e-5 or not all(math.isfinite(v) for v in position):
        raise ValueError('invalid actual optical camera pose')
    return dict(position=position,orientation_wxyz=[v/norm for v in quaternion])


def camera_metadata(env,sensors,source,world_from_environment):
    from robot_sim.asset_cfg.objects.coffee_machine import RigidTransform
    result={}
    for camera,sensor in sensors.items():
        source_calibration=env.cfg.d405_rig['device_calibrations'][camera]
        calibration=rectified_calibration(env,camera)
        k=calibration['intrinsics'];world_pose=sensor_pose(sensor)
        pose=world_from_environment.inverse().compose(RigidTransform(**world_pose))
        result[camera]=dict(image_domain='rectified',width=calibration['width'],height=calibration['height'],
            serial=calibration['serial'],K=[[k['fx'],0,k['cx']],[0,k['fy'],k['cy']],[0,0,1]],
            distortion_coefficients=list(calibration['distortion']['coefficients']),
            source_distortion=deepcopy(source_calibration['distortion']),
            rectification=dict(source_domain='raw_overscan_pinhole',output_domain='rectified',
                method='native sampling_map/grid_sample with copied zero coefficients',
                output_intrinsics='unchanged calibrated K',resampling_passes=1,
                source_calibration_sha256=_digest(source_calibration)),
            world_from_optical=world_pose,environment_from_optical=asdict(pose),
            overscan_K=sensor.data.intrinsic_matrices[0].cpu().tolist(),
            pose_source='actual Isaac sensor.data.pos_w / quat_w_ros')
    expected=RigidTransform(**source['calibration_profile']['frames']['environment_from_camera'])
    actual=RigidTransform(**result['base_0']['environment_from_optical'])
    position_error=math.dist(actual.position,expected.position)
    quaternion_error=min(max(abs(a-sign*b) for a,b in zip(actual.orientation_wxyz,expected.orientation_wxyz)) for sign in (-1,1))
    if position_error>1e-5 or quaternion_error>1e-5:
        raise ValueError('actual head optical pose differs from source calibration')
    return dict(image_domain='rectified',cameras=result,head_source_pose_check=dict(passed=True,
        translation_error_m=position_error,quaternion_max_error=quaternion_error,tolerance=1e-5))


def decode_target(raw, target, source, spec):
    """Decode normalized arms + total finger travel mm; return exact joint targets."""
    if (not isinstance(raw,list) or len(raw)!=14
            or any(type(v) not in (int,float) or not math.isfinite(v) for v in raw)):
        raise ValueError('fourteen finite native values required')
    normalized=raw.copy();normalized[6]=normalized[13]=1.0
    decoded=spec.decode(normalized)  # Native spec owns arm normalization and limits.
    model=convert_state(target,source['calibration_profile'],to_reported=False)
    values={}
    for side,offset,prefix in (('left',0,'left_joint'),('right',7,'right_joint1')):
        joints=getattr(decoded,side).joint_positions_rad
        if any(abs(a-b)>1e-9 for a,b in zip(joints,model[side+'_arm_joints'])):
            raise ValueError('normalized native arm differs from physical target')
        gap=raw[offset+6]
        if gap<0 or abs(gap-model[side+'_gripper'])>1e-9:
            raise ValueError('native finger travel differs from calibrated physical target')
        values.update({prefix+str(i+1):q for i,q in enumerate(joints)})
        values.update({prefix+str(i):gap/2000 for i in (7,8)})
    return values


def normalize_initial_arm_limits(state, spec, environment='fake'):
    """Admit a measured start just outside a soft limit; reset inside the model."""
    from robot_workbench.joint_bounds import project_joint_positions
    result=deepcopy(state); changes=[]
    for side in ('left','right'):
        arm=getattr(spec,side).arm
        key=side+'_arm_joints'
        if not (len(state[key])==len(arm.soft_lower_limits_rad)==len(arm.soft_upper_limits_rad)==6):
            raise ValueError('six initial joints and soft limits required per arm')
        result[key],projected=project_joint_positions(state[key],list(zip(arm.soft_lower_limits_rad,arm.soft_upper_limits_rad)),environment=environment)
        changes.extend(dict(joint=f'{side}_{row["index"]+1}',measured_model_rad=row['original_rad'],
            reset_model_rad=row['bounded_rad'],delta_rad=row['delta_rad']) for row in projected)
    return result,changes


def source_environment(source):
    from real_contract import namespace,validate_namespace
    environment=source.get('environment','physical' if source.get('schema_version')=='physical_collection_source.v1' else None)
    if source.get('schema_version')!=namespace(environment)['source_schema']:raise ValueError('L3 source environment/schema differ')
    validate_namespace(source,environment)
    return environment


def validate_inputs(source,binding,suite,plan=None,execution_timeline=None,*,environment='physical'):
    from real_observation import validate_binding
    from real_contract import validate_real_source
    validate_binding(source['calibration_profile'],binding,environment=environment)
    validate_real_source(source,binding,environment=environment)
    validate_initial_state(source['initial_state'],source['calibration_profile'])
    episodes=suite.get('episodes')
    if not isinstance(episodes,list) or len(episodes)!=1:
        raise ValueError('one detected episode required')
    episode=episodes[0]
    if (episode.get('metadata') or {}).get('scene_asset_set') != source['scene'].get('asset_set'):
        raise ValueError('suite and observed scene asset sets differ')
    keys=('episode_id','seed','machine_asset','environment_from_machine_root','environment_from_cup')
    if ({k:episode[k] for k in keys}!=source['scene']['episode_scene'] or episode.get('randomization') is not None
            or (episode.get('metadata') or {}).get('initial_robot_state')!=source['initial_state']['model']
            or episode['metadata'].get('initial_robot_state_digest')!=_digest(source['initial_state']['model'])):
        raise ValueError('suite must preserve detected scene and measured model start')
    if plan is not None:
        from real_contract import validate_real_plan
        from frozen_executor import validate_plan_evidence
        validate_real_plan(plan,source,binding,environment=environment)
        validate_plan_evidence(plan,source)
        if plan.get('native_action_encoding')!=NATIVE_DISTANCE_ENCODING:
            raise ValueError('playground replay requires explicit native distance encoding')
        validate_distance_control(plan,source)
        hz,n=plan['control']['physics_hz'],plan['control']['substeps']
        steps=plan['native_substeps']
        if (type(hz) is not int or type(n) is not int or n<=0 or hz!=30*n
                or len(steps)!=len(plan['ticks']) or not steps
                or any(type(v) is not int or not 1<=v<=n for v in steps)
                or any(v!=n for v in steps[:-1])
                or abs(sum(steps)/hz-plan['duration_s'])>1e-9
                or any(abs(t['time_s']-i/30)>1e-9 for i,t in enumerate(plan['ticks']))):
            raise ValueError('30 Hz timeline and final-only partial tick required')
    if execution_timeline is not None:
        if plan is None:raise ValueError('execution timeline requires its validated parent plan')
        from execution_timeline import validate
        validate(execution_timeline,plan)


def validate_l3_replay_derivation(plan,source,timeline,spec,saved=None):
    """Rebuild or verify the bounded L3-only prefix derivation from its frozen parents."""
    from control_motion import validate_control
    from execution_timeline import validate
    from frozen_executor import validate_plan_evidence
    from robot_workbench.joint_bounds import position_noise_rad
    from robot_sim.specs.action import ArmCommand,DualArmCommand
    environment=source_environment(source)
    validate_initial_state(source['initial_state'],source['calibration_profile'])
    validate_plan_evidence(plan,source)
    validate(timeline,plan)
    commands=deepcopy(timeline['commands']);prefix=len(commands)-len(plan['ticks'])
    actions=[];adjustments=[]
    for row in commands:
        if row['source_tick'] is not None:
            raw=deepcopy(plan['native_actions'][row['source_tick']])
        else:
            target=convert_state(row['target'],source['calibration_profile'],to_reported=False)
            target,changes=normalize_initial_arm_limits(target,spec,environment=environment)
            if changes:
                reported=convert_state(target,source['calibration_profile'],to_reported=True)
                for change in changes:
                    side,index=change['joint'].rsplit('_',1);index=int(index)-1;key=side+'_arm_joints'
                    adjustments.append(dict(tick=row['tick'],source_tick=None,joint=change['joint'],
                        parent_reported_rad=row['target'][key][index],derived_reported_rad=reported[key][index],
                        parent_model_rad=change['measured_model_rad'],derived_model_rad=change['reset_model_rad'],
                        delta_rad=change['delta_rad']))
                    # Only the admitted arm coordinate changes; preserve every other original field exactly.
                    row['target'][key][index]=reported[key][index]
            raw=list(spec.encode(DualArmCommand(ArmCommand(tuple(target['left_arm_joints']),True),
                                                ArmCommand(tuple(target['right_arm_joints']),True))))
            raw[6],raw[13]=target['left_gripper'],target['right_gripper']
        decode_target(raw,row['target'],source,spec)
        actions.append(raw)
    view=dict(deepcopy(plan),ticks=commands,native_actions=actions,
        native_substeps=[plan['control']['substeps']]*prefix+deepcopy(plan['native_substeps']),
        duration_s=timeline['duration_s'])
    control=validate_control(view)
    derivation=dict(schema_version='l3_initialization_derivation.v1',scope='L3_auxiliary_simulation_only',
        command_authority=False,task_outcome='unverified',source_environment=environment,limit_allowance_rad=position_noise_rad(environment),
        source_digest=_digest(source),calibration_profile_digest=_digest(source['calibration_profile']),
        source_initial_state_digest=_digest(source['initial_state']),parent_timeline_digest=plan['timeline_digest'],
        execution_timeline_digest=timeline['timeline_digest'],shared_commands_digest=_digest(timeline['commands']),
        parent_native_actions_sha256=_digest(plan['native_actions']),
        native_arm_limits={side:dict(lower=list(getattr(spec,side).arm.soft_lower_limits_rad),
                                    upper=list(getattr(spec,side).arm.soft_upper_limits_rad)) for side in ('left','right')},
        prefix_count=prefix,prefix_commands=deepcopy(commands[:prefix]),prefix_native_actions=deepcopy(actions[:prefix]),
        adjustments=adjustments,derived_commands_digest=_digest(commands),derived_native_actions_sha256=_digest(actions),
        derived_native_substeps_sha256=_digest(view['native_substeps']),duration_s=view['duration_s'],
        replay_matches_shared_commands=commands==timeline['commands'],control_validation=control,
        parent_suffix_unchanged=True,planning_evidence_unchanged=True,collision_audit_rerun=False)
    derivation['derivation_digest']=_digest(derivation)
    if saved is not None and saved!=derivation:
        raise ValueError('L3 initialization derivation differs from the frozen parents or bounded projection')
    view.update(l3_derivation=derivation,l3_control_validation=control)
    return view


def runtime_replay_plan(plan,source,timeline,spec):
    return validate_l3_replay_derivation(plan,source,timeline,spec)


def runtime_initial_model(source,timeline):
    """The simulated reset reproduces legal commands; raw sensor values remain provenance."""
    return convert_state(timeline['commands'][0]['target'],source['calibration_profile'],to_reported=False)


def requested_cup_preset(args,plan):
    preset=getattr(args,'cup_contact_preset',None)
    if preset is not None:
        if getattr(args,'environment','physical')!='fake':raise ValueError('Playground cup preset is fake-only')
        from cup_presets import bind_preset_identity
        bind_preset_identity({},preset)
    if plan is not None and (plan.get('cup_contact_preset')!=preset
            or plan.get('native_identity',{}).get('physical_configuration',{}).get('cup_contact_preset')!=preset):
        raise ValueError('replay cup preset differs from the frozen plan/physical configuration')
    if plan is not None and preset is not None:
        identity=plan['native_identity']
        if identity.get('physical_configuration_sha256')!=bind_preset_identity(dict(identity['physical_configuration']),preset):
            raise ValueError('replay physical configuration digest differs')
    return deepcopy(preset)


def setup(args,source,suite,plan,execution_timeline=None):
    preset=requested_cup_preset(args,plan)
    os.environ['ROBOT_SIM_ASSET_ROOT']=str(args.assets)
    from isaaclab.app import AppLauncher
    app=AppLauncher(headless=True,enable_cameras=True,device='cuda:0',active_gpu=args.gpu,multi_gpu=False,
        kit_args=f'--/renderer/activeGpu={args.gpu} --/renderer/multiGpu/enabled=false --/renderer/multiGpu/autoEnable=false --/renderer/gpuEnumeration/glInterop/enabled=false').app
    env=None
    try:
        from robot_sim.tasks.manager_based.coffee_make.env import CoffeeMakeEnv
        from robot_sim.tasks.manager_based.coffee_make.episode_scene import EpisodeScene
        from robot_sim.tasks.manager_based.coffee_make.generalization_env_cfg import make_generalization_env_cfg
        from robot_sim.tasks.manager_based.coffee_make.generalization_execution import ControlSpec
        from robot_sim.tasks.manager_based.coffee_make.interaction_evaluation import EpisodeAcceptanceSpec
        control=ControlSpec(**plan['control']) if plan else ControlSpec(240,8,2.,5.,300.)
        opening=source['calibration_profile']['gripper']['max_opening_mm']-gripper_offset_mm(source['calibration_profile'])
        acceptance=replace(EpisodeAcceptanceSpec.development(),open_position_m=opening/2000)
        episode_data=deepcopy(suite['episodes'][0])
        episode_data['metadata']['gripper_mode']='distance'
        if execution_timeline is not None:
            initial=runtime_initial_model(source,execution_timeline)
            episode_data['metadata'].update(initial_robot_state=initial,initial_robot_state_digest=_digest(initial))
        elif plan and plan.get('execution_resume'):
            initial=convert_state(plan['ticks'][0]['target'],source['calibration_profile'],to_reported=False)
            episode_data['metadata'].update(initial_robot_state=initial,initial_robot_state_digest=_digest(initial))
        from robot_sim.specs.action import ac_one_dual_arm_zero_home_action_spec
        from robot_workbench.joint_bounds import position_noise_rad
        environment=source_environment(source)
        original=episode_data['metadata']['initial_robot_state']
        initial,adjustments=normalize_initial_arm_limits(original,
            ac_one_dual_arm_zero_home_action_spec(),environment=environment)
        episode_data['metadata'].update(initial_robot_state=initial,initial_robot_state_digest=_digest(initial))
        normalization=dict(source_initial_digest=_digest(source['initial_state']['model']),
              input_initial_digest=_digest(original),reset_initial_digest=_digest(initial),
              source_environment=environment,limit_allowance_rad=position_noise_rad(environment),adjustments=adjustments)
        normalization.update(getattr(args,'scene_reset_identity',{}))
        if execution_timeline is not None:
            normalization.update(execution_timeline_digest=execution_timeline['timeline_digest'],
                initialization=deepcopy(execution_timeline.get('initialization')),
                simulation_start_scope='first legal model command; raw physical measurement retained in initialization')
        write(args.output/'initial-normalization.json',normalization)
        episode=EpisodeScene.from_dict(episode_data)
        cfg=make_generalization_env_cfg(control=control,acceptance=acceptance,asset_root=args.assets,
            cameras=True,camera_rig='d405',physics_device='cuda:0',pad_torsional_radius_m=.0015,
            solver_profile='contact_32_8',episode=episode,continuous_motion=True)
        for name in ('room','d435_model','handoff_stand'):
            if hasattr(cfg.scene,name):setattr(cfg.scene,name,None)
        for name in ('base_0','left_wrist_0','right_wrist_0'):
            camera=getattr(cfg.scene,'d405_'+name)
            camera.data_types=['rgb'];camera.update_period=1/30
        env=CoffeeMakeEnv(cfg)
        materials=apply_whitebox_materials()
        runtime=env._m2_runtime
        if preset is not None:
            import isaaclab.sim as sim_utils
            from cup_presets import configure_preset
            material=configure_preset(runtime,sim_utils.get_current_stage(),preset)
            write(args.output/'cup-contact-preset.json',dict(preset=preset,material=material,
                default_selected=False,scope='frozen fake configuration; actual material applied before reset'))
        bind_effective_gripper_identity(runtime,acceptance.open_position_m,env.action_manager.get_term('guarded').gripper_limits())
        if args.mode=='maintenance-geometry-inputs':
            from functools import partial
            from maintenance_geometry import PURPOSE,audit_initial_support
            _,info=runtime.maintenance_geometry_reset(purpose=PURPOSE,
                audit=partial(audit_initial_support,support=args.maintenance_scope['support']),options={'episode':episode})
            write(args.output/'maintenance-initialization-audit.json',dict(task_characterization='not_run',
                audits=info['maintenance_audit_trace'],history=runtime.evaluator.history))
        else:
            _,info=env.reset(options={'episode':episode})
        observed,_=runtime.read(runtime.loop.physics_steps)
        # Projection was checked above; readback independently retains the reset tracking gate.
        if max(abs(a-b) for a,b in zip(observed.arm_positions,
                initial['left_arm_joints']+initial['right_arm_joints']))>.005:
            raise ValueError('simulated reset differs from its accepted initial state by over 0.005 rad')
        runtime._render()
        return app,env,runtime,episode,control,acceptance,info,materials
    except BaseException as error:
        # Kit shutdown may terminate the interpreter; preserve the cause first.
        traceback.print_exc()
        write(args.output/'result.json',dict(status='failed',error=str(error),
            traceback=traceback.format_exc(),phase='setup',task_outcome='unverified',
            reset_diagnostics=reset_failure_context(env)))
        close_runtime(env,app)
        raise


def reset_scene(args,source,suite,env,runtime,acceptance):
    """Rebind only this isolated scene's start, then use the native reset/readback."""
    import torch
    from robot_sim.tasks.manager_based.coffee_make.episode_scene import EpisodeScene
    from robot_sim.tasks.manager_based.coffee_make.episode_randomization import scene_build_key
    from robot_sim.tasks.manager_based.coffee_make.initial_robot_state import initial_robot_state,joint_positions,arm_positions,gripper_mode
    from robot_sim.specs.action import ArmCommand,DualArmCommand,ac_one_dual_arm_zero_home_action_spec
    from robot_workbench.joint_bounds import position_noise_rad
    data=deepcopy(suite['episodes'][0]);original=data['metadata']['initial_robot_state']
    environment=source_environment(source)
    initial,adjustments=normalize_initial_arm_limits(original,ac_one_dual_arm_zero_home_action_spec(),environment=environment)
    data['metadata'].update(initial_robot_state=initial,initial_robot_state_digest=_digest(initial),gripper_mode='distance')
    episode=EpisodeScene.from_dict(data)
    if (scene_build_key(episode)!=scene_build_key(runtime.spec.episode)
            or gripper_mode(episode)!=runtime.gripper_mode):
        raise ValueError('scene model changed; explicitly prepare a new scene service')
    state=initial_robot_state(episode,acceptance.open_position_m*2000)
    joints=joint_positions(state);robot=env.scene['robot']
    ids,names=robot.find_joints(list(joints),preserve_order=True)
    if list(names)!=list(joints):raise ValueError('scene reset joint ordering differs')
    values=torch.tensor([[joints[name] for name in names]],device=env.device,dtype=robot.data.default_joint_pos.dtype)
    # The original builder sets these same default positions. Keep the original
    # configured machine pose: reset_episode_poses computes deltas from it.
    robot.data.default_joint_pos[:,ids]=values
    robot.data.default_joint_vel[:,ids]=0.
    env.cfg.scene.robot.init_state.joint_pos.update(joints)
    runtime.initial_robot_state=state;runtime.home_positions=arm_positions(state)
    runtime.home=runtime.action.encode(DualArmCommand(ArmCommand(runtime.home_positions[:6],True),ArmCommand(runtime.home_positions[6:],True)))
    runtime.evaluator.home_positions=runtime.home_positions
    runtime.physical_identity.update(initial_robot_state=state,initial_robot_state_digest=_digest(state))
    runtime.physical_sha256=hashlib.sha256(json.dumps(runtime.physical_identity,sort_keys=True,allow_nan=False).encode()).hexdigest()
    write(args.output/'initial-normalization.json',dict(source_initial_digest=_digest(source['initial_state']['model']),
        input_initial_digest=_digest(original),reset_initial_digest=_digest(initial),source_environment=environment,
        limit_allowance_rad=position_noise_rad(environment),adjustments=adjustments,scene_runtime_reused=True,
        **getattr(args,'scene_reset_identity',{})))
    _,info=env.reset(options={'episode':episode})
    observed,_=runtime.read(runtime.loop.physics_steps)
    if max(abs(a-b) for a,b in zip(observed.arm_positions,initial['left_arm_joints']+initial['right_arm_joints']))>.005:
        raise ValueError('simulated reset differs from its accepted initial state by over 0.005 rad')
    runtime._render()
    return episode,info


def first_frame(args,source,env,runtime,materials):
    """Render this reset's calibrated scene; no previous source pixels are reused."""
    from PIL import Image
    sensors=preview_sensors(env.scene)
    _,scene=runtime.read(runtime.loop.physics_steps)
    metadata=camera_metadata(env,sensors,source,scene.instance.world_from_environment)
    write(args.output/'camera-metadata.json',metadata)
    Image.fromarray(camera_frame(env,'base_0')).save(args.output/'first-frame.png')
    write(args.output/'first-frame.json',dict(source_digest=_digest(source),clock='simulation',simulation_time_s=0,
        camera='base_0',image_domain='rectified',camera_metadata=metadata,materials=materials,
        task_outcome='unverified',sha256=file_sha256(args.output/'first-frame.png')))
    return sensors


def plan_episode(args,source,runtime,episode,control,acceptance,info,*,planner=None):
    started=time.monotonic()
    import isaaclab.sim as sim_utils
    from robot_sim.asset_cfg.objects.coffee_machine import RigidTransform
    from robot_sim.policies.rule_coffee import RuleObservation
    from robot_sim.tasks.manager_based.coffee_make.episode_planner import EpisodePlanner
    from robot_sim.tasks.manager_based.coffee_make.layout import canonical_coffee_make_v1_layout
    from robot_sim.tasks.manager_based.coffee_make.native_finger_geometry import export_native_fingers
    from robot_sim.tasks.manager_based.coffee_make.scene_instance import ResolvedPrimitive
    from playground_plan import build
    if planner is None:
        native=export_native_fingers(sim_utils.get_current_stage(),runtime.root+'/Robot')
        if not native['complete']:raise ValueError('incomplete native geometry')
        digest=hashlib.sha256(json.dumps(dict(parts=native['parts'],planes=native['native_planes']),sort_keys=True).encode()).hexdigest()
        layout=canonical_coffee_make_v1_layout()
        table=ResolvedPrimitive(runtime.evaluator.table_shape,runtime.evaluator.table_shape,'shell',
            RigidTransform(layout.worktable.position),layout.worktable_size_m,'box',None)
        planner=EpisodePlanner(args.assets,args.finger_envelope,args.finger_envelope_sha256,digest,table,
            episode=episode,open_position_m=acceptance.open_position_m)
    else:
        from robot_sim.tasks.manager_based.coffee_make.initial_robot_state import initial_robot_state,joint_positions
        planner.home_state=joint_positions(initial_robot_state(episode,acceptance.open_position_m*2000))
        digest=planner.identity['native_geometry_sha256']
    geometry_ready=time.monotonic()
    print(json.dumps(dict(stage='geometry-ready',elapsed_s=geometry_ready-started)),flush=True)
    sample,scene=runtime.read(runtime.loop.physics_steps)
    if source['scene'].get('asset_set') is not None:
        from scene_assets import check_geometry
        check_geometry(source['scene']['asset_set'], scene.profile_sha256, planner.cup.profile_sha256)
    obs=RuleObservation(info['episode_id'],info['step_id'],dict(info['oracle_facts']),sample,scene,
        ((info.get('execution_report') or {}).get('event') or {}).get('reason'),bool(info['contact_state']['support_acquired']))
    identity=dict(control=asdict(control),acceptance=asdict(acceptance),physical_configuration=runtime.physical_identity,
        physical_configuration_sha256=runtime.physical_sha256,finger_envelope_sha256=args.finger_envelope_sha256,
        native_geometry_sha256=digest,planner_identity=planner.identity,task_outcome='unverified')
    from robot_sim.asset_cfg.robots.ac_one_spec import ac_one_asset_spec
    from real_contract import validate_assisted_cup_geometry
    validate_assisted_cup_geometry(source,asdict(planner.cup))
    exported=dict(parts=[asdict(p) for p in planner.parts],
        cup=asdict(planner.cup),table=asdict(planner.table),obstacles=[asdict(p) for p in planner.obstacles],
        observation=dict(episode_id=obs.episode_id,step_id=obs.step_id,facts=obs.facts,sample=asdict(sample),
            scene=asdict(scene),terminal_reason=obs.terminal_reason,support_now=obs.support_now),
        planner_identity=planner.identity,open_position_m=planner.open_position_m,source_digest=_digest(source),
        native_identity=identity,urdf_sha256=file_sha256(ac_one_asset_spec(args.assets).urdf_path))
    if args.mode=='maintenance-geometry-inputs':
        import inspect
        scope=dict(args.maintenance_scope,initial_support_audit=info['maintenance_geometry'],
                   initialization_audit=info['maintenance_audit_trace'],
                   native_runtime_sha256=file_sha256(Path(inspect.getfile(type(runtime)))))
        exported.update(export_scope=scope,export_scope_digest=_digest(scope))
    write(args.output/'planning-input.json',exported)
    if args.mode in ('geometry-inputs','maintenance-geometry-inputs'):return planner
    initial=planner.initial(obs,acceptance.edge_margin_m)
    initial_ready=time.monotonic()
    print(json.dumps(dict(stage='initial-route-ready',elapsed_s=initial_ready-geometry_ready,passed=initial.get('passed'))),flush=True)
    write(args.output/'initial-plan.json',initial)
    plan=build(planner,obs,initial,source,identity,args.speed_scale,environment=getattr(args,'environment','physical'))
    write(args.output/'plan.json',plan)
    write(args.output/'planning-timings.json',dict(geometry=geometry_ready-started,
        route=initial_ready-geometry_ready,timeline_audit=time.monotonic()-initial_ready,total=time.monotonic()-started))
    return plan


def validate_native_limits(targets,names,limits):
    for index,row in enumerate(targets):
        if len(row)!=len(names) or len(limits)!=len(names):raise ValueError('joint limit dimensions differ')
        for name,value,(lo,hi) in zip(names,row,limits):
            if not all(math.isfinite(v) for v in (value,lo,hi)) or not lo<=value<=hi:
                raise ValueError(f'joint target outside native limits: tick={index}, joint={name}, target={value}, limits={[lo,hi]}')


def replay(args,source,plan,env,runtime,sensors,execution_timeline=None):
    import torch
    from robot_sim.specs.action import ac_one_dual_arm_zero_home_action_spec
    spec=ac_one_dual_arm_zero_home_action_spec()
    if execution_timeline is not None:
        plan=runtime_replay_plan(plan,source,execution_timeline,spec)
        write(args.output/'execution-timeline.json',execution_timeline)
        write(args.output/'l3-initialization-derivation.json',plan['l3_derivation'])
    robot=env.scene['robot']
    decoded=[decode_target(raw,tick['target'],source,spec) for raw,tick in zip(plan['native_actions'],plan['ticks'],strict=True)]
    names=list(decoded[0]);ids=robot.find_joints(names,preserve_order=True)[0]
    # Soft limits are the RL training margin (90% here), not mechanical limits;
    # applying them to this trajectory incorrectly forbids the calibrated 83 mm opening.
    limits=robot.data.joint_pos_limits[0,ids].tolist()
    targets=torch.tensor([[row[name] for name in names] for row in decoded],
                         device=robot.device,dtype=robot.data.joint_pos.dtype)
    validate_native_limits(targets.cpu().tolist(),names,limits)
    writers={};logs={};completed=0;hz=plan['control']['physics_hz'];started=time.monotonic()
    try:
        ffmpeg=shutil.which('ffmpeg')
        if ffmpeg is None:raise ValueError('ffmpeg required for three-camera simulation replay')
        for camera,sensor in sensors.items():
            frame=camera_frame(env,camera)
            h,w=frame.shape[:2];logs[camera]=(args.output/(camera+'-ffmpeg.log')).open('x')
            path=args.output/f'{camera}.mp4'
            tail=plan['native_substeps'][-1]/hz
            command=[ffmpeg,'-nostdin','-v','error','-n','-f','rawvideo','-pix_fmt','rgb24','-s',f'{w}x{h}',
                '-framerate','30','-i','pipe:0','-an','-c:v','libx264','-threads','2','-preset','fast',
                '-crf','24','-pix_fmt','yuv420p','-bf','0','-fps_mode','passthrough','-enc_time_base','1:90000',
                '-bsf:v',f'setts=duration=if(eq(N\\,{len(decoded)-1})\\,{tail:.12f}/TB\\,DURATION)',
                '-video_track_timescale','90000','-movflags','+faststart',str(path)]
            writers[camera]=subprocess.Popen(command,stdin=subprocess.PIPE,stderr=logs[camera])
        with (args.output/'frames.jsonl').open('x') as frames, (args.output/'execution.jsonl').open('x') as execution:
            for tick,row,count in zip(plan['ticks'],decoded,plan['native_substeps'],strict=True):
                # Frame at tick start; no fabricated terminal duplicate frame.
                runtime._render()
                for camera,sensor in sensors.items():
                    writers[camera].stdin.write(camera_frame(env,camera).tobytes())
                frames.write(json.dumps(dict(frame=tick['tick'],simulation_time_s=tick['time_s'],
                    image_domain='rectified',world_from_optical={key:sensor_pose(sensor) for key,sensor in sensors.items()}))+'\n')
                robot.set_joint_position_target(targets[tick['tick']].unsqueeze(0),joint_ids=ids)
                contact_count=0
                for _ in range(count):
                    runtime.absolute_step+=1
                    env._sim_step_counter+=1
                    env._m2_contact_reader.begin_step(runtime.absolute_step)
                    env.scene.write_data_to_sim();env.sim.step(render=False)
                    contacts=env._m2_contact_reader.read(runtime.absolute_step)
                    contact_count+=len(contacts)
                    env.scene.update(dt=1/hz)
                measured=robot.data.joint_pos[0,ids].tolist()
                if any(not math.isfinite(v) for v in measured):raise ValueError('nonfinite native joint feedback')
                record=dict(tick=tick['tick'],target=tick['target'],native_joint_targets=row,
                    measured_joint_positions=dict(zip(names,measured)),actual_substeps=count,
                    contact_count=contact_count,task_outcome='unverified')
                if execution_timeline is not None:
                    record['shared_command']=execution_timeline['commands'][tick['tick']]
                    record['l3_derived_command']=tick
                execution.write(json.dumps(record)+'\n')
                completed+=1
        for process in writers.values():process.stdin.close()
        for camera,process in writers.items():
            if process.wait(timeout=120)!=0:raise ValueError('simulation encoder failed: '+camera)
        files={}
        for camera in sensors:
            path=args.output/f'{camera}.mp4'
            probe=json.loads(subprocess.run(['ffprobe','-v','error','-count_frames','-show_streams','-of','json',str(path)],
                check=True,capture_output=True,text=True,timeout=30).stdout)['streams']
            if (len(probe)!=1 or probe[0].get('codec_name')!='h264' or probe[0].get('pix_fmt')!='yuv420p'
                    or int(probe[0]['nb_read_frames'])!=len(decoded) or abs(float(probe[0]['duration'])-plan['duration_s'])>.001):
                raise ValueError('simulation video frame count/format/duration differs')
            files[path.name]=dict(size_bytes=path.stat().st_size,sha256=file_sha256(path),probe=probe[0])
        result=dict(schema_version='playground_replay.v1',status='completed',evidence_source='simulation',
            source_run_id=source['run_id'],plan_run_id=plan['run_id'],timeline_digest=plan['timeline_digest'],
            clock='simulation',fps=30,duration_s=plan['duration_s'],files=files,image_domain='rectified',
            camera_metadata=json.loads((args.output/'camera-metadata.json').read_text()),
            trajectory_complete=completed==len(decoded),task_outcome='unverified',wall_seconds=time.monotonic()-started)
        if execution_timeline is not None:
            result.update(parent_timeline_digest=plan['timeline_digest'],execution_timeline_digest=execution_timeline['timeline_digest'],
                command_count=completed,parent_command_count=len(execution_timeline['commands'])-sum(row['source_tick'] is None for row in execution_timeline['commands']),
                initialization_duration_s=execution_timeline['initialization_duration_s'],
                execution_derivation='L3-only initialization projection within the source-environment position allowance; original shared commands retained',
                replay_matches_shared_commands=plan['l3_derivation']['replay_matches_shared_commands'],
                l3_derivation_digest=plan['l3_derivation']['derivation_digest'],
                l3_commands_digest=plan['l3_derivation']['derived_commands_digest'],
                l3_derivation_ref='l3-initialization-derivation.json',l3_derivation_sha256=file_sha256(args.output/'l3-initialization-derivation.json'),
                planning_evidence_unchanged=True,collision_audit_rerun=False,
                execution_timeline_ref='execution-timeline.json',execution_timeline_sha256=file_sha256(args.output/'execution-timeline.json'))
        write(args.output/'manifest.json',result)
        return result
    finally:
        for process in writers.values():
            if process.stdin and not process.stdin.closed:process.stdin.close()
            if process.poll() is None:
                process.terminate()
                try:process.wait(timeout=5)
                except subprocess.TimeoutExpired:process.kill();process.wait()
        for log in logs.values():log.close()


def main(args):
    from local_execution import register_auxiliary
    register_auxiliary(args,__file__)
    started=time.monotonic()
    source=json.loads(args.source.read_text());suite=json.loads(args.suite.read_text())
    binding=json.loads(args.hardware_binding.read_text())
    plan=json.loads(args.plan.read_text()) if args.plan else None
    timeline_path=getattr(args,'execution_timeline',None)
    timeline=json.loads(timeline_path.read_text()) if timeline_path else None
    if timeline is not None and args.mode!='replay':raise ValueError('execution timeline is replay-only')
    validate_inputs(source,binding,suite,plan,timeline,environment=getattr(args,'environment','physical'))
    if args.mode=='maintenance-geometry-inputs':
        from maintenance_geometry import validate_request
        if not getattr(args,'maintenance_request',None):raise ValueError('sealed maintenance request required')
        args.maintenance_scope=validate_request(args.maintenance_request,source,args.environment)
    elif getattr(args,'maintenance_request',None):raise ValueError('maintenance request is restricted to maintenance geometry')
    if args.mode=='replay' and plan is None:raise ValueError('replay requires plan')
    if args.mode in ('plan','geometry-inputs','maintenance-geometry-inputs') and (not args.finger_envelope or not args.finger_envelope_sha256
            or file_sha256(args.finger_envelope)!=args.finger_envelope_sha256):
        raise ValueError('plan requires pinned native finger envelope')
    args.output.mkdir(parents=True,exist_ok=False)
    app=env=None
    try:
        app,env,runtime,episode,control,acceptance,info,materials=setup(args,source,suite,plan,timeline)
        initialized=time.monotonic()
        sensors=first_frame(args,source,env,runtime,materials)
        result=dict(status='first-frame-ready',task_outcome='unverified')
        if args.mode in ('geometry-inputs','maintenance-geometry-inputs'):
            plan_episode(args,source,runtime,episode,control,acceptance,info)
            if args.mode=='maintenance-geometry-inputs':
                from maintenance_geometry import STATUS
                result.update(status=STATUS,task_characterization='not_run')
            else:result.update(status='geometry-inputs-ready')
        elif args.mode=='plan':
            faulthandler.dump_traceback_later(60,repeat=True)
            try:plan=plan_episode(args,source,runtime,episode,control,acceptance,info)
            finally:faulthandler.cancel_dump_traceback_later()
            from frozen_executor import validate_plan_evidence
            validate_plan_evidence(plan,source)
            result.update(status='geometric-plan-ready',timeline_digest=plan['timeline_digest'],simulation=plan['simulation'])
        elif args.mode=='replay':result=replay(args,source,plan,env,runtime,sensors,timeline)
        result['timings_seconds']=dict(initialization=initialized-started,
                                      operation=time.monotonic()-initialized,total=time.monotonic()-started)
        write(args.output/'result.json',result)
    except Exception as error:
        traceback.print_exc()
        if not (args.output/'result.json').is_file():
            write(args.output/'result.json',dict(status='failed',error=str(error),traceback=traceback.format_exc(),
                                                 wall_seconds=time.monotonic()-started,task_outcome='unverified'))
        raise
    finally:close_runtime(env,app)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode',choices=('plan','first-frame','geometry-inputs','maintenance-geometry-inputs','replay'),required=True)
    parser.add_argument('--maintenance-request',type=Path,help='sealed parent/current/asset identities; maintenance geometry only')
    for name in ('source','suite','assets','hardware-binding','output'):
        parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--plan',type=Path)
    parser.add_argument('--execution-timeline',type=Path,help='validated 30 Hz initialization prefix and unchanged parent targets')
    parser.add_argument('--finger-envelope',type=Path)
    parser.add_argument('--finger-envelope-sha256')
    parser.add_argument('--speed-scale',type=float,default=.3)
    parser.add_argument('--environment',choices=('physical','fake'),default='physical')
    parser.add_argument('--cup-contact-preset',type=json.loads,help='explicit frozen fake material preset; never a physical setting')
    parser.add_argument('--execution-json',help='trusted local deployment, supplied by the frozen pipeline')
    parser.add_argument('--gpu',type=int,required=True)
    parser.add_argument('--instance',required=True)
    parser.add_argument('--run-id',required=True)
    args=parser.parse_args()
    if args.execution_json:
        from local_execution import validate
        execution=validate({'execution':json.loads(args.execution_json)})
        lane=dict(instance_id=execution['instance_id'],gpu=execution['gpu_index'])
    else:lane=PIPELINE_LANES['dsw-scene']
    if (args.instance,args.gpu)!=(lane['instance_id'],lane['gpu']) or os.environ.get('CUDA_VISIBLE_DEVICES')!=str(args.gpu):
        raise ValueError('instance/GPU not authorized for target simulation')
    if not .3<=args.speed_scale<=1.7:raise ValueError('speed scale outside [0.3,1.7]')
    main(args)
