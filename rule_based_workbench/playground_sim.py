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

from integration import _digest, PIPELINE_LANES
from gripper_distance_plan import NATIVE_DISTANCE_ENCODING, validate_distance_control
from joint_space import convert_state, validate_initial_state, gripper_offset_mm
from isaac_lane import (apply_whitebox_materials, bind_effective_gripper_identity,
                        close_runtime, file_sha256, preview_sensors, write)


def camera_frame(env,camera):
    """Calibrated rectified pixels; the scene RGB tensor is overscan, not output."""
    from robot_sim.tasks.manager_based.coffee_make.d405_camera import color_observation
    calibration=env.cfg.d405_rig['device_calibrations'][camera]
    if any(abs(value)>1e-12 for value in calibration['distortion']['coefficients']):
        raise ValueError('Playground rectified output requires zero-distortion rig coefficients')
    frame=color_observation(env,camera)[0].cpu().numpy()
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
        calibration=env.cfg.d405_rig['device_calibrations'][camera]
        if any(abs(value)>1e-12 for value in calibration['distortion']['coefficients']):
            raise ValueError('rectified camera metadata requires zero-distortion rig coefficients')
        k=calibration['intrinsics'];world_pose=sensor_pose(sensor)
        pose=world_from_environment.inverse().compose(RigidTransform(**world_pose))
        result[camera]=dict(image_domain='rectified',width=calibration['width'],height=calibration['height'],
            serial=calibration['serial'],K=[[k['fx'],0,k['cx']],[0,k['fy'],k['cy']],[0,0,1]],
            distortion_coefficients=list(calibration['distortion']['coefficients']),
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


def normalize_initial_arm_limits(state, spec, tolerance_rad=.005):
    """Admit a measured start just outside a soft limit; reset inside the model."""
    result=deepcopy(state); changes=[]
    for side in ('left','right'):
        arm=getattr(spec,side).arm
        key=side+'_arm_joints'
        if not (len(state[key])==len(arm.soft_lower_limits_rad)==len(arm.soft_upper_limits_rad)==6):
            raise ValueError('six initial joints and soft limits required per arm')
        for index,(value,lower,upper) in enumerate(zip(state[key],arm.soft_lower_limits_rad,
                                                         arm.soft_upper_limits_rad)):
            if not math.isfinite(value) or value<lower-tolerance_rad or value>upper+tolerance_rad:
                raise ValueError(f'{side} joint {index+1} initial pose exceeds the 0.005 rad soft-limit allowance')
            bounded=min(max(value,lower),upper)
            result[key][index]=bounded
            if bounded!=value:
                changes.append(dict(joint=f'{side}_{index+1}',measured_model_rad=value,
                                    reset_model_rad=bounded,delta_rad=bounded-value))
    return result,changes


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


def runtime_replay_plan(plan,source,timeline,spec):
    """Consume shared commands without rewriting their parent planning proof."""
    from execution_timeline import validate
    from robot_sim.specs.action import ArmCommand,DualArmCommand
    validate(timeline,plan)
    commands=timeline['commands'];prefix=len(commands)-len(plan['ticks'])
    actions=[]
    for row in commands:
        if row['source_tick'] is not None:
            raw=deepcopy(plan['native_actions'][row['source_tick']])
        else:
            target=convert_state(row['target'],source['calibration_profile'],to_reported=False)
            raw=list(spec.encode(DualArmCommand(ArmCommand(tuple(target['left_arm_joints']),True),
                                                ArmCommand(tuple(target['right_arm_joints']),True))))
            raw[6],raw[13]=target['left_gripper'],target['right_gripper']
        decode_target(raw,row['target'],source,spec)
        actions.append(raw)
    return dict(plan,ticks=deepcopy(commands),native_actions=actions,
        native_substeps=[plan['control']['substeps']]*prefix+deepcopy(plan['native_substeps']),
        duration_s=timeline['duration_s'])


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
        initial,adjustments=normalize_initial_arm_limits(episode_data['metadata']['initial_robot_state'],
                                                          ac_one_dual_arm_zero_home_action_spec())
        episode_data['metadata'].update(initial_robot_state=initial,initial_robot_state_digest=_digest(initial))
        normalization=dict(source_initial_digest=_digest(source['initial_state']['model']),
              reset_initial_digest=_digest(initial),limit_allowance_rad=.005,adjustments=adjustments)
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
        _,info=env.reset(options={'episode':episode})
        observed,_=runtime.read(runtime.loop.physics_steps)
        original=source['initial_state']['model'] if execution_timeline is None else runtime_initial_model(source,execution_timeline)
        if (max(abs(a-b) for a,b in zip(observed.arm_positions,
                initial['left_arm_joints']+initial['right_arm_joints']))>.005
                or max(abs(a-b) for a,b in zip(observed.arm_positions,
                original['left_arm_joints']+original['right_arm_joints']))>.005):
            raise ValueError('simulated reset differs from its accepted initial state by over 0.005 rad')
        runtime._render()
        return app,env,runtime,episode,control,acceptance,info,materials
    except BaseException as error:
        # Kit shutdown may terminate the interpreter; preserve the cause first.
        traceback.print_exc()
        write(args.output/'result.json',dict(status='failed',error=str(error),
            traceback=traceback.format_exc(),phase='setup',task_outcome='unverified'))
        close_runtime(env,app)
        raise


def plan_episode(args,source,runtime,episode,control,acceptance,info):
    started=time.monotonic()
    import isaaclab.sim as sim_utils
    from robot_sim.asset_cfg.objects.coffee_machine import RigidTransform
    from robot_sim.policies.rule_coffee import RuleObservation
    from robot_sim.tasks.manager_based.coffee_make.episode_planner import EpisodePlanner
    from robot_sim.tasks.manager_based.coffee_make.layout import canonical_coffee_make_v1_layout
    from robot_sim.tasks.manager_based.coffee_make.native_finger_geometry import export_native_fingers
    from robot_sim.tasks.manager_based.coffee_make.scene_instance import ResolvedPrimitive
    from playground_plan import build
    native=export_native_fingers(sim_utils.get_current_stage(),runtime.root+'/Robot')
    if not native['complete']:raise ValueError('incomplete native geometry')
    digest=hashlib.sha256(json.dumps(dict(parts=native['parts'],planes=native['native_planes']),sort_keys=True).encode()).hexdigest()
    layout=canonical_coffee_make_v1_layout()
    table=ResolvedPrimitive(runtime.evaluator.table_shape,runtime.evaluator.table_shape,'shell',
        RigidTransform(layout.worktable.position),layout.worktable_size_m,'box',None)
    planner=EpisodePlanner(args.assets,args.finger_envelope,args.finger_envelope_sha256,digest,table,
        episode=episode,open_position_m=acceptance.open_position_m)
    geometry_ready=time.monotonic()
    print(json.dumps(dict(stage='geometry-ready',elapsed_s=geometry_ready-started)),flush=True)
    sample,scene=runtime.read(runtime.loop.physics_steps)
    obs=RuleObservation(info['episode_id'],info['step_id'],dict(info['oracle_facts']),sample,scene,
        ((info.get('execution_report') or {}).get('event') or {}).get('reason'),bool(info['contact_state']['support_acquired']))
    identity=dict(control=asdict(control),acceptance=asdict(acceptance),physical_configuration=runtime.physical_identity,
        physical_configuration_sha256=runtime.physical_sha256,finger_envelope_sha256=args.finger_envelope_sha256,
        native_geometry_sha256=digest,planner_identity=planner.identity,task_outcome='unverified')
    from robot_sim.asset_cfg.robots.ac_one_spec import ac_one_asset_spec
    write(args.output/'planning-input.json',dict(parts=[asdict(p) for p in planner.parts],
        cup=asdict(planner.cup),table=asdict(planner.table),obstacles=[asdict(p) for p in planner.obstacles],
        observation=dict(episode_id=obs.episode_id,step_id=obs.step_id,facts=obs.facts,sample=asdict(sample),
            scene=asdict(scene),terminal_reason=obs.terminal_reason,support_now=obs.support_now),
        planner_identity=planner.identity,open_position_m=planner.open_position_m,source_digest=_digest(source),
        native_identity=identity,urdf_sha256=file_sha256(ac_one_asset_spec(args.assets).urdf_path)))
    if args.mode=='geometry-inputs':return None
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
                if execution_timeline is not None:record['shared_command']=tick
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
                execution_derivation=execution_timeline['derivation'],planning_evidence_unchanged=True,collision_audit_rerun=False,
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
    if args.mode=='replay' and plan is None:raise ValueError('replay requires plan')
    if args.mode in ('plan','geometry-inputs') and (not args.finger_envelope or not args.finger_envelope_sha256
            or file_sha256(args.finger_envelope)!=args.finger_envelope_sha256):
        raise ValueError('plan requires pinned native finger envelope')
    args.output.mkdir(parents=True,exist_ok=False)
    app=env=None
    try:
        app,env,runtime,episode,control,acceptance,info,materials=setup(args,source,suite,plan,timeline)
        initialized=time.monotonic()
        sensors=preview_sensors(env.scene)
        from PIL import Image
        _,scene=runtime.read(runtime.loop.physics_steps)
        metadata=camera_metadata(env,sensors,source,scene.instance.world_from_environment)
        write(args.output/'camera-metadata.json',metadata)
        Image.fromarray(camera_frame(env,'base_0')).save(args.output/'first-frame.png')
        write(args.output/'first-frame.json',dict(source_digest=_digest(source),clock='simulation',simulation_time_s=0,
            camera='base_0',image_domain='rectified',camera_metadata=metadata,materials=materials,
            task_outcome='unverified',sha256=file_sha256(args.output/'first-frame.png')))
        result=dict(status='first-frame-ready',task_outcome='unverified')
        if args.mode=='geometry-inputs':
            plan_episode(args,source,runtime,episode,control,acceptance,info)
            result.update(status='geometry-inputs-ready')
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
        write(args.output/'result.json',dict(status='failed',error=str(error),traceback=traceback.format_exc(),
                                             wall_seconds=time.monotonic()-started,task_outcome='unverified'))
        raise
    finally:close_runtime(env,app)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode',choices=('plan','first-frame','geometry-inputs','replay'),required=True)
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
    if not .1<=args.speed_scale<=1:raise ValueError('speed scale outside [0.1,1]')
    main(args)
