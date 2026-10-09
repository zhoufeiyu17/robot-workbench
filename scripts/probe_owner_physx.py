#!/usr/bin/env python3
"""Bounded S0 control/PhysX/camera probe, never a coffee or hardware qualification."""
import argparse
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT/'calibration'), str(ROOT/'calibration/infra'),
               str(ROOT/'rule_based_workbench')]


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, allow_nan=False, indent=2)+'\n')


def preflight(base):
    native = Path(base['pipeline_config']['native_code'])
    pins = base['lanes']['source']['raw_provenance']['source_files']
    native_pins={name:digest for name,digest in pins.items() if name.startswith('source/')}
    for name, digest in native_pins.items():
        if hashlib.sha256((native/name).read_bytes()).hexdigest() != digest:
            raise ValueError('native source mismatch: '+name)
    assets = Path(base['pipeline_config']['assets'])
    physical = base['native_identity']['physical_configuration']
    for group in ('robot_dependencies', 'machine_dependencies', 'cup_dependencies'):
        for name, digest in physical[group].items():
            if hashlib.sha256((assets/name).read_bytes()).hexdigest() != digest:
                raise ValueError('asset mismatch: '+name)
    return dict(native_files_checked=len(native_pins), asset_dependencies_checked=sum(len(physical[g]) for g in
                ('robot_dependencies','machine_dependencies','cup_dependencies')),
                excluded_historical_recording_helpers=[name for name in pins if name not in native_pins])


def run(args, base):
    import os
    os.environ['ROBOT_SIM_ASSET_ROOT'] = base['pipeline_config']['assets']
    from isaaclab.app import AppLauncher
    app = AppLauncher(headless=True, enable_cameras=True, device='cuda:0', active_gpu=args.gpu,
                      multi_gpu=False, kit_args=f'--/renderer/activeGpu={args.gpu} --/renderer/multiGpu/enabled=false '
                      '--/renderer/multiGpu/autoEnable=false --/renderer/gpuEnumeration/glInterop/enabled=false').app
    env = None
    try:
        from robot_sim.tasks.manager_based.coffee_make.env import CoffeeMakeEnv
        from robot_sim.tasks.manager_based.coffee_make.episode_scene import EpisodeScene
        from robot_sim.tasks.manager_based.coffee_make.generalization_env_cfg import make_generalization_env_cfg
        from robot_sim.tasks.manager_based.coffee_make.generalization_execution import ControlSpec
        from robot_sim.tasks.manager_based.coffee_make.interaction_evaluation import EpisodeAcceptanceSpec
        from isaac_lane import configure_observation_scene
        from examples.scripts.debug.joint_head_real_owner import Owner
        from examples.scripts.debug.joint_head_fake_hardware import FakeSafety
        from arx_env.joint_head_jaw_mapping import configure_mapping
        from arx_env.joint_head_real_motion import feedback_to_command
        from jaw_mapping_v2 import COMMAND, FEEDBACK, GAP_MM
        from robot_workbench.physx import SimulationClock, PhysxBackend, IsaacPlant
        import numpy as np
        mapping = dict(schema_version='jaw_mapping_audit.v1', reinterpreted_mapping={side: dict(
            candidate_use_allowed=True, command_sdk_knots=COMMAND[side],
            feedback_sdk_knots=FEEDBACK[side], metal_knots_mm=GAP_MM[side]) for side in ('left','right')})
        save(args.output/'jaw-mapping.json',mapping)
        configure_mapping(args.output/'jaw-mapping.json')
        episode = EpisodeScene.from_dict(base['initial_suite']['episodes'][0])
        control=ControlSpec(240,8,2.,5.,300.)
        acceptance=replace(EpisodeAcceptanceSpec.development(),open_position_m=82.992/2000)
        cfg=make_generalization_env_cfg(control=control,acceptance=acceptance,
            asset_root=Path(base['pipeline_config']['assets']),cameras=True,camera_rig='d405',
            physics_device='cuda:0',pad_torsional_radius_m=.0015,solver_profile='contact_32_8',
            episode=episode,continuous_motion=True)
        configure_observation_scene(cfg)
        env=CoffeeMakeEnv(cfg)
        env.reset(options={'episode':episode})
        plant=IsaacPlant(env,trace_steps=48)
        clock=SimulationClock()
        events=[]
        def event(kind,**fields):
            events.append(dict(kind=kind,wall_ns=time.time_ns(),control_time_s=clock(),**fields))
        # The frozen A1-cal asset defines this probe plant. This is not a cold-start
        # calibration run or proof that installed calibration was estimated here.
        backend=PhysxBackend(plant,joint_offsets=[0.]*12,metal_gap_offset_mm=.008,event=event)
        owner=Owner(backend,FakeSafety,control_clock=clock,event=event)
        backend.owner=owner
        plant.trace_context=lambda: dict(control_time_s=clock(),
            backend_physics_step_before=backend.physics_step,
            published_sample_time_s=backend.sample_time,
            published_sample_wall_ns=backend.sample_wall_ns,
            published_command_sequences=dict(backend.sample_sequences),
            pending_applied_sequences=dict(backend.applied_sequences))
        def advance(seconds, *, physics=True):
            for _ in range(round(seconds*1200)):
                clock.advance(owner,backend,physics=physics)
        advance(.8)
        initial=owner.snapshot()
        save(args.output/'initial-owner.json',initial)
        if not initial.get('feedback_healthy') or not initial.get('stationary'):
            raise RuntimeError('initial actual PhysX feedback not healthy/stationary: '+str(initial.get('reason')))
        measurements=[]
        for side,start in (('left',0),('right',7)):
            before=backend.read_calibration_state()
            # The preview API takes measured inactive axes/jaws unchanged; the
            # MotionRunner performs its existing feedback -> command conversion.
            target=list(before['qpos'])
            target[start]+=.01
            review=owner.submit(dict(action='preview',active_arm=side,q_target=list(target)))
            owner.submit(dict(action='confirm',token=review['token'],
                              confirmation='START_REVIEWED_SINGLE_ARM',physical_clearance=True))
            for _ in range(1200*8):
                clock.advance(owner,backend)
                if owner.runner and owner.runner.status in ('FAULT','STOPPED'):
                    raise RuntimeError('Owner rejected PhysX motion: '+owner.runner.reason)
                if (owner.runner and owner.runner.plan.sha256==review['plan']['sha256']
                        and owner.runner.status=='READY'):break
            else:raise RuntimeError('Owner arrival timeout in S0 probe')
            after=backend.read_calibration_state()
            if abs(after['qpos'][start]-before['qpos'][start])<.001:
                save(args.output/'no-response.json',dict(before=before,after=after,target=target,
                    commanded_native=plant.target.cpu().tolist(),
                    tensor_native=plant.robot.data.joint_pos.cpu().tolist(),
                    physx_native=plant.robot.root_physx_view.get_dof_positions().tolist(),
                    joint_names=plant.robot.joint_names,
                    simulator_playing=env.sim.is_playing(),
                    simulation_time=env.sim.current_time,
                    stiffness=plant.robot.data.joint_stiffness.cpu().tolist(),
                    damping=plant.robot.data.joint_damping.cpu().tolist()))
                raise RuntimeError('no measured response to Owner target')
            cameras=plant.cameras()
            camera_rows={}
            for name,camera in cameras.items():
                rgb,depth=camera['rgb'],camera['depth']
                if rgb.shape!=(480,640,3) or depth.shape!=(480,640) or not np.isfinite(rgb).all():
                    raise RuntimeError('RGB-D shape/finite failure: '+name)
                if float(rgb.std())<1 or not np.isfinite(depth).any():
                    raise RuntimeError('empty camera observation: '+name)
                path=args.output/(side+'-'+name+'.npz')
                np.savez_compressed(path,rgb=rgb,depth=depth,qpos=after['qpos'],qvel=after['qvel'],
                                    physics_step=after['physics_step'],sample_time_s=after['sample_time_s'])
                camera_rows[name]=dict(path=path.name,sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                    rgb_shape=list(rgb.shape),depth_shape=list(depth.shape),intrinsics=camera['intrinsics'],
                    physics_step=after['physics_step'],clock_domain='simulation')
            measurements.append(dict(side=side,plan_sha256=review['plan']['sha256'],
                                     before=before,after=after,cameras=camera_rows))
        old_epoch=owner.epoch
        backend.send('left',list(feedback_to_command(backend.read_calibration_state()['qpos'])))
        barrier=len(events)
        owner.submit({'action':'stop'})
        advance(.04)
        if any(e['kind']=='applied' and e.get('epoch')==old_epoch for e in events[barrier:]):
            raise RuntimeError('old target applied after stop revocation')
        if not any(e['kind']=='revoked' for e in events[barrier:]):
            raise RuntimeError('pending stop race was not exercised')
        advance(.2,physics=False)
        if owner.control.state is not owner.states.FAULT:
            raise RuntimeError('frozen plant feedback did not trigger stale protection')
        save(args.output/'measurements.json',measurements)
        save(args.output/'events.json',events)
        save(args.output/'final-owner.json',owner.snapshot())
        result = dict(status='s0-calibration-physx-causality-passed',
                    scope='same calibration Owner send -> PhysX actual q/v -> three paired RGB-D; stop/stale injection',
                    collection_strategy_qualified=False,full_calibration_qualified=False,
                    coffee_task_qualified=False,hardware_qualified=False,
                    physical_step_count=backend.physics_step,control_time_s=clock(),
                    safety_profile='existing FakeSafety double; not production pedal/gain qualification',
                    holding_profile=backend.mode,camera_samples=len(measurements)*3,
                    teardown_verified=False)
        save(args.output/'final.json',result)
        return result
    except BaseException as error:
        # Isaac shutdown may terminate the interpreter; preserve the first error
        # before calling it, rather than trusting the launcher exit status.
        save(args.output/'final.json',dict(status='failed',error=str(error),
                                         traceback=traceback.format_exc(),teardown_verified=False))
        if 'events' in locals():save(args.output/'events.json',events)
        raise
    finally:
        from isaac_lane import close_runtime
        try:
            if 'plant' in locals():
                save(args.output/'readback-trace.json',dict(schema='owner_physx_readback_trace.v1',
                    limit_steps=plant.trace_steps,triggered=plant.trace_started,
                    trigger='first native target difference greater than 1e-8 from initial hold',
                    read_only=True,sync_behavior_changed=False,
                    source_freshness_qualified=False,
                    context_scope='backend feedback before this physical step is published',
                    joint_names=plant.robot.joint_names,rows=plant.readback_trace))
        finally:
            close_runtime(env,app)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--gpu',type=int,required=True)
    parser.add_argument('--dry-run',action='store_true')
    args=parser.parse_args()
    base=json.loads(args.base.read_text())
    checked=preflight(base)
    if args.output.exists():raise FileExistsError(args.output)
    if args.dry_run:
        print(json.dumps(dict(status='dry-run-passed',**checked)))
        return
    args.output.mkdir(parents=True,exist_ok=False)
    started=time.monotonic()
    try:
        result=run(args,base)
    except BaseException as error:
        save(args.output/'final.json',dict(status='failed',error=str(error),traceback=traceback.format_exc(),
                                         wall_s=time.monotonic()-started,**checked))
        raise
    save(args.output/'final.json',dict(**result,wall_s=time.monotonic()-started,**checked))


if __name__=='__main__':main()
