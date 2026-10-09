"""Local grasp/lift/release characterization, never a complete coffee episode."""
import argparse
from dataclasses import replace,asdict
import json
import math
import os
from pathlib import Path
import time


def main(args):
    args.output.mkdir(parents=True,exist_ok=False)
    os.environ['ROBOT_SIM_ASSET_ROOT']=str(args.assets)
    from isaaclab.app import AppLauncher
    app=AppLauncher(headless=True,enable_cameras=False,device='cuda:0',active_gpu=0,multi_gpu=False).app
    env=None;started=time.perf_counter();result=dict(passed=False,hardware_access=False,complete_pipeline=False,trials=[])
    try:
        from robot_sim.tasks.manager_based.coffee_make.env import CoffeeMakeEnv
        from robot_sim.tasks.manager_based.coffee_make.episode_scene import EpisodeScene
        from robot_sim.tasks.manager_based.coffee_make.generalization_env_cfg import make_generalization_env_cfg
        from robot_sim.tasks.manager_based.coffee_make.generalization_execution import ControlSpec,ActionRequest,StepEvent
        from robot_sim.tasks.manager_based.coffee_make.interaction_evaluation import EpisodeAcceptanceSpec
        from robot_sim.specs.action import ac_one_dual_arm_zero_home_action_spec,ArmCommand,DualArmCommand
        from isaac_lane import install_distance_replay,install_grasp_contact_adapter,close_runtime
        from gpu_acceptance import bind_runtime_support_gpu
        from robot_sim.asset_cfg.robots.ac_one_spec import ac_one_asset_spec
        from cup_presets import configure,contact_forces,gap_summary,diagnostic_summary
        from contract import demo_state
        from integration import _digest
        import torch
        proof=json.loads(args.contact.read_text())
        if not proof['passed'] or not proof['wrist_source_separation_proven']:raise ValueError('new nominal contact proof required')
        data=json.loads(args.input.read_text());q=proof['grasp_q_model'];right=data['simulation_initial_command']['target']['right_arm_joints']
        state=dict(demo_state(),left_arm_joints=q,right_arm_joints=right,left_gripper=82.992,right_gripper=82.992)
        episode=json.loads(args.suite.read_text())['episodes'][0]
        episode={**episode,'episode_id':'local-cup-preset-'+args.preset_id,
                 'metadata':dict(initial_robot_state=state,initial_robot_state_digest=_digest(state),gripper_mode='distance')}
        episode=EpisodeScene.from_dict(episode)
        control=ControlSpec(240,8,2.,5.,300.)
        acceptance=replace(EpisodeAcceptanceSpec.development(),open_position_m=.041496)
        cfg=make_generalization_env_cfg(control=control,acceptance=acceptance,asset_root=args.assets,
            cameras=False,camera_rig=None,physics_device='cuda:0',pad_torsional_radius_m=.0015,
            solver_profile='contact_32_8',episode=episode,continuous_motion=True)
        env=CoffeeMakeEnv(cfg);runtime=env._m2_runtime
        contact_policy=install_grasp_contact_adapter(runtime,diagnostic_compression=True)
        import isaaclab.sim as sim_utils
        material=configure(sim_utils.get_current_stage(),runtime.root,args.stiffness,args.damping)
        support=bind_runtime_support_gpu(runtime,ac_one_asset_spec(args.assets).urdf_path)
        result.update(preset_id=args.preset_id,material=material,support=support,contact_policy=contact_policy,
                      initialization='explicit reset into pinned grasp pose; no approach/path-success claim',
                      control=asdict(control),gripper_hold_seconds=.6,gripper_target_nominal_rubber_mm=62.)
        spec=ac_one_dual_arm_zero_home_action_spec()
        term=env.action_manager.get_term('guarded');native_process=term.process_actions
        def action(joints,gap):
            raw=list(spec.encode(DualArmCommand(ArmCommand(tuple(joints),True),ArmCommand(tuple(right),True))))
            raw[6]=float(gap);raw[13]=82.992
            return raw
        # Smooth .3-speed equivalent close, fixed .6s wait. All groups identical.
        n=math.ceil((82.992-64.992)/.6)
        closes=[action(q,82.992+(64.992-82.992)*i/n) for i in range(n+1)]
        close=action(q,64.992);opened=action(q,82.992)
        for trial in range(args.trials):
            print('TRIAL START',args.preset_id,trial,flush=True)
            trial_episode=replace(episode,episode_id=f'{episode.episode_id}-{trial}')
            _,info=env.reset(options={'episode':trial_episode});adapter=install_distance_replay(env,runtime,spec,
                dict(planner_id='local_cup_characterization',gripper_opening_mm=82.992,native_actions=closes+[opened]))
            records=[];hold=[];summary=None;observe=runtime.evaluator.observe;phase=['closing',0.]
            def observed(sample,scene,contacts,**kwargs):
                force=contact_forces(contacts,runtime.root,kwargs['dt'])
                event=observe(sample,scene,contacts,**kwargs)
                row=dict(phase=phase[0],physics_step=sample.physics_step,
                         jaw_positions_m=list(sample.jaw_positions),forces=force,
                         jaw_velocities_m_s=list(sample.jaw_velocities),
                         driver_saturation_estimate=[abs(2000*(.032496-p)-100*v)>=6. for p,v in zip(sample.jaw_positions,sample.jaw_velocities)] if phase[0]=='hold' else None,
                         cup_pose=asdict(sample.world_from_cup),
                         cup_tilt_rad=math.acos(max(-1.,min(1.,sample.world_from_cup.rotate((0.,0.,1.))[2]))),
                         event=event.reason if event else None)
                if event:
                    row['finger_cup_relative_speed_m_s']=sample.finger_cup_relative_speed_m_s
                    row['failure_contacts']=[asdict(contact) for contact in contacts]
                    row['accepted_body_band_m']=list(runtime.evaluator.cup.candidate_grasp_band_m)
                    row['contact_speed_limit_m_s']=acceptance.finger_contact_speed_limit(runtime.evaluator.history['grasp_confirmed'])
                records.append(row)
                if phase[0]=='hold':
                    phase[1]+=kwargs['dt'];hold.append({**row,'hold_time_s':phase[1]})
                return event
            runtime.evaluator.observe=observed
            def step(raw,stage):
                adapter.set_stage(stage)
                _,_,terminated,truncated,info=env.step(ActionRequest(runtime.loop.episode_id,runtime.loop.step_id,tuple(raw)))
                if terminated or truncated:raise RuntimeError('local native guard stopped: '+str(info.get('execution_report')))
                return info
            try:
                for raw in closes:step(raw,'close_ramp')
                phase[:]=['hold',0.]
                for _ in range(18):info=step(close,'close_and_hold')
                summary=gap_summary(hold)
                summary.update(native_grasp_confirmed=bool(runtime.evaluator.history['grasp_confirmed']),
                               maximum_cup_tilt_rad=max(r['cup_tilt_rad'] for r in records),
                               pad_mean_force_n={name:sum(r['forces'][name]['pad_n'] for r in hold[-48:])/48 for name in ('left_link7','left_link8')},
                               metal_mean_force_n={name:sum(r['forces'][name]['metal_n'] for r in hold[-48:])/48 for name in ('left_link7','left_link8')})
                summary['driver_saturation_fraction_estimate']=[sum(r['driver_saturation_estimate'][i] for r in hold[-48:])/48 for i in range(2)]
                summary['driver_saturation_method']='nominal Kp2000/Kd100 demand versus6N cap; not measured actuator force'
                print('HOLD RESULT',args.preset_id,trial,json.dumps(summary),flush=True)
                if not summary['gap_stable'] or not summary['native_grasp_confirmed']:raise RuntimeError('closed grasp did not stabilize')
                # Local lift goal is prepared by the same GPU IK and held pitch.
                lifted=proof['lift_q_model']
                phase[:]=['lift',0.]
                for i in range(51):
                    u=i/50;s=10*u**3-15*u**4+6*u**5
                    step(action([a+(b-a)*s for a,b in zip(q,lifted)],64.992),'lift')
                for _ in range(6):step(action(lifted,64.992),'lift_hold')
                sample,_=runtime.read(runtime.loop.physics_steps)
                summary['short_lift_verified']=bool(runtime.evaluator.history['table_separated'] and runtime.evaluator.facts['grasp_confirmed'])
                if not summary['short_lift_verified']:raise RuntimeError('short lift did not retain bilateral grasp')
                # Release to the original table is a separate local test goal,
                # not the coffee evaluator's tray-placement task. Motion/support
                # guards remain; this observer checks release contact and depth.
                previous=[runtime.evaluator.previous.physics_step]
                def release_observe(sample,scene,contacts,*,dt,complete,commanded_gripper_open=None):
                    if not complete or sample.physics_step!=previous[0]+1:raise RuntimeError('local release feedback incomplete/stale')
                    previous[0]=sample.physics_step;runtime.evaluator.previous=sample
                    f=contact_forces(contacts,runtime.root,dt)
                    records.append(dict(phase=phase[0],physics_step=sample.physics_step,jaw_positions_m=list(sample.jaw_positions),forces=f,cup_pose=asdict(sample.world_from_cup),
                                        cup_tilt_rad=math.acos(max(-1.,min(1.,sample.world_from_cup.rotate((0.,0.,1.))[2])))))
                    limit=acceptance.residual_force_max_n if commanded_gripper_open else acceptance.finger_force_max_n
                    if any(v['pad_n']+v['metal_n']>limit or v['max_other_metal_penetration_m']>.003 for v in f.values()):
                        return runtime.evaluator.fail('local_release_force_or_metal_depth_limit')
                    for contact in contacts:
                        if contact.separation_m>0 and not any(contact.impulse):continue
                        if runtime.root+'/Cup' in contact.actors:
                            ci=contact.actors.index(runtime.root+'/Cup');other=contact.actors[1-ci]
                            if contact.shapes[1-ci]==runtime.evaluator.table_shape:
                                local=sample.world_from_cup.inverse().transform_point(contact.position)
                                if abs(local[2])>acceptance.surface_tolerance_m or math.sqrt(sum(v*v for v in contact.impulse))/dt>acceptance.support_force_max_n:
                                    return runtime.evaluator.fail('local_table_support_limit')
                            if other.startswith(runtime.root+'/CoffeeMachine') and contact.separation_m<-.003:
                                return runtime.evaluator.fail('local_cup_machine_depth_limit')
                            if other.startswith(runtime.root+'/Robot/'):
                                shape=contact.shapes[1-ci]
                                if other.rsplit('/',1)[-1] not in f or not (shape==other+'/RealFingerRubberPad' or shape.startswith(other+'/collisions/')):
                                    return runtime.evaluator.fail('local_cup_robot_forbidden_contact')
                            elif contact.shapes[1-ci]!=runtime.evaluator.table_shape and not other.startswith(runtime.root+'/CoffeeMachine'):
                                return runtime.evaluator.fail('local_cup_unknown_contact')
                        elif any(actor.startswith(runtime.root+'/Robot/') for actor in contact.actors):
                            return runtime.evaluator.fail('robot_forbidden_contact')
                    if sample.finger_cup_relative_speed_m_s>acceptance.contact_speed_m_s:
                        return runtime.evaluator.fail('local_release_contact_speed_limit')
                    return None
                runtime.evaluator.observe=release_observe
                # Place on the original support before opening; a25mm free drop
                # measures impact/tipping rather than the preset's release.
                phase[:]=['lower_to_table',0.]
                for i in range(51):
                    u=i/50;s=10*u**3-15*u**4+6*u**5
                    step(action([a+(b-a)*s for a,b in zip(lifted,q)],64.992),'lift')
                for _ in range(6):step(close,'close_and_hold')
                # Remove grip loading before release, without raising any force limit.
                phase[:]=['unload',0.]
                nominal=proof['terminal_jaw_position_m']*2000
                unload_steps=max(1,math.ceil((nominal-64.992)/.06))
                for i in range(unload_steps+1):step(action(q,64.992+(nominal-64.992)*i/unload_steps),'close_and_hold')
                open_steps=max(1,math.ceil((82.992-nominal)/.06))
                phase[:]=['release',0.]
                for i in range(open_steps+1):step(action(q,nominal+(82.992-nominal)*i/open_steps),'open')
                for _ in range(30):step(action(q,82.992),'release_hold')
                sample,_=runtime.read(runtime.loop.physics_steps)
                summary['release_verified']=bool(
                    all(abs(v-.041496)<=acceptance.open_tolerance_m for v in sample.jaw_positions)
                    and math.sqrt(sum(v*v for v in sample.cup_velocity))<=acceptance.stable_speed_m_s
                    and math.sqrt(sum(v*v for v in sample.cup_angular_velocity))<=acceptance.stable_angular_speed_rad_s
                    and abs(runtime.evaluator._lowest_z(sample.world_from_cup)-runtime.evaluator.initial_bottom_z)<=acceptance.surface_tolerance_m)
                summary['passed']=bool(summary['release_verified'])
                summary['maximum_cup_tilt_rad']=max(r['cup_tilt_rad'] for r in records)
                summary['maximum_cup_tilt_deg']=math.degrees(summary['maximum_cup_tilt_rad'])
                summary['maximum_observed_metal_compression_m']=max(v['max_metal_penetration_m'] for r in records for v in r['forces'].values())
                result['trials'].append(summary)
            except Exception as error:
                result['trials'].append(dict(passed=False,error=str(error),hold_summary=summary or (gap_summary(hold) if len(hold)>=48 and abs(hold[-1]['hold_time_s']-.6)<.005 else None),
                                            native_facts=dict(runtime.evaluator.facts),native_audit=runtime.evaluator.last_audit,
                                            failure_sample=records[-1] if records and records[-1].get('event') else None))
            finally:
                result['trials'][-1].update(diagnostic_summary(records))
                print('TRIAL RESULT',args.preset_id,trial,json.dumps(result['trials'][-1]),flush=True)
                runtime.evaluator.observe=observe
                term.process_actions=native_process;term.spec=spec
                runtime.loop.action_spec=runtime.action=spec;runtime.loop.previous=tuple(runtime.home)
                term._raw[:]=torch.tensor([runtime.home],dtype=term._raw.dtype,device=term.device)
                (args.output/f'trial-{trial}.json').write_text(json.dumps(records))
        result['passed']=len(result['trials'])==args.trials and all(t['passed'] for t in result['trials'])
        result['material']['active_physics_verification_required']=not result['passed']
    except Exception as error:
        import traceback
        result.update(error=str(error),traceback=traceback.format_exc())
    finally:
        from isaac_lane import close_runtime
        result['seconds']=time.perf_counter()-started
        (args.output/'result.json').write_text(json.dumps(result,indent=2))
        print(json.dumps(result,indent=2),flush=True)
        close_runtime(env,app)
    print(json.dumps(result,indent=2));return 0 if result['passed'] else 1


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('input','suite','assets','contact','output'):p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--preset-id',required=True);p.add_argument('--stiffness',type=float,required=True)
    p.add_argument('--damping',type=float,default=10.);p.add_argument('--trials',type=int,default=3,choices=(1,3))
    raise SystemExit(main(p.parse_args()))
