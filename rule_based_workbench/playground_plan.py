"""Geometric planning/export for the development playground; no physical rollout claim."""
from contextlib import ExitStack
from copy import deepcopy
import math
import struct
import time

from integration import _digest, SIM_PLANNER_ID, SIM_POLICY_ID
from joint_space import convert_state, freeze_reported, gripper_offset_mm, project_model_state


def build(planner, observation, initial, source, identity, speed_scale=.3, *, environment='physical'):
    from robot_sim.asset_cfg.objects.coffee_machine import RigidTransform
    from robot_sim.policies.continuous_motion import MotionLimits, continuous_trajectory
    from robot_sim.specs.action import ArmCommand, DualArmCommand, ac_one_dual_arm_zero_home_action_spec
    from gripper_distance_plan import distance_plan

    if initial.get('passed') is not True or not planner.identity.get('chassis_collision'):
        raise ValueError('complete route and chassis collision coverage required')
    if identity['control'].get('physics_hz') != 240 or identity['control'].get('substeps') != 8:
        raise ValueError('playground requires 240 Hz physics / 8 substeps = 30 Hz control')
    closing=initial['selected']['closing']
    contact=closing['terminal_state']
    if abs(contact['left_joint7']-contact['left_joint8'])>1e-9 or 'world_from_cup' not in closing:
        raise ValueError('symmetric linked-finger closing with centered cup pose required; replan old route')
    spec = ac_one_dual_arm_zero_home_action_spec()
    sample = observation.sample
    right = tuple(sample.arm_positions[6:])
    current = list(sample.arm_positions[:6])
    opening = planner.open_position_m*2000
    initial_gap = source['initial_state']['model']['left_gripper']
    right_gap = source['initial_state']['model']['right_gripper']
    ticks, actions, stages, corrections = [], [], [], []

    def emit(q, opened):
        from isaac_lane import physical
        model,changes=project_model_state(dict(left_arm_joints=list(q),right_arm_joints=list(right)),spec,environment=environment,source=source)
        if changes:corrections.append(dict(tick=len(ticks),changes=changes))
        action = list(spec.encode(DualArmCommand(ArmCommand(tuple(model['left_arm_joints']), opened),
                                               ArmCommand(tuple(model['right_arm_joints']), True))))
        ticks.append(dict(tick=len(ticks), time_s=len(ticks)/30, target=physical(spec.decode(action), opening)))
        ticks[-1]['target']['right_gripper']=right_gap
        actions.append(action)

    def jaw_ramp(name,start,end):
        if abs(end-start)<1e-9:return
        stages.append(dict(name=name,start_tick=len(ticks),start_s=len(ticks)/30))
        steps=math.ceil(abs(end-start)/.2)
        for index in range(steps+1):
            emit(current,True)
            ticks[-1]['target']['left_gripper']=start+(end-start)*index/steps

    def stage(name, checks=(), hold=0, opened=False):
        nonlocal current
        stages.append(dict(name=name,start_tick=len(ticks),start_s=len(ticks)/30))
        if hold:
            for _ in range(round(hold*30)): emit(current,opened)
            return
        # Preserve explicit route stops, including the rear/front transfer corners.
        groups, group = [], [current]
        for row in checks:
            group.append(row['q'])
            if row.get('stop_after'):
                groups.append(group); group=[group[-1]]
        if len(group)>1: groups.append(group)
        limits = (MotionLimits(.15,.5,4) if name in ('lift','lower','separate','retreat') else
                  MotionLimits(2,4.938271604938271,123.45679012345677) if opened else
                  MotionLimits(1.2,1.2 if name=='raise' else 2,60))
        for group in groups:
            trajectory = continuous_trajectory(group,1/30,limits,clock='s_curve' if name=='home' else 'quintic')
            for q in trajectory.positions: emit(q.tolist(),opened)
            current=list(group[-1])

    stages.append(dict(name='initial_hold',start_tick=0,start_s=0.))
    emit(current,True)
    ticks[-1]['target']['left_gripper']=initial_gap
    jaw_ramp('initial_open',initial_gap,opening)
    approach=list(initial['approach'].values())
    stage('approach_pre',approach[:1],opened=True)
    stage('approach',approach[1:],opened=True)
    stage('preclose_settle',hold=.1,opened=True)
    stage('close_and_hold',hold=.6)
    chosen=initial['selected']; route=chosen['placement']['stages']
    stage('lift',chosen['lift']['checks'])
    for name in ('raise','transport','lower'): stage(name,route[name])
    stage('support_settle',hold=.1)
    # Start from the commanded preload, not the rigid model's contact gap:
    # the real cup may compress or be absent, so jumping to contact can fault.
    contact=chosen['closing']['terminal_state']
    contact_gap=(contact['left_joint7']+contact['left_joint8'])*1000
    release_gap=65.0-gripper_offset_mm(source['calibration_profile'])
    if not math.isfinite(release_gap) or not 0 <= release_gap <= opening:
        raise ValueError('geometric release gap outside model opening range')
    stages.append(dict(name='open',start_tick=len(ticks),start_s=len(ticks)/30))
    increments=max(1,math.ceil((opening-release_gap)/.2))
    for index in range(increments+1):
        emit(current,True)
        ticks[-1]['target']['left_gripper']=min(opening,release_gap+index*.2)
    stage('release_hold',hold=.2,opened=True)
    for name in ('separate','retreat'): stage(name,route[name],opened=True)
    stage('home',route.get('home') or [dict(q=list(sample.arm_positions[:6]))],opened=True)
    jaw_ramp('home_gripper_restore',opening,initial_gap)
    stage('home_hold',hold=.3,opened=True)
    for tick in ticks[stages[-1]['start_tick']:]:tick['target']['left_gripper']=initial_gap
    # Remove empty stages (e.g. separate) so the stage contract remains unambiguous.
    stages=[s for i,s in enumerate(stages) if i==len(stages)-1 or s['start_tick']<stages[i+1]['start_tick']]
    plan=dict(schema_version='dual_gpu_pipeline.v1',lane='dsw-scene',run_id=source['run_id']+'-playground',
              source_run_id=source['run_id'],source_sequence=source['sequence'],scene_digest=source['scene']['scene_digest'],
              calibration_profile_digest=_digest(source['calibration_profile']),planner_id=SIM_PLANNER_ID,policy_id=SIM_POLICY_ID,
              ticks=ticks,timeline_digest=_digest(ticks),native_actions=actions,native_substeps=[8]*len(ticks),
              duration_s=len(ticks)/30,control=identity['control'],native_identity=identity,
              motion_profile='continuous-target',gripper_opening_mm=opening,stages=stages,
              gripper_release=dict(schema_version='commanded_gap_release.v2',
                  initial_model_gap_mm=release_gap,initial_gap_source='reviewed closed metal target minus model offset',
                  geometric_contact_model_gap_mm=contact_gap,
                  contact_state_digest=_digest(contact),maximum_step_mm=.2,rate_mm_s=6.,
                  arm_motion=False,tracking_protection='opening lag guarded; wider contact-limited gap allowed during unloading'),
              simulation=dict(validated=False,outcome='not_run'),initial_state_digest=_digest(source['initial_state']),
              position_normalization=dict(environment=environment,corrections=corrections,
                  input_timeline_digest=_digest(ticks),row_index_space='original model ticks before reported conversion and time resampling'))
    freeze_reported(plan,source)
    plan=distance_plan(plan,source,close_metal_gap_mm=65.)
    audit(plan,source,planner,observation,initial)
    return plan if speed_scale is None else retime(plan,source,speed_scale,environment=environment)


def retime(plan,source,speed_scale,*,environment='physical'):
    """Change timing along the checked polyline, without another geometric audit."""
    from half_speed_plan import resample_speed
    # The source binding is checked again by the caller; here no robot is accessed.
    from real_contract import namespace
    spec=namespace(environment)
    binding=dict(calibration_accepted=True,evidence_source=spec['evidence'],robot_id=source['robot_id'],
                 camera_serials=source['camera_serials'],source_digest=_digest(source),
                 calibration_profile_digest=_digest(source['calibration_profile']),initial_state=source['initial_state'])
    if environment=='fake':
        from robot_workbench.sample_clock import clock_for_source
        binding.update(environment='fake',simulation_only=True,clock=clock_for_source(source,environment))
    if plan.get('physical_gpu_conversion'):
        from physical_motiongen import retime_physical
        return retime_physical(plan,source,binding,speed_scale,environment=environment)
    return resample_speed(plan,source,binding,speed_scale,environment=environment)


def audit(plan,source,planner,observation,initial):
    """Check the final command polyline once, before any speed resampling."""
    from robot_sim.asset_cfg.objects.coffee_machine import RigidTransform
    plan.pop('planning_validation',None)
    if plan['timeline_digest'] != _digest(plan['ticks']):
        raise ValueError('timeline digest differs before geometry audit')
    geo,start=planner.geometry(observation)
    fk=planner.fk
    initial_cup=observation.sample.world_from_cup
    placed_cup=RigidTransform(**initial['selected']['placement']['candidate']['world_from_cup'])
    contact=initial['selected']['closing']['terminal_state']
    # Geometry uses contact-limited fingers; the command requests elastic preload.
    # This is an explicit held-cup assumption, never evidence of force closure.
    closing=initial['selected']['closing']
    if abs(contact['left_joint7']-contact['left_joint8'])>1e-9:
        raise ValueError('asymmetric contact state cannot represent linked fingers')
    held_cup=RigidTransform(**closing['world_from_cup'])
    relation=geo.mount.compose(fk.pose(fk.forward(contact)['left_link6'])).inverse().compose(held_cup)
    from gripper_distance_plan import HOLD_STAGES
    previous=start; previous_held=False; count=0; started=time.monotonic()
    contexts={}
    with ExitStack() as stack:
        for tick in plan['ticks']:
            name=next((s['name'] for s in reversed(plan['stages']) if s['start_tick']<=tick['tick']),'approach_pre')
            model=convert_state(tick['target'],source['calibration_profile'],to_reported=False)
            state={**start,**dict(zip((f'left_joint{i}' for i in range(1,7)),model['left_arm_joints'])),
                   **dict(zip((f'right_joint1{i}' for i in range(1,7)),model['right_arm_joints']))}
            held=name in HOLD_STAGES
            # During preload unloading the rigid cup still limits the fingers.
            # As in holding, a command below contact is not rigid penetration.
            left_gap=max(model['left_gripper']/2000,contact['left_joint7']) if name=='open' else model['left_gripper']/2000
            state.update(left_joint7=contact['left_joint7'] if held else left_gap,
                         left_joint8=contact['left_joint8'] if held else left_gap,
                         right_joint17=model['right_gripper']/2000,right_joint18=model['right_gripper']/2000)
            released=name in ('open','release_hold','separate','retreat','home','home_gripper_restore','home_hold')
            cup=placed_cup if released else initial_cup
            if name=='close_and_hold' and (tick['tick']==0 or not previous_held):
                count+=audit_closing(geo,fk,previous,contact,initial_cup,closing)
                previous=state  # The checked coupled jaw/cup sweep ends at this hold.
            options=dict(world_from_cup=cup,tool_from_cup=relation if held else None,
                         grasp_contact=held or name in ('open','release_hold','separate'),
                         support_contact=name in ('lower','support_settle') or released,
                         initial_table_contact=name in ('initial_hold','initial_open','approach_pre','approach','preclose_settle','close_and_hold','lift'))
            key=(released,held,options['grasp_contact'],options['support_contact'],options['initial_table_contact'])
            if key not in contexts:
                contexts[key]=stack.enter_context(geo.trajectory_state_reuse(fk,**options))
            result=contexts[key].check(previous,state,maximum_joint_step=.01)
            count+=result.get('actual_samples',0)
            if not result['free']:
                raise ValueError(f'playground geometry rejected tick {tick["tick"]} / {name}: {result.get("failures",[])[:3]}')
            previous=state
            previous_held=held
    plan['planning_validation']=dict(schema_version='geometric_path_validation.v1',passed=True,
        timeline_digest=plan['timeline_digest'],source_digest=_digest(source),collision_coverage=['base_link','left','right'],
        max_sample_step_rad=.01,samples=count,input_ticks=len(plan['ticks']),elapsed_s=time.monotonic()-started,
        chassis_collision=planner.identity['chassis_collision'],
        state_reuse=dict(scope='exact states within this audit and fixed cup/contact semantics; closing sweep excluded',
            contexts=[dict(semantics=list(key),**context.stats) for key,context in contexts.items()],
            totals={name:sum(context.stats[name] for context in contexts.values()) for name in ('visits','hits','misses','stored','capacity_bypasses')}),
        grasp_assumption='cup held at geometric bilateral pad contact; commanded preload is not geometric penetration')


def audit_closing(geo,fk,start,terminal,initial_cup,closing):
    """Recheck the native coupled-jaw/cup sweep before switching to held geometry."""
    from robot_sim.asset_cfg.objects.coffee_machine import RigidTransform
    samples=closing.get('samples',[])
    if closing.get('passed') is not True or len(samples)<2:
        raise ValueError('complete symmetric closing sweep required')
    if any(abs(start[k]-terminal[k])>1e-8 for k in start if k not in ('left_joint7','left_joint8')):
        raise ValueError('closing must keep both arms and right jaw stationary')
    first=RigidTransform(**samples[0]['world_from_cup'])
    if (first.position!=initial_cup.position or first.orientation_wxyz!=initial_cup.orientation_wxyz
            or any(abs(samples[0]['jaws'][k]-start[k])>1e-8 for k in ('left_joint7','left_joint8'))):
        raise ValueError('closing sweep does not begin at the observed cup and approach state')
    count=0; last_checked=None
    names=tuple(sorted(start))
    for index,row in enumerate(samples):
        pose=RigidTransform(**row['world_from_cup']);jaw=row['jaws']
        if (abs(jaw['left_joint7']-jaw['left_joint8'])>1e-9
                or pose.orientation_wxyz!=first.orientation_wxyz):
            raise ValueError('closing sweep must use linked fingers and fixed cup orientation')
        previous=samples[max(0,index-1)];old=RigidTransform(**previous['world_from_cup'])
        steps=max(1,math.ceil(max(abs(jaw[k]-previous['jaws'][k]) for k in jaw)/.00005),
                  math.ceil(math.dist(old.position,pose.position)/.00005))
        for step in range(steps+1):
            alpha=step/steps
            state={**start,**{k:previous['jaws'][k]+alpha*(v-previous['jaws'][k]) for k,v in jaw.items()}}
            cup=RigidTransform(tuple(a+alpha*(b-a) for a,b in zip(old.position,pose.position)),pose.orientation_wxyz)
            values=(*(state[n] for n in names),*cup.position,*cup.orientation_wxyz)
            key=struct.pack('!'+str(len(values))+'d',*values)
            if key==last_checked:continue  # Only the identical previous successful sample, without rounding.
            result=geo.audit({n:fk.pose(v) for n,v in fk.forward(state).items()},cup,
                             grasp_contact=True,initial_table_contact=True)
            count+=1
            if not result.free:raise ValueError('coupled closing sweep collision: '+repr(result.violations[:3]))
            last_checked=key
    last=samples[-1]
    if (any(abs(last['jaws'][k]-terminal[k])>1e-9 for k in last['jaws'])
            or _digest(last['world_from_cup'])!=_digest(closing['world_from_cup'])):
        raise ValueError('closing sweep terminal differs from held geometry')
    return count
