"""Fake complete orchestration and CPU geometry/clock checks; no GPU or robot access."""
import argparse
from copy import deepcopy
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
from types import ModuleType, SimpleNamespace as NS
from unittest.mock import patch

import numpy as np


def main(native_code, urdf):
    root = native_code/'source/robot_sim/robot_sim'
    for suffix in ('', 'asset_cfg', 'asset_cfg.objects', 'asset_cfg.robots', 'specs', 'policies',
                   'tasks', 'tasks.manager_based', 'tasks.manager_based.coffee_make'):
        module = ModuleType('robot_sim'+('.'+suffix if suffix else ''))
        module.__path__ = [str(root.joinpath(*suffix.split('.'))) if suffix else str(root)]
        sys.modules[module.__name__] = module
    from robot_sim.asset_cfg.objects.coffee_machine import RigidTransform as Pose
    from robot_sim.tasks.manager_based.coffee_make.episode_planner import EpisodePlanner
    from test_motiongen_pipeline import motiongen_fixture
    from motiongen_plan import LEFT, RIGHT, gripper_geometry, plan_loaded, scaled_stages, encode_stage, target, enforce_stage_motion_limits
    from robot_sim.specs.action import ac_one_dual_arm_zero_home_action_spec
    from isaac_lane import native_target, simulation_initial_command, SimDistanceActionSpec
    from robot_sim.specs.action import ArmCommand, DualArmCommand
    from integration import DualGpuPipeline, _digest
    import motiongen_plan

    spec=ac_one_dual_arm_zero_home_action_spec()
    motion=encode_stage(dict(name='approach',rows=[target([i*.008,0.,0.,0.,0.,0.],
        [0.]*6,82.992,82.992) for i in range(3)]),spec,82.992,{})
    wait=encode_stage(dict(name='close_and_hold',rows=[motion['rows'][-1]]*18),spec,82.992,{})
    adjusted,evidence=enforce_stage_motion_limits([motion,wait],
        dict(max_target_velocity_rad_s=2.,max_target_acceleration_rad_s2=5.),spec,82.992)
    assert len(evidence)==1 and evidence[0]['control_validation']['passed']
    assert adjusted[0]['rows'][0]==motion['rows'][0] and adjusted[0]['rows'][-1]==motion['rows'][-1]
    assert adjusted[1]==wait and len(adjusted[1]['rows'])==18

    def forbidden(*args, **kwargs):
        raise AssertionError('legacy CPU planner/IK/continuous trajectory was invoked')

    class FK:
        movable = set((*LEFT, *RIGHT, 'left_joint7', 'left_joint8', 'right_joint17', 'right_joint18'))
        inverse = forbidden
        def forward(self, state):
            result = {'base_link': np.eye(4)}
            for side, names, offset in (('left', LEFT, 0), ('right', RIGHT, 10)):
                yaw = state[names[3]]
                rotation = np.array([[math.cos(yaw), -math.sin(yaw), 0.], [math.sin(yaw), math.cos(yaw), 0.], [0., 0., 1.]])
                wrist = np.eye(4); wrist[:3, :3] = rotation; wrist[:3, 3] = [state[name] for name in names[:3]]
                result[f'{side}_link{offset+6}'] = wrist
                for finger, y, sign in ((7, .02721, 1), (8, -.023638, -1)):
                    local = np.eye(4); local[:3, 3] = [.08397, y+sign*state[f'{side}_joint{offset+finger}'], -.0062702]
                    result[f'{side}_link{offset+finger}'] = wrist@local
            return result
        def pose(self, matrix):
            yaw = math.atan2(matrix[1, 0], matrix[0, 0])
            return Pose(tuple(matrix[:3, 3]), (math.cos(yaw/2), 0., 0., math.sin(yaw/2)))

    class Geo:
        mount = Pose()
        whole = NS(ignores=[frozenset(('left_link1','left_link2'))])
        def __init__(self, scene, fk):
            self.scene, self.fk, self.checked = scene, fk, []
        def audit(self, frames, cup, **options):
            self.checked.append((frames, cup, options))
            return NS(free=True, violations=())
        def closing_sweep(self, fk, state, cup):
            terminal = dict(state, left_joint7=.037, left_joint8=.037)
            return dict(passed=True, terminal_state=terminal, world_from_cup=asdict(cup),
                        samples=[dict(jaws={key: state[key] for key in ('left_joint7', 'left_joint8')}, world_from_cup=asdict(cup)),
                                 dict(jaws={key: terminal[key] for key in ('left_joint7', 'left_joint8')}, world_from_cup=asdict(cup))])
        def trajectory_state_reuse(self, fk, *, world_from_cup, tool_from_cup=None, **flags):
            geo = self
            class Context:
                def __enter__(self): return self
                def __exit__(self, *args): pass
                def check(self, start, goal, **kwargs):
                    for state in (start, goal):
                        frames = {name: fk.pose(matrix) for name, matrix in fk.forward(state).items()}
                        cup = frames['left_link6'].compose(tool_from_cup) if tool_from_cup else world_from_cup
                        result = geo.audit(frames, cup, **flags)
                        if not result.free: return dict(free=False, failures=['fixture'])
                    return dict(free=True, actual_samples=2)
            return Context()

    class Engine:
        twist = False
        projection_noise = False
        def __init__(self, config, world, model_report=None):
            self.config = deepcopy(config); self.receipts = []
            report={}
            self.cup_contact=NS(certify_held=lambda *args:dict(passed=True),report=report,
                allow_claw_contact=lambda allowed:report.update(current_claw_allowance_m=.003 if allowed else 0.))
            self.seed_resets = 0
            def reset_seed(): self.seed_resets += 1
            self.generator = NS(update_world=lambda world: None, detach_object_from_robot=lambda **kw: None,
                                reset_seed=reset_seed)
            self.tensor_args=NS(to_device=lambda q:q)
            def fk(q):
                positions=NS()
                positions.detach=lambda:positions
                positions.cpu=lambda:positions
                positions.tolist=lambda:[row[:3] for row in q]
                return NS(ee_position=positions)
            self.generator.kinematics=NS(get_state=fk)
            self.initialization_seconds = self.warmup_seconds = 0.
            self.self_collision_partition = {'fixture': True}
        def warmup(self): pass
        def validate_continuous_path(self,path):
            from integration import _digest
            return dict(passed=True,path_sha256=_digest(path),fixture=True)
        def update_world(self, world): self.generator.update_world(world)
        def update_locked_joints(self, values): self.config['kinematics']['lock_joints'] = dict(values)
        def path(self,start,end):
            steps=max(2,math.ceil(max(abs(b-a) for a,b in zip(start,end))/.0045))
            return [[a+(b-a)*i/steps for a,b in zip(start,end)] for i in range(steps+1)]
        def plan_pose(self, start, goals, *, name, hold_orientation=False, linear_approach=False, linear_axis=None,project_to_goal_frame=True):
            end = [*goals[0]['position'], *start[3:]]
            rows=self.path(start,end)
            if self.twist and name.endswith('/cross'):rows[len(rows)//2][3]=.01
            if self.projection_noise and name in ('approach_pre','retreat_insert'):
                rows[len(rows)//2][3]=0.  # Owner lower bound is model +.002 in the projection fixture.
            receipt = dict(stage=name, method='MotionGen.plan_single' if len(goals) == 1 else 'MotionGen.plan_goalset', success=True)
            self.receipts.append(receipt)
            return rows, receipt
        def plan_joint(self, start, goal, *, name):
            rows=self.path(start,goal)
            receipt = dict(stage=name, method='MotionGen.plan_single_js', success=True)
            self.receipts.append(receipt); return rows, receipt

    source, _ = motiongen_fixture()
    state = dict.fromkeys(FK.movable, 0.)
    state.update(dict(zip(LEFT, (.1, .2, .3, 0., 0., 0.))))
    state.update(dict(zip(RIGHT, (.1, .2, .3, 0., 0., 0.))))
    for name in ('left_joint7', 'left_joint8', 'right_joint17', 'right_joint18'): state[name] = .01
    scene = NS(world_from_support=Pose((.4, 1., .3)), support_shape_path='tray',
               sites=(NS(site_id='liquid_outlet',position_world_m=(.4,.2,.5)),),
               shapes=(NS(shape_path='machine', world_from_shape=Pose((.4, 1., .5)), dimensions_m=(.2, .2, .4)),))
    cup = Pose((.3, .2, .3))
    fk = FK()
    planner = NS(fk=fk, home_state=state, open_position_m=.041496000000000005,PLACEMENT_TRACKING_RESERVE_M=.002,
                 parts=[NS(name='palm', link='left_link6', vertices=[(x, y, z) for x in (-.01, .01) for y in (-.01, .01) for z in (-.01, .01)])],
                 cup=NS(bands=[NS(shape_path='body', radius_m=.0367, z_min_m=0., z_max_m=.13)]))
    planner.geo = Geo(scene, fk)
    observation = NS(scene=scene, sample=NS(world_from_cup=cup))
    data = dict(urdf_sha256=hashlib.sha256(urdf.read_bytes()).hexdigest(), planner_identity={'fixture': 'native'},
                native_identity=dict(control=dict(physics_hz=240, substeps=8, max_target_velocity_rad_s=2., max_target_acceleration_rad_s2=5.)))
    grip = gripper_geometry(planner, urdf)
    spec = ac_one_dual_arm_zero_home_action_spec()
    reset_action = spec.encode(DualArmCommand(ArmCommand(tuple(state[n] for n in LEFT), True),
                                             ArmCommand(tuple(state[n] for n in RIGHT), True)))
    data['simulation_initial_command'] = simulation_initial_command(reset_action, spec, grip['open_travel_mm'], 0)
    # Feedback is not a command: retain real joint/gap noise in the final audit.
    state.update({name: state[name]+.0002 for name in RIGHT})
    for name in ('left_joint7', 'left_joint8', 'right_joint17', 'right_joint18'):
        state[name] = grip['open_travel_mm']/2000-.000005
    state['right_joint18'] += .000001
    assert abs(grip['native_travel_offset_mm']-.008) < 1e-9
    assert abs(grip['open_travel_mm']-82.992) < 1e-9 and abs(grip['closed_travel_mm']-64.992) < 1e-9
    grasp = dict(candidate_id='grasp', world_from_pregrasp=asdict(Pose((.15, .2, .3))),
                 world_from_tool=asdict(Pose((.2, .2, .3))), cup_from_tool=asdict(Pose((-.1, 0., 0.))))
    placement = dict(candidate_id='placement', world_from_cup=asdict(Pose((.4, .2, .3))))
    targets = dict(grasp_candidates=[grasp], placement_candidates=[placement],
        placement_goal=dict(schema_version='cup_outlet_goal.v1',metric='cup_mouth_world_xy_to_liquid_outlet',
            tolerance_m=.01,target_tolerance_m=.008,tracking_reserve_m=.002))

    model_calls=[]
    def model(*args, **kwargs):
        model_calls.append(1)
        return dict(kinematics=dict(cspace={}, lock_joints={name: args[3][name] for name in FK.movable- set(LEFT)})), dict(
            kinematics=dict(joints={name: {'velocity': 5.5} for name in LEFT}))

    def route(*args, **kwargs):
        return dict(stages=[dict(name=name, world_from_tool=asdict(Pose(point))) for name, point in
            (('lift', (.2, .2, .32)), ('raise', (.2, .2, .35)), ('withdraw', (.2, .17, .35)),
             ('cross', (.3, .17, .35)), ('front_height', (.3, .17, .305)), ('insert', (.3, .2, .305)), ('lower', (.3, .2, .3)))])

    legacy_trajectory = ModuleType('robot_sim.policies.continuous_motion')
    legacy_trajectory.continuous_trajectory = forbidden
    def closing_fixture(engine,planner,state,cup,**options):
        from test_grasp_contact_certificate import certificate_fixture
        pose=cup.inverse().compose(planner.geo.mount.compose(planner.fk.pose(planner.fk.forward(state)['left_link6'])))
        proof=certificate_fixture(options['source_digest'],options['data']['urdf_sha256'],
            exported_geometry_sha256=_digest(options['data']),candidate_id=options['candidate_id'],
            actual_cup_from_tool=asdict(pose),grasp_q_model=[state[n] for n in LEFT])
        proof['tool_from_held_cup']=asdict(pose.inverse())
        proof['certificate_digest']=_digest({k:v for k,v in proof.items() if k!='certificate_digest'})
        engine.grasp_contact_certificate=proof
        return dict(passed=True,terminal_state=dict(state,left_joint7=.038,left_joint8=.038),
            world_from_cup=asdict(cup),contact_certificate=proof,
            gpu_sweep=dict(passed=True,contact_certificate_digest=proof['certificate_digest']))
    patches = [patch.object(EpisodePlanner, 'initial', forbidden),
        patch.dict(sys.modules, {'robot_sim.policies.continuous_motion': legacy_trajectory}),
        patch('curobo_robot_model.build_conservative_robot_model', side_effect=model),
        patch('curobo_robot_model.fixed_world_mesh_descriptors', return_value=[]),
        patch('motiongen_targets.generate_targets', return_value=targets),
        patch('motiongen_targets.straight_waypoints',side_effect=lambda begin,end:[end]), patch('motiongen_targets.carry_targets', side_effect=route),
        patch('motiongen_motion.MotionGenerator', Engine), patch('curobo_whole_geometry.CuroboWholeGeometry', side_effect=lambda value: value),
        patch('curobo_relative_geometry.install',side_effect=forbidden),
        patch.object(Geo,'closing_sweep',side_effect=forbidden),
        patch.object(Geo,'trajectory_state_reuse',side_effect=forbidden),
        patch('motiongen_plan.world_config', return_value=(object(), ['cup'], [{'fixture': 'world'}])),
        patch('cup_envelope.prepare',return_value={'sphere_count':264}),
        patch('cup_envelope.attach',return_value={'proxy_only':True}),
        patch('gpu_acceptance.SupportBounds',return_value=NS(check=lambda q:dict(passed=True))),
        patch('gpu_acceptance.fixed_world_check',return_value=dict(passed=True)),
        patch('gpu_acceptance.fixed_self_check',return_value=dict(passed=True)),
        patch('gpu_acceptance.jaw_sweep',return_value=dict(passed=True)),
        patch('gpu_acceptance.closing_geometry',side_effect=closing_fixture),
        patch('gpu_acceptance.task_check',side_effect=lambda e,p,stages,closing:dict(passed=not e.twist,
            checked_cross_samples=3,max_ee_orientation_error_rad=.01 if e.twist else 0.)), patch('motiongen_plan.code_identity', return_value={'fixture': 'code'})]
    from contextlib import ExitStack
    with ExitStack() as stack:
        for item in patches: stack.enter_context(item)
        diagnostics=[]
        base, plan, evidence = plan_loaded(planner, observation, data, source, urdf, speed_scale=.3,
            diagnostic=lambda name,value:diagnostics.append((name,deepcopy(value))))
        assert sum(name=='motiongen-receipts-latest' for name,value in diagnostics)==1
        individual=[value for name,value in diagnostics if name.startswith('motiongen-receipt-')]
        assert len(individual)==len(evidence['motiongen_receipts'])
        assert all('source_digest' in value and 'robot_config_sha256' in value for value in individual)
        assert plan['task_targets']['placement_goal']==targets['placement_goal']
        assert evidence['gpu_final_acceptance']['receiving_goal']['passed']
        # Reuse the prepared geometry only while its source/state/rules match.
        cached_data={**data,'parts':[{'name':'fixture'}]};planner.identity=data['planner_identity']
        cache={};before=len(model_calls)
        motiongen_plan.prepare_model(planner,observation,cached_data,urdf,state,cache)
        assert cache['engine'].seed_resets == 0, 'fresh warmup sampling state was reset'
        cache['engine'].grasp_contact_certificate={'stale':'previous request'}
        motiongen_plan.prepare_model(planner,observation,cached_data,urdf,state,cache)
        assert cache['engine'].seed_resets == 1, 'reused request retained the previous sampling sequence'
        assert cache['engine'].grasp_contact_certificate is None
        assert len(model_calls)==before+1
        changed={**cached_data,'parts':[{'name':'changed geometry'}]}
        motiongen_plan.prepare_model(planner,observation,changed,urdf,state,cache)
        assert len(model_calls)==before+2, 'changed geometry reused a stale sphere model'
        bridge = DualGpuPipeline(); bridge.ingest_source(source); bridge.accept_plan(plan)
        assert plan['planning_validation']['selected_motiongen_method_counts']['MotionGen.plan_single_js'] == 1
        assert plan['planning_validation']['selected_motiongen_method_counts']['MotionGen.plan_single'] == 10
        assert any(row.get('substage')=='retreat_insert' for row in plan['stages'])
        assert all(row['claw_cup_allowance_m']==0. for row in plan['motiongen_receipts'] if row['stage']=='home')
        assert not plan['planning_validation']['cpu_search_fallback']
        assert any(row.get('encoding_roundoff_rad', 0.) > 0 for row in plan['motiongen_receipts']), 'fixture must exercise real encode/decode rounding'
        for candidate in (base, plan):
            spec = ac_one_dual_arm_zero_home_action_spec()
            previous = list(reset_action)
            previous[6] = previous[13] = candidate['gripper_opening_mm']
            adapter = SimDistanceActionSpec(spec, candidate['gripper_opening_mm'])
            for action in candidate['native_actions']:
                adapter.decode_single_arm_transition(previous, action, active_arm='left')
                previous = action
            assert candidate['simulation_initial_command'] == data['simulation_initial_command']
            assert candidate['initial_state_validation']['measured_joint_state'] == state
            assert candidate['initial_state_validation']['max_arm_adjustment_rad'] > 0
            assert all(row['target'] == native_target(action, spec, candidate['gripper_opening_mm'], distance=True)
                       for row, action in zip(candidate['ticks'], candidate['native_actions']))
            starts = candidate['stages']
            for index, stage in enumerate(starts):
                end = starts[index+1]['start_tick'] if index+1 < len(starts) else len(candidate['ticks'])
                if stage['name'] == 'close_and_hold':
                    assert end-stage['start_tick'] == 18
                    assert all(abs(row['target']['left_gripper']-64.992) < 1e-9 for row in candidate['ticks'][stage['start_tick']:end])
                if stage['name'] == 'open':
                    gaps = [row['target']['left_gripper'] for row in candidate['ticks'][stage['start_tick']:end]]
                    assert abs(gaps[0]-64.992) < 1e-9 and abs(gaps[-1]-82.992) < 1e-9
                    assert all(0 <= b-a <= .20000001 for a, b in zip(gaps, gaps[1:]))
        assert evidence['gpu_final_acceptance']['task_semantics']['checked_cross_samples'] > 0
        assert evidence['gpu_final_acceptance']['task_semantics']['max_ee_orientation_error_rad'] == 0.
        alternatives=deepcopy(targets)
        alternatives['grasp_candidates']=[dict(grasp,candidate_id='rejected-contact'),
            dict(grasp,candidate_id='next-contact',world_from_tool=asdict(Pose((.2,.201,.3))))]
        def reject_first(engine,planner,state,cup,**options):
            if options['candidate_id']=='rejected-contact':raise ValueError('fixture native closing rejects first candidate')
            return closing_fixture(engine,planner,state,cup,**options)
        with patch('motiongen_targets.generate_targets',return_value=alternatives),\
                patch('gpu_acceptance.closing_geometry',side_effect=reject_first):
            _,next_plan,_=plan_loaded(planner,observation,data,source,urdf,speed_scale=.3)
        assert next_plan['task_targets']['grasp']['candidate_id']=='next-contact'
        assert next_plan['grasp_contact_certificate']['candidate_id']=='next-contact'
        assert next_plan['task_targets']['failed_grasp_candidates'][0]['candidate_id']=='rejected-contact'
        relation=Pose(**next_plan['grasp_contact_certificate']['tool_from_held_cup'])
        expected_tool=Pose(**next_plan['task_targets']['placement']['world_from_cup']).compose(relation.inverse())
        assert next_plan['task_targets']['placement']['world_from_tool']==asdict(expected_tool)
        # Exercise a physical input through the same numerical producer. All
        # source/owner/GPU objects here are CPU fixtures, never live services.
        from test_physical_motiongen import fake_inputs
        from test_initial_state import owner
        from real_observation import read_initial_sample,seal_initial_state
        from joint_space import convert_state
        from real_contract import validate_real_plan
        from frozen_executor import FrozenExecutor
        physical_source,_,_=fake_inputs()
        physical_source['scene']=deepcopy(source['scene'])
        physical_source['calibration_profile']['model_binding']['urdf_sha256']=data['urdf_sha256']
        model_start=target([state[n] for n in LEFT],[state[n] for n in RIGHT],grip['open_travel_mm'],grip['open_travel_mm'])
        reported=convert_state(model_start,physical_source['calibration_profile'],to_reported=True)
        before=read_initial_sample(lambda:owner(reported),wall_ns=lambda:1_003_000_000)
        after=read_initial_sample(lambda:owner(reported,stamp=9_000_000_000),wall_ns=lambda:9_003_000_000)
        physical_source['initial_state']=seal_initial_state(before,after,physical_source['calibration_profile'])
        binding=dict(calibration_accepted=True,evidence_source='physical_rgbd',robot_id=physical_source['robot_id'],
            camera_serials=physical_source['camera_serials'],source_digest=_digest(physical_source),
            calibration_profile_digest=_digest(physical_source['calibration_profile']),initial_state=physical_source['initial_state'])
        physical_base,physical_plan,_=plan_loaded(planner,observation,data,physical_source,urdf,
            speed_scale=.3,physical_binding=binding)
        for value in (physical_base,physical_plan):
            validate_real_plan(value,physical_source,binding)
            assert value['planning_validation']['schema_version']=='physical_curobo_motiongen_path_validation.v1'
            assert value['gripper_distance_control']['rubber_gap_mm']==62.
            assert value['initial_state_validation']['inactive_arm_source']=='physical_source.initial_state.model'
        from physical_motiongen import retime_physical
        for speed in (.1,.3,.55,1.):
            adjusted=retime_physical(physical_base,physical_source,binding,speed)
            validate_real_plan(adjusted,physical_source,binding)
            assert adjusted['planning_validation']['schema_version']=='physical_curobo_motiongen_path_validation.v1'
            assert adjusted['physical_gpu_conversion']['model_validation']['derivation']['collision_audit_rerun'] is False
            holds=adjusted['stages']
            index=next(i for i,s in enumerate(holds) if s['name']=='close_and_hold')
            assert holds[index+1]['start_tick']-holds[index]['start_tick']==18
        # A real model/Owner-domain projection must retain its proof after both
        # continuous stage merge and subsequent physical-plan retiming.
        noisy_source=deepcopy(physical_source)
        delta=noisy_source['calibration_profile']['joints']['zero_offsets_rad']
        limits=[]
        for side,offset in (('left',0),('right',6)):
            arm=getattr(spec,side).arm
            limits.extend([[lo-d,hi-d] for lo,hi,d in zip(arm.soft_lower_limits_rad,
                arm.soft_upper_limits_rad,delta[offset:offset+6])])
            limits.append([.008,83.])
        limits[3][0]=.002-delta[3]
        before=read_initial_sample(lambda:dict(owner(reported),joint_limits=limits),wall_ns=lambda:1_003_000_000)
        after=read_initial_sample(lambda:dict(owner(reported,stamp=9_000_000_000),joint_limits=limits),wall_ns=lambda:9_003_000_000)
        noisy_source['initial_state']=seal_initial_state(before,after,noisy_source['calibration_profile'])
        noisy_binding=dict(binding,source_digest=_digest(noisy_source),initial_state=noisy_source['initial_state'])
        with patch.object(Engine,'projection_noise',True):
            noisy_base,noisy_plan,_=plan_loaded(planner,observation,data,noisy_source,urdf,
                speed_scale=.3,physical_binding=noisy_binding)
        original=deepcopy(noisy_base)
        noisy_slow=retime_physical(noisy_base,noisy_source,noisy_binding,.55)
        assert noisy_base==original
        for value in (noisy_base,noisy_plan,noisy_slow):
            validate_real_plan(value,noisy_source,noisy_binding)
            proof=value['physical_gpu_conversion']['model_validation']
            merged=next(row for row in proof['continuous_motion']
                        if any(parent.get('position_normalization') for parent in row['parent_paths']))
            normalization=next(parent['position_normalization'] for parent in merged['parent_paths']
                               if parent.get('position_normalization'))
            assert normalization['corrections'][0]['changes'][0]['original_rad']==0.
            assert normalization['corrections'][0]['changes'][0]['bounded_rad']==.002
            assert normalization['normalized_joint_path_sha256']==normalization['geometric_validation']['path_sha256']
            assert 'original stage input rows' in normalization['row_index_space']
            assert next(stage for stage in value['stages'] if stage['substage']=='approach_pre')['motiongen_receipt_indices']
            direct=next(stage for stage in value['stages'] if stage['substage']=='retreat_insert')['position_normalization']
            assert direct['geometric_validation']['passed']
            assert direct['normalized_joint_path_sha256']==direct['geometric_validation']['path_sha256']
        direct=next(stage for stage in noisy_slow['stages'] if stage['substage']=='retreat_insert')['position_normalization']
        assert direct['inherited_from']['model_timeline_digest']==noisy_base['physical_gpu_conversion']['model_timeline_digest']
        assert direct['corrections']==next(stage for stage in noisy_base['stages']
            if stage['substage']=='retreat_insert')['position_normalization']['corrections']
        from physical_motiongen import validate_evidence,validate_model_evidence
        checked=validate_evidence(noisy_base,noisy_source)
        for mutation in ('drop_stage','drop_geometry','forge_geometry','drop_parent'):
            bad=deepcopy(checked)
            stage=next(stage for stage in bad['stages'] if stage['substage']=='retreat_insert')
            if mutation=='drop_stage':stage.pop('position_normalization')
            elif mutation=='drop_parent':
                parent=next(parent for row in bad['planning_validation']['continuous_motion']
                            for parent in row['parent_paths'] if parent.get('position_normalization'))
                parent.pop('position_normalization')
            else:
                normalization=stage['position_normalization']
                if mutation=='drop_geometry':normalization.pop('geometric_validation')
                else:normalization['geometric_validation']['path_sha256']='f'*64
                bad['motiongen_receipts'][stage['motiongen_receipt_index']]['position_normalization']=deepcopy(normalization)
                bad['planning_validation']['motiongen_receipts_sha256']=_digest(bad['motiongen_receipts'])
            try:validate_model_evidence(bad,noisy_source)
            except ValueError as error:assert 'normalization' in str(error),error
            else:raise AssertionError('missing/forged projection proof accepted: '+mutation)
        # Explicit fake-device goes through this same numerical producer and
        # retiming path; it never gains physical evidence labels or permission.
        fake_source=deepcopy(physical_source)
        fake_source.update(schema_version='fake_device_collection_source.v1',lane='fake-device',
            environment='fake',simulation_only=True,evidence_source='fake_device_rgbd')
        fake_source['calibration_profile']['source']['simulation_only']=True
        def fake_owner(stamp):
            value=dict(owner(reported,stamp=stamp),mode='simulation',environment='fake',evidence_source='fake',runtime_nonce='cpu-motiongen-owner')
            value['calibration_feedback'].update(clock_domain='simulation',source_clock_id='cpu-motiongen-clock',
                source_time_ns=stamp,physics_step=stamp*240//1_000_000_000)
            return value
        before=read_initial_sample(lambda:fake_owner(1_000_000_000),wall_ns=lambda:1_003_000_000,environment='fake')
        after=read_initial_sample(lambda:fake_owner(9_000_000_000),wall_ns=lambda:9_003_000_000,environment='fake')
        fake_source['initial_state']=seal_initial_state(before,after,fake_source['calibration_profile'])
        fake_source['clock']=fake_source['initial_state']['clock']
        fake_binding=dict(binding,environment='fake',simulation_only=True,evidence_source='fake_device_rgbd',
            source_digest=_digest(fake_source),calibration_profile_digest=_digest(fake_source['calibration_profile']),
            initial_state=fake_source['initial_state'],clock=fake_source['clock'])
        fake_base,fake_plan,_=plan_loaded(planner,observation,data,fake_source,urdf,
            speed_scale=.3,physical_binding=fake_binding,environment='fake')
        for value in (fake_base,fake_plan,retime_physical(fake_base,fake_source,fake_binding,.55,environment='fake')):
            validate_real_plan(value,fake_source,fake_binding,environment='fake')
            assert value['initial_state_validation']['inactive_arm_source']=='fake_device_source.initial_state.model'
            assert value['environment']=='fake' and value['simulation_only'] is True
            try:validate_real_plan(value,fake_source,fake_binding)
            except ValueError:pass
            else:raise AssertionError('fake numerical output accepted by physical validator')
        FrozenExecutor(physical_plan,physical_source,sink=lambda _:None,gate=lambda:False,stop=lambda _:None,
                       source_validator=lambda p,s:validate_real_plan(p,s,binding))
        historical_data = {key: value for key, value in data.items() if key != 'simulation_initial_command'}
        _, historical_plan, _ = plan_loaded(planner, observation, historical_data, source, urdf)
        assert historical_plan['offline_candidate'] and historical_plan['synthetic_fixture']
        try:
            bridge.accept_plan(historical_plan)
        except ValueError:
            pass
        else:
            raise AssertionError('historical measured-state fallback could be published')
        Engine.twist = True
        try:
            plan_loaded(planner, observation, data, source, urdf)
        except ValueError as error:
            assert 'GPU final acceptance failed' in str(error), error
        else:
            raise AssertionError('soft-cost intermediate transport twist passed final acceptance')
        Engine.twist = False
    from verify_motiongen_chain import seal_offline, synthetic_source
    provenance = dict(planning_input_sha256='a'*64, parent_source_sha256='b'*64, parent_source_schema='physical_collection_source.v1')
    synthetic = synthetic_source(source, provenance)
    assert synthetic['synthetic_fixture'] and synthetic['fixture_provenance'] == provenance
    assert 'initial_state' not in synthetic and synthetic['scene']['source']['synthetic_fixture']
    offline = seal_offline(plan, provenance)
    bridge = DualGpuPipeline(); bridge.ingest_source(source)
    try:
        bridge.accept_plan(offline)
    except ValueError:
        pass
    else:
        raise AssertionError('offline synthetic plan could be published')
    # Stage endpoints and fixed waits are independent of normalization/IK.
    rows = [target([value]*6, [0.]*6, 70+value, 20.) for value in (0., .1, .2)]
    spec, cache = ac_one_dual_arm_zero_home_action_spec(), {}
    encoded = [encode_stage(stage, spec, 82.992, cache) for stage in
               (dict(name='transport', rows=rows), dict(name='close_and_hold', rows=[rows[-1]]*18))]
    resampled, vertices = scaled_stages(encoded, .3, spec, 82.992)
    assert resampled[0]['rows'][0] == encoded[0]['rows'][0]
    assert resampled[0]['rows'][-1] == encoded[0]['rows'][-1]
    assert vertices[0][0] == 0 and vertices[0][-1] == len(encoded[0]['rows'])-1
    assert len(resampled[1]['rows']) == 18
    with TemporaryDirectory() as folder:
        folder = Path(folder)
        value = {'说明': '轨迹', 'targets': [[.1, .2], [None, False]]}
        motiongen_plan.write_new(folder/'compact.json', value)
        assert json.loads((folder/'compact.json').read_text()) == value
        assert '\n' not in (folder/'compact.json').read_text()
        try:
            motiongen_plan.write_new(folder/'compact.json', {})
        except FileExistsError:
            pass
        else:
            raise AssertionError('existing planning artifact overwritten')
        try:
            motiongen_plan.write_new(folder/'invalid.json', float('nan'))
        except ValueError:
            pass
        else:
            raise AssertionError('nonfinite planning artifact accepted')
        assert not (folder/'invalid.json').exists()
        physical = dict(source, schema_version='physical_collection_source.v1', lane='physical-real')
        (folder/'source.json').write_text(json.dumps(physical))
        args = NS(source=folder/'source.json', input=folder/'absent.json', output=folder/'out', native_code=native_code, assets=folder, speed_scale=.3)
        assert motiongen_plan.main(args) == 1
        assert not (args.output/'plan.json').exists() and not (args.output/'base-plan.json').exists()
    assert 'torch' not in sys.modules
    print('MotionGen complete fake orchestration, forbidden CPU fallback, geometry-derived 65/62, waits, orientation and failure-output checks passed')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    root = Path(__file__).resolve().parents[1]
    parser.add_argument('--native-code', type=Path, default=root.parent/'robot_sim-gripper-profile-20260921')
    parser.add_argument('--urdf', type=Path, default=root/'.agents/runs/20260923-214900-arx-a1cal-stage/stage/A1-cal-v5-v6/robot.urdf')
    args = parser.parse_args(); main(args.native_code, args.urdf)
