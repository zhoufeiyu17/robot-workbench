"""CPU-only reverse cup transfer, derived from a completed fake Owner execution.

The old RGB-D source stays historical. A separate current start/plant context
binds this replay after a service restart. No geometry query or actuator access.
"""
from copy import deepcopy
import math
import re

from integration import CUROBO_PLANNER_ID, SIM_POLICY_ID, _digest
from joint_space import contract, convert_state, gripper_offset_mm, validate_initial_state

REPLAY_PLANNER_ID = 'cup-return-exact-reverse/v1'
OFFSET_PLANNER_ID = 'cup-return-local-offset/v1'
REPLAY_SOURCE_SCHEMA = 'fake_device_cup_return_source.v1'
REVERSE_STAGE = {
    'home_hold': 'initial_hold', 'home_gripper_restore': 'initial_open',
    'home': 'approach_pre', 'retreat': 'approach', 'release_hold': 'preclose_settle',
    'open': 'close_ramp', 'support_settle': 'close_and_hold', 'lower': 'lift',
    'transport': 'transport', 'lift': 'lower', 'close_and_hold': 'support_settle',
    'close_ramp': 'open', 'preclose_settle': 'release_hold', 'approach': 'retreat',
    'approach_pre': 'home', 'initial_open': 'home_gripper_restore', 'initial_hold': 'home_hold',
}


def is_replay(value):
    """Routing only; callers must still validate the full evidence and live review."""
    return isinstance(value, dict) and (value.get('planner_id') in (REPLAY_PLANNER_ID,OFFSET_PLANNER_ID)
        or value.get('schema_version') == REPLAY_SOURCE_SCHEMA
        or value.get('planner_id') == 'physical-cup-return/v1'
        or value.get('schema_version') == 'physical_cup_maintenance_source.v1')


def build_replay_source(parent_source, current_initial_state, replay_context, *, run_id):
    """Current feedback plus explicitly historical observations, never a new capture."""
    if parent_source.get('schema_version') != 'fake_device_collection_source.v1':
        raise ValueError('return replay v1 requires an original fake-device source')
    validate_initial_state(current_initial_state, parent_source['calibration_profile'])
    result = deepcopy(parent_source)
    result.update(schema_version=REPLAY_SOURCE_SCHEMA, run_id=run_id, sequence=0,
        clock=deepcopy(current_initial_state['clock']), initial_state=deepcopy(current_initial_state),
        observation_scope='historical_parent_only', parent_source_digest=_digest(parent_source),
        replay_context=deepcopy(replay_context))
    validate_source_context(result)
    return result


def validate_source_context(source):
    context = source.get('replay_context', {})
    initial = source.get('initial_state', {})
    if (source.get('schema_version') != REPLAY_SOURCE_SCHEMA
            or source.get('environment') != 'fake' or source.get('simulation_only') is not True
            or source.get('observation_scope') != 'historical_parent_only'
            or not isinstance(source.get('parent_source_digest'), str)
            or len(source['parent_source_digest']) != 64
            or context.get('schema_version') != 'cup_return_context.v1'
            or context.get('clock') != source.get('clock')
            or initial.get('clock') != source.get('clock')
            or not context.get('parent_execution_instance_id')
            or not context.get('physical_configuration_sha256')
            or context.get('pose_basis') != 'parent_plant_fixture'
            or not isinstance(context.get('parent_fixture_sha256'), str)
            or re.fullmatch('[0-9a-f]{64}', context['parent_fixture_sha256']) is None):
        raise ValueError('explicit fake replay/current-start context required')
    for name in ('current_cup_pose', 'current_machine_pose', 'parent_fixture_machine_pose'):
        _pose_values(context.get(name))
    return context


def _pose_values(pose):
    if not isinstance(pose, dict):
        raise ValueError('finite rigid pose required')
    p, q = pose.get('position'), pose.get('orientation_wxyz')
    if (not isinstance(p, list) or len(p) != 3 or not isinstance(q, list) or len(q) != 4
            or any(type(v) not in (int, float) or not math.isfinite(v) for v in [*p, *q])
            or abs(sum(v*v for v in q)-1.) > 1e-5):
        raise ValueError('finite rigid pose required')
    return p, q


def _same_pose(actual, reference, label):
    p, q = _pose_values(actual)
    r, s = _pose_values(reference)
    # Existing observation drift bound; no new collision tolerance is implied.
    distance = math.dist(p, r)
    angle = 2*math.acos(min(1., abs(sum(a*b for a,b in zip(q, s)))))
    if distance >= .006 or angle > .02:
        raise ValueError(label+' changed from the completed parent scene')


def _validate_parent(parent, parent_source, receipt, source):
    from physical_motiongen import validate_evidence as validate_parent_evidence
    if (parent.get('planner_id') != CUROBO_PLANNER_ID
            or parent_source.get('schema_version') != 'fake_device_collection_source.v1'
            or parent.get('environment') != 'fake' or parent.get('simulation_only') is not True
            or 'replay_derivation' in parent):
        raise ValueError('one original fake physical-target plan required')
    validate_parent_evidence(parent, parent_source)
    context = validate_source_context(source)
    if (source['parent_source_digest'] != _digest(parent_source)
            or any(source.get(key) != parent_source.get(key)
                   for key in ('scene', 'calibration_profile', 'robot_id', 'camera_serials'))
            or receipt.get('schema_version') != 'owner_physx_native_task.v1'
            or receipt.get('environment') != 'fake' or receipt.get('simulation_only') is not True
            or receipt.get('motion_outcome') != 'completed' or receipt.get('task_outcome') != 'success'
            or receipt.get('task_qualified') is not True
            or receipt.get('source_digest') != _digest(parent_source)
            or receipt.get('parent_timeline_digest') != parent['timeline_digest']
            or receipt.get('source_clock_id') != parent_source['clock']['source_clock_id']
            or context['parent_execution_instance_id'] != receipt.get('execution_instance_id')
            or context['physical_configuration_sha256'] != receipt.get('physical_configuration_sha256')):
        raise ValueError('completed same-source parent execution and unchanged plant configuration required')
    _same_pose(context['current_cup_pose'], receipt['last_observation']['sample']['world_from_cup'], 'cup')
    # Both sides are actual plant poses. The parent's RGB-D estimate remains
    # historical planning evidence and is not a measurement of fixture movement.
    # The trusted publisher verifies the fixture file SHA before extracting it;
    # its source binding seals this pose and SHA together for the Owner.
    _same_pose(context['current_machine_pose'], context['parent_fixture_machine_pose'], 'machine')
    speed = parent.get('normal_recipe', {}).get('requested_speed_scale', 1.)
    if type(speed) not in (int, float) or not math.isfinite(speed) or not .3 <= speed <= 1.7:
        raise ValueError('return replay parent speed must remain within the reviewed 30%-170% range')
    control = parent['control']
    if (control.get('physics_hz') != 30*control.get('substeps', 0)
            or parent.get('native_substeps') != [control['substeps']]*len(parent['ticks'])):
        raise ValueError('return replay requires an exact complete 30 Hz parent')


def _derive(parent, parent_source, receipt, source):
    """Rebuild exact arm order; only stationary release timing may be expanded."""
    ticks, actions, stages, indices = [], [], [], []
    starts = parent['stages']
    if (starts[0]['name'] != 'initial_hold' or starts[-1]['name'] != 'home_hold'
            or any(stage['name'] not in REVERSE_STAGE for stage in starts)):
        raise ValueError('unsupported parent cup-transfer stage sequence')
    for stage_index in reversed(range(len(starts))):
        stage = starts[stage_index]
        begin = stage['start_tick']
        end = starts[stage_index+1]['start_tick'] if stage_index+1 < len(starts) else len(parent['ticks'])
        original_indices = list(reversed(range(begin, end)))
        rows = [deepcopy(parent['ticks'][i]['target']) for i in original_indices]
        native = [deepcopy(parent['native_actions'][i]) for i in original_indices]
        name = REVERSE_STAGE[stage['name']]
        if name in ('open', 'release_hold'):
            static = ('left_arm_joints', 'right_arm_joints', 'right_gripper')
            if not rows or any(any(row[k] != rows[0][k] for k in static) for row in rows):
                raise ValueError('only stationary jaw stages may be resampled')
            count = 45 if name == 'open' else 6
            if len(rows) > count:
                raise ValueError('return replay may not shorten stationary release/hold')
            closed = parent['gripper_distance_control']['closed_metal_gap_mm']
            opening = source['calibration_profile']['gripper']['max_opening_mm']
            offset = gripper_offset_mm(source['calibration_profile'])
            rows = [deepcopy(rows[0]) for _ in range(count)]
            native = [deepcopy(native[0]) for _ in range(count)]
            original_indices = [original_indices[0]]*count
            for i, (row, action) in enumerate(zip(rows, native)):
                gap = closed+(opening-closed)*(1-math.cos(math.pi*i/44))/2 if name == 'open' else opening
                row['left_gripper'] = gap
                action[6] = gap-offset
        start = len(ticks)
        stages.append(dict(name=name, substage='reverse_'+(stage.get('substage') or stage['name']),
                           start_tick=start, start_s=start/30, parent_stage_index=stage_index))
        for row in rows:
            ticks.append(dict(tick=len(ticks), time_s=len(ticks)/30, target=row))
        actions.extend(native)
        indices.extend(original_indices)
    fields = ('schema_version', 'lane', 'policy_id', 'control', 'native_identity',
              'native_action_encoding', 'gripper_opening_mm', 'gripper_distance_control',
              'gripper_release', 'gripper_closing', 'cup_contact_preset')
    result = {key: deepcopy(parent[key]) for key in fields if key in parent}
    result.update(run_id=source['run_id']+'-plan', source_run_id=source['run_id'], source_sequence=source['sequence'],
        scene_digest=source['scene']['scene_digest'], calibration_profile_digest=_digest(source['calibration_profile']),
        planner_id=REPLAY_PLANNER_ID, environment='fake', simulation_only=True,
        simulation=dict(validated=False, outcome='not_run'), motion_profile='completed-parent-reverse-30hz',
        ticks=ticks, native_actions=actions, stages=stages,
        native_substeps=[parent['control']['substeps']]*len(ticks), duration_s=len(ticks)/30,
        timeline_digest=_digest(ticks), initial_state_digest=_digest(source['initial_state']),
        joint_space=contract(source['calibration_profile'], actions))
    proof = dict(schema_version='cup_return_replay_derivation.v1', method='reverse_existing_arm_targets',
        parent_plan=deepcopy(parent), parent_source=deepcopy(parent_source), parent_native_receipt=deepcopy(receipt),
        parent_plan_digest=_digest(parent), parent_source_digest=_digest(parent_source),
        parent_native_receipt_digest=_digest(receipt), parent_tick_indices=indices,
        collision_check=dict(status='not_run', reason='user_requested_reverse_replay'),
        stationary_release=dict(commands=45, duration_s=1.5, maximum_gap_speed_mm_s=20., hold_commands=6),
        return_goal=deepcopy(parent_source['scene']['environment_from_cup']),
        goal_basis='historical_parent_observation; not a new detection',
        execution_authorization='separate current Owner review and confirmation required')
    closing = parent['physical_gpu_conversion']['model_validation']['gpu_final_acceptance']['closing']
    proof['historical_release_contact_model_gap_mm'] = closing.get('end_m',
        (parent['gripper_distance_control']['closed_metal_gap_mm']-parent['gripper_distance_control']['native_travel_offset_mm'])/2000)*2000
    result['replay_derivation'] = proof
    result['planning_validation'] = dict(schema_version='cup_return_replay_validation.v1',
        source_digest=_digest(source), timeline_digest=result['timeline_digest'], derivation_digest=_digest(proof),
        scope='exact parent arm reversal and current CPU motion limits; no new collision or simulation qualification',
        hardware_validated=False)
    from control_motion import validate_control
    result['control_validation'] = validate_control(result)
    return result


def _requested_plan(parent,parent_source,receipt,source,*,return_goal=None,offset_xy_mm=None,kinematics=None,evidence=None):
    from cup_return_offset import offset_request,apply_offset,NativeReturnKinematics
    result=_derive(parent,parent_source,receipt,source)
    request=offset_request(source,parent_source,return_goal,offset_xy_mm)
    if 'return_origin' in source['replay_context'] or 'offset_xy_mm' in source['replay_context']:
        result['replay_derivation'].update(request,goal_basis='fixed_scene_origin_plus_user_xy_offset')
    if not request['exact_parent_reversal']:
        if kinematics is None and evidence is not None:
            identity=evidence.get('kinematics',{})
            kinematics=NativeReturnKinematics(identity.get('urdf_path'),source,identity.get('world_from_robot'))
        result,offset_proof=apply_offset(result,source,request,kinematics,evidence=evidence)
        result.update(planner_id=OFFSET_PLANNER_ID,motion_profile='parent-reverse-with-local-table-offset-30hz')
        proof=result['replay_derivation']
        proof.pop('parent_tick_indices')
        proof.update(method='parent_reverse_with_local_cartesian_offset',offset_derivation=offset_proof,
            collision_check=deepcopy(offset_proof['collision_check']),return_goal=request['return_goal'])
        result['planning_validation']['scope']='parent reversal with local calibrated CPU IK/FK and dynamics; no collision or simulation qualification'
    proof=result['replay_derivation']
    result['planning_validation'].update(timeline_digest=result['timeline_digest'],derivation_digest=_digest(proof))
    return result


def build_replay(parent, parent_source, parent_native_receipt, source, *, return_goal=None, offset_xy_mm=None, kinematics=None):
    _validate_parent(parent, parent_source, parent_native_receipt, source)
    result = _requested_plan(parent,parent_source,parent_native_receipt,source,
        return_goal=return_goal,offset_xy_mm=offset_xy_mm,kinematics=kinematics)
    _validate_controls(result, source)
    return result


build_return_plan = build_replay


def _validate_controls(plan, source):
    from control_motion import validate_receipt
    from gripper_distance_plan import validate_distance_control
    from integration import _validate_timeline
    from joint_space import validate
    _validate_timeline(plan['ticks'], source['calibration_profile']['gripper']['max_opening_mm'])
    validate(plan, source)
    validate_receipt(plan)
    validate_distance_control(plan, source)


def validate_evidence(plan, source):
    proof = plan.get('replay_derivation', {})
    if (plan.get('planner_id') not in (REPLAY_PLANNER_ID,OFFSET_PLANNER_ID) or plan.get('policy_id') != SIM_POLICY_ID
            or proof.get('schema_version') != 'cup_return_replay_derivation.v1'):
        raise ValueError('explicit cup return replay derivation required')
    parent, original, receipt = (proof.get(key, {}) for key in ('parent_plan', 'parent_source', 'parent_native_receipt'))
    _validate_parent(parent, original, receipt, source)
    expected = _requested_plan(parent,original,receipt,source,evidence=proof.get('offset_derivation'))
    if plan != expected:
        raise ValueError('return replay differs from its exact parent-derived targets or provenance')
    _validate_controls(plan, source)
    return plan


def replay_model_plan(plan, source):
    """For read-only task observation; this carries no GPU geometry qualification."""
    validate_evidence(plan, source)
    result = deepcopy(plan)
    for row in result['ticks']:
        row['target'] = convert_state(row['target'], source['calibration_profile'], to_reported=False)
    result['timeline_digest'] = _digest(result['ticks'])
    result.pop('joint_space')
    result['gripper_distance_control']['schema_version'] = 'sim_gripper_distance_control.v1'
    return result
