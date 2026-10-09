"""Export measured simulated finger travel as explicit physical distance targets."""
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path

from integration import _digest
from joint_space import gripper_offset_mm, contract

NATIVE_DISTANCE_ENCODING = 'normalized_joint_gripper_travel_mm.v1'
HOLDING_STAGES = frozenset({'close_and_hold', 'lift', 'lift_hold', 'raise', 'transport', 'lower', 'support_settle'})
HOLD_STAGES = HOLDING_STAGES


def holding_ticks(plan, stage_names=HOLDING_STAGES):
    stages = plan.get('stages', [])
    if not stages or stages[0].get('start_tick') != 0:
        raise ValueError('distance grip requires explicit stages covering the timeline')
    names = []
    for i, stage in enumerate(stages):
        start = stage.get('start_tick')
        end = stages[i+1].get('start_tick') if i+1 < len(stages) else len(plan['ticks'])
        if type(start) is not int or type(end) is not int or start != len(names) or not start < end <= len(plan['ticks']):
            raise ValueError('distance grip stage boundaries must increase and cover every tick')
        names.extend([stage['name']] * (end-start))
    return [name in stage_names for name in names]


def validate_distance_control(plan, source):
    policy = plan.get('gripper_distance_control', {})
    if policy.get('schema_version') != 'gripper_distance_control.v2':
        if plan.get('native_action_encoding') == NATIVE_DISTANCE_ENCODING:
            raise ValueError('native distance encoding requires the v2 distance policy')
        return [False] * len(plan['ticks'])
    target=policy.get('closed_metal_gap_mm')
    if ((target,policy.get('rubber_gap_mm')) not in ((71.0,68.0),(65.0,62.0))
            or policy.get('controlled_gripper') != 'left_gripper'
            or policy.get('holding_stages') != sorted(HOLDING_STAGES)
            or plan.get('native_action_encoding') != NATIVE_DISTANCE_ENCODING):
        raise ValueError('unreviewed fixed-distance grip policy or native encoding')
    holding = holding_ticks(plan)
    if not any(holding):
        raise ValueError('explicit holding stage required')
    offset = gripper_offset_mm(source['calibration_profile'])
    maximum = source['calibration_profile']['gripper']['max_opening_mm']
    actions = plan.get('native_actions', [])
    if len(actions) != len(plan['ticks']):
        raise ValueError('native distance actions must cover all ticks')
    for tick, action, held in zip(plan['ticks'], actions, holding):
        if not isinstance(action, list) or len(action) != 14:
            raise ValueError('fourteen native action values required')
        if any(type(value) not in (int,float) or not math.isfinite(value) or not -1 <= value <= 1
               for slot,value in enumerate(action) if slot not in (6,13)):
            raise ValueError('native arm actions must retain finite normalized joint coordinates')
        for key, slot in (('left_gripper', 6), ('right_gripper', 13)):
            gap = tick['target'][key]
            if (type(action[slot]) not in (int,float) or not math.isfinite(action[slot])
                    or not 0 <= action[slot] <= maximum-offset
                    or abs(action[slot]+offset-gap) > 1e-9):
                raise ValueError('physical/native gripper distance targets differ')
        if held and tick['target']['left_gripper'] != target:
            raise ValueError('holding stage requires the declared metal-gap target')
    if plan.get('gripper_release',{}).get('schema_version')=='commanded_gap_release.v2':
        opening=holding_ticks(plan, {'open'})
        indices=[i for i,active in enumerate(opening) if active]
        if not indices or indices[0]==0 or not holding[indices[0]-1]:
            raise ValueError('preload release must follow holding')
        rows=[plan['ticks'][i]['target'] for i in indices]
        if (rows[0]['left_gripper']!=target or abs(rows[-1]['left_gripper']-maximum)>1e-9
                or any(any(row[k]!=rows[0][k] for k in ('left_arm_joints','right_arm_joints','right_gripper')) for row in rows)
                or any(not 0<=b['left_gripper']-a['left_gripper']<=.20000001 for a,b in zip(rows,rows[1:]))):
            raise ValueError('preload release requires stationary arms and a continuous target-to-open ramp')
    return holding


def read_trace(episode, plan):
    episode = Path(episode)
    raw = (episode/'report.json').read_bytes()
    if hashlib.sha256(raw).hexdigest() != plan['native_report_sha256']:
        raise ValueError('native report changed')
    report = json.loads(raw)
    digest = hashlib.sha256()
    gaps, stages, grasp = [], [], []
    with (episode/'steps.jsonl').open('rb') as stream:
        for i, line in enumerate(stream):
            digest.update(line)
            row = json.loads(line)
            if row['info']['execution_report']['applied_action'] != plan['native_actions'][i]:
                raise ValueError('native action differs from frozen plan')
            # This runtime records the LEFT two fingers, not two robot grippers.
            fingers = row['observation']['sample']['jaw_positions']
            if len(fingers) != 2 or any(not math.isfinite(q) or q < 0 for q in fingers):
                raise ValueError('finite nonnegative left finger positions required')
            gaps.append(sum(fingers)*1000)
            stages.append(row['stage'])
            grasp.append(row['observation']['facts']['grasp_confirmed'])
    if digest.hexdigest() != report['files']['steps.jsonl']['sha256'] or len(gaps) != len(plan['ticks']):
        raise ValueError('native step evidence changed or incomplete')
    return dict(native_report_sha256=plan['native_report_sha256'],
                steps_sha256=digest.hexdigest(), left_model_gap_mm=gaps,
                stages=stages, grasp_confirmed=grasp)


def distance_plan(plan, source, trace=None, *, close_metal_gap_mm=None):
    if 'execution_derivation' in plan or 'gripper_distance_control' in plan:
        raise ValueError('distance conversion requires original exported plan')
    if close_metal_gap_mm is None:
        if not isinstance(trace, dict) or trace['native_report_sha256'] != plan['native_report_sha256']:
            raise ValueError('trace belongs to another native report')
        gaps = trace['left_model_gap_mm']
        if len(gaps) != len(plan['ticks']) or not any(trace['grasp_confirmed']):
            raise ValueError('complete distance trace and native grasp evidence required')
    # Right jaw is not recorded by this episode sampler; only unchanged-open is supported.
    if any(a[13] != 1 for a in plan['native_actions']):
        raise ValueError('moving right gripper requires its own measured trace')
    result = deepcopy(plan)
    offset = gripper_offset_mm(source['calibration_profile'])
    maximum = source['calibration_profile']['gripper']['max_opening_mm']
    if close_metal_gap_mm is not None:
        if type(close_metal_gap_mm) not in (int,float) or close_metal_gap_mm not in (71.0,65.0):
            raise ValueError('only the reviewed 71/68 or 65/62 mm metal/rubber targets are supported')
        held = holding_ticks(plan)
        if trace is not None and len(trace.get('stages', [])) != len(plan['ticks']):
            raise ValueError('native stage trace must cover all ticks')
        if trace is not None and [name in HOLDING_STAGES for name in trace['stages']] != held:
            raise ValueError('native and exported holding stages differ')
        for tick, action, holding in zip(result['ticks'], result['native_actions'], held):
            if action[6] not in (-1, 1) or (action[6] == -1) != holding:
                raise ValueError('native closed intent must match explicit holding stages')
            if holding:
                tick['target']['left_gripper'] = close_metal_gap_mm
            for key, slot in (('left_gripper', 6), ('right_gripper', 13)):
                action[slot] = tick['target'][key] - offset
        result.update(run_id=plan['run_id']+'-metal'+str(int(close_metal_gap_mm)),timeline_digest=_digest(result['ticks']),
                      simulation={'validated':False,'outcome':'not_run'},
                      native_action_encoding=NATIVE_DISTANCE_ENCODING,
                      joint_space=contract(source['calibration_profile'],result['native_actions']),
                      gripper_distance_control=dict(schema_version='gripper_distance_control.v2',
                          units='metal_inner_gap_mm',closed_metal_gap_mm=close_metal_gap_mm,rubber_gap_mm=close_metal_gap_mm-3.0,
                          controlled_gripper='left_gripper',holding_stages=sorted(HOLDING_STAGES),
                          command_semantics='fixed_distance_target; larger measured left gap permitted only in holding stages',
                          contact_detection=False,force_control=False,
                          native_report_sha256=plan.get('native_report_sha256'),parent_plan_digest=_digest(plan),
                          simulation_evidence_scope='original_parent_rollout_only; distance_target_rollout_required',
                          hardware_validated=False))
        result.pop('planning_validation', None)
        validate_distance_control(result, source)
        return result
    for tick, gap in zip(result['ticks'], gaps):
        if type(gap) not in (int,float) or not math.isfinite(gap) or not 0 <= gap+offset <= maximum+.01:
            raise ValueError('jaw distance outside calibrated range')
        tick['target']['left_gripper'] = min(maximum, gap+offset)
    result.update(run_id=plan['run_id']+'-jaw-distance', timeline_digest=_digest(result['ticks']),
                  gripper_distance_control=dict(schema_version='gripper_distance_control.v1',
                      units='metal_inner_gap_mm', left_source='native_pre_step_measured_two_finger_sum',
                      right_source='original_unchanged_open_target; no measured right trace',
                      native_actions_semantics='original binary intent, not physical gap targets',
                      native_report_sha256=plan['native_report_sha256'], trace_sha256=_digest(trace),
                      steps_sha256=trace['steps_sha256'], parent_plan_digest=_digest(plan),
                      command_semantics='distance_trajectory_then_hold_observed_contact_gap',
                      tracking_semantics='existing calibrated distance tolerance; no zero-gap grasp requirement',
                      simulation_evidence_scope='original_parent_rollout_only', hardware_validated=False))
    return result
