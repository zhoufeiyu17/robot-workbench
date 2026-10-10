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


def validate_release_control(plan, *, closed=None, opening=None, legacy=False):
    """Validate the frozen release before granting its one-sided tracking window."""
    release = plan.get('gripper_release')
    if release is None:
        if not legacy:
            return [False] * len(plan['ticks'])
        release = {'schema_version': 'commanded_gap_release.v2'}
    if not isinstance(release, dict) or release.get('schema_version') not in (
            'commanded_gap_release.v2', 'commanded_gap_release.v3'):
        raise ValueError('unsupported commanded gap release policy')
    v3 = release['schema_version'] == 'commanded_gap_release.v3'
    if (release.get('arm_motion', False) is not False
            or not v3 and (release.get('maximum_step_mm', .2) != .2
                           or 'maximum_gap_speed_mm_s' in release)
            or v3 and 'maximum_step_mm' in release):
        raise ValueError('release declaration conflicts with its versioned limits')
    mask = holding_ticks(plan, {'open'})
    indices = [i for i, active in enumerate(mask) if active]
    if (sum(stage['name'] == 'open' for stage in plan['stages']) != 1 or not indices
            or indices[0] == 0 or not holding_ticks(plan)[indices[0]-1]):
        raise ValueError('one contiguous preload release must follow holding')
    rows = [plan['ticks'][i]['target'] for i in indices]
    policy = plan.get('gripper_distance_control', {})
    if closed is None:
        closed = policy.get('closed_metal_gap_mm', rows[0]['left_gripper'])
        if policy.get('schema_version') == 'sim_gripper_distance_control.v1':
            closed -= policy['native_travel_offset_mm']
    if opening is None:
        opening = (rows[-1]['left_gripper'] if policy.get('schema_version') == 'gripper_distance_control.v2'
                   else plan.get('gripper_opening_mm', rows[-1]['left_gripper']))
    gaps = [row['left_gripper'] for row in rows]
    stationary = ('left_arm_joints', 'right_arm_joints', 'right_gripper')
    if (any(type(gap) not in (int, float) or not math.isfinite(gap) for gap in [closed, opening, *gaps])
            or not closed < opening or abs(gaps[0]-closed) > 1e-9 or abs(gaps[-1]-opening) > 1e-9
            or any(any(row[key] != rows[0][key] for key in stationary) for row in rows)
            or any(not 0 <= b-a <= (20/30+1e-8 if v3 else .20000001) for a,b in zip(gaps,gaps[1:]))):
        raise ValueError('release requires stationary arms and a continuous target-to-open ramp')
    if v3:
        if (set(release) - {'schema_version', 'duration_s', 'maximum_gap_speed_mm_s', 'initial_model_gap_mm', 'arm_motion'}
                or plan.get('native_action_encoding') != NATIVE_DISTANCE_ENCODING
                or len(rows) != 45 or release.get('duration_s') != 1.5
                or release.get('maximum_gap_speed_mm_s') != 20.):
            raise ValueError('v3 release requires 45 commands, 1.5 seconds and a 20 mm/s gap limit')
        offset = policy.get('native_travel_offset_mm', 0.) if policy.get('schema_version') == 'gripper_distance_control.v2' else 0.
        initial = release.get('initial_model_gap_mm', closed-offset)
        if type(initial) not in (int, float) or not math.isfinite(initial) or abs(initial+offset-closed) > 1e-9:
            raise ValueError('v3 release initial model gap differs from its closed target')
        expected = [closed+(opening-closed)*(1-math.cos(math.pi*i/44))/2 for i in range(45)]
        if any(abs(gap-value) > 1e-9 for gap,value in zip(gaps,expected)):
            raise ValueError('v3 release must retain its frozen cosine curve')
        hz, nominal = plan.get('control', {}).get('physics_hz'), plan.get('control', {}).get('substeps')
        counts, ticks, duration = plan.get('native_substeps'), plan['ticks'], plan.get('duration_s')
        if (type(hz) is not int or type(nominal) is not int or nominal <= 0 or hz != 30*nominal
                or not isinstance(counts, list) or len(counts) != len(ticks)
                or any(type(n) is not int or not 0 < n <= nominal for n in counts)
                or any(n != nominal for n in counts[:-1])
                or any(type(tick.get('time_s')) not in (int, float) or not math.isfinite(tick['time_s'])
                       or tick.get('tick') != i or abs(tick['time_s']-i/30) > 1e-9 for i,tick in enumerate(ticks))
                or type(duration) not in (int, float) or not math.isfinite(duration)
                or abs(duration-sum(counts)/hz) > 1e-9):
            raise ValueError('v3 release requires the actual 30 Hz native clock and duration')
        if any((b-a)/(ticks[j]['time_s']-ticks[i]['time_s']) > 20.+1e-8
               for i,j,a,b in zip(indices,indices[1:],gaps,gaps[1:])):
            raise ValueError('v3 commanded gap speed exceeds 20 mm/s')
        end = indices[-1]+1
        hold = [i for i, active in enumerate(holding_ticks(plan, {'release_hold'})) if active]
        if (hold != list(range(end, end+6)) or any(counts[i] != nominal for i in [*indices, *hold])
                or any(ticks[i]['target'] != rows[-1] for i in hold)
                or any(ticks[indices[0]-1]['target'][key] != rows[0][key] for key in stationary)):
            raise ValueError('v3 release must preserve stationary arms and its 0.2 second release hold')
    return mask


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
    validate_release_control(plan, closed=target, opening=maximum)
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
