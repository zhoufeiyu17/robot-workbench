"""Explicit model/reported boundary for physical plans; no implicit legacy conversion."""
from copy import deepcopy
from real_contract import is_device_source
import math

from integration import _digest
from contract import validate_physical14, validate_physical_feedback
from robot_workbench.joint_bounds import position_noise_rad, project_joint_positions


def model_joint_limits(spec, source=None):
    """Use the intersection of model limits and the observed Owner command domain."""
    limits={side+'_arm_joints':list(zip(getattr(spec,side).arm.soft_lower_limits_rad,
        getattr(spec,side).arm.soft_upper_limits_rad)) for side in ('left','right')}
    reported=(source or {}).get('initial_state',{}).get('reported_joint_limits')
    if reported is not None:
        delta=offsets(source['calibration_profile'])
        for side, start in (('left',0),('right',6)):
            key=side+'_arm_joints'
            limits[key]=[(max(lo,rlo+d),min(hi,rhi+d))
                for (lo,hi),(rlo,rhi),d in zip(limits[key],reported[key],delta[start:start+6],strict=True)]
            if any(lo>hi for lo,hi in limits[key]):raise ValueError('model and Owner joint limits do not intersect')
    return limits


def project_model_state(state, spec, *, environment, source=None):
    """Project position noise before encoding; keep raw input and jaw semantics."""
    result, changes = deepcopy(state), []
    limits=model_joint_limits(spec,source)
    for side in ('left', 'right'):
        key = side+'_arm_joints'
        result[key], corrections = project_joint_positions(state[key],limits[key], environment=environment)
        changes.extend(dict(row, side=side) for row in corrections)
    return result, changes


def state_near(actual, expected, *, joint_rad=.01):
    validate_physical14(actual,gripper_min_mm=-1.)
    validate_physical14(expected,gripper_min_mm=-1.)
    return all(abs(a-b) <= joint_rad for side in ('left', 'right')
               for a,b in zip(actual[side+'_arm_joints'], expected[side+'_arm_joints'])) and all(
                   abs(actual[side+'_gripper']-expected[side+'_gripper']) <= 2 for side in ('left', 'right'))


def validate_initial_state(initial, profile):
    if (not isinstance(initial, dict) or initial.get('schema_version') != 'physical_initial_state.v1'
            or initial.get('calibration_profile_digest') != _digest(profile)):
        raise ValueError('sealed physical initial state/profile required')
    measured, planned, model = (initial.get(key) for key in ('measured_reported', 'planned_reported', 'model'))
    validate_physical_feedback(measured,profile)
    validate_physical14(planned, gripper_max_mm=profile['gripper']['max_opening_mm'])
    validate_physical14(model)
    normalization = initial.get('jaw_normalization')
    if normalization not in ('clamp_to_profile_upper_limit_within_2mm', 'clamp_to_model_command_limits_within_2mm'):
        raise ValueError('explicit initial jaw normalization required')
    expected = deepcopy(measured)
    for side in ('left', 'right'):
        key = side+'_gripper'
        expected[key] = min(measured[key], profile['gripper']['max_opening_mm'])
        if normalization == 'clamp_to_model_command_limits_within_2mm':
            expected[key] = max(expected[key], max(0., gripper_offset_mm(profile)))
    if planned != expected or not state_near(measured, planned, joint_rad=.005):
        raise ValueError('initial target may only clamp measured jaw to model command limits within 2 mm; arms must be unchanged')
    if model != convert_state(planned, profile, to_reported=False):
        raise ValueError('initial model/reported conversion differs')
    evidence = initial.get('owner_samples')
    if not isinstance(evidence, list) or len(evidence) != 2:
        raise ValueError('initial state requires before/after observation owner samples')
    reported=initial.get('reported_joint_limits')
    if reported is not None:
        if any(sample.get('reported_joint_limits')!=reported for sample in evidence):
            raise ValueError('Owner command limits changed during observation')
        if set(reported)!={'left_arm_joints','right_arm_joints'}:raise ValueError('two arm limit sets required')
        for key in reported:
            if len(reported[key])!=6:raise ValueError('six Owner joint limits required')
            for pair in reported[key]:
                if len(pair)!=2 or any(type(v) not in (int,float) or not math.isfinite(v) for v in pair) or pair[0]>pair[1]:
                    raise ValueError('finite ordered Owner joint limits required')
    for sample in evidence:
        if (any(type(sample.get(k)) is not int or sample[k] < 0 for k in ('pid', 'command_epoch', 'sample_wall_ns', 'received_wall_ns'))
                or sample['pid'] == 0 or sample['sample_wall_ns'] == 0
                or type(sample.get('sample_time_s')) not in (int,float) or not math.isfinite(sample['sample_time_s'])
                or type(sample.get('sample_age_s')) not in (int,float) or not 0 <= sample['sample_age_s'] <= .1):
            raise ValueError('fresh owner PID/epoch/timestamp evidence required')
    if (any(evidence[0][k] != evidence[1][k] for k in ('pid', 'command_epoch'))
            or evidence[1]['sample_wall_ns'] <= evidence[0]['sample_wall_ns']
            or evidence[1]['sample_time_s'] <= evidence[0]['sample_time_s']):
        raise ValueError('owner identity/epoch changed or feedback did not progress during observation')
    if 'clock' in initial:
        from robot_workbench.sample_clock import clock_for_source, sample_stamp, assert_progress
        clock=clock_for_source(initial,'fake')
        assert_progress(sample_stamp(evidence[0],clock,wall_key='sample_wall_ns'),sample_stamp(evidence[1],clock,wall_key='sample_wall_ns'))
    validate_physical_feedback(initial.get('after_observation_reported'),profile)
    if not state_near(initial['after_observation_reported'], measured, joint_rad=.005):
        raise ValueError('robot moved during observation')
    return initial


def gripper_offset_mm(profile):
    mapping = profile.get('gripper', {}).get('mapping')
    binding = profile.get('model_binding', {})
    if mapping == 'operator-pad-basis-1p5mm-per-side-v2':
        if (binding.get('a1_sha256') != '113b975458ae273ea583e6d1b3376dc9f0df7e71bf1e3bca825985a0da650d76'
                or binding.get('geometry_id') != '44c6160a0a33a78710c228c5af2b1f9e364d8dc808f3330500830b79a2dc3b16'
                or not isinstance(binding.get('jaw_mapping_sha256'), str)
                or len(binding['jaw_mapping_sha256']) != 64):
            raise ValueError('A1 gripper mapping requires the known baseline model binding')
        return .008
    if mapping == 'total_opening_mm' and profile.get('source', {}).get('kind') != 'installed_physical_calibration' and not binding:
        return 0.0  # Explicit legacy/fake total-opening contract, not installed A1.
    raise ValueError('unknown physical gripper model/mapping conversion')


def offsets(profile):
    joints = profile.get('joints', {})
    values = joints.get('zero_offsets_rad')
    if (not isinstance(values, list) or len(values) != 12 or
            any(type(v) not in (int, float) or not math.isfinite(v) for v in values)):
        raise ValueError('twelve finite joint zero offsets required')
    if joints.get('signs') != [1] * 12:
        raise ValueError('physical calibration currently requires positive joint signs')
    return values


def convert_state(state, profile, *, to_reported):
    result = deepcopy(state)
    delta = offsets(profile)
    gap_offset = gripper_offset_mm(profile)
    for index, side in enumerate(('left', 'right')):
        key = side + '_arm_joints'
        values = state.get(key)
        if (not isinstance(values, list) or len(values) != 6 or
                any(type(v) not in (int, float) or not math.isfinite(v) for v in values)):
            raise ValueError('six finite joints required for ' + side)
        result[key] = [q - d if to_reported else q + d
                       for q, d in zip(values, delta[index * 6:index * 6 + 6])]
        key = side + '_gripper'
        gap = state.get(key)
        if type(gap) not in (int, float) or not math.isfinite(gap):
            raise ValueError('finite gripper gap required for '+side)
        result[key] = gap + gap_offset if to_reported else gap - gap_offset
    return result


def reported_command(state, source):
    """Convert a checked model command, removing only arithmetic boundary roundoff."""
    result=convert_state(state,source['calibration_profile'],to_reported=True)
    limits=source.get('initial_state',{}).get('reported_joint_limits')
    if limits is not None:
        for key in ('left_arm_joints','right_arm_joints'):
            for index,(lo,hi) in enumerate(limits[key]):
                original=result[key][index];bounded=min(hi,max(lo,original))
                if abs(bounded-original)>1e-12:
                    raise ValueError('model command was not normalized to observed Owner limits before encoding')
                result[key][index]=bounded
    return result


def contract(profile, native_actions):
    offsets(profile)
    return {'schema_version': 'physical_joint_space.v1',
            'ticks': 'reported', 'native_actions': 'model', 'measured_states': 'reported',
            'formula': 'q_model=q_reported+zero_offsets_rad',
            'gripper_formula': 'metal_gap_reported_mm=native_total_travel_mm+gripper_offset_mm',
            'gripper_offset_mm': gripper_offset_mm(profile),
            'calibration_profile_digest': _digest(profile),
            'native_actions_digest': _digest(native_actions)}


def validate(plan, source):
    physical = is_device_source(source)
    if physical:
        if not isinstance(plan.get('native_actions'), list) or len(plan['native_actions']) != len(plan['ticks']):
            raise ValueError('physical native actions must cover every tick')
        if plan.get('joint_space') != contract(source['calibration_profile'], plan['native_actions']):
            raise ValueError('physical plan requires current explicit model/reported joint-space contract')
        if 'initial_state' in source:
            initial = validate_initial_state(source['initial_state'], source['calibration_profile'])
            if plan.get('initial_state_digest') != _digest(initial):
                raise ValueError('plan initial state differs from sealed observation')
            first=deepcopy(plan['ticks'][0]['target']) if plan.get('ticks') else None
            resume=plan.get('execution_resume')
            if resume is not None:
                receipt=plan.get('planning_validation',{})
                if (not isinstance(resume,dict) or resume.get('schema_version')!='initial_open_resume.v1'
                        or resume.get('initial_state_digest')!=_digest(initial)
                        or resume.get('first_target_digest')!=_digest(first)
                        or receipt.get('execution_resume_digest')!=_digest(resume)
                        or type(resume.get('skipped_ticks')) is not int or resume['skipped_ticks']<=0
                        or not plan.get('stages') or plan['stages'][0].get('name')!='initial_open'
                        or plan['stages'][0].get('start_tick')!=0
                        or first['right_gripper']!=initial['planned_reported']['right_gripper']
                        or any(abs(a-b)>1e-8 for k in ('left_arm_joints','right_arm_joints') for a,b in zip(first[k],initial['planned_reported'][k]))
                        or not initial['planned_reported']['left_gripper']<=resume.get('measured_metal_gap_mm',-1)<=first['left_gripper']):
                    raise ValueError('invalid stationary initial-opening resume receipt')
                first['left_gripper']=initial['planned_reported']['left_gripper']
            allowance = position_noise_rad(source.get('environment', 'physical'))
            if (first is None or not state_near(first,initial['planned_reported'],joint_rad=allowance)
                    or not state_near(plan['ticks'][-1]['target'],initial['planned_reported'])):
                raise ValueError(f'native first target must match measured start within {allowance:g} rad/2 mm; return within 0.01 rad/2 mm')
        elif 'initial_state_digest' in plan:
            raise ValueError('initial state plan marker requires sealed source')
    elif 'joint_space' in plan:
        raise ValueError('physical joint-space contract cannot authorize simulated source')


def freeze_reported(plan, source):
    """Called once on freshly exported model ticks, never on an existing physical plan."""
    if not is_device_source(source):
        if 'joint_space' in plan:
            raise ValueError('unexpected physical joint-space contract')
        return
    if 'joint_space' in plan:
        raise ValueError('joint-space conversion has already been applied')
    marker = contract(source['calibration_profile'], plan['native_actions'])
    ticks = [{**tick, 'target': reported_command(tick['target'], source)}
             for tick in plan['ticks']]
    plan.update(ticks=ticks, timeline_digest=_digest(ticks), joint_space=marker)
    if source.get('environment') == 'fake':
        plan.update(environment='fake', simulation_only=True)


def replay_state(model_state, source):
    if is_device_source(source):
        return convert_state(model_state, source['calibration_profile'], to_reported=True)
    return model_state
