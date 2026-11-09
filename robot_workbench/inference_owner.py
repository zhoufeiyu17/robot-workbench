"""Policy runner for the existing sole Owner. No SDK calls from model/data threads."""
from copy import deepcopy
from dataclasses import asdict, replace
import hashlib
import json
import math
import os
import queue
import secrets
import threading
import time
import weakref

from .inference import InferenceLoop, StableTraySupport, audit_dynamics, vector14
from .sample_clock import POLICY_DURATION_TOLERANCE_S
from .wam import NATIVE_SCHEMA
from .execution_settings import DEFAULT_JOINT_TOLERANCE_RAD


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def validate_mode_request(request, *, identity, current_mode, clock_id, now_ns, recorder_ids, recording_required=True):
    """Candidate normal-mode admission, before any SDK output (not fault Stop).

    The existing Owner request ledger must handle retries before this check.
    No IO, waiting, epoch mutation or deferred MODEL request is performed here.
    """
    if (not isinstance(request,dict)or not isinstance(identity,dict)
            or set(identity)!={'session_id','runtime_nonce','epoch'}
            or any(not isinstance(identity[key],str)or not identity[key]for key in('session_id','runtime_nonce'))
            or any(request.get(key)!=value for key,value in identity.items())
            or type(request.get('epoch'))is not int or request['epoch']<0
            or not isinstance(request.get('request_id'),str)or not request['request_id']
            or current_mode not in ('MODEL','HOLD','TEACH','BRAKE')
            or request.get('mode')not in ('MODEL','HOLD','TEACH','HOME')):
        raise ValueError('normal mode request requires current owner/session/epoch identity')
    mode=request['mode']
    if current_mode=='BRAKE':raise ValueError('mode transition pending; MODEL is never queued')
    if current_mode=='TEACH'and mode=='MODEL':
        raise ValueError('TEACH requires an explicit HOLD completion followed by a new MODEL request')
    if mode=='HOME'and current_mode!='HOLD':raise ValueError('native Home requires a completed HOLD')
    if mode==current_mode and request.get('target')not in('END','DISCARD'):return dict(no_op=True,mode=mode)
    if mode=='MODEL':return dict(no_op=False,requires_new_observation_and_epoch=True,mode=mode)
    if type(recording_required)is not bool:raise ValueError('explicit recording consumer policy required')
    if not recording_required:
        if recorder_ids:raise ValueError('non-recording mode cannot hide active recorders')
        return dict(no_op=False,mode=mode,recording_policy='explicit_no_recording_consumers')
    gate=request.get('recording_gate')
    if (not isinstance(gate,dict)or gate.get('closed')is not True
            or any(gate.get(key)!=request[key]for key in (*identity,'request_id'))
            or type(gate.get('epoch'))is not int
            or not isinstance(clock_id,str)or not clock_id or gate.get('clock_id')!=clock_id
            or type(now_ns)is not int or type(gate.get('cutoff_ns'))is not int
            or not 0<=gate['cutoff_ns']<=now_ns
            or type(gate.get('gate_generation'))is not int or gate['gate_generation']<0
            or not isinstance(gate.get('recorders'),list)
            or not isinstance(recorder_ids,(list,tuple))or any(not isinstance(x,str)or not x for x in recorder_ids)
            or len(set(recorder_ids))!=len(recorder_ids)):
        raise ValueError('normal transition requires closed Raw gate on the same source clock')
    rows=gate['recorders']
    if (len(rows)!=len(recorder_ids)or any(not isinstance(row,dict)for row in rows)
            or any(not isinstance(row.get('record_id'),str)for row in rows)
            or {row['record_id']for row in rows}!=set(recorder_ids)
            or any(row.get('closed')is not True or row.get('clock_id')!=clock_id
                   or type(row.get('cutoff_ns'))is not int or row['cutoff_ns']!=gate['cutoff_ns']
                   or type(row.get('generation'))is not int or row['generation']!=gate['gate_generation']for row in rows)):
        raise ValueError('all active recorders must acknowledge the exact cutoff/generation')
    return dict(no_op=False,mode=mode,raw_cutoff_ns=gate['cutoff_ns'],gate_generation=gate['gate_generation'])


def map_recording_cutoff(cutoff_ns, recorder_pair, owner_pair, *, max_uncertainty_ns):
    """Map same-host ROS wall cutoff to a conservative monotonic interval.

    Both pairs are monotonic-before / realtime / monotonic-after reads. The
    budget must be explicitly frozen by the integrator, not learned by accepting
    a slow call. This checks supplied evidence; it does not acquire clocks or
    certify a remote camera's timestamp source.
    """
    if type(max_uncertainty_ns)is not int or max_uncertainty_ns<0 or type(cutoff_ns)is not int:
        raise ValueError('explicit clock uncertainty budget and integer cutoff required')
    fields={'host_id','boot_id','mapping_id','wall_ns','monotonic_before_ns','monotonic_after_ns'}
    for pair in(recorder_pair,owner_pair):
        if (not isinstance(pair,dict)or set(pair)!=fields
                or any(not isinstance(pair[k],str)or not pair[k]for k in('host_id','boot_id','mapping_id'))
                or any(type(pair[k])is not int or pair[k]<0 for k in('wall_ns','monotonic_before_ns','monotonic_after_ns'))
                or not 0<=pair['monotonic_after_ns']-pair['monotonic_before_ns']<=max_uncertainty_ns):
            raise ValueError('bounded identified clock sandwich required')
    if (any(recorder_pair[k]!=owner_pair[k]for k in('host_id','boot_id','mapping_id'))
            or cutoff_ns!=recorder_pair['wall_ns']
            or recorder_pair['monotonic_after_ns']>owner_pair['monotonic_before_ns']):
        raise ValueError('same-host/boot/mapping and ordered close-boundary clocks required')
    offsets=[(p['wall_ns']-p['monotonic_after_ns'],p['wall_ns']-p['monotonic_before_ns'])
             for p in(recorder_pair,owner_pair)]
    # An interval disagreement beyond the frozen budget indicates a step/drift
    # or a wrong clock identity. Never quietly rebase an episode.
    if max(lo for lo,hi in offsets)-min(hi for lo,hi in offsets)>max_uncertainty_ns:
        raise ValueError('wall/monotonic mapping changed beyond the frozen uncertainty budget')
    return dict(clock_id='host_monotonic_ns',host_id=owner_pair['host_id'],boot_id=owner_pair['boot_id'],
        mapping_id=owner_pair['mapping_id'],wall_cutoff_ns=cutoff_ns,
        earliest_ns=recorder_pair['monotonic_before_ns'],latest_ns=recorder_pair['monotonic_after_ns'],
        uncertainty_ns=recorder_pair['monotonic_after_ns']-recorder_pair['monotonic_before_ns'],
        scope='clock_read_bracket_not_motor_or_recorder_ack')


def episode_duration(value=120):
    if type(value) not in (int, float) or not math.isfinite(value) or value<=0:
        raise ValueError('推理时长须为有限的正数秒数，包含观测、推理、等待和运动')
    return float(value)


def execution_steps(value=15,*,horizon=60):
    if type(horizon)is not int or horizon not in(32,60):raise ValueError('bound model horizon required')
    if type(value) is not int or not 1 <= value <= horizon:
        raise ValueError('推理执行长度须为 1～'+str(horizon)+' 的整数')
    return value


def model_capabilities(value):
    binding=value.get('action_chunk_binding')
    if binding is not None:
        from .inference import action_shape
        contract=binding.get('action_contract')or{}
        horizon=action_shape(contract)[0];history=contract.get('history_steps',0 if horizon==32 else 60)
        if type(history)is not int or history not in(0,60):raise ValueError('bound model history capability required')
        return horizon,history
    from .inference_model import NOHISTORY_MODELS
    return 60,0 if value.get('model_id')in NOHISTORY_MODELS else 60


def configuration(value):
    """Freeze the supported episode duration and policy configuration for review."""
    if not isinstance(value, dict):
        raise ValueError('inference configuration object required')
    result = deepcopy(value)
    if result.get('data_plane')is not None and result.get('action_chunk_binding')is None and result.get('trajectory_kind')!='prepared_native_reference':raise ValueError('online model execution requires its bound action contract')
    horizon,_=model_capabilities(result)
    age_limit=2.5 if horizon==32 else .5
    if result.get('recording_backend') not in (None,'capture_session_v1'):
        raise ValueError('known recording backend required')
    result['execute_steps']=execution_steps(result.get('execute_steps',15),horizon=horizon)
    for key in ('state_max_age_s','max_camera_span_s'):
        limit=result.setdefault(key,.1)
        if type(limit) not in (int,float) or not math.isfinite(limit) or not 0<limit<=.1:
            raise ValueError(key+' must fit the qualified0.1s input profile')
    for key, default in (('prepare_drift_joint_rad', DEFAULT_JOINT_TOLERANCE_RAD), ('prepare_drift_jaw_mm', 2.)):
        limit = result.setdefault(key, default)
        if type(limit) not in (int, float) or not math.isfinite(limit) or limit <= 0:
            raise ValueError(key+' must be finite and positive')
    if result.get('reference_planning')is not None:
        from .motion_reference import infer_reference_policy
        from .inference_model import model_family,NOHISTORY_MODELS
        result['reference_planning']=infer_reference_policy(result['reference_planning'])
        if (model_family(result.get('model_id'))not in('mixed366','wam') or result.get('trajectory_kind')=='prepared_native_reference'):
            raise ValueError('new planning policy is scoped to current online infer; frozen research/normal-brake budgets are unchanged')
        if result.get('bounded_live_prediction'):
            raise ValueError('0.5s diagnostic prefix is not a complete-segment endpoint; decide execution scope before enabling this curve')
        if result.get('normal_modes') is not None:
            from .execution_settings import NORMAL_BRAKE_DURATION_S,NORMAL_GAIN_TRANSITION_S
            expected=dict(brake_duration_s=NORMAL_BRAKE_DURATION_S,velocity_limit=result['reference_planning']['velocity_limit_rad_s'],
                acceleration_limit=result['reference_planning']['acceleration_limit_rad_s2'],gain_transition_s=NORMAL_GAIN_TRANSITION_S,teach_kd_scale=.1)
            if (not isinstance(result['normal_modes'],dict)or any(result['normal_modes'].get(k)!=v for k,v in expected.items())
                    or result.get('finite_chunk_count')is not None or result.get('normal_initial_mode')!='HOLD'
                    or not math.isclose(result['reference_planning']['max_execution_duration_s'],result['execute_steps']/30.*4,abs_tol=1e-9)
                    or result.get('task')not in('coffee','lid_handoff')
                    or result.get('target_execution')!='shared_reference_windows'
                    or type(result.get('maximum_observation_age_s')) not in (int,float) or not 0<result['maximum_observation_age_s']<=age_limit
                    or type(result.get('reference_supply_timeout_s')) not in (int,float) or not 0<result['reference_supply_timeout_s']<=.5):
                raise ValueError('reviewed normal policy requires0.2s brake/0.05s gains/1rad/s/40rad/s2, fresh complete1-60-point source and initial HOLD')
        elif result.get('finite_chunk_count')not in(3,4):
            raise ValueError('reviewed infer planning policy currently requires the explicit three-or-four-segment contract')
    if result.get('finite_chunk_count')is not None:
        from .inference_model import model_family,NOHISTORY_MODELS
        if (type(result['finite_chunk_count'])is not int or result['finite_chunk_count']not in(3,4)
                or result.get('reference_planning')is None or model_family(result.get('model_id'))not in('mixed366','wam')
                or result.get('task')not in('coffee','lid_handoff')or result.get('duration_s')!=.5
                or result.get('execute_steps')!=15 or result.get('reference_planning',{}).get('max_execution_duration_s')!=2.
                or result.get('target_execution')!='shared_reference_windows'
                or result.get('maximum_observation_age_s')!=age_limit or result.get('reference_supply_timeout_s')!=.5
                or result.get('bounded_live_prediction')or result.get('normal_modes')is not None
                or result.get('recording_enabled')is not False
                or result.get('trajectory_kind')=='prepared_native_reference'):
            raise ValueError('finite sync infer requires three or four complete fresh15-step source0.5s segments and explicit planning policy')
    if result.get('normal_finish_policy')is not None:
        if result.get('normal_finish_kind')not in('dagger_reset_v1','maintenance_home_v1')or not result.get('normal_modes'):
            raise ValueError('finish requires an explicit supported purpose and normal modes')
    normal=result.get('normal_modes')
    context_policy=result.get('recording_context_policy','fixed')
    if context_policy not in('fixed','per_episode_review'):raise ValueError('explicit recording context policy required')
    if context_policy=='per_episode_review'and(not isinstance(normal,dict)or normal.get('recording_required')is not True or result.get('normal_initial_mode')!='HOLD'):
        raise ValueError('per-episode recorder binding requires recording and initial HOLD')
    if result.get('normal_initial_mode','MODEL')not in('MODEL','HOLD'):
        raise ValueError('normal initial mode must be explicitly MODEL or HOLD')
    if result.get('normal_initial_mode')=='HOLD'and(not normal or result.get('trajectory_kind')=='prepared_native_reference'):
        raise ValueError('initial HOLD requires normal online modes, not a preloaded execution')
    if 'mode_session_id'in result and (not isinstance(result['mode_session_id'],str)or not 0<len(result['mode_session_id'])<=200):
        raise ValueError('bounded explicit mode session identity required')
    if normal is not None:
        keys={'brake_duration_s','velocity_limit','acceleration_limit','gain_transition_s','teach_kd_scale',
              'recording_required','recorder_ids','recording_clock_id'}
        if (not isinstance(normal,dict)or set(normal)!=keys
                or any(type(normal[k])not in(int,float)or not math.isfinite(normal[k])or normal[k]<=0
                       for k in('brake_duration_s','velocity_limit','acceleration_limit','gain_transition_s'))
                or normal['brake_duration_s']>30
                or normal['velocity_limit']>(1. if result.get('reference_planning')else .25)
                or normal['acceleration_limit']>(40. if result.get('reference_planning')else 1.)
                or normal['gain_transition_s']>normal['brake_duration_s'] or result.get('bounded_live_prediction')
                or type(normal['teach_kd_scale'])not in(int,float)or not math.isfinite(normal['teach_kd_scale'])or not 0<=normal['teach_kd_scale']<=1
                or type(normal['recording_required'])is not bool or not isinstance(normal['recorder_ids'],list)
                or any(not isinstance(x,str)or not x for x in normal['recorder_ids'])
                or len(set(normal['recorder_ids']))!=len(normal['recorder_ids'])
                or normal['recording_clock_id']!='host_wall_time_ns'
                or normal['recording_required']and not normal['recorder_ids']and context_policy!='per_episode_review'
                or not normal['recording_required']and normal['recorder_ids']
                or result.get('reference_integral_feedback')or result.get('reference_feedforward')
                or result.get('disabled_grippers')):
            raise ValueError('explicit bounded normal-mode configuration with retained active jaws required')
    binding=result.get('action_chunk_binding')
    if binding is not None and (not isinstance(binding,dict)or set(binding)!=
            {'session_id','model_binding','conversion_id','action_contract','task'}):
        raise ValueError('trusted prepared action chunk binding required')
    if horizon==32:
        from .inference import action_shape
        if binding is None:raise ValueError('WAM requires its pinned32-step action contract before control')
        action_shape(binding['action_contract'])
    if binding is not None:
        maximum=result.get('maximum_observation_age_s',age_limit)
        if type(maximum)not in(int,float)or not math.isfinite(maximum)or not 0<maximum<=age_limit:
            raise ValueError('observation/result freshness exceeds the bound model capability')
        result['maximum_observation_age_s']=maximum
    if 'reference_supply_timeout_s'in result and (type(result['reference_supply_timeout_s'])not in(int,float)
            or not 0<result['reference_supply_timeout_s']<=.5):raise ValueError('reference supply timeout must be positive and at most0.5 s')
    integral=result.get('reference_integral_feedback',False)
    if type(integral)is not bool or integral and result.get('trajectory_kind')!='prepared_native_reference':
        raise ValueError('reference integral requires prepared native reference configuration')
    if result.get('reference_feedforward')and not integral:raise ValueError('feedforward requires qualified integral research profile')
    result['duration_s'] = episode_duration(result.get('duration_s', 120))
    if result.get('bounded_live_prediction'):
        from .inference_model import model_family,NOHISTORY_MODELS
        if (model_family(result.get('model_id'))!='mixed366' or result['duration_s']!=.5
                or result.get('task')not in ('coffee','lid_handoff') or result.get('execute_steps')!=15
                or result.get('target_execution')!='shared_reference_windows'):
            raise ValueError('bounded live prediction requires mixed366, both-arm shared windows and 15 future samples')
    if 'execution_duration_s'in result:
        prepared=result.get('prepared_reference')or{};actual=result['execution_duration_s']
        from .motion_reference import native_holds
        if (result.get('trajectory_kind')!='prepared_native_reference' or type(actual)not in(int,float)
                or not math.isfinite(actual)or not 0<actual<=40
                or not math.isclose(actual,prepared.get('duration_s',-1)+sum(native_holds(result)),abs_tol=1e-9)):
            raise ValueError('actual execution duration must match the prepared reference and explicit holds')
    fixed = dict(control_hz=30, collision_check=False,
                 position_projection=True, motion_audit='record_only',duration_tolerance_s=POLICY_DURATION_TOLERANCE_S)
    for key, expected in fixed.items():
        if key in result and (result[key] != expected or isinstance(expected, bool) and type(result[key]) is not bool):
            raise ValueError('unsupported inference configuration: '+key)
        result[key] = expected
    result.setdefault('seed', 200000)
    if type(result['seed']) is not int or not 0 <= result['seed'] < 2**63:
        raise ValueError('inference seed must be an integer in [0, 2**63)')
    disabled=result.setdefault('disabled_grippers',[])
    if disabled not in ([],['right']):raise ValueError('inference requires an active left gripper; only right may be disabled')
    if disabled:
        mapping=(result.get('conversion_identity') or {}).get('mapping') or {}
        gap=result.get('disabled_right_gripper_device_mm');model=result.get('disabled_right_gripper_model_m')
        offset=mapping.get('model_gap_offset_mm')
        if (any(type(v) not in (int,float) or not math.isfinite(v) for v in (gap,model,offset))
                or result.get('disabled_right_gripper_assumption')!='profile_max_opening_not_measured'
                or gap!=mapping.get('profile_max_opening_mm')
                or not math.isclose(model,(gap-offset)/1000.,rel_tol=0,abs_tol=1e-12)):
            raise ValueError('disabled right jaw requires its calibrated profile-opening assumption, not measured feedback')
    digest(result)  # Reject callables, NaN and unrecordable configuration before review.
    return result


def prepare_finish_binding(policy, owner, kind):
    """Bootstrap-only pin resolution; the Owner tick never reads policy files."""
    from pathlib import Path
    from .control_plan import _read_file
    from .execution_settings import validate
    from .sdk_quiesce import load_gripper_calibration
    if not isinstance(policy,dict)or set(policy)!={'jaw_mapping','execution_settings','max_velocity_rad_s','jaw_speed_mm_s','timeout_s'}:
        raise ValueError('exact frozen normal finish policy required')
    policy=deepcopy(policy);settings=validate(policy['execution_settings'],environment=owner.environment)
    if not all(settings[k]for k in('arrival_enabled','tracking_enabled'))or not settings['speed_protection']['enabled']:
        raise ValueError('normal finish requires arrival, tracking and measured-speed protection')
    if any(type(policy[k])not in(int,float)or not math.isfinite(policy[k])or policy[k]<=0
           for k in('max_velocity_rad_s','jaw_speed_mm_s','timeout_s')):
        raise ValueError('explicit positive normal finish limits required')
    pin=policy['jaw_mapping']
    if not isinstance(pin,dict)or set(pin)!={'path','sha256'}or not isinstance(pin['path'],str)or not Path(pin['path']).is_absolute():
        raise ValueError('absolute pinned finish jaw mapping required')
    raw=_read_file(pin['path'],128*1024)
    if hashlib.sha256(raw).hexdigest()!=pin['sha256']:raise ValueError('finish jaw mapping pin changed')
    data=json.loads(raw)
    if not isinstance(data,dict)or set(data)!={'schema_version','robot_id','native_gripper_calibration'}or data['schema_version']!='native_gripper_mapping.v1':
        raise ValueError('normal finish requires the bound native jaw mapping')
    native=load_gripper_calibration(data['native_gripper_calibration'],robot_id=data['robot_id'])
    mapping=deepcopy(owner.cache.get('jaw_mapping'))
    if (not mapping or mapping.get('source_sha256')!=pin['sha256']or mapping.get('native_calibration_id')!=native['identity']
            or getattr(owner.backend,'native_gripper_calibration_id',None)!=native['identity']):
        raise ValueError('finish endpoints differ from the connected SDK and current mapping')
    policy['execution_settings']=settings
    return dict(policy=policy,mapping=mapping,kind=kind,policy_digest=digest(dict(policy=policy,mapping=mapping,kind=kind)))


def check_gripper_policy(owner, config, feedback=None):
    expected=tuple(config.get('disabled_grippers',()))
    actual=tuple(getattr(owner.backend,'passive_jaws',()))
    return validate_gripper_feedback(actual,expected,feedback)


def validate_gripper_feedback(actual,expected,feedback=None):
    if actual!=expected:raise ValueError('inference disabled-gripper policy changed from the reviewed backend')
    flags=(feedback or {}).get('gripper_feedback_valid')
    if (expected and flags!=[True,False]) or (not expected and flags is not None and flags!=[True,True]):
        raise ValueError('inference gripper feedback availability differs from the reviewed backend')
    return expected


def preview(owner, command, value):
    from arx_env.joint_head_real_motion import feedback_to_command, make_plan
    if set(command)-{'action', 'seed', 'duration_s','recording_context'}:
        raise ValueError('inference preview accepts a seed and 1, 40 or 120 source seconds only')
    config = configuration(dict(value, seed=command.get('seed', value.get('seed', 200000))))
    context=command.get('recording_context')
    if config.get('recording_context_policy')=='per_episode_review':
        if (not isinstance(context,dict)or set(context)!={'session_id','recorder_ids','clock_id'}
                or not isinstance(context['session_id'],str)or not 0<len(context['session_id'])<=200
                or context['clock_id']!='host_wall_time_ns' or not isinstance(context['recorder_ids'],list)
                or not 1<=len(context['recorder_ids'])<=64
                or any(not isinstance(x,str)or not 0<len(x)<=200 for x in context['recorder_ids'])
                or len(set(context['recorder_ids']))!=len(context['recorder_ids'])):
            raise ValueError('this review requires exact fresh episode/recorder membership')
        config['mode_session_id']=context['session_id']
        config['normal_modes']['recorder_ids']=context['recorder_ids'][:]
    elif context is not None:raise ValueError('recorder membership is fixed by the configured policy')
    duration = episode_duration(command.get('duration_s', config['duration_s']))
    if duration != config['duration_s']:
        raise ValueError('inference duration differs from the configured episode')
    feedback = owner.cache['calibration_feedback']
    check_gripper_policy(owner,config,feedback)
    limits = deepcopy(owner.cache['joint_limits'])
    target = feedback_to_command(feedback['qpos'], limits, environment=owner.environment)
    execution_duration=config.get('execution_duration_s',duration)
    proxy = make_plan('both', target, target, limits, kind='workbench', duration_s=execution_duration,
                      q_start_is_command=True, created_at_s=owner.control_time(), environment=owner.environment)
    # This is an approval identity for online execution, not a certified full trajectory.
    payload = dict(configuration=config, joint_limits=limits, initial_qpos=deepcopy(feedback['qpos']),
                   clock=deepcopy(getattr(owner, 'feedback_clock', None)), review_nonce=secrets.token_hex(16))
    proxy = replace(proxy, kind='inference', sha256=digest(dict(proxy=asdict(proxy), payload=payload)))
    token = secrets.token_urlsafe(24)
    owner.pending = {token: ((proxy,), owner.epoch, owner.pedal_snapshot()[0], os.getpid(), [None])}
    owner.inference_payloads = {proxy.sha256: payload}
    owner.review_wall_deadline = time.monotonic()+30.
    return dict(token=token, owner_pid=os.getpid(), command_epoch=owner.epoch, expires_in_s=30,
                plan=asdict(proxy), plans=[asdict(proxy)], sequence_id=proxy.sha256, duration_s=duration,
                total_motion_duration_s=execution_duration,execution_duration_s=execution_duration,purpose='policy_inference_episode',
                configuration=deepcopy(config), configuration_digest=digest(config),
                control_hz=30, collision_checked=False, position_projection='recorded',
                motion_audit='record_only', trajectory_kind='online_predictions_not_frozen_plan')


def device_state(raw, *, disabled_grippers=(), right_gripper_device_mm=None):
    from arx_env.joint_head_jaw_mapping import feedback_gaps
    value = vector14(raw, 'SDK feedback').tolist()
    value[6], value[13] = feedback_gaps(value)
    if 'right' in disabled_grippers:
        if type(right_gripper_device_mm) not in (int,float) or not math.isfinite(right_gripper_device_mm):
            raise ValueError('explicit assumed disabled right jaw device value required')
        value[13]=float(right_gripper_device_mm)
    return value


def sdk_target(value):
    from arx_env.joint_head_jaw_mapping import command_sdk
    result = vector14(value, 'device command').tolist()
    result[6], result[13] = command_sdk('left', result[6]), command_sdk('right', result[13])
    return result


def device_limits(sdk_limits,*,native_coordinates=False):
    from arx_env.joint_head_jaw_mapping import command_gap_mm
    result = [list(pair) for pair in sdk_limits]
    for side, slot in (('left', 6), ('right', 13)):
        lo, hi = (command_gap_mm(side, value) for value in result[slot])
        result[slot] = [max(0., lo), hi if native_coordinates else min(83., hi)]
    return result


class InferenceRunner:
    is_inference = True

    def __init__(self, owner, proxy, feedback, config, *, predict, observe, adapter,
                 support=None, event=None, record=None, record_prepared=None, applied=None,preloaded_prediction=None,prepared_reference=None,feedforward_profile=None,reference_executor=None):
        from arx_env.joint_head_real_motion import StationaryWindow, feedback_to_command
        from arx_env.joint_head_jaw_mapping import command_gap_mm
        self.owner, self.plan, self.config = owner, proxy, configuration(config)
        if self.config.get('reference_planning')is not None:
            if not owner.preserve_begin_jaws:raise ValueError('new infer curve requires an explicitly initialized native hold')
            if owner.environment=='physical':
                speed=owner.motion_error_settings()['speed_protection']
                if not speed['enabled']or speed['limit_rad_s']<self.config['reference_planning']['velocity_limit_rad_s']:
                    raise ValueError('planning velocity exceeds current measured-speed protection; explicit policy alignment required')
        self.disabled_grippers=check_gripper_policy(owner,self.config,feedback)
        self.observe, self.support_observe, self.applied_check = observe, support, applied
        self.finite_chunk_count=self.config.get('finite_chunk_count',0)
        self.finite_completed=[];self.finite_stage=None;self.finite_observation_floor=None
        self.finite_candidate=None;self.finite_current=None;self.finite_partial_s=0.
        self.bounded_live=bool(self.config.get('bounded_live_prediction')or self.finite_chunk_count)
        self.bounded_result=None;self.bounded_request=None;self.predict=predict
        self.bounded_worker=None
        self.adapter=adapter
        self.event, self.record = event, record
        self.record_prepared = record_prepared
        self.preloaded_prediction=preloaded_prediction
        self.prepared_reference=prepared_reference
        self.reference_executor=reference_executor
        previous_runner=getattr(owner,'runner',None)
        previous=getattr(previous_runner,'target_interpreter',None)
        self.previous_native_hold=(previous if previous is not None and previous_runner.status=='READY'
            and not previous.cancelled and previous.epoch==owner.epoch and previous.active_arm=='both'
            and not previous.integrals and previous.feedforward is None
            and (previous.reference.metadata.get('kind')=='normal_hold'
                 or owner.control_time()>=previous.start_s+previous.reference.duration_s)else None)
        self.initialize_online_hold=bool(getattr(owner,'preserve_begin_jaws',False)
            and prepared_reference is None and self.previous_native_hold is None)
        self.initialization=None;self.initialization_source_s=None
        self.normal_config=self.config.get('normal_modes')
        self.normal_planned=bool(self.normal_config and self.config.get('reference_planning'))
        self.normal_single_segment=bool(self.normal_planned and self.normal_config['recording_required']and self.config['duration_s']==.5)
        self.normal_observation_floor=None
        self.normal_segment=None
        self.mode='INITIALIZING'if self.normal_config else'MODEL'
        self.mode_receipts={};self._mode_payloads={};self.normal_job=None
        self.normal_ended=False
        self.finish_completed=False
        self._accepted_chunk_source_time=None
        if self.normal_config and prepared_reference is not None:
            from .motion_reference import check_brake_reserve
            check_brake_reserve(prepared_reference,proxy.joint_limits,duration_s=self.normal_config['brake_duration_s'],
                velocity_limit=self.normal_config['velocity_limit'],acceleration_limit=self.normal_config['acceleration_limit'])
        self.target_interpreter=None
        self.chunks_exhausted=False
        self.latest_feedback=feedback
        from .motion_reference import ReferenceIntegral,native_holds
        self.reference_integral=ReferenceIntegral()if self.config.get('reference_integral_feedback')else None
        self.native_initial_hold,self.native_terminal_hold=native_holds(self.config)
        self.feedforward_profile=feedforward_profile;self.reference_feedforward=None
        if (feedforward_profile is not None and (self.config.get('reference_feedforward')or{}).get('sha256')!=feedforward_profile['sha256']):
            raise ValueError('feedforward differs from frozen review')
        self.execution_duration_s=(prepared_reference.duration_s+self.native_initial_hold+self.native_terminal_hold if prepared_reference is not None else self.config['duration_s'])
        if prepared_reference is not None and (self.config.get('prepared_reference')or{}).get('reference_id')!=prepared_reference.metadata['reference_id']:
            raise ValueError('prepared reference differs from the frozen execution configuration')
        self.native_next_upload=0.;self.native_queued_until=None;self.native_pending=[]
        self.status, self.reason = 'ARMED', ''
        self.stationary = StationaryWindow()
        if getattr(owner,'control_guard',None) is not None:self.stationary.guard=owner.control_guard
        self.execution_start_corrections = []
        self.execution_target = None
        self.target_error = None
        self.ready_since = self.previous = self.started = None
        self.execution_instance_id = owner.execution_instance_id
        self.mode_session_id=self.config.get('mode_session_id',self.execution_instance_id)
        self.command_context = None
        self.outstanding = None
        self.history_output=None
        self.recording_error = None
        self.common_capture=self.config.get('recording_backend')=='capture_session_v1'
        self.capture_sequence=0
        self.capture_lock=threading.Lock()
        self.data_done = threading.Event()
        self.data_queue = queue.Queue(maxsize=8192) if record else None
        self._record_thread = None
        self._data_finish_requested = False
        self.preparation_complete = False
        self.preparation_deadline = self.verify_started = None
        self.actual_submission_count = 0
        self._epoch = owner.epoch
        self.sdk_limits = tuple(tuple(pair) for pair in proxy.joint_limits)
        self.jaw_identity = deepcopy(owner.cache.get('jaw_mapping'))
        held = list(feedback_to_command(feedback['qpos'], self.sdk_limits, environment=owner.environment,
                                       corrections=self.execution_start_corrections))
        self.initial_output_evidence=None
        if getattr(owner,'preserve_begin_jaws',False):
            self.initial_output_evidence=owner.backend.read_execution_output()
            if self.previous_native_hold is not None:held=list(self.initial_output_evidence['q_sdk14'])
            else:held[6],held[13]=(self.initial_output_evidence['q_sdk14'][i]for i in(6,13))
        held[6], held[13] = command_gap_mm('left', held[6]), command_gap_mm('right', held[13])
        if self.disabled_grippers:held[13]=self.config['disabled_right_gripper_device_mm']
        self.initial_device14 = held
        self.last_device_target = list(held)
        self.limits = device_limits(proxy.joint_limits,native_coordinates=self.config.get('binding',{}).get('schema')==NATIVE_SCHEMA)
        if self.disabled_grippers:self.limits[13]=[0.,self.config['disabled_right_gripper_device_mm']]
        velocity = self.config.get('velocity_limits', [2.]*6+[None]+[2.]*6+[None])
        acceleration = self.config.get('acceleration_limits', [5.]*6+[None]+[5.]*6+[None])
        # The runner owns the loop; callbacks must not own the runner back.
        runner_ref=weakref.ref(self)
        def predict_before_deadline(request):
            runner=runner_ref()
            if runner is None:raise ValueError('inference runner retired before model query')
            now=runner.owner.control_time()
            if runner._guard_decision('G23',dict(start_time=runner.started,end_time=now),now,now,'model_query',phase='query')['verdict']=='reject':
                raise ValueError('inference episode deadline passed before model query')
            return predict(request)
        def emit(name,value):
            runner=runner_ref()
            if runner is not None:runner._emit(name,value)
        def prepare_chunk(*args):
            runner=runner_ref()
            return runner._prepare_native_chunk(*args)if runner is not None else None
        def prepare_response(*args):
            runner=runner_ref()
            if runner is None:raise ValueError('inference runner retired before preparation')
            return runner._prepare_native_response(*args)
        limits=self.limits
        from .inference_model import model_family,NOHISTORY_MODELS
        self.loop = InferenceLoop(predict=predict_before_deadline, decode=lambda row, _state: adapter.decode(row, held),
            device_to_model=adapter.device_to_model,
            audit=lambda rows, prior:audit_dynamics(rows, prior, position_limits=limits,
                velocity_limits=velocity, acceleration_limits=acceleration),
            event=emit, seed=self.config['seed'], duration_s=self.execution_duration_s,execute_steps=self.config['execute_steps'],
            history_steps=model_capabilities(self.config)[1],
            action_horizon=model_capabilities(self.config)[0],
            maximum_observation_age_s=self.config.get('maximum_observation_age_s', 2.),
            support=StableTraySupport(.1, .1),
            prepare_chunk=prepare_chunk if owner.shared_target_profile and prepared_reference is None else None,
            prepare_response=prepare_response if owner.shared_target_profile and prepared_reference is None else None)
        if not record:
            self.data_done.set()

    @property
    def recording_pending(self):
        return self.record is not None and not self.data_done.is_set()

    def _guard_decision(self, rule_id, observed, source_time, checked_time, signal_id,*,phase='submission'):
        from .execution_settings import Guard, control_parameters, defaults, policy_hash
        binding=getattr(self.owner,'control_guard',None)or getattr(self,'_bound_guard',None)
        if binding is None:
            policy = getattr(self, 'guard_policy', None)
            if policy is None:
                frozen = (getattr(self.owner, 'execution_calibration', None) or {}).get('control_policy')
                if frozen is not None:policy = deepcopy(frozen)
                else:
                    settings = self.owner.motion_error_settings() if callable(getattr(self.owner, 'motion_error_settings', None)) else defaults('inference')
                    parameters = control_parameters(settings, configuration=self.config, business='inference')
                    policy = dict(revision=None, policy_hash=policy_hash(parameters), parameters=parameters)
                self.guard_policy = policy
            context={'run_duration_s':self.config['duration_s']}if 'duration_s'in self.config else{}
            if getattr(self,'sdk_limits',None) is not None:
                limits=device_limits(self.sdk_limits,native_coordinates=self.config.get('binding',{}).get('schema')==NATIVE_SCHEMA);context['jaw_limits_mm']=[list(limits[i])for i in (6,13)]
                source=getattr(getattr(self,'adapter',None),'limits',None)
                context['model_jaw_limits_mm']=[list((source if source is not None else limits)[i])for i in (6,13)]
                context['disabled_grippers']=list(getattr(self,'disabled_grippers',self.config.get('disabled_grippers',())))
            if self.config.get('normal_modes') is not None:
                context.update(normal_brake_s=self.config['normal_modes']['brake_duration_s'],gain_transition_s=self.config['normal_modes']['gain_transition_s'])
            binding=Guard.bind(policy,context);self._bound_guard=binding
        signal = dict(rule_id=rule_id, signal_id=signal_id, observed=observed,
                      source_time=source_time, checked_time=checked_time, epoch=self._epoch)
        request = getattr(self, 'bounded_request', None)
        if request is not None:signal['request_id'] = request['request_id']
        decision=binding.evaluate(signal,phase)
        self._emit('guard_decision', decision)
        return decision

    def _emit(self, name, data):
        value = dict(data, execution_instance_id=self.execution_instance_id)
        if self.config.get('data_plane') is not None:
            # The one ROS fact is observed independently by UI and recorder.
            if self.record is not None:self._record(name,value)
            elif self.event:self.event(name,value)
            return
        self.owner.event('inference_'+name, **deepcopy(value))
        if self.event:
            self.event(name, deepcopy(value))
        self._record(name, value)

    def _check_model_reference(self,reference):
        """Admit a complete model segment against the Run's one frozen policy."""
        if self.config.get('reference_planning') is None:return
        now=self.owner.control_time()
        source=(getattr(self,'bounded_request',None)or{}).get('observation',{}).get('source_time',now)
        values=[('G13',reference.metadata['max_velocity_rad_s']),
                ('G15',reference.metadata['max_acceleration_rad_s2']),
                ('G16',reference.metadata['max_jaw_velocity_mm_s']),
                ('G21',len(reference.breaks)-1),('G22',reference.duration_s)]
        for rule,observed in values:
            decision=self._guard_decision(rule,observed,source,now,reference.metadata['reference_id'])
            if decision['verdict']=='reject':
                raise ValueError(f"{rule}: complete model segment rejected: {observed} {decision['unit']}; limit {decision['limit']}")

    def _record(self, name, data):
        if self.bounded_live and self.record is None and name in ('feedback','command','native_window'):
            self.owner.event('inference_diagnostic_'+name,execution_instance_id=self.execution_instance_id,**deepcopy(data))
        common=getattr(self,'common_capture',False)
        if self.data_queue is None or self.recording_error and not common or self._data_finish_requested:
            return
        try:
            copied=deepcopy(data)
            if common:
                with self.capture_lock:
                    if self._data_finish_requested:return
                    self.capture_sequence+=1
                    copied.update(capture_sequence=self.capture_sequence,capture_source_time=self.owner.control_time(),
                                  capture_wall_ns=time.time_ns())
                    self.data_queue.put_nowait((name,copied))
            else:self.data_queue.put_nowait((name,copied))
        except queue.Full:
            self._recording_failure('policy recording event queue overflow; evidence incomplete')

    def _recording_failure(self,error):
        first=self.recording_error is None
        self.recording_error=self.recording_error or str(error)
        if not getattr(self,'common_capture',False):
            self.owner.recording_failed(error);return
        from .execution_settings import Guard
        policy=self.control_policy
        now=self.owner.control_time()
        decision=Guard.evaluate(dict(rule_id='G38',signal_id=self.execution_instance_id,observed=1,
            source_time=now,checked_time=now,run_id=self.execution_instance_id),'capture',policy)
        if first:self.owner.event('capture_fault',reason=self.recording_error,decision=decision)
        if decision['action']!='CONTINUE':self.owner.recording_failed(error)

    def _data_worker(self):
        try:
            while True:
                name, value = self.data_queue.get()
                try:
                    self.record('finish' if name == '_finish' else name, value)
                except Exception as exc:
                    self._recording_failure(exc)
                if name == '_finish':
                    return
        finally:
            self.data_done.set()

    def _finish_data(self, status, reason):
        if self._data_finish_requested or self.data_queue is None:
            return
        if getattr(self,'common_capture',False):
            with self.capture_lock:
                if self._data_finish_requested:return
                self._data_finish_requested=True
                final_sequence=self.capture_sequence
        else:self._data_finish_requested = True
        if self._record_thread is None:
            self.data_done.set()
            return
        value = dict(status=status, reason=reason, execution_instance_id=self.execution_instance_id,
                     inference=self.loop.snapshot(), recording_error=self.recording_error)
        if getattr(self,'common_capture',False):value.update(final_capture_sequence=final_sequence,
            source_time=self.owner.control_time(),finished_wall_ns=time.time_ns())
        # Queued on another thread so a stuck data consumer never blocks stop.
        threading.Thread(target=lambda:self.data_queue.put(('_finish', value)), daemon=True).start()

    def revoke(self, reason='owner_invalidated', *, finish_data=True):
        if self.finite_chunk_count and self.finite_stage not in('completed','failed'):
            self._finite_freeze_elapsed();self.finite_stage='stopped'
        if self.initialization is not None and self.started is None and self.status!='FAULT':
            self.initialization.update(stage='cancelled',reason=reason)
        if self.target_interpreter is not None:self.target_interpreter.revoke()
        self.loop.stop(reason)
        self.status, self.reason = 'STOPPED', reason
        self.outstanding = None
        self.history_output=None
        self.command_context = None
        if finish_data:
            self.mode='STOPPED'
            for request_id,receipt in self.mode_receipts.items():
                if receipt.get('status')in('accepted','running'):
                    self.reject_mode(request_id,reason)
            self.normal_job=None
            self._finish_data('stopped', reason)

    def submit_mode(self, request):
        if not self.normal_config:raise ValueError('normal modes are not configured')
        allowed={'action','session_id','runtime_nonce','request_id','epoch','mode','target'}
        if (not isinstance(request,dict)or set(request)not in(allowed,allowed|{'recording_gate'})
                or request['action']!='normal_mode' or request['target']not in('MODEL','HOLD','TEACH','HOME','END','DISCARD')
                or not isinstance(request['request_id'],str)or not 0<len(request['request_id'])<=128
                or request['mode']!=('HOLD'if request['target']in('END','DISCARD')else request['target'])):
            raise ValueError('exact normal mode request/terminal target required')
        key=request['request_id'];fingerprint=digest(request)
        if key in self.mode_receipts:
            if self._mode_payloads[key]!=fingerprint:raise ValueError('mode request ID reused with different content')
            return deepcopy(self.mode_receipts[key])
        if self.normal_ended:raise ValueError('normal episode ended; prepare a new reviewed episode')
        if self.started is None:raise ValueError('normal mode initialization has not completed')
        if getattr(self,'normal_supply_exhausted',False)and self.mode=='MODEL'and request['mode']=='MODEL':
            raise ValueError('exhausted model requires completed HOLD before a fresh MODEL request')
        if self.status in('STOPPED','FAULT')or self.owner.control.state is self.owner.states.FAULT:
            raise ValueError('normal mode cannot recover a fault or revoked runner')
        if len(self.mode_receipts)>=256:raise ValueError('mode receipt ledger full; start a new reviewed episode')
        preempt=(self.normal_job is not None and self.normal_job['stage']=='RESUMING'
            and self.normal_planned and self.normal_segment is None and request['mode']=='HOLD'
            and request['target']=='HOLD'and not self.normal_config['recording_required'])
        proof=validate_mode_request(request,identity=dict(session_id=self.mode_session_id,
            runtime_nonce=self.owner.runtime_nonce,epoch=self.owner.epoch),
            current_mode='MODEL'if preempt else'BRAKE'if self.normal_job else self.mode,clock_id='host_wall_time_ns',now_ns=time.time_ns(),
            recorder_ids=self.normal_config['recorder_ids'],recording_required=self.normal_config['recording_required'])
        if preempt:
            self.reject_mode(self.normal_job['request']['request_id'],'operator cancelled pending MODEL preparation',error_class='operator')
        receipt={k:deepcopy(request[k])for k in('session_id','runtime_nonce','request_id','epoch','mode','target')}
        receipt.update(op='normal_mode',requested_mode=request['mode'],mode=self.mode,
            accepted=True,completed=proof['no_op'],no_op=proof['no_op'],
            command_epoch=self.owner.epoch,status='completed'if proof['no_op']else'accepted',
            accepted_source_s=self.owner.control_time(),source_clock_id=self.owner.execution_clock_id,firmware_ack=False)
        if not proof['no_op']:
            self.owner.commands.put_nowait(('normal_mode',self.owner.epoch,deepcopy(request)))
            # Mark intent pending before another HTTP request can queue MODEL.
            self.normal_job=dict(request=deepcopy(request),stage='QUEUED')
        self._mode_payloads[key]=fingerprint;self.mode_receipts[key]=receipt
        self._emit('mode_transition',deepcopy(receipt))
        return deepcopy(receipt)

    def reject_mode(self, request_id, reason, *, error_class='control', **evidence):
        receipt=self.mode_receipts.get(request_id)
        if receipt is not None and receipt.get('status')!='completed':
            receipt.update(status='failed',completed=False,error=str(reason),error_class=error_class,
                mode=self.mode,command_epoch=self.owner.epoch,**evidence)
            self._emit('mode_transition',deepcopy(receipt))
    def submit_finish(self, request):
        binding=self.config.get('_normal_finish')
        if not binding:raise ValueError('normal finish policy was not qualified at bootstrap')
        fields={'action','session_id','runtime_nonce','request_id','epoch','ending_request_id','policy_digest'}
        if (not isinstance(request,dict)or set(request)not in(fields,fields|{'raw_sealed'})
                or request['action']!='normal_finish'or not isinstance(request['request_id'],str)
                or not 0<len(request['request_id'])<=128):raise ValueError('exact identified normal finish request required')
        key=request['request_id'];fingerprint=digest(request)
        if key in self.mode_receipts:
            if self._mode_payloads[key]!=fingerprint:raise ValueError('finish request ID reused with different content')
            return deepcopy(self.mode_receipts[key])
        ending=self.mode_receipts.get(request['ending_request_id'],{})
        if (request['session_id']!=self.mode_session_id or request['runtime_nonce']!=self.owner.runtime_nonce
                or type(request['epoch'])is not int or request['epoch']!=self.owner.epoch
                or request['policy_digest']!=binding['policy_digest']or not self.normal_ended
                or self.normal_job is not None or self.mode!='HOLD'or self.status!='READY'
                or self.owner.control.state is self.owner.states.FAULT or not self.owner.feedback_healthy
                or ending.get('target')not in('END','DISCARD')or ending.get('completed')is not True
                or ending.get('command_epoch')!=self.owner.epoch):
            raise ValueError('normal finish requires this healthy terminal HOLD, epoch and frozen policy')
        if self.normal_config['recording_required']:
            sealed=request.get('raw_sealed')or{};rows=sealed.get('recorders')if isinstance(sealed,dict)else None
            if (not isinstance(sealed,dict)or sealed.get('completed')is not True
                    or type(sealed.get('epoch'))is not int
                    or any(sealed.get(k)!=request[k]for k in('session_id','runtime_nonce','epoch','ending_request_id'))
                    or not isinstance(rows,list)or len(rows)!=len(self.normal_config['recorder_ids'])
                    or any(not isinstance(r,dict)or r.get('completed')is not True or not isinstance(r.get('record_id'),str)
                           or r.get('status')!=('saved'if ending['target']=='END'else'discarded')for r in rows)
                    or {r['record_id']for r in rows}!=set(self.normal_config['recorder_ids'])):
                raise ValueError('all terminal Raw recorders must be sealed before normal finish')
        if len(self.mode_receipts)>=256:raise ValueError('mode receipt ledger full')
        internal=dict(deepcopy(request),mode='HOLD',target=ending['target'])
        self.owner.commands.put_nowait(('normal_finish',self.owner.epoch,internal))
        self.normal_job=dict(request=internal,stage='FINISH_QUEUED')
        receipt={k:internal[k]for k in('session_id','runtime_nonce','request_id','epoch','mode','target')}
        receipt.update(op='normal_finish',accepted=True,completed=False,status='accepted',command_epoch=self.owner.epoch,
            accepted_source_s=self.owner.control_time(),
            source_clock_id=self.owner.execution_clock_id,policy_digest=binding['policy_digest'],
            home_completed=False,jaw_open_completed=False,release_ready=False,sdk_released=False)
        self.mode_receipts[key]=receipt;self._mode_payloads[key]=fingerprint
        self._emit('mode_transition',deepcopy(receipt));return deepcopy(receipt)

    def start_finish(self, request, feedback):
        job=self.normal_job
        if not job or job['request']!=request:raise ValueError('finish queued identity changed')
        output=self._normal_output();binding=self.config['_normal_finish'];policy=binding['policy']
        from arx_env.joint_head_real_motion import jaw_peak_scale
        limits=vector14(output.get('velocity_limits_sdk14'),'native velocity limits')
        if (any(x<=0 for x in limits)or policy['max_velocity_rad_s']>min(limits[i]for i in(*range(6),*range(7,13)))
                or policy['max_velocity_rad_s']>policy['execution_settings']['speed_protection']['limit_rad_s']
                or any(policy['jaw_speed_mm_s']/jaw_peak_scale(s)>limits[i]for s,i in(('left',6),('right',13)))):
            raise ValueError('normal finish exceeds frozen SDK or speed protection limits')
        previous=self.target_interpreter;reference,origin=previous.reference,previous.start_s
        self._normal_epoch();self._bind_normal(reference,origin)
        job.update(stage='FINISH',started_s=self.owner.control_time(),finish_index=0,finish_records=[],finish_runner=None)
        self.mode='FINISH';self._mode_event('finish_started',mode='FINISH',mapping=binding['mapping'],policy=policy)
        self._start_finish_stage(feedback)

    def _start_finish_stage(self, feedback):
        from arx_env.joint_head_real_motion import MotionRunner,make_plan
        from arx_env.joint_head_jaw_mapping import command_sdk,command_max_mm
        job=self.normal_job;binding=self.config['_normal_finish'];policy=binding['policy']
        output=self._normal_output();start=list(output['q_sdk14']);target=start[:];index=job['finish_index']
        if binding['kind']=='maintenance_home_v1':
            for i in(*range(6),*range(7,13)):target[i]=0.
        else:
            offset=0 if index<2 else 7;side='left'if offset==0 else'right'
            target[offset:offset+6]=[0.]*6;target[offset+2]=.03 if index%2==0 else 0.
            target[offset+6]=command_sdk(side,command_max_mm(side))
        now=self.owner.control_time()
        options=dict(kind='workbench',created_at_s=now,max_velocity=policy['max_velocity_rad_s'],
            jaw_speed_mm_s=policy['jaw_speed_mm_s'],environment=self.owner.environment)
        plan=make_plan('both',feedback['qpos'],target,self.sdk_limits,**options)
        if binding['kind']=='dagger_reset_v1'and plan.duration_s<.5:
            plan=make_plan('both',feedback['qpos'],target,self.sdk_limits,duration_s=.5,**options)
        def interpreter(reference,side,stamp):
            self.target_interpreter.revoke();self._bind_normal(reference,stamp)
            return self.target_interpreter
        runner=MotionRunner(plan,begin=lambda q:q,send=lambda q:(_ for _ in()).throw(AssertionError('finish cannot point-send')),
            stop=self.owner.motion_fault,consumed=self.owner.consumed,environment=self.owner.environment,
            execution_settings=policy['execution_settings'],interpreter_factory=interpreter,command_start_factory=lambda:start[:])
        def record_window(result):
            self.actual_submission_count+=1
            self._record('native_window',dict(result,finish_stage=index))
        runner.reference_event=record_window
        job['finish_runner']=runner;runner.start(now,feedback)
        self._mode_event('finish_stage_started',finish_stage=index,target_sdk14=target,plan_sha256=plan.sha256)

    def _tick_finish(self, now, feedback):
        from .execution_settings import measured_joint_speed,JointSpeedProtectionError
        from .target_interpreter import prepare_measured_hold
        job=self.normal_job;binding=self.config['_normal_finish'];policy=binding['policy']
        if self.owner.cache.get('jaw_mapping')!=binding['mapping']:raise ValueError('finish mapping changed')
        if now-job['started_s']>policy['timeout_s']:raise ValueError('normal finish exceeded frozen operation budget')
        speed=measured_joint_speed(feedback,policy['execution_settings']['speed_protection'])
        if speed['peak_rad_s']>speed['limit_rad_s']:raise JointSpeedProtectionError(speed)
        runner=job['finish_runner']
        if runner.target_interpreter is None:self._queue_native(now,feedback)
        status=runner.tick(now,feedback)
        self.execution_target=runner.execution_target;self.target_error=runner.target_error
        if status in('FAULT','STOPPED'):
            self.status='FAULT';self.reason='normal finish failed: '+runner.reason
            return True  # MotionRunner already invoked the sole Owner fault path.
        if status!='READY':return True
        record={k:v for k,v in runner.motion_record().items()if k not in('rows','native_samples')}
        record.update(reference_id=runner.target_interpreter.reference.metadata['reference_id'],
            native_window_submissions=runner.target_interpreter.submissions,completed_source_s=feedback['sample_time_s'])
        job['finish_records'].append(record);job['finish_index']+=1
        self._mode_event('finish_stage_completed',finish_stage=job['finish_index']-1,stage_receipt=record)
        if job['finish_index']<(4 if binding['kind']=='dagger_reset_v1'else 1):
            self._start_finish_stage(feedback);return True
        output=self._normal_output()
        hold=prepare_measured_hold(feedback,self.sdk_limits,now=now,
            brake_end_s=runner.started+runner.execution_duration_s,stationary_window=runner.stationary,
            held_jaws_sdk=[output['q_sdk14'][6],output['q_sdk14'][13]],source_sha256=record['reference_id'])
        self.target_interpreter.revoke();self._bind_normal(hold,now);self._queue_native(now,feedback)
        job.update(stage='HOLD_PENDING',hold_target=hold.initial.tolist(),
            hold_first_s=self.target_interpreter.last_packet['host_times_s'][0])
        self._mode_event('finish_arrival_verified',home_completed=True,
            jaw_open_completed=binding['kind']=='dagger_reset_v1',stages=job['finish_records'],
            scope='measured_arrival_and_stationary_not_collision_clearance')
        return True

    def _mode_event(self, stage, **values):
        row=self.mode_receipts[self.normal_job['request']['request_id']]
        row.update(stage=stage,status='running',command_epoch=self.owner.epoch,
            current_reference_id=self.target_interpreter.reference.metadata['reference_id']if self.target_interpreter else None,
            parent_reference_id=self.normal_job.get('source_reference_id'),
            committed_prefix_end_source_s=self.normal_job.get('boundary_s'),**values)
        self._emit('mode_transition',deepcopy(row))

    def _normal_output(self):
        value=self.owner.backend.read_execution_output()
        for key in('q_sdk14','dq_sdk14'):vector14(value[key])
        now=self.owner.control_time()
        if any(not 0<=now-row['sample_monotonic_ns']/1e9<=.1 for row in value['observations'].values()):
            raise ValueError('normal mode needs fresh output evidence on the Owner clock')
        return value

    def _normal_epoch(self):
        self.owner.invalidate(normal_transition=True)
        self.owner.active_epoch=self._epoch=self.owner.epoch
        self.owner.review_press=self.owner.pedal_snapshot()[0]
        self.status='RUNNING'

    def _bind_normal(self, reference, origin, *, prefix=None):
        self.prepared_reference=reference;self.native_start=origin
        self.target_interpreter=self.owner.make_target_interpreter(reference,'both',origin,integral=False)
        if prefix is not None:self.target_interpreter.preserved_prefix=deepcopy(prefix)
        self.native_next_upload=0.;self.native_queued_until=None
        self.execution_target,self.last_device_target=self._native_target(self.owner.control_time())

    def _normal_gain_target(self, output, teach):
        from .target_interpreter import gains
        import numpy as np
        target=deepcopy(output['gains'])
        for side,row in target.items():
            kp,kd=gains('normal_X5',side)
            row['kp']=[0.]*6 if teach else kp
            row['kd']=(np.asarray(kd)*(self.normal_config['teach_kd_scale']if teach else 1.)).tolist()
            if row['gripper_kp']<=0:raise ValueError('normal HOLD requires positive jaw gains before TEACH')
            if teach:row['gripper_kp']=row['gripper_kd']=0.
        return target

    def start_mode(self, request, feedback):
        """Owner tick only. Request handlers never write SDK or wait for writers."""
        job=self.normal_job
        if not job or job['request']!=request:raise ValueError('normal mode queued identity changed')
        output=self._normal_output();now=self.owner.control_time()
        if self._guard_decision('G24',dict(brake_duration_s=self.normal_config['brake_duration_s'],
                gain_transition_s=self.normal_config['gain_transition_s']),feedback['sample_time_s'],now,'normal_mode_profile')['verdict']=='reject':
            raise ValueError('G24: normal transition differs from the bound profile')
        previous=self.target_interpreter
        if previous is None:raise ValueError('normal modes require an admitted shared reference')
        if previous.integrals or previous.feedforward is not None:raise ValueError('normal modes require the frozen zero-added-bias profile')
        self.loop.stop('normal mode transition');self.outstanding=None
        job.update(started_s=now,source_epoch=self.owner.epoch,source_reference_id=previous.reference.metadata['reference_id'])
        if request['mode']!='MODEL'and self.normal_segment and self.normal_segment['status']in('executing','verifying_endpoint'):
            self.normal_segment.update(status='interrupted',interrupted_source_s=now,
                executed_s=max(0.,min(self.normal_segment['execution_duration_s'],now-self.normal_segment['start_source_s'])))
        if request['mode']=='MODEL':
            if self.mode!='HOLD':raise ValueError('MODEL requires a completed explicit HOLD')
            reference,origin=previous.reference,previous.start_s
            self._normal_epoch();self._bind_normal(reference,origin)
            self.prepared_reference=None;self.preloaded_prediction=None;self.bounded_live=False
            self.normal_supply_exhausted=False
            if self.normal_planned:
                self.bounded_result=self.bounded_request=self.normal_segment=None
                self.normal_observation_floor=now
            self.chunks_exhausted=False;self.started=now;self.execution_duration_s=self.config['duration_s']
            self.initial_device14=device_state(output['q_sdk14'])
            from arx_env.joint_head_jaw_mapping import command_gap_mm
            for side,i in(('left',6),('right',13)):self.initial_device14[i]=command_gap_mm(side,output['q_sdk14'][i])
            self.loop.start(epoch=self._epoch,source_time=now,episode_id=self.execution_instance_id)
            job.update(stage='RESUMING',observation_floor=now)
            self._queue_native(now,feedback)
            job['resume_hold_first_s']=self.target_interpreter.last_packet['host_times_s'][0]
            self._mode_event('awaiting_new_observation',mode='HOLD')
            return
        if self.mode=='HOLD':
            if request['mode']=='HOME':
                if set(feedback.get('native_home',{}))!={'left','right'}:
                    raise ValueError('connected SDK has no qualified native Home capability')
                self._normal_epoch()
                receipt=self.owner.backend.start_native_home('left')
                if receipt.get('accepted')is not True:raise ValueError('native Home start not confirmed')
                job.update(stage='NATIVE_HOME',side='left',native_receipts=[receipt])
                self.stationary.reset()
                self._mode_event('native_home_started',mode='HOME',side='left',native_receipt=receipt)
                self.mode='HOME'
                return
            reference,origin=previous.reference,previous.start_s
            self._normal_epoch();self._bind_normal(reference,origin)
            job.update(stage='GAINS',gain_started_s=now,initial_gains=output['gains'],
                target_gains=self._normal_gain_target(output,request['mode']=='TEACH'),output=output)
            self._mode_event('gain_transition',mode='HOLD')
            return
        if self.mode=='TEACH':
            # Manual feedback may lie inside the approved observation margin
            # while remaining outside legal command limits. Use the existing
            # measured-position SDK stop before binding a normal reference.
            self._normal_epoch()
            # TEACH releases the jaws too. Anchor the newly measured opening
            # before begin restores normal position gains.
            self.owner.backend.set_damping_hold(manual=False)
            stopped=self.owner.control_time()
            job.update(stage='MANUAL_HOLD',hold_return_source_s=stopped)
            self.stationary.reset();self.mode='BRAKE'
            self._mode_event('manual_hold_returned',mode='BRAKE',hold_return_source_s=stopped)
            return
        # Compile against a frozen last-submitted endpoint off the Owner thread.
        packet=deepcopy(previous.last_packet)
        if packet is None:raise ValueError('normal brake needs the committed native prefix')
        boundary=packet['host_times_s'][-1];origin=previous.start_s;reference=previous.reference
        if boundary<=now+.03:raise ValueError('insufficient queued lead for normal brake preparation')
        result=queue.Queue(maxsize=1)
        stationary_tail=(reference.metadata.get('kind')=='inference_initial_hold'or boundary-origin>=reference.duration_s)
        def compile_brake():
            try:
                from .motion_reference import prepare_brake_after,prepare_brake
                options=dict(duration_s=self.normal_config['brake_duration_s'],velocity_limit=self.normal_config['velocity_limit'],
                    acceleration_limit=self.normal_config['acceleration_limit'])
                if stationary_tail:
                    q,v,a=reference.sample([boundary-origin])
                    value=prepare_brake(q[0],v[0],a[0],self.sdk_limits,source_sha256=reference.metadata['reference_id'],**options)
                else:value=prepare_brake_after(reference,boundary-origin,self.sdk_limits,**options)
                result.put((value,None))
            except Exception as error:result.put((None,str(error)))
        job.update(stage='PREPARING',boundary_s=boundary,origin=boundary if stationary_tail else origin,prefix=packet,result=result)
        threading.Thread(target=compile_brake,name='normal-brake-prepare',daemon=True).start()
        self._mode_event('preparing_brake',mode='MODEL')

    def _complete_mode(self, mode, feedback, output):
        job=self.normal_job;request=job['request'];self.mode=mode
        row=self.mode_receipts[request['request_id']]
        row.update(status='completed',stage='mode_completed',completed=True,mode=mode,
            command_epoch=self.owner.epoch,completed_source_s=feedback['sample_time_s'],
            feedback=deepcopy(feedback),native_output=deepcopy(output),
            reference_id=self.target_interpreter.reference.metadata['reference_id'],
            sdk_window_admission=deepcopy(self.target_interpreter.last_receipt),
            scope='SDK_return_native_output_and_fresh_feedback_not_motor_ACK')
        if row.get('op')=='normal_finish':
            self.finish_completed=True;row.update(release_ready=True,sdk_released=False)
        self._emit('mode_transition',deepcopy(row));self.normal_job=None
        if request['target']in('END','DISCARD'):
            self.normal_ended=True
            self.status='READY';self.reason='normal '+request['target'].lower()+' completed in HOLD; not released'
            # D owns actual discard; keep the existing recorder terminal enum.
            self._finish_data('completed'if request['target']=='END'else'stopped',self.reason)

    def _install_normal_segment(self,reference,now,feedback):
        steps=self.config['execute_steps'];policy=self.config['reference_planning']
        self._check_model_reference(reference)
        if (reference.metadata.get('planning_policy')!=policy
                or not math.isclose(reference.metadata.get('source_duration_s',-1),steps/30.,abs_tol=1e-9)):
            raise ValueError('normal complete-segment step count, source duration or planning policy changed')
        origin=self.owner.control_time()+.02
        if not self.normal_single_segment and self._guard_decision('G23',dict(start_time=self.started,end_time=origin+reference.duration_s),
                now,now,reference.metadata['reference_id'])['verdict']=='reject':
            self.prepared_reference=self.target_interpreter.reference
            self._pause_for_supply('complete next segment exceeds remaining episode budget')
            return False
        index=1 if self.normal_segment is None else self.normal_segment['index']+1
        self._bind_normal(reference,origin);self._queue_native(self.owner.control_time(),feedback)
        self._accepted_chunk_source_time=self.bounded_request['observation']['source_time']
        self.normal_segment=dict(index=index,request_id=self.bounded_request['request_id'],
            command_epoch=self.owner.epoch,source_clock_id=self.owner.execution_clock_id,
            reference_id=reference.metadata['reference_id'],source_duration_s=steps/30.,source_steps=steps,
            execution_duration_s=reference.duration_s,start_source_s=origin,end_source_s=origin+reference.duration_s,
            observation_source_s=self._accepted_chunk_source_time,status='executing',complete_source_steps=0)
        self._emit('normal_model_segment_started',deepcopy(self.normal_segment))
        return True

    def _tick_normal_segment(self,now,feedback):
        """Synchronous complete chunks keep MODEL/Raw open until intent or budget."""
        import numpy as np
        self.loop.last_source_time=now
        self._queue_native(now,feedback)
        current=self.normal_segment
        if current is None:raise ValueError('normal MODEL has no admitted complete segment')
        end=current['end_source_s']
        if current['status']=='completed':
            if self._guard_decision('G23',dict(start_time=self.started,end_time=now+.02+self.config['execute_steps']/30.),
                    now,now,'next_segment_budget')['verdict']=='reject':
                self._pause_for_supply('normal MODEL episode budget exhausted');return True
            observation=(self.bounded_request or{}).get('observation')
            # At a proven zero-speed endpoint, wait separately for an input
            # and its result, using the same freshness clock as MODEL entry.
            basis=end if observation is None else observation['source_time']
            if self._guard_decision('G07'if observation is None else'G08',dict(current_time=now,basis_time=basis),
                    basis,now,'segment_supply')['verdict']=='reject':
                self._pause_for_supply('fresh observation supply timeout at zero-speed endpoint'if observation is None
                    else 'model preparation exceeded observation freshness deadline');return True
            try:ready=self._prepare_bounded_prediction(now,feedback)
            except Exception as error:
                self._pause_for_supply(str(error));return True
            if ready:self._install_normal_segment(self.prepared_reference,now,feedback)
            return True
        if now<end or feedback['sample_time_s']<end:return True
        if self.native_queued_until is None or self.native_queued_until<end:
            raise ValueError('normal model window did not cover the complete endpoint')
        if current['status']=='executing':
            post_end=[stamp for stamp in self.target_interpreter.last_packet['host_times_s']if stamp>end]
            if not post_end:return True
            current.update(status='verifying_endpoint',endpoint_verification_after_s=post_end[0]);return True
        output=self._normal_output()
        if any(row['sample_monotonic_ns']/1e9<current['endpoint_verification_after_s']for row in output['observations'].values()):return True
        reference=self.target_interpreter.reference;target,velocity,_=reference.sample([reference.duration_s])
        if (not np.allclose(output['q_sdk14'],target[0],rtol=0,atol=1e-12)
                or not np.allclose(output['dq_sdk14'],velocity[0],rtol=0,atol=1e-12)):
            raise ValueError('normal complete-segment native endpoint mismatch')
        current.update(status='completed',complete_source_steps=current['source_steps'],completed_source_s=now,native_endpoint=output,firmware_ack=False,
            measured_endpoint=dict(sample_time_s=feedback['sample_time_s'],sample_wall_ns=feedback.get('sample_wall_ns'),
                qpos=list(feedback['qpos']),qvel=list(feedback['qvel']),
                target_error_sdk14=(np.asarray(feedback['qpos'])-target[0]).tolist(),
                stationary=self.owner.stationary,actual_arrival_qualified=False))
        self._emit('normal_model_segment_completed',deepcopy(current))
        if self.normal_single_segment:
            self._pause_for_supply('explicit single-segment acceptance budget finished; normal HOLD requires original Raw gate')
        else:
            self.normal_observation_floor=now
            self.bounded_result=self.bounded_request=None
        return True

    def _normal_tick(self, now, feedback):
        """Return true when normal-mode handling consumes this control tick."""
        import numpy as np
        job=self.normal_job
        if job is None:
            # A completed HOLD has already confirmed its fixed zero-velocity
            # native output. The SDK retains that endpoint; recurring identical
            # windows only add a future-admission deadline while standing still.
            # Feedback, epoch and device checks still run before this branch.
            if self.mode in ('HOLD','TEACH'):return True
            if getattr(self,'normal_supply_exhausted',False):
                self._queue_native(now,feedback);return True
            if self.normal_planned and self.mode=='MODEL':
                return self._tick_normal_segment(now,feedback)
            if (self.target_interpreter is not None and self.started is not None
                    and self.target_interpreter.reference.metadata.get('kind')!='inference_initial_hold'
                    and now>self.native_start+self.target_interpreter.reference.duration_s+.5):
                self._queue_native(now,feedback)
                self._pause_for_supply('reference supply exhausted at planned zero-speed endpoint')
                return True
            return False
        stage=job['stage']
        if stage in('QUEUED','FINISH_QUEUED'):return True
        if stage=='MANUAL_HOLD':
            if feedback['sample_time_s']<=job['hold_return_source_s']:return True
            if not self.stationary.update(feedback['sample_time_s'],feedback['qpos'],feedback['qvel']):return True
            self.owner.begin('both',feedback['qpos'])
            command=self.owner.last_begin_command
            if not command or command['epoch']!=self._epoch:raise ValueError('manual HOLD needs current begin evidence')
            from .target_interpreter import prepare_samples
            held=command['command_sdk14']
            hold=prepare_samples([0.,self.loop.period],[held,held],self.sdk_limits,
                source_sha256=self.plan.sha256,kind='normal_hold')
            self._bind_normal(hold,self.owner.control_time());self._queue_native(self.owner.control_time(),feedback)
            job.update(stage='HOLD_PENDING',hold_target=held,
                hold_first_s=self.target_interpreter.last_packet['host_times_s'][0])
            self._mode_event('manual_hold_bound',mode='BRAKE',hold_target=held,
                stationary_check=deepcopy(self.stationary.last_diagnostic))
            return True
        if stage=='NATIVE_HOME':
            if feedback['sample_time_s']<=job['native_receipts'][-1]['started_source_s']:return True
            status=feedback.get('native_home',{}).get(job['side'])
            if status=='running':return True
            if status!='completed':raise ValueError('native Home '+str(status))
            if job['side']=='left':
                receipt=self.owner.backend.start_native_home('right')
                if receipt.get('accepted')is not True:raise ValueError('right native Home start not confirmed')
                job['native_receipts'].append(receipt);job['side']='right'
                self._mode_event('native_home_started',mode='HOME',side='right',native_receipt=receipt)
                return True
            output=self._normal_output()
            if not np.allclose(np.asarray(output['q_sdk14'])[[*range(6),*range(7,13)]],0.,atol=1e-6):return True
            if not self.stationary.update(feedback['sample_time_s'],feedback['qpos'],feedback['qvel']):return True
            from .target_interpreter import prepare_samples
            hold=prepare_samples([0.,self.loop.period],[output['q_sdk14'],output['q_sdk14']],self.sdk_limits,
                source_sha256=self.plan.sha256,kind='normal_hold')
            self._bind_normal(hold,now);self._queue_native(now,feedback)
            job.update(stage='HOLD_PENDING',hold_target=output['q_sdk14'],
                hold_first_s=self.target_interpreter.last_packet['host_times_s'][0])
            self._mode_event('native_home_returned',mode='HOME',native_receipts=job['native_receipts'],
                measured_error_sdk14=(np.asarray(feedback['qpos'])-np.asarray(output['q_sdk14'])).tolist(),
                stationary_check=deepcopy(self.stationary.last_diagnostic))
            return True
        if stage=='FINISH':return self._tick_finish(now,feedback)
        if stage=='RESUMING':
            if self.normal_planned:
                self._queue_native(now,feedback)
                observation=(self.bounded_request or{}).get('observation')
                basis=job['started_s']if observation is None else observation['source_time']
                budget=self.config['reference_supply_timeout_s']if observation is None else self.config['maximum_observation_age_s']
                deadline=basis+budget
                self.mode_receipts[job['request']['request_id']].update(supply_deadline_source_s=deadline,
                    supply_deadline_basis='waiting_for_fresh_observation'if observation is None else'captured_observation',
                    observation_source_s=None if observation is None else observation['source_time'])
                if self._guard_decision('G07'if observation is None else'G08',dict(current_time=now,basis_time=basis),
                        basis,now,'model_entry_supply')['verdict']=='reject':
                    self._resume_supply_failed('fresh model supply timeout');return True
                try:ready=self._prepare_bounded_prediction(now,feedback)
                except Exception as error:
                    self._resume_supply_failed(str(error));return True
                if ready:
                    if self._install_normal_segment(self.prepared_reference,now,feedback):
                        job.update(stage='MODEL_ADMITTED',first_model_source_s=self.target_interpreter.last_packet['host_times_s'][0])
                        self._mode_event('model_admitted',mode='HOLD')
                return True
            if now>job['started_s']+self.config.get('maximum_observation_age_s',2.):
                self._resume_supply_failed('fresh model supply timeout')
                return True
            return False
        if stage=='RESUME_HOLD_CHECK':
            self._queue_native(now,feedback)
            if now>job['hold_check_started_s']+3.:raise ValueError('retained HOLD could not be confirmed after model supply failure')
            output=self._normal_output();target=self.target_interpreter.reference.sample([now-self.native_start])[0][0]
            if (self.owner.stationary and feedback['sample_time_s']>=job['resume_hold_first_s']
                    and min(row['sample_monotonic_ns']/1e9 for row in output['observations'].values())>=job['resume_hold_first_s']
                    and np.allclose(output['q_sdk14'],target,rtol=2*np.finfo(np.float32).eps,atol=2*np.finfo(np.float32).eps)):
                self.mode='HOLD'
                evidence=dict(confirmed=True,command_epoch=self.owner.epoch,source_clock_id=self.owner.execution_clock_id,
                    confirmed_source_s=feedback['sample_time_s'],reference_id=self.target_interpreter.reference.metadata['reference_id'],
                    target_sdk14=target.tolist(),sdk_window_admission=deepcopy(self.target_interpreter.last_receipt),
                    native_output=output,feedback=deepcopy(feedback),stationary_check=deepcopy(self.owner.stationary_window.last_diagnostic),
                    scope='retained_reference_and_native_output_with_fresh_stationary_feedback_not_motor_ACK')
                self.reject_mode(job['request']['request_id'],job['supply_error'],error_class='prediction_supply',
                    failure_phase='before_model_admission',retained_hold=evidence)
                self.normal_job=None
            return True
        if stage=='MODEL_ADMITTED':
            self._queue_native(now,feedback)
            if feedback['sample_time_s']>=job['first_model_source_s']:
                output=self._normal_output()
                if min(x['sample_monotonic_ns']/1e9 for x in output['observations'].values())>=job['first_model_source_s']:
                    self._complete_mode('MODEL',feedback,output)
            return bool(self.normal_planned)
        if stage=='PREPARING':
            if now+.03>=job['boundary_s']:raise ValueError('normal brake preparation missed committed prefix; fault Stop required')
            try:reference,error=job['result'].get_nowait()
            except queue.Empty:return True
            if error:raise ValueError(error)
            if self.owner.epoch!=job['source_epoch']:raise ValueError('normal brake preparation epoch revoked')
            self._normal_epoch();self._bind_normal(reference,job['origin'],prefix=job['prefix'])
            job.update(stage='BRAKE',end_s=job['origin']+reference.duration_s)
            self.stationary.reset();self._mode_event('brake_admitted',mode='MODEL')
            stage='BRAKE'
        if stage=='BRAKE':
            self._queue_native(now,feedback)
            if now>=job['boundary_s']and not job.get('brake_started'):
                job['brake_started']=True;self.mode='BRAKE'
                self._mode_event('brake_start',mode='BRAKE',brake_start_source_s=job['boundary_s'],
                    scope='admitted_native_timeline_due_not_motor_ACK')
            if 'target_gains'in job:self._ramp_normal_gains(job,now)
            if feedback['sample_time_s']>=job['end_s']:
                if now>job['end_s']+3.:raise ValueError('normal brake settling exceeded existing 3s budget')
                if self.stationary.update(feedback['sample_time_s'],feedback['qpos'],feedback['qvel']):
                    from .target_interpreter import prepare_measured_hold
                    output=self._normal_output()
                    hold=prepare_measured_hold(feedback,self.sdk_limits,now=now,brake_end_s=job['end_s'],
                        stationary_window=self.stationary,held_jaws_sdk=[output['q_sdk14'][6],output['q_sdk14'][13]],
                        source_sha256=self.prepared_reference.metadata['reference_id'])
                    self.target_interpreter.revoke();self._bind_normal(hold,now)
                    self._queue_native(now,feedback)
                    job.update(stage='HOLD_PENDING',hold_target=hold.initial.tolist(),
                        hold_first_s=self.target_interpreter.last_packet['host_times_s'][0],hold_output=output)
                    self._mode_event('brake_completed',brake_completed_source_s=feedback['sample_time_s'],hold_target=job['hold_target'],
                        stationary_check=deepcopy(self.stationary.last_diagnostic),
                        scope='planned_zero_v_a_and_existing_position_span; measured_velocity_diagnostic_not_motor_ACK')
            return True
        if stage=='HOLD_PENDING':
            self._queue_native(now,feedback)
            if now>job['hold_first_s']+3.:raise ValueError('normal HOLD output confirmation timed out')
            output=self._normal_output()
            if (feedback['sample_time_s']>=job['hold_first_s']and
                    min(x['sample_monotonic_ns']/1e9 for x in output['observations'].values())>=job['hold_first_s']and
                    np.allclose(output['q_sdk14'],job['hold_target'],rtol=2*np.finfo(np.float32).eps,atol=2*np.finfo(np.float32).eps)):
                self.mode='HOLD';self._mode_event('hold_locked',mode='HOLD',hold_locked_source_s=feedback['sample_time_s'],native_output=output)
                if job['request']['mode']=='TEACH':
                    job.update(stage='GAINS',gain_started_s=now,initial_gains=output['gains'],
                        target_gains=self._normal_gain_target(output,True))
                else:self._complete_mode('HOLD',feedback,output)
            return True
        if stage=='GAINS':
            self._queue_native(now,feedback)
            done=self._ramp_normal_gains(job,now)
            gain_end=job['gain_started_s']+self.normal_config['gain_transition_s']
            if now>gain_end+3.:raise ValueError('normal mode gain/output confirmation timed out')
            if done:
                output=self._normal_output()
                if (feedback['sample_time_s']>=gain_end and
                    min(row['sample_monotonic_ns']/1e9 for row in output['observations'].values())>=gain_end and
                    all(np.allclose(output['gains'][s][k],job['target_gains'][s][k],rtol=2*np.finfo(np.float32).eps,atol=2*np.finfo(np.float32).eps)
                       for s in('left','right')for k in('kp','kd','gripper_kp','gripper_kd'))):
                    self._complete_mode(job['request']['mode'],feedback,output)
            return True
        raise ValueError('unknown normal transition stage')

    def _pause_for_supply(self,reason):
        self.loop.stop(reason)
        if self.normal_job and self.normal_job['stage']=='RESUMING':
            self._resume_supply_failed(reason)
            return
        self.normal_supply_exhausted=True
        if self.normal_config['recording_required']:
            self._emit('normal_pause_required',dict(reason=reason,mode=self.mode,recording_gate_required=True,
                at_zero_speed_endpoint=self.owner.control_time()>=self.native_start+self.target_interpreter.reference.duration_s))
        else:
            self.submit_mode(dict(action='normal_mode',session_id=self.mode_session_id,
                runtime_nonce=self.owner.runtime_nonce,epoch=self.owner.epoch,request_id=secrets.token_hex(16),mode='HOLD',target='HOLD'))

    def _resume_supply_failed(self,reason):
        self.loop.stop(reason);self.prepared_reference=self.target_interpreter.reference
        self.normal_job.update(stage='RESUME_HOLD_CHECK',supply_error=str(reason),hold_check_started_s=self.owner.control_time())
        self._mode_event('model_supply_failed_retaining_hold',mode='HOLD')

    def _ramp_normal_gains(self,job,now):
        alpha=max(0.,min(1.,(now-job['gain_started_s'])/self.normal_config['gain_transition_s']))
        rows=deepcopy(job['initial_gains'])
        for side,row in rows.items():
            for key in('kp','kd'):row[key]=[a+alpha*(b-a)for a,b in zip(row[key],job['target_gains'][side][key])]
            for key in('gripper_kp','gripper_kd'):row[key]+=alpha*(job['target_gains'][side][key]-row[key])
        self.owner.command_gate();self.owner.backend.set_execution_gains(rows);self.owner.command_gate()
        return alpha>=1.

    def cancel(self, reason='operator cancelled'):
        self.revoke(reason)
        return self.status

    def start(self, now, feedback, **_):
        from arx_env.joint_head_real_motion import _state
        if self.status != 'ARMED' or self.plan.sha256 in self.owner.consumed:
            raise ValueError('inference review already consumed')
        q, _, sample, _ = _state(feedback)
        check_gripper_policy(self.owner,self.config,feedback)
        if not self.owner.feedback_healthy or not 0 <= now-sample <= .1:
            raise ValueError('fresh healthy feedback required for inference')
        self.owner.consumed.add(self.plan.sha256)
        self.owner.command_gate()
        self.previous = now
        self.execution_target = sdk_target(self.initial_device14)
        self.status = 'VERIFYING'
        self.preparation_deadline = time.monotonic()+1.
        if self.record:
            self._record_thread = threading.Thread(target=self._data_worker, name='policy-record', daemon=True)
            self._record_thread.start()
            self._record('prepare',dict(configuration=deepcopy(self.config),execution_instance_id=self.execution_instance_id,
                execution_owner=dict(owner_pid=os.getpid(),command_epoch=self._epoch,runtime_nonce=self.owner.runtime_nonce),
                clock=deepcopy(getattr(self.owner,'feedback_clock',None)),
                control_clock_id=self.owner.execution_clock_id,
                preparation_deadline_monotonic=self.preparation_deadline))

    def _verify_initial_hold(self):
        self.owner.command_gate()
        command=self.owner.last_begin_command
        if (not command or command['epoch']!=self._epoch or self.owner.epoch!=self._epoch
                or command['runtime_nonce']!=self.owner.runtime_nonce):
            raise ValueError('initial hold identity changed; no model admission')
        output=self._normal_output()
        expected=self.initialization['begin_command']['command_sdk14']
        if (any(row['sample_monotonic_ns']/1e9<self.initialization_source_s for row in output['observations'].values())
                or any(output['gains'][side]['gripper_kp']<=0 for side in('left','right'))
                or any(abs(output['q_sdk14'][i]-expected[i])>1e-12
                       or abs(command['command_sdk14'][i]-expected[i])>1e-12 for i in(6,13))):
            self._emit('initial_hold_mismatch',dict(expected_jaws_sdk=[expected[6],expected[13]],
                output=output,begin_command=command,command_epoch=self.owner.epoch))
            raise ValueError('strict initialized jaw hold changed; no reprepare or model admission')
        if self.config.get('reference_planning')is not None and (
                any(abs(output['q_sdk14'][i]-expected[i])>1e-12 for i in range(14))
                or any(abs(output['dq_sdk14'][i])>1e-12 for i in(*range(6),*range(7,13)))):
            raise ValueError('synchronous start is not a fixed native q/dq hold; a moving boundary cannot be zeroed')
        return output

    def _initialize_hold(self,feedback):
        """One existing lifecycle begin, before choosing any model observation."""
        self.initialization=dict(stage='begin_requested',requested_source_s=self.owner.control_time(),
            command_epoch=self._epoch,model_windows=0,firmware_ack=False)
        self._emit('initial_hold_begin',deepcopy(self.initialization))
        self.owner.begin('both',feedback['qpos'])
        command=deepcopy(self.owner.last_begin_command)
        if not command or command['epoch']!=self._epoch:raise ValueError('initial hold needs current begin evidence')
        from arx_env.joint_head_jaw_mapping import command_gap_mm
        held=command['command_sdk14'][:]
        for side,i in(('left',6),('right',13)):held[i]=command_gap_mm(side,held[i])
        # The loop adapter closes over this same list; bind once, before a query.
        self.initial_device14[:]=held;self.last_device_target=held[:]
        self.execution_target=command['command_sdk14'][:]
        self.initialization_source_s=self.owner.control_time()
        self.initialization.update(stage='begin_returned',begin_command=command,
            returned_source_s=self.initialization_source_s)
        self._emit('initial_hold_returned',deepcopy(self.initialization))
        self.stationary.reset();self.verify_started=self.initialization_source_s

    def _begin(self, now, feedback):
        if self.initialize_online_hold:
            self._verify_initial_hold()  # Never issue a second begin after preparing the model path.
        elif getattr(self.owner,'preserve_begin_jaws',False)and self.previous_native_hold is not None:
            from .target_interpreter import gains
            import numpy as np
            previous=self.previous_native_hold;self.owner.command_gate();output=self._normal_output()
            if (previous.cancelled or previous.epoch!=self.owner.epoch
                    or any(abs(output['dq_sdk14'][i])>1e-12 for i in(*range(6),*range(7,13)))
                    or not np.allclose(output['q_sdk14'],sdk_target(self.initial_device14),rtol=0,atol=1e-12)
                    or any(not np.allclose(output['gains'][side][key],gains('normal_X5',side)[index])
                           for side in('left','right')for index,key in enumerate(('kp','kd')))):
                raise ValueError('continued native hold identity/output/gains changed; reprepare required')
            self.owner.last_begin_command=dict(command_sdk14=output['q_sdk14'],source_outputs={'continuation':output},
                epoch=self.owner.epoch,runtime_nonce=self.owner.runtime_nonce,firmware_ack=False)
            previous.revoke();self.owner.control.auto_aborted=False;self.owner.control.state=self.owner.states.READY
            self._emit('native_hold_reused',dict(parent_reference_id=previous.reference.metadata['reference_id'],native_output=output))
        else:self.owner.begin('both', feedback['qpos'])
        if getattr(self.owner,'preserve_begin_jaws',False):
            command=self.owner.last_begin_command
            expected=sdk_target(self.initial_device14)
            if (not command or command['epoch']!=self._epoch
                    or any(abs(command['command_sdk14'][i]-expected[i])>1e-12 for i in(6,13))):
                raise ValueError('strict begin jaw command changed after reference preparation; reprepare required')
        now = self.owner.control_time()
        if self.bounded_live and self._guard_decision('G08', now-self.bounded_request['observation']['source_time'],
                self.bounded_request['observation']['source_time'], now, 'model_observation')['verdict']=='reject':
            raise ValueError('bounded live prediction observation expired before motion; no output')
        self.started = self.previous = now
        self.status = 'RUNNING'
        if self.normal_config and self.config.get('normal_initial_mode')=='HOLD':
            from .target_interpreter import prepare_samples
            command=self.owner.last_begin_command
            if not command or command['epoch']!=self._epoch:raise ValueError('initial HOLD needs current strict begin output')
            target=command['command_sdk14']
            reference=prepare_samples([0.,self.loop.period],[target,target],self.sdk_limits,
                source_sha256=self.plan.sha256,kind='inference_initial_hold')
            self._bind_normal(reference,now);self._queue_native(now,feedback)
            key='initialize-'+self.execution_instance_id
            request=dict(session_id=self.mode_session_id,runtime_nonce=self.owner.runtime_nonce,
                request_id=key,epoch=self._epoch,mode='HOLD',target='HOLD')
            self.mode_receipts[key]=dict(request,op='normal_initialize',accepted=True,completed=False,status='running',
                command_epoch=self._epoch,source_clock_id=self.owner.execution_clock_id,firmware_ack=False)
            self._mode_payloads[key]=digest(request)
            self.normal_job=dict(request=request,stage='HOLD_PENDING',hold_target=list(target),
                hold_first_s=self.target_interpreter.last_packet['host_times_s'][0])
            self.mode='INITIALIZING'
            self._mode_event('initial_hold_admitted',mode='INITIALIZING')
            return
        self.mode='MODEL'
        self.loop.start(epoch=self._epoch, source_time=now, episode_id=self.execution_instance_id)
        self._emit('configuration', dict(configuration=deepcopy(self.config), initial_device14=self.initial_device14,
            sdk_joint_limits=self.sdk_limits, source_clock=deepcopy(getattr(self.owner, 'feedback_clock', None)),
            source_start_s=now, source_end_s=now+self.execution_duration_s, runtime_nonce=self.owner.runtime_nonce,
            owner_pid=os.getpid(), command_epoch=self._epoch))
        if self.prepared_reference is not None:
            if self.finite_chunk_count:
                self._install_finite_chunk(now,feedback,self.prepared_reference,self.bounded_request)
                return
            self.native_start=now+self.native_initial_hold
            if self.bounded_live:
                self.native_start=now+.02;self.started=self.native_start
                self.loop.started=self.native_start
            if self.feedforward_profile is not None:
                from .motion_reference import ReferenceFeedforward
                self.reference_feedforward=ReferenceFeedforward(self.feedforward_profile,self.native_start,self.prepared_reference.duration_s,now)
                self._record('feedforward_profile',dict(sha256=self.feedforward_profile['sha256'],
                    phase=self.feedforward_profile['phase'].tolist(),torque_bias_sdk6=self.feedforward_profile['bias'].tolist(),provenance=self.feedforward_profile['provenance']))
            self._emit('native_reference_started',dict(reference=deepcopy(self.prepared_reference.metadata),
                trajectory_start_source_s=self.native_start,source_material_duration_s=self.config['duration_s'],
                execution_duration_s=self.execution_duration_s,initial_hold_s=self.native_initial_hold,terminal_hold_s=self.native_terminal_hold))
            self._queue_native(now,feedback)
            return
        if self.preloaded_prediction is not None:
            # begin() already held the measured pose. A saved reference needs
            # no extra 33ms initial command or async query inside its 2s clock.
            self.loop.prime_reference(self.preloaded_prediction,self.initial_device14)
            command=self.loop.tick(now,epoch=self._epoch)
            if command is None:raise ValueError('preloaded reference did not produce its first command')
            self._submit(command,feedback)
        else:
            self._submit(dict(command_id=secrets.token_hex(16),kind='initial_hold',request_id=None,
                due_source_time=now,submitted_source_time=now,final_device14=list(self.initial_device14)), feedback)

    def _prepare(self, now, feedback):
        if not self.preparation_complete:
            try:
                ready = self.record is None or self.record_prepared is None or self.record_prepared() is True
            except Exception as exc:
                self._recording_failure(exc); ready = False
            if not ready and not self.recording_error and time.monotonic() < self.preparation_deadline:
                return False
            if not ready:
                self._recording_failure(self.recording_error or 'inference camera preparation exceeded 1 wall second')
            self.preparation_complete = True
            self.verify_started = now
            self.stationary.reset()
        if now-self.verify_started > 3.:
            raise ValueError('inference stationary verification deadline exceeded')
        qpos,qvel=list(feedback['qpos']),list(feedback['qvel'])
        if self.disabled_grippers:qpos[13]=qvel[13]=0.  # Excluded axis; original feedback remains in the recording.
        if not self.stationary.update(feedback['sample_time_s'],qpos,qvel):
            return False
        if self.initialize_online_hold:
            if self.initialization_source_s is None:
                self._initialize_hold(feedback);return False
            if self.initialization['stage']=='begin_returned':
                output=self._verify_initial_hold()
                self.initialization.update(stage='hold_confirmed',confirmed_source_s=self.owner.control_time(),output=output)
                self._emit('initial_hold_confirmed',deepcopy(self.initialization))
        if self.bounded_live and not self._prepare_bounded_prediction(now,feedback):return False
        self._begin(now,feedback)
        return True

    def _prepare_bounded_prediction(self,now,feedback):
        """Predict while holding; the requested half-second starts with real output."""
        if self.bounded_result is None:
            if self.bounded_worker is not None and self.bounded_worker.is_alive():return False
            observation=self.observe(now,feedback)
            if observation is None:return False
            if self.initialization_source_s is not None and observation['source_time']<=self.initialization_source_s:return False
            if self.finite_observation_floor is not None and observation['source_time']<=self.finite_observation_floor:return False
            if self.normal_planned and observation['source_time']<=self.normal_observation_floor:return False
            import numpy as np
            request=dict(request_id=secrets.token_hex(16),seed=self.config['seed'],task=self.config['task'],
                observation=observation,applied_history=np.empty((0,14),np.float32))
            if self.finite_chunk_count:self.finite_stage='preparing'
            self.bounded_request=deepcopy(request);self.bounded_result=queue.Queue(maxsize=1)
            pending=self.bounded_result;epoch=self._epoch
            def prepare():
                try:
                    if self.normal_planned and not self.normal_single_segment:
                        current=self.owner.control_time()
                        if self._guard_decision('G23',dict(start_time=self.started,end_time=current),current,current,
                                'bounded_model_query',phase='query')['verdict']=='reject':
                            raise ValueError('normal model episode budget exhausted before model query')
                    response=self.predict(request)
                    if self._epoch!=epoch or self.owner.epoch!=epoch or self.status in('STOPPED','FAULT'):
                        raise ValueError('model preparation epoch revoked')
                    pending.put((self._prepare_native_response(response,request),None))
                except Exception as error:pending.put((None,str(error)))
            self.bounded_worker=threading.Thread(target=prepare,name='bounded-live-policy',daemon=True)
            self.bounded_worker.start()
            self._emit('prediction_requested',dict(request_id=request['request_id'],task=request['task'],
                observation_source_time=observation['source_time'],frame_ids=observation.get('frame_ids'),
                state_device14=vector14(observation['device_state14']).tolist(),applied_history_model14=[]))
            return False
        try:response,error=self.bounded_result.get_nowait()
        except queue.Empty:return False
        if error:raise ValueError('bounded live prediction failed: '+error)
        if self.normal_planned:self.loop.predictions+=1
        now=max(now,self.owner.control_time())
        self._emit('bounded_prediction_result',dict(request_id=response['request_id'],
            preparation=response.get('_preparation_timing'),observation_age_s=now-self.bounded_request['observation']['source_time'],
            maximum_observation_age_s=self.config['maximum_observation_age_s'],actual_submission_count=self.actual_submission_count))
        now=max(now,self.owner.control_time())
        source_time = self.bounded_request['observation']['source_time']
        if self._guard_decision('G08', now-source_time, source_time, now, 'model_observation')['verdict']=='reject':
            raise ValueError('bounded live prediction observation expired before motion; no output')
        current=device_state(feedback['qpos']);observed=self.bounded_request['observation']['device_state14']
        from .motion_reference import JOINTS
        self._emit('prediction_motion_check',dict(request_id=response['request_id'],
            observation_source_s=self.bounded_request['observation']['source_time'],
            feedback_source_s=feedback['sample_time_s'],feedback_wall_ns=feedback.get('sample_wall_ns'),
            observed_device14=list(observed),current_device14=current,
            delta_device14=(__import__('numpy').asarray(current)-observed).tolist(),
            joint_limit_rad=self.config['prepare_drift_joint_rad'],jaw_limit_mm=self.config['prepare_drift_jaw_mm']))
        joint = max(JOINTS, key=lambda i: abs(current[i]-observed[i]))
        jaw = max((6, 13), key=lambda i: abs(current[i]-observed[i]))
        decisions = [self._guard_decision(rule, current[i]-observed[i], feedback['sample_time_s'], now, 'device14_'+str(i))
                     for rule, i in (('G11', joint), ('G12', jaw))]
        if any(decision['verdict']=='reject' for decision in decisions):
            raise ValueError('device moved while bounded prediction was prepared; no output')
        candidate=response['_prepared_reference']
        if self.finite_chunk_count and self.finite_completed:self.finite_candidate=response
        else:self.prepared_reference=candidate
        self.native_initial_hold=self.native_terminal_hold=0.
        complete=bool(self.finite_chunk_count or self.normal_planned)
        self.execution_duration_s=candidate.duration_s if complete else .5
        self._emit('finite_chunk_prepared'if self.finite_chunk_count else'bounded_prediction_prepared',dict(request_id=response['request_id'],task=self.config['task'],
            prediction_model14=__import__('numpy').asarray(response['action14']).tolist(),worker_audit=response.get('audit'),
            reference=deepcopy(candidate.metadata),source_first_action_offset_s=1/30.,
            execution_prefix_s=None if complete else .5,stop_at_prefix_end=not complete))
        return True

    def _finite_freeze_elapsed(self):
        if self.finite_current is not None and self.finite_stage=='executing':
            self.finite_partial_s=max(0.,min(self.finite_current['execution_duration_s'],
                self.owner.control_time()-self.finite_current['start_source_s']))

    def _install_finite_chunk(self,now,feedback,reference,request):
        import numpy as np
        from .motion_reference import _check_c2
        policy=self.config['reference_planning'];now=self.owner.control_time();origin=now+.02
        self._check_model_reference(reference)
        if (self.owner.epoch!=self._epoch or len(self.finite_completed)>=self.finite_chunk_count
                or reference.metadata.get('planning_policy')!=policy
                or reference.metadata['source_duration_s']!=.5
                or self._guard_decision('G08', origin-request['observation']['source_time'],
                    request['observation']['source_time'], now, 'scheduled_model_observation')['verdict']=='reject'):
            raise ValueError('finite complete-segment identity, duration or observation deadline changed')
        _check_c2(reference.coefficients,reference.breaks)
        start,velocity,acceleration=reference.sample([0.])
        if np.any(abs(velocity[0])>1e-9)or np.any(abs(acceleration[0])>1e-9):
            raise ValueError('synchronous segment requires its proven zero-derivative hold boundary')
        if self.target_interpreter is not None:
            prior,_,_=self.target_interpreter.reference.sample([self.target_interpreter.reference.duration_s])
            if not np.allclose(prior[0],start[0],rtol=0,atol=1e-12):raise ValueError('finite segment start differs from prior held endpoint')
        self.owner.command_gate()
        previous=self.target_interpreter
        self.prepared_reference=reference;self.native_start=origin;self.execution_duration_s=reference.duration_s
        self.target_interpreter=self.owner.make_target_interpreter(reference,'both',origin,previous=previous)
        self._queue_native(now,feedback)
        wait_start=(self.finite_completed[-1]['end_source_s']if self.finite_completed else
                    (self.initialization or{}).get('confirmed_source_s',now))
        self.finite_current=dict(index=len(self.finite_completed)+1,request_id=request['request_id'],
            reference_id=reference.metadata['reference_id'],observation_source_s=request['observation']['source_time'],
            source_duration_s=.5,source_steps=15,execution_duration_s=reference.duration_s,
            start_source_s=origin,end_source_s=origin+reference.duration_s,waiting_s=max(0.,origin-wait_start))
        self.finite_partial_s=0.;self.finite_stage='executing';self.finite_candidate=None
        self._accepted_chunk_source_time=request['observation']['source_time'];self.loop.predictions+=1
        self._emit('finite_chunk_started',deepcopy(self.finite_current))

    def _tick_finite_chunks(self,now,feedback):
        import numpy as np
        self._queue_native(now,feedback)  # Same interpreter keeps the zero-speed endpoint alive while waiting.
        if self.finite_stage in('executing','verifying_endpoint'):
            current=self.finite_current;end=current['end_source_s']
            if now<end or feedback['sample_time_s']<end:return self.status
            if self.native_queued_until is None or self.native_queued_until<end:
                raise ValueError('finite reference window did not cover the complete endpoint')
            if self.finite_stage=='executing':
                # Use a real queued post-end hold knot, not an invented arrival
                # tolerance or a model point count, as the output proof barrier.
                post_end=[stamp for stamp in self.target_interpreter.last_packet['host_times_s']if stamp>end]
                if not post_end:return self.status
                current['endpoint_verification_after_s']=post_end[0]
                self.finite_partial_s=current['execution_duration_s'];self.finite_stage='verifying_endpoint'
                return self.status
            output=self._normal_output()
            if any(row['sample_monotonic_ns']/1e9<current['endpoint_verification_after_s']for row in output['observations'].values()):return self.status
            target,velocity,_=self.prepared_reference.sample([self.prepared_reference.duration_s])
            if (not np.allclose(output['q_sdk14'],target[0],rtol=0,atol=1e-12)
                    or not np.allclose(output['dq_sdk14'],velocity[0],rtol=0,atol=1e-12)):
                raise ValueError('finite segment endpoint native output mismatch; no next query')
            completed=dict(current,completed_source_s=self.owner.control_time(),native_endpoint=output,
                complete_source_steps=15,firmware_ack=False)
            self.finite_completed.append(completed);self.finite_current=None;self.finite_partial_s=0.
            self._emit('finite_chunk_completed',deepcopy(completed))
            if len(self.finite_completed)==self.finite_chunk_count:
                self.finite_stage='completed'
                self._emit('finite_chunks_completed',dict(completed_count=len(self.finite_completed),
                    actual_model_execution_s=sum(row['execution_duration_s']for row in self.finite_completed),
                    source_model_duration_s=.5*self.finite_chunk_count,stop_confirmed=False))
                reason=f'{self.finite_chunk_count} complete synchronous model segments finished; independent Stop'
                self.revoke(reason);self.owner.stop(reason)
                return self.status
            self.finite_stage='waiting_observation';self.finite_observation_floor=self.owner.control_time()
            self.bounded_result=None;self.bounded_request=None
            return self.status
        if self.finite_stage in('waiting_observation','preparing'):
            observation=(self.bounded_request or{}).get('observation')
            basis=self.finite_completed[-1]['end_source_s']if observation is None else observation['source_time']
            if self._guard_decision('G07'if observation is None else'G08',dict(current_time=now,basis_time=basis),
                    basis,now,'finite_supply')['verdict']=='reject':
                reason=('fresh observation supply timeout at finite zero-speed endpoint'if observation is None
                    else 'finite model preparation exceeded observation freshness deadline')
                self.finite_stage='failed';self.revoke(reason);self.owner.stop(reason)
                return self.status
            if self._prepare_bounded_prediction(now,feedback):
                self._install_finite_chunk(now,feedback,self.finite_candidate['_prepared_reference'],self.bounded_request)
        return self.status

    def _finite_snapshot(self):
        if not self.finite_chunk_count:return None
        active=self.finite_partial_s
        if self.finite_current is not None and self.finite_stage=='executing':
            active=max(0.,min(self.finite_current['execution_duration_s'],self.owner.control_time()-self.finite_current['start_source_s']))
        completed=sum(row['execution_duration_s']for row in self.finite_completed)
        wait_start=(self.finite_completed[-1]['end_source_s']if self.finite_completed else
                    (self.initialization or{}).get('confirmed_source_s',self.owner.control_time()))
        waiting=max(0.,self.owner.control_time()-wait_start)if self.finite_stage in('waiting_observation','preparing')else 0.
        return dict(requested_count=self.finite_chunk_count,completed_count=len(self.finite_completed),
            stage=self.finite_stage or'initializing',source_duration_s=.5,steps_per_chunk=15,
            completed_execution_s=completed,active_execution_s=completed+active,
            waiting_elapsed_s=waiting,total_waiting_s=sum(row['waiting_s']for row in self.finite_completed)+
                (self.finite_current['waiting_s']if self.finite_current else waiting),
            current=deepcopy(self.finite_current),completed=deepcopy(self.finite_completed))

    def _submit(self, command, feedback):
        check_gripper_policy(self.owner,self.config,feedback)
        target = sdk_target(command['final_device14'])
        if any(not lo <= value <= hi for value,(lo,hi) in zip(target,self.sdk_limits)):
            raise ValueError('recorded projection did not produce a legal SDK command')
        tick = self.actual_submission_count
        self.command_context = dict(execution_id=self.plan.sha256, execution_instance_id=self.execution_instance_id,
            execution_tick=tick, source_tick=None, stage='model_initial_hold' if command.get('kind')=='initial_hold' else 'model_inference',
            runtime_nonce=self.owner.runtime_nonce, command_epoch=self.owner.epoch, command_id=command['command_id'])
        send_started = self.owner.control_time()
        if send_started >= self.started+self.config['duration_s']:
            self.loop.tick(send_started, epoch=self.owner.epoch)
            self.command_context = None
            return
        common=bool(self.owner.shared_target_profile)
        if common:
            if self.target_interpreter is None:
                from .target_interpreter import prepare_samples
                held=sdk_target(self.initial_device14)
                reference=prepare_samples([0.,self.loop.period],[held,held],self.sdk_limits,
                    source_sha256=self.plan.sha256,kind='inference_initial_hold')
                self.native_start=self.owner.control_time()
                self.target_interpreter=self.owner.make_target_interpreter(reference,'both',self.native_start)
                self._queue_native(self.owner.control_time(),feedback)
        else:self.owner.send('both', target)
        send_returned, send_wall_ns = self.owner.control_time(), time.time_ns()
        sequence = getattr(self.owner.backend, 'sequence', None)
        if not common and type(sequence) is int and 'applied_command_sequences' in feedback:
            command['backend_sequences'] = dict(left=sequence-1, right=sequence)
        if not common:self.actual_submission_count += 1
        self.outstanding = command
        self.execution_target = target
        self.last_device_target = list(command['final_device14'])
        self._record('source_target'if common else'command', dict(command, target_sdk14=target, command_context=deepcopy(self.command_context),
            disabled_sdk_slots=[13] if self.disabled_grippers else [],
            tick=tick, send_monotonic_s=send_started, send_returned_monotonic_s=send_returned,
            send_wall_ns=send_wall_ns, send_source_time_ns=round(send_started*1e9),
            feedback_at_send=deepcopy(feedback), owner_pid=os.getpid(), command_epoch=self.owner.epoch,
            runtime_nonce=self.owner.runtime_nonce, environment=self.owner.environment))

    def _completed(self, now, feedback):
        command = self.outstanding
        interval=0. if self.owner.shared_target_profile and command and command.get('kind')!='initial_hold'else self.loop.period
        if command is None or feedback['sample_time_s']+1e-9 < max(
                command['due_source_time']+interval,command['submitted_source_time']):
            return
        sequences = command.get('backend_sequences')
        if sequences is not None:
            observed = feedback.get('applied_command_sequences')
            if observed != sequences:
                if now-command['submitted_source_time'] > .1:
                    raise ValueError('fake command application identity unavailable after 0.1 source second')
                return
        evidence = dict(raw_feedback=deepcopy(feedback), firmware_ack='unsupported',
                        application_evidence='physx_readback_sequence' if sequences else 'SDK_return_and_fresh_feedback',
                        backend_sequences=deepcopy(sequences))
        if self.disabled_grippers:
            evidence['disabled_right_gripper']=dict(model_m=self.config['disabled_right_gripper_model_m'],
                device_mm=self.config['disabled_right_gripper_device_mm'],source='profile_max_opening_not_measured',sdk_output=False)
        if self.applied_check is not None:
            extra = self.applied_check(command, feedback)
            if extra is None:
                return
            evidence['device_application'] = extra
        acknowledge = self.loop.acknowledge_external_command if command.get('kind')=='initial_hold' else self.loop.acknowledge
        applied=command['final_device14']
        applied_source=feedback['sample_time_s']
        if self.normal_config:
            from arx_env.joint_head_jaw_mapping import command_gap_mm
            if self.history_output is None or self.history_output[0]!=command['command_id']:
                self.history_output=(command['command_id'],self._normal_output())
            native=self.history_output[1]
            stamps=[x['sample_monotonic_ns']/1e9 for x in native['observations'].values()]
            if min(stamps)<max(command['due_source_time'],command['submitted_source_time']):
                self.history_output=(command['command_id'],self._normal_output());return
            if max(stamps)>feedback['sample_time_s']:return
            applied=list(native['q_sdk14'])
            for side,i in(('left',6),('right',13)):applied[i]=command_gap_mm(side,applied[i])
            applied_source=max(stamps)
            evidence.update(application_evidence='coherent_native_output_and_later_feedback_not_motor_ACK',native_output=native)
            self.history_output=None
        elif self.target_interpreter is not None:
            _,applied=self._native_target(feedback['sample_time_s'])
            evidence['application_evidence']='prepared_reference_evaluation_and_fresh_feedback_not_firmware_ACK'
        acknowledge(command['command_id'], applied_device14=applied,
                    source_time=applied_source, feedback=evidence)
        self.outstanding = None

    def _native_target(self, source_time):
        from arx_env.joint_head_jaw_mapping import command_gap_mm
        reference=self.prepared_reference if self.prepared_reference is not None else self.target_interpreter.reference
        q,_,_=reference.sample([source_time-self.native_start]);target=q[0].tolist()
        device=target[:];device[6]=command_gap_mm('left',target[6]);device[13]=command_gap_mm('right',target[13])
        return target,device

    def _prepare_native_response(self,response,request):
        """Model-query worker only. No SDK calls, no Owner state mutation."""
        import numpy as np
        from .motion_reference import prepare_reference_process
        response_at=self.owner.control_time();conversion_started=time.perf_counter()
        if self.config.get('action_chunk_binding')is not None:
            from .inference import validate_action_chunk
            response=validate_action_chunk(response,binding=self.config['action_chunk_binding'],
                request_id=request['request_id'],epoch=self._epoch,clock_domain=self.config['action_chunk_clock_id'],
                now=self.owner.control_time(),observation_source_time=request['observation']['source_time'],
                valid_until_source_time=request['observation']['source_time']+self.config.get('maximum_observation_age_s',2.),
                last_observation_source_time=self._accepted_chunk_source_time)
        if not isinstance(response,dict)or response.get('request_id')!=request['request_id']:
            raise ValueError('model response request identity mismatch')
        raw=np.asarray(response.get('action14'))
        if raw.shape!=(model_capabilities(self.config)[0],14)or raw.dtype.kind not in 'fiu'or not np.isfinite(raw).all():
            raise ValueError('model response requires finite action14 matching the bound horizon')
        held=list(self.initial_device14)
        converted=[self.adapter.decode(row,held)['final_device14']for row in raw[:self.config['execute_steps']]]
        jaws=np.asarray(converted)[:,[6,13]]
        decision=self._guard_decision('G17',dict(minimum=jaws.min(axis=0).tolist(),maximum=jaws.max(axis=0).tolist()),
            request['observation']['source_time'],self.owner.control_time(),request['request_id'],phase='model_command')
        if decision['verdict']=='reject':raise ValueError('G17: converted model jaw command exceeds the bound calibrated range')
        initial=(self.target_interpreter.reference.sample([self.owner.control_time()-self.native_start])[0][0].tolist()
                 if self.target_interpreter is not None else sdk_target(held))
        device=initial[:]
        from arx_env.joint_head_jaw_mapping import command_gap_mm
        device[6]=command_gap_mm('left',initial[6]);device[13]=command_gap_mm('right',initial[13])
        source=dict(device_targets14=[device,*converted],duration_s=len(converted)/30.,
            kind='online_prediction_chunk',
            model_id=self.config.get('model_id'),calibration_profile_digest=self.config.get('calibration_profile_digest'))
        if self.config.get('task')in ('coffee','lid_handoff'):source['active_arms']=['left','right']
        preparation_started=time.perf_counter()
        options=dict(source_sha256=digest(dict(request_id=request['request_id'],targets=converted)))
        policy=self.config.get('reference_planning')
        if policy is not None:
            if self.target_interpreter is not None:
                reference=self.target_interpreter.reference;elapsed=self.owner.control_time()-self.native_start
                if elapsed<reference.duration_s and reference.metadata.get('kind')not in('inference_initial_hold','normal_hold'):
                    raise ValueError('moving segment handoff needs a committed future boundary; synchronous curve cannot zero its derivatives')
                initial=reference.sample([reference.duration_s])[0][0].tolist()
                source['device_targets14'][0]=initial[:]
                for side,i in(('left',6),('right',13)):source['device_targets14'][0][i]=command_gap_mm(side,initial[i])
            elif self.initialization_source_s is None and self.previous_native_hold is None:
                raise ValueError('known zero-derivative native hold required before first synchronous segment')
            # Synchronous execution only admits a completed zero-v/a reference or
            # an explicitly initialized fixed hold. Pure CPU API also supports
            # nonzero q/v/a boundaries; live moving joins remain separate work.
            options.update(planning_policy=policy,initial_velocity_sdk14=[0.]*14,initial_acceleration_sdk14=[0.]*14)
        if self.reference_executor is not None:options['executor']=self.reference_executor
        cpu_timing={};reference=prepare_reference_process(source,initial,self.sdk_limits,timing=cpu_timing,**options)
        if self.normal_config:
            from .motion_reference import check_brake_reserve,retime_for_brake_reserve
            if self.normal_planned:
                reference=retime_for_brake_reserve(reference,self.sdk_limits,duration_s=self.normal_config['brake_duration_s'],
                    velocity_limit=self.normal_config['velocity_limit'],acceleration_limit=self.normal_config['acceleration_limit'],
                    maximum_duration_s=self.config['reference_planning']['max_execution_duration_s'])
            else:
                check_brake_reserve(reference,self.sdk_limits,duration_s=self.normal_config['brake_duration_s'],
                    velocity_limit=self.normal_config['velocity_limit'],acceleration_limit=self.normal_config['acceleration_limit'])
        from .owner_inference import reference_wire
        self._record('reference',dict(request_id=request['request_id'],reference=reference_wire(reference)))
        return dict(response,_prepared_reference=reference,_preparation_timing=dict(
            response_received_source_s=response_at,prepared_source_s=self.owner.control_time(),
            validation_and_conversion_wall_s=preparation_started-conversion_started,
            preparation_and_reserve_wall_s=time.perf_counter()-preparation_started,
            cpu_stages=cpu_timing,
            mode='warmed_cpu'if self.reference_executor is not None else'one_shot_cpu'))

    def _prepare_native_chunk(self,targets,now,request,response):
        from .motion_reference import PreparedReference
        if self.chunks_exhausted:return None
        observed=request['observation']['source_time']
        if self._accepted_chunk_source_time is not None and observed<=self._accepted_chunk_source_time:
            raise ValueError('prepared chunk observation is duplicated or reordered')
        reference=response.get('_prepared_reference')
        if not isinstance(reference,PreparedReference)or reference.metadata['source_sha256']!=digest(dict(request_id=request['request_id'],targets=targets)):
            raise ValueError('online chunk must be prepared by its CPU query worker before Owner admission')
        self._check_model_reference(reference)
        origin=now+.02
        if self._guard_decision('G23',dict(start_time=self.started,end_time=origin+reference.duration_s),now,now,
                reference.metadata['reference_id'])['verdict']=='reject':
            self.chunks_exhausted=True
            return None  # Keep the prior zero-speed endpoint, never truncate a moving chunk.
        self.native_start=origin
        self.target_interpreter=self.owner.make_target_interpreter(reference,'both',origin,previous=self.target_interpreter)
        self._emit('prepared_chunk',dict(reference=reference.metadata,source_time_origin_s=origin,
            original_prediction_period_s=self.loop.period,actual_knot_times_s=(origin+reference.breaks[1:]).tolist()))
        self._queue_native(now,self.latest_feedback)
        self._accepted_chunk_source_time=observed
        if self.normal_job and self.normal_job['stage']=='RESUMING':
            self.normal_job.update(stage='MODEL_ADMITTED',first_model_source_s=self.target_interpreter.last_packet['host_times_s'][0])
            self._mode_event('model_admitted',mode='HOLD')
        return (origin+reference.breaks[1:]).tolist()

    def _queue_native(self,now,feedback):
        if self.owner.shared_target_profile:
            if self.target_interpreter is None:
                self.target_interpreter=self.owner.make_target_interpreter(self.prepared_reference,'both'if self.bounded_live else'left',self.native_start,
                    integral=bool(self.config.get('reference_integral_feedback')or self.owner.shared_target_profile=='x5_shared_precision_v1'),
                    feedforward=self.reference_feedforward)
            result=self.target_interpreter.tick(now,feedback)
            if result is None:return
            packet,receipt=result['packet'],result['admission']
            relative=[t-self.native_start for t in packet['host_times_s']]
        else:
            packet=self.prepared_reference.packet(now-self.native_start)
            relative=packet.pop('relative_times_s')
            packet.update(host_times_s=[self.native_start+t for t in relative],active_arm='left')
        before=result['submitted_monotonic_s']if self.owner.shared_target_profile else self.owner.control_time()
        if self.reference_integral is not None and not self.owner.shared_target_profile:
            sample=feedback['sample_time_s'];target,_=self._native_target(sample)
            packet['torque_bias_sdk6']=self.reference_integral.packet(before,sample,
                [a-b for a,b in zip(target[:6],feedback['qpos'][:6])],packet['host_times_s'])
            if self.reference_feedforward is not None:
                packet['torque_bias_sdk6']=self.reference_feedforward.packet(packet['host_times_s'],packet['torque_bias_sdk6'],integrator=self.reference_integral)
        try:
            self.owner.command_gate()
            if not self.owner.shared_target_profile:receipt=self.owner.backend.queue_reference(packet)
        except Exception:
            if self.owner.epoch!=self._epoch or self.owner.stop_requested or self.status=='STOPPED':
                self.revoke('control reference revoked');return
            raise
        returned=result['returned_monotonic_s']if self.owner.shared_target_profile else self.owner.control_time();tick=self.actual_submission_count
        self.actual_submission_count+=1
        if self.prepared_reference is not None and not self.normal_config:
            self.loop.submitted+=1;self.native_pending.append(returned)
        self.native_queued_until=packet['host_times_s'][-1]
        self.native_next_upload=now+.1
        # One row represents the actual native queue call, not fictitious per-tick SDK sends.
        from arx_env.joint_head_jaw_mapping import command_gap_mm
        target=list(packet['q_sdk14'][0]);device=target[:]
        for side,i in(('left',6),('right',13)):device[i]=command_gap_mm(side,target[i])
        meta=dict(reference_id=packet['reference_id'],frame_count=len(relative),
            first_host_s=packet['host_times_s'][0],last_host_s=packet['host_times_s'][-1],
            admission=deepcopy(receipt),command_semantics='SDK_future_window_admission_not_point_arrival')
        self._record('command',dict(final_device14=device,target_sdk14=target,tick=tick,
            send_monotonic_s=before,send_returned_monotonic_s=returned,send_wall_ns=time.time_ns(),
            send_source_time_ns=round(before*1e9),
            feedback_at_send=deepcopy(feedback),owner_pid=os.getpid(),command_epoch=self.owner.epoch,
            runtime_nonce=self.owner.runtime_nonce,native_reference=meta))
        if not self.owner.shared_target_profile:
            self._record('native_reference_packet',dict(packet=packet,admission=receipt,
                send_monotonic_s=before,send_returned_monotonic_s=returned))
        if self.owner.shared_target_profile:
            self._record('native_window',dict(packet=packet,admission=receipt,
                send_monotonic_s=before,send_wall_ns=time.time_ns()))

    def _tick_native(self,now,feedback):
        self.loop.last_source_time=now
        if self.owner.epoch!=self._epoch:
            from .target_interpreter import ReferenceRevokedError
            raise ReferenceRevokedError('native reference epoch revoked')
        sample=feedback['sample_time_s']
        while self.native_pending and self.native_pending[0]<=sample:
            self.native_pending.pop(0);self.loop.completed+=1
        if self.finite_chunk_count:return self._tick_finite_chunks(now,feedback)
        if now>=self.started+self.execution_duration_s:
            if self.native_queued_until is None or self.native_queued_until<self.started+self.execution_duration_s:
                raise ValueError('native reference window does not cover the execution endpoint')
            if self.bounded_live:
                self._emit('bounded_execution_elapsed',dict(start_source_s=self.started,stop_requested_source_s=now,
                    requested_duration_s=.5,actual_interval_s=now-self.started,submissions=self.actual_submission_count))
                reason='bounded 0.5s live inference completed; future references revoked'
                self.revoke(reason);self.owner.stop(reason)
                return self.status
            self.status='READY';self.reason='prepared continuous reference completed'
            self.ready_since=now;self.loop.status='completed';self.loop.reason=self.reason
            self.loop.outcome=dict(success=None,reason='control_reference_study_not_task_acceptance')
            self._emit('episode_completed',dict(outcome=self.loop.outcome,completed_commands=self.loop.completed))
            self._finish_data('completed',self.reason)
            return self.status
        if self.native_queued_until is not None and now>self.native_queued_until-.015:
            raise ValueError('native reference window starved; stop before stale nonzero tail')
        if now>=self.native_next_upload:self._queue_native(now,feedback)
        return self.status

    def tick(self, now, feedback):
        from arx_env.joint_head_real_motion import _state
        if self.status in ('ARMED', 'STOPPED', 'FAULT'):
            return self.status
        try:
            q, _, sample, _ = _state(feedback)
            self.latest_feedback=feedback
            check_gripper_policy(self.owner,self.config,feedback)
            if not self.owner.feedback_healthy or not 0 <= now-sample <= .1:
                raise ValueError('fresh healthy feedback required during inference')
            if now < self.previous:
                raise ValueError('inference source clock regressed')
            self.previous = now
            if self.normal_config:
                self.owner.command_gate()
                if tuple(tuple(pair)for pair in self.owner.cache['joint_limits'])!=self.sdk_limits or self.owner.cache.get('jaw_mapping')!=self.jaw_identity:
                    raise ValueError('normal mode device limits or jaw mapping changed')
                self._record('feedback',dict(source_time=sample,raw=deepcopy(feedback),device_state14=device_state(q)))
                if self.target_interpreter is not None:
                    self.execution_target,self.last_device_target=self._native_target(sample)
                if self._normal_tick(now,feedback):return self.status
            if self.prepared_reference is not None and self.started is not None:
                self.execution_target,self.last_device_target=self._native_target(sample)
            elif self.target_interpreter is not None:
                self.execution_target,self.last_device_target=self._native_target(sample)
                self._queue_native(now,feedback)
            self.target_error = dict(device14=[None if self.disabled_grippers and i==13 else a-b
                for i,(a,b) in enumerate(zip(device_state(q), self.last_device_target))],
                unavailable_axes=[13] if self.disabled_grippers else [],policy='record_only') if self.execution_target is not None else None
            if self.status == 'READY':
                return self.status
            self.owner.command_gate()
            if tuple(tuple(pair) for pair in self.owner.cache['joint_limits']) != self.sdk_limits:
                raise ValueError('device joint limits changed during inference')
            if self.owner.cache.get('jaw_mapping') != self.jaw_identity:
                raise ValueError('jaw mapping changed during inference')
            # Recording keeps the invalid raw right readback. Only model-facing
            # state and completed-command history use the explicit assumption.
            if not self.normal_config:
                self._record('feedback', dict(source_time=sample, raw=deepcopy(feedback), device_state14=device_state(q),
                    unavailable_axes=[13] if self.disabled_grippers else []))
            if self.status == 'VERIFYING':
                self._prepare(now,feedback)
                return self.status
            if self.prepared_reference is not None:
                return self._tick_native(self.owner.control_time(),feedback)
            # Never acknowledge a command beyond the accepted episode deadline.
            deadline = self.started+self.config['duration_s']
            supply_timeout=self.config.get('reference_supply_timeout_s')
            if supply_timeout is not None and not self.chunks_exhausted:
                endpoint=(self.native_start+self.target_interpreter.reference.duration_s
                    if self.target_interpreter is not None else self.started)
                if now>endpoint+supply_timeout and now<deadline:
                    raise ValueError('reference supply stopped after planned zero-speed endpoint')
            if sample <= deadline+1e-9:
                self._completed(now, feedback)
            query_ready = (not self.chunks_exhausted and now < deadline and self.outstanding is None
                           and not self.loop._commands and not self.loop._query_pending)
            observation = self.observe(now, feedback) if query_ready else None
            if observation is not None and self.initialization_source_s is not None and observation['source_time']<=self.initialization_source_s:
                observation=None
            if (observation is not None and self.normal_job and self.normal_job['stage']=='RESUMING'
                    and observation['source_time']<=self.normal_job['observation_floor']):observation=None
            try:
                support = self.support_observe(now, feedback) if self.support_observe else None
            except Exception as exc:
                support = {'error': str(exc)}  # Acceptance unavailable; does not interrupt control.
            now = self.owner.control_time()  # Observation copying must not extend the motion deadline.
            command = self.loop.tick(now, epoch=self.owner.epoch, observation=observation, support=support)
            if self.loop.status == 'completed':
                self.outstanding = None
                # Retain the real sensor timestamp; a <=30 ms coverage deficit
                # is accepted explicitly, never relabelled as the configured endpoint.
                if sample+self.config['duration_tolerance_s']+1e-9 < deadline:
                    self.status, self.reason = 'SETTLING', 'waiting for measured episode endpoint'
                    return self.status
                self.status, self.reason, self.ready_since = 'READY', f'{self.config["duration_s"]:g} source seconds completed', now
                self._finish_data('completed', self.reason)
            elif self.loop.status == 'stopped':
                if self.normal_config and self.loop.failure_phase=='prediction_supply':
                    self._pause_for_supply(self.loop.reason);return self.status
                raise ValueError(self.loop.reason)
            elif command is not None:
                self._submit(command, feedback)
            return self.status
        except Exception as exc:
            if self.owner.cancel_requested_reference(exc):
                self.revoke('operator stop/release');return self.status
            if self.normal_job:self.reject_mode(self.normal_job['request']['request_id'],str(exc))
            self.loop.stop(str(exc))
            if self.finite_chunk_count:
                self._finite_freeze_elapsed();self.finite_stage='failed'
            if self.initialization is not None and self.started is None:
                self.initialization.update(stage='failed',reason=str(exc))
            self.status, self.reason = 'FAULT', str(exc)
            self.mode='FAULT'
            self._finish_data('failed', self.reason)
            self.owner.motion_fault(exc)
            self.status, self.reason = 'FAULT', str(exc)
            self.mode='FAULT'
            return self.status

    def snapshot(self):
        return dict(self.loop.snapshot(), runner_status=self.status, recording_error=self.recording_error,
                    model_pending=(self.bounded_worker.is_alive()if self.normal_planned and self.bounded_worker else self.loop._query_pending),
                    normal_model_segment=deepcopy(self.normal_segment),
                    normal_model_budget=('single_complete_source_segment_acceptance'if self.normal_single_segment else'episode_wall_budget'if self.normal_planned else None),
                    finite_chunks=self._finite_snapshot(),
                    initialization=deepcopy(self.initialization),
                    command_epoch=self.owner.epoch,runtime_nonce=self.owner.runtime_nonce,
                    control_state=self.owner.control.state.value,feedback_healthy=self.owner.feedback_healthy,stationary=self.owner.stationary,
                    recording_pending=self.recording_pending,actual_submission_count=self.actual_submission_count,
                    preparation_complete=self.preparation_complete,mode=self.mode,
                    transition_stage=(self.normal_job['stage']if self.normal_job else
                        'AWAITING_RECORDING_GATE'if getattr(self,'normal_supply_exhausted',False)and self.mode=='MODEL'else None),
                    mode_receipts=deepcopy(self.mode_receipts),mode_session_id=self.mode_session_id,
                    finish_capability=dict(configured=bool(self.config.get('_normal_finish')),
                        completed=self.finish_completed,
                        kind=(self.config.get('_normal_finish')or{}).get('kind'),
                        policy_digest=(self.config.get('_normal_finish')or{}).get('policy_digest')))


def make_inference_runner(owner, proxy, feedback, config, **dependencies):
    return InferenceRunner(owner, proxy, feedback, config, **dependencies)
