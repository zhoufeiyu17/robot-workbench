"""Execution error policy and independent measured joint-speed protection."""
from copy import deepcopy
import hashlib
import json
import math
from numbers import Real

CAPTURE_QUEUE_BYTES=128*1024**2
CAPTURE_MESSAGE_BYTES=512*1024
CAPTURE_PREPARATION_TIMEOUT_S=1.
FEEDBACK_TIMEOUT_S=.1
INTEGRAL_FEEDBACK_TIMEOUT_S=.05
REFERENCE_STARVATION_MARGIN_S=.015
TRAINING_RGB_PAIR_ERROR_S=.025
TRAINING_STATE_PAIR_ERROR_S=.012
MODEL_JAW_OPEN_MARGIN_MM=2.
NORMAL_BRAKE_DURATION_S=.2
NORMAL_GAIN_TRANSITION_S=.05
PEDAL_DEBOUNCE_S=.05
NATIVE_WATCHDOG_TIMEOUT_S=.1
DEFAULT_JOINT_TOLERANCE_RAD=math.radians(2.)
DEFAULT_MEASURED_SPEED_LIMIT_RAD_S=2.5
PHYSICAL_FEEDBACK_MARGIN_RAD=math.radians(5.)
STATIONARY_PROFILE=dict(window_s=.3,max_sample_gap_s=.1,joint_span_rad=DEFAULT_JOINT_TOLERANCE_RAD,jaw_span_mm=2.)
LEGACY_REFERENCE_LIMITS=dict(velocity_limit_rad_s=.25,acceleration_limit_rad_s2=1.)


def recording_parameters(business=None, value=None):
    """One approved purpose rule and resource ceiling for policy and recorder."""
    fixed={'calibration':'data_required','collection':'data_required','inference':'infer_debug'}
    if business not in (None,*fixed,'research'):raise ValueError('known recording business required')
    maximum=dict(event_queue_bytes=CAPTURE_QUEUE_BYTES,message_bytes=CAPTURE_MESSAGE_BYTES)
    if value is None:value=dict(purpose_profile=fixed.get(business),resource_budget=maximum)
    if not isinstance(value,dict) or set(value)!={'purpose_profile','resource_budget'}:
        raise ValueError('recording policy requires purpose_profile and resource_budget')
    purpose=value['purpose_profile']
    if purpose not in ('data_required','infer_debug',None) or purpose is None and business not in (None,'research'):
        raise ValueError('known recording purpose required')
    if business in fixed and purpose!=fixed[business]:raise ValueError('recording purpose is fixed for this business')
    budget=value['resource_budget']
    if not isinstance(budget,dict) or set(budget)!=set(maximum) or any(
            type(v) is not int or not 256<v<=maximum[key] for key,v in budget.items()):
        raise ValueError('recording resource budget exceeds the approved limits')
    return dict(purpose_profile=purpose,resource_budget=dict(budget))


def reference_execution_limits(policy=None):
    """Project the frozen planning policy to the bounds used by SDK admission."""
    if policy is None:
        return dict(velocity_limit_rad_s=2., acceleration_limit_rad_s2=5.)
    from .motion_reference import infer_reference_policy
    normalized = infer_reference_policy(policy)
    return {key: normalized[key] for key in (
        'velocity_limit_rad_s', 'acceleration_limit_rad_s2', 'jaw_velocity_limit_mm_s')}


def validate_execution_context(value):
    """Validate the once-per-generation SDK binding; retain only consumed fields."""
    fields = {'controller_profile', 'jaw_calibration_id'}
    if not isinstance(value, dict) or not fields <= set(value) or set(value) - fields - {'limits', 'velocity_source'}:
        raise ValueError('execution context requires controller profile and jaw calibration identity')
    if not isinstance(value['controller_profile'], str) or not 0 < len(value['controller_profile']) <= 128:
        raise ValueError('bounded controller profile required')
    calibration = value['jaw_calibration_id']
    if calibration is not None and (not isinstance(calibration, str) or len(calibration) != 64
                                   or any(c not in '0123456789abcdef' for c in calibration)):
        raise ValueError('jaw calibration identity must be a SHA256 or explicit null')
    if 'limits' in value:
        limits = value['limits']
        expected = {'velocity_limit_rad_s', 'acceleration_limit_rad_s2', 'jaw_velocity_limit_mm_s'}
        if not isinstance(limits, dict) or set(limits) != expected:
            raise ValueError('complete executable joint and jaw limits required')
        if any(type(v) not in (int, float) or not math.isfinite(v) or v <= 0 for v in limits.values()):
            raise ValueError('execution limits must be finite positive numbers')
    if 'velocity_source' in value and value['velocity_source'] != 'sdk_position_only':
        raise ValueError('known execution velocity source required')
    return deepcopy(value)


def control_parameters(settings, *, configuration=None, business='calibration'):
    """Project existing values into the common policy without adding identities."""
    value = validate(settings)
    result = dict(
        measured=deepcopy(value['speed_protection']),
        tracking=dict(enabled=value['tracking_enabled'], joint_rad=value['tracking_joint_rad'],
                      jaw_mm=value['tracking_jaw_mm'], duration_s=value['tracking_duration_s']),
        arrival=dict(enabled=value['arrival_enabled'], tolerances_rad=value['arrival_tolerances_rad'],
                     jaw_mm=value['arrival_jaw_mm']),
        execution=dict(fake_error=value['fake_error']), recording=recording_parameters(business))
    if business == 'inference':
        config = configuration or {}
        result['observation']=dict(maximum_age_s=config.get('maximum_observation_age_s',.5),
            state_max_age_s=config.get('state_max_age_s',.1),max_camera_span_s=config.get('max_camera_span_s',.1))
        result['execution']['execute_steps'] = config.get('execute_steps', 15)
        result['execution']['prepare_drift_joint_rad'] = config.get('prepare_drift_joint_rad', DEFAULT_JOINT_TOLERANCE_RAD)
        result['execution']['prepare_drift_jaw_mm'] = config.get('prepare_drift_jaw_mm', 2.)
        if configuration is not None:
            if config.get('reference_planning') is not None:
                result['planning'] = {key: config['reference_planning'][key] for key in (
                    'velocity_limit_rad_s', 'acceleration_limit_rad_s2', 'jaw_velocity_limit_mm_s')}
            if 'reference_supply_timeout_s' in config:
                result['model'] = dict(reference_supply_timeout_s=config['reference_supply_timeout_s'])
    return result


def execution_settings_from_policy(parameters, *, environment=None):
    return validate(dict(
        speed_protection=deepcopy(parameters['measured']),
        tracking_enabled=parameters['tracking']['enabled'],
        tracking_joint_rad=parameters['tracking']['joint_rad'],
        tracking_jaw_mm=parameters['tracking']['jaw_mm'],
        tracking_duration_s=parameters['tracking']['duration_s'],
        arrival_enabled=parameters['arrival']['enabled'],
        arrival_tolerances_rad=deepcopy(parameters['arrival']['tolerances_rad']),
        arrival_jaw_mm=parameters['arrival']['jaw_mm'],
        fake_error=deepcopy(parameters['execution']['fake_error'])), environment=environment)


def policy_hash(parameters):
    content = json.dumps(dict(schema='workbench.control-policy.v1', parameters=parameters),
                         sort_keys=True, separators=(',', ':'), allow_nan=False).encode()
    return hashlib.sha256(content).hexdigest()


def editable_policy_paths(parameters, *, business=None):
    result = [group+'.'+key for group, values in parameters.items() for key in values if key not in ('fake_error','resource_budget')]
    result.extend('execution.fake_error.'+key for key in parameters['execution']['fake_error'])
    if 'recording' in parameters:
        result.extend('recording.resource_budget.'+key for key in parameters['recording']['resource_budget'])
    if business in ('calibration','collection','inference'):
        result=[path for path in result if path!='recording.purpose_profile']
    return sorted(result)


def merge_control_policy(parameters, changes, *, environment, business=None):
    if not isinstance(changes, dict) or not changes:
        raise ValueError('nonempty control policy field changes required')
    allowed = set(editable_policy_paths(parameters,business=business))
    if set(changes) - allowed:
        raise ValueError('unknown or unavailable control policy field')
    result = deepcopy(parameters)
    for path, value in changes.items():
        target = result
        parts = path.split('.')
        for part in parts[:-1]:target = target[part]
        target[parts[-1]] = deepcopy(value)
    return validate_control_parameters(result, environment=environment,business=business)


def validate_control_parameters(parameters, *, environment,business=None):
    if not isinstance(parameters, dict):
        raise ValueError('control policy parameters must be an object')
    parameters=deepcopy(parameters)
    if business=='inference' and 'observation' not in parameters:
        parameters['observation']=dict(maximum_age_s=.5)
    if isinstance(parameters.get('observation'),dict):
        parameters['observation'].setdefault('state_max_age_s',.1)
        parameters['observation'].setdefault('max_camera_span_s',.1)
    fields = dict(measured={'enabled', 'limit_rad_s'}, tracking={'enabled', 'joint_rad', 'jaw_mm', 'duration_s'},
                  arrival={'enabled', 'tolerances_rad', 'jaw_mm'},
                  execution={'fake_error', 'execute_steps', 'prepare_drift_joint_rad', 'prepare_drift_jaw_mm'},
                  planning={'velocity_limit_rad_s', 'acceleration_limit_rad_s2', 'jaw_velocity_limit_mm_s'},
                  observation={'maximum_age_s','state_max_age_s','max_camera_span_s'}, model={'reference_supply_timeout_s'},
                  recording={'purpose_profile','resource_budget'})
    if not {'measured', 'tracking', 'arrival', 'execution'} <= parameters.keys() or parameters.keys() - fields.keys():
        raise ValueError('complete known control policy groups required')
    for group, values in parameters.items():
        if not isinstance(values, dict) or values.keys() - fields[group]:
            raise ValueError('unknown control policy group field')
        required = {'fake_error'} if group == 'execution' else fields[group]
        if not required <= values.keys():
            raise ValueError('complete control policy group required')
    result = deepcopy(parameters)
    if 'recording' in result or business is not None:
        result['recording']=recording_parameters(business,result.get('recording'))
    settings = execution_settings_from_policy(result, environment=environment)
    normalized = control_parameters(settings)
    for group in ('measured', 'tracking', 'arrival'):
        result[group] = normalized[group]
    result['execution']['fake_error'] = normalized['execution']['fake_error']
    steps = result['execution'].get('execute_steps')
    if steps is not None and (type(steps) is not int or not 1 <= steps <= 60):
        raise ValueError('execute_steps must be an integer from1 to60')
    for key in ('prepare_drift_joint_rad', 'prepare_drift_jaw_mm'):
        if key in result['execution']:
            value = result['execution'][key]
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError(key+' must be finite and positive')
            result['execution'][key] = float(value)
    planning = result.get('planning')
    if planning:
        fields={'velocity_limit_rad_s','acceleration_limit_rad_s2','jaw_velocity_limit_mm_s'}
        if set(planning) != fields or any(type(v) not in (int, float) or not math.isfinite(v)
                or v<=0 for v in planning.values()) or planning['acceleration_limit_rad_s2']>40.:
            raise ValueError('规划速度须为有限正数，加速度上限为40 rad/s²')
        if environment == 'physical' and (not result['measured']['enabled']
                or planning['velocity_limit_rad_s'] > result['measured']['limit_rad_s']):
            raise ValueError('planning velocity requires enabled and sufficient measured-speed protection')
        result['planning'] = {key: float(value) for key, value in planning.items()}
    for group, key in [('observation', 'maximum_age_s'), ('model', 'reference_supply_timeout_s')]:
        if group in result:
            value = result[group][key]
            # Model-specific limits are checked by Owner.configuration before
            # committing a bound policy; unbound stores can hold either family.
            maximum = 2.5 if group == 'observation' else .5 if planning else math.inf
            if type(value) not in (int, float) or not math.isfinite(value) or not 0 < value <= maximum:
                raise ValueError(group+'.'+key+' exceeds the supported time window')
            result[group][key] = float(value)
    if 'observation' in result:
        for key in ('state_max_age_s','max_camera_span_s'):
            value=result['observation'][key]
            if type(value) not in (int,float) or not math.isfinite(value) or not 0<value<=.1:
                raise ValueError('observation.'+key+' must fit the qualified0.1s input profile')
            result['observation'][key]=float(value)
    return result


class Guard:
    """Pure bounded decisions; callers retain responsibility for the action."""
    _rules = {
        'G02': ('observation', 'maximum_age_s', 's', '0<=observed<=limit', 'REJECT_INPUT'),
        'G03': ('observation', 'state_max_age_s', 's', '0<=observed<=limit', 'REJECT_INPUT'),
        'G04': ('observation', 'max_camera_span_s', 's', '0<=observed<=limit', 'REJECT_INPUT'),
        'G11': ('execution', 'prepare_drift_joint_rad', 'rad', 'abs(observed)<=limit', 'REJECT_PLAN'),
        'G12': ('execution', 'prepare_drift_jaw_mm', 'mm', 'abs(observed)<=limit', 'REJECT_PLAN'),
        'G13': ('planning', 'velocity_limit_rad_s', 'rad/s', '0<=observed<=limit+1e-10', 'REJECT_PLAN'),
        'G14': ('measured', 'limit_rad_s', 'rad/s', 'abs(observed)<=limit', 'FAULT_STOP'),
        'G15': ('planning', 'acceleration_limit_rad_s2', 'rad/s²', '0<=observed<=limit+1e-10', 'REJECT_PLAN'),
        'G16': ('planning', 'jaw_velocity_limit_mm_s', 'mm/s', '0<=observed<=limit+1e-10', 'REJECT_PLAN'),
        'G21': ('execution', 'execute_steps', 'steps', 'observed==limit', 'REJECT_PLAN'),
        'G22': ('execution', 'execute_steps', 's', 'finite observed>0', 'REJECT_PLAN'),
        'G35': ('recording', 'purpose_profile', 'failed_conditions', 'observed==0', 'REJECT_INPUT'),
        'G38': ('recording', 'purpose_profile', 'failed_conditions', 'observed==0', 'BRAKE_TO_HOLD'),
    }
    _bound_rules={'G01','G05','G06','G07','G08','G09','G10','G17','G18','G19','G20','G23','G24','G25','G26','G27','G28','G29','G30','G31','G33','G36','G37'}

    @classmethod
    def bind(cls,policy,context):
        return _BoundGuard(policy,context)

    @classmethod
    def evaluate(cls, signal, phase, policy):
        return cls._evaluate(signal,phase,policy,None)

    @classmethod
    def _evaluate(cls,signal,phase,policy,context):
        required = {'rule_id', 'signal_id', 'observed', 'source_time', 'checked_time'}
        identity_fields = {'run_id', 'epoch', 'request_id'}
        if (not isinstance(signal, dict) or not required <= signal.keys()
                or signal.keys() - required - identity_fields or not isinstance(signal['rule_id'], str)
                or signal['rule_id'] not in cls._rules.keys()|cls._bound_rules
                or any(not isinstance(value, str) or not 0 < len(value) <= 128
                       for value in (phase, signal['signal_id']))):
            raise ValueError('known bounded guard signal and phase required')
        for key in ('source_time', 'checked_time'):
            if type(signal[key]) not in (int, float) or not math.isfinite(signal[key]):
                raise ValueError('finite guard '+key+' required')
        for key in identity_fields & signal.keys():
            value = signal[key]
            if ((key == 'epoch' and (type(value) is not int or value < 0))
                    or (key != 'epoch' and (not isinstance(value, str) or not 0 < len(value) <= 200))):
                raise ValueError('bounded existing guard context required')
        override=None
        if signal['rule_id'] in cls._bound_rules:
            if context is None:raise ValueError('this rule requires the bound read-only capability context')
            observed,limit,unit,comparison,failure,enabled,passed,override=_bound_values(signal['rule_id'],signal['observed'],policy['parameters'],context,phase)
            recording=False
        else:
            group, key, unit, comparison, failure = cls._rules[signal['rule_id']]
            settings = policy['parameters'][group]
            observed = signal['observed']
            if type(observed) not in (int,float) or not math.isfinite(observed):raise ValueError('finite guard observed required')
            recording = group == 'recording'
        if signal['rule_id'] in cls._bound_rules:pass
        elif signal['rule_id']=='G22':
            limit,enabled,passed=None,True,observed>0
        elif recording:
            purpose = settings[key]
            if purpose not in ('data_required', 'infer_debug') or type(observed) is not int or observed < 0:
                raise ValueError('known recording purpose and failed-condition count required')
            limit, enabled, passed = 0, purpose == 'data_required'and phase!='seal', observed == 0
        else:
            limit = settings[key]
            if type(limit) not in (int, float) or not math.isfinite(limit) or limit <= 0:
                raise ValueError('finite positive guard limit required')
            enabled = settings.get('enabled', True)
            if signal['rule_id']=='G21':
                passed=type(observed) is int and observed==limit
            elif signal['rule_id'] in ('G13','G15','G16'):passed=0<=observed<=limit+1e-10
            else:passed = 0 <= observed <= limit if signal['rule_id'] in ('G02','G03','G04') else abs(observed) <= limit
        if type(enabled) is not bool:raise ValueError('boolean guard enabled required')
        planning_audit=signal['rule_id']in ('G13','G15','G16')
        if planning_audit and observed>=0 and not passed:override='observe'
        verdict = override or (('pass' if passed else 'observe' if not enabled else 'reject') if recording else (
            'observe' if not enabled else 'pass' if passed else 'reject')
        )
        return dict(rule_id=signal['rule_id'], phase=phase, signal_id=signal['signal_id'],
                    verdict=verdict, severity='error' if verdict == 'reject' else 'warning' if (recording or planning_audit or signal['rule_id'] in cls._bound_rules) and not passed else 'info',
                    action=failure if verdict == 'reject' else 'CONTINUE',
                    observed=observed, limit=limit, unit=unit, comparison=comparison,
                    revision=policy['revision'], policy_hash=policy['policy_hash'],
                    source_time=signal['source_time'], checked_time=signal['checked_time'],
                    **{key: signal[key] for key in identity_fields if key in signal})


class _BoundGuard:
    def __init__(self,policy,context):
        if (not isinstance(policy,dict) or not {'revision','policy_hash','parameters'}<=policy.keys()
                or policy_hash(policy['parameters'])!=policy['policy_hash']):
            raise ValueError('bound Guard requires the unchanged frozen policy')
        allowed={'environment','joint_limits','feedback_timeout_s','integral_feedback_timeout_s','starvation_margin_s','expected_identity',
                 'training_rgb_pair_error_s','training_state_pair_error_s','infer_timeout_s','pin_ttl_s','run_duration_s',
                 'jaw_limits_mm','model_jaw_limits_mm','disabled_grippers','normal_brake_s','gain_transition_s'}
        if not isinstance(context,dict) or context.keys()-allowed:raise ValueError('only consumed Guard context fields allowed')
        context=deepcopy(context)
        if 'environment' in context and context['environment'] not in ('fake','physical'):raise ValueError('explicit Guard environment required')
        if 'expected_identity' in context:context['expected_identity']=_guard_identity(context['expected_identity'])
        for key in ('jaw_limits_mm','model_jaw_limits_mm'):
            if key in context:
                if not isinstance(context[key],(list,tuple))or len(context[key])!=2:raise ValueError('two calibrated jaw bounds required')
                context[key]=[_guard_vector(pair,2,'jaw bounds')for pair in context[key]]
                if any(low>high for low,high in context[key]):raise ValueError('ordered jaw bounds required')
        if 'disabled_grippers'in context and (not isinstance(context['disabled_grippers'],(list,tuple))or
                len(set(context['disabled_grippers']))!=len(context['disabled_grippers'])or any(s not in ('left','right')for s in context['disabled_grippers'])):
            raise ValueError('declared disabled grippers required')
        validate_control_parameters(policy['parameters'],environment=context.get('environment'))
        for key in ('feedback_timeout_s','integral_feedback_timeout_s','starvation_margin_s','training_rgb_pair_error_s','training_state_pair_error_s','infer_timeout_s','pin_ttl_s','run_duration_s','normal_brake_s','gain_transition_s'):
            if key in context:context[key]=_guard_number(context[key],key,positive=True)
        if 'joint_limits' in context:
            limits=context['joint_limits']
            if not isinstance(limits,(list,tuple)) or not 1<=len(limits)<=12:raise ValueError('bounded rotary joint limits required')
            for pair in limits:
                row=_guard_vector(pair,2,'joint limits')
                if row[0]>row[1]:raise ValueError('ordered joint limits required')
            context['joint_limits']=[_guard_vector(pair,2,'joint limits')for pair in limits]
        self._policy={key:deepcopy(policy[key])for key in ('revision','policy_hash','parameters')}
        self._context=context

    def with_policy(self, policy):
        return _BoundGuard(policy, self._context)

    def evaluate(self,signal,phase):
        required={'G01':{'expected_identity'},'G05':{'training_rgb_pair_error_s'},'G06':{'training_state_pair_error_s'},
                  'G09':{'infer_timeout_s'},'G10':{'pin_ttl_s'},
                  'G23':{'run_duration_s'},
                  'G24':{'normal_brake_s','gain_transition_s'},
                  'G17':{'model_jaw_limits_mm'if phase=='model_command'else'jaw_limits_mm'},
                  'G18':{'joint_limits'},'G19':{'joint_limits','environment'},'G20':{'joint_limits','environment'},
                  'G28':{'feedback_timeout_s'},'G29':{'starvation_margin_s'}}.get(signal.get('rule_id')if isinstance(signal,dict)else None,set())
        if isinstance(signal,dict)and signal.get('rule_id')=='G28'and phase=='feedback_integral':required={'integral_feedback_timeout_s'}
        if not required<=self._context.keys():raise ValueError('bound Guard context missing: '+', '.join(sorted(required-self._context.keys())))
        return Guard._evaluate(signal,phase,self._policy,self._context)


def _guard_number(value,name,*,positive=False):
    if isinstance(value,bool) or not isinstance(value,Real) or not math.isfinite(value) or positive and value<=0:
        raise ValueError('finite '+name+(' >0'if positive else'')+' required')
    return value if type(value)in(int,float)else float(value)


def _guard_vector(value,length,name):
    if not isinstance(value,(list,tuple)) or len(value)!=length:raise ValueError(name+' shape differs')
    return [_guard_number(part,name)for part in value]


def _guard_identity(value):
    count=0
    def copy(row,depth=0):
        nonlocal count
        count+=1
        if count>256 or depth>5:raise ValueError('bounded identity fields required')
        if isinstance(row,dict):
            if len(row)>32 or any(not isinstance(k,str)or not 0<len(k)<=128 for k in row):raise ValueError('bounded identity keys required')
            if any(key.lower()in('token','api_key','apikey')or key.lower().endswith('_token')or
                   any(secret in key.lower()for secret in ('password','secret','authorization','csrf'))for key in row):
                raise ValueError('credentials are not Guard identities')
            return {key:copy(part,depth+1)for key,part in row.items()}
        if isinstance(row,(list,tuple)):
            if len(row)>32:raise ValueError('bounded identity vector required')
            return [copy(part,depth+1)for part in row]
        if row is None or type(row)is bool:return row
        if isinstance(row,str):
            if len(row)>200:raise ValueError('bounded identity string required')
            return row
        return _guard_number(row,'identity value')
    if not isinstance(value,dict)or not value:raise ValueError('nonempty declared identity required')
    return copy(value)


def _same_identity(left,right):
    if type(left)is not type(right):return False
    if isinstance(left,dict):return left.keys()==right.keys()and all(_same_identity(v,right[k])for k,v in left.items())
    if isinstance(left,list):return len(left)==len(right)and all(_same_identity(a,b)for a,b in zip(left,right))
    return left==right


def _bound_values(rule,value,parameters,context,phase):
    override=None
    if rule in ('G33','G36','G37'):
        if (not isinstance(value,dict)or set(value)!={'failed_conditions','receipt_ref'}
                or type(value['failed_conditions'])is not int or value['failed_conditions']<0
                or not isinstance(value['receipt_ref'],str)or not 0<len(value['receipt_ref'])<=512):
            raise ValueError('declared recording condition count and receipt reference required')
        observed=dict(value);passed=value['failed_conditions']==0
        required=parameters['recording']['purpose_profile']=='data_required'
        # Sealing reports what was retained; it cannot retroactively stop motion.
        report_only=phase=='seal'or rule=='G33'
        return observed,0,'failed_conditions','failed_conditions==0','CONTINUE'if report_only else'BRAKE_TO_HOLD',True,passed,(
            'observe'if not passed and (report_only or not required) else None)
    if rule=='G27':
        if phase=='stationary_gap':
            observed=_guard_number(value,'stationary sample gap');limit=STATIONARY_PROFILE['max_sample_gap_s']
            passed=0<observed<=limit;comparison='0<gap<=limit';unit='s'
        elif phase=='stationary_span':
            observed=_guard_vector(value,14,'stationary position span')
            limit=[STATIONARY_PROFILE['jaw_span_mm']if i in (6,13)else STATIONARY_PROFILE['joint_span_rad']for i in range(14)]
            passed=all(0<=v<=bound for v,bound in zip(observed,limit));comparison='0<=span<=axis_limit';unit='rad / mm'
        elif phase=='stationary_window':
            observed=_guard_number(value,'stationary window duration');limit=STATIONARY_PROFILE['window_s']
            passed=observed>=limit-1e-9;comparison='duration>=window-1e-9';unit='s'
        else:raise ValueError('declared stationary phase required')
        return observed,limit,unit,comparison,'WAIT',True,passed,None
    if rule=='G30':
        if not isinstance(value,dict)or set(value)!={'monitored','age_s','fault_latched'}or any(type(value[k])is not bool for k in ('monitored','fault_latched')):
            raise ValueError('declared native watchdog facts required')
        observed=dict(value)
        if value['age_s']is not None:observed['age_s']=_guard_number(value['age_s'],'watchdog age')
        passed=not value['monitored']or not value['fault_latched']and observed['age_s']is not None and 0<=observed['age_s']<=NATIVE_WATCHDOG_TIMEOUT_S
        return observed,NATIVE_WATCHDOG_TIMEOUT_S,'s','unmonitored or healthy heartbeat within native timeout','FAULT_STOP',True,passed,None
    if rule=='G31':
        if not isinstance(value,dict)or set(value)!=set(LEGACY_REFERENCE_LIMITS):raise ValueError('declared legacy reference peaks required')
        observed={key:_guard_number(v,'legacy reference peak')for key,v in value.items()}
        passed=all(0<=observed[key]<=bound+1e-9 for key,bound in LEGACY_REFERENCE_LIMITS.items())
        return observed,dict(LEGACY_REFERENCE_LIMITS),'rad/s / rad/s²','legacy peaks<=profile+1e-9','REJECT_PLAN',True,passed,None
    if rule=='G01':
        observed=_guard_identity(value);expected=context['expected_identity']
        return observed,deepcopy(expected),'identity','typed identities equal','INVALIDATE_RUN',True,_same_identity(observed,expected),None
    if rule in ('G05','G06'):
        observed=_guard_number(value,'pairing error')
        limit=context['training_rgb_pair_error_s'if rule=='G05'else'training_state_pair_error_s']
        passed=abs(observed)<=limit
        return observed,limit,'s','abs(error)<=limit','CONTINUE',True,passed,None if passed else'observe'
    if rule in ('G07','G08'):
        limit=parameters['model']['reference_supply_timeout_s']if rule=='G07'else parameters['observation']['maximum_age_s']
        _guard_number(limit,'model supply limit',positive=True)
        if isinstance(value,dict):
            if set(value)!={'current_time','basis_time'}:raise ValueError('declared supply timestamps required')
            observed={key:_guard_number(stamp,'supply timestamp')for key,stamp in value.items()}
            observed['elapsed_s']=observed['current_time']-observed['basis_time']
            passed=observed['basis_time']<=observed['current_time']<=observed['basis_time']+limit
        else:
            observed=_guard_number(value,'supply age');passed=0<=observed<=limit
        return observed,limit,'s','basis<=now<=basis+limit','BRAKE_TO_HOLD'if rule=='G07'else'REJECT_INPUT',True,passed,None
    if rule in ('G09','G10'):
        limit=context['infer_timeout_s'if rule=='G09'else'pin_ttl_s']
        if rule=='G10':
            if not isinstance(value,dict)or set(value)!={'current_time','renewed_time'}:raise ValueError('declared lease timestamps required')
            observed={key:_guard_number(stamp,'lease timestamp')for key,stamp in value.items()}
            observed['age_s']=observed['current_time']-observed['renewed_time']
            passed=(0<=observed['age_s']<limit if phase=='model_pin_client'else
                observed['renewed_time']<=observed['current_time']<observed['renewed_time']+limit)
        else:
            if isinstance(value,dict):
                if set(value)!={'elapsed_s','timed_out'}or type(value['timed_out'])is not bool:raise ValueError('declared HTTP timeout fact required')
                observed=dict(elapsed_s=_guard_number(value['elapsed_s'],'HTTP elapsed'),timed_out=value['timed_out'])
                passed=not observed['timed_out']and 0<=observed['elapsed_s']<=limit
            else:observed=_guard_number(value,'HTTP elapsed');passed=0<=observed<=limit
        return observed,limit,'s','elapsed<=timeout'if rule=='G09'else'now<renewed+ttl','REJECT_INPUT'if rule=='G09'else'INVALIDATE_RUN',True,passed,None
    if rule=='G23':
        if not isinstance(value,dict)or set(value)!={'start_time','end_time'}:raise ValueError('declared Run budget timestamps required')
        observed={key:_guard_number(stamp,'Run budget timestamp')for key,stamp in value.items()}
        limit=context['run_duration_s'];deadline=observed['start_time']+limit
        passed=observed['start_time']<=observed['end_time']and(observed['end_time']<deadline if phase=='query'else observed['end_time']<=deadline)
        return observed,limit,'s','end<start+budget'if phase=='query'else'end<=start+budget','BRAKE_TO_HOLD',True,passed,None
    if rule=='G24':
        if not isinstance(value,dict)or set(value)!={'brake_duration_s','gain_transition_s'}:raise ValueError('declared normal transition durations required')
        observed={key:_guard_number(v,'normal transition duration',positive=True)for key,v in value.items()}
        limit=dict(brake_duration_s=context['normal_brake_s'],gain_transition_s=context['gain_transition_s'])
        passed=all(abs(observed[key]-expected)<=1e-10 for key,expected in limit.items())
        return observed,limit,'s','durations match bound normal profile (1e-10 arithmetic allowance)','REJECT_PLAN',True,passed,None
    if rule=='G17':
        if not isinstance(value,dict)or not {'minimum','maximum'}<=value.keys()or value.keys()-{'minimum','maximum','active_arm'}:
            raise ValueError('declared jaw command extrema required')
        observed={key:_guard_vector(value[key],2,'jaw command extrema')for key in ('minimum','maximum')}
        arm=value.get('active_arm','both')
        if arm not in ('left','right','both'):raise ValueError('declared jaw command arm required')
        if 'active_arm'in value:observed['active_arm']=arm
        limits=context['model_jaw_limits_mm'if phase=='model_command'else'jaw_limits_mm']
        enabled=[i for i,side in enumerate(('left','right'))if (arm=='both'or side==arm)and side not in context.get('disabled_grippers',())]
        passed=all(limits[i][0]<=observed['minimum'][i]<=observed['maximum'][i]<=limits[i][1]for i in enabled)
        return observed,dict(bounds=deepcopy(limits),enabled_indices=enabled),'mm','lower<=command<=upper','REJECT_PLAN',True,passed,None
    if rule in ('G18','G19','G20'):
        limits=context['joint_limits']
        if rule=='G18'and isinstance(value,dict):
            if not {'minimum','maximum'}<=value.keys() or value.keys()-{'minimum','maximum','active_arm'}:
                raise ValueError('declared command extrema required')
            arm=value.get('active_arm','both')
            if arm not in ('left','right','both')or arm!='both'and len(limits)!=12:raise ValueError('declared command arm required')
            observed={key:_guard_vector(value[key],len(limits),'command extrema')for key in ('minimum','maximum')}
            if 'active_arm' in value:observed['active_arm']=arm
            indices=range(6)if arm=='left'else range(6,12)if arm=='right'else range(len(limits))
            passed=all(limits[i][0]<=observed['minimum'][i]<=observed['maximum'][i]<=limits[i][1] for i in indices)
            return observed,dict(bounds=deepcopy(limits),allowance_rad=0.),'rad','lower<=minimum<=maximum<=upper','REJECT_PLAN',True,passed,None
        observed=_guard_vector(value,len(limits),'joint positions')
        margin=0.
        if rule!='G18':
            from .joint_bounds import position_noise_rad,feedback_position_margin_rad
            margin=(position_noise_rad if rule=='G19'else feedback_position_margin_rad)(context['environment'])
        epsilon=0. if rule=='G18'else 1e-12
        passed=all(low-margin-epsilon<=v<=high+margin+epsilon for v,(low,high)in zip(observed,limits))
        limit=dict(bounds=deepcopy(limits),allowance_rad=margin)
        return observed,limit,'rad','lower-margin<=q<=upper+margin','FAULT_STOP'if rule=='G20'else'REJECT_PLAN',True,passed,None
    if rule in ('G28','G29'):
        limit=context[('integral_feedback_timeout_s'if phase=='feedback_integral'else'feedback_timeout_s')if rule=='G28'else'starvation_margin_s']
        if rule=='G29'and isinstance(value,dict):
            if set(value)!={'current_time','queued_until'}:raise ValueError('declared queue timestamps required')
            observed={key:_guard_number(stamp,'queue timestamp')for key,stamp in value.items()}
            observed['remaining_s']=observed['queued_until']-observed['current_time']
            passed=observed['current_time']<=observed['queued_until']-limit
        else:
            observed=_guard_number(value,'feedback age'if rule=='G28'else'queue lead')
            passed=0<=observed<=limit if rule=='G28'else observed>=limit
        return observed,limit,'s','0<=age<=limit'if rule=='G28'else'lead>=limit','FAULT_STOP',True,passed,None
    fields={'joint_errors_rad','jaw_errors_mm'}|({'exceeded_for_s'}if rule=='G25'else set())
    if not isinstance(value,dict) or set(value)!=fields:raise ValueError('declared tracking/arrival observation required')
    observed=dict(joint_errors_rad=_guard_vector(value['joint_errors_rad'],12,'joint errors'),
                  jaw_errors_mm=_guard_vector(value['jaw_errors_mm'],2,'jaw errors'))
    settings=parameters['tracking'if rule=='G25'else'arrival']
    joint_limits=[settings['joint_rad']]*12 if rule=='G25'else settings['tolerances_rad']
    joints=[i for i,(v,limit)in enumerate(zip(observed['joint_errors_rad'],joint_limits))if abs(v)>limit]
    jaws=[i for i,v in enumerate(observed['jaw_errors_mm'])if abs(v)>settings['jaw_mm']]
    exceeded=bool(joints or jaws)
    observed.update(exceeded=exceeded,exceeded_joint_indices=joints,exceeded_jaw_indices=jaws)
    limit=dict(joint_rad=list(joint_limits),jaw_mm=settings['jaw_mm'])
    if rule=='G25':
        elapsed=_guard_number(value['exceeded_for_s'],'tracking elapsed')
        if elapsed<0:raise ValueError('nonnegative tracking elapsed required')
        observed['exceeded_for_s']=elapsed;limit['duration_s']=settings['duration_s']
        passed=not exceeded
        epsilon=1e-12 if phase=='tracking_frozen'else 0.
        if exceeded and elapsed+epsilon<settings['duration_s']:override='observe'
    else:passed=not exceeded
    return observed,limit,'rad / mm','per-axis absolute errors within policy','FAULT_STOP'if rule=='G25'else'WAIT',settings['enabled'],passed,override


def defaults(business='calibration', arrival_tolerances_rad=None):
    enabled = business == 'calibration'
    return dict(arrival_enabled=enabled,
        arrival_tolerances_rad=list(arrival_tolerances_rad or [DEFAULT_JOINT_TOLERANCE_RAD] * 12), arrival_jaw_mm=2.,
        tracking_enabled=enabled, tracking_joint_rad=.08, tracking_jaw_mm=4., tracking_duration_s=.2,
        fake_error=dict(enabled=False, joint_bias_max_rad=0., seed=0),
        speed_protection=dict(enabled=True, limit_rad_s=DEFAULT_MEASURED_SPEED_LIMIT_RAD_S))


def validate(value, *, environment=None, allow_legacy=False):
    if allow_legacy and isinstance(value,dict) and set(value)==set(defaults())-{'speed_protection'}:
        value=dict(value,speed_protection=defaults()['speed_protection'])
    if not isinstance(value, dict) or set(value) != set(defaults()):
        raise ValueError('complete execution settings required')
    result = deepcopy(value)
    for key in ('arrival_enabled', 'tracking_enabled'):
        if type(result[key]) is not bool:
            raise ValueError(key + ' must be boolean')
    def number(value, name, *, zero=False):
        if type(value) not in (int, float) or not math.isfinite(value) or (value < 0 if zero else value <= 0):
            raise ValueError(name + ' must be finite and ' + ('nonnegative' if zero else 'positive'))
        return float(value)
    values = result['arrival_tolerances_rad']
    if not isinstance(values, (list, tuple)) or len(values) != 12:
        raise ValueError('12 arrival tolerances required, left J1..J6 then right J1..J6')
    result['arrival_tolerances_rad'] = [number(v, 'arrival tolerance') for v in values]
    for key in ('arrival_jaw_mm', 'tracking_joint_rad', 'tracking_jaw_mm', 'tracking_duration_s'):
        result[key] = number(result[key], key, zero=key == 'tracking_duration_s')
    noise = result['fake_error']
    if not isinstance(noise, dict) or set(noise) != {'enabled', 'joint_bias_max_rad', 'seed'}:
        raise ValueError('complete fake error configuration required')
    if type(noise['enabled']) is not bool or type(noise['seed']) is not int or not 0 <= noise['seed'] < 2**32:
        raise ValueError('fake error requires boolean enabled and uint32 seed')
    noise['joint_bias_max_rad'] = number(noise['joint_bias_max_rad'], 'joint_bias_max_rad', zero=True)
    if noise['joint_bias_max_rad'] > .05:
        raise ValueError('fake joint bias must be at most 0.05 rad')
    if environment == 'physical' and noise['enabled']:
        raise ValueError('random actuation error is available only in fake')
    speed=result['speed_protection']
    if not isinstance(speed,dict) or set(speed)!={'enabled','limit_rad_s'} or type(speed['enabled']) is not bool:
        raise ValueError('speed_protection requires enabled boolean and limit_rad_s')
    speed['limit_rad_s']=number(speed['limit_rad_s'],'speed_protection.limit_rad_s')
    return result


class NativeCommandMismatchError(ValueError):
    def __init__(self, message, diagnostic):
        self.diagnostic = deepcopy(diagnostic)
        super().__init__(message)


class JointSpeedProtectionError(ValueError):
    def __init__(self, observation):
        self.observation=observation
        super().__init__('关节速度保护触发：'+observation['joint_label']+
            f" 反馈 {observation['velocity_rad_s']:+.6f} rad/s，绝对值超过 {observation['limit_rad_s']:.6g} rad/s；本次执行已中断，请核对状态后中断恢复")


def measured_joint_speed(feedback, settings):
    """One fresh sample, no filtering/debounce; jaws never share the rad/s limit."""
    velocity=feedback.get('qvel')
    if not isinstance(velocity,(list,tuple)) or len(velocity)!=14 or any(
            type(v) not in (int,float) or not math.isfinite(v) for v in velocity):
        raise ValueError('speed protection requires 14 finite feedback velocities')
    slot=max((*range(6),*range(7,13)),key=lambda i:abs(velocity[i]))
    side='left' if slot<7 else 'right';joint=slot%7+1
    return dict(enabled=settings['enabled'],limit_rad_s=settings['limit_rad_s'],joint=f'{side}_J{joint}',
        joint_label=('左' if side=='left' else '右')+f' J{joint}',velocity_rad_s=velocity[slot],
        peak_rad_s=abs(velocity[slot]),sample_time_s=feedback['sample_time_s'],sample_wall_ns=feedback.get('sample_wall_ns'))


def study_defaults():
    from .target_interpreter import gains
    return dict(gains={side:dict(zip(('kp','kd'),gains('x5_control50_damping_v1',side)))for side in('left','right')},
        integral_enabled=True,alignment_speed_rad_s=.1,home_speed_rad_s=.1,jaw_speed_mm_s=10.,tcp_tolerance_mm=1.)


def study_caps():
    caps=study_defaults()
    # Approved static model study: P400 on all twelve rotary joints.
    for side in ('left','right'):
        caps['gains'][side]['kp']=[max(400.,v)for v in caps['gains'][side]['kp']]
    # User-approved left J2 response sweep; the default remains 210.
    caps['gains']['left']['kp'][1]=490.
    # User-approved left J3 research ceiling; the default remains 210.
    caps['gains']['left']['kp'][2]=400.
    return caps


def study_presets():
    from .target_interpreter import gains
    baseline=study_defaults()
    baseline['gains']={side:dict(zip(('kp','kd'),gains('normal_X5',side)))for side in ('left','right')}
    baseline['integral_gains']={side:[0.]*6 for side in ('left','right')}
    result=[dict(id='base-v1',label='base-v1 · SDK默认PD / I=0',source_profile='normal_X5',settings=baseline)]
    for identity,profile in (('v8','x5_control50_pd30_feedback50_v1'),('v10','x5_control50_damping_v1')):
        settings=study_defaults()
        settings['gains']={side:dict(zip(('kp','kd'),gains(profile,side)))for side in('left','right')}
        result.append(dict(id=identity,label=identity.upper(),source_profile=profile,settings=settings))
    return result


def validate_integral_gains(value):
    from .motion_reference import INTEGRAL_ERROR_GAINS
    if not isinstance(value,dict)or set(value)!={'left','right'}:
        raise ValueError('integral_gains requires left/right six-joint SDK coefficients')
    result=deepcopy(value)
    for side in ('left','right'):
        values=result[side];caps=[v*(8 if side=='left'else 1)for v in INTEGRAL_ERROR_GAINS]
        if not isinstance(values,list)or len(values)!=6:raise ValueError('six '+side+' integral gains required')
        for j,(v,cap)in enumerate(zip(values,caps)):
            if type(v)not in(int,float)or not math.isfinite(v)or not 0<=v<=cap:
                raise ValueError(f'{side} J{j+1} integral gain must be finite, >=0 and <= {cap:g} SDK/(rad*s)')
    return result


def validate_study(value):
    caps=study_caps()
    if not isinstance(value,dict)or set(value)-{'integral_gains'}!=set(caps)or type(value['integral_enabled'])is not bool:
        raise ValueError('complete versioned study configuration required')
    result=deepcopy(value)
    if 'integral_gains'in result:result['integral_gains']=validate_integral_gains(result['integral_gains'])
    if not isinstance(result['gains'],dict)or set(result['gains'])!={'left','right'}:raise ValueError('left/right study gains required')
    for side in('left','right'):
        arm=result['gains'][side]
        if not isinstance(arm,dict)or set(arm)!={'kp','kd'}:raise ValueError('per-joint kp/kd required')
        for name in('kp','kd'):
            values=arm[name]
            if not isinstance(values,list)or len(values)!=6:raise ValueError('six '+side+' '+name+' values required')
            for i,(v,cap)in enumerate(zip(values,caps['gains'][side][name])):
                if type(v)not in(int,float)or not math.isfinite(v)or v>cap or (v<0 if name=='kd'else v<=0):
                    lower='>=0'if name=='kd'else '>0'
                    raise ValueError(f'{side} J{i+1} {name} must be finite, {lower} and <= {cap:g}')
    for name,cap in(('alignment_speed_rad_s',.1),('home_speed_rad_s',.1),('jaw_speed_mm_s',40.),('tcp_tolerance_mm',10.)):
        v=result[name]
        if type(v)not in(int,float)or not math.isfinite(v)or not 0<v<=cap:raise ValueError(f'{name} must be finite, >0 and <= {cap:g}')
    return result
