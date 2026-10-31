"""Execution error policy and independent measured joint-speed protection."""
from copy import deepcopy
import math


def defaults(business='calibration', arrival_tolerances_rad=None):
    enabled = business == 'calibration'
    return dict(arrival_enabled=enabled,
        arrival_tolerances_rad=list(arrival_tolerances_rad or [.01] * 12), arrival_jaw_mm=2.,
        tracking_enabled=enabled, tracking_joint_rad=.08, tracking_jaw_mm=4., tracking_duration_s=.2,
        fake_error=dict(enabled=False, joint_bias_max_rad=0., seed=0),
        speed_protection=dict(enabled=True, limit_rad_s=.8))


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
