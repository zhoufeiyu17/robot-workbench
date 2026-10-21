"""Execution error policy, separate from command limits and feedback freshness."""
from copy import deepcopy
import math


def defaults(business='calibration', arrival_tolerances_rad=None):
    enabled = business == 'calibration'
    return dict(arrival_enabled=enabled,
        arrival_tolerances_rad=list(arrival_tolerances_rad or [.01] * 12), arrival_jaw_mm=2.,
        tracking_enabled=enabled, tracking_joint_rad=.08, tracking_jaw_mm=4., tracking_duration_s=.2,
        fake_error=dict(enabled=False, joint_bias_max_rad=0., seed=0))


def validate(value, *, environment=None):
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
    return result
