"""Position-boundary noise only; never a tracking, settling or collision policy."""
import math


def position_noise_rad(environment):
    if environment not in ('fake', 'physical'):
        raise ValueError('explicit fake/physical position-noise environment required')
    return .005 if environment == 'fake' else .01


def project_joint_positions(values, limits, *, environment):
    """Return legal positions and explicit corrections, preserving the input."""
    tolerance = position_noise_rad(environment)
    if len(values) != len(limits) or not values:
        raise ValueError('joint positions and limits must have equal nonzero length')
    projected, changes = [], []
    for index, (value, pair) in enumerate(zip(values, limits)):
        if len(pair) != 2:
            raise ValueError('joint limit must contain lower and upper positions')
        lower, upper = pair
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
               for v in (value, lower, upper)) or lower > upper:
            raise ValueError('finite ordered joint position limits required')
        bounded = min(upper, max(lower, value))
        if abs(bounded - value) > tolerance + 1e-12:
            raise ValueError(f'joint {index} exceeds {tolerance:g} rad position-boundary noise allowance')
        projected.append(bounded)
        if bounded != value:
            changes.append(dict(index=index, original_rad=value, bounded_rad=bounded,
                                delta_rad=bounded-value))
    return projected, changes
