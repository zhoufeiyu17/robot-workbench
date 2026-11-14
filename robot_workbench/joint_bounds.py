"""Position-boundary noise only; never a tracking, settling or collision policy."""
import math
from .execution_settings import PHYSICAL_FEEDBACK_MARGIN_RAD,numeric_tolerance


class JointPositionBoundaryError(ValueError):
    """A measured boundary violation is distinct from an invalid command."""
    def __init__(self,index,value,lower,upper,tolerance,count,context):
        joint=f'{"left" if index<6 else "right"}_J{index%6+1}' if count==12 else f'joint_{index}'
        excess=max(lower-value,value-upper)
        self.details=dict(joint=joint,index=index,context=context,value_rad=value,
            lower_rad=lower,upper_rad=upper,excess_rad=excess,allowance_rad=tolerance)
        super().__init__(f'{joint} {context}={value:.8g} rad, limits=[{lower:.8g}, {upper:.8g}] rad, '
            f'excess={excess:.8g} rad; exceeds {tolerance:g} rad position-boundary noise allowance')


def position_noise_rad(environment):
    if environment not in ('fake', 'physical'):
        raise ValueError('explicit fake/physical position-noise environment required')
    return .005 if environment == 'fake' else .01


def project_joint_positions(values, limits, *, environment):
    """Return legal positions and explicit corrections, preserving the input."""
    return _project(values,limits,position_noise_rad(environment))


def feedback_position_margin_rad(environment):
    """Observation validity only; never expands the legal command interval."""
    return PHYSICAL_FEEDBACK_MARGIN_RAD if environment=='physical' else position_noise_rad(environment)


def inspect_joint_feedback(values,limits,*,environment):
    """Return bounded diagnostic values; these are not command authorization."""
    return _project(values,limits,feedback_position_margin_rad(environment),context='feedback')


def _project(values,limits,tolerance,context='command'):
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
        if abs(bounded - value) > tolerance + numeric_tolerance(max(abs(lower),abs(upper)),'rad'):
            raise JointPositionBoundaryError(index,value,lower,upper,tolerance,len(values),context)
        projected.append(bounded)
        if bounded != value:
            changes.append(dict(index=index, original_rad=value, bounded_rad=bounded,
                                delta_rad=bounded-value))
    return projected, changes
