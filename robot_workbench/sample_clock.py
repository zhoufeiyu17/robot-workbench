"""Shared source-clock validation; transport wall time never advances simulation."""
from bisect import bisect_left
from math import isfinite


def clock_spec(environment='physical', *, clock_domain=None, source_clock_id=None):
    if environment not in ('physical', 'fake'):
        raise ValueError('explicit physical/fake clock environment required')
    domain = clock_domain or ('simulation' if environment == 'fake' else 'host_wall')
    if environment == 'fake':
        if domain != 'simulation' or not isinstance(source_clock_id, str) or not 0 < len(source_clock_id.strip()) <= 200:
            raise ValueError('fake source requires simulation domain and unique source_clock_id')
    elif domain != 'host_wall' or source_clock_id is not None:
        raise ValueError('physical pairing retains the host wall clock')
    return dict(environment=environment, clock_domain=domain, source_clock_id=source_clock_id)


def clock_for_source(source, environment='physical'):
    value = source.get('clock')
    if value is None and environment == 'physical':
        return clock_spec()
    if (not isinstance(value, dict) or set(value) != {'environment', 'clock_domain', 'source_clock_id'}
            or value.get('environment') != environment):
        raise ValueError('sealed source clock must match the trusted environment')
    result=clock_spec(**value)
    if result!=value:
        raise ValueError('sealed source clock domain must be explicit')
    return result


def sample_stamp(sample, clock, *, wall_key='host_receive_ns', source_key='source_time_ns', require_step=True):
    """Return (matching ns, physics step); keep the caller's real wall stamp intact."""
    clock = clock_spec(**clock)
    if not isinstance(sample, dict):
        raise ValueError('timestamped sample object required')
    wall = sample.get(wall_key)
    if type(wall) is not int or wall <= 0:
        raise ValueError('positive actual host wall timestamp required: '+wall_key)
    if sample.get('environment', clock['environment']) != clock['environment']:
        raise ValueError('sample belongs to another environment')
    if clock['environment'] == 'physical':
        if (sample.get('clock_domain', 'host_wall') != 'host_wall' or sample.get('source_clock_id') is not None
                or sample.get('simulation_only') is True):
            raise ValueError('simulation timestamps cannot authorize physical pairing')
        return wall, None
    if (sample.get('clock_domain') != 'simulation' or sample.get('source_clock_id') != clock['source_clock_id']
            or sample.get('simulation_only', True) is not True):
        raise ValueError('sample source clock differs from the sealed simulation instance')
    stamp, step = sample.get(source_key), sample.get('physics_step')
    if type(stamp) is not int or stamp < 0 or (require_step and (type(step) is not int or step < 0)):
        raise ValueError('nonnegative simulation source_time_ns and physics_step required')
    if step is not None and (type(step) is not int or step < 0):
        raise ValueError('invalid simulation physics_step')
    for key in ('color_physics_step','depth_physics_step'):
        if key in sample and sample[key]!=step:
            raise ValueError('RGB-D components belong to different physics steps')
    if 'device_timestamp_ms' in sample or 'depth_device_timestamp_ms' in sample:
        if sample.get('timestamp_domain')!='simulation':
            raise ValueError('fake camera device timestamps must use simulation domain')
        for key in ('device_timestamp_ms','depth_device_timestamp_ms'):
            value=sample.get(key)
            if type(value) not in (int,float) or not isfinite(value) or abs(value-stamp/1e6)>1e-6:
                raise ValueError('RGB-D device timestamp differs from its source physics step')
    return stamp, step


def assert_progress(previous, current, *, allow_same=False):
    """Reject resets and inconsistent step/time reuse; exact duplicates are opt-in."""
    if previous is None:
        return current
    old_ns, old_step = previous
    ns, step = current
    if ns < old_ns or (ns == old_ns and not allow_same):
        raise ValueError('source clock repeated or regressed')
    if (old_step is None) != (step is None):
        raise ValueError('source clock step domain changed')
    if step is not None and (step < old_step or (step == old_step) != (ns == old_ns)):
        raise ValueError('physics_step and source time must advance together')
    return current


def bracket_samples(stamps, target):
    """Return exact/bracketing indices; a bracket is not an exact-step measurement."""
    index = bisect_left(stamps, target[0], key=lambda row: row[0])
    if index < len(stamps) and stamps[index][0] == target[0]:
        if stamps[index][1] != target[1]:
            raise ValueError('equal source time is bound to a different physics step')
        return index, index
    if index == 0 or index == len(stamps):
        raise ValueError('source frame is not bracketed by feedback')
    left, right = stamps[index-1], stamps[index]
    assert_progress(left, target)
    assert_progress(target, right)
    return index-1, index


def owner_identity(sample, *, pid_key='owner_pid', require_available=False):
    if require_available and sample.get('owner_available') is not True:
        raise ValueError('formal frame requires its sampling-time Owner identity')
    pid, epoch, nonce = sample.get(pid_key), sample.get('command_epoch'), sample.get('runtime_nonce')
    if (type(pid) is not int or pid <= 0 or type(epoch) is not int or epoch < 0
            or not isinstance(nonce,str) or not nonce.strip()):
        raise ValueError('sampling-time Owner PID, command epoch and runtime nonce required')
    return dict(owner_pid=pid,command_epoch=epoch,runtime_nonce=nonce)
