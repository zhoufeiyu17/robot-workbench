"""Shared interpretation of published facts; no IO or control side effects."""
from copy import deepcopy


CONTROL_STATES = {'READY', 'STOPPED_DAMPING_HOLD', 'MANUAL_TAKEOVER'}


def control_healthy(value):
    value = value or {}
    age = value.get('sample_age_s')
    return (value.get('connected') is True and value.get('feedback_healthy') is True
        and value.get('stationary') is True and value.get('automatic_enabled') is False
        and type(age) in (int, float) and 0 <= age <= .1 and bool(value.get('runtime_nonce'))
        and value.get('control_coherence', {}).get('consistent', True) is True
        and str(value.get('state', '')).upper() in CONTROL_STATES)


def model_readiness(binding, service, *, required=True):
    """A cached binding never overrides the selected worker's current health."""
    binding, service = binding or {}, service or {}
    selected = binding.get('selected_model_id') or binding.get('id')
    loaded = service.get('loaded')
    warm = (service.get('warmup') or {}).get('completed')
    process_ready = (loaded is True and service.get('status') == 'ready'
        and warm is True and service.get('model_id') == selected)
    if (service.get('fact')or{}).get('valid') is False:process_ready=False
    if 'worker_pid' in service and (type(service['worker_pid']) is not int or service['worker_pid']<=0):process_ready=False
    for key in ('service_pid','worker_pid'):
        if binding.get('bound_'+key) is not None and service.get(key)!=binding['bound_'+key]:
            process_ready=False
    bound_instance=binding.get('bound_model_instance')
    if bound_instance is not None and (service.get('fact')or{}).get('instance_id')!=bound_instance:
        process_ready=False
    bound = binding.get('model_ready', binding.get('ready')) is True
    if not required:
        ready=None if binding.get('status') in (None,'unknown') else bound
        return dict(ready=ready, process_ready=None, workspace_bound=bound, reason=None)
    if loaded is None:
        reason=service.get('error') or '所选模型进程状态尚未核验'
        ready=None
    elif not process_ready:
        reason=service.get('error') or '所选模型进程未就绪或身份已变化'
        ready=False
    else:
        reason=None if bound else '模型已加载，当前工作区尚未绑定'
        ready=bound
    return dict(ready=ready, process_ready=process_ready,
        workspace_bound=bound, reason=reason, selected_model_id=selected,
        service_model_id=service.get('model_id'))


def fact(domain, producer, instance, revision, sampled_wall_ns, *, dependencies=None, valid=True, reason=None):
    return dict(schema='workbench.domain_fact.v1', domain=domain, producer=producer,
        instance_id=instance, revision=revision, sampled_wall_ns=sampled_wall_ns,
        dependencies=deepcopy(dependencies or {}), valid=valid, reason=reason)
