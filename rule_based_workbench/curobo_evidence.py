"""Bind loaded planning code identity; offline receipts cannot authorize execution."""
from copy import deepcopy
import hashlib
import importlib
from pathlib import Path


def backend_identity(backend, *, relative_geometry=False):
    if backend not in ('native', 'batch-cpu', 'curobo'):
        raise ValueError('unknown planning backend')
    result = dict(backend={'curobo': 'curobo_world_broadphase_native_exact',
                          'batch-cpu': 'batch_cpu_broadphase_native_exact'}.get(backend, backend), gpu_ik=False)
    modules = []
    if backend == 'curobo':
        version = importlib.import_module('curobo').__version__
        if version != '0.7.8':
            raise ValueError('cuRobo evidence requires actual version 0.7.8')
        result['curobo_version'] = version
        modules = ['curobo_collision', 'curobo_whole_geometry', 'curobo_relative_geometry']
    elif backend == 'batch-cpu':
        modules = ['curobo_whole_geometry']
    if relative_geometry and 'curobo_relative_geometry' not in modules:
        modules.append('curobo_relative_geometry')
    if modules:
        result['relative_geometry'] = relative_geometry
        # Resolve the loaded modules, not similarly named files next to this helper.
        paths = [Path(importlib.import_module(name).__file__).resolve(strict=True) for name in modules]
        result['code_sha256'] = {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in paths}
    return result


def stamp_plan(plan, identity, *, offline=False):
    """Copy only after build/retime validation; retain every target and its digest."""
    result = deepcopy(plan)
    receipt = result.get('planning_validation', {})
    if (receipt.get('schema_version') != 'geometric_path_validation.v1' or receipt.get('passed') is not True
            or result.get('simulation') != dict(validated=False, outcome='not_run')):
        raise ValueError('backend stamping requires a passed geometric-only plan')
    result['planning_backend'] = deepcopy(identity)
    receipt['backend'] = deepcopy(identity)
    if offline:
        result['offline_candidate'] = receipt['offline_candidate'] = True
        receipt['schema_version'] = 'offline_geometric_path_validation.v1'
    return result


def self_check():
    from frozen_executor import validate_plan_evidence
    from integration import _digest
    source = {'fixture': 'offline-binding'}
    ticks = [dict(tick=0, time_s=0., target={'fixture': 'unchanged'})]
    plan = dict(ticks=ticks, timeline_digest=_digest(ticks), simulation=dict(validated=False, outcome='not_run'),
                planning_validation=dict(schema_version='geometric_path_validation.v1', passed=True,
                    timeline_digest=_digest(ticks), source_digest=_digest(source),
                    max_sample_step_rad=.01, collision_coverage=['base_link', 'left', 'right']))
    before = deepcopy(plan)
    identity = backend_identity('native')
    formal = stamp_plan(plan, identity)
    validate_plan_evidence(formal, source)
    offline = stamp_plan(plan, identity, offline=True)
    try:
        validate_plan_evidence(offline, source)
    except ValueError:
        pass
    else:
        raise AssertionError('offline geometry receipt authorized execution')
    assert plan == before
    assert formal['ticks'] == offline['ticks'] == plan['ticks']
    assert formal['timeline_digest'] == offline['timeline_digest'] == plan['timeline_digest']
    assert formal['planning_validation']['backend'] == offline['planning_validation']['backend'] == identity
    print('backend identity/formal acceptance/offline rejection checks passed; no CUDA imported')


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--self-check', action='store_true', required=True)
    parser.parse_args()
    self_check()
