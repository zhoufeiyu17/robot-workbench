"""Physical observation binding. The binding comes from a trusted local receipt,
never from an operator HTTP payload. Missing calibration cannot be inferred from
simulation. An observation producer must seal its complete source digest here.
"""
from integration import (_digest, _validate_scene, _validate_timeline, validate_profile,
                         SIM_PLANNER_ID, SIM_POLICY_ID)


def namespace(environment='physical'):
    """Provenance separation only; fake does not relax geometry, pairing or clock checks."""
    if environment not in ('physical', 'fake'):
        raise ValueError('trusted device environment must be physical or fake')
    fake = environment == 'fake'
    return dict(environment=environment, evidence='fake_device_rgbd' if fake else 'physical_rgbd',
        source_schema='fake_device_collection_source.v1' if fake else 'physical_collection_source.v1',
        raw_schema='fake_device_raw_dataset.v1' if fake else 'physical_raw_dataset.v1',
        playback_schema='fake_device_workbench_playback.v1' if fake else 'physical_workbench_playback.v1',
        target_playback_schema='fake_device_target_playback.v1' if fake else 'physical_target_playback.v1',
        lane='fake-device' if fake else 'physical-real', mode='fake_device' if fake else 'real_robot',
        label='fake 真机（仿真）' if fake else '真机')


def is_device_source(source):
    """Coordinate routing only; this is not evidence validation or authorization."""
    return source.get('schema_version') in ('physical_collection_source.v1', 'fake_device_collection_source.v1')


def validate_namespace(value, environment='physical'):
    spec = namespace(environment)
    if not isinstance(value, dict) or value.get('evidence_source') != spec['evidence']:
        raise ValueError('device evidence does not match trusted '+environment+' environment')
    if environment == 'fake':
        if value.get('environment') != 'fake' or value.get('simulation_only') is not True:
            raise ValueError('fake-device evidence requires explicit fake environment and simulation_only')
    elif value.get('environment', 'physical') != 'physical' or value.get('simulation_only') is True:
        raise ValueError('fake evidence cannot authorize physical hardware')
    return spec


def owner_matches_environment(owner, environment='physical'):
    namespace(environment)
    if environment == 'fake':
        return (owner.get('mode') == 'simulation' and owner.get('environment') == 'fake'
                and owner.get('evidence_source') in ('physx', 'fake', 'simulated_owner', 'fake_device'))
    return (owner.get('mode') == 'hardware' and owner.get('environment', 'physical') == 'physical'
            and owner.get('simulation_only') is not True
            and owner.get('evidence_source', 'physical_hardware') == 'physical_hardware')


def validate_real_source(source, binding, *, environment='physical'):
    spec = namespace(environment)
    if not isinstance(binding, dict) or binding.get('calibration_accepted') is not True:
        raise ValueError('accepted real calibration receipt required')
    if not isinstance(source, dict) or source.get('schema_version') != spec['source_schema']:
        raise ValueError(spec['source_schema']+' required; cross-environment source is forbidden')
    if (source.get('lane') not in ((None, spec['lane']) if environment == 'physical' else (spec['lane'],)) or 'instance_id' in source or 'gpu' in source
            or source.get('adapter') not in (None, 'foundationpose')):
        raise ValueError('physical source must not carry simulated lane provenance')
    validate_namespace(source, environment)
    validate_namespace(binding, environment)
    robot = source.get('robot_id')
    cameras = source.get('camera_serials')
    if not isinstance(robot, str) or not robot.strip() or robot != binding.get('robot_id'):
        raise ValueError('robot identity differs from trusted binding')
    if (not isinstance(cameras, dict) or set(cameras) != {'head','left','right'}
            or any(not isinstance(v,str) or not v.strip() for v in cameras.values())
            or len(set(cameras.values())) != 3 or cameras != binding.get('camera_serials')):
        raise ValueError('camera serials differ from trusted binding')
    validate_profile(source.get('calibration_profile'))
    profile = source['calibration_profile']
    if environment == 'physical' and (profile['source'].get('simulation_only') is True
            or profile.get('physical_calibration', {}).get('simulation_only') is True):
        raise ValueError('fixture calibration cannot authorize a physical plan')
    if environment == 'fake' and profile['source'].get('simulation_only') is not True:
        raise ValueError('fake-device requires an explicit simulation-only calibration snapshot')
    _validate_scene(source.get('scene'))
    if binding.get('calibration_profile_digest') != _digest(source['calibration_profile']):
        raise ValueError('calibration digest differs from accepted real calibration')
    if binding.get('source_digest') != _digest(source):
        raise ValueError('observation differs from trusted physical acquisition receipt')
    if source.get('initial_state') != binding.get('initial_state'):
        raise ValueError('initial state differs from trusted physical acquisition receipt')
    if environment=='physical':
        from robot_workbench.sample_clock import clock_for_source
        for value in (source,binding,source.get('initial_state') or {}):
            clock_for_source(value,'physical')
    if environment=='fake':
        from joint_space import validate_initial_state
        from robot_workbench.sample_clock import clock_for_source
        clock=clock_for_source(source,environment)
        if binding.get('clock')!=clock or source.get('initial_state',{}).get('clock')!=clock:
            raise ValueError('source, binding and measured initial state clocks differ')
        validate_initial_state(source.get('initial_state'),profile)
    if (not isinstance(source.get('run_id'),str) or not source['run_id'].strip()
            or type(source.get('sequence')) is not int or source['sequence'] < 0):
        raise ValueError('physical source run and sequence required')
    return source


def validate_real_plan(plan, source, binding, *, environment='physical'):
    validate_real_source(source,binding,environment=environment)
    from integration import CUROBO_PLANNER_ID
    if not isinstance(plan,dict) or plan.get('planner_id') not in (SIM_PLANNER_ID,CUROBO_PLANNER_ID) or plan.get('policy_id') != SIM_POLICY_ID:
        raise ValueError('reviewed simulation planner and policy required')
    if environment == 'fake':
        if plan.get('environment') != 'fake' or plan.get('simulation_only') is not True:
            raise ValueError('fake-device plan requires explicit fake environment and simulation_only')
    elif plan.get('environment', 'physical') != 'physical' or plan.get('simulation_only') is True:
        raise ValueError('fake-device plan cannot authorize physical hardware')
    expected = dict(source_run_id=source['run_id'], source_sequence=source['sequence'],
                    scene_digest=source['scene']['scene_digest'],
                    calibration_profile_digest=_digest(source['calibration_profile']))
    if type(plan.get('source_sequence')) is not int or any(plan.get(k) != v for k,v in expected.items()):
        raise ValueError('plan differs from physical source identity')
    if not isinstance(plan.get('run_id'),str) or not plan['run_id'].strip():
        raise ValueError('plan run identity required')
    _validate_timeline(plan.get('ticks'),source['calibration_profile']['gripper']['max_opening_mm'])
    if plan.get('timeline_digest') != _digest(plan['ticks']):
        raise ValueError('frozen timeline digest differs')
    from joint_space import validate
    validate(plan, source)
    if plan.get('planner_id') == CUROBO_PLANNER_ID:
        from physical_motiongen import validate_evidence
        validate_evidence(plan,source)
