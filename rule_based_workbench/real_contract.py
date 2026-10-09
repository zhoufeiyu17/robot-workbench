"""Physical observation binding. The binding comes from a trusted local receipt,
never from an operator HTTP payload. Missing calibration cannot be inferred from
simulation. An observation producer must seal its complete source digest here.
"""
from integration import (_digest, _validate_scene, _validate_timeline, validate_profile,
                         SIM_PLANNER_ID, SIM_POLICY_ID)


def validate_real_source(source, binding):
    if not isinstance(binding, dict) or binding.get('calibration_accepted') is not True:
        raise ValueError('accepted real calibration receipt required')
    if not isinstance(source, dict) or source.get('schema_version') != 'physical_collection_source.v1':
        raise ValueError('physical_collection_source.v1 required; simulated source is forbidden')
    if (source.get('lane') not in (None, 'physical-real') or 'instance_id' in source or 'gpu' in source
            or source.get('adapter') not in (None, 'foundationpose')):
        raise ValueError('physical source must not carry simulated lane provenance')
    if source.get('evidence_source') != 'physical_rgbd' or binding.get('evidence_source') != 'physical_rgbd':
        raise ValueError('physical RGB-D provenance required')
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
    if (profile['source'].get('simulation_only') is True
            or profile.get('physical_calibration', {}).get('simulation_only') is True):
        raise ValueError('fixture calibration cannot authorize a physical plan')
    _validate_scene(source.get('scene'))
    if binding.get('calibration_profile_digest') != _digest(source['calibration_profile']):
        raise ValueError('calibration digest differs from accepted real calibration')
    if binding.get('source_digest') != _digest(source):
        raise ValueError('observation differs from trusted physical acquisition receipt')
    if source.get('initial_state') != binding.get('initial_state'):
        raise ValueError('initial state differs from trusted physical acquisition receipt')
    if (not isinstance(source.get('run_id'),str) or not source['run_id'].strip()
            or type(source.get('sequence')) is not int or source['sequence'] < 0):
        raise ValueError('physical source run and sequence required')
    return source


def validate_real_plan(plan, source, binding):
    validate_real_source(source,binding)
    from integration import CUROBO_PLANNER_ID
    if not isinstance(plan,dict) or plan.get('planner_id') not in (SIM_PLANNER_ID,CUROBO_PLANNER_ID) or plan.get('policy_id') != SIM_POLICY_ID:
        raise ValueError('reviewed simulation planner and policy required')
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
