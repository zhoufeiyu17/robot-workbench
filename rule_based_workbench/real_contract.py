"""Physical observation binding. The binding comes from a trusted local receipt,
never from an operator HTTP payload. Missing calibration cannot be inferred from
simulation. An observation producer must seal its complete source digest here.
"""
from integration import (_digest, _validate_scene, _validate_timeline, validate_profile,
                         SIM_PLANNER_ID, SIM_POLICY_ID)
import math


def fixed_right_collision_policy(source):
    """One sealed human-assisted request may omit only fixed-right static gates."""
    receipt=source.get('scene',{}).get('source',{}).get('assistance_receipt',{})
    policy=receipt.get('fixed_right_collision_policy')
    if policy is None:return None
    initial=source.get('initial_state',{})
    keys={'schema_version','mode','approved_by','approval_id','request_id',
          'initial_state_digest','right_arm_model_joints'}
    q=policy.get('right_arm_model_joints') if isinstance(policy,dict) else None
    if (not isinstance(policy,dict) or set(policy)!=keys
            or source.get('schema_version')!='physical_collection_source.v1'
            or source.get('environment','physical')!='physical' or source.get('simulation_only') is True
            or source.get('adapter')!='operator_assisted_rgbd'
            or receipt.get('status')!='operator_accepted' or receipt.get('scope')!='single_observation'
            or policy.get('schema_version')!='operator_fixed_right_collision_policy.v1'
            or policy.get('mode')!='skip_fixed_static_checks' or policy.get('approved_by')!='user'
            or not isinstance(policy.get('approval_id'),str) or not policy['approval_id'].strip()
            or not isinstance(policy.get('request_id'),str) or not policy['request_id'].strip()
            or policy.get('request_id')!=receipt.get('request_id')
            or policy.get('initial_state_digest')!=_digest(initial)
            or receipt.get('initial_state_digest')!=_digest(initial)
            or 'right' not in initial.get('disabled_grippers',[]) or initial.get('held_arms')!=['right']
            or source.get('grasp_command') not in [dict(metal_gap_mm=gap,units='metal_inner_gap_mm',controlled_gripper='left_gripper') for gap in (61.,65.)]
            or source.get('grasp_command')!=receipt.get('grasp_command')
            or not isinstance(q,list) or len(q)!=6
            or any(type(v) not in (int,float) or not math.isfinite(v) for v in q)
            or q!=initial.get('model',{}).get('right_arm_joints')):
        raise ValueError('fixed-right static exception requires this approved assisted61/65 request and measured right hold')
    return policy


def fixed_right_hold_reference(source,initial_validation=None):
    """Reproduce only the existing recorded initial position-boundary projection."""
    policy=fixed_right_collision_policy(source)
    if policy is None:return None
    measured=list(policy['right_arm_model_joints']);held=list(measured);changes=[]
    if initial_validation is not None:
        from robot_workbench.joint_bounds import position_noise_rad,project_joint_positions
        frozen=initial_validation.get('simulation_initial_command',{})
        if (frozen.get('schema_version')!='physical_model_initial_command.v1'
                or frozen.get('evidence_source')!='physical_source.initial_state.model'
                or frozen.get('source_digest')!=_digest(source)
                or frozen.get('initial_state_digest')!=_digest(source['initial_state'])
                or frozen.get('target')!=source['initial_state']['model']
                or initial_validation.get('position_noise_environment')!='physical'
                or initial_validation.get('arm_allowance_rad')!=position_noise_rad('physical')):
            raise ValueError('fixed-right hold requires the bound existing initial-position receipt')
        seen=set()
        for row in initial_validation.get('soft_limit_projection',[]):
            joint=row.get('joint')
            if joint not in [f'right_joint1{i}' for i in range(1,7)]:continue
            index=int(joint[-1])-1
            if index in seen or row.get('measured_rad')!=measured[index]:
                raise ValueError('fixed-right boundary correction differs from measured source')
            projected,_=project_joint_positions([measured[index]],[(row.get('lower_rad'),row.get('upper_rad'))],environment='physical')
            if projected[0]!=row.get('projected_rad') or projected[0]==measured[index]:
                raise ValueError('fixed-right boundary correction is not the existing clipping operation')
            held[index]=projected[0];changes.append(dict(row));seen.add(index)
        planned=initial_validation.get('planned_joint_state',{})
        if [planned.get(f'right_joint1{i}') for i in range(1,7)]!=held:
            raise ValueError('fixed-right planned reference differs from recorded boundary projection')
    return dict(measured_model_joints=measured,held_model_joints=held,position_boundary_projection=changes,
                initial_state_digest=_digest(source['initial_state']))


def validate_fixed_right_hold(source,joints,initial_validation=None):
    policy=fixed_right_collision_policy(source)
    reference=fixed_right_hold_reference(source,initial_validation)
    if policy is not None and (not isinstance(joints,(list,tuple)) or len(joints)!=6
            or any(type(a) not in (int,float) or not math.isfinite(a) or abs(a-b)>1e-10
                   for a,b in zip(joints,reference['held_model_joints']))):
        raise ValueError('fixed-right static exception requires unchanged measured right-arm joints')
    return policy


def skipped_fixed_right_acceptance(source,joints,initial_validation=None):
    policy=validate_fixed_right_hold(source,joints,initial_validation)
    if policy is None:return None
    reference=fixed_right_hold_reference(source,initial_validation)
    return {name:dict(status='skipped_by_operator',passed=None,performed=False,
        check=check,source_digest=_digest(source),policy_digest=_digest(policy),
        request_id=policy['request_id'],approval_id=policy['approval_id'],
        right_arm_model_joints=reference['held_model_joints'],right_hold_reference=reference,
        right_hold_reference_digest=_digest(reference))
        for name,check in (('world','fixed_world_check'),('self','fixed_self_check'))}


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
    return source.get('schema_version') in ('physical_collection_source.v1', 'fake_device_collection_source.v1',
                                          'fake_device_cup_return_source.v1', 'physical_cup_maintenance_source.v1')


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
    if isinstance(source,dict) and source.get('schema_version')=='physical_cup_maintenance_source.v1':
        if environment!='physical':raise ValueError('physical maintenance cannot authorize fake execution')
        from physical_cup_maintenance import validate_source
        return validate_source(source,binding)
    replay = environment == 'fake' and isinstance(source, dict) and source.get('schema_version') == 'fake_device_cup_return_source.v1'
    if not isinstance(binding, dict) or binding.get('calibration_accepted') is not True:
        raise ValueError('accepted real calibration receipt required')
    if not isinstance(source, dict) or not replay and source.get('schema_version') != spec['source_schema']:
        raise ValueError(spec['source_schema']+' required; cross-environment source is forbidden')
    if (source.get('lane') not in ((None, spec['lane']) if environment == 'physical' else (spec['lane'],)) or 'instance_id' in source or 'gpu' in source
            or source.get('adapter') not in (None, 'foundationpose','operator_assisted_rgbd')):
        raise ValueError('physical source must not carry simulated lane provenance')
    validate_namespace(source, environment)
    if replay:
        if (binding.get('schema_version')!='fake_current_replay_binding.v1'
                or binding.get('evidence_source')!='fake_current_replay_context'
                or binding.get('observation_scope')!='historical_parent_only'
                or binding.get('environment')!='fake' or binding.get('simulation_only') is not True):
            raise ValueError('current fake replay binding required; historical RGB-D is not a fresh capture')
    else:validate_namespace(binding, environment)
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
    assisted=source.get('adapter')=='operator_assisted_rgbd'
    _validate_scene(source.get('scene'),allow_operator_assisted=assisted and environment=='physical')
    if assisted != (source['scene']['source'].get('observation_mode')=='operator_assisted_rgbd'):
        raise ValueError('operator-assisted geometry requires its explicit source adapter')
    if assisted:
        from joint_space import validate_initial_state
        receipt=source['scene']['source']['assistance_receipt']
        geometry=receipt['cup_geometry']
        if (environment!='physical' or source.get('observation_mode')!='operator_assisted_rgbd'
                or receipt['robot_id']!=source.get('robot_id') or receipt['camera_serials']!=source.get('camera_serials')
                or receipt['calibration_profile_digest']!=_digest(profile)
                or receipt['initial_state_digest']!=_digest(source.get('initial_state'))
                or binding.get('operator_assistance_digest')!=_digest(receipt)
                or binding.get('operator_assistance_request_id')!=receipt['request_id']
                or source.get('cup_geometry')!=geometry or source.get('cup_geometry_digest')!=_digest(geometry)
                or source.get('grasp_command')!=receipt['grasp_command']
                or binding.get('cup_geometry_digest')!=_digest(geometry)):
            raise ValueError('operator assistance differs from the bound robot/calibration/initial state/geometry')
        validate_initial_state(source.get('initial_state'),profile)
    fixed_right_collision_policy(source)
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
    if replay:
        from cup_return_replay import validate_source_context
        validate_source_context(source)
    return source


def validate_assisted_cup_geometry(source, exported_cup):
    """Planner-load boundary: a box label alone cannot replace collision geometry."""
    if source.get('adapter')!='operator_assisted_rgbd':return
    from integration import validate_assisted_scene
    expected=validate_assisted_scene(source['scene'])['cup_geometry']
    if (source.get('cup_geometry')!=expected or source.get('cup_geometry_digest')!=_digest(expected)
            or not isinstance(exported_cup,dict) or exported_cup.get('profile_sha256')!=expected['profile_sha256']
            or _digest(exported_cup.get('bands'))!=_digest(expected['bands'])):
        raise ValueError('actual planner cup profile/bands differ from the accepted assisted dimensions')


def validate_real_plan(plan, source, binding, *, environment='physical'):
    validate_real_source(source,binding,environment=environment)
    if source.get('schema_version')=='physical_cup_maintenance_source.v1':
        from physical_cup_maintenance import validate_evidence
        return validate_evidence(plan,source)
    from integration import CUROBO_PLANNER_ID
    from cup_return_replay import REPLAY_PLANNER_ID, OFFSET_PLANNER_ID, REPLAY_SOURCE_SCHEMA
    replay_planners=(REPLAY_PLANNER_ID,OFFSET_PLANNER_ID)
    planners = (SIM_PLANNER_ID, CUROBO_PLANNER_ID, *replay_planners) if environment == 'fake' else (SIM_PLANNER_ID, CUROBO_PLANNER_ID)
    if not isinstance(plan,dict) or plan.get('planner_id') not in planners or plan.get('policy_id') != SIM_POLICY_ID:
        raise ValueError('reviewed simulation planner and policy required')
    if (source.get('schema_version') == REPLAY_SOURCE_SCHEMA) != (plan.get('planner_id') in replay_planners):
        raise ValueError('historical replay source requires its dedicated reverse plan')
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
    elif plan.get('planner_id') in replay_planners:
        from cup_return_replay import validate_evidence
        validate_evidence(plan, source)
