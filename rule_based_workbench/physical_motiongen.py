"""Bind GPU model-space evidence to a physical source without relabelling it as sim.

Pure data conversion/validation only. No SDK, CUDA, camera or actuator access.
"""
from copy import deepcopy
import math

from contract import demo_state, validate_physical14
from gripper_distance_plan import HOLDING_STAGES, NATIVE_DISTANCE_ENCODING, validate_distance_control
from integration import CUROBO_PLANNER_ID, SIM_POLICY_ID, _digest
from joint_space import convert_state, freeze_reported, gripper_offset_mm, validate_initial_state


def target_vector(target):
    validate_physical14(target)
    return [*target['left_arm_joints'],target['left_gripper'],*target['right_arm_joints'],target['right_gripper']]


def vector_target(values):
    if not isinstance(values,list) or len(values)!=14:
        raise ValueError('fourteen absolute model target values required')
    target=dict(demo_state(),left_arm_joints=values[:6],left_gripper=values[6],
                right_arm_joints=values[7:13],right_gripper=values[13])
    validate_physical14(target)
    return target


def reported_target(target,profile):
    converted=convert_state(target,profile,to_reported=True)
    # Only arithmetic roundoff at the declared metal endpoints is canonicalized.
    for key in ('left_gripper','right_gripper'):
        for endpoint in (65.,profile['gripper']['max_opening_mm']):
            if abs(converted[key]-endpoint)<=1e-10:converted[key]=endpoint
    return converted


def validate_model_evidence(plan,source):
    from control_motion import validate_receipt
    validate_receipt(plan)
    if any(value.get(key) is True for value in (plan,source,plan.get('planning_validation',{}))
           for key in ('offline_candidate','synthetic_fixture','test_fixture')):
        raise ValueError('offline/fixture evidence cannot authorize a physical GPU plan')
    backend=plan.get('planning_backend',{});receipt=plan.get('planning_validation',{})
    if (plan.get('planner_id')!=CUROBO_PLANNER_ID or plan.get('policy_id')!=SIM_POLICY_ID
            or backend.get('backend')!='curobo_motiongen' or backend.get('curobo_version')!='0.7.8'
            or backend.get('gpu_ik') is not True or backend.get('gpu_trajectory_search') is not True
            or plan.get('simulation')!={'validated':False,'outcome':'not_run'}
            or receipt.get('schema_version')!='curobo_motiongen_path_validation.v1'
            or receipt.get('passed') is not True or receipt.get('cpu_search_fallback') is not False
            or receipt.get('source_digest')!=_digest(source)
            or receipt.get('timeline_digest')!=_digest(plan.get('ticks'))
            or receipt.get('timeline_digest')!=plan.get('timeline_digest')):
        raise ValueError('bound complete model-space GPU evidence required')
    binding=source['calibration_profile'].get('model_binding',{})
    if not binding.get('urdf_sha256') or backend.get('native_robot_sha256')!=binding['urdf_sha256']:
        raise ValueError('GPU robot differs from the physical calibration model')
    acceptance=receipt.get('gpu_final_acceptance',{})
    checks=[acceptance,*acceptance.get('fixed',{}).values(),*acceptance.get('jaw_sweeps',[]),
            *[acceptance.get(k,{}) for k in ('closing','support','task_semantics','receiving_goal')]]
    if (set(acceptance.get('fixed',{}))!={'world','self'} or not acceptance.get('jaw_sweeps')
            or not all(isinstance(row,dict) and row.get('passed') is True for row in checks)):
        raise ValueError('physical GPU plan requires every final geometry acceptance check')
    records=plan.get('motiongen_receipts',[]);selected=receipt.get('selected_motiongen_receipt_indices')
    if (not isinstance(selected,list) or not selected or receipt.get('motiongen_receipts_sha256')!=_digest(records)
            or any(type(i) is not int or not 0<=i<len(records) or records[i].get('success') is not True for i in selected)):
        raise ValueError('selected successful GPU path receipts required')
    stages=plan.get('stages',[])
    if any(row.get('motiongen_receipt_index') is not None and row['motiongen_receipt_index'] not in selected for row in stages):
        raise ValueError('frozen movement stage is not covered by selected GPU receipts')
    if any(i not in selected for row in stages for i in row.get('motiongen_receipt_indices',[])):
        raise ValueError('joined movement stage has an unselected parent GPU receipt')
    for joined in [*receipt.get('continuous_cross',[]),*receipt.get('continuous_motion',[])]:
        if (joined.get('gpu',{}).get('passed') is not True
                or any(i not in selected for i in joined.get('parent_receipt_indices',[]))):
            raise ValueError('continuous cross GPU derivation is incomplete')
    policy=plan.get('gripper_distance_control',{})
    if (plan.get('native_action_encoding')!=NATIVE_DISTANCE_ENCODING
            or policy.get('schema_version')!='sim_gripper_distance_control.v1'
            or policy.get('closed_metal_gap_mm')!=65. or policy.get('rubber_gap_mm')!=62.
            or abs(policy.get('native_travel_offset_mm',math.inf)-gripper_offset_mm(source['calibration_profile']))>1e-9):
        raise ValueError('physical and GPU 65/62 mm distance definitions differ')


def freeze_physical(plan,source,binding,*,environment='physical'):
    from real_contract import validate_real_source, validate_real_plan
    validate_real_source(source,binding,environment=environment)
    validate_initial_state(source.get('initial_state'),source['calibration_profile'])
    if 'joint_space' in plan or 'physical_gpu_conversion' in plan:
        raise ValueError('physical conversion has already been applied')
    validate_model_evidence(plan,source)
    result=deepcopy(plan)
    vectors=[target_vector(row['target']) for row in plan['ticks']]
    proof=dict(schema_version='physical_gpu_conversion.v1',model_target_vectors=vectors,
               model_target_vectors_sha256=_digest(vectors),model_timeline_digest=plan['timeline_digest'],
               model_validation=deepcopy(plan['planning_validation']),
               model_control_validation=deepcopy(plan['control_validation']),
               calibration_profile_digest=_digest(source['calibration_profile']),
               native_actions_sha256=_digest(plan['native_actions']),
               operation='model joints minus zero offsets; native travel plus metal offset exactly once',
               endpoint_gap_roundoff_mm=1e-10,hardware_validated=False)
    freeze_reported(result,source)
    for row,old in zip(result['ticks'],plan['ticks']):
        row['target']=reported_target(old['target'],source['calibration_profile'])
    result.update(timeline_digest=_digest(result['ticks']),initial_state_digest=_digest(source['initial_state']),
                  physical_gpu_conversion=proof)
    from control_motion import validate_control
    result['control_validation']=validate_control(result)
    result['planning_validation']=dict(schema_version='physical_curobo_motiongen_path_validation.v1',passed=True,
        source_digest=_digest(source),timeline_digest=result['timeline_digest'],
        model_validation_digest=_digest(proof['model_validation']),conversion_digest=_digest(proof),
        hardware_validated=False)
    result['gripper_distance_control']=dict(result['gripper_distance_control'],
        schema_version='gripper_distance_control.v2',units='metal_inner_gap_mm',
        controlled_gripper='left_gripper',holding_stages=sorted(HOLDING_STAGES),
        command_semantics='fixed_distance_target; contact may prevent reaching the target',
        contact_detection=False,force_control=False,hardware_validated=False)
    validate_real_plan(result,source,binding,environment=environment)
    validate_distance_control(result,source)
    return result


def validate_evidence(plan,source):
    proof=plan.get('physical_gpu_conversion',{});receipt=plan.get('planning_validation',{})
    vectors=proof.get('model_target_vectors')
    if (proof.get('schema_version')!='physical_gpu_conversion.v1'
            or receipt.get('schema_version')!='physical_curobo_motiongen_path_validation.v1'
            or receipt.get('passed') is not True or receipt.get('source_digest')!=_digest(source)
            or receipt.get('timeline_digest')!=plan.get('timeline_digest')
            or receipt.get('timeline_digest')!=_digest(plan.get('ticks'))
            or receipt.get('conversion_digest')!=_digest(proof)
            or receipt.get('model_validation_digest')!=_digest(proof.get('model_validation'))
            or proof.get('calibration_profile_digest')!=_digest(source['calibration_profile'])
            or proof.get('native_actions_sha256')!=_digest(plan.get('native_actions'))
            or not isinstance(vectors,list) or len(vectors)!=len(plan.get('ticks',[]))
            or proof.get('model_target_vectors_sha256')!=_digest(vectors)):
        raise ValueError('physical GPU conversion/receipt binding differs')
    model=deepcopy(plan);model.pop('joint_space',None);model.pop('physical_gpu_conversion',None)
    for row,values in zip(model['ticks'],vectors):
        row['target']=vector_target(values)
    model['timeline_digest']=_digest(model['ticks'])
    model['planning_validation']=proof['model_validation']
    model['gripper_distance_control']=dict(model['gripper_distance_control'],schema_version='sim_gripper_distance_control.v1')
    from control_motion import validate_control,validate_receipt
    validate_receipt(plan)
    model['control_validation']=proof.get('model_control_validation')
    if model['timeline_digest']!=proof.get('model_timeline_digest'):
        raise ValueError('original GPU model timeline differs')
    validate_model_evidence(model,source)
    for row,values in zip(plan['ticks'],vectors):
        if row['target']!=reported_target(vector_target(values),source['calibration_profile']):
            raise ValueError('reported target does not match the checked model-space target')
    validate_distance_control(plan,source)
    return model


def retime_physical(plan,source,binding,speed_scale,*,environment='physical'):
    """Reuse the existing GPU planner's stage time interpolation, without queries."""
    from real_contract import validate_real_plan
    from motiongen_plan import scaled_stages,freeze
    from robot_sim.specs.action import ac_one_dual_arm_zero_home_action_spec
    validate_real_plan(plan,source,binding,environment=environment)
    model=validate_evidence(plan,source)
    if model['planning_validation'].get('derivation') is not None:
        raise ValueError('speed adjustment requires the original checked base plan')
    stages=[];starts=model['stages']
    for i,stage in enumerate(starts):
        begin=stage['start_tick'];end=starts[i+1]['start_tick'] if i+1<len(starts) else len(model['ticks'])
        stages.append(dict(name=stage['name'],substage=stage.get('substage',stage['name']),
            rows=[row['target'] for row in model['ticks'][begin:end]],
            native_actions=model['native_actions'][begin:end],receipt_index=stage.get('motiongen_receipt_index')))
    spec=ac_one_dual_arm_zero_home_action_spec()
    scaled,times=scaled_stages(stages,speed_scale,spec,model['gripper_opening_mm'])
    grip=model['gripper_distance_control']['geometry_derivation']
    encoded=freeze(scaled,source,dict(native_identity=model['native_identity']),grip,spec,
                   run_id=model['run_id']+'-speed-'+str(speed_scale))
    derived=deepcopy(model)
    for key in ('run_id','ticks','timeline_digest','native_actions','native_substeps','duration_s','stages','control_validation'):
        derived[key]=encoded[key]
    derived['planning_validation'].update(timeline_digest=derived['timeline_digest'],derivation=dict(
        method='uniform_source_time_stage_scale',parent_timeline_digest=model['timeline_digest'],
        parent_receipt_sha256=_digest(model['planning_validation']),speed_scale=speed_scale,
        parent_stage_sample_times=times,collision_audit_rerun=False))
    return freeze_physical(derived,source,binding,environment=environment)
