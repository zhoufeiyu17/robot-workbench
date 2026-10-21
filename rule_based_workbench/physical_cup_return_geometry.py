"""Explicit physical return geometry; no camera, SDK, network, or execution calls.

Run on the already reserved planning GPU. The original export/asset geometry is
reused, but every return command and stage join is queried against the current
cup and the newly derived held relation. Failed checks remain failed artifacts.
"""
from copy import deepcopy
from contextlib import contextmanager
from dataclasses import asdict, replace
import hashlib
import json
import math
import os
from pathlib import Path
import time

from integration import _digest
from joint_space import convert_state
from physical_motiongen import target_vector


def current_geometry_inputs(parent,parent_source,source,geometry_path,assets,geometry_source=None):
    """Verify the new input chain before importing native geometry or starting CUDA."""
    from artifact_transport import digest
    from physical_cup_maintenance import (current_geometry_binding,validate_current_geometry_source,
                                        validate_current_native_identity)
    binding=current_geometry_binding(source)
    if binding is None:
        if geometry_source is not None:raise ValueError('legacy maintenance cannot override its parent geometry source')
        return parent_source,None
    from physical_motiongen import validate_evidence
    validate_evidence(parent,parent_source)
    if (_digest(parent_source)!=source['parent_source_digest']
            or parent['timeline_digest']!=source['maintenance_context']['parent_timeline_digest']):
        raise ValueError('current geometry must retain the immutable historical parent seed')
    validate_current_geometry_source(source,geometry_source)
    root=Path(assets).resolve(strict=True);pins_path=root.parent/'asset-pins.json';receipt_path=root.parent/'receipt.json'
    if digest(geometry_path)!=binding['export_sha256'] or digest(pins_path)!=binding['asset_pins_sha256'] or digest(receipt_path)!=binding['asset_receipt_sha256']:
        raise ValueError('current geometry export or asset receipts changed')
    pins=json.loads(pins_path.read_text());asset=json.loads(receipt_path.read_text())
    if not isinstance(pins,dict) or not pins:raise ValueError('current asset file pins required')
    for name,expected in pins.items():
        path=Path(name)
        try:resolved=path.resolve(strict=True)
        except (OSError,RuntimeError) as error:
            raise ValueError('current asset path or pinned bytes differ') from error
        link=os.readlink(path) if path.is_symlink() else None
        if (not path.is_absolute() or '..' in path.parts or not resolved.is_relative_to(root)
                or not resolved.is_file() or (resolved!=path and
                    (link!=resolved.name or resolved.parent!=path.parent or pins.get(str(resolved))!=expected))
                or digest(resolved)!=expected):
            raise ValueError('current asset path or pinned bytes differ')
    cup=geometry_source.get('cup_geometry',{})
    if (asset.get('status')!='qualified' or Path(asset.get('assets','')).resolve()!=root
            or asset.get('asset_pins_sha256')!=binding['asset_pins_sha256']
            or asset.get('cup_profile_sha256')!=cup.get('profile_sha256') or asset.get('bands')!=cup.get('bands')
            or asset.get('a1_asset_validation',{}).get('model_binding')!=source['calibration_profile']['model_binding']
            or asset.get('envelope_proof',{}).get('continuous_core_verification',{}).get('passed') is not True):
        raise ValueError('current CPU asset qualification or unchanged robot/calibration binding differs')
    estimate=asset.get('estimate',{});proof_path=Path(estimate.get('path',''))
    if not proof_path.is_file() or digest(proof_path)!=estimate.get('sha256'):
        raise ValueError('mesh containment provenance changed')
    proof=json.loads(proof_path.read_text())
    if (proof.get('mesh_sha256')!=binding['object_mesh_sha256'] or not proof.get('mesh_path')
            or digest(proof['mesh_path'])!=binding['object_mesh_sha256']
            or proof.get('proof',{}).get('all_mesh_triangle_clips_inside_proxy') is not True):
        raise ValueError('original object mesh or its containment proof differs')
    data=json.loads(Path(geometry_path).read_text())
    validate_current_native_identity(source,parent,data.get('native_identity'))
    entry=root/'objects/anyverse_lidded_paper_cup_9cm_13cm/v1/sim/cup.usda'
    if (data.get('source_digest')!=binding['source_digest']
            or data.get('urdf_sha256')!=source['calibration_profile']['model_binding']['urdf_sha256']
            or data['native_identity'].get('physical_configuration',{}).get('base',{}).get('cup_asset_entrypoint_sha256')!=digest(entry)):
        raise ValueError('current exported source, robot or active cup identity differs')
    from maintenance_geometry import validate_export
    validate_export(geometry_path,geometry_source,source)
    return geometry_source,binding


@contextmanager
def tray_support_for_closing(geometry):
    """Use the existing tray-bottom contact policy only during tray regrasp."""
    original=geometry.audit
    def audit(frames,cup,**options):
        if options.get('initial_table_contact'):
            options.update(initial_table_contact=False,support_contact=True)
        return original(frames,cup,**options)
    geometry.audit=audit
    try:yield
    finally:geometry.audit=original


def interpolated_rows(rows, previous=None, *, maximum_step_rad=.01, maximum_gap_step_mm=None):
    """Include every command and the preceding stage join; keep the source clock separate."""
    if not rows or not 0<maximum_step_rad<=.01:raise ValueError('nonempty path and original 0.01 rad sampling bound required')
    if maximum_gap_step_mm is not None and not 0<maximum_gap_step_mm<=.1:
        raise ValueError('initializer jaw sampling must not exceed the existing 50um single-finger step')
    output=[];prior=deepcopy(previous if previous is not None else rows[0]);largest=0.
    output.append(prior)
    for row in rows:
        a=target_vector(prior);b=target_vector(row)
        indices=(*range(6),*range(7,13))
        count=max(1,math.ceil(max(abs(b[i]-a[i]) for i in indices)/(maximum_step_rad*(1-1e-9))))
        if maximum_gap_step_mm is not None:
            count=max(count,math.ceil(max(abs(row[k]-prior[k]) for k in ('left_gripper','right_gripper'))/maximum_gap_step_mm))
        for index in range(1,count+1):
            fraction=index/count;sample=deepcopy(row)
            for key in ('left_arm_joints','right_arm_joints'):
                sample[key]=[x+(y-x)*fraction for x,y in zip(prior[key],row[key])]
            for key in ('left_gripper','right_gripper'):sample[key]=prior[key]+(row[key]-prior[key])*fraction
            largest=max(largest,max(abs(x-y) for key in ('left_arm_joints','right_arm_joints')
                                   for x,y in zip(output[-1][key],sample[key])))
            output.append(sample)
        prior=row
    if largest>maximum_step_rad+1e-12:raise ValueError('interpolated path exceeds original sampling bound')
    return output,largest


def same_pose(a,b,*,tolerance=1e-9):
    from cup_return_replay import _pose_values
    a={key:list(value) for key,value in a.items()};b={key:list(value) for key,value in b.items()}
    p,q=_pose_values(a);r,s=_pose_values(b)
    return math.dist(p,r)<=tolerance and min(max(abs(x-sign*y) for x,y in zip(q,s)) for sign in (-1,1))<=tolerance


def offset_base(parent, reversed_stages):
    keys=('control','native_identity','native_action_encoding','gripper_opening_mm','gripper_distance_control')
    result={key:deepcopy(parent[key]) for key in keys};ticks=[];actions=[];stages=[]
    for index,row in enumerate(reversed_stages):
        stages.append(dict(name=row['name'],substage=row['substage'],start_tick=len(ticks),start_s=len(ticks)/30,
                           parent_stage_index=row['parent_stage_index'],base_reverse_stage_index=index))
        for target in row['rows']:ticks.append(dict(tick=len(ticks),time_s=len(ticks)/30,target=deepcopy(target)))
        actions.extend(deepcopy(row['native_actions']))
    result.update(ticks=ticks,native_actions=actions,stages=stages,timeline_digest=_digest(ticks),
                  native_substeps=[parent['control']['substeps']]*len(ticks),duration_s=len(ticks)/30)
    return result


def bind_current_right(stages, source, kinematics, opening):
    """Rebind the inactive arm to fresh feedback; retain raw feedback and record projection."""
    from robot_workbench.joint_bounds import project_joint_positions
    from motiongen_plan import encode_stage
    requested=source['maintenance_context']['right_hold_reported']
    reference=deepcopy(source['initial_state']['planned_reported']);reference['right_arm_joints']=list(requested)
    model=convert_state(reference,source['calibration_profile'],to_reported=False)
    bounded,changes=project_joint_positions(model['right_arm_joints'],kinematics.limits['right'],environment='physical')
    original=list(model['right_arm_joints']);model['right_arm_joints']=bounded
    command=convert_state(model,source['calibration_profile'],to_reported=True)
    proof=dict(schema='physical_return_right_hold_projection.v1',environment='physical',allowance_rad=.01,
        original_reported=list(requested),original_model=original,intersection_limits=kinematics.limits['right'],
        bounded_model=bounded,bounded_reported=command['right_arm_joints'],changes=changes,
        right_gripper_reported=reference['right_gripper'],source='current maintenance initial feedback; not parent targets')
    cache={};result=deepcopy(stages)
    for stage in result:
        for row in stage['rows']:
            row['right_arm_joints']=deepcopy(command['right_arm_joints']);row['right_gripper']=reference['right_gripper']
        models=[convert_state(row,source['calibration_profile'],to_reported=False) for row in stage['rows']]
        encoded=encode_stage(dict(stage,rows=models),kinematics.spec,opening,cache,environment='physical',source=source)
        if any(abs(c['delta_rad'])>1e-12 for item in encoded.get('position_corrections',[]) for c in item['changes']):
            raise ValueError('right rebind unexpectedly needs another position projection')
        # Preserve the original left command bytes; refresh only the newly held right slots.
        stage['native_actions']=[old[:7]+new[7:] for old,new in zip(stage['native_actions'],encoded['native_actions'],strict=True)]
        stage['right_hold_rebound']=True
    return result,proof


def offset_artifact(base, proof, original_stages, pickup=None):
    stages=[];changed=set(proof['local_stage_indices']);pickup_indices=set((pickup or {}).get('affected_parent_indices',[]))
    for index,stage in enumerate(base['stages']):
        begin=stage['start_tick'];end=base['stages'][index+1]['start_tick'] if index+1<len(base['stages']) else len(base['ticks'])
        parent=stage['base_reverse_stage_index']
        value=dict(name=stage['name'],substage=stage['substage'],parent_stage_index=stage['parent_stage_index'],
            base_reverse_stage_index=parent,rows=[deepcopy(row['target']) for row in base['ticks'][begin:end]],
            native_actions=deepcopy(base['native_actions'][begin:end]),derived_from_parent_stage=index in changed or parent in pickup_indices,
            right_hold_rebound=True)
        if index not in changed and parent not in pickup_indices:value['parent_tick_indices']=deepcopy(original_stages[parent]['parent_tick_indices'])
        stages.append(value)
    return dict(schema='physical_cup_return_offset.v1',stages=stages,kinematics=deepcopy(proof['kinematics']),derivation=proof,
                **({'pickup_derivation':pickup}if pickup is not None else {}))


def verify_rebound_right_hold(stage,reference,expected_native_right):
    """Keep native right commands exact; decoded row arithmetic uses the contract's 1e-12 bound."""
    expected=_digest(expected_native_right)
    if not stage['rows'] or len(stage['rows'])!=len(stage['native_actions']):
        raise ValueError('pickup right hold requires aligned command rows')
    for row,action in zip(stage['rows'],stage['native_actions'],strict=True):
        if _digest(action[7:])!=expected:
            raise ValueError('pickup encoded right command differs from rebound right hold')
        if (any(abs(a-b)>1e-12 for a,b in zip(row['right_arm_joints'],reference['right_arm_joints'],strict=True))
                or abs(row['right_gripper']-reference['right_gripper'])>1e-12):
            raise ValueError('pickup decoded right target differs from rebound right hold')
    return stage


def fixed_arm_jaw_path_check(engine,samples,jaw,certificate,*,stage_name):
    """Compose the existing continuous GPU jaw proof, never a raw no-contact recheck."""
    from gpu_acceptance import jaw_input_digest
    if stage_name not in ('close_ramp','open'):raise ValueError('jaw composition is limited to closing and opening')
    if not samples or jaw.get('passed') is not True or jaw.get('unresolved_claw_samples')!=0:
        raise ValueError('passed complete GPU jaw sweep required for fixed-arm path coverage')
    q=[row['left_arm_joints'] for row in samples];reference=jaw.get('model_arm_q')
    if not isinstance(reference,list) or len(reference)!=6 or any(len(row)!=6 for row in q):
        raise ValueError('jaw proof must record its six-joint GPU input')
    actual=engine.tensor_args.to_device(q);expected=engine.tensor_args.to_device([reference]*len(q))
    if not engine.torch.equal(actual,expected):
        raise ValueError('jaw sweep cannot cover a different GPU arm configuration')
    if stage_name=='close_ramp' and not engine.torch.equal(actual[0],engine.tensor_args.to_device(certificate.get('grasp_q_model',[]))):
        raise ValueError('closing certificate differs from this GPU grasp pose')
    locked=jaw.get('model_locked_joints')
    keys=sorted(set(engine.config['kinematics']['lock_joints'])-{'left_joint7','left_joint8'})
    if not isinstance(locked,dict) or any(k not in locked for k in keys) or not engine.torch.equal(
            engine.tensor_args.to_device([locked[k]for k in keys]),
            engine.tensor_args.to_device([engine.config['kinematics']['lock_joints'][k]for k in keys])):
        raise ValueError('jaw sweep cannot cover changed fixed/right GPU geometry')
    for row in samples:
        right={**dict(zip([f'right_joint{i}'for i in range(11,17)],row['right_arm_joints'])),
            'right_joint17':row['right_gripper']/2000,'right_joint18':row['right_gripper']/2000}
        names=sorted(set(keys)&set(right))
        if not engine.torch.equal(engine.tensor_args.to_device([right[k]for k in names]),engine.tensor_args.to_device([locked[k]for k in names])):
            raise ValueError('jaw sweep cannot cover changed right command inputs')
    input_digest=jaw_input_digest(engine,reference,locked)
    if jaw.get('native_input_digest')!=input_digest:raise ValueError('jaw GPU input digest differs')
    strict=('self_max','environment_max','cup_wrist_max','cup_world_max')
    if any(type(jaw.get(k)) not in (int,float)or not math.isfinite(jaw[k])or jaw[k]>0 for k in strict):
        raise ValueError('fixed-arm jaw proof has an unresolved strict GPU constraint')
    if jaw.get('maximum_jaw_step_m')!=.00005 or jaw.get('continuous_translation_padding_m')!=.000026:
        raise ValueError('original continuous jaw resolution/padding required')
    raw=jaw.get('cup_claw_max')
    if type(raw)not in (int,float) or not math.isfinite(raw):raise ValueError('finite raw GPU claw query required')
    if raw>0 and (jaw.get('source_bound_used_samples',0)<=0 or certificate.get('claw_cup_allowance_m')!=.003
            or jaw.get('contact_certificate_digest')!=certificate.get('certificate_digest')):
        raise ValueError('positive raw claw cost requires this selected-pose source contact bound')
    clip=certificate.get('terminal_jaw_position_m')
    if certificate.get('linked_jaws')is not True or type(clip)not in (int,float)or not math.isfinite(clip):
        raise ValueError('linked source contact stop required for existing preload geometry')
    gaps=[max(row['left_gripper']/2000,clip)for row in samples]
    if any(not math.isfinite(v)for v in gaps)or any(not math.isfinite(jaw[k])for k in ('start_m','end_m')):
        raise ValueError('finite jaw geometry range required')
    values=engine.tensor_args.to_device(gaps);ends=engine.tensor_args.to_device([jaw['start_m'],jaw['end_m']])
    if ((values<ends.min())|(values>ends.max())).any().item():
        raise ValueError('command geometry lies outside the continuous GPU jaw sweep')
    if ((values[1:]-values[:-1])*(ends[1]-ends[0])<0).any().item():
        raise ValueError('jaw command geometry reverses the canonical sweep direction')
    return dict(passed=True,self_max=jaw['self_max'],world_max=max(jaw[k]for k in strict[1:]),samples=len(samples),
        path_sha256=_digest(q),backend='complete GPU jaw/centering sweep at bit-identical native arm inputs',
        geometry_backend='gpu_jaw_sweep',native_input_digest=input_digest,
        world_max_scope='strict non-contact constraints; certified claw contact recorded separately',
        raw_cup_claw_max=raw,source_bound_used_samples=jaw['source_bound_used_samples'],
        contact_certificate_digest=certificate.get('certificate_digest'),gpu_jaw_sweep_digest=_digest(jaw),
        native_arm_inputs_identical=True,contact_preload_geometry_clipped=True)


def verify_pickup_model_indices(engine,prepared):
    """Check live cuRobo link mappings, retaining the native attached-slot query."""
    model=engine.generator.kinematics.kinematics_config
    def ids(alias):return set(model.get_sphere_index_from_link_name(alias).cpu().tolist())
    expected_wrist=set();expected_claw=set()
    for alias in engine.config['kinematics']['collision_link_names']:
        link=engine.config['kinematics']['extra_links'][alias]['parent_link_name']
        if link=='left_link6':expected_wrist.update(ids(alias))
        if link in ('left_link7','left_link8'):expected_claw.update(ids(alias))
    native=engine.generator.rollout_fn.robot_self_collision_constraint.self_collision_kin_config
    total=int(model.total_spheres)
    if (set(engine.cup_contact.wrist_indices)!=expected_wrist or set(engine.cup_contact.claw_indices)!=expected_claw
            or total!=prepared['proxy']['allocated_spheres'] or native.offset.numel()!=total):
        raise ValueError('pickup GPU index/kernel dimensions differ from frozen C11 spheres')
    # Native CupContact includes the attached slots in its wrist query; they are
    # negative placeholders before attachment and certified separately while held.
    physical_wrist=expected_wrist-ids('attached_object')
    return dict(total_spheres=total,wrist_indices=engine.cup_contact.wrist_indices,
        physical_wrist_spheres=len(physical_wrist),attached_slot_indices=sorted(ids('attached_object')),
        claw_indices=engine.cup_contact.claw_indices,self_offset_count=native.offset.numel(),
        native_matrix_shape=list(native.collision_matrix.shape)if native.collision_matrix is not None else None,
        experimental_kernel=bool(native.experimental_kernel),rebuilt_from_current_link_names=True)


def retarget_pickup(planner,observation,data,engine,prepared,kine,stages,parent,source,mesh_path,save):
    """Existing candidate/IK/closing/path primitives; no SDK or source-pose changes."""
    from robot_sim.asset_cfg.objects.coffee_machine import RigidTransform
    from motiongen_targets import generate_targets
    from motiongen_motion import continuous_path,task_pose_error
    from motiongen_plan import encode_stage,target,world_config
    from physical_motiongen import reported_target
    from physical_cup_maintenance import pickup_binding,PICKUP_TRANSITION_POLICY
    from gpu_acceptance import closing_geometry
    from grasp_contact_geometry import original_mesh_sections
    from robot_sim.asset_cfg.robots.ac_one_finger_contact import ac_one_finger_contact_spec
    from cup_envelope import attach
    from control_motion import MotionLimitError
    import trimesh
    indices={s['name']:i for i,s in enumerate(stages)};opening=parent['gripper_opening_mm']
    metal_gap=parent['gripper_distance_control']['closed_metal_gap_mm']
    if metal_gap!=source['grasp_command']['metal_gap_mm']:raise ValueError('pickup metal command differs from current source')
    closed=(metal_gap-parent['gripper_distance_control']['native_travel_offset_mm'])/2000
    candidates=generate_targets(planner,observation,data,urdf_path=kine.identity['urdf_path'],closed_jaw_m=closed)
    save('pickup-candidates.json',candidates)
    mesh_sha=source['maintenance_context']['current_geometry']['object_mesh_sha256']
    if hashlib.sha256(Path(mesh_path).read_bytes()).hexdigest()!=mesh_sha:raise ValueError('original pickup OBJ changed')
    mesh=trimesh.load(str(mesh_path),force='mesh',process=False)
    original_models=[[convert_state(r,source['calibration_profile'],to_reported=False)for r in s['rows']]for s in stages]
    reference=original_models[indices['approach_pre']-1][-1];right=reference['right_arm_joints'];right_gap=reference['right_gripper']
    right_native=stages[indices['approach_pre']-1]['native_actions'][-1][7:]
    opened=kine.positions(reference);opened.update(left_joint7=opening/2000,left_joint8=opening/2000)
    control=parent['control'];cache={};attempts=[]
    if source['maintenance_context'].get('pickup_proxy_profile'):
        save('pickup-runtime-model-check.json',verify_pickup_model_indices(engine,prepared))
    def encoded(name,path,gaps=None,*,gap=None):
        moving=len(path)>1 and any(math.dist(path[0],q)>1e-12 for q in path[1:])
        if moving and gaps is not None:raise ValueError('jaw ramp requires a stationary arm')
        if moving:
            path,timing=continuous_path(path,control['max_target_velocity_rad_s'],control['max_target_acceleration_rad_s2'])
        else:timing=None
        rows=[target(q,right,jaw,right_gap)for q,jaw in zip(path,
            [opening if gap is None else gap]*len(path)if gaps is None else gaps,strict=True)]
        value=encode_stage(dict(name=name,rows=rows),kine.spec,opening,cache,environment=source['environment'],source=source)
        value['motion_continuity']=timing
        save('pickup-stage-'+candidate['candidate_id']+'-'+name+'.json',value)
        return verify_rebound_right_hold(value,reference,right_native)
    base_from_world=planner.geo.mount.inverse();cup=observation.sample.world_from_cup
    diagnosis=plane_inputs=None
    if source['maintenance_context'].get('pregrasp_policy'):
        from pickup_pregrasp import geometry_inputs,derive,original_target_diagnosis
        engine.update_locked_joints({k:opened[k]for k in engine.config['kinematics']['lock_joints']})
        world,_,description=world_config(planner,cup,prepared['fixed_meshes']);engine.update_world(world)
        prepared['diagnostic_world_mesh_poses']={mesh.name:list(mesh.pose)for mesh in world.mesh or []}
        save('original-target-state.json',dict(q0=reference['left_arm_joints'],locked_joints=deepcopy(engine.config['kinematics']['lock_joints']),
            cup_pose=asdict(cup),world_sha256=_digest(description),source_digest=_digest(source)))
        if source['maintenance_context'].get('pickup_proxy_profile'):
            diagnosis=source['maintenance_context'].get('pregrasp_diagnosis')
            promotion=prepared['proxy']['pickup_promotion']
            if (not isinstance(diagnosis,dict) or _digest(diagnosis)!=promotion['pregrasp_diagnosis_digest']
                    or diagnosis.get('passed') is not True or not diagnosis.get('witnesses')
                    or diagnosis.get('q0')!=reference['left_arm_joints']
                    or diagnosis.get('original_candidates_sha256')!=_digest(candidates['grasp_candidates'])):
                raise ValueError('C11 must reuse the exact recorded C9 diagnostic inputs and witness')
            save('original-target-diagnosis-reused.json',diagnosis)
        else:
            diagnosis=original_target_diagnosis(engine,prepared,candidates['grasp_candidates'],reference['left_arm_joints'],base_from_world,save)
        plane_inputs=geometry_inputs(planner,data,source);save('pregrasp-geometry-inputs.json',plane_inputs)
    for original_candidate in candidates['grasp_candidates']:
        candidate=deepcopy(original_candidate);plane=None
        attempt=dict(candidate_id=candidate['candidate_id'],status='pending');attempts.append(attempt);attached=False
        try:
            if plane_inputs is not None:
                plane=derive(plane_inputs,candidate['cup_from_tool'],asdict(cup))
                save('pregrasp-'+candidate['candidate_id']+'.json',plane)
                if not plane['within_old_bound']:raise ValueError('support-plane retreat exceeds original bound')
                candidate['world_from_pregrasp']=plane['world_from_pregrasp']
            engine.cup_contact.allow_claw_contact(True)
            q0=reference['left_arm_joints']
            pre,_=engine.plan_pose(q0,[asdict(base_from_world.compose(RigidTransform(**candidate['world_from_pregrasp'])))],name='pickup_approach_pre')
            pre=encoded('approach_pre',pre)
            approach,_=engine.plan_pose(pre['rows'][-1]['left_arm_joints'],
                [asdict(base_from_world.compose(RigidTransform(**candidate['world_from_tool'])))],
                name='pickup_approach',hold_orientation=True,linear_approach=True)
            approach=encoded('approach',approach);grasp=approach['rows'][-1];q=grasp['left_arm_joints']
            actual=kine.pose(grasp);goal=candidate['world_from_tool']
            position_error,angle_error,passed=task_pose_error(actual['position'],actual['orientation_wxyz'],
                goal['position'],goal['orientation_wxyz'])
            if not passed:raise ValueError('MotionGen pickup_approach exceeds accepted 2mm/0.5deg pose tolerance')
            state={**opened,**dict(zip(kine.names('left'),q))}
            save('pickup-selected-pose-'+candidate['candidate_id']+'.json',dict(model_state=state,
                actual_fk=actual,goal=goal,pre=pre,approach=approach,source_digest=_digest(source)))
            engine.cup_contact.allow_claw_contact(False)
            with tray_support_for_closing(planner.geo):
                closing=closing_geometry(engine,planner,state,cup,data=data,source_digest=_digest(source),
                    candidate_id=source['run_id']+'/'+candidate['candidate_id'],closed_jaw_m=closed)
            save('pickup-closing-'+candidate['candidate_id']+'.json',closing)
            section=original_mesh_sections(mesh,closing['contact_certificate']['grasp_contact_region'],mesh_sha256=mesh_sha,
                metal_gap_mm=metal_gap,opening_mm=opening+parent['gripper_distance_control']['native_travel_offset_mm'],
                rubber_thickness_m=ac_one_finger_contact_spec().rubber_thickness_m)
            save('pickup-obj-'+candidate['candidate_id']+'.json',section)
            if not section['passed']:raise ValueError('original OBJ section does not fit existing jaw span')
            contact=closing['terminal_state'];held=RigidTransform(**closing['world_from_cup'])
            held_world,_,held_world_description=world_config(planner,held,prepared['fixed_meshes'])
            engine.update_world(held_world)
            engine.update_locked_joints({k:contact[k]for k in engine.config['kinematics']['lock_joints']})
            attachment=attach(engine,q,prepared['cup_envelope']);attached=True
            attachment['contact']=engine.cup_contact.certify_held(engine,q,closing['contact_certificate'])
            attachment.update(model_q=q,closed_state=contact,world_from_cup=asdict(held),
                local_spheres=engine.generator.kinematics.kinematics_config.get_link_spheres('attached_object').detach().cpu().tolist())
            save('pickup-attachment-'+candidate['candidate_id']+'.json',attachment)
            engine.cup_contact.allow_claw_contact(True)
            tool=planner.geo.mount.compose(planner.fk.pose(planner.fk.forward(contact)['left_link6']))
            base_grasp=base_from_world.compose(RigidTransform(**actual))
            if not same_pose(asdict(base_from_world.compose(tool)),asdict(base_grasp)):
                raise ValueError('pickup native FK frames differ at the same encoded arm joints')
            carry_row=original_models[indices['lift']][-1];carry_q=carry_row['left_arm_joints'];transition=None
            if source['maintenance_context'].get('pickup_transition_policy'):
                if source['maintenance_context']['pickup_transition_policy']!=PICKUP_TRANSITION_POLICY:
                    raise ValueError('unknown current pickup transition policy')
                goal=kine.pose(carry_row)
                tray=planner.geo.scene.support_shape_path
                if not any(item['name']==tray for item in held_world_description):
                    raise ValueError('held-cup transition requires the original tray world obstacle')
                transition=dict(policy=PICKUP_TRANSITION_POLICY,inherited_stage_index=indices['lift'],
                    inherited_model_joints=carry_q,world_from_tool_goal=goal,
                    actual_grasp_model_joints=q,urdf_sha256=kine.identity['urdf_sha256'],
                    fk_implementation_sha256=kine.identity['implementation_sha256'],
                    contact_certificate_digest=closing['contact_certificate']['certificate_digest'],
                    world_from_cup=asdict(held),tool_from_held_cup=asdict(tool.inverse().compose(held)),
                    tray_world_shape=tray,tray_world_obstacle_retained=True,free_xyz=True)
                goal_cup=RigidTransform(**goal).compose(tool.inverse().compose(held));world_checks=[]
                for band,shape in planner.geo.cup_bands:
                    band_pose=goal_cup.compose(RigidTransform((0,0,(band.z_min_m+band.z_max_m)/2)))
                    obj=planner.geo.fcl.CollisionObject(shape,planner.geo.whole.transform(band_pose))
                    for name,other,*_ in planner.geo.whole.world:
                        distance,witnesses=planner.geo.cup_pair(obj,other)
                        depth=max((value for _,value in witnesses),default=0.)
                        if not math.isfinite(distance) or not math.isfinite(depth):raise ValueError('nonfinite source carry-entry cup/world geometry')
                        allowed=(depth<=planner.geo.cup_machine_contact_allowance_m if name in planner.geo.machine_shape_paths
                            else distance>=.0001 and depth==0.)
                        world_checks.append(dict(cup_band=band.shape_path,world_shape=name,distance_m=distance,penetration_m=depth,passed=allowed))
                transition['actual_held_cup_goal_check']=dict(passed=bool(world_checks)and all(item['passed']for item in world_checks),
                    world_from_cup=asdict(goal_cup),checks=world_checks,policy='unchanged native cup/world allowance; no new contact waiver')
                save('pickup-transition-'+candidate['candidate_id']+'.json',transition)
                if not transition['actual_held_cup_goal_check']['passed']:
                    raise ValueError('actual held cup carry-entry goal fails original world policy')
                lift_path,_=engine.plan_pose(q,[asdict(base_from_world.compose(RigidTransform(**goal)))],
                    name='pickup_to_carry_entry',hold_orientation=True)
                endpoint=kine.pose(target(lift_path[-1],right,closed*2000,right_gap))
                error=task_pose_error(endpoint['position'],endpoint['orientation_wxyz'],goal['position'],goal['orientation_wxyz'])
                if not error[2]:raise ValueError('MotionGen pickup_to_carry_entry exceeds accepted 2mm/0.5deg pose tolerance')
                transition['native_fk_goal_check']=dict(position_error_m=error[0],angle_error_rad=error[1],passed=True)
            else:
                lift=RigidTransform((tool.position[0],tool.position[1],tool.position[2]+.02),tool.orientation_wxyz)
                lift_path,_=engine.plan_pose(q,[asdict(base_from_world.compose(lift))],name='pickup_lift',
                    hold_orientation=True,linear_axis=2,project_to_goal_frame=False)
            if math.dist(lift_path[-1],carry_q)>1e-12:
                join,_=engine.plan_joint(lift_path[-1],carry_q,name='pickup_join_inherited_carry')
            else:join=[lift_path[-1]]
            if not join or math.dist(join[0],lift_path[-1])>1e-12:raise ValueError('pickup connector start differs from lift endpoint')
            if transition is not None:
                transition.update(join_length_l2_rad=sum(math.dist(a,b)for a,b in zip(join,join[1:])),
                    joined_model_joint_goal=join[-1])
                save('pickup-transition-'+candidate['candidate_id']+'.json',transition)
            closed_model=2*closed*1000
            if abs(original_models[indices['close_and_hold']][-1]['left_gripper']-closed_model)>1e-9:
                raise ValueError('inherited close gap differs from the current native metal command')
            lift_rows=encoded('lift',lift_path+join[1:],gap=closed_model)
            if transition is not None:
                endpoint=kine.pose(lift_rows['rows'][-1]);goal=transition['world_from_tool_goal']
                error=task_pose_error(endpoint['position'],endpoint['orientation_wxyz'],goal['position'],goal['orientation_wxyz'])
                if not error[2]:raise ValueError('MotionGen pickup_to_carry_entry exceeds accepted 2mm/0.5deg pose tolerance')
                transition['native_fk_goal_check']=dict(position_error_m=error[0],angle_error_rad=error[1],passed=True)
                transition['final_encoded_model_joints']=lift_rows['rows'][-1]['left_arm_joints']
                save('pickup-transition-'+candidate['candidate_id']+'.json',transition)
            # A new orientation family cannot silently tilt the cup while joining
            # inherited carry joints. Check the same held-pose corridor first.
            held_orientation=[]
            carried=[lift_rows['rows']]+[rows for stage,rows in zip(stages,original_models,strict=True)if stage['name']=='transport']
            previous=grasp
            for carried_rows in carried:
                samples,_=interpolated_rows(carried_rows,previous);evidence={}
                engine.validate_held_path([row['left_arm_joints']for row in samples],base_grasp.position,
                    base_grasp.orientation_wxyz,name='pickup_held',receipt=evidence)
                held_orientation.append(evidence);previous=carried_rows[-1]
            changed={indices['approach_pre']:pre,indices['approach']:approach,indices['lift']:lift_rows}
            for name in ('preclose_settle','close_ramp','close_and_hold'):
                rows=original_models[indices[name]]
                changed[indices[name]]=encoded(name,[q]*len(rows),[row['left_gripper']for row in rows])
            result=deepcopy(stages)
            for index,value in changed.items():
                result[index].update(rows=[reported_target(row,source['calibration_profile'],source)for row in value['rows']],
                    native_actions=deepcopy(value['native_actions']),pickup_motion_continuity=value.get('motion_continuity'))
            attempt.update(status='qualified_pickup',actual_fk_position_error_m=position_error,actual_fk_angle_error_rad=angle_error)
            proof=dict(schema_version='physical_cup_pickup_derivation.v1',binding=pickup_binding(source),
                affected_parent_indices=sorted(changed),actual_fk_qualified=True,metal_gap_mm=metal_gap,
                selected_candidate=deepcopy(candidate),candidate_selection=candidates['grasp_candidate_selection'],attempts=attempts,
                candidate_order=[row['candidate_id']for row in candidates['grasp_candidates']],
                pickup_lift_height_m=.02,pickup_lift_displacement_frame='world Z; solver corridor uses robot-base Z',
                inherited_carry_orientation=held_orientation,
                stages={str(i):dict(rows_sha256=_digest(result[i]['rows']),native_actions_sha256=_digest(result[i]['native_actions']))for i in changed},
                actual_fk=dict(position_error_m=position_error,angle_error_rad=angle_error,model_joints=q),
                contact_certificate_digest=closing['contact_certificate']['certificate_digest'],original_obj_sections=section,
                motiongen_receipts=deepcopy(engine.receipts),geometry_scope='pickup derivation only; full return validation follows')
            if plane_inputs is not None:
                proof.update(pregrasp_inputs=deepcopy(plane_inputs),pregrasp_derivation=plane,original_target_diagnosis=deepcopy(diagnosis))
            if transition is not None:
                proof.pop('pickup_lift_height_m');proof.pop('pickup_lift_displacement_frame')
                proof['carry_entry_derivation']=transition
            proof['derivation_digest']=_digest(proof);save('pickup-derivation.json',proof)
            return result,closing,proof
        except MotionLimitError as error:
            attempt.update(status='aborted',error=str(error));save('pickup-attempts.json',attempts)
            raise
        except ValueError as error:
            # Unknown structure, identity, missing geometry and numerical failures
            # abort this job; only known candidate qualifications may continue.
            reason=str(error)
            allowed=('held orientation differs from start by more than 0.5deg',
                'start pose is outside the native held-pose constraint',
                'actual MotionGen pose lacks 2 mm contact-band reserve: potential_contact_height_outside_reserved_band',
                'actual MotionGen pose lacks 2 mm contact-band reserve: contact_geometry_unproven: open pad already contacts cup:',
                'actual MotionGen pose lacks 2 mm contact-band reserve: contact_geometry_unproven: commanded closure does not reach bilateral rigid contact',
                'GPU closing/centering acceptance failed:', 'original OBJ section does not fit existing jaw span')
            allowed+=('support-plane retreat exceeds original bound',)
            allowed+=('actual held cup carry-entry goal fails original world policy',)
            if not (reason.startswith(allowed)or any(reason.startswith('MotionGen '+name+tail)
                    for name in ('pickup_approach_pre','pickup_approach','pickup_lift','pickup_to_carry_entry','pickup_join_inherited_carry','pickup_held')
                    for tail in (' failed:', ' exceeds accepted 2mm/0.5deg pose tolerance',
                                 ' exceeds 0.5deg along the held-orientation path', ' exceeds the straight-motion 2mm corridor'))):
                attempt.update(status='aborted',error=reason);save('pickup-attempts.json',attempts);raise
            attempt.update(status='rejected',error=str(error));engine.grasp_contact_certificate=None
            save('pickup-attempts.json',attempts)
        except Exception as error:
            attempt.update(status='aborted',error=str(error),error_type=type(error).__name__)
            save('pickup-attempts.json',attempts);raise
        finally:
            try:save('pickup-motiongen-receipts.json',deepcopy(engine.receipts))
            finally:
                # A native attach may mutate buffers before raising. Cleanup also
                # runs if receipt persistence fails; neither error may continue.
                config=getattr(getattr(engine.generator,'kinematics',None),'kinematics_config',None)if hasattr(engine,'generator')else None
                partial=config is not None and bool((config.get_link_spheres('attached_object')[:,3]>0).any().item())
                if attached or partial:engine.generator.detach_object_from_robot(link_name='attached_object')
                engine.cup_contact.allow_claw_contact(False)
                engine.update_locked_joints({k:opened[k]for k in engine.config['kinematics']['lock_joints']})
                engine.update_world(world_config(planner,cup,prepared['fixed_meshes'])[0])
    raise ValueError('existing bounded pickup candidates exhausted; see pickup-attempts.json')


def table_support(planner, cup_pose, goal, acceptance):
    """Same authored table, original surface/edge/tilt bounds; no physical support claim."""
    from robot_sim.asset_cfg.objects.coffee_machine import RigidTransform
    table=planner.table
    if table.kind!='box':raise ValueError('return requires the existing box support table')
    cup=RigidTransform(**cup_pose);local=table.world_from_shape.inverse().compose(cup)
    radius=planner.cup.bottom_radius_m;half=[d/2 for d in table.dimensions_m]
    edge=min(half[i]-abs(local.position[i])-radius for i in (0,1))
    height=local.position[2]-half[2]
    tilt=math.acos(max(-1.,min(1.,local.rotate((0.,0.,1.))[2])))
    goal_error=math.dist(cup.position,goal['position'])
    passed=(abs(height)<=acceptance['surface_tolerance_m'] and edge>=acceptance['edge_margin_m']
            and tilt<=acceptance['tilt_rad'] and goal_error<=.002+1e-7)
    return dict(performed=True,passed=passed,table_shape=table.shape_path,support_height_error_m=height,
        edge_clearance_m=edge,cup_tilt_rad=tilt,goal_error_m=goal_error,goal_tolerance_m=.002,
        limits={key:acceptance[key] for key in ('surface_tolerance_m','edge_margin_m','tilt_rad')},
        scope='planned cup bottom and original table support footprint; not measured contact force')


def run(parent, parent_source, source, geometry_path, native_code, assets, output, *, geometry_source=None):
    load_source,current=current_geometry_inputs(parent,parent_source,source,geometry_path,assets,geometry_source)
    from playground_pipeline import save
    from playground_geometry import load
    from physical_cup_maintenance import reverse_stages
    from cup_return_offset import NativeReturnKinematics,apply_offset
    from motiongen_plan import prepare_model,world_config
    from gpu_acceptance import closing_geometry,jaw_sweep,SupportBounds
    from cup_envelope import attach
    from real_contract import fixed_right_collision_policy
    started=time.perf_counter();output=Path(output);context=source['maintenance_context'];scene=context['current_scene']
    planner,observation,data=load(geometry_path,load_source,native_code,assets,
                                  **({'maintenance_source':source} if current else {}))
    from robot_sim.asset_cfg.objects.coffee_machine import RigidTransform
    from robot_sim.asset_cfg.robots.ac_one_spec import ac_one_asset_spec
    urdf=ac_one_asset_spec(assets).urdf_path
    if not same_pose(scene['machine_pose'],load_source['scene']['environment_from_machine_root']):
        raise ValueError('machine moved from the frozen export; rebuild its geometry before return')
    if not same_pose(scene['world_from_robot'],asdict(planner.geo.mount)):
        raise ValueError('current world/base transform differs from exported geometry')
    observation=replace(observation,sample=replace(observation.sample,world_from_cup=RigidTransform(**scene['cup_pose'])))
    planner.geometry(observation)
    kine=NativeReturnKinematics(urdf,source,scene['world_from_robot'])
    reversed_stages,right_projection=bind_current_right(reverse_stages(parent,parent_source),source,kine,parent['gripper_opening_mm'])
    base=offset_base(parent,reversed_stages)
    rows=[[convert_state(row,source['calibration_profile'],to_reported=False) for row in stage['rows']]
          for stage in reversed_stages]
    if any(row['right_arm_joints']!=rows[0][0]['right_arm_joints'] or row['right_gripper']!=rows[0][0]['right_gripper']
           for stage in rows for row in stage):raise ValueError('return geometry requires the frozen right arm and jaw')
    opened=kine.positions(rows[0][0]);opening=parent['gripper_opening_mm']
    opened.update(left_joint7=opening/2000,left_joint8=opening/2000)
    profile=context.get('pickup_proxy_profile')
    prepared=prepare_model(planner,observation,data,urdf,opened,**({'pickup_profile':profile}if profile else {}))
    engine=prepared['engine'];engine.cup_contact.allow_claw_contact(True)
    import curobo_robot_model
    from a1_asset_derivation import certificate_path
    refinement=prepared['proxy'].get('local_refinement')
    parent_codes=parent['planning_backend']['code_sha256']
    code_shas={value for path,value in parent_codes.items() if Path(path).name=='curobo_robot_model.py'}
    recipe=certificate_path('a1_conservative_refinement.json',urdf)
    recipe_shas={value for path,value in parent_codes.items() if Path(path).name==recipe.name}
    promotion=prepared['proxy'].get('pickup_promotion')
    model_code_sha=hashlib.sha256(Path(curobo_robot_model.__file__).read_bytes()).hexdigest()
    code_matches=(model_code_sha in code_shas)
    if profile:
        if (profile!='pickup-c11-v1' or not isinstance(promotion,dict) or promotion.get('profile')!=profile
                or promotion.get('parent_model_code_sha256') not in code_shas
                or promotion.get('parent_asset_recipe_sha256') not in recipe_shas
                or promotion.get('model_code_sha256')!=model_code_sha
                or context.get('pickup_proxy_promotion')!={k:promotion[k] for k in
                    ('profile','recipe_file_sha256','offline_generation_digest','parent_c10_failed_result_digest',
                     'pregrasp_diagnosis_digest','source_parts_sha256','left_sphere_cap','total_sphere_cap')}):
            raise ValueError('C11 requires its explicit parent/code/recipe/CPU evidence promotion')
        code_matches=True
    if (not refinement or refinement.get('source_part_sha256')!=curobo_robot_model.LEFT3_PART_SHA256
            or not code_matches
            or hashlib.sha256(recipe.read_bytes()).hexdigest() not in recipe_shas
            or prepared['proxy']['source_parts_sha256']!=parent['planning_backend']['source_parts_sha256']):
        raise ValueError('return must reuse the exact C9 refined left_link3 proxy')
    support=SupportBounds(engine,planner,urdf)
    close_index=next(i for i,stage in enumerate(reversed_stages) if stage['name']=='close_ramp')
    grasp_row=rows[close_index][0];grasp_q=grasp_row['left_arm_joints']
    grasp_state={**opened,**dict(zip(kine.names('left'),grasp_q))}
    command_gap=parent['gripper_distance_control']['closed_metal_gap_mm']
    model_gap=command_gap-parent['gripper_distance_control']['native_travel_offset_mm']
    pickup=None
    if context.get('pickup_policy'):
        asset_receipt=json.loads((Path(assets).parent/'receipt.json').read_text())
        mesh_proof=json.loads(Path(asset_receipt['estimate']['path']).read_text())
        reversed_stages,closing,pickup=retarget_pickup(planner,observation,data,engine,prepared,kine,
            reversed_stages,parent,source,mesh_proof['mesh_path'],lambda name,value:save(output/name,value))
        base=offset_base(parent,reversed_stages)
        rows=[[convert_state(row,source['calibration_profile'],to_reported=False)for row in stage['rows']]for stage in reversed_stages]
        grasp_q=rows[close_index][0]['left_arm_joints']
    else:
        with tray_support_for_closing(planner.geo):
            closing=closing_geometry(engine,planner,grasp_state,RigidTransform(**scene['cup_pose']),data=data,
                source_digest=_digest(source),candidate_id=source['run_id']+'/tray-regrasp',closed_jaw_m=model_gap/2000)
    save(output/'grasp-contact.json',closing)
    contact=closing['terminal_state'];held_cup=RigidTransform(**closing['world_from_cup'])
    tool=planner.geo.mount.compose(planner.fk.pose(planner.fk.forward(contact)['left_link6']))
    relation=tool.inverse().compose(held_cup)
    held_reference=planner.geo.mount.inverse().compose(tool)
    goal=context['return_goal'];placement_tool=RigidTransform(**goal).compose(relation.inverse())
    request=dict(return_origin=context['return_origin'],offset_xy_mm=context['offset_world_xy_mm'],return_goal=goal,
        effective_delta_m=[goal['position'][i]-parent_source['scene']['environment_from_cup']['position'][i] for i in (0,1)],
        placement_tool_pose=dict(position=list(placement_tool.position),orientation_wxyz=list(placement_tool.orientation_wxyz)))
    shifted,proof=apply_offset(base,source,request,kine,environment='physical')
    proof.update(return_origin=deepcopy(context['return_origin']),offset_world_xy_mm=deepcopy(context['offset_world_xy_mm']),
        return_goal=deepcopy(goal),current_cup_pose=deepcopy(scene['cup_pose']),tool_from_held_cup=asdict(relation),
        grasp_contact_certificate_digest=closing['contact_certificate']['certificate_digest'],
        control_validation=deepcopy(shifted['control_validation']),parent_timeline_digest=parent['timeline_digest'],
        right_hold_projection=right_projection)
    artifact=offset_artifact(shifted,proof,reversed_stages,pickup)
    save(output/'offset-result.json',artifact)
    model_stages=[dict(stage,rows=[convert_state(row,source['calibration_profile'],to_reported=False) for row in stage['rows']])
                  for stage in artifact['stages']]
    vectors=[target_vector(row) for stage in model_stages for row in stage['rows']]
    actions=[action for stage in artifact['stages'] for action in stage['native_actions']]
    checks={key:dict(performed=False,passed=False) for key in ('robot_self','robot_world','cup_world','grasp_contact','table_support')}
    checks['fixed_right_static']=dict(performed=False,passed=None,status='skipped_by_operator')
    checks['grasp_contact']=dict(performed=True,passed=True,certificate_digest=closing['contact_certificate']['certificate_digest'])
    receipt=dict(schema='physical_cup_maintenance_geometry.v1',source_digest=_digest(source),
        parent_timeline_digest=parent['timeline_digest'],urdf_sha256=data['urdf_sha256'],scene_digest=_digest(scene),
        model_target_vectors_sha256=_digest(vectors),native_actions_sha256=_digest(actions),scope='complete_return_path',
        passed=False,checks=checks,max_sample_step_rad=0.,path_checks=[],collision_coverage=['base_link','left','right'],
        fixed_right_static_checks=dict(performed=False,skipped_by_operator=True,policy=fixed_right_collision_policy(parent_source)),
        right_static_authorization_digest=_digest(context['right_static_authorization']),
        model_proxy=refinement,right_hold_projection=right_projection,physical_contact_verified=False,
        sampling_scope='every command and stage join; joint-linear subdivisions <=0.01rad; not a mathematical swept-volume certificate')
    if current is not None:
        receipt.update(current_geometry=deepcopy(current),current_geometry_source=deepcopy(load_source),
            current_native_identity=deepcopy(data['native_identity']),parent_source_digest=_digest(parent_source))
    if pickup is not None:
        receipt.update(pickup_derivation_digest=_digest(pickup),gripper_closing=deepcopy(closing))
    if profile:
        receipt.update(pickup_proxy_promotion=deepcopy(promotion))
        from motiongen_plan import code_identity
        receipt['planning_backend']=dict(backend='curobo_motiongen',curobo_version='0.7.8',gpu_ik=True,gpu_trajectory_search=True,
            scope='new pickup plus complete GPU recheck of inherited return stages',code_sha256=code_identity(urdf),
            robot_proxy_sha256=_digest(prepared['proxy']),initial_robot_config_sha256=prepared['proxy']['robot_config_sha256'],
            source_parts_sha256=prepared['proxy']['source_parts_sha256'],pickup_proxy_profile=profile,
            promotion_digest=_digest(promotion))
        checks['approach_source']=dict(performed=False,passed=False,stage_names=[])
    if receipt['fixed_right_static_checks']['policy'] is None:raise ValueError('same-task fixed-right static exception is required')
    save(output/'geometry-receipt.json',receipt)
    previous=None;attached=False;placed=None;geometry_gap=None
    try:
        # Owner initialization follows this same position-space line with a
        # smooth time law. Include the fresh arm and jaw feedback, not merely
        # the first already-frozen parent target. No execution commands change.
        initial_samples,initial_step=interpolated_rows([model_stages[0]['rows'][0]],source['initial_state']['model'],
            maximum_gap_step_mm=.1)
        initial_checks=[]
        for sample in initial_samples:
            state=kine.positions(sample)
            engine.update_locked_joints({key:state[key] for key in engine.config['kinematics']['lock_joints']},update_right=True)
            frames=planner.fk.forward(state)
            fixed=[dict(mesh,base_from_shape=asdict(planner.fk.pose(frames[mesh['link']]))) for mesh in prepared['fixed_meshes']]
            engine.update_world(world_config(planner,RigidTransform(**scene['cup_pose']),fixed)[0])
            path=engine.validate_continuous_path([sample['left_arm_joints']])
            support_result=support.check([sample['left_arm_joints']])
            # The independent fixed-right gate remains the approved exception;
            # all moving-left/cup interactions use its actual sampled geometry.
            left_support=[row for row in support_result['checks'] if row['pair'][0].startswith('left_')]
            if not left_support or not all(row['passed'] for row in left_support):
                raise ValueError('fresh initialization left support geometry failed')
            initial_checks.append(dict(gpu=path,left_support=left_support))
        receipt['initialization_query']=dict(performed=True,passed=True,source_initial_model=source['initial_state']['model'],
            target_model=model_stages[0]['rows'][0],sampled_model_rows_sha256=_digest(initial_samples),
            samples=len(initial_samples),max_sample_step_rad=initial_step,maximum_gap_step_mm=.1,
            maximum_single_finger_step_m=.00005,checks=initial_checks,
            scope='existing Owner initialization position-space line; current right geometry at each sample; static-right gate skipped')
        receipt['max_sample_step_rad']=initial_step
        save(output/'geometry-receipt.json',receipt)
        for index,stage in enumerate(model_stages):
            samples,step=interpolated_rows(stage['rows'],previous)
            receipt['max_sample_step_rad']=max(receipt['max_sample_step_rad'],step)
            name=stage['name'];q=[row['left_arm_joints'] for row in samples]
            approach_source=None
            if profile and name in ('approach_pre','approach','preclose_settle'):
                engine.cup_contact.allow_claw_contact(True)
                for sample in samples:
                    frames={key:planner.fk.pose(value) for key,value in planner.fk.forward(kine.positions(sample)).items()}
                    audit=planner.geo.audit(frames,RigidTransform(**scene['cup_pose']),grasp_contact=False,support_contact=True)
                    if not audit.free:
                        save(output/'approach-source-failure.json',dict(stage=name,sample=sample,violations=audit.violations))
                        raise ValueError('exact native open-hand approach geometry failed: '+name)
                approach_source=dict(performed=True,passed=True,samples=len(samples),maximum_step_rad=step,
                    sampled_model_rows_sha256=_digest(samples),source_parts_sha256=prepared['proxy']['source_parts_sha256'],
                    cup_profile_sha256=planner.cup.profile_sha256,grasp_contact=False,support_contact=True)
                checks['approach_source']['stage_names'].append(name)
                checks['approach_source'].update(performed=True,
                    passed=set(checks['approach_source']['stage_names'])=={'approach_pre','approach','preclose_settle'})
            if profile and name=='close_ramp':engine.cup_contact.allow_claw_contact(False)
            if name=='close_and_hold':
                engine.update_world(world_config(planner,held_cup,prepared['fixed_meshes'])[0])
                engine.update_locked_joints({key:contact[key] for key in engine.config['kinematics']['lock_joints']})
                attachment=attach(engine,grasp_q,prepared['cup_envelope'])
                attachment['contact']=engine.cup_contact.certify_held(engine,grasp_q,closing['contact_certificate'])
                if pickup is not None:engine.cup_contact.allow_claw_contact(True)
                save(output/'attachment.json',attachment);attached=True;geometry_gap=contact['left_joint7']*2000
            if name=='open':
                final=kine.positions(stage['rows'][0]);final.update(left_joint7=contact['left_joint7'],left_joint8=contact['left_joint8'])
                placed=planner.geo.mount.compose(planner.fk.pose(planner.fk.forward(final)['left_link6'])).compose(relation)
                checks['table_support']=table_support(planner,asdict(placed),goal,parent['native_identity']['acceptance'])
                if not checks['table_support']['passed']:raise ValueError('new return endpoint lacks original table support/goal acceptance')
                engine.generator.detach_object_from_robot(link_name='attached_object');attached=False
                engine.update_world(world_config(planner,placed,prepared['fixed_meshes'])[0])
            changes=any(abs(row['left_gripper']-samples[0]['left_gripper'])>1e-10 for row in samples)
            if changes:
                if any(max(abs(a-b) for a,b in zip(row['left_arm_joints'],q[0]))>1e-10 for row in samples):
                    raise ValueError('return jaw sweep must remain at one arm pose')
                if name=='close_ramp':
                    jaw=deepcopy(closing['gpu_sweep'])
                    jaw['commanded_model_gap_mm']=model_gap
                    jaw['geometry_stops_before_command_m']=closing['contact_geometry_stops_m']
                    geometry_gap=contact['left_joint7']*2000
                else:
                    start=contact['left_joint7'] if name=='open' else samples[0]['left_gripper']/2000
                    jaw=jaw_sweep(engine,planner,q[0],start,samples[-1]['left_gripper']/2000)
                    geometry_gap=samples[-1]['left_gripper']
                if not jaw['passed']:raise ValueError('return jaw geometry rejected: '+name)
                if name in ('close_ramp','open') or jaw.get('source_bound_used_samples',0)>0:
                    selected=getattr(engine,'grasp_contact_certificate',None)
                    if selected is None or selected.get('certificate_digest')!=closing['contact_certificate']['certificate_digest']:
                        raise ValueError('jaw path source contact certificate differs from this selected grasp')
                    jaw['contact_certificate_digest']=selected['certificate_digest']
            else:jaw=None
            if name=='close_ramp':
                # The source closing proof includes its geometric centering.
                # Subsequent fixed-pose queries must use that same terminal cup.
                engine.update_world(world_config(planner,held_cup,prepared['fixed_meshes'])[0])
            if not attached:
                gap=(geometry_gap if name=='close_ramp' else stage['rows'][-1]['left_gripper'])
                locks=dict(engine.config['kinematics']['lock_joints'],left_joint7=gap/2000,left_joint8=gap/2000)
                engine.update_locked_joints(locks)
            if name=='home':engine.cup_contact.allow_claw_contact(False)
            engine.cup_contact.assert_ready(engine)
            orientation=None
            if attached and pickup is not None:
                orientation={}
                engine.validate_held_path(q,held_reference.position,held_reference.orientation_wxyz,
                    name='return_held',receipt=orientation)
            composed=jaw is not None and name in ('close_ramp','open')
            path=(fixed_arm_jaw_path_check(engine,samples,jaw,closing['contact_certificate'],stage_name=name)
                if composed else engine.validate_continuous_path(q))
            support_result=support.check(q)
            item=dict(index=index,name=name,substage=stage['substage'],model_rows_sha256=_digest(stage['rows']),
                complete_sampled_model_rows_sha256=_digest(samples),samples=len(samples),maximum_step_rad=step,
                cup_mode=('closing_sweep'if name=='close_ramp'else'opening_sweep')if composed else('held'if attached else'stationary'),
                gpu=path,support=support_result,jaw=jaw,held_orientation=orientation)
            if approach_source is not None:item['approach_source']=approach_source
            receipt['path_checks'].append(item)
            save(output/'geometry-receipt.json',receipt)
            if not path['passed'] or not support_result['passed']:raise ValueError('return full-path geometry failed: '+stage['substage'])
            previous=stage['rows'][-1]
        if placed is None:raise ValueError('full return did not contain its table release')
        for key in ('robot_self','robot_world','cup_world'):
            checks[key]=dict(performed=True,passed=True,checked_stages=len(model_stages),
                sampled_positions=len(initial_samples)+sum(row['samples'] for row in receipt['path_checks']),
                initialization_included=True,
                scope='native conservative robot plus frozen inward cup; fixed right and chassis stay in moving-left/cup queries')
        receipt.update(passed=all(checks[key]['performed'] and checks[key]['passed'] for key in
                                 ('robot_self','robot_world','cup_world','grasp_contact','table_support')),
            expected_placed_cup=asdict(placed),held_tool_from_cup=asdict(relation),wall_seconds=time.perf_counter()-started)
        save(output/'geometry-receipt.json',receipt)
        return artifact,receipt
    except Exception as error:
        receipt.update(passed=False,error=f'{type(error).__name__}: {error}',wall_seconds=time.perf_counter()-started)
        save(output/'geometry-receipt.json',receipt)
        raise


def main():
    import argparse
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('parent-plan','parent-source','source','input','native-code','assets','output'):
        parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--geometry-source',type=Path)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    from playground_geometry import configure_native
    from playground_pipeline import save
    configure_native(args.native_code)
    try:
        read=lambda path:json.loads(path.read_text())
        _,receipt=run(read(args.parent_plan),read(args.parent_source),read(args.source),args.input,args.native_code,args.assets,args.output,
            geometry_source=read(args.geometry_source) if args.geometry_source is not None else None)
        save(args.output/'result.json',dict(status='geometry-ready',passed=receipt['passed'],
            geometry_receipt_sha256=hashlib.sha256((args.output/'geometry-receipt.json').read_bytes()).hexdigest()))
    except Exception as error:
        save(args.output/'result.json',dict(status='failed',passed=False,error=f'{type(error).__name__}: {error}',automatic_retry=False))
        raise


if __name__=='__main__':main()
