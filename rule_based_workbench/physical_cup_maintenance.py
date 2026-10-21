"""Physical cup-return data contracts; no SDK, native FK, GPU, or implicit execution."""
from copy import deepcopy
import math
import re

from integration import _digest, SIM_POLICY_ID, validate_profile, _validate_timeline
from joint_space import contract, convert_state, gripper_offset_mm, validate_initial_state, state_near
from gripper_distance_plan import HOLDING_STAGES, NATIVE_DISTANCE_ENCODING, release_targets
from cup_return_replay import REVERSE_STAGE, _pose_values

SOURCE_SCHEMA='physical_cup_maintenance_source.v1'
PLANNER_ID='physical-cup-return/v1'
CONTEXT_SCHEMA='physical_cup_maintenance_context.v1'
SCOPE='maintenance_current_state_and_historical_parent'
CURRENT_GEOMETRY_SCHEMA='physical_cup_current_geometry.v1'
PICKUP_POLICY='current-cup-candidates-v1'
PICKUP_TRANSITION_POLICY='motiongen-to-inherited-carry-entry-v1'
PICKUP_STAGES={'approach_pre','approach','preclose_settle','close_ramp','close_and_hold','lift','transport'}


def pickup_binding(source):
    current=current_geometry_binding(source)
    if current is None:raise ValueError('current pickup requires bound current geometry')
    return dict(source_digest=_digest(source),current_geometry_digest=_digest(current),
        parent_timeline_digest=source['maintenance_context']['parent_timeline_digest'],
        native_identity_digest=current['native_identity_digest'],object_mesh_sha256=current['object_mesh_sha256'])


def validate_pickup_derivation(value,source,parent):
    if source['maintenance_context'].get('pickup_policy')!=PICKUP_POLICY:
        if value is not None:raise ValueError('pickup derivation requires explicit current-cup policy')
        return None
    if (not isinstance(value,dict) or value.get('schema_version')!='physical_cup_pickup_derivation.v1'
            or value.get('binding')!=pickup_binding(source) or value.get('actual_fk_qualified') is not True
            or value.get('metal_gap_mm')!=source['grasp_command']['metal_gap_mm']
            or not _sha(value.get('contact_certificate_digest'))
            or value.get('derivation_digest')!=_digest({k:v for k,v in value.items() if k!='derivation_digest'})):
        raise ValueError('current pickup provenance or binding differs')
    indices=value.get('affected_parent_indices');base=reverse_stages(parent,source['historical_parent_source'])
    if (not isinstance(indices,list) or not indices or len(set(indices))!=len(indices)
            or any(type(i) is not int or not 0<=i<len(base) or base[i]['name']not in PICKUP_STAGES for i in indices)
            or set(value.get('stages',{}))!={str(i)for i in indices}
            or any(not _sha(row.get(k))for row in value['stages'].values()for k in ('rows_sha256','native_actions_sha256'))):
        raise ValueError('explicit valid pickup stage indices and command hashes required')
    order=value.get('candidate_order');attempts=value.get('attempts');selected=value.get('selected_candidate',{}).get('candidate_id')
    if (not isinstance(order,list)or not order or len(order)>20 or len(order)!=len(set(order))
            or any(not isinstance(i,str)or not i for i in order)or not isinstance(attempts,list)or not attempts
            or [a.get('candidate_id')for a in attempts]!=order[:len(attempts)]
            or any(a.get('status')!='rejected'for a in attempts[:-1])
            or attempts[-1].get('status')!='qualified_pickup'or attempts[-1].get('candidate_id')!=selected):
        raise ValueError('pickup attempts must preserve the bounded existing candidate order')
    if source['maintenance_context'].get('pregrasp_policy'):
        from pickup_pregrasp import validate
        plane=validate(value.get('pregrasp_derivation',{}),value.get('pregrasp_inputs',{}),source,parent)
        selected=value['selected_candidate'];diagnostic=value.get('original_target_diagnosis',{})
        if (selected.get('world_from_pregrasp')!=plane['world_from_pregrasp']
                or selected.get('cup_from_tool')!=plane['cup_from_tool']or diagnostic.get('passed')is not True
                or [r['candidate_id']for r in diagnostic.get('rows',[])]!=order or not diagnostic.get('witnesses')):
            raise ValueError('pickup pregrasp or original-target witness gate differs')
    policy=source['maintenance_context'].get('pickup_transition_policy')
    if policy is not None:
        transition=value.get('carry_entry_derivation') or {}
        index=next(i for i,stage in enumerate(base)if stage['name']=='lift')
        inherited=convert_state(base[index]['rows'][-1],source['calibration_profile'],to_reported=False)
        if (policy!=PICKUP_TRANSITION_POLICY or transition.get('policy')!=policy
                or transition.get('inherited_stage_index')!=index
                or index not in indices
                or transition.get('inherited_model_joints')!=inherited['left_arm_joints']
                or transition.get('urdf_sha256')!=source['calibration_profile']['model_binding']['urdf_sha256']
                or transition.get('actual_grasp_model_joints')!=value['actual_fk']['model_joints']
                or transition.get('contact_certificate_digest')!=value['contact_certificate_digest']
                or transition.get('tray_world_obstacle_retained') is not True
                or transition.get('free_xyz') is not True or 'pickup_lift_height_m' in value
                or type(transition.get('join_length_l2_rad')) not in (float,int)
                or not math.isfinite(transition['join_length_l2_rad']) or transition['join_length_l2_rad']<0
                or transition.get('native_fk_goal_check',{}).get('passed') is not True):
            raise ValueError('pickup transition must bind the existing carry-entry joint goal and unchanged held-cup world')
        check=transition['native_fk_goal_check']
        if (not _sha(transition.get('fk_implementation_sha256')) or any(
                type(check.get(key)) not in (float,int) or not math.isfinite(check[key]) or not 0<=check[key]<=limit
                for key,limit in (('position_error_m',.0020001),('angle_error_rad',math.radians(.5)+1e-6)))):
            raise ValueError('pickup transition requires bound native FK source and existing endpoint tolerances')
        if transition.get('actual_held_cup_goal_check',{}).get('passed') is not True:
            raise ValueError('pickup transition requires the actual held-cup world goal check')
        _pose_values(transition.get('world_from_tool_goal'))
    return value


def current_geometry_binding(source):
    context=source.get('maintenance_context',{})
    if 'current_geometry' not in context:return None
    value=context['current_geometry']
    hashes=('source_digest','export_sha256','asset_pins_sha256','asset_receipt_sha256',
            'object_mesh_sha256','native_identity_digest')
    if (not isinstance(value,dict) or set(value)!={'schema_version','revision','parent_role',*hashes}
            or value['schema_version']!=CURRENT_GEOMETRY_SCHEMA or value['parent_role']!='trajectory_seed_only'
            or not isinstance(value['revision'],str) or re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,127}',value['revision']) is None
            or any(not _sha(value[k]) for k in hashes)):
        raise ValueError('explicit versioned current geometry binding required')
    return value


def validate_current_geometry_source(source,current):
    binding=current_geometry_binding(source)
    if (binding is None or not isinstance(current,dict) or _digest(current)!=binding['source_digest']
            or current.get('schema_version')!='physical_collection_source.v1'
            or current.get('environment')!='physical' or current.get('simulation_only') is not False
            or current.get('adapter')!='operator_assisted_rgbd' or current.get('training_observation') is not False
            or any(current.get(k)!=source.get(k) for k in ('robot_id','camera_serials','calibration_profile','initial_state','grasp_command'))):
        raise ValueError('current geometry must use the same physical calibration, initial seal and unchanged grip')
    scene=source['maintenance_context']['current_scene'];observed=current.get('scene',{})
    if (observed.get('environment_from_cup')!=scene['cup_pose']
            or observed.get('environment_from_machine_root')!=scene['machine_pose']):
        raise ValueError('current geometry source differs from the observed maintenance scene')
    from integration import validate_assisted_scene
    from real_contract import validate_assisted_cup_geometry,fixed_right_collision_policy
    receipt=validate_assisted_scene(observed)
    cup=current.get('cup_geometry',{})
    validate_assisted_cup_geometry(current,dict(profile_sha256=cup.get('profile_sha256'),bands=cup.get('bands')))
    if (receipt['calibration_profile_digest']!=_digest(source['calibration_profile'])
            or receipt['initial_state_digest']!=_digest(source['initial_state'])
            or any(receipt[k]!=current[k] for k in ('robot_id','camera_serials','grasp_command'))):
        raise ValueError('current geometry assistance does not bind this calibration and fresh initial seal')
    policy=fixed_right_collision_policy(current)
    if policy is None or policy['approval_id']!=source['maintenance_context']['right_static_authorization']['approval_id']:
        raise ValueError('current geometry cannot expand the historical fixed-right authorization')
    return current


def validate_current_native_identity(source,parent,native):
    binding=current_geometry_binding(source)
    if (binding is None or not isinstance(native,dict) or _digest(native)!=binding['native_identity_digest']
            or any(native.get(k)!=parent['native_identity'].get(k) for k in ('control','acceptance'))):
        raise ValueError('current native identity or original control/acceptance policy differs')
    return native


def is_maintenance(value):
    return isinstance(value,dict) and (value.get('schema_version')==SOURCE_SCHEMA or value.get('planner_id')==PLANNER_ID)


def _sha(value):
    return isinstance(value,str) and re.fullmatch('[0-9a-f]{64}',value) is not None


def build_source(parent_source,initial,context,run_id):
    if (parent_source.get('schema_version')!='physical_collection_source.v1'
            or parent_source.get('environment','physical')!='physical' or parent_source.get('simulation_only') is True):
        raise ValueError('physical maintenance requires a physical collection parent')
    context=deepcopy(context)
    context.setdefault('right_hold_reported',deepcopy(initial['planned_reported']['right_arm_joints']))
    inherited=parent_source.get('scene',{}).get('source',{}).get('assistance_receipt',{}).get('fixed_right_collision_policy')
    if inherited is not None:
        context.setdefault('right_static_authorization',dict(mode='skip_fixed_static_checks',approved_by='user',
            approval_id=inherited['approval_id'],parent_policy_digest=_digest(inherited),
            scope='same_left_arm_cup_task; fresh fixed-right reference; moving-left/cup checks retained'))
    origin=context['return_origin'];goal=deepcopy(origin)
    goal['position'][:2]=[q+d/1000 for q,d in zip(origin['position'][:2],context['offset_world_xy_mm'],strict=True)]
    if context.get('return_goal',goal)!=goal:raise ValueError('maintenance goal differs from fixed origin plus world XY offset')
    context['return_goal']=goal
    source=dict(schema_version=SOURCE_SCHEMA,environment='physical',simulation_only=False,lane='physical-real',
        evidence_source='physical_rgbd',adapter='physical_cup_maintenance',observation_scope=SCOPE,training_observation=False,
        run_id=run_id,sequence=0,robot_id=parent_source['robot_id'],camera_serials=deepcopy(parent_source['camera_serials']),
        calibration_profile=deepcopy(parent_source['calibration_profile']),initial_state=deepcopy(initial),
        parent_source_digest=_digest(parent_source),historical_parent_source=deepcopy(parent_source),
        grasp_command=deepcopy(parent_source.get('grasp_command') or dict(metal_gap_mm=65.,units='metal_inner_gap_mm',controlled_gripper='left_gripper')),
        maintenance_context=context,scene=dict(schema_version='physical_maintenance_scene.v1',
            scene_digest=_digest(context['current_scene']),observation_scope=SCOPE))
    validate_source(source)
    return source


def validate_source(source,binding=None):
    if (source.get('schema_version')!=SOURCE_SCHEMA or source.get('environment')!='physical'
            or source.get('simulation_only') is not False or source.get('adapter')!='physical_cup_maintenance'
            or source.get('evidence_source')!='physical_rgbd' or source.get('lane')!='physical-real'
            or source.get('observation_scope')!=SCOPE or source.get('training_observation') is not False
            or not isinstance(source.get('run_id'),str) or not source['run_id'].strip() or source.get('sequence')!=0):
        raise ValueError('explicit physical maintenance source required; historical RGB-D is not a fresh training capture')
    parent=source.get('historical_parent_source',{});profile=source.get('calibration_profile')
    if (parent.get('schema_version')!='physical_collection_source.v1' or parent.get('simulation_only') is True
            or parent.get('environment','physical')!='physical' or _digest(parent)!=source.get('parent_source_digest')
            or any(source.get(k)!=parent.get(k) for k in ('robot_id','camera_serials','calibration_profile'))):
        raise ValueError('maintenance parent/calibration/robot identity differs')
    validate_profile(profile)
    if profile['source'].get('simulation_only') is True:raise ValueError('physical calibration required')
    initial=validate_initial_state(source.get('initial_state'),profile)
    if initial.get('disabled_grippers')!=['right'] or initial.get('held_arms')!=['right']:
        raise ValueError('physical cup maintenance requires the current disabled right gripper and held right arm')
    if not initial.get('reported_joint_limits'):raise ValueError('maintenance requires current observed Owner joint limits')
    command=source.get('grasp_command',{})
    expected=parent.get('grasp_command') or dict(metal_gap_mm=65.,units='metal_inner_gap_mm',controlled_gripper='left_gripper')
    if command!=expected or command not in [dict(metal_gap_mm=g,units='metal_inner_gap_mm',controlled_gripper='left_gripper') for g in (61.,65.)]:
        raise ValueError('maintenance grip must preserve the physical parent command')
    c=source.get('maintenance_context',{})
    current_geometry_binding(source)
    if c.get('pickup_policy')not in (None,PICKUP_POLICY):raise ValueError('unknown current pickup policy')
    if c.get('pickup_policy')and current_geometry_binding(source)is None:
        raise ValueError('pickup policy requires current geometry')
    if c.get('pregrasp_policy')is not None:
        from pickup_pregrasp import RULE
        if c['pregrasp_policy']!=RULE or c.get('pickup_policy')!=PICKUP_POLICY:
            raise ValueError('explicit current pickup support-plane policy required')
    if c.get('pickup_proxy_profile') is not None:
        if (c['pickup_proxy_profile']!='pickup-c11-v1' or not c.get('pregrasp_policy')
                or not isinstance(c.get('pickup_proxy_promotion'),dict)
                or c['pickup_proxy_promotion'].get('profile')!=c['pickup_proxy_profile']
                or _digest(c.get('pregrasp_diagnosis'))!=c['pickup_proxy_promotion'].get('pregrasp_diagnosis_digest')):
            raise ValueError('explicit C11 promotion and frozen historical GPU diagnosis required')
    if c.get('pickup_transition_policy') is not None and (c['pickup_transition_policy']!=PICKUP_TRANSITION_POLICY or c.get('pickup_policy')!=PICKUP_POLICY):
        raise ValueError('explicit existing-carry-entry pickup transition policy required')
    if (c.get('schema_version')!=CONTEXT_SCHEMA or not _sha(c.get('parent_timeline_digest'))
            or not _sha(c.get('parent_execution_digest'))):raise ValueError('sealed parent execution context required')
    if c.get('right_hold_reported')!=initial['planned_reported']['right_arm_joints']:
        raise ValueError('maintenance right reference must be the fresh observed arm; no old parent hold')
    inherited=parent.get('scene',{}).get('source',{}).get('assistance_receipt',{}).get('fixed_right_collision_policy')
    expected_authorization=(dict(mode='skip_fixed_static_checks',approved_by='user',approval_id=inherited['approval_id'],
        parent_policy_digest=_digest(inherited),scope='same_left_arm_cup_task; fresh fixed-right reference; moving-left/cup checks retained')
        if inherited is not None else None)
    if c.get('right_static_authorization')!=expected_authorization:raise ValueError('fixed-right authorization inheritance differs')
    offset=c.get('offset_world_xy_mm')
    if (not isinstance(offset,list) or len(offset)!=2 or any(type(v) not in (int,float) or not math.isfinite(v) or abs(v)>50 for v in offset)):
        raise ValueError('two finite world XY offsets within +/-50 mm required')
    origin=c.get('return_origin');_pose_values(origin);goal=c.get('return_goal');_pose_values(goal)
    expected_goal=deepcopy(origin);expected_goal['position'][:2]=[q+d/1000 for q,d in zip(origin['position'][:2],offset)]
    if goal!=expected_goal:raise ValueError('maintenance target must use the fixed origin, not accumulate offsets')
    scene=c.get('current_scene',{})
    for key in ('cup_pose','machine_pose','world_from_robot'):_pose_values(scene.get(key))
    observation=scene.get('observation',{})
    if (observation.get('environment')!='physical' or observation.get('simulation_only') is True
            or type(observation.get('observed_wall_ns')) is not int or observation['observed_wall_ns']<=0
            or not _sha(observation.get('evidence_sha256')) or not observation.get('method')):
        raise ValueError('independent physical cup/machine observation with timestamp and pinned evidence required')
    if source.get('scene')!=dict(schema_version='physical_maintenance_scene.v1',scene_digest=_digest(scene),observation_scope=SCOPE):
        raise ValueError('maintenance current scene digest differs')
    from robot_workbench.sample_clock import clock_for_source
    clock_for_source(initial,'physical')
    if binding is not None:
        if (binding.get('schema_version')!='physical_cup_maintenance_binding.v1' or binding.get('environment')!='physical'
                or binding.get('simulation_only') is True or binding.get('evidence_source')!='physical_rgbd'
                or binding.get('observation_scope')!=SCOPE or binding.get('calibration_accepted') is not True
                or binding.get('source_digest')!=_digest(source) or binding.get('initial_state')!=initial
                or binding.get('calibration_profile_digest')!=_digest(profile)
                or any(binding.get(k)!=source.get(k) for k in ('robot_id','camera_serials'))):
            raise ValueError('physical maintenance source differs from trusted current binding')
    return source


def build_binding(parent_binding,source):
    from real_contract import validate_real_source
    validate_real_source(source['historical_parent_source'],parent_binding,environment='physical')
    result=deepcopy(parent_binding)
    for key in ('operator_assistance_digest','operator_assistance_request_id'):result.pop(key,None)
    result.update(schema_version='physical_cup_maintenance_binding.v1',environment='physical',simulation_only=False,
        evidence_source='physical_rgbd',observation_scope=SCOPE,training_observation=False,source_digest=_digest(source),
        initial_state=deepcopy(source['initial_state']),historical_parent_binding_digest=_digest(parent_binding))
    validate_source(source,result)
    return result


def reverse_stages(parent,parent_source):
    """Pure reported-target reversal; no environment relabel or second zero-offset conversion."""
    starts=parent['stages'];result=[]
    if not starts or starts[0]['name']!='initial_hold' or starts[-1]['name']!='home_hold':raise ValueError('complete parent stage sequence required')
    gap=parent['gripper_distance_control']['closed_metal_gap_mm'];opening=parent_source['calibration_profile']['gripper']['max_opening_mm']
    offset=gripper_offset_mm(parent_source['calibration_profile'])
    for i in reversed(range(len(starts))):
        stage=starts[i];end=starts[i+1]['start_tick'] if i+1<len(starts) else len(parent['ticks'])
        name=REVERSE_STAGE[stage['name']];indices=list(reversed(range(stage['start_tick'],end)))
        rows=[deepcopy(parent['ticks'][j]['target']) for j in indices];actions=[deepcopy(parent['native_actions'][j]) for j in indices]
        if name in ('open','release_hold'):
            if any(any(row[k]!=rows[0][k] for k in ('left_arm_joints','right_arm_joints','right_gripper')) for row in rows):
                raise ValueError('reverse release requires stationary arms and right gripper')
            gaps=release_targets(gap,opening)[0] if name=='open' else [opening]*6
            if len(rows)>len(gaps):raise ValueError('reverse release cannot shorten the parent stationary stage')
            rows=[dict(deepcopy(rows[0]),left_gripper=value) for value in gaps]
            actions=[[*actions[0][:6],value-offset,*actions[0][7:]] for value in gaps]
            indices=[indices[0]]*len(gaps)
        result.append(dict(name=name,substage='reverse_'+(stage.get('substage') or stage['name']),rows=rows,
            native_actions=actions,parent_stage_index=i,parent_tick_indices=indices))
    return result


def parent_execution_receipt(events,parent,parent_source,events_sha256):
    """Bind preserved controller events, not a potentially empty/failed raw recorder."""
    commands=[];completion=None;terminal=None
    for row in events:
        event=row.get('event')
        if event=='command':commands.append(deepcopy(row))
        elif event=='feedback' and (row.get('frozen') or {}).get('status')=='completed':
            f=row['frozen']
            completion=dict(wall_ns=row['wall_ns'],**{key:deepcopy(f.get(key)) for key in
                ('status','timeline_digest','execution_timeline_digest','last_tick','targets_submitted','timeline_elapsed')})
        elif event=='motion_state' and row.get('status')=='READY':terminal=deepcopy(row)
    result=dict(schema_version='physical_controller_execution.v1',source_digest=_digest(parent_source),
        parent_timeline_digest=parent['timeline_digest'],controller_events_sha256=events_sha256,
        commands=commands,completion=completion,terminal=terminal,
        evidence_scope='submitted controller targets and terminal state; not raw-data validity or task success')
    validate_parent_execution(parent,parent_source,result)
    return result


def validate_parent_execution(parent,parent_source,receipt):
    from physical_motiongen import validate_evidence
    if (parent.get('environment','physical')!='physical' or parent.get('simulation_only') is True
            or parent_source.get('schema_version')!='physical_collection_source.v1'):
        raise ValueError('one physical MotionGen parent required')
    validate_evidence(parent,parent_source)
    if (receipt.get('schema_version')!='physical_controller_execution.v1'
            or receipt.get('source_digest')!=_digest(parent_source)
            or receipt.get('parent_timeline_digest')!=parent['timeline_digest']
            or not _sha(receipt.get('controller_events_sha256'))):raise ValueError('physical parent controller receipt identity differs')
    commands=receipt.get('commands');completion=receipt.get('completion') or {};terminal=receipt.get('terminal') or {}
    if not isinstance(commands,list) or not commands:raise ValueError('complete parent command evidence required')
    source_index=0;previous=-math.inf;runtime_digest=None
    for index,row in enumerate(commands):
        stamp=row.get('send_monotonic_s')
        if (row.get('event')!='command' or row.get('tick')!=index or row.get('timeline_digest')!=parent['timeline_digest']
                or type(stamp) not in (int,float) or not math.isfinite(stamp) or stamp<=previous
                or type(row.get('wall_ns')) is not int or not _sha(row.get('execution_timeline_digest'))):
            raise ValueError('parent controller command order/clock/timeline is incomplete')
        previous=stamp;runtime_digest=runtime_digest or row['execution_timeline_digest']
        if runtime_digest!=row['execution_timeline_digest']:raise ValueError('mixed parent executions')
        tick=row.get('source_tick')
        if tick is None:
            if source_index or row.get('stage')!='initialize':raise ValueError('only leading initialization may lack a parent source tick')
        else:
            if (type(tick) is not int or tick!=source_index or tick>=len(parent['ticks'])
                    or row.get('target')!=parent['ticks'][tick]['target']):
                raise ValueError('controller commands do not reproduce every frozen parent target exactly')
            source_index+=1
    if (source_index!=len(parent['ticks']) or completion.get('status')!='completed'
            or completion.get('targets_submitted') is not True or completion.get('timeline_elapsed') is not True
            or completion.get('timeline_digest')!=parent['timeline_digest']
            or completion.get('execution_timeline_digest')!=runtime_digest or completion.get('last_tick')!=len(commands)-1
            or type(completion.get('wall_ns')) is not int or completion['wall_ns']<commands[-1]['wall_ns']
            or terminal.get('event')!='motion_state' or terminal.get('status')!='READY'
            or type(terminal.get('wall_ns')) is not int or terminal['wall_ns']<completion['wall_ns']):
        raise ValueError('parent execution needs all submitted targets, completed feedback and subsequent READY')
    return receipt


def model_target_vectors(stages,source):
    from physical_motiongen import target_vector
    return [target_vector(convert_state(row,source['calibration_profile'],to_reported=False))
            for stage in stages for row in stage['rows']]


def validate_jaw_path_receipts(receipt):
    """Bind the two contact-stage substitutions; all other stages retain generic GPU checks."""
    certificate=receipt.get('gripper_closing',{}).get('contact_certificate',{})
    digest=certificate.get('certificate_digest');seen=[]
    if not digest or certificate.get('source_digest')!=receipt['source_digest']:
        raise ValueError('jaw composition requires this source closing certificate')
    for stage in receipt.get('path_checks',[]):
        gpu=stage.get('gpu',{});name=stage.get('name');composed=gpu.get('geometry_backend')=='gpu_jaw_sweep'
        if name not in ('close_ramp','open'):
            if composed:raise ValueError('jaw composition cannot replace a moving or hold stage')
            continue
        jaw=stage.get('jaw',{})
        if (not composed or gpu.get('passed')is not True or jaw.get('passed')is not True
                or gpu.get('gpu_jaw_sweep_digest')!=_digest(jaw)
                or gpu.get('contact_certificate_digest')!=digest or jaw.get('contact_certificate_digest')!=digest
                or not jaw.get('native_input_digest')or gpu.get('native_input_digest')!=jaw['native_input_digest']
                or gpu.get('samples')!=stage.get('samples')
                or stage.get('cup_mode')!=('closing_sweep'if name=='close_ramp'else'opening_sweep')):
            raise ValueError('contact-stage backend must bind its own GPU sweep, source certificate and native inputs')
        seen.append(name)
    if sorted(seen)!=['close_ramp','open']:
        raise ValueError('one independently checked closing and opening sweep required')


def _validate_geometry(receipt,stages,source,parent,pickup=None):
    expected=dict(source_digest=_digest(source),parent_timeline_digest=parent['timeline_digest'],
        urdf_sha256=source['calibration_profile']['model_binding']['urdf_sha256'],
        model_target_vectors_sha256=_digest(model_target_vectors(stages,source)),
        native_actions_sha256=_digest([row for stage in stages for row in stage['native_actions']]),
        scene_digest=_digest(source['maintenance_context']['current_scene']))
    checks=(receipt or {}).get('checks',{})
    step=(receipt or {}).get('max_sample_step_rad')
    if (not isinstance(receipt,dict) or receipt.get('schema')!='physical_cup_maintenance_geometry.v1'
            or receipt.get('scope')!='complete_return_path' or receipt.get('passed') is not True
            or any(receipt.get(k)!=v for k,v in expected.items())
            or any(receipt.get(k) is True for k in ('synthetic_fixture','offline_candidate','simulation_only'))
            or type(step) not in (int,float) or not math.isfinite(step) or not 0<step<=.01
            or any(not isinstance(checks.get(k),dict) or checks[k].get('performed') is not True or checks[k].get('passed') is not True
                   for k in ('robot_self','robot_world','cup_world','grasp_contact','table_support'))):
        raise ValueError('complete current-scene physical maintenance geometry receipt required')
    authorization=source['maintenance_context'].get('right_static_authorization')
    if authorization is not None and (receipt.get('right_static_authorization_digest')!=_digest(authorization)
            or checks.get('fixed_right_static')!=dict(performed=False,passed=None,status='skipped_by_operator')):
        raise ValueError('fixed-right static exception must remain explicitly skipped and bound to its inherited authorization')
    current=current_geometry_binding(source)
    fields=('current_geometry','current_geometry_source','current_native_identity','parent_source_digest')
    if current is None:
        if any(k in receipt for k in fields):raise ValueError('legacy maintenance cannot mix current geometry evidence')
    else:
        if receipt.get('current_geometry')!=current or receipt.get('parent_source_digest')!=source['parent_source_digest']:
            raise ValueError('current geometry receipt must close both current and historical source identities')
        validate_current_geometry_source(source,receipt.get('current_geometry_source'))
        validate_current_native_identity(source,parent,receipt.get('current_native_identity'))
    if source['maintenance_context'].get('pickup_policy'):
        validate_pickup_derivation(pickup,source,parent)
        if (receipt.get('pickup_derivation_digest')!=_digest(pickup)
                or checks['grasp_contact'].get('certificate_digest')!=pickup['contact_certificate_digest']):
            raise ValueError('full-path receipt does not bind current pickup')
        if source['maintenance_context'].get('pregrasp_policy'):
            inputs=pickup['pregrasp_inputs'];cup=receipt['current_geometry_source']['cup_geometry']
            if (inputs['cup_profile_sha256']!=cup['profile_sha256']
                    or inputs['max_band_radius_m']!=max(b['radius_m']for b in cup['bands'])
                    or inputs['band_profiles']!=[dict(name=b['shape_path'],z_min_m=b['z_min_m'],z_max_m=b['z_max_m'],radius_m=b['radius_m'])for b in cup['bands']]
                    or inputs['source_parts_sha256']!=receipt['gripper_closing']['contact_certificate']['source_parts_sha256']):
                raise ValueError('support-plane geometry differs from current full-path closing inputs')
    if source['maintenance_context'].get('pickup_proxy_profile'):
        validate_jaw_path_receipts(receipt)
        promotion=receipt.get('pickup_proxy_promotion') or {}
        backend=receipt.get('planning_backend') or {}
        if (source['maintenance_context']['pickup_proxy_promotion']!={k:promotion.get(k) for k in
                ('profile','recipe_file_sha256','offline_generation_digest','parent_c10_failed_result_digest',
                 'pregrasp_diagnosis_digest','source_parts_sha256','left_sphere_cap','total_sphere_cap')}
                or checks.get('approach_source')!=dict(performed=True,passed=True,
                    stage_names=['approach_pre','approach','preclose_settle'])
                or backend.get('pickup_proxy_profile')!=promotion.get('profile')
                or backend.get('promotion_digest')!=_digest(promotion)
                or backend.get('source_parts_sha256')!=promotion.get('source_parts_sha256')
                or backend.get('initial_robot_config_sha256')!=promotion.get('robot_config_sha256')):
            raise ValueError('C11 model promotion and exact full open-hand approach evidence required')
    if source['maintenance_context'].get('pickup_transition_policy'):
        transition=pickup['carry_entry_derivation'];index=transition['inherited_stage_index']
        actual=convert_state(stages[index]['rows'][-1],source['calibration_profile'],to_reported=False)['left_arm_joints']
        recorded=transition.get('final_encoded_model_joints')
        if not isinstance(recorded,list)or len(recorded)!=6 or any(abs(a-b)>1e-12 for a,b in zip(actual,recorded)):
            raise ValueError('pickup transition FK proof differs from the final decoded command row')
    return receipt


def _right_reference(source,offset_result):
    reference=source['maintenance_context']['right_hold_reported']
    proof=(offset_result or {}).get('derivation',{}).get('right_hold_projection')
    if proof is None:return reference
    from joint_space import offsets
    from robot_workbench.joint_bounds import project_joint_positions
    delta=offsets(source['calibration_profile'])[6:]
    model=[q+d for q,d in zip(reference,delta)]
    limits=proof.get('intersection_limits');owner=source['initial_state']['reported_joint_limits']['right_arm_joints']
    if (proof.get('environment')!='physical' or proof.get('allowance_rad')!=.01
            or proof.get('original_reported')!=reference or proof.get('original_model')!=model
            or not isinstance(limits,list) or len(limits)!=6
            or any(len(pair)!=2 or any(type(v) not in (int,float) or not math.isfinite(v) for v in pair)
                   or not lo+d-1e-12<=pair[0]<=pair[1]<=hi+d+1e-12 for pair,(lo,hi),d in zip(limits,owner,delta))):
        raise ValueError('fresh right hold projection differs from observed state/command domain')
    bounded,changes=project_joint_positions(model,limits,environment='physical')
    reported=[q-d for q,d in zip(bounded,delta)]
    for i,(lo,hi) in enumerate(owner):
        clipped=min(hi,max(lo,reported[i]))
        if abs(clipped-reported[i])>1e-12:raise ValueError('right projection exceeds current Owner limits')
        reported[i]=clipped
    if proof.get('bounded_model')!=bounded or proof.get('bounded_reported')!=reported or proof.get('changes')!=changes:
        raise ValueError('right hold projection must reproduce the existing physical position-noise rule')
    return reported


def _stages(parent,parent_source,source,offset_result):
    base=reverse_stages(parent,parent_source)
    right=_right_reference(source,offset_result)
    needs_offset=any(abs(a-b)>1e-10 for a,b in zip(source['maintenance_context']['return_goal']['position'],
        parent_source['scene']['environment_from_cup']['position']))
    changed_right=right!=base[0]['rows'][0]['right_arm_joints']
    if offset_result is None:
        if source['maintenance_context'].get('pickup_policy'):
            raise ValueError('explicit checked pickup derivation required')
        if needs_offset or changed_right:raise ValueError('offset or fresh right hold requires explicitly re-encoded geometry stages')
        return base
    if (offset_result.get('schema')!='physical_cup_return_offset.v1'
            or offset_result.get('kinematics',{}).get('urdf_sha256')!=source['calibration_profile']['model_binding']['urdf_sha256']):
        raise ValueError('physical return offset model identity differs')
    derivation=offset_result.get('derivation',{})
    pickup=validate_pickup_derivation(offset_result.get('pickup_derivation'),source,parent)
    if (any(derivation.get(k)!=source['maintenance_context'][k] for k in ('return_origin','offset_world_xy_mm','return_goal'))
            or derivation.get('parent_timeline_digest')!=parent['timeline_digest']):
        raise ValueError('offset derivation differs from the fixed origin/current request/parent')
    stages=deepcopy(offset_result.get('stages',[]));sequence=[]
    for stage in stages:
        index=stage.get('base_reverse_stage_index')
        if type(index) is not int or not 0<=index<len(base):raise ValueError('offset stage must name its reversed parent stage')
        old=base[index]
        if stage.get('parent_stage_index')!=old['parent_stage_index']:raise ValueError('offset original parent stage differs')
        if not sequence or sequence[-1]!=index:sequence.append(index)
        changed=stage.get('derived_from_parent_stage') is True
        pickup_changed=pickup is not None and index in pickup['affected_parent_indices']
        if pickup_changed:
            pins=pickup['stages'][str(index)]
            if (not changed or stage.get('name')!=old['name']
                    or _digest(stage.get('rows'))!=pins['rows_sha256']
                    or _digest(stage.get('native_actions'))!=pins['native_actions_sha256']):
                raise ValueError('pickup stage commands differ from the checked derivation')
        if changed and not pickup_changed and old['name'] not in ('lower','support_settle','open','release_hold','retreat'):
            raise ValueError('offset may only alter the table placement/release/retreat region')
        if stage.get('name') not in set(REVERSE_STAGE.values())|{'lift_hold'}:raise ValueError('unknown maintenance stage')
        if not changed:
            if stage.get('name')!=old['name'] or stage.get('parent_tick_indices')!=old['parent_tick_indices']:
                raise ValueError('unchanged reversed stage provenance differs')
            if len(stage['rows'])!=len(old['rows']):raise ValueError('unchanged stage length differs')
            for row,previous in zip(stage['rows'],old['rows'],strict=True):
                if any(row.get(k)!=previous.get(k) for k in ('left_arm_joints','left_gripper','right_gripper')):
                    raise ValueError('unchanged moving-left parent targets differ')
            if any(action[:7]!=previous[:7] for action,previous in zip(stage['native_actions'],old['native_actions'],strict=True)):
                raise ValueError('unchanged moving-left native commands differ')
        if not stage.get('rows') or len(stage['rows'])!=len(stage.get('native_actions',[])):
            raise ValueError('maintenance stage rows/actions must align')
        if any(len(row.get('right_arm_joints',[]))!=6 or any(abs(a-b)>1e-12 for a,b in zip(row['right_arm_joints'],right))
               or row.get('right_gripper')!=source['initial_state']['planned_reported']['right_gripper']
               for row in stage['rows']):raise ValueError('every maintenance row must hold the fresh right reference')
    if sequence!=list(range(len(base))):raise ValueError('offset stages must retain every parent phase in order')
    return stages


def _freeze(stages,parent,parent_source,receipt,source,offset_result,geometry):
    ticks=[];actions=[];starts=[]
    for stage in stages:
        starts.append(dict(name=stage['name'],substage=stage.get('substage',stage['name']),start_tick=len(ticks),start_s=len(ticks)/30))
        for row,action in zip(stage['rows'],stage['native_actions'],strict=True):
            ticks.append(dict(tick=len(ticks),time_s=len(ticks)/30,target=deepcopy(row)));actions.append(deepcopy(action))
    gap=parent['gripper_distance_control']['closed_metal_gap_mm'];opening=source['calibration_profile']['gripper']['max_opening_mm']
    release=release_targets(gap,opening)[1];release['initial_model_gap_mm']=gap-gripper_offset_mm(source['calibration_profile'])
    proof=dict(schema_version='physical_cup_maintenance_derivation.v1',parent_plan=deepcopy(parent),
        parent_source=deepcopy(parent_source),parent_execution=deepcopy(receipt),geometry_receipt=deepcopy(geometry),
        parent_plan_digest=_digest(parent),parent_execution_digest=_digest(receipt),
        stages_sha256=_digest(stages),offset_result=deepcopy(offset_result),
        right_reference_scope='moving-left parent reversal; newly observed fixed right arm',
        task_success='unverified',raw_data_validity='not_a_parent_motion_gate')
    pickup=(offset_result or {}).get('pickup_derivation')
    if pickup is not None:
        proof.update(pickup_derivation=deepcopy(pickup),derivation_kind='current_pickup_with_explicit_inherited_stages',
                     right_reference_scope='new current-cup pickup; same freshly observed fixed right reference')
    result=dict(schema_version='physical_cup_maintenance_plan.v1',planner_id=PLANNER_ID,policy_id=SIM_POLICY_ID,
        environment='physical',simulation_only=False,lane='physical-real',run_id=source['run_id']+'-plan',
        source_run_id=source['run_id'],source_sequence=source['sequence'],scene_digest=source['scene']['scene_digest'],
        calibration_profile_digest=_digest(source['calibration_profile']),initial_state_digest=_digest(source['initial_state']),
        ticks=ticks,native_actions=actions,stages=starts,timeline_digest=_digest(ticks),duration_s=len(ticks)/30,
        control=deepcopy(parent['control']),native_substeps=[parent['control']['substeps']]*len(ticks),
        native_action_encoding=NATIVE_DISTANCE_ENCODING,
        native_identity=deepcopy(geometry['current_native_identity'] if current_geometry_binding(source) is not None else parent['native_identity']),
        gripper_opening_mm=parent['gripper_opening_mm'],gripper_distance_control=deepcopy(parent['gripper_distance_control']),
        gripper_release=release,gripper_closing=deepcopy(geometry.get('gripper_closing',parent.get('gripper_closing',{}))),
        simulation=dict(validated=False,outcome='not_run'),motion_profile='physical-maintenance-current-pickup-30hz'if pickup else'physical-maintenance-reverse-offset-30hz',
        joint_space=contract(source['calibration_profile'],actions),maintenance_derivation=proof)
    from control_motion import validate_control
    result['control_validation']=validate_control(result)
    result['planning_validation']=dict(schema_version='geometric_path_validation.v1',passed=True,source_digest=_digest(source),
        timeline_digest=result['timeline_digest'],collision_coverage=['base_link','left','right'],max_sample_step_rad=geometry['max_sample_step_rad'],
        maintenance_derivation_digest=_digest(proof),hardware_validated=False,scope='complete checked maintenance commands; task outcome unverified')
    if source['maintenance_context'].get('pickup_proxy_profile'):
        result['planning_backend']=deepcopy(geometry['planning_backend'])
    return result


def derive_return(parent_plan,parent_source,parent_execution,source,offset_result=None,geometry_receipt=None):
    validate_source(source);validate_parent_execution(parent_plan,parent_source,parent_execution)
    if (source['parent_source_digest']!=_digest(parent_source)
            or source['maintenance_context']['parent_timeline_digest']!=parent_plan['timeline_digest']
            or source['maintenance_context']['parent_execution_digest']!=_digest(parent_execution)):
        raise ValueError('maintenance source differs from its actual parent execution')
    stages=_stages(parent_plan,parent_source,source,offset_result)
    _validate_geometry(geometry_receipt,stages,source,parent_plan,(offset_result or {}).get('pickup_derivation'))
    result=_freeze(stages,parent_plan,parent_source,parent_execution,source,offset_result,geometry_receipt)
    _validate_controls(result,source)
    return result


def _validate_controls(plan,source):
    from joint_space import validate
    from control_motion import validate_receipt
    from gripper_distance_plan import validate_distance_control
    _validate_timeline(plan['ticks'],source['calibration_profile']['gripper']['max_opening_mm'])
    validate(plan,source);validate_receipt(plan);validate_distance_control(plan,source)
    limits=source['initial_state']['reported_joint_limits']
    for tick in plan['ticks']:
        for side in ('left_arm_joints','right_arm_joints'):
            if any(not lo<=q<=hi for q,(lo,hi) in zip(tick['target'][side],limits[side],strict=True)):
                raise ValueError('maintenance target exceeds current Owner command limits')


def validate_evidence(plan,source):
    if plan.get('planner_id')!=PLANNER_ID:raise ValueError('physical maintenance planner identity required')
    proof=plan.get('maintenance_derivation',{})
    resolved=plan
    if proof.get('schema_version')=='physical_cup_maintenance_derivation_refs.v1':
        from robot_workbench.control_plan import load_artifact
        allowed={'parent_plan','parent_source','parent_execution','geometry_receipt','offset_result','pickup_derivation'}
        refs=proof.get('artifacts')
        if not isinstance(refs,dict) or not {'parent_plan','parent_source','parent_execution'}<=set(refs) or set(refs)-allowed:
            raise ValueError('bounded maintenance evidence references required')
        proof={key:deepcopy(value) for key,value in proof.items() if key not in ('artifacts','schema_version')}
        proof.update(schema_version='physical_cup_maintenance_derivation.v1',**{name:load_artifact(ref) for name,ref in refs.items()})
        resolved=dict(plan,maintenance_derivation=proof)
    if proof.get('schema_version')!='physical_cup_maintenance_derivation.v1':raise ValueError('physical maintenance derivation required')
    expected=derive_return(proof['parent_plan'],proof['parent_source'],proof['parent_execution'],source,
                           proof.get('offset_result'),proof.get('geometry_receipt'))
    if resolved!=expected:raise ValueError('physical maintenance targets or provenance differ from checked derivation')
    return plan


def execution_package(plan,directory):
    """Separate cold derivation evidence from the command-count-bounded execution JSON."""
    import hashlib,json
    from pathlib import Path
    from robot_workbench.control_plan import MAX_INPUT
    result=deepcopy(plan);proof=result['maintenance_derivation'];artifacts={};files={}
    for name in ('parent_plan','parent_source','parent_execution','geometry_receipt','offset_result','pickup_derivation'):
        if name not in proof:continue
        raw=json.dumps(proof.pop(name),allow_nan=False,separators=(',',':')).encode()
        if len(raw)>MAX_INPUT:raise ValueError('maintenance evidence artifact exceeds bounded file limit')
        sha=hashlib.sha256(raw).hexdigest();filename='evidence-'+sha+'.json';files[filename]=raw
        artifacts[name]=dict(path=str(Path(directory)/filename),sha256=sha,size_bytes=len(raw))
    proof.update(schema_version='physical_cup_maintenance_derivation_refs.v1',artifacts=artifacts)
    raw=json.dumps(result,allow_nan=False,separators=(',',':')).encode()
    if len(raw)>min(MAX_INPUT,256*1024+len(plan['ticks'])*4096):
        raise ValueError('maintenance execution plan exceeds command-count size budget')
    return raw,files
