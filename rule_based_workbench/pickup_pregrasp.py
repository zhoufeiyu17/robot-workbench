"""Pickup-only support-plane recipe and bounded original-target diagnosis."""
from copy import deepcopy
import math,re
from integration import _digest,_qrotate,_transform
from cup_return_replay import _pose_values

RULE='support-plane-separation-v1'
CLEARANCE_M=.0001  # Existing RelativeGeometry.audit free hand/cup boundary.

def geometry_inputs(planner,data,source):
    from motiongen_targets import _hand_points_tool
    hand=[dict(name=name,vertices=[[float(x)for x in v]for v in points])for name,points in _hand_points_tool(planner,planner.open_position_m)]
    expected={p['name']for p in data['parts']if p['link']in ('left_link6','left_link7','left_link8')}
    if {row['name']for row in hand}!=expected:raise ValueError('complete original open-hand vertices required')
    bands=[]
    for band in planner.cup.bands:
        vertices,faces=band.mesh();center=(band.z_min_m+band.z_max_m)/2
        bands.append(dict(name=band.shape_path,vertices=[[float(v[0]),float(v[1]),float(v[2])+center]for v in vertices]))
    return dict(hand=hand,bands=bands,open_jaw_m=planner.open_position_m,
        band_profiles=[dict(name=b.shape_path,z_min_m=b.z_min_m,z_max_m=b.z_max_m,radius_m=b.radius_m)for b in planner.cup.bands],
        max_band_radius_m=max(b.radius_m for b in planner.cup.bands),
        source_parts_sha256=_digest(data['parts']),cup_geometry_sha256=_digest(data['cup']),
        cup_profile_sha256=planner.cup.profile_sha256,
        geometry_export_sha256=source['maintenance_context']['current_geometry']['export_sha256'],
        coordinate_frame='cup_bottom_center',units='m',hand_representation='original_convex_body_vertices')

def derive(inputs,cup_from_tool,world_from_cup):
    try:
        cup_from_tool={k:list(cup_from_tool[k])for k in ('position','orientation_wxyz')}
        world_from_cup={k:list(world_from_cup[k])for k in ('position','orientation_wxyz')}
    except (KeyError,TypeError)as error:raise ValueError('complete support-plane rigid poses required')from error
    p,q=_pose_values(cup_from_tool);_pose_values(world_from_cup)
    if abs(sum(x*x for x in q)-1)>1e-9:raise ValueError('unit support-plane approach axis required')
    if (inputs.get('coordinate_frame')!='cup_bottom_center'or inputs.get('units')!='m'
            or inputs.get('hand_representation')!='original_convex_body_vertices'):
        raise ValueError('support plane requires original metric convex vertices in the cup frame')
    def vertices(groups):
        if not isinstance(groups,list)or not groups or len({r['name']for r in groups})!=len(groups):raise ValueError('unique complete geometry groups required')
        for group in groups:
            points=group.get('vertices')
            if not isinstance(points,list)or len(points)<4:raise ValueError('complete convex body vertices required')
            for i,v in enumerate(points):
                if len(v)!=3 or any(type(x)not in(int,float)or not math.isfinite(x)for x in v):raise ValueError('finite metric support-plane vertices required')
                yield group['name'],i,v
    n=_qrotate(q,[1.,0.,0.]);dot=lambda a,b:sum(x*y for x,y in zip(a,b,strict=True))
    hand=list(vertices(inputs['hand']));cup=list(vertices(inputs['bands']))
    profiles={r['name']:r for r in inputs.get('band_profiles',[])}
    if set(profiles)!={r['name']for r in inputs['bands']}:raise ValueError('pinned cup band profile required')
    for band in inputs['bands']:
        profile=profiles[band['name']];points=band['vertices'];radius=profile['radius_m']
        lower=[min(v[i]for v in points)for i in range(3)];upper=[max(v[i]for v in points)for i in range(3)]
        expected=[[-radius,-radius,profile['z_min_m']],[radius,radius,profile['z_max_m']]]
        if any(abs(x-y)>1e-9 for a,b in zip((lower,upper),expected,strict=True)for x,y in zip(a,b,strict=True)):
            raise ValueError('cup band vertices differ from pinned canonical frame/bounds')
    projected=[(dot(n,[a+b for a,b in zip(p,_qrotate(q,v),strict=True)]),name,i)for name,i,v in hand]
    h=max(projected,key=lambda row:row[0]);near=min((dot(n,v),name,i)for name,i,v in cup)
    radius=inputs['max_band_radius_m']
    if type(radius)not in(int,float)or not math.isfinite(radius)or radius<=0:raise ValueError('finite positive current cup radius required')
    old=max(v[0]for _,_,v in hand)+radius;r=h[0]-near[0]+CLEARANCE_M
    if not math.isfinite(r)or r<=0:raise ValueError('nonpositive support-plane retreat contradicts contact-reserve grasp')
    pre_position=[a-r*b for a,b in zip(p,n,strict=True)]
    wp,wq=_transform(world_from_cup,pre_position,q)
    result=dict(rule=RULE,inputs_sha256=_digest(inputs),cup_from_tool=deepcopy(cup_from_tool),
        world_from_cup=deepcopy(world_from_cup),direction_cup=list(n),old_retreat_m=old,retreat_m=r,
        clearance_m=CLEARANCE_M,clearance_source='native relative_geometry.RelativeGeometry.audit, free hand/cup distance',
        hand_support=dict(projection_m=h[0],part=h[1],vertex=h[2]),
        cup_support=dict(projection_m=near[0],band=near[1],vertex=near[2]),
        separation_m=near[0]-(h[0]-r),within_old_bound=r<=old+1e-12,
        world_from_pregrasp=dict(position=list(wp),orientation_wxyz=list(wq)),
        scope='whole convex hand/cup solids separated by one plane at pregrasp only; no path or robot/chassis qualification')
    result['derivation_digest']=_digest(result);return result

def validate(value,inputs,source,parent):
    current=source['maintenance_context']['current_geometry']
    if (not isinstance(value,dict)or not isinstance(inputs,dict)
            or not {'cup_from_tool','world_from_pregrasp'}.issubset(value)
            or inputs.get('geometry_export_sha256')!=current['export_sha256']
            or inputs.get('source_parts_sha256')!=parent['planning_backend']['source_parts_sha256']
            or abs(inputs.get('open_jaw_m',-1)*2000-parent['gripper_opening_mm'])>1e-9):
        raise ValueError('support-plane inputs differ from pinned current export or original robot')
    expected=derive(inputs,value['cup_from_tool'],source['maintenance_context']['current_scene']['cup_pose'])
    if value!=expected or not value['within_old_bound']:raise ValueError('support-plane proof differs from CPU recomputation')
    return value

def _finite(value):
    if isinstance(value,float)and not math.isfinite(value):return False
    if isinstance(value,dict):return all(_finite(v)for v in value.values())
    if isinstance(value,(list,tuple)):return all(_finite(v)for v in value)
    return True

def _positive(value):
    return any(_positive(v)for v in value)if isinstance(value,(list,tuple))else float(value)>0

def fixed_body_queries(engine,prepared):
    """Independent, read-only native GPU queries for chassis remapped to world meshes."""
    meshes=[m for m in prepared.get('fixed_meshes',[])if m['link']=='base_link'and re.search(r'/chassis_0/component_(?:0|1|2|32)$',m['source_part_name'])]
    if not meshes:return []
    from curobo.geom.types import Mesh,WorldConfig
    from curobo.geom.sdf.world import WorldCollisionConfig
    from curobo.geom.sdf.world_mesh import WorldMeshCollision
    from curobo.rollout.cost.primitive_collision_cost import PrimitiveCollisionCost,PrimitiveCollisionCostConfig
    queries=[]
    for item in meshes:
        pose=item['base_from_shape'];mesh=Mesh(name=item['name'],pose=[*pose['position'],*pose['orientation_wxyz']],vertices=item['vertices'],faces=item['faces'])
        if list(mesh.pose)!=prepared['diagnostic_world_mesh_poses'][item['name']]:raise ValueError('independent chassis mesh differs from active main-world base-frame pose')
        checker=WorldMeshCollision(WorldCollisionConfig(tensor_args=engine.tensor_args,world_model=WorldConfig(mesh=[mesh]),cache={'mesh':1,'obb':1}))
        cost=PrimitiveCollisionCost(PrimitiveCollisionCostConfig(weight=1.,tensor_args=engine.tensor_args,world_coll_checker=checker,activation_distance=0.,return_loss=True))
        queries.append((item,cost))
    return queries

def fixed_body_witnesses(engine,queries,receipt,candidate_id):
    if not queries:return []
    q=receipt['failed_ik_joints']
    if len(q)!=1 or len(q[0])!=6:raise ValueError('one actual failed GPU IK branch required')
    spheres=engine.generator.kinematics.get_state(engine.tensor_args.to_device(q)).get_link_spheres().unsqueeze(1)
    result=[]
    for link in ('left_link3','left_link4'):
        ids=[];expected_radii=[];buffers={}
        for alias in engine.config['kinematics']['collision_link_names']:
            if engine.config['kinematics']['extra_links'][alias]['parent_link_name']==link:
                ids.extend(engine.generator.kinematics.kinematics_config.get_sphere_index_from_link_name(alias).cpu().tolist())
                buffer=engine.config['kinematics']['collision_sphere_buffer'][alias];buffers[alias]=buffer
                expected_radii.extend(s['radius']+buffer for s in engine.config['kinematics']['collision_spheres'][alias])
        if not ids:raise ValueError('native failed-IK moving link spheres missing')
        query=spheres.index_select(2,engine.torch.tensor(ids,device=spheres.device,dtype=engine.torch.long))
        if not engine.torch.equal(query[0,0,:,3],engine.tensor_args.to_device(expected_radii)):
            raise ValueError('diagnostic radii differ from pinned main-solver buffered spheres')
        for item,cost in queries:
            value=cost(query).detach()
            if not engine.torch.isfinite(value).all().item():raise ValueError('nonfinite fixed-chassis GPU diagnostic')
            maximum=float(value.max())
            if maximum>0:result.append(dict(candidate_id=candidate_id,moving_link=link,fixed_part=item['source_part_name'],
                fixed_part_sha256=item['source_part_sha256'],constraint_max=maximum,
                sphere_ids=ids,sphere_radii_m=query[0,0,:,3].cpu().tolist(),pinned_buffers_m=buffers,
                base_from_shape=item['base_from_shape'],curobo_version='0.7.8',activation_distance=0.,return_loss=True,
                backend='independent native GPU world-mesh query; same robot self pair, main solver unchanged'))
    return result

def original_target_diagnosis(engine,prepared,candidates,q0,base_from_world,save):
    """Same plan_pose path and constraints; only observed failed GPU branches gate phase 1."""
    from dataclasses import asdict
    from robot_sim.asset_cfg.objects.coffee_machine import RigidTransform
    parts={r['alias']:r for r in prepared['proxy']['part_reports']};rows=[];witnesses=[]
    names=list(engine.generator.joint_names)
    if names!=[f'left_joint{i}'for i in range(1,7)]or names!=engine.config['kinematics']['cspace']['joint_names']or engine.config['kinematics']['base_link']!='base_link':
        raise ValueError('failed GPU IK vector must use the same six active model joints/base frame')
    fixed_queries=fixed_body_queries(engine,prepared)
    expected='MotionGen pickup_approach_pre failed: MotionGenStatus.IK_FAIL'
    result=dict(schema='pickup_original_pregrasp_diagnosis.v1',passed=False,original_candidates_sha256=_digest(candidates),
        q0=list(q0),active_model_joint_names=names,frame='base_link',rows=rows,witnesses=witnesses,scope='first returned failed GPU IK branch and native diagnostic pairs; not a global cause proof')
    try:
        for candidate in candidates:
            row=dict(candidate_id=candidate['candidate_id'],status='pending');rows.append(row)
            engine.cup_contact.allow_claw_contact(False)
            count=len(engine.receipts)
            try:
                engine.plan_pose(q0,[asdict(base_from_world.compose(RigidTransform(**candidate['world_from_pregrasp'])))],name='pickup_approach_pre')
            except ValueError as error:
                row['error']=str(error)
                if str(error)!=expected:raise ValueError('original-target diagnostic unexpected refusal: '+str(error))from error
            else:raise ValueError('original-target diagnostic unexpectedly passed; phase 1 not authorized by its witness gate')
            if len(engine.receipts)!=count+1:raise ValueError('original-target diagnostic receipt missing')
            receipt=deepcopy(engine.receipts[-1]);row['receipt']=receipt
            if (not _finite(receipt)or receipt.get('failure_diagnostic_error')or not receipt.get('failed_ik_constraints')
                    or not receipt.get('failed_ik_joints')or 'proxy_overlap_diagnostic'not in receipt):
                raise ValueError('original-target diagnostic missing or nonfinite GPU witness')
            for pair in receipt['proxy_overlap_diagnostic']:
                pair_parts=[parts.get(a)for a in pair['aliases']]
                if any(p is None for p in pair_parts):raise ValueError('diagnostic sphere alias missing from pinned model')
                base=next((p for p in pair_parts if p['link']=='base_link'),None)
                arm=next((p for p in pair_parts if p['link']in('left_link3','left_link4')),None)
                if base and arm and re.search(r'/chassis_0/component_(?:0|1|2|32)$',base['name'])and pair['proxy_overlap_m']>0:
                    witnesses.append(dict(candidate_id=candidate['candidate_id'],pair=pair,parts=pair_parts))
            fixed=fixed_body_witnesses(engine,fixed_queries,receipt,candidate['candidate_id'])
            row['fixed_chassis_gpu_queries']=fixed
            if fixed and not _positive(receipt['failed_ik_constraints']['world']):
                raise ValueError('isolated body witness differs from original full-world failure cost')
            witnesses.extend(fixed)
            row['status']='expected_ik_rejected';save('original-target-diagnosis.json',result)
        if not witnesses:raise ValueError('original-target diagnostic has no matching chassis/left3/4 GPU witness; phase 1 stopped')
        result['passed']=True;return result
    except Exception as error:
        result.update(error=str(error),error_type=type(error).__name__);raise
    finally:
        engine.cup_contact.allow_claw_contact(False)
        save('original-target-diagnosis.json',result);save('pickup-motiongen-receipts.json',deepcopy(engine.receipts))
