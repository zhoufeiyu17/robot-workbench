"""Candidate-bound source geometry proofs; no simulator, IK or force claims."""
from copy import deepcopy
from dataclasses import asdict
import math
import time
from integration import _digest

POLICY=dict(schema_version='grasp_contact_reserve.v1',nominal_band_m=[.035,.1],margin_m=.002,
    required_region_m=[.037,.098],grasp_depth_m=.016,numerical_tolerance_consumed=False)
GRASP_CONTACT_POLICY=POLICY


def _checked_pose(value):
    if not isinstance(value,dict):raise ValueError('complete finite contact pose required')
    p,q=value.get('position'),value.get('orientation_wxyz')
    if (not isinstance(p,(list,tuple)) or len(p)!=3 or not isinstance(q,(list,tuple)) or len(q)!=4
            or any(type(v) not in (int,float) or not math.isfinite(v) for v in [*p,*q])
            or abs(sum(v*v for v in q)-1)>1e-7):raise ValueError('complete finite contact pose required')
    return p,q


def _rotate(q,p):
    w,x,y,z=q;vx,vy,vz=p
    tx,ty,tz=2*(y*vz-z*vy),2*(z*vx-x*vz),2*(x*vy-y*vx)
    return [vx+w*tx+y*tz-z*ty,vy+w*ty+z*tx-x*tz,vz+w*tz+x*ty-y*tx]


def inverse_pose(value):
    p,q=_checked_pose(value);q=[q[0],-q[1],-q[2],-q[3]]
    return dict(position=_rotate(q,[-v for v in p]),orientation_wxyz=q)


def _check_relation(cup_from_tool,tool_from_cup,position):
    origin,rotation=_checked_pose(cup_from_tool);point,inverse=_checked_pose(tool_from_cup)
    recovered=[a+b for a,b in zip(origin,_rotate(rotation,point))]
    expected=[rotation[0],-rotation[1],-rotation[2],-rotation[3]]
    if (max(abs(a-b) for a,b in zip(recovered,position))>1e-9
            or min(max(abs(a-sign*b) for a,b in zip(inverse,expected)) for sign in (-1,1))>1e-9):
        raise ValueError('contact source poses do not share the native centering relation')


def source_closing(planner,state,cup,*,require_passed=True):
    """Use the existing linked-jaw native sweep and its scoped 3 mm claw allowance."""
    from robot_sim.asset_cfg.objects.coffee_machine import RigidTransform
    from robot_sim.tasks.manager_based.coffee_make.relative_geometry import GeometryAudit
    from cup_contact import ALLOWANCE_M,NUMERICAL_EPS_M
    geo=planner.geo;original=geo.audit
    parts={name:(link,geometry) for link,name,geometry,_,_ in geo.whole.parts}
    bands={'cup/'+band.shape_path:(band,shape) for band,shape in geo.cup_bands}
    def audit(frames,pose,**options):
        report=original(frames,pose,**options);bad=[]
        for row in report.violations:
            if row[0] in bands and row[1] in parts and parts[row[1]][0] in ('left_link7','left_link8'):
                band,shape=bands[row[0]];link,geometry=parts[row[1]]
                band_pose=pose.compose(RigidTransform((0,0,(band.z_min_m+band.z_max_m)/2)))
                cup_obj=geo.fcl.CollisionObject(shape,geo.whole.transform(band_pose))
                hand_obj=geo.fcl.CollisionObject(geometry,geo.whole.transform(geo.mount.compose(frames[link])))
                _,witnesses=geo.cup_pair(cup_obj,hand_obj)
                if all(depth<=ALLOWANCE_M+NUMERICAL_EPS_M for _,depth in witnesses):continue
            bad.append(row)
        return GeometryAudit(not bad,tuple(bad))
    geo.audit=audit
    try:closing=geo.closing_sweep(planner.fk,state,cup)
    finally:geo.audit=original
    if require_passed and (closing.get('passed') is not True or not closing.get('samples')
            or 'world_from_cup' not in closing or 'contact_geometry_stops_m' not in closing):
        raise ValueError('complete native linked-jaw closing proof required: '+str(closing.get('error')))
    return closing


def source_contact_certificate(planner,data,closing,initial_cup):
    """Extract the same source-mesh terminal/wrist proof as contact preparation."""
    from robot_sim.asset_cfg.objects.coffee_machine import RigidTransform
    from motiongen_plan import world_config
    from cup_contact import cup_geometry_digest,ALLOWANCE_M
    if closing.get('passed') is not True:raise ValueError('passed native source closing required')
    geo=planner.geo;state=closing['terminal_state']
    if state['left_joint7']!=state['left_joint8']:raise ValueError('linked terminal jaws required')
    frames={name:planner.fk.pose(value) for name,value in planner.fk.forward(state).items()}
    tool=geo.mount.compose(frames['left_link6']);cup=RigidTransform(**closing['world_from_cup'])
    maximum=0.;contacts=[];distances=[]
    for band,shape in geo.cup_bands:
        pose=cup.compose(RigidTransform((0,0,(band.z_min_m+band.z_max_m)/2)))
        obj=geo.fcl.CollisionObject(shape,geo.whole.transform(pose))
        for link,name,geometry,_,_ in geo.whole.parts:
            if link not in ('left_link6','left_link7','left_link8'):continue
            other=geo.fcl.CollisionObject(geometry,geo.whole.transform(geo.mount.compose(frames[link])))
            if link=='left_link6':
                hit=geo.fcl.collide(obj,other,geo.fcl.CollisionRequest(),geo.fcl.CollisionResult())
                distance=float(geo.fcl.distance(obj,other))
                distances.append(dict(part=name,band=band.shape_path,collision=bool(hit),distance_m=distance))
            else:
                _,witnesses=geo.cup_pair(obj,other);depth=max((d for _,d in witnesses),default=0.)
                maximum=max(maximum,depth)
                if depth>0:contacts.append(dict(part=name,band=band.shape_path,penetration_m=depth))
    if maximum+1e-5>=ALLOWANCE_M:raise ValueError('source terminal claw contact exceeds 3 mm')
    if not distances or any(row['collision'] or not math.isfinite(row['distance_m']) or row['distance_m']<=1e-5 for row in distances):
        raise ValueError('positive source wrist/cup separation required')
    world,_,_=world_config(planner,cup)
    return dict(passed=True,hardware_access=False,offline_model_preparation=True,
        grasp_q_model=[state[f'left_joint{i}'] for i in range(1,7)],
        urdf_sha256=data['urdf_sha256'],source_parts_sha256=_digest(data['parts']),cup_profile_sha256=planner.cup.profile_sha256,
        terminal_jaw_position_m=state['left_joint7'],linked_jaws=True,
        original_cup_from_tool=asdict(initial_cup.inverse().compose(tool)),tool_from_held_cup=asdict(tool.inverse().compose(cup)),
        original_cup_from_held_cup=asdict(initial_cup.inverse().compose(cup)),claw_cup_allowance_m=ALLOWANCE_M,
        physical_force_verified=False,samples=len(closing['samples']),cup_geometry_sha256=cup_geometry_digest(world),
        maximum_claw_penetration_m=maximum,
        cup_radius_from_origin_m=max(math.hypot(b.radius_m,max(abs(b.z_min_m),abs(b.z_max_m))) for b in planner.cup.bands),
        numerical_contact_reserve_m=1e-5,first_band_center_z_m=(planner.cup.bands[0].z_min_m+planner.cup.bands[0].z_max_m)/2,
        contact_geometry_stops_m=deepcopy(closing['contact_geometry_stops_m']),contacts=contacts,
        certificate_scope='source-mesh terminal contact plus runtime rigid pose/jaw displacement bound; not a force or full-path certificate',
        maximum_body_metal_penetration_m=max((c['penetration_m'] for c in contacts if c['band'] in ('/Cup/Collisions/MiddleBody','/Cup/Collisions/UpperBody')),default=0.),
        wrist_cup_source_checks=distances,minimum_wrist_clearance_m=min(row['distance_m'] for row in distances),wrist_source_separation_proven=True)


def enrich_contact_sweep(planner,proof,closing):
    """Retain every checked linked-jaw source sample for the existing bound."""
    from robot_sim.asset_cfg.objects.coffee_machine import RigidTransform
    from cup_contact import ALLOWANCE_M
    if proof.get('passed') is not True or closing.get('passed') is not True:raise ValueError('passed source contact required')
    geo=planner.geo;samples=[]
    for row in closing['samples']:
        if not row['free']:raise ValueError('source sweep has a failed sample')
        state={**closing['terminal_state'],**row['jaws']};frames=planner.fk.forward(state)
        cup=RigidTransform(**row['world_from_cup']);maximum=0.
        tool=geo.mount.compose(planner.fk.pose(frames['left_link6']))
        for band,shape in geo.cup_bands:
            pose=cup.compose(RigidTransform((0,0,(band.z_min_m+band.z_max_m)/2)))
            obj=geo.fcl.CollisionObject(shape,geo.whole.transform(pose))
            for link,name,geometry,_,_ in geo.whole.parts:
                if link not in ('left_link7','left_link8'):continue
                other=geo.fcl.CollisionObject(geometry,geo.whole.transform(geo.mount.compose(planner.fk.pose(frames[link]))))
                _,witnesses=geo.cup_pair(obj,other)
                maximum=max(maximum,max((depth for _,depth in witnesses),default=0.))
        if maximum+proof['numerical_contact_reserve_m']>=ALLOWANCE_M:raise ValueError('source sweep contact exceeds 3 mm')
        samples.append(dict(jaw_position_m=row['jaws']['left_joint7'],tool_from_cup=asdict(tool.inverse().compose(cup)),maximum_claw_penetration_m=maximum))
    return dict(proof,jaw_sweep_source_samples=samples,native_closing_digest=_digest(closing))


def build_contact_certificate(planner,data,state,cup,*,source_digest,candidate_id,closed_jaw_m):
    from grasp_contact_geometry import analyze_contact_pose
    began=time.perf_counter()
    actual=cup.inverse().compose(planner.geo.mount.compose(planner.fk.pose(planner.fk.forward(state)['left_link6'])))
    normal=cup.inverse().rotate(planner.table.world_from_shape.rotate((0.,0.,1.)))
    region=analyze_contact_pose(planner,data,actual,closed_jaw_m=closed_jaw_m,table_normal_cup=normal)
    if region.get('passed') is not True:raise ValueError('actual MotionGen pose lacks 2 mm contact-band reserve: '+str(region.get('reason')))
    closing=source_closing(planner,state,cup)
    native_path=dict(contact_geometry_stops_m=closing['contact_geometry_stops_m'],
        terminal_jaw_position_m=closing['terminal_state']['left_joint7'],
        centering_displacement_cup_m=cup.inverse().rotate(closing['centering_displacement_world_m']))
    region=analyze_contact_pose(planner,data,actual,closed_jaw_m=closed_jaw_m,
        table_normal_cup=normal,closing_geometry=native_path)
    if region.get('passed') is not True:raise ValueError('native linked-jaw path lacks 2 mm continuous contact reserve: '+str(region.get('error',region.get('reason'))))
    proof=enrich_contact_sweep(planner,source_contact_certificate(planner,data,closing,cup),closing)
    proof.update(schema_version='grasp_contact_certificate.v1',policy=deepcopy(POLICY),source_digest=source_digest,
                 candidate_id=candidate_id,grasp_contact_region=region,offline_model_preparation=False,
                 exported_geometry_sha256=_digest(data),
                 source_preparation_seconds=time.perf_counter()-began,
                 proof_scope='new selected actual pose; native linked-jaw source checks and continuous contact-region reserve')
    proof['certificate_digest']=_digest(proof)
    validate_contact_certificate(proof,source_digest=source_digest,urdf_sha256=data['urdf_sha256'],
        cup_profile_sha256=planner.cup.profile_sha256,source_parts_sha256=_digest(data['parts']),
        actual_cup_from_tool=asdict(actual))
    return closing,proof


def validate_contact_certificate(certificate,*,source_digest,urdf_sha256,expected_digest=None,
        cup_profile_sha256=None,source_parts_sha256=None,actual_cup_from_tool=None):
    value=certificate
    if (not isinstance(value,dict) or value.get('schema_version')!='grasp_contact_certificate.v1'
            or value.get('policy')!=POLICY or value.get('source_digest')!=source_digest
            or value.get('urdf_sha256')!=urdf_sha256 or value.get('passed') is not True
            or value.get('certificate_digest')!=_digest({k:v for k,v in value.items() if k!='certificate_digest'})
            or expected_digest is not None and value['certificate_digest']!=expected_digest
            or cup_profile_sha256 is not None and value.get('cup_profile_sha256')!=cup_profile_sha256
            or source_parts_sha256 is not None and value.get('source_parts_sha256')!=source_parts_sha256
            or actual_cup_from_tool is not None and value.get('original_cup_from_tool')!=actual_cup_from_tool):
        raise ValueError('selected grasp contact certificate identity or binding differs')
    region=value.get('grasp_contact_region') or {}
    if (region.get('schema_version')!='grasp_contact_geometry.v1' or region.get('passed') is not True
            or region.get('native_closing_checked') is not True
            or region.get('geometry_digest')!=_digest({k:v for k,v in region.items() if k!='geometry_digest'})
            or region.get('nominal_height_band_m')!=POLICY['nominal_band_m'] or region.get('reserve_m')!=.002
            or region.get('native_acceptance_tolerance_consumed_m')!=0.
            or region.get('numerical_bound_padding_m')!=1e-9
            or _digest(region.get('original_cup_from_tool'))!=_digest(value['original_cup_from_tool'])
            or any(region.get(key)!=value.get(key) for key in ('urdf_sha256','source_parts_sha256','cup_profile_sha256'))):
        raise ValueError('bound native-path continuous grasp contact region required')
    rows=region.get('regions');interval=region.get('contact_height_interval_m');breaks=region.get('jaw_breakpoints_m')
    if (not isinstance(rows,list) or not rows or any(not isinstance(row,dict) for row in rows)
            or {row.get('link') for row in rows}!={'left_link7','left_link8'}
            or not isinstance(breaks,list) or len(breaks)<2
            or any(type(v) not in (int,float) or not math.isfinite(v) for v in breaks)
            or any(a<=b for a,b in zip(breaks,breaks[1:]))
            or breaks[-1]!=region.get('commanded_closed_jaw_m')):
        raise ValueError('complete linked-jaw potential-contact regions required')
    lower,upper=POLICY['required_region_m']
    for row in rows:
        heights=row.get('height_interval_m');jaws=row.get('jaw_interval_m')
        if (not isinstance(heights,list) or len(heights)!=2 or not isinstance(jaws,list) or len(jaws)!=2
                or any(type(v) not in (int,float) or not math.isfinite(v) for v in heights+jaws)
                or not lower<=heights[0]<=heights[1]<=upper
                or not breaks[-1]<=jaws[0]<jaws[1]<=breaks[0]
                or not any(jaws==[b,a] for a,b in zip(breaks,breaks[1:]))):
            raise ValueError('potential contact region must preserve nominal 2 mm reserve through closing')
    if interval!=[min(row['height_interval_m'][0] for row in rows),max(row['height_interval_m'][1] for row in rows)]:
        raise ValueError('continuous contact height extrema differ from its regions')
    stops=region.get('contact_geometry_stops_m',{});lp=region.get('independent_contact_stops_lp_m',{})
    delta=region.get('centering_displacement_cup_m');axis=region.get('centering_axis_cup');normal=region.get('table_normal_cup')
    if any(not isinstance(vector,(list,tuple)) or len(vector)!=3 or any(type(v) not in (int,float) or not math.isfinite(v) for v in vector)
           for vector in (delta,axis,normal)):
        raise ValueError('complete finite native centering vectors required')
    if abs(sum(v*v for v in axis)-1)>1e-8 or abs(sum(v*v for v in normal)-1)>1e-8 or abs(sum(a*b for a,b in zip(axis,normal)))>1e-8:
        raise ValueError('native centering must use a unit table-plane axis')
    if (not isinstance(stops,dict) or not isinstance(lp,dict)
            or set(stops)!={'left_joint7','left_joint8'} or set(lp)!=set(stops)
            or stops!=value.get('contact_geometry_stops_m')
            or any(type(v) not in (int,float) or not math.isfinite(v) for v in [*stops.values(),*lp.values()])
            or any(not -1e-9<=lp[j]-stops[j]<=.00005+1e-9 for j in stops)
            or abs(sum(stops.values())/2-value.get('terminal_jaw_position_m',math.inf))>1e-9
            or abs(region.get('terminal_jaw_position_m',math.inf)-value['terminal_jaw_position_m'])>1e-9
            or any(abs(a-b)>1e-9 for a,b in zip(region.get('centering_displacement_cup_m',[]),
                                               value.get('original_cup_from_held_cup',{}).get('position',[]),strict=True))):
        raise ValueError('native closing and continuous contact path differ')
    shift=(stops['left_joint8']-stops['left_joint7'])/2
    held_position,held_rotation=_checked_pose(value.get('original_cup_from_held_cup'))
    if (max(abs(a-shift*b) for a,b in zip(delta,axis))>1e-9
            or max(abs(a-b) for a,b in zip(held_position,delta))>1e-9
            or abs(abs(held_rotation[0])-1)>1e-9 or max(map(abs,held_rotation[1:]))>1e-9):
        raise ValueError('native held-cup pose differs from the checked linked-jaw shift')
    _check_relation(value['original_cup_from_tool'],value['tool_from_held_cup'],delta)
    if (value.get('claw_cup_allowance_m')!=.003 or value.get('linked_jaws') is not True
            or value.get('wrist_source_separation_proven') is not True
            or not value.get('jaw_sweep_source_samples') or not value.get('native_closing_digest')):
        raise ValueError('new pose requires complete native linked-jaw, claw and wrist source proofs')
    maximum=value.get('maximum_claw_penetration_m',math.inf);epsilon=value.get('numerical_contact_reserve_m',-1.)
    if (any(type(v) not in (int,float) or not math.isfinite(v) for v in (maximum,epsilon,value.get('minimum_wrist_clearance_m')))
            or not 0<=maximum or not epsilon>=1e-5 or maximum+epsilon>=.003 or value['minimum_wrist_clearance_m']<=1e-5):
        raise ValueError('source claw/wrist geometry bounds are invalid')
    samples=value['jaw_sweep_source_samples'];terminal=value['terminal_jaw_position_m']
    if not 2<=len(samples)<=500:raise ValueError('complete source claw sweep required')
    previous=breaks[0]
    for sample in samples:
        gap=sample.get('jaw_position_m');depth=sample.get('maximum_claw_penetration_m')
        if (type(gap) not in (int,float) or not math.isfinite(gap) or not terminal-1e-9<=gap<=previous+1e-9
                or type(depth) not in (int,float) or not math.isfinite(depth) or not 0<=depth or depth+epsilon>=.003):
            raise ValueError('source claw sweep bounds are invalid')
        moved=math.copysign(min(abs(shift),max(0.,max(stops.values())-gap)),shift)
        _check_relation(value['original_cup_from_tool'],sample.get('tool_from_cup'),[moved*v for v in axis])
        previous=gap
    if abs(samples[0]['jaw_position_m']-breaks[0])>1e-9 or abs(samples[-1]['jaw_position_m']-terminal)>1e-9:
        raise ValueError('source claw sweep must cover open through rigid contact')
    return value
