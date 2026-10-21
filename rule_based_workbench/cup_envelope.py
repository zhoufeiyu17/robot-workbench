"""Frozen inward cup proxy, attached through the native cuRobo sphere API.

The offline proof covers the entire 6mm core and keeps all balls at least 3mm inside the
source cup. Attachment only transforms the frozen spheres; interaction is separate.
"""
import math


def cylinder_sagitta(mesh):
    radius = max(math.hypot(v[0], v[1]) for v in mesh.vertices)
    angles = sorted(set(round(math.atan2(v[1],v[0]),12) for v in mesh.vertices
                        if math.hypot(v[0],v[1]) > radius*.999999))
    if radius <= 0 or len(angles) < 8:
        raise ValueError('explicit cylindrical cup band required')
    gap = max(b-a for a,b in zip(angles,angles[1:]+[angles[0]+2*math.pi]))
    sagitta = radius*(1-math.cos(gap/2))+1e-8
    return sagitta


def prepare(world, *, path=None):
    """Load the frozen inward-sphere asset; no fitting or CPU mesh audit online."""
    import json
    from pathlib import Path
    from cup_contact import cup_geometry_digest
    prepared=json.loads((Path(path) if path is not None else Path(__file__).with_name('a1_cup_inset.json')).read_text())
    validate_prepared(prepared,cup_geometry_digest(world))
    return prepared


def validate_prepared(prepared, geometry_digest):
    from curobo_robot_model import digest
    if (prepared.get('schema_version')!='inward_cup_envelope.v2'
            or prepared.get('cup_geometry_sha256')!=geometry_digest
            or prepared.get('core_coverage_proven') is not True
            or prepared.get('proxy_inside_source') is not True
            or prepared.get('maximum_uncovered_core_depth_m')!=.006
            or prepared.get('minimum_surface_inset_m')!=.003
            or prepared.get('target_surface_inset_m')!=.003
            or prepared.get('maximum_additional_inset_m')!=.003
            or prepared.get('additional_pair_allowance_m')!=0.
            or prepared.get('interaction_geometry_modified') is not False
            or prepared.get('external_pair_scope')!='all'
            or prepared.get('sphere_count')!=264
            or prepared.get('proof',{}).get('spheres_sha256')!=digest(prepared.get('first_band_spheres'))):
        raise ValueError('prepared inward cup policy/coverage identity differs')


def transformed_spheres(prepared, geometry_digest, tool_from_first_band, capacity):
    validate_prepared(prepared,geometry_digest)
    rows = prepared['first_band_spheres']
    if not rows or len(rows) != prepared['sphere_count'] or len(rows) > capacity:
        raise ValueError('prepared cup coverage exceeds attachment capacity')
    result = []
    for row in rows:
        if len(row) != 4 or not all(math.isfinite(v) for v in row) or row[3] <= 0:
            raise ValueError('finite positive cup cover spheres required')
        result.append([*tool_from_first_band.transform_point(tuple(row[:3])), row[3]])
    return result + [[0., 0., 0., -10.] for _ in range(capacity-len(result))]


def attach(engine, q, prepared):
    from robot_sim.asset_cfg.objects.coffee_machine import RigidTransform
    from curobo_robot_model import digest
    contact = engine.cup_contact
    if not contact.active:
        raise ValueError('restore the current cup world before attachment')
    fk = engine.generator.compute_kinematics(engine._start(q)).ee_pose
    from motiongen_motion import normalized_fk_quaternion
    tool = RigidTransform(tuple(fk.position[0].tolist()), normalized_fk_quaternion(fk.quaternion[0].tolist()))
    first = RigidTransform(tuple(contact.first_cup_pose[:3]), tuple(contact.first_cup_pose[3:]))
    config = engine.generator.kinematics.kinematics_config
    capacity = len(config.get_link_spheres('attached_object'))
    spheres = transformed_spheres(prepared, contact.geometry_digest, tool.inverse().compose(first), capacity)
    engine.generator.attach_spheres_to_robot(sphere_tensor=engine.tensor_args.to_device(spheres),
                                             link_name='attached_object')
    return dict(method='native attach_spheres_to_robot / frozen inward proxy',
        active_spheres=prepared['sphere_count'], reserved_spheres=capacity,
        sphere_sha256=digest(spheres), core_coverage_proven=True, minimum_surface_inset_m=.003, maximum_uncovered_core_depth_m=.006, proxy_inside_source=True,
        environment_path_validated=False)


def nominal_clearance(points, bands):
    """Offline analytic signed clearance, positive inside the whole band union."""
    import numpy as np
    rz=np.column_stack((np.linalg.norm(points[:,:2],axis=1),points[:,2]))
    inside=np.zeros(len(points),bool);minimum=np.full(len(points),np.inf)
    segments=[]
    for i,band in enumerate(bands):
        radius,low,high=(band[key] for key in ('radius_m','z_min_m','z_max_m'))
        if (not all(math.isfinite(v) for v in (radius,low,high)) or radius<=0 or high<=low
                or (i and abs(low-bands[i-1]['z_max_m'])>1e-12)):
            raise ValueError('finite contiguous cylinder bands required')
        start=band.get('radius_start_m') or radius
        end=band.get('radius_end_m') or radius
        local_radius=start+(end-start)*np.clip((rz[:,1]-low)/(high-low),0.,1.)
        inside|=(rz[:,0]<=local_radius)&(rz[:,1]>=low)&(rz[:,1]<=high)
        segments.append(((start,low),(end,high)))
        previous=(bands[i-1].get('radius_end_m') or bands[i-1]['radius_m']) if i else 0.
        if abs(previous-start)>1e-12:segments.append(((previous,low),(start,low)))
        if i==len(bands)-1:segments.append(((0.,high),(end,high)))
    for a,b in segments:
        a,b=np.asarray(a),np.asarray(b);edge=b-a
        t=np.clip((rz-a)@edge/max(float(edge@edge),1e-30),0.,1.)
        minimum=np.minimum(minimum,np.linalg.norm(rz-a-t[:,None]*edge,axis=1))
    return np.where(inside,minimum,-minimum)


def verify_offline(prepared, *, maximum_cells=500000):
    """Bounded 3-D cell proof. Not called by planning or GPU query paths."""
    import numpy as np
    validate_prepared(prepared,prepared['cup_geometry_sha256'])
    bands=prepared['source_bands'];spheres=np.asarray(prepared['first_band_spheres'],dtype=float).copy()
    if spheres.shape!=(264,4) or not np.isfinite(spheres).all() or np.any(spheres[:,3]<=0):
        raise ValueError('264 finite positive spheres required')
    spheres[:,2]+=(bands[0]['z_min_m']+bands[0]['z_max_m'])/2
    if np.any(spheres[:,3]>nominal_clearance(spheres[:,:3],bands)-.003):
        raise ValueError('cup collision sphere violates 3mm inward offset')
    radius=max(b['radius_m'] for b in bands);low=bands[0]['z_min_m'];high=bands[-1]['z_max_m']
    pending=np.array([[0.,0.,(low+high)/2,radius,radius,(high-low)/2]])
    checked=0;unresolved=0;failures=[];unresolved_points=[]
    while len(pending):
        centers,half=pending[:,:3],pending[:,3:];bound=np.linalg.norm(half,axis=1)
        depth=nominal_clearance(centers,bands);margin=np.empty(len(centers))
        for begin in range(0,len(centers),2048):
            margin[begin:begin+2048]=(spheres[None,:,3]-np.linalg.norm(
                centers[begin:begin+2048,None]-spheres[None,:,:3],axis=2)).max(1)
        checked+=len(pending)
        keep=(depth+bound>.006-1e-8)&(margin<bound+1e-8)
        bad=keep&(depth>.006+1e-8)&(margin < -1e-8)
        failures.extend(centers[bad].tolist());keep&=~bad
        small=keep&(half.max(1)<.0000625);unresolved+=int(small.sum())
        unresolved_points.extend(centers[small].tolist());keep&=~small
        pending=pending[keep]
        if checked>maximum_cells or len(failures)>128:break
        if not len(pending):break
        axis=pending[:,3:].argmax(1);rows=np.arange(len(pending));pending[rows,axis+3]/=2
        plus=pending.copy();minus=pending.copy()
        plus[rows,axis]+=pending[rows,axis+3];minus[rows,axis]-=pending[rows,axis+3]
        pending=np.concatenate((plus,minus))
    return dict(passed=not failures and not unresolved and not len(pending),checked_cells=checked,
        minimum_surface_inset_m=.003, maximum_uncovered_core_depth_m=.006,failure_count=len(failures),unresolved_count=unresolved,
        pending_cells=len(pending),proxy_inside_source=True,failure_points_m=failures[:128],unresolved_points_m=unresolved_points[:128])
