"""Simulation-only cross-wrist plan; nominal optical mounts are not real calibration."""
import json
import copy
from collections import Counter
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from arx_env.joint_head_workbench_geometry import Geometry
from arx_env.joint_head_isaac_fit import rigid
from arx_env.joint_head_wrist_calibration import project_marker, rectified_camera
def _sibling(name):
    """Load a module from this directory.

    The workbench loads this file by path (spec_from_file_location), so package
    imports such as examples.scripts.debug.* are not available in that process.
    """
    from importlib import util as _util
    from pathlib import Path as _Path
    spec = _util.spec_from_file_location(name, _Path(__file__).with_name(name + '.py'))
    module = _util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


MastClearance = _sibling('joint_head_wrist_clearance').MastClearance

UNCERTAIN = 'intersection solver uncertain'


def path_verdict(check):
    """Split a `path_check` failure into what must refuse and what must only be reported.

    The collision LP returns an undecided pair when it cannot separate two hulls - the
    shoulder bracket against the chassis mesh it is bolted to, for instance. That is not
    a collision, and refusing on it makes a frozen, already accepted plan unrunnable:
    the wrist replay would reject its own entry leg from wherever the arms happen to be
    parked, and every route away from that pose measures undecided too, so there is no
    pose left to try. Definite overlaps and joint-limit violations still refuse.
    """
    if check.get('ok'):
        return {'blocking': False, 'undecided': False, 'definite': [], 'uncertain': []}
    pairs = list(check.get('collisions') or [])
    definite = [pair for pair in pairs if UNCERTAIN not in pair]
    uncertain = [pair for pair in pairs if UNCERTAIN in pair]
    return {'blocking': bool(definite) or not pairs,
            'undecided': bool(uncertain) and not definite,
            'definite': definite, 'uncertain': uncertain}


def marker_occluder(geometry, frames, optical, marker):
    """Check nine paper sightlines against full triangles, including the mast."""
    # ponytail: finite ray sampling; rendered detection remains the coverage gate.
    extent=geometry.marker_size*2/3  # Include the printed white border.
    points=np.array([[x,y,0.] for x in (-extent,0.,extent) for y in (-extent,0.,extent)])
    points=points@marker[:3,:3].T+marker[:3,3]
    origin=optical[:3,3]
    meshes=[('environment/'+name,v) for name,v,_ in getattr(geometry,'static_meshes',[])]
    for side,parts in getattr(geometry,'side_meshes',{}).items():
        for name,vertices,_ in parts:
            T=frames[side][name]
            meshes.append((side+'/'+name,vertices@T[:3,:3].T+T[:3,3]))
    for name,vertices in meshes:
        triangles=vertices.reshape(-1,3,3)
        lo=np.minimum(origin,points.min(0))-.0001;hi=np.maximum(origin,points.max(0))+.0001
        triangles=triangles[np.all(triangles.max(1)>=lo,axis=1)&np.all(triangles.min(1)<=hi,axis=1)]
        if not len(triangles):continue
        a,b,c=triangles[:,0],triangles[:,1],triangles[:,2];e1,e2=b-a,c-a
        offset=origin-a;q=np.cross(offset,e1)
        for point in points:
            direction=point-origin;length=np.linalg.norm(direction)
            h=np.cross(direction,e2);det=np.einsum('ij,ij->i',e1,h)
            inv=np.divide(1.,det,out=np.zeros_like(det),where=abs(det)>1e-12)
            u=inv*np.einsum('ij,ij->i',offset,h);v=inv*(q@direction)
            t=inv*np.einsum('ij,ij->i',e2,q)
            if np.any((abs(det)>1e-12)&(u>=0)&(v>=0)&(u+v<=1)&(t*length>.0001)&((1-t)*length>.0001)):
                return name
    return None


def wrist_visibility(geometry, q, observer, target, optical_mount, camera):
    frames=geometry.frames(q);observer_frame=frames[observer]['link6']
    marker=frames[target]['link6']@geometry.mounts[target];optical=observer_frame@optical_mount
    try:uv,depth=project_marker(optical_mount,observer_frame,marker,camera,geometry.marker_size)
    except ValueError:return {'visible':False,'reason':'marker behind camera'}
    margin=float(geometry.config.get('margin_px',18))
    facing=float(marker[:3,2]@(optical[:3,3]-marker[:3,3]))<0
    framed=bool(np.all(depth>.005) and np.all(uv>=margin) and np.all(uv<np.array([camera['width'],camera['height']])-margin))
    occluder=marker_occluder(geometry,frames,optical,marker) if facing and framed else None
    return {'visible':facing and framed and occluder is None,'front_facing':facing,'framed':framed,'occluder':occluder,
            'reason':'occluded by '+occluder if occluder else 'outside view or back-facing' if not (facing and framed) else None,'corners_uv':uv.tolist(),
            'marker_position_camera_m':(optical[:3,:3].T@(marker[:3,3]-optical[:3,3])).tolist(),
            'marker_normal_camera':(optical[:3,:3].T@marker[:3,2]).tolist()}


def plan_wrist(geometry, nominal_mounts, *, poses_per_side=16, seed=20,
               start_q=None, joint_offsets=None, cameras=None, marker_mounts=None, progress=None, motion_seed=None,
               min_clearance_m=0.0):
    """Search new poses with collision checks; replay frozen seeds without them."""
    if type(poses_per_side) is not int or poses_per_side<4:raise ValueError('at least four poses per side required')
    if marker_mounts is not None and hasattr(geometry,'with_marker_mounts'):
        geometry=geometry.with_marker_mounts({side:marker_mounts[str(marker)] for side,marker in [('left',24),('right',30)]})
    geometry=geometry.with_gaps(80.,80.) if hasattr(geometry,'with_gaps') else geometry
    g=copy.copy(geometry);rng=np.random.default_rng(seed);bounds=g.limits.copy()
    margin=float(min_clearance_m or 0.)
    mast=MastClearance(geometry) if margin and motion_seed is None else None
    bounds[[0,6],0]=np.maximum(bounds[[0,6],0],-np.deg2rad(100));bounds[[0,6],1]=np.minimum(bounds[[0,6],1],np.deg2rad(100))
    g.limits=bounds
    offsets=np.asarray(np.zeros(12) if joint_offsets is None else joint_offsets,float)
    home=np.asarray(np.zeros(12) if start_q is None else start_q,float).copy()
    if offsets.shape!=(12,) or home.shape!=(12,) or not np.isfinite(offsets).all() or not np.isfinite(home).all():
        raise ValueError('twelve finite start joints and offsets required')
    g.frames=lambda q:geometry.frames(np.asarray(q)+offsets)
    g.mounts={side:rigid(marker_mounts[str(marker)] if marker_mounts is not None else geometry.mounts[side]) for side,marker in [('left',24),('right',30)]}
    cameras={side:rectified_camera(cameras[side] if cameras is not None else {**geometry.k,'rectified':True}) for side in ('left','right')}
    rows=[];rejected=[];rejection_counts={}
    if motion_seed is not None:
        return plan_from_seed(g,nominal_mounts,cameras,home,offsets,motion_seed,progress)
    if not g.path_check(home,home)['ok']:raise ValueError('actual start violates collision/table bounds')
    for observer,target in [('left','right'),('right','left')]:
        rng=np.random.default_rng(seed if observer == 'left' else seed + 1)
        if observer == 'right':
            # Reproduce the previously validated shared-seed right-arm stream:
            # seed+1 spent 49 left attempts before entering the right loop.
            for _ in range(49):
                rng.uniform(0.,1.,3); rng.uniform(0.,1.,2); rng.uniform(0.,1.,12)
        X=rigid(nominal_mounts[observer]);accepted=[];current=home.copy();route=[home.copy()];counts=Counter();to_home=None
        def publish(attempt):
            if progress is not None:
                progress({'observer':observer,'attempt':attempt+1,'accepted':len(accepted),
                          'target':poses_per_side,'target_arm':target,'rejection_counts':dict(counts)})
        def reject(attempt,reason,stage):
            rejected.append({'observer':observer,'attempt':attempt,'stage':stage,'reason':reason})
            collisions=reason.get('collisions',[])
            if collisions:
                counts.update('collision: '+' / '.join(map(str,pair)) for pair in collisions)
            else:counts[reason.get('reason',stage)]+=1
            publish(attempt)
        for attempt in range(300):
            publish(attempt)
            desired=np.array([rng.uniform(-.014,.014),rng.uniform(-.012,.012),rng.uniform(.13,.22)])
            normal=np.r_[rng.uniform(-.25,.25,2),1.];normal/=np.linalg.norm(normal)
            warm=bool(accepted) and attempt%8!=0
            start=accepted[-1]+rng.normal(0,.15,12) if warm else rng.uniform(np.maximum(bounds[:,0],-1.2),np.minimum(bounds[:,1],1.2))
            start=np.clip(start,bounds[:,0]+1e-6,bounds[:,1]-1e-6)
            q_ref=accepted[-1].copy() if warm else start.copy()
            def residual(q):
                f=g.frames(q);cam=f[observer]['link6']@X;tag=f[target]['link6']@g.mounts[target]
                p=cam[:3,:3].T@(tag[:3,3]-cam[:3,3]);n=cam[:3,:3].T@tag[:3,2]
                return np.r_[p-desired,(n-normal)*.1,(q-q_ref)*.002]
            fit=least_squares(residual,start,bounds=(bounds[:,0],bounds[:,1]),max_nfev=90)
            q=fit.x
            if np.linalg.norm(residual(q)[:6])>.035:
                counts['IK residual']+=1;publish(attempt);continue
            visibility=wrist_visibility(g,q,observer,target,X,cameras[observer])
            if not visibility['visible']:
                reject(attempt,visibility,'visibility');continue
            if mast is not None:
                gap=mast.clearance(g,q)
                if gap<margin:
                    counts['clearance: worse than %d mm'%round(margin*1000)]+=1
                    rejected.append({'observer':observer,'attempt':attempt,'stage':'clearance',
                                     'reason':{'mast_clearance_m':gap,'margin_m':margin}})
                    publish(attempt);continue
            endpoint=g.path_check(q,q)
            if not endpoint['ok']:
                reject(attempt,endpoint,'endpoint');continue
            check=g.path_check(current,q)
            if not check['ok']:
                reject(attempt,check,'direct path')
                if to_home is None:to_home=g.path_check(current,home)
                if not to_home['ok']:
                    reject(attempt,to_home,'via start: return');continue
                from_home=g.path_check(home,q)
                if not from_home['ok']:
                    reject(attempt,from_home,'via start: outbound');continue
                rows.append({'id':observer+'-via-start-'+str(attempt),'q':home.tolist(),'capture':False,'jaw_gap_mm':[80.,80.],'path_check':to_home})
                route.append(home.copy());check=from_home
            index=len(accepted);accepted.append(q);current=q;route.append(q.copy());to_home=None
            holdout = index in (2,5) if poses_per_side==6 else index%4==3
            rows.append({'id':observer+'-'+str(index),'observer':observer,'target':target,'q':q.tolist(),'jaw_gap_mm':[80.,80.],'split':'holdout' if holdout else 'fit','path_check':check,'visibility':visibility})
            publish(attempt)
            if len(accepted)>=poses_per_side:break
        rejection_counts[observer]=dict(counts)
        if len(accepted)<poses_per_side:raise ValueError(f'{observer}: only {len(accepted)}/{poses_per_side} collision-free poses after 300 attempts; rejection_counts='+json.dumps(dict(counts),sort_keys=True))
        check=g.path_check(current,home)
        if check['ok']:
            rows.append({'id':observer+'-return','q':home.tolist(),'capture':False,'jaw_gap_mm':[80.,80.],'path_check':check})
        else:
            for index,target_q in enumerate(reversed(route[:-1])):
                check=g.path_check(current,target_q)
                if not check['ok']:raise ValueError(f'{observer}: reverse return path rejected at segment {index}')
                rows.append({'id':observer+'-return-'+str(index),'q':target_q.tolist(),'capture':False,'jaw_gap_mm':[80.,80.],'path_check':check})
                current=target_q
    planned=[{key:row[key] for key in ('id','observer','target','split')} for row in rows if row.get('capture') is not False]
    return {'mode':'simulation','jaw_gap_override_mm':[80.,80.],'waypoints':rows,'planned_frames':planned,'start_q':home.tolist(),'joint_offsets_rad':offsets.tolist(),'nominal_mounts':nominal_mounts,'cameras':cameras,'table_z_m':g.config.get('table_z_m'),'rejected':rejected,'rejection_counts':rejection_counts,'collision_scope':getattr(g,'collision_method','full arm convex hulls + table; middle chassis absent'),'hardware_execution_approved':False}

def plan_from_seed(g, mounts, cameras, start, offsets, seed, progress=None):
    """Replay frozen poses without collision or mast-clearance screening."""
    g=g.with_gaps(80.,80.) if hasattr(g,'with_gaps') else g
    order=[f'{s}/joint{i}' for s in ('left','right') for i in range(1,7)]
    if seed.get('schema')!='x5-wrist-motion-seed-v1' or seed.get('joint_order')!=order:
        raise ValueError('wrist seed schema/joint order does not match X5')
    rows=seed.get('waypoints')
    if not isinstance(rows,list) or not rows:raise ValueError('wrist seed has no waypoints')
    anchor_offsets=seed.get('anchored_joint_offsets_rad')
    drift=np.zeros(12)
    if anchor_offsets is not None:
        # A seed stores *nominal* joint values, and the value to command is
        # `nominal - offsets`, so the same seed commands different numbers under a
        # different head. Re-anchoring keeps the model-space pose identical and moves
        # only the commanded number - which is what a new head's offsets require, and
        # what stops a seed from being refused on j2/j3 limits that it never actually
        # left. A seed without the record is taken as anchored on the current offsets.
        values=np.asarray(anchor_offsets,float)
        if values.shape!=(12,) or not np.isfinite(values).all():
            raise ValueError('wrist seed anchor must hold twelve finite joint offsets')
        drift=np.asarray(offsets,float)-values
    def reported(value):
        q=np.asarray(value,float)
        if q.shape!=(12,) or not np.isfinite(q).all():raise ValueError('wrist seed requires twelve finite nominal joints')
        return q-offsets+drift
    anchor=reported(seed.get('start_q_nominal_rad'))
    route=[];ids=set()
    for row in rows:
        identity=row.get('id')
        if not isinstance(identity,str) or not identity or identity in ids:raise ValueError('wrist seed requires unique waypoint IDs')
        ids.add(identity)
        item={'id':identity,'q':reported(row.get('q_nominal_rad')).tolist(),'jaw_gap_mm':[80.,80.]}
        if row.get('jaw_gap_mm') not in ([83.,83.],[80.,80.]):raise ValueError('wrist seed jaw metadata must be 83 mm legacy or 80 mm')
        if row.get('capture') is False:item['capture']=False
        else:
            observer=row.get('observer');target=row.get('target');split=row.get('split')
            if (observer,target) not in (('left','right'),('right','left')) or split not in ('fit','holdout'):
                raise ValueError('invalid wrist seed observer/target/split')
            item.update(observer=observer,target=target,split=split)
        route.append(item)
    planned=[{key:row[key] for key in ('id','observer','target','split')} for row in route if row.get('capture') is not False]
    for side in ('left','right'):
        counts=Counter(row['split'] for row in planned if row['observer']==side)
        if counts['fit']<3 or counts['holdout']<1:raise ValueError('wrist seed requires fit and independent holdout poses per side')
    if ids.intersection(('seed-entry','seed-return')):raise ValueError('reserved wrist seed waypoint ID')
    if np.max(abs(start-anchor))>1e-6:
        route.insert(0,{'id':'seed-entry','q':anchor.tolist(),'capture':False,'jaw_gap_mm':[80.,80.]})
    if np.max(abs(np.asarray(route[-1]['q'])-start))>1e-6:
        route.append({'id':'seed-return','q':start.tolist(),'capture':False,'jaw_gap_mm':[80.,80.]})
    accepted=Counter()
    for row in route:
        q=np.asarray(row['q'])
        if not (np.all(q>=g.limits[:,0]) and np.all(q<=g.limits[:,1])):
            raise ValueError('wrist seed '+row['id']+' exceeds joint limits')
        if row.get('capture') is not False:
            side=row['observer']
            visibility=wrist_visibility(g,row['q'],side,row['target'],rigid(mounts[side]),cameras[side])
            if not visibility['visible'] and not g.config.get('simulation_only'):
                raise ValueError('wrist seed '+row['id']+' not visible on current machine: '+str(visibility.get('reason')))
            row['visibility']=visibility;accepted[side]+=1
            if progress:progress({'observer':side,'accepted':accepted[side],'target':sum(r['observer']==side for r in planned)})
    return {'mode':'simulation','jaw_gap_override_mm':[80.,80.],
            'seed_jaw_gap_mm':[list(v) for v in sorted({tuple(row['jaw_gap_mm']) for row in rows})],
            'motion_seed':seed['id'],'min_clearance_m':None,'waypoints':route,'planned_frames':planned,
            'start_q':start.tolist(),'joint_offsets_rad':offsets.tolist(),'anchored_joint_offsets_rad':np.asarray(offsets,float).tolist(),
            'reanchored_from':seed.get('anchored_joint_offsets_rad'),'nominal_mounts':mounts,'cameras':cameras,
            'table_z_m':g.config.get('table_z_m'),'rejected':[],'rejection_counts':{},'plan_complete':True,
            'path_undecided':[], 'collision_checked':False,
            'collision_scope':'not checked: frozen calibration seed','hardware_execution_approved':False}


if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--config',required=True);p.add_argument('--mounts',required=True);p.add_argument('--output',required=True);a=p.parse_args()
    config=json.loads(Path(a.config).read_text());g=Geometry(config)
    result=plan_wrist(g,json.loads(Path(a.mounts).read_text()));Path(a.output).write_text(json.dumps(result,indent=2))
