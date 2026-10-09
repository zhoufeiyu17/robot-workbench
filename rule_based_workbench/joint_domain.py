"""Offline interval proof for a sphere pair over a bounded joint domain.

Shared ancestors cancel in pair distance. For the remaining joints, a bound on
point displacement converts midpoint separation into a whole-interval result.
An inconclusive result keeps the collision pair enabled; it never exempts it.
"""
import math


def check_domain(positions, bounds):
    for name, (low, high) in bounds.items():
        value = positions.get(name)
        if value is None or not math.isfinite(value) or not low <= value <= high:
            raise ValueError('outside precomputed joint domain: '+name)


def certify_pair(fk, link_a, spheres_a, link_b, spheres_b, positions, bounds, *, max_cells=128, distance_at=None, retain_cells=False, accept_at=None):
    import numpy as np
    if set(bounds) != set(fk.movable) or max_cells < 1:
        raise ValueError('complete joint bounds and positive offline budget required')
    if any(not math.isfinite(v) for pair in bounds.values() for v in pair) or any(a>b for a,b in bounds.values()):
        raise ValueError('finite ordered joint bounds required')
    check_domain(positions, bounds)
    a,b=np.asarray(spheres_a,dtype=float),np.asarray(spheres_b,dtype=float)
    if any(x.ndim!=2 or x.shape[1]!=4 or not len(x) or not np.isfinite(x).all() or np.any(x[:,3]<=0) for x in (a,b)):
        raise ValueError('positive conservative spheres required')
    chains=[fk._chains[link] for link in (link_a,link_b)]
    ancestors=[{j[0] for j in chain if j[1]!='fixed'} for chain in chains]
    relative=ancestors[0]^ancestors[1]
    leverage={}
    for chain,spheres in zip(chains,(a,b)):
        for index,joint in enumerate(chain):
            name,kind,_,_,_,axis=joint
            if name not in relative:continue
            axis_norm=float(np.linalg.norm(axis))
            if abs(axis_norm-1)>1e-8:raise ValueError('unit URDF joint axes required')
            reach=float(np.linalg.norm(spheres[:,:3],axis=1).max())
            if distance_at is not None:
                reach+=float(spheres[:,3].max())  # bound any source solid point, not only sphere centers
            for child in chain[index+1:]:
                reach+=float(np.linalg.norm(child[4][:3,3]))
                if child[1]=='prismatic':reach+=max(abs(v) for v in bounds[child[0]])
            leverage[name]=(kind,reach)
    from collections import deque
    pending=deque([{n:bounds[n] for n in relative}]); checked=0; minimum=math.inf
    certified_cells=[];rejected_cells=0
    while pending and checked<max_cells:
        cell=pending.popleft() if retain_cells else pending.pop();checked+=1
        state=dict(positions,**{n:(lo+hi)/2 for n,(lo,hi) in cell.items()})
        frames=fk.forward(state)
        if distance_at is None:
            aa=a[:,:3]@frames[link_a][:3,:3].T+frames[link_a][:3,3]
            bb=b[:,:3]@frames[link_b][:3,:3].T+frames[link_b][:3,3]
            gap=float((np.linalg.norm(aa[:,None]-bb[None],axis=2)-a[:,None,3]-b[None,:,3]).min())
        else:
            gap=float(distance_at(frames))
            if not math.isfinite(gap) or gap < 0:raise ValueError('nonnegative finite offline geometry distance required')
        motion={n:((hi-lo)/2 if leverage[n][0]=='prismatic' else
                    2*leverage[n][1]*math.sin(min(math.pi,(hi-lo)/2)/2)) for n,(lo,hi) in cell.items()}
        lower=gap-sum(motion.values())
        if lower>1e-7:
            if accept_at is not None and not accept_at(frames):
                rejected_cells+=1
                if retain_cells:continue
                return dict(certified=False,reason='invalid_interval_anchor',cells=checked)
            minimum=min(minimum,lower)
            if retain_cells:certified_cells.append(dict(joint_domain=cell,minimum_separation_lower_bound_m=lower))
            continue
        if gap<=1e-7 or not motion:
            if retain_cells:rejected_cells+=1;continue
            return dict(certified=False,reason='geometry_overlap_or_touch' if distance_at else 'proxy_overlap_or_touch',cells=checked,midpoint=state)
        axis=max(motion,key=motion.get);lo,hi=cell[axis];mid=(lo+hi)/2
        if hi-lo<1e-8:
            if retain_cells:rejected_cells+=1;continue
            return dict(certified=False,reason='unresolved_small_interval',cells=checked)
        pending.extend((dict(cell,**{axis:(lo,mid)}),dict(cell,**{axis:(mid,hi)})))
    passed=not pending and not rejected_cells and math.isfinite(minimum)
    return dict(certified=passed,reason='continuous_interval_separation' if passed else 'offline_budget_exhausted',
                cells=checked,joint_domain=bounds,relative_joints=sorted(relative),
                minimum_separation_lower_bound_m=minimum if passed else None,
                **(dict(certified_cells=certified_cells,rejected_cells=rejected_cells,pending_cells=len(pending)) if retain_cells else {}))


if __name__=='__main__':
    import numpy as np
    class FK:
        movable={'hinge'}
        origin=np.eye(4)
        _chains={'base':(), 'arm':(('hinge','revolute','base','arm',origin,np.array([0.,0.,1.])),)}
        def forward(self,q):
            angle=q['hinge'];c,s=math.cos(angle),math.sin(angle);m=np.eye(4);m[:2,:2]=[[c,-s],[s,c]]
            return {'base':np.eye(4),'arm':m}
    fk=FK();state={'hinge':0.};domain={'hinge':(-math.pi,math.pi)}
    # Full revolution: an arm sphere 1m from hinge cannot reach a base sphere 3m away.
    assert certify_pair(fk,'base',[[3,0,0,.1]],'arm',[[1,0,0,.1]],state,domain)['certified']
    # Clear interval endpoints cannot prove safety: the mid-angle contains a collision.
    assert not certify_pair(fk,'base',[[1,0,0,.1]],'arm',[[1,0,0,.1]],state,domain)['certified']
    try:check_domain({'hinge':4.},domain)
    except ValueError:pass
    else:raise AssertionError('out-of-domain motion accepted')
    print('continuous joint-domain separation and runtime bounds: PASS')
