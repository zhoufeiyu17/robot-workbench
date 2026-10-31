"""Bounded offline refit of the approved 264 inward spheres to the new solid."""
import argparse
import json
import math
from pathlib import Path
import time
import numpy as np

from cup_envelope import nominal_clearance,verify_offline
from curobo_robot_model import refine_balls,digest
from cup_contact import cup_geometry_digest
from playground_geometry import load
from motiongen_plan import world_config


def main(args):
    args.output.mkdir(parents=True,exist_ok=False);started=time.perf_counter()
    planner,obs,data=load(args.input,json.loads(args.source.read_text()),args.native_code,args.assets)
    world,_,_=world_config(planner,obs.sample.world_from_cup)
    bands=data['cup']['bands'];parent=json.loads(args.parent.read_text())
    sagitta=max(b['radius_m'] for b in bands)*(1-math.cos(math.pi/32))+1e-8
    def radius(points):return nominal_clearance(points,bands)-.003-sagitta-1e-7
    old=np.asarray(parent['first_band_spheres'],float).copy()
    old[:,2]+=(parent['source_bands'][0]['z_min_m']+parent['source_bands'][0]['z_max_m'])/2
    old[:,3]=radius(old[:,:3]);old=old[old[:,3]>0.]
    order=np.argsort(-old[:,3],kind='stable');seed=old[order[:24]]
    axes=[np.arange(-.045,.0451,.0015),np.arange(-.045,.0451,.0015),np.arange(0,.1301,.0015)]
    grid=np.stack(np.meshgrid(*axes,indexing='ij'),axis=-1).reshape(-1,3)
    # Sample a buffer outside the6mm core as well; otherwise the first grid
    # plane misses the thin cap-adjacent core slab. Proof remains unchanged.
    points=grid[nominal_clearance(grid,bands)>.0054]
    if args.counterexamples:
        prior=json.loads(args.counterexamples.read_text())
        extra=prior['proof']['continuous_core_verification']['failure_points_m']
        points=np.unique(np.vstack((points,np.asarray(extra))),axis=0)
    construction_gaps=np.full(len(points),np.inf)
    for row in seed:
        np.minimum(construction_gaps,np.linalg.norm(points-row[:3],axis=1)-row[3],out=construction_gaps)
    def candidate(index):
        point=points[index];eps=1e-6
        gradient=(radius(point+np.eye(3)*eps)-radius(point-np.eye(3)*eps))/(2*eps)
        gradient/=max(np.linalg.norm(gradient),1e-30)
        radial=np.r_[-point[:2]/max(np.linalg.norm(point[:2]),1e-30),0.]
        directions=np.array([gradient,radial,[0,0,1.],[0,0,-1.],radial+[0,0,1.],radial+[0,0,-1.]])
        directions/=np.maximum(np.linalg.norm(directions,axis=1,keepdims=True),1e-30)
        positions=(point+np.r_[0.,np.geomspace(.0001,.1,32)][:,None,None]*directions[None]).reshape(-1,3)
        radii=radius(positions);valid=(radii>0)&(radii-np.linalg.norm(positions-point,axis=1)>=.0004)
        ids=np.flatnonzero(valid)
        if not len(ids):raise ValueError('no contained sphere covers selected core point')
        pending=points[construction_gaps>-.0004]
        probes=pending[::max(1,len(pending)//2048)]
        covered=(np.linalg.norm(probes[:,None]-positions[None,ids],axis=2)<=radii[None,ids]-.0004).sum(0)
        chosen=ids[max(range(len(ids)),key=lambda i:(int(covered[i]),float(radii[ids[i]])))]
        np.minimum(construction_gaps,np.linalg.norm(points-positions[chosen],axis=1)-radii[chosen],out=construction_gaps)
        return positions[chosen],radii[chosen]
    c,r,gaps=refine_balls(np,points,np.full(len(points),-.0004),seed[:,:3],seed[:,3],candidate,264)
    fitted=np.column_stack((c,r));existing={tuple(row) for row in fitted.tolist()}
    for row in old[order]:
        if len(fitted)==264:break
        if tuple(row) not in existing:fitted=np.vstack((fitted,row));existing.add(tuple(row))
    if len(fitted)<264:
        # Extra contained balls do not weaken coverage; keep the frozen capacity.
        while len(fitted)<264:fitted=np.vstack((fitted,fitted[0]))
    fitted[:,2]-=(bands[0]['z_min_m']+bands[0]['z_max_m'])/2
    result={**parent,'cup_geometry_sha256':cup_geometry_digest(world),'cup_profile_sha256':data['cup']['profile_sha256'],'source_bands':bands,
            'first_band_spheres':fitted.tolist(),'source_polygon_sagitta_bound_m':sagitta}
    result['proof']={'spheres_sha256':digest(result['first_band_spheres']),
                     'construction_grid_m':.0015,'construction_points':len(points)}
    proof=verify_offline(result)
    repairs=[]
    # Keep accepted balls and spend the remaining capacity on proof counterexamples,
    # rather than refitting the grid and reintroducing holes elsewhere.
    for iteration in range(3):
        if proof['passed']:break
        points=np.asarray(proof['failure_points_m']+proof['unresolved_points_m'])
        if not len(points):break
        construction_gaps=np.full(len(points),np.inf)
        for center,value in zip(c,r):
            np.minimum(construction_gaps,np.linalg.norm(points-center,axis=1)-value,out=construction_gaps)
        c,r,gaps=refine_balls(np,points,np.full(len(points),-.0004),c,r,candidate,264)
        fitted=np.column_stack((c,r))
        while len(fitted)<264:fitted=np.vstack((fitted,fitted[0]))
        fitted[:,2]-=(bands[0]['z_min_m']+bands[0]['z_max_m'])/2
        result['first_band_spheres']=fitted.tolist()
        result['proof']['spheres_sha256']=digest(result['first_band_spheres'])
        proof=verify_offline(result)
        repairs.append(dict(iteration=iteration,fitted_spheres=len(c),passed=proof['passed']))
    result['proof']['continuous_core_verification']=proof
    result['proof']['passed']=proof['passed']
    result['proof']['counterexample_repairs']=repairs
    if not proof['passed']:
        result['core_coverage_proven']=False
        (args.output/'candidate.json').write_text(json.dumps(result))
        raise ValueError('offline6mm core coverage proof failed: '+str(proof))
    (args.output/'cup-envelope.json').write_text(json.dumps(result))
    print(json.dumps(dict(passed=True,spheres=264,seconds=time.perf_counter()-started,proof=proof)))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('input','source','native-code','assets','parent','output'):p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--counterexamples',type=Path)
    main(p.parse_args())
