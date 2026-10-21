#!/usr/bin/env python3
"""Verify and freeze offline D sphere proposals against the original nominal C CAD.

The prepared JSON contains link-local sphere arrays from the bounded offline
interior fitter. This final build never trusts a proposal's radius/inside claim:
it rechecks every center and radius against the original C visual component
union. No camera calibration, robot state, GPU, or controller is consumed.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'calibration/infra')]
from arx_env.joint_head_workbench_geometry import Geometry, point_in_mesh
from robot_workbench.manual_self_collision import ManualSelfCollision, POLICY, components, source_fingerprint, proxy_digest


def verify_containment(geometry, data):
    import trimesh  # Offline-only dependency; the workbench loads the frozen JSON.
    centers=np.asarray(data['centers'],float);radii=np.asarray(data['radii'],float)
    membership=np.asarray(data['sphere_parts'],dtype=np.intp)
    source={}
    for side,rows in geometry.full_side_meshes.items():
        for name,vertices,faces in rows:source[side,name]=np.asarray(vertices)[faces]
    for name,vertices,faces in geometry.static_meshes:source[None,name]=np.asarray(vertices)[faces]
    groups={}
    for i,part in enumerate(data['parts']):groups.setdefault((part['side'],part['name']),[]).extend(np.flatnonzero(membership==i).tolist())
    reports=[]
    for key,ids in groups.items():
        if key not in source:raise ValueError('sphere group has no original nominal robot mesh: '+str(key))
        triangles=source[key];vertices,inverse=np.unique(triangles.reshape(-1,3),axis=0,return_inverse=True)
        mesh=trimesh.Trimesh(vertices=vertices,faces=inverse.reshape(-1,3),process=False)
        points=centers[ids];inside=np.zeros(len(ids),bool)
        for part,_ in components(triangles.reshape(-1,3),np.arange(triangles.size//3).reshape(-1,3)):
            part=part.reshape(-1,3,3)
            selected=np.flatnonzero(~inside & (points>=part.min((0,1))).all(1) & (points<=part.max((0,1))).all(1))
            inside[selected]=[point_in_mesh(points[i],part) for i in selected]
        _,distance,_=trimesh.proximity.ProximityQuery(mesh).on_surface(points)
        margin=distance-radii[ids]
        if not inside.all() or not np.isfinite(margin).all() or np.any(margin < POLICY['surface_inset_m']-1e-11):
            raise ValueError('sphere center or radius leaves original visual material: '+str(key))
        reports.append(dict(side=key[0],link=key[1],sphere_count=len(ids),all_centers_inside=True,
                            minimum_surface_margin_m=float(margin.min())))
    return dict(passed=True,checked_spheres=len(radii),groups=reports,
        minimum_surface_margin_m=min(row['minimum_surface_margin_m'] for row in reports),
        method='original triangle component union, fixed-ray parity and exact closest-triangle distance',
        scope='practical CAD containment; nonmanifold source STL is retained; sparse surface coverage is not certified')


def build(config, prepared, collision_parts, proposal_cache, output):
    if output.exists():raise FileExistsError('refusing to overwrite an existing frozen proxy')
    import trimesh
    began=time.perf_counter();g=Geometry(json.loads(config.read_text()))
    source=json.loads(collision_parts.read_text())
    expected={name for aliases in g.aliases.values() for name in aliases if name.endswith(tuple(str(i) for i in range(1,9)))}
    actual={part['link'] for part in source['parts']}
    if not expected <= actual or not actual <= (expected|{'base_link'}):
        raise ValueError('collection collision parts omit a nominal arm link or include an unrelated body')
    for part in source['parts']:
        if not trimesh.Trimesh(vertices=part['vertices'],faces=part['faces'],process=False).is_watertight:
            raise ValueError('collection proposal source has an open part: '+part['name'])
    data=json.loads(prepared.read_text())
    for key in ('description','enabled','front_pairs','proxy_id'):data.pop(key,None)
    data.update(schema='manual_inscribed_spheres.v1',policy=POLICY,source_fingerprint=source_fingerprint(g),
        provenance={'collision_parts_sha256':hashlib.sha256(collision_parts.read_bytes()).hexdigest(),
            'proposal_cache_sha256':hashlib.sha256(proposal_cache.read_bytes()).hexdigest(),
            'geometry_source':'D link-local collision parts; C nominal visual meshes, FK and three camera housings',
            'method':'D interior-distance proposals, capped to C component-union interior and nearest original visual surface',
            'installed_calibration_consumed':False,'D_FK_or_joint_state_consumed':False},
        containment={'positive_radii':True,'all_centers_in_original_visual_component_union':True,
            'radius_cap':'minimum proposal radius, D internal surface distance, C original visual unsigned distance, with 1e-7 m inset',
            'scope':'practical original-triangle containment check; source STL is not always a watertight manifold; no complete surface coverage claim'})
    data['proxy_id']=proxy_digest(data)
    checker=ManualSelfCollision(g,prebuilt=data)  # Structural/source/policy gate before expensive offline checks.
    containment=verify_containment(g,data)
    receipt=dict(proxy_id=data['proxy_id'],containment=containment,model=checker.description,
                 wall_seconds=time.perf_counter()-began,offline_only=True)
    output.parent.mkdir(parents=True,exist_ok=True)
    with output.open('x') as stream:stream.write(json.dumps(data,sort_keys=True,separators=(',',':'),allow_nan=False)+'\n')
    output.with_suffix('.qualification.json').write_text(json.dumps(receipt,indent=2)+'\n')
    return receipt


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('config','prepared','collision-parts','proposal-cache','output'):
        parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args()
    print(json.dumps(build(args.config,args.prepared,args.collision_parts,args.proposal_cache,args.output)))


if __name__=='__main__':main()
