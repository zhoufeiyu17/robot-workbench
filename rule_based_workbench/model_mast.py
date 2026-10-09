"""Add an upper mast without moving the original AC-One camera seat.

The source chassis is preserved triangle-for-triangle. The extension reuses
the upright cross-section and an upper copy of the original mounting head.
This is an assembly reference, not a strength or collision-cooking validation.
"""
import hashlib
import json
from pathlib import Path
import sys

import numpy as np


def close_extrusion_bottom(triangles, cut=.23):
    """Cap the single concave cut loop; retain every boundary vertex/edge."""
    triangles=np.asarray(triangles,dtype=np.float32).astype(float)
    vertices,inverse=np.unique(triangles.reshape(-1,3),axis=0,return_inverse=True)
    faces=inverse.reshape(-1,3)
    edges=np.concatenate([faces[:,[0,1]],faces[:,[1,2]],faces[:,[2,0]]])
    unique,counts=np.unique(np.sort(edges,axis=1),axis=0,return_counts=True)
    if np.any(counts>2):raise ValueError('nonmanifold source cannot be repaired by a cap')
    boundary=unique[counts==1];adj={}
    if not len(boundary):raise ValueError('expected one open extrusion boundary')
    for a,b in boundary:
        adj.setdefault(int(a),[]).append(int(b));adj.setdefault(int(b),[]).append(int(a))
    if any(len(v)!=2 for v in adj.values()):raise ValueError('boundary is not a simple loop')
    loop=[];start=min(adj);current=start;previous=None
    while True:
        if current in loop:raise ValueError('boundary repeats a vertex')
        loop.append(current);nxt=next(v for v in adj[current] if v!=previous)
        if nxt==start:break
        previous,current=current,nxt
    if len(loop)!=len(adj) or not np.allclose(vertices[loop,2],cut,atol=1e-8,rtol=0):
        raise ValueError('unexpected open boundary outside extrusion bottom')
    points=vertices[:,:2]
    def cross(a,b,c):
        u,v=points[b]-points[a],points[c]-points[a]
        return u[0]*v[1]-u[1]*v[0]
    area=sum(points[a,0]*points[b,1]-points[b,0]*points[a,1] for a,b in zip(loop,loop[1:]+loop[:1]))
    if area<0:loop.reverse()
    pending=loop[:];cap=[];eps=1e-16
    while len(pending)>3:
        for i,b in enumerate(pending):
            a,c=pending[i-1],pending[(i+1)%len(pending)]
            if cross(a,b,c)<=eps:continue
            if any(cross(a,b,p)>=-eps and cross(b,c,p)>=-eps and cross(c,a,p)>=-eps
                   for p in pending if p not in (a,b,c)):continue
            cap.append([c,b,a]);pending.pop(i);break
        else:raise ValueError('cannot triangulate concave cut without changing boundary')
    if cross(*pending)<=eps:raise ValueError('degenerate final cap')
    cap.append(pending[::-1])
    result=np.concatenate([triangles,vertices[np.array(cap)]])
    return result,dict(boundary_edges=len(boundary),cap_triangles=len(cap),plane_z_m=cut,
                       original_triangles_unchanged=True,method='planar concave ear clipping; no vertex displacement')


def build(source, output, native_scripts):
    sys.path.insert(0, str(native_scripts))
    from build_ac_one_no_handle_variant import _read_binary_stl, _component_indices
    from calibrated_urdf import write_stl
    data = Path(source).read_bytes()
    digest = hashlib.sha256(data).hexdigest()
    if digest != '29c919ed8251ad28fc107cf10d3b4c4c7708ae1b42b5104e716a783fd3f10728':
        raise ValueError('unexpected original chassis')
    _, records = _read_binary_stl(data)
    components = _component_indices(records)
    mast = next(c for c in components if len(c) == 4832)
    seat = next(c for c in components if len(c) == 512)
    original = np.array([r.vertices for r in records])
    cut, extension = .23, .24
    extra = []
    for index in mast:
        triangle = original[index]
        clipped, intersections = [], []
        for a, b in zip(triangle, np.roll(triangle, -1, axis=0)):
            if a[2] >= cut:
                clipped.append(a.copy())
            if (a[2] - cut) * (b[2] - cut) < 0:
                p = a + (cut-a[2]) / (b[2]-a[2]) * (b-a)
                clipped.append(p); intersections.append(p)
        if len(clipped) >= 3:
            upper = np.array(clipped) + [0, 0, extension]
            extra.extend([upper[[0, i, i+1]] for i in range(1, len(upper)-1)])
        if len(intersections) == 2:
            a, b = intersections
            aa, bb = a + [0, 0, extension], b + [0, 0, extension]
            extra.extend([np.array([a, b, bb]), np.array([a, bb, aa])])
    extra.extend(original[list(seat)] + [0, 0, extension])
    result,cap_report = close_extrusion_bottom(np.concatenate([original, np.asarray(extra)]))
    output = Path(output); output.parent.mkdir(parents=True, exist_ok=True)
    write_stl(output, result.reshape(-1, 3), np.arange(result.size//3).reshape(-1, 3))
    # Exact source vertices survive float32 STL roundtrip at the same indices.
    _, checked = _read_binary_stl(output.read_bytes())
    assert np.array_equal(np.array([r.vertices for r in checked[:len(records)]]), original)
    assert np.array_equal(np.array([checked[i].vertices for i in seat]), original[list(seat)])
    assert abs(result[:,:,2].max() - (original[:,:,2].max()+extension)) < 1e-7
    report = dict(source_sha256=digest, output_sha256=hashlib.sha256(output.read_bytes()).hexdigest(),
                  source_triangles=len(original), added_triangles=len(result)-len(original),
                  original_chassis_and_seat_preserved=True, original_seat_triangles=len(seat),
                  extension_m=extension, extrusion_start_z_m=cut,
                  upper_head_translation_m=[0,0,extension],
                  assembly_overlap='original upper head retained inside added upright; no boolean union',
                  closed_bottom=cap_report,
                  validation='geometry only; attachment fabrication and collision cooking not validated')
    from arx_env.joint_head_workbench_geometry import closed_component_representatives
    representatives=closed_component_representatives(np.array([r.vertices for r in checked]).reshape(-1,3))
    if representatives is None:raise ValueError('calibration closed component gate failed')
    report['closed_component_gate_passed']=True
    report['closed_components']=len(representatives)
    output.with_suffix('.json').write_text(json.dumps(report, indent=2)+'\n')
    return report


if __name__ == '__main__':
    import argparse
    p = argparse.ArgumentParser()
    for key in ('source', 'output', 'native-scripts'): p.add_argument('--'+key, required=True, type=Path)
    a=p.parse_args(); print(json.dumps(build(a.source, a.output, a.native_scripts), indent=2))
