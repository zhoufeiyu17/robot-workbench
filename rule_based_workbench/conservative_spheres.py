"""Offline conservative box-cell cover. No mesh distance fitting or online audit.

Each sphere contains an entire cell of an enclosing oriented box, hence the
whole solid, not only sampled vertices. Separate material/part identities stay
with the caller. Extra occupied space is reported, never called a mesh margin.
"""
from itertools import product
import math


def cover(vertices, *, faces=None, pitch_m=.055, face_overhang_m=None, protected_normal=None, backing_m=0., clip_boundary=False, refine_cells=()):
    import numpy as np
    points = np.asarray(vertices, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 4 or not np.isfinite(points).all():
        raise ValueError('finite solid vertices required')
    if not math.isfinite(pitch_m) or pitch_m <= 0:
        raise ValueError('positive cell pitch required')
    # PCA only chooses orientation. Coverage follows from projecting ALL points
    # to the enclosing box and covering every box cell, not from PCA accuracy.
    origin = points.mean(0)
    _, axes = np.linalg.eigh((points-origin).T@(points-origin))
    local = (points-origin)@axes
    lo, hi = local.min(0), local.max(0)
    widths = hi-lo
    if np.any(widths <= 1e-12):
        raise ValueError('nonzero solid thickness required')
    counts = np.maximum(1, np.ceil(widths/pitch_m)).astype(int)
    thin = int(widths.argmin())
    direction = 1.
    if protected_normal is not None:
        normal = np.asarray(protected_normal, dtype=float)
        if normal.shape != (3,) or not np.isfinite(normal).all() or np.linalg.norm(normal) < 1e-9:
            raise ValueError('finite nonzero protected normal required')
        alignment = normal@axes
        thin = int(np.abs(alignment).argmax())
        direction = 1. if alignment[thin] > 0 else -1.
    if not math.isfinite(backing_m) or backing_m < 0 or (backing_m and protected_normal is None):
        raise ValueError('backing shift requires a protected face normal')
    if face_overhang_m is not None:
        if not math.isfinite(face_overhang_m) or face_overhang_m <= 0:
            raise ValueError('positive face overhang required')
        # Keep the thin dimension whole. Limit the sphere's protrusion through
        # either thin-axis BOX face; source mesh deviation from box is separate.
        counts[thin] = 1
        half = widths[thin]/2+backing_m
        tangent_pitch = math.sqrt(2*((half+face_overhang_m)**2-half**2))
        for axis in range(3):
            if axis != thin:
                counts[axis] = max(counts[axis], math.ceil(widths[axis]/tangent_pitch))
    if math.prod(map(int, counts)) > 4096:
        raise ValueError('box cover exceeds bounded offline cell count')
    cell = widths/counts
    half = cell/2
    farthest = half.copy(); farthest[thin] += backing_m
    radius = float(np.linalg.norm(farthest))+1e-8
    lattice = np.asarray(list(product(*(range(int(n)) for n in counts))), dtype=float)
    local_centers = lo+(lattice+.5)*cell
    cells=[(str(i),center,half.copy()) for i,center in enumerate(local_centers)]
    requested=set(refine_cells)
    for _ in range(8):
        if not any(key in requested for key,_,_ in cells):break
        split=[]
        for key,center,size in cells:
            if key not in requested:split.append((key,center,size));continue
            axis=int(size.argmax());child=size.copy();child[axis]/=2
            for sign in (-1,1):
                shift=np.zeros(3);shift[axis]=sign*child[axis]
                split.append((key+('/0' if sign<0 else '/1'),center+shift,child.copy()))
        cells=split
        if len(cells)>4096:raise ValueError('local refinement exceeds offline cell budget')
    keys=[row[0] for row in cells]
    half_sizes=np.asarray([row[2] for row in cells])
    centers=np.asarray([row[1] for row in cells])@axes.T+origin
    kept = np.ones(len(centers), dtype=bool)
    if faces is not None:
        indices=np.asarray(faces)
        if indices.ndim != 2 or indices.shape[1] != 3 or indices.dtype.kind not in 'iu' or indices.min()<0 or indices.max()>=len(points):
            raise ValueError('valid source triangle indices required')
        triangles=points[indices]
        normals=np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0])
        normals=normals[np.linalg.norm(normals,axis=1)>1e-15]
        for normal in normals:
            # Whole-cell separation from a slab containing ALL source vertices
            # proves this cell contains no source solid, even for concave parts.
            source=points@normal; projected=centers@normal
            extent=half_sizes@np.abs(axes.T@normal)
            kept &= (projected+extent>=source.min()-1e-12)&(projected-extent<=source.max()+1e-12)
        centers=centers[kept]
        half_sizes=half_sizes[kept];keys=[key for key,keep in zip(keys,kept) if keep]
    centers -= direction*backing_m*axes[:,thin]
    farthest_sizes=half_sizes.copy();farthest_sizes[:,thin]+=backing_m
    radii = np.linalg.norm(farthest_sizes,axis=1)+1e-8
    clipped=0;fallback=0
    if clip_boundary:
        if backing_m:
            raise ValueError('boundary clipping requires unbiased cell centers')
        from scipy.spatial import ConvexHull,HalfspaceIntersection,QhullError
        from scipy.optimize import linprog
        from trimesh.nsphere import minimum_nsphere
        planes=ConvexHull(points).equations

        box_normals=np.vstack((axes.T,-axes.T))
        for i,center in enumerate(centers.copy()):
            half=half_sizes[i]
            extent=np.abs(planes[:,:3]@axes)@half
            if np.max(planes[:,:3]@center+planes[:,3]+extent)<=0:
                continue
            box_offsets=np.r_[-axes.T@center-half,axes.T@center-half]
            hs=np.vstack((planes,np.column_stack((box_normals,box_offsets))))
            # Standard convex intersection, once during model preparation.
            # On numerical ambiguity retain the original enclosing cell.
            fit=linprog([0.,0.,0.,-1.],A_ub=np.column_stack((hs[:,:3],np.linalg.norm(hs[:,:3],axis=1))),
                b_ub=-hs[:,3],bounds=[(None,None)]*3+[(0,None)],method='highs')
            if not fit.success or fit.x[3]<=1e-9:
                fallback+=1;continue
            try:vertices_inside=HalfspaceIntersection(hs,fit.x[:3]).intersections
            except QhullError:
                fallback+=1;continue
            if not len(vertices_inside) or not np.isfinite(vertices_inside).all():
                fallback+=1;continue
            candidate=vertices_inside.mean(0)
            try:
                fitted,_=minimum_nsphere(vertices_inside)
                if np.isfinite(fitted).all() and np.linalg.norm(vertices_inside-fitted,axis=1).max()<np.linalg.norm(vertices_inside-candidate,axis=1).max():
                    candidate=fitted
            except (QhullError,ValueError):
                pass  # the vertex-mean enclosing sphere is still conservative
            candidate_radius=float(np.linalg.norm(vertices_inside-candidate,axis=1).max())+1e-7
            if protected_normal is not None:
                unit=np.asarray(protected_normal,dtype=float);unit/=np.linalg.norm(unit)
                if candidate@unit+candidate_radius>center@unit+radii[i]:
                    candidate=center
                    candidate_radius=float(np.linalg.norm(vertices_inside-candidate,axis=1).max())+1e-7
            if candidate_radius<radii[i]:
                centers[i]=candidate;radii[i]=candidate_radius;clipped+=1
    # Remove only whole balls already contained in a retained ball. This leaves
    # the covered solid and max sphere-pair penetration unchanged.
    retained=[]
    for index in np.argsort(-radii,kind='stable'):
        if retained and np.any(np.linalg.norm(centers[retained]-centers[index],axis=1)+radii[index] <= radii[retained]-1e-9):
            continue
        retained.append(int(index))
    removed=len(radii)-len(retained);retained.sort()
    centers=centers[retained];radii=radii[retained];keys=[keys[i] for i in retained]
    report = dict(contained_spheres_removed=removed,method='oriented_box_cells', whole_solid_covered=True,
        proof=('source solid lies in convex hull; each retained hull/cell section lies in its vertex circumsphere; ambiguous cells keep full cell cover'
               if clip_boundary else 'source solid lies in vertex enclosing box; every nonempty box cell lies in its circumsphere'),
        cell_keys=keys, local_refinement_count=len(refine_cells), source_vertex_count=len(points), box_origin_m=origin.tolist(), box_axes=axes.tolist(),
        box_min_m=lo.tolist(), box_max_m=hi.tolist(), cell_counts=counts.tolist(),
        cell_size_m=cell.tolist(), sphere_radius_m=None if clip_boundary else radius,
        radius_range_m=[float(radii.min()),float(radii.max())],source_box_volume_m3=float(widths.prod()),
        retained_cells=int(kept.sum()), discarded_empty_cells=int((~kept).sum()),
        box_face_overhang_m=(radius-cell/2).tolist(), thin_axis=thin,
        thin_face_overhang_m=float(radius-cell[thin]/2),
        protected_face_overhang_m=float(radius-cell[thin]/2-backing_m),
        backing_face_overhang_m=float(radius-cell[thin]/2+backing_m),
        backing_m=backing_m, protected_box_axis_direction=direction,
        clipped_boundary_cells=clipped,clipping_fallback_cells=fallback,
        clipping_method='scipy convex hull / cell intersection' if clip_boundary else None,
        mesh_outward_distance_certified=False, cavities_may_be_conservatively_occupied=True)
    if protected_normal is not None:
        unit = np.asarray(protected_normal,dtype=float); unit /= np.linalg.norm(unit)
        report['protected_direction_overhang_m'] = float((centers@unit+radii).max()-(points@unit).max())
    if clip_boundary:
        local_centers=(centers-origin)@axes
        report['actual_box_positive_overhang_m']=((local_centers+radii[:,None]).max(0)-hi).tolist()
        report['actual_box_negative_overhang_m']=(lo-(local_centers-radii[:,None]).min(0)).tolist()
    return centers, radii, report


if __name__ == '__main__':
    import numpy as np
    # A rotated thin solid exercises both coverage and the grip-face overhang.
    points = np.asarray(list(product((-.035,.035),(-.00075,.00075),(-.03,.03))))
    angle=.37
    rotation=np.array([[math.cos(angle),-math.sin(angle),0],[math.sin(angle),math.cos(angle),0],[0,0,1]])
    points=points@rotation.T
    centers,radii,report=cover(points,face_overhang_m=.002)
    assert report['thin_face_overhang_m'] <= .002+2e-8
    axes=np.asarray(report['box_axes'])
    half=np.asarray(report['cell_size_m'])/2
    for signs in product((-1,1),repeat=3):
        corners=centers+(half*np.asarray(signs))@axes.T
        assert np.all(np.linalg.norm(corners-centers,axis=1)<=radii)
    for alpha in np.random.default_rng(3).dirichlet(np.ones(8),200):
        point=alpha@points
        assert np.min(np.linalg.norm(centers-point,axis=1)-radii)<=1e-9
    centers,radii,report=cover(points,face_overhang_m=.002,protected_normal=rotation[:,1],backing_m=.01)
    assert report['protected_face_overhang_m'] <= .002+2e-8
    assert report['protected_direction_overhang_m'] <= .002+2e-8
    for alpha in np.random.default_rng(4).dirichlet(np.ones(8),200):
        assert np.min(np.linalg.norm(centers-alpha@points,axis=1)-radii)<=1e-9
    refined,rr,proof=cover(points,face_overhang_m=.002,refine_cells=('0','0/0'))
    assert '0' not in proof['cell_keys'] and '0/0' not in proof['cell_keys']
    assert {'0/0/0','0/0/1','0/1'} <= set(proof['cell_keys'])
    for alpha in np.random.default_rng(5).dirichlet(np.ones(8),1000):
        assert np.min(np.linalg.norm(refined-alpha@points,axis=1)-rr)<=1e-9
    print('conservative rotated solid cover, local subdivision and thin-face bound: PASS')
