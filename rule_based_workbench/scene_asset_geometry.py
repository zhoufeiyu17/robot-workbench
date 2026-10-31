"""Source-mesh planning envelopes; no mass, friction or manipulation claims."""
import math
import numpy as np


def mesh_edges(vertices, faces):
    triangles=np.asarray(vertices,dtype=float)[np.asarray(faces,dtype=int)]
    return triangles[:,[0,1,2]].reshape(-1,3),triangles[:,[1,2,0]].reshape(-1,3)


def section_radius(vertices, edges, z):
    a,b=edges
    mask=(np.minimum(a[:,2],b[:,2])<=z)&(np.maximum(a[:,2],b[:,2])>=z)&(abs(b[:,2]-a[:,2])>1e-12)
    left,right=a[mask],b[mask]
    points=left+(right-left)*((z-left[:,2])/(right[:,2]-left[:,2]))[:,None]
    on=vertices[abs(vertices[:,2]-z)<1e-12]
    points=np.vstack((points,on))
    if not len(points):raise ValueError('cup mesh has no section at z='+str(z))
    return float(np.linalg.norm(points[:,:2],axis=1).max())


def cup_bands(vertices,faces,*,radial_tolerance_m=.00005,segments=32,max_bands=64,minimum_band_height_m=.0001):
    """Circumscribed frusta covering every source triangle, including between samples.

    Sampled sections propose knots only. Source vertices and edge intersections
    at every knot certify coverage: inside a frustum slab a triangle's extrema
    occur at those clipped vertices. Convexity then covers the whole triangle.
    """
    vertices=np.asarray(vertices,dtype=float)
    if (vertices.ndim!=2 or vertices.shape[1]!=3 or not np.isfinite(vertices).all()
            or abs(vertices[:,2].min())>1e-9 or vertices[:,2].max()<=.1
            or not 0<radial_tolerance_m<=.0002 or segments<32):
        raise ValueError('finite bottom-origin cup mesh and bounded approximation required')
    height=float(vertices[:,2].max());edges=mesh_edges(vertices,faces)
    z=np.linspace(0.,height,257)
    r=np.array([section_radius(vertices,edges,t)for t in z])
    knots=[0,len(z)-1]
    while True:
        error=abs(r-np.interp(z,z[knots],r[knots]));i=int(error.argmax())
        if error[i]<=radial_tolerance_m:break
        knots=sorted({*knots,i})
        if len(knots)>max_bands+1:raise ValueError('cup silhouette exceeds the band budget')
    zs=z[knots].tolist();cosine=math.cos(math.pi/segments)
    vertex_radius=np.linalg.norm(vertices[:,:2],axis=1)
    while True:
        # Circumscription compensates the native regular mesh's inscribed faces.
        radii=np.array([(section_radius(vertices,edges,t)+radial_tolerance_m)/cosine for t in zs])
        violation=vertex_radius-np.interp(vertices[:,2],zs,radii)*cosine
        i=int(violation.argmax())
        if violation[i]<=1e-10:break
        zs=sorted({*zs,float(vertices[i,2])})
        if len(zs)>max_bands+1:raise ValueError('cup vertex coverage exceeds the band budget')
    original_zs,original_radii=zs.copy(),radii.copy()
    bands=[dict(shape_path=f'/Cup/Collisions/Band{i:03d}',z_min_m=a,z_max_m=b,
        radius_m=float(max(ra,rb)),radius_start_m=float(ra),radius_end_m=float(rb))
        for i,(a,b,ra,rb)in enumerate(zip(zs,zs[1:],radii,radii[1:]))]
    # Blender bevels can contain micron-thin layers. Merge their conservative
    # envelope before USD convex cooking; never erase their source protrusions.
    merged=0
    while len(bands)>1:
        small=next((i for i,b in enumerate(bands)if b['z_max_m']-b['z_min_m']<minimum_band_height_m),None)
        if small is None:break
        neighbor=small-1 if small else 1
        left,right=sorted((small,neighbor));a,b=bands[left],bands[right]
        radius=max(a['radius_m'],b['radius_m'])
        bands[left:right+1]=[dict(z_min_m=a['z_min_m'],z_max_m=b['z_max_m'],
            radius_start_m=radius,radius_end_m=radius,radius_m=radius)];merged+=1
    for a,b in zip(bands,bands[1:]):
        boundary=max(a['radius_end_m'],b['radius_start_m'])
        a['radius_end_m']=b['radius_start_m']=boundary
        a['radius_m']=max(a['radius_start_m'],boundary);b['radius_m']=max(boundary,b['radius_end_m'])
    for i,b in enumerate(bands):b['shape_path']=f'/Cup/Collisions/Band{i:03d}'
    zs=[b['z_min_m']for b in bands]+[bands[-1]['z_max_m']]
    radii=np.array([b['radius_start_m']for b in bands]+[bands[-1]['radius_end_m']])
    enlargement=np.interp(original_zs,zs,radii)-original_radii
    violation=vertex_radius-np.interp(vertices[:,2],zs,radii)*cosine
    knot_reserve=min(radius*cosine-section_radius(vertices,edges,t)for t,radius in zip(zs,radii))
    if knot_reserve<0 or float(violation.max())>1e-10 or enlargement.min()<-1e-12:
        raise ValueError('cup coverage proof failed')
    return bands,dict(source_vertices=len(vertices),source_triangles=len(faces),bands=len(bands),
        polygon_segments=segments,radial_fit_tolerance_m=radial_tolerance_m,
        minimum_vertex_containment_m=float(-violation.max()),minimum_knot_containment_m=float(knot_reserve),
        minimum_band_height_m=min(b['z_max_m']-b['z_min_m']for b in bands),merged_thin_bands=merged,
        maximum_radial_enlargement_from_merging_m=float(enlargement.max()),
        full_triangle_surface_containment=True,proof='convex frustum per slab; all original vertices and all edge/plane intersections covered',
        hollow_interior_modelled=False,physical_metrology=False,grasp_qualified=False)


def surface_boxes(vertices,faces,clip,*,pitch_m=.01,skin_m=.0002):
    """Reuse native halfspace clipping; preserve concave free space in X/Z cells.

    Y intervals merge only when their clipped surfaces overlap. Each emitted
    AABB contains all assigned clipped polygons. This describes authored
    surfaces, with no inference about unseen solid interiors.
    """
    triangles=np.asarray(vertices,dtype=float)[np.asarray(faces,dtype=int)]
    lower,upper=triangles.min((0,1)),triangles.max((0,1))
    boxes=[];covered=set();polygon_count=0
    for x in np.arange(lower[0],upper[0]+1e-10,pitch_m):
        for z in np.arange(lower[2],upper[2]+1e-10,pitch_m):
            lo,hi=lower.copy(),upper.copy();lo[[0,2]]=(x,z);hi[[0,2]]=np.minimum((x+pitch_m,z+pitch_m),upper[[0,2]])
            selected=np.flatnonzero(np.all(triangles.max(1)>=lo-1e-12,axis=1)&np.all(triangles.min(1)<=hi+1e-12,axis=1))
            intervals=[]
            for index in selected:
                polygon=triangles[index]
                for axis in (0,2):
                    for bound,is_lower in ((lo[axis],True),(hi[axis],False)):
                        polygon=clip(polygon,axis,bound,is_lower)
                        if len(polygon)<3:break
                    if len(polygon)<3:break
                if len(polygon)<3:continue
                covered.add(int(index));polygon_count+=1
                intervals.append((float(polygon[:,1].min()),float(polygon[:,1].max()),polygon))
            intervals.sort(key=lambda row:row[0])
            groups=[]
            for a,b,points in intervals:
                if not groups or a>groups[-1][1]+2*skin_m:groups.append([a,b,[points]])
                else:groups[-1][1]=max(groups[-1][1],b);groups[-1][2].append(points)
            for _,_,polygons in groups:
                points=np.concatenate(polygons);a,b=points.min(0)-skin_m,points.max(0)+skin_m
                boxes.append(dict(center=((a+b)/2).tolist(),dimensions=(b-a).tolist()))
    # Exactly zero-area source triangles carry no collision surface.
    valid=set(map(int,np.flatnonzero(np.linalg.norm(np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0]),axis=1)>0)))
    if not valid<=covered:raise ValueError('source triangles missing from the surface envelope')
    return boxes,dict(source_triangles=len(triangles),covered_triangles=len(covered),nonzero_triangles=len(valid),
        clipped_polygons=polygon_count,box_count=len(boxes),pitch_m=pitch_m,skin_m=skin_m,
        triangle_surface_coverage=True,solid_interior_inferred=False)
