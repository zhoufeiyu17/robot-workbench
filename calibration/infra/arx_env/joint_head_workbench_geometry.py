"""CPU X5 preview/planning. Nominal mesh checks are not real hardware clearance.

Projection uses rectified pinhole intrinsics. Full local mesh convex hulls are
used for collision after OBB screening; sparse display triangles NEVER drive checks.
Configured chassis uses connected-component screening and full triangle narrow phase.
Adjacent links are excluded. Wrist shells/brackets are optional estimated proxies;
visibility means geometric framing/front-facing, not image detection.
"""
import json
import copy
from functools import lru_cache
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.optimize import least_squares, linprog
from scipy.spatial import ConvexHull
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from .joint_head_isaac_fit import Model, rigid
from .urdf_mesh import origin


def _read(value):
    return json.loads(Path(value).read_text()) if isinstance(value, (str, Path)) else value


def _points(points, T):
    return points @ T[:3, :3].T + T[:3, 3]


def stl_vertices(path):
    raw = Path(path).read_bytes()
    n = int.from_bytes(raw[80:84], 'little')
    if len(raw) >= 84 and len(raw) == 84+50*n:
        dt = np.dtype([('normal','<f4',(3,)),('v','<f4',(3,3)),('attr','<u2')])
        vertices = np.frombuffer(raw, dtype=dt, offset=84)['v'].astype(float).reshape(-1,3)
    else:
        vertices = np.array([list(map(float,line.split()[1:])) for line in raw.decode().splitlines()
                             if line.strip().startswith('vertex ')], dtype=float)
    if vertices.ndim != 2 or vertices.shape[1] != 3 or not len(vertices) or len(vertices)%3 or not np.isfinite(vertices).all():
        raise ValueError('STL requires finite, nonempty triangles')
    return vertices


def point_in_mesh(point, triangles):
    direction = np.array([1., .371390676, .529173]); direction /= np.linalg.norm(direction)
    a,b,c = triangles[:,0], triangles[:,1], triangles[:,2]
    e1,e2 = b-a,c-a
    h = np.cross(direction,e2); determinant = np.einsum('ij,ij->i',e1,h)
    valid = abs(determinant)>1e-12
    inverse = np.divide(1.,determinant,out=np.zeros_like(determinant),where=valid)
    offset = point-a
    u = inverse*np.einsum('ij,ij->i',offset,h)
    q = np.cross(offset,e1)
    v = inverse*(q@direction)
    t = inverse*np.einsum('ij,ij->i',e2,q)
    hits = t[valid & (u>=0) & (v>=0) & (u+v<=1) & (t>1e-9)]
    return bool(len(np.unique(np.round(hits,9)))%2)


def closed_component_representatives(vertices):
    """Flattened triangle vertex indices, or None when closed topology is unproven."""
    vertices = np.asarray(vertices).reshape(-1,3)
    unique, first, inverse = np.unique(vertices, axis=0, return_index=True, return_inverse=True)
    faces = inverse.reshape(-1,3)
    if not len(faces) or np.any(np.linalg.norm(np.cross(vertices[1::3]-vertices[::3],
                                                       vertices[2::3]-vertices[::3]),axis=1) == 0):
        return None
    edges = np.concatenate((faces[:,[0,1]],faces[:,[1,2]],faces[:,[2,0]]))
    _, incidence = np.unique(np.sort(edges,axis=1),axis=0,return_counts=True)
    if np.any(incidence != 2):
        return None
    graph = coo_matrix((np.ones(len(edges)),(edges[:,0],edges[:,1])),shape=(len(unique),len(unique)))
    _, labels = connected_components(graph,directed=False)
    _, representatives = np.unique(labels,return_index=True)
    return first[representatives]


def surface_intersection_pairs(a, b):
    """Yield full-mesh triangle pairs using the existing AABB/SAT test."""
    amin,amax = a.min(1),a.max(1)
    bmin,bmax = b.min(1),b.max(1)
    lo,hi = bmin.min(0),bmax.max(0)
    active = np.flatnonzero(np.all(amax>=lo-1e-9,axis=1)&np.all(amin<=hi+1e-9,axis=1))
    # ponytail: triangle AABB scan is quadratic in close meshes; use a BVH if planning profiles make this dominant.
    for i in active:
        indices = np.flatnonzero(np.all(bmax>=amin[i]-1e-9,axis=1)&np.all(bmin<=amax[i]+1e-9,axis=1))
        candidates = b[indices]
        if not len(candidates): continue
        ea = np.roll(a[i],-1,axis=0)-a[i]
        eb = np.roll(candidates,-1,axis=1)-candidates
        na = np.cross(ea[0],ea[1]); nb = np.cross(eb[:,0],eb[:,1])
        axes = np.concatenate((np.broadcast_to(na,(len(eb),1,3)),nb[:,None,:],
            np.cross(ea[None,:,None,:],eb[:,None,:,:]).reshape(-1,9,3),
            np.broadcast_to(np.cross(na,ea),(len(eb),3,3)),np.cross(nb[:,None,:],eb)),axis=1)
        length = np.linalg.norm(axes,axis=2)
        axes = axes/np.maximum(length[:,:,None],1e-30)
        pa = np.einsum('vj,naj->nav',a[i],axes)
        pb = np.einsum('nvj,naj->nav',candidates,axes)
        separated = ((pa.max(2)<pb.min(2)-1e-9)|(pb.max(2)<pa.min(2)-1e-9))&(length>1e-14)
        for hit in np.flatnonzero(~separated.any(1)):
            yield int(i), int(indices[hit])


def meshes_intersect(a, b, representatives=None):
    """Real triangle SAT after AABB screening, then closed-component containment."""
    if next(surface_intersection_pairs(a, b), None) is not None:
        return True
    lo, hi = b.min((0,1)), b.max((0,1))
    # Full SAT above proved no surface crossing. A closed connected surface then
    # has one inside/outside classification; retain all vertices for unknown topology.
    points = (np.unique(a.reshape(-1,3),axis=0) if representatives is None
              else a.reshape(-1,3)[representatives])
    points = points[np.all(points>=lo,axis=1)&np.all(points<=hi,axis=1)]
    if any(point_in_mesh(point,b) for point in points): return True
    return point_in_mesh(b[0,0],a)


def box_triangles(size, T):
    size = np.asarray(size, float)
    if size.shape != (3,) or not np.isfinite(size).all() or np.any(size <= 0):
        raise ValueError('proxy box needs three positive finite dimensions')
    vertices = np.array([[x,y,z] for x in (-.5,.5) for y in (-.5,.5) for z in (-.5,.5)])*size
    faces = np.array([[0,1,3],[0,3,2],[4,6,7],[4,7,5],[0,4,5],[0,5,1],
                      [2,3,7],[2,7,6],[0,2,6],[0,6,4],[1,5,7],[1,7,3]])
    return _points(vertices[faces].reshape(-1,3), rigid(T))


def sticker_support(marker, size, thickness):
    """Estimated body behind the observed sticker; no optical-frame claim."""
    marker = rigid(marker)
    if not np.isfinite(thickness) or thickness <= 0:
        raise ValueError('positive bracket thickness required')
    body = marker.copy()
    # Marker coordinates are image-like; -Z is the printed/front side.
    body[:3,3] += marker[:3,2]*(size[2]/2+.0002)
    top = np.eye(4); top[2,3] = -size[2]/2
    sticker = np.eye(4); sticker[2,3] = -.0002
    endpoint = body[:3,3]
    length = np.linalg.norm(endpoint)
    if length < thickness: raise ValueError('proxy bracket endpoint too near flange')
    z = endpoint/length
    x = np.cross(z, [0,0,1] if abs(z[2])<.9 else [0,1,0]); x /= np.linalg.norm(x)
    rod = np.eye(4); rod[:3,:3] = np.column_stack((x,np.cross(z,x),z)); rod[:3,3]=endpoint/2
    return body, top, sticker, box_triangles(size,body), box_triangles([thickness,thickness,length],rod)


def boxes_overlap(a, b):
    """Separating axes for two OBBs: (center, axes-columns, half-lengths)."""
    ca, ra, ea = a
    cb, rb, eb = b
    axes = [*ra.T, *rb.T, *np.cross(ra.T[:, None], rb.T[None]).reshape(-1, 3)]
    for axis in axes:
        if np.linalg.norm(axis) < 1e-9:
            continue
        if abs((cb-ca) @ axis) > np.abs(ra.T @ axis) @ ea + np.abs(rb.T @ axis) @ eb:
            return False
    return True


def hand_template(meshes):
    matches = [(vertices, faces) for name, vertices, faces in meshes if name == "link6"]
    if len(matches) != 1:
        raise ValueError("expected exactly one link6 visual mesh")
    vertices, faces = matches[0]
    unique, inverse = np.unique(vertices, axis=0, return_inverse=True)
    merged_faces = inverse[faces]
    parent = np.arange(len(unique))

    def find(index):
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = parent[index]
        return index

    def union(a, b):
        a, b = find(a), find(b)
        if a != b:
            parent[b] = a

    for triangle in merged_faces:
        union(triangle[0], triangle[1])
        union(triangle[1], triangle[2])
    roots = np.array([find(index) for index in range(len(unique))])
    signs = np.zeros(len(unique), dtype=np.int8)
    positive_min, negative_max = [], []
    for root in np.unique(roots):
        indices = np.flatnonzero(roots == root)
        points = unique[indices]
        lo, hi = points.min(0), points.max(0)
        if lo[1] >= .025 and hi[0] >= .06:
            signs[indices] = 1
            positive_min.append(lo[1])
        elif hi[1] <= -.025 and hi[0] >= .06:
            signs[indices] = -1
            negative_max.append(hi[1])
    if not positive_min or not negative_max:
        raise ValueError("link6 finger components are not separable")
    nominal_gap = min(positive_min) - max(negative_max)
    if not .04 <= nominal_gap <= .08:
        raise ValueError(f"unexpected nominal finger gap: {nominal_gap}")
    return {"vertices": vertices, "faces": faces, "signs": signs[inverse], "nominal_gap_m": float(nominal_gap)}


def jaw_gaps(value):
    """Physical inner metal gaps, millimetres; never SDK encoder values."""
    from .joint_head_jaw_mapping import command_max_mm
    gaps = np.asarray(value, float)
    maximum = np.asarray([command_max_mm(side) for side in ('left', 'right')])
    if gaps.shape != (2,) or not np.isfinite(gaps).all() or np.any(gaps < 0) or np.any(gaps > maximum):
        raise ValueError('left/right metal inner gaps must be finite and within device command limits')
    return gaps


class Geometry:
    def __new__(cls, config=None, *args, **kwargs):
        if cls is Geometry and config and config.get('model_family') == 'ac_one':
            from .joint_head_ac_one_geometry import ACOneGeometry
            return object.__new__(ACOneGeometry)
        return object.__new__(cls)

    def __init__(self, config):
        self.config = config
        self.model = Model(config['urdf'], [f'joint{i}' for i in range(1, 7)])
        tree = ET.parse(config['urdf']).getroot()
        joints = {j.get('name'): j for j in tree.findall('joint')}
        limits = [joints[n].find('limit') for n in self.model.names]
        self.limits = np.tile([[float(r.get('lower')), float(r.get('upper'))] for r in limits], (2,1))
        self.velocity_limits = np.tile([float(r.get('velocity')) for r in limits], 2)
        self.bases = {s: rigid(v) for s,v in _read(config['scene_seed'])['T_world_arm_base'].items()}
        candidate = _read(config['candidate'])
        self.camera = rigid(candidate['T_world_optical'])
        self.mounts = {s: rigid(candidate['T_link6_marker'][str(i)]) for s,i in [('left',24),('right',30)]}
        self.mount_status = '历史自由拟合贴纸；缺少上壳几何'
        self.mount_chain = {}
        self.k = _read(config['camera'])
        self.margin = float(config.get('margin_px', 18))
        self.marker_size = float(config.get('marker_size_m', .015))
        self.gaps = config.get('gap_m', {'left':.083,'right':.083})
        manifest = _read(config['mesh_manifest'])['model']['meshes']
        root = Path(config['mesh_manifest']).parent
        self.meshes = []
        for link in tree.findall('link'):
            for visual in link.findall('visual'):
                m = visual.find('geometry/mesh')
                if m is None: continue
                v=stl_vertices(root / manifest[link.get('name')])*np.fromstring(m.get('scale','1 1 1'),sep=' ')
                v=_points(v,origin(visual.find('origin')))
                self.meshes.append((link.get('name'),v,np.arange(len(v)).reshape(-1,3)))
        hand = hand_template(self.meshes)
        self.hand = hand
        proxy = config.get('wrist_support_proxy')
        if proxy:
            size = np.asarray(proxy['shell_size_m'],float)
            box_triangles(size,np.eye(4))
            if np.min(size[:2]) < self.marker_size*4/3:
                raise ValueError('proxy upper surface must support full paper border')
            self.mount_status = '简化上壳/支架：按历史贴纸反推；非实物CAD，未标定腕相机光学坐标'
        housings = config.get('wrist_camera_housings', {})
        if not isinstance(housings,dict) or any(side not in ('left','right') for side in housings):
            raise ValueError('wrist_camera_housings must contain left/right objects')
        self.side_meshes = {}
        self.bounds = {}
        self.hulls = {}
        self.containment_representatives = {}
        for side in ('left','right'):
            self.side_meshes[side] = []
            meshes = list(self.meshes)
            if proxy:
                body,top,sticker,shell,rod = sticker_support(self.mounts[side],size,float(proxy['bracket_thickness_m']))
                self.mount_chain[side] = {'T_link6_camera_body_proxy':body.tolist(), 'T_body_upper_shell':top.tolist(),
                    'T_upper_shell_marker':sticker.tolist(), 'physical_geometry_verified':False,
                    'source':'historical sticker fit; user-authorized simple support approximation',
                    'shell_size_m':size.tolist(), 'bracket_thickness_m':proxy['bracket_thickness_m']}
                self.mounts[side] = body@top@sticker
                for name,v in [('camera_shell_proxy',shell),('camera_bracket_proxy',rod)]:
                    meshes.append((name,v,np.arange(len(v)).reshape(-1,3)))
            if side in housings:
                body = housings[side]
                vertices = box_triangles(body['size_m'],rigid(body['T_link6_body']))
                meshes.append(('camera_body',vertices,np.arange(len(vertices)).reshape(-1,3)))
            for name,v,f in meshes:
                v = v.copy()
                if name == 'link6': v[:,1] += hand['signs']*(self.gaps[side]-hand['nominal_gap_m'])/2
                lo,hi=v.min(0),v.max(0)
                self.hulls[side,name] = ConvexHull(np.unique(v,axis=0)).equations
                self.bounds[side,name] = ((lo+hi)/2, (hi-lo)/2)
                self.containment_representatives[side,name] = closed_component_representatives(v)
                # Display-only sampling, collision bounds retain EVERY vertex.
                self.side_meshes[side].append((name, v, f[::max(1,int(np.ceil(len(f)/500)))]))
        self.adjacent = {frozenset((j[1],j[2])) for j in self.model.joints}
        if proxy:
            for a,b in [('camera_shell_proxy','camera_bracket_proxy'),('camera_shell_proxy','link6'),('camera_bracket_proxy','link6'),('camera_bracket_proxy','link5')]:
                self.adjacent.add(frozenset((a,b)))
        if housings:
            for attached in ('link6','camera_shell_proxy','camera_bracket_proxy'):
                self.adjacent.add(frozenset(('camera_body',attached)))
        self.collision_method='full-mesh convex hull intersections; adjacent/attached parts excluded; estimated shell/bracket included when configured; no cables or unconfigured obstacles; sampled paths only'
        self._load_chassis(root)
        if self.static_meshes:
            self.collision_method += '; chassis component hull screening plus real-triangle SAT/closed-mesh containment; fixed arm-base contact excluded'

    def _load_chassis(self, root):
        self.static_meshes, self.static_hulls, self.static_triangles, self.static_labels = [], [], [], []
        if self.config.get('chassis_mesh'):
            scale = np.asarray(self.config.get('chassis_scale', [1,1,1]), float)
            if scale.shape != (3,) or not np.isfinite(scale).all() or np.any(scale <= 0):
                raise ValueError('chassis_scale requires three positive finite values')
            T = rigid(self.config.get('chassis_T_world_mesh', np.eye(4)))
            vertices = _points(stl_vertices(Path(root)/self.config['chassis_mesh'])*scale, T)
            self.static_meshes.append(('chassis',vertices,np.arange(len(vertices)).reshape(-1,3)))
        boxes = self.config.get('static_boxes',[])
        if not isinstance(boxes,list):
            raise ValueError('static_boxes must be a list')
        names = {'chassis'}  # Reserved for the sole intentional base-mount contact exemption.
        for box in boxes:
            if not isinstance(box,dict):
                raise ValueError('static box must be an object')
            name = box.get('name')
            if not isinstance(name,str) or not name or not name.isidentifier() or not name.isascii() or name in names:
                raise ValueError('static box names must be unique ASCII identifiers')
            names.add(name)
            vertices = box_triangles(box['size_m'],rigid(box['T_world_box']))
            self.static_meshes.append((name,vertices,np.arange(len(vertices)).reshape(-1,3)))
        self._build_static_hulls()

    def _build_static_hulls(self):
        for name,vertices,faces in self.static_meshes:
            unique, inverse = np.unique(vertices, axis=0, return_inverse=True)
            merged = inverse[faces]
            edges = np.concatenate((merged[:,[0,1]], merged[:,[1,2]], merged[:,[2,0]]))
            _, incidence = np.unique(np.sort(edges,axis=1), axis=0, return_counts=True)
            if np.any(incidence != 2):
                raise ValueError(name+' must contain closed two-manifold components for containment checks')
            graph = coo_matrix((np.ones(len(edges)), (edges[:,0],edges[:,1])), shape=(len(unique),len(unique)))
            count, labels = connected_components(graph, directed=False)
            # Component hulls are broad phase only: real faces below preserve mounting holes.
            for component in range(count):
                points = unique[labels == component]
                hull = ConvexHull(points)
                lo, hi = points.min(0), points.max(0)
                self.static_hulls.append((((lo+hi)/2, np.eye(3), (hi-lo)/2), hull.equations))
                self.static_triangles.append(vertices[faces[labels[merged[:,0]] == component]])
                self.static_labels.append(name)

    def with_marker_mounts(self, mounts):
        """Isolated geometry view: frozen stickers and their support proxies agree."""
        if set(mounts) != {'left','right'}:
            raise ValueError('marker mounts require left and right transforms')
        mounts = {side:rigid(T).copy() for side,T in mounts.items()}
        changed = [side for side in mounts if not np.array_equal(mounts[side],self.mounts[side])]
        if not changed:
            return self
        view = copy.copy(self)
        view.mounts = mounts
        view.mount_chain = dict(self.mount_chain)
        view.side_meshes = {side:list(parts) for side,parts in self.side_meshes.items()}
        view.bounds, view.hulls = dict(self.bounds), dict(self.hulls)
        view.containment_representatives = dict(getattr(self,'containment_representatives',{}))
        proxy = self.config.get('wrist_support_proxy')
        if proxy:
            size = np.asarray(proxy['shell_size_m'],float)
            for side in changed:
                body,top,sticker,shell,rod = sticker_support(mounts[side],size,float(proxy['bracket_thickness_m']))
                view.mount_chain[side] = {**self.mount_chain[side],
                    'T_link6_camera_body_proxy':body.tolist(), 'T_body_upper_shell':top.tolist(),
                    'T_upper_shell_marker':sticker.tolist(),
                    'source':'frozen marker mount; user-authorized simple support approximation'}
                replacement = {'camera_shell_proxy':shell, 'camera_bracket_proxy':rod}
                for i,(name,_,faces) in enumerate(view.side_meshes[side]):
                    if name not in replacement: continue
                    vertices = replacement[name]
                    lo,hi = vertices.min(0),vertices.max(0)
                    view.bounds[side,name] = ((lo+hi)/2,(hi-lo)/2)
                    view.hulls[side,name] = ConvexHull(np.unique(vertices,axis=0)).equations
                    view.containment_representatives[side,name] = closed_component_representatives(vertices)
                    view.side_meshes[side][i] = (name,vertices,faces)
        return view

    @lru_cache(maxsize=8)
    def with_gaps(self, left_mm, right_mm):
        gaps = jaw_gaps([left_mm, right_mm])
        view = copy.copy(self)
        view.gaps = dict(zip(('left','right'), gaps/1000))
        view.side_meshes = {side:list(parts) for side,parts in self.side_meshes.items()}
        view.bounds, view.hulls = dict(self.bounds), dict(self.hulls)
        view.containment_representatives = dict(getattr(self,'containment_representatives',{}))
        for side in ('left','right'):
            for i,(name,v,f) in enumerate(view.side_meshes[side]):
                if name != 'link6': continue
                v = v.copy()
                v[:,1] += self.hand['signs']*(view.gaps[side]-self.gaps[side])/2
                lo,hi = v.min(0),v.max(0)
                view.bounds[side,name] = ((lo+hi)/2,(hi-lo)/2)
                view.hulls[side,name] = ConvexHull(np.unique(v,axis=0)).equations
                view.containment_representatives[side,name] = closed_component_representatives(v)
                view.side_meshes[side][i] = (name,v,f)
        return view

    def _q(self,q):
        q=np.asarray(q,float)
        if q.shape != (12,) or not np.isfinite(q).all(): raise ValueError('12 finite joint angles required')
        return q

    def frames(self,q):
        q=self._q(q)
        frames = {side:{n:self.bases[side]@T for n,T in self.model.frames(q[j:j+6]).items()} for side,j in [('left',0),('right',6)]}
        for side, meshes in self.side_meshes.items():
            for name,_,_ in meshes:
                if name in ('camera_shell_proxy','camera_bracket_proxy','camera_body'):
                    frames[side][name]=frames[side]['link6']
        return frames

    def project(self,points):
        pc=(points-self.camera[:3,3])@self.camera[:3,:3]
        uv=pc[:,:2]/np.maximum(pc[:,2,None],1e-6)*[self.k['fx'],self.k['fy']]+[self.k['cx'],self.k['cy']]
        return uv,pc[:,2]

    def _marker(self,frames,side):
        T=frames[side]['link6']@self.mounts[side]
        s=self.marker_size/2
        points=_points(np.array([[-s,-s,0],[s,-s,0],[s,s,0],[-s,s,0]]),T)
        uv,z=self.project(points)
        center,_=self.project(T[None,:3,3])
        front = float(T[:3,2] @ (self.camera[:3,3]-T[:3,3])) < -.005
        # Bounding-box corners conservatively keep the entire claw in frame.
        c,e=self.bounds[side,'link6']
        corners=c+np.array([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)])*e
        hand_uv,hand_z=self.project(_points(corners,frames[side]['link6']))
        both=np.vstack((uv,hand_uv)); m=self.margin
        framed=bool(np.all(z>.03) and np.all(hand_z>.03) and np.all(both>[m,m]) and np.all(both<[self.k['width']-m,self.k['height']-m]))
        return {'id':24 if side=='left' else 30,'arm':side,'corners_uv':uv.tolist(),'center_uv':center[0].tolist(),'visible':framed and front,'framed':framed,'front_facing':front,'hand_uv':hand_uv.tolist(),'normal_facing':float(T[:3,2] @ (self.camera[:3,3]-T[:3,3]))}

    def collisions(self,frames):
        boxes=[]
        for (side,name),(c,e) in self.bounds.items():
            T=frames[side][name]; boxes.append((side,name,(_points(c,T),T[:3,:3],e+.002)))
        pairs=[]
        for side,name,box in boxes:
            T = frames[side][name]
            for component,(fixed_box,fixed_eq) in enumerate(getattr(self, 'static_hulls', [])):
                label = self.static_labels[component]
                if (name == 'base_link' and label == 'chassis') or (name, label) in getattr(self, 'static_adjacent', set()):
                    continue  # Only the original chassis mounting contact is intentional.
                if not boxes_overlap(box, fixed_box):
                    continue
                eq = self.hulls[side,name]
                normals = eq[:,:3]@T[:3,:3].T
                moving = np.c_[normals, eq[:,3]-normals@T[:3,3]]
                both = np.vstack((moving, fixed_eq))
                overlap = linprog(np.zeros(3), A_ub=both[:,:3], b_ub=-both[:,3], bounds=[(None,None)]*3, method='highs')
                if not overlap.success and overlap.status == 2:
                    continue
                representatives = getattr(self,'containment_representatives',{}).get((side,name))
                narrow_checked = overlap.success or representatives is not None
                if narrow_checked:
                    vertices = next(v for part,v,_ in self.side_meshes[side] if part == name)
                    triangles = _points(vertices,T).reshape(-1,3,3)
                    if not meshes_intersect(triangles, self.static_triangles[component], representatives):
                        continue
                pair = [side+'/'+name, 'environment/'+label]
                if not narrow_checked: pair.append('intersection solver uncertain')
                pairs.append(pair)
                break
        table=self.config.get('table_z_m')
        if table is not None:
            # Fixed mounting bases may intersect the support plane by construction.
            for side,meshes in self.side_meshes.items():
                for name,vertices,_ in meshes:
                    if name=='base_link':continue
                    T=frames[side][name]
                    if float(np.min(vertices@T[2,:3]+T[2,3]))<float(table)+float(self.config.get('table_clearance_m',.005)):
                        pairs.append([side+'/'+name,'environment/table'])
        for i,(sa,na,a) in enumerate(boxes):
            for sb,nb,b in boxes[i+1:]:
                if sa==sb and frozenset((na,nb)) in self.adjacent: continue
                if boxes_overlap(a,b):
                    inequalities=[]
                    for side,name in ((sa,na),(sb,nb)):
                        eq=self.hulls[side,name];T=frames[side][name]
                        normals=eq[:,:3]@T[:3,:3].T
                        inequalities.append(np.c_[normals,eq[:,3]-normals@T[:3,3]])
                    eq=np.vstack(inequalities)
                    overlap=linprog(np.zeros(3),A_ub=eq[:,:3],b_ub=-eq[:,3],bounds=[(None,None)]*3,method='highs')
                    if overlap.success: pairs.append([sa+'/'+na,sb+'/'+nb])
                    elif overlap.status != 2: pairs.append([sa+'/'+na,sb+'/'+nb,'intersection solver uncertain'])
        return pairs

    def preview(self,q,include_parts=True):
        q=self._q(q); frames=self.frames(q); collisions=self.collisions(frames)
        markers=[self._marker(frames,s) for s in ('left','right')]
        parts=[]
        if include_parts:
            for name,v,f in self.static_meshes:
                parts.append({'arm':'environment','link':name,'triangles':v[f[::max(1,int(np.ceil(len(f)/500)))]].tolist()})
            for side,meshes in self.side_meshes.items():
                for name,v,f in meshes:
                    parts.append({'arm':side,'link':name,'triangles':_points(v[f],frames[side][name]).tolist()})
        return {'q':q.tolist(),'parts':parts,'markers':markers,'visible_ids':[m['id'] for m in markers if m['visible']], 'collision_free':not collisions,'collisions':collisions,'in_limits':bool(np.all(q>=self.limits[:,0]) and np.all(q<=self.limits[:,1])),'collision_method':self.collision_method,'hardware_execution_approved':False}

    def path_check(self,start,target,side=None):
        start=self._q(start);target=self._q(target)
        steps=max(2,int(np.ceil(np.max(abs(target-start))/.04))+1)
        for t in np.linspace(0,1,steps):
            p=self.preview(start+(target-start)*t,False)
            if not p['in_limits'] or not p['collision_free']:
                return {'ok':False,'fraction':float(t),'reason':'limits or conservative collision','collisions':p['collisions']}
            if side and (24 if side=='left' else 30) not in p['visible_ids']:
                return {'ok':False,'fraction':float(t),'reason':'claw/tag outside geometric visibility'}
        return {'ok':True,'samples':steps,'certified_continuous_collision':False}

    @staticmethod
    def mirror_right_to_left(q):
        """X5 arm symmetry used by the standard calibration trajectory."""
        q=np.asarray(q,float).copy()
        if q.shape!=(12,): raise ValueError('12 finite joint angles required')
        # right J1..J6 -> left J1..J6; signs come from the nominal CAD bases.
        q[:6]=q[6:12]*np.array([-1.,1.,1.,1.,-1.,-1.])
        return q

    def auto_plan(self,q):
        seed_path = self.config.get('head_motion_seed')
        if seed_path is None:
            candidate = Path(__file__).resolve().parents[2] / 'standard_head_motion_seed.json'
            seed_path = candidate if candidate.exists() else None
        if seed_path is not None and Path(seed_path).exists():
            return self._plan_from_seed(self._q(q), _read(seed_path))
        # Plan around the observable target, not a decorative 3x3 image grid.
        # The center pose anchors visibility; bounded joint excitation supplies rank.
        current=self._q(q).copy(); waypoints=[];unreachable=[];coverage={}
        initial=self.preview(current,False)
        if not initial['in_limits'] or not initial['collision_free']:
            return {'waypoints':[],'plan_complete':False,'coverage':{},'unreachable':[{'reason':'initial pose fails model limits/collision','collisions':initial['collisions']}], 'hardware_execution_approved':False}
        zero=np.zeros(12)
        zero_preview=self.preview(zero,False)
        if not zero_preview['in_limits'] or not zero_preview['collision_free']:
            return {'waypoints':[],'plan_complete':False,'coverage':{},'unreachable':[{'reason':'all-joint zero pose is outside safe endpoint geometry'}], 'hardware_execution_approved':False}
        waypoints.append({'arm':'both','cell':None,'kind':'zero_pose','capture':False,'transition':True,'jaw_gap_mm':[0.,0.],'q':zero.tolist()})
        current=zero.copy()
        for side,j in [('right',6)]:
            parked=current.copy()
            arm_path=[]
            # Keep the tag just off the optical center: this avoids the zero-motion
            # symmetry trap while leaving the full hand inside the central margin.
            target=np.array([self.k['width']/2+18,self.k['height']/2])
            seeds=[current[j:j+6].copy()]
            for joint in range(6):
                for delta in (-.18,.18):
                    seed=current[j:j+6].copy();seed[joint]=np.clip(seed[joint]+delta,self.limits[j+joint,0]+1e-7,self.limits[j+joint,1]-1e-7);seeds.append(seed)
            accepted=None
            for seed in seeds:
                def residual(x):
                    trial=current.copy();trial[j:j+6]=x;marker=self._marker(self.frames(trial),side)
                    uv=np.vstack((marker['corners_uv'],marker['hand_uv'])); margin=self.margin+12
                    overflow=np.maximum(margin-uv,0)+np.maximum(uv-[self.k['width']-margin,self.k['height']-margin],0)
                    return np.r_[(np.asarray(marker['center_uv'])-target)/24,overflow.ravel()/12,max(0,marker['normal_facing']+.03)*30,(x-seed)*.10]
                fit=least_squares(residual,seed,bounds=(self.limits[j:j+6,0],self.limits[j:j+6,1]),max_nfev=100)
                trial=current.copy();trial[j:j+6]=fit.x;marker=next(m for m in self.preview(trial,False)['markers'] if m['arm']==side)
                if marker['visible'] and np.linalg.norm(np.asarray(marker['center_uv'])-target)<35 and self.path_check(current,trial)['ok']:
                    accepted=trial;break
            if accepted is None:
                unreachable.append({'arm':side,'kind':'center','target_uv':target.tolist(),'reason':'no visible collision-free center pose found'})
                coverage[side]={'center_visible':False,'capture_points':0,'joint_pose_rank':0,'sufficient':False};continue
            current=accepted;center=current.copy();arm_path.append(current.copy())
            waypoints.append({'arm':side,'cell':[1,1],'kind':'center','q':current.tolist(),'target_uv':target.tolist(),'inset_scale':1.0})
            excitation=[]
            for joint in range(6):
                for delta in (-.22,.22):
                    trial=center.copy();trial[j+joint]=np.clip(trial[j+joint]+delta,self.limits[j+joint,0]+1e-7,self.limits[j+joint,1]-1e-7)
                    if self.path_check(current,trial,side)['ok']:
                        excitation.append((joint,delta,trial.copy()));current=trial;arm_path.append(current.copy())
                        waypoints.append({'arm':side,'cell':[1,1],'kind':'joint_diversity','joint':joint,'q':current.tolist()})
            if self.path_check(current,center)['ok']:
                waypoints.append({'arm':side,'cell':[1,1],'kind':'center_return','capture':True,'transition':True,'q':center.tolist()});current=center.copy()
            samples=np.asarray([center,*[x[2] for x in excitation]])
            rank=int(np.linalg.matrix_rank(samples[:,j:j+6]-center[j:j+6],tol=1e-4)) if len(samples)>1 else 0
            coverage[side]={'center_visible':True,'capture_points':1+len(excitation),'joint_pose_rank':rank,'sufficient':len(excitation)>=2 and rank>=2}
            # Return along the already checked path before the other arm moves.
            for target in [*reversed(arm_path[:-1]),parked]:
                if np.max(abs(current-target)) < 1e-9: continue
                if self.path_check(current,target)['ok']:
                    waypoints.append({'arm':side,'cell':None,'kind':'park_return','capture':True,'transition':True,'q':target.tolist()}); current=target.copy()
                else:
                    unreachable.append({'arm':side,'reason':'park return blocked; stop planning before next arm'})
                    coverage[side]['return_verified']=False
                    break
            coverage.setdefault(side,{}).setdefault('return_verified',True)
        # The left arm is a fixed mirror of the validated right-arm trajectory.
        right_waypoints=[w for w in waypoints if w.get('arm')=='right']
        left_current=current.copy()
        left_current[6:12]=self._q(q)[6:12]
        left_waypoints=[]
        for source in right_waypoints:
            mirrored=self.mirror_right_to_left(source['q'])
            endpoint_visible = 24 in self.preview(mirrored,False)['visible_ids']
            if not endpoint_visible or not self.path_check(left_current,mirrored)['ok']:
                unreachable.append({'arm':'left','source_kind':source.get('kind'),'reason':'mirrored right-arm waypoint fails left visibility/collision check'})
                coverage['left']={'center_visible':False,'capture_points':0,'joint_pose_rank':0,'sufficient':False,'return_verified':False}
                break
            item={**source,'arm':'left','q':mirrored.tolist(),'mirror_of':'right','capture':source.get('capture',True)}
            left_waypoints.append(item); left_current=mirrored
        if len(left_waypoints)==len(right_waypoints):
            waypoints.extend(left_waypoints)
            captures=[np.asarray(w['q'])[0:6] for w in left_waypoints if w.get('capture',True)]
            center=next((np.asarray(w['q']) for w in left_waypoints if w.get('kind')=='center'),None)
            rank=int(np.linalg.matrix_rank(np.asarray(captures)[:,0:6]-center[0:6],tol=1e-4)) if center is not None and len(captures)>1 else 0
            coverage['left']={'center_visible':bool(center is not None),'capture_points':len(captures),'joint_pose_rank':rank,'sufficient':bool(center is not None and rank>=2),'return_verified':True}
        return {'waypoints':waypoints,'unreachable':unreachable,'coverage':coverage,'plan_complete':all(v['sufficient'] for v in coverage.values()),'collision_method':self.collision_method,'hardware_execution_approved':False,'visibility_scope':'mesh framing/front-facing only; marker detector is optional evidence'}

    def _plan_from_seed(self, start, seed):
        """Apply the frozen right-arm trajectory, then its deterministic mirror."""
        rows = seed.get('right_waypoints')
        if not isinstance(rows, list) or not rows:
            raise ValueError('standard head motion seed has no right waypoints')
        if seed.get('mirror_mapping') != [-1, 1, 1, 1, -1, -1]:
            raise ValueError('standard head motion seed uses an unsupported mirror mapping')
        zero = np.zeros(12)
        def check_limits(q):
            if not (np.all(q>=self.limits[:,0]) and np.all(q<=self.limits[:,1])):
                raise ValueError('standard head motion seed exceeds joint limits')
        check_limits(zero)
        zero_waypoint = {'arm':'both','cell':None,'kind':'zero_pose','capture':False,'transition':True,'jaw_gap_mm':[0.,0.],'q':zero.tolist()}
        parked = zero.copy()
        right = []
        for source in rows:
            q = np.asarray(source.get('q'), float)
            if q.shape != (12,) or not np.isfinite(q).all():
                raise ValueError('standard head motion seed contains an invalid waypoint')
            target = zero.copy(); target[6:12] = q[6:12]
            check_limits(target)
            right.append({**source, 'arm': 'right', 'q': target.tolist()})
            parked = target
        # Freeze two pose groups before capture; repeated returns share the split.
        center = next((w for w in right if w.get('kind') == 'center'), None)
        wrist = next((w for w in reversed(right) if w.get('kind') == 'joint_diversity' and w.get('joint') == 5), None)
        if center is None or wrist is None:
            raise ValueError('standard head motion seed requires center and wrist holdout poses')
        heldout = [np.asarray(w['q'])[6:12] for w in (center, wrist)]
        for waypoint in right:
            q = np.asarray(waypoint['q'])[6:12]
            waypoint['split'] = 'holdout' if any(np.max(abs(q-pose)) <= 1e-4 for pose in heldout) else 'fit'
        # The seed includes the validated return, so the left pass starts with
        # the same parked right arm and never moves both arms accidentally.
        left = []
        left_holdout = seed.get('left_holdout_indices')
        if left_holdout is not None and (not isinstance(left_holdout, list) or len(left_holdout) < 2
                or any(type(i) is not int or i < 0 or i >= len(right) or not right[i].get('capture', True)
                       for i in left_holdout) or len(set(left_holdout)) != len(left_holdout)):
            raise ValueError('invalid left holdout indices in head motion seed')
        left_current = zero.copy()
        for index, source in enumerate(right):
            mirrored = self.mirror_right_to_left(np.asarray(source['q']))
            target = zero.copy(); target[:6] = mirrored[:6]
            check_limits(target)
            left.append({**source, 'arm': 'left', 'q': target.tolist(), 'mirror_of': 'right',
                         'split': ('holdout' if index in left_holdout else 'fit') if left_holdout is not None else source['split']})
            left_current = target
        def coverage(rows, arm):
            sl = slice(0, 6) if arm == 'left' else slice(6, 12)
            captures = [np.asarray(w['q'])[sl] for w in rows if w.get('capture', True)]
            center = next((np.asarray(w['q'])[sl] for w in rows if w.get('kind') == 'center'), None)
            rank = int(np.linalg.matrix_rank(np.asarray(captures) - center, tol=1e-4)) if center is not None and len(captures) > 1 else 0
            return {'center_visible': center is not None, 'capture_points': len(captures), 'joint_pose_rank': rank, 'sufficient': bool(center is not None and rank >= 2), 'return_verified': True}
        waypoints = [zero_waypoint] + right + left
        coverage_result = {'right': coverage(right, 'right'), 'left': coverage(left, 'left')}
        return {'waypoints': waypoints, 'unreachable': [], 'coverage': coverage_result,
                'plan_complete': True, 'collision_method': 'not checked: frozen calibration seed',
                'collision_checked': False, 'hardware_execution_approved': False,
                'visibility_scope': 'frozen seed; no per-execution planner or marker-visibility gate', 'motion_seed': seed.get('id', 'standard-head-motion-seed')}
