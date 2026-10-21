"""Frozen inscribed A1 self spheres; neither startup nor requests fit geometry."""
import hashlib
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET
import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components

POLICY = dict(scope='robot_self_spheres', jaws_mm=[83., 83.], maximum_joint_step_rad=.04,
              support_radius_scale=.5, support_contact_band_m=.003,
              front_support_x_min_m=-.1, surface_inset_m=1e-7)
DEFAULT_PROXY = Path(__file__).resolve().parents[1] / 'configs/a1-manual-self-spheres-v1.json'


def proxy_digest(data):
    return hashlib.sha256(json.dumps({k:v for k,v in data.items() if k != 'proxy_id'},
        sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def components(vertices, faces):
    """CAD component union avoids merged ray parity treating overlap as a void."""
    points, indices = np.asarray(vertices, dtype=float), np.asarray(faces)
    if (points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all()
            or indices.ndim != 2 or indices.shape[1] != 3 or indices.dtype.kind not in 'iu'
            or not len(indices) or indices.min() < 0 or indices.max() >= len(points)):
        raise ValueError('finite triangle vertices and valid faces required')
    unique, inverse = np.unique(points, axis=0, return_inverse=True)
    merged = inverse[indices]
    edges = np.concatenate((merged[:, [0, 1]], merged[:, [1, 2]], merged[:, [2, 0]]))
    graph = coo_matrix((np.ones(len(edges)), (edges[:, 0], edges[:, 1])), shape=(len(unique), len(unique)))
    _, labels = connected_components(graph, directed=False)
    for label in np.unique(labels[merged[:, 0]]):
        ids = np.flatnonzero(labels[merged[:, 0]] == label)
        yield points[indices[ids]].reshape(-1, 3), ids


def source_fingerprint(geometry):
    """Exact authored geometry, nominal mapping and instantiated mesh layout."""
    path = Path(geometry.config['urdf']).resolve(); meshes = {}
    for node in ET.fromstring(path.read_bytes()).iter('mesh'):
        name = node.get('filename'); relative = Path(name)
        if relative.is_absolute() or '..' in relative.parts or '://' in name:
            raise ValueError('nominal mesh must remain in its authored asset root')
        file = (path.parent / relative).resolve()
        if not file.is_relative_to(path.parent): raise ValueError('nominal mesh escapes its asset root')
        meshes[name] = hashlib.sha256(file.read_bytes()).hexdigest()
    def mesh_hash(rows):
        digest = hashlib.sha256()
        for name, vertices, faces in rows:
            digest.update(name.encode())
            # Exact raw hashes bind assets; picometre rounding only makes the
            # derived-array check portable across floating-point implementations.
            digest.update((np.round(np.asarray(vertices, float), 12)+0.).astype('<f8').tobytes())
            digest.update(np.asarray(faces, dtype='<i8').tobytes())
        return digest.hexdigest()
    return dict(urdf_sha256=hashlib.sha256(path.read_bytes()).hexdigest(), meshes_sha256=meshes,
        jaw_mapping_sha256=(hashlib.sha256(Path(geometry.config['jaw_mapping_file']).read_bytes()).hexdigest()
                            if geometry.config.get('jaw_mapping_file') else None),
        active_joints=list(geometry.model.names), aliases={s:dict(v) for s,v in geometry.aliases.items()},
        adjacent=sorted(sorted(pair) for pair in geometry.adjacent),
        static_adjacent=sorted(list(pair) for pair in geometry.static_adjacent),
        local_meshes={side: mesh_hash(rows) for side, rows in geometry.full_side_meshes.items()},
        static_robot_meshes=mesh_hash(geometry.static_meshes), derived_geometry_rounding_m=1e-12)


class ManualSelfCollision:
    def __init__(self, geometry, *, prebuilt=None):
        began = time.perf_counter()
        if geometry.config.get('model_family') != 'ac_one': raise ValueError('nominal A1 geometry required')
        data = json.loads(DEFAULT_PROXY.read_text()) if prebuilt is None else prebuilt
        if (data.get('proxy_id') != proxy_digest(data) or data.get('schema') != 'manual_inscribed_spheres.v1' or data.get('policy') != POLICY
                or data.get('source_fingerprint') != source_fingerprint(geometry)):
            raise ValueError('manual sphere asset/model/policy differs; explicitly rebuild and qualify the offline proxy')
        self.geometry = geometry.with_gaps(83., 83.); self.parts = data['parts']
        self.centers = np.asarray(data['centers'], dtype=float)
        self.radii = np.asarray(data['radii'], dtype=float)
        self.original_radii = np.asarray(data['original_radii'], dtype=float)
        indices, local = np.asarray(data['sphere_parts']), np.asarray(data['support_local'])
        count = len(self.radii)
        if (not self.parts or not 0 < count <= 100000 or self.centers.shape != (count, 3)
                or self.radii.shape != (count,) or self.original_radii.shape != (count,)
                or not np.isfinite(self.centers).all() or not np.isfinite(self.radii).all()
                or not np.isfinite(self.original_radii).all() or np.any(self.radii <= 0)
                or np.any(self.original_radii < self.radii) or indices.shape != (count,)
                or indices.dtype.kind not in 'iu' or indices.min() < 0 or indices.max() >= len(self.parts)
                or local.shape != (count,) or local.dtype.kind != 'b'):
            raise ValueError('invalid finite positive sphere proxy arrays')
        self.sphere_parts, self.support_local = indices.astype(np.intp), local
        self.groups = [np.flatnonzero(self.sphere_parts == i) for i in range(len(self.parts))]
        fixed = {geometry.model.root}
        for _, parent, child, _, _, kind in geometry.model.joints:
            if kind == 'fixed' and parent in fixed: fixed.add(child)
        covered = set()
        for i, part in enumerate(self.parts):
            side, name = part['side'], part['name']
            valid = (('base_link' if name == 'chassis' else name) in fixed if side is None else
                     side in geometry.aliases and name in geometry.aliases[side].values())
            if not valid or not len(self.groups[i]) or part['sphere_count'] != len(self.groups[i]):
                raise ValueError('sphere part outside robot scope or incorrect membership')
            covered.add((side, name))
        required = {(side, name) for side, rows in geometry.full_side_meshes.items() for name, *_ in rows}
        required.update((None, name) for name, *_ in geometry.static_meshes
                        if ('base_link' if name == 'chassis' else name) in fixed)
        if not required <= covered: raise ValueError('sphere proxy omits a robot link or fixed body')
        self.enabled = np.zeros((len(self.parts), len(self.parts)), dtype=bool)
        self.front_pairs = np.zeros_like(self.enabled)
        for i, a in enumerate(self.parts):
            for j in range(i+1, len(self.parts)):
                b = self.parts[j]
                if a['side'] is None and b['side'] is None: continue
                if a['side'] == b['side']:
                    front = {a['name'], b['name']} == {'link2', 'link3'}
                    self.front_pairs[i, j] = self.front_pairs[j, i] = front
                    if a['name'] == b['name'] or (not front and frozenset((a['name'], b['name'])) in geometry.adjacent): continue
                moving, static = (a, b) if b['side'] is None else (b, a)
                if static['side'] is None and (moving['name'], static['name']) in geometry.static_adjacent: continue
                self.enabled[i, j] = self.enabled[j, i] = True
        self.part_sides = np.asarray([p['side'] or '' for p in self.parts]); self.front_roi = POLICY['front_support_x_min_m']
        self.description = dict(backend='collection_inscribed_spheres_cpu', scope=POLICY['scope'], jaws_mm=POLICY['jaws_mm'],
            environment_checked=False, exact_support_depth_checked=False, support_policy='coarse_local_inscribed_proxy',
            support_pair_exemptions=False, sphere_count=count, part_count=len(self.parts),
            support_local_sphere_count=int(self.support_local.sum()), proxy_id=data['proxy_id'],
            cached_proxy_loaded=True, preparation_seconds=time.perf_counter()-began)

    def contact_spheres(self, q):
        frames = self.geometry.frames(q)
        transforms = np.asarray([np.eye(4) if p['side'] is None else frames[p['side']][p['name']]
                                 for p in self.parts])[self.sphere_parts]
        centers = np.einsum('nij,nj->ni', transforms[:, :3, :3], self.centers)+transforms[:, :3, 3]
        lower = np.full((len(self.parts), 3), np.inf); upper = -lower
        np.minimum.at(lower, self.sphere_parts, centers-self.radii[:, None])
        np.maximum.at(upper, self.sphere_parts, centers+self.radii[:, None])
        candidates = np.argwhere(np.triu(self.enabled, 1)
            & np.all(upper[:, None] >= lower[None]-1e-9, axis=2)
            & np.all(upper[None] >= lower[:, None]-1e-9, axis=2))
        found = []
        for i, j in candidates:
            a, b = self.groups[i], self.groups[j]
            delta = centers[a, None]-centers[b][None]
            hits = np.argwhere(np.einsum('ijk,ijk->ij', delta, delta)
                               <= (self.radii[a, None]+self.radii[b][None]+1e-9)**2)
            if len(hits): found.append(np.column_stack((a[hits[:, 0]], b[hits[:, 1]])))
        if not found: return np.empty((0, 2), dtype=np.intp)
        pairs = np.concatenate(found); a, b = pairs.T
        front = self.front_pairs[self.sphere_parts[a], self.sphere_parts[b]]; eligible = np.ones(len(a), dtype=bool)
        for side in ('left', 'right'):
            ids = np.flatnonzero(front & (self.part_sides[self.sphere_parts[a]] == side))
            if not len(ids): continue
            frame = frames[side]['link2']; xa = (centers[a[ids]]-frame[:3, 3]) @ frame[:3, 0]
            xb = (centers[b[ids]]-frame[:3, 3]) @ frame[:3, 0]
            eligible[ids] = (xa+self.radii[a[ids]] >= self.front_roi) & (xb+self.radii[b[ids]] >= self.front_roi)
        return pairs[eligible]

    def collisions(self, q):
        pairs = self.contact_spheres(q); part_pairs = np.unique(np.sort(self.sphere_parts[pairs], axis=1), axis=0)
        def name(index):
            part = self.parts[index]; return (part['side'] or 'robot')+'/'+part['name']
        return [list(pair) for pair in dict.fromkeys((name(i), name(j)) for i, j in part_pairs)]

    def check_path(self, start, target):
        began = time.perf_counter(); start, target = self.geometry._q(start), self.geometry._q(target)
        count = max(2, int(np.ceil(np.max(abs(target-start))/POLICY['maximum_joint_step_rad']))+1)
        result = dict(self.description, ok=True, samples=count, certified_continuous_collision=False)
        for index, fraction in enumerate(np.linspace(0., 1., count)):
            q = np.clip(start+fraction*(target-start), np.minimum(start, target), np.maximum(start, target))
            if np.any(q < self.geometry.limits[:, 0]) or np.any(q > self.geometry.limits[:, 1]):
                result.update(ok=False, fraction=float(fraction), reason='joint limits', collisions=[]); break
            pairs = self.collisions(q)
            if pairs:
                result.update(ok=False, fraction=float(fraction), reason='coarse self sphere overlap', collisions=pairs); break
        result.update(checked_samples=index+1, wall_seconds=time.perf_counter()-began)
        return result
