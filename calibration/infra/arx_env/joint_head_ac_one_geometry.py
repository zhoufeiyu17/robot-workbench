"""Native AC-One tree behind the workbench's side/link aliases."""
import copy
from functools import lru_cache
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation

from .joint_head_isaac_fit import Model, rigid
from .joint_head_jaw_mapping import measured_jaw_gaps
from .joint_head_workbench_geometry import (
    Geometry, _read, _points, stl_vertices, closed_component_representatives, jaw_gaps,
    meshes_intersect, surface_intersection_pairs,
)
from .urdf_mesh import origin


class ACOneGeometry(Geometry):
    def __init__(self, config):
        self.config = config
        root = Path(config['urdf']).parent
        tree = ET.parse(config['urdf']).getroot()
        names = [*(f'left_joint{i}' for i in range(1, 7)),
                 *(f'right_joint{i}' for i in range(11, 17))]
        self.model = Model(config['urdf'], names)
        joints = {j.get('name'): j for j in tree.findall('joint')}
        limits = [joints[name].find('limit') for name in names]
        self.limits = np.array([[float(j.get('lower')), float(j.get('upper'))] for j in limits])
        self.velocity_limits = np.array([float(j.get('velocity')) for j in limits])
        self.bases = {side: np.eye(4) for side in ('left', 'right')}
        candidate = _read(config['candidate'])
        self.camera = rigid(candidate['T_world_optical'])
        self.mounts = {side: rigid(candidate['T_link6_marker'][str(marker)])
                       for side, marker in (('left', 24), ('right', 30))}
        self.k = _read(config['camera'])
        self.margin = float(config.get('margin_px', 18))
        self.marker_size = float(config.get('marker_size_m', .015))
        self.gaps = dict(config.get('gap_m', {'left': .083, 'right': .083}))
        jaw_gaps([self.gaps[s]*1000 for s in ('left', 'right')])
        self.hand = None  # Independent URDF fingers; never deform a merged X5 mesh.
        self.mount_status = 'AC-One native wrist housing; marker mounts require calibration'
        self.mount_chain = {}
        self.aliases = {}
        for side, offset in (('left', 0), ('right', 10)):
            self.aliases[side] = {f'{side}_link{i+offset}': f'link{i}' for i in range(1, 7)}
            self.aliases[side].update({f'{side}_link{7+offset}': 'finger_positive',
                                       f'{side}_link{8+offset}': 'finger_negative',
                                       f'{side}_wrist_camera_link': 'camera_body',
                                       f'{side}_wrist_camera_rgb_optical_frame': 'camera_optical'})
        self.side_meshes = {side: [] for side in self.aliases}
        self.full_side_meshes = {side: [] for side in self.aliases}
        self.bounds, self.hulls, self.containment_representatives = {}, {}, {}
        self.static_meshes, self.static_hulls, self.static_triangles, self.static_labels = [], [], [], []
        zero = self.model.frames(np.zeros(12))
        for link in tree.findall('link'):
            native = link.get('name')
            vertices = []
            for visual in link.findall('visual'):
                mesh = visual.find('geometry/mesh')
                if mesh is None:
                    raise ValueError('AC-One visuals must use sealed mesh assets')
                path = Path(mesh.get('filename'))
                if path.is_absolute() or '..' in path.parts:
                    raise ValueError('AC-One mesh must be relative to its asset root')
                v = stl_vertices(root/path)*np.fromstring(mesh.get('scale', '1 1 1'), sep=' ')
                vertices.append(_points(v, origin(visual.find('origin'))))
            if not vertices:
                continue
            v = np.concatenate(vertices)
            faces = np.arange(len(v)).reshape(-1, 3)
            side = next((s for s in self.aliases if native in self.aliases[s]), None)
            if side is None:
                label = 'chassis' if native == 'base_link' else native
                self.static_meshes.append((label, _points(v, zero[native]), faces))
                continue
            name = self.aliases[side][native]
            self.full_side_meshes[side].append((name, v, faces))
            self.side_meshes[side].append((name, v, faces[::max(1, int(np.ceil(len(faces)/500)))]))
            lo, hi = v.min(0), v.max(0)
            self.bounds[side, name] = ((lo+hi)/2, (hi-lo)/2)
            self.hulls[side, name] = ConvexHull(np.unique(v, axis=0)).equations
            self.containment_representatives[side, name] = closed_component_representatives(v)
        self._build_static_hulls()
        self.adjacent = set()
        self.static_adjacent = set()
        for _, parent, child, _, _, _ in self.model.joints:
            for side, aliases in self.aliases.items():
                if parent in aliases and child in aliases:
                    self.adjacent.add(frozenset((aliases[parent], aliases[child])))
                elif parent == 'base_link' and child in aliases:
                    self.static_adjacent.add((aliases[child], 'chassis'))
        self.collision_method = 'native AC-One meshes; adjacent URDF links excluded; full convex hulls and static triangle checks; sampled paths only'
        self.parking_support = config.get('parking_support_contact')
        self.support_meshes, self.support_faces, self.support_chains = {}, {}, {}
        if self.parking_support:
            from .joint_head_support_contact import PreparedMesh
            policy = self.parking_support
            if policy.get('asset_geometry_id') != config.get('asset_geometry_id'):
                raise ValueError('support policy must match the reviewed A1 geometry')
            if (policy.get('max_interference_m') != .0012
                    or policy.get('separation_direction_link2') != [0., 0., 1.]
                    or policy.get('front_support_x_min_m') != -.1):
                raise ValueError('unreviewed A1 support interference policy')
            for side, offset in (('left', 0), ('right', 10)):
                meshes = {n: PreparedMesh(v.reshape(-1, 3, 3), require_closed=(n != 'link5'))
                          for n, v, _ in self.full_side_meshes[side] if n in ('link2', 'link3', 'link5')}
                self.support_meshes[side, 'link3'] = (meshes['link2'], meshes['link3'])
                selected = []
                for name in ('link2', 'link5'):
                    seed = policy['claw_support_component_seed_faces'][name]
                    component = next((ids for ids in meshes[name].components if seed in ids), None)
                    if component is None: raise ValueError('support component seed missing')
                    self.support_faces[side, name] = set(map(int, component))
                    selected.append(PreparedMesh(meshes[name].triangles[component]))
                self.support_meshes[side, 'link5'] = tuple(selected)
                for link, last in (('link3', 3), ('link5', 5)):
                    names = [f'{side}_joint{i+offset}' for i in range(3, last+1)]
                    self.support_chains[side, link] = [next((T, axis) for n, _, _, T, axis, _ in self.model.joints if n == name) for name in names]
            self.collision_method += '; both support regions limited to 1.2 mm separation along link2 +Z, including return home'

    def _support_transform(self, side, link, angles):
        T = np.eye(4)
        for angle, (origin_T, axis) in zip(angles, self.support_chains[side, link]):
            delta = np.eye(4); delta[:3, :3] = Rotation.from_rotvec(axis * angle).as_matrix()
            T = T @ origin_T @ delta
        return T

    @lru_cache(maxsize=8192)
    def _support_pair(self, side, link, angles):
        from .joint_head_support_contact import region_overlap
        a, b = self.support_meshes[side, link]
        T = self._support_transform(side, link, angles)
        roi = self.parking_support['front_support_x_min_m'] if link == 'link3' else None
        moved = _points(b.triangles.reshape(-1, 3), T).reshape(-1, 3, 3)
        contact = region_overlap(a, b, b_triangles=moved, roi_x_min=roi)
        limit = self.parking_support['max_interference_m']
        # Directional separation is a conservative depth measure, not an AABB overlap.
        over_limit = contact and region_overlap(a, b, b_triangles=moved + [0., 0., limit + 1e-9], roi_x_min=roi)
        return dict(pair=[side+'/link2', side+'/'+link], contact=bool(contact),
                    ok=not over_limit, limit_m=limit, metric='separation_along_link2_positive_z')

    def support_contacts(self, q):
        q = self._q(q)
        if not self.parking_support: return []
        return [self._support_pair(side, link, tuple(q[offset+2:offset+last]))
                for side, offset in (('left', 0), ('right', 6))
                for link, last in (('link3', 3), ('link5', 5))]

    @lru_cache(maxsize=2048)
    def _support_segment(self, side, link, start, target):
        start, target = np.asarray(start), np.asarray(target)
        from .joint_head_support_contact import region_separated_with_margin
        a, b = self.support_meshes[side, link]
        roi = self.parking_support['front_support_x_min_m'] if link == 'link3' else None
        fixed = a.triangles if roi is None else a.triangles[a.triangles[:, :, 0].max(1) >= roi]
        alo, ahi = fixed.min((0, 1)), fixed.max((0, 1))
        # Sum of link translations plus mesh radius bounds every downstream
        # vertex's distance from each relevant rotation axis for any pose.
        reach = sum(np.linalg.norm(T[:3, 3]) for T, _ in self.support_chains[side, link]) + np.linalg.norm(b.triangles, axis=2).max()
        samples, contacts, cleared = 0, 0, 0
        stack = [(0., 1.)]
        while stack:
            lo, hi = stack.pop(); mid = (lo+hi)/2
            angles = start + (target-start)*mid
            radius = reach * np.abs(target-start).sum() * (hi-lo)/2
            T = self._support_transform(side, link, angles)
            full_moved = _points(b.triangles.reshape(-1, 3), T).reshape(-1, 3, 3)
            moved = full_moved
            if roi is not None:
                moved = moved[moved[:, :, 0].max(1) + radius >= roi]
            # This swept bound can only certify separation, never tolerate an
            # intersection. It avoids sampling long spans far from the support.
            if not len(moved) or np.any(moved.max((0, 1))+radius < alo) or np.any(moved.min((0, 1))-radius > ahi):
                cleared += 1; continue
            if region_separated_with_margin(a, b, b_triangles=full_moved + [0., 0., .0012 + 1e-9], roi_x_min=roi, margin_m=radius):
                cleared += 1; continue
            for fraction in (lo, mid, hi):
                row = self._support_pair(side, link, tuple(start+(target-start)*fraction))
                samples += 1; contacts += int(row['contact'])
                if not row['ok']:
                    return {'ok': False, 'fraction': fraction, 'details': [row], 'collisions': [row['pair']]}
            if radius > .00005:
                stack.extend(((mid, hi), (lo, mid)))
        return {'ok': True, 'samples': samples, 'contact_samples': contacts, 'separated_intervals': cleared}

    def support_path_check(self, start, target):
        start, target = self._q(start), self._q(target)
        if not self.parking_support:
            return {'ok': False, 'reason': 'A1 support policy missing'}
        checks = []
        for side, offset in (('left', 0), ('right', 6)):
            for link, last in (('link3', 3), ('link5', 5)):
                check = self._support_segment(side, link, tuple(start[offset+2:offset+last]), tuple(target[offset+2:offset+last]))
                if not check['ok']: return check
                checks.append(check)
        return {'ok': True, 'samples': sum(c['samples'] for c in checks),
                'contact_samples': sum(c['contact_samples'] for c in checks),
                'separated_intervals': sum(c['separated_intervals'] for c in checks),
                'max_support_vertex_step_m': .0001, 'limit_m': .0012,
                'certified_continuous_collision': False}

    def path_check(self, start, target, side=None):
        support = self.support_path_check(start, target) if self.parking_support else {'ok': True}
        if not support['ok']: return support
        return {**super().path_check(start, target, side), 'support_check': support}

    def collisions(self, frames):
        collisions = super().collisions(frames)
        result = []
        for pair in collisions:
            side = pair[0].split('/')[0]
            if pair[:2] != [side+'/link2', side+'/link5']:
                result.append(pair)
                continue
            triangles = {name: _points(vertices, frames[side][name]).reshape(-1, 3, 3)
                         for name, vertices, _ in self.full_side_meshes[side] if name in ('link2', 'link5')}
            a, b = triangles['link2'], triangles['link5']
            if not meshes_intersect(a, b, self.containment_representatives[side, 'link2']): continue
            intersections = list(surface_intersection_pairs(a, b))
            if not self.parking_support or not intersections or any(
                    i not in self.support_faces[side, 'link2'] or j not in self.support_faces[side, 'link5']
                    for i, j in intersections):
                result.append(pair)
        if self.parking_support:
            from .joint_head_support_contact import region_overlap
            for (side, link), (a, b) in self.support_meshes.items():
                T = np.linalg.inv(frames[side]['link2']) @ frames[side][link]
                moved = _points(b.triangles.reshape(-1, 3), T).reshape(-1, 3, 3)
                roi = self.parking_support['front_support_x_min_m'] if link == 'link3' else None
                if region_overlap(a, b, b_triangles=moved, roi_x_min=roi) and region_overlap(
                        a, b, b_triangles=moved + [0., 0., .0012 + 1e-9], roi_x_min=roi):
                    pair = [side+'/link2', side+'/'+link]
                    if pair not in result: result.append(pair)
        return result

    @lru_cache(maxsize=8)
    def with_gaps(self, left_mm, right_mm):
        gaps = jaw_gaps([left_mm, right_mm])
        view = copy.copy(self)
        # Support links do not depend on finger opening; share verified path cache.
        view._support_segment = self._support_segment
        view._support_pair = self._support_pair
        view.gaps = dict(zip(('left', 'right'), gaps/1000))
        return view

    def with_feedback_gaps(self, left_mm, right_mm):
        gaps = measured_jaw_gaps([left_mm, right_mm])
        view = copy.copy(self)
        # Support links do not depend on finger opening; share verified path cache.
        view._support_segment = self._support_segment
        view._support_pair = self._support_pair
        view.gaps = dict(zip(('left', 'right'), gaps/1000))
        return view

    def frames(self, q):
        q = self._q(q)
        fingers = {}
        for side, offset in (('left', 0), ('right', 10)):
            # Native CAD has an 8 micrometre metal gap at prismatic zero.
            opening = (self.gaps[side] - .000008)/2
            for i in (7, 8):
                fingers[f'{side}_joint{i+offset}'] = opening
        native = self.model.frames(q, joint_values=fingers)
        return {side: {alias: native[name] for name, alias in aliases.items()}
                for side, aliases in self.aliases.items()}

    def _marker(self, frames, side):
        marker = super()._marker(frames, side)
        points = []
        for name in ('link6', 'finger_positive', 'finger_negative'):
            center, half = self.bounds[side, name]
            corners = center + np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)])*half
            points.extend(_points(corners, frames[side][name]))
        uv, depth = self.project(np.asarray(points))
        framed = bool(marker['framed'] and np.all(depth > .03) and
                      np.all(uv > self.margin) and
                      np.all(uv < [self.k['width']-self.margin, self.k['height']-self.margin]))
        marker.update(hand_uv=uv.tolist(), framed=framed, visible=framed and marker['front_facing'])
        return marker
