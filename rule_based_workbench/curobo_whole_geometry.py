"""Candidate broad phase for native whole-robot audit; keep its exact narrow phase.

The optional callback receives conservative float32 spheres [parts, 4] in world
coordinates and a scalar clearance, returning a NumPy bool mask [parts, world].
True means possible contact, never a collision verdict. World order must match
native.world. Without the callback, vectorized float64 AABBs are the reference.
"""
import math


def near_aabbs(np, centers_a, halves_a, centers_b, halves_b, clearance):
    """Broadcastable AABB distance filter; rounding guard can only add candidates."""
    gaps = np.maximum(np.abs(centers_a-centers_b)-halves_a-halves_b, 0.)
    guard = 32*np.finfo(np.float64).eps*(
        1.+np.max(np.abs(centers_a)+np.abs(centers_b)+halves_a+halves_b, axis=-1))
    return np.einsum('...i,...i->...', gaps, gaps) <= (clearance+guard)**2


def enclosing_spheres(np, centers, local_halves):
    """Enclose every rotated local AABB, including center/radius float32 rounding."""
    radii = np.linalg.norm(local_halves, axis=1)
    padding = 16*np.finfo(np.float32).eps*(1.+np.abs(centers).sum(axis=1)+radii)
    return np.column_stack((centers, radii+padding)).astype(np.float32)


def support_separated(np, bounds_a, bounds_b, a_from_b):
    """Enclosing visual bounds in link A; preserve the native 1 nm contact tolerance."""
    low_a, high_a = bounds_a
    low_b, high_b = bounds_b
    center = a_from_b[:3, :3] @ ((low_b+high_b)/2)+a_from_b[:3, 3]
    half = np.abs(a_from_b[:3, :3]) @ ((high_b-low_b)/2)
    guard = 1e-9+32*np.finfo(np.float64).eps*(1.+np.max(np.abs(center)+half+np.abs(low_a)+np.abs(high_a)))
    return bool(np.any(high_a < center-half-guard) or np.any(center+half < low_a-guard))


def front_support_bounds(np, triangles):
    """Native front ROI retains whole crossing triangles, never a clipped/open mesh."""
    selected = triangles[triangles[:, :, 0].max(1) >= -.1]
    return (selected.min((0, 1)), selected.max((0, 1))) if len(selected) else None


def install_vectorized_bounds(mesh, np):
    """Cache immutable local bounds; batch every supplied transformed triangle array."""
    from types import MethodType
    if getattr(getattr(mesh.bounds, '__func__', mesh.bounds), '_vectorized_bounds', False):
        return
    leaves, heights = [], []
    for index, (left, right, triangles) in enumerate(mesh.nodes):
        heights.append(0 if triangles is not None else 1+max(heights[left], heights[right]))
        if triangles is not None:
            if not len(triangles):
                raise ValueError('empty BVH leaf')
            leaves.append(index)
    leaf_ids = np.asarray(leaves, dtype=np.intp)
    width = max(len(mesh.nodes[index][2]) for index in leaves)
    leaf_table = np.asarray([np.pad(mesh.nodes[index][2], (0, width-len(mesh.nodes[index][2])), mode='edge')
                             for index in leaves], dtype=np.intp)
    levels = []
    for height in range(1, max(heights)+1):
        parents = np.flatnonzero(np.asarray(heights) == height)
        children = np.asarray([mesh.nodes[index][:2] for index in parents], dtype=np.intp)
        levels.append((parents, children[:, 0], children[:, 1]))
    node_count = len(mesh.nodes)
    static_triangles, static_bounds = mesh.triangles, None

    def bounds(self, triangles):
        if triangles is static_triangles and static_bounds is not None:
            return static_bounds
        low, high = np.empty((node_count, 3)), np.empty((node_count, 3))
        leaf_points = triangles[leaf_table]
        low[leaf_ids], high[leaf_ids] = leaf_points.min((1, 2)), leaf_points.max((1, 2))
        for parents, left, right in levels:
            low[parents], high[parents] = np.minimum(low[left], low[right]), np.maximum(high[left], high[right])
        return low, high

    static_bounds = bounds(mesh, static_triangles)
    # PreparedMesh topology/local coordinates are fixed after construction.
    # Enforce that assumption for this instance; transformed inputs never cache.
    static_triangles.setflags(write=False)
    for values in static_bounds:
        values.setflags(write=False)
    bounds._vectorized_bounds = True
    mesh.bounds = MethodType(bounds, mesh)


class CuroboWholeGeometry:
    """Explicit opt-in wrapper; robot topology and native exclusions stay fixed."""

    def __init__(self, native, spheres_to_world_mask=None):
        self.native, self.spheres_to_world_mask = native, spheres_to_world_mask
        np = native.np
        self._links = tuple(part[0] for part in native.parts)
        self._unique_links = tuple(dict.fromkeys(self._links))
        self._link_indices = np.asarray([self._unique_links.index(link) for link in self._links])
        self._local_centers = np.asarray([part[3] for part in native.parts], dtype=float).reshape(-1, 3)
        self._local_halves = np.asarray([part[4] for part in native.parts], dtype=float).reshape(-1, 3)
        if (not len(native.parts) or not np.isfinite(self._local_centers).all()
                or not np.isfinite(self._local_halves).all() or np.any(self._local_halves < 0)):
            raise ValueError('finite nonempty native robot AABBs required')
        self._support_bounds = ({name: (mesh.triangles.min((0, 1)), mesh.triangles.max((0, 1)))
                                 for name, mesh in native.support_contact.meshes.items()}
                                if native.support_contact else {})
        self._support_front_bounds = ({a: front_support_bounds(np, native.support_contact.meshes[a].triangles)
                                       for a, b in native.support_contact.pairs if not b.endswith('5')}
                                      if native.support_contact else {})
        if native.support_contact:
            for mesh in (*native.support_contact.meshes.values(), *native.support_contact.selected.values()):
                install_vectorized_bounds(mesh, np)

    def __getattr__(self, name):
        return getattr(self.native, name)

    def __setattr__(self, name, value):
        # Scene pose updates in EpisodePlanner.geometry must reach the native cache.
        if name.startswith('_') or name in ('native', 'spheres_to_world_mask'):
            object.__setattr__(self, name, value)
        else:
            setattr(self.native, name, value)

    def support_checks(self, frames):
        if self.native.support_contact is None:
            return []
        from robot_sim.tasks.manager_based.coffee_make.a1_support_contact import _matrix
        support, np, checks = self.native.support_contact, self.np, []
        for a, b in support.pairs:
            relative = np.linalg.inv(_matrix(frames[a])) @ _matrix(frames[b])
            # Only link2/link3 has a front ROI without outside_support checks.
            # B's complete transformed box encloses its ROI too, so separation
            # implies the same region_overlap AABB rejection before mesh work.
            bounds_a = self._support_bounds[a] if b.endswith('5') else self._support_front_bounds[a]
            if bounds_a is None or support_separated(np, bounds_a, self._support_bounds[b], relative):
                # Geometric separation avoids the old ray-parity false positives
                # outside full mesh bounds; this is not a pair-wide exemption.
                checks.append(dict(pair=(a, b), ok=True, contact=False, over_limit=False,
                                   outside_support=False, limit_m=.0012, metric='separation_along_link2_positive_z'))
            else:
                checks.append(support._pair(a, b, tuple(relative.ravel())))
        return checks

    def audit(self, frames, *, allowed_pairs=(), margin=.0005, world_margin_paths=(), world_margin=None,
              _consume_parts=None):
        """Optionally consume this call's part data inline; nothing persists across frames."""
        whole, np, fcl = self.native, self.np, self.fcl
        if not math.isfinite(margin) or margin < 0:
            raise ValueError('finite nonnegative collision margin required')
        if world_margin is not None and (not math.isfinite(world_margin) or world_margin < margin):
            raise ValueError('world collision margin must be finite and no smaller than base margin')
        allowed, world_margin_paths = {frozenset(pair) for pair in allowed_pairs}, set(world_margin_paths)
        support = self.support_checks(frames)
        bad = [(*check['pair'], -1.) for check in support if not check['ok']]
        delegated = whole.support_contact.delegated_pairs if whole.support_contact else frozenset()
        poses = {link: whole.world_from_robot.compose(frames[link]) for link in self._unique_links}
        rotations = np.asarray([np.column_stack([poses[link].rotate(tuple(float(i == j) for i in range(3)))
                                                   for j in range(3)]) for link in self._unique_links])
        translations = np.asarray([poses[link].position for link in self._unique_links])
        part_rotations, part_translations = rotations[self._link_indices], translations[self._link_indices]
        centers = np.einsum('nij,nj->ni', part_rotations, self._local_centers)+part_translations
        halves = np.einsum('nij,nj->ni', np.abs(part_rotations), self._local_halves)
        if not np.isfinite(centers).all() or not np.isfinite(halves).all():
            raise ValueError('nonfinite robot frame')
        indices = whole._self_indices
        first, second = indices.T
        self_mask = near_aabbs(np, centers[first], halves[first], centers[second], halves[second], margin)
        world_centers = np.asarray(whole.centers).reshape(-1, 3)
        world_halves = np.asarray(whole.halves).reshape(-1, 3)
        clearances = np.asarray([world_margin if world_margin is not None and row[0] in world_margin_paths
                                 else margin for row in whole.world])
        if (len(world_centers) != len(whole.world) or len(world_halves) != len(whole.world)
                or not np.isfinite(world_centers).all() or not np.isfinite(world_halves).all()
                or np.any(world_halves < 0)):
            raise ValueError('native world bounds do not match world geometry')
        if self.spheres_to_world_mask is None or not len(whole.world):
            world_mask = near_aabbs(np, centers[:, None, :], halves[:, None, :],
                                   world_centers[None, :, :], world_halves[None, :, :], clearances)
        else:
            world_mask = np.asarray(self.spheres_to_world_mask(
                enclosing_spheres(np, centers, self._local_halves), float(clearances.max())))
            if world_mask.shape != (len(whole.parts), len(whole.world)) or world_mask.dtype != np.bool_:
                raise ValueError('GPU broad phase must return a boolean [parts, world] mask')
        objects, transforms, triangles, distances = {}, {}, {}, []

        def robot_object(index):
            if index not in objects:
                link = self._links[index]
                if link not in transforms:
                    link_index = self._link_indices[index]
                    transforms[link] = fcl.Transform(rotations[link_index], translations[link_index])
                objects[index] = fcl.CollisionObject(whole.parts[index][2], transforms[link])
            return objects[index]

        def world_triangles(index):
            if index not in triangles:
                triangles[index] = whole.part_triangles[index] @ part_rotations[index].T+part_translations[index]
            return triangles[index]

        def pair(a, b, index, other, pair_margin, solid_indices=None):
            overlap = fcl.collide(robot_object(index), other, fcl.CollisionRequest(), fcl.CollisionResult()) > 0
            if not overlap and solid_indices is not None:
                from robot_sim.tasks.manager_based.coffee_make.a1_support_triangles import point_in_mesh
                first_triangles, second_triangles = map(world_triangles, solid_indices)
                overlap = (point_in_mesh(first_triangles[0, 0], second_triangles)
                           or point_in_mesh(second_triangles[0, 0], first_triangles))
            distance = 0. if overlap else float(fcl.distance(robot_object(index), other))
            if not math.isfinite(distance) or distance < 0:
                raise RuntimeError('invalid FCL distance')
            distances.append(distance)
            if overlap or distance < pair_margin:
                bad.append((a, b, distance))

        for i, j in indices[self_mask]:
            link, other_link = self._links[i], self._links[j]
            if frozenset((link, other_link)) in delegated:
                continue
            a = whole.parts[i][1] if link == 'base_link' else link
            b = whole.parts[j][1] if other_link == 'base_link' else other_link
            if frozenset((a, b)) not in allowed:
                pair(a, b, i, robot_object(j), margin, (i, j) if 'base_link' in (link, other_link) else None)
        for index, link in enumerate(self._links):
            if link == 'base_link':
                continue  # Native installation semantics: base checked against robot/cup, not its table.
            for world_index in np.flatnonzero(world_mask[index]):
                name, other, _, _ = whole.world[world_index]
                if frozenset((link, name)) not in allowed:
                    pair(link, name, index, other, float(clearances[world_index]))
        result = dict(free=not bad, violations=bad, minimum_distance_m=None,
                    candidate_minimum_distance_m=min(distances) if distances else None,
                    minimum_distance_scope='candidate pairs only; global minimum not computed',
                    broad_phase='conservative_gpu_spheres' if self.spheres_to_world_mask else 'vectorized_aabb',
                    broad_phase_candidates=dict(self_pairs=int(self_mask.sum()), world_pairs=int(world_mask.sum()),
                                                constructed_robot_objects=len(objects)),
                    **dict(support_contacts=support) if whole.support_contact else {})
        if _consume_parts is not None:
            return _consume_parts(result, dict(frames=frames, poses=poses, centers=centers, halves=halves,
                                               world_centers=world_centers, world_halves=world_halves,
                                               robot_object=robot_object))
        return result


def self_check():
    """Prove broad-phase inclusion on synthetic near/tangent and random boxes."""
    import itertools
    import numpy as np
    rng = np.random.default_rng(17)
    centers = rng.normal(size=(256, 3))
    halves = rng.uniform(.0001, .1, size=(256, 3))
    others = centers+np.array([.003, 0, 0])+2*halves
    others[0] = centers[0]+[2*halves[0, 0]+.003, 0, 0]
    mask = near_aabbs(np, centers, halves, others, halves, .003)
    reference = np.asarray([np.linalg.norm(np.maximum(np.abs(a-b)-h-k, 0)) <= .003
                            for a, h, b, k in zip(centers, halves, others, halves)])
    assert np.all(mask[reference]) and mask[0]
    spheres = enclosing_spheres(np, centers, halves).astype(float)
    corners = centers[:, None, :]+halves[:, None, :]*np.asarray(list(itertools.product((-1, 1), repeat=3)))
    assert np.all(np.linalg.norm(corners-spheres[:, None, :3], axis=-1) <= spheres[:, 3, None])
    empty = near_aabbs(np, centers[:, None, :], halves[:, None, :], np.empty((1, 0, 3)), np.empty((1, 0, 3)), np.empty(0))
    assert empty.shape == (256, 0)
    bounds = (np.array([-.1, -.1, -.1]), np.array([.1, .1, .1]))
    relative = np.eye(4)
    assert not support_separated(np, bounds, bounds, relative)  # containment
    relative[0, 3] = .2
    assert not support_separated(np, bounds, bounds, relative)  # touching
    relative[0, 3] = .2+5e-10
    assert not support_separated(np, bounds, bounds, relative)  # native numerical band
    relative[0, 3] = .21
    assert support_separated(np, bounds, bounds, relative)
    angle = np.pi/4
    relative[:2, :2] = [[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]]
    assert not support_separated(np, bounds, bounds, relative)  # rotated box still overlaps
    relative[0, 3] = .25
    assert support_separated(np, bounds, bounds, relative)
    front = np.array([[[-.11, -.02, 0.], [-.1, .02, 0.], [-.09, 0., .02]]])
    rear = front-np.array([.2, 0., 0.])
    assert front_support_bounds(np, rear) is None
    roi = front_support_bounds(np, np.concatenate((front, rear)))
    assert np.array_equal(roi[0], front.min((0, 1))) and roi[0][0] == -.11
    boundary = front.copy(); boundary[:, :, 0] = -.1
    assert front_support_bounds(np, boundary) is not None
    b_bounds = (np.array([-.005, -.005, -.005]), np.array([.005, .005, .005]))
    full = (np.concatenate((front, rear)).min((0, 1)), np.concatenate((front, rear)).max((0, 1)))
    relative = np.eye(4); relative[0, 3] = -.2
    assert not support_separated(np, full, b_bounds, relative)
    assert support_separated(np, roi, b_bounds, relative)
    for gap in (0., 5e-10):
        relative[0, 3] = roi[0][0]-b_bounds[1][0]-gap
        assert not support_separated(np, roi, b_bounds, relative)
    relative[0, 3] = roi[0][0]-b_bounds[1][0]-2e-9
    assert support_separated(np, roi, b_bounds, relative)
    check_vectorized_bounds(np, rng)
    print('conservative whole-geometry broad-phase self-check passed')


def check_vectorized_bounds(np, rng):
    """Use the native median-split tree layout and compare all node arrays exactly."""
    from types import MethodType, SimpleNamespace
    from time import perf_counter

    def reference(mesh, triangles):
        low, high = np.empty((len(mesh.nodes), 3)), np.empty((len(mesh.nodes), 3))
        for index, (left, right, indices) in enumerate(mesh.nodes):
            if indices is not None:
                low[index], high[index] = triangles[indices].min((0, 1)), triangles[indices].max((0, 1))
            else:
                low[index], high[index] = np.minimum(low[left], low[right]), np.maximum(high[left], high[right])
        return low, high

    for count in (1, 7, 16, 17, 31, 129, 2049, 20003):
        triangles = rng.normal(size=(count, 3, 3))
        centers, nodes = triangles.mean(axis=1), []

        def build(indices):
            if len(indices) <= 16:
                node = (None, None, indices)
            else:
                axis = np.ptp(centers[indices], axis=0).argmax()
                ordered = indices[np.argsort(centers[indices, axis])]
                middle = len(ordered)//2
                node = (build(ordered[:middle]), build(ordered[middle:]), None)
            nodes.append(node)
            return len(nodes)-1

        build(np.arange(count))
        mesh = SimpleNamespace(nodes=nodes, triangles=triangles)
        mesh.bounds = MethodType(reference, mesh)
        install_vectorized_bounds(mesh, np)
        cached = mesh.bounds(mesh.triangles)
        assert cached is mesh.bounds(mesh.triangles)
        assert not mesh.triangles.flags.writeable and all(not value.flags.writeable for value in cached)
        assert all(np.array_equal(a, b) for a, b in zip(cached, reference(mesh, triangles)))
        for angle, translation in ((0., (0., 0., 0.)), (.3, (1e-9, .0012, 3.)), (.9, (1e5, -7., .2))):
            rotation = np.array([[np.cos(angle), -np.sin(angle), 0], [np.sin(angle), np.cos(angle), 0], [0, 0, 1]])
            moved = triangles @ rotation.T+translation
            expected, actual = reference(mesh, moved), mesh.bounds(moved)
            assert all(np.array_equal(a, b) for a, b in zip(expected, actual))
        # The same transformed buffer may be updated by a caller; never cache it.
        moved += .0003
        assert all(np.array_equal(a, b) for a, b in zip(reference(mesh, moved), mesh.bounds(moved)))
        if count == 20003:
            timings = []
            for function in (lambda: reference(mesh, moved), lambda: mesh.bounds(moved)):
                trials = []
                for _ in range(7):
                    started = perf_counter(); function(); trials.append(perf_counter()-started)
                timings.append(float(np.median(trials)))
            print(dict(triangles=count, nodes=len(nodes), scalar_median_s=timings[0],
                       vectorized_median_s=timings[1], speedup=timings[0]/timings[1]))


if __name__ == '__main__':
    self_check()
