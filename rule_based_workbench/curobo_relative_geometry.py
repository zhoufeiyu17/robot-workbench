"""Per-instance cup-audit candidate: shared per-call part data, exact native contacts."""
import math
import struct
from contextlib import ExitStack
from types import MethodType

from curobo_whole_geometry import CuroboWholeGeometry, near_aabbs
from cup_contact import permits_claw_contact


def initial(planner, observation, edge_margin):
    """Reuse native exact-state certificates during this single rule search only."""
    geo = planner.geo
    original = geo.corridor
    previous_override = geo.__dict__.get('corridor')
    scene = geo.scene
    contexts = {}
    stats = dict(contexts=0, visits=0, hits=0, misses=0)

    def pose_key(pose):
        return None if pose is None else struct.pack('!7d', *pose.position, *pose.orientation_wxyz)

    with ExitStack() as stack:
        def corridor(self, fk, start, goal, *, world_from_cup, tool_from_cup=None,
                     grasp_contact=False, support_contact=False, initial_table_contact=False,
                     maximum_joint_step=.01, _state_reuse=None):
            options = dict(world_from_cup=world_from_cup, tool_from_cup=tool_from_cup,
                           grasp_contact=grasp_contact, support_contact=support_contact,
                           initial_table_contact=initial_table_contact)
            if _state_reuse is not None or self.scene is not scene:
                return original(fk, start, goal, maximum_joint_step=maximum_joint_step,
                                _state_reuse=_state_reuse, **options)
            key = (id(fk), pose_key(world_from_cup), pose_key(tool_from_cup),
                   grasp_contact, support_contact, initial_table_contact)
            if key not in contexts:
                # ponytail: bound per-search memory; unusual scenes fall back to native checks.
                if len(contexts) >= 32:
                    return original(fk, start, goal, maximum_joint_step=maximum_joint_step, **options)
                contexts[key] = stack.enter_context(self.trajectory_state_reuse(fk, **options))
            return contexts[key].check(start, goal, maximum_joint_step=maximum_joint_step)

        geo.corridor = MethodType(corridor, geo)
        try:
            return planner.initial(observation, edge_margin)
        finally:
            if previous_override is None:
                del geo.corridor
            else:
                geo.corridor = previous_override
            stats.update(contexts=len(contexts), **{key: sum(c.stats[key] for c in contexts.values())
                         for key in ('visits', 'hits', 'misses')})
            geo._candidate_audit_stats['initial_state_reuse'] = stats


def install(geo):
    """Opt in only this loaded RelativeGeometry; native classes/files stay untouched."""
    if not isinstance(geo.whole, CuroboWholeGeometry) or geo.mount != geo.whole.world_from_robot:
        raise ValueError('same-mount CuroboWholeGeometry wrapper required before cup audit installation')
    if getattr(geo.audit, '__func__', None) is not audit:
        geo._candidate_audit_stats = dict(calls=0, cup_world_queries=0, cup_robot_queries=0)
        geo.audit = MethodType(audit, geo)
    return geo


def audit(geo, frames, world_from_cup, *, grasp_contact=False, support_contact=False, initial_table_contact=False):
    """Evaluate cup bands against only conservatively near world/robot parts."""
    from robot_sim.asset_cfg.objects.coffee_machine import RigidTransform
    from robot_sim.tasks.manager_based.coffee_make.relative_geometry import GeometryAudit
    np, fcl, whole = geo.np, geo.fcl, geo.whole
    stats = geo._candidate_audit_stats
    stats['calls'] += 1

    def cup_audit(result, parts):
        # The callback executes within this exact whole.audit call. It never saves
        # these poses/FCL objects, so a later arm/jaw state cannot reuse them.
        if parts['frames'] is not frames:
            raise ValueError('whole/cup audit frame identity mismatch')
        bad = list(result['violations'])
        world_clearance = np.asarray([max(.0001, geo.machine_collision_reserve_m)
                                      if name in geo.machine_shape_paths else .0001
                                      for name, _, _, _ in whole.world])
        for band, shape in geo.cup_bands:
            pose = world_from_cup.compose(RigidTransform((0, 0, (band.z_min_m+band.z_max_m)/2)))
            obj = fcl.CollisionObject(shape, whole.transform(pose))
            center, half = whole.bounds(pose, np.zeros(3), np.asarray(
                [band.radius_m, band.radius_m, (band.z_max_m-band.z_min_m)/2]))
            world_mask = near_aabbs(np, center, half, parts['world_centers'], parts['world_halves'], world_clearance)
            for index in np.flatnonzero(world_mask):
                name, other, _, _ = whole.world[index]
                if band.z_min_m == 0 and ((support_contact and name == geo.scene.support_shape_path)
                                         or (initial_table_contact and name == geo.table.shape_path)):
                    continue
                stats['cup_world_queries'] += 1
                if name in geo.machine_shape_paths and geo.cup_machine_contact_allowance_m > 0:
                    distance, witnesses = geo.cup_pair(obj, other)
                    if any(depth > geo.cup_machine_contact_allowance_m for _, depth in witnesses):
                        bad.append(('cup/'+band.shape_path, name, -max(depth for _, depth in witnesses)))
                    continue
                clearance = geo.machine_collision_reserve_m if name in geo.machine_shape_paths else .0001
                if (fcl.collide(obj, other, fcl.CollisionRequest(), fcl.CollisionResult())
                        or fcl.distance(obj, other) < clearance):
                    bad.append(('cup/'+band.shape_path, name, 0.))
            robot_mask = near_aabbs(np, center, half, parts['centers'], parts['halves'], .0001)
            for index in np.flatnonzero(robot_mask):
                link, name, _, _, _ = whole.parts[index]
                other = parts['robot_object'](index)
                stats['cup_robot_queries'] += 1
                if link == 'base_link':
                    from robot_sim.tasks.manager_based.coffee_make.a1_support_triangles import point_in_mesh
                    body_pose = parts['poses'][link]
                    collided = bool(fcl.collide(obj, other, fcl.CollisionRequest(), fcl.CollisionResult()))
                    if not collided:
                        triangles = whole.part_triangles[index]
                        collided = point_in_mesh(body_pose.inverse().transform_point(pose.position), triangles)
                        vertex = pose.inverse().transform_point(body_pose.transform_point(triangles[0, 0]))
                        collided = collided or (vertex[0]**2+vertex[1]**2 <= band.radius_m**2
                                                and abs(vertex[2]) <= (band.z_max_m-band.z_min_m)/2)
                    distance = 0. if collided else float(fcl.distance(obj, other))
                    if not math.isfinite(distance) or distance < 0:
                        raise ValueError('invalid chassis/cup geometric distance')
                    if collided or distance < .0001:
                        bad.append(('cup/'+band.shape_path, name, distance))
                    continue
                distance, witnesses = geo.cup_pair(obj, other)
                if distance < .0001:
                    allowed = permits_claw_contact(link,(depth for _,depth in witnesses))
                    if not allowed:
                        bad.append(('cup/'+band.shape_path, name, distance))
        return GeometryAudit(not bad, tuple(bad))

    return whole.audit(frames, margin=.0001, world_margin_paths=geo.machine_shape_paths,
                       world_margin=max(.0001, geo.machine_collision_reserve_m), _consume_parts=cup_audit)


def self_check():
    """Compare cup filters with scalar native AABB rules at both contact margins."""
    import numpy as np
    rng = np.random.default_rng(83)
    centers, halves = rng.normal(size=(400, 3)), rng.uniform(.00001, .1, size=(400, 3))
    cup_center, cup_half = np.array([.2, -.1, .3]), np.array([.045, .045, .025])
    for clearance in (.0001, .003):
        centers[:6] = cup_center
        centers[:6, 0] += cup_half[0]+halves[:6, 0]+np.array([0., clearance/2, clearance, clearance+1e-6, -.01, -.02])
        mask = near_aabbs(np, cup_center, cup_half, centers, halves, clearance)
        native = np.asarray([np.linalg.norm(np.maximum(np.abs(c-cup_center)-h-cup_half, 0)) <= clearance
                             for c, h in zip(centers, halves)])
        assert np.all(mask[native]) and mask[0] and mask[1] and not mask[3]
        # A cup wholly inside a larger object must reach the exact containment check.
        assert near_aabbs(np, cup_center, cup_half, cup_center, cup_half*3, clearance)
    print('cup-audit conservative broad-phase reference checks passed')


if __name__ == '__main__':
    self_check()
