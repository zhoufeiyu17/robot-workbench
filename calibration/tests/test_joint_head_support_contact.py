import unittest
import numpy as np
from arx_env.joint_head_support_contact import PreparedMesh, region_overlap, region_separated_with_margin, _separated_triangles
from arx_env.joint_head_workbench_geometry import box_triangles


def box(center, size):
    transform = np.eye(4)
    transform[:3, 3] = center
    return box_triangles(size, transform).reshape(-1, 3, 3)


class SupportContactTest(unittest.TestCase):
    def test_levelwise_bounds_equal_every_scalar_tree_node_after_transforms(self):
        from scipy.spatial.transform import Rotation
        mesh = PreparedMesh(np.concatenate([box([x, 0, 0], [.01, .02, .03])
                                            for x in np.linspace(-.1, .1, 13)]))
        rng = np.random.default_rng(341)
        for _ in range(8):
            triangles = mesh.triangles @ Rotation.from_rotvec(rng.normal(size=3)).as_matrix().T
            triangles += rng.uniform(-1, 1, 3)
            low, high = mesh.bounds(triangles)
            expected_low, expected_high = np.empty_like(low), np.empty_like(high)
            for i, (left, right, ids) in enumerate(mesh.nodes):
                if ids is not None:
                    expected_low[i], expected_high[i] = triangles[ids].min((0, 1)), triangles[ids].max((0, 1))
                else:
                    expected_low[i] = np.minimum(expected_low[left], expected_low[right])
                    expected_high[i] = np.maximum(expected_high[left], expected_high[right])
            np.testing.assert_array_equal(low, expected_low)
            np.testing.assert_array_equal(high, expected_high)

    def test_batched_sat_matches_scalar_reference_at_contact_and_margin(self):
        # Independent scalar evaluation of the original separating-axis rule.
        def reference(a, b, margin):
            ea, eb = np.roll(a, -1, axis=0)-a, np.roll(b, -1, axis=0)-b
            na, nb = np.cross(ea[0], ea[1]), np.cross(eb[0], eb[1])
            axes = [na, nb, *np.cross(ea[:, None, :], eb[None, :, :]).reshape(-1, 3),
                    *np.cross(na, ea), *np.cross(nb, eb)]
            for axis in axes:
                length = np.linalg.norm(axis)
                if length <= 1e-14:
                    continue
                pa, pb = a @ (axis/length), b @ (axis/length)
                if max(pa.min()-pb.max(), pb.min()-pa.max()) > margin+1e-9:
                    return True
            return False
        rng = np.random.default_rng(341)
        first = rng.uniform(-.03, .03, (400, 3, 3))
        second = rng.uniform(-.03, .03, (400, 3, 3))
        triangle = np.array([[0., 0., 0.], [.02, 0., 0.], [0., .02, 0.]])
        offsets = [-.002, 0., .0012-2e-9, .0012, .0012+2e-9, .002]
        first = np.concatenate((first, np.repeat(triangle[None], len(offsets), axis=0)))
        second = np.concatenate((second, np.array([triangle+[0, 0, z] for z in offsets])))
        for margin in (0., .0001, .0012):
            expected = [reference(a, b, margin) for a, b in zip(first, second)]
            np.testing.assert_array_equal(_separated_triangles(first, second, margin), expected)
            expected_single = [reference(triangle, b, margin) for b in second]
            np.testing.assert_array_equal(_separated_triangles(triangle, second, margin), expected_single)

    def test_direction_threshold_and_transform_reuse(self):
        a = PreparedMesh(box([0, 0, 0], [.02, .02, .02]))
        b = PreparedMesh(box([0, 0, .019], [.02, .02, .02]))
        self.assertTrue(region_overlap(a, b))
        self.assertTrue(region_overlap(a, b, b_triangles=b.triangles+[0, 0, .0008]))
        self.assertFalse(region_overlap(a, b, b_triangles=b.triangles+[0, 0, .0012]))

    def test_nested_components_use_union_not_parity(self):
        a = PreparedMesh(np.concatenate([box([0, 0, 0], [4, 4, 4]), box([0, 0, 0], [2, 2, 2])]))
        b = PreparedMesh(box([0, 0, 0], [.1, .1, .1]))
        self.assertEqual(len(a.components), 2)
        self.assertTrue(region_overlap(a, b))
        self.assertTrue(region_overlap(a, b, roi_x_min=0))
        self.assertTrue(region_overlap(b, a, roi_x_min=0))

    def test_roi_excludes_rear_contact_but_keeps_boundary_and_containment(self):
        a = PreparedMesh(box([-2, 0, 0], [1, 1, 1]))
        b = PreparedMesh(box([-2, 0, .4], [1, 1, 1]))
        self.assertTrue(region_overlap(a, b))
        self.assertFalse(region_overlap(a, b, roi_x_min=-.1))
        a = PreparedMesh(box([-.1, 0, 0], [.1, .1, .1]))
        b = PreparedMesh(box([-.07, 0, 0], [.01, .01, .01]))
        self.assertTrue(region_overlap(a, b, roi_x_min=-.1))
        c = PreparedMesh(box([-.04, 0, 0], [.04, .04, .04]))
        self.assertTrue(region_overlap(a, c, roi_x_min=-.1))

    def test_bvh_matches_separated_and_overlapping_box_components(self):
        centers = np.array([[-2., 0., 0.], [0., 0., 0.], [2., 0., 0.]])
        a = PreparedMesh(np.concatenate([box(c, [.5, .5, .5]) for c in centers]))
        b = PreparedMesh(np.concatenate([box([0., 0., 0.], [.1, .1, .1]),
                                         box([0., 2., 0.], [.1, .1, .1])]))
        for x in [-3., -2., -1.8, -1., 0., .8, 1.8, 2., 3.]:
            expected = bool(np.any(np.abs(centers[:, 0]-x) < .3))
            self.assertEqual(region_overlap(a, b, b_triangles=b.triangles+[x, 0, 0]), expected)

    def test_margin_certificate_is_conservative_near_contact(self):
        a = PreparedMesh(box([0, 0, 0], [.02, .02, .02]))
        b = PreparedMesh(box([0, 0, .021], [.02, .02, .02]))
        self.assertTrue(region_separated_with_margin(a, b, margin_m=.0008))
        self.assertFalse(region_separated_with_margin(a, b, margin_m=.0012))
        self.assertFalse(region_separated_with_margin(a, a, margin_m=.001))
        for displacement in [-.0008, 0, .0008]:
            self.assertFalse(region_overlap(a, b, b_triangles=b.triangles+[0, 0, displacement]))
        with self.assertRaises(ValueError):
            region_separated_with_margin(a, b, margin_m=-1)

    def test_margin_expands_roi_and_rejects_nested_solids(self):
        a = PreparedMesh(box([-.1005, 0, 0], [.0004, .02, .02]))
        b = PreparedMesh(box([-.1005, 0, 0], [.0002, .01, .01]))
        self.assertFalse(region_overlap(a, b, roi_x_min=-.1))
        self.assertFalse(region_separated_with_margin(a, b, roi_x_min=-.1, margin_m=.001))

    def test_open_mesh_and_invalid_coordinates_rejected(self):
        triangles = box([0, 0, 0], [1, 1, 1])
        with self.assertRaises(ValueError):
            PreparedMesh(triangles[:-1])
        discovered = PreparedMesh(triangles[:-1], require_closed=False)
        with self.assertRaises(ValueError):
            region_overlap(discovered, discovered)
        a = PreparedMesh(triangles)
        with self.assertRaises(ValueError):
            region_overlap(a, a, b_triangles=triangles*np.nan)


if __name__ == '__main__':
    unittest.main()
