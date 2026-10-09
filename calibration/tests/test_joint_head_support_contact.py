import unittest
import numpy as np
from arx_env.joint_head_support_contact import PreparedMesh, region_overlap, region_separated_with_margin
from arx_env.joint_head_workbench_geometry import box_triangles


def box(center, size):
    transform = np.eye(4)
    transform[:3, 3] = center
    return box_triangles(size, transform).reshape(-1, 3, 3)


class SupportContactTest(unittest.TestCase):
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
