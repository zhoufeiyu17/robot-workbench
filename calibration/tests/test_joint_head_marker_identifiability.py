"""The marker-identifiability measures must mean what this tool's report says they mean.

The tool exists because a two-round pipeline was built on the assumption that the wrist
half measures the sticker mount better than the head half and can hand it back. Everything
that decides that question is arithmetic on rigid transforms and pixel residuals, so the
arithmetic is what is pinned here: an SE(3) path that really starts and ends on the two
estimates, spread statistics that are zero for identical poses and scale with the motion,
and a residual scorer that reports an unusable candidate instead of aborting the report.
"""
import unittest
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from examples.scripts.debug import joint_head_marker_identifiability as module


def rotation(axis, degrees):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_rotvec(np.asarray(axis, float) * np.radians(degrees)).as_matrix()
    return T


class InterpolationTest(unittest.TestCase):
    def test_the_path_starts_and_ends_exactly_on_the_two_estimates(self):
        a = rotation([0, 0, 1], 0.)
        a[:3, 3] = [0.1, 0.2, 0.3]
        b = rotation([0, 0, 1], 90.)
        b[:3, 3] = [0.2, 0.2, 0.5]
        np.testing.assert_allclose(module.interpolate(a, b, 0.), a, atol=1e-12)
        np.testing.assert_allclose(module.interpolate(a, b, 1.), b, atol=1e-12)
        middle = module.interpolate(a, b, 0.5)
        np.testing.assert_allclose(middle[:3, 3], [0.15, 0.2, 0.4], atol=1e-12,
                                   err_msg='translation interpolates linearly')
        angle = np.degrees(np.arccos(np.clip((np.trace(middle[:3, :3]) - 1) / 2, -1, 1)))
        self.assertAlmostEqual(angle, 45., places=6, msg='rotation interpolates on the geodesic')
        # A path must stay rigid at every point, or a "mount" halfway along it is not a mount.
        for t in (0.1, 0.37, 0.9):
            sample = module.interpolate(a, b, t)
            np.testing.assert_allclose(sample[:3, :3] @ sample[:3, :3].T, np.eye(3), atol=1e-12)
            self.assertAlmostEqual(np.linalg.det(sample[:3, :3]), 1., places=12)

    def test_a_translation_only_path_still_reports_the_rotation_it_carries(self):
        a = np.eye(4)
        b = np.eye(4)
        b[:3, 3] = [0.004, 0., 0.]
        moved = module.interpolate(a, b, 1.)
        self.assertAlmostEqual(moved[:3, 3][0], 0.004, places=12)
        np.testing.assert_allclose(moved[:3, :3], np.eye(3), atol=1e-12)


class SpreadTest(unittest.TestCase):
    def test_identical_poses_have_no_spread(self):
        poses = [np.eye(4) for _ in range(5)]
        result = module.spread(poses)
        self.assertEqual(result['poses'], 5)
        self.assertAlmostEqual(result['translation_std_mm'], 0., places=9)
        self.assertAlmostEqual(result['rotation_std_deg'], 0., places=9)

    def test_spread_grows_with_the_motion_and_is_frame_invariant(self):
        poses = [rotation([0, 0, 1], angle) for angle in (0., 10., 20.)]
        for index, pose in enumerate(poses):
            pose[:3, 3] = [0.01 * index, 0., 0.]
        result = module.spread(poses)
        self.assertGreater(result['rotation_std_deg'], 4.)
        self.assertLess(result['rotation_std_deg'], 15.)
        self.assertGreater(result['translation_std_mm'], 5.)
        # A constant change of frame must not change how much a set of poses differs.
        shift = rotation([1, 0, 0], 90.)
        shift[:3, 3] = [0.5, -0.2, 0.7]
        moved = module.spread([shift @ pose for pose in poses])
        self.assertAlmostEqual(moved['translation_std_mm'], result['translation_std_mm'], places=5)
        self.assertAlmostEqual(moved['rotation_std_deg'], result['rotation_std_deg'], places=5)


class CornerRmsTest(unittest.TestCase):
    """The scorer is what turns a candidate mount into evidence, so it has to be exact."""

    def setUp(self):
        self.size = 0.015
        self.intrinsics = {'fx': 394.0, 'fy': 393.0, 'cx': 317.0, 'cy': 236.0}
        self.camera = np.eye(4)
        self.camera[:3, 3] = [0., 0., -0.4]
        self.link6 = rotation([0, 1, 0], 180.)
        self.link6[:3, 3] = [0.12, -0.03, 0.25]
        self.mount = np.eye(4)
        self.mount[:3, 3] = [0.0, 0.0, 0.07]
        corners = module.project(self.camera, self.intrinsics,
                                 self.link6 @ self.mount, self.size)
        self.row = {'link6': self.link6, 'marker': 30, 'corners': corners}

    def test_the_exact_mount_reproduces_the_pixels(self):
        result = module.corner_rms([self.row], {'30': self.mount}, self.camera, self.intrinsics, self.size)
        self.assertAlmostEqual(result['rms'], 0., places=9)
        self.assertEqual(result['corners'], 4)
        self.assertEqual(result['poses'], 1)

    def test_a_shifted_mount_costs_residual_and_keeps_the_other_marker_out_of_it(self):
        shifted = np.eye(4)
        shifted[:3, 3] = [0.004, 0., 0.07]
        moved = module.corner_rms([self.row], {'30': shifted}, self.camera, self.intrinsics, self.size)
        self.assertGreater(moved['rms'], 1.)
        # Only the marker the row carries may be looked up: a wrong key is a reportable
        # gap, not a silent substitution of the other sticker's installation.
        with self.assertRaises(KeyError):
            module.corner_rms([self.row], {'24': shifted}, self.camera, self.intrinsics, self.size)

    def test_a_candidate_behind_the_camera_is_reported_not_raised(self):
        absurd = np.eye(4)
        # link6 carries a 180 degree flip about y, so a +3 m offset along its own z
        # lands the sticker 2.35 m *behind* the optical centre.
        absurd[:3, 3] = [0., 0., 3.0]
        result = module.corner_rms([self.row], {'30': absurd}, self.camera, self.intrinsics, self.size)
        self.assertIn('error', result)
        self.assertIn('behind camera', result['error'])


class BatchSelectionTest(unittest.TestCase):
    """Picking the newest *passing* capture keeps a retried, half-finished batch out."""

    def build(self, root, name, wrist, passed):
        capture = root / 'data' / name / 'capture'
        capture.mkdir(parents=True)
        (capture / 'dataset.json').write_text('{"wrist_contract": %s}'
                                              % ('{}' if wrist else 'null'))
        (capture / 'completion.json').write_text('{"passed": %s}' % ('true' if passed else 'false'))
        return capture

    def test_only_passing_captures_of_the_requested_kind_are_candidates(self):
        import tempfile
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            self.build(root, 'batch-0001', wrist=False, passed=True)
            self.build(root, 'batch-0002', wrist=True, passed=True)
            self.build(root, 'batch-0003', wrist=True, passed=False)
            (root / 'data' / 'batch-0004' / 'capture').mkdir(parents=True)
            chosen = module.newest_batch(root, 'wrist')
            # the candidate is the capture directory: <root>/data/<batch>/capture
            self.assertEqual(chosen.parent.name, 'batch-0002',
                             'the failed retry must not be adopted as evidence')
            self.assertEqual(module.newest_batch(root, 'head').parent.name, 'batch-0001')
            empty = root / 'nothing'
            empty.mkdir()
            self.assertIsNone(module.newest_batch(empty, 'wrist'))


if __name__ == '__main__':
    unittest.main()
