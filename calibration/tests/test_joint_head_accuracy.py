import copy
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from arx_env.joint_head_accuracy import score_head_accuracy, score_wrist_accuracy


class AccuracyCheck(unittest.TestCase):
    def test_head_thresholds_include_all_joints_and_geodesic_rotation(self):
        truth = {'camera_T_world_optical': np.eye(4).tolist(),
                 'joint_offsets_rad': {'left': [0.] * 6, 'right': [0.] * 6}}
        report = copy.deepcopy(truth)
        report['camera_T_world_optical'][0][3] = .002
        report['joint_offsets_rad']['right'][5] = np.deg2rad(.5)
        T = np.asarray(report['camera_T_world_optical'])
        T[:3, :3] = Rotation.from_rotvec(np.deg2rad([.3, 0, 0])).as_matrix()
        report['camera_T_world_optical'] = T.tolist()
        result = score_head_accuracy(report, truth)
        self.assertTrue(result['physical_passed'])
        self.assertAlmostEqual(result['metrics']['camera_translation_error_mm'], 2.)
        self.assertAlmostEqual(result['metrics']['camera_rotation_error_deg'], .3)
        self.assertEqual(len(result['metrics']['joint_offset_errors_deg']), 12)
        report['joint_offsets_rad']['right'][5] = np.deg2rad(.51)
        self.assertFalse(score_head_accuracy(report, truth)['physical_passed'])
        self.assertEqual(score_head_accuracy(report)['status'], 'unverified')
        incomplete = copy.deepcopy(truth)
        del incomplete['joint_offsets_rad']['right']
        self.assertEqual(score_head_accuracy(report, incomplete)['status'], 'unverified')
        report['joint_offsets_rad']['right'][5] = float('nan')
        with self.assertRaises(ValueError):
            score_head_accuracy(report, truth)

    def test_wrist_requires_both_truths_and_rejects_nonrigid_transforms(self):
        truth = {side: np.eye(4).tolist() for side in ('left', 'right')}
        report = {side: {'T_link6_camera': copy.deepcopy(T)} for side, T in truth.items()}
        self.assertTrue(score_wrist_accuracy(report, truth)['physical_passed'])
        result = score_wrist_accuracy(report, {'left': truth['left']})
        self.assertEqual(result['status'], 'unverified')
        self.assertFalse(result['physical_passed'])
        T = np.eye(4)
        T[:3, :3] = Rotation.from_euler('z', 359.6, degrees=True).as_matrix()
        report['right']['T_link6_camera'] = T.tolist()
        result = score_wrist_accuracy(report, truth)
        self.assertFalse(result['physical_passed'])
        self.assertAlmostEqual(result['sides']['right']['metrics']['camera_rotation_error_deg'], .4)
        report['right']['T_link6_camera'][0][0] = 2.
        with self.assertRaises(ValueError):
            score_wrist_accuracy(report, truth)


if __name__ == '__main__':
    unittest.main()
