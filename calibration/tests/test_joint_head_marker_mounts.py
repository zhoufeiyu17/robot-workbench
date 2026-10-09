"""Exercise the actual CPU mount helper without loading Isaac."""
import importlib.util
from pathlib import Path
import unittest

import numpy as np


class MarkerMountCheck(unittest.TestCase):
    def test_injection_is_bounded_and_never_changes_nominal_fit_input(self):
        path = Path(__file__).resolve().parents[1] / 'examples/scripts/debug/joint_head_isaac_capture.py'
        spec = importlib.util.spec_from_file_location('capture', path)
        capture = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(capture)
        plan = {'markers': [{'id': 0, 'T_link_marker': np.eye(4).tolist()}],
                'marker_mount_errors': {'0': {'xyz_m': [.001, 0, 0], 'rpy_deg': [0, 0, 1]}}}
        actual = capture.marker_mounts(plan)
        self.assertEqual(actual[0]['T_link_marker'][0][3], .001)
        np.testing.assert_array_equal(plan['markers'][0]['T_link_marker'], np.eye(4))
        np.testing.assert_allclose(np.linalg.det(np.array(actual[0]['T_link_marker'])[:3, :3]), 1)
        plan['marker_mount_errors']['0']['xyz_m'] = [.1, 0, 0]
        with self.assertRaises(ValueError):
            capture.marker_mounts(plan)
        plan['marker_mount_errors'] = {'2': {}}
        with self.assertRaises(ValueError):
            capture.marker_mounts(plan)


if __name__ == '__main__':
    unittest.main()
