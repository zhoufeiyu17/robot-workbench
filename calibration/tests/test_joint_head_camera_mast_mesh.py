"""The mast mesh correction must stay closed, put the seat back, and be re-runnable.

synthetic case: a "rod" box, a "seat" plate welded above it in the original file,
and an extension file in which the seat was shifted up while the rod was stretched.
"""
import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'examples/scripts/debug/joint_head_camera_mast_mesh.py'
spec = importlib.util.spec_from_file_location('mast_mesh', SCRIPT)
mast_mesh = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mast_mesh)


def box(x0, x1, y, z0, z1):
    """Twelve triangles of an axis-aligned box, closed on its own."""
    corners = np.array([[x0, -y, z0], [x1, -y, z0], [x1, y, z0], [x0, y, z0],
                        [x0, -y, z1], [x1, -y, z1], [x1, y, z1], [x0, y, z1]], float)
    quads = [[0, 1, 2, 3], [4, 5, 6, 7], [0, 1, 5, 4], [2, 3, 7, 6], [0, 3, 7, 4], [1, 2, 6, 5]]
    triangles = []
    for a, b, c, d in quads:
        triangles.append([corners[a], corners[b], corners[c]])
        triangles.append([corners[a], corners[c], corners[d]])
    return np.array(triangles)


class MastMeshCorrection(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.dir = Path(self.temp.name)
        self.rod = box(0.0, 0.02, 0.01, 0.0, 0.24)          # camera mounting rod
        self.seat = box(0.02, 0.06, 0.02, 0.24, 0.28)        # the mount seat above it
        self.original_triangles = np.concatenate([self.rod, self.seat])
        # The extension file moved the seat up by 0.5 and stretched the rod to reach it.
        stretched = box(0.0, 0.02, 0.01, 0.0, 0.74)
        self.extension_triangles = np.concatenate([stretched, self.seat + np.array([0, 0, 0.5])])
        self.original = self.dir / 'chassis.STL'
        self.extended = self.dir / 'chassis-extended.STL'
        self.output = self.dir / 'corrected.STL'
        mast_mesh.write_stl(self.original, self.original_triangles)
        mast_mesh.write_stl(self.extended, self.extension_triangles)

    def tearDown(self):
        self.temp.cleanup()

    def run_tool(self):
        argv = sys.argv
        sys.argv = ['joint_head_camera_mast_mesh.py', '--original', str(self.original),
                    '--extended', str(self.extended), '--output', str(self.output), '--cut-z', '0.2']
        try:
            code = mast_mesh.main()
        finally:
            sys.argv = argv
        self.assertEqual(code or 0, 0)
        return mast_mesh.read_stl(self.output)

    def block(self):
        return self.original_triangles[self.original_triangles[:, :, 2].min(1) >= 0.2]

    def test_the_seat_comes_back_and_the_mesh_stays_closed(self):
        corrected = self.run_tool()
        _, block_counts, _ = mast_mesh.edge_incidence(self.block())
        # One fan triangle per open boundary edge closes the cut, nothing else is added.
        self.assertEqual(len(corrected), len(self.extension_triangles) + len(self.block()) + int((block_counts == 1).sum()))
        _, counts, _ = mast_mesh.edge_incidence(corrected)
        self.assertTrue((counts == 2).all(), 'corrected chassis must stay a closed two-manifold')
        keys = mast_mesh.triangle_keys(corrected)
        missing = np.setdiff1d(mast_mesh.triangle_keys(self.seat), keys, assume_unique=False)
        self.assertEqual(len(missing), 0, 'the original seat must be present at its original height')
        self.assertTrue(np.isclose(corrected[:, :, 2], 0.28).any(), 'the seat top must sit back at 0.28 m')
        self.assertFalse(np.isclose(self.extension_triangles[:, :, 2], 0.28).any(),
                         'the fixture must really have moved the seat out of place')

    def test_the_top_assembly_is_left_alone(self):
        corrected = self.run_tool()
        keys = set(map(tuple, mast_mesh.triangle_keys(corrected)))
        for triangle in mast_mesh.triangle_keys(self.extension_triangles):
            self.assertIn(tuple(triangle), keys, 'existing extension geometry must survive verbatim')

    def test_rerunning_the_tool_is_a_no_op(self):
        corrected_before = self.run_tool()
        raw = self.output.read_bytes()
        again = self.run_tool()
        self.assertEqual(raw, self.output.read_bytes())
        self.assertEqual(len(again), len(corrected_before))

    def test_an_open_extension_mesh_is_refused(self):
        broken = self.dir / 'broken.STL'
        mast_mesh.write_stl(broken, self.extension_triangles[:-1])  # one face missing
        argv = sys.argv
        sys.argv = ['joint_head_camera_mast_mesh.py', '--original', str(self.original),
                    '--extended', str(broken), '--output', str(self.output), '--cut-z', '0.2']
        try:
            with self.assertRaises(ValueError):
                mast_mesh.main()
        finally:
            sys.argv = argv


if __name__ == '__main__':
    unittest.main()
