import os
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET
import numpy as np

from arx_env.joint_head_workbench_geometry import Geometry, box_triangles
from arx_env.joint_head_ac_one_geometry import ACOneGeometry
from arx_env.joint_head_isaac_fit import Model


def config(path):
    return {'model_family': 'ac_one', 'urdf': str(path),
            'candidate': {'T_world_optical': np.eye(4).tolist(),
                          'T_link6_marker': {str(i): np.eye(4).tolist() for i in (24, 30)}},
            'camera': {'fx': 500, 'fy': 500, 'cx': 320, 'cy': 240, 'width': 640, 'height': 480}}


class ACOneTest(unittest.TestCase):
    @unittest.skipUnless(os.environ.get('AC_ONE_URDF'), 'explicit sealed native asset fixture')
    def test_support_limit_applies_to_both_arms_and_return_home(self):
        import json
        policy=json.loads((Path(__file__).resolve().parents[1]/'configs/ac-one-a1-parking-support.json').read_text())
        c=config(Path(os.environ['AC_ONE_URDF']))
        c.update(asset_geometry_id=policy['asset_geometry_id'],parking_support_contact=policy)
        g=Geometry(c)
        zero=np.zeros(12)
        rows=g.support_contacts(zero)
        self.assertTrue(all(r['ok'] for r in rows))
        self.assertTrue(all(r['contact'] for r in rows if r['pair'][1].endswith('link3')))
        for offset in (0,6):
            shallow=zero.copy();shallow[offset+2]=-.003
            deep=zero.copy();deep[offset+2]=-.008
            self.assertTrue(all(r['ok'] for r in g.support_contacts(shallow)))
            self.assertFalse(all(r['ok'] for r in g.support_contacts(deep)))
            self.assertIn(['left/link2','left/link3'] if offset==0 else ['right/link2','right/link3'],g.collisions(g.frames(deep)))
        reference=np.asarray(policy['reference_q_rad'])
        self.assertTrue(all(r['ok'] for r in g.support_contacts(reference)))
        self.assertNotIn(['right/link2','right/link5'],g.collisions(g.frames(reference)))
        lifted=zero.copy();lifted[2]=.005
        self.assertTrue(g.support_path_check(zero,lifted)['ok'])
        self.assertTrue(g.support_path_check(lifted,zero)['ok'])
        self.assertTrue(g.support_path_check(zero,zero)['ok'])
        with self.assertRaisesRegex(ValueError,'unreviewed'):
            Geometry({**c,'parking_support_contact':{**policy,'max_interference_m':.002}})

    def test_native_aliases_fingers_and_fixed_mounts(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            v = box_triangles([.01, .01, .01], np.eye(4))
            stl = 'solid fixture\n' + ''.join('facet normal 0 0 0\nouter loop\n' +
                   ''.join('vertex %g %g %g\n' % tuple(p) for p in tri) +
                   'endloop\nendfacet\n' for tri in v.reshape(-1, 3, 3)) + 'endsolid\n'
            (root/'box.stl').write_text(stl)
            tree = ET.Element('robot', name='fixture')
            def link(name):
                item = ET.SubElement(tree, 'link', name=name)
                visual = ET.SubElement(item, 'visual')
                ET.SubElement(ET.SubElement(visual, 'geometry'), 'mesh', filename='box.stl')
            link('base_link')
            for side, offset, y in (('left', 0, .25), ('right', 10, -.25)):
                for i in range(1, 9):
                    name = f'{side}_link{i+offset}'; link(name)
                    joint = ET.SubElement(tree, 'joint', name=f'{side}_joint{i+offset}',
                                          type='revolute' if i <= 6 else 'prismatic')
                    parent = 'base_link' if i == 1 else f'{side}_link{min(i-1,6)+offset}'
                    ET.SubElement(joint, 'parent', link=parent)
                    ET.SubElement(joint, 'child', link=name)
                    ET.SubElement(joint, 'origin', xyz=f'0 {y} 0' if i == 1 else '0.1 0 0')
                    ET.SubElement(joint, 'axis', xyz='0 1 0' if i != 8 else '0 -1 0')
                    ET.SubElement(joint, 'limit', lower='-3', upper='3', velocity='1')
                for name, parent in ((f'{side}_wrist_camera_link', f'{side}_link{6+offset}'),
                                     (f'{side}_wrist_camera_rgb_optical_frame', f'{side}_wrist_camera_link')):
                    link(name)
                    joint = ET.SubElement(tree, 'joint', name=name+'_fixed', type='fixed')
                    ET.SubElement(joint, 'parent', link=parent)
                    ET.SubElement(joint, 'child', link=name)
                    ET.SubElement(joint, 'origin', xyz='0 0 .02')
            path = root/'robot.urdf'; ET.ElementTree(tree).write(path)
            g = Geometry(config(path))
            self.assertIsInstance(g, ACOneGeometry)
            self.assertIsNone(g.hand)
            q = np.linspace(-.1, .1, 12)
            native = g.model.frames(q)
            frames = g.frames(q)
            np.testing.assert_allclose(frames['left']['link6'], native['left_link6'])
            np.testing.assert_allclose(frames['right']['link6'], native['right_link16'])
            np.testing.assert_allclose(frames['right']['camera_body'], native['right_wrist_camera_link'])
            closed, opened = g.with_gaps(0, 0).frames(np.zeros(12)), g.with_gaps(80, 80).frames(np.zeros(12))
            for side in ('left', 'right'):
                np.testing.assert_allclose(opened[side]['finger_positive'][:3,3]-closed[side]['finger_positive'][:3,3], [0,.04,0])
                np.testing.assert_allclose(opened[side]['finger_negative'][:3,3]-closed[side]['finger_negative'][:3,3], [0,-.04,0])
            self.assertEqual(g.with_feedback_gaps(-1, 84).gaps, {'left': -.001, 'right': .084})
            with self.assertRaises(ValueError): g.with_gaps(-1, 84)
            with self.assertRaises(ValueError): g.model.frames(q, {'left_joint1': .2})
            with self.assertRaises(ValueError): g.model.frames(q, {'left_joint7': float('nan')})
            self.assertIn(frozenset(('link6', 'camera_body')), g.adjacent)
            self.assertNotIn(frozenset(('link5', 'camera_body')), g.adjacent)
            self.assertEqual(g.static_adjacent, {('link1', 'chassis')})

    @unittest.skipUnless(os.environ.get('AC_ONE_URDF'), 'explicit sealed native asset fixture')
    def test_sealed_asset_fk(self):
        g = Geometry(config(Path(os.environ['AC_ONE_URDF'])))
        self.assertEqual(len(g.side_meshes['left']), 9)
        self.assertEqual(len(g.side_meshes['right']), 9)
        self.assertEqual({p[0] for p in g.static_meshes}, {'chassis', 'top_camera_link'})
        rng = np.random.default_rng(24)
        for q in rng.uniform(-.2, .2, (20, 12)):
            expected = g.model.frames(q)
            actual = g.frames(q)
            for side, terminal in (('left', 'left_link6'), ('right', 'right_link16')):
                np.testing.assert_allclose(actual[side]['link6'], expected[terminal], atol=1e-12)


    @unittest.skipUnless(os.environ.get('AC_ONE_URDF'), 'explicit sealed native asset fixture')
    def test_native_head_and_mesh_offsets_agree_without_double_base(self):
        from arx_env.joint_head_x5_hybrid import X5MeshModel
        from joint_head_real_fit import Fit
        g = Geometry(config(Path(os.environ['AC_ONE_URDF'])))
        adapter = X5MeshModel(g)
        q = np.linspace(-.25, .3, 12)
        offsets = np.array([0, .012, -.021, .031, -.042, 0,
                            0, -.015, .022, -.032, .043, 0])
        # A stale X5 base transform must not be applied to native AC-One FK.
        old_base = np.eye(4); old_base[:3, 3] = [10, 20, 30]
        scene = {'T_world_arm_base': {s: old_base for s in ('left', 'right')}}
        fit = Fit(g.config['urdf'], scene, {'T_world_camera': np.eye(4)}, g.k, .015, geometry=g)
        row = {'q_reported_rad': q.tolist(), 'jaw_gap_mm': [-1., 84.25]}
        actual = adapter.frames_with_feedback(row, {}, offsets)
        finger_values = {f'{side}_joint{i+shift}': (gap/1000-.000008)/2
                         for side, shift, gap in (('left', 0, -1.), ('right', 10, 84.25))
                         for i in (7, 8)}
        expected = g.model.frames(q+offsets, joint_values=finger_values)
        for side, shift, index in (('left', 0, 0), ('right', 10, 1)):
            terminal = fit.terminal({'q': q}, index, offsets[index*6+1:index*6+5])
            np.testing.assert_allclose(terminal, expected[f'{side}_link{6+shift}'], atol=1e-12)
            for native, alias in g.aliases[side].items():
                np.testing.assert_allclose(actual[f'{side}/{alias}'], expected[native], atol=1e-12)
            for name, vertices, faces in g.full_side_meshes[side]:
                mesh = next(m for m in adapter.meshes if m['name'] == f'{side}/{name}')
                self.assertEqual(len(mesh['f']), len(faces))
                np.testing.assert_array_equal(mesh['v'][mesh['f']], vertices[faces])
        for name, _, _ in g.static_meshes:
            np.testing.assert_array_equal(actual[f'static/{name}'], np.eye(4))
        np.testing.assert_array_equal(q, np.linspace(-.25, .3, 12))


if __name__ == '__main__': unittest.main()
