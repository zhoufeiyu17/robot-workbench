import tempfile
from pathlib import Path
import unittest

import numpy as np
from scipy.optimize import least_squares

from arx_env.joint_head_isaac_fit import Model
from arx_env.joint_head_markerless import (contours, frames_with_feedback, image_edges,
                                          load_meshes, match, project, raster_depth, exterior_distance,
                                          foreground_edges)


class MarkerlessCheck(unittest.TestCase):
    def test_capture_joint_limit_roundoff(self):
        # Execute the real guard without importing Isaac or mirroring its arithmetic.
        import ast
        source = Path(__file__).resolve().parents[1]/'examples/scripts/debug/joint_head_isaac_capture.py'
        tree = ast.parse(source.read_text())
        loop = next(n for n in ast.walk(tree) if isinstance(n, ast.For)
                    and isinstance(n.target, ast.Name) and n.target.id == 'station'
                    and isinstance(n.iter, ast.Name) and n.iter.id == 'expanded')
        start = next(i for i, n in enumerate(loop.body) if isinstance(n, ast.Assign)
                     and isinstance(n.targets[0], ast.Name) and n.targets[0].id == 'invalid')
        guard = compile(ast.Module(body=loop.body[start:start+3], type_ignores=[]), str(source), 'exec')
        for value in [-3.469446951953614e-18, -1e-13, .5, -1e-6, 1+1e-6]:
            scope = dict(np=np, physical_goal=np.array([value]), limits=np.array([[0., 1.]]), names=['j'], station={'id':'test'})
            if value in [-1e-6, 1+1e-6]:
                with self.assertRaises(ValueError): exec(guard, scope)
            else:
                exec(guard, scope)
                self.assertEqual(scope['physical_goal'][0], max(value, 0))

    def test_exterior_rejects_internal_edges_and_rgb_texture(self):
        import cv2
        mask = np.zeros((140, 180), np.uint8)
        mask[30:110, 40:140] = 1
        distance = exterior_distance(mask)
        self.assertEqual(distance[30, 70], 0)
        self.assertGreater(distance[70, 90], 30)
        image = np.full((140, 180, 3), (100, 150, 190), np.uint8)
        image[32:112, 43:143] = 240
        cv2.line(image, (75, 45), (75, 95), (20, 20, 20), 3)
        edge = foreground_edges(image, mask)
        self.assertGreater(len(edge['points']), 200)
        xy = edge['points']
        self.assertFalse(((abs(xy[:, 0]-75)<3) & (xy[:, 1]>45) & (xy[:, 1]<95)).any())
        self.assertGreater(edge['mask'][32:112, 43:143].mean(), .98)

    def test_depth_occlusion_and_full_feedback(self):
        k = {'fx': 100., 'fy': 100., 'cx': 50., 'cy': 50.}
        v = np.array([[-.3, -.3, 1], [.3, -.3, 1], [.3, .3, 1], [-.3, .3, 1]])
        f = np.array([[0, 1, 2], [0, 2, 3]])
        depth = raster_depth([(v*2, f), (v, f)], k, (100, 100))
        self.assertAlmostEqual(depth[50, 50], 1.)
        self.assertTrue(np.isinf(depth[0, 0]))
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root/'test.urdf').write_text('''<robot name="test"><link name="base_link"/><link name="finger"/>
                <joint name="grip" type="prismatic"><parent link="base_link"/><child link="finger"/>
                <axis xyz="1 0 0"/></joint></robot>''')
            model = Model(root/'test.urdf', [])
            row = {'q_reported_rad': [], 'q_all_reported': {'grip': .044}}
            self.assertAlmostEqual(frames_with_feedback(model, row, {}, [])['finger'][0, 3], .044)
            np.savez(root/'mesh.npz', names=np.array(['finger']), vertices_0=v, faces_0=f)
            meshes = load_meshes(root/'mesh.npz', model.links)
            manifest = {'camera_nominal_T_base_optical': np.eye(4).tolist(), 'intrinsics': k}
            contour = contours(model, manifest, row, meshes, np.zeros(6), (100, 100), 'finger', 400)
            self.assertGreater(len(contour['uv']), 50)
            # There is no internal diagonal in the mesh exterior observations.
            uv = contour['uv']
            boundary = np.minimum.reduce([abs(uv[:, 0]-24.4), abs(uv[:, 0]-84.4), abs(uv[:, 1]-20), abs(uv[:, 1]-80)])
            self.assertLess(boundary.max(), 1e-8)
            # Camera translation recovered from RGB-only rectangle edges.
            import cv2
            image = np.zeros((100, 100, 3), np.uint8)
            cv2.rectangle(image, (21, 20), (81, 80), (255, 255, 255), -1)
            cv2.imwrite(str(root/'image.png'), image)
            edge = image_edges(root/'image.png')
            target, valid = match(contour, edge)
            self.assertGreater(valid.mean(), .85)
            n = contour['normals'][valid]
            uv0 = contour['uv'][valid]
            result = least_squares(lambda d: np.einsum('ij,ij->i', uv0-[100*d[0], 100*d[1]]-target[valid], n), [0., 0.])
            self.assertLess(abs(result.x[0]-.034), .012)
            self.assertLess(abs(result.x[1]), .012)
            # No RGB edges must remain unmatched, never silently fabricated.
            cv2.imwrite(str(root/'blank.png'), np.zeros_like(image))
            self.assertFalse(match(contour, image_edges(root/'blank.png'))[1].any())

    def test_camera_housing_does_not_occlude_target_contour(self):
        k = {'fx': 100., 'fy': 100., 'cx': 50., 'cy': 50.}
        target = np.array([[-.3, -.3, 1], [.3, -.3, 1], [.3, .3, 1], [-.3, .3, 1]])
        housing = target * np.array([1., 1., .02])
        faces = np.array([[0, 1, 2], [0, 2, 3]])
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root/'test.urdf').write_text('''<robot name="test"><link name="base_link"/>
                <link name="static/chassis"/><link name="finger"/>
                <joint name="housing" type="fixed"><parent link="base_link"/><child link="static/chassis"/></joint>
                <joint name="arm" type="fixed"><parent link="base_link"/><child link="finger"/></joint></robot>''')
            model = Model(root/'test.urdf', [])
            np.savez(root/'mesh.npz', names=np.array(['static/chassis', 'finger']),
                     vertices_0=housing, faces_0=faces, vertices_1=target, faces_1=faces)
            meshes = load_meshes(root/'mesh.npz', model.links)
            manifest = {'camera_nominal_T_base_optical': np.eye(4).tolist(), 'intrinsics': k}
            contour = contours(model, manifest, {'q_reported_rad': []}, meshes,
                               np.zeros(6), (100, 100), 'finger', 400)
            self.assertGreater(len(contour['uv']), 50)


if __name__ == '__main__':
    unittest.main()
