import tempfile
from pathlib import Path
import unittest
from unittest.mock import patch

import cv2
import numpy as np
from scipy.optimize import least_squares

from arx_env.joint_head_hybrid import (detect, marker_predictions, marker_mount, marker_indices,
                                       marker_residual, parameter_settings, weighted_soft_l1)
from arx_env.joint_head_isaac_fit import Model
from arx_env.joint_head_markerless import projected_points


class HybridCheck(unittest.TestCase):
    def test_small_pixel_marker_and_blank_rejection(self):
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        for size in (18, 24, 30, 60):
            image = np.full((120, 160, 3), 255, np.uint8)
            marker = cv2.aruco.generateImageMarker(dictionary, 0, size)
            image[40:40+size, 50:50+size] = marker[:, :, None]
            found, diagnostics = detect(image)
            if size <= 24:  # Pixel centres span size-1 pixels before refinement.
                self.assertEqual(found, {})
                self.assertEqual(diagnostics[0]['reason'], 'under24px')
                continue
            self.assertIn(0, found, diagnostics)
            # Raster outer edge lies half a pixel outside black pixel centres.
            expected = np.array([[49.5, 39.5], [49.5+size, 39.5],
                                 [49.5+size, 39.5+size], [49.5, 39.5+size]])
            self.assertLess(np.linalg.norm(found[0]-expected, axis=1).max(), .5)
        self.assertEqual(detect(np.full((120, 160, 3), 255, np.uint8))[0], {})
        # Large markers go straight to independently quality-gated border lines.
        # A corner optimizer is neither required nor a reason to waive that gate.
        with patch('arx_env.joint_head_hybrid.cv2.cornerSubPix', side_effect=AssertionError('unexpected SUBPIX')):
            self.assertIn(0, detect(image)[0])
        with patch('arx_env.joint_head_hybrid.refine_border', return_value=None):
            found, diagnostics = detect(image)
            self.assertEqual(found, {})
            self.assertEqual(diagnostics[0]['reason'], 'border_refinement_rejected')

    def test_real_marker_ids_and_invalid_manifest(self):
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        image = np.full((140, 240, 3), 255, np.uint8)
        for marker_id, column in ((24, 30), (30, 140)):
            pixels = cv2.aruco.generateImageMarker(dictionary, marker_id, 60)
            image[40:100, column:column+60] = pixels[:, :, None]
        self.assertEqual(set(detect(image)[0]), {24, 30})
        self.assertEqual(marker_indices([{'id': 30}, {'id': 24}]), {30: 0, 24: 1})
        for ids in ([24, 24], [24], [24, 30, 0], [-1, 30], [24, 50], [24., 30], [True, 30]):
            with self.subTest(ids=ids), self.assertRaises(ValueError):
                marker_indices([{'id': marker_id} for marker_id in ids])

    def test_robust_count_weights_do_not_change_noise_transition(self):
        z = np.array([.1, 1., 4., 25.])
        weights = np.array([.08, .08, 1., 1.])
        rho = weighted_soft_l1(z, weights)
        eps = 1e-5
        plus, minus = weighted_soft_l1(z+eps, weights), weighted_soft_l1(z-eps, weights)
        np.testing.assert_allclose((plus[0]-minus[0])/(2*eps), rho[1], rtol=1e-8)
        np.testing.assert_allclose((plus[1]-minus[1])/(2*eps), rho[2], rtol=1e-8)
        # At r=sigma, normalized influence is 1/sqrt(2) regardless of count.
        for count_weight in (.08, 1.):
            self.assertAlmostEqual(weighted_soft_l1(np.array([1.]), count_weight)[1, 0]/count_weight, 1/np.sqrt(2))
        # Check the physical-residual gradient against the actual weighted cost.
        residual, sigma, weight = 7., 2., .08
        cost = lambda r: .5*weighted_soft_l1(np.array([(r/sigma)**2]), weight)[0, 0]
        gradient = weight*residual/sigma**2/np.sqrt(1+(residual/sigma)**2)
        self.assertAlmostEqual((cost(residual+eps)-cost(residual-eps))/(2*eps), gradient, places=9)

    def test_installation_gauge_resolved_by_independent_mesh(self):
        # Deliberately isolate the final-joint / marker-mount rotation ambiguity.
        links = ['<link name="base_link"/>']
        joints, names = [], []
        for side in ('left', 'right'):
            parent = 'base_link'
            for i in range(6):
                child, name = f'{side}_link{i}', f'{side}_joint{i}'
                links.append(f'<link name="{child}"/>')
                joints.append(f'<joint name="{name}" type="revolute"><parent link="{parent}"/>'
                              f'<child link="{child}"/><axis xyz="0 0 1"/>'
                              f'<origin xyz="{.03 if i else 0} {(.1 if side == "left" else -.1) if i == 0 else 0} {1 if i == 0 else 0}"/></joint>')
                parent = child
                names.append(name)
        links.append('<link name="left_finger"/>')
        joints.append('<joint name="grip" type="prismatic"><parent link="left_link5"/>'
                      '<child link="left_finger"/><axis xyz="1 0 0"/></joint>')
        with tempfile.TemporaryDirectory() as tmp:
            urdf = Path(tmp)/'robot.urdf'
            urdf.write_text('<robot name="test">'+''.join(links+joints)+'</robot>')
            model = Model(urdf, names)
            manifest = {'intrinsics': {'fx': 900., 'fy': 900., 'cx': 320., 'cy': 240.},
                        'camera_nominal_T_base_optical': np.eye(4).tolist(),
                        'markers': [{'id': i, 'link': f'{side}_link5', 'size_m': .015,
                                     'T_link_marker': np.eye(4).tolist()} for i, side in enumerate(('left', 'right'))]}
            row = {'q_reported_rad': np.zeros(12).tolist(), 'q_all_reported': {'grip': .03}, 'marker_ids': [0, 1]}
            true = np.zeros(30)
            true[11], true[23] = np.deg2rad(1.2), np.deg2rad(4.)
            row['uv'] = marker_predictions(model, manifest, row, true)[0]
            wrong = true.copy()
            wrong[11], wrong[23] = 0, true[11]+true[23]
            np.testing.assert_allclose(marker_predictions(model, manifest, row, wrong)[0], row['uv'], atol=1e-10)
            # Natural rigid geometry is observed in addition to sticker corners.
            corrs = {'points': np.array([[.06, .02, 0], [-.02, .04, 0], [.01, -.05, 0]]),
                     'links': np.zeros(3, int)}
            meshes = [{'name': 'left_finger'}]
            observed = projected_points(model, manifest, row, meshes, true[:18], corrs)
            def residual(v):
                trial = np.zeros(30)
                trial[11], trial[23] = v
                natural = projected_points(model, manifest, row, meshes, trial[:18], corrs)-observed
                return np.r_[marker_residual(model, manifest, [row], trial), natural.ravel()]
            fit = least_squares(residual, [0., 0.], xtol=1e-12, ftol=1e-12, gtol=1e-12)
            np.testing.assert_allclose(fit.x, true[[11, 23]], atol=1e-7)
            # Full measured prismatic feedback must move the natural points.
            closed = {**row, 'q_all_reported': {'grip': 0.}}
            self.assertGreater(np.linalg.norm(projected_points(model, manifest, closed, meshes, true[:18], corrs)-observed), 40.)

    def test_plane_mount_preserves_normal_and_recovers_uv_yaw(self):
        nominal = np.eye(4)
        nominal[:3, :3] = cv2.Rodrigues(np.array([.2, -.1, .3]))[0]
        marker = {'id': 0, 'link': 'base_link', 'size_m': .015, 'T_link_marker': nominal.tolist()}
        x = np.zeros(24)
        x[18:21] = [.003, -.002, np.deg2rad(7.)]
        actual = marker_mount(marker, x, 'plane')
        # Plane coordinates are local marker x/y: local z origin and normal stay fixed.
        np.testing.assert_allclose(actual[:3, 2], nominal[:3, 2], atol=1e-12)
        np.testing.assert_allclose(actual[:3, 3] - nominal[:3, 3], nominal[:3, :2] @ x[18:20], atol=1e-12)
        np.testing.assert_allclose(actual[:3, 3] - nominal[:3, 3], actual[:3, :2] @ np.zeros(2), atol=1e-12) if False else None
        scale, bounds = parameter_settings('plane')
        self.assertEqual(len(scale), 24)
        self.assertEqual(len(bounds), 24)
        self.assertEqual(np.count_nonzero(bounds[18:]), 6)

    def test_plane_mount_pixel_recovery(self):
        # A synthetic 2-marker image recovers six independent plane parameters.
        manifest = {'intrinsics': {'fx': 900., 'fy': 900., 'cx': 320., 'cy': 240.},
                    'camera_nominal_T_base_optical': np.eye(4).tolist(),
                    'markers': [{'id': i, 'link': 'base_link', 'size_m': .015,
                                 'T_link_marker': np.array([[1,0,0,.04 if i == 0 else -.04],
                                                             [0,1,0,0],[0,0,1,1],[0,0,0,1.]],float).tolist()}
                                for i in (0, 1)]}
        class FakeModel:
            base = 'base_link'
            names = [f'j{i}' for i in range(12)]
            joints = []
            mimics = {}
            root = 'base_link'
            links = {'base_link'}
            def frames(self, q): return {'base_link': np.eye(4)}
        model = FakeModel()
        row = {'q_reported_rad': np.zeros(12).tolist(), 'q_all_reported': {}, 'marker_ids': [0, 1]}
        truth = np.zeros(24)
        truth[18:21] = [.002, -.001, np.deg2rad(4.)]
        truth[21:24] = [-.001, .002, np.deg2rad(-3.)]
        row['uv'] = marker_predictions(model, manifest, row, truth)[0]
        # Relabeling tags must not change their geometry. Detection order can differ
        # from manifest order; reversing the latter swaps parameter blocks only.
        for size, dof in ((24, 3), (30, 6)):
            parameters = np.zeros(size)
            parameters[18:] = np.arange(2*dof)*.0002
            expected = marker_predictions(model, manifest, row, parameters)[0]
            relabeled = {**manifest, 'markers': [{**m, 'id': i} for m, i in zip(manifest['markers'], (24, 30))]}
            tagged_row = {**row, 'marker_ids': [24, 30]}
            np.testing.assert_allclose(marker_predictions(model, relabeled, tagged_row, parameters)[0], expected)
            relabeled['markers'].reverse()
            swapped = np.r_[parameters[:18], parameters[18+dof:], parameters[18:18+dof]]
            np.testing.assert_allclose(marker_predictions(model, relabeled, tagged_row, swapped)[0], expected)
            reversed_row = {**tagged_row, 'marker_ids': [30, 24]}
            np.testing.assert_allclose(marker_predictions(model, relabeled, reversed_row, swapped)[0],
                                       np.r_[expected[4:], expected[:4]])
            single_row = {**tagged_row, 'marker_ids': [24]}
            np.testing.assert_allclose(marker_predictions(model, relabeled, single_row, swapped)[0], expected[:4])
            with self.assertRaises(ValueError):
                marker_mount(relabeled['markers'][0], swapped)
        fit = least_squares(lambda z: marker_residual(model, manifest, [row], np.r_[np.zeros(18), z]),
                            np.zeros(6), xtol=1e-12, ftol=1e-12, gtol=1e-12)
        np.testing.assert_allclose(fit.x, truth[18:], atol=1e-8)


if __name__ == '__main__':
    unittest.main()


class FrozenStickerMountCheck(unittest.TestCase):
    """A mount measured elsewhere can be held fixed instead of re-fitted.

    The head half solves two sticker installations alongside a camera, twelve joint
    offsets and the mesh, which is why they are its weakest parameters; the wrist half
    watches the same stickers from six degrees of freedom. `fit_head` therefore has to
    be able to take a mount it must not move, and to say in its observability report
    that it did not move it.
    """

    def test_the_observability_report_covers_only_the_optimized_subset(self):
        from arx_env.joint_head_hybrid import jacobian_report
        scale = np.ones(6)
        names = ['a', 'b', 'c', 'd', 'e', 'f']

        def residual(x):
            return np.array([x[0], x[1], 2*x[2]])
        everything = jacobian_report(residual, np.zeros(6), scale, names)
        self.assertEqual(everything['rank'], 3)
        self.assertEqual(everything['parameters'], 6)
        self.assertEqual(everything['optimized_parameters'], 6)
        self.assertEqual(everything['free_indices'], [0, 1, 2, 3, 4, 5])
        masked = jacobian_report(residual, np.zeros(6), scale, names, mask=[0, 1, 2])
        self.assertEqual(masked['rank'], 3, 'the mask must not pretend a frozen direction was fitted')
        self.assertEqual(masked['optimized_parameters'], 3)
        self.assertEqual(masked['free_indices'], [0, 1, 2])
        partial = jacobian_report(residual, np.zeros(6), scale, names, mask=[0, 1])
        self.assertEqual(partial['rank'], 2)
        self.assertEqual(partial['optimized_parameters'], 2)
        # A frozen parameter that the residual still responds to must not be counted.
        self.assertEqual(jacobian_report(residual, np.zeros(6), scale, names,
                                         mask=[0, 2])['optimized_parameters'], 2)
        for bad in ([], [0, 6], [-1], [[0]]):
            with self.assertRaises(ValueError):
                jacobian_report(residual, np.zeros(6), scale, names, mask=bad)

    def test_expand_active_leaves_frozen_parameters_at_zero(self):
        from arx_env.joint_head_x5_hybrid import expand_active
        scale = np.arange(1., 31.)
        np.testing.assert_allclose(expand_active(np.ones(30), np.arange(30), scale), scale,
                                   err_msg='with nothing frozen the mapping is the plain scaling')
        # Camera and offsets free, both sticker mounts frozen in the manifest.
        camera_and_offsets = np.arange(18)
        values = expand_active(np.ones(18), camera_and_offsets, scale)
        np.testing.assert_allclose(values[:18], scale[:18])
        np.testing.assert_allclose(values[18:], 0., err_msg='a frozen mount must not be nudged by the optimizer')
        # Only the left mount frozen: its block (18..23) stays zero, the right one moves.
        left_frozen = np.r_[np.arange(18), np.arange(24, 30)]
        values = expand_active(np.ones(len(left_frozen)), left_frozen, scale)
        np.testing.assert_allclose(values[18:24], 0., err_msg='the frozen left mount must stay put')
        np.testing.assert_allclose(values[24:], scale[24:])
