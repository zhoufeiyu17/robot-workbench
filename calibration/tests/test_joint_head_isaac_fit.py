"""CPU regression fixtures only; these do not constitute Isaac scene validation."""
import json
import tempfile
import unittest
from unittest.mock import patch
from pathlib import Path

import numpy as np

from arx_env.joint_head_isaac_fit import Model, camera_candidate, coverage_report, detect, fit_dataset, predict, refine_border, score_truth, solve
from arx_env.urdf_mesh import transform


def fixture(root):
    urdf = '<robot name="unit"><link name="world"/><link name="base_link"/>'
    urdf += '<joint name="base" type="fixed"><parent link="world"/><child link="base_link"/><origin xyz="4 5 6" rpy=".2 .3 .4"/></joint>'
    names, markers = [], []
    for side, sign in (('left', 1), ('right', -1)):
        parent = 'base_link'
        for i, axis in enumerate(['0 0 1', '0 1 0', '0 1 0', '1 0 0', '0 1 0', '0 0 1'], 1):
            child, name = f'{side}_link{i}', f'{side}_joint{i}'
            names.append(name)
            xyz = f'{sign * .22} 0 1.2' if i == 1 else '.07 .015 .05'
            urdf += f'<link name="{child}"/><joint name="{name}" type="revolute"><parent link="{parent}"/><child link="{child}"/><origin xyz="{xyz}"/><axis xyz="{axis}"/></joint>'
            if i in (4, 6):
                markers.append({'id': len(markers), 'link': child, 'size_m': .06,
                                'T_link_marker': transform([.04, .025, .015], [.1, .2, .15]).tolist()})
            parent = child
    markers.append({'id': 4, 'link': 'base_link', 'size_m': .12,
                    'T_link_marker': transform([-.15, -.2, 1.2], [.2, .1, .1]).tolist()})
    urdf += '<link name="slider"/><joint name="slide" type="prismatic"><parent link="base_link"/><child link="slider"/><origin xyz=".1 .2 .3"/><axis xyz="1 0 0"/></joint></robot>'
    path = root / 'robot.urdf'
    path.write_text(urdf)
    manifest = {'joint_names': names, 'camera_nominal_T_base_optical': np.eye(4).tolist(),
                'intrinsics': {'fx': 500., 'fy': 500., 'cx': 320., 'cy': 240.}, 'markers': markers}
    return Model(path, names), manifest


class IsaacFitTest(unittest.TestCase):
    def test_manifest_reads_pixels_never_truth_and_refuses_overwrite(self):
        import cv2
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            _, manifest = fixture(root)
            image = np.full((480, 640), 255, np.uint8)
            image[132:180, 232:280] = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), 4, 48)
            manifest.update(schema='joint-head-isaac-v1', urdf_path='robot.urdf', frames=[])
            for i in range(6):
                cv2.imwrite(str(root / f'{i}.png'), image)
                manifest['frames'].append({'id': str(i), 'split': 'fit' if i < 5 else 'holdout',
                                           'image': f'{i}.png', 'q_reported_rad': [0.] * 12})
            (root / 'dataset.json').write_text(json.dumps(manifest))
            (root / 'truth.json').write_text('invalid JSON; must never be opened')
            report, output = fit_dataset(root)
            self.assertFalse(report['used_truth_in_fit'])
            self.assertEqual(report['observability']['rank'], 6)
            self.assertEqual(report['status'], 'IMAGE_FIT_FAIL')
            self.assertEqual(report['observations'][0]['marker_ids'], [4])
            with self.assertRaises(FileExistsError):
                fit_dataset(root, output)
            (root / 'completion.json').write_text('{"passed": false}')
            with self.assertRaisesRegex(ValueError, 'completion must pass'):
                fit_dataset(root)
            (root / 'completion.json').write_text('{"passed": true}')
            def mutate_during_fit(*args):
                (root / 'dataset.json').write_text(json.dumps({**manifest, 'changed': True}))
                return {}
            with patch('arx_env.joint_head_isaac_fit.solve', side_effect=mutate_during_fit):
                with self.assertRaisesRegex(ValueError, 'dataset changed during'):
                    fit_dataset(root)

    def test_pixel_detector_corner_order_and_blank_image(self):
        import cv2
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'image.png'
            image = np.full((200, 240), 255, np.uint8)
            image[50:150, 70:170] = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), 7, 100)
            cv2.imwrite(str(path), image)
            np.testing.assert_allclose(detect(path)[7], [[70, 50], [169, 50], [169, 149], [70, 149]], atol=.6)
            cv2.imwrite(str(path), np.full_like(image, 255))
            self.assertEqual(detect(path), {})

    def test_outer_border_intersections_follow_half_intensity(self):
        import cv2
        image = np.full((160, 180), 240, np.uint8)
        image[40:120, 50:130] = 20
        image = cv2.GaussianBlur(image, (5, 5), .8)
        corners = [[50, 40], [129, 40], [129, 119], [50, 119]]
        # Pixels represent samples at integer centers: physical boundary is halfway
        # between the adjacent white and black pixel centers, not a tuned expansion.
        np.testing.assert_allclose(refine_border(image, corners),
                                   [[49.5, 39.5], [129.5, 39.5], [129.5, 119.5], [49.5, 119.5]], atol=.05)
        self.assertIsNone(refine_border(np.full_like(image, 240), corners))

    def test_joint_camera_recovery_holdout_and_degenerate_control(self):
        with tempfile.TemporaryDirectory() as directory:
            model, manifest = fixture(Path(directory))
            frames = model.frames(np.zeros(12))
            np.testing.assert_allclose(frames['base_link'], np.eye(4), atol=1e-14)
            np.testing.assert_allclose(frames['slider'][:3, 3], [.1, .2, .3], atol=1e-14)
            rng = np.random.default_rng(192)
            truth = np.r_[[.008, -.006, .004], np.deg2rad([.5, -.7, .8]), rng.uniform(-.025, .025, 12)]
            rows = []
            for i in range(24):
                row = {'id': str(i), 'split': 'fit' if i < 18 else 'holdout',
                       'q_reported_rad': rng.uniform(-.4, .4, 12), 'marker_ids': list(range(5))}
                row['uv'] = predict(model, manifest, row, truth)[0] + rng.normal(0, .005, (20, 2))
                rows.append(row)
            report = solve(model, manifest, rows)
            self.assertEqual(report['status'], 'IMAGE_FIT_PASS')
            self.assertEqual(report['observability']['rank'], 18)
            self.assertEqual(report['metrics']['holdout']['frames'], 6)
            self.assertLess(report['metrics']['holdout']['mean_px'], .02)
            score = score_truth(report, {'camera_T_base_optical': camera_candidate(np.eye(4), truth),
                                         'joint_offsets_rad': dict(zip(model.names, truth[6:]))})
            self.assertLess(score['max_joint_offset_error_deg'], .03)
            self.assertLess(score['camera_translation_error_mm'], .2)
            self.assertEqual(report['coverage']['splits']['holdout']['captured_frames'], 6)
            self.assertIsNone(report['coverage']['splits']['fit']['planned_frames'])
            self.assertEqual(len(report['observability']['weakest_direction_physical']), 18)
            self.assertEqual(len(report['per_frame']), 24)
            self.assertEqual(set(report['macro_metrics']['holdout']['per_side']), {'left', 'right', 'body'})
            missing_holdout = rows + [{'id': 'lost', 'split': 'holdout', 'q_reported_rad': np.zeros(12),
                                       'marker_ids': [], 'uv': np.empty((0, 2))}]
            failed_coverage = solve(model, manifest, missing_holdout)
            self.assertEqual(failed_coverage['status'], 'IMAGE_FIT_FAIL')
            self.assertEqual(failed_coverage['coverage']['splits']['holdout']['zero_detection_ids'], ['lost'])
            self.assertEqual(failed_coverage['metrics']['holdout'], report['metrics']['holdout'])
            planned = [{k: r[k] for k in ('id', 'split')} for r in rows] + [{'id': 'not-captured', 'split': 'holdout'}]
            coverage = coverage_report({**manifest, 'planned_frames': planned}, rows)
            self.assertFalse(coverage['holdout_coverage_pass'])
            self.assertEqual(coverage['splits']['holdout']['missing_capture_ids'], ['not-captured'])
            # Body detections alone must not pass, regardless of their excellent fit.
            body_only = [{**r, 'marker_ids': [4], 'uv': r['uv'][-4:]} for r in rows]
            failed = solve(model, manifest, body_only)
            self.assertEqual(failed['status'], 'IMAGE_FIT_FAIL')
            self.assertEqual(failed['observability']['rank'], 6)


if __name__ == '__main__':
    unittest.main()
