import json
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'examples/scripts/debug'))
from joint_head_wrist_plan import plan_from_seed


class WristSeedTest(unittest.TestCase):
    def test_seed_reuses_nominal_poses_but_rechecks_current_start_and_offsets(self):
        class Geometry:
            config = {'table_z_m': 0.0}
            limits = np.tile([-3.,3.],(12,1))

            def __init__(self):
                self.paths = []

            def with_gaps(self, left, right):
                self.checked_gaps = (left, right)
                return self

            def path_check(self, start, target):
                assert self.checked_gaps == (80., 80.)
                self.paths.append((np.asarray(start).copy(), np.asarray(target).copy()))
                return {'ok': True, 'samples': 2}

        order = [f'{side}/joint{i}' for side in ('left', 'right') for i in range(1, 7)]
        offsets = np.ones(12) * .1
        nominal = np.zeros(12)
        seed = {'schema': 'x5-wrist-motion-seed-v1', 'id': 'test-seed-v1',
                'joint_order': order, 'start_q_nominal_rad': nominal.tolist(), 'min_clearance_m':.1, 'waypoints': []}
        for side, target in (('left', 'right'), ('right', 'left')):
            for i in range(4):
                seed['waypoints'].append({'id': f'{side}-{i}', 'observer': side, 'target': target,
                                          'split': 'holdout' if i == 3 else 'fit',
                                          'q_nominal_rad': (nominal + i * .01).tolist(),
                                          'jaw_gap_mm': [83.0, 83.0]})
        geometry = Geometry(); start = np.ones(12) * .2; cameras = {'left': {}, 'right': {}}
        def visibility(g, *args, **kwargs):
            self.assertEqual(g.checked_gaps, (80., 80.))
            return {'visible': True}
        with patch('joint_head_wrist_plan.MastClearance', side_effect=AssertionError('seed must not check mast')), patch('joint_head_wrist_plan.wrist_visibility', side_effect=visibility):
            result = plan_from_seed(geometry, {'left': np.eye(4).tolist(), 'right': np.eye(4).tolist()},
                                    cameras, start, offsets, seed)
        captures = [row for row in result['waypoints'] if row.get('capture') is not False]
        self.assertEqual(len(captures), 8)
        self.assertEqual(sum(row['split'] == 'holdout' for row in captures), 2)
        self.assertEqual(result['waypoints'][0]['id'], 'seed-entry')
        self.assertEqual(result['waypoints'][-1]['id'], 'seed-return')
        np.testing.assert_allclose(captures[0]['q'], nominal - offsets)
        np.testing.assert_allclose(result['waypoints'][-1]['q'], start)
        self.assertEqual(geometry.paths, [])
        self.assertFalse(result['collision_checked'])
        self.assertTrue(all(row['jaw_gap_mm']==[80.,80.] for row in result['waypoints']))
        self.assertEqual(result['jaw_gap_override_mm'],[80.,80.])
        self.assertEqual(result['seed_jaw_gap_mm'],[[83.,83.]])
        self.assertTrue(all(row['jaw_gap_mm']==[83.,83.] for row in seed['waypoints']))
        geometry.config = {**geometry.config, 'simulation_only': True}
        mounts = {'left': np.eye(4).tolist(), 'right': np.eye(4).tolist()}
        with patch('joint_head_wrist_plan.wrist_visibility', return_value={'visible': False, 'reason': 'model occlusion'}):
            fake = plan_from_seed(geometry, mounts, cameras, start, offsets, seed)
        self.assertFalse(fake['waypoints'][1]['visibility']['visible'])
        geometry.config['simulation_only'] = False
        with patch('joint_head_wrist_plan.wrist_visibility', return_value={'visible': False, 'reason': 'model occlusion'}):
            with self.assertRaisesRegex(ValueError, 'not visible'):
                plan_from_seed(geometry, mounts, cameras, start, offsets, seed)


    def test_a_seed_is_re_anchored_for_the_head_it_is_replayed_under(self):
        # A seed stores nominal joint values and the robot is commanded `nominal - offsets`,
        # so the same seed commands different numbers under a different head. Re-anchoring
        # has to move the commanded number by exactly the offsets' own change and leave the
        # model-space pose alone - otherwise a new head pushes poses a fraction of a
        # milliradian past a zero joint limit and the entry leg is refused.
        class Geometry:
            config = {'table_z_m': 0.0}
            limits = np.tile([-3.,3.],(12,1))

            def path_check(self, start, target):
                return {'ok': True, 'samples': 2}

        order = [f'{side}/joint{i}' for side in ('left', 'right') for i in range(1, 7)]
        anchored = np.linspace(-.004, .004, 12)
        current = anchored + np.linspace(.002, -.002, 12)
        nominal = np.linspace(.1, .5, 12)
        seed = {'schema': 'x5-wrist-motion-seed-v1', 'id': 'test-seed-v1', 'joint_order': order,
                'start_q_nominal_rad': nominal.tolist(), 'anchored_joint_offsets_rad': anchored.tolist(),
                'waypoints': [{'id': f'{side}-{i}', 'observer': side, 'target': target,
                               'split': 'holdout' if i >= 3 else 'fit',
                               'q_nominal_rad': (nominal + i * .01).tolist(),
                               'jaw_gap_mm': [83.0, 83.0]}
                              for side, target in (('left', 'right'), ('right', 'left'))
                              for i in range(4)]}
        mounts = {'left': np.eye(4).tolist(), 'right': np.eye(4).tolist()}
        with patch('joint_head_wrist_plan.wrist_visibility', return_value={'visible': True}):
            result = plan_from_seed(Geometry(), mounts, {'left': {}, 'right': {}}, nominal.copy(),
                                    current, seed)
        captures = [row for row in result['waypoints'] if row.get('capture') is not False]
        expected = nominal - anchored
        np.testing.assert_allclose(captures[0]['q'], expected,
                                   err_msg='the model-space pose is what the seed froze; only the command moves')
        # Adding the anchor back recovers the seed's own model-space pose, unchanged.
        np.testing.assert_allclose(captures[0]['q'] + anchored, nominal)
        old_reading = nominal - current
        self.assertGreater(np.max(np.abs(captures[0]['q'] - old_reading)), 1e-4,
                           're-anchoring must actually move the command')
        without_anchor = dict(seed)
        without_anchor.pop('anchored_joint_offsets_rad')
        with patch('joint_head_wrist_plan.wrist_visibility', return_value={'visible': True}):
            plain = plan_from_seed(Geometry(), mounts, {'left': {}, 'right': {}}, nominal.copy(),
                                   current, without_anchor)
        plain_captures = [row for row in plain['waypoints'] if row.get('capture') is not False]
        np.testing.assert_allclose(plain_captures[0]['q'], nominal - current,
                                   err_msg='a seed without an anchor record keeps the old reading')
        self.assertEqual(result['anchored_joint_offsets_rad'], current.tolist())
        self.assertEqual(result['reanchored_from'], anchored.tolist())
        bad = dict(seed); bad['anchored_joint_offsets_rad'] = [0.] * 6
        with patch('joint_head_wrist_plan.wrist_visibility', return_value={'visible': True}):
            with self.assertRaisesRegex(ValueError, 'anchor'):
                plan_from_seed(Geometry(), mounts, {'left': {}, 'right': {}}, nominal.copy(),
                               current, bad)


if __name__ == '__main__':
    unittest.main()
