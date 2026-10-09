"""Mast metrics remain valid for pose search; frozen seeds skip collision screening."""
import unittest
from types import SimpleNamespace

import numpy as np

from examples.scripts.debug import joint_head_wrist_clearance as clearance_module
from examples.scripts.debug import joint_head_wrist_plan as planner
from examples.scripts.debug.joint_head_wrist_clearance import MAST_BOX_NAMES, MastClearance, mast_triangles


def box(lo, hi):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    quads = [(0, 1, 3, 2), (4, 5, 7, 6), (0, 1, 5, 4), (2, 3, 7, 6), (0, 2, 6, 4), (1, 3, 7, 5)]
    triangles = []
    for a, b, c, d in quads:
        triangles += [[corners[a], corners[b], corners[c]], [corners[a], corners[c], corners[d]]]
    return np.array(triangles)


class MastSelection(unittest.TestCase):
    def setUp(self):
        # A pole on the centre line, an arm base off to the side, and the chassis plate below.
        self.pole = box((0.01, -0.02, 0.03), (0.04, 0.02, 0.30))
        self.seat = box((0.01, -0.025, 0.24), (0.09, 0.025, 0.29))
        self.base = box((-0.04, 0.22, -0.03), (0.01, 0.27, 0.02))
        self.plate = box((-0.06, -0.3, -0.08), (0.07, 0.3, 0.0))
        self.camera = box((0.07, -0.02, 0.49), (0.10, 0.02, 0.53))
        self.geometry = SimpleNamespace(
            static_triangles=[self.plate, self.base, self.pole, self.seat, self.camera],
            static_labels=['chassis', 'chassis', 'chassis', 'chassis', 'top_camera'])

    def test_the_pole_is_selected_and_the_arm_bases_are_not(self):
        chosen = mast_triangles(self.geometry)
        self.assertEqual(len(chosen), len(self.pole) + len(self.seat) + len(self.camera))
        centre = chosen.mean(1)
        self.assertTrue((np.abs(centre[:, 1]) <= clearance_module.MAST_HALF_WIDTH_M).all())
        self.assertTrue((centre[:, 2] >= clearance_module.MAST_MIN_Z_M).all())

    def test_a_named_pole_box_is_kept_even_off_the_centre_line(self):
        far = box((0.3, 0.3, 0.3), (0.35, 0.35, 0.35))
        geometry = SimpleNamespace(static_triangles=[self.plate, far], static_labels=['chassis', MAST_BOX_NAMES[0]])
        self.assertEqual(len(mast_triangles(geometry)), len(far))

    def test_a_configuration_without_a_pole_is_reported(self):
        geometry = SimpleNamespace(static_triangles=[self.plate, self.base], static_labels=['chassis', 'chassis'])
        with self.assertRaisesRegex(ValueError, 'no mast geometry'):
            mast_triangles(geometry)

    def metric_for(self, point):
        """The metric caches the meshes it was built from, so build one per probe."""
        side_meshes = {'left': [('link1', np.array([point], float), None)], 'right': []}
        geometry = SimpleNamespace(static_triangles=[self.pole], static_labels=['chassis'],
                                   side_meshes=side_meshes)
        metric = MastClearance(geometry, spacing=0.004, pad=0.1)
        bound = SimpleNamespace(side_meshes=side_meshes,
                                frames=lambda q: {'left': {'link1': np.eye(4)}, 'right': {}})
        return metric, bound

    def test_distance_reads_the_gap_and_never_claims_more_than_there_is(self):
        # The pole spans x 0.01..0.04, so a point at x = 0.01 - gap sits `gap` from its face.
        for gap in (0.05, 0.01, 0.0):
            metric, bound = self.metric_for([0.01 - gap, 0.0, 0.15])
            value = metric.clearance(bound, np.zeros(12))
            self.assertLessEqual(value, gap + 1e-9, 'a measured gap must never be optimistic')
            self.assertGreaterEqual(value, gap - 2 * metric.spacing - 1e-9)
        metric, bound = self.metric_for([2.0, 2.0, 2.0])
        self.assertEqual(metric.clearance(bound, np.zeros(12)), metric.pad)


class SeedMargin(unittest.TestCase):
    """Frozen seeds retain limits but do not repeat search-time mast checks."""

    def setUp(self):
        self.order = [f'{side}/joint{i}' for side in ('left', 'right') for i in range(1, 7)]
        self.pose = list(np.zeros(12))
        self.geometry = SimpleNamespace(limits=np.tile([-3., 3.], (12, 1)),
                                        path_check=lambda a, b: {'ok': True},
                                        frames=lambda q: {'left': {'link6': np.eye(4)},
                                                          'right': {'link6': np.eye(4)}},
                                        mounts={'left': np.eye(4), 'right': np.eye(4)},
                                        side_meshes={}, static_triangles=[], static_labels=[],
                                        config={})
        self.saved = (planner.wrist_visibility, planner.MastClearance)
        planner.wrist_visibility = lambda *a, **k: {'visible': True}

    def tearDown(self):
        planner.wrist_visibility, planner.MastClearance = self.saved

    def seed(self, margin, gap):
        rows = []
        for side, target in (('left', 'right'), ('right', 'left')):
            for index in range(4):
                rows.append({'id': f'{side}-{index}', 'q_nominal_rad': self.pose, 'jaw_gap_mm': [83., 83.],
                             'observer': side, 'target': target,
                             'split': 'holdout' if index == 3 else 'fit'})
        seed = {'schema': 'x5-wrist-motion-seed-v1', 'id': 'test-seed', 'joint_order': self.order,
                'start_q_nominal_rad': self.pose, 'waypoints': rows}
        if margin is not None:
            seed['min_clearance_m'] = margin
        planner.MastClearance = lambda geometry: SimpleNamespace(clearance=lambda g, q: gap)
        return seed

    def run_seed(self, seed):
        mounts = {'left': np.eye(4), 'right': np.eye(4)}
        return planner.plan_from_seed(self.geometry, mounts, {'left': {}, 'right': {}},
                                      np.zeros(12), np.zeros(12), seed)

    def test_declared_margins_do_not_trigger_mast_or_path_checks(self):
        for margin, gap in ((0.02, 0.012), (0.02, 0.031), (None, 0.001)):
            with self.subTest(margin=margin, gap=gap):
                seed = self.seed(margin, gap)
                def unexpected(*args, **kwargs):
                    raise AssertionError('frozen seed must not repeat collision screening')
                planner.MastClearance = unexpected
                self.geometry.path_check = unexpected
                plan = self.run_seed(seed)
                self.assertIsNone(plan['min_clearance_m'])
                self.assertFalse(plan['collision_checked'])
                self.assertNotIn('mast_clearance_m', plan['waypoints'][0])


class UndecidedLegs(unittest.TestCase):
    """Frozen seeds ignore collision verdicts; pose search keeps its verdict rules."""

    def setUp(self):
        self.order = [f'{side}/joint{i}' for side in ('left', 'right') for i in range(1, 7)]
        self.pose = list(np.zeros(12))
        self.geometry = SimpleNamespace(limits=np.tile([-3., 3.], (12, 1)),
                                        path_check=lambda a, b: {'ok': True},
                                        frames=lambda q: {'left': {'link6': np.eye(4)},
                                                          'right': {'link6': np.eye(4)}},
                                        side_meshes={}, static_triangles=[], static_labels=[],
                                        config={})
        self.saved = planner.wrist_visibility
        planner.wrist_visibility = lambda *a, **k: {'visible': True}

    def tearDown(self):
        planner.wrist_visibility = self.saved

    def seed(self):
        rows = []
        for side, target in (('left', 'right'), ('right', 'left')):
            for index in range(4):
                rows.append({'id': f'{side}-{index}', 'q_nominal_rad': self.pose, 'jaw_gap_mm': [83., 83.],
                             'observer': side, 'target': target,
                             'split': 'holdout' if index == 3 else 'fit'})
        return {'schema': 'x5-wrist-motion-seed-v1', 'id': 'test-seed', 'joint_order': self.order,
                'start_q_nominal_rad': self.pose, 'waypoints': rows}

    def run_seed(self, verdicts):
        self.geometry.path_check = lambda a, b: verdicts.pop(0)
        return planner.plan_from_seed(self.geometry, {'left': np.eye(4), 'right': np.eye(4)},
                                      {'left': {}, 'right': {}}, np.zeros(12), np.zeros(12), self.seed())

    def test_seed_does_not_consume_uncertain_or_definite_collision_verdicts(self):
        for collisions in ([['left', 'link1', 'environment', 'chassis', 'intersection solver uncertain']],
                           [['left', 'link2', 'right', 'link2']]):
            verdicts = [{'ok': False, 'fraction': .5, 'collisions': collisions}]
            plan = self.run_seed(verdicts)
            self.assertEqual(len(verdicts), 1, 'seed must not call path_check')
            self.assertEqual(len(plan['waypoints']), 8)
            self.assertEqual(plan['path_undecided'], [])
            self.assertFalse(plan['collision_checked'])

    def test_a_limit_violation_still_refuses(self):
        self.pose[0] = 3.1
        with self.assertRaisesRegex(ValueError, 'exceeds joint limits'):
            self.run_seed([])

    def test_the_verdict_splits_both_kinds_when_they_appear_together(self):
        verdict = planner.path_verdict({'ok': False, 'fraction': .5, 'collisions': [
            ['left', 'link2', 'right', 'link2'],
            ['left', 'link1', 'environment', 'chassis', 'intersection solver uncertain']]})
        self.assertTrue(verdict['blocking'], 'a definite pair must block even beside an undecided one')
        self.assertFalse(verdict['undecided'])
        self.assertEqual(len(verdict['definite']), 1)
        self.assertEqual(len(verdict['uncertain']), 1)


if __name__ == '__main__':
    unittest.main()
