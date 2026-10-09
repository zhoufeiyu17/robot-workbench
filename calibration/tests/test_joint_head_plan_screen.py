"""The head plan screen must separate undecided LP verdicts from real overlaps.

The standard head seed carries two `intersection solver uncertain` segments by design,
so a screen that treats any failed path_check as a collision would refuse the trajectory
the tool is meant to run; one that ignores failures would be pointless. What matters is
that the classification travels with the verdict, that a definite pair blocks, and that
segments are chained from the given start pose rather than from the previous waypoint.
"""
import unittest
from types import SimpleNamespace

import numpy as np

from examples.scripts.debug import joint_head_plan_screen as screen_module


class FakeGeometry:
    """Stands in for the real geometry: scripted verdicts keyed by target index."""

    def __init__(self, verdicts):
        self.verdicts = verdicts
        self.seen = []

    def with_gaps(self, left, right):
        self.gaps = (left, right)
        return self

    def _q(self, q):
        return np.asarray(q, float)

    def path_check(self, start, target):
        self.seen.append((tuple(np.asarray(start, float)), tuple(np.asarray(target, float))))
        return self.verdicts[len(self.seen) - 1]


def row(target):
    return {'q': [float(target)] * 12, 'name': 'wp%d' % target}


class ScreenVerdicts(unittest.TestCase):
    def test_definite_overlap_blocks_and_uncertain_does_not(self):
        geometry = FakeGeometry([
            {'ok': True, 'samples': 4},
            {'ok': False, 'fraction': .73, 'collisions': [['right', 'link1', 'environment', 'chassis', 'intersection solver uncertain']]},
            {'ok': False, 'fraction': .5, 'collisions': [['left', 'link2', 'right', 'link2']]},
            {'ok': False, 'fraction': .1, 'collisions': []},
        ])
        verdicts = screen_module.screen(geometry, [row(i) for i in range(4)], [0.0] * 12)
        summary = screen_module.summarise(verdicts)
        self.assertEqual(summary['segments'], 4)
        self.assertEqual(summary['blocking_segments'], [2, 3])
        self.assertEqual(summary['uncertain_segments'], [1])
        self.assertIn('求解器不确定', verdicts[1]['detail'])
        self.assertIn('干涉/自碰撞', verdicts[2]['detail'])
        self.assertIn('超出关节限位', verdicts[3]['detail'])

    def test_segments_chain_from_the_given_start(self):
        geometry = FakeGeometry([{'ok': True, 'samples': 3}] * 3)
        screen_module.screen(geometry, [row(1), row(2), row(3)], [9.0] * 12, gaps=(80., 81.))
        starts = [pair[0][0] for pair in geometry.seen]
        self.assertEqual(starts, [9.0, 1.0, 2.0], 'each segment must start where the last one ended')
        self.assertEqual(geometry.gaps, (80., 81.), 'the jaws must be screened held open')

    def test_uncertain_only_pairs_are_reported_not_swallowed(self):
        pairs = [['right', 'link1', 'environment', 'chassis', 'intersection solver uncertain']]
        definite, uncertain = screen_module.classify(pairs)
        self.assertEqual(definite, [])
        self.assertEqual(len(uncertain), 1)
        self.assertEqual(screen_module.segment_detail({'ok': False, 'collisions': pairs}),
                         '求解器不确定（保守判定，非实测干涉）：right/link1/environment/chassis')


if __name__ == '__main__':
    unittest.main()
