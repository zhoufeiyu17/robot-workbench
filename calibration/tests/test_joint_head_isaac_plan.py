import copy
from pathlib import Path
import tempfile
import unittest

import numpy as np

from arx_env.joint_head_isaac_plan import JOINT_NAMES, SPEED, authorize_plan, preview_plan, validate_plan
from arx_env.joint_head_isaac_plan import digest


class IsaacPlanTest(unittest.TestCase):
    def test_preview_and_confirmation(self):
        with tempfile.TemporaryDirectory() as tmp:
            urdf = Path(tmp) / 'robot.urdf'
            urdf.write_text('<robot>' + ''.join(
                f'<joint name="{name}" type="revolute"><limit lower="-2" upper="2"/></joint>'
                for name in JOINT_NAMES) + '</robot>')
            identity = dict(scene_sha256='a' * 64, asset_sha256='b' * 64, nominal_camera_sha256='c' * 64)
            current, target = np.zeros(12), np.deg2rad([10, 0, 0, 0, 0, 0, -10, 0, 0, 0, 0, 0])
            plan = preview_plan(urdf, identity, current, target)
            self.assertEqual(plan, preview_plan(urdf, identity, current, target))
            validate_plan(plan, urdf, identity)
            points = np.array([p['q_reported_rad'] for p in plan['trajectory']])
            dt = np.diff([p['time_s'] for p in plan['trajectory']])
            self.assertTrue(np.all((dt > 0) & (dt <= .1000000001)))
            self.assertLessEqual(np.max(abs(np.diff(points, axis=0) / dt[:, None])), SPEED + 1e-12)
            for movement in np.diff(points, axis=0):
                self.assertFalse(np.any(abs(movement[:6]) > 0) and np.any(abs(movement[6:]) > 0))
            np.testing.assert_allclose(points[-1], target)
            consumed = set()
            with self.assertRaisesRegex(ValueError, 'drifted'):
                authorize_plan(plan, urdf, identity, current + .002, plan['hash'], consumed)
            self.assertFalse(consumed)
            authorized = authorize_plan(plan, urdf, identity, current, plan['hash'], consumed)
            self.assertIsNot(authorized, plan)
            with self.assertRaisesRegex(ValueError, 'consumed'):
                authorize_plan(plan, urdf, identity, current, plan['hash'], consumed)
            changed = copy.deepcopy(plan)
            changed['trajectory'][1]['q_reported_rad'][0] += .5
            changed['hash'] = digest({k: v for k, v in changed.items() if k != 'hash'})
            with self.assertRaisesRegex(ValueError, 'canonical'):
                validate_plan(changed, urdf, identity)
            with self.assertRaises(ValueError):
                validate_plan(plan, urdf, {**identity, 'scene_sha256': 'd' * 64})
            with self.assertRaises(ValueError):
                preview_plan(urdf, identity, current, [float('nan')] * 12)
            with self.assertRaisesRegex(ValueError, 'margin'):
                preview_plan(urdf, identity, current, [1.99] * 12)
            home = preview_plan(urdf, identity, target, kind='home')
            self.assertEqual(home['target_q_reported_rad'], [0.] * 12)
            # A hard-limit start is permitted when the requested destination is interior.
            preview_plan(urdf, identity, [-2.] * 12, [0.] * 12)
            urdf.write_text(urdf.read_text().replace('lower="-2"', 'lower="0"'))
            # Unmoved joints remain at known hard-limit starts, with no silent nudge.
            preview_plan(urdf, identity, [0.] * 12, [.2] + [0.] * 11)
            biased = preview_plan(urdf, identity, np.deg2rad([-1.5] * 12), [.2] + np.deg2rad([-1.5] * 11).tolist())
            self.assertTrue(biased['current_outside_nominal_limits'])
            with self.assertRaisesRegex(ValueError, 'calibration allowance'):
                preview_plan(urdf, identity, np.deg2rad([-6.] * 12), [.2] * 12)
            with self.assertRaisesRegex(ValueError, 'margin'):
                preview_plan(urdf, identity, [0.] * 12, [.01] + [0.] * 11)
            with self.assertRaisesRegex(ValueError, 'home target'):
                preview_plan(urdf, identity, [.2] * 12, kind='home')


if __name__ == '__main__':
    unittest.main()
