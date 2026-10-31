import tempfile
from pathlib import Path
import unittest

import numpy as np

from robot_workbench.gravity_identification import GravityFeatures, fit_residual, predict_residual


URDF = '''<robot name="test"><link name="base_link"/>
<link name="a"><inertial><origin xyz=".5 .03 .02"/><mass value="2"/></inertial></link>
<link name="b"><inertial><origin xyz=".3 -.02 .04"/><mass value="1"/></inertial></link>
<link name="tool"/>
<joint name="j1" type="revolute"><parent link="base_link"/><child link="a"/><origin xyz="0 0 .2"/><axis xyz="0 1 0"/></joint>
<joint name="j2" type="revolute"><parent link="a"/><child link="b"/><origin xyz="1 0 0"/><axis xyz="0 1 0"/></joint>
<joint name="tip" type="fixed"><parent link="b"/><child link="tool"/><origin xyz=".7 0 0"/></joint></robot>'''


class GravityIdentificationTests(unittest.TestCase):
    def test_gravity_and_extra_payload_are_potential_energy_derivatives(self):
        with tempfile.TemporaryDirectory() as root:
            path = Path(root) / 'model.urdf'; path.write_text(URDF)
            feature = GravityFeatures(path, ['j1', 'j2'])
            com = np.array([.1, .02, -.03]); mass = .25
            def energy(q, payload=False):
                frames = feature.model.frames(q)
                if payload:
                    t = frames['tool']; return -mass * feature.gravity @ (t[:3, 3] + t[:3, :3] @ com)
                return sum(-m * feature.gravity @ (frames[name][:3, 3] + frames[name][:3, :3] @ c)
                           for name, m, c in [('a', 2, np.array([.5, .03, .02])), ('b', 1, np.array([.3, -.02, .04]))])
            rng = np.random.default_rng(27)
            for q in rng.uniform(-1, 1, (20, 2)):
                for payload in (False, True):
                    numeric = np.array([(energy(q + np.eye(2)[i]*1e-6, payload)-energy(q-np.eye(2)[i]*1e-6, payload))/2e-6 for i in range(2)])
                    actual = feature.payload(q, attachment_link='tool', mass_kg=mass, com_m=com,
                                             output_units_per_nm=[1, 1]) if payload else feature.nominal(q)
                    np.testing.assert_allclose(actual, numeric, atol=1e-8, rtol=0)
            q = [.2, .3]
            physical = feature.payload(q, attachment_link='tool', mass_kg=mass, com_m=com, output_units_per_nm=[1, 1])
            converted = feature.payload(q, attachment_link='tool', mass_kg=mass, com_m=com, output_units_per_nm=[1/1.4, 2])
            np.testing.assert_allclose(converted, physical * [1/1.4, 2])
            self.assertFalse(feature.payload(q, attachment_link='tool', mass_kg=0, com_m=com, output_units_per_nm=[1, 1]).any())
            for kwargs in ({'mass_kg': -1}, {'attachment_link': 'unknown'}, {'com_m': [np.nan, 0, 0]}, {'output_units_per_nm': [0, 1]}):
                args = dict(attachment_link='tool', mass_kg=mass, com_m=com, output_units_per_nm=[1, 1]); args.update(kwargs)
                with self.assertRaises(ValueError): feature.payload(q, **args)

    def test_fit_uses_held_out_poses_and_keeps_direction_and_output_bounds(self):
        rng = np.random.default_rng(28)
        x = rng.normal(size=(100, 2, 3)); direction = rng.choice([-1, 1], (100, 2))
        beta = np.array([[.1, -.03, .02], [.02, .04, -.02]])
        y = (x * beta).sum(axis=2) + [.01, .03] + direction * [.02, -.04]
        model = fit_residual(x[:70], y[:70], direction[:70], rcond=1e-10)
        predicted = predict_residual(model, x[70:], direction[70:], output_limits=[1, 1])
        np.testing.assert_allclose(predicted, y[70:], atol=1e-12, rtol=0)
        self.assertFalse(model['hardware_qualified']); self.assertFalse(model['payload_calibrated'])
        with self.assertRaisesRegex(ValueError, 'envelope'):
            predict_residual(model, x[70:], direction[70:], output_limits=[1e-5, 1e-5])
        with self.assertRaises(ValueError): fit_residual(x, y, direction * 2, rcond=1e-10)
        poor = fit_residual(np.zeros_like(x), y, direction, rcond=1e-4)
        self.assertTrue(all(a['rank'] < a['columns'] for a in poor['axes']))


if __name__ == '__main__': unittest.main()
