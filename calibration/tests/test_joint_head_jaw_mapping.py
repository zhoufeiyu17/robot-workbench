import unittest
from arx_env.joint_head_jaw_mapping import COMMAND, FEEDBACK, GAP_MM, command_sdk, feedback_gaps

class MappingTest(unittest.TestCase):
    def test_internal_type_error_is_distinct_from_nonfinite_feedback(self):
        import numpy as np
        from arx_env.joint_head_jaw_mapping import command_gap_mm
        with self.assertRaisesRegex(ValueError,r'left jaw command SDK value: unsupported type numpy.float64'):
            command_gap_mm('left',np.float64(.01))
        with self.assertRaisesRegex(ValueError,r'right jaw command SDK value: non-finite builtins.float'):
            command_gap_mm('right',float('nan'))

    def test_knots_midpoint_and_range(self):
        for side, index in [('left',6),('right',13)]:
            for sdk, feedback, mm in zip(COMMAND[side],FEEDBACK[side],GAP_MM[side]):
                self.assertAlmostEqual(command_sdk(side, mm), sdk)
                q=[0.]*14;q[index]=feedback
                self.assertAlmostEqual(feedback_gaps(q)[0 if side=='left' else 1], mm)
            self.assertAlmostEqual(command_sdk(side,53),sum(COMMAND[side][2:4])/2)
            self.assertEqual(command_sdk(side,0),0)
            self.assertAlmostEqual(command_sdk(side,GAP_MM[side][1]/2),COMMAND[side][1]/2)
            for bad in [-1,84,float('nan'),True]:
                with self.assertRaises(ValueError):command_sdk(side,bad)

class A1MappingTest(unittest.TestCase):
    def test_a1_mapping_and_invalid_input_is_atomic(self):
        import copy
        import hashlib
        import json
        import tempfile
        from pathlib import Path
        from arx_env.joint_head_jaw_mapping import (configure_mapping, mapping_identity,
            command_gap_mm, feedback_sdk, hold_command_sdk, feedback_state, command_gap_slope)
        data = {'schema_version': 'jaw_mapping_audit.v1', 'reinterpreted_mapping': {
            side: {'candidate_use_allowed': True, 'command_sdk_knots': list(COMMAND[side]),
                   'feedback_sdk_knots': list(FEEDBACK[side]),
                   'metal_knots_mm': [0.] + [v+1 for v in GAP_MM[side][1:]]}
            for side in ('left', 'right')}}
        try:
            with tempfile.TemporaryDirectory() as directory:
                path = Path(directory)/'jaw-mapping.json'
                path.write_text(json.dumps(data))
                identity = configure_mapping(path)
                self.assertEqual(identity['source_sha256'], hashlib.sha256(path.read_bytes()).hexdigest())
                for side, endpoint in [('left', .077815), ('right', .07633375)]:
                    self.assertAlmostEqual(command_sdk(side,83),endpoint)
                    self.assertAlmostEqual(command_gap_mm(side,endpoint),83)
                    self.assertEqual(command_sdk(side,0),0)
                    self.assertEqual(hold_command_sdk(side,-2),0)
                    for measured in (85.019,87.,89.):self.assertAlmostEqual(hold_command_sdk(side,measured),endpoint)
                    self.assertGreater(command_gap_slope(side),0)
                    for bad in (-.1, 83.1, 84, True, float('nan')):
                        with self.assertRaises(ValueError): command_sdk(side,bad)
                    for measured in (-2.000001,89.000001):
                        with self.assertRaises(ValueError):hold_command_sdk(side,measured)
                raw = [0.]*14
                raw[6],raw[13] = feedback_sdk('left',84),feedback_sdk('right',84)
                self.assertEqual(feedback_gaps(raw),[84.,84.])
                self.assertEqual(feedback_state(raw)['mapping_identity'],identity)
                self.assertEqual(feedback_state(raw)['feedback_range_mm'],[[-2.,89.],[-2.,89.]])
                self.assertEqual(feedback_state(raw)['control_range_mm'],[[0.,83.],[0.,83.]])
                from arx_env.joint_head_jaw_mapping import measured_jaw_gaps
                self.assertEqual(measured_jaw_gaps([85.019,89.]).tolist(),[85.019,89.])
                for bad in (-2.000001,89.000001,float('nan'),float('inf')):
                    with self.assertRaises(ValueError):measured_jaw_gaps([bad,0.])
                for bad in ([0,24,24,64,84], [3,24,44,64,84], [0,24,44,64,82],
                            [0,24,float('nan'),64,84], [0,24,44,84], [0,True,44,64,84]):
                    invalid = copy.deepcopy(data)
                    invalid['reinterpreted_mapping']['right']['metal_knots_mm'] = bad
                    path.write_text(json.dumps(invalid))
                    with self.assertRaises(ValueError): configure_mapping(path)
                    self.assertEqual(mapping_identity(),identity)
                    self.assertAlmostEqual(command_sdk('left',83),.077815)
        finally:
            configure_mapping(None)
