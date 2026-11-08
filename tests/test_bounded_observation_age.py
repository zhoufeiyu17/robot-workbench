"""Late model results must stop before any device conversion or admission."""
import queue
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from robot_workbench.inference_owner import InferenceRunner


class BoundedObservationAgeTest(unittest.TestCase):
    def test_expired_or_future_observation_never_reaches_device_conversion(self):
        for observed in (9.499, 9.0, 10.001):
            with self.subTest(observed=observed):
                runner = object.__new__(InferenceRunner)
                runner.bounded_result = queue.Queue()
                runner.bounded_result.put(({'request_id': 'late-result'}, None))
                runner.bounded_request = {'request_id': 'late-result', 'observation': {'source_time': observed}}
                runner._epoch = 2
                runner.normal_planned = False
                runner.owner = SimpleNamespace(control_time=lambda: 10.)
                runner.config = {'maximum_observation_age_s': .5}
                runner.actual_submission_count = 0
                runner._emit = Mock()
                with patch('robot_workbench.inference_owner.device_state') as convert:
                    with self.assertRaisesRegex(ValueError, 'observation expired before motion; no output'):
                        runner._prepare_bounded_prediction(10., {})
                    convert.assert_not_called()
                decision = runner._emit.call_args_list[-1].args[1]
                self.assertEqual((decision['rule_id'], decision['verdict']), ('G08', 'reject'))
                self.assertEqual((decision['source_time'], decision['checked_time']), (observed, 10.))
                self.assertEqual(decision['request_id'], 'late-result')


if __name__ == '__main__':
    unittest.main()
