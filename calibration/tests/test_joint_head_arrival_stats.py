import json
import math
from pathlib import Path
import tempfile
import unittest
from concurrent.futures import ThreadPoolExecutor

from arx_env.joint_head_arrival_stats import ArrivalStats


def sample(event='a', value=1., status='success'):
    return dict(event_id=event, batch_id='batch', workflow='head', status=status,
                q_target_rad=[.5] * 12, q_actual_rad=[.5 + value] * 12,
                timestamp_ns=1, owner_pid=3, command_epoch=0, plan_sha256='f' * 64)


class ArrivalStatsTests(unittest.TestCase):
    def test_statistics_restart_dedup_excludes_failures(self):
        with tempfile.TemporaryDirectory() as root:
            stats = ArrivalStats(root, 'robot', 'physical')
            self.assertTrue(stats.record(sample(value=-1.)))
            self.assertTrue(stats.record(sample('b', 3.)))
            self.assertFalse(stats.record(sample('c', 9., 'failed')))
            # Old failure records remain on disk but never enter the rebuilt summary.
            with stats.samples_path.open('a') as stream:
                stream.write(json.dumps(stats._validate(sample('legacy', 9., 'failed'))) + '\n')
            self.assertFalse(stats.record(sample(value=-1.)))
            summary = ArrivalStats(root, 'robot', 'physical').summary()
            self.assertEqual(summary['total_count'], 2)
            self.assertNotIn('groups', summary)
            row = summary['joints']['left'][0]
            self.assertEqual(row['count'], 2)
            self.assertEqual(row['mean_rad'], 1.)
            self.assertEqual(row['std_rad'], 2.)
            self.assertAlmostEqual(row['rms_rad'], math.sqrt(5))
            self.assertAlmostEqual(row['abs_p95_rad'], 2.9)
            self.assertEqual(row['abs_max_rad'], 3.)
            self.assertEqual(summary, json.loads((Path(root) / 'summary.json').read_text()))
            with self.assertRaisesRegex(ValueError, 'conflicting'):
                stats.record(sample(value=2.))

    def test_source_and_robot_isolation_even_with_empty_log(self):
        with tempfile.TemporaryDirectory() as root:
            stats = ArrivalStats(root, 'robot', 'physical')
            for robot, source in (('robot', 'simulated'), ('other', 'physical')):
                with self.assertRaisesRegex(ValueError, 'mix'):
                    ArrivalStats(root, robot, source)
            with self.assertRaisesRegex(ValueError, 'mix'):
                stats.record({**sample(), 'robot_source': 'simulated'})
            self.assertEqual(stats.summary()['total_count'], 0)

    def test_truncated_tail_repaired_but_middle_and_complete_corruption_rejected(self):
        with tempfile.TemporaryDirectory() as root:
            stats = ArrivalStats(root, 'robot', 'physical')
            stats.record(sample())
            path = Path(root) / 'samples.jsonl'
            good = path.read_bytes()
            path.write_bytes(good + b'{"event_id":"partial')
            stats = ArrivalStats(root, 'robot', 'physical')
            self.assertEqual(path.read_bytes(), good)
            stats.record(sample('b'))
            self.assertEqual(stats.summary()['total_count'], 2)
            for bad in (b'bad\n' + good, good + b'bad\n'):
                path.write_bytes(bad)
                with self.assertRaisesRegex(ValueError, 'corrupt'):
                    ArrivalStats(root, 'robot', 'physical')
                self.assertEqual(path.read_bytes(), bad)

    def test_validation_and_empty_summary(self):
        with tempfile.TemporaryDirectory() as root:
            stats = ArrivalStats(root, 'r', 'simulated')
            self.assertIsNone(stats.summary()['joints']['left'][0]['mean_rad'])
            for values in ([0] * 11, [float('nan')] * 12, [float('inf')] * 12, [True] * 12):
                with self.assertRaises(ValueError):
                    stats.record({**sample(), 'q_actual_rad': values})
            with self.assertRaises(ValueError):
                stats.record({**sample(), 'status': 'pending'})
            self.assertFalse((Path(root) / 'samples.jsonl').exists())

    def test_summary_replace_failure_keeps_old_snapshot_and_raw_log_recovers(self):
        from unittest.mock import patch
        with tempfile.TemporaryDirectory() as root:
            stats = ArrivalStats(root, 'r', 'physical')
            path = Path(root) / 'summary.json'
            previous = path.read_bytes()
            with patch('arx_env.joint_head_arrival_stats.os.replace', side_effect=OSError('disk error')):
                with self.assertRaises(OSError):
                    stats.record(sample())
            self.assertEqual(path.read_bytes(), previous)
            self.assertEqual(len((Path(root) / 'samples.jsonl').read_text().splitlines()), 1)
            recovered = ArrivalStats(root, 'r', 'physical')
            self.assertFalse(recovered.record(sample()))
            self.assertEqual(recovered.summary()['total_count'], 1)
            self.assertEqual(recovered.summary()['raw_path'], str((Path(root) / 'samples.jsonl').resolve()))
            self.assertFalse(list(Path(root).glob('.summary-*.tmp')))

    def test_independent_instances_serialize_and_restore_missing_summary(self):
        with tempfile.TemporaryDirectory() as root:
            first = ArrivalStats(root, 'r', 'physical')
            second = ArrivalStats(root, 'r', 'physical')
            with ThreadPoolExecutor(max_workers=2) as workers:
                list(workers.map(lambda i: (first if i % 2 else second).record(sample(str(i))), range(12)))
            (Path(root) / 'summary.json').unlink()
            self.assertEqual(ArrivalStats(root, 'r', 'physical').summary()['total_count'], 12)


if __name__ == '__main__':
    unittest.main()
