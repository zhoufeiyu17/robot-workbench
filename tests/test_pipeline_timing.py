"""Bounded diagnostics preserve correlation and cannot change source samples."""
from collections import OrderedDict
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from robot_workbench.pipeline_timing import PipelineTiming
from robot_workbench.ros_inputs import RosInputs


class PipelineTimingTests(unittest.TestCase):
    def test_bounded_history_cursor_fault_retention_and_validation(self):
        data=PipelineTiming(capacity=4,sample_count=2)
        for n in range(5):data.record('decode',10,ended_ns=10+n,source_time_ns=123)
        data.fault('freshness timeout',run_id='run')
        for n in range(5):data.record('decode',10,ended_ns=20)
        view=data.snapshot(after=1,limit=2)
        self.assertTrue(view['cursor_gap']);self.assertEqual([x['sequence']for x in view['events']],[7,8])
        self.assertEqual(view['faults'][0]['oldest_sequence'],2)
        self.assertEqual(view['faults'][0]['events'][0]['source_time_ns'],123)
        self.assertEqual(view['stages']['decode']['count'],10)
        self.assertEqual(view['stages']['decode']['recent_count'],2)
        data.record('invalid',20,ended_ns=10)
        data.record('payload',0,image=b'not allowed')
        data.record('reserved',0,sequence=10)
        self.assertEqual(data.snapshot(limit=0)['invalid_measurements'],3)
        self.assertEqual(data.snapshot(limit=0)['events'],[])
        for values in ({'after':-1},{'limit':513},{'limit':True}):
            with self.assertRaises(ValueError):data.snapshot(**values)
        json.dumps(view,allow_nan=False)

    def test_original_publisher_clock_joins_by_stamp_in_both_arrival_orders(self):
        data=PipelineTiming();inputs=RosInputs.__new__(RosInputs)
        inputs.clock_samples={'head':OrderedDict()};inputs.image_arrivals={'head':OrderedDict()}
        now=time.monotonic_ns();stamp=123456
        row=SimpleNamespace(data=json.dumps(dict(kind='frame',stamp_ns=stamp,
            data=dict(recv_mono_ns=now-4_000_000,publish_done_mono_ns=now-3_000_000))))
        with patch('robot_workbench.ros_inputs.timing',data):
            inputs._receive_clock('head',row)
            inputs.image_arrivals['head'][stamp]=now
            inputs._join_timing('head',stamp)
            self.assertEqual(data.snapshot()['stages']['camera.publisher.head']['max_ns'],1_000_000)
            self.assertEqual(data.snapshot()['stages']['camera.publish_return_to_callback.head']['max_ns'],3_000_000)
            inputs.image_arrivals['head'][stamp]=now-3_500_000
            inputs._receive_clock('head',row)
            events=data.snapshot()['events'];self.assertTrue(events[-1]['publish_return_overlapped'])
            self.assertEqual(events[-1]['signed_gap_ns'],-500_000)
            self.assertEqual(data.invalid,0)
        self.assertFalse(inputs.image_arrivals['head']);self.assertFalse(inputs.clock_samples['head'])


if __name__=='__main__':unittest.main()
