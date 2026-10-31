"""Slow GC evidence must outlive a burst of short collections; no hardware."""
import sys
import gc
from pathlib import Path
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from robot_workbench import gc_timing


class GCTimingTest(unittest.TestCase):
    def test_collection_waits_until_all_sdk_workers_exit_and_restores_prior_policy(self):
        original=gc.isenabled();self.addCleanup(gc.enable if original else gc.disable)
        for enabled in (True,False):
            (gc.enable if enabled else gc.disable)()
            first,second=object(),object()
            with patch.object(gc,'collect',wraps=gc.collect) as collect:
                gc_timing.acquire_control(first);gc_timing.acquire_control(second)
                try:
                    self.assertFalse(gc.isenabled());self.assertEqual(collect.call_count,1)
                    cycle=[];cycle.append(cycle);del cycle
                    for _ in range(10000):dict(value=[])
                    self.assertEqual(collect.call_count,1)
                    gc_timing.release_control(first)
                    self.assertFalse(gc.isenabled());self.assertEqual(collect.call_count,1)
                finally:
                    gc_timing.release_control(first);gc_timing.release_control(second)
                self.assertEqual(gc.isenabled(),enabled);self.assertEqual(collect.call_count,2)
                self.assertEqual(gc_timing.snapshot()['control_owners'],0)

    def test_long_pause_survives_short_collection_ring_turnover(self):
        gc_timing._pauses.clear();gc_timing._slow_pauses.clear();gc_timing._started.clear()
        self.addCleanup(gc_timing._pauses.clear);self.addCleanup(gc_timing._slow_pauses.clear)
        with patch.object(gc_timing.time,'monotonic',side_effect=[10.,10.12]):
            gc_timing._record('start',{'generation':2});gc_timing._record('stop',{'generation':2})
        for i in range(100):
            with patch.object(gc_timing.time,'monotonic',side_effect=[20.+i,20.+i+.0001]):
                gc_timing._record('start',{'generation':0});gc_timing._record('stop',{'generation':0})
        result=gc_timing.snapshot();self.assertEqual(len(result['pauses']),64)
        self.assertEqual(len(result['slow_pauses']),1);self.assertAlmostEqual(result['slow_pauses'][0]['duration_s'],.12)
        enabled=gc.isenabled();recent=gc_timing.recent()
        self.assertEqual(gc.isenabled(),enabled);self.assertEqual(len(recent['events']),4)
        self.assertAlmostEqual(recent['last_slow_event']['duration_s'],.12)
        self.assertIsInstance(recent['last_slow_event']['thread_id'],int)


if __name__=='__main__':unittest.main()
