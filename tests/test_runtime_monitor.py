import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from robot_workbench.runtime_monitor import RuntimeProbes, IncidentCollector, analyze, memory_issues, replay_incident, MAX_CHANNELS, MAX_SAMPLES


class RuntimeMonitorTests(unittest.TestCase):
    def test_bounded_nonblocking_history_and_fault_markers(self):
        now = [1_000_000_000]
        data = RuntimeProbes(clock=lambda: now[0])
        for i in range(300):
            now[0] += 50_000_000
            data.mark('camera', source_ns=i)
        view = data.snapshot(history=True)
        self.assertLessEqual(len(view['channels']['camera']['history']), MAX_SAMPLES)
        self.assertGreaterEqual(view['channels']['camera']['history'][0]['at_ns'], now[0]-6_000_000_000)
        self.assertNotIn('history', data.snapshot()['channels']['camera'])
        with data.lock:
            data.mark('camera', source_ns=999)
            data.fault('blocked')
        self.assertEqual(data.dropped, 2)
        data.mark('invalid', pixels=b'private-payload')
        self.assertNotIn('invalid', data.snapshot()['channels'])
        for i in range(MAX_CHANNELS+1):
            data.mark('channel'+str(i), count=i)
        self.assertEqual(len(data.channels), MAX_CHANNELS)
        data.fault('G03', run_id='run', epoch=1)
        data.fault('G03', run_id='run', epoch=1)
        self.assertEqual(len(data.faults), 1)
        now[0] += 6_000_000_001
        data.fault('G03', run_id='run', epoch=1)
        self.assertEqual(len(data.faults), 2)

    def test_callback_metadata_and_selection_are_distinguished(self):
        data = RuntimeProbes(clock=lambda: 2_000_000_000)
        data.mark('input/head/raw', kind='raw', source_ns=900_000_000)
        data.mark('input/head/metadata', kind='metadata', source_ns=600_000_000)
        data.mark('input/head/ready', kind='ready', source_ns=600_000_000)
        data.mark('input/selection', kind='selection', head_latest_ns=900_000_000, head_selected_ns=600_000_000)
        data.mark('callback/input', now_ns=1_000_000_000, active=True, stage='DataEvent')
        findings = analyze(data.snapshot())
        self.assertEqual({x['kind'] for x in findings}, {'image_ready_lag','selection_behind_available','active_stage'})
        self.assertEqual(next(x for x in findings if x['kind']=='image_ready_lag')['stage'], 'metadata_pending')
        data.mark('input/head/metadata', kind='metadata', source_ns=900_000_000)
        self.assertEqual(next(x for x in analyze(data.snapshot()) if x['kind']=='image_ready_lag')['stage'], 'metadata_join')

    def test_frequent_edges_preserve_the_full_sampled_window(self):
        now=[1_000_000_000]
        data=RuntimeProbes(clock=lambda:now[0])
        for _ in range(700):
            data.mark('memory',force=True,stage='waiting')
            now[0]+=10_000_000
        row=data.snapshot(history=True)['channels']['memory']
        self.assertGreater(now[0]-row['history'][0]['at_ns'],5_000_000_000)
        self.assertGreater(row['edge_history_dropped'],0)
        self.assertLessEqual(len(row['history']),2*MAX_SAMPLES)

    def test_collector_saves_pre_and_post_without_robot_or_model_calls(self):
        clock = [0.]
        data = RuntimeProbes(clock=lambda: 2_000_000_000)
        data.mark('memory/write', force=True, active=True, stage='write', step=16)
        data.fault('input_G03', run_id='run', epoch=1)
        def reply(history=False):
            return dict(context={'run_id':'run','model_id':'model','memory':{'revision':1,'observations':1}},
                        business={'monitor':data.snapshot(history=history)}, owner={'status':'unavailable'})
        with tempfile.TemporaryDirectory() as directory:
            collector = IncidentCollector('http://127.0.0.1:1', directory, clock=lambda: clock[0])
            with patch.object(collector, 'get', side_effect=reply):
                collector.poll()
                saved = json.loads((Path(directory)/'incident-00.json').read_text())
                self.assertFalse(saved['complete_post_window'])
                self.assertEqual(saved['before']['business']['channels']['memory/write']['history'][0]['step'],16)
                clock[0] = 1.1
                data.mark('memory/write', force=True, active=False, stage='committed', step=16)
                collector.poll()
                saved = json.loads((Path(directory)/'incident-00.json').read_text())
                self.assertTrue(saved['complete_post_window'])
                self.assertEqual(saved['after']['business']['channels']['memory/write']['latest']['stage'],'committed')
                self.assertEqual(collector.saved,1)
                self.assertEqual(collector.pending,{})
                self.assertIsInstance(replay_incident(saved),list)

    def test_delayed_state_and_memory_stride_are_separate_findings(self):
        data=RuntimeProbes(clock=lambda: 2_000_000_000)
        data.mark('input/state/raw',kind='raw',receive_age_ns=350_000_000,source_sequence=7)
        for step in (16,31):
            data.mark('memory/write',force=True,stage='committed',generation=0,kind='MODEL',step=step,stride=16)
        findings=analyze(data.snapshot(history=True))
        self.assertEqual({f['kind']for f in findings},{'source_delivery_lag','memory_stride_mismatch'})

    def test_collector_failures_are_bounded_and_memory_revision_reset_is_allowed(self):
        self.assertFalse(memory_issues({'revision':1,'observations':8},{'revision':2,'observations':0}))
        self.assertTrue(memory_issues({'revision':1,'observations':8},{'revision':1,'observations':7}))
        self.assertTrue(memory_issues(None,{'prediction_commits_memory':True}))
        with tempfile.TemporaryDirectory() as directory:
            collector = IncidentCollector('http://127.0.0.1:1', directory)
            with patch.object(collector, 'get', side_effect=OSError('offline')):
                collector.poll()
            self.assertEqual(collector.read_errors,1)
            with patch.object(collector, 'get', side_effect=OSError('offline')), patch.object(collector, '_write', side_effect=OSError('disk full')):
                collector.poll()
            self.assertEqual(collector.write_errors,1)


if __name__ == '__main__':
    unittest.main()
