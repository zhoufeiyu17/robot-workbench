"""CPU-only timestamp pairing and a deliberately stalled evidence disk."""
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import threading
import unittest
from unittest.mock import patch

import numpy as np

from robot_workbench.inference_evidence import InferenceEvidence
from robot_workbench.observation_pairing import TimestampPairer,PairingPending


class InputIOTests(unittest.TestCase):
    def frames(self, stamps):
        return {name:dict(metadata=dict(host_receive_ns=10_000_000_000+ms*1_000_000,
                    capture_instance_id='camera',serial=name,frame_number=ms),rgb=b'pixels')
                for name,ms in zip(('head','left','right'),stamps)}

    def feedback(self, ms):
        return dict(sample_wall_ns=10_000_000_000+ms*1_000_000,sample_time_s=ms/1000,qpos=[ms])

    def test_common_grid_uses_nearest_history_not_latest_and_preserves_real_stamps(self):
        pairer=TimestampPairer();owner=dict(runtime_nonce='runtime',command_epoch=1)
        pairer.pair(self.frames([0,0,0]),self.feedback(0),owner)
        pairer.pair(self.frames([33,40,30]),self.feedback(35),owner)
        frames,feedback,diagnostic=pairer.pair(self.frames([116,143,60]),self.feedback(150),owner)
        self.assertEqual(diagnostic['target_wall_ns'],10_033_000_000)
        self.assertEqual([frames[k]['metadata']['frame_number']for k in ('head','left','right')],[33,40,30])
        self.assertEqual(feedback['qpos'],[35])
        self.assertEqual(diagnostic['camera_span_ns'],10_000_000)
        self.assertTrue(diagnostic['training_alignment_within_bounds'])
        self.assertFalse(diagnostic['training_bounds_enforced'])

    def test_ties_choose_earlier_and_epoch_reset_cannot_reuse_prior_feedback(self):
        pairer=TimestampPairer();owner=dict(runtime_nonce='runtime',command_epoch=1)
        pairer.pair(self.frames([0,0,0]),self.feedback(0),owner)
        frames,feedback,diagnostic=pairer.pair(self.frames([66,66,33]),self.feedback(66),owner)
        self.assertEqual(frames['head']['metadata']['frame_number'],0)
        self.assertEqual(feedback['qpos'],[0]);self.assertFalse(diagnostic['training_alignment_within_bounds'])
        with self.assertRaises(PairingPending):pairer.pair(self.frames([66,66,33]),self.feedback(67),dict(owner,command_epoch=2))
        self.assertEqual(len(pairer.feedback),1)

    def test_histories_are_bounded_and_regressed_source_time_is_rejected(self):
        pairer=TimestampPairer();owner=dict(runtime_nonce='runtime',command_epoch=1)
        for ms in range(0,1200,10):pairer.pair(self.frames([ms]*3),self.feedback(ms),owner)
        self.assertTrue(all(len(rows)<=16 for rows in pairer.frames.values()))
        self.assertLessEqual(len(pairer.feedback),32)
        with self.assertRaisesRegex(ValueError,'regressed'):
            pairer.pair(self.frames([1180]*3),self.feedback(1200),owner)

    def test_slow_disk_does_not_delay_model_or_result_and_snapshots_are_immutable(self):
        with TemporaryDirectory()as directory:
            writer=InferenceEvidence(directory);entered=threading.Event();release=threading.Event()
            write=writer._write
            def delayed(*args):
                entered.set()
                if not release.wait(3):raise TimeoutError('test disk remained blocked')
                return write(*args)
            writer._write=delayed
            state=np.zeros(14);image=np.zeros((2,2,3),np.uint8);metadata={'task':'coffee'}
            try:
                pending=writer.input('request',state,{'rgb':image},metadata)
                self.assertTrue(entered.wait(1))
                state[:]=7;image[:]=7;metadata['task']='mutated'
                # The same submission ordering as the model callback: no Future.result gate.
                result={'action14':np.zeros((60,14),np.float32),'audit':{'wall_s':.2},'request_id':'request'}
                writer.output('request',result,chunk=True)
                result['action14'][:]=9;result['audit']['wall_s']=9
                self.assertFalse(pending.done());self.assertEqual(writer.snapshot()['pending'],2)
                self.assertFalse((Path(directory)/'request/input.npz').exists())
            finally:release.set();writer.close()
            self.assertEqual(writer.snapshot()['pending'],0);self.assertIsNone(writer.snapshot()['error'])
            with np.load(Path(directory)/'request/input.npz')as data:
                self.assertTrue((data['state14']==0).all());self.assertTrue((data['rgb']==0).all())
            with np.load(Path(directory)/'request/output.npz')as data:self.assertTrue((data['action14']==0).all())
            self.assertEqual(json.loads((Path(directory)/'request/request.json').read_text())['task'],'coffee')
            self.assertEqual(json.loads((Path(directory)/'request/audit.json').read_text())['wall_s'],.2)

    def test_write_failure_and_queue_overflow_are_visible_without_waiting(self):
        with TemporaryDirectory()as directory:
            writer=InferenceEvidence(directory)
            with patch.object(writer,'_write',side_effect=OSError('CPU injected disk error')):
                future=writer.input('failed',np.zeros(14),{}, {})
                with self.assertRaises(OSError):future.result(timeout=1)
            writer.close();self.assertIn('disk error',writer.snapshot()['error'])
            with self.assertRaises(ValueError):writer.input('next',np.zeros(14),{}, {})
            writer=InferenceEvidence(directory,capacity=1);entered=threading.Event();release=threading.Event()
            def blocked(*args):entered.set();release.wait(3)
            with patch.object(writer,'_write',side_effect=blocked):
                try:
                    writer.input('busy',np.zeros(14),{}, {});self.assertTrue(entered.wait(1))
                    with self.assertRaisesRegex(ValueError,'queue full'):writer.input('overflow',np.zeros(14),{}, {})
                finally:release.set();writer.close()
            self.assertIn('queue full',writer.snapshot()['error'])

    def test_failed_prediction_preserves_a_frozen_guard_receipt(self):
        with TemporaryDirectory()as directory:
            writer=InferenceEvidence(directory)
            writer.input('guard-failure',np.zeros(14),{},{}).result(timeout=1)
            error=ValueError('model transport deadline exceeded')
            error.guard_decision=dict(rule_id='G09',verdict='reject',source_time=1.,checked_time=1.1)
            pending=writer.failure('guard-failure',error);error.guard_decision['verdict']='changed'
            pending.result(timeout=1);writer.close()
            value=json.loads((Path(directory)/'guard-failure/prediction-error.json').read_text())
            self.assertEqual(value['guard_decision']['verdict'],'reject')
            self.assertEqual(value['guard_decision']['rule_id'],'G09')


if __name__=='__main__':unittest.main()
