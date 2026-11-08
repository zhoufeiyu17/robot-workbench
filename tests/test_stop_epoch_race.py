"""Deterministic Stop between feedback/read and shared-reference admission."""
from copy import deepcopy
import unittest
import test_shared_target_routes as shared
from robot_workbench.sdk_process import SDKCommandRejected

class StopEpochRaceTest(unittest.TestCase):
    profile='normal_X5'
    setUp=shared.SharedTargetRoutesTest.setUp
    cleanup=shared.SharedTargetRoutesTest.cleanup
    create=shared.SharedTargetRoutesTest.create
    advance=shared.SharedTargetRoutesTest.advance
    confirm=shared.SharedTargetRoutesTest.confirm

    def exercise(self,phase,real_failure=False,unexpected_epoch=False):
        self.create()
        review=self.runtime.submit(self.creds['calibration'],dict(action='preview_workbench',waypoints=[dict(q12=[.15]+[0.]*11,speeds12=[.1]*12,jaw_gap_mm=[40.,40.])]))
        self.confirm('calibration',review);self.advance(60);self.assertIsNotNone(self.raw.window)
        fired=[];name='read_calibration_state'if phase=='feedback'else'queue_reference';original=getattr(self.raw,name)
        def inject(*args,**kwargs):
            result=original(*args,**kwargs)
            if not fired:
                fired.append(True)
                if unexpected_epoch:self.runtime.owner.epoch+=1
                else:self.runtime.request_stop()
                if real_failure:raise SDKCommandRejected('real native failure alongside stop')
            return result
        setattr(self.raw,name,inject);self.advance(12);state=self.runtime.snapshot(self.creds['calibration']);count=len(self.raw.packets)
        self.advance(60);self.assertEqual(len(self.raw.packets),count);self.assertIsNone(self.raw.window);self.assertTrue(fired)
        self.assertFalse(state['automatic_enabled']);self.assertTrue(state['connected'])
        if real_failure or unexpected_epoch:
            self.assertIn('FAULT',state['state'])
            self.assertIn('real native failure'if real_failure else'epoch revoked',state['reason'])
        else:
            self.assertNotIn('FAULT',state['state']);self.assertEqual(state['motion']['status'],'STOPPED')
            self.assertTrue(state['stop_receipt']['stop_hold_return_confirmed']);self.assertTrue(state['stop_receipt']['prior_calls_complete'])
            self.assertFalse(any(k in ('fault','fault_diagnostic')for k,v in self.events))

    def test_stop_during_feedback_is_cancellation(self):self.exercise('feedback')
    def test_stop_during_successful_native_admission_is_cancellation(self):self.exercise('native')
    def test_concurrent_real_native_error_still_faults(self):self.exercise('native',True)
    def test_unrequested_epoch_change_still_faults(self):self.exercise('feedback',unexpected_epoch=True)

    def test_later_fault_hold_cannot_rewrite_old_operator_stop_receipt(self):
        self.create();self.runtime.request_stop();self.advance(2)
        receipt=deepcopy(self.runtime.stop_receipt)
        self.assertTrue(receipt['stop_hold_return_confirmed'])
        self.runtime.owner.motion_fault(ValueError('independent later fault'))
        self.advance(2)
        self.assertGreater(self.runtime.hold_returns['left']['epoch'],receipt['barrier_epoch']+1)
        self.assertEqual(self.runtime.stop_receipt,receipt)

    def test_missing_stop_return_cannot_be_filled_by_later_epoch(self):
        self.create();self.runtime.request_stop();self.advance(2)
        self.runtime.stop_receipt['stop_hold_return_confirmed']=False
        self.runtime.stop_receipt.pop('sdk_hold_returns',None)
        self.runtime.owner.motion_fault(ValueError('independent later fault'))
        self.advance(2)
        self.assertFalse(self.runtime.stop_receipt['stop_hold_return_confirmed'])
        self.assertNotIn('sdk_hold_returns',self.runtime.stop_receipt)

    def test_inference_stop_during_native_admission_is_not_data_failure(self):
        import numpy as np
        from robot_workbench.motion_reference import prepare_reference
        self.create();self.runtime.handoff(self.creds['calibration'],'inference')
        q=np.zeros((61,14));q[:,0]=np.linspace(0.,.02,61)
        compiled=prepare_reference(dict(device_targets14=q.tolist(),model_id='cpu',calibration_profile_digest='c'*64),
            [0.]*14,np.asarray(self.raw.get_bimanual_joint_position_limits()),source_sha256='a'*64)
        shared.inference_fixture.InferenceOwnerTests.configure(self,duration_s=2,prepared_reference=compiled)
        review=self.runtime.submit(self.creds['inference'],dict(action='preview_inference',duration_s=2))
        self.confirm('inference',review);self.advance(80);self.assertIsNotNone(self.raw.window)
        original=self.raw.queue_reference;fired=[]
        def inject(packet):
            result=original(packet)
            if not fired:fired.append(True);self.runtime.request_stop()
            return result
        self.raw.queue_reference=inject;self.advance(30);state=self.runtime.snapshot(self.creds['inference'])
        self.assertTrue(fired);self.assertNotIn('FAULT',state['state']);self.assertEqual(state['motion']['status'],'STOPPED')
        self.assertTrue(state['stop_receipt']['stop_hold_return_confirmed']);self.assertIsNone(self.raw.window)

if __name__=='__main__':unittest.main()
