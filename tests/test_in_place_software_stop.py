"""Explicit evidence-gated recovery of a legacy Stop/epoch misclassification."""
from copy import deepcopy
import threading,time,unittest
from unittest.mock import patch
import test_shared_target_routes as shared

class InPlaceSoftwareStopTest(unittest.TestCase):
    profile='normal_X5'
    setUp=shared.SharedTargetRoutesTest.setUp
    cleanup=shared.SharedTargetRoutesTest.cleanup
    create=shared.SharedTargetRoutesTest.create
    advance=shared.SharedTargetRoutesTest.advance
    confirm=shared.SharedTargetRoutesTest.confirm

    def legacy_fault(self):
        self.create();self.guard=dict(active_anomaly=False,recovery_required=False,event=None,
            worker_exit_confirmed=False,supervisor_exit_confirmed=False,heartbeat=dict(status='healthy',monitored=True))
        self.raw.protection_snapshot=lambda:deepcopy(self.guard)
        review=self.runtime.submit(self.creds['calibration'],dict(action='preview_workbench',waypoints=[dict(q12=[.15]+[0.]*11,speeds12=[.1]*12,jaw_gap_mm=[40.,40.])]))
        self.confirm('calibration',review);self.advance(60)
        original=self.raw.read_calibration_state;fired=[]
        def inject():
            result=original()
            if not fired:fired.append(True);self.runtime.request_stop()
            return result
        self.raw.read_calibration_state=inject
        with patch.object(self.runtime.owner,'cancel_requested_reference',return_value=False),patch('robot_workbench.target_interpreter.ReferenceRevokedError',ValueError):self.advance(12)
        self.raw.read_calibration_state=original
        self.assertEqual(self.runtime.owner.control.reason,'target reference epoch revoked')
        self.assertTrue(self.runtime._software_stop_recovery_ready())
        self.assertIsNone(self.runtime.owner.recoverable_fault,'eligibility reads must not clear or mutate fault')

    def test_explicit_recovery_keeps_connection_and_requires_new_review(self):
        self.check_explicit_recovery()

    def test_explicit_recovery_accepts_valid_jaw_feedback_above_command_bound(self):
        self.check_explicit_recovery(jaw_feedback=.082394)

    def check_explicit_recovery(self,jaw_feedback=None):
        self.legacy_fault();runtime=self.runtime;owner=runtime.owner;nonce=runtime.nonce;count=len(self.raw.packets)
        if jaw_feedback is not None:
            self.raw.q[13]=jaw_feedback
            self.advance(50)
            self.assertTrue(owner.recovery_ready())
        snapshot=runtime.snapshot(self.creds['calibration']);self.assertEqual(snapshot['recoverable_fault'],owner.control.reason)
        self.assertFalse(snapshot['allowed_operations']['preview']['allowed'])
        class Actor:
            action='recover_control'
            def require(self,value):assert value is runtime
        result=[]
        def recover():
            try:result.append(runtime.control_recover(Actor()))
            except Exception as e:result.append(e)
        with patch.object(self.raw,'close',side_effect=AssertionError('in-place recovery must not disconnect')):
            thread=threading.Thread(target=recover);thread.start()
            for _ in range(200):
                self.advance();time.sleep(.001)
                if not thread.is_alive():break
            thread.join(1)
        self.assertFalse(thread.is_alive());self.assertIsInstance(result[0],dict,result)
        self.assertTrue(result[0]['recovered']);self.assertFalse(result[0]['motion_resumed']);self.assertEqual(runtime.nonce,nonce)
        self.assertEqual(len(self.raw.packets),count);self.assertIsNone(self.raw.window);self.assertIsNone(owner.runner)
        self.assertEqual(owner.control.state,owner.states.STOPPED)
        if jaw_feedback is not None:self.assertEqual(self.raw.q[13],jaw_feedback)
        self.advance(60);self.assertEqual(len(self.raw.packets),count)
        review=runtime.submit(self.creds['calibration'],dict(action='preview_workbench',waypoints=[dict(q12=[.02]+[0.]*11,speeds12=[.1]*12,jaw_gap_mm=[40.,40.])]))
        self.confirm('calibration',review);self.advance(60);self.assertGreater(len(self.raw.packets),count)

    def test_unknown_or_changed_evidence_never_enables_recovery(self):
        self.legacy_fault();r=self.runtime;o=r.owner
        invalid=deepcopy(o.cache['calibration_feedback']);invalid['qpos'][13]=.095
        self.assertFalse(o.recovery_feedback_valid(invalid,o.cache['joint_limits']))
        with patch.object(o.control,'reason','unknown fault'):self.assertFalse(r._software_stop_recovery_ready())
        with patch.object(o,'stationary',False):self.assertFalse(r._software_stop_recovery_ready())
        with patch.object(o,'feedback_healthy',False):self.assertFalse(r._software_stop_recovery_ready())
        with patch.object(o,'stop_requested',True):self.assertFalse(r._software_stop_recovery_ready())
        with patch.object(o,'pedal_snapshot',return_value=(0,True,True)):self.assertFalse(r._software_stop_recovery_ready())
        receipt=deepcopy(r.stop_receipt);receipt['barrier_epoch']+=1
        with patch.object(r,'stop_receipt',receipt):self.assertFalse(r._software_stop_recovery_ready())
        calls=deepcopy(r.calls);calls[r.stop_receipt['in_flight_calls'][0]['id']]=dict(operation='read_calibration_state')
        with patch.object(r,'calls',calls):self.assertFalse(r._software_stop_recovery_ready())
        with patch.object(r,'hold_returns',dict(left=None,right=r.hold_returns['right'])):self.assertFalse(r._software_stop_recovery_ready())
        with patch.object(r,'hold_returns',{}):self.assertFalse(r._software_stop_recovery_ready())
        for values in [dict(active_anomaly=True),dict(event={'reason':'CAN fault'}),dict(heartbeat=dict(status='lost',monitored=True)),dict(worker_exit_confirmed=True)]:
            guard=deepcopy(self.guard);guard.update(values);self.assertFalse(r._software_stop_recovery_ready(guard))
        self.now+=.2;self.assertFalse(r._software_stop_recovery_ready())

    def test_physical_recovery_requires_fresh_armed_guard_in_feedback(self):
        self.legacy_fault();r=self.runtime
        stamp=max(row['returned_monotonic_s']for row in r.hold_returns.values())+.01
        r.owner.cache['calibration_feedback']['sample_time_s']=stamp
        with patch.object(r,'environment','physical'),patch.object(r.owner,'control_time',return_value=stamp):
            self.assertFalse(r._software_stop_recovery_ready())
            r.owner.cache['calibration_feedback']['can_output_guard']=dict(state='armed',heartbeat_monotonic_s=stamp)
            with patch.object(self.raw,'protection_snapshot',side_effect=AssertionError('no file/RPC read in Owner recovery gate')):
                self.assertTrue(r._software_stop_recovery_ready())
            r.owner.cache['calibration_feedback']['sample_time_s']=stamp-.02
            self.assertFalse(r._software_stop_recovery_ready(),'pre-Stop feedback cannot acknowledge a later hold')
            r.owner.cache['calibration_feedback']['sample_time_s']=stamp
            r.owner.cache['calibration_feedback']['can_output_guard']['state']='latched'
            self.assertFalse(r._software_stop_recovery_ready())
            r.owner.cache['calibration_feedback']['can_output_guard'].update(state='armed',heartbeat_monotonic_s=stamp-.2)
            self.assertFalse(r._software_stop_recovery_ready())

if __name__=='__main__':unittest.main()
