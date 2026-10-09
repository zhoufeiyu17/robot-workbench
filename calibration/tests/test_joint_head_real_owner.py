"""Offline owner state-machine checks; never imports CAN or starts a robot."""
import sys, time, unittest
import json, threading
from http.server import ThreadingHTTPServer
from urllib.request import Request, build_opener, ProxyHandler
from urllib.error import HTTPError
from enum import Enum
from types import SimpleNamespace, ModuleType
import numpy as np
from unittest.mock import patch
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'infra'))
from examples.scripts.debug.joint_head_real_owner import Owner, SimulatedBackend, handler, hardware_backend
from arx_env.joint_head_real_motion import feedback_to_command

class Safety:
    class TakeoverState(str, Enum):
        READY='READY'; STOPPED='STOPPED_DAMPING_HOLD'; MANUAL='MANUAL_TAKEOVER'; FAULT='FAULT_STOP_UNCONFIRMED'; RELEASED='RELEASED'
    class PedalTakeover:
        def __init__(self, backend, stop_damping_scale=1.): self.backend=backend; self.state=Safety.TakeoverState.READY; self.reason=''; self.auto_aborted=False
        def press(self): self.auto_aborted=True; self.backend.set_damping_hold(manual=self.state=='STOPPED_DAMPING_HOLD'); self.state=Safety.TakeoverState.MANUAL if self.state==Safety.TakeoverState.STOPPED else Safety.TakeoverState.STOPPED; return self.state
        def fault(self, reason): self.auto_aborted=True; self.reason=str(reason); self.state=Safety.TakeoverState.FAULT; self.backend.set_damping_hold(manual=False)
        def release(self): self.backend.close(); self.state=Safety.TakeoverState.RELEASED

class OwnerTest(unittest.TestCase):
    def setUp(self):
        self.now=10000.; self.clock=patch('time.monotonic', side_effect=lambda:self.now); self.clock.start(); self.addCleanup(self.clock.stop)
        self.events=[]; self.owner=Owner(SimulatedBackend(), Safety, event=lambda *a,**k:self.events.append((a,k)))
        self.owner.tick()
        for _ in range(10): self.advance(.04)
    def advance(self, dt):
        self.now += dt; self.owner.tick()

    def test_owner_passes_joint_tolerances_and_exposes_ordered_vector(self):
        from arx_env.joint_head_real_motion import make_plan
        state=self.owner.snapshot()
        self.assertEqual(state['arm_arrival_tolerance_rad'],.01)
        self.assertEqual(state['arm_arrival_tolerances_rad'],[.01]*12)
        self.owner.arm_tolerances=[.005+.003*i for i in range(12)]
        self.advance(.04)
        state=self.owner.snapshot()
        self.assertIsNone(state['arm_arrival_tolerance_rad'])
        self.assertEqual(state['arm_arrival_tolerances_rad'],self.owner.arm_tolerances)
        feedback=state['calibration_feedback']
        plan=make_plan('both',feedback['qpos'],feedback['qpos'],state['joint_limits'],kind='workbench',created_at_s=self.now)
        self.owner.start_motion(plan,feedback)
        self.assertEqual(self.owner.runner.arm_tolerances,tuple(self.owner.arm_tolerances))
        self.assertEqual(self.owner.runner.plan.q_target,tuple(feedback['qpos']))

    def test_arrival_cli_rejects_conflicts_and_invalid_vector_before_backend(self):
        import io
        from examples.scripts.debug.joint_head_real_owner import main
        common=['owner','--legacy-source','/unused','--output','/unused/log']
        for options in (['--arm-tolerance','.02','--arm-tolerances']+['.01']*12,
                        ['--arm-tolerances']+['.01']*11,
                        ['--arm-tolerances','nan']+['.01']*11):
            with patch('sys.argv',common+options),patch('sys.stderr',new_callable=io.StringIO), \
                 patch('examples.scripts.debug.joint_head_real_owner.legacy_modules') as backend:
                with self.assertRaises(SystemExit) as result:main()
                self.assertEqual(result.exception.code,2)
                backend.assert_not_called()

    def test_snapshot_declares_simulation_source(self):
        state=self.owner.snapshot()
        self.assertIs(state['simulation_only'],True)
        self.assertEqual(state['evidence_source'],'simulated_owner')

    def test_explicit_fake_capture_window_carries_source_boundary_and_ack_identity(self):
        self.owner.environment='fake'
        original=self.owner.backend.read_calibration_state
        def feedback():
            return {**original(),'clock_domain':'simulation','source_clock_id':'fixture-plant',
                    'source_time_ns':round((self.now-10000.)*1e9),'physics_step':round((self.now-10000.)*100)}
        with patch.object(self.owner.backend,'read_calibration_state',feedback):
            self.advance(.04)
            self.capture_sequence();self.wait_capture('sample-0')
            state=self.owner.snapshot();motion=state['motion'];sample=state['calibration_feedback']
            self.assertEqual(state['mode'],'simulation')
            self.assertEqual(motion['capture_ready_source_ns'],sample['source_time_ns'])
            self.assertEqual(motion['capture_ready_physics_step'],sample['physics_step'])
            self.assertEqual(motion['capture_source_clock_id'],'fixture-plant')
            self.assertNotEqual(motion['capture_ready_wall_ns'],motion['capture_ready_source_ns'])
            epoch=self.owner.epoch;self.capture_ack();self.advance(.04)
            receipt=self.owner.snapshot()['last_capture_ack']
            self.assertEqual(receipt['command_epoch'],epoch)
            self.assertEqual(receipt['source_clock_id'],'fixture-plant')
            self.assertEqual(receipt['capture_ready_source_ns'],motion['capture_ready_source_ns'])
            with patch.object(self.owner.backend,'read_calibration_state',lambda:{**feedback(),'source_clock_id':'restarted'}):
                with self.assertRaisesRegex(ValueError,'clock differs'):self.owner.read_feedback()

    def test_explicit_fake_capture_rejects_missing_source_clock(self):
        self.owner.environment='fake'
        self.capture_sequence()
        for _ in range(200):
            self.advance(.04)
            if self.owner.runner.status=='FAULT':break
        self.assertEqual(self.owner.runner.status,'FAULT')
        self.assertIn('source_clock_id',self.owner.runner.reason)
        self.assertIsNone(self.owner.capture_wait)

    def test_arrival_events_record_each_workbench_plan_once_without_jaws(self):
        preview=self.capture_sequence();self.wait_capture('sample-0')
        first=self.owner.snapshot()['arrival_events']
        self.assertEqual(len(first),1)
        event=first[0]
        self.assertEqual(event['status'],'success')
        self.assertEqual(event['sample_id'],'sample-0')
        self.assertEqual(event['command_epoch'],preview['command_epoch'])
        self.assertEqual(event['q_target_rad'],[.01]*12)
        self.assertEqual(event['q_actual_rad'],[.01]*12)
        for _ in range(5):self.advance(.04)
        self.assertEqual(self.owner.snapshot()['arrival_events'],first)
        self.capture_ack();self.advance(.04)
        self.owner.runner.tick=lambda *args: (_ for _ in ()).throw(ValueError('tracking test failure'))
        self.advance(.04)
        events=self.owner.snapshot()['arrival_events']
        self.assertEqual(events,first)
        self.assertIsNone(self.owner.arrival_pending)
        for _ in range(5):self.advance(.04)
        self.assertEqual(self.owner.snapshot()['arrival_events'],events)
        self.assertEqual(len([e for e in self.events if e[0]==('arrival',)]),1)

    def test_arrival_success_is_not_recounted_as_capture_failure(self):
        self.capture_sequence();self.wait_capture('sample-0')
        first=self.owner.snapshot()['arrival_events']
        for _ in range(260):self.advance(.04)
        self.assertEqual(self.owner.runner.status,'FAULT')
        self.assertEqual(self.owner.snapshot()['arrival_events'],first)

    def test_stopped_waypoint_does_not_emit_arrival(self):
        self.capture_sequence()
        self.owner.stop('operator stopped before arrival')
        for _ in range(5):self.advance(.04)
        self.assertEqual(self.owner.snapshot()['arrival_events'],[])
        self.assertIsNone(self.owner.arrival_pending)

    def test_manual_plan_does_not_emit_arrival_event(self):
        preview=self.owner.submit({'action':'preview_home','active_arm':'left'})
        self.confirm(preview['token']);self.owner.tick()
        for _ in range(200):self.advance(.04)
        self.assertEqual(self.owner.snapshot()['arrival_events'],[])

    def capture_sequence(self):
        rows=[{'q':[v]*12,'speeds':[.1]*12,'capture':True,'sample_id':s}
              for v,s in ((.01,'sample-0'),(.02,'sample-1'))]
        preview=self.owner.submit({'action':'preview_workbench','waypoints':rows})
        self.assertEqual(preview['capture_samples'],['sample-0','sample-1'])
        self.confirm(preview['token']);self.owner.tick()
        return preview

    def wait_capture(self, sample):
        for _ in range(200):
            self.advance(.04)
            if self.owner.snapshot()['motion']['status']=='AWAITING_CAPTURE':break
        self.assertEqual(self.owner.snapshot()['motion']['status'],'AWAITING_CAPTURE')
        self.assertEqual(self.owner.snapshot()['motion']['sample_id'],sample)

    def capture_ack(self, **changes):
        command=dict(action='capture_ack',owner_pid=self.owner.snapshot()['pid'],
                     command_epoch=self.owner.epoch,sample_id=self.owner.capture_sample,
                     capture_ready_wall_ns=(self.owner.capture_wait or {}).get('ready_wall_ns'))
        command.update(changes)
        return self.owner.submit(command)

    def test_capture_sequence_extra_hold_ack_and_final_sample(self):
        preview=self.capture_sequence()
        for _ in range(200):
            self.advance(.04)
            if self.owner.runner.status=='READY':break
        ready_at=self.now
        self.assertNotEqual(self.owner.snapshot()['motion']['status'],'AWAITING_CAPTURE')
        for _ in range(10):self.advance(.04)
        self.assertNotEqual(self.owner.snapshot()['motion']['status'],'AWAITING_CAPTURE')
        self.wait_capture('sample-0');self.assertGreaterEqual(self.now-ready_at,.5)
        begin=sum(e[0]=='begin' for e in self.owner.backend.events)
        for _ in range(50):self.advance(.04)
        self.assertEqual(sum(e[0]=='begin' for e in self.owner.backend.events),begin)
        self.assertAlmostEqual(self.owner.backend.q[0],.01)
        for changes in ({'sample_id':'wrong'},{'owner_pid':-1},{'command_epoch':preview['command_epoch']+1}):
            with self.assertRaises(ValueError):self.capture_ack(**changes)
        self.capture_ack()
        with self.assertRaises(ValueError):self.capture_ack()
        self.advance(.04);self.wait_capture('sample-1')
        self.assertFalse(self.owner.sequence_remaining)
        self.assertTrue(self.owner.snapshot()['automatic_enabled'])
        self.capture_ack();self.advance(.04)
        self.assertEqual(self.owner.snapshot()['motion']['status'],'READY')
        self.assertFalse(self.owner.snapshot()['automatic_enabled'])
        self.assertEqual(self.owner.epoch,preview['command_epoch'])
        with self.assertRaises(ValueError):self.capture_ack(sample_id='sample-1')

    def test_capture_timeout_stops_hold_and_never_starts_next_segment(self):
        self.capture_sequence();self.wait_capture('sample-0')
        begin=sum(e[0]=='begin' for e in self.owner.backend.events)
        for _ in range(260):self.advance(.04)
        self.assertEqual(self.owner.snapshot()['motion']['status'],'FAULT')
        self.assertIn('timeout',self.owner.runner.reason)
        self.assertEqual(self.owner.control.state,Safety.TakeoverState.FAULT)
        self.assertFalse(self.owner.sequence_remaining)
        self.assertEqual(sum(e[0]=='begin' for e in self.owner.backend.events),begin)
        with self.assertRaises(ValueError):self.capture_ack(sample_id='sample-0')

    def test_capture_instability_recovers_and_rejects_revoked_ack(self):
        self.capture_sequence();self.wait_capture('sample-0')
        old_window=self.owner.capture_wait['ready_wall_ns']
        self.capture_ack()
        with patch.object(self.owner.stationary_window,'update',return_value=False):self.advance(.01)
        deadline=self.owner.capture_recovery_deadline
        self.assertAlmostEqual(deadline,self.now+2.)
        self.assertIsNone(self.owner.capture_wait)
        self.assertIsNone(self.owner.last_capture_ack)
        self.assertEqual(self.owner.capture_sample,'sample-0')
        self.wait_capture('sample-0')
        self.assertEqual(self.owner.capture_recovery_deadline,deadline)
        self.assertGreater(self.owner.capture_wait['ready_wall_ns'],old_window)
        with self.assertRaises(ValueError):self.capture_ack(capture_ready_wall_ns=old_window)
        window=self.owner.capture_wait['ready_wall_ns']
        self.capture_ack();self.advance(.01)
        self.assertEqual(self.owner.snapshot()['last_capture_ack'],dict(sample_id='sample-0',capture_ready_wall_ns=window))
        self.assertIsNone(self.owner.capture_recovery_deadline)

    def test_capture_recovery_deadline_survives_new_windows_and_freezes_fault(self):
        self.capture_sequence();self.wait_capture('sample-0')
        old_epoch=self.owner.epoch
        for attempt in range(2):
            with patch.object(self.owner.stationary_window,'update',return_value=False):self.advance(.01)
            if attempt==0:deadline=self.owner.capture_recovery_deadline
            self.assertEqual(self.owner.capture_recovery_deadline,deadline)
            self.wait_capture('sample-0')
        while self.now<deadline:self.advance(.01)
        self.assertEqual(self.owner.control.state,Safety.TakeoverState.FAULT)
        diagnostic=self.owner.snapshot()['last_fault_diagnostic']
        self.assertEqual(diagnostic['command_epoch'],old_epoch)
        self.assertGreater(self.owner.epoch,old_epoch)
        self.assertEqual(diagnostic['sample_id'],'sample-0')
        self.assertIn('capture_recovery_timeout',diagnostic['reasons'])
        self.assertEqual(diagnostic['capture_recovery_deadline_monotonic_s'],deadline)
        records=[fields for (kind,),fields in self.events if kind=='fault_diagnostic']
        self.assertEqual(len(records),1)
        self.assertLessEqual(len(records[0]['history']),200)
        frozen=json.dumps(diagnostic,allow_nan=False)
        self.advance(.01)
        self.assertEqual(json.dumps(self.owner.snapshot()['last_fault_diagnostic'],allow_nan=False),frozen)

    def test_capture_velocity_spike_is_diagnostic_only(self):
        self.capture_sequence();self.wait_capture('sample-0')
        read=self.owner.backend.read_calibration_state
        window=self.owner.capture_wait.copy()
        def spike():
            value=read();value['qvel'][2]=.075;return value
        with patch.object(self.owner.backend,'read_calibration_state',side_effect=spike):self.advance(.05)
        self.assertEqual(self.owner.runner.status,'READY')
        self.assertEqual(self.owner.capture_wait,window)
        self.assertIsNone(self.owner.capture_recovery_deadline)
        self.assertEqual(self.owner.stationary_window.last_diagnostic['velocity_exceeded_indices'],[2])
        self.assertFalse(any(kind=='capture_window_revoked' for (kind,),fields in self.events))
        self.capture_ack();self.advance(.01)
        self.assertEqual(self.owner.last_capture_ack['sample_id'],'sample-0')

    def test_capture_recovery_target_drift_still_faults(self):
        self.capture_sequence();self.wait_capture('sample-0')
        with patch.object(self.owner.stationary_window,'update',return_value=False):self.advance(.01)
        self.assertIsNotNone(self.owner.capture_recovery_deadline)
        self.owner.backend.q[0]+=.011
        self.advance(.01)
        self.assertIn('target_outside_arrival_tolerance',self.owner.runner.reason)
        self.assertEqual(self.owner.control.state,Safety.TakeoverState.FAULT)
        self.assertIsNone(self.owner.last_capture_ack)

    def test_capture_recovery_unhealthy_feedback_still_faults(self):
        self.capture_sequence();self.wait_capture('sample-0')
        with patch.object(self.owner.stationary_window,'update',return_value=False):self.advance(.01)
        self.assertIsNotNone(self.owner.capture_recovery_deadline)
        with patch.object(self.owner.backend,'read_calibration_state',side_effect=ValueError('feedback disconnected')):self.advance(.01)
        self.assertEqual(self.owner.control.state,Safety.TakeoverState.FAULT)
        self.assertIsNone(self.owner.last_capture_ack)

    def test_capture_boundary_events_include_stationary_evidence(self):
        self.capture_sequence();self.wait_capture('sample-0');self.capture_ack();self.advance(.01)
        for event in ('awaiting_capture','capture_acknowledged'):
            rows=[fields for (kind,),fields in self.events if kind==event]
            self.assertTrue(rows)
            self.assertTrue(rows[0]['diagnostic']['stationary'])
            self.assertEqual(rows[0]['diagnostic']['sample_id'],'sample-0')

    def test_capture_ack_then_stop_or_feedback_failure_never_continues(self):
        self.capture_sequence();self.wait_capture('sample-0')
        begin=sum(e[0]=='begin' for e in self.owner.backend.events)
        self.capture_ack();self.owner.submit({'action':'stop'});self.advance(.04)
        self.assertFalse(self.owner.sequence_remaining)
        self.assertEqual(sum(e[0]=='begin' for e in self.owner.backend.events),begin)
        for _ in range(12):self.advance(.04)
        self.capture_sequence();self.wait_capture('sample-0')
        begin=sum(e[0]=='begin' for e in self.owner.backend.events)
        self.capture_ack()
        with patch.object(self.owner.backend,'read_calibration_state',side_effect=ValueError('feedback disconnected')):
            self.advance(.04)
        self.assertEqual(self.owner.snapshot()['motion']['status'],'FAULT')
        self.assertFalse(self.owner.sequence_remaining)
        self.assertEqual(sum(e[0]=='begin' for e in self.owner.backend.events),begin)

    def test_capture_metadata_must_be_explicit_unique_and_frozen(self):
        base={'q':[.01]*12,'speeds':[.1]*12}
        for row in ({**base,'capture':1},{**base,'capture':True},{**base,'capture':True,'sample_id':''},
                    {**base,'capture':False,'sample_id':'x'}):
            with self.assertRaises(ValueError):self.owner.submit({'action':'preview_workbench','waypoints':[row]})
        row={**base,'capture':True,'sample_id':'one'}
        with self.assertRaises(ValueError):self.owner.submit({'action':'preview_workbench','waypoints':[row,row]})
        preview=self.owner.submit({'action':'preview_workbench','waypoints':[row]})
        row['sample_id']='changed-after-review'
        self.confirm(preview['token']);self.owner.tick();self.wait_capture('one')
        with self.assertRaises(ValueError):self.preview()

    def test_capture_ack_does_not_override_new_pose_drift(self):
        self.capture_sequence();self.wait_capture('sample-0')
        begin=sum(e[0]=='begin' for e in self.owner.backend.events)
        self.capture_ack();self.owner.backend.q[0]+=.03
        self.advance(.04)
        self.assertEqual(self.owner.snapshot()['motion']['status'],'FAULT')
        self.assertFalse(self.owner.sequence_remaining)
        self.assertEqual(sum(e[0]=='begin' for e in self.owner.backend.events),begin)

    def test_capture_ready_time_is_published_and_conditional_stop_is_atomic(self):
        self.capture_sequence()
        with patch('time.time_ns',return_value=123456789):self.wait_capture('sample-0')
        self.assertEqual(self.owner.snapshot()['motion']['capture_ready_wall_ns'],123456789)
        epoch=self.owner.epoch;pid=self.owner.snapshot()['pid']
        for command in ({'owner_pid':pid+1,'command_epoch':epoch},{'owner_pid':pid,'command_epoch':epoch+1}):
            with self.assertRaises(ValueError):self.owner.submit({'action':'stop',**command})
            self.assertFalse(self.owner.stop_requested);self.assertEqual(self.owner.epoch,epoch)
        self.owner.submit({'action':'stop','owner_pid':pid,'command_epoch':epoch})
        self.assertTrue(self.owner.stop_requested)
        self.owner.tick();self.assertFalse(self.owner.sequence_remaining)
        with self.assertRaises(ValueError):self.owner.submit({'action':'stop','owner_pid':pid,'command_epoch':epoch})
        self.owner.submit({'action':'stop'});self.assertTrue(self.owner.stop_requested)

    def test_capture_hold_restarts_after_stability_is_interrupted(self):
        self.capture_sequence()
        for _ in range(200):
            self.advance(.04)
            if self.owner.runner.status=='READY':break
        for _ in range(8):self.advance(.04)
        with patch.object(self.owner.stationary_window,'update',return_value=False):self.advance(.04)
        self.assertIsNone(self.owner.capture_stable_since)
        for _ in range(10):self.advance(.04)
        self.assertNotEqual(self.owner.snapshot()['motion']['status'],'AWAITING_CAPTURE')
        self.wait_capture('sample-0')
    def test_workbench_zero_pose_uses_home_bound_and_preserves_jaws(self):
        self.owner.backend.q = [.2]*14
        self.owner.backend.q[6] = .04
        self.owner.backend.q[13] = .03
        for _ in range(12):self.advance(.04)
        p=self.owner.submit({'action':'preview_workbench','waypoints':[{'q':[0.]*12,'speeds':[.2]*12}]})
        self.assertEqual(p['plan']['kind'],'workbench')
        expected=feedback_to_command(np.array([0.]*6+[.04]+[0.]*6+[.03]))
        self.assertAlmostEqual(p['plan']['q_target'][6],expected[6],places=12)
        self.assertAlmostEqual(p['plan']['q_target'][13],expected[13],places=12)
        self.owner.backend.q[0]=.7
        for _ in range(12):self.advance(.04)
        self.assertTrue(self.owner.submit({'action':'preview_workbench','waypoints':[{'q':[0.]*12,'speeds':[.2]*12}]}))

    def test_workbench_sequence_both_arms_fixed_jaws_and_stop(self):
        waypoints = [{'q':[value]*12, 'speeds':[.1]*12} for value in (.01,.02)]
        preview = self.owner.submit({'action':'preview_workbench','waypoints':waypoints})
        self.assertEqual(len(preview['plans']),2)
        for plan in preview['plans']:
            self.assertEqual(plan['active_arm'],'both')
            self.assertEqual(plan['q_start'][6],plan['q_target'][6])
            self.assertEqual(plan['q_start'][13],plan['q_target'][13])
        self.confirm(preview['token']); self.owner.tick()
        for _ in range(250):self.advance(.04)
        self.assertEqual(self.owner.runner.status,'READY')
        self.assertFalse(self.owner.sequence_remaining)
        self.assertAlmostEqual(self.owner.backend.q[0],.02)
        self.assertAlmostEqual(self.owner.backend.q[7],.02)
        bad = [{'q':[.1]*12,'speeds':[.1]*12},{'q':[.2]*12,'speeds':[.1]*12}]
        self.assertTrue(self.owner.submit({'action':'preview_workbench','waypoints':bad}))
        bad = [{'q':[.02]*12,'speeds':[.21]*12}]
        self.assertTrue(self.owner.submit({'action':'preview_workbench','waypoints':bad}))
        p=self.owner.submit({'action':'preview_workbench','waypoints':waypoints})
        self.confirm(p['token']);self.owner.tick()
        self.owner.submit({'action':'stop'});self.owner.tick()
        self.assertFalse(self.owner.sequence_remaining)
        for _ in range(100):self.advance(.04)
        self.assertNotEqual(self.owner.runner.status,'RUNNING')

    def test_preview_confirm_single_arm_and_advance(self):
        s=self.owner.snapshot(); target=s['calibration_feedback']['qpos'][:]; target[7]=.01
        preview=self.owner.submit({'action':'preview','active_arm':'right','q_target':target})
        self.assertEqual(preview['owner_pid'], s['pid'])
        self.assertEqual(self.owner.submit({'action':'confirm','token':preview['token'],'confirmation':'START_REVIEWED_SINGLE_ARM','physical_clearance':True})['queued'],'confirm')
        self.owner.tick(); self.assertEqual(self.owner.snapshot()['motion']['status'],'VERIFYING')
        for _ in range(10): self.advance(.04)
        self.assertTrue(any(e[0][0]=='motion_confirmed' for e in self.events))
        self.assertIn(self.owner.runner.status, ('RUNNING','VERIFYING'))
    def test_sdk_jaw_target_and_passive_rejection(self):
        self.owner.backend.passive_jaws = ('left',)
        p = self.owner.submit({'action':'preview_jaw_target','active_arm':'right','sdk_gap_mm':77.7})
        self.assertEqual(p['plan']['kind'], 'jaw_target')
        self.assertAlmostEqual(p['plan']['q_target'][13], .0777)
        self.assertEqual(p['plan']['q_target'][:13], p['plan']['q_start'][:13])
        self.assertEqual(p['plan']['jaw_speed_mm_s'], 10.)
        with self.assertRaisesRegex(ValueError, 'passive'):
            self.owner.submit({'action':'preview_jaw_target','active_arm':'left','sdk_gap_mm':40.})
        with self.assertRaises(ValueError):
            self.owner.submit({'action':'preview_jaw_target','active_arm':'right','sdk_gap_mm':83.})
        self.confirm(p['token']); self.owner.tick()
        for _ in range(1000):
            self.advance(.04)
            if self.owner.runner.status == 'READY': break
        self.assertEqual(self.owner.runner.status, 'READY')
        self.assertAlmostEqual(self.owner.snapshot()['calibration_feedback']['qpos'][13], .0777)

    def test_stop_invalidates_preview_and_replay(self):
        s=self.owner.snapshot(); target=s['calibration_feedback']['qpos'][:]; target[7]=.01
        p=self.owner.submit({'action':'preview','active_arm':'right','q_target':target})
        self.owner.submit({'action':'stop'}); self.owner.tick()
        with self.assertRaises(ValueError): self.owner.submit({'action':'confirm','token':p['token'],'confirmation':'START_REVIEWED_SINGLE_ARM','physical_clearance':True})
        self.assertEqual(self.owner.snapshot()['state'],'STOPPED_DAMPING_HOLD')
    def test_rejects_jaw_and_inactive_changes(self):
        s=self.owner.snapshot(); q=s['calibration_feedback']['qpos'][:]
        for i in (0,6,13):
            target=q[:]; target[i]+=.001
            with self.assertRaises(ValueError): self.owner.submit({'action':'preview','active_arm':'right','q_target':target})
    def test_second_stop_manual_never_auto_resumes(self):
        self.owner.submit({'action':'simulate_pedal'}); self.owner.tick(); self.assertEqual(self.owner.snapshot()['state'],'STOPPED_DAMPING_HOLD')
        self.owner.submit({'action':'simulate_pedal'}); self.owner.tick(); self.assertEqual(self.owner.snapshot()['state'],'MANUAL_TAKEOVER')

    def preview(self):
        q=self.owner.snapshot()['calibration_feedback']['qpos'][:]; q[7]+=.01
        return self.owner.submit({'action':'preview','active_arm':'right','q_target':q})
    def confirm(self, token):
        return self.owner.submit({'action':'confirm','token':token,'confirmation':'START_REVIEWED_SINGLE_ARM','physical_clearance':True})
    def test_expired_replay_stop_priority_and_explicit_rearm(self):
        p=self.preview(); self.now+=31
        with self.assertRaises(ValueError): self.confirm(p['token'])
        self.owner.tick()
        for _ in range(10): self.advance(.04)
        p=self.preview(); self.confirm(p['token'])
        with self.assertRaises(ValueError): self.confirm(p['token'])
        self.owner.submit({'action':'stop'}); self.owner.tick()
        self.assertFalse(any(e[0]=='begin' for e in self.owner.backend.events))
        self.assertEqual(self.owner.snapshot()['state'],'STOPPED_DAMPING_HOLD')
        for _ in range(10): self.advance(.04)
        p=self.preview(); self.confirm(p['token']); self.owner.tick()
        for _ in range(10): self.advance(.04)
        self.assertEqual(self.owner.runner.status,'RUNNING')
        self.assertEqual(sum(e[0]=='begin' for e in self.owner.backend.events),1)
    def test_runtime_fault_requires_diagnosis(self):
        p=self.preview(); self.confirm(p['token']); self.owner.tick()
        self.advance(.2)
        self.assertEqual(self.owner.snapshot()['state'],'FAULT_STOP_UNCONFIRMED')
        with self.assertRaises(ValueError): self.preview()
        self.owner.submit({'action':'stop'}); self.owner.tick()
        self.assertEqual(self.owner.snapshot()['state'],'FAULT_STOP_UNCONFIRMED')
        with self.assertRaises(ValueError): self.preview()
    def test_acknowledge_only_motion_fault_without_sdk_write_or_resume(self):
        from arx_env.joint_head_real_motion import MotionTrackingError
        p=self.preview()
        self.owner.motion_fault(MotionTrackingError('tracking failed'))
        self.advance(.04)
        def recover():
            return self.owner.submit(dict(action='recover', confirmation='ACKNOWLEDGE_MOTION_FAULT',
                                          owner_pid=self.owner.snapshot()['pid'], command_epoch=self.owner.epoch))
        count=len(self.owner.backend.events)
        recover(); self.advance(.04)
        self.assertEqual(len(self.owner.backend.events),count)
        self.assertEqual(self.owner.snapshot()['state'],'STOPPED_DAMPING_HOLD')
        self.assertIsNone(self.owner.runner)
        with self.assertRaises(ValueError): self.confirm(p['token'])
        self.preview()
        self.owner.motion_fault(RuntimeError('SDK failure')); self.advance(.04)
        with self.assertRaises(ValueError): recover()
        self.owner.motion_fault(MotionTrackingError('tracking failed')); self.advance(.04)
        recover(); self.owner.submit({'action':'stop'}); self.owner.tick()
        self.assertEqual(self.owner.snapshot()['state'],'FAULT_STOP_UNCONFIRMED')

    def test_http_offline_gate(self):
        server=ThreadingHTTPServer(('127.0.0.1',0),handler(self.owner))
        thread=threading.Thread(target=server.serve_forever,daemon=True); thread.start()
        base=f'http://127.0.0.1:{server.server_port}'
        http=build_opener(ProxyHandler({}))
        def post(payload,origin=base):
            req=Request(base+'/operator',json.dumps(payload).encode(),headers={'Content-Type':'application/json','Origin':origin})
            with http.open(req,timeout=.8) as response: return json.load(response)
        try:
            with http.open(base+'/state',timeout=.8) as response: state=json.load(response)
            self.assertEqual(len(state['joint_limits']),14)
            with self.assertRaises(HTTPError): post({'action':'stop'},'https://attacker.invalid')
            q=state['calibration_feedback']['qpos'][:]; q[7]=.01
            p=post({'action':'preview','active_arm':'right','q_target':q})
            post({'action':'confirm','token':p['token'],'confirmation':'START_REVIEWED_SINGLE_ARM','physical_clearance':True})
            post({'action':'stop'}); self.owner.tick()
            self.assertFalse(any(e[0]=='begin' for e in self.owner.backend.events))
        finally:
            server.shutdown(); server.server_close(); thread.join()

    def test_hardware_adapter_only_selected_arm_measured_hold_and_guard(self):
        class JointState:
            def __init__(self,n):
                self.p=np.zeros(n); self.v=np.zeros(n); self.t=np.zeros(n)
                self.gripper_pos=.02
            def pos(self): return self.p
            def vel(self): return self.v
            def torque(self): return self.t
        class Gain:
            def __init__(self,n): self.p=np.zeros(n); self.d=np.zeros(n)
            def kp(self): return self.p
            def kd(self): return self.d
        class Controller:
            def __init__(self): self.state=JointState(6); self.calls=[]
            def get_joint_state(self): return self.state
            def set_joint_cmd(self,cmd): self.calls.append(('cmd',cmd))
            def set_gain(self,gain): self.calls.append(('gain',gain))
        class Base:
            def __init__(self,config):
                self.sides=('left','right'); self.config=config; config.gravity_compensation=True; config.teach_kd_scale=.1; config.gripper_width_m=.082
                self._controllers={x:Controller() for x in ('left','right')}
                self._robot_configs={x:SimpleNamespace(robot_model='X5') for x in ('left','right')}
                cfg=SimpleNamespace(default_kp=np.array([80,70,70,30,30,20]),default_kd=np.array([2,2,2,1,1,.7]),default_gripper_kp=5.,default_gripper_kd=float(np.float32(.2)))
                self._controller_configs={x:cfg for x in ('left','right')}
                self._arx5=SimpleNamespace(Gain=Gain,JointState=JointState)
            def set_damping_hold(self,manual=False,**kwargs):
                for controller in self._controllers.values():
                    gain=Gain(6);gain.kd()[:]=.2 if manual else 2
                    gain.gripper_kp=5.;gain.gripper_kd=.2
                    controller.set_gain(gain)
            def _require_connected(self): pass
            def get_joint_position_limits(self,side): return np.array([[-3.,3.]]*6+[[0.,.082]])
            def get_bimanual_joint_position_limits(self):
                return np.concatenate([self.get_joint_position_limits(side) for side in self.sides])
        module=ModuleType('arx_env.backends.arx5_backend'); module.Arx5Backend=Base; module.Arx5BackendConfig=lambda **kw:SimpleNamespace(**kw)
        with patch.dict(sys.modules, {'arx_env.backends.arx5_backend':module}):
            b=hardware_backend()
        b.command_guard=lambda:None
        q=[0.]*14; q[6]=q[13]=.02
        b._controllers['left'].state.p[0]=.0001
        exact=b.begin('left',q)
        self.assertEqual(b._controllers['right'].calls,[])
        self.assertEqual([c[0] for c in b._controllers['left'].calls],['cmd','gain'])
        self.assertEqual(b._controllers['left'].calls[0][1].pos()[0],.0001)
        from arx_env.joint_head_jaw_mapping import command_sdk, feedback_gaps
        self.assertEqual(exact[6],.02, 'resampled feedback remains in the feedback domain')
        self.assertAlmostEqual(b._controllers['left'].calls[0][1].gripper_pos,
                               command_sdk('left',feedback_gaps(q)[0]))
        b.set_damping_hold(manual=True)
        for controller in b._controllers.values():
            gain=controller.calls[-1][1]
            self.assertEqual((gain.gripper_kp,gain.gripper_kd),(0.,0.))
            np.testing.assert_allclose(gain.kd(),.2)
        b.set_damping_hold(manual=False)
        self.assertEqual(b._controllers['right'].calls[-1][1].gripper_kp,.12)
        self.assertEqual(b._controllers['right'].calls[-1][1].gripper_kd,.15)
        np.testing.assert_allclose(b._controllers['right'].calls[-1][1].kd(),[1.2,1.2,1.2,1.1,1.1,1.07])
        count=len(b._controllers['left'].calls)
        def blocked(): raise RuntimeError('stop')
        b.command_guard=blocked
        with self.assertRaises(RuntimeError): b.send('left',q)
        self.assertEqual(len(b._controllers['left'].calls),count)

    def test_fault_pedal_releases_jaws_without_clearing_fault(self):
        self.owner.control.state=Safety.TakeoverState.FAULT
        self.owner.control.reason='jaw drift'
        self.owner.pedal()
        self.assertTrue(self.owner.fault_manual)
        self.assertEqual(self.owner.control.state,Safety.TakeoverState.FAULT)
        self.assertEqual(self.owner.control.reason,'jaw drift')
        self.assertTrue(self.owner.control.auto_aborted)
        self.owner.pedal()
        self.assertFalse(self.owner.fault_manual)

    def test_pedal_race_never_clears_latch_or_resumes(self):
        class Listener:
            def __init__(self):
                self._stop_lock=threading.Lock(); self._last_press_ns=None; self._key_down=False
                self.input_healthy=True; self.fault_reason=None; self.requested=threading.Event(); self.presses=[]
            def take_presses(self):
                result=tuple(self.presses); self.presses.clear(); return result
            def press(self,n):
                self._last_press_ns=n; self._key_down=True; self.presses.append(n); self.requested.set()
        listener=Listener(); self.owner.listener=listener
        p=self.preview(); self.confirm(p['token']); self.owner.tick()
        for _ in range(10): self.advance(.04)
        self.assertEqual(self.owner.runner.status,'RUNNING')
        listener.press(1)
        # Model an event arriving between the tick's drain and command callback.
        with self.assertRaises(RuntimeError): self.owner.send('right',self.owner.backend.q)
        self.owner.motion_fault('callback gate')
        self.assertEqual(self.owner.control.state,'STOPPED_DAMPING_HOLD')
        listener._key_down=False
        listener.press(2); self.owner.tick()
        self.assertEqual(self.owner.control.state,'MANUAL_TAKEOVER')
        self.assertTrue(listener.requested.is_set())
        self.assertEqual(self.owner.runner.status,'STOPPED')
        with self.assertRaises(ValueError): self.preview()
        listener._key_down=False
        for _ in range(10): self.advance(.04)
        self.assertEqual(self.owner.control.state,'MANUAL_TAKEOVER')
        self.assertTrue(listener.requested.is_set())
    def test_home_preview_is_frozen_single_arm_and_opens_selected_jaw(self):
        self.owner.backend.q=[.12,0.,0.,0.,0.,0.,.023,.353,0.,0.,0.,0.,0.,.0078]
        for _ in range(10): self.advance(.04)
        p=self.owner.submit({'action':'preview_home','active_arm':'right'})
        self.assertEqual(p['plan']['q_target'][7:13],(0.,)*6)
        self.assertEqual(p['plan']['q_target'][:7],tuple(self.owner.backend.q[:7]))
        self.assertEqual(p['plan']['q_target'][13],.0773)
        self.assertEqual(p['plan']['kind'],'home')
        self.assertEqual(p['plan']['max_velocity'],.2)
        self.assertGreater(p['plan']['duration_s'],0.)
        self.assertFalse(any(e[0]=='begin' for e in self.owner.backend.events))


    def test_hardware_full_open_preview_and_jaw_probe(self):
        self.owner.hardware=True
        home=self.owner.submit({'action':'preview_home','active_arm':'right'})
        self.assertEqual(home['plan']['q_target'][13],.0773)
        self.assertFalse(self.owner.backend.events)
        with self.assertRaises(ValueError):
            self.owner.submit({'action':'preview_jaw_probe','active_arm':'right'})


    def test_both_home_same_tick_common_duration_and_no_replay(self):
        self.owner.backend.q=[.1,0.,0.,0.,0.,0.,.075,-.1,0.,0.,0.,0.,0.,.075]
        for _ in range(10): self.advance(.04)
        p=self.owner.submit({'action':'preview_both_home'})
        self.assertEqual(len(p['plans']),1)
        self.assertEqual(p['plan']['active_arm'],'both')
        self.assertEqual(p['plan']['kind'],'both_home')
        self.assertFalse(self.owner.backend.events)
        self.confirm(p['token']); self.owner.tick()
        for _ in range(250): self.advance(.04)
        self.assertEqual(self.owner.runner.status,'READY')
        commands=self.owner.backend.events
        self.assertEqual([e[1] for e in commands if e[0]=='begin'],['left','right'])
        sends=[e for e in commands if e[0]=='send']
        for left,right in zip(sends[::2],sends[1::2]):
            self.assertEqual((left[1],right[1]),('left','right'))
            self.assertEqual(left[2],right[2])
        for i,v in enumerate(self.owner.backend.q): self.assertAlmostEqual(v,(.0788 if i==6 else .0773) if i in (6,13) else 0.)
        with self.assertRaises(ValueError): self.confirm(p['token'])

    def test_both_home_stop_fault_and_between_arm_write_race(self):
        p=self.owner.submit({'action':'preview_both_home'})
        self.confirm(p['token']); self.owner.tick()
        for _ in range(12): self.advance(.04)
        original=self.owner.backend.send
        def stop_after_left(side,q):
            original(side,q)
            if side=='left': self.owner.submit({'action':'stop'})
        self.owner.backend.send=stop_after_left
        count=len(self.owner.backend.events)
        self.advance(.04)
        self.assertEqual([e[1] for e in self.owner.backend.events[count:] if e[0]=='send'],['left'])
        self.assertFalse(self.owner.snapshot()['automatic_enabled'])
        self.assertIn(self.owner.runner.status,('STOPPED','FAULT'))
        count=len([e for e in self.owner.backend.events if e[0]=='send'])
        for _ in range(100): self.advance(.04)
        self.assertEqual(len([e for e in self.owner.backend.events if e[0]=='send']),count)

    def test_both_home_long_duration_float32_jaws(self):
        self.owner.backend.get_bimanual_joint_position_limits=lambda: np.array([[0.,.082] if i in (6,13) else [-3.,3.] for i in range(14)],dtype=np.float32)
        self.advance(.04)
        p=self.owner.submit({'action':'preview_both_home'})
        self.assertGreater(p['plan']['duration_s'],0.)
        self.confirm(p['token']); self.owner.tick()
        for _ in range(900): self.advance(.04)
        self.assertEqual(self.owner.runner.status,'READY')
        self.assertAlmostEqual(self.owner.backend.q[6],.0788)
        self.assertAlmostEqual(self.owner.snapshot()['calibration_feedback']['qpos'][13],.0773)

    def test_signal_stop_while_owner_lock_held(self):
        import signal
        old=signal.signal(signal.SIGUSR1,lambda *_:self.owner.submit({'action':'stop'}))
        try:
            with self.owner.lock: signal.raise_signal(signal.SIGUSR1)
            self.owner.tick()
            self.assertEqual(self.owner.snapshot()['state'],'STOPPED_DAMPING_HOLD')
        finally: signal.signal(signal.SIGUSR1,old)

if __name__ == '__main__': unittest.main()
