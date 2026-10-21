"""Actual Runtime/runners for all four sources, with point-send forbidden.

The plant is an ideal CPU protocol adapter, not a physical precision test.
"""
from copy import deepcopy
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'tests'),str(ROOT/'calibration'),str(ROOT/'calibration/infra'),
              str(ROOT/'calibration/tests'),str(ROOT/'rule_based_workbench')]
from robot_workbench.runtime import Runtime
from robot_workbench.target_interpreter import SCHEMA
from test_owner_clock import ClockBackend,Safety,calibration,jaw
import test_inference_owner as inference_fixture


class WindowBackend(ClockBackend):
    target_interpreter_profile='normal_X5'
    def __init__(self,clock,q=None):
        super().__init__(clock,q,source_clock_id='shared-interpreter-cpu-clock')
        self.packets=[];self.window=None;self.applied_command=None
        self.position_bias=np.zeros(14);self.control_thread=threading.get_ident()

    def queue_reference(self,packet):
        assert threading.get_ident()==self.control_thread,'model/data worker attempted device output'
        assert packet['schema']==SCHEMA,'source escaped shared interpreter'
        self.packets.append(deepcopy(packet));self.window=deepcopy(packet)
        command=self.applied_command[:]if self.applied_command is not None else self.q[:]
        if self.applied_command is None:
            command[6]=jaw.command_sdk('left',jaw.feedback_gaps(self.q)[0])
            command[13]=jaw.command_sdk('right',jaw.feedback_gaps(self.q)[1])
        self.window['host_times_s'].insert(0,self.clock())
        self.window['q_sdk14'].insert(0,command)
        return dict(firmware_ack=False,frames=len(packet['host_times_s']))

    def send(self,*_):
        raise AssertionError('ordinary motion bypassed the shared interpreter via old point-send')

    def read_calibration_state(self):
        if self.window is not None:
            times=self.window['host_times_s'];rows=np.asarray(self.window['q_sdk14'])
            command=[float(np.interp(self.clock(),times,rows[:,j]))for j in range(14)]
            self.applied_command=command[:]
            sides=('left','right')if self.window['active_arm']=='both'else(self.window['active_arm'],)
            for side in sides:
                start,end=(0,6)if side=='left'else(7,13)
                self.q[start:end]=[command[i]+self.position_bias[i]for i in range(start,end)]
                self.q[end]=jaw.feedback_sdk(side,jaw.command_gap_mm(side,command[end]))
        return super().read_calibration_state()

    def set_damping_hold(self,**kwargs):
        self.window=None
        return super().set_damping_hold(**kwargs)


class SharedTargetRoutesTest(unittest.TestCase):
    profile='normal_X5'
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.now=100.;self.events=[];self.instances=[];self.runtime=None
        jaw.configure_mapping(None);self.addCleanup(jaw.configure_mapping,None)
        self.addCleanup(self.cleanup)

    def create(self,q=None,mapping=None,binding=None):
        if mapping:jaw.configure_mapping(mapping)
        self.raw=WindowBackend(lambda:self.now,q)
        self.raw.target_interpreter_profile=self.profile
        self.runtime=Runtime(calibration,Safety,lambda:self.raw,device_id='shared-target-cpu',environment='fake',
            lock_root=self.temp.name,initialize_backend=lambda b:b.set_damping_hold(manual=True),
            control_clock=lambda:self.now,jaw_mapping_file=mapping,hardware_binding=binding,
            event=lambda name,**data:self.events.append((name,data)))
        self.creds={business:self.runtime.register_business(business)for business in ('calibration','collection','inference')}
        self.advance(42)

    def cleanup(self):
        if self.runtime is not None and self.runtime.fd is not None:
            self.runtime.owner.recording=None
            self.runtime.owner.submit(dict(action='release',confirmation='ARMS_SUPPORTED_RELEASE'))
            self.runtime.tick();self.runtime.close()

    def advance(self,count=1):
        for _ in range(count):
            runner=self.runtime.owner.runner
            if runner and getattr(runner,'runtime_started',False):runner.runtime_ready.wait(.05)
            if getattr(runner,'is_inference',False)and runner.loop._query_pending:time.sleep(.001)
            self.now+=.01;self.runtime.tick()

    def confirm(self,business,review):
        self.runtime.submit(self.creds[business],dict(action='confirm',token=review['token'],
            confirmation='START_REVIEWED_SINGLE_ARM',physical_clearance=True))

    def assert_shared(self):
        self.assertTrue(self.raw.packets)
        self.assertTrue(all(packet['schema']==SCHEMA for packet in self.raw.packets))
        self.assertIsNotNone(self.runtime.owner.runner.target_interpreter)
        state=self.runtime.snapshot(self.creds[self.runtime.holder])
        self.assertTrue(state['target_interpreter']['enabled'])
        self.assertEqual(state['target_interpreter']['controller_profile'],self.profile)

    def assert_stop_cancels(self):
        self.runtime.request_stop();self.advance(3);count=len(self.raw.packets)
        self.assertIsNone(self.raw.window);self.advance(100)
        self.assertEqual(len(self.raw.packets),count)

    def test_calibration_single_target_reaches_shared_executor_and_stop_cancels(self):
        self.create();q=self.runtime.owner.cache['calibration_feedback']['qpos'][:];q[0]=.02
        review=self.runtime.submit(self.creds['calibration'],dict(action='preview',active_arm='left',q_target=q))
        self.confirm('calibration',review);self.advance(230);self.assert_shared()
        self.assertEqual(self.runtime.owner.runner.status,'READY',self.runtime.owner.runner.reason)
        self.assert_stop_cancels()

    def test_manual_both_home_has_both_arms_in_the_same_window(self):
        q=[0.]*14;q[0]=.02;q[10]=-.02
        self.create(q=q)
        review=self.runtime.submit(self.creds['calibration'],dict(action='preview_both_home'))
        duration=review.get('total_motion_duration_s')or max(plan['duration_s']for plan in review['plans'])
        self.confirm('calibration',review);self.advance(round((duration+2)*100));self.assert_shared()
        self.assertEqual(self.runtime.owner.runner.status,'READY',self.runtime.owner.runner.reason)
        self.assertTrue(all(p['active_arm']=='both'for p in self.raw.packets))
        self.assertLess(abs(self.raw.q[0]),1e-5);self.assertLess(abs(self.raw.q[10]),1e-5)
        self.assert_stop_cancels()

    def test_continuous_start_retime_with_moving_jaw_completes_full_execution(self):
        q=[0.]*14;q[6]=jaw.feedback_sdk('left',1.)
        self.create(q=q);self.raw.passive_jaws=('right',)
        waypoint=dict(q12=[0.,0.,.2]+[0.]*9,speeds12=[.2]*12,jaw_gap_mm=[1.,0.])
        review=self.runtime.submit(self.creds['calibration'],dict(action='preview_workbench',waypoints=[waypoint]))
        self.confirm('calibration',review);self.advance(400)
        self.assertEqual(self.runtime.owner.runner.status,'READY',self.runtime.owner.runner.reason)
        previous=self.runtime.owner.runner.target_interpreter
        read=self.raw.read_calibration_state
        def noisy_feedback():
            state=read();state['qpos'][6]=jaw.feedback_sdk('left',jaw.feedback_gaps(state['qpos'])[0]+.2)
            return state
        self.raw.read_calibration_state=noisy_feedback;self.advance(40)
        waypoint.update(q12=[0.]*12,jaw_gap_mm=[83.,0.])
        review=self.runtime.submit(self.creds['calibration'],dict(action='preview_workbench',waypoints=[waypoint]))
        self.confirm('calibration',review);self.advance(50)
        runner=self.runtime.owner.runner
        self.assertIs(type(runner.execution_duration_s),float)
        self.assertGreater(runner.execution_duration_s,runner.plan.duration_s)
        self.assertEqual(runner.target_interpreter.compensation_parent,previous.reference.metadata['reference_id'])
        self.advance(round((runner.execution_duration_s+2)*100))
        self.assertEqual(runner.status,'READY',runner.reason)
        self.assertLess(abs(jaw.feedback_gaps(self.raw.q)[0]-83.),1e-6)
        self.assertLess(np.max(np.abs(np.asarray(self.raw.q)[[0,1,2,3,4,5,7,8,9,10,11,12]])),1e-6)
        self.assert_stop_cancels()

    def test_rule_collection_preserves_reference_knots_without_old_send(self):
        from test_collection_physx_protocol import protocol_fixture
        _,source,plan,_,mapping=protocol_fixture(Path(self.temp.name),source_clock_id='shared-interpreter-cpu-clock')
        jaw.configure_mapping(mapping);first=plan['ticks'][0]['target']
        q=[*first['left_arm_joints'],jaw.feedback_sdk('left',first['left_gripper']),
           *first['right_arm_joints'],jaw.feedback_sdk('right',first['right_gripper'])]
        self.create(q=q,mapping=mapping,binding=Path(self.temp.name)/'binding.json')
        self.runtime.handoff(self.creds['calibration'],'collection')
        review=self.runtime.submit(self.creds['collection'],dict(action='preview_frozen',source=source,plan=plan))
        self.confirm('collection',review)
        for _ in range(3000):
            self.advance()
            if self.runtime.owner.runner.status in ('READY','FAULT'):break
        runner=self.runtime.owner.runner;self.assert_shared()
        self.assertEqual(runner.status,'READY',runner.reason)
        self.assertEqual(runner.frozen.snapshot()['submission_semantics'],'source_reference_knot_evaluations')
        self.assertEqual(runner.shared_reference.metadata['timing_policy'],'strict_duration')
        self.assertFalse(any(name=='command'for name,_ in self.events))
        self.assertTrue(any(name=='source_target'for name,_ in self.events))
        self.assert_stop_cancels()

    def test_online_infer_uses_shared_chunks_and_only_completed_reference_history(self):
        self.create();self.runtime.handoff(self.creds['calibration'],'inference')
        inference_fixture.InferenceOwnerTests.configure(self,execute_steps=10)
        review=self.runtime.submit(self.creds['inference'],dict(action='preview_inference'))
        self.confirm('inference',review);self.advance(100)
        # The real preparation subprocess needs wall time; this accelerated
        # source-clock fixture must not consume the episode while it starts.
        runner=self.runtime.owner.runner;deadline=time.monotonic()+5
        while runner.loop._result.empty()and not runner.loop.predictions and time.monotonic()<deadline:time.sleep(.005)
        self.assertTrue(not runner.loop._result.empty()or runner.loop.predictions,'isolated CPU preparation did not return or get consumed')
        self.advance(400);self.assert_shared()
        runner=self.runtime.owner.runner
        self.assertEqual(runner.status,'RUNNING',runner.reason)
        self.assertGreater(runner.loop.completed,0)
        self.assertGreater(len(runner.loop.history),0)
        self.assertEqual(runner.target_interpreter.reference.metadata['kind'],'online_prediction_chunk')
        self.assertTrue(any(name=='inference_prepared_chunk'for name,_ in self.events))
        self.assert_stop_cancels()

    def test_fixed_prepared_infer_uses_the_identical_executor(self):
        from robot_workbench.motion_reference import prepare_reference
        self.create();self.runtime.handoff(self.creds['calibration'],'inference')
        q=np.zeros((61,14));q[:,0]=np.linspace(0.,.02,61)
        source=dict(device_targets14=q.tolist(),model_id='cpu',calibration_profile_digest='c'*64)
        limits=np.asarray(self.raw.get_bimanual_joint_position_limits())
        compiled=prepare_reference(source,[0.]*14,limits,source_sha256='a'*64)
        inference_fixture.InferenceOwnerTests.configure(self,duration_s=2,prepared_reference=compiled)
        review=self.runtime.submit(self.creds['inference'],dict(action='preview_inference',duration_s=2))
        self.confirm('inference',review);self.advance(360);self.assert_shared()
        self.assertEqual(self.runtime.owner.runner.status,'READY',self.runtime.owner.runner.reason)
        self.assert_stop_cancels()

    def test_fixed_reference_completion_does_not_submit_after_the_source_boundary(self):
        from robot_workbench.motion_reference import prepare_reference
        self.create();self.runtime.handoff(self.creds['calibration'],'inference')
        q=np.zeros((61,14));q[:,0]=np.linspace(0.,.02,61)
        source=dict(device_targets14=q.tolist(),model_id='cpu',calibration_profile_digest='c'*64)
        compiled=prepare_reference(source,[0.]*14,np.asarray(self.raw.get_bimanual_joint_position_limits()),source_sha256='a'*64)
        inference_fixture.InferenceOwnerTests.configure(self,duration_s=2,prepared_reference=compiled)
        review=self.runtime.submit(self.creds['inference'],dict(action='preview_inference',duration_s=2))
        self.confirm('inference',review)
        for _ in range(200):
            self.advance()
            if self.runtime.owner.runner is not None and self.runtime.owner.runner.started is not None:break
        self.assertIsNotNone(self.runtime.owner.runner.started)
        runner=self.runtime.owner.runner;end=runner.started+runner.execution_duration_s
        while self.now+.01<end:self.advance()
        self.assertGreaterEqual(runner.native_queued_until,end)
        # Reproduce the old boundary: completion falls on an eligible upload tick.
        runner.target_interpreter.next_upload=end-.001
        submissions=len(self.raw.packets);self.advance()
        self.assertEqual(runner.status,'READY',runner.reason)
        self.assertEqual(len(self.raw.packets),submissions,'completion must not add a late actual command')

    def test_late_online_prediction_cannot_restart_the_stopped_common_executor(self):
        entered,release=threading.Event(),threading.Event();self.addCleanup(release.set)
        def predict(request):
            entered.set()
            if not release.wait(3):raise TimeoutError('CPU late-result fixture not released')
            return dict(request_id=request['request_id'],action14=np.tile([.02]*6+[.088]+[0.]*7,(60,1)))
        self.create();self.runtime.handoff(self.creds['calibration'],'inference')
        inference_fixture.InferenceOwnerTests.configure(self,predict=predict)
        review=self.runtime.submit(self.creds['inference'],dict(action='preview_inference'))
        self.confirm('inference',review);self.advance(50);self.assertTrue(entered.wait(1))
        self.runtime.request_stop();self.advance(3);count=len(self.raw.packets)
        release.set();self.advance(100)
        self.assertEqual(len(self.raw.packets),count);self.assertIsNone(self.raw.window)

    def test_repeated_point_motion_preserves_command_hold_and_integral_without_gain_reenable(self):
        self.profile='x5_shared_precision_v1';self.create();self.raw.position_bias[0]=-.003
        def move(target):
            q=self.runtime.owner.cache['calibration_feedback']['qpos'][:];q[0]=target
            review=self.runtime.submit(self.creds['calibration'],dict(action='preview',active_arm='left',q_target=q))
            self.confirm('calibration',review);self.advance(230)
            self.assertEqual(self.runtime.owner.runner.status,'READY',self.runtime.owner.runner.reason)
        move(.02);previous=self.runtime.owner.runner.target_interpreter
        before=sum(row[0]=='begin'for row in self.raw.events)
        self.assertGreater(previous.last_packet['torque_bias_sdk14'][0][0],.05)
        move(.03);current=self.runtime.owner.runner.target_interpreter
        self.assertEqual(sum(row[0]=='begin'for row in self.raw.events),before)
        self.assertEqual(current.compensation_parent,previous.reference.metadata['reference_id'])
        self.assertTrue(previous.cancelled)
        self.assertGreater(current.last_packet['torque_bias_sdk14'][0][0],.05)


class SharedPrecisionRoutesTest(SharedTargetRoutesTest):
    profile='x5_shared_precision_v1'


if __name__=='__main__':unittest.main()
