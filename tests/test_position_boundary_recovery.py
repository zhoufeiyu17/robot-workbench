"""Boundary protection remains active; explicit recovery cannot replay motion."""
import math,os,unittest
from unittest.mock import patch
import test_shared_target_routes as shared
import test_joint_head_real_owner as owner_fixture
from robot_workbench.joint_bounds import JointPositionBoundaryError,inspect_joint_feedback,project_joint_positions


class PositionBoundaryRecoveryTest(unittest.TestCase):
    setUp=owner_fixture.OwnerTest.setUp
    advance=owner_fixture.OwnerTest.advance

    def fault(self):
        owner=self.owner;owner.position_environment='physical'
        limits=owner.backend.get_bimanual_joint_position_limits();limits[5]=[-2.,2.]
        owner.backend.get_bimanual_joint_position_limits=lambda:limits
        owner.backend.q[5]=2.036126136779785;self.advance(.01)
        self.assertEqual(owner.control.state,owner.states.FAULT)
        self.assertEqual(owner.last_fault_diagnostic['reasons'],['joint_position_boundary'])
        self.assertEqual(owner.position_boundary_fault['joint'],'left_J6')
        self.assertEqual(owner.position_boundary_fault['qpos_index'],5)
        return owner

    def test_exact_boundary_and_command_margin_do_not_change(self):
        limits=[[-2.,2.]]*12;values=[0.]*12;values[5]=2.+math.radians(2.)
        self.assertEqual(inspect_joint_feedback(values,limits,environment='physical')[0][5],2.)
        values[5]+=.000001
        with self.assertRaises(JointPositionBoundaryError)as e:inspect_joint_feedback(values,limits,environment='physical')
        self.assertEqual(e.exception.details['context'],'feedback');self.assertEqual(e.exception.details['joint'],'left_J6')
        self.assertIn('limits=',str(e.exception));self.assertIn('excess=',str(e.exception))
        values[5]=2.010001
        with self.assertRaises(JointPositionBoundaryError)as e:project_joint_positions(values,limits,environment='physical')
        self.assertEqual(e.exception.details['context'],'command')

    def test_must_return_inside_feedback_envelope_and_acknowledge_without_motion(self):
        owner=self.fault();self.assertFalse(owner.recovery_ready())
        for q in(2.04,2.035):
            owner.backend.q[5]=q
            for _ in range(50):self.advance(.01)
            self.assertFalse(owner.recovery_ready(),'feedback beyond the expanded envelope is still invalid')
        owner.backend.q[5]=2.02
        for _ in range(50):self.advance(.01)
        self.assertTrue(owner.recovery_ready());self.assertEqual(owner.control.state,owner.states.FAULT)
        count=len(owner.backend.events)
        owner.submit(dict(action='recover',confirmation='ACKNOWLEDGE_MOTION_FAULT',owner_pid=os.getpid(),command_epoch=owner.epoch))
        self.advance(.01)
        self.assertEqual(owner.control.state,owner.states.STOPPED);self.assertIsNone(owner.runner)
        self.assertEqual(len(owner.backend.events),count);self.assertIsNone(owner.recoverable_fault)
        self.assertEqual(owner.backend.q[5],2.02,'acknowledgement does not mutate raw feedback or replay commands')

    def test_failed_stop_and_real_hardware_fault_cannot_clear(self):
        with patch.object(self.owner.backend,'set_damping_hold',side_effect=RuntimeError('hold failed')):owner=self.fault()
        owner.backend.q[5]=1.97
        for _ in range(50):self.advance(.01)
        self.assertFalse(owner.position_boundary_fault['stop_hold_return_confirmed']);self.assertFalse(owner.recovery_ready())

    def test_fresh_feedback_and_guard_still_required(self):
        owner=self.fault();owner.backend.q[5]=1.97
        for _ in range(50):self.advance(.01)
        self.assertTrue(owner.recovery_ready())
        owner.native_faults={'left':{'latched':True}};self.assertFalse(owner.recovery_ready());owner.native_faults={}
        owner.hardware=True;self.assertFalse(owner.recovery_ready())
        f=owner.cache['calibration_feedback'];f['can_output_guard']=dict(state='armed',heartbeat_monotonic_s=self.now)
        self.assertTrue(owner.recovery_ready())
        f['can_output_guard']['state']='latched';self.assertFalse(owner.recovery_ready())
        f['can_output_guard'].update(state='armed',heartbeat_monotonic_s=self.now-.2);self.assertFalse(owner.recovery_ready())
        f['can_output_guard']['heartbeat_monotonic_s']=self.now;self.now+=.2;self.assertFalse(owner.recovery_ready())

class RuntimePositionBoundaryRecoveryTest(unittest.TestCase):
    profile='normal_X5'
    setUp=shared.SharedTargetRoutesTest.setUp
    cleanup=shared.SharedTargetRoutesTest.cleanup
    create=shared.SharedTargetRoutesTest.create
    advance=shared.SharedTargetRoutesTest.advance
    confirm=shared.SharedTargetRoutesTest.confirm

    def test_shared_motion_is_cancelled_and_explicit_operation_does_not_resume(self):
        import threading
        self.create();r=self.runtime
        limits=self.raw.get_bimanual_joint_position_limits();limits[5]=[-2.,2.]
        self.raw.get_bimanual_joint_position_limits=lambda:limits
        self.advance(2)
        review=r.submit(self.creds['calibration'],dict(action='preview_workbench',waypoints=[dict(q12=[.1]+[0.]*11,speeds12=[.1]*12)]))
        self.confirm('calibration',review)
        for _ in range(500):
            self.advance()
            if self.raw.window is not None:break
        self.assertIsNotNone(self.raw.window)
        self.raw.position_bias[5]=2.04;self.advance();count=len(self.raw.packets)
        self.assertEqual(r.owner.control.state,r.owner.states.FAULT);self.assertIsNone(self.raw.window)
        r.request_stop();self.advance(2)
        self.assertTrue(r.stop_receipt['stop_hold_return_confirmed'])
        self.assertEqual(r.owner.control.state,r.owner.states.FAULT,'confirmed hold must not clear fault')
        self.raw.position_bias[5]=0.;self.raw.q[5]=1.97;self.advance(50)
        self.assertTrue(r.owner.recovery_ready());self.assertEqual(len(self.raw.packets),count)
        class Actor:
            action='recover_control'
            def require(self,value):assert value is r
        result=[]
        def recover():
            try:result.append(r.control_recover(Actor()))
            except Exception as error:result.append(error)
        with patch.object(self.raw,'close',side_effect=AssertionError('no reconnect')):
            thread=threading.Thread(target=recover);thread.start()
            for _ in range(200):
                self.advance();__import__('time').sleep(.001)
                if not thread.is_alive():break
            thread.join(1)
        self.assertFalse(thread.is_alive());self.assertIsInstance(result[0],dict,result)
        self.assertTrue(result[0]['recovered']);self.assertFalse(result[0]['motion_resumed'])
        self.advance(40);self.assertEqual(len(self.raw.packets),count);self.assertIsNone(r.owner.runner)


if __name__=='__main__':unittest.main()
