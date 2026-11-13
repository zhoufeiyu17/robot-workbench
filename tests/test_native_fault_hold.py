"""Shared latch and real native SDK with an explicitly inert CAN transport."""
import ctypes
import importlib.util
import os
from pathlib import Path
import sys
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from robot_workbench.native_fault_hold import NativeFaultHold, SCHEMA, LEFT_ACK, RIGHT_ACK


@unittest.skipUnless(sys.platform == 'linux', 'Linux shared native hold')
class FaultHoldTests(unittest.TestCase):
    def test_latch_is_monotonic_and_cannot_rearm(self):
        hold = NativeFaultHold.create()
        try:
            hold.arm()
            hold.trip(1)
            hold.trip(3)
            self.assertEqual(hold.snapshot()['fault_code'], 1)
            self.assertEqual(hold.snapshot()['mechanical_support'], 'unconfirmed')
            with self.assertRaises(ValueError):hold.arm()
        finally:hold.close()

    @unittest.skipUnless(os.environ.get('WORKBENCH_TEST_NATIVE_HOLD_MODULE'), 'isolated native module required')
    def test_actual_sdk_latches_pose_keeps_sending_and_never_resumes_future(self):
        module_path = os.environ['WORKBENCH_TEST_NATIVE_HOLD_MODULE']
        library = ctypes.CDLL(module_path)
        library.workbench_test_sends.restype = ctypes.c_uint64
        library.workbench_test_set_gripper.argtypes = [ctypes.c_float]
        library.workbench_test_initial_joint_four.argtypes = [ctypes.c_float]
        spec = importlib.util.spec_from_file_location('arx5_interface', module_path)
        sdk = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(sdk)
        self.assertEqual(sdk.fault_hold_schema, SCHEMA)
        import numpy as np
        for cause in ('feedback', 'heartbeat', 'invalid_feedback', 'boundary_feedback'):
            with self.subTest(cause=cause):
                hold = NativeFaultHold.create()
                arms = []
                try:
                    library.workbench_test_initial_joint_four(-1.61 if cause == 'boundary_feedback' else .2)
                    for index in range(2):
                        robot = sdk.RobotConfigFactory.get_instance().get_config('X5')
                        config = sdk.ControllerConfigFactory.get_instance().get_config('joint_controller', 6)
                        config.gravity_compensation = True
                        arm = sdk.Arx5JointController(robot, config, f'isolated-{cause}-{index}')
                        arm.bind_fault_hold(hold.fd, index)
                        gain = sdk.Gain(6)
                        gain.kp()[:] = config.default_kp
                        gain.kd()[:] = config.default_kd
                        arm.set_gain(gain)
                        future = sdk.JointState(6)
                        future.pos()[:] = .6
                        future.timestamp = arm.get_timestamp() + .3
                        arm.set_joint_cmd(future)
                        arms.append(arm)
                    hold.arm()
                    if cause in ('feedback','boundary_feedback'):hold.trip(1)
                    if cause == 'invalid_feedback':library.workbench_test_set_gripper(-10.)
                    deadline = time.monotonic() + 1.
                    while not all(hold.read(word) for word in (LEFT_ACK, RIGHT_ACK)):
                        self.assertLess(time.monotonic(), deadline, 'native hold was not committed')
                        time.sleep(.002)
                    poses = []
                    for arm in arms:
                        state = arm.get_joint_cmd()
                        poses.append(np.asarray(state.pos()).copy())
                    count = library.workbench_test_sends()
                    time.sleep(.35)
                    self.assertGreater(library.workbench_test_sends(), count + 50)
                    for arm, pose in zip(arms, poses):
                        report = arm.get_emergency_state()
                        self.assertIn(report.reason, ('external_guard_fault','gripper_state_invalid'))
                        self.assertTrue(report.damping_committed)
                        self.assertFalse(report.feedback_valid)
                        self.assertEqual(report.support_mode, 'cached_position_hold')
                        expected = [.2]*6
                        if cause == 'boundary_feedback':expected[3] = -1.6
                        np.testing.assert_allclose(pose, expected, atol=1e-6)
                        state = arm.get_joint_cmd()
                        np.testing.assert_allclose(state.pos(), pose, atol=1e-12)
                        np.testing.assert_allclose(state.vel(), 0., atol=1e-12)
                        gain = arm.get_gain()
                        self.assertTrue(np.all(np.asarray(gain.kp()) > 0))
                        with self.assertRaises(sdk.NativeEmergencyLatched):arm.set_joint_cmd(sdk.JointState(6))
                        hold.heartbeat()
                        arm.set_to_damping()
                        self.assertTrue(arm.get_emergency_state().latched)
                    for arm in arms:arm.quiesce('isolated-supported-release')
                    count = library.workbench_test_sends()
                    time.sleep(.025)
                    self.assertEqual(library.workbench_test_sends(), count)
                finally:
                    for arm in arms:arm.quiesce('isolated-supported-release')
                    arms.clear()
                    hold.close()


if __name__ == '__main__':unittest.main()
