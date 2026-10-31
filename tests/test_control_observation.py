from types import ModuleType, SimpleNamespace as NS
import sys
import time
import unittest
from unittest.mock import patch

from test_prepared_reference_backend import Base, State, Gain, hardware_backend


class ControlObservationTests(unittest.TestCase):
    def backend(self, *, atomic=True, valid=True):
        class ReadBase(Base):
            def read_calibration_state(self):
                return dict(qpos=[0.]*14, sample_time_s=time.monotonic())
        module = ModuleType('arx_env.backends.arx5_backend')
        module.Arx5Backend = ReadBase; module.Arx5BackendConfig = lambda **kw: NS(**kw)
        with patch.dict(sys.modules, {'arx_env.backends.arx5_backend': module}):
            backend = hardware_backend()
        backend.output_guard = NS(call=lambda method: dict(heartbeat_monotonic_s=time.monotonic()))
        for c in backend._controllers.values():
            old = State(6); old.gripper_torque = 0
            state = State(6); state.p[:] = .2
            cmd = State(6); cmd.t[:] = .7; cmd.timestamp = 12.
            gain = Gain(6); gain.p[:] = 400
            c.get_joint_state = lambda old=old: old
            c.get_joint_cmd = lambda cmd=cmd: cmd
            c.get_gain = lambda gain=gain: gain
            if atomic:
                c.get_control_observation = lambda state=state, cmd=cmd, gain=gain: NS(
                    valid=valid, feedback=state, command=cmd, gain=gain,
                    sequence=123, sample_monotonic_ns=time.monotonic_ns()-1_000_000, emergency=False)
                c.get_joint_cmd = lambda: self.fail('atomic telemetry must not use legacy command getter')
                c.get_gain = lambda: self.fail('atomic telemetry must not use legacy gain getter')
        return backend

    def test_atomic_snapshot_and_original_timestamp(self):
        backend = self.backend()
        telemetry = backend.read_calibration_state()['controller_telemetry']
        self.assertEqual(telemetry['status'], 'observed')
        self.assertEqual(telemetry['arms']['left']['qpos'], [.2]*6)
        self.assertEqual(telemetry['arms']['left']['output_torque'], [.7]*6)
        for item in telemetry['observations'].values():
            self.assertTrue(item['coherent']); self.assertEqual(item['sequence'], 123)
            self.assertGreaterEqual(item['age_s'], .001)

    def test_not_ready_stays_unavailable_and_legacy_is_labelled(self):
        telemetry = self.backend(valid=False).read_calibration_state()['controller_telemetry']
        self.assertEqual(telemetry['status'], 'unavailable')
        self.assertIn('not ready', telemetry['error'])
        telemetry = self.backend(atomic=False).read_calibration_state()['controller_telemetry']
        self.assertEqual(telemetry['status'], 'observed')
        self.assertFalse(telemetry['observations']['left']['coherent'])


if __name__ == '__main__': unittest.main()
