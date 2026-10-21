"""CPU actuator injection checks; no assertion of hardware/PhysX qualification."""
from copy import deepcopy
from types import SimpleNamespace as NS
import unittest

from test_physx_boundary import LaggingPlant, PhysxBackend, command_sdk


class ActuationBiasTest(unittest.TestCase):
    def backend(self, *, enabled=True, seed=7, amplitude=.02):
        backend = PhysxBackend(LaggingPlant(), joint_offsets=[.01]*12)
        backend.configure_error_model(dict(enabled=enabled, joint_bias_max_rad=amplitude, seed=seed))
        backend.owner = NS(epoch=1, stop_requested=False, release_requested=False,
                           execution_instance_id='first', runner=NS(status='READY'))
        return backend

    def target(self, value=0.):
        return [value]*6+[command_sdk('left', 40.)]+[value]*6+[command_sdk('right', 40.)]

    def begin(self, backend, *, sides=('left', 'right'), value=0.):
        backend.owner.runner.status = 'RUNNING'
        q = self.target(value)
        for side in sides:
            backend.begin(side, q)
            backend.send(side, q)

    def advance(self, backend, steps=1):
        for _ in range(steps):
            backend.advance((backend.physics_step+1)/240)

    def test_bias_drives_plant_but_preserves_nominal_commands_and_true_readback(self):
        backend = self.backend()
        self.begin(backend)
        self.advance(backend)
        self.assertEqual(backend.active, backend.actuation_targets)  # Smooth ramp starts at zero.
        first = backend.error_model_evidence()['execution']
        self.assertEqual(first['draw_index'], 1)  # Both arms share one sample.
        self.advance(backend, 60)
        half = backend.error_model_evidence()['execution']
        self.assertEqual(half['ramp_fraction'], .5)
        self.assertAlmostEqual(backend.actuation_targets['left'][0],
                               backend.active['left'][0]+half['sampled_joint_bias_rad'][0]*.5)
        self.advance(backend, 60)
        evidence = backend.error_model_evidence()['execution']
        self.assertEqual(evidence['ramp_fraction'], 1.)
        for n, side in enumerate(('left', 'right')):
            self.assertEqual(backend.active[side], [.01]*6+[40.])
            for i in range(6):
                self.assertAlmostEqual(backend.actuation_targets[side][i], .01+evidence['sampled_joint_bias_rad'][n*6+i])
            self.assertEqual(backend.actuation_targets[side][6], 40.)
        self.assertNotEqual(backend.actual, backend.active)
        self.assertEqual(backend.actual, backend.plant.read()[0])
        self.assertAlmostEqual(backend.read_calibration_state()['qpos'][0], backend.actual['left'][0]-.01)
        self.assertNotEqual(backend.actual['left'][0], backend.actuation_targets['left'][0])

    def test_seed_reproduces_sequence_and_repeated_begin_does_not_resample(self):
        def samples():
            backend = self.backend()
            self.begin(backend)
            first = backend.error_model_evidence()['execution']['sampled_joint_bias_rad']
            backend.begin('left', self.target())
            self.assertEqual(backend.error_model_evidence()['execution']['draw_index'], 1)
            backend.set_damping_hold()
            backend.owner.runner.status = 'READY'
            backend.configure_error_model(deepcopy(backend.error_model))  # Handoff must not reseed.
            backend.owner.execution_instance_id = 'second'
            self.begin(backend)
            second = backend.error_model_evidence()['execution']['sampled_joint_bias_rad']
            self.assertEqual(backend.error_model_evidence()['execution']['draw_index'], 2)
            self.assertNotEqual(first, second)
            return first, second
        self.assertEqual(samples(), samples())

    def test_configuration_and_begin_do_not_move_and_disabled_path_is_exact(self):
        backend = self.backend(enabled=False)
        before = deepcopy(backend.actuation_targets)
        backend.configure_error_model(dict(enabled=True, joint_bias_max_rad=.02, seed=2))
        backend.begin('left', self.target())
        self.assertEqual(backend.actuation_targets, before)
        self.assertEqual(backend.plant.steps, 0)
        self.advance(backend, 10)
        self.assertEqual(backend.actual, before)
        self.assertIsNone(backend.error_model_evidence()['execution']['first_applied_physics_step'])
        plain, disabled = self.backend(enabled=False), self.backend(enabled=False)
        for item in (plain, disabled):
            self.begin(item)
            self.advance(item, 130)
        self.assertEqual(plain.actual, disabled.actual)
        self.assertEqual(plain.active, plain.actuation_targets)
        self.assertIsNone(plain.error_model_evidence()['execution'])

    def test_single_arm_does_not_perturb_idle_arm_or_jaw(self):
        backend = self.backend()
        right = deepcopy(backend.actual['right'])
        self.begin(backend, sides=('left',))
        self.advance(backend, 130)
        self.assertEqual(backend.actual['right'], right)
        self.assertEqual(backend.actuation_targets['left'][6], 40.)
        self.assertEqual(backend.error_model_evidence()['execution']['applied_joint_bias_rad'][6:], [0.]*6)
        self.begin(backend, sides=('right',))
        self.advance(backend)
        self.assertEqual(backend.actuation_targets['right'], backend.active['right'])
        self.assertEqual(backend.error_model_evidence()['execution']['draw_index'], 1)

    def test_new_motion_starts_from_measured_pose_without_carrying_old_bias(self):
        backend = self.backend()
        self.begin(backend)
        self.advance(backend, 130)
        backend.set_damping_hold()
        actual = deepcopy(backend.actual)
        backend.owner.execution_instance_id = 'second'
        q = backend.read_calibration_state()['qpos']
        q[6], q[13] = command_sdk('left', 40.), command_sdk('right', 40.)
        for side in ('left', 'right'):
            backend.begin(side, q)
            backend.send(side, q)
        self.advance(backend)
        for side in ('left', 'right'):
            for target, measured, previous in zip(backend.actuation_targets[side], backend.actual[side], actual[side]):
                self.assertAlmostEqual(target, previous, places=14)
                self.assertAlmostEqual(measured, previous, places=14)
        self.assertEqual(backend.error_model_evidence()['execution']['ramp_fraction'], 0.)

    def test_hold_freezes_actual_pose_and_reconfigure_does_not_spring_back(self):
        backend = self.backend()
        self.begin(backend)
        self.advance(backend, 45)
        actual = deepcopy(backend.actual)
        backend.set_damping_hold()
        backend.owner.runner.status = 'STOPPED'
        backend.configure_error_model(dict(enabled=False, joint_bias_max_rad=0., seed=0))
        self.advance(backend, 130)
        self.assertEqual((backend.active, backend.actual, backend.actuation_targets), (actual, actual, actual))
        self.assertEqual(backend.error_model_evidence()['execution']['state'], 'held')

    def test_revoked_or_completed_execution_does_not_advance_ramp(self):
        for boundary in ('stop', 'release', 'epoch', 'instance', 'ready'):
            with self.subTest(boundary=boundary):
                backend = self.backend()
                self.begin(backend)
                self.advance(backend, 45)
                applied = deepcopy(backend.actuation_targets)
                if boundary in ('stop', 'release'):setattr(backend.owner, boundary+'_requested', True)
                elif boundary == 'epoch':backend.owner.epoch += 1
                elif boundary == 'instance':backend.owner.execution_instance_id = 'replacement'
                else:backend.owner.runner.status = 'READY'
                self.advance(backend, 130)
                self.assertEqual(backend.actuation_targets, applied)

    def test_last_latch_on_ready_uses_frozen_bias_without_spring_back(self):
        backend = self.backend()
        self.begin(backend)
        self.advance(backend, 45)
        applied = deepcopy(backend.actuation_targets)
        backend.send('left', self.target())
        backend.send('right', self.target())
        backend.owner.runner.status = 'READY'
        self.advance(backend, 130)
        self.assertEqual(backend.actuation_targets, applied)

    def test_joint_limit_projection_is_explicit_and_does_not_clip_nominal_target(self):
        backend = self.backend(seed=2, amplitude=.05)
        self.begin(backend, value=1.99)
        self.advance(backend, 130)
        evidence = backend.error_model_evidence()['execution']
        self.assertGreater(evidence['limit_correction_count'], 0)
        self.assertGreater(evidence['maximum_limit_correction_rad'], 0.)
        for side in ('left', 'right'):
            self.assertEqual(backend.active[side][:6], [2.]*6)
            self.assertTrue(all(q <= 2. for q in backend.actuation_targets[side][:6]))
        self.assertTrue(all(row['correction_rad'] < 0 for row in evidence['limit_corrections']))

    def test_configuration_rejects_malformed_or_active_inputs(self):
        backend = self.backend()
        valid = dict(enabled=True, joint_bias_max_rad=.02, seed=2)
        for key, invalid in [('enabled', 1), ('joint_bias_max_rad', True), ('joint_bias_max_rad', float('nan')),
                             ('joint_bias_max_rad', .05001), ('joint_bias_max_rad', -.01),
                             ('seed', True), ('seed', -1), ('seed', 2**32), ('seed', 1.)]:
            with self.subTest(key=key, invalid=invalid), self.assertRaises(ValueError):
                backend.configure_error_model(dict(valid, **{key:invalid}))
        for config in ({}, None, dict(valid, unknown=1)):
            with self.assertRaises(ValueError):backend.configure_error_model(config)
        self.begin(backend)
        with self.assertRaises(ValueError):backend.configure_error_model(valid)

    def test_shared_runtime_accepted_policy_controls_actual_biased_motion(self):
        from pathlib import Path
        from tempfile import TemporaryDirectory
        from test_unified_runtime import Runtime, calibration, Safety, jaw
        from robot_workbench.execution_settings import defaults
        from robot_workbench.physx import SimulationClock
        for tracking, noise, expected in ((False, True, 'READY'), (True, True, 'FAULT'),
                                          (True, False, 'READY')):
            with self.subTest(tracking=tracking, noise=noise), TemporaryDirectory() as directory:
                jaw.configure_mapping(None)
                clock = SimulationClock()
                backend = PhysxBackend(LaggingPlant(), joint_offsets=[0.]*12)
                runtime = Runtime(calibration, Safety, lambda:backend, device_id='bias-integration',
                    environment='fake', lock_root=Path(directory), control_clock=clock,
                    initialize_backend=lambda b:b.set_damping_hold(manual=True))
                backend.owner = runtime.owner
                backend.command_guard = runtime.owner.command_gate
                try:
                    credential = runtime.register_business('calibration')
                    for _ in range(600):clock.advance(runtime, backend)
                    settings = defaults()
                    settings.update(arrival_enabled=False, tracking_enabled=tracking,
                                    tracking_joint_rad=.005, tracking_duration_s=.05)
                    settings['fake_error'] = dict(enabled=noise, joint_bias_max_rad=.03, seed=7)
                    runtime.update_execution_settings(credential, settings)
                    review = runtime.submit(credential, dict(action='preview_workbench', waypoints=[
                        dict(q=[.05]*12, speeds=[.1]*12, capture=False)]))
                    runtime.submit(credential, dict(action='confirm', token=review['token'],
                        confirmation='START_REVIEWED_SINGLE_ARM', physical_clearance=True))
                    for _ in range(12000):
                        clock.advance(runtime, backend)
                        runner = runtime.owner.runner
                        if runner is not None and runner.status in ('READY', 'FAULT'):break
                    self.assertEqual(runner.status, expected, runner.reason)
                    binding = runtime.execution_calibration
                    self.assertEqual(binding['execution_settings']['settings'], settings)
                    self.assertEqual(runner.error_settings, settings)
                    injection = backend.error_model_evidence()['execution']
                    if noise:
                        self.assertEqual(injection['execution_instance_id'], binding['execution_instance_id'])
                        self.assertEqual(injection['command_epoch'], binding['command_epoch'])
                        self.assertEqual(injection['config'], settings['fake_error'])
                        self.assertEqual(injection['draw_index'], 1)
                        self.assertGreater(runner.tracking_exceeded_samples, 0)
                    else:
                        self.assertIsNone(injection)
                        self.assertEqual(runner.tracking_exceeded_samples, 0)
                    if expected == 'FAULT':
                        self.assertIn('tracking error exceeded', runner.reason)
                        self.assertEqual(injection['state'], 'held')
                    elif noise:
                        # Endpoint acceptance does not rewrite feedback or pull the plant to nominal.
                        for _ in range(480):clock.advance(runtime, backend)
                        self.assertEqual(runtime.owner.runner.status, 'READY')
                        self.assertGreater(max(abs(a-b) for side in ('left', 'right')
                            for a,b in zip(backend.actual[side][:6], backend.active[side][:6])), .02)
                        self.assertEqual(backend.actual, backend.plant.read()[0])
                        self.assertEqual(backend.read_calibration_state()['qpos'][:6], backend.actual['left'][:6])
                finally:
                    if backend.is_connected:
                        runtime.owner.submit(dict(action='release', confirmation='ARMS_SUPPORTED_RELEASE'))
                        runtime.tick()
                    runtime.close()


if __name__ == '__main__':
    unittest.main()
