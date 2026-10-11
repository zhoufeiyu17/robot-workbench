#!/usr/bin/env python3
"""One SDK owner; default simulation. Native SDK blocks cannot be interrupted.

A pedal stop retains damping; second press selects manual, never resumes motion.
Only a new, physically reviewed plan can enable position control.
"""
import argparse
from collections import deque
from dataclasses import asdict
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import importlib
import json
import math
import os
from pathlib import Path
import queue
import secrets
import signal
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'infra'))
from arx_env.joint_head_jaw_mapping import command_sdk, feedback_gaps, hold_command_sdk, mapping_identity
from arx_env.joint_head_real_motion import MotionRunner, make_plan, make_workbench_plans, ARM_SLICES, StationaryWindow, MotionTrackingError, arrival_tolerances, active_indices, feedback_to_command, project_arm_positions
from robot_workbench.joint_bounds import position_noise_rad


def legacy_modules(source):
    source = Path(source)
    if not source.is_absolute():
        raise ValueError('--legacy-source must be an absolute reviewed source root')
    package = source.resolve(strict=True) / 'infra/arx_env'
    for file in ('coffee_safety.py', 'pedal_stop.py', 'backends/arx5_backend.py'):
        if not (package / file).is_file():
            raise ValueError(f'legacy source missing {file}')
    import arx_env
    arx_env.__path__.append(str(package))
    return importlib.import_module('arx_env.coffee_safety')


def hardware_backend(*, passive_left_jaw=False):
    # Importing this factory does not initialize CAN; only connect() does.
    from arx_env.backends.arx5_backend import Arx5Backend, Arx5BackendConfig
    import numpy as np

    class PassiveJawController:
        def __init__(self, controller, release=lambda: True):
            self.controller = controller
            self.release = release
        def __getattr__(self, name):
            return getattr(self.controller, name)
        def set_gain(self, gain):
            if self.release():gain.gripper_kp = gain.gripper_kd = 0.
            return self.controller.set_gain(gain)
        def set_joint_cmd(self, cmd):
            if self.release():cmd.gripper_vel = cmd.gripper_torque = 0.
            return self.controller.set_joint_cmd(cmd)

    class SingleArmBackend(Arx5Backend):
        passive_jaws = ('left',) if passive_left_jaw else ()
        manual_jaws = frozenset()

        def set_damping_hold(self, *, manual=False, **kwargs):
            # Keep both jaw position gains off throughout manual gravity mode.
            # Wrap before the legacy backend writes gains, avoiding a brief re-clamp.
            self.manual_jaws = set(self.sides) if manual else set()
            for side in ('left', 'right'):
                if not isinstance(self._controllers[side], PassiveJawController):
                    self._controllers[side] = PassiveJawController(
                        self._controllers[side], lambda side=side: side in self.manual_jaws or side in self.passive_jaws)
            result = super().set_damping_hold(manual=manual, **kwargs)
            if not manual:
                # User-reviewed per-joint stop profile; passive motion restriction remains separate.
                for side in self.sides:
                    gain = self._arx5.Gain(6)
                    gain.kp()[:] = np.asarray(self._controller_configs[side].default_kp)*.3
                    gain.kd()[:] = np.minimum(np.asarray(self._controller_configs[side].default_kd)*self.config.teach_kd_scale+1.,4.5)
                    gain.gripper_kp, gain.gripper_kd = .12, .15
                    self._controllers[side].controller.set_gain(gain)
            return result

        def set_control_mode(self, mode):
            if passive_left_jaw and not isinstance(self._controllers['left'], PassiveJawController):
                self._controllers['left'] = PassiveJawController(self._controllers['left'])
            return super().set_control_mode(mode)

        def begin(self, side, physical14):
            self._require_connected()
            self.manual_jaws = set(self.manual_jaws) - {side}
            start, end = ARM_SLICES[side]
            cfg = self._controller_configs[side]
            if (self._robot_configs[side].robot_model != 'X5' or not self.config.gravity_compensation
                    or not np.allclose(cfg.default_kp, [80, 70, 70, 30, 30, 20], rtol=0, atol=1e-9)
                    or not np.allclose(cfg.default_kd, [2, 2, 2, 1, 1, .7], rtol=0, atol=1e-9)
                    or not np.isclose(cfg.default_gripper_kp, 5., rtol=0, atol=1e-6)
                    or not np.isclose(cfg.default_gripper_kd, .2, rtol=0, atol=1e-7)):
                raise ValueError('X5 gain identity changed')
            measured = self._controllers[side].get_joint_state()
            q, jaw = list(map(float, measured.pos())), float(measured.gripper_pos)
            if (len(q) != 6 or not all(math.isfinite(x) for x in q + [jaw])
                    or max(abs(a-b) for a, b in zip(q, physical14[start:end])) > .01
                    or abs(feedback_gaps([jaw if i==end else physical14[i] for i in range(14)])[0 if side=='left' else 1]-feedback_gaps(physical14)[0 if side=='left' else 1]) > 2.):
                raise ValueError('measured begin pose changed')
            exact = list(physical14)
            exact[start:end], exact[end] = q, jaw
            corrections=[]
            hold = list(feedback_to_command(exact, np.asarray(self.get_bimanual_joint_position_limits(),dtype=float).tolist(),
                environment='physical',corrections=corrections))
            self.last_begin_position_boundary=dict(raw_qpos=list(exact),command_qpos=hold[:],corrections=corrections,
                environment='physical',position_noise_rad=position_noise_rad('physical'))
            self.send(side, hold)  # Hold the measured physical gap in command units before position gains.
            gain = self._arx5.Gain(6)
            gain.kp()[:], gain.kd()[:] = cfg.default_kp, cfg.default_kd
            gain.gripper_kp, gain.gripper_kd = 5., .2
            self.command_guard()
            self._controllers[side].set_gain(gain)
            self._control_mode = f'{side}_calibration_position_other_unchanged'
            return tuple(exact)

        def read_calibration_state(self):
            result=super().read_calibration_state()
            result['gripper_torque']=[float(self._controllers[side].get_joint_state().gripper_torque) for side in self.sides]
            result['gripper_contact_detection']='unvalidated; torque telemetry only'
            result['last_begin_position_boundary']=getattr(self,'last_begin_position_boundary',None)
            return result

        def send(self, side, physical14):
            self._require_connected()
            start, end = ARM_SLICES[side]
            q, jaw = np.asarray(physical14[start:end], dtype=float), float(physical14[end])
            limits = self.get_joint_position_limits(side)[:6]
            if (q.shape != (6,) or not np.isfinite(q).all() or np.any(q < limits[:, 0])
                    or np.any(q > limits[:, 1]) or not math.isfinite(jaw)
                    or not 0 <= jaw <= self.config.gripper_width_m + 1e-8):
                raise ValueError('invalid single-arm target')
            cmd = self._arx5.JointState(6)
            cmd.pos()[:], cmd.vel()[:], cmd.torque()[:] = q, 0., 0.
            cmd.gripper_pos, cmd.gripper_vel, cmd.gripper_torque = jaw, 0., 0.
            self.command_guard()
            self._controllers[side].set_joint_cmd(cmd)

    return SingleArmBackend(Arx5BackendConfig(
        control_mode='damping', left_can='can3', right_can='can1',
        left_gripper_open_readout=-3.37205, right_gripper_open_readout=-3.45827))


class SimulatedBackend:
    def __init__(self, q=None):
        self.q = [0.] * 14 if q is None else list(q)
        self.is_connected = True
        self.events = []
        self.origin = time.monotonic()

    def read_calibration_state(self):
        now = time.monotonic()
        return dict(qpos=self.q[:], qvel=[0.]*14, sample_time_s=now,
                    sample_wall_ns=time.time_ns(), sdk_timestamps=[now-self.origin, now-self.origin+41.])

    def get_bimanual_joint_position_limits(self):
        return [[0., .082] if i in (6, 13) else [-3.14, 3.5] for i in range(14)]

    def read_gains(self):
        return {'simulation': True}

    def set_damping_hold(self, **kwargs):
        self.events.append(('damping', kwargs))

    def begin(self, side, q):
        self.events.append(('begin', side, list(q)))
        return tuple(q)

    def send(self, side, q):
        a, b = ARM_SLICES[side]
        self.q[a:b+1] = q[a:b+1]
        self.events.append(('send', side, list(q)))

    def close(self):
        self.events.append(('release',))
        self.is_connected = False


class Owner:
    """HTTP only queues commands or reads cache. tick() alone touches the SDK."""
    def __init__(self, backend, safety, *, listener=None, hardware=False, stop_scale=1., event=None, control_clock=None,position_environment=None):
        self.backend, self.listener, self.hardware = backend, listener, hardware
        self.position_environment=position_environment or ('physical' if hardware else 'fake')
        position_noise_rad(self.position_environment)
        if control_clock is not None and not callable(control_clock):
            raise TypeError("control_clock must be callable")
        self.control_clock = control_clock
        self.review_wall_deadline = None
        self.accepted_review_age_s = None
        self.last_confirm_rejection = None
        self.arm_tolerance = .01
        self.arm_tolerances = None
        self.control = safety.PedalTakeover(backend, stop_damping_scale=stop_scale)
        self.states = safety.TakeoverState
        self.stop_scale = stop_scale
        self.event = event or (lambda *args, **kwargs: None)
        self.lock = threading.RLock()
        self.commands = queue.Queue(maxsize=16)
        self.pending = {}
        self.stop_requested = False
        self.release_requested = False
        self.epoch = 0
        self.runner = None
        self.arrival_events = deque(maxlen=128)
        self.plan_sequence = 0
        self.arrival_nonce = secrets.token_hex(8)
        self.arrival_pending = None
        self.sequence_remaining = []
        self.sequence_deadline = None
        self.sequence_samples = []
        self.capture_sample = None
        self.capture_wait = None
        self.capture_stable_since = None
        self.capture_recovery_deadline = None
        self.last_capture_ack = None
        self.last_capture_window_ns = 0
        self.recoverable_fault = None
        self.consumed = set()
        self.cache = {'pid': os.getpid(), 'state': 'INITIALIZING', 'connected': False}
        self.last_sdk = None
        self.sdk_progress = None
        self.feedback_healthy = False
        self.stationary_window = StationaryWindow()
        self.stationary = False
        self.diagnostic_history = deque(maxlen=200)
        self.last_fault_diagnostic = None
        self.feedback_read_duration_s = None
        self.last_tick_s = None
        self.tick_gap_s = None
        self.evidence_logging_healthy = True
        self.current_feedback = None
        self.feedback_clock = None
        self.feedback_source_stamp = None
        self.review_press = None
        self.active_epoch = None
        self.backend.command_guard = self.command_gate

    def control_time(self):
        """Execution/feedback clock; review TTL and operational timing remain wall time."""
        return time.monotonic() if self.control_clock is None else self.control_clock()

    def review_age(self):
        if self.accepted_review_age_s is not None:
            return self.accepted_review_age_s
        # Only an injected execution clock needs a separate wall-clock review age.
        if self.control_clock is not None and self.review_wall_deadline is not None:
            return 30. + time.monotonic() - self.review_wall_deadline
        return None

    def pedal_snapshot(self):
        if self.listener is None:
            return (None, False, True)
        with self.listener._stop_lock:
            return (self.listener._last_press_ns, self.listener._key_down, self.listener.input_healthy)

    def snapshot(self):
        with self.lock:
            return json.loads(json.dumps(self.cache, allow_nan=False))

    def submit(self, command):
        if not isinstance(command, dict):
            raise ValueError('command object required')
        action = command.get('action')
        with self.lock:
            if action == 'stop':
                if set(command) != {'action'}:
                    if (set(command) != {'action','owner_pid','command_epoch'}
                            or type(command['owner_pid']) is not int or command['owner_pid'] != os.getpid()
                            or type(command['command_epoch']) is not int or command['command_epoch'] != self.epoch):
                        raise ValueError('conditional stop owner or epoch changed')
                self.epoch += 1
                self.pending.clear()
                self.stop_requested = True
                return {'queued': 'stop', 'command_epoch': self.epoch}
            if action == 'release' and set(command) == {'action', 'confirmation'}:
                if command['confirmation'] != 'ARMS_SUPPORTED_RELEASE':
                    raise ValueError('explicit supported-arm release required')
                self.epoch += 1
                self.pending.clear()
                self.release_requested = True
                return {'queued': 'release'}
            if action == 'simulate_pedal' and not self.hardware and set(command) == {'action'}:
                self.epoch += 1
                self.pending.clear()
                self.commands.put_nowait(('pedal', self.epoch, None))
                return {'queued': 'simulate_pedal'}
            if action == 'recover':
                if (set(command) != {'action', 'confirmation', 'owner_pid', 'command_epoch'}
                        or command['confirmation'] != 'ACKNOWLEDGE_MOTION_FAULT'
                        or command['owner_pid'] != os.getpid() or command['command_epoch'] != self.epoch):
                    raise ValueError('explicit acknowledgement of current owner and fault required')
                if not self.recovery_ready():
                    raise ValueError('recoverable motion fault and fresh stationary feedback required')
                self.commands.put_nowait(('recover', self.epoch, self.pedal_snapshot()))
                return {'queued': 'recover'}
            if action == 'capture_ack':
                wait = self.capture_wait
                if (set(command) != {'action','owner_pid','command_epoch','sample_id','capture_ready_wall_ns'}
                        or type(command['owner_pid']) is not int or command['owner_pid'] != os.getpid()
                        or type(command['command_epoch']) is not int or command['command_epoch'] != self.epoch
                        or not wait or command['sample_id'] != wait['sample_id'] or wait['ack_queued']
                        or type(command['capture_ready_wall_ns']) is not int
                        or command['capture_ready_wall_ns'] != wait['ready_wall_ns']
                        or self.capture_recovery_deadline is not None and self.control_time() >= self.capture_recovery_deadline
                        or self.control_time() >= wait['deadline'] or self.stop_requested or self.release_requested
                        or self.cache.get('motion',{}).get('status') != 'AWAITING_CAPTURE'
                        or not self.feedback_healthy or not self.stationary
                        or not 0 <= self.control_time()-self.cache['calibration_feedback']['sample_time_s'] <= .1):
                    raise ValueError('capture ACK requires current owner, epoch and pending stable sample')
                self.command_gate()
                self.commands.put_nowait(('capture_ack', self.epoch, dict(sample_id=wait['sample_id'],capture_ready_wall_ns=wait['ready_wall_ns'])))
                wait['ack_queued'] = True
                return {'queued':'capture_ack','sample_id':wait['sample_id'],'capture_ready_wall_ns':wait['ready_wall_ns']}
            if action in ('preview', 'preview_home', 'preview_arm_home', 'preview_jaw_target', 'preview_both_home', 'preview_workbench'):
                expected = {'action'} if action == 'preview_both_home' else {'action', 'active_arm'} if action != 'preview' else {'action', 'active_arm', 'q_target'}
                if action == 'preview_workbench':
                    expected = {'action', 'waypoints'}
                if action == 'preview_jaw_target':
                    expected = {'action', 'active_arm', 'sdk_gap_mm'}
                if set(command) != expected:
                    raise ValueError('preview fields do not match selected action')
                current = self.cache
                if (not current.get('feedback_healthy') or not current.get('stationary')
                        or current.get('automatic_enabled') or self.stop_requested or self.release_requested
                        or current.get('command_epoch') != self.epoch
                        or current.get('state') not in ('READY', 'STOPPED_DAMPING_HOLD', 'MANUAL_TAKEOVER')):
                    raise ValueError('fresh stationary healthy owner required')
                stamp = current['calibration_feedback']['sample_time_s']
                if not 0 <= self.control_time()-stamp <= .1:
                    raise ValueError('feedback cache expired')
                press = self.pedal_snapshot()
                if press[1] or not press[2]:
                    raise ValueError('pedal must be released and healthy')
                q_start = current['calibration_feedback']['qpos']
                samples = []
                if action == 'preview_workbench':
                    waypoints = command['waypoints']
                    if not isinstance(waypoints,list) or not waypoints:raise ValueError('nonempty waypoints required')
                    seen = set(); motion = []
                    for row in waypoints:
                        if not isinstance(row,dict):raise ValueError('waypoint object required')
                        capture = row.get('capture',False); sample = row.get('sample_id')
                        if type(capture) is not bool:raise ValueError('capture must be boolean')
                        if capture:
                            if not isinstance(sample,str) or not sample.strip() or len(sample)>200 or sample in seen:
                                raise ValueError('capture requires unique nonempty sample_id')
                            seen.add(sample)
                        elif sample is not None:raise ValueError('sample_id requires capture=true')
                        samples.append(sample if capture else None)
                        motion.append({k:v for k,v in row.items() if k not in ('capture','sample_id')})
                    plans = make_workbench_plans(q_start, motion, current['joint_limits'], created_at_s=self.control_time(),environment=self.position_environment)
                elif action == 'preview_both_home':
                    target = list(q_start)
                    for a, b in ARM_SLICES.values():
                        target[a:b] = [0.]*6
                        target[b] = command_sdk('left' if b==6 else 'right',83)
                    plans = (make_plan('both', q_start, target, current['joint_limits'], kind='both_home', created_at_s=self.control_time(),environment=self.position_environment),)
                elif action == 'preview_jaw_target' and command['active_arm'] == 'both':
                    if getattr(self.backend, 'passive_jaws', ()):
                        raise ValueError('both jaws must be active')
                    gap = command['sdk_gap_mm']
                    gaps = gap if isinstance(gap, dict) else dict(left=gap, right=gap)
                    if set(gaps) != {'left', 'right'} or any(type(v) not in (int, float) or not math.isfinite(v) for v in gaps.values()):
                        raise ValueError('finite left and right SDK gaps required')
                    target = list(q_start)
                    target[6], target[13] = gaps['left'] / 1000., gaps['right'] / 1000.
                    plans = (make_plan('both', q_start, target, current['joint_limits'], kind='jaw_target', created_at_s=self.control_time(),environment=self.position_environment),)
                elif action != 'preview':
                    if command['active_arm'] not in ARM_SLICES:
                        raise ValueError('active_arm must be left or right')
                    target = list(q_start)
                    a, b = ARM_SLICES[command['active_arm']]
                    if action in ('preview_jaw_target') and command['active_arm'] in getattr(self.backend, 'passive_jaws', ()):
                        raise ValueError('selected jaw feedback invalid / passive; cannot command jaw')
                    if action == 'preview_jaw_target':
                        gap = command['sdk_gap_mm']
                        if type(gap) not in (int, float) or not math.isfinite(gap):
                            raise ValueError('finite SDK gap required')
                        target[b] = gap / 1000.
                        kind = 'jaw_target'
                    else:
                        target[a:b] = [0.]*6
                        kind = 'home' if action == 'preview_home' else 'arm_home'
                        if kind == 'home':
                            target[b] = command_sdk('left' if b==6 else 'right',83)
                    plan = make_plan(command['active_arm'], q_start, target, current['joint_limits'],
                                     kind=kind, created_at_s=self.control_time(),environment=self.position_environment)
                    plans = (plan,)
                else:
                    plan = make_plan(command['active_arm'], q_start, command['q_target'], current['joint_limits'],
                                     created_at_s=self.control_time(),environment=self.position_environment)
                    plans = (plan,)
                token = secrets.token_urlsafe(24)
                self.pending.clear()
                self.pending[token] = (plans, self.epoch, press[0], os.getpid(), samples or [None]*len(plans))
                self.review_wall_deadline = time.monotonic() + 30.
                return dict(token=token, plan=asdict(plans[0]), plans=[asdict(p) for p in plans],
                            capture_samples=samples, capture_timeout_s=10., capture_hold_s=.5,
                            sequence_id=plans[0].sha256, total_motion_duration_s=sum(p.duration_s for p in plans),
                            owner_pid=os.getpid(), command_epoch=self.epoch,
                            expires_in_s=30, clearance='Operator must review complete swept path; no collision certification')
            if action == 'confirm':
                if (set(command) != {'action', 'token', 'confirmation', 'physical_clearance'}
                        or command['confirmation'] != 'START_REVIEWED_SINGLE_ARM'
                        or command['physical_clearance'] is not True or not isinstance(command['token'], str)):
                    raise ValueError('reviewed single-arm confirmation and physical_clearance=true required')
                review = self.pending.pop(command['token'], None)
                if review is None:
                    raise ValueError('unknown or consumed preview')
                plans, epoch, press, pid, _ = review
                plan = plans[0]
                if (self.review_wall_deadline is None or time.monotonic() > self.review_wall_deadline or epoch != self.epoch
                        or self.stop_requested or self.release_requested or pid != os.getpid()
                        or self.pedal_snapshot() != (press, False, True)):
                    raise ValueError('preview expired or owner/pedal changed')
                self.commands.put_nowait(('confirm', epoch, (review, self.review_wall_deadline)))
                return {'queued': 'confirm', 'plan_sha256': plan.sha256, 'sequence_id': plans[0].sha256}
            raise ValueError('unsupported action or fields')

    def recovery_ready(self):
        f = self.cache.get('calibration_feedback') or {}
        return bool(self.recoverable_fault and self.control.state is self.states.FAULT
                    and self.backend.is_connected and self.feedback_healthy and self.stationary
                    and not self.stop_requested and not self.release_requested
                    and 0 <= self.control_time()-f.get('sample_time_s', -1) <= .1
                    and self.pedal_snapshot()[1:] == (False, True))

    def invalidate(self):
        with self.lock:
            self.arrival_pending = None
            self.epoch += 1
            self.pending.clear()
            self.sequence_remaining = []
            self.sequence_deadline = None
            self.sequence_samples = []
            self.capture_sample = None
            self.capture_wait = None
            self.capture_stable_since = None
            self.capture_recovery_deadline = None

    def stop(self, reason):
        # A press arriving inside a callback gets its original first/second-press
        # meaning exactly once; a generic fault must not toggle it again.
        if self.listener and self.listener.input_healthy:
            presses = self.listener.take_presses()
            if presses:
                for _ in presses:
                    self.pedal()
                return
        was_fault = self.control.state is self.states.FAULT
        previous_reason = self.control.reason
        self.invalidate()
        self.control.auto_aborted = True
        self.control.reason = previous_reason if was_fault else str(reason)
        self.control.state = self.states.FAULT
        self.backend.set_damping_hold(manual=False, experimental_stop_scale=self.stop_scale)
        self.control.state = self.states.FAULT if was_fault else self.states.STOPPED
        self.event('stop', reason=str(reason), epoch=self.epoch)

    def motion_fault(self, reason):
        diagnostic = self.diagnostic_sample(self.current_feedback)
        diagnostic.update(reasons=['motion_fault'],error=str(reason))
        self.recoverable_fault = None
        self.stop(reason)
        # A runner fault needs diagnosis; new previews do not reset faults.
        if not (self.runner and self.runner.reason == 'physical/operator pedal'):
            self.control.state = self.states.FAULT
            if isinstance(reason, MotionTrackingError):
                self.recoverable_fault = str(reason)
            self.last_fault_diagnostic = diagnostic
            self.event('fault_diagnostic',diagnostic=diagnostic,
                       history=list(self.diagnostic_history),history_capacity=200,epoch_after=self.epoch)

    def pedal(self):
        self.invalidate()
        if self.runner:
            self.runner.status, self.runner.reason = 'STOPPED', 'physical/operator pedal'
        if self.control.state is self.states.FAULT:
            # Toggle physical damping without clearing the fault or enabling motion.
            manual = not getattr(self, 'fault_manual', False)
            self.backend.set_damping_hold(manual=manual)
            self.fault_manual = manual
            self.control.auto_aborted = True
        else:
            self.control.press()
        self.event('pedal', state=self.control.state.value, epoch=self.epoch)

    def command_gate(self):
        press, down, healthy = self.pedal_snapshot()
        with self.lock:
            blocked = (self.stop_requested or self.release_requested or self.epoch != self.active_epoch)
        if blocked or not healthy or down or press != self.review_press:
            raise RuntimeError('stop/pedal/epoch changed before SDK command')

    def begin(self, arm, measured):
        self.command_gate()
        exact = measured
        for side in (('left', 'right') if arm == 'both' else (arm,)):
            self.command_gate()
            exact = self.backend.begin(side, exact)
        self.command_gate()
        self.control.auto_aborted = False
        self.control.state = self.states.READY
        self.control.reason = 'new explicitly reviewed single-arm motion'
        return exact

    def send(self, arm, desired):
        self.command_gate()
        for side in (('left', 'right') if arm == 'both' else (arm,)):
            self.command_gate()
            self.backend.send(side, desired)

    def read_feedback(self):
        read_started = time.monotonic()
        feedback = self.backend.read_calibration_state()
        self.feedback_read_duration_s = time.monotonic()-read_started
        now = self.control_time()
        q, v, sdk = feedback['qpos'], feedback['qvel'], feedback['sdk_timestamps']
        if any(key in feedback for key in ('clock_domain','source_clock_id','source_time_ns','physics_step')):
            from robot_workbench.sample_clock import assert_progress, clock_spec, sample_stamp
            environment=getattr(self,'environment',None) or 'physical'
            clock=self.feedback_clock or clock_spec(environment,clock_domain=feedback.get('clock_domain'),
                                                    source_clock_id=feedback.get('source_clock_id'))
            point=sample_stamp(feedback,clock,wall_key='sample_wall_ns')
            assert_progress(self.feedback_source_stamp,point,allow_same=True)
            self.feedback_clock,self.feedback_source_stamp=clock,point
        values = [*q, *v, *sdk, feedback['sample_time_s']]
        if (len(q) != 14 or len(v) != 14 or len(sdk) != 2
                or not all(isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) for x in values)
                or not 0 <= now-feedback['sample_time_s'] <= .1 or min(sdk) < 0):
            raise ValueError('invalid or stale raw feedback')
        if self.last_sdk is None:
            self.sdk_progress = [None, None]
        else:
            for i, (new, old) in enumerate(zip(sdk, self.last_sdk)):
                if new < old:
                    raise ValueError('SDK clock regressed')
                if new > old:
                    self.sdk_progress[i] = now
        self.last_sdk = list(sdk)
        self.feedback_healthy = all(t is not None and now-t <= .1 for t in self.sdk_progress)
        if not self.feedback_healthy:
            self.stationary_window.reset()
            self.stationary = False
        else:
            self.stationary = self.stationary_window.update(feedback['sample_time_s'], q, v)
        return feedback

    def diagnostic_sample(self, feedback):
        """Bounded in-memory evidence; the existing async logger writes boundary events."""
        now = self.control_time()
        runner = self.runner
        return dict(schema='joint-head-capture-diagnostic-v1',wall_ns=time.time_ns(),monotonic_s=time.monotonic(),control_time_s=now,
                    owner_pid=os.getpid(),command_epoch=self.epoch,sample_id=self.capture_sample,
                    plan_sha256=runner.plan.sha256 if runner else None,
                    target_qpos14=list(runner.plan.q_target) if runner else None,
                    target_error=dict(runner.target_error) if runner and runner.target_error else None,
                    motion_state=runner.status if runner else None,control_state=self.control.state.value,
                    feedback_healthy=self.feedback_healthy,stationary=self.stationary,
                    stationary_check=getattr(self.stationary_window,'last_diagnostic',None),
                    runner_stationary_check=getattr(runner.stationary,'last_diagnostic',None) if runner else None,
                    arm_arrival_tolerances_rad=list(arrival_tolerances(self.arm_tolerance,self.arm_tolerances)),
                    feedback=feedback,tick_gap_s=self.tick_gap_s,feedback_read_duration_s=self.feedback_read_duration_s,
                    sdk_progress_age_s=[None if t is None else now-t for t in (self.sdk_progress or [])],
                    capture_ready_wall_ns=self.capture_wait['ready_wall_ns'] if self.capture_wait else None,
                    capture_ready_source_ns=self.capture_wait.get('ready_source_ns') if self.capture_wait else None,
                    capture_source_clock_id=self.capture_wait.get('source_clock_id') if self.capture_wait else None,
                    capture_ack_remaining_s=self.capture_wait['deadline']-now if self.capture_wait else None,
                    capture_recovery_deadline_monotonic_s=self.capture_recovery_deadline,
                    pedal=self.pedal_snapshot())

    def tick(self):
        """Called at 100 Hz on the owner thread; never from a request handler."""
        feedback = None
        self.current_feedback = None
        tick_started = time.monotonic()
        self.tick_gap_s = None if self.last_tick_s is None else tick_started-self.last_tick_s
        self.last_tick_s = tick_started
        guard_reasons = []
        try:
            with self.lock:
                release, stop = self.release_requested, self.stop_requested
                self.stop_requested = False
            if release:
                if self.runner:
                    self.runner.status, self.runner.reason = 'STOPPED', 'explicit release'
                self.control.release()
                self.cache_state(None, None)
                return False
            if self.listener:
                if not self.listener.input_healthy:
                    raise RuntimeError(self.listener.fault_reason or 'pedal input unhealthy')
                for _ in self.listener.take_presses():
                    self.pedal()
            if stop:
                if self.runner:
                    self.runner.status, self.runner.reason = 'STOPPED', 'operator stop'
                self.stop('operator stop')
            feedback = self.read_feedback()
            self.current_feedback = feedback
            limits = self.backend.get_bimanual_joint_position_limits()
            if hasattr(limits, 'tolist'):
                limits = limits.tolist()
            corrections=[]
            bounded=project_arm_positions(feedback['qpos'],limits,environment=self.position_environment,corrections=corrections)
            feedback['position_boundary']=dict(environment=self.position_environment,
                position_noise_rad=position_noise_rad(self.position_environment),corrections=corrections,
                command_joint_positions_rad=[bounded[i] for i in active_indices('both')])
            capture_ack = None
            while not self.commands.empty():
                action, epoch, review = self.commands.get_nowait()
                if action == 'capture_ack':
                    if epoch == self.epoch:capture_ack = review
                    continue
                if action == 'pedal':
                    self.pedal()
                    continue
                if action == 'recover':
                    with self.lock:
                        if (epoch != self.epoch or review != self.pedal_snapshot() or not self.recovery_ready()
                                or any(not lo <= q <= hi for q, (lo, hi) in zip(bounded, limits))):
                            self.event('recover_rejected', reason='owner/feedback/pedal changed')
                            continue
                        reason = self.recoverable_fault
                        self.invalidate()
                        self.runner = None
                        self.recoverable_fault = None
                        self.control.state = self.states.STOPPED
                        self.control.auto_aborted = True
                        self.control.reason = 'motion fault acknowledged; preview a new action'
                        self.event('fault_acknowledged', reason=reason, epoch=self.epoch)
                    continue
                review, review_deadline = review
                plans, _, press, _, samples = review
                plan = plans[0]
                accepted_age = 30. + time.monotonic() - review_deadline
                if not 0. <= accepted_age <= 30.:
                    self.reject_confirmation(plans, 'review_expired', review_deadline=review_deadline)
                    continue
                if (epoch != self.epoch or not self.feedback_healthy
                        or self.control.state not in (self.states.READY, self.states.STOPPED, self.states.MANUAL)
                        or self.runner and self.runner.status not in ('READY', 'HOLD_UNSETTLED', 'STOPPED', 'FAULT')
                        or tuple(map(tuple, limits)) != plan.joint_limits
                        or self.pedal_snapshot() != (press, False, True)):
                    self.reject_confirmation(plans, 'owner/pedal/state changed')
                    continue
                if not self.accept_review(plans):
                    self.reject_confirmation(plans, 'review revoked before execution')
                    continue
                self.active_epoch, self.review_press = epoch, press
                self.sequence_remaining = list(plans[1:])
                self.sequence_samples = list(samples[1:]);self.capture_sample = samples[0];self.capture_wait = None;self.capture_stable_since = None
                self.capture_recovery_deadline = None;self.last_capture_ack = None
                self.sequence_deadline = self.control_time() + sum(p.duration_s for p in plans) + 4 * len(plans) + 11*sum(s is not None for s in samples)
                # TTL is checked at the Owner acceptance boundary, before any
                # runner/gain/recording work. Construction cannot turn this
                # accepted review into a hardware fault by crossing the deadline.
                self.accepted_review_age_s = accepted_age
                try:self.start_motion(plan, feedback)
                finally:self.accepted_review_age_s = None
                self.event('motion_confirmed', plan=asdict(plan), status=self.runner.status)
            if self.runner and self.runner.status not in ('ARMED', 'STOPPED', 'FAULT'):
                # begin/send check the live press counter immediately before writes.
                before = self.runner.status
                self.runner.tick(self.control_time(), feedback)
                if before != self.runner.status:
                    self.event('motion_state', status=self.runner.status, reason=self.runner.reason)
            self.record_arrival(feedback)
            self.diagnostic_history.append(self.diagnostic_sample(feedback))
            if self.capture_sample and self.runner:
                now = self.control_time()
                if self.capture_wait or self.capture_recovery_deadline is not None:
                    self.command_gate()
                    if self.capture_recovery_deadline is not None and now >= self.capture_recovery_deadline:
                        guard_reasons.append('capture_recovery_timeout')
                    if self.capture_wait and now >= self.capture_wait['deadline']:guard_reasons.append('capture_ack_timeout')
                    if not self.feedback_healthy:guard_reasons.append('feedback_unhealthy')
                    target = self.runner.execution_target
                    from arx_env.joint_head_real_motion import command_gap
                    if (target is None or any(abs(feedback['qpos'][i]-target[i]) > tolerance
                            for i,tolerance in zip(active_indices('both'),arrival_tolerances(self.arm_tolerance,self.arm_tolerances)))
                            or any(abs(gap-command_gap(side,target[i])) > 2.
                                   for gap,side,i in zip(feedback_gaps(feedback['qpos']),('left','right'),(6,13)))):
                        guard_reasons.append('target_outside_arrival_tolerance')
                    if self.runner.status not in ('READY','HOLD_UNSETTLED'):guard_reasons.append('motion_not_ready')
                    if guard_reasons:
                        raise ValueError('capture guard: '+', '.join(guard_reasons))
                    if self.capture_wait and (not self.stationary or self.runner.status != 'READY'):
                        if self.capture_recovery_deadline is None:self.capture_recovery_deadline = now+2.
                        self.event('capture_window_revoked',sample_id=self.capture_sample,diagnostic=self.diagnostic_sample(feedback))
                        self.capture_wait = None
                        self.capture_stable_since = None
                if capture_ack is not None:
                    if (self.capture_wait and capture_ack == dict(sample_id=self.capture_sample,
                            capture_ready_wall_ns=self.capture_wait['ready_wall_ns'])):
                        self.event('capture_acknowledged',sample_id=self.capture_sample,diagnostic=self.diagnostic_sample(feedback))
                        self.last_capture_ack = dict(capture_ack)
                        if getattr(self,'environment',None)=='fake':
                            self.last_capture_ack.update(command_epoch=self.epoch,
                                clock_domain=self.capture_wait['clock_domain'],source_clock_id=self.capture_wait['source_clock_id'],
                                capture_ready_source_ns=self.capture_wait['ready_source_ns'])
                        self.capture_sample = None;self.capture_wait = None
                        self.capture_recovery_deadline = None
                    else:
                        self.event('capture_ack_rejected',ack=capture_ack,reason='capture window revoked or changed')
                if self.capture_sample and not self.capture_wait:
                    if self.runner.status == 'READY' and self.feedback_healthy and self.stationary:
                        if self.capture_stable_since is None:self.capture_stable_since = now
                        if now-self.capture_stable_since >= .5:
                            self.last_capture_window_ns = max(time.time_ns(),self.last_capture_window_ns+1)
                            self.capture_wait = dict(sample_id=self.capture_sample,deadline=now+10.,ready_wall_ns=self.last_capture_window_ns,ack_queued=False)
                            if getattr(self,'environment',None)=='fake':
                                from robot_workbench.sample_clock import clock_spec, sample_stamp
                                clock=self.feedback_clock or clock_spec('fake',clock_domain=feedback.get('clock_domain'),
                                                                       source_clock_id=feedback.get('source_clock_id'))
                                source_ns,step=sample_stamp(feedback,clock,wall_key='sample_wall_ns')
                                self.capture_wait.update(ready_source_ns=source_ns,ready_physics_step=step,
                                                         clock_domain='simulation',source_clock_id=clock['source_clock_id'])
                            self.event('awaiting_capture',sample_id=self.capture_sample,diagnostic=self.diagnostic_sample(feedback))
                    else:self.capture_stable_since = None
            if self.sequence_remaining and self.runner and self.runner.status == 'READY' and self.capture_sample is None:
                next_plan = self.sequence_remaining[0]
                actual_gap=feedback_gaps(feedback['qpos'])
                from arx_env.joint_head_jaw_mapping import command_gap_mm
                target_gap=[command_gap_mm(side,next_plan.q_start[i]) for side,i in [('left',6),('right',13)]]
                if (any(abs(feedback['qpos'][i]-next_plan.q_start[i]) > tolerance for i,tolerance in zip(active_indices('both'),arrival_tolerances(self.arm_tolerance,self.arm_tolerances)))
                        or any(abs(a-b)>2. for a,b in zip(actual_gap,target_gap))):
                    raise ValueError('waypoint outside arrival tolerance')
                self.command_gate()
                self.sequence_remaining.pop(0)
                self.capture_sample = self.sequence_samples.pop(0)
                self.capture_stable_since = None
                self.capture_recovery_deadline = None
                self.start_motion(next_plan, feedback, reviewed_deadline=self.sequence_deadline)
            self.cache_state(feedback, limits)
        except Exception as error:
            # Freeze identity and the triggering tick BEFORE epoch invalidation or damping.
            first_fault = self.control.state is not self.states.FAULT
            diagnostic = self.diagnostic_sample(feedback) if first_fault else None
            if diagnostic is not None:
                diagnostic.update(reasons=guard_reasons or ['owner_exception'],error=str(error))
            self.recoverable_fault = None
            self.invalidate()
            if self.runner:
                self.runner.status, self.runner.reason = 'FAULT', str(error)
                self.record_arrival(feedback)
            if self.control.state is not self.states.FAULT:
                try:
                    self.control.fault(str(error))
                except Exception as stop_error:
                    self.control.reason += f'; damping failed: {stop_error}'
            if diagnostic is not None:
                self.last_fault_diagnostic = diagnostic
                self.event('fault_diagnostic',diagnostic=diagnostic,
                           history=list(self.diagnostic_history),history_capacity=200,
                           epoch_after=self.epoch)
            self.event('fault', reason=self.control.reason)
            with self.lock:
                self.cache.update(state=self.control.state.value, reason=self.control.reason,
                                  automatic_enabled=False, feedback_healthy=False, stationary=False,
                                  command_epoch=self.epoch,last_fault_diagnostic=self.last_fault_diagnostic)
                if self.runner:self.cache['motion']={'status':self.runner.status,'reason':self.runner.reason,'sample_id':None}
        return self.control.state is not self.states.RELEASED

    def accept_review(self, plans):
        """Embedding runtimes may atomically revoke queued, not-yet-started reviews."""
        return True

    def reject_confirmation(self, plans, reason, *, review_deadline=None):
        for plan in plans:
            self.consumed.add(plan.sha256)
            for name in ('frozen_payloads', 'alignment_plans'):
                getattr(self, name, {}).pop(plan.sha256, None)
        self.last_confirm_rejection = dict(id=secrets.token_hex(12),owner_pid=os.getpid(),
            command_epoch=self.epoch,plan_sha256=[p.sha256 for p in plans],reason=reason,
            motion_started=False,monotonic_s=time.monotonic(),wall_ns=time.time_ns(),
            review_deadline_monotonic_s=review_deadline)
        self.event('confirm_rejected', **self.last_confirm_rejection)

    def start_motion(self, plan, feedback, *, reviewed_deadline=None):
        self.plan_sequence += 1
        epoch = self.active_epoch
        self.arrival_pending = dict(event_id=f'{os.getpid()}:{self.arrival_nonce}:{epoch}:{self.plan_sequence}',
                                    owner_pid=os.getpid(), command_epoch=epoch,
                                    plan_sha256=plan.sha256, sample_id=self.capture_sample) if plan.kind == 'workbench' else None
        self.runner = MotionRunner(plan, begin=lambda q: self.begin(plan.active_arm, q),
                                   send=lambda q: self.send(plan.active_arm, q),
                                   stop=self.motion_fault, consumed=self.consumed, arm_tolerance=self.arm_tolerance, arm_tolerances=self.arm_tolerances,
                                   environment=self.position_environment)
        self.runner.start(self.control_time(), feedback, reviewed_deadline=reviewed_deadline,
                          review_age_s=self.review_age())

    def record_arrival(self, feedback):
        if self.runner and self.runner.status in ('STOPPED', 'FAULT'):
            self.arrival_pending = None
        if not self.arrival_pending or not self.runner or self.runner.status != 'READY' or feedback is None:
            return
        target = self.runner.execution_target if self.runner.execution_target is not None else self.runner.plan.q_target
        event = dict(self.arrival_pending, status='success',
                     q_target_rad=[float(q) for i,q in enumerate(target) if i not in (6,13)],
                     q_actual_rad=[float(q) for i,q in enumerate(feedback['qpos']) if i not in (6,13)],
                     timestamp_ns=time.time_ns(), reason=self.runner.reason)
        self.arrival_pending = None
        with self.lock:
            self.arrival_events.append(event)
            self.cache['arrival_events'] = list(self.arrival_events)
        self.event('arrival', **event)

    def cache_state(self, feedback, limits):
        press, down, healthy = self.pedal_snapshot()
        value = dict(pid=os.getpid(), mode='hardware' if self.hardware else 'simulation',
                     arrival_events=list(self.arrival_events),
                     capture_diagnostics_schema='joint-head-capture-diagnostic-v1',
                     evidence_logging_healthy=self.evidence_logging_healthy,
                     stationary_diagnostic=getattr(self.stationary_window,'last_diagnostic',None),
                     last_fault_diagnostic=self.last_fault_diagnostic,
                     last_capture_ack=self.last_capture_ack,
                     last_confirm_rejection=self.last_confirm_rejection,
                     simulation_only=not self.hardware,
                     jaw_mapping=mapping_identity(),
                     evidence_source='physical_hardware' if self.hardware else 'simulated_owner',
                     control_clock='wall_monotonic' if self.control_clock is None else 'injected',
                     motion_protocol='reviewed-single-arm-v1',
                     passive_jaws=list(getattr(self.backend, 'passive_jaws', ())),
                     stop_gain_profile={"joint_kp":[24.,21.,21.,9.,9.,6.],"joint_kd":[1.2,1.2,1.2,1.1,1.1,1.07],"gripper_kp":.12,"gripper_kd":.15}, fault_manual=getattr(self,'fault_manual',False),
                     capabilities=['single_arm_home', 'both_home_simultaneous', 'joint_speed_0.2', 'workbench_trajectory_v1', 'calibration_capture_ack_v1', 'calibration_capture_window_v2'],
                     state=self.control.state.value, reason=self.control.reason,
                     recoverable_fault=self.recoverable_fault,
                     connected=self.backend.is_connected, command_epoch=self.epoch,
                     automatic_enabled=bool(self.capture_sample or self.runner and self.runner.status in ('VERIFYING', 'RUNNING', 'SETTLING')),
                     receive_ns=time.time_ns(), calibration_feedback=feedback, joint_limits=limits,
                     feedback_healthy=self.feedback_healthy,
                     arm_arrival_tolerance_rad=self.arm_tolerance if self.arm_tolerances is None else None,
                     arm_arrival_tolerances_rad=list(arrival_tolerances(self.arm_tolerance,self.arm_tolerances)),
                     position_noise_rad=position_noise_rad(self.position_environment),position_environment=self.position_environment,
                     stationary=self.stationary,
                     local_gains=self.backend.read_gains() if self.backend.is_connected else None,
                     release_requires='ARMS_SUPPORTED_RELEASE', stop_damping_scale=self.stop_scale,
                     pedal=dict(input_healthy=healthy, physically_down=down, last_press_ns=press,
                                latched=bool(self.listener and self.listener.requested.is_set())),
                     motion=None if self.runner is None else dict(status='AWAITING_CAPTURE' if self.capture_wait else self.runner.status, reason=self.runner.reason,
                                                                 sample_id=self.capture_sample,
                                                                 capture_recovery_deadline_monotonic_s=self.capture_recovery_deadline,
                                                                 capture_deadline_monotonic_s=self.capture_wait['deadline'] if self.capture_wait else None,
                                                                 capture_ready_wall_ns=self.capture_wait['ready_wall_ns'] if self.capture_wait else None,
                                                                 capture_ready_source_ns=self.capture_wait.get('ready_source_ns') if self.capture_wait else None,
                                                                 capture_ready_physics_step=self.capture_wait.get('ready_physics_step') if self.capture_wait else None,
                                                                 capture_clock_domain=self.capture_wait.get('clock_domain') if self.capture_wait else None,
                                                                 capture_source_clock_id=self.capture_wait.get('source_clock_id') if self.capture_wait else None,
                                                                 plan_sha256=self.runner.plan.sha256,
                                                                 active_arm=self.runner.plan.active_arm,
                                                                 position_boundary=dict(environment=self.runner.plan.environment,
                                                                     plan_corrections=self.runner.plan.position_corrections,
                                                                     execution_start_corrections=self.runner.execution_start_corrections),
                                                                 target_error=self.runner.target_error),
                     sdk_stop_limit='Software stop cannot interrupt a blocked native SDK call')
        with self.lock:
            self.cache = value


def handler(owner):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def reply(self, status, value):
            body = json.dumps(value, allow_nan=False).encode()
            self.send_response(status)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def valid_host(self):
            return self.headers.get('Host') in (f'127.0.0.1:{self.server.server_port}', f'localhost:{self.server.server_port}')

        def do_GET(self):
            if not self.valid_host():
                return self.reply(403, {'error': 'loopback Host required'})
            self.reply(200 if self.path == '/state' else 404, owner.snapshot() if self.path == '/state' else {})

        def do_POST(self):
            if (self.path != '/operator' or not self.valid_host()
                    or self.headers.get('Origin') != 'http://' + self.headers.get('Host', '')):
                return self.reply(403, {'error': 'same-origin operator command required'})
            try:
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length <= 4*1024*1024 or self.headers.get_content_type() != 'application/json':
                    raise ValueError('small JSON command required')
                self.connection.settimeout(.5)
                result = owner.submit(json.loads(self.rfile.read(length)))
                self.reply(202, result)
            except (ValueError, TypeError, queue.Full) as error:
                self.reply(400, {'error': str(error)})
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--legacy-source', type=Path, required=True)
    parser.add_argument('--connect-hardware', action='store_true')
    parser.add_argument('--passive-left-jaw', action='store_true', help='Disable left jaw position/velocity gains; raw feedback remains invalid')
    arrival = parser.add_mutually_exclusive_group()
    arrival.add_argument('--arm-tolerance', type=float, choices=(.01, .02, .05), default=.01)
    arrival.add_argument('--arm-tolerances', type=float, nargs=12, metavar='RAD', help='Left J1..J6 then right J1..J6 arrival tolerances')
    parser.add_argument('--port', type=int, default=8098)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--jaw-mapping-file', type=Path)
    parser.add_argument('--simulation-state', type=Path, help='Simulation-only initial physical14 JSON')
    parser.add_argument('--stop-damping-scale', type=float, choices=(1., 1.8), default=1.)
    args = parser.parse_args()
    if args.jaw_mapping_file:
        from arx_env.joint_head_jaw_mapping import configure_mapping
        configure_mapping(args.jaw_mapping_file)
    try:arrival_tolerances(args.arm_tolerance,args.arm_tolerances)
    except (TypeError,ValueError) as exc:parser.error(str(exc))
    if not 1024 <= args.port <= 65535:
        parser.error('unprivileged loopback port required')
    if args.connect_hardware and args.simulation_state:
        parser.error('simulation state cannot initialize hardware')
    simulation_q = None
    if args.simulation_state:
        simulation_q = json.loads(args.simulation_state.read_text())
        if (not isinstance(simulation_q, list) or len(simulation_q) != 14
                or not all(isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) for x in simulation_q)):
            parser.error('simulation state must contain 14 finite numbers')
    safety = legacy_modules(args.legacy_source)
    args.output.mkdir(parents=True, exist_ok=False)
    # Bounded asynchronous evidence output cannot block physical-pedal processing.
    logs = queue.Queue(maxsize=2048)
    log_failed = threading.Event()
    def event(kind, **fields):
        try:
            logs.put_nowait(dict(time_ns=time.time_ns(), event=kind, **fields))
        except queue.Full:
            log_failed.set()
    def log_worker():
        try:
            with (args.output / 'events.jsonl').open('x') as stream:
                while True:
                    item = logs.get()
                    try:
                        if item is None:
                            return
                        stream.write(json.dumps(item, allow_nan=False)+'\n')
                        stream.flush()
                        if item.get('event')=='fault_diagnostic':os.fsync(stream.fileno())
                    finally:
                        logs.task_done()
        except Exception:
            log_failed.set()
    threading.Thread(target=log_worker, daemon=True).start()
    listener = None
    backend = hardware_backend(passive_left_jaw=args.passive_left_jaw) if args.connect_hardware else SimulatedBackend(simulation_q)
    if args.connect_hardware:
        from arx_env.pedal_stop import PedalStopListener
        listener = PedalStopListener()
        listener.start()
    owner = Owner(backend, safety, listener=listener, hardware=args.connect_hardware,
                  stop_scale=args.stop_damping_scale, event=event)
    owner.arm_tolerance = args.arm_tolerance
    owner.arm_tolerances = args.arm_tolerances
    server = ThreadingHTTPServer(('127.0.0.1', args.port), handler(owner))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    def stop_signal(*_):
        owner.submit({'action': 'stop'})
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, stop_signal)
    try:
        if args.connect_hardware:
            backend.connect()
            backend.set_damping_hold(manual=True)
    except Exception as error:
        owner.control.state = owner.states.FAULT
        owner.control.reason = f'initialization failed: {error}'
    event('started', pid=os.getpid(), hardware=args.connect_hardware, legacy_source=str(args.legacy_source))
    last_evidence = 0.
    while True:
        owner.evidence_logging_healthy = not log_failed.is_set()
        if log_failed.is_set():
            with owner.lock:owner.cache['logging_warning']='evidence logger unavailable; motion unaffected'
        if not owner.tick():
            break
        now = time.monotonic()
        if now-last_evidence >= .1:
            event('state', **owner.snapshot())
            last_evidence = now
        time.sleep(.01)
    if listener:
        listener.close()
    server.shutdown()
    server.server_close()
    event('released', pid=os.getpid())
    if not log_failed.is_set():
        logs.put(None)
        logs.join()


if __name__ == '__main__':
    main()
