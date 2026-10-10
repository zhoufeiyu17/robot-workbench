"""Collection-owned loopback controller; calibration remains the SDK/safety source.

Importing or starting the service never connects hardware. Acquisition is explicit.
Reviewed plans remain intact; derived execution runs independently of rendering.
"""
import argparse
from collections import deque
from copy import deepcopy
from dataclasses import asdict
import gc
import importlib.util
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
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from contract import validate_physical14, validate_physical_feedback
from frozen_executor import FrozenExecutor, validate_plan_evidence
from real_contract import validate_real_plan
from robot_workbench.sample_clock import clock_spec, clock_for_source, sample_stamp, assert_progress, owner_identity


class LoopTimingProbe:
    """Owner-thread scalar spans; bounded context, serialized by the existing logger.

    Spans are inclusive (nested durations must not be summed). CPU is this thread's
    CPU only; wall minus CPU includes waits, descheduling and native blocking.
    """
    def __init__(self):
        self.thread_id = threading.get_ident()
        self.history = deque(maxlen=12)
        self.current = None
        self.pending = None
        self.sequence = 0
        self.dropped_spans = 0
        self.gc_started = None

    def gc_callback(self, phase, info):
        # GC callbacks execute on the collecting thread; do not mix CPU clocks.
        if threading.get_ident() != self.thread_id:
            return
        if phase == 'start':
            self.gc_started = ((self.current, time.monotonic(), time.thread_time(), info['generation'])
                               if self.current is not None else None)
        elif phase == 'stop' and self.gc_started is not None:
            frame, wall, cpu, generation = self.gc_started
            self.gc_started = None
            if len(frame[3]) < 48:
                frame[3].append((('gc_gen_0', 'gc_gen_1', 'gc_gen_2')[generation],
                                 wall, time.monotonic()-wall, time.thread_time()-cpu))
            else:
                self.dropped_spans += 1

    def call(self, stage, function, *args, **kwargs):
        if self.current is None or threading.get_ident() != self.thread_id:
            return function(*args, **kwargs)
        wall, cpu = time.monotonic(), time.thread_time()
        try:
            return function(*args, **kwargs)
        finally:
            span = (stage, wall, time.monotonic()-wall, time.thread_time()-cpu)
            # Tuple allocation may itself trigger the GC callback before this bound.
            if len(self.current[3]) < 48:
                self.current[3].append(span)
            else:
                self.dropped_spans += 1

    def begin(self, emit, *, log_queue_size=0):
        wall, cpu = time.monotonic(), time.thread_time()
        if self.current is not None:
            seq, started, cpu_started, spans = self.current
            frame = (seq, started, wall-started, cpu-cpu_started, tuple(spans), log_queue_size)
            self.history.append(frame)
            if self.pending is not None:
                self.pending[2].append(frame)
                if len(self.pending[2]) == 5:
                    self.flush(emit)
            elif wall-started > .025:
                self.pending = [seq, tuple(self.history), []]
        self.sequence += 1
        self.current = (self.sequence, wall, cpu, [])

    def flush(self, emit):
        if self.pending is not None:
            trigger, before, after = self.pending
            self.pending = None
            emit('owner_loop_trace', schema='owner-loop-timing.v1', trigger_loop=trigger,
                 columns=('loop', 'monotonic_s', 'gap_s', 'thread_cpu_s', 'spans', 'log_queue_size'),
                 span_columns=('stage', 'monotonic_s', 'wall_s', 'thread_cpu_s'),
                 before=before, after=tuple(after), dropped_spans=self.dropped_spans,
                 inclusive_spans=True)


def load_calibration(root):
    root = Path(root).resolve(strict=True)
    sys.path.insert(0, str(root / 'infra'))
    spec = importlib.util.spec_from_file_location('collection_calibration_owner', root / 'examples/scripts/debug/joint_head_real_owner.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def classes(ref, *, jaw_mapping_file=None):
    from arx_env.joint_head_real_motion import MotionRunner, make_plan, MotionTrackingError, _state
    from arx_env import joint_head_jaw_mapping as jaw
    mapping_id = jaw.mapping_identity()['version']
    if jaw_mapping_file is not None:
        import jaw_mapping_v2 as candidate
        rows = json.loads(Path(jaw_mapping_file).read_text())['reinterpreted_mapping']
        for side in ('left', 'right'):
            for field, table in (('command_sdk_knots', candidate.COMMAND),
                                 ('feedback_sdk_knots', candidate.FEEDBACK),
                                 ('metal_knots_mm', candidate.GAP_MM)):
                if rows[side].get(field) != list(table[side]):
                    raise ValueError('jaw mapping file differs from reviewed A1 mapping')
        # Configure the shared implementation: Owner hold/drift checks also use it.
        jaw.configure_mapping(jaw_mapping_file)
        mapping_id = candidate.MAPPING_ID
    mapping_identity = jaw.mapping_identity()

    def check_mapping():
        if jaw.mapping_identity() != mapping_identity:
            raise ValueError('jaw mapping changed after collection owner configuration')

    def command_sdk(side, gap):
        check_mapping()
        return jaw.command_sdk(side, gap)

    def feedback_gaps(raw):
        check_mapping()
        return jaw.feedback_gaps(raw)

    def sdk_target(target):
        validate_physical14(target)
        return [*target['left_arm_joints'], command_sdk('left', target['left_gripper']),
                *target['right_arm_joints'], command_sdk('right', target['right_gripper'])]

    def physical_feedback(raw):
        left, right = feedback_gaps(raw)
        result = dict(schema_version='physical14.v1', encoding='absolute', units={'joints':'rad','grippers':'mm'},
                      left_arm_joints=list(raw[:6]), right_arm_joints=list(raw[7:13]), left_gripper=left, right_gripper=right)
        return validate_physical_feedback(result, enforce_range=False)

    class Runner(MotionRunner):
        def __init__(self, owner, proxy, payload, frozen, base):
            super().__init__(proxy, begin=owner.begin_collection, send=lambda q: owner.send('both', q),
                             stop=owner.motion_fault, consumed=owner.consumed, arm_tolerance=owner.arm_tolerance,
                             environment=owner.environment or 'physical')
            self.owner = owner
            self.payload, self.base = payload, base
            self.recording = None
            self.preparation_complete=owner.environment!='fake' or owner.recording_factory is None
            self.preparation_closed=False
            self.preparation_thread=None
            self.preparation_error=None
            self.preparation_deadline=None
            self.managed_execution_instance=getattr(owner,'execution_instance_id',None)
            self.execution_instance_id=self.managed_execution_instance or secrets.token_hex(16)
            self.recording_owner=(owner_identity(dict(owner_pid=os.getpid(),command_epoch=owner.epoch,
                runtime_nonce=owner.runtime_nonce)) if not self.preparation_complete else None)
            self.runtime_ready = threading.Event()
            self.recording_waits_for_endpoint=False
            self.runtime_started = False
            self.runtime = self.runtime_error = None
            self.last_sent_index = -1
            self.last_submission_s = None
            self.index = -1
            # Preview already validated this private, single-use executor. Repeating
            # the full scan here blocks the owner feedback loop before its first tick.
            self.frozen = frozen
            self.frozen.sink = self.submit_target
            self.frozen.gate = self.gate
            self.frozen.stop = lambda reason: self._fault(MotionTrackingError(reason))

        def preparation_current(self):
            owner=self.owner
            return (not self.preparation_closed and owner.runner is self and self.status not in ('STOPPED','FAULT')
                and not owner.stop_requested and not owner.release_requested
                and owner.epoch==self.recording_owner['command_epoch']
                and owner.runtime_nonce==self.recording_owner['runtime_nonce']
                and getattr(owner,'execution_instance_id',None)==self.managed_execution_instance)

        def preparation_fault(self,reason):
            self.preparation_error=self.preparation_error or str(reason)
            with self.owner.lock:
                if (self.owner.runner is self and self.owner.epoch==self.recording_owner['command_epoch']
                        and getattr(self.owner,'execution_instance_id',None)==self.managed_execution_instance):
                    self.owner.recording_failed(reason)

        def prepare_recording(self,now):
            if self.preparation_complete:return True
            if self.preparation_thread is None:
                self.preparation_deadline=time.monotonic()+1.
                self.owner.event('recording_preparation_started',execution_instance_id=self.execution_instance_id,
                    deadline_monotonic_s=self.preparation_deadline,maximum_wall_s=1.,image_readiness_gate=False)
                def prepare():
                    candidate=None
                    try:
                        candidate=self.owner.recording_factory(dict(self.payload,execution_owner=self.recording_owner,
                            recording_preparation=dict(defer_execution=True,execution_instance_id=self.execution_instance_id,
                                preparation_active=self.preparation_current,preparation_deadline_monotonic=self.preparation_deadline)),
                            self.preparation_fault)
                        if not hasattr(candidate,'bind_execution') or not hasattr(candidate,'prepared'):
                            raise ValueError('fake recording factory must support deferred execution binding')
                        with self.owner.lock:
                            current=self.preparation_current()
                            if current:self.recording=self.owner.recording=candidate
                        if not current:candidate.finish('stopped','recording preparation revoked before attachment')
                    except Exception as error:
                        self.preparation_fault(error)
                        if candidate is not None:
                            try:candidate.finish('failed',str(error))
                            except Exception:pass
                self.preparation_thread=threading.Thread(target=prepare,daemon=True,name='collection-recording-prepare')
                self.preparation_thread.start()
                return False
            recording=self.recording
            prepared=recording is not None and recording.prepared.is_set() and not recording.error
            failed=self.preparation_error is not None or (recording is not None and (recording.error or recording.done.is_set()))
            if not prepared and not failed and time.monotonic()<self.preparation_deadline:return False
            if not prepared:
                self.preparation_closed=True
                reason=self.preparation_error or (recording.error if recording is not None else None) or 'fake recording preparation exceeded 1 wall second'
                self.preparation_fault(reason)
                if recording is not None:recording.fail(reason)
            self.preparation_complete=True
            # Resource setup precedes the existing 0.3 source-second window.
            # Neither this point nor motion waits for image/encoder/writer ready.
            self.stationary.reset()
            self.verify_started=self.previous=now
            self.owner.event('recording_preparation_finished',execution_instance_id=self.execution_instance_id,
                prepared=prepared,error=self.preparation_error,source_time_s=now,image_readiness_gate=False)
            return False

        def compile_runtime(self, q):
            from execution_timeline import initialize
            limits = self.owner.cache['joint_limits']
            maximum = min(83., self.frozen.profile['gripper']['max_opening_mm'])
            physical_limits = {side+'_arm_joints': [list(pair) for pair in limits[start:start+6]]
                               for side, start in (('left', 0), ('right', 7))}
            for side, slot in (('left', 6), ('right', 13)):
                lo, hi = limits[slot]
                physical_limits[side+'_gripper'] = [max(0., jaw.command_gap_mm(side, lo)),
                                                    min(maximum, jaw.command_gap_mm(side, hi))]
            timeline = initialize(self.base, current_state=physical_feedback(q), limits=physical_limits,
                environment=self.owner.environment or 'physical')
            # The immutable suffix was checked at preview; only new return commands
            # need SDK conversion and limits checks in the compilation worker.
            for command in timeline['commands']:
                if command['time_s'] > timeline['initialization_duration_s'] + 1e-9:
                    break
                target = sdk_target(command['target'])
                if any(not lo <= value <= hi for value, (lo, hi) in zip(target, limits)):
                    raise ValueError('initialization command exceeds SDK joint limits')
            if self.owner.execution_root is not None:
                self.runtime_path=self.owner.execution_root/('execution-'+secrets.token_hex(12)+'.json')
                try:
                    with self.runtime_path.open('x') as stream:json.dump(timeline,stream,allow_nan=False)
                except OSError as error:
                    self.runtime_path=None
                    self.owner.recording_failed('execution evidence write failed: '+str(error))
            return timeline

        def initialize_runtime(self, q):
            if not self.runtime_started:
                self.runtime_started = True
                def compile_commands():
                    try:
                        self.runtime = self.compile_runtime(tuple(q))
                    except Exception as error:
                        self.runtime_error = error
                    finally:
                        self.runtime_ready.set()
                threading.Thread(target=compile_commands, daemon=True).start()
            if not self.runtime_ready.is_set():
                return False
            if self.runtime_error is not None:
                raise self.runtime_error
            timeline = self.runtime
            self.frozen.configure_runtime(timeline)
            self.owner.event('execution_initialized', initialization=timeline['initialization'],
                             execution_timeline_digest=timeline['timeline_digest'])
            try:
                if self.owner.environment=='fake':
                    if self.recording is not None and not self.recording.error and not self.preparation_closed:
                        self.recording.bind_execution(timeline,execution_instance_id=self.execution_instance_id,
                            execution_owner=self.recording_owner)
                elif self.owner.recording_factory:
                    self.recording = self.owner.recording_factory(
                        dict(self.payload, execution_timeline=timeline), self.owner.recording_failed)
            except Exception as error:
                self.owner.recording_failed(error)
            self.owner.recording = self.recording
            return True

        def _check_start_drift(self, q):
            # Collection returns to the reviewed start; calibration retains its drift gate.
            pass

        def _feedback(self, now, state):
            # Keep the raw measured pose, including values outside command limits.
            q, v, sample, sdk = _state(state)
            if sample > now or now-sample > .1:
                raise ValueError('host feedback is older than 0.1 s')
            if self.last_sample is not None and sample < self.last_sample:
                raise ValueError('host feedback clock regressed')
            if self.last_sdk is None:
                self.sdk_progress = [now, now]
            else:
                for arm, (old, new) in enumerate(zip(self.last_sdk, sdk)):
                    if new < old:
                        raise ValueError('SDK feedback clock regressed')
                    if new > old:
                        self.sdk_progress[arm] = now
                        self.sdk_advanced[arm] = True
                    if now-self.sdk_progress[arm] > .1:
                        raise ValueError(f'{("left", "right")[arm]} SDK feedback stopped advancing for 0.1 s')
            self.last_sample, self.last_sdk = sample, sdk
            check_mapping()
            return q, v

        def gate(self):
            self.owner.command_gate()
            return self.owner.feedback_healthy

        def submit_target(self, target):
            # Both lanes preserve the 30 Hz sequence; submit each target once.
            self.index = self.frozen.command_index
            if self.index != self.last_sent_index:
                self.send(sdk_target(target))
                self.last_submission_s = self.owner.control_time()
                send_wall_ns=time.time_ns()
                self.last_sent_index = self.index
                timing={}
                if self.owner.environment=='fake':
                    current=self.owner.current_feedback
                    timing=dict(clock_domain=current.get('clock_domain'),source_clock_id=current.get('source_clock_id'),
                        send_source_time_ns=round(self.last_submission_s*1e9),physics_step=None,
                        feedback_at_send={key:current.get(key) for key in
                            ('clock_domain','source_clock_id','source_time_ns','physics_step','sample_wall_ns')},
                        owner_pid=os.getpid(),command_epoch=self.owner.epoch,runtime_nonce=self.owner.runtime_nonce)
                if self.recording:
                    self.owner.timing_probe.call('recording_command_submit', self.owner.recording_call, 'submit', 'command',tick=self.index,target=target,send_monotonic_s=self.last_submission_s,send_wall_ns=send_wall_ns,**timing)
                self.owner.event('command', timeline_digest=self.frozen.plan['timeline_digest'], tick=self.index,
                                 execution_timeline_digest=self.runtime['timeline_digest'],
                                 source_tick=self.frozen.ticks[self.index]['source_tick'],
                                 stage=self.frozen.ticks[self.index]['stage'],
                                 target=target, send_monotonic_s=self.last_submission_s,send_wall_ns=send_wall_ns,**timing)
            return self.last_submission_s

        def _fault(self, reason, *, cancelled=False):
            self.preparation_closed=True
            if self.recording:
                self.owner.recording_call('finish', 'stopped' if cancelled else 'failed',str(reason))
            return super()._fault(reason,cancelled=cancelled)

        def tick(self, now, state):
            if self.status not in ('VERIFYING', 'RUNNING', 'READY'):
                return self.status
            try:
                if self.recording and self.recording.error:
                    self.owner.recording_failed(self.recording.error)
                if not math.isfinite(now) or now < self.previous:
                    raise ValueError('control clock invalid or regressed')
                # A paused host is distinct from lost SDK feedback. Freshness
                # below remains required; the frozen schedule resumes in order.
                self.maximum_control_gap_s=max(getattr(self,'maximum_control_gap_s',0.),now-self.previous)
                q, v = self._feedback(now, state)
                if self.status == 'READY':
                    self.frozen._feedback(now, dict(sample_monotonic_s=state['sample_time_s'], state=physical_feedback(q)))
                    if self.recording_waits_for_endpoint:
                        self.owner.recording_call('submit','feedback',tick=max(0,self.index),state=physical_feedback(q),
                            sample_monotonic_s=state['sample_time_s'],sample_wall_ns=state['sample_wall_ns'],raw=state)
                        if self.frozen.final_feedback_qualified:
                            self.owner.recording_call('finish','completed')
                            self.recording_waits_for_endpoint=False
                    self.target_error = self.frozen.errors
                    self.previous = now
                    return self.status
                if self.status == 'VERIFYING':
                    if not self.prepare_recording(now):
                        self.previous=now
                        return self.status
                    if not self.runtime_started and now-self.verify_started > 3:
                        raise ValueError('stationary verification deadline exceeded')
                    if not self.runtime_started and (not self.stationary.update(self.last_sample, q, v) or not all(self.sdk_advanced)):
                        self.previous = now
                        return self.status
                    if not self.initialize_runtime(q):
                        self.previous = now
                        return self.status
                    self.begin(tuple(q))
                    self.started = self.owner.control_time()
                    self.status = 'RUNNING'
                    now = self.started
                feedback = dict(sample_monotonic_s=state['sample_time_s'], state=physical_feedback(q))
                result = self.owner.timing_probe.call('frozen_step', self.frozen.start if self.frozen.status == 'new' else self.frozen.tick, now, feedback)
                self.target_error = result['tracking_error']
                if self.recording:
                    self.owner.timing_probe.call('recording_feedback_submit', self.owner.recording_call, 'submit', 'feedback',tick=max(0,self.index),state=feedback['state'],
                                          sample_monotonic_s=state['sample_time_s'],sample_wall_ns=state['sample_wall_ns'],raw=state)
                self.owner.event('feedback', raw=state, frozen=result)
                if result['status'] == 'completed':
                    self.status = 'READY'
                    if self.recording:
                        if self.frozen.final_feedback_qualified:self.owner.recording_call('finish','completed')
                        else:self.recording_waits_for_endpoint=True
                self.previous = now
                return self.status
            except Exception as error:
                return self._fault(error)

    class CollectionOwner(ref.Owner):
        def __init__(self, *args, hardware_binding=None, recording_factory=None, alignment_profile=None, timing_probe=None, execution_root=None, environment=None, source_clock_id=None, runtime_nonce=None, **kwargs):
            self.execution_root=Path(execution_root) if execution_root is not None else None
            self.timing_probe = timing_probe or LoopTimingProbe()
            super().__init__(*args, **kwargs)
            from real_contract import namespace
            if environment is not None:namespace(environment)
            if self.hardware and environment not in (None,'physical') or not self.hardware and environment=='physical':
                raise ValueError('trusted environment differs from hardware backend mode')
            self.environment = environment
            self.runtime_nonce=runtime_nonce
            self.sample_clock=clock_spec('fake',source_clock_id=source_clock_id) if source_clock_id is not None else None
            self.previous_source_stamp=None
            if environment=='fake' and hardware_binding is not None and '/calibration/hardware/active' in str(Path(hardware_binding).resolve()):
                raise ValueError('fake owner must use its isolated calibration binding')
            self.hardware_binding = hardware_binding
            self.recording_factory = recording_factory
            self.recording = None
            self.recording_error = None
            self.frozen_payloads = {}
            self.alignment_plans = {}
            self.alignment_profile = None
            if alignment_profile is not None:
                from real_observation import validate_binding
                from joint_space import offsets, gripper_offset_mm
                if hardware_binding is None or jaw_mapping_file is None:
                    raise ValueError('alignment requires trusted binding and reviewed jaw mapping')
                profile = json.loads(Path(alignment_profile).read_text())
                validate_binding(profile, json.loads(Path(hardware_binding).read_text()),environment=environment or 'physical')
                gripper_offset_mm(profile)
                if (profile.get('source', {}).get('kind') != ('installed_fake_calibration' if environment=='fake' else 'installed_physical_calibration')
                        or profile['model_binding']['jaw_mapping_sha256'] != mapping_identity['source_sha256']):
                    raise ValueError('alignment profile differs from installed model/mapping')
                self.alignment_profile = deepcopy(profile)
                self.alignment_target = [-v for v in offsets(profile)]

        def read_feedback(self):
            feedback=self.timing_probe.call('feedback_read', super().read_feedback)
            if self.environment=='fake':
                clock=self.sample_clock or clock_spec('fake',clock_domain=feedback.get('clock_domain'),source_clock_id=feedback.get('source_clock_id'))
                stamp=sample_stamp(feedback,clock,wall_key='sample_wall_ns')
                assert_progress(self.previous_source_stamp,stamp,allow_same=True)
                if abs(round(feedback['sample_time_s']*1e9)-stamp[0])>1:
                    raise ValueError('Owner feedback sample clock differs from source time')
                self.sample_clock,self.previous_source_stamp=clock,stamp
                feedback.update(owner_pid=os.getpid(),command_epoch=self.epoch,runtime_nonce=self.runtime_nonce)
            return feedback

        def tick(self):
            return self.timing_probe.call('owner_tick', super().tick)

        def cached_snapshot_fields(self, result):
            """Derive collection fields from copied feedback; no SDK or recorder IO."""
            raw = result.get('calibration_feedback')
            if raw:
                result['sample_age_s'] = max(0., self.control_time()-raw['sample_time_s'])
                result['physical_feedback'] = physical_feedback(raw['qpos'])
                result['jaw_gap_mm'] = feedback_gaps(raw['qpos'])
            result['recording_configured'] = self.recording_factory is not None
            result['collection_jaw_mapping'] = mapping_id
            result['recording_error'] = self.recording_error
            runner=self.runner
            result['position_hold_active']=bool(runner is not None and runner.status=='READY'
                and result.get('connected') is True and result.get('state')==self.states.READY.value)
            result['maximum_control_gap_s']=getattr(runner,'maximum_control_gap_s',None)
            if runner is not None and getattr(runner,'runtime',None) is not None:
                result['execution_timeline']=dict(path=str(getattr(runner,'runtime_path',None) or ''),
                    timeline_digest=runner.runtime['timeline_digest'],duration_s=runner.runtime['duration_s'],
                    initialization_duration_s=runner.runtime['initialization_duration_s'],
                    control_hz=runner.runtime['control_hz'],command_count=len(runner.runtime['commands']),
                    parent_timeline_digest=runner.runtime['parent_timeline_digest'])
            return result

        def snapshot(self):
            result = self.cached_snapshot_fields(super().snapshot())
            result['recording'] = self.recording.snapshot() if self.recording else None
            return result

        def recording_failed(self, reason):
            self.recording_error = self.recording_error or str(reason)

        def recording_call(self, method, *args, **kwargs):
            if self.recording is None:
                return
            try:
                return getattr(self.recording, method)(*args, **kwargs)
            except Exception as error:
                self.recording_failed(error)

        def begin_collection(self, measured):
            """Enable existing reviewed gains at a legal command, retaining raw feedback."""
            self.command_gate()
            target = sdk_target(self.runner.frozen.ticks[0]['target'])
            if not self.hardware:
                self.begin('both', target)
                return tuple(measured)
            backend = self.backend
            backend._require_connected()
            for side in ('left', 'right'):
                self.command_gate()
                cfg = backend._controller_configs[side]
                if (backend._robot_configs[side].robot_model != 'X5' or not backend.config.gravity_compensation
                        or len(cfg.default_kp) != 6 or len(cfg.default_kd) != 6
                        or any(not math.isclose(float(a), b, rel_tol=0, abs_tol=1e-9)
                               for a, b in zip(cfg.default_kp, (80, 70, 70, 30, 30, 20)))
                        or any(not math.isclose(float(a), b, rel_tol=0, abs_tol=1e-9)
                               for a, b in zip(cfg.default_kd, (2, 2, 2, 1, 1, .7)))
                        or not math.isclose(float(cfg.default_gripper_kp), 5., rel_tol=0, abs_tol=1e-6)
                        or not math.isclose(float(cfg.default_gripper_kd), .2, rel_tol=0, abs_tol=1e-7)):
                    raise ValueError('X5 gain identity changed')
                backend.manual_jaws = set(backend.manual_jaws) - {side}
                backend.send(side, target)
                self.event('collection_gain_enable_hold', side=side,
                           target=self.runner.frozen.ticks[0]['target'], send_monotonic_s=self.control_time())
                gain = backend._arx5.Gain(6)
                gain.kp()[:], gain.kd()[:] = cfg.default_kp, cfg.default_kd
                gain.gripper_kp, gain.gripper_kd = 5., .2
                self.command_gate()
                backend._controllers[side].set_gain(gain)
            self.command_gate()
            backend._control_mode = 'both_collection_position'
            self.control.auto_aborted = False
            self.control.state = self.states.READY
            self.control.reason = 'new explicitly reviewed collection motion'
            return tuple(measured)

        def source_validator(self, plan, source):
            if self.hardware or self.environment=='fake':
                if self.hardware_binding is None:
                    raise ValueError('real calibration/source provenance binding unavailable')
                binding = json.loads(Path(self.hardware_binding).read_text())
                validate_real_plan(plan, source, binding,environment=self.environment or 'physical')
                if self.environment=='fake' and clock_for_source(source,'fake')!=self.sample_clock:
                    raise ValueError('plan source clock differs from the live fake device')
            else:
                from integration import DualGpuPipeline
                bridge = DualGpuPipeline()
                bridge.ingest_source(source)
                bridge.accept_plan(plan)
            check_mapping()
            declared = source['calibration_profile']['gripper']['mapping']
            # Preserve the original simulated fixture contract only for legacy mode.
            legacy_sim = (not self.hardware and jaw_mapping_file is None
                          and mapping_id == 'legacy-pad-correction-2mm' and declared == 'total_opening_mm')
            if declared != mapping_id and not legacy_sim:
                raise ValueError('calibration profile gripper mapping differs from collection controller')
            if (jaw_mapping_file is not None and source['calibration_profile'].get('model_binding', {}).get('jaw_mapping_sha256')
                    != mapping_identity['source_sha256']):
                raise ValueError('calibration profile jaw mapping artifact digest differs from collection controller')

        def submit(self, command):
            if not isinstance(command, dict):
                raise ValueError('command object required')
            if command.get('action') == 'preview_start_alignment':
                if set(command) != {'action', 'active_arm'} or command['active_arm'] not in ('left', 'right'):
                    raise ValueError('alignment accepts one arm only; targets and speeds are fixed')
                if self.alignment_profile is None:
                    raise ValueError('trusted alignment profile unavailable')
                check_mapping()
                with self.lock:
                    if self.recording and not self.recording.done.is_set():
                        raise ValueError('previous raw recording is still sealing')
                    side = command['active_arm']
                    start = self.cache['calibration_feedback']['qpos']
                    target = list(start)
                    index, offset = (0, 0) if side == 'left' else (7, 6)
                    target[index:index+6] = self.alignment_target[offset:offset+6]
                    if any(abs(target[i]-start[i]) > .05 for i in range(index, index+6)):
                        raise ValueError('alignment exceeds 0.05 rad small-motion bound')
                    # Reuse the owner preview's feedback, pedal, epoch and stationary gates.
                    review = super().submit(dict(action='preview', active_arm=side, q_target=target))
                    plans, epoch, press, pid, samples = self.pending[review['token']]
                    plan = make_plan(side, plans[0].q_start, target, plans[0].joint_limits,
                                     max_velocity=.01, created_at_s=plans[0].created_at_s,
                                     environment=self.environment or 'physical')
                    self.pending[review['token']] = ((plan,), epoch, press, pid, samples)
                    self.frozen_payloads.clear()
                    self.alignment_plans = {plan.sha256: plan}
                    review.update(plan=asdict(plan), plans=[asdict(plan)], sequence_id=plan.sha256,
                                  total_motion_duration_s=plan.duration_s, alignment_profile_id=self.alignment_profile['profile_id'],
                                  purpose='single-arm alignment to calibrated model zero; jaws held')
                    return review
            if command.get('action') != 'preview_frozen':
                if command.get('action') not in ('confirm','stop','release','recover','simulate_pedal'):
                    raise ValueError('collection owner accepts frozen trajectories only')
                result = super().submit(command)
                if command.get('action') in ('stop','release','simulate_pedal'):
                    self.alignment_plans.clear()
                if command.get('action') in ('stop','release') and self.recording:
                    self.recording_call('finish', 'stopped','operator '+command['action'])
                return result
            if set(command) != {'action','plan','source'}:
                raise ValueError('preview_frozen requires plan and source')
            with self.lock:
                preview_epoch=self.epoch
                limits=deepcopy(self.cache.get('joint_limits'))
            payload = deepcopy(command)
            # Full existing timeline/source/calibration/digest checks, without SDK calls.
            frozen = FrozenExecutor(payload['plan'], payload['source'], sink=lambda _:None, gate=lambda:True,
                                    stop=lambda _:None, source_validator=self.source_validator, execution_policy='observation')
            if self.hardware:
                receipt = payload['plan'].get('planning_validation', {})
                gpu=receipt.get('schema_version')=='physical_curobo_motiongen_path_validation.v1'
                if (not gpu and (not isinstance(receipt, dict) or receipt.get('schema_version') != 'geometric_path_validation.v1'
                        or receipt.get('passed') is not True
                        or not isinstance(receipt.get('collision_coverage'), list)
                        or not all(name in receipt['collision_coverage'] for name in ('base_link', 'left', 'right')))):
                    raise ValueError('旧轨迹未通过含立柱的完整几何检查，请在 Playground 重新规划')
                # A prior rollout success must not short-circuit the new geometry
                # receipt's timeline/source/density verification.
                geometry_plan = dict(payload['plan'], simulation={'validated': False, 'outcome': 'not_run'})
                validate_plan_evidence(geometry_plan, payload['source'])
            control = frozen.plan['control']
            if control['physics_hz'] / control['substeps'] != 30:
                raise ValueError('physical collection requires the frozen 30 Hz cadence')
            if not limits:
                raise ValueError('joint limits unavailable for preview')
            for tick in frozen.ticks:
                target = sdk_target(tick['target'])
                if any(not lo <= q <= hi for q,(lo,hi) in zip(target, limits)):
                    raise ValueError('frozen target exceeds SDK joint limits')
            from execution_timeline import prepare
            base = prepare(payload['plan'])
            with self.lock:
                current = self.cache
                if self.epoch != preview_epoch or current.get('joint_limits') != limits:
                    raise ValueError('owner epoch or joint limits changed during preview')
                if (not current.get('feedback_healthy') or not current.get('stationary') or current.get('automatic_enabled')
                        or self.stop_requested or self.release_requested or current.get('command_epoch') != self.epoch
                        or current.get('state') not in ('READY','STOPPED_DAMPING_HOLD','MANUAL_TAKEOVER')):
                    raise ValueError('fresh stationary healthy owner required')
                raw = current['calibration_feedback']
                if not 0 <= self.control_time()-raw['sample_time_s'] <= .1:
                    raise ValueError('feedback cache expired')
                press = self.pedal_snapshot()
                if press[1:] != (False, True):
                    raise ValueError('pedal must be released and healthy')
                first = frozen.ticks[0]['target']
                # The first absolute command initializes the planned start.
                # Ordinary tracking protection applies after command submission.
                proxy = make_plan('both', sdk_target(first), sdk_target(first), limits,
                                  kind='workbench', q_start_is_command=True, created_at_s=self.control_time(),
                                  environment=self.environment or 'physical')
                self.pending.clear()
                self.frozen_payloads.clear()
                self.alignment_plans.clear()
                token = secrets.token_urlsafe(24)
                self.pending[token] = ((proxy,), self.epoch, press[0], os.getpid(), [None])
                self.review_wall_deadline = time.monotonic() + 30.
                self.frozen_payloads[proxy.sha256] = (payload, frozen, base)
                return dict(token=token, owner_pid=os.getpid(), command_epoch=self.epoch, expires_in_s=30,
                            timeline_digest=frozen.plan['timeline_digest'], ticks=len(frozen.ticks), tick_count=len(frozen.ticks), duration_s=frozen.duration_s,
                            source_control_hz=30, source_tick_count=len(frozen.ticks),
                            compiled_tick_count_before_initialization=len(base['commands']),
                            initial_target=first, initialization_target='reviewed frozen initial_target',
                            initialization='smooth return from current measured pose; commands clipped to SDK limits, raw feedback retained',
                            final_target=frozen.ticks[-1]['target'], control_hz=base['control_hz'],
                            clearance='Operator must review complete swept path; no collision certification')

        def pedal(self):
            super().pedal()
            if self.recording:self.recording_call('finish', 'stopped','physical pedal')

        def start_motion(self, plan, feedback, *, reviewed_deadline=None):
            if self.alignment_plans.pop(plan.sha256, None) == plan:
                return super().start_motion(plan, feedback, reviewed_deadline=reviewed_deadline)
            payload, frozen, base = self.frozen_payloads.pop(plan.sha256)
            self.recording_error = None
            runner = self.timing_probe.call('runner_init', Runner, self, plan, payload, frozen, base)
            with self.lock:
                self.runner=runner
                if self.environment=='fake':self.recording=None
            self.runner.start(self.control_time(), feedback, review_age_s=self.review_age())

        def cache_state(self, feedback, limits):
            return self.timing_probe.call('cache_state', self._cache_state, feedback, limits)

        def _cache_state(self, feedback, limits):
            self.timing_probe.call('cache_state_base', super().cache_state, feedback, limits)
            with self.lock:
                self.cache['service'] = 'collection-real-control'
                if self.environment is not None:self.cache['environment']=self.environment
                self.cache['capabilities'] = ['collection_execution_v2']
                if self.runner and hasattr(self.runner, 'frozen'):
                    self.cache['frozen'] = self.timing_probe.call('frozen_snapshot', self.runner.frozen.snapshot, include_history=False)
                    self.cache['frozen']['validation_scope'] = 'hardware' if self.hardware else 'sim/mock'
                    self.cache['frozen']['submitted_command_ticks'] = self.runner.last_sent_index + 1
                    self.cache['frozen']['execution_tick_count'] = len(self.runner.frozen.ticks)
                    # No claim of task success: even a completed command stream requires acceptance.

    return CollectionOwner, sdk_target, physical_feedback


class Lifecycle:
    """Serialize HTTP and owner-thread lifecycle without holding a lock in SDK calls."""
    def __init__(self):
        self.lock = threading.RLock()
        self.owner = None
        self.last_error = None
        self.pending = self.connecting = self.stop_latched = self.exit_requested = False

    def submit(self, command):
        with self.lock:
            action = command.get('action')
            if action == 'acquire':
                if command != {'action':'acquire','confirmation':'ARMS_SUPPORTED_CONNECT'}:
                    raise ValueError('supported arms confirmation required')
                if self.owner is not None or self.pending or self.connecting or self.exit_requested:
                    raise ValueError('owner exists, acquisition pending or service stopping')
                self.pending, self.stop_latched = True, False
                self.last_error = None
                return {'queued':'acquire'}
            if action == 'stop' and self.owner is None:
                if command != {'action':'stop'}:
                    raise ValueError('unconditional stop required while disconnected')
                self.pending, self.stop_latched = False, True
                return {'queued':'stop', 'acquisition_cancelled':True}
            if self.owner is None:
                raise ValueError('acquire the collection owner first')
            if action in ('stop','release'):
                result = self.owner.submit(command)
                self.stop_latched = True
                return result
            if action != 'preview_frozen':
                return self.owner.submit(command)
            owner=self.owner
            with owner.lock:
                epoch=owner.epoch
            state=(self.pending,self.connecting,self.stop_latched,self.exit_requested)
        # Full-plan validation must not block take_acquisition()/the owner tick loop.
        result=owner.submit(command)
        with self.lock, owner.lock:
            if (self.owner is not owner or owner.epoch != epoch
                    or state != (self.pending,self.connecting,self.stop_latched,self.exit_requested)):
                owner.pending.pop(result.get('token'),None)
                raise ValueError('owner or lifecycle changed during preview; request a new preview')
        return result

    def take_acquisition(self):
        with self.lock:
            if not self.pending or self.stop_latched:
                return False
            self.pending, self.connecting = False, True
            return True

    def install(self, owner):
        with self.lock:
            if self.stop_latched:
                self.connecting = False
                return False
            self.owner = owner
            return True

    def finish_acquisition(self):
        with self.lock:
            self.connecting = False
            if self.stop_latched and self.owner is not None:
                self.owner.submit({'action':'stop'})

    def signal(self):
        with self.lock:
            self.exit_requested = True
            self.submit({'action':'stop'})

    def snapshot(self, hardware):
        with self.lock:
            if self.owner is not None:
                result = self.owner.snapshot()
                if self.exit_requested:
                    result['shutdown_pending'] = 'stopped; supported release required before exit'
                return result
            return dict(service='collection-real-control', connected=False, state='DISCONNECTED',
                        acquisition_pending=self.pending or self.connecting, hardware=hardware, acquisition_error=self.last_error)


def controller_conflicts():
    """Read Linux process identities; never stop or take over an existing owner."""
    conflicts = []
    for path in Path('/proc').glob('[0-9]*/cmdline'):
        if int(path.parent.name) == os.getpid():
            continue
        try:
            argv = path.read_bytes().split(b'\0')
            text = ' '.join(v.decode(errors='replace') for v in argv)
        except (OSError, ValueError):
            continue
        if (('joint_head_real_owner.py' in text and '--connect-hardware' in text)
                or ('real_control.py' in text and '--hardware' in argv_as_strings(argv))
                or any(value == 'prometheus.web' for value in argv_as_strings(argv))):
            conflicts.append(int(path.parent.name))
    return conflicts


def argv_as_strings(argv):
    return [v.decode(errors='replace') for v in argv]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--calibration-code', type=Path, required=True)
    parser.add_argument('--legacy-source', type=Path, required=True)
    parser.add_argument('--port', type=int, required=True)
    parser.add_argument('--token-file', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--hardware', action='store_true')
    parser.add_argument('--hardware-binding', type=Path, help='Trusted local accepted calibration and observation receipt')
    parser.add_argument('--alignment-profile', type=Path, help='Trusted local calibrated profile for bounded single-arm model-zero alignment')
    parser.add_argument('--jaw-mapping-file', type=Path, help='Reviewed A1 +3 mm jaw mapping artifact; omit for legacy mapping')
    parser.add_argument('--recording-root', type=Path)
    parser.add_argument('--camera-recording-dirs', type=Path, help='JSON mapping of three cameras to bounded native record-raw spool directories')
    args = parser.parse_args()
    token = args.token_file.read_text().strip()
    if len(token) < 32 or not 1024 <= args.port <= 65535:
        parser.error('32-character token and unprivileged loopback port required')
    if bool(args.recording_root) != bool(args.camera_recording_dirs):
        parser.error('--recording-root and --camera-recording-dirs are required together')
    ref = load_calibration(args.calibration_code)
    Owner, _, _ = classes(ref, jaw_mapping_file=args.jaw_mapping_file)
    safety = ref.legacy_modules(args.legacy_source)
    args.output.mkdir(parents=True, exist_ok=True)
    import fcntl
    lease = (args.output / 'owner.lock').open('a')
    fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
    recording_factory = None
    if args.recording_root:
        from real_recording import RecordingSession
        def recording_factory(payload,on_fault):
            binding = json.loads(args.hardware_binding.read_text()) if args.hardware_binding else {}
            directories = json.loads(args.camera_recording_dirs.read_text())
            destination = args.recording_root / f'run-{time.time_ns()}' / 'raw'
            return RecordingSession(destination,payload['source'],payload['plan'],binding,
                                    camera_directories=directories,on_fault=on_fault,
                                    execution_timeline=payload.get('execution_timeline'))
    lifecycle = Lifecycle()
    events = queue.Queue(maxsize=4096)
    log_failed = threading.Event()
    timing_probe = LoopTimingProbe()
    def enqueue_event(kind, **fields):
        try: events.put_nowait(dict(time_ns=time.time_ns(), event=kind, **fields))
        except queue.Full: log_failed.set()
    def event(kind, **fields):
        timing_probe.call('event_enqueue', enqueue_event, kind, **fields)
    def writer():
        try:
            with (args.output / f'events-{time.time_ns()}.jsonl').open('x') as stream:
                while True:
                    row = events.get()
                    stream.write(json.dumps(row, allow_nan=False)+'\n')
                    stream.flush()
        except Exception: log_failed.set()
    threading.Thread(target=writer, daemon=True).start()
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_): pass
        def reply(self, code, value):
            body = json.dumps(value, allow_nan=False).encode()
            self.send_response(code)
            self.send_header('Content-Type','application/json')
            self.send_header('Cache-Control','no-store')
            self.send_header('Content-Length',str(len(body)))
            self.end_headers()
            try: self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError): pass
        def authorized(self):
            return (self.headers.get('Host') in (f'127.0.0.1:{args.port}', f'localhost:{args.port}')
                    and secrets.compare_digest(self.headers.get('Authorization',''), 'Bearer '+token)
                    and self.headers.get('Origin') is None)
        def do_GET(self):
            if not self.authorized(): return self.reply(403, {'error':'loopback service authentication required'})
            self.reply(200 if self.path == '/state' else 404, lifecycle.snapshot(args.hardware))
        def do_POST(self):
            if not self.authorized() or self.path != '/operator': return self.reply(403, {'error':'loopback service authentication required'})
            try:
                length = int(self.headers.get('Content-Length','0'))
                if not 0 < length <= 16*1024*1024 or self.headers.get_content_type() != 'application/json':
                    raise ValueError('bounded JSON required')
                self.connection.settimeout(2)
                command = json.loads(self.rfile.read(length))
                if not isinstance(command, dict): raise ValueError('command object required')
                result = lifecycle.submit(command)
                self.reply(202,result)
            except (ValueError, TypeError, KeyError, queue.Full) as error:
                self.reply(400,{'error':str(error)})
    server = ThreadingHTTPServer(('127.0.0.1',args.port),Handler)
    threading.Thread(target=server.serve_forever,daemon=True).start()
    def stop_signal(*_):
        lifecycle.signal()
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, stop_signal)
    previous_loop = None
    previous_work_s = 0.
    gc_callback = timing_probe.gc_callback
    gc.callbacks.append(gc_callback)
    try:
        while True:
            loop_started = time.monotonic()
            timing_probe.begin(enqueue_event, log_queue_size=events.qsize())
            if previous_loop is not None and loop_started-previous_loop > .025:
                event('owner_loop_delay', gap_s=loop_started-previous_loop,
                      previous_work_s=previous_work_s,
                      wait_s=loop_started-previous_loop-previous_work_s, log_queue_size=events.qsize())
            previous_loop = loop_started
            if timing_probe.call('lifecycle_poll', lifecycle.take_acquisition):
                listener = None
                try:
                    if args.hardware:
                        conflicts = controller_conflicts()
                        if conflicts:
                            raise ValueError(f'existing physical controller processes: {conflicts}; no takeover')
                    backend = ref.hardware_backend() if args.hardware else ref.SimulatedBackend()
                    if args.hardware:
                        from arx_env.pedal_stop import PedalStopListener
                        listener = PedalStopListener()
                        listener.start()
                    new_owner = Owner(backend,safety,listener=listener,hardware=args.hardware,event=event,hardware_binding=args.hardware_binding,recording_factory=recording_factory,alignment_profile=args.alignment_profile,timing_probe=timing_probe,execution_root=args.output)
                    if lifecycle.install(new_owner):
                        try:
                            if args.hardware:
                                backend.connect()
                                if not lifecycle.stop_latched:
                                    backend.set_damping_hold(manual=True)
                        except Exception as error:
                            new_owner.control.state = new_owner.states.FAULT
                            new_owner.control.reason = f'initialization failed: {error}'
                    elif listener:
                        listener.close()
                except Exception as error:
                    event('acquisition_rejected',reason=str(error))
                    with lifecycle.lock:
                        lifecycle.stop_latched = True
                        lifecycle.last_error = str(error)
                finally:
                    lifecycle.finish_acquisition()
            owner = lifecycle.owner
            if owner:
                owner.evidence_logging_healthy = not log_failed.is_set()
                if not owner.tick():
                    if owner.listener: owner.listener.close()
                    with lifecycle.lock:
                        lifecycle.owner = None
            if lifecycle.exit_requested and lifecycle.owner is None and not lifecycle.connecting:
                break
            previous_work_s = time.monotonic()-loop_started
            timing_probe.call('sleep', time.sleep, max(0., .01-previous_work_s))
        timing_probe.flush(enqueue_event)
    finally:
        gc.callbacks.remove(gc_callback)
        timing_probe.gc_started = None
    server.shutdown()
    server.server_close()


if __name__ == '__main__':
    main()
