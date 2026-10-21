"""Policy runner for the existing sole Owner. No SDK calls from model/data threads."""
from copy import deepcopy
from dataclasses import asdict, replace
import hashlib
import json
import math
import os
import queue
import secrets
import threading
import time
import weakref

from .inference import InferenceLoop, StableTraySupport, audit_dynamics, vector14
from .sample_clock import POLICY_DURATION_TOLERANCE_S


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def episode_duration(value=120):
    if type(value) not in (int, float) or value not in (1, 2, 40, 120):
        raise ValueError('推理时长仅支持 1、2 秒诊断、40 秒或 120 秒整轮，使用设备源时间')
    return float(value)


def execution_steps(value=30):
    if type(value) is not int or not 1 <= value <= 60:
        raise ValueError('推理执行长度须为 1～60 的整数')
    return value


def configuration(value):
    """Freeze the supported episode duration and policy configuration for review."""
    if not isinstance(value, dict):
        raise ValueError('inference configuration object required')
    result = deepcopy(value)
    if 'reference_supply_timeout_s'in result and (type(result['reference_supply_timeout_s'])not in(int,float)
            or result['reference_supply_timeout_s']!=.5):raise ValueError('reference supply timeout must be 0.5 s')
    integral=result.get('reference_integral_feedback',False)
    if type(integral)is not bool or integral and result.get('trajectory_kind')!='prepared_native_reference':
        raise ValueError('reference integral requires prepared native reference configuration')
    if result.get('reference_feedforward')and not integral:raise ValueError('feedforward requires qualified integral research profile')
    result['duration_s'] = episode_duration(result.get('duration_s', 120))
    if 'execution_duration_s'in result:
        prepared=result.get('prepared_reference')or{};actual=result['execution_duration_s']
        from .motion_reference import native_holds
        if (result.get('trajectory_kind')!='prepared_native_reference' or type(actual)not in(int,float)
                or not math.isfinite(actual)or not 0<actual<=40
                or not math.isclose(actual,prepared.get('duration_s',-1)+sum(native_holds(result)),abs_tol=1e-9)):
            raise ValueError('actual execution duration must match the prepared reference and explicit holds')
    result['execute_steps']=execution_steps(result.get('execute_steps',30))
    fixed = dict(control_hz=30, collision_check=False,
                 position_projection=True, motion_audit='record_only',duration_tolerance_s=POLICY_DURATION_TOLERANCE_S)
    for key, expected in fixed.items():
        if key in result and (result[key] != expected or isinstance(expected, bool) and type(result[key]) is not bool):
            raise ValueError('unsupported inference configuration: '+key)
        result[key] = expected
    result.setdefault('seed', 200000)
    if type(result['seed']) is not int or not 0 <= result['seed'] < 2**63:
        raise ValueError('inference seed must be an integer in [0, 2**63)')
    disabled=result.setdefault('disabled_grippers',[])
    if disabled not in ([],['right']):raise ValueError('inference requires an active left gripper; only right may be disabled')
    if disabled:
        mapping=(result.get('conversion_identity') or {}).get('mapping') or {}
        gap=result.get('disabled_right_gripper_device_mm');model=result.get('disabled_right_gripper_model_m')
        offset=mapping.get('model_gap_offset_mm')
        if (any(type(v) not in (int,float) or not math.isfinite(v) for v in (gap,model,offset))
                or result.get('disabled_right_gripper_assumption')!='profile_max_opening_not_measured'
                or gap!=mapping.get('profile_max_opening_mm')
                or not math.isclose(model,(gap-offset)/1000.,rel_tol=0,abs_tol=1e-12)):
            raise ValueError('disabled right jaw requires its calibrated profile-opening assumption, not measured feedback')
    digest(result)  # Reject callables, NaN and unrecordable configuration before review.
    return result


def check_gripper_policy(owner, config, feedback=None):
    expected=tuple(config.get('disabled_grippers',()))
    actual=tuple(getattr(owner.backend,'passive_jaws',()))
    return validate_gripper_feedback(actual,expected,feedback)


def validate_gripper_feedback(actual,expected,feedback=None):
    if actual!=expected:raise ValueError('inference disabled-gripper policy changed from the reviewed backend')
    flags=(feedback or {}).get('gripper_feedback_valid')
    if (expected and flags!=[True,False]) or (not expected and flags is not None and flags!=[True,True]):
        raise ValueError('inference gripper feedback availability differs from the reviewed backend')
    return expected


def preview(owner, command, value):
    from arx_env.joint_head_real_motion import feedback_to_command, make_plan
    if set(command)-{'action', 'seed', 'duration_s'}:
        raise ValueError('inference preview accepts a seed and 1, 40 or 120 source seconds only')
    config = configuration(dict(value, seed=command.get('seed', value.get('seed', 200000))))
    duration = episode_duration(command.get('duration_s', config['duration_s']))
    if duration != config['duration_s']:
        raise ValueError('inference duration differs from the configured episode')
    feedback = owner.cache['calibration_feedback']
    check_gripper_policy(owner,config,feedback)
    limits = deepcopy(owner.cache['joint_limits'])
    target = feedback_to_command(feedback['qpos'], limits, environment=owner.environment)
    execution_duration=config.get('execution_duration_s',duration)
    proxy = make_plan('both', target, target, limits, kind='workbench', duration_s=execution_duration,
                      q_start_is_command=True, created_at_s=owner.control_time(), environment=owner.environment)
    # This is an approval identity for online execution, not a certified full trajectory.
    payload = dict(configuration=config, joint_limits=limits, initial_qpos=deepcopy(feedback['qpos']),
                   clock=deepcopy(getattr(owner, 'feedback_clock', None)), review_nonce=secrets.token_hex(16))
    proxy = replace(proxy, kind='inference', sha256=digest(dict(proxy=asdict(proxy), payload=payload)))
    token = secrets.token_urlsafe(24)
    owner.pending = {token: ((proxy,), owner.epoch, owner.pedal_snapshot()[0], os.getpid(), [None])}
    owner.inference_payloads = {proxy.sha256: payload}
    owner.review_wall_deadline = time.monotonic()+30.
    return dict(token=token, owner_pid=os.getpid(), command_epoch=owner.epoch, expires_in_s=30,
                plan=asdict(proxy), plans=[asdict(proxy)], sequence_id=proxy.sha256, duration_s=duration,
                total_motion_duration_s=execution_duration,execution_duration_s=execution_duration,purpose='policy_inference_episode',
                configuration=deepcopy(config), configuration_digest=digest(config),
                control_hz=30, collision_checked=False, position_projection='recorded',
                motion_audit='record_only', trajectory_kind='online_predictions_not_frozen_plan')


def device_state(raw, *, disabled_grippers=(), right_gripper_device_mm=None):
    from arx_env.joint_head_jaw_mapping import feedback_gaps
    value = vector14(raw, 'SDK feedback').tolist()
    value[6], value[13] = feedback_gaps(value)
    if 'right' in disabled_grippers:
        if type(right_gripper_device_mm) not in (int,float) or not math.isfinite(right_gripper_device_mm):
            raise ValueError('explicit assumed disabled right jaw device value required')
        value[13]=float(right_gripper_device_mm)
    return value


def sdk_target(value):
    from arx_env.joint_head_jaw_mapping import command_sdk
    result = vector14(value, 'device command').tolist()
    result[6], result[13] = command_sdk('left', result[6]), command_sdk('right', result[13])
    return result


def device_limits(sdk_limits):
    from arx_env.joint_head_jaw_mapping import command_gap_mm
    result = [list(pair) for pair in sdk_limits]
    for side, slot in (('left', 6), ('right', 13)):
        lo, hi = (command_gap_mm(side, value) for value in result[slot])
        result[slot] = [max(0., lo), min(83., hi)]
    return result


class InferenceRunner:
    is_inference = True

    def __init__(self, owner, proxy, feedback, config, *, predict, observe, adapter,
                 support=None, event=None, record=None, record_prepared=None, applied=None,preloaded_prediction=None,prepared_reference=None,feedforward_profile=None):
        from arx_env.joint_head_real_motion import StationaryWindow, feedback_to_command
        from arx_env.joint_head_jaw_mapping import command_gap_mm
        self.owner, self.plan, self.config = owner, proxy, configuration(config)
        self.disabled_grippers=check_gripper_policy(owner,self.config,feedback)
        self.observe, self.support_observe, self.applied_check = observe, support, applied
        self.adapter=adapter
        self.event, self.record = event, record
        self.record_prepared = record_prepared
        self.preloaded_prediction=preloaded_prediction
        self.prepared_reference=prepared_reference
        self.target_interpreter=None
        self.chunks_exhausted=False
        self.latest_feedback=feedback
        from .motion_reference import ReferenceIntegral,native_holds
        self.reference_integral=ReferenceIntegral()if self.config.get('reference_integral_feedback')else None
        self.native_initial_hold,self.native_terminal_hold=native_holds(self.config)
        self.feedforward_profile=feedforward_profile;self.reference_feedforward=None
        if (feedforward_profile is not None and (self.config.get('reference_feedforward')or{}).get('sha256')!=feedforward_profile['sha256']):
            raise ValueError('feedforward differs from frozen review')
        self.execution_duration_s=(prepared_reference.duration_s+self.native_initial_hold+self.native_terminal_hold if prepared_reference is not None else self.config['duration_s'])
        if prepared_reference is not None and (self.config.get('prepared_reference')or{}).get('reference_id')!=prepared_reference.metadata['reference_id']:
            raise ValueError('prepared reference differs from the frozen execution configuration')
        self.native_next_upload=0.;self.native_queued_until=None;self.native_pending=[]
        self.status, self.reason = 'ARMED', ''
        self.stationary = StationaryWindow()
        self.execution_start_corrections = []
        self.execution_target = None
        self.target_error = None
        self.ready_since = self.previous = self.started = None
        self.execution_instance_id = owner.execution_instance_id
        self.command_context = None
        self.outstanding = None
        self.recording_error = None
        self.data_done = threading.Event()
        self.data_queue = queue.Queue(maxsize=8192) if record else None
        self._record_thread = None
        self._data_finish_requested = False
        self.preparation_complete = False
        self.preparation_deadline = self.verify_started = None
        self.actual_submission_count = 0
        self._epoch = owner.epoch
        self.sdk_limits = tuple(tuple(pair) for pair in proxy.joint_limits)
        self.jaw_identity = deepcopy(owner.cache.get('jaw_mapping'))
        held = list(feedback_to_command(feedback['qpos'], self.sdk_limits, environment=owner.environment,
                                       corrections=self.execution_start_corrections))
        held[6], held[13] = command_gap_mm('left', held[6]), command_gap_mm('right', held[13])
        if self.disabled_grippers:held[13]=self.config['disabled_right_gripper_device_mm']
        self.initial_device14 = held
        self.last_device_target = list(held)
        self.limits = device_limits(proxy.joint_limits)
        if self.disabled_grippers:self.limits[13]=[0.,self.config['disabled_right_gripper_device_mm']]
        velocity = self.config.get('velocity_limits', [2.]*6+[None]+[2.]*6+[None])
        acceleration = self.config.get('acceleration_limits', [5.]*6+[None]+[5.]*6+[None])
        # The runner owns the loop; callbacks must not own the runner back.
        runner_ref=weakref.ref(self)
        def predict_before_deadline(request):
            runner=runner_ref()
            if runner is None:raise ValueError('inference runner retired before model query')
            if runner.owner.control_time() >= runner.started+runner.config['duration_s']:
                raise ValueError('inference episode deadline passed before model query')
            return predict(request)
        def emit(name,value):
            runner=runner_ref()
            if runner is not None:runner._emit(name,value)
        def prepare_chunk(*args):
            runner=runner_ref()
            return runner._prepare_native_chunk(*args)if runner is not None else None
        def prepare_response(*args):
            runner=runner_ref()
            if runner is None:raise ValueError('inference runner retired before preparation')
            return runner._prepare_native_response(*args)
        limits=self.limits
        self.loop = InferenceLoop(predict=predict_before_deadline, decode=lambda row, _state: adapter.decode(row, held),
            device_to_model=adapter.device_to_model,
            audit=lambda rows, prior:audit_dynamics(rows, prior, position_limits=limits,
                velocity_limits=velocity, acceleration_limits=acceleration),
            event=emit, seed=self.config['seed'], duration_s=self.execution_duration_s,execute_steps=self.config['execute_steps'],
            maximum_observation_age_s=self.config.get('maximum_observation_age_s', 2.),
            support=StableTraySupport(.1, .1),
            prepare_chunk=prepare_chunk if owner.shared_target_profile and prepared_reference is None else None,
            prepare_response=prepare_response if owner.shared_target_profile and prepared_reference is None else None)
        if not record:
            self.data_done.set()

    @property
    def recording_pending(self):
        return self.record is not None and not self.data_done.is_set()

    def _emit(self, name, data):
        value = dict(data, execution_instance_id=self.execution_instance_id)
        self.owner.event('inference_'+name, **deepcopy(value))
        if self.event:
            self.event(name, deepcopy(value))
        self._record(name, value)

    def _record(self, name, data):
        if self.data_queue is None or self.recording_error or self._data_finish_requested:
            return
        try:
            self.data_queue.put_nowait((name, deepcopy(data)))
        except queue.Full:
            self.recording_error = 'policy recording event queue overflow; evidence incomplete'
            self.owner.recording_failed(self.recording_error)

    def _data_worker(self):
        try:
            while True:
                name, value = self.data_queue.get()
                try:
                    self.record('finish' if name == '_finish' else name, value)
                except Exception as exc:
                    self.recording_error = str(exc)
                    self.owner.recording_failed(exc)
                if name == '_finish':
                    return
        finally:
            self.data_done.set()

    def _finish_data(self, status, reason):
        if self._data_finish_requested or self.data_queue is None:
            return
        self._data_finish_requested = True
        if self._record_thread is None:
            self.data_done.set()
            return
        value = dict(status=status, reason=reason, execution_instance_id=self.execution_instance_id,
                     inference=self.loop.snapshot(), recording_error=self.recording_error)
        # Queued on another thread so a stuck data consumer never blocks stop.
        threading.Thread(target=lambda:self.data_queue.put(('_finish', value)), daemon=True).start()

    def revoke(self, reason='owner_invalidated'):
        if self.target_interpreter is not None:self.target_interpreter.revoke()
        self.loop.stop(reason)
        self.status, self.reason = 'STOPPED', reason
        self.outstanding = None
        self.command_context = None
        self._finish_data('stopped', reason)

    def cancel(self, reason='operator cancelled'):
        self.revoke(reason)
        return self.status

    def start(self, now, feedback, **_):
        from arx_env.joint_head_real_motion import _state
        if self.status != 'ARMED' or self.plan.sha256 in self.owner.consumed:
            raise ValueError('inference review already consumed')
        q, _, sample, _ = _state(feedback)
        check_gripper_policy(self.owner,self.config,feedback)
        if not self.owner.feedback_healthy or not 0 <= now-sample <= .1:
            raise ValueError('fresh healthy feedback required for inference')
        self.owner.consumed.add(self.plan.sha256)
        self.owner.command_gate()
        self.previous = now
        self.execution_target = sdk_target(self.initial_device14)
        self.status = 'VERIFYING'
        self.preparation_deadline = time.monotonic()+1.
        if self.record:
            self._record_thread = threading.Thread(target=self._data_worker, name='policy-record', daemon=True)
            self._record_thread.start()
            self._record('prepare',dict(configuration=deepcopy(self.config),execution_instance_id=self.execution_instance_id,
                execution_owner=dict(owner_pid=os.getpid(),command_epoch=self._epoch,runtime_nonce=self.owner.runtime_nonce),
                clock=deepcopy(getattr(self.owner,'feedback_clock',None)),
                preparation_deadline_monotonic=self.preparation_deadline))

    def _begin(self, now, feedback):
        self.owner.begin('both', feedback['qpos'])
        now = self.owner.control_time()
        self.started = self.previous = now
        self.status = 'RUNNING'
        self.loop.start(epoch=self._epoch, source_time=now, episode_id=self.execution_instance_id)
        self._emit('configuration', dict(configuration=deepcopy(self.config), initial_device14=self.initial_device14,
            sdk_joint_limits=self.sdk_limits, source_clock=deepcopy(getattr(self.owner, 'feedback_clock', None)),
            source_start_s=now, source_end_s=now+self.execution_duration_s, runtime_nonce=self.owner.runtime_nonce,
            owner_pid=os.getpid(), command_epoch=self._epoch))
        if self.prepared_reference is not None:
            self.native_start=now+self.native_initial_hold
            if self.feedforward_profile is not None:
                from .motion_reference import ReferenceFeedforward
                self.reference_feedforward=ReferenceFeedforward(self.feedforward_profile,self.native_start,self.prepared_reference.duration_s,now)
                self._record('feedforward_profile',dict(sha256=self.feedforward_profile['sha256'],
                    phase=self.feedforward_profile['phase'].tolist(),torque_bias_sdk6=self.feedforward_profile['bias'].tolist(),provenance=self.feedforward_profile['provenance']))
            self._emit('native_reference_started',dict(reference=deepcopy(self.prepared_reference.metadata),
                trajectory_start_source_s=self.native_start,source_material_duration_s=self.config['duration_s'],
                execution_duration_s=self.execution_duration_s,initial_hold_s=self.native_initial_hold,terminal_hold_s=self.native_terminal_hold))
            self._queue_native(now,feedback)
            return
        if self.preloaded_prediction is not None:
            # begin() already held the measured pose. A saved reference needs
            # no extra 33ms initial command or async query inside its 2s clock.
            self.loop.prime_reference(self.preloaded_prediction,self.initial_device14)
            command=self.loop.tick(now,epoch=self._epoch)
            if command is None:raise ValueError('preloaded reference did not produce its first command')
            self._submit(command,feedback)
        else:
            self._submit(dict(command_id=secrets.token_hex(16),kind='initial_hold',request_id=None,
                due_source_time=now,submitted_source_time=now,final_device14=list(self.initial_device14)), feedback)

    def _prepare(self, now, feedback):
        if not self.preparation_complete:
            try:
                ready = self.record is None or self.record_prepared is None or self.record_prepared() is True
            except Exception as exc:
                self.recording_error = str(exc); self.owner.recording_failed(exc); ready = False
            if not ready and not self.recording_error and time.monotonic() < self.preparation_deadline:
                return False
            if not ready:
                self.recording_error = self.recording_error or 'inference camera preparation exceeded 1 wall second'
                self.owner.recording_failed(self.recording_error)
            self.preparation_complete = True
            self.verify_started = now
            self.stationary.reset()
        if now-self.verify_started > 3.:
            raise ValueError('inference stationary verification deadline exceeded')
        qpos,qvel=list(feedback['qpos']),list(feedback['qvel'])
        if self.disabled_grippers:qpos[13]=qvel[13]=0.  # Excluded axis; original feedback remains in the recording.
        if not self.stationary.update(feedback['sample_time_s'],qpos,qvel):
            return False
        self._begin(now,feedback)
        return True

    def _submit(self, command, feedback):
        check_gripper_policy(self.owner,self.config,feedback)
        target = sdk_target(command['final_device14'])
        if any(not lo <= value <= hi for value,(lo,hi) in zip(target,self.sdk_limits)):
            raise ValueError('recorded projection did not produce a legal SDK command')
        tick = self.actual_submission_count
        self.command_context = dict(execution_id=self.plan.sha256, execution_instance_id=self.execution_instance_id,
            execution_tick=tick, source_tick=None, stage='model_initial_hold' if command.get('kind')=='initial_hold' else 'model_inference',
            runtime_nonce=self.owner.runtime_nonce, command_epoch=self.owner.epoch, command_id=command['command_id'])
        send_started = self.owner.control_time()
        if send_started >= self.started+self.config['duration_s']:
            self.loop.tick(send_started, epoch=self.owner.epoch)
            self.command_context = None
            return
        common=bool(self.owner.shared_target_profile)
        if common:
            if self.target_interpreter is None:
                from .target_interpreter import prepare_samples
                held=sdk_target(self.initial_device14)
                reference=prepare_samples([0.,self.loop.period],[held,held],self.sdk_limits,
                    source_sha256=self.plan.sha256,kind='inference_initial_hold')
                self.native_start=self.owner.control_time()
                self.target_interpreter=self.owner.make_target_interpreter(reference,'both',self.native_start)
                self._queue_native(self.owner.control_time(),feedback)
        else:self.owner.send('both', target)
        send_returned, send_wall_ns = self.owner.control_time(), time.time_ns()
        sequence = getattr(self.owner.backend, 'sequence', None)
        if not common and type(sequence) is int and 'applied_command_sequences' in feedback:
            command['backend_sequences'] = dict(left=sequence-1, right=sequence)
        if not common:self.actual_submission_count += 1
        self.outstanding = command
        self.execution_target = target
        self.last_device_target = list(command['final_device14'])
        self._record('source_target'if common else'command', dict(command, target_sdk14=target, command_context=deepcopy(self.command_context),
            disabled_sdk_slots=[13] if self.disabled_grippers else [],
            tick=tick, send_monotonic_s=send_started, send_returned_monotonic_s=send_returned,
            send_wall_ns=send_wall_ns, send_source_time_ns=round(send_started*1e9),
            feedback_at_send=deepcopy(feedback), owner_pid=os.getpid(), command_epoch=self.owner.epoch,
            runtime_nonce=self.owner.runtime_nonce, environment=self.owner.environment))

    def _completed(self, now, feedback):
        command = self.outstanding
        interval=0. if self.owner.shared_target_profile and command and command.get('kind')!='initial_hold'else self.loop.period
        if command is None or feedback['sample_time_s']+1e-9 < max(
                command['due_source_time']+interval,command['submitted_source_time']):
            return
        sequences = command.get('backend_sequences')
        if sequences is not None:
            observed = feedback.get('applied_command_sequences')
            if observed != sequences:
                if now-command['submitted_source_time'] > .1:
                    raise ValueError('fake command application identity unavailable after 0.1 source second')
                return
        evidence = dict(raw_feedback=deepcopy(feedback), firmware_ack='unsupported',
                        application_evidence='physx_readback_sequence' if sequences else 'SDK_return_and_fresh_feedback',
                        backend_sequences=deepcopy(sequences))
        if self.disabled_grippers:
            evidence['disabled_right_gripper']=dict(model_m=self.config['disabled_right_gripper_model_m'],
                device_mm=self.config['disabled_right_gripper_device_mm'],source='profile_max_opening_not_measured',sdk_output=False)
        if self.applied_check is not None:
            extra = self.applied_check(command, feedback)
            if extra is None:
                return
            evidence['device_application'] = extra
        acknowledge = self.loop.acknowledge_external_command if command.get('kind')=='initial_hold' else self.loop.acknowledge
        applied=command['final_device14']
        if self.target_interpreter is not None:
            _,applied=self._native_target(feedback['sample_time_s'])
            evidence['application_evidence']='prepared_reference_evaluation_and_fresh_feedback_not_firmware_ACK'
        acknowledge(command['command_id'], applied_device14=applied,
                    source_time=feedback['sample_time_s'], feedback=evidence)
        self.outstanding = None

    def _native_target(self, source_time):
        from arx_env.joint_head_jaw_mapping import command_gap_mm
        reference=self.prepared_reference if self.prepared_reference is not None else self.target_interpreter.reference
        q,_,_=reference.sample([source_time-self.native_start]);target=q[0].tolist()
        device=target[:];device[6]=command_gap_mm('left',target[6]);device[13]=command_gap_mm('right',target[13])
        return target,device

    def _prepare_native_response(self,response,request):
        """Model-query worker only. No SDK calls, no Owner state mutation."""
        import numpy as np
        from .motion_reference import prepare_reference_process
        if not isinstance(response,dict)or response.get('request_id')!=request['request_id']:
            raise ValueError('model response request identity mismatch')
        raw=np.asarray(response.get('action14'))
        if raw.shape!=(60,14)or raw.dtype.kind not in 'fiu'or not np.isfinite(raw).all():
            raise ValueError('model response requires finite action14 [60,14]')
        held=list(self.initial_device14)
        converted=[self.adapter.decode(row,held)['final_device14']for row in raw[:self.config['execute_steps']]]
        initial=(self.target_interpreter.reference.sample([self.owner.control_time()-self.native_start])[0][0].tolist()
                 if self.target_interpreter is not None else sdk_target(held))
        device=initial[:]
        from arx_env.joint_head_jaw_mapping import command_gap_mm
        device[6]=command_gap_mm('left',initial[6]);device[13]=command_gap_mm('right',initial[13])
        source=dict(device_targets14=[device,*converted],duration_s=len(converted)/30.,
            kind='online_prediction_chunk',
            model_id=self.config.get('model_id'),calibration_profile_digest=self.config.get('calibration_profile_digest'))
        reference=prepare_reference_process(source,initial,self.sdk_limits,
            source_sha256=digest(dict(request_id=request['request_id'],targets=converted)))
        return dict(response,_prepared_reference=reference)

    def _prepare_native_chunk(self,targets,now,request,response):
        from .motion_reference import PreparedReference
        if self.chunks_exhausted:return None
        reference=response.get('_prepared_reference')
        if not isinstance(reference,PreparedReference)or reference.metadata['source_sha256']!=digest(dict(request_id=request['request_id'],targets=targets)):
            raise ValueError('online chunk must be prepared by its CPU query worker before Owner admission')
        origin=now+.02
        if origin+reference.duration_s>self.started+self.config['duration_s']:
            self.chunks_exhausted=True
            return None  # Keep the prior zero-speed endpoint, never truncate a moving chunk.
        self.native_start=origin
        self.target_interpreter=self.owner.make_target_interpreter(reference,'both',origin,previous=self.target_interpreter)
        self._emit('prepared_chunk',dict(reference=reference.metadata,source_time_origin_s=origin,
            original_prediction_period_s=self.loop.period,actual_knot_times_s=(origin+reference.breaks[1:]).tolist()))
        self._queue_native(now,self.latest_feedback)
        return (origin+reference.breaks[1:]).tolist()

    def _queue_native(self,now,feedback):
        if self.owner.shared_target_profile:
            if self.target_interpreter is None:
                self.target_interpreter=self.owner.make_target_interpreter(self.prepared_reference,'left',self.native_start,
                    integral=bool(self.config.get('reference_integral_feedback')or self.owner.shared_target_profile=='x5_shared_precision_v1'),
                    feedforward=self.reference_feedforward)
            result=self.target_interpreter.tick(now,feedback)
            if result is None:return
            packet,receipt=result['packet'],result['admission']
            relative=[t-self.native_start for t in packet['host_times_s']]
        else:
            packet=self.prepared_reference.packet(now-self.native_start)
            relative=packet.pop('relative_times_s')
            packet.update(host_times_s=[self.native_start+t for t in relative],active_arm='left')
        before=result['submitted_monotonic_s']if self.owner.shared_target_profile else self.owner.control_time()
        if self.reference_integral is not None and not self.owner.shared_target_profile:
            sample=feedback['sample_time_s'];target,_=self._native_target(sample)
            packet['torque_bias_sdk6']=self.reference_integral.packet(before,sample,
                [a-b for a,b in zip(target[:6],feedback['qpos'][:6])],packet['host_times_s'])
            if self.reference_feedforward is not None:
                packet['torque_bias_sdk6']=self.reference_feedforward.packet(packet['host_times_s'],packet['torque_bias_sdk6'])
        try:
            self.owner.command_gate()
            if not self.owner.shared_target_profile:receipt=self.owner.backend.queue_reference(packet)
        except Exception:
            if self.owner.epoch!=self._epoch or self.owner.stop_requested or self.status=='STOPPED':
                self.revoke('control reference revoked');return
            raise
        returned=result['returned_monotonic_s']if self.owner.shared_target_profile else self.owner.control_time();tick=self.actual_submission_count
        self.actual_submission_count+=1
        if self.prepared_reference is not None:
            self.loop.submitted+=1;self.native_pending.append(returned)
        self.native_queued_until=packet['host_times_s'][-1]
        self.native_next_upload=now+.1
        # One row represents the actual native queue call, not fictitious per-tick SDK sends.
        target,device=self._native_target(packet['host_times_s'][0])
        meta=dict(reference_id=packet['reference_id'],frame_count=len(relative),
            first_host_s=packet['host_times_s'][0],last_host_s=packet['host_times_s'][-1],
            admission=deepcopy(receipt),command_semantics='SDK_future_window_admission_not_point_arrival')
        self._record('command',dict(final_device14=device,target_sdk14=target,tick=tick,
            send_monotonic_s=before,send_returned_monotonic_s=returned,send_wall_ns=time.time_ns(),
            send_source_time_ns=round(before*1e9),
            feedback_at_send=deepcopy(feedback),owner_pid=os.getpid(),command_epoch=self.owner.epoch,
            runtime_nonce=self.owner.runtime_nonce,native_reference=meta))
        if not self.owner.shared_target_profile:
            self._record('native_reference_packet',dict(packet=packet,admission=receipt,
                send_monotonic_s=before,send_returned_monotonic_s=returned))
        if self.owner.shared_target_profile:
            self._record('native_window',dict(packet=packet,admission=receipt,
                send_monotonic_s=before,send_wall_ns=time.time_ns()))

    def _tick_native(self,now,feedback):
        self.loop.last_source_time=now
        if self.owner.epoch!=self._epoch:raise ValueError('native reference epoch revoked')
        sample=feedback['sample_time_s']
        while self.native_pending and self.native_pending[0]<=sample:
            self.native_pending.pop(0);self.loop.completed+=1
        if now>=self.started+self.execution_duration_s:
            if self.native_queued_until is None or self.native_queued_until<self.started+self.execution_duration_s:
                raise ValueError('native reference window does not cover the execution endpoint')
            self.status='READY';self.reason='prepared continuous reference completed'
            self.ready_since=now;self.loop.status='completed';self.loop.reason=self.reason
            self.loop.outcome=dict(success=None,reason='control_reference_study_not_task_acceptance')
            self._emit('episode_completed',dict(outcome=self.loop.outcome,completed_commands=self.loop.completed))
            self._finish_data('completed',self.reason)
            return self.status
        if self.native_queued_until is not None and now>self.native_queued_until-.015:
            raise ValueError('native reference window starved; stop before stale nonzero tail')
        if now>=self.native_next_upload:self._queue_native(now,feedback)
        return self.status

    def tick(self, now, feedback):
        from arx_env.joint_head_real_motion import _state
        if self.status in ('ARMED', 'STOPPED', 'FAULT'):
            return self.status
        try:
            q, _, sample, _ = _state(feedback)
            self.latest_feedback=feedback
            check_gripper_policy(self.owner,self.config,feedback)
            if not self.owner.feedback_healthy or not 0 <= now-sample <= .1:
                raise ValueError('fresh healthy feedback required during inference')
            if now < self.previous:
                raise ValueError('inference source clock regressed')
            self.previous = now
            if self.prepared_reference is not None and self.started is not None:
                self.execution_target,self.last_device_target=self._native_target(sample)
            elif self.target_interpreter is not None:
                self.execution_target,self.last_device_target=self._native_target(sample)
                self._queue_native(now,feedback)
            self.target_error = dict(device14=[None if self.disabled_grippers and i==13 else a-b
                for i,(a,b) in enumerate(zip(device_state(q), self.last_device_target))],
                unavailable_axes=[13] if self.disabled_grippers else [],policy='record_only') if self.execution_target is not None else None
            if self.status == 'READY':
                return self.status
            self.owner.command_gate()
            if tuple(tuple(pair) for pair in self.owner.cache['joint_limits']) != self.sdk_limits:
                raise ValueError('device joint limits changed during inference')
            if self.owner.cache.get('jaw_mapping') != self.jaw_identity:
                raise ValueError('jaw mapping changed during inference')
            # Recording keeps the invalid raw right readback. Only model-facing
            # state and completed-command history use the explicit assumption.
            self._record('feedback', dict(source_time=sample, raw=deepcopy(feedback), device_state14=device_state(q),
                unavailable_axes=[13] if self.disabled_grippers else []))
            if self.status == 'VERIFYING':
                self._prepare(now,feedback)
                return self.status
            if self.prepared_reference is not None:
                return self._tick_native(self.owner.control_time(),feedback)
            # Never acknowledge a command beyond the accepted episode deadline.
            deadline = self.started+self.config['duration_s']
            supply_timeout=self.config.get('reference_supply_timeout_s')
            if supply_timeout is not None and not self.chunks_exhausted:
                endpoint=(self.native_start+self.target_interpreter.reference.duration_s
                    if self.target_interpreter is not None else self.started)
                if now>endpoint+supply_timeout and now<deadline:
                    raise ValueError('reference supply stopped after planned zero-speed endpoint')
            if sample <= deadline+1e-9:
                self._completed(now, feedback)
            query_ready = (not self.chunks_exhausted and now < deadline and self.outstanding is None
                           and not self.loop._commands and not self.loop._query_pending)
            observation = self.observe(now, feedback) if query_ready else None
            try:
                support = self.support_observe(now, feedback) if self.support_observe else None
            except Exception as exc:
                support = {'error': str(exc)}  # Acceptance unavailable; does not interrupt control.
            now = self.owner.control_time()  # Observation copying must not extend the motion deadline.
            command = self.loop.tick(now, epoch=self.owner.epoch, observation=observation, support=support)
            if self.loop.status == 'completed':
                self.outstanding = None
                # Retain the real sensor timestamp; a <=30 ms coverage deficit
                # is accepted explicitly, never relabelled as the configured endpoint.
                if sample+self.config['duration_tolerance_s']+1e-9 < deadline:
                    self.status, self.reason = 'SETTLING', 'waiting for measured episode endpoint'
                    return self.status
                self.status, self.reason, self.ready_since = 'READY', f'{self.config["duration_s"]:g} source seconds completed', now
                self._finish_data('completed', self.reason)
            elif self.loop.status == 'stopped':
                raise ValueError(self.loop.reason)
            elif command is not None:
                self._submit(command, feedback)
            return self.status
        except Exception as exc:
            self.loop.stop(str(exc))
            self.status, self.reason = 'FAULT', str(exc)
            self._finish_data('failed', self.reason)
            self.owner.motion_fault(exc)
            self.status, self.reason = 'FAULT', str(exc)
            return self.status

    def snapshot(self):
        return dict(self.loop.snapshot(), runner_status=self.status, recording_error=self.recording_error,
                    recording_pending=self.recording_pending,actual_submission_count=self.actual_submission_count,
                    preparation_complete=self.preparation_complete)


def make_inference_runner(owner, proxy, feedback, config, **dependencies):
    return InferenceRunner(owner, proxy, feedback, config, **dependencies)
