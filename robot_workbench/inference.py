"""Source-clock policy execution; the caller owns hardware, capture and confirmation.

The only background operation is one model query. No model result can call a
device. The Owner polls due commands and acknowledges completed applications.
"""
from collections import deque
from copy import deepcopy
import math
import queue
import threading
import time
import uuid

import numpy as np


def vector14(value, name='action'):
    raw = np.asarray(value)
    if raw.shape != (14,) or raw.dtype.kind not in 'fiu':
        raise ValueError(name + ' must contain 14 finite numbers')
    result = raw.astype(np.float64)
    if not np.isfinite(result).all():
        raise ValueError(name + ' must contain 14 finite numbers')
    return result


def positive(value, name):
    if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
        raise ValueError(name + ' must be finite and positive')
    return float(value)


def audit_dynamics(actions, previous, *, position_limits, velocity_limits, acceleration_limits, period_s=1/30):
    """Diagnostic only. All 14 axes keep their own units (rad or jaw mm)."""
    values = np.asarray(actions, dtype=np.float64)
    old = np.asarray(previous, dtype=np.float64).reshape(-1, 14)
    limits = np.asarray(position_limits, dtype=np.float64)
    def dynamic_limits(value):
        if not isinstance(value, (list, tuple, np.ndarray)) or len(value) != 14:
            raise ValueError('14 explicit dynamic limits required; None means unqualified axis')
        if any(v is not None and (type(v) not in (int, float) or not math.isfinite(v) or v <= 0) for v in value):
            raise ValueError('dynamic limits must be positive finite numbers or None')
        return np.asarray([float('inf') if v is None else v for v in value])
    velocity = dynamic_limits(velocity_limits)
    acceleration = dynamic_limits(acceleration_limits)
    if (values.ndim != 2 or values.shape[1] != 14 or not len(values)
            or not np.isfinite(values).all() or not np.isfinite(old).all()
            or limits.shape != (14, 2) or not np.isfinite(limits).all()
            or (limits[:, 0] > limits[:, 1]).any() or (velocity <= 0).any() or (acceleration <= 0).any()):
        raise ValueError('invalid finite action/limit diagnostic inputs')
    dt = positive(period_s, 'period')
    problems = []
    for row, axis in np.argwhere((values < limits[:, 0]) | (values > limits[:, 1])):
        problems.append(dict(kind='position', step=int(row), axis=int(axis), value=float(values[row, axis]),
                             lower=float(limits[axis, 0]), upper=float(limits[axis, 1])))
    joined = np.concatenate((old[-2:], values))
    prefix = min(2, len(old))
    for kind, order, bound in (('velocity', 1, velocity), ('acceleration', 2, acceleration)):
        derivative = np.diff(joined, n=order, axis=0) / dt**order
        for row, axis in np.argwhere(np.abs(derivative) > bound):
            step = int(row) + order - prefix
            if step >= 0:
                problems.append(dict(kind=kind, step=step, axis=int(axis), value=float(derivative[row, axis]),
                                     limit=float(bound[axis])))
    return dict(policy='record_only', collision_checked=False, problems=problems,
                unavailable_velocity_axes=np.flatnonzero(np.isinf(velocity)).tolist(),
                unavailable_acceleration_axes=np.flatnonzero(np.isinf(acceleration)).tolist())


class StableTraySupport:
    """Acceptance side channel, never included in a model request.

    The source adapter proves tray support and stability. It supplies source
    timestamps and its evidence identity; no verifier means an unknown result.
    """
    def __init__(self, stable_seconds, maximum_sample_gap_s):
        self.stable_seconds = positive(stable_seconds, 'stable support duration')
        self.maximum_gap = positive(maximum_sample_gap_s, 'support sample gap')
        self.first = self.last = self.latest = None

    def update(self, sample):
        if sample is None:
            return
        stamp = sample.get('source_time')
        if (type(stamp) not in (int, float) or not math.isfinite(stamp)
                or type(sample.get('tray_supported')) is not bool or type(sample.get('stable')) is not bool
                or not sample.get('evidence_source')):
            raise ValueError('support observation requires source time, booleans and evidence source')
        if self.last is not None and stamp <= self.last:
            return  # Repeated camera/physics samples cannot accumulate stable time.
        continuous = self.last is not None and stamp-self.last <= self.maximum_gap+1e-9
        qualified = sample['tray_supported'] and sample['stable']
        self.first = (self.first if continuous and self.first is not None else stamp) if qualified else None
        self.last, self.latest = stamp, deepcopy(sample)

    def result(self, source_time):
        if self.last is None or source_time-self.last > self.maximum_gap+1e-9:
            return dict(success=None, reason='tray_support_evidence_unavailable')
        duration = 0. if self.first is None else self.last-self.first
        return dict(success=self.first is not None and duration+1e-9 >= self.stable_seconds,
                    stable_source_seconds=duration, required_source_seconds=self.stable_seconds,
                    last_observation=deepcopy(self.latest))


class InferenceLoop:
    """Single Owner-thread API. predict runs in a daemon thread; all other callbacks are CPU-only.

    ``tick`` returns one command or None, never sends or waits. ``acknowledge``
    must follow a completed device interval, not a queued SDK call. Until that
    acknowledgement the next command is withheld. Model history contains only
    actual completed targets converted back to the declared model domain.
    """
    def __init__(self, *, predict, decode, device_to_model, audit=None, event=None,
                 duration_s=40., hz=30., execute_steps=10, seed=200000,
                 maximum_observation_age_s=2., support=None, prepare_chunk=None, prepare_response=None):
        self.predict, self.decode, self.device_to_model = predict, decode, device_to_model
        self.audit, self.event = audit, event or (lambda name, value: None)
        self.prepare_chunk=prepare_chunk
        self.prepare_response=prepare_response
        self.duration = positive(duration_s, 'episode duration')
        self.period = 1/positive(hz, 'command rate')
        self.maximum_age = positive(maximum_observation_age_s, 'observation age')
        if type(execute_steps) is not int or not 1 <= execute_steps <= 60:
            raise ValueError('execute_steps must be 1..60')
        if type(seed) is not int or not 0 <= seed < 2**63:
            raise ValueError('seed must be an integer in [0, 2**63)')
        self.execute_steps, self.seed, self.support = execute_steps, seed, support
        self.generation = 0
        self.status = 'idle'
        self.reason = ''
        self._result = queue.Queue(maxsize=1)
        self._query_pending = False
        self._commands = deque()
        self._outstanding = None
        self.history = deque(maxlen=60)
        self.applied = deque(maxlen=2)
        self.episode_id = self.epoch = self.started = self.last_source_time = None
        self.predictions = self.submitted = self.completed = self.discarded = 0
        self.external_completed = set()
        self.outcome = None
        self.support_error = None

    def start(self, *, epoch, source_time, episode_id=None):
        if self.status == 'running':
            raise ValueError('infer episode already running')
        self._clock(source_time)
        self.generation += 1
        self.epoch, self.started, self.last_source_time = epoch, source_time, source_time
        self.episode_id = episode_id or uuid.uuid4().hex
        self.status, self.reason, self.outcome = 'running', '', None
        self.support_error = None
        self._commands.clear()
        self._outstanding = None
        self.history.clear()
        self.applied.clear()
        self.predictions = self.submitted = self.completed = self.discarded = 0
        self.external_completed.clear()
        if self.support is not None:
            self.support.first = self.support.last = self.support.latest = None
        self._emit('episode_started', duration_source_s=self.duration, command_hz=1/self.period,
                   execute_steps=self.execute_steps, seed=self.seed, collision_check=False,
                   limits_policy='record_only')

    @staticmethod
    def _clock(value):
        if type(value) not in (int, float) or not math.isfinite(value):
            raise ValueError('finite device source clock required')

    def _emit(self, name, **fields):
        value = dict(episode_id=self.episode_id, epoch=self.epoch, generation=self.generation,
                     source_time=self.last_source_time, wall_monotonic_s=time.monotonic())
        value.update(fields)
        self.event(name, value)

    def stop(self, reason='operator_stop'):
        if self.status != 'running':
            return
        self.status, self.reason = 'stopped', reason
        self._emit('episode_stopped', reason=reason, abandoned_command_id=(self._outstanding or {}).get('command_id'))
        self.generation += 1
        self._commands.clear()
        self._outstanding = None

    def _launch(self, observation):
        stamp = observation.get('source_time')
        self._clock(stamp)
        if not 0 <= self.last_source_time-stamp <= self.maximum_age:
            raise ValueError('stale or future policy observation')
        vector14(observation.get('device_state14'), 'observation state')
        # Copy only the public observation envelope: task truth/acceptance cannot leak.
        obs = {key: deepcopy(observation[key]) for key in ('source_time', 'device_state14', 'images', 'frame_ids') if key in observation}
        request = dict(request_id=uuid.uuid4().hex, seed=(self.seed+self.predictions) % 2**63,
                       observation=obs, applied_history=np.asarray(list(self.history), dtype=np.float32).reshape(-1, 14))
        generation, epoch = self.generation, self.epoch
        self._query_pending = True
        self.predictions += 1
        self._emit('prediction_requested', request_id=request['request_id'], seed=request['seed'],
                   observation_source_time=stamp, frame_ids=deepcopy(obs.get('frame_ids')),
                   state_device14=vector14(obs['device_state14']).tolist(),
                   applied_history_model14=request['applied_history'].tolist())
        def run():
            started = time.monotonic()
            try:
                response, error = self.predict(request), None
                if self.prepare_response is not None:
                    preparing=time.monotonic()
                    response=self.prepare_response(response,request)
                    response['_preparation_wall_s']=time.monotonic()-preparing
            except Exception as exc:
                response, error = None, f'{type(exc).__name__}: {exc}'
            self._result.put((generation, epoch, request, response, error, time.monotonic()-started))
        threading.Thread(target=run, name='policy-infer', daemon=True).start()

    def _receive(self):
        try:
            generation, epoch, request, response, error, wall_s = self._result.get_nowait()
        except queue.Empty:
            return
        self._query_pending = False
        if generation != self.generation or epoch != self.epoch or self.status != 'running':
            self.discarded += 1
            self._emit('prediction_discarded', request_id=request['request_id'], reason='revoked_generation', model_wall_s=wall_s,
                       worker_audit=deepcopy(response.get('audit', {})) if isinstance(response,dict) else {})
            return
        if error:
            raise RuntimeError(error)
        if not isinstance(response, dict) or response.get('request_id') != request['request_id']:
            raise ValueError('model response request identity mismatch')
        values = np.asarray(response.get('action14'))
        if values.shape != (60, 14) or values.dtype.kind not in 'fiu' or not np.isfinite(values).all():
            raise ValueError('model response requires finite action14 [60,14]')
        age = self.last_source_time-request['observation']['source_time']
        if not 0 <= age <= self.maximum_age:
            self.discarded += 1
            self._emit('prediction_discarded', request_id=request['request_id'], reason='stale_observation',
                       observation_age_source_s=age, model_wall_s=wall_s,worker_audit=deepcopy(response.get('audit', {})))
            return
        decoded = [self.decode(row, request['observation']['device_state14']) for row in values[:self.execute_steps]]
        final = [vector14(row['final_device14']).tolist() for row in decoded]
        raw_device = [vector14(row['decoded_device14']).tolist() for row in decoded]
        previous = list(self.applied) or [request['observation']['device_state14']]
        diagnostics = self.audit(raw_device, previous) if self.audit else {'policy': 'record_only', 'problems': []}
        final_diagnostics = self.audit(final, previous) if self.audit else {'policy': 'record_only', 'problems': []}
        self._emit('prediction_received', request_id=request['request_id'], prediction_model14=values.tolist(),
                   conversions=deepcopy(decoded), audit=diagnostics, final_device_audit=final_diagnostics,
                   model_wall_s=max(0.,wall_s-response.get('_preparation_wall_s',0.)),
                   motion_preparation_wall_s=response.get('_preparation_wall_s',0.),prediction_end_to_end_wall_s=wall_s,
                   observation_age_source_s=age, worker_audit=deepcopy(response.get('audit', {})))
        due=None
        if self.prepare_chunk is not None:
            due=self.prepare_chunk(final,self.last_source_time,request,response)
            if due is None:
                self.discarded+=1
                self._emit('prediction_discarded',request_id=request['request_id'],reason='prepared_chunk_exceeds_remaining_episode')
                return
            if (len(due)!=len(final)or any(not math.isfinite(t)or t<self.last_source_time for t in due)
                    or any(b<=a for a,b in zip(due,due[1:]))):
                raise ValueError('prepared chunk requires ordered future source knot times')
        for index, target in enumerate(final):
            self._commands.append(dict(command_id=uuid.uuid4().hex, generation=generation, epoch=epoch,
                request_id=request['request_id'], chunk_index=index, due_source_time=due[index]if due is not None else self.last_source_time+index*self.period,
                prediction_model14=values[index].tolist(), conversion=deepcopy(decoded[index]), final_device14=target))

    def prime_reference(self, response, initial_device14):
        """Admit a preloaded reference before the first tick; never query a model."""
        if self.status!='running' or self.predictions or self._commands or self._outstanding or self._query_pending:
            raise ValueError('reference may be primed only once before execution')
        request=dict(request_id=response['request_id'],observation=dict(
            source_time=self.started,device_state14=deepcopy(initial_device14)))
        self._result.put((self.generation,self.epoch,request,response,None,0.))
        self._receive()
        self._emit('reference_preloaded',request_id=request['request_id'],model_called=False)

    def _update_support(self, sample):
        if self.support is None:
            return
        try:
            if sample is not None and sample.get('source_time', float('inf')) > self.last_source_time:
                raise ValueError('future support evidence')
            self.support.update(sample)
            if sample is not None:
                self.support_error = None
        except (ValueError, TypeError, AttributeError) as exc:
            # The acceptance side channel cannot abort policy control.
            self.support.first = self.support.last = self.support.latest = None
            self.support_error = str(exc)
            self._emit('support_observation_invalid', error=self.support_error)

    def tick(self, source_time, *, epoch, observation=None, support=None):
        self._clock(source_time)
        if self.status == 'running' and source_time < self.last_source_time:
            self.stop('source_clock_reversed')
            return None
        self.last_source_time = source_time
        if self.status == 'running' and epoch != self.epoch:
            self.stop('owner_epoch_changed')
        if self.status == 'running':
            self._update_support(support)
        if self.status == 'running' and source_time-self.started+1e-9 >= self.duration:
            self.outcome = self.support.result(source_time) if self.support else {'success': None, 'reason': 'tray_support_evidence_unavailable'}
            self.status, self.reason = 'completed', 'episode_duration_elapsed'
            self._emit('episode_completed', outcome=deepcopy(self.outcome), completed_commands=self.completed)
            self.generation += 1
            self._commands.clear()
            self._outstanding = None
        try:
            self._receive()
            if self.status != 'running':
                return None
            if self._outstanding is not None:
                return None
            if self._commands and source_time+1e-9 >= self._commands[0]['due_source_time']:
                self._outstanding = self._commands.popleft()
                self._outstanding['submitted_source_time'] = source_time
                self.submitted += 1
                self._emit('command_submitted', **deepcopy(self._outstanding))
                return deepcopy(self._outstanding)
            if not self._commands and not self._query_pending and observation is not None:
                self._launch(observation)
        except Exception as exc:
            self.stop(f'{type(exc).__name__}: {exc}')
        return None

    def acknowledge(self, command_id, *, applied_device14, source_time, feedback=None):
        self._clock(source_time)
        command = self._outstanding
        if self.status != 'running' or command is None or command_id != command['command_id']:
            raise ValueError('unknown or revoked completed command')
        if not command['submitted_source_time'] <= source_time <= self.started+self.duration+1e-9:
            raise ValueError('completion time outside submitted episode interval')
        device = vector14(applied_device14, 'applied command')
        model = vector14(self.device_to_model(device), 'completed model-domain command')
        self._emit('command_applied', command_id=command_id, completion_source_time=source_time,
                   applied_device14=device.tolist(), applied_model14=model.tolist(), feedback=deepcopy(feedback))
        self.applied.append(device.tolist())
        self.history.append(model.tolist())
        self.completed += 1
        self._outstanding = None

    def acknowledge_external_command(self, command_id, *, applied_device14, source_time, feedback=None):
        """Record a genuinely completed Owner hold interval, never a fabricated prediction."""
        self._clock(source_time)
        if (self.status != 'running' or not isinstance(command_id, str) or not command_id
                or command_id in self.external_completed or not self.started <= source_time <= self.started+self.duration+1e-9):
            raise ValueError('invalid or duplicate completed external command')
        device = vector14(applied_device14, 'completed external command')
        model = vector14(self.device_to_model(device), 'completed external model-domain command')
        self._emit('command_applied', command_id=command_id, completion_source_time=source_time,
                   applied_device14=device.tolist(), applied_model14=model.tolist(), feedback=deepcopy(feedback), external=True)
        self.applied.append(device.tolist())
        self.history.append(model.tolist())
        self.external_completed.add(command_id)

    def snapshot(self):
        elapsed = 0. if self.started is None else max(0., (self.last_source_time or self.started)-self.started)
        return dict(status=self.status, reason=self.reason, episode_id=self.episode_id, epoch=self.epoch,
                    duration_source_s=self.duration, elapsed_source_s=min(elapsed, self.duration),
                    remaining_source_s=max(0., self.duration-elapsed), predictions=self.predictions,
                    submitted_commands=self.submitted, completed_commands=self.completed,
                    completed_external_commands=len(self.external_completed),
                    history_length=len(self.history), discarded_predictions=self.discarded,
                    model_pending=self._query_pending, pending_commands=len(self._commands),
                    outstanding_command_id=(self._outstanding or {}).get('command_id'), outcome=deepcopy(self.outcome),
                    support_error=self.support_error)
