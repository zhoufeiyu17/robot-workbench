"""Hardware-agnostic synchronized arm/jaw plans for the sole real X5 owner.

Requested speeds size smooth trajectories; hardware position limits remain
validated. No SDK, CAN, or robot imports belong in this module.
"""
from dataclasses import dataclass
from collections import deque
import hashlib
import json
import math
import time

from .joint_head_jaw_mapping import COMMAND, GAP_MM, command_sdk, feedback_gaps, hold_command_sdk, interpolate,command_max_mm
from robot_workbench.joint_bounds import position_noise_rad, project_joint_positions, inspect_joint_feedback, JointPositionBoundaryError
from robot_workbench.execution_settings import Guard,control_parameters,defaults,policy_hash,STATIONARY_PROFILE,DEFAULT_JOINT_TOLERANCE_RAD

_stationary_parameters=control_parameters(defaults())
_stationary_guard=Guard.bind(dict(revision=None,policy_hash=policy_hash(_stationary_parameters),parameters=_stationary_parameters),{})


class ControlTickGapError(ValueError):
    """A stopped transient control stall may be explicitly acknowledged later."""
    def __init__(self,gap_s):
        self.gap_s=gap_s
        super().__init__(f'control tick gap exceeds 0.1 s; observed_gap_ms={gap_s*1000:.3f}')


class MotionTrackingError(ValueError):
    """Trajectory stopped successfully; operator may acknowledge before replanning."""


ARM_SLICES = {"left": (0, 6), "right": (7, 13)}
JAW_INDICES = (6, 13)

def active_indices(side):
    return tuple(i for a, (start, end) in ARM_SLICES.items() if side in (a, "both") for i in range(start, end))

def arrival_tolerances(arm_tolerance=DEFAULT_JOINT_TOLERANCE_RAD, arm_tolerances=None):
    """Left J1..J6 then right J1..J6, radians; never includes jaw slots."""
    values = tuple(arm_tolerances) if arm_tolerances is not None else (arm_tolerance,) * 12
    if len(values) != 12 or any(type(v) not in (int, float) or not math.isfinite(v) or not 0 < v <= .05 for v in values):
        raise ValueError('12 finite positive arm arrival tolerances <= 0.05 rad required')
    return values


def active_jaws(side):
    return tuple(end for a, (_, end) in ARM_SLICES.items() if side in (a, "both"))
DEFAULT_VELOCITY = 0.2
START_DRIFT = DEFAULT_JOINT_TOLERANCE_RAD
JAW_DRIFT_MM = 2.0
MOVING_JAW_KINDS = ("home", "both_home", "jaw_target", "workbench")


def moving_jaw_indices(plan):
    return active_jaws(plan.active_arm) if plan.kind in MOVING_JAW_KINDS else ()


def command_gap(side, sdk):
    from .joint_head_jaw_mapping import command_gap_mm
    return command_gap_mm(side, sdk)


def project_arm_positions(q,joint_limits,*,environment,corrections=None):
    """Bound only the twelve arm positions; jaw values/policies stay separate."""
    return _arm_positions(q,joint_limits,environment,corrections,project_joint_positions)


def observe_arm_positions(q,joint_limits,*,environment,corrections=None):
    """Inspect manually displaced feedback without authorizing an out-of-range command."""
    return _arm_positions(q,joint_limits,environment,corrections,inspect_joint_feedback)


def _arm_positions(q,joint_limits,environment,corrections,policy):
    values=_numbers(q,14,'qpos');limits=_limits(joint_limits);indices=active_indices('both')
    try:bounded,changes=policy([values[i] for i in indices],[limits[i] for i in indices],environment=environment)
    except JointPositionBoundaryError as error:
        error.details['qpos_index']=indices[error.details['index']]
        raise
    result=list(values)
    for index,value in zip(indices,bounded):result[index]=value
    if corrections is not None:corrections.extend(dict(row,qpos_index=indices[row['index']]) for row in changes)
    return tuple(result)


def feedback_to_command(q, joint_limits=None,*,environment='physical',corrections=None):
    """Convert a measured hold/start into legal commands; leave raw feedback intact."""
    command = list(q)
    for side, i, gap in zip(("left", "right"), JAW_INDICES, feedback_gaps(q)):
        command[i] = hold_command_sdk(side, gap)
    if joint_limits is not None:
        command=observe_arm_positions(command,joint_limits,environment=environment,corrections=corrections)
    return tuple(command)


def jaw_peak_scale(side):
    # SDK interpolation is linear; bound actual speed across every mapping segment.
    from .joint_head_jaw_mapping import command_gap_slope
    return command_gap_slope(side)


def positive(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be finite and positive")
    return float(value)


def _joint_duration(deltas, speeds):
    """One quintic velocity/acceleration rule for single and multi-point plans."""
    return max(1e-9,*(1.875*abs(delta)/speed for delta,speed in zip(deltas,speeds)),
               *(math.sqrt((10./math.sqrt(3.))*abs(delta)/5.)for delta in deltas))


class StationaryWindow:
    """Position evidence over >=0.3 s; raw velocity is diagnostic only.

    Input times are monotonic host sample times, not SDK timestamps. Callers
    separately establish host freshness and progress of both SDK clocks.
    No sample gap may exceed 0.1 s. Range violations restart the window.
    """
    def __init__(self):
        self.guard=_stationary_guard
        self.samples = deque()
        self.last_diagnostic = None

    def _decision(self,observed,phase,sample_time):
        return self.guard.evaluate(dict(rule_id='G27',signal_id='stationary',observed=observed,
            source_time=sample_time,checked_time=time.monotonic()),phase)

    def reset(self):
        self.samples.clear()

    def _window_diagnostic(self):
        return {
            'window_start_s': self.samples[0][0] if self.samples else None,
            'window_end_s': self.samples[-1][0] if self.samples else None,
            'window_duration_s': self.samples[-1][0] - self.samples[0][0] if self.samples else 0.,
            'sample_count': len(self.samples),
            'position_span': [max(row[1][i] for row in self.samples) - min(row[1][i] for row in self.samples)
                              for i in range(14)] if self.samples else [0.] * 14,
        }

    def update(self, sample_time_s, qpos, qvel):
        if isinstance(sample_time_s, bool) or not isinstance(sample_time_s, (int, float)) or not math.isfinite(sample_time_s):
            raise ValueError("stationary sample time must be finite")
        q = _numbers(qpos, 14, "stationary qpos")
        v = _numbers(qvel, 14, "stationary qvel")
        gap = sample_time_s - self.samples[-1][0] if self.samples else None
        diagnostic = {
            'sample_time_s': sample_time_s, 'sample_gap_s': gap,
            'raw_qpos': list(q), 'raw_qvel': list(v),
            'axis_order': ['left_J1', 'left_J2', 'left_J3', 'left_J4', 'left_J5', 'left_J6', 'left_jaw',
                           'right_J1', 'right_J2', 'right_J3', 'right_J4', 'right_J5', 'right_J6', 'right_jaw'],
            'position_units': ['mm' if i in JAW_INDICES else 'rad' for i in range(14)],
            'velocity_units': ['SDK_units/s' if i in JAW_INDICES else 'rad/s' for i in range(14)],
            'raw_qpos_units': ['SDK_units' if i in JAW_INDICES else 'rad' for i in range(14)],
            'position_thresholds': [STATIONARY_PROFILE['jaw_span_mm'] if i in JAW_INDICES else STATIONARY_PROFILE['joint_span_rad'] for i in range(14)],
            'velocity_thresholds': [.005 if i in JAW_INDICES else .06 for i in range(14)],
            'velocity_guard_enabled': False,
            'max_sample_gap_s': STATIONARY_PROFILE['max_sample_gap_s'], 'required_window_duration_s': STATIONARY_PROFILE['window_s'],
            'position_exceeded_indices': [], 'velocity_exceeded_indices': [],
            'reasons': [], 'stationary': False,
        }
        self.last_diagnostic = diagnostic
        diagnostic['guard_decisions']=[]
        if gap is not None:
            diagnostic['guard_decisions'].append(self._decision(gap,'stationary_gap',sample_time_s))
        if gap is not None and diagnostic['guard_decisions'][-1]['verdict']=='reject':
            diagnostic['reasons'].append('nonmonotonic' if gap <= 0 else 'sample_gap')
            diagnostic['reset_window'] = self._window_diagnostic()
            self.reset()
        diagnostic['velocity_exceeded_indices'] = [i for i in range(14) if abs(v[i]) > diagnostic['velocity_thresholds'][i]]
        q = list(q)
        for i, gap in zip(JAW_INDICES, feedback_gaps(q)):
            q[i] = gap
        self.samples.append((sample_time_s, q))
        while len(self.samples) > 1 and self.samples[1][0] <= sample_time_s - STATIONARY_PROFILE['window_s']:
            self.samples.popleft()
        diagnostic.update(self._window_diagnostic())
        diagnostic['position_exceeded_indices'] = [i for i in range(14) if diagnostic['position_span'][i] > diagnostic['position_thresholds'][i]]
        span=self._decision(diagnostic['position_span'],'stationary_span',sample_time_s)
        diagnostic['guard_decisions'].append(span)
        if span['verdict']=='reject':
            diagnostic['reasons'].append('position_span')
            diagnostic['reason'] = diagnostic['reasons'][0]
            self.reset()
            self.samples.append((sample_time_s, q))
            return False
        window=self._decision(sample_time_s-self.samples[0][0],'stationary_window',sample_time_s)
        diagnostic['guard_decisions'].append(window)
        stationary = window['verdict']=='pass'
        diagnostic['stationary'] = stationary
        diagnostic['reasons'].append('stationary' if stationary else 'window_not_ready')
        diagnostic['reason'] = diagnostic['reasons'][0]
        return stationary


def _numbers(value, n, name):
    if not isinstance(value, (list, tuple)) or len(value) != n:
        raise ValueError(f"{name} must contain {n} numbers")
    out = []
    for item in value:
        if isinstance(item, bool) or not isinstance(item, (int, float)) or not math.isfinite(item):
            raise ValueError(f"{name} must contain finite numbers")
        out.append(float(item))
    return tuple(out)


def _limits(value):
    if not isinstance(value, (list, tuple)) or len(value) != 14:
        raise ValueError("joint_limits must contain 14 [low, high] pairs")
    result = []
    for pair in value:
        if not isinstance(pair, (list, tuple)) or len(pair) != 2:
            raise ValueError("joint_limits must contain 14 [low, high] pairs")
        low, high = _numbers(pair, 2, "joint limit")
        if low >= high:
            raise ValueError("joint limits must be increasing")
        result.append((low, high))
    return tuple(result)


def _canonical(payload):
    return json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()


def plan_sha256(payload):
    return hashlib.sha256(_canonical(payload)).hexdigest()


def _check_pose(q, name, limits, *, feedback=False,environment='physical',corrections=None):
    if feedback:
        q = feedback_to_command(q, limits,environment=environment,corrections=corrections)
    for index, (value, (low, high)) in enumerate(zip(q, limits)):
        if not low <= value <= high:
            raise ValueError(f"{name}[{index}] outside SDK limit")


@dataclass(frozen=True)
class MotionPlan:
    sha256: str
    active_arm: str
    q_start: tuple
    q_target: tuple
    joint_limits: tuple
    duration_s: float
    max_velocity: float = DEFAULT_VELOCITY
    kind: str = "target"
    created_at_s: float = 0.0
    max_jaw_velocity: float = 0.0
    jaw_speed_mm_s: float = 10.0
    q_start_is_command: bool = False
    environment: str = 'physical'
    requested_q_target: tuple = ()
    position_corrections: tuple = ()
    joint_velocity_limits: tuple = ()

    @property
    def max_delta(self):
        return max(abs(self.q_target[i] - self.q_start[i]) for i in active_indices(self.active_arm))


def make_plan(active_arm, q_start, q_target, joint_limits, *, duration_s=None,
              max_velocity=None, kind="target", created_at_s=None, jaw_speed_mm_s=10.0,
              q_start_is_command=False,environment='physical',joint_velocity_limits=None):
    """Seal synchronized joint/jaw interpolation at the caller's requested speeds."""
    if active_arm not in (*ARM_SLICES, "both"):
        raise ValueError("active_arm must be left or right")
    if kind not in ("target", "home", "arm_home", "jaw_target", "both_home", "workbench", "both_arm_home"):
        raise ValueError("unsupported plan kind")
    if kind != "jaw_target" and (active_arm == "both") != (kind in ("both_home", "workbench", "both_arm_home")):
        raise ValueError("both arms only supported for an explicitly reviewed both_home")
    start = _numbers(q_start, 14, "q_start")
    if type(q_start_is_command) is not bool:
        raise ValueError("q_start_is_command must be boolean")
    limits = _limits(joint_limits)
    position_noise_rad(environment)
    start_changes=[];target_changes=[]
    command_start = start if q_start_is_command else feedback_to_command(start, limits,environment=environment,corrections=start_changes)
    requested_target = _numbers(q_target, 14, "q_target")
    target=project_arm_positions(requested_target,limits,environment=environment,corrections=target_changes)
    _check_pose(start, "q_start", limits, feedback=not q_start_is_command,environment=environment)
    indices, jaws = active_indices(active_arm), active_jaws(active_arm)
    _check_pose(target, "q_target", limits)
    moving_jaws = jaws if kind in MOVING_JAW_KINDS else ()
    fixed = [i for i in range(14) if i not in indices and i not in moving_jaws]
    if any(target[i] not in (start[i], command_start[i]) for i in fixed):
        raise ValueError("inactive arm and jaws must remain unchanged")
    if kind in ("home", "both_home", "arm_home", "both_arm_home") and any(target[i] != 0.0 for i in indices):
        raise ValueError("home requires exactly zero for all active-arm joints")
    if kind in ("home", "both_home") and any(abs(target[i]-command_sdk("left" if i == 6 else "right", command_max_mm("left" if i==6 else "right"))) > 1e-8 for i in jaws):
        raise ValueError("home requires selected jaws at their mapped maximum opening")
    if kind == "jaw_target" and any(target[i] not in (start[i], command_start[i]) for i in indices):
        raise ValueError("jaw target keeps arm joints unchanged")
    delta = max(abs(target[i] - command_start[i]) for i in indices)
    max_velocity = positive(DEFAULT_VELOCITY if max_velocity is None else max_velocity, "max_velocity")
    velocities=(_numbers(joint_velocity_limits,12,'joint_velocity_limits')if joint_velocity_limits is not None
                else(max_velocity,)*12)
    if any(speed<=0 or speed>max_velocity+1e-12 for speed in velocities):
        raise ValueError('per-joint velocity limits must be positive and within the declared maximum')
    jaw_speed_mm_s = positive(jaw_speed_mm_s, "jaw_speed_mm_s")
    # The common point primitive is quintic. Bound its analytic acceleration as
    # well as velocity before review, including short moves at high speed.
    minimum = _joint_duration([target[i]-command_start[i]for i in indices],
                              [velocities[i if i<6 else i-1]for i in indices])
    jaw_speed = min(jaw_speed_mm_s/jaw_peak_scale("left" if i == 6 else "right") for i in jaws) if moving_jaws else 0.0
    if moving_jaws:
        minimum = max(minimum, *(1.875 * abs(target[i]-command_start[i]) * jaw_peak_scale("left" if i == 6 else "right") / jaw_speed_mm_s for i in moving_jaws))
    duration = minimum if duration_s is None else positive(duration_s, "duration_s")
    if duration < minimum - 1e-12:
        raise ValueError("duration is too short for the requested velocity/acceleration bounds")
    if created_at_s is None:
        created_at_s = time.monotonic()
    if isinstance(created_at_s, bool) or not isinstance(created_at_s, (int, float)) or not math.isfinite(created_at_s):
        raise ValueError("created_at_s must be finite")
    created_at_s = float(created_at_s)
    corrections=tuple([dict(row,source='q_start') for row in start_changes]+[dict(row,source='q_target') for row in target_changes])
    body = {"active_arm": active_arm, "q_start": list(start), "q_target": list(target),
            "joint_limits": [list(pair) for pair in limits], "duration_s": duration,
            "max_velocity": max_velocity, "kind": kind, "created_at_s": created_at_s,
            "max_jaw_velocity": jaw_speed, "jaw_speed_mm_s": jaw_speed_mm_s,
            "q_start_is_command": q_start_is_command,"environment":environment,
            "requested_q_target":list(requested_target),"position_corrections":list(corrections),
            "joint_velocity_limits":list(velocities)}
    return MotionPlan(plan_sha256(body), active_arm, start, target, limits, duration,
                      max_velocity, kind, created_at_s,
                      jaw_speed, jaw_speed_mm_s, q_start_is_command,environment,requested_target,corrections,velocities)


def make_workbench_plans(q_start, waypoints, joint_limits, *, created_at_s=None,environment='physical'):
    if not isinstance(waypoints, list) or not waypoints:
        raise ValueError('nonempty workbench waypoints required')
    previous = _numbers(q_start, 14, 'q_start')
    indices = active_indices('both')
    plans, created = [], time.monotonic() if created_at_s is None else created_at_s
    for waypoint in waypoints:
        if not isinstance(waypoint, dict) or set(waypoint) - {'q', 'speeds', 'q12', 'speeds12', 'jaw_gap_mm', 'jaw_speed_mm_s'}:
            raise ValueError('unsupported workbench waypoint fields')
        if ('q' in waypoint and 'q12' in waypoint) or ('speeds' in waypoint and 'speeds12' in waypoint):
            raise ValueError('duplicate joint waypoint fields')
        q = _numbers(waypoint.get('q12', waypoint.get('q')), 12, 'q12')
        speeds = _numbers(waypoint.get('speeds12', waypoint.get('speeds')), 12, 'speeds12')
        for speed in speeds:
            positive(speed, 'joint speed')
        jaw_speed = positive(waypoint.get('jaw_speed_mm_s', 10.), 'jaw_speed_mm_s')
        command_start = previous if plans else feedback_to_command(previous, joint_limits,environment=environment)
        target = list(command_start)
        for i,value in zip(indices,q):
            target[i] = value
        if 'jaw_gap_mm' in waypoint:
            gaps = _numbers(waypoint['jaw_gap_mm'], 2, 'jaw_gap_mm')
            for side, i, gap in zip(('left', 'right'), JAW_INDICES, gaps):
                target[i] = command_sdk(side, gap)
        bounded_target=project_arm_positions(target,joint_limits,environment=environment)
        duration = max(_joint_duration([bounded_target[i]-command_start[i]for i in indices],speeds),
                       *(1.875*abs(target[i]-command_start[i])*jaw_peak_scale(side)/jaw_speed for side,i in zip(('left','right'), JAW_INDICES)))
        plan = make_plan('both', previous, target, joint_limits, kind='workbench',
                         duration_s=duration, max_velocity=max(speeds), created_at_s=created, jaw_speed_mm_s=jaw_speed,
                         q_start_is_command=bool(plans),environment=environment,joint_velocity_limits=speeds)
        plans.append(plan)
        previous = plan.q_target
    return tuple(plans)


def _state(state):
    if not isinstance(state, dict):
        raise ValueError("feedback state must be a mapping")
    qpos = _numbers(state.get("qpos"), 14, "qpos")
    qvel = _numbers(state.get("qvel"), 14, "qvel")
    sample = state.get("sample_time_s")
    sdk = _numbers(state.get("sdk_timestamps"), 2, "sdk_timestamps")
    if isinstance(sample, bool) or not isinstance(sample, (int, float)) or not math.isfinite(sample):
        raise ValueError("sample_time_s must be finite")
    if any(t < 0 for t in sdk):
        raise ValueError("sdk_timestamps must be nonnegative")
    return qpos, qvel, float(sample), sdk


class MotionRunner:
    """Owner-thread runner with stop-on-fault and one-shot plan consumption.

    ``begin`` sets measured hold after stationary verification. ``send`` receives
    the complete 14-element target each tick, so jaw preservation is explicit.
    All callbacks must run on the sole control-owner thread. SDK clocks have
    independent epochs: only their advancement is compared with host time.
    The owner must retain ``consumed`` for its session and invalidate plans on
    restart. Neither this module nor its hash certifies collision clearance.
    """
    def __init__(self, plan, *, begin, send, stop, consumed=None, arm_tolerance=DEFAULT_JOINT_TOLERANCE_RAD, arm_tolerances=None,environment=None,
                 execution_settings=None, disabled_grippers=(), interpreter_factory=None,command_start_factory=None,guard=None,record_event=None):
        if record_event is not None and not callable(record_event):raise ValueError('optional bounded recording callback required')
        self.record_event=record_event
        if not isinstance(plan, MotionPlan):
            raise TypeError("plan must be MotionPlan")
        self.environment=plan.environment if environment is None else environment
        if self.environment!=plan.environment:raise ValueError('motion plan position-boundary environment differs from Owner')
        checked = make_plan(plan.active_arm, plan.q_start, plan.requested_q_target, plan.joint_limits,
                            duration_s=plan.duration_s, max_velocity=plan.max_velocity,
                            kind=plan.kind, created_at_s=plan.created_at_s, jaw_speed_mm_s=plan.jaw_speed_mm_s,
                            q_start_is_command=plan.q_start_is_command,environment=self.environment,
                            joint_velocity_limits=plan.joint_velocity_limits or None)
        if checked != plan:
            raise ValueError("plan hash or canonical contents changed")
        from robot_workbench.execution_settings import defaults, validate
        self.configured_error_policy = execution_settings is not None
        self.error_settings = validate(execution_settings if execution_settings is not None else
            defaults('calibration', arrival_tolerances(arm_tolerance, arm_tolerances)), environment=self.environment)
        if guard is None:
            from robot_workbench.execution_settings import Guard,control_parameters,policy_hash,FEEDBACK_TIMEOUT_S
            parameters=control_parameters(self.error_settings,business='calibration')
            guard=Guard.bind(dict(revision=None,policy_hash=policy_hash(parameters),parameters=parameters),dict(environment=self.environment,feedback_timeout_s=FEEDBACK_TIMEOUT_S))
        self.guard=guard
        self.arm_tolerances = tuple(self.error_settings['arrival_tolerances_rad'])
        self.disabled_grippers = tuple(disabled_grippers)
        if any(side not in ARM_SLICES for side in self.disabled_grippers):
            raise ValueError('disabled grippers must name left or right')
        self.enabled_jaws = tuple(i for side, (_, i) in ARM_SLICES.items() if side not in self.disabled_grippers)
        self.tracking_exceeded_samples = 0
        self.maximum_tracking_error = dict(joint_rad=0., jaw_mm=0.)
        self.plan, self.begin, self.send, self.stop = plan, begin, send, stop
        self.consumed = consumed if consumed is not None else set()
        self.status, self.reason = "ARMED", ""
        self.started = self.previous = self.last_sample = None
        self.last_sdk = None
        self.sdk_progress = None
        self.sdk_advanced = [False, False]
        self.bad_tracking_since = self.settled_since = None
        self.ready_since = None
        self.hold_bad_since = None
        self.verify_started = None
        self.execution_start = None
        self.execution_target = None
        self.stationary = StationaryWindow()
        self.stationary.guard=self.guard
        self.target_error = None
        self.continuation = False
        self.feedback_position_corrections=[];self.execution_start_corrections=[]
        # ponytail: bounded per-motion evidence, stream externally if >10s traces are needed.
        self.feedback_rows=deque(maxlen=1024);self.feedback_row_count=0
        self.native_rows=deque(maxlen=128);self.last_native_stamp=None
        self.last_sent=None
        self.interpreter_factory=interpreter_factory
        self.command_start_factory=command_start_factory
        self.target_interpreter=None
        self.execution_duration_s=plan.duration_s
        self.prepared_reference=None
        self.reference_event=None
        self.feedback_event=None

    def send_reference(self, desired, now, state):
        if self.target_interpreter is not None:
            result=self.target_interpreter.tick(now,state)
            if result is not None and self.reference_event is not None:self.reference_event(result)
            return result
        return self.send(desired)

    def motion_record(self):
        return dict(schema='motion-feedback-record.v1',plan_sha256=self.plan.sha256,
            plan_duration_s=self.plan.duration_s,max_command_velocity_rad_s=self.plan.max_velocity,
            execution_duration_s=self.execution_duration_s,
            execution_start=self.execution_start,execution_target=self.execution_target,
            motion_started_s=self.started,status=self.status,target_error=self.target_error,
            columns=['control_time_s','sample_time_s','qpos14','qvel14','sdk_timestamps','previous_command14'],
            rows=list(self.feedback_rows),sample_count=self.feedback_row_count,
            truncated=self.feedback_row_count>self.feedback_rows.maxlen,native_samples=list(self.native_rows),
            native_scope='cached native output at its own timestamps; no additional SDK calls')

    def _fault(self, reason, *, cancelled=False):
        if self.status in ("FAULT", "STOPPED"):
            return self.status
        self.reason = str(reason)
        if self.target_interpreter is not None:self.target_interpreter.revoke()
        self.status = "STOPPED" if cancelled else "FAULT"
        try:
            self.stop(reason)
        except Exception as error:
            self.status, self.reason = "FAULT", f"{self.reason}; stop failed: {error}"
        return self.status

    def cancel(self, reason="operator cancelled"):
        self.consumed.add(self.plan.sha256)
        return self._fault(reason, cancelled=True)

    def _feedback(self, now, state):
        q, v, sample, sdk = _state(state)
        if not isinstance(now, (int, float)) or isinstance(now, bool) or not math.isfinite(now):
            raise ValueError("now must be finite")
        decision=self.guard.evaluate(dict(rule_id='G28',signal_id=self.plan.sha256,observed=now-sample,
            source_time=sample,checked_time=now),'feedback')
        self.last_feedback_guard=decision
        if decision['verdict']=='reject':
            raise ValueError("host feedback is older than 0.1 s")
        if self.last_sample is not None and sample < self.last_sample:
            raise ValueError("host feedback clock regressed")
        if self.last_sdk is not None:
            for arm, (old, new) in enumerate(zip(self.last_sdk, sdk)):
                if new < old:
                    raise ValueError("SDK feedback clock regressed")
                if new > old:
                    self.sdk_progress[arm] = now
                    self.sdk_advanced[arm] = True
                sdk_decision=self.guard.evaluate(dict(rule_id='G28',signal_id='sdk_'+str(arm),observed=now-self.sdk_progress[arm],
                    source_time=self.sdk_progress[arm],checked_time=now),'feedback')
                if sdk_decision['verdict']=='reject':
                    raise ValueError(f"{('left', 'right')[arm]} SDK feedback stopped advancing for 0.1 s")
        else:
            self.sdk_progress = [now, now]
        self.feedback_position_corrections=[]
        _check_pose(q, "measured qpos", self.plan.joint_limits, feedback=True,environment=self.environment,
                    corrections=self.feedback_position_corrections)
        for arm, (a, b) in ARM_SLICES.items():
            if not self.configured_error_policy and self.plan.active_arm not in (arm, "both") and any(abs(q[i] - self.plan.q_start[i]) > START_DRIFT for i in range(a, b)):
                raise ValueError(f"inactive arm drift exceeds {START_DRIFT:g} rad")
        fixed_jaws = [i for i in self.enabled_jaws if i not in moving_jaw_indices(self.plan)]
        if not self.configured_error_policy and any(abs(feedback_gaps(q)[JAW_INDICES.index(i)] - self._start_gaps()[JAW_INDICES.index(i)]) > JAW_DRIFT_MM for i in fixed_jaws):
            raise ValueError("jaw drift exceeds 2 mm")
        self.last_sample, self.last_sdk = sample, sdk
        if self.feedback_event is not None:self.feedback_event(now,state)
        self.feedback_rows.append((now,sample,q,v,sdk,self.last_sent))
        self.feedback_row_count+=1
        telemetry=state.get('controller_telemetry') or {}
        stamp=telemetry.get('sample_wall_ns')
        if stamp is not None and stamp!=self.last_native_stamp:
            self.native_rows.append(telemetry);self.last_native_stamp=stamp
        return q, v

    def _start_gaps(self):
        p = self.plan
        return ([command_gap(side, p.q_start[i]) for side, i in zip(("left", "right"), JAW_INDICES)]
                if p.q_start_is_command else feedback_gaps(p.q_start))

    def _check_start_drift(self, q):
        p = self.plan
        if self.continuation and not self.error_settings['arrival_enabled']:
            return  # The first review/start-drift gate remains independent of arrival policy.
        jaw_limit = self.error_settings['arrival_jaw_mm'] if self.continuation else JAW_DRIFT_MM
        if any(abs(q[i] - p.q_start[i]) > (self.arm_tolerances[j] if self.continuation else START_DRIFT) for j,i in enumerate(active_indices("both"))) or any(abs(a-b) > jaw_limit for i,a,b in zip(JAW_INDICES, feedback_gaps(q), self._start_gaps()) if i in self.enabled_jaws):
            raise ValueError("start pose drifted beyond review tolerance")

    def start(self, now, state, *, reviewed_deadline=None, review_age_s=None):
        if self.status != "ARMED" or self.plan.sha256 in self.consumed:
            raise ValueError("plan already consumed; no replay or resume")
        self.consumed.add(self.plan.sha256)
        try:
            if isinstance(now, bool) or not isinstance(now, (int, float)) or not math.isfinite(now):
                raise ValueError("now must be finite")
            age = now-self.plan.created_at_s if review_age_s is None else review_age_s
            if (now < self.plan.created_at_s or
                    (reviewed_deadline is None and not 0 <= age <= 30.0) or
                    (reviewed_deadline is not None and now > reviewed_deadline)):
                raise ValueError("plan expired before start")
            self.continuation = reviewed_deadline is not None
            q, v = self._feedback(now, state)
            self._check_start_drift(q)
            self.stationary.update(self.last_sample, q, v)
            if self.record_event is not None:self.record_event('execution_binding',dict(plan_sha256=self.plan.sha256,
                active_arm=self.plan.active_arm,q_start_sdk14=list(self.plan.q_start),q_target_sdk14=list(self.plan.q_target),
                duration_s=self.plan.duration_s,source_created_at_s=self.plan.created_at_s))
            self.verify_started = self.previous = now
            self.status = "VERIFYING"
            return self.status
        except Exception as error:
            return self._fault(error)

    def tick(self, now, state):
        if self.status not in ("VERIFYING", "RUNNING", "SETTLING", "HOLD_UNSETTLED", "READY"):
            return self.status
        try:
            if isinstance(now, bool) or not isinstance(now, (int, float)) or not math.isfinite(now):
                raise ValueError("now must be finite")
            if now < self.previous or now - self.previous > 0.1:
                raise ControlTickGapError(now-self.previous)
            q, v = self._feedback(now, state)
            stationary = self.stationary.update(self.last_sample, q, v)
            if self.status == "VERIFYING":
                self._check_start_drift(q)
                if now - self.verify_started > 3.0:
                    raise ValueError("stationary verification deadline exceeded")
                continuing_observation = self.continuation and not self.error_settings['arrival_enabled']
                if (not stationary and not continuing_observation) or not all(self.sdk_advanced):
                    self.previous = now
                    return self.status
                measured = self.begin(tuple(q))  # Adapter returns its exact resampled hold.
                measured = tuple(q) if measured is None else _numbers(measured, 14, "begin pose")
                self._check_start_drift(measured)
                self.execution_start_corrections=[]
                self.execution_start = feedback_to_command(measured, self.plan.joint_limits,environment=self.environment,
                    corrections=self.execution_start_corrections)
                if self.command_start_factory is not None:
                    commanded=self.command_start_factory()
                    if commanded is not None:self.execution_start=_numbers(commanded,14,'continuous command start')
                _check_pose(self.execution_start, "measured hold command", self.plan.joint_limits)
                self.execution_target = list(self.plan.q_target)
                indices, jaws = active_indices(self.plan.active_arm), active_jaws(self.plan.active_arm)
                for i in range(14):
                    if (i in JAW_INDICES and i not in self.enabled_jaws) or ((i not in indices or self.plan.kind == "jaw_target") and i not in moving_jaw_indices(self.plan)):
                        self.execution_target[i] = self.execution_start[i]
                self.started = now
                if self.interpreter_factory is not None:
                    from robot_workbench.target_interpreter import prepare_point
                    reference=self.prepared_reference or prepare_point(self.plan,self.execution_start,self.execution_target)
                    if self.prepared_reference is not None:
                        self.execution_start=reference.sample([0.])[0][0].tolist()
                        self.execution_target=reference.sample([reference.duration_s])[0][0].tolist()
                    self.execution_duration_s=reference.duration_s
                    self.target_interpreter=self.interpreter_factory(reference,self.plan.active_arm,now)
                self.status = "RUNNING"
            elapsed = now - self.started
            indices, jaws = active_indices(self.plan.active_arm), active_jaws(self.plan.active_arm)
            arm_errors = {f"{'left' if i < 7 else 'right'}_J{i%7+1}": q[i]-self.execution_target[i] for i in indices}
            measured_gaps = feedback_gaps(q)
            jaw_errors_mm = {side: measured_gaps[n]-command_gap(side, self.execution_target[i])
                            for n, (side, i) in enumerate(zip(("left", "right"), JAW_INDICES)) if i in jaws and i in self.enabled_jaws}
            jaw_errors = {side: error/1000 for side, error in jaw_errors_mm.items()}
            self.target_error = dict(arm_rad=arm_errors, jaw_m=jaw_errors, jaw_mm=jaw_errors_mm,
                                     disabled_grippers=list(self.disabled_grippers), stationary=stationary,
                                     feedback_guard=self.last_feedback_guard)
            arrival=self.guard.evaluate(dict(rule_id='G26',signal_id=self.plan.sha256,
                observed=dict(joint_errors_rad=[q[i]-self.execution_target[i]if i in indices else 0. for i in active_indices('both')],
                              jaw_errors_mm=[jaw_errors_mm.get(side,0.)for side in ('left','right')]),
                source_time=self.last_sample,checked_time=now),'arrival')
            self.target_error['arrival_guard']=arrival
            if self.error_settings['arrival_enabled'] and self.ready_since is None and elapsed > self.execution_duration_s + 3.0:
                worst = max(arm_errors, key=lambda j: abs(arm_errors[j]))
                tolerance = self.arm_tolerances[active_indices('both').index(indices[list(arm_errors).index(worst)])]
                joints_ok=not arrival['observed']['exceeded_joint_indices']
                jaws_ok=not arrival['observed']['exceeded_jaw_indices']
                confirmation_s=0. if self.settled_since is None else max(0.,now-self.settled_since)
                failure_kind=('target_tolerance_exceeded' if not joints_ok or not jaws_ok else
                    'not_stationary_before_deadline' if not stationary else 'arrival_not_confirmed_before_deadline')
                self.target_error['arrival_deadline']=dict(failure_kind=failure_kind,
                    instantaneous_joints_in_tolerance=joints_ok,instantaneous_jaws_in_tolerance=jaws_ok,
                    stationary=stationary,continuous_confirmation_s=confirmation_s,required_confirmation_s=.5,
                    settle_budget_s=3.,elapsed_s=elapsed)
                raise MotionTrackingError(f"motion/settle deadline exceeded: {worst} error={arm_errors[worst]:+.6f} rad; "
                                 f"arrival_tolerance={tolerance:.6f} rad (target error, not joint position limit); "
                                 f"jaw_mm={ {k:round(v*1000,3) for k,v in jaw_errors.items()} }; stationary={stationary}; "
                                 f"failure_kind={failure_kind}; continuous_confirmation={confirmation_s:.3f}/0.500 s")
            u = min(1.0, max(0.0, elapsed / self.execution_duration_s))
            # The polynomial can round above 1 near the endpoint. Bound its
            # progress, never the approved targets or measured feedback.
            s = min(1.0, max(0.0, u * u * u * (10.0 + u * (-15.0 + 6.0 * u))))
            # Even a constant at the exact SDK limit can round one ULP outside
            # it in (1-s)*a+s*b. Stay within the already validated endpoints.
            desired = tuple(min(max(a+(b-a)*s,min(a,b)),max(a,b))
                            for a, b in zip(self.execution_start, self.execution_target))
            if self.target_interpreter is not None:
                desired=tuple(self.target_interpreter.reference.sample([self.last_sample-self.started])[0][0].tolist())
            error = max(abs(q[i] - desired[i]) for i in active_indices('both'))
            jaw_error_mm = max((abs(measured_gaps[n]-command_gap(side, desired[i]))
                               for n, (side, i) in enumerate(zip(("left", "right"), JAW_INDICES)) if i in self.enabled_jaws), default=0.)
            self.maximum_tracking_error['joint_rad'] = max(self.maximum_tracking_error['joint_rad'], error)
            self.maximum_tracking_error['jaw_mm'] = max(self.maximum_tracking_error['jaw_mm'], jaw_error_mm)
            tracking=self.guard.evaluate(dict(rule_id='G25',signal_id=self.plan.sha256,
                observed=dict(joint_errors_rad=[q[i]-desired[i]for i in active_indices('both')],
                    jaw_errors_mm=[measured_gaps[n]-command_gap(side,desired[i])if i in self.enabled_jaws else 0.
                                   for n,(side,i)in enumerate(zip(('left','right'),JAW_INDICES))],
                    exceeded_for_s=0. if self.bad_tracking_since is None else now-self.bad_tracking_since),
                source_time=self.last_sample,checked_time=now),'tracking')
            if tracking['observed']['exceeded']:
                self.tracking_exceeded_samples += 1
                if self.bad_tracking_since is None:
                    self.bad_tracking_since = now
            else:
                self.bad_tracking_since = None
            self.target_error['tracking'] = dict(joint_rad=error, jaw_mm=jaw_error_mm,
                exceeded_samples=self.tracking_exceeded_samples, maximum=dict(self.maximum_tracking_error),guard_decision=tracking)
            if tracking['verdict']=='reject':
                raise MotionTrackingError(f"tracking error exceeded arm {self.error_settings['tracking_joint_rad']} rad or jaw "
                    f"{self.error_settings['tracking_jaw_mm']} mm for {self.error_settings['tracking_duration_s']} s"
                    f"; measured arm={error:.6f} rad, jaw={jaw_error_mm:.3f} mm")
            jaw_error_mm = max((abs(value) for value in jaw_errors_mm.values()), default=0.)
            if not self.error_settings['arrival_enabled']:
                if self.ready_since is None:
                    self.send_reference(desired,now,state)
                    self.last_sent=desired
                elif self.target_interpreter is not None:
                    self.send_reference(desired,now,state)
                if elapsed >= self.execution_duration_s:
                    self.status = 'READY'
                    if self.ready_since is None:
                        self.ready_since = now
                self.previous = now
                return self.status
            # During motion arrival follows the current reference; at the
            # endpoint the current reference equals the final target above.
            current_arrival=self.guard.evaluate(dict(rule_id='G26',signal_id=self.plan.sha256,
                observed=dict(joint_errors_rad=[q[i]-desired[i]if i in indices else 0. for i in active_indices('both')],
                              jaw_errors_mm=[measured_gaps[n]-command_gap(side,desired[i])if i in jaws and i in self.enabled_jaws else 0.
                                             for n,(side,i)in enumerate(zip(('left','right'),JAW_INDICES))]),
                source_time=self.last_sample,checked_time=now),'arrival')
            stable = current_arrival['verdict']=='pass' and stationary
            if self.ready_since is not None:
                if self.target_interpreter is not None:self.send_reference(desired,now,state)
                if not stable:
                    self.status = "HOLD_UNSETTLED"
                    self.settled_since = None
                    if self.hold_bad_since is None:
                        self.hold_bad_since = now
                elif self.status == "HOLD_UNSETTLED":
                    if self.settled_since is None:
                        self.settled_since = now
                    if now - self.settled_since >= 0.5:
                        self.status = "READY"
                        self.hold_bad_since = None
                self.previous = now
                return self.status  # SDK keeps the endpoint hold; no new sends.
            if elapsed >= self.execution_duration_s:
                self.status = "SETTLING"
                if stable:
                    if self.settled_since is None:
                        self.settled_since = now
                    if now - self.settled_since >= 0.5:
                        self.status = "READY"
                        self.ready_since = now
                        self.previous = now
                        return self.status
                else:
                    self.settled_since = None
            self.send_reference(desired,now,state)
            self.last_sent=desired
            self.previous = now
            return self.status
        except Exception as error:
            return self._fault(error)
