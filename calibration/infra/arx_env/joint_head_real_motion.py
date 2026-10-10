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

from .joint_head_jaw_mapping import COMMAND, GAP_MM, command_sdk, feedback_gaps, hold_command_sdk, interpolate
from robot_workbench.joint_bounds import position_noise_rad, project_joint_positions


class MotionTrackingError(ValueError):
    """Trajectory stopped successfully; operator may acknowledge before replanning."""


ARM_SLICES = {"left": (0, 6), "right": (7, 13)}
JAW_INDICES = (6, 13)

def active_indices(side):
    return tuple(i for a, (start, end) in ARM_SLICES.items() if side in (a, "both") for i in range(start, end))

def arrival_tolerances(arm_tolerance=.05, arm_tolerances=None):
    """Left J1..J6 then right J1..J6, radians; never includes jaw slots."""
    values = tuple(arm_tolerances) if arm_tolerances is not None else (arm_tolerance,) * 12
    if len(values) != 12 or any(type(v) not in (int, float) or not math.isfinite(v) or not 0 < v <= .05 for v in values):
        raise ValueError('12 finite positive arm arrival tolerances <= 0.05 rad required')
    return values


def active_jaws(side):
    return tuple(end for a, (_, end) in ARM_SLICES.items() if side in (a, "both"))
DEFAULT_VELOCITY = 0.2
START_DRIFT = 0.01
JAW_DRIFT_MM = 2.0
MOVING_JAW_KINDS = ("home", "both_home", "jaw_target", "workbench")


def moving_jaw_indices(plan):
    return active_jaws(plan.active_arm) if plan.kind in MOVING_JAW_KINDS else ()


def command_gap(side, sdk):
    from .joint_head_jaw_mapping import command_gap_mm
    return command_gap_mm(side, sdk)


def project_arm_positions(q,joint_limits,*,environment,corrections=None):
    """Bound only the twelve arm positions; jaw values/policies stay separate."""
    values=_numbers(q,14,'qpos');limits=_limits(joint_limits);indices=active_indices('both')
    bounded,changes=project_joint_positions([values[i] for i in indices],[limits[i] for i in indices],environment=environment)
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
        command=project_arm_positions(command,joint_limits,environment=environment,corrections=corrections)
    return tuple(command)


def jaw_peak_scale(side):
    # SDK interpolation is linear; bound actual speed across every mapping segment.
    from .joint_head_jaw_mapping import command_gap_slope
    return command_gap_slope(side)


def positive(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be finite and positive")
    return float(value)


class StationaryWindow:
    """Position evidence over >=0.3 s; raw velocity is diagnostic only.

    Input times are monotonic host sample times, not SDK timestamps. Callers
    separately establish host freshness and progress of both SDK clocks.
    No sample gap may exceed 0.1 s. Range violations restart the window.
    """
    def __init__(self):
        self.samples = deque()
        self.last_diagnostic = None

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
            'position_thresholds': [2.0 if i in JAW_INDICES else .003 for i in range(14)],
            'velocity_thresholds': [.005 if i in JAW_INDICES else .06 for i in range(14)],
            'velocity_guard_enabled': False,
            'max_sample_gap_s': .1, 'required_window_duration_s': .3,
            'position_exceeded_indices': [], 'velocity_exceeded_indices': [],
            'reasons': [], 'stationary': False,
        }
        self.last_diagnostic = diagnostic
        if gap is not None and not 0 < gap <= .1:
            diagnostic['reasons'].append('nonmonotonic' if gap <= 0 else 'sample_gap')
            diagnostic['reset_window'] = self._window_diagnostic()
            self.reset()
        diagnostic['velocity_exceeded_indices'] = [i for i in range(14) if abs(v[i]) > diagnostic['velocity_thresholds'][i]]
        q = list(q)
        for i, gap in zip(JAW_INDICES, feedback_gaps(q)):
            q[i] = gap
        self.samples.append((sample_time_s, q))
        while len(self.samples) > 1 and self.samples[1][0] <= sample_time_s - .3:
            self.samples.popleft()
        diagnostic.update(self._window_diagnostic())
        diagnostic['position_exceeded_indices'] = [i for i in range(14) if diagnostic['position_span'][i] > diagnostic['position_thresholds'][i]]
        if diagnostic['position_exceeded_indices']:
            diagnostic['reasons'].append('position_span')
            diagnostic['reason'] = diagnostic['reasons'][0]
            self.reset()
            self.samples.append((sample_time_s, q))
            return False
        stationary = sample_time_s - self.samples[0][0] >= .3 - 1e-9
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

    @property
    def max_delta(self):
        return max(abs(self.q_target[i] - self.q_start[i]) for i in active_indices(self.active_arm))


def make_plan(active_arm, q_start, q_target, joint_limits, *, duration_s=None,
              max_velocity=None, kind="target", created_at_s=None, jaw_speed_mm_s=10.0,
              q_start_is_command=False,environment='physical'):
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
    if kind in ("home", "both_home") and any(abs(target[i]-command_sdk("left" if i == 6 else "right", 83)) > 1e-8 for i in jaws):
        raise ValueError("home requires selected jaws at mapped 83 mm opening")
    if kind == "jaw_target" and any(target[i] not in (start[i], command_start[i]) for i in indices):
        raise ValueError("jaw target keeps arm joints unchanged")
    delta = max(abs(target[i] - command_start[i]) for i in indices)
    max_velocity = positive(DEFAULT_VELOCITY if max_velocity is None else max_velocity, "max_velocity")
    jaw_speed_mm_s = positive(jaw_speed_mm_s, "jaw_speed_mm_s")
    minimum = max(1e-9, 1.875 * delta / max_velocity)
    jaw_speed = min(jaw_speed_mm_s/jaw_peak_scale("left" if i == 6 else "right") for i in jaws) if moving_jaws else 0.0
    if moving_jaws:
        minimum = max(minimum, *(1.875 * abs(target[i]-command_start[i]) * jaw_peak_scale("left" if i == 6 else "right") / jaw_speed_mm_s for i in moving_jaws))
    duration = minimum if duration_s is None else positive(duration_s, "duration_s")
    if duration < minimum - 1e-12:
        raise ValueError("duration is too short for the requested peak velocity")
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
            "requested_q_target":list(requested_target),"position_corrections":list(corrections)}
    return MotionPlan(plan_sha256(body), active_arm, start, target, limits, duration,
                      max_velocity, kind, created_at_s,
                      jaw_speed, jaw_speed_mm_s, q_start_is_command,environment,requested_target,corrections)


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
        duration = max(1e-9, *(1.875*abs(bounded_target[i]-command_start[i])/speed for i,speed in zip(indices,speeds)),
                       *(1.875*abs(target[i]-command_start[i])*jaw_peak_scale(side)/jaw_speed for side,i in zip(('left','right'), JAW_INDICES)))
        plan = make_plan('both', previous, target, joint_limits, kind='workbench',
                         duration_s=duration, max_velocity=max(speeds), created_at_s=created, jaw_speed_mm_s=jaw_speed,
                         q_start_is_command=bool(plans),environment=environment)
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
    def __init__(self, plan, *, begin, send, stop, consumed=None, arm_tolerance=.05, arm_tolerances=None,environment=None):
        if not isinstance(plan, MotionPlan):
            raise TypeError("plan must be MotionPlan")
        self.environment=plan.environment if environment is None else environment
        if self.environment!=plan.environment:raise ValueError('motion plan position-boundary environment differs from Owner')
        checked = make_plan(plan.active_arm, plan.q_start, plan.requested_q_target, plan.joint_limits,
                            duration_s=plan.duration_s, max_velocity=plan.max_velocity,
                            kind=plan.kind, created_at_s=plan.created_at_s, jaw_speed_mm_s=plan.jaw_speed_mm_s,
                            q_start_is_command=plan.q_start_is_command,environment=self.environment)
        if checked != plan:
            raise ValueError("plan hash or canonical contents changed")
        self.arm_tolerances = arrival_tolerances(arm_tolerance, arm_tolerances)
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
        self.target_error = None
        self.continuation = False
        self.feedback_position_corrections=[];self.execution_start_corrections=[]

    def _fault(self, reason, *, cancelled=False):
        if self.status in ("FAULT", "STOPPED"):
            return self.status
        self.reason = str(reason)
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
        if sample > now or now - sample > 0.1:
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
                if now - self.sdk_progress[arm] > 0.1:
                    raise ValueError(f"{('left', 'right')[arm]} SDK feedback stopped advancing for 0.1 s")
        else:
            self.sdk_progress = [now, now]
        self.feedback_position_corrections=[]
        _check_pose(q, "measured qpos", self.plan.joint_limits, feedback=True,environment=self.environment,
                    corrections=self.feedback_position_corrections)
        for arm, (a, b) in ARM_SLICES.items():
            if self.plan.active_arm not in (arm, "both") and any(abs(q[i] - self.plan.q_start[i]) > START_DRIFT for i in range(a, b)):
                raise ValueError("inactive arm drift exceeds 0.01 rad")
        fixed_jaws = [i for i in JAW_INDICES if i not in moving_jaw_indices(self.plan)]
        if any(abs(feedback_gaps(q)[JAW_INDICES.index(i)] - self._start_gaps()[JAW_INDICES.index(i)]) > JAW_DRIFT_MM for i in fixed_jaws):
            raise ValueError("jaw drift exceeds 2 mm")
        self.last_sample, self.last_sdk = sample, sdk
        return q, v

    def _start_gaps(self):
        p = self.plan
        return ([command_gap(side, p.q_start[i]) for side, i in zip(("left", "right"), JAW_INDICES)]
                if p.q_start_is_command else feedback_gaps(p.q_start))

    def _check_start_drift(self, q):
        p = self.plan
        if any(abs(q[i] - p.q_start[i]) > (self.arm_tolerances[j] if self.continuation else START_DRIFT) for j,i in enumerate(active_indices("both"))) or any(abs(a-b) > JAW_DRIFT_MM for a,b in zip(feedback_gaps(q), self._start_gaps())):
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
                raise ValueError("control tick gap exceeds 0.1 s")
            q, v = self._feedback(now, state)
            stationary = self.stationary.update(self.last_sample, q, v)
            if self.status == "VERIFYING":
                self._check_start_drift(q)
                if now - self.verify_started > 3.0:
                    raise ValueError("stationary verification deadline exceeded")
                if not stationary or not all(self.sdk_advanced):
                    self.previous = now
                    return self.status
                measured = self.begin(tuple(q))  # Adapter returns its exact resampled hold.
                measured = tuple(q) if measured is None else _numbers(measured, 14, "begin pose")
                self._check_start_drift(measured)
                self.execution_start_corrections=[]
                self.execution_start = feedback_to_command(measured, self.plan.joint_limits,environment=self.environment,
                    corrections=self.execution_start_corrections)
                _check_pose(self.execution_start, "measured hold command", self.plan.joint_limits)
                self.execution_target = list(self.plan.q_target)
                indices, jaws = active_indices(self.plan.active_arm), active_jaws(self.plan.active_arm)
                for i in range(14):
                    if (i not in indices or self.plan.kind == "jaw_target") and not (i in moving_jaw_indices(self.plan)):
                        self.execution_target[i] = self.execution_start[i]
                self.started = now
                self.status = "RUNNING"
            elapsed = now - self.started
            indices, jaws = active_indices(self.plan.active_arm), active_jaws(self.plan.active_arm)
            arm_errors = {f"{'left' if i < 7 else 'right'}_J{i%7+1}": q[i]-self.execution_target[i] for i in indices}
            measured_gaps = feedback_gaps(q)
            jaw_errors_mm = {side: measured_gaps[n]-command_gap(side, self.execution_target[i])
                            for n, (side, i) in enumerate(zip(("left", "right"), JAW_INDICES)) if i in jaws}
            jaw_errors = {side: error/1000 for side, error in jaw_errors_mm.items()}
            self.target_error = dict(arm_rad=arm_errors, jaw_m=jaw_errors, stationary=stationary)
            if self.ready_since is None and elapsed > self.plan.duration_s + 3.0:
                worst = max(arm_errors, key=lambda j: abs(arm_errors[j]))
                raise MotionTrackingError(f"motion/settle deadline exceeded: {worst} error={arm_errors[worst]:+.6f} rad; "
                                 f"jaw_mm={ {k:round(v*1000,3) for k,v in jaw_errors.items()} }; stationary={stationary}")
            u = min(1.0, max(0.0, elapsed / self.plan.duration_s))
            # The polynomial can round above 1 near the endpoint. Bound its
            # progress, never the approved targets or measured feedback.
            s = min(1.0, max(0.0, u * u * u * (10.0 + u * (-15.0 + 6.0 * u))))
            desired = tuple((1.-s)*a + s*b for a, b in zip(self.execution_start, self.execution_target))
            error = max(abs(q[i] - desired[i]) for i in indices)
            jaw_error_mm = max(abs(measured_gaps[n]-command_gap(side, desired[i]))
                               for n, (side, i) in enumerate(zip(("left", "right"), JAW_INDICES)) if i in jaws)
            if error > 0.08 or jaw_error_mm > 4.0:
                if self.bad_tracking_since is None:
                    self.bad_tracking_since = now
                if now - self.bad_tracking_since >= 0.2:
                    raise MotionTrackingError("tracking error exceeded arm 0.08 rad or jaw 0.004 m for 0.2 s")
            else:
                self.bad_tracking_since = None
            jaw_error_mm = max(abs(value) for value in jaw_errors_mm.values())
            self.target_error["jaw_mm"] = jaw_errors_mm
            stable = all(abs(q[i]-desired[i]) <= self.arm_tolerances[i if i < 6 else i-1] for i in indices) and jaw_error_mm <= 2.0 and stationary
            if self.ready_since is not None:
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
            if elapsed >= self.plan.duration_s:
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
            self.send(desired)
            self.previous = now
            return self.status
        except Exception as error:
            return self._fault(error)
