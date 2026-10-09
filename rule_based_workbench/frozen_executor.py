"""CPU-only frozen-target follower. No SDK, network, thread, sleep or hardware enablement.

An owner calls start() once, then tick() at its local control rate (e.g. 100 Hz).
The sink receives physical14 dictionaries unchanged; a future owner boundary owns
SDK conversion. Feedback is {sample_monotonic_s, state: physical14}. No automatic
interpolation, per-waypoint settling, skipped targets or restart. Bounded host
jitter delays the remaining schedule rather than causing burst catch-up.
All methods and callbacks belong to one owner thread; callbacks must be bounded.
An external watchdog is still needed if the owner loop or a native SDK call blocks.
"""
from bisect import bisect_right
from copy import deepcopy
import math

from contract import ARM_KEYS, GRIPPER_KEYS, validate_physical14, validate_physical_feedback
from integration import DualGpuPipeline, _digest, _finite


def validate_plan_evidence(plan, source):
    if plan.get('planning_validation',{}).get('schema_version')=='physical_curobo_motiongen_path_validation.v1':
        from physical_motiongen import validate_evidence
        validate_evidence(plan,source)
        return
    if plan.get('simulation') == {'validated': True, 'outcome': 'success'}:
        return
    receipt = plan.get('planning_validation', {})
    if not isinstance(receipt, dict):
        raise ValueError('geometric validation receipt must be an object')
    step = receipt.get('max_sample_step_rad')
    coverage = receipt.get('collision_coverage')
    if (plan.get('simulation') != {'validated': False, 'outcome': 'not_run'}
            or receipt.get('schema_version') != 'geometric_path_validation.v1'
            or receipt.get('passed') is not True
            or receipt.get('timeline_digest') != _digest(plan.get('ticks'))
            or receipt.get('timeline_digest') != plan.get('timeline_digest')
            or receipt.get('source_digest') != _digest(source)
            or not isinstance(coverage, list) or not all(isinstance(v,str) for v in coverage)
            or not {'base_link','left','right'} <= set(coverage)
            or type(step) not in (int,float) or not math.isfinite(step) or not 0 < step <= .01):
        raise ValueError('successful simulation or bound complete geometric path validation required')


class FrozenExecutor:
    def __init__(self, plan, source, *, sink, gate, stop, feedback_timeout_s=.1, source_validator=None, execution_policy='strict'):
        if execution_policy not in ('strict','observation'):
            raise ValueError('unknown execution policy')
        self.execution_policy=execution_policy
        if source_validator is None:
            bridge = DualGpuPipeline()
            bridge.ingest_source(source)
            bridge.accept_plan(plan)
        else:
            source_validator(plan, source)
        validate_plan_evidence(plan, source)
        self.plan = deepcopy(plan)
        self.profile = deepcopy(source["calibration_profile"])
        self.profile_digest = _digest(self.profile)
        self.limits = self.profile["divergence_thresholds"]
        for key in ("joint_rad", "gripper_mm", "timing_s"):
            if key not in self.limits:
                raise ValueError("profile requires joint_rad, gripper_mm and timing_s tolerances")
        self.feedback_timeout_s = _finite(feedback_timeout_s, "feedback_timeout_s")
        if self.feedback_timeout_s <= 0 or not all(callable(f) for f in (sink, gate, stop)):
            raise ValueError("positive feedback timeout and sink/gate/stop callbacks required")
        control = plan.get("control", {})
        hz, nominal = control.get("physics_hz"), control.get("substeps")
        steps = plan.get("native_substeps")
        self.ticks = self.plan["ticks"]
        from gripper_distance_plan import validate_distance_control, holding_ticks
        self.holding_ticks = validate_distance_control(self.plan, source)
        self.unloading_ticks = (holding_ticks(self.plan, {'open'})
            if self.plan.get('gripper_release',{}).get('schema_version')=='commanded_gap_release.v2'
            else [False]*len(self.ticks))
        if (type(hz) is not int or hz <= 0 or type(nominal) is not int or nominal <= 0
                or not isinstance(steps, list) or len(steps) != len(self.ticks)
                or any(type(n) is not int or not 1 <= n <= nominal for n in steps)
                or any(n != nominal for n in steps[:-1])):
            raise ValueError("native physics cadence and only-final partial tick are required")
        elapsed_steps = 0
        for i, (tick, count) in enumerate(zip(self.ticks, steps)):
            if tick["tick"] != i or not math.isclose(tick["time_s"], elapsed_steps / hz, rel_tol=0, abs_tol=1e-9):
                raise ValueError("frozen ticks must start at zero and match native physics cadence")
            elapsed_steps += count
        self.duration_s = elapsed_steps / hz
        if not math.isclose(_finite(plan.get("duration_s"), "duration_s"), self.duration_s, rel_tol=0, abs_tol=1e-9):
            raise ValueError("duration_s must preserve the final native partial tick")
        self.times = [tick["time_s"] for tick in self.ticks]
        self.sink, self.gate, self.stop = sink, gate, stop
        self.status, self.reason = "new", None
        self.start_s = self.previous_s = self.last_sample_s = None
        self.last_tick = -1
        self.command_index = -1
        self.scheduling_delay_s = 0.
        self.schedule_slips = 0
        self.send_count = 0
        self.first_sends = []
        self.errors = None
        self.timeline_elapsed = self.final_feedback_qualified = False
        self.stop_invoked, self.stop_error = False, None
        self.tracking_exceeded_samples=0
        self.maximum_errors={'joint_rad':0.,'gripper_mm':0.}
        self.runtime_timeline=None

    def configure_runtime(self, timeline):
        """Bind the privately compiled preview/initialization result, without rescanning it."""
        if (self.status!='new' or self.runtime_timeline is not None
                or self.execution_policy!='observation'
                or timeline.get('schema_version')!='execution_timeline.v1'
                or timeline.get('parent_timeline_digest')!=self.plan['timeline_digest']
                or timeline.get('control_hz')!=30 or not timeline.get('commands')
                or not timeline.get('timeline_digest')):
            raise ValueError('one private runtime derived from this preview required')
        self.runtime_timeline=timeline
        self.ticks=timeline['commands']
        self.times=timeline['times']
        self.holding_ticks=timeline['holding_ticks']
        self.unloading_ticks=timeline['unloading_ticks']
        self.duration_s=timeline['duration_s']

    def snapshot(self, *, include_history=True):
        result = {"status": self.status, "reason": self.reason,
                "validation_scope": "geometric_path_only" if self.plan['simulation']['outcome']=='not_run' else "sim/mock",
                "hardware_validated": False, "timeline_digest": self.plan["timeline_digest"],
                "calibration_profile_digest": self.profile_digest, "duration_s": self.duration_s,
                "scheduling_delay_s": self.scheduling_delay_s, "schedule_slips": self.schedule_slips,
                "last_tick": self.last_tick, "send_count": self.send_count,
                "targets_submitted": self.last_tick == len(self.ticks) - 1,
                "timeline_elapsed": self.timeline_elapsed, "final_feedback_qualified": self.final_feedback_qualified,
                "tracking_error": deepcopy(self.errors), "stop_invoked": self.stop_invoked,
                "tracking_policy":self.execution_policy,"tracking_exceeded_samples":self.tracking_exceeded_samples,
                "maximum_tracking_error":dict(self.maximum_errors),
                "execution_timeline_digest":(self.runtime_timeline or {}).get('timeline_digest'),
                "execution_hz":(self.runtime_timeline or {}).get('control_hz'),
                "stop_error": self.stop_error,
                "completion_scope": "command timeline and feedback thresholds; task success and hardware acceptance unverified"}
        if include_history:result['tick_first_send_monotonic_s']=list(self.first_sends)
        return result

    def _halt(self, reason, status="failed"):
        if self.status in ("completed", "failed", "cancelled"):
            return self.snapshot()
        self.status, self.reason = status, str(reason)
        self.stop_invoked = True
        try:
            self.stop(self.reason)
        except Exception as error:
            self.stop_error = str(error)
        return self.snapshot()

    def cancel(self, reason="cancelled by owner"):
        return self._halt(reason, "cancelled")

    def start(self, now, feedback):
        if self.status != "new":
            raise ValueError("executor is single-use; a stopped or consumed timeline cannot restart")
        try:
            self.start_s = _finite(now, "start monotonic time")
            if self.start_s < 0:
                raise ValueError("monotonic time must be nonnegative")
            self.status = "running"
            return self.tick(now, feedback)
        except Exception as error:
            return self._halt(error)

    def _feedback(self, now, feedback):
        if not isinstance(feedback, dict) or set(feedback) != {"sample_monotonic_s", "state"}:
            raise ValueError("feedback requires sample_monotonic_s and physical14 state")
        sample = _finite(feedback["sample_monotonic_s"], "feedback sample time")
        if not 0 <= now - sample <= self.feedback_timeout_s:
            raise ValueError("feedback is stale or from the future")
        if self.last_sample_s is not None and sample < self.last_sample_s:
            raise ValueError("feedback time regressed")
        measured = validate_physical_feedback(feedback["state"], self.profile,
            enforce_range=self.execution_policy=='strict')
        if not self.first_sends or sample < self.first_sends[0]:
            # No command exists to track yet. Validate feedback health/freshness,
            # then submit the first frozen target unchanged; normal tracking
            # protection applies from the next owner tick onward.
            self.errors = None
            self.last_sample_s = sample
            return sample
        # Compare against the command effective at the sample time, not a target about to be sent.
        reference_index = max(0, bisect_right(self.first_sends, sample) - 1)
        target = self.ticks[reference_index]["target"]
        jaw_errors = {key: abs(measured[key]-target[key]) for key in GRIPPER_KEYS}
        if self.execution_policy=='strict' and (self.holding_ticks[reference_index] or self.unloading_ticks[reference_index]):
            # This is an allowed position residual, not evidence of contact or force.
            jaw_errors['left_gripper'] = max(0.0, target['left_gripper']-measured['left_gripper'])
        self.errors = {
            "joint_rad": max(abs(a-b) for key in ARM_KEYS for a,b in zip(measured[key], target[key])),
            "gripper_mm": max(jaw_errors.values())}
        exceeded={key:dict(error=value,limit=self.limits[key]) for key,value in self.errors.items() if value>self.limits[key]}
        for key,value in self.errors.items():self.maximum_errors[key]=max(self.maximum_errors[key],value)
        if exceeded:self.tracking_exceeded_samples+=1
        if exceeded and self.execution_policy=='strict':
            raise ValueError(f"feedback tracking error exceeds the frozen profile tolerance at tick {reference_index}: {exceeded}")
        self.last_sample_s = sample
        if self.timeline_elapsed and sample+1e-12>=self.start_s+self.duration_s+self.scheduling_delay_s:
            self.final_feedback_qualified=True
        return sample

    def tick(self, now, feedback):
        if self.status == "new":
            raise ValueError("start the executor before ticking")
        if self.status in ("completed", "failed", "cancelled"):
            return self.snapshot()
        try:
            now = _finite(now, "tick monotonic time")
            if now < self.start_s or self.previous_s is not None and now <= self.previous_s:
                raise ValueError("control time did not advance monotonically")
            if self.gate() is not True:
                raise ValueError("owner command gate closed")
            sample = self._feedback(now, feedback)
            elapsed = now - self.start_s - self.scheduling_delay_s
            index = min(len(self.ticks)-1, bisect_right(self.times, elapsed + 1e-12)-1)
            if index > self.last_tick + 1:
                index = self.last_tick + 1
                delay = elapsed - self.times[index]
                if delay > self.limits['timing_s'] + 1e-12 and self.execution_policy=='strict':
                    raise ValueError('target submission missed its profile timing tolerance')
                # Send the next unchanged target once, then preserve the remaining
                # intervals. Never skip a waypoint or accelerate to catch up.
                self.scheduling_delay_s += delay
                self.schedule_slips += 1
                elapsed -= delay
            if (index > self.last_tick and elapsed - self.times[index] > self.limits["timing_s"] + 1e-12
                    and self.execution_policy=='strict'):
                raise ValueError("target submission missed its profile timing tolerance")
            self.timeline_elapsed = elapsed + 1e-12 >= self.duration_s
            if self.timeline_elapsed:
                if self.last_tick != len(self.ticks)-1:
                    raise ValueError("timeline ended before every target was submitted")
                if elapsed - self.duration_s > self.limits["timing_s"] + 1e-12 and self.execution_policy=='strict':
                    raise ValueError("final feedback deadline exceeded")
                if sample + 1e-12 >= self.start_s + self.duration_s + self.scheduling_delay_s:
                    self.final_feedback_qualified = True
                    self.status = "completed"
                    self.previous_s = now
                    return self.snapshot()
                if self.execution_policy=='observation':
                    self.status='completed'
                    self.previous_s=now
                    return self.snapshot()
                self.status = "awaiting_final_feedback"
            if self.gate() is not True:
                raise ValueError("owner command gate closed before target submission")
            self.command_index = index
            sent_at=self.sink(deepcopy(self.ticks[index]["target"]))
            if index > self.last_tick:
                self.first_sends.append(now if sent_at is None else _finite(sent_at,'target submitted time'))
                self.last_tick = index
            self.send_count += 1
            self.previous_s = now
            return self.snapshot(include_history=False)
        except Exception as error:
            return self._halt(error)
