"""Shared, bounded q/dq execution for points, planned paths and policy chunks.

Preparation owns timing/path changes. This module never performs IK, collision
checks, model calls or disk IO. The existing business runners keep their guards.
"""
from dataclasses import dataclass
import hashlib
import json
import math
import time

import numpy as np

from .motion_reference import JOINTS, MAX_PACKET_ROWS, ReferenceIntegral, STUDY_BIAS_PROFILE, _frozen, SCHEMA as REFERENCE_SCHEMA

SCHEMA = 'workbench.target_window.v1'
PROFILES = ('normal_X5', 'x5_shared_precision_v1', 'x5_control50_pd15_v1','x5_control50_pd15_fast_i_v1','x5_control50_feedback50_v1','x5_control50_pd30_feedback50_v1',STUDY_BIAS_PROFILE,'x5_control50_damping_v1')
FEEDBACK50_PROFILES = ('x5_control50_feedback50_v1','x5_control50_pd30_feedback50_v1',STUDY_BIAS_PROFILE,'x5_control50_damping_v1')


def native_period_s(metadata, profile):
    grid = metadata.get('native_grid')
    velocity_source = metadata.get('velocity_source')
    if velocity_source is not None and (velocity_source != 'sdk_position_only'
            or grid != 'reference_origin_30Hz' or profile != 'x5_control50_damping_v1'):
        raise ValueError('SDK position-only velocity requires the explicit 30 Hz research reference')
    if grid not in (None, 'reference_origin_30Hz', 'reference_origin_100Hz', 'reference_origin_200Hz'):
        raise ValueError('native_grid must explicitly select reference_origin_30Hz, reference_origin_100Hz or reference_origin_200Hz')
    if grid in ('reference_origin_30Hz','reference_origin_200Hz'):
        if profile not in FEEDBACK50_PROFILES:
            raise ValueError('non-default reference rate requires the bounded 0.1 s research window')
        return 1./30. if grid=='reference_origin_30Hz' else .005
    return .01


class ReferenceRevokedError(ValueError):
    """A revoked reference cannot submit; Owner decides requested cancellation."""


class ReferenceFeedbackStaleError(ValueError):
    def __init__(self, age_s, maximum_age_s, *, phase):
        self.age_s,self.maximum_age_s,self.phase=age_s,maximum_age_s,phase
        super().__init__('shared reference requires fresh feedback within'+str(round(maximum_age_s*1000))+
            f'ms; observed_age_ms={age_s*1000:.3f}; phase={phase}')


def gains(profile, side):
    if profile not in PROFILES or side not in ('left', 'right'):
        raise ValueError('known device motion profile and arm required')
    kp, kd = [80., 70., 70., 30., 30., 20.], [2., 2., 2., 1., 1., .7]
    if profile != 'normal_X5' and side == 'right':
        kp[3], kd[3] = 80., 1.6
    if profile in ('x5_control50_pd15_v1','x5_control50_pd15_fast_i_v1','x5_control50_feedback50_v1') and side == 'left':
        kp,kd=[1.5*x for x in kp],[1.5*x for x in kd]
    if profile in ('x5_control50_pd30_feedback50_v1',STUDY_BIAS_PROFILE,'x5_control50_damping_v1') and side == 'left':
        kp,kd=[3.*x for x in kp],[1.5*math.sqrt(2.)*x for x in kd]
    if profile == 'x5_control50_damping_v1' and side == 'left':
        kd=[5.,5.,5.,3.,3.,2.1]
    return kp, kd


def _identity(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False,
                                    separators=(',', ':')).encode()).hexdigest()


def _vectors(q, limits):
    q, limits = np.asarray(q, dtype=float), np.asarray(limits, dtype=float)
    if (q.ndim != 2 or q.shape[1] != 14 or limits.shape != (14, 2)
            or not np.isfinite(q).all() or not np.isfinite(limits).all()
            or np.any(limits[:, 0] > limits[:, 1])
            or np.any(q < limits[:, 0]) or np.any(q > limits[:, 1])):
        raise ValueError('finite, in-limit SDK14 reference required')
    return q, limits


@dataclass(frozen=True)
class PointReference:
    initial: np.ndarray
    target: np.ndarray
    duration_s: float
    metadata: dict

    def sample(self, times):
        t = np.asarray(times, dtype=float)
        if t.ndim != 1 or not 0 < len(t) <= MAX_PACKET_ROWS or not np.isfinite(t).all():
            raise ValueError('bounded finite reference times required')
        u = np.clip(t / self.duration_s, 0., 1.)[:, None]
        delta = self.target - self.initial
        s = np.clip(u**3 * (10. + u * (-15. + 6. * u)), 0., 1.)
        q = np.clip(self.initial + s * delta, np.minimum(self.initial, self.target),
                    np.maximum(self.initial, self.target))
        dq = 30. * u**2 * (1. - u)**2 * delta / self.duration_s
        ddq = 60. * u * (1. - u) * (1. - 2. * u) * delta / self.duration_s**2
        return q, dq, ddq


def prepare_point(plan, initial, target):
    """The already reviewed quintic path, including a fresh bounded start bind."""
    q, _ = _vectors([initial, target], plan.joint_limits)
    from arx_env.joint_head_real_motion import _joint_duration,jaw_peak_scale
    limits=plan.joint_velocity_limits or (plan.max_velocity,)*12
    duration=max(plan.duration_s,_joint_duration(q[1,JOINTS]-q[0,JOINTS],limits))
    duration=max(duration,*(1.875*abs(q[1,slot]-q[0,slot])*jaw_peak_scale(side)/plan.jaw_speed_mm_s
        for side,slot in (('left',6),('right',13))))
    duration=float(duration)
    velocity = 1.875 * np.abs(q[1, JOINTS] - q[0, JOINTS]) / duration
    acceleration = (10. / math.sqrt(3.)) * np.abs(q[1, JOINTS] - q[0, JOINTS]) / duration**2
    meta = dict(schema=REFERENCE_SCHEMA,kind='point', source_sha256=plan.sha256, duration_s=duration,
                source_duration_s=plan.duration_s,timing_policy='retime_allowed',retime_reason='fresh_start_binding',
                clock='host_monotonic_relative_seconds',joint_units='rad',jaw_units='SDK_m',
                max_velocity_rad_s=float(np.max(velocity)), max_acceleration_rad_s2=float(np.max(acceleration)),
                path_policy='reviewed_quintic', initial_sdk14=q[0].tolist(), target_sdk14=q[1].tolist())
    meta['reference_id'] = _identity(meta)
    return PointReference(_frozen(q[0]), _frozen(q[1]), duration, meta)


def prepare_measured_hold(feedback, limits, *, now, brake_end_s, stationary_window,
                          held_jaws_sdk, source_sha256):
    """Bind a fixed measured joint anchor once after normal braking.

    The Owner supplies jaw commands from its current-epoch output evidence,
    never measured jaw gaps. The existing stationary detector remains the
    evidence policy; no new physical threshold or motor ACK is inferred here.
    """
    sample=feedback.get('sample_time_s');check=stationary_window.last_diagnostic or{}
    jaws=np.asarray(held_jaws_sdk,dtype=float)
    if (any(type(t)not in(int,float)or not math.isfinite(t)for t in(now,brake_end_s,sample))
            or not brake_end_s<=sample<=now or now-sample>.1
            or check.get('stationary')is not True or check.get('sample_time_s')!=sample
            or check.get('window_start_s',-math.inf)<brake_end_s
            or check.get('raw_qpos')!=list(feedback.get('qpos',()))
            or check.get('raw_qvel')!=list(feedback.get('qvel',()))
            or jaws.shape!=(2,)or not np.isfinite(jaws).all()):
        raise ValueError('fresh post-brake stationary evidence and explicit held jaw commands required')
    target=np.asarray(feedback['qpos'],dtype=float).copy()
    if target.shape!=(14,):raise ValueError('14 measured axes required')
    target[[6,13]]=jaws
    q,_=_vectors([target],limits)
    meta=dict(schema=REFERENCE_SCHEMA,kind='normal_hold',source_sha256=source_sha256,
        clock='host_monotonic_relative_seconds',joint_units='rad',jaw_units='SDK_m',duration_s=1.,
        brake_end_source_s=brake_end_s,anchor_sample_source_s=sample,anchor_sdk14=q[0].tolist(),
        anchor_policy='fixed_measured_joints_once; retained_command_jaws',
        stationary_scope='existing_0.3s_position_span_not_motor_ACK',max_velocity_rad_s=0.,max_acceleration_rad_s2=0.)
    meta['reference_id']=_identity(meta)
    return PointReference(_frozen(q[0]),_frozen(q[0]),1.,meta)


@dataclass(frozen=True)
class SampledReference:
    times: np.ndarray
    targets: np.ndarray
    velocities: np.ndarray
    metadata: dict
    event_times: np.ndarray

    @property
    def duration_s(self):
        return float(self.times[-1])

    def sample(self, times):
        t = np.asarray(times, dtype=float)
        if t.ndim != 1 or not 0 < len(t) <= MAX_PACKET_ROWS or not np.isfinite(t).all():
            raise ValueError('bounded finite reference times required')
        i = np.clip(np.searchsorted(self.times, t, side='right') - 1, 0, len(self.times) - 2)
        u = np.clip((t - self.times[i]) / np.diff(self.times)[i], 0., 1.)[:, None]
        q = self.targets[i] + u * (self.targets[i+1] - self.targets[i])
        dq = self.velocities[i] + u * (self.velocities[i+1]-self.velocities[i])
        dq[(t < 0.) | (t >= self.duration_s)] = 0.
        # Legacy frozen targets have discrete derivatives, not a C2 path proof.
        ddq = (self.velocities[i+1]-self.velocities[i])/np.diff(self.times)[i,None]
        ddq[(t<0.)|(t>=self.duration_s)]=0.
        return q, dq, ddq


def prepare_samples(times, targets, limits, *, source_sha256, timing_policy='strict_duration',
                    velocity_limit=2., acceleration_limit=5., kind='planned_path',
                    source_semantics='arrivals',duration_s=None):
    """Preserve the piecewise-linear spatial path; time changes are explicit.

    Frozen paths cannot silently become a fitted spline. Their existing path
    qualification remains a separate requirement of the business adapter.
    """
    q, _ = _vectors(targets, limits)
    t = np.asarray(times, dtype=float)
    if (t.shape != (len(q),) or not 2 <= len(t) <= 65536 or not np.isfinite(t).all()
            or t[0] != 0. or np.any(np.diff(t) <= 0.)
            or timing_policy not in ('strict_duration', 'retime_allowed')
            or source_semantics not in ('arrivals','command_intervals')
            or any(type(x) not in (int, float) or not math.isfinite(x) or x <= 0.
                   for x in (velocity_limit, acceleration_limit))):
        raise ValueError('bounded ordered reference with explicit timing policy required')
    if source_semantics=='command_intervals':
        if type(duration_s)not in(int,float)or not math.isfinite(duration_s)or duration_s<=t[-1]:
            raise ValueError('command intervals require the complete final partial duration')
        # A frozen row is a goal activated at its due time, not permission to
        # start the next grip/release during the preceding interval.
        t=np.r_[0.,t[1:],duration_s];q=np.vstack([q[0],q])
    dt = np.diff(t)
    interval_v = np.diff(q, axis=0) / dt[:, None]
    v=np.vstack([np.zeros(14),interval_v]);v[-1]=0.
    a=np.diff(v,axis=0)/dt[:,None]
    peak_v, peak_a = float(np.max(np.abs(interval_v[:, JOINTS]))), float(np.max(np.abs(a[:, JOINTS])))
    scale = max(1., peak_v / velocity_limit, math.sqrt(peak_a / acceleration_limit))
    if scale > 1. + 1e-9 and timing_policy == 'strict_duration':
        raise ValueError('frozen timing exceeds motion bounds; explicit prepared derivation required')
    t, v = t * scale, v / scale
    meta = dict(schema=REFERENCE_SCHEMA,kind=kind, source_sha256=source_sha256, source_duration_s=float(times[-1]),
                clock='host_monotonic_relative_seconds',joint_units='rad',jaw_units='SDK_m',
                duration_s=float(t[-1]), timing_policy=timing_policy, time_scale=scale,
                path_policy='unchanged_piecewise_linear', acceleration_scope='discrete_reference_velocity_field',
                velocity_scope='causal_finite_differences_not_analytic_PWL_corners',
                source_knot_semantics=source_semantics,
                transition_activation='not_before_source_command_due'if source_semantics=='command_intervals'else'at_reference_knots',
                max_velocity_rad_s=peak_v / scale, max_acceleration_rad_s2=peak_a / scale**2)
    identity = hashlib.sha256(json.dumps(meta, sort_keys=True).encode())
    identity.update(t.tobytes()); identity.update(q.tobytes()); meta['reference_id'] = identity.hexdigest()
    changes=np.unique(np.r_[np.flatnonzero(np.any(np.abs(np.diff(interval_v,axis=0))>1e-12,axis=1))+1,
                            np.flatnonzero(np.any(np.abs(np.diff(a,axis=0))>1e-12,axis=1))+1])
    events=np.r_[t[0],t[changes],t[-1]]
    return SampledReference(_frozen(t), _frozen(q), _frozen(v), meta,_frozen(events))


class TargetInterpreter:
    """One execution implementation, independent of the source business."""
    def __init__(self, backend, reference, *, active_arm, start_s, epoch, current_epoch,
                 gate, clock, profile='normal_X5', integral=False, feedforward=None,integral_gains=None,guard=None,record_event=None):
        if record_event is not None and not callable(record_event):raise ValueError('optional bounded recording callback required')
        self.record_event=record_event
        if active_arm not in ('left', 'right', 'both') or profile not in PROFILES:
            raise ValueError('known active arm and motion profile required')
        identity=reference.metadata.get('reference_id')
        if (reference.metadata.get('schema')!=REFERENCE_SCHEMA or not isinstance(identity,str)
                or len(identity)!=64 or any(c not in '0123456789abcdef'for c in identity)):
            raise ValueError('prepared common reference schema and identity required')
        if not callable(getattr(backend, 'queue_reference', None)):
            raise ValueError('shared target interpreter requires native-window adapter; no point-send fallback')
        if type(integral) is not bool or feedforward is not None and active_arm == 'right':
            raise ValueError('explicit feedback policy and left-only qualified frozen feedforward required')
        self.backend, self.reference, self.active_arm = backend, reference, active_arm
        self.start_s, self.epoch, self.current_epoch = start_s, epoch, current_epoch
        self.gate, self.clock, self.profile = gate, clock, profile
        self.position_guard=guard
        if guard is None:
            from .execution_settings import Guard,control_parameters,defaults,policy_hash,FEEDBACK_TIMEOUT_S,INTEGRAL_FEEDBACK_TIMEOUT_S,REFERENCE_STARVATION_MARGIN_S
            parameters=control_parameters(defaults('inference'),business='inference')
            guard=Guard.bind(dict(revision=None,policy_hash=policy_hash(parameters),parameters=parameters),
                dict(feedback_timeout_s=FEEDBACK_TIMEOUT_S,integral_feedback_timeout_s=INTEGRAL_FEEDBACK_TIMEOUT_S,starvation_margin_s=REFERENCE_STARVATION_MARGIN_S))
        self.guard=guard
        self.native_period_s = native_period_s(reference.metadata, profile)
        if integral_gains is not None:
            from .execution_settings import validate_integral_gains
            if profile!='x5_control50_damping_v1':raise ValueError('explicit integral gains require the research profile')
            integral_gains=validate_integral_gains(integral_gains)
        self.integrals = {side: ReferenceIntegral(rapid=profile in ('x5_control50_pd15_fast_i_v1','x5_control50_feedback50_v1','x5_control50_pd30_feedback50_v1',STUDY_BIAS_PROFILE,'x5_control50_damping_v1')and side=='left',
                          rapid_scale=8 if profile in ('x5_control50_feedback50_v1','x5_control50_pd30_feedback50_v1',STUDY_BIAS_PROFILE,'x5_control50_damping_v1')and side=='left'else 4,profile=profile,side=side,
                          gains_sdk6=None if integral_gains is None else integral_gains[side]) for side in
                          (('left', 'right') if active_arm == 'both' else (active_arm,))} if integral else {}
        units=getattr(backend,'torque_unit_contract','legacy_sdk_v1')
        if units!='legacy_sdk_v1':
            from .sdk_quiesce import EC_STATIC_UNITS,EC_J2_GRAVITY_SCALES,DM_EQUIVALENT_UNITS
            from .sdk_process import ReferenceWindowRejected
            if units in (EC_STATIC_UNITS,*EC_J2_GRAVITY_SCALES):
                if 'left' in self.integrals and np.any(self.integrals['left'].gains[1:3]!=0):
                    raise ReferenceWindowRejected('EC static unit qualification requires left J2/J3 Ki0')
            elif units==DM_EQUIVALENT_UNITS:
                if any(np.any(value.gains[3:]!=0) for value in self.integrals.values()):
                    raise ReferenceWindowRejected('DM unit qualification requires J4/J5/J6 Ki0 on each active arm')
            else:
                raise ReferenceWindowRejected('unknown torque unit contract; qualification required')
            if feedforward is not None and getattr(feedforward,'profile',{}).get('provenance',{}).get('torque_unit_contract')!=units:
                raise ReferenceWindowRejected('feedforward torque unit contract differs; requalification required')
        self.feedforward = feedforward
        if reference.metadata.get('velocity_source') == 'sdk_position_only' and (
                feedforward is not None or any(np.any(x.gains != 0) for x in self.integrals.values())):
            raise ValueError('SDK position-only comparison requires zero integral and no added feedforward')
        self.next_upload = self.queued_until = None
        self.last_packet = self.last_receipt = None
        self.submissions = 0
        self.last_timing = None
        self.cancelled = False
        self.compensation_parent=None
        self.preserved_prefix=None

    def inherit_hold(self,previous,now):
        """Carry the actual bounded output; never carry a prior profile or path."""
        if (previous is None or previous.cancelled or previous.profile!=self.profile
                or previous.epoch!=self.epoch or previous.active_arm!=self.active_arm
                or self.feedforward is not None
                or previous.feedforward is not None and not self.integrals
                or previous.last_packet is None):return
        times=previous.last_packet['host_times_s']
        bias=np.asarray(previous.last_packet['torque_bias_sdk14'])
        first=now+(.03 if self.active_arm=='both'else .02)
        for side,integral in self.integrals.items():
            if side not in previous.integrals:continue
            indices=range(6)if side=='left'else range(7,13)
            integral.bias=np.asarray([np.interp(first,times,bias[:,j])for j in indices])
        self.compensation_parent=previous.reference.metadata['reference_id']
        previous.revoke()

    def revoke(self):
        self.cancelled = True
        self.integrals.clear(); self.feedforward = None

    def snapshot(self):
        return dict(schema=SCHEMA,reference_id=self.reference.metadata['reference_id'],
                    source_kind=self.reference.metadata.get('kind','prepared_trajectory'),
                    active_arm=self.active_arm,controller_profile=self.profile,
                    reference_node_hz=round(1./self.native_period_s),
                    velocity_source=self.reference.metadata.get('velocity_source','prepared_dq'),
                    integral_gains_sdk={side:value.gains.tolist()for side,value in self.integrals.items()},
                    integral_enabled=bool(self.integrals),cancelled=self.cancelled,
                    source_time_origin_s=self.start_s,duration_s=self.reference.duration_s,
                    compensation_parent_reference_id=self.compensation_parent,
                    timing_s=self.last_timing,
                    native_window_submissions=self.submissions,queued_until_source_s=self.queued_until)

    def tick(self, now, feedback, *, force=False):
        # Match the adapter's bounded phase trace, including refused windows.
        started,cpu=time.monotonic(),time.thread_time_ns()
        timing=dict(schema='target-translation-timing.v1',phase='entry_validation',status='failed',
            started_s=started,reference_id=self.reference.metadata['reference_id'],epoch=self.epoch,stages=[])
        last,last_cpu=started,cpu
        def mark(phase=None,**values):
            nonlocal last,last_cpu
            wall,current=time.monotonic(),time.thread_time_ns()
            timing['stages'].append(dict(phase=timing['phase'],started_s=last,wall_s=wall-last,cpu_s=(current-last_cpu)/1e9))
            last,last_cpu=wall,current
            if phase is not None:timing['phase']=phase
            timing.update(values)
        result=None
        try:
            result=self._tick(now,feedback,force=force,mark=mark)
            timing['status']='admitted'if result is not None else'skipped'
            return result
        except BaseException as error:
            timing['error_type']=type(error).__name__
            raise
        finally:
            mark()
            timing.update(returned_s=last,wall_s=last-started,cpu_s=(last_cpu-cpu)/1e9)
            if result is not None:
                timing.update(result['timing_s'])
                result['timing_s']=timing
            # An idle cadence tick must not overwrite the last admission/failure.
            if timing['status']!='skipped':self.last_timing=timing

    def _tick(self, now, feedback, *, force, mark):
        if self.cancelled or self.current_epoch() != self.epoch:
            raise ReferenceRevokedError('target reference epoch revoked')
        # Upstream feedback/evidence work may have waited after sampling `now`.
        # Build against the actual clock, retaining the true feedback timestamp
        # and the original freshness/starvation budgets.
        now=max(now,self.clock())
        sample = feedback.get('sample_time_s')
        if type(sample) not in (int, float) or not math.isfinite(sample) or now-sample<0.:
            raise ValueError('shared reference requires a finite nonfuture feedback timestamp')
        decision=self.guard.evaluate(dict(rule_id='G28',signal_id=self.reference.metadata['reference_id'],
            observed=now-sample,source_time=sample,checked_time=now,epoch=self.epoch),
            'feedback_integral'if self.integrals else'feedback')
        decisions=[decision];mark('feedback_guard',guard_decisions=decisions)
        if decision['verdict']=='reject':
            raise ReferenceFeedbackStaleError(now-sample,decision['limit'],
                phase='terminal_hold'if now-self.start_s>=self.reference.duration_s else'motion')
        if self.queued_until is not None:
            decision=self.guard.evaluate(dict(rule_id='G29',signal_id=self.reference.metadata['reference_id'],
                observed=dict(current_time=now,queued_until=self.queued_until),source_time=sample,checked_time=now,epoch=self.epoch),'submission')
            decisions.append(decision);mark('queue_guard')
            if decision['verdict']=='reject':raise ValueError('shared reference starved; stop before stale nonzero tail')
        if not force and self.next_upload is not None and now < self.next_upload:
            return None
        mark('command_gate')
        self.gate()
        mark('window_grid')
        feedback50=self.profile in FEEDBACK50_PROFILES
        period=self.native_period_s
        rows=round((.1 if feedback50 else .2)/period)+1
        host = now + (.03 if self.active_arm=='both'else .02) + np.arange(rows) * period
        if self.reference.metadata.get('native_grid') is not None:
            # Identical overlap knots across uploads: scheduling jitter cannot
            # move the linear interpolator's corners on every replacement.
            first=math.ceil((host[0]-self.start_s)/period-1e-9)
            host=self.start_s+(first+np.arange(len(host)))*period
        if not self.submissions and now<self.start_s<host[0]:
            # Native append retains old knots before the first new sample.
            # Anchor the curve origin so a later HOLD knot cannot delay motion.
            host=np.r_[self.start_s,host]
        knots=getattr(self.reference,'event_times',getattr(self.reference,'breaks',None))
        if knots is None:knots=np.asarray([self.reference.duration_s])
        lo,hi=np.searchsorted(knots,[host[0]-self.start_s,host[-1]-self.start_s],side='left')
        for event in knots[lo:hi]:
            stamp=self.start_s+float(event)
            # Preserve declared boundaries rather than interpolating a gripper
            # event a fraction of one sampling period before its source stage.
            host=np.r_[host[np.abs(host-stamp)>1e-6],stamp]
        host=np.sort(host)  # Replaced event knots must precede first/last-time checks.
        prefix=self.preserved_prefix
        if prefix is not None and host[0]>prefix['host_times_s'][-1]:
            if not self.submissions:raise ValueError('committed prefix expired before brake installation')
            prefix=self.preserved_prefix=None
        if prefix is not None:
            for stamp in prefix['host_times_s']:
                if host[0]<=stamp<=host[-1]:host=np.r_[host[np.abs(host-stamp)>1e-6],stamp]
        host=np.sort(host)
        if (not self.submissions and self.reference.metadata.get('planning_policy',{}).get('boundary_scope')=='complete_prepared_segment'
                and host[0]>self.start_s+self.reference.breaks[1]):
            raise ValueError('first target knot already passed; complete segment cannot be admitted')
        if (not self.submissions and self.reference.metadata.get('kind')=='raw_replay'
                and host[0]>self.start_s+self.reference.breaks[2]):
            raise ValueError('first Raw target knot already passed; replay cannot skip its source prefix')
        if len(host)>MAX_PACKET_ROWS:raise ValueError('reference events exceed bounded native window; reprepare')
        mark('reference_sample',first_point_source_s=float(host[0]),feedback_sample_source_s=sample)
        sample_started=time.perf_counter()
        q, dq, _ = self.reference.sample(host - self.start_s)
        if prefix is not None:
            retained=host<=prefix['host_times_s'][-1]
            for output,key in((q,'q_sdk14'),(dq,'dq_sdk14')):
                values=np.asarray(prefix[key])
                for j in range(14):output[retained,j]=np.interp(host[retained],prefix['host_times_s'],values[:,j])
        if self.position_guard is not None:
            bounds=self.position_guard.evaluate(dict(rule_id='G18',signal_id=self.reference.metadata['reference_id'],
                observed=dict(minimum=q[:,JOINTS].min(axis=0).tolist(),maximum=q[:,JOINTS].max(axis=0).tolist(),active_arm=self.active_arm),
                source_time=sample,checked_time=now,epoch=self.epoch),'submission')
            decisions.append(bounds);mark('position_guard')
            if bounds['verdict']=='reject':raise ValueError('G18: native command exceeds the bound joint capability')
            from arx_env.joint_head_jaw_mapping import command_gap_mm
            jaws=self.position_guard.evaluate(dict(rule_id='G17',signal_id=self.reference.metadata['reference_id'],
                observed=dict(minimum=[command_gap_mm(side,float(q[:,slot].min()))for side,slot in (('left',6),('right',13))],
                    maximum=[command_gap_mm(side,float(q[:,slot].max()))for side,slot in (('left',6),('right',13))],active_arm=self.active_arm),
                source_time=sample,checked_time=now,epoch=self.epoch),'submission')
            decisions.append(jaws)
            if jaws['verdict']=='reject':raise ValueError('G17: native jaw command exceeds the bound SDK capability')
        mark('feedback_target_sample')
        target, _, _ = self.reference.sample([sample - self.start_s])
        mark('bias_allocation')
        bias_started=time.perf_counter();integral_times={}
        bias = np.zeros_like(q)
        for side in ('left','right'):
            integral=self.integrals.get(side)
            if integral is None and not(self.feedforward is not None and side=='left'):continue
            mark(side+('_integral'if integral is not None else'_bias'))
            integral_started=time.perf_counter()
            sl = slice(0, 6) if side == 'left' else slice(7, 13)
            pause=getattr(self.feedforward,'j3_pause',None)if side=='left'else None
            if pause is not None and integral is None:raise ValueError('J3 integral pause requires enabled residual integral')
            values = bias[:,sl] if integral is None else integral.packet(now, sample, target[0, sl] - np.asarray(feedback['qpos'])[sl], host,j3_pause=pause)
            if self.feedforward is not None and side=='left':
                mark(side+'_feedforward')
                values = self.feedforward.packet(host, values, integrator=integral)
            bias[:, sl] = values
            integral_times[side]=time.perf_counter()-integral_started
        mark('packet_build')
        packet_started=time.perf_counter()
        # Jaw velocities retain the established calibrated position-only contract.
        dq[:, [6, 13]] = 0.
        packet = dict(schema=SCHEMA, reference_id=self.reference.metadata['reference_id'],
                      active_arm=self.active_arm, controller_profile=self.profile,
                      host_times_s=host.tolist(), q_sdk14=q.tolist(), dq_sdk14=dq.tolist(),
                      torque_bias_sdk14=bias.tolist())
        if self.reference.metadata.get('velocity_source') == 'sdk_position_only':
            packet['velocity_source'] = 'sdk_position_only'
        before = self.clock()
        packed=time.perf_counter()
        submitted_wall_ns=time.time_ns()
        mark('native_admission',first_point_lead_at_submit_s=float(host[0])-before)
        receipt = self.backend.queue_reference(packet)
        admitted=time.perf_counter()
        mark('post_admission_gate')
        self.gate()
        if self.cancelled or self.current_epoch() != self.epoch:
            raise ReferenceRevokedError('target reference revoked during native admission')
        self.last_packet, self.last_receipt = packet, receipt
        self.submissions += 1
        timing=dict(reference_sample_s=bias_started-sample_started,
            integral_and_feedforward_s=integral_times,packet_build_s=packed-packet_started,
            native_admission_s=admitted-packed,reference_through_admission_s=admitted-sample_started)
        self.next_upload, self.queued_until = now + (.02 if feedback50 else .1), float(host[-1])
        if self.record_event is not None:self.record_event('native_window',dict(reference_id=packet['reference_id'],
            first_source_s=float(host[0]),last_source_s=float(host[-1]),guard_decisions=decisions,admission=receipt))
        return dict(packet=packet, admission=receipt, submitted_monotonic_s=before,
                    integral_state={side:dict(bias_sdk6=value.bias.tolist(),rate_sdk6_per_s=value.rate.tolist(),
                        effective_next_rate_sdk6_per_s=value.effective_rate.tolist(),first_knot_s=value.time)
                        for side,value in self.integrals.items()},
                    timing_s=timing,
                    returned_monotonic_s=self.clock(), submitted_wall_ns=submitted_wall_ns,
                    command_semantics='native_future_window_admission')
