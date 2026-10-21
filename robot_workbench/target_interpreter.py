"""Shared, bounded q/dq execution for points, planned paths and policy chunks.

Preparation owns timing/path changes. This module never performs IK, collision
checks, model calls or disk IO. The existing business runners keep their guards.
"""
from dataclasses import dataclass
import hashlib
import json
import math

import numpy as np

from .motion_reference import JOINTS, MAX_PACKET_ROWS, ReferenceIntegral, _frozen, SCHEMA as REFERENCE_SCHEMA

SCHEMA = 'workbench.target_window.v1'
PROFILES = ('normal_X5', 'x5_shared_precision_v1')


def gains(profile, side):
    if profile not in PROFILES or side not in ('left', 'right'):
        raise ValueError('known device motion profile and arm required')
    kp, kd = [80., 70., 70., 30., 30., 20.], [2., 2., 2., 1., 1., .7]
    if profile == 'x5_shared_precision_v1' and side == 'right':
        kp[3], kd[3] = 80., 1.6
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
                 gate, clock, profile='normal_X5', integral=False, feedforward=None):
        if active_arm not in ('left', 'right', 'both') or profile not in PROFILES:
            raise ValueError('known active arm and motion profile required')
        identity=reference.metadata.get('reference_id')
        if (reference.metadata.get('schema')!=REFERENCE_SCHEMA or not isinstance(identity,str)
                or len(identity)!=64 or any(c not in '0123456789abcdef'for c in identity)):
            raise ValueError('prepared common reference schema and identity required')
        if not callable(getattr(backend, 'queue_reference', None)):
            raise ValueError('shared target interpreter requires native-window adapter; no point-send fallback')
        if type(integral) is not bool or feedforward is not None and active_arm != 'left':
            raise ValueError('explicit feedback policy and left-only qualified frozen feedforward required')
        self.backend, self.reference, self.active_arm = backend, reference, active_arm
        self.start_s, self.epoch, self.current_epoch = start_s, epoch, current_epoch
        self.gate, self.clock, self.profile = gate, clock, profile
        self.integrals = {side: ReferenceIntegral() for side in
                          (('left', 'right') if active_arm == 'both' else (active_arm,))} if integral else {}
        self.feedforward = feedforward
        self.next_upload = self.queued_until = None
        self.last_packet = self.last_receipt = None
        self.submissions = 0
        self.cancelled = False
        self.compensation_parent=None

    def inherit_hold(self,previous,now):
        """Keep a same-epoch integral continuous; never fold frozen FF into it."""
        if (previous is None or previous.cancelled or previous.profile!=self.profile
                or previous.epoch!=self.epoch or previous.active_arm!=self.active_arm
                or previous.feedforward is not None or self.feedforward is not None
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
                    integral_enabled=bool(self.integrals),cancelled=self.cancelled,
                    source_time_origin_s=self.start_s,duration_s=self.reference.duration_s,
                    compensation_parent_reference_id=self.compensation_parent,
                    native_window_submissions=self.submissions,queued_until_source_s=self.queued_until)

    def tick(self, now, feedback, *, force=False):
        if self.cancelled or self.current_epoch() != self.epoch:
            raise ValueError('target reference epoch revoked')
        sample = feedback.get('sample_time_s')
        maximum_age=.05 if self.integrals else .1
        if type(sample) not in (int, float) or not 0. <= now - sample <= maximum_age:
            raise ValueError('shared reference requires fresh feedback within'+str(round(maximum_age*1000))+'ms')
        if self.queued_until is not None and now > self.queued_until - .015:
            raise ValueError('shared reference starved; stop before stale nonzero tail')
        if not force and self.next_upload is not None and now < self.next_upload:
            return None
        self.gate()
        host = now + (.03 if self.active_arm=='both'else .02) + np.arange(21) * .01
        knots=getattr(self.reference,'event_times',getattr(self.reference,'breaks',None))
        if knots is None:knots=np.asarray([self.reference.duration_s])
        lo,hi=np.searchsorted(knots,[host[0]-self.start_s,host[-1]-self.start_s],side='left')
        for event in knots[lo:hi]:
            stamp=self.start_s+float(event)
            # Preserve declared boundaries rather than interpolating a gripper
            # event a fraction of one sampling period before its source stage.
            host=np.r_[host[np.abs(host-stamp)>1e-6],stamp]
        host=np.sort(host)
        if len(host)>MAX_PACKET_ROWS:raise ValueError('reference events exceed bounded native window; reprepare')
        q, dq, _ = self.reference.sample(host - self.start_s)
        target, _, _ = self.reference.sample([sample - self.start_s])
        bias = np.zeros_like(q)
        for side, integral in self.integrals.items():
            sl = slice(0, 6) if side == 'left' else slice(7, 13)
            values = integral.packet(now, sample, target[0, sl] - np.asarray(feedback['qpos'])[sl], host)
            if self.feedforward is not None:
                values = self.feedforward.packet(host, values)
            bias[:, sl] = values
        # Jaw velocities retain the established calibrated position-only contract.
        dq[:, [6, 13]] = 0.
        packet = dict(schema=SCHEMA, reference_id=self.reference.metadata['reference_id'],
                      active_arm=self.active_arm, controller_profile=self.profile,
                      host_times_s=host.tolist(), q_sdk14=q.tolist(), dq_sdk14=dq.tolist(),
                      torque_bias_sdk14=bias.tolist())
        before = self.clock()
        receipt = self.backend.queue_reference(packet)
        self.gate()
        if self.cancelled or self.current_epoch() != self.epoch:
            raise ValueError('target reference revoked during native admission')
        self.last_packet, self.last_receipt = packet, receipt
        self.submissions += 1
        self.next_upload, self.queued_until = now + .1, float(host[-1])
        return dict(packet=packet, admission=receipt, submitted_monotonic_s=before,
                    returned_monotonic_s=self.clock(), command_semantics='native_future_window_admission')
