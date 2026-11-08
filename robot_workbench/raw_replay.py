"""Bounded Raw positions/times into the existing reference planner and preview."""
from copy import deepcopy
import math
import numpy as np

from .inference_owner import digest,sdk_target
from .motion_reference import prepare_reference


def prepare(value,limits,*,device_id,calibration_snapshot_id,jaw_mapping,velocity_limit):
    fields={'schema','device_id','source_sha256','source_clock_id','source_timestamps_ms','device_targets14',
        'calibration_snapshot_id','jaw_mapping_digest','speed_multiplier','max_execution_duration_s',
        'requested_source_duration_s','maximum_source_gap_s'}
    if (not isinstance(value,dict)or set(value)!=fields or value.get('schema')!='workbench.raw-replay.v1'
            or value['device_id']!=device_id
            or value['calibration_snapshot_id']is not None and(
                not isinstance(value['calibration_snapshot_id'],str)or not 0<len(value['calibration_snapshot_id'])<=256)
            or not isinstance(jaw_mapping,dict)or value['jaw_mapping_digest']!=digest(jaw_mapping)
            or not isinstance(value['source_sha256'],str)or len(value['source_sha256'])!=64
            or any(c not in '0123456789abcdef'for c in value['source_sha256'])
            or not isinstance(value['source_clock_id'],str)or not 0<len(value['source_clock_id'])<=128
            or value['requested_source_duration_s']!=2.):
        raise ValueError('bound Raw device, source clock, digest and current jaw mapping required')
    # Joint replay consumes no camera/workbench transform. This legacy field is
    # nullable provenance; the actual device and jaw mapping above remain bound.
    speed=value['speed_multiplier'];maximum=value['max_execution_duration_s'];gap=value['maximum_source_gap_s']
    if (any(type(x)not in(int,float)or not math.isfinite(x)for x in(speed,maximum,gap,velocity_limit))
            or not 0<speed<=1 or not 0<maximum<=30 or not 0<gap<=.1 or not 0<velocity_limit<=1):
        raise ValueError('explicit bounded replay speed, execution budget and source gap required')
    stamps=np.asarray(value['source_timestamps_ms']);q=np.asarray(value['device_targets14'])
    if stamps.dtype.kind not in 'fiu'or q.dtype.kind not in 'fiu':raise ValueError('numeric Raw samples required')
    stamps=stamps.astype(float);q=q.astype(float)
    if (stamps.ndim!=1 or not 2<=len(stamps)<=513 or q.shape!=(len(stamps),14)
            or not np.isfinite(stamps).all()or not np.isfinite(q).all()or np.any(np.diff(stamps)<=0)):
        raise ValueError('finite original ordered Raw timestamps and14-axis rad/mm positions required')
    times=(stamps-stamps[0])/1000.;span=float(times[-1])
    if not 2.-gap<=span<=2. or np.max(np.diff(times))>gap+1e-9:
        raise ValueError('Raw first2s selection is incomplete or exceeds its source gap budget')
    timing=np.r_[0.,.05+times/speed]
    if timing[-1]>maximum:raise ValueError('requested replay speed exceeds execution duration budget')
    # Preserve the real final sample; do not synthesize a timestamp at exactly2s.
    provenance=deepcopy(value);provenance.pop('device_targets14')
    provenance.update(retained_source_span_s=span,source_steps=len(q),input_units={'joints':'rad','jaws':'device_mm'},
        initialization_hold_s=.05,
        final_sample_policy='original_last_sample; analytic joint zero-v/a endpoint',
        spatial_policy='interpolate every original target; continuous extrema checked; collision qualification separate')
    source=dict(kind='raw_replay',device_targets14=np.vstack([q[0],q]).tolist(),source_times_s=timing.tolist(),duration_s=float(timing[-1]),
        active_arms=['left','right'],raw_replay=provenance)
    reference=prepare_reference(source,sdk_target(q[0]),limits,source_sha256=digest(value),
        velocity_limit=velocity_limit,acceleration_limit=5.,timing_policy='retime_allowed')
    if reference.duration_s>maximum:raise ValueError('bounded replay cannot preserve all targets within execution budget')
    return reference


def preview(ref,owner,command,*,device_id,calibration_snapshot_id,maximum_velocity):
    if set(command)!={'action','sampled_trajectory'}:raise ValueError('Raw replay accepts only its bound sampled trajectory')
    if owner.environment=='physical'and maximum_velocity.get('enabled')is not True:
        raise ValueError('physical replay requires existing measured-speed protection')
    velocity=min(1.,maximum_velocity['limit_rad_s'])if maximum_velocity.get('enabled')else 1.
    reference=prepare(command['sampled_trajectory'],owner.cache['joint_limits'],device_id=device_id,
        calibration_snapshot_id=calibration_snapshot_id,jaw_mapping=owner.cache.get('jaw_mapping'),velocity_limit=velocity)
    from .target_interpreter import FEEDBACK50_PROFILES,MAX_PACKET_ROWS
    span=.1 if owner.shared_target_profile in FEEDBACK50_PROFILES else .2
    # All possible fixed-grid window starts; source knots are retained, not dropped
    # to fit the native row cap. Slower explicit replay may make a dense path fit.
    for start in np.arange(0.,reference.duration_s,.01):
        events=reference.breaks[(reference.breaks>=start)&(reference.breaks<start+span)]
        extras=np.count_nonzero(abs(events/.01-np.round(events/.01))>1e-4)
        if round(span/.01)+1+extras>MAX_PACKET_ROWS:
            raise ValueError('Raw sample density exceeds native window capacity; reduce replay speed')
    from .control_reference import preview as reference_preview
    return reference_preview(ref,owner,command,prepared_reference=reference)
