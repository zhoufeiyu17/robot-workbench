"""Explicit prepared-reference review for control studies, using the common runner."""
from dataclasses import asdict
from copy import deepcopy
from .execution_settings import DEFAULT_JOINT_TOLERANCE_RAD
import hashlib
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import zipfile

import numpy as np

from .motion_reference import PreparedReference, JOINTS, _extrema, _frozen

SCHEMA = 'workbench.control_reference.v1'
LIMIT = 2 * 1024 * 1024


def _joint_response_arrays(initial, slot, amplitude_deg, direction):
    if (type(slot) is not int or slot not in range(6) or type(amplitude_deg) not in (int,float)
            or amplitude_deg not in (1.,3.,5.) or type(direction) is not int or direction not in (-1,1)):
        raise ValueError('joint response requires one left arm joint, 1/3/5 degrees and signed direction')
    q=np.asarray(initial,dtype=float)
    if q.shape!=(14,) or not np.isfinite(q).all():raise ValueError('finite joint response start required')
    large=direction*np.deg2rad(amplitude_deg);small=direction*np.deg2rad(.15)
    duration=max(1.,2.1875*abs(large)/.03,(7.514*abs(large)/.2)**.5,(52.5*abs(large)/2.)**(1/3))
    duration=float(np.ceil(duration*100.)/100.)  # Exact stage boundaries on the 100 Hz grid.
    b=np.array([0.,1.,1.+duration,2.+duration,2.75+duration,4.75+duration])
    c=np.zeros((5,8,12));c[:,-1]=q[list(JOINTS)];column=JOINTS.index(slot)
    c[2:,-1,column]+=large;c[4,-1,column]+=small
    for segment,delta,width in ((1,large,duration),(3,small,.75)):
        c[segment,:-1,column]=delta*np.array([-20.,70.,-84.,35.,0.,0.,0.])/width**np.arange(7,0,-1)
    return b,c,np.tile(q[[6,13]],(6,1))


def joint_response(initial, limits, *, slot, amplitude_deg, direction):
    """Bounded single-joint large/1s/small probe, using the common q/dq translator."""
    b,c,j=_joint_response_arrays(initial,slot,amplitude_deg,direction)
    m=dict(schema='prepared_motion_reference.v1',study_schema=SCHEMA,study_kind='joint_response_probe',
        initial_sdk_command14=np.asarray(initial,dtype=float).tolist(),probe_joint_index=slot,
        probe_large_deg=amplitude_deg,probe_direction=direction,probe_micro_deg=.15,probe_wait_s=1.,
        duration_s=float(b[-1]),source_range_s=[0.,float(b[-1])],speed_multiplier=None,
        event_times_s=[0.,float(b[-1])],native_grid='reference_origin_100Hz',
        interpolation='bounded_septic_joint_response',max_velocity_rad_s=.03,max_acceleration_rad_s2=.2,
        max_jerk_rad_s3=2.)
    digest=hashlib.sha256(json.dumps(m,sort_keys=True).encode())
    for a in (b,c,j):digest.update(a.tobytes())
    m['reference_id']=digest.hexdigest();raw=io.BytesIO()
    np.savez(raw,breaks=b,coefficients=c,jaws=j,metadata=np.array(json.dumps(m)))
    return decode(raw.getvalue(),limits)


def decode(raw, limits, *, verify=True):
    with zipfile.ZipFile(io.BytesIO(raw)) as archive:
        entries=archive.infolist()
        if len(entries)!=4 or sum(x.file_size for x in entries)>LIMIT:
            raise ValueError('bounded control reference archive required')
    with np.load(io.BytesIO(raw), allow_pickle=False) as saved:
        if set(saved.files) != {'breaks', 'coefficients', 'jaws', 'metadata'}:
            raise ValueError('control reference arrays differ')
        b, c, j = [np.asarray(saved[k], dtype=np.float64) for k in ('breaks', 'coefficients', 'jaws')]
        if saved['metadata'].shape != () or saved['metadata'].dtype.kind not in 'US':
            raise ValueError('control reference metadata must be a scalar string')
        text = str(saved['metadata'])
        if len(text) > 32 * 1024:
            raise ValueError('control reference metadata exceeds bound')
        m = json.loads(text)
    n = len(b)
    static = m.get('study_kind') == 'static_unit_probe'
    response = m.get('study_kind') == 'joint_response_probe'
    if (b.ndim != 1 or not 2 <= n <= 8192 or c.shape not in ((n-1, 4, 12),(n-1, 8, 12)) or j.shape != (n, 2)
            or not all(np.isfinite(x).all() for x in (b, c, j)) or b[0] != 0.
            or not 0 < b[-1] <= 180. or np.any(np.diff(b) <= 0)
            or m.get('study_schema') != SCHEMA or m.get('duration_s') != b[-1]
            or (not (static or response) and (m.get('source_range_s') != [0., 50.] or m.get('speed_multiplier') not in (.5, 1., 1.5, 2.)))
            or m.get('schema') != 'prepared_motion_reference.v1'
            or m.get('event_times_s') != [0., float(b[-1])]):
        raise ValueError('bounded, explicit 50s control reference required')
    if static and (b[-1] > 30. or m.get('source_range_s') != [0., float(b[-1])]
            or m.get('speed_multiplier') is not None or np.any(c[:, :-1] != 0.)
            or np.any(c[:, -1] != c[0, -1]) or np.any(j != j[0])):
        raise ValueError('static unit probe requires fixed joints/jaws, zero derivatives and <=30s')
    if response:
        expected=_joint_response_arrays(m.get('initial_sdk_command14'),m.get('probe_joint_index'),
                                        m.get('probe_large_deg'),m.get('probe_direction'))
        if (b[-1]>30. or m.get('source_range_s')!=[0.,float(b[-1])] or m.get('speed_multiplier') is not None
                or m.get('probe_micro_deg')!=.15 or m.get('probe_wait_s')!=1.
                or any(not np.array_equal(actual,wanted) for actual,wanted in zip((b,c,j),expected))):
            raise ValueError('joint response must match its bounded single-axis large/1s/small definition')
    identity = m.pop('reference_id', None)
    digest = hashlib.sha256(json.dumps(m, sort_keys=True).encode())
    for array in (b, c, j):
        digest.update(array.tobytes())
    if digest.hexdigest() != identity:
        raise ValueError('control reference content digest differs')
    m['reference_id'] = identity
    if verify:
        lim = np.asarray(limits)
        if lim.shape != (14, 2) or not np.isfinite(lim).all():
            raise ValueError('current SDK command limits required')
        lo, hi, velocity, acceleration = _extrema(c, b)
        jaw_speed = np.max(np.abs(np.diff(j, axis=0) / np.diff(b)[:, None]))
        if (np.any(lo < lim[list(JOINTS), 0]-1e-10) or np.any(hi > lim[list(JOINTS), 1]+1e-10)
                or np.max(velocity) > 2.+1e-9 or np.max(acceleration) > 5.+1e-9
                or np.any(j < lim[[6, 13], 0]) or np.any(j > lim[[6, 13], 1]) or jaw_speed > .04+1e-9):
            raise ValueError('control reference exceeds existing position/velocity/acceleration/jaw bounds')
    return PreparedReference(_frozen(b), _frozen(c), _frozen(j), m)


def smooth(reference, *, sigma_s, period_s=.02):
    """Off-thread research preparation; spatial qualification remains mandatory."""
    from scipy.ndimage import gaussian_filter1d
    from scipy.interpolate import make_interp_spline, PPoly
    if (not np.isfinite(sigma_s)or not .02<=sigma_s<=1.
            or not np.isfinite(period_s)or not .01<=period_s<=.05):
        raise ValueError('bounded explicit preparation smoothing and sample period required')
    t=np.linspace(0.,reference.duration_s,int(np.ceil(reference.duration_s/period_s))+1)
    if len(t)>8000:raise ValueError('smooth reference exceeds bounded knot count')
    q=np.vstack([reference.sample(t[i:i+32])[0][:,JOINTS]for i in range(0,len(t),32)])
    filtered=gaussian_filter1d(q,sigma_s/(t[1]-t[0]),axis=0,mode='nearest')
    # Endpoint correction decays locally; never translate the whole spatial path.
    for index in (0,-1):
        weight=np.exp(-.5*((t-t[index])/(3*sigma_s))**2)
        filtered+=weight[:,None]*(q[index]-filtered[index])
    filtered[[0,-1]]=q[[0,-1]]
    # Three zero derivatives join stationary hold without an endpoint jerk jump.
    boundary=[(order,np.zeros(12))for order in (1,2,3)]
    spline=make_interp_spline(t,filtered,k=7,axis=0,bc_type=(boundary,boundary))
    columns=[PPoly.from_spline((spline.t,spline.c[:,j],spline.k))for j in range(12)]
    keep=np.diff(columns[0].x)>0
    breaks=np.r_[columns[0].x[:-1][keep],reference.duration_s]
    # Split the same polynomial at jaw slope changes; do not smooth or advance
    # gripping/releasing while preparing the arm path.
    jaw_v=np.diff(reference.jaws,axis=0)/np.diff(reference.breaks)[:,None]
    jaw_events=reference.breaks[1:-1][np.any(abs(np.diff(jaw_v,axis=0))>1e-10,axis=1)]
    breaks=np.unique(np.r_[breaks,jaw_events])
    import math
    c=np.stack([np.column_stack([p(breaks[:-1],nu=order)for p in columns])/math.factorial(order)
        for order in range(7,-1,-1)],axis=1)
    jaws=np.column_stack([np.interp(breaks,reference.breaks,reference.jaws[:,j])for j in range(2)])
    m=dict(reference.metadata);m.pop('reference_id')
    m.update(parent_reference_id=reference.metadata['reference_id'],
        interpolation='CPU_Gaussian_septic_C6_to_SDK_linear_samples',smoothing_sigma_s=float(sigma_s),
        preparation_period_s=float(t[1]-t[0]),native_grid='reference_origin_100Hz',
        spatial_qualification='required_separately_against_original_path')
    lo,hi,v,a=_extrema(c,breaks)
    m.update(max_velocity_rad_s=float(max(v)),max_acceleration_rad_s2=float(max(a)))
    digest=hashlib.sha256(json.dumps(m,sort_keys=True).encode())
    for array in (breaks,c,jaws):digest.update(array.tobytes())
    m['reference_id']=digest.hexdigest()
    return PreparedReference(_frozen(breaks),_frozen(c),_frozen(jaws),m)


def load(reference, limits):
    """File IO, hashing and whole-path extrema run outside the control process."""
    request = json.dumps(dict(reference=reference, limits=limits), allow_nan=False).encode()
    if len(request) > 64 * 1024:
        raise ValueError('small control reference request required')
    from .recording_process import _environment
    result = subprocess.run([os.environ.get('WORKBENCH_CPU_PYTHON') or sys.executable,
                             '-B', '-m', __name__, '--validate'], input=request,
                            capture_output=True, env=_environment(), timeout=30)
    if result.returncode or not 0 < len(result.stdout) <= LIMIT:
        raise ValueError('control reference validation failed: '+result.stderr.decode(errors='replace')[-700:])
    return decode(result.stdout, limits, verify=False)


def study_input_sha256(study, command):
    """Bind a transient start allowance to exact validated settings and FF identity."""
    return hashlib.sha256(json.dumps(dict(settings=study,feedforward=command.get('feedforward_artifact')),
                                    sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()


def start_allowance_matches(owner, study, command):
    from .target_interpreter import gains
    if not study:return False
    default=(not command.get('feedforward_artifact')
        and study['gains']=={s:dict(zip(('kp','kd'),gains('normal_X5',s)))for s in('left','right')}
        and(not study['integral_enabled']or study.get('integral_gains')=={s:[0.]*6 for s in('left','right')}))
    policy=getattr(owner.backend,'reference_start_allowance',None)or{}
    return default or study_input_sha256(study,command)in policy.get('study_bindings',[])


def check_initial_tracking(owner, reference, measured):
    """Optional frozen-reference study allowance; require continuous commanded hold."""
    start=reference.sample([0.])[0][0]
    error=float(np.max(abs(np.asarray(measured)[list(JOINTS)]-start[list(JOINTS)])))
    if error<=DEFAULT_JOINT_TOLERANCE_RAD:return dict(maximum_error_rad=error,limit_rad=DEFAULT_JOINT_TOLERANCE_RAD,command_continuity_required=False)
    policy=getattr(owner.backend,'reference_start_allowance',None)
    if not policy or policy['reference_id']!=reference.metadata['reference_id'] or error>policy['maximum_error_rad']:
        raise ValueError('control reference start differs from fresh feedback; align first')
    live=getattr(owner,'_preview_live_owner',owner)
    with live.lock:
        runner=getattr(live,'runner',None);prior=getattr(runner,'target_interpreter',None)
        now=live.control_time();feedback=live.cache.get('calibration_feedback')or{}
        if (prior is None or prior.cancelled or prior.epoch!=live.epoch or prior.active_arm!='both'
                or prior.profile!=live.shared_target_profile or runner.status!='READY'
                or now<prior.start_s+prior.reference.duration_s or not live.feedback_healthy
                or not live.cache.get('stationary') or not 0<=now-feedback.get('sample_time_s',-1)<=.05):
            raise ValueError('research start allowance requires fresh stationary completed command hold')
        packet=prior.last_packet or{}
        q=np.asarray(packet.get('q_sdk14',[]));dq=np.asarray(packet.get('dq_sdk14',[]))
        if (q.ndim!=2 or q.shape[1]!=14 or len(q)<2 or dq.shape!=q.shape
                or not np.isfinite(q).all() or not np.isfinite(dq).all()
                or np.max(abs(q[:,list(JOINTS)]-start[list(JOINTS)]))>1e-8
                or np.max(abs(dq[:,list(JOINTS)]))>1e-8):
            raise ValueError('research start allowance requires continuous stationary command nodes')
        arms=(feedback.get('controller_telemetry')or{}).get('arms',{})
        command=np.asarray([*(arms.get('left')or{}).get('output_qpos',[]),*(arms.get('right')or{}).get('output_qpos',[])])
        if command.shape!=(12,)or not np.isfinite(command).all()or np.max(abs(command-start[list(JOINTS)]))>1e-8:
            raise ValueError('research start allowance differs from actual native output')
    return dict(maximum_error_rad=error,limit_rad=policy['maximum_error_rad'],command_continuity_required=True,
        parent_reference_id=prior.reference.metadata['reference_id'],native_output_matched=True)


def preview(ref, owner, command,*,allow_j3_integral_pause=False,allow_start_allowance=False,prepared_reference=None):
    expected=({'action','sampled_trajectory'},)if prepared_reference is not None else(
        {'action', 'reference_artifact'}, {'action', 'reference_artifact','feedforward_artifact'})
    if set(command) not in expected or command['action'] != 'preview_workbench':
        raise ValueError('prepared reference review accepts only its immutable artifact')
    if owner.shared_target_profile is None:
        raise ValueError('prepared reference requires the common target interpreter')
    reference = prepared_reference if prepared_reference is not None else load(command['reference_artifact'], owner.cache['joint_limits'])
    from .target_interpreter import native_period_s
    node_hz=round(1./native_period_s(reference.metadata,owner.shared_target_profile))
    feedforward=None
    if 'feedforward_artifact' in command:
        if owner.shared_target_profile=='normal_X5':
            raise ValueError('control study feedforward requires the shared integral precision profile')
        from .motion_reference import load_feedforward
        artifact=command['feedforward_artifact']
        if not isinstance(artifact,dict)or set(artifact)!={'path','sha256','size_bytes'}:
            raise ValueError('exact immutable feedforward artifact required')
        path=Path(artifact['path'])
        if path.name!=artifact['sha256']+'.json' or not path.is_absolute() or '..' in path.parts:
            raise ValueError('hash-addressed absolute feedforward artifact required')
        feedforward=load_feedforward(path.parent,artifact['sha256'],reference.metadata['reference_id'],controller_profile=owner.shared_target_profile,
            native_reference_api=getattr(owner.backend,'native_reference_api','legacy_recompute'),reference_duration_s=reference.duration_s,
            torque_unit_contract=getattr(owner.backend,'torque_unit_contract','legacy_sdk_v1'),
            gravity_calibration_id=getattr(owner.backend,'gravity_calibration_id',None))
        if path.stat().st_size!=artifact['size_bytes']:
            raise ValueError('feedforward artifact size changed')
        if feedforward.get('j3_integral_pause_s')is not None and not allow_j3_integral_pause:
            raise ValueError('J3 integral pause requires explicit research settings with integral enabled')
    live=getattr(owner,'_preview_live_owner',owner)
    with live.lock:
        owner.cache=deepcopy(live.cache)
    start = reference.sample([0.])[0][0]
    end = reference.sample([reference.duration_s])[0][0]
    from arx_env.joint_head_jaw_mapping import command_gap_mm
    from arx_env.joint_head_real_motion import make_plan
    maximum_velocity=reference.metadata['velocity_limit_rad_s']if prepared_reference is not None else 2.
    waypoint = dict(q12=end[list(JOINTS)].tolist(), speeds12=[maximum_velocity]*12,
                    jaw_gap_mm=[command_gap_mm(s, float(end[i])) for s, i in (('left', 6), ('right', 13))],
                    jaw_speed_mm_s=40.)
    # Reuse the ordinary fresh-feedback, stationary, pedal and pending-plan gates.
    review = ref.Owner.submit(owner, dict(action='preview_workbench', waypoints=[waypoint]))
    token = review['token']
    plans, epoch, press, pid, samples = owner.pending[token]
    previous = plans[0]
    if not allow_start_allowance and np.max(abs(np.asarray(previous.q_start)[list(JOINTS)]-start[list(JOINTS)]))>DEFAULT_JOINT_TOLERANCE_RAD:
        raise ValueError('control reference start differs from fresh feedback; align first')
    initial_tracking=check_initial_tracking(owner,reference,previous.q_start)
    for side, slot in (('left', 6), ('right', 13)):
        if side not in getattr(owner.backend, 'passive_jaws', ()):
            from arx_env.joint_head_jaw_mapping import feedback_gaps
            if abs(feedback_gaps(previous.q_start)[0 if side=='left' else 1]-command_gap_mm(side, float(start[slot]))) > 2.:
                raise ValueError('control reference initial jaw differs from fresh feedback')
    plan = make_plan('both', previous.q_start, end.tolist(), previous.joint_limits,
                     duration_s=reference.duration_s, max_velocity=maximum_velocity, kind='workbench',
                     jaw_speed_mm_s=40., created_at_s=previous.created_at_s, environment=owner.environment)
    owner.pending[token] = ((plan,), epoch, press, pid, samples)
    owner.reference_payloads = {plan.sha256: (reference,feedforward)}
    review.update(plan=asdict(plan), plans=[asdict(plan)], sequence_id=plan.sha256,
                  total_motion_duration_s=reference.duration_s, prepared_reference_id=reference.metadata['reference_id'],
                  prepared_reference_node_hz=node_hz,
                  prepared_velocity_source=reference.metadata.get('velocity_source','prepared_dq'),
                  initial_tracking=initial_tracking,
                  prepared_feedforward_sha256=None if feedforward is None else feedforward['sha256'],
                  prepared_j3_integral_pause_s=None if feedforward is None else feedforward.get('j3_integral_pause_s'),
                  purpose='control precision study; not perception collection or training data')
    if prepared_reference is not None:
        review.update(purpose='bounded Raw replay through the shared executor',raw_replay=deepcopy(reference.metadata['raw_replay']),
            reference_dynamics={k:reference.metadata[k]for k in('max_velocity_rad_s','max_acceleration_rad_s2','max_jaw_velocity_mm_s')})
    return review


if __name__ == '__main__':
    if sys.argv[1:] != ['--validate']:
        raise SystemExit('private CPU reference validation only')
    request = json.loads(sys.stdin.buffer.read(64*1024+1))
    from .control_plan import _read_file
    ref = request['reference']
    if not isinstance(ref, dict) or set(ref) != {'path', 'sha256', 'size_bytes'}:
        raise ValueError('exact immutable control reference artifact required')
    raw = _read_file(ref['path'], LIMIT)
    if len(raw) != ref['size_bytes'] or hashlib.sha256(raw).hexdigest() != ref['sha256']:
        raise ValueError('control reference artifact changed')
    decode(raw, request['limits'])
    sys.stdout.buffer.write(raw)
