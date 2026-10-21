"""Prepare once off-thread; evaluate a bounded continuous reference in memory.

SDK cubic interpolation is deliberately unused. Its qualified linear interpolator
executes the short sampled q/dq packets produced here.
"""
from dataclasses import dataclass
import hashlib
import json
import math

import numpy as np

JOINTS=tuple([*range(6),*range(7,13)])
SCHEMA='prepared_motion_reference.v1'
MAX_PACKET_ROWS=32
TORQUE_BIAS_LIMITS=(.4,1.2,1.5,.3,.15,.15)
TORQUE_BIAS_SLEW_LIMITS=(.2,.2,.3,.08,.04,.04)
FEEDFORWARD_SLEW_LIMITS=(2.,2.,3.,.8,.4,.4)


def native_holds(config):
    return (4. if config.get('reference_feedforward')else .04,
            4. if config.get('reference_integral_feedback')else .1)


def load_feedforward(root,digest,source_sha256):
    """Read one hash-addressed, bounded study result before the Owner fast path."""
    import os
    import stat
    from pathlib import Path
    if not isinstance(digest,str)or len(digest)!=64 or any(c not in '0123456789abcdef'for c in digest):
        raise ValueError('feedforward SHA256 required')
    path=Path(root)/(digest+'.json');fd=os.open(path,os.O_RDONLY|os.O_NOFOLLOW|os.O_NONBLOCK)
    with os.fdopen(fd,'rb')as f:
        info=os.fstat(f.fileno())
        if not stat.S_ISREG(info.st_mode)or info.st_uid!=os.getuid()or info.st_size>128*1024:
            raise ValueError('owned bounded feedforward result required')
        raw=f.read(128*1024+1)
    if hashlib.sha256(raw).hexdigest()!=digest:raise ValueError('feedforward result digest changed')
    value=json.loads(raw)
    if (set(value)!={'schema','source_sha256','controller_profile','phase','torque_bias_sdk6','provenance'}
            or value['schema']!='reference_feedforward.v1'or value['source_sha256']!=source_sha256
            or value['controller_profile']!='normal_X5'):
        raise ValueError('feedforward source or normal gain profile differs')
    phase=np.asarray(value['phase'],dtype=float);bias=np.asarray(value['torque_bias_sdk6'],dtype=float)
    if (phase.ndim!=1 or not 2<=len(phase)<=2048 or bias.shape!=(len(phase),6)
            or not np.isfinite(phase).all()or not np.isfinite(bias).all()
            or phase[0]!=0. or phase[-1]!=1. or np.any(np.diff(phase)<=0)
            or np.any(np.abs(bias)>np.asarray(TORQUE_BIAS_LIMITS)+1e-12)):
        raise ValueError('feedforward phase, shape or torque bound invalid')
    return dict(sha256=digest,phase=_frozen(phase),bias=_frozen(bias),provenance=value['provenance'])


class ReferenceFeedforward:
    """Frozen repeat feedforward plus slow integral, with one total output limiter."""
    def __init__(self,profile,start,duration,prepared_at):
        self.profile=profile;self.start=start;self.duration=duration;self.prepared_at=prepared_at;self.previous=None

    def packet(self,times,integral):
        times=np.asarray(times);phase=np.clip((times-self.start)/self.duration,0.,1.)
        base=np.asarray([np.interp(phase,self.profile['phase'],self.profile['bias'][:,j])for j in range(6)]).T
        base*=np.clip((times-self.prepared_at)/1.2,0.,1.)[:,None]
        wanted=np.clip(base+np.asarray(integral),-np.asarray(TORQUE_BIAS_LIMITS),TORQUE_BIAS_LIMITS)
        output=np.zeros_like(wanted)
        if self.previous is not None:
            output[0]=[np.interp(times[0],self.previous[0],self.previous[1][:,j])for j in range(6)]
        for i in range(1,len(times)):
            bound=np.asarray(FEEDFORWARD_SLEW_LIMITS)*(times[i]-times[i-1])
            output[i]=np.clip(wanted[i],output[i-1]-bound,output[i-1]+bound)
        self.previous=(times.copy(),output.copy())
        return output.tolist()


class ReferenceIntegral:
    """Experimental slow bias, in SDK units; never changes q/dq or native gains."""
    def __init__(self):
        self.time=None;self.bias=np.zeros(6);self.rate=np.zeros(6)

    def packet(self,now,sample_time,error,host_times):
        error=np.asarray(error,dtype=float);times=np.asarray(host_times,dtype=float)
        if (error.shape!=(6,)or not np.isfinite(error).all()or not math.isfinite(now)
                or not math.isfinite(sample_time)or not 0<=now-sample_time<=.05
                or times.ndim!=1 or not 2<=len(times)<=MAX_PACKET_ROWS
                or not np.isfinite(times).all()or np.any(np.diff(times)<=0)
                or times[0]<now or times[-1]>now+.35):
            raise ValueError('fresh feedback and bounded future window required for reference integral')
        if self.time is not None:
            if not 0<now-self.time<=.2:raise ValueError('reference integral clock gap or reversal')
            self.bias=np.clip(self.bias+self.rate*(now-self.time),-np.array(TORQUE_BIAS_LIMITS),TORQUE_BIAS_LIMITS)
        # Slow integral only; firmware PD and all existing guards remain in charge.
        error=np.where(np.abs(error)<.0005,0.,error)
        rate=np.clip(error*np.array([50.,50.,60.,20.,20.,10.]),
                     -np.array(TORQUE_BIAS_SLEW_LIMITS),TORQUE_BIAS_SLEW_LIMITS)
        # Change slope at the first future knot, leaving transport time on the
        # previous ramp. Admission latency cannot turn a valid ramp into a jump.
        self.bias=np.clip(self.bias+self.rate*(times[0]-now),-np.array(TORQUE_BIAS_LIMITS),TORQUE_BIAS_LIMITS)
        self.time=float(times[0]);self.rate=rate
        values=self.bias[None,:]+(times-times[0])[:,None]*self.rate[None,:]
        return np.clip(values,-np.array(TORQUE_BIAS_LIMITS),TORQUE_BIAS_LIMITS).tolist()


def _frozen(values):
    a=np.asarray(values,dtype=np.float64)
    return np.frombuffer(a.tobytes(),dtype=np.float64).reshape(a.shape)


def _extrema(coefficients,breaks):
    """Exact cubic extrema, including each segment's interior stationary points."""
    lo=np.full(12,np.inf);hi=-lo.copy();velocity=np.zeros(12);acceleration=np.zeros(12)
    for c,width in zip(coefficients,np.diff(breaks)):
        for j in range(12):
            a,b,d,e=c[:,j];ts=[0.,width]
            roots=np.roots([3*a,2*b,d]) if abs(a)+abs(b)>0 else []
            ts.extend(float(r.real)for r in roots if abs(r.imag)<1e-10 and 0<r.real<width)
            q=[((a*t+b)*t+d)*t+e for t in ts]
            lo[j]=min(lo[j],min(q));hi[j]=max(hi[j],max(q))
            vs=[0.,width]
            if abs(a)>1e-15 and 0<-b/(3*a)<width:vs.append(-b/(3*a))
            velocity[j]=max(velocity[j],*(abs((3*a*t+2*b)*t+d)for t in vs))
            acceleration[j]=max(acceleration[j],abs(2*b),abs(6*a*width+2*b))
    return lo,hi,velocity,acceleration


@dataclass(frozen=True)
class PreparedReference:
    breaks:np.ndarray
    coefficients:np.ndarray
    jaws:np.ndarray
    metadata:dict

    @property
    def duration_s(self):return float(self.breaks[-1])

    def sample(self,times):
        t=np.asarray(times,dtype=np.float64)
        if t.ndim!=1 or not 0<len(t)<=MAX_PACKET_ROWS or not np.isfinite(t).all():
            raise ValueError('bounded finite reference sample times required')
        clipped=np.clip(t,0.,self.duration_s)
        i=np.clip(np.searchsorted(self.breaks,clipped,side='right')-1,0,len(self.breaks)-2)
        u=(clipped-self.breaks[i])[:,None];c=self.coefficients[i]
        joint=((c[:,0]*u+c[:,1])*u+c[:,2])*u+c[:,3]
        velocity=(3*c[:,0]*u+2*c[:,1])*u+c[:,2]
        acceleration=6*c[:,0]*u+2*c[:,1]
        velocity[(t<=0)|(t>=self.duration_s)]=0.
        acceleration[(t<0)|(t>self.duration_s)]=0.
        q=np.zeros((len(t),14));dq=np.zeros_like(q);ddq=np.zeros_like(q)
        q[:,JOINTS]=joint;dq[:,JOINTS]=velocity;ddq[:,JOINTS]=acceleration
        alpha=((clipped-self.breaks[i])/np.diff(self.breaks)[i])[:,None]
        q[:,[6,13]]=self.jaws[i]+alpha*(self.jaws[i+1]-self.jaws[i])
        return q,dq,ddq

    def packet(self,elapsed,*,lead_s=.02,horizon_s=.25,period_s=.01):
        if not all(math.isfinite(v)for v in (elapsed,lead_s,horizon_s,period_s)) or not (
                0<=lead_s<=.05 and .05<=horizon_s<=.3 and .002<=period_s<=.02):
            raise ValueError('invalid bounded native reference window')
        n=math.ceil((horizon_s-lead_s)/period_s)+1
        if not 2<=n<=MAX_PACKET_ROWS:raise ValueError('native window exceeds row bound')
        times=elapsed+lead_s+np.arange(n)*period_s
        q,dq,_=self.sample(times)
        return dict(schema=SCHEMA,reference_id=self.metadata['reference_id'],
                    relative_times_s=times.tolist(),q_sdk14=q.tolist(),dq_sdk14=dq.tolist())


def prepare_reference(source,initial_sdk_command14,sdk_limits,*,source_sha256,
                      velocity_limit=.25,acceleration_limit=1.,timing_policy='retime_allowed'):
    """Pure CPU preparation. No SDK, camera, model or file access."""
    from scipy.interpolate import CubicSpline, PchipInterpolator, CubicHermiteSpline
    from .inference_owner import sdk_target
    values=np.asarray(source['device_targets14'],dtype=np.float64)
    initial=np.asarray(initial_sdk_command14,dtype=np.float64)
    limits=np.asarray(sdk_limits,dtype=np.float64)
    duration=source.get('duration_s',2.)
    if (values.ndim!=2 or values.shape[1]!=14 or not 2<=len(values)<=61 or initial.shape!=(14,)or limits.shape!=(14,2)
            or not np.isfinite(values).all()or not np.isfinite(initial).all()
            or not np.isfinite(limits).all()or np.any(limits[:,0]>limits[:,1])
            or type(duration)not in(int,float)or not math.isfinite(duration)or not 0<duration<=2.
            or timing_policy not in ('retime_allowed','strict_duration')
            or not all(math.isfinite(v)and v>0 for v in (velocity_limit,acceleration_limit))):
        raise ValueError('finite bounded path, source duration, initial command and limits required')
    if np.any(initial<limits[:,0])or np.any(initial>limits[:,1]):raise ValueError('initial reference command outside limits')
    sdk=np.asarray([sdk_target(row)for row in values]);sdk[0]=initial
    sdk[:,7:]=initial[7:]  # Existing inactive-right-arm policy; disabled jaw never gains an output.
    if np.any(sdk<limits[:,0]-1e-9)or np.any(sdk>limits[:,1]+1e-9):raise ValueError('reference source outside SDK command limits')
    breaks=np.linspace(0.,duration,len(values))
    online=source.get('kind')=='online_prediction_chunk'
    if online:
        slopes=PchipInterpolator(breaks,sdk[:,JOINTS],axis=0).derivative()(breaks)
        slopes[[0,-1]]=0.
        spline=CubicHermiteSpline(breaks,sdk[:,JOINTS],slopes,axis=0)
    else:spline=CubicSpline(breaks,sdk[:,JOINTS],axis=0,bc_type='clamped')
    coefficients=np.moveaxis(spline.c,1,0)
    low,high,v,a=_extrema(coefficients,breaks)
    if np.any(low<limits[list(JOINTS),0]-1e-9)or np.any(high>limits[list(JOINTS),1]+1e-9):
        raise ValueError('continuous spline exceeds joint command limits; no silent clipping')
    scale=max(1.,float(np.max(v))/velocity_limit,math.sqrt(float(np.max(a))/acceleration_limit))
    from arx_env.joint_head_real_motion import jaw_peak_scale
    jaw_velocity=max(float(np.max(np.abs(np.diff(sdk[:,slot]))/np.diff(breaks)))*jaw_peak_scale(side)
        for side,slot in (('left',6),('right',13)))
    scale=max(scale,jaw_velocity/40.)
    if scale>1.+1e-9 and timing_policy=='strict_duration':raise ValueError('strict 2s reference violates velocity/acceleration bounds')
    scale=scale*1.02 if scale>1 else 1.
    if scale*duration>30.:raise ValueError('prepared path exceeds 30s bound')
    breaks=breaks*scale
    coefficients=coefficients.copy()
    for i,power in enumerate((3,2,1,0)):coefficients[:,i]/=scale**power
    metadata=dict(schema=SCHEMA,source_sha256=source_sha256,source_duration_s=float(duration),duration_s=float(breaks[-1]),
        timing_policy=timing_policy,clock='host_monotonic_relative_seconds',joint_units='rad',jaw_units='SDK_m',
        gain_profile='normal_X5',active_arms=['left'],disabled_grippers=['right'],
        velocity_limit_rad_s=velocity_limit,acceleration_limit_rad_s2=acceleration_limit,
        max_velocity_rad_s=float(np.max(v)/scale),max_acceleration_rad_s2=float(np.max(a)/scale**2),
        interpolation='CPU_correct_cubic_to_SDK_linear_samples',initial_sdk_command14=initial.tolist(),
        model_id=source.get('model_id'),calibration_profile_digest=source.get('calibration_profile_digest'))
    if online:metadata.update(kind='online_prediction_chunk',interpolation='CPU_monotone_Hermite_to_SDK_linear_samples')
    metadata.update(max_jaw_velocity_mm_s=jaw_velocity/scale,jaw_velocity_limit_mm_s=40.)
    digest=hashlib.sha256(json.dumps(metadata,sort_keys=True).encode())
    for array in (breaks,coefficients,sdk[:,[6,13]]):digest.update(array.tobytes())
    metadata['reference_id']=digest.hexdigest()
    return PreparedReference(_frozen(breaks),_frozen(coefficients),_frozen(sdk[:,[6,13]]),metadata)


def prepare_reference_process(source,initial_sdk_command14,sdk_limits,**options):
    """Keep SciPy imports, spline fitting and whole-path checks outside Owner Python."""
    import os
    import subprocess
    import sys
    from .recording_process import _environment
    request=dict(source=source,initial=initial_sdk_command14,limits=sdk_limits,options=options)
    body=json.dumps(request,allow_nan=False).encode()
    if len(body)>128*1024:raise ValueError('preparation request exceeds bounded source size')
    result=subprocess.run([os.environ.get('WORKBENCH_CPU_PYTHON')or sys.executable,'-B','-m',__name__,'--prepare'],
        input=body,capture_output=True,env=_environment(),timeout=20)
    if result.returncode or len(result.stdout)>256*1024:
        raise ValueError('isolated CPU motion preparation failed: '+result.stderr.decode(errors='replace')[-1000:])
    value=json.loads(result.stdout)
    return PreparedReference(_frozen(value['breaks']),_frozen(value['coefficients']),_frozen(value['jaws']),value['metadata'])


if __name__=='__main__':
    import sys
    if sys.argv[1:]!=['--prepare']:raise SystemExit('private CPU preparation worker only')
    raw=sys.stdin.buffer.read(128*1024+1)
    if len(raw)>128*1024:raise ValueError('preparation input exceeds bound')
    request=json.loads(raw)
    reference=prepare_reference(request['source'],request['initial'],request['limits'],**request['options'])
    print(json.dumps(dict(breaks=reference.breaks.tolist(),coefficients=reference.coefficients.tolist(),
                         jaws=reference.jaws.tolist(),metadata=reference.metadata),allow_nan=False,separators=(',',':')))
