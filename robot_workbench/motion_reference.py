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
NATIVE_REFERENCE_APIS=('legacy_recompute','preserve_reference_v1')
TORQUE_BIAS_LIMITS=(.4,1.2,1.5,.3,.15,.15)
TORQUE_BIAS_SLEW_LIMITS=(.2,.2,.3,.08,.04,.04)
FEEDFORWARD_SLEW_LIMITS=(2.,2.,3.,.8,.4,.4)
INTEGRAL_ERROR_GAINS=(50.,50.,60.,20.,20.,10.)
STUDY_BIAS_PROFILE='x5_control50_bias_v1'


def torque_bias_limits(profile='normal_X5',side='left'):
    if side not in ('left','right'):raise ValueError('known arm required')
    if profile=='x5_control50_damping_v1' and side=='left':return (.4,1.2,2.,.3,.15,.15)
    return (.8,1.2,2.3,.65,.15,.15) if profile==STUDY_BIAS_PROFILE and side=='left' else TORQUE_BIAS_LIMITS


def native_holds(config):
    return (4. if config.get('reference_feedforward')else .04,
            4. if config.get('reference_integral_feedback')else .1)


def load_feedforward(root,digest,source_sha256,*,controller_profile='normal_X5',native_reference_api='legacy_recompute',reference_duration_s=None,torque_unit_contract='legacy_sdk_v1',gravity_calibration_id=None):
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
    scheduled=value.get('schema')=='reference_feedforward.v2'
    keys={'schema','source_sha256','controller_profile','phase','torque_bias_sdk6','provenance'}
    if (set(value)!=keys|({'j3_integral_pause_s'}if scheduled else set())
            or value['schema']not in ('reference_feedforward.v1','reference_feedforward.v2')or value['source_sha256']!=source_sha256
            or value['controller_profile']!=controller_profile):
        raise ValueError('feedforward source or gain profile differs')
    if (native_reference_api not in NATIVE_REFERENCE_APIS or not isinstance(value['provenance'],dict)
            or value['provenance'].get('native_reference_api','legacy_recompute')!=native_reference_api):
        raise ValueError('feedforward native reference API differs; requalification required')
    if value['provenance'].get('torque_unit_contract','legacy_sdk_v1')!=torque_unit_contract:
        raise ValueError('feedforward torque unit contract differs; requalification required')
    if value['provenance'].get('gravity_calibration_id')!=gravity_calibration_id:
        raise ValueError('feedforward gravity calibration differs; requalification required')
    phase=np.asarray(value['phase'],dtype=float);bias=np.asarray(value['torque_bias_sdk6'],dtype=float)
    if (phase.ndim!=1 or not 2<=len(phase)<=2048 or bias.shape!=(len(phase),6)
            or not np.isfinite(phase).all()or not np.isfinite(bias).all()
            or phase[0]!=0. or phase[-1]!=1. or np.any(np.diff(phase)<=0)
            or np.any(np.abs(bias)>np.asarray(torque_bias_limits(controller_profile))+1e-12)):
        raise ValueError('feedforward phase, shape or torque bound invalid')
    result=dict(sha256=digest,phase=_frozen(phase),bias=_frozen(bias),provenance=value['provenance'],
                controller_profile=controller_profile)
    if scheduled:
        if controller_profile!='x5_control50_damping_v1'or native_reference_api!='preserve_reference_v1':
            raise ValueError('J3 integral pause requires the qualified research native-reference profile')
        result['j3_integral_pause_s']=_j3_pause(value['j3_integral_pause_s'],reference_duration_s)
    return result


def _j3_pause(value,duration):
    if (not isinstance(value,(list,tuple))or len(value)!=4
            or any(type(x)not in(int,float)or not math.isfinite(x)for x in value)
            or type(duration)not in(int,float)or not math.isfinite(duration)
            or not 0<=value[0]<value[1]<value[2]<value[3]<=duration
            or min(value[1]-value[0],value[3]-value[2])<.5):
        raise ValueError('J3 integral pause requires four ordered reference seconds, >=0.5s ramps, within duration')
    return tuple(float(x)for x in value)


class ReferenceFeedforward:
    """Frozen repeat feedforward plus slow integral, with one total output limiter."""
    def __init__(self,profile,start,duration,prepared_at,initial=None):
        self.profile=profile;self.start=start;self.duration=duration;self.prepared_at=prepared_at;self.previous=None
        self.limits=np.asarray(torque_bias_limits(profile.get('controller_profile','normal_X5')))
        self.initial=np.zeros(6) if initial is None else np.asarray(initial,dtype=float)
        if self.initial.shape!=(6,)or not np.isfinite(self.initial).all()or np.any(np.abs(self.initial)>self.limits):
            raise ValueError('bounded initial feedforward compensation required')
        pause=profile.get('j3_integral_pause_s')
        self.j3_pause=None if pause is None else tuple(start+x for x in _j3_pause(pause,duration))

    def packet(self,times,integral,*,integrator=None):
        times=np.asarray(times);phase=np.clip((times-self.start)/self.duration,0.,1.)
        base=np.asarray([np.interp(phase,self.profile['phase'],self.profile['bias'][:,j])for j in range(6)]).T
        ramp=np.clip((times-self.prepared_at)/1.2,0.,1.)[:,None]
        base=base*ramp+self.initial[None,:]*(1.-ramp)
        residual=np.asarray(integral)
        if integrator is not None:
            # The frozen base consumes part of the SAME output allowance.
            # Do not store torque which the combined limiter cannot deliver.
            prior=integrator.bias.copy()
            integrator.bias=np.clip(prior,np.maximum(-integrator.limits,-self.limits-base[0]),
                np.minimum(integrator.limits,self.limits-base[0]))
            residual=np.clip(residual+integrator.bias-prior,-integrator.limits,integrator.limits)
        wanted=np.clip(base+residual,-self.limits,self.limits)
        output=np.zeros_like(wanted);output[0]=self.initial
        if self.previous is not None:
            output[0]=[np.interp(times[0],self.previous[0],self.previous[1][:,j])for j in range(6)]
        for i in range(1,len(times)):
            bound=np.asarray(FEEDFORWARD_SLEW_LIMITS)*(times[i]-times[i-1])
            output[i]=np.clip(wanted[i],output[i-1]-bound,output[i-1]+bound)
        self.previous=(times.copy(),output.copy())
        return output.tolist()


class ReferenceIntegral:
    """Experimental slow bias, in SDK units; never changes q/dq or native gains."""
    def __init__(self,rapid=False,rapid_scale=4,profile='normal_X5',side='left',gains_sdk6=None):
        if type(rapid)is not bool or type(rapid_scale)is not int or rapid_scale not in (4,8):raise ValueError('explicit integral policy required')
        self.rapid=rapid;self.rapid_scale=rapid_scale;self.time=None;self.observed_time=None;self.bias=np.zeros(6);self.rate=np.zeros(6)
        self.limits=np.asarray(torque_bias_limits(profile,side))
        self.side=side;self.j3_pause=None
        self.gains=np.asarray(INTEGRAL_ERROR_GAINS)*(rapid_scale if rapid else 1.)
        if gains_sdk6 is not None:
            if (not isinstance(gains_sdk6,(list,tuple))or len(gains_sdk6)!=6 or any(
                    type(v)not in(int,float)or not math.isfinite(v)or not 0<=v<=cap
                    for v,cap in zip(gains_sdk6,self.gains))):
                raise ValueError('six finite integral gains within the existing profile envelope required')
            self.gains=np.asarray(gains_sdk6,dtype=float)
        self.gains=_frozen(self.gains)
        self.effective_rate=np.zeros(6)

    def packet(self,now,sample_time,error,host_times,*,j3_pause=None):
        error=np.asarray(error,dtype=float);times=np.asarray(host_times,dtype=float)
        if (error.shape!=(6,)or not np.isfinite(error).all()or not math.isfinite(now)
                or not math.isfinite(sample_time)or not 0<=now-sample_time<=.05
                or times.ndim!=1 or not 2<=len(times)<=MAX_PACKET_ROWS
                or not np.isfinite(times).all()or np.any(np.diff(times)<=0)
                or times[0]<now or times[-1]>now+.35):
            raise ValueError('fresh feedback and bounded future window required for reference integral')
        if j3_pause is not None and (self.side!='left'or not isinstance(j3_pause,tuple)or len(j3_pause)!=4
                or not all(math.isfinite(x)for x in j3_pause)or not all(a<b for a,b in zip(j3_pause,j3_pause[1:]))):
            raise ValueError('validated left J3 integral pause required')
        if self.time is not None and j3_pause!=self.j3_pause:
            raise ValueError('integral pause cannot change within an active reference')
        self.j3_pause=j3_pause
        weighted=None
        if j3_pause is not None:
            # Integrate the quintic update weight across BOTH elapsed and future
            # intervals; window boundaries cannot leak accumulation into a pause.
            a,b,c,d=j3_pause;x=np.r_[times[0]if self.time is None else self.time,times]-a
            def primitive(x,width):
                z=np.clip(x/width,0.,1.)
                return width*z**4*(z*z-3*z+2.5)+np.maximum(x-width,0.)
            weighted=x-primitive(x,b-a)+primitive(x-(c-a),d-c)
        if self.time is not None:
            # A coarser knot grid can reuse a still-future first knot on the next
            # upload. Its bias advances by zero; the observation clock must advance.
            if not 0<now-self.observed_time<=.2 or not 0<=times[0]-self.time<=.2:raise ValueError('reference integral clock gap or reversal')
            advance=self.rate*(times[0]-self.time)
            if weighted is not None:advance[2]=self.rate[2]*(weighted[1]-weighted[0])
            self.bias=np.clip(self.bias+advance,-self.limits,self.limits)
        # Slow integral only; firmware PD and all existing guards remain in charge.
        error=np.where(np.abs(error)<.0005,0.,error)
        slew=np.asarray(FEEDFORWARD_SLEW_LIMITS)*(.8 if self.rapid_scale==8 else .5) if self.rapid else np.asarray(TORQUE_BIAS_SLEW_LIMITS)
        rate=np.clip(error*self.gains,-slew,slew)
        # Change slope at the first future knot, leaving transport time on the
        # previous ramp. Admission latency cannot turn a valid ramp into a jump.
        self.time=float(times[0]);self.observed_time=float(now);self.rate=rate
        values=self.bias[None,:]+(times-times[0])[:,None]*self.rate[None,:]
        if weighted is not None:values[:,2]=self.bias[2]+(weighted[1:]-weighted[1])*self.rate[2]
        values=np.clip(values,-self.limits,self.limits)
        self.effective_rate=(values[1]-values[0])/(times[1]-times[0])
        return values.tolist()


def _frozen(values):
    a=np.asarray(values,dtype=np.float64)
    return np.frombuffer(a.tobytes(),dtype=np.float64).reshape(a.shape)


def _extrema(coefficients,breaks):
    """Polynomial extrema at endpoints and all real interior stationary points."""
    lo=np.full(12,np.inf);hi=-lo.copy();velocity=np.zeros(12);acceleration=np.zeros(12)
    for c,width in zip(coefficients,np.diff(breaks)):
        for j in range(12):
            if len(c)!=4:
                derivatives=[np.polyder(c[:,j],order)for order in range(4)]
                extrema=[]
                for values,derivative in zip(derivatives,derivatives[1:]):
                    roots=np.roots(derivative)
                    points=[0.,width,*[float(r.real)for r in roots if abs(r.imag)<1e-9 and 0<r.real<width]]
                    extrema.append(np.polyval(values,points))
                lo[j]=min(lo[j],min(extrema[0]));hi[j]=max(hi[j],max(extrema[0]))
                velocity[j]=max(velocity[j],max(abs(extrema[1])))
                acceleration[j]=max(acceleration[j],max(abs(extrema[2])))
                continue
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

    @property
    def event_times(self):
        return np.asarray(self.metadata['event_times_s']) if 'event_times_s' in self.metadata else self.breaks

    def sample(self,times):
        t=np.asarray(times,dtype=np.float64)
        if t.ndim!=1 or not 0<len(t)<=MAX_PACKET_ROWS or not np.isfinite(t).all():
            raise ValueError('bounded finite reference sample times required')
        clipped=np.clip(t,0.,self.duration_s)
        i=np.clip(np.searchsorted(self.breaks,clipped,side='right')-1,0,len(self.breaks)-2)
        u=(clipped-self.breaks[i])[:,None];c=self.coefficients[i]
        joint=c[:,0].copy();velocity=np.zeros_like(joint);acceleration=np.zeros_like(joint)
        for term in np.moveaxis(c[:,1:],1,0):
            acceleration=acceleration*u+2*velocity
            velocity=velocity*u+joint
            joint=joint*u+term
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
