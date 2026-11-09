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


def _check_c2(coefficients,breaks):
    """Numerical polynomial identity check, not a physical tracking tolerance."""
    for index,(c,width)in enumerate(zip(coefficients,np.diff(breaks))):
        q=c[0].copy();v=np.zeros(12);a=np.zeros(12)
        for term in c[1:]:
            a=a*width+2*v;v=v*width+q;q=q*width+term
        expected=([coefficients[index+1,-1],coefficients[index+1,-2],2*coefficients[index+1,-3]]
                  if index+1<len(coefficients)else[q,np.zeros(12),np.zeros(12)])
        if any(not np.allclose(x,y,rtol=1e-9,atol=1e-9)for x,y in zip((q,v,a),expected)):
            raise ValueError('normal reference requires joint C2 joins and a zero-v/a terminal hold')


def infer_reference_policy(value):
    """Explicit current-infer policy; never changes legacy/research defaults."""
    expected=dict(schema='workbench.infer-reference-policy.v1',curve='cubic_c2_quintic_ends',
        velocity_limit_rad_s=1.,acceleration_limit_rad_s2=40.,jaw_velocity_limit_mm_s=40.,
        boundary_scope='complete_prepared_segment',timing_policy='retime_allowed')
    if (not isinstance(value,dict)or set(value)!=set(expected)|{'max_execution_duration_s'}
            or any(value[k]!=v for k,v in expected.items() if k not in ('velocity_limit_rad_s','acceleration_limit_rad_s2','jaw_velocity_limit_mm_s'))
            or any(type(value[k])not in(int,float) for k in('velocity_limit_rad_s','acceleration_limit_rad_s2','jaw_velocity_limit_mm_s','max_execution_duration_s'))
            or any(not math.isfinite(value[k]) or not 0<value[k]<=expected[k] for k in ('velocity_limit_rad_s','acceleration_limit_rad_s2','jaw_velocity_limit_mm_s'))
            or not math.isfinite(value['max_execution_duration_s'])or not 0<value['max_execution_duration_s']<=8.):
        raise ValueError('explicit reviewed infer curve policy and complete-segment boundaries required')
    result=dict(expected,max_execution_duration_s=float(value['max_execution_duration_s']))
    for key in ('velocity_limit_rad_s','acceleration_limit_rad_s2','jaw_velocity_limit_mm_s'):
        result[key]=float(value[key])
    return result


def _quintic_end(q0,q1,v0,v1,a0,a1,width):
    """Hermite coefficients for one endpoint segment, vectorized over joints."""
    d=q1-q0-v0*width-a0*width**2/2
    v=(v1-v0)*width-a0*width**2;a=(a1-a0)*width**2
    return np.stack([6*d-3*v+a/2,-15*d+7*v-a,10*d-4*v+a/2,
        a0*width**2/2,v0*width,q0])/width**np.arange(5,-1,-1)[:,None]


def _cubic_with_quintic_ends(breaks,q,initial_velocity,initial_acceleration):
    from scipy.interpolate import CubicSpline
    zero=np.zeros(12)
    if len(breaks)==2:
        return _quintic_end(q[0],q[-1],initial_velocity,zero,initial_acceleration,zero,breaks[-1])[None]
    spline=CubicSpline(breaks,q,axis=0,bc_type=((1,initial_velocity),(1,zero)))
    c=np.pad(np.moveaxis(spline.c,1,0),((0,0),(2,0),(0,0)))
    # Interior cubic q/v/a are shared at every join. Only the two outer a
    # constraints need quintics; interior accelerations are never forced to 0.
    c[0]=_quintic_end(q[0],q[1],initial_velocity,spline(breaks[1],1),
        initial_acceleration,spline(breaks[1],2),breaks[1]-breaks[0])
    c[-1]=_quintic_end(q[-2],q[-1],spline(breaks[-2],1),zero,
        spline(breaks[-2],2),zero,breaks[-1]-breaks[-2])
    return c


def _root_real_parts(rows):
    """All real-root candidates, grouped by exact polynomial degree.

    Include the real parts of complex roots as extra sample points too. This
    cannot enlarge the true range and avoids dropping near-real multiple roots
    via an imaginary tolerance. No stationary-root sampling shortcut is used.
    """
    if rows.shape[1]<=1:return np.empty((len(rows),0))
    maximum=rows.shape[1]-1;roots=np.full((len(rows),maximum),np.nan)
    nonzero=rows!=0;degree=np.where(nonzero.any(axis=1),maximum-nonzero.argmax(axis=1),0)
    for order in range(1,maximum+1):
        indices=np.flatnonzero(degree==order)
        if not len(indices):continue
        c=rows[indices,-order-1:]
        if order==1:values=(-c[:,1]/c[:,0])[:,None]
        else:
            companion=np.zeros((len(indices),order,order))
            companion[:,1:,:-1]=np.eye(order-1)
            companion[:,0,:]=-c[:,1:]/c[:,0,None]
            if not np.isfinite(companion).all():raise ValueError('nonfinite polynomial companion; no path admission')
            values=np.linalg.eigvals(companion).real
        if not np.isfinite(values).all():raise ValueError('nonfinite polynomial roots; no path admission')
        roots[indices,:order]=values
    return roots


def _extrema_batched(coefficients,breaks):
    """Full polynomial extrema, vectorized in normalized segment time [0,1]."""
    c=np.asarray(coefficients,dtype=float);b=np.asarray(breaks,dtype=float)
    if (c.ndim!=3 or c.shape[2]!=12 or not 2<=c.shape[1]<=6 or b.shape!=(len(c)+1,)
            or not np.isfinite(c).all()or not np.isfinite(b).all()or np.any(np.diff(b)<=0)):
        raise ValueError('finite piecewise polynomial and increasing breaks required')
    widths=np.diff(b);powers=np.arange(c.shape[1]-1,-1,-1)
    rows=(c*widths[:,None,None]**powers[None,:,None]).transpose(0,2,1).reshape(-1,len(powers))
    bounds=[]
    for order in range(3):
        derivative=rows[:,:-1]*np.arange(rows.shape[1]-1,0,-1)
        roots=_root_real_parts(derivative)
        candidates=np.c_[np.zeros(len(rows)),np.ones(len(rows)),np.where((roots>0)&(roots<1),roots,0.)]
        values=np.zeros_like(candidates)
        for coefficient in rows.T:values=values*candidates+coefficient[:,None]
        values/=np.repeat(widths,12)[:,None]**order
        if not np.isfinite(values).all():raise ValueError('nonfinite polynomial extrema; no path admission')
        bounds.append((values.min(axis=1).reshape(-1,12).min(axis=0),
                       values.max(axis=1).reshape(-1,12).max(axis=0)))
        rows=derivative
    return bounds[0][0],bounds[0][1],np.maximum(abs(bounds[1][0]),abs(bounds[1][1])),np.maximum(abs(bounds[2][0]),abs(bounds[2][1]))


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
        velocity[(t<0)|(t>=self.duration_s)]=0.
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
                      velocity_limit=.25,acceleration_limit=1.,timing_policy='retime_allowed',sdk_targets14=None,jaw_scales=None,
                      planning_policy=None,initial_velocity_sdk14=None,initial_acceleration_sdk14=None,timing=None):
    """Pure CPU preparation. No SDK, camera, model or file access."""
    from scipy.interpolate import CubicSpline, PchipInterpolator, BPoly, PPoly
    from .inference_owner import sdk_target
    import time
    started=time.perf_counter();fit_wall=extrema_wall=0.
    policy=None if planning_policy is None else infer_reference_policy(planning_policy)
    if policy is not None:
        if source.get('kind')!='online_prediction_chunk':raise ValueError('new planning policy is only for explicit online infer segments')
        if timing_policy!=policy['timing_policy']:raise ValueError('timing policy differs from frozen infer policy')
        velocity_limit=policy['velocity_limit_rad_s'];acceleration_limit=policy['acceleration_limit_rad_s2']
    elif initial_velocity_sdk14 is not None or initial_acceleration_sdk14 is not None:
        raise ValueError('explicit start derivatives require the reviewed infer curve policy')
    values=np.asarray(source['device_targets14'],dtype=np.float64)
    initial=np.asarray(initial_sdk_command14,dtype=np.float64)
    limits=np.asarray(sdk_limits,dtype=np.float64)
    duration=source.get('duration_s',2.)
    replay=source.get('kind')=='raw_replay'
    if (values.ndim!=2 or values.shape[1]!=14 or not 2<=len(values)<=(514 if replay else 61) or initial.shape!=(14,)or limits.shape!=(14,2)
            or not np.isfinite(values).all()or not np.isfinite(initial).all()
            or not np.isfinite(limits).all()or np.any(limits[:,0]>limits[:,1])
            or type(duration)not in(int,float)or not math.isfinite(duration)or not 0<duration<=(30. if replay else 2.)
            or timing_policy not in ('retime_allowed','strict_duration')
            or not all(math.isfinite(v)and v>0 for v in (velocity_limit,acceleration_limit))):
        raise ValueError('finite bounded path, source duration, initial command and limits required')
    if np.any(initial<limits[:,0])or np.any(initial>limits[:,1]):raise ValueError('initial reference command outside limits')
    sdk=np.asarray([sdk_target(row)for row in values]if sdk_targets14 is None else sdk_targets14,dtype=float).copy()
    if sdk.shape!=values.shape or not np.isfinite(sdk).all():raise ValueError('finite preconverted SDK reference required')
    sdk[0]=initial
    active_arms=source.get('active_arms',['left'])
    if active_arms not in (['left'],['left','right']):raise ValueError('explicit supported active arms required')
    if active_arms==['left']:sdk[:,7:]=initial[7:]
    if np.any(sdk<limits[:,0]-1e-9)or np.any(sdk>limits[:,1]+1e-9):raise ValueError('reference source outside SDK command limits')
    breaks=np.linspace(0.,duration,len(values))
    if replay:
        breaks=np.asarray(source['source_times_s'],dtype=float)
        if (breaks.shape!=(len(values),)or not np.isfinite(breaks).all()or breaks[0]!=0.
                or breaks[-1]!=duration or np.any(np.diff(breaks)<=0)):
            raise ValueError('Raw replay must retain ordered source sample intervals')
    online=source.get('kind')=='online_prediction_chunk'
    fit_started=time.perf_counter()
    if policy is not None:
        v0=np.asarray(initial_velocity_sdk14,dtype=float);a0=np.asarray(initial_acceleration_sdk14,dtype=float)
        if (v0.shape!=(14,)or a0.shape!=(14,)or not np.isfinite(v0).all()or not np.isfinite(a0).all()
                or np.any(v0[[6,13]]!=0)or np.any(a0[[6,13]]!=0)
                or np.max(abs(v0))>velocity_limit or np.max(abs(a0))>acceleration_limit
                or active_arms==['left']and(np.any(v0[7:]!=0)or np.any(a0[7:]!=0))):
            raise ValueError('explicit feasible start q/v/a and position-only jaws required')
        fit_started=time.perf_counter()
        coefficients=_cubic_with_quintic_ends(breaks,sdk[:,JOINTS],v0[list(JOINTS)],a0[list(JOINTS)])
        fit_wall+=time.perf_counter()-fit_started
    elif online or replay:
        slopes=PchipInterpolator(breaks,sdk[:,JOINTS],axis=0).derivative()(breaks)
        slopes[[0,-1]]=0.
        # One C2 polynomial supplies q/v/a, including zero-acceleration ends.
        # Jaws retain their separate position-only, piecewise-linear contract.
        spline=PPoly.from_bernstein_basis(BPoly.from_derivatives(
            breaks,[[q,v,np.zeros(12)]for q,v in zip(sdk[:,JOINTS],slopes)]))
    else:spline=CubicSpline(breaks,sdk[:,JOINTS],axis=0,bc_type='clamped')
    if policy is None:
        coefficients=np.moveaxis(spline.c,1,0)
        fit_wall=time.perf_counter()-fit_started
    extrema_started=time.perf_counter()
    extrema=_extrema_batched if policy is not None else _extrema
    low,high,v,a=extrema(coefficients,breaks)
    extrema_wall+=time.perf_counter()-extrema_started
    if np.any(low<limits[list(JOINTS),0]-1e-9)or np.any(high>limits[list(JOINTS),1]+1e-9):
        raise ValueError('continuous spline exceeds joint command limits; no silent clipping')
    scale=max(1.,float(np.max(v))/velocity_limit,math.sqrt(float(np.max(a))/acceleration_limit))
    from arx_env.joint_head_real_motion import jaw_peak_scale
    jaw_scales=jaw_scales or {side:jaw_peak_scale(side)for side in ('left','right')}
    if set(jaw_scales)!={'left','right'}or any(not math.isfinite(v)or v<=0 for v in jaw_scales.values()):raise ValueError('finite positive jaw unit scales required')
    jaw_velocity=max(float(np.max(np.abs(np.diff(sdk[:,slot]))/np.diff(breaks)))*jaw_scales[side]
        for side,slot in (('left',6),('right',13)))
    scale=max(scale,jaw_velocity/40.)
    if scale>1.+1e-9 and timing_policy=='strict_duration':raise ValueError('strict 2s reference violates velocity/acceleration bounds')
    scale=scale*1.02 if scale>1 else 1.
    maximum_duration=policy['max_execution_duration_s']if policy is not None else 30.
    if scale*duration>maximum_duration:raise ValueError(f'prepared path exceeds {maximum_duration:g}s bound')
    refits=0
    if policy is not None and (np.any(v0!=0)or np.any(a0!=0)) and scale>1:
        # Stretching coefficients would silently change nonzero initial v/a.
        # Refit at the longer duration with the same physical boundary instead.
        for refits in range(1,9):
            breaks=breaks*scale
            if breaks[-1]>maximum_duration:raise ValueError(f'prepared path exceeds {maximum_duration:g}s bound')
            fit_started=time.perf_counter()
            coefficients=_cubic_with_quintic_ends(breaks,sdk[:,JOINTS],v0[list(JOINTS)],a0[list(JOINTS)])
            fit_wall+=time.perf_counter()-fit_started
            extrema_started=time.perf_counter();low,high,v,a=extrema(coefficients,breaks)
            extrema_wall+=time.perf_counter()-extrema_started
            if np.any(low<limits[list(JOINTS),0]-1e-9)or np.any(high>limits[list(JOINTS),1]+1e-9):
                raise ValueError('retimed curve exceeds joint command limits; no silent clipping')
            jaw_velocity=max(float(np.max(np.abs(np.diff(sdk[:,slot]))/np.diff(breaks)))*jaw_scales[side]
                for side,slot in(('left',6),('right',13)))
            scale=max(1.,float(np.max(v))/velocity_limit,math.sqrt(float(np.max(a))/acceleration_limit),jaw_velocity/40.)
            if scale<=1.+1e-9:scale=1.;break
            scale*=1.02
        else:raise ValueError('bounded retiming did not preserve feasible initial derivatives')
    else:
        breaks=breaks*scale
        coefficients=coefficients.copy()
        for i,power in enumerate(range(coefficients.shape[1]-1,-1,-1)):coefficients[:,i]/=scale**power
    metadata=dict(schema=SCHEMA,source_sha256=source_sha256,source_duration_s=float(duration),duration_s=float(breaks[-1]),
        timing_policy=timing_policy,clock='host_monotonic_relative_seconds',joint_units='rad',jaw_units='SDK_m',
        gain_profile='normal_X5',active_arms=active_arms,disabled_grippers=['right']if active_arms==['left']else[],
        velocity_limit_rad_s=velocity_limit,acceleration_limit_rad_s2=acceleration_limit,
        max_velocity_rad_s=float(np.max(v)/scale),max_acceleration_rad_s2=float(np.max(a)/scale**2),
        interpolation='CPU_correct_cubic_to_SDK_linear_samples',initial_sdk_command14=initial.tolist(),
        model_id=source.get('model_id'),calibration_profile_digest=source.get('calibration_profile_digest'))
    if online:metadata.update(kind='online_prediction_chunk',interpolation='CPU_C2_quintic_to_SDK_linear_samples',
        native_grid='reference_origin_100Hz',
        derivative_scope='analytic_joint_q_v_a; jaws_position_only',continuity='joint_C2',
        jerk_limit='not_configured_not_qualified')
    if replay:
        _check_c2(coefficients,breaks)
        metadata.update(kind='raw_replay',raw_replay=source['raw_replay'],native_grid='reference_origin_100Hz',
            interpolation='CPU_C2_quintic_to_SDK_linear_samples',continuity='joint_C2',
            derivative_scope='analytic_joint_q_v_a; jaws_position_only',jerk_limit='not_configured_not_qualified')
    if policy is not None:
        _check_c2(coefficients,breaks)
        metadata.update(planning_policy=policy,interpolation='CPU_C2_cubic_with_quintic_ends_to_SDK_linear_samples',
            initial_velocity_sdk14=v0.tolist(),initial_acceleration_sdk14=a0.tolist(),retiming_refits=refits,
            internal_derivatives='cubic_spline_solved_v_and_a; not_zeroed')
    metadata.update(max_jaw_velocity_mm_s=jaw_velocity/scale,jaw_velocity_limit_mm_s=40.)
    digest=hashlib.sha256(json.dumps(metadata,sort_keys=True).encode())
    for array in (breaks,coefficients,sdk[:,[6,13]]):digest.update(array.tobytes())
    metadata['reference_id']=digest.hexdigest()
    if timing is not None:timing.update(fit_s=fit_wall,extrema_s=extrema_wall,prepare_total_s=time.perf_counter()-started,retiming_refits=refits)
    return PreparedReference(_frozen(breaks),_frozen(coefficients),_frozen(sdk[:,[6,13]]),metadata)


def prepare_brake(initial, velocity, acceleration, limits, *, duration_s,
                  velocity_limit, acceleration_limit, source_sha256):
    """Finite normal brake from a known future reference state; no SDK/IO.

    Duration and dynamic bounds come from the caller's frozen configuration.
    A longer brake is not necessarily feasible near a joint limit: reject the
    entire polynomial on any violation, never clip or relax limits.
    """
    q,v,a=(np.asarray(x,dtype=float)for x in(initial,velocity,acceleration))
    bounds=np.asarray(limits,dtype=float)
    if (any(x.shape!=(14,)or not np.isfinite(x).all()for x in(q,v,a))
            or bounds.shape!=(14,2)or not np.isfinite(bounds).all()
            or np.any(bounds[:,0]>bounds[:,1])
            or np.any(q<bounds[:,0])or np.any(q>bounds[:,1])
            or any(type(x)not in(int,float)or not math.isfinite(x)or x<=0
                   for x in(duration_s,velocity_limit,acceleration_limit))
            or duration_s>30. or np.any(v[[6,13]]!=0)or np.any(a[[6,13]]!=0)):
        raise ValueError('bounded brake state, explicit dynamics and held jaws required')
    t=float(duration_s);qj,vj,aj=q[list(JOINTS)],v[list(JOINTS)],a[list(JOINTS)]
    # Cubic velocity meets (v0,a0)->(0,0); its integral is the position.
    c=np.asarray([vj/(2*t**3)+aj/(4*t*t),-vj/(t*t)-2*aj/(3*t),aj/2,vj,qj])[None,:,:]
    breaks=np.asarray([0.,t]);low,high,peak_v,peak_a=_extrema_batched(c,breaks)
    if (np.any(low<bounds[list(JOINTS),0]-1e-10)or np.any(high>bounds[list(JOINTS),1]+1e-10)
            or np.max(peak_v)>velocity_limit+1e-10 or np.max(peak_a)>acceleration_limit+1e-10):
        raise ValueError('normal brake infeasible within position/velocity/acceleration bounds')
    meta=dict(schema=SCHEMA,kind='normal_brake',source_sha256=source_sha256,duration_s=t,
        clock='host_monotonic_relative_seconds',joint_units='rad',jaw_units='SDK_m',
        max_velocity_rad_s=float(np.max(peak_v)),max_acceleration_rad_s2=float(np.max(peak_a)),
        velocity_limit_rad_s=velocity_limit,acceleration_limit_rad_s2=acceleration_limit,
        initial_sdk_command14=q.tolist(),initial_velocity_sdk14=v.tolist(),initial_acceleration_sdk14=a.tolist(),
        hold_policy='fresh_measured_pose_after_brake; fixed_once; separate_from_brake_endpoint',
        jerk_limit='not_configured_not_qualified')
    jaws=np.tile(q[[6,13]],(2,1))
    identity=hashlib.sha256(json.dumps(meta,sort_keys=True,allow_nan=False).encode())
    for array in(breaks,c,jaws):identity.update(array.tobytes())
    meta['reference_id']=identity.hexdigest()
    return PreparedReference(_frozen(breaks),_frozen(c),_frozen(jaws),meta)


def prepare_brake_after(reference, boundary_s, limits, **options):
    """Keep the prepared prefix exactly, then brake from its future q/v/a.

    The Owner must separately verify that the boundary is still future and
    covers its committed native prefix before installing this candidate.
    """
    if (not isinstance(reference,PreparedReference)or type(boundary_s)not in(int,float)
            or not math.isfinite(boundary_s)or not 0<boundary_s<30.):
        raise ValueError('prepared reference and an interior/end brake boundary required')
    q,v,a=reference.sample([boundary_s])
    brake=prepare_brake(q[0],v[0],a[0],limits,
        source_sha256=reference.metadata['reference_id'],**options)
    if boundary_s+brake.duration_s>30.:raise ValueError('prefix and brake exceed the 30s reference bound')
    starts=reference.breaks[reference.breaks<boundary_s]
    degree=max(reference.coefficients.shape[1],brake.coefficients.shape[1])
    def pad(c):return np.pad(c,((0,0),(degree-c.shape[1],0),(0,0)))
    prefix=pad(reference.coefficients[:len(starts)])
    if boundary_s>reference.duration_s:
        tail=np.zeros((1,degree,12));tail[0,-1]=q[0,list(JOINTS)]
        prefix=np.concatenate([prefix,tail])
    c=np.concatenate([prefix,pad(brake.coefficients)])
    breaks=np.r_[starts,boundary_s,boundary_s+brake.duration_s]
    jaws=np.vstack([reference.jaws[:len(starts)],brake.jaws])
    # Keep the full prefix proof within the live admission window, using the
    # same complete polynomial extrema calculation as model preparation.
    bounds=np.asarray(limits,dtype=float);lo,hi,pv,pa=_extrema_batched(c,breaks)
    if (np.any(lo<bounds[list(JOINTS),0]-1e-10)or np.any(hi>bounds[list(JOINTS),1]+1e-10)
            or max(pv)>options['velocity_limit']+1e-10 or max(pa)>options['acceleration_limit']+1e-10):
        raise ValueError('committed prefix and brake exceed declared bounds')
    meta=dict(brake.metadata,kind='prefix_then_normal_brake',duration_s=float(breaks[-1]),
        parent_reference_id=reference.metadata['reference_id'],brake_start_relative_s=float(boundary_s),
        preserved_prefix_end_relative_s=float(boundary_s),max_velocity_rad_s=float(max(pv)),
        max_acceleration_rad_s2=float(max(pa)))
    first=reference.sample([0.])
    for index,key in enumerate(('initial_sdk_command14','initial_velocity_sdk14','initial_acceleration_sdk14')):
        meta['brake_'+key]=meta[key];meta[key]=first[index][0].tolist()
    if 'native_grid'in reference.metadata:meta['native_grid']=reference.metadata['native_grid']
    meta.pop('reference_id')
    identity=hashlib.sha256(json.dumps(meta,sort_keys=True,allow_nan=False).encode())
    for array in(breaks,c,jaws):identity.update(array.tobytes())
    meta['reference_id']=identity.hexdigest()
    _check_c2(c,breaks)
    return PreparedReference(_frozen(breaks),_frozen(c),_frozen(jaws),meta)


def _brake_envelope_bounds(coefficients,breaks,duration_s):
    """All-time bounds retaining the correlation of q(t), v(t), and a(t).

    Brake position bases lie in [0,T/2] and [0,T²/12], velocity bases
    in [0,1] and [0,4T/27], and acceleration bases in [-1.5/T,0]
    and [-1/3,1]. Rectangle corners therefore enclose every brake state.
    Each corner is a polynomial in the original path time; find all extrema.
    """
    q=np.asarray(coefficients,dtype=float);powers=np.arange(q.shape[1]-1,-1,-1)
    v=np.zeros_like(q);v[:,1:]=q[:,:-1]*powers[:-1][None,:,None]
    a=np.zeros_like(v);a[:,1:]=v[:,:-1]*powers[:-1][None,:,None]
    t=duration_s;widths=np.diff(breaks)
    def extent(corners):
        sequence=np.concatenate(corners)
        timeline=np.r_[0.,np.cumsum(np.tile(widths,len(corners)))]
        low,high,_,_=_extrema_batched(sequence,timeline)
        return low,high
    low,high=extent([q,q+t*v/2,q+t*t*a/12,q+t*v/2+t*t*a/12])
    vl,vh=extent([np.zeros_like(v),v,4*t*a/27,v+4*t*a/27])
    al,ah=extent([-a/3,a,-1.5*v/t-a/3,-1.5*v/t+a])
    return low,high,np.maximum(abs(vl),abs(vh)),np.maximum(abs(al),abs(ah))


def retime_for_brake_reserve(reference,limits,*,duration_s,velocity_limit,acceleration_limit,maximum_duration_s):
    """Stretch the existing zero-boundary curve to satisfy its normal brake proof."""
    if not isinstance(reference,PreparedReference):raise ValueError('analytic reference required for brake reserve')
    _check_c2(reference.coefficients,reference.breaks)
    _,start_v,start_a=reference.sample([0.])
    if np.any(start_v!=0.)or np.any(start_a!=0.):raise ValueError('reserve retiming requires a confirmed zero-v/a start')
    if any(type(x)not in(int,float)or not math.isfinite(x)or x<=0
           for x in(duration_s,velocity_limit,acceleration_limit,maximum_duration_s)):
        raise ValueError('explicit positive normal brake budgets required')
    bounds=np.asarray(limits,dtype=float)
    if bounds.shape!=(14,2)or not np.isfinite(bounds).all():raise ValueError('finite SDK position limits required')
    lo,hi,v,a=_extrema_batched(reference.coefficients,reference.breaks)
    margin=np.minimum(lo-bounds[list(JOINTS),0],bounds[list(JOINTS),1]-hi)
    def stretch(quadratic,linear,limit):
        limit=np.broadcast_to(limit,v.shape)
        if np.any(limit<0)or np.any((limit==0)&((quadratic!=0)|(linear!=0))):
            return math.inf
        positive=limit>0
        scale=np.ones_like(v)
        scale[positive]=(linear[positive]+np.sqrt(linear[positive]**2+4*limit[positive]*quadratic[positive]))/(2*limit[positive])
        return float(max(scale))
    scale=max(1.,stretch(a*duration_s**2/12,v*duration_s/2,margin),
        stretch((4/27)*a*duration_s,v,velocity_limit),stretch(a,1.5*v/duration_s,acceleration_limit))
    if scale>1.:scale*=1.02  # Same timing margin as the existing reference planner.
    if reference.duration_s*scale>maximum_duration_s:
        maximum_scale=maximum_duration_s/reference.duration_s
        powers=np.arange(reference.coefficients.shape[1]-1,-1,-1)
        def feasible(value):
            c=reference.coefficients/value**powers[None,:,None]
            lo,hi,pv,pa=_brake_envelope_bounds(c,reference.breaks*value,duration_s)
            return (np.all(lo>=bounds[list(JOINTS),0]-1e-10)
                and np.all(hi<=bounds[list(JOINTS),1]+1e-10)
                and max(pv)<=velocity_limit+1e-10 and max(pa)<=acceleration_limit+1e-10)
        if maximum_scale<1. or not feasible(maximum_scale):
            raise ValueError('complete model segment lacks normal brake reserve within execution budget')
        low,high=1.,maximum_scale
        if feasible(low):high=low
        else:
            for _ in range(6):
                middle=(low+high)/2
                if feasible(middle):high=middle
                else:low=middle
        scale=min(high*1.02,maximum_scale)
    if scale==1.:
        check_brake_reserve(reference,limits,duration_s=duration_s,velocity_limit=velocity_limit,acceleration_limit=acceleration_limit)
        return reference
    breaks=reference.breaks*scale;c=reference.coefficients.copy()
    for i,power in enumerate(range(c.shape[1]-1,-1,-1)):c[:,i]/=scale**power
    meta=dict(reference.metadata);parent=meta.pop('reference_id')
    meta.update(parent_reference_id=parent,duration_s=float(breaks[-1]),normal_brake_time_scale=scale,
        max_velocity_rad_s=float(max(v)/scale),max_acceleration_rad_s2=float(max(a)/scale**2),
        max_jaw_velocity_mm_s=meta['max_jaw_velocity_mm_s']/scale,
        normal_brake_reserve_policy=dict(duration_s=duration_s,velocity_limit=velocity_limit,acceleration_limit=acceleration_limit))
    identity=hashlib.sha256(json.dumps(meta,sort_keys=True).encode())
    for array in(breaks,c,reference.jaws):identity.update(array.tobytes())
    meta['reference_id']=identity.hexdigest()
    result=PreparedReference(_frozen(breaks),_frozen(c),reference.jaws,meta)
    check_brake_reserve(result,limits,duration_s=duration_s,velocity_limit=velocity_limit,acceleration_limit=acceleration_limit)
    return result


def check_brake_reserve(reference, limits, *, duration_s, velocity_limit, acceleration_limit):
    """Conservative all-times joint reserve for this analytic brake family.

    Bounds cover every polynomial state, not a sampled collection of stops.
    They are deliberately sufficient, not necessary; near a boundary they may
    reject an inward-moving path. Tracking error, transport lead and geometry
    remain separate Owner obligations.
    """
    if not isinstance(reference,PreparedReference):raise ValueError('prepared analytic reference required')
    _check_c2(reference.coefficients,reference.breaks)
    # Reuse state/bounds validation without changing the reference or its time.
    q,_,_=reference.sample([0.])
    prepare_brake(q[0],np.zeros(14),np.zeros(14),limits,duration_s=duration_s,
        velocity_limit=velocity_limit,acceleration_limit=acceleration_limit,source_sha256=reference.metadata['reference_id'])
    lo,hi,v,a=_extrema(reference.coefficients,reference.breaks)
    # Integrals of velocity bases are bounded by T/2 and T^2/12.
    distance=v*duration_s/2+a*duration_s**2/12
    peak_v=v+(4/27)*a*duration_s
    peak_a=1.5*v/duration_s+a
    bounds=np.asarray(limits,dtype=float)[list(JOINTS)]
    if (np.any(lo-distance<bounds[:,0])or np.any(hi+distance>bounds[:,1])
            or np.any(peak_v>velocity_limit)or np.any(peak_a>acceleration_limit)):
        low,high,peak_v,peak_a=_brake_envelope_bounds(reference.coefficients,reference.breaks,duration_s)
        if (np.any(low<bounds[:,0]-1e-10)or np.any(high>bounds[:,1]+1e-10)
                or np.any(peak_v>velocity_limit+1e-10)or np.any(peak_a>acceleration_limit+1e-10)):
            raise ValueError('reference lacks conservative all-times normal-brake reserve')
        return dict(reference_id=reference.metadata['reference_id'],brake_duration_s=duration_s,
            lower_position_rad=low.tolist(),upper_position_rad=high.tolist(),
            maximum_velocity_rad_s=float(max(peak_v)),maximum_acceleration_rad_s2=float(max(peak_a)),
            scope='analytic_joint_reference_all_times; correlated_brake_envelope; excludes_tracking_latency_geometry_and_jaw_events')
    return dict(reference_id=reference.metadata['reference_id'],brake_duration_s=duration_s,
        lower_position_rad=(lo-distance).tolist(),upper_position_rad=(hi+distance).tolist(),
        maximum_velocity_rad_s=float(max(peak_v)),maximum_acceleration_rad_s2=float(max(peak_a)),
        scope='analytic_joint_reference_all_times; excludes_tracking_latency_geometry_and_jaw_events')


def _prepare_encoded(body,profile=False):
    """Identical bounded CPU calculation for one-shot and warmed child paths."""
    if len(body)>128*1024:raise ValueError('preparation input exceeds bound')
    import time
    started=time.perf_counter();request=json.loads(body);decoded=time.perf_counter();timing={}
    reference=prepare_reference(request['source'],request['initial'],request['limits'],timing=timing,**request['options'])
    encoding=time.perf_counter()
    encoded=json.dumps(dict(breaks=reference.breaks.tolist(),coefficients=reference.coefficients.tolist(),
        jaws=reference.jaws.tolist(),metadata=reference.metadata),allow_nan=False,separators=(',',':')).encode()
    if len(encoded)>256*1024:raise ValueError('preparation output exceeds bound')
    timing.update(worker_json_decode_s=decoded-started,worker_json_encode_s=time.perf_counter()-encoding,
        worker_total_s=time.perf_counter()-started)
    return (encoded,timing)if profile else encoded


def prepare_reference_process(source,initial_sdk_command14,sdk_limits,*,executor=None,timing=None,**options):
    """Keep SciPy imports, spline fitting and whole-path checks outside Owner Python."""
    import os
    import subprocess
    import sys
    import time
    from .recording_process import _environment
    from .inference_owner import sdk_target
    from arx_env.joint_head_real_motion import jaw_peak_scale
    # The isolated CPU child must not fall back to its default jaw calibration.
    if 'sdk_targets14'not in options:options['sdk_targets14']=[sdk_target(row)for row in source['device_targets14']]
    if 'jaw_scales'not in options:options['jaw_scales']={side:jaw_peak_scale(side)for side in ('left','right')}
    request=dict(source=source,initial=initial_sdk_command14,limits=sdk_limits,options=options)
    encoding=time.perf_counter();body=json.dumps(request,allow_nan=False).encode();encoded_at=time.perf_counter()
    if len(body)>128*1024:raise ValueError('preparation request exceeds bounded source size')
    stages={}
    if executor is None:
        result=subprocess.run([os.environ.get('WORKBENCH_CPU_PYTHON')or sys.executable,'-B','-m',__name__,'--prepare'],
            input=body,capture_output=True,env=_environment(),timeout=20)
        if result.returncode or len(result.stdout)>256*1024:
            raise ValueError('isolated CPU motion preparation failed: '+result.stderr.decode(errors='replace')[-1000:])
        encoded=result.stdout
    else:encoded,stages=executor.submit(_prepare_encoded,body,True).result(timeout=20)
    returned=time.perf_counter();value=json.loads(encoded);decoded=time.perf_counter()
    if timing is not None:timing.update(stages,request_json_encode_s=encoded_at-encoding,
        parent_json_decode_s=decoded-returned,roundtrip_s=returned-encoded_at,
        transport_and_wait_s=max(0.,returned-encoded_at-stages.get('worker_total_s',0.)))
    return PreparedReference(_frozen(value['breaks']),_frozen(value['coefficients']),_frozen(value['jaws']),value['metadata'])


if __name__=='__main__':
    import sys
    if sys.argv[1:]!=['--prepare']:raise SystemExit('private CPU preparation worker only')
    raw=sys.stdin.buffer.read(128*1024+1)
    if len(raw)>128*1024:raise ValueError('preparation input exceeds bound')
    sys.stdout.buffer.write(_prepare_encoded(raw))
