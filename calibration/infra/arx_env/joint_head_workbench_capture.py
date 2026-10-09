"""Read-only, all-or-nothing raw-camera/stationary-feedback capture transaction."""
import hashlib
from functools import lru_cache
import json
import time
import numpy as np
from robot_workbench.sample_clock import assert_progress, bracket_samples, clock_spec, sample_stamp


class CaptureError(ValueError):
    def __init__(self, message, evidence):
        super().__init__(message)
        self.evidence = evidence


def rectify_corners(uv, camera):
    """Same librealsense inverse-Brown equations as joint_head_real_fit.rectify."""
    uv=np.asarray(uv,float)
    coefficients=np.asarray(camera['coefficients'],float)
    if not np.any(coefficients):return uv.copy()
    if camera['distortion_model'].split('.')[-1]!='inverse_brown_conrady':
        raise ValueError('only verified inverse_brown_conrady supported')
    original=(uv-[camera['cx'],camera['cy']])/[camera['fx'],camera['fy']]
    x,y=original[:,0].copy(),original[:,1].copy()
    k1,k2,p1,p2,k3=coefficients
    for _ in range(10):
        r2=x*x+y*y;ir=1/(1+((k3*r2+k2)*r2+k1)*r2)
        xq,yq=x/ir,y/ir
        dx=2*p1*xq*yq+p2*(r2+2*xq*xq);dy=2*p2*xq*yq+p1*(r2+2*yq*yq)
        x,y=(original[:,0]-dx)*ir,(original[:,1]-dy)*ir
    return np.c_[x,y]*[camera['fx'],camera['fy']]+[camera['cx'],camera['cy']]


@lru_cache(maxsize=8)
def _rectification_map(serialized_camera):
    camera=json.loads(serialized_camera)
    yy,xx=np.indices((camera['height'],camera['width']),dtype=float)
    target=np.c_[xx.ravel(),yy.ravel()];raw=target.copy()
    for _ in range(20):
        error=target-rectify_corners(raw,camera)
        raw+=error
        if np.max(abs(error))<1e-4:break
    if not np.isfinite(raw).all() or np.max(abs(rectify_corners(raw,camera)-target))>.01:
        raise ValueError('inverse-Brown image remap failed convergence')
    return raw.reshape(camera['height'],camera['width'],2).astype(np.float32)


def _owner_evidence(owner):
    result={key:owner.get(key) for key in ('pid','command_epoch','state','reason','feedback_healthy','environment','mode')}
    diagnostic=owner.get('last_fault_diagnostic') or {}
    result['fault_reasons']=diagnostic.get('reasons',[])
    feedback=owner.get('calibration_feedback') or {}
    result.update({key:feedback.get(key) for key in ('sample_wall_ns','clock_domain','source_clock_id','source_time_ns','physics_step')})
    return result


def _bracket(history, stamp, pid, epoch, *, clock=None, physics_step=None):
    clock=clock or clock_spec()
    simulated=clock['environment']=='fake'
    samples=[];previous=None
    for row in history:
        owner=row.get('owner',{});feedback=owner.get('calibration_feedback',{})
        wall=feedback.get('sample_wall_ns')
        if simulated:
            if owner.get('environment')!='fake' or owner.get('mode')!='simulation' or owner.get('simulation_only') is not True:
                raise ValueError('feedback belongs to another environment or is not explicit fake')
            point=sample_stamp(feedback,clock,wall_key='sample_wall_ns')
            previous=assert_progress(previous,point,allow_same=True)
            samples.append((point[0],owner))
        elif type(wall) is int:
            sample_stamp(feedback,clock,wall_key='sample_wall_ns')
            if owner.get('environment') not in (None,'physical') or owner.get('simulation_only') is True:
                raise ValueError('fake owner cannot authorize physical capture')
            samples.append((wall,owner))
    if simulated:
        bracket_samples([(point,owner['calibration_feedback']['physics_step']) for point,owner in samples],
                        (stamp,physics_step))
    # A simulator image may share an exact step with feedback. Still require
    # distinct surrounding samples, with advancing SDK clocks, for interpolation.
    before=[row for row in samples if row[0]<stamp] if simulated else [row for row in samples if row[0]<=stamp]
    after=[row for row in samples if row[0]>stamp] if simulated else [row for row in samples if row[0]>=stamp]
    if not before or not after:raise ValueError('feedback bracket missing; retry whole capture')
    lo=max(before,key=lambda row:row[0]);hi=min(after,key=lambda row:row[0])
    if hi[0]<=lo[0] or hi[0]-lo[0]>100_000_000:raise ValueError('feedback bracket must span 0..0.1s')
    if simulated and (type(physics_step) is not int or not lo[1]['calibration_feedback']['physics_step']<=physics_step<=hi[1]['calibration_feedback']['physics_step']):
        raise ValueError('image physics_step outside feedback bracket')
    selected=sorted([row for row in samples if lo[0]<=row[0]<=hi[0]],key=lambda row:row[0])
    positions=[];sdk=[]
    for _,owner in selected:
        reasons=[]
        if owner.get('pid')!=pid:reasons.append('owner PID changed')
        if owner.get('command_epoch')!=epoch:reasons.append('command epoch changed')
        if owner.get('feedback_healthy') is not True:reasons.append('feedback unhealthy')
        if reasons:
            actual=_owner_evidence(owner)
            message=('%s; PID expected=%s actual=%s; epoch expected=%s actual=%s; state=%s; reason=%s; fault criteria=%s'
                     %(', '.join(reasons),pid,actual['pid'],epoch,actual['command_epoch'],actual['state'],actual['reason'],actual['fault_reasons']))
            raise CaptureError(message,{'expected_owner':{'pid':pid,'command_epoch':epoch},'failed_owner':actual})
        feedback=owner['calibration_feedback'];q=np.asarray(feedback.get('qpos'),float);s=np.asarray(feedback.get('sdk_timestamps'),float)
        if q.shape!=(14,) or s.shape!=(2,) or not np.isfinite(q).all() or not np.isfinite(s).all() or np.any(s<0):
            raise ValueError('invalid raw14 feedback or SDK clocks')
        positions.append(q);sdk.append(s)
    positions=np.asarray(positions);sdk=np.asarray(sdk)
    if np.any(np.diff(sdk,axis=0)<0) or not np.all(sdk[-1]>sdk[0]):raise ValueError('both SDK clocks must advance without regression')
    indices=[*range(6),*range(7,13)]
    span=np.ptp(positions[:,indices],axis=0)
    if np.max(span)>.005:raise ValueError('feedback not stationary: joint span exceeds .005rad')
    fraction=(stamp-lo[0])/(hi[0]-lo[0]);q=positions[0]+fraction*(positions[-1]-positions[0])
    return dict(qpos=q.tolist(),qpos14=q.tolist(),q_reported12=q[indices].tolist(),
                bracket_gap_s=(hi[0]-lo[0])/1e9,pose_span_rad=span.tolist(),
                feedback_bracket=dict(before_ns=lo[0],after_ns=hi[0],owner_pid=pid,command_epoch=epoch,clock=clock,
                                      physics_step_before=lo[1]['calibration_feedback'].get('physics_step'),
                                      physics_step_after=hi[1]['calibration_feedback'].get('physics_step'),
                                      sdk_before=sdk[0].tolist(),sdk_after=sdk[-1].tolist(),qpos_before=positions[0].tolist(),qpos_after=positions[-1].tolist(),interpolation_fraction=fraction))


def capture_sample(cameras, intrinsics, feedback_history, *, owner_pid, command_epoch, required_markers=None, feedback_wait_s=.15, not_before_ns=None, feedback_diagnostics=None, environment='physical', source_clock_id=None, not_before_source_ns=None):
    """Bounded future-feedback wait; no motion or files and no frame rereads.

    Return rows per camera, rectified PNG images, original raw_images and cameras.
    Failure retains completed rows and bytes in CaptureError.evidence.
    """
    import cv2
    evidence={'rows':{},'images':{},'raw_images':{},'cameras':{},'camera_metadata':{}}
    try:
        clock=clock_spec(environment,source_clock_id=source_clock_id)
        simulated=environment=='fake'
        if simulated:
            if not_before_ns is not None or type(not_before_source_ns) is not int or not_before_source_ns<0:
                raise ValueError('fake capture requires nonnegative source readiness, never wall readiness')
        elif not_before_source_ns is not None:
            raise ValueError('physical capture requires wall readiness, not simulation time')
        evidence['clock']=clock
        if type(owner_pid) is not int or owner_pid<=0 or type(command_epoch) is not int or command_epoch<0:
            raise ValueError('explicit owner PID and epoch required')
        if not_before_ns is not None and (type(not_before_ns) is not int or not_before_ns<=0):
            raise ValueError('positive capture readiness timestamp required')
        if not cameras or intrinsics is not None and set(cameras)!=set(intrinsics):raise ValueError('intrinsics must bind every required camera')
        if not isinstance(feedback_wait_s,(int,float)) or not 0<=feedback_wait_s<=.15:
            raise ValueError('feedback wait must be within 0..0.15s')
        required_markers=required_markers or {}
        if set(required_markers)-set(cameras):raise ValueError('marker requirements reference missing cameras')
        dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters=(cv2.aruco.DetectorParameters() if hasattr(cv2.aruco,'ArucoDetector')
                    else cv2.aruco.DetectorParameters_create())
        parameters.adaptiveThreshWinSizeMin=3
        parameters.adaptiveThreshWinSizeMax=63
        parameters.adaptiveThreshWinSizeStep=4
        if hasattr(cv2.aruco,'ArucoDetector'):
            detect=cv2.aruco.ArucoDetector(dictionary,parameters).detectMarkers
        else:
            detect=lambda image:cv2.aruco.detectMarkers(image,dictionary,parameters=parameters)
        captured={};frame_stamps={}
        for name,adapter in cameras.items():
            evidence['failed_camera']=name
            raw,meta=adapter.read_calibration_frame();evidence['raw_images'][name]=raw
            captured[name]=(raw,meta)
            evidence['camera_metadata'][name]=meta
            wall=meta.get('host_receive_ns')
            if isinstance(wall,str) and wall.isdecimal():wall=int(wall)
            if simulated and meta.get('timestamp_kind')!='simulation_step':
                raise ValueError('fake RGB frame requires simulation_step timestamp')
            if not simulated and meta.get('timestamp_kind')=='simulation_step':
                raise ValueError('simulation frame cannot authorize physical capture')
            frame_stamps[name]=sample_stamp({**meta,'host_receive_ns':wall},clock)
        deadline=time.monotonic()+feedback_wait_s
        while True:
            history=list(feedback_history() if callable(feedback_history) else feedback_history)
            if history:evidence['latest_owner']=_owner_evidence(history[-1].get('owner',{}))
            stamps=[point[0] for point in frame_stamps.values()]
            latest=max((sample_stamp(row.get('owner',{}).get('calibration_feedback',{}),clock,wall_key='sample_wall_ns')[0] for row in history),default=-1)
            if latest>max(stamps) or not callable(feedback_history) or time.monotonic()>=deadline:break
            time.sleep(min(.01,max(0,deadline-time.monotonic())))
        walls=[sample_stamp(row.get('owner',{}).get('calibration_feedback',{}),clock,wall_key='sample_wall_ns')[0] for row in history]
        evidence['feedback_history']={'count':len(walls),'oldest_ns':min(walls) if walls else None,
                                      'newest_ns':max(walls) if walls else None,'checked_at_ns':time.time_ns(),'clock':clock}
        for name,(raw,meta) in captured.items():
            evidence['failed_camera']=name
            if meta.get('raw') is not True or meta.get('annotated') is not False or meta.get('image_domain')!='raw' or meta.get('metadata_matched') is not True or meta.get('ready') is not True:
                raise ValueError('atomic unannotated raw frame required')
            source=meta.get('intrinsics',{});k=intrinsics[name] if intrinsics is not None else source
            if not k.get('serial') or meta.get('serial')!=k['serial'] or source.get('serial')!=k['serial']:
                raise ValueError('camera serial binding mismatch')
            for key in ('width','height','fx','fy','cx','cy','coefficients','distortion_model','rectified'):
                if key not in k or source.get(key)!=k[key]:raise ValueError('camera intrinsics binding mismatch: '+key)
            values=np.asarray([k['fx'],k['fy'],k['cx'],k['cy'],*k['coefficients']],float)
            if len(k['coefficients'])!=5 or not np.isfinite(values).all() or min(k['fx'],k['fy'])<=0 or k['rectified'] is not False:
                raise ValueError('finite raw intrinsics required')
            stamp=meta.get('host_receive_ns')
            if isinstance(stamp,str) and stamp.isdecimal():stamp=int(stamp)
            if type(stamp) is not int or stamp<=0 or type(meta.get('frame_number')) is not int:
                raise ValueError('atomic frame timestamp/number required')
            if not_before_ns is not None and stamp<not_before_ns:
                raise ValueError('camera frame predates capture readiness')
            matching_stamp,physics_step=frame_stamps[name]
            if simulated and matching_stamp<not_before_source_ns:
                raise ValueError('camera frame predates source capture readiness')
            image=cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_COLOR)
            if image is None or image.shape[:2]!=(k['height'],k['width']):raise ValueError('invalid raw image or dimensions')
            paired=_bracket(history,matching_stamp,owner_pid,command_epoch,clock=clock,physics_step=physics_step)
            corners,ids,_=detect(image);detections={}
            for marker,uv in zip([] if ids is None else ids.ravel(),corners):
                if int(marker) not in (24,30):continue
                if str(int(marker)) in detections:raise ValueError('duplicate marker identity')
                detections[str(int(marker))]=np.asarray(uv).reshape(4,2).tolist()
            expected=required_markers.get(name,[])
            if any(marker not in (24,30) for marker in expected):raise ValueError('only marker24/30 supported')
            missing=[marker for marker in expected if str(marker) not in detections]
            corrected={marker:rectify_corners(uv,k).tolist() for marker,uv in detections.items()}
            rectified=cv2.remap(image,_rectification_map(json.dumps(k,sort_keys=True)),None,cv2.INTER_LINEAR)
            ok,png=cv2.imencode('.png',rectified)
            if not ok:raise ValueError('rectified PNG encoding failed')
            evidence['images'][name]=png.tobytes()
            evidence['cameras'][name]={**k,'rectified':True,'coefficients':[0.]*5,'distortion_model':'distortion.inverse_brown_conrady'}
            evidence['rows'][name]=dict(**paired,detections=detections,detections_rectified=corrected,
                expected_markers=list(expected),missing_markers=missing,
                image_sha256=hashlib.sha256(raw).hexdigest(),image_domain='raw',metadata=meta,
                host_receive_ns=stamp,frame_number=meta['frame_number'],camera=k,
                runtime_environment=environment,clock=clock,
                source_time_ns=matching_stamp if simulated else None,physics_step=physics_step,
                rectified_camera={**k,'rectified':True,'coefficients':[0.]*5},
                synchronized=False,pairing='stationary simulation-step bracket' if simulated else 'stationary host-receive bracket; not exposure synchronization')
        evidence.pop('failed_camera',None)
        evidence['row']={'camera_detections':{name:row['detections_rectified'] for name,row in evidence['rows'].items()},
                         'camera_feedback':{name:{key:row[key] for key in ('qpos14','q_reported12','feedback_bracket','bracket_gap_s','pose_span_rad')} for name,row in evidence['rows'].items()},
                         'owner_pid':owner_pid,'command_epoch':command_epoch,'synchronized':False,
                         'runtime_environment':environment,'clock':clock,'simulation_only':simulated}
        return evidence
    except (ValueError,TypeError,KeyError,OSError,cv2.error) as exc:
        if isinstance(exc,CaptureError):evidence.update(exc.evidence)
        if feedback_diagnostics is not None:evidence['feedback_reader']=feedback_diagnostics()
        evidence['error']=str(exc)
        raise CaptureError(str(exc),evidence) from exc
