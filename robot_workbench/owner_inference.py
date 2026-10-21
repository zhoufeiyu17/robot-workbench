"""Declared inference bridge: metadata/results/events, never images or closures."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import queue
import secrets
import threading
import time
import weakref

import numpy as np


def plain(value):
    if isinstance(value,np.ndarray):return value.tolist()
    if isinstance(value,dict):return {key:plain(part)for key,part in value.items()}
    if isinstance(value,(tuple,list)):return [plain(part)for part in value]
    return value


def reference_wire(reference):
    if reference is None:return None
    return dict(breaks=reference.breaks.tolist(),coefficients=reference.coefficients.tolist(),
        jaws=reference.jaws.tolist(),metadata=deepcopy(reference.metadata))


def reference_read(value,limits):
    if value is None:return None
    from .motion_reference import PreparedReference,_frozen,_extrema,JOINTS
    if set(value)!={'breaks','coefficients','jaws','metadata'}:raise ValueError('declared prepared reference fields required')
    b,c,j=map(lambda key:np.asarray(value[key],dtype=np.float64),('breaks','coefficients','jaws'))
    m=deepcopy(value['metadata']);n=len(b)
    if (not 2<=n<=61 or b.ndim!=1 or c.shape!=(n-1,4,12)or j.shape!=(n,2)
            or not all(np.isfinite(a).all()for a in(b,c,j))or b[0]!=0. or not 0<b[-1]<=30.
            or np.any(np.diff(b)<=0)or m.get('duration_s')!=b[-1]):
        raise ValueError('bounded continuous reference required')
    identity=m.pop('reference_id',None)
    sha=hashlib.sha256(json.dumps(m,sort_keys=True).encode())
    for a in(b,c,j):sha.update(a.tobytes())
    if sha.hexdigest()!=identity:raise ValueError('prepared reference content digest differs')
    lo,hi,v,a=_extrema(c,b);lim=np.asarray(limits)
    if (np.any(lo<lim[list(JOINTS),0]-1e-9)or np.any(hi>lim[list(JOINTS),1]+1e-9)
            or np.max(v)>.25+1e-9 or np.max(a)>1.+1e-9
            or np.any(j<lim[[6,13],0])or np.any(j>lim[[6,13],1])):
        raise ValueError('prepared reference fails command/dynamic bounds')
    m['reference_id']=identity
    return PreparedReference(_frozen(b),_frozen(c),_frozen(j),m)


class RecordingFacts:
    """Only recording lifecycle facts and Stop notification live in Owner."""
    def __init__(self,bridge):
        self.bridge=weakref.ref(bridge);self.ready=threading.Event();self.done=threading.Event()
        self.prepared=self.ready;self.error=None;self.controller_result=None;self.root=None;self.manifest=None
        self.thread=self;self.alive=True
    def is_alive(self):return self.alive
    def finish(self,status,reason=None):
        bridge=self.bridge()
        if bridge:bridge.emit('recording_stop',status=status,reason=reason)
    def snapshot(self):return dict(ready=self.ready.is_set(),done=self.done.is_set(),error=self.error)


class OwnerInference:
    def __init__(self,service,configuration,specification):
        if set(specification)!={'profile','reference','preloaded_prediction','feedforward'}:
            raise ValueError('explicit inference dependencies required')
        if specification['feedforward']is not None:
            raise ValueError('remote frozen feedforward is not qualified for this controller profile')
        from .inference_workflow import calibrated_adapter
        inputs=service.device.runtime.control_inputs()
        self.adapter,mapping=calibrated_adapter(specification['profile'],inputs['joint_limits'],
            model_id=configuration['model_id'],disabled_grippers=configuration['disabled_grippers'])
        if mapping!=configuration['conversion_identity']['mapping']:raise ValueError('calibrated conversion changed')
        self.reference=reference_read(specification['reference'],inputs['joint_limits'])
        self.camera_bindings={role:(configuration.get('camera_capture_instances',{}).get(role),configuration.get('camera_serials',{}).get(role))
            for role in('head','left','right')}if configuration.get('camera_capture_instances')else {}
        if service.device.environment=='physical'and(not self.camera_bindings or any(not all(pair)for pair in self.camera_bindings.values())):
            raise ValueError('physical inference requires all three frozen camera identities')
        self.preloaded=deepcopy(specification['preloaded_prediction'])
        self.service=service;self.session_id=secrets.token_hex(16);self.runner=None
        self.lock=threading.Lock();self.observation=None;self.support=None;self.pending={}
        self.recording_ready=False;self.recording_error=None

    def emit(self,kind,**value):
        self.service.event('owner_inference',session_id=self.session_id,kind=kind,**plain(value))

    def factory(self,owner,proxy,feedback,configuration):
        from .inference_owner import make_inference_runner
        bridge=weakref.ref(self)
        def borrow():
            value=bridge()
            if value is None:raise ValueError('inference configuration retired')
            return value
        def observe(now,fb):
            value=borrow()
            with value.lock:observation=deepcopy(value.observation)
            if observation is None:return None
            if not 0<=now-observation['source_time']<=.5:return None
            if not 0<=now-observation.pop('feedback_sample_time_s')<=.1:return None
            return observation
        def predict(request):return borrow().predict(request)
        def support(now,fb):
            value=borrow()
            with value.lock:return deepcopy(value.support)
        def event(name,value):borrow().emit('metric',name=name,value=value)
        def record(name,value):borrow().emit('record',name=name,value=value)
        def prepared():
            value=borrow()
            if value.recording_error:raise ValueError(value.recording_error)
            return value.recording_ready
        runner=make_inference_runner(owner,proxy,feedback,configuration,predict=predict,observe=observe,
            support=support,adapter=self.adapter,event=event,
            record=record if configuration['recording_enabled']else None,
            record_prepared=prepared if configuration['recording_enabled']else None,
            prepared_reference=self.reference,preloaded_prediction=self.preloaded)
        if configuration['recording_enabled']:
            previous=owner.recording
            if previous is not None and (not previous.done.is_set()or previous.thread.is_alive()):
                raise ValueError('previous recording is still sealing')
            owner.recording=RecordingFacts(self)
        # Business acknowledgement gates recording completion, not Owner motion.
        self.runner=weakref.ref(runner)
        self.emit('created',execution_instance_id=owner.execution_instance_id,configuration=configuration)
        return runner

    def predict(self,request):
        request=plain(request);identity=request['request_id'];result=queue.Queue(maxsize=1)
        with self.lock:
            if self.pending:raise ValueError('only one prediction may be in flight')
            self.pending[identity]=result
        self.emit('query',request=request)
        try:
            deadline=time.monotonic()+2.
            while time.monotonic()<deadline:
                runner=self.runner()if self.runner else None
                if runner is None or runner.status in('STOPPED','FAULT','READY'):raise ValueError('prediction revoked')
                try:
                    value=result.get(timeout=.05)
                    if value.get('error'):raise ValueError(value['error'])
                    return value['response']
                except queue.Empty:pass
            raise ValueError('business prediction supply timed out')
        finally:
            with self.lock:self.pending.pop(identity,None)

    def update(self,operation,args):
        if operation=='inference_observation':
            value=args['observation'];runtime=self.service.device.runtime
            if args['command_epoch']!=runtime.owner.epoch:raise ValueError('observation belongs to revoked control epoch')
            if (not isinstance(value,dict)or set(value)!={'source_time','device_state14','frame_ids','feedback_sample_time_s'}
                    or not isinstance(value['frame_ids'],dict)):
                raise ValueError('observation metadata only; pixels forbidden')
            from .inference import vector14
            vector14(value['device_state14'])
            names=dict(head='base_0_rgb',left='left_wrist_0_rgb',right='right_wrist_0_rgb')
            for role,(capture,serial)in self.camera_bindings.items():
                row=value['frame_ids'].get(names[role],{})
                if row.get('capture_instance_id')!=capture or row.get('serial')!=serial:
                    raise ValueError('observation camera identity differs from the frozen inference review')
            now=runtime.owner.control_time()
            if not 0<=now-value['source_time']<=.5 or not 0<=now-value['feedback_sample_time_s']<=.1:
                raise ValueError('fresh actual observation and feedback required')
            with self.lock:self.observation=deepcopy(value);self.support=deepcopy(args.get('support'))
        elif operation=='inference_result':
            response=args.get('response');identity=args.get('request_id')
            if response is not None:
                rows=np.asarray(response.get('action14'),dtype=float)
                if rows.shape!=(60,14)or not np.isfinite(rows).all()or response.get('request_id')!=identity:
                    raise ValueError('finite response for original prediction request required')
            with self.lock:
                result=self.pending.get(identity)
                if result is None:raise ValueError('late or revoked prediction rejected')
                try:result.put_nowait(dict(response=response,error=args.get('error')))
                except queue.Full:raise ValueError('prediction result already submitted')
        elif operation=='inference_recording':
            if type(args.get('prepared'))is not bool:raise ValueError('explicit recording readiness required')
            self.recording_ready=args['prepared'];self.recording_error=args.get('error')
            runtime=self.service.device.runtime;recording=runtime.owner.recording
            if not isinstance(recording,RecordingFacts)or recording.bridge()is not self:raise ValueError('recording lease changed')
            recording.error=self.recording_error
            if args['prepared']:recording.ready.set()
            if args.get('done')is True:recording.done.set()
            recording.alive=args.get('alive',True)
        return dict(accepted=True)


class RemoteRunner:
    is_inference=True
    def __init__(self,client,identity):self.client=client;self.identity=identity;self.value={}
    @property
    def status(self):return self.value.get('runner_status','ARMED')
    @property
    def recording_pending(self):return self.value.get('recording_pending',False)
    def snapshot(self):return deepcopy(self.value)
