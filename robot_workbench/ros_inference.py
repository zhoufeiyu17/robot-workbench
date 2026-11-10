"""Source-reference model requests and results over ROS; no artifact writer.

The model adapter is called once. Input consumers and the recorder reconstruct
images from the same source samples rather than forwarding image payloads.
"""
from collections import OrderedDict
from copy import deepcopy
import hashlib
import json
import threading
import time

import numpy as np

from .ros_data import StreamRef, descriptor_message, json_text, json_value
from .pipeline_timing import timing


def input_transforms(raw_images, prepared_images, *, black_head=False):
    from .inference_model import letterbox_rgb224
    if set(raw_images) != set(prepared_images):
        raise ValueError('model input roles differ from selected source frames')
    if black_head and ('base_0_rgb'not in prepared_images or np.asarray(prepared_images['base_0_rgb']).dtype!=np.uint16):
        raise ValueError('head cover requires the WAM BF16 input contract')
    transforms = {}
    for role, original in raw_images.items():
        covered=black_head and role=='base_0_rgb'
        if covered:original=np.zeros_like(original)
        actual = np.asarray(prepared_images[role])
        if actual.dtype==np.uint16:
            from .wam import image_bits
            if actual.shape!=(256,256,3)or not np.array_equal(image_bits(original),actual):
                raise ValueError('WAM BF16 input differs from its original RGB transform')
            kind='wam_zero_head_bicubic256_wan_bf16'if covered else'wam_bicubic256_wan_bf16'
        elif actual.dtype != np.uint8 or actual.ndim != 3 or actual.shape[2] != 3:
            raise ValueError('declared RGB model input required')
        elif np.array_equal(original, actual): kind='identity_rgb'
        elif np.array_equal(letterbox_rgb224(original), actual): kind='letterbox_rgb224'
        else: raise ValueError('model image preprocessing needs an explicit reconstructable transform')
        transforms[role] = dict(kind=kind, shape=list(actual.shape),
            sha256=hashlib.sha256(actual.tobytes()).hexdigest())
    return transforms


def reconstruct_image(role,frame,rule):
    from .inference_model import letterbox_rgb224
    original=np.frombuffer(frame['rgb'],dtype=np.uint8).reshape(frame['height'],frame['width'],3)
    if rule['kind']=='identity_rgb':actual=original
    elif rule['kind']=='letterbox_rgb224':actual=letterbox_rgb224(original)
    elif rule['kind']in('wam_bicubic256_wan_bf16','wam_zero_head_bicubic256_wan_bf16'):
        from .wam import image_bits
        if rule['kind']=='wam_zero_head_bicubic256_wan_bf16':
            if role!='base_0_rgb':raise ValueError('WAM stall cover is only valid for the head camera')
            original=np.zeros_like(original)
        actual=image_bits(original)
    else:raise ValueError('unknown model input transform')
    if list(actual.shape)!=rule['shape'] or hashlib.sha256(actual.tobytes()).hexdigest()!=rule['sha256']:
        raise ValueError('reconstructed model input differs from the actual input')
    return actual


def reconstruct_input(message, sources, *, timeout=0.):
    """Used by the recorder as well as tests; it never invokes a model."""
    _, frames = sources.resolve_observation(message.observation, timeout=timeout)
    transforms = json_value(message.preprocessing_json); images={}
    if set(transforms) != set(frames): raise ValueError('model input transform roles differ')
    for role, frame in frames.items():
        images[role]=reconstruct_image(role,frame,transforms[role])
    metadata=json_value(message.metadata_json);dtype=np.dtype(metadata.get('model_state_dtype','float64'))
    if dtype.kind!='f' or dtype.itemsize>8:raise ValueError('declared floating model state dtype required')
    state=np.asarray(message.model_state,dtype=dtype)
    if state.shape!=(14,) or not np.isfinite(state).all():raise ValueError('finite model state14 required')
    return state, images


class RosModelWorker:
    """Infer-module subscriber; model implementation is an injected local callable."""
    def __init__(self, ros, sources, prepare, predict, *, binding, clock=time.monotonic_ns, clock_id='host_monotonic',state_to_device=None,
                 stall_recovery=None):
        self.ros,self.sources,self.prepare,self.predict=ros,sources,prepare,predict
        self.binding=deepcopy(binding);self.clock=clock;self.lock=threading.RLock()
        from .inference import action_shape
        self.shape=action_shape(binding.get('action_contract'))
        self.state_to_device=state_to_device
        self.stall_recovery=stall_recovery
        self.current=None;self.pending=None;self.completed=OrderedDict();self.closed=False;self.thread=None
        self.sequences={'model_input':0,'model_result':0};self.clock_id=clock_id
        self.inputs=ros.publisher('model/inputs',ros.types.ModelInput)
        self.results=ros.publisher('model/results',ros.types.ModelResult)
        ros.subscribe('model/requests',ros.types.ModelRequest,self._receive)

    def source_descriptors(self,device_id):
        self.descriptor_device_id=device_id
        rows=[]
        for stream,topic,kind in [('model_input','inputs','ModelInput'),('model_result','results','ModelResult')]:
            row=dict(device_id=device_id,stream_id=stream,source_instance_id=self.binding['session_id'],
                source_clock_id=self.clock_id,topic=self.ros.node.get_namespace().rstrip('/')+'/model/'+topic,
                message_type='workbench_interfaces/msg/'+kind,kind=stream,units='model',coordinate_frame='training',
                nominal_hz=0.,available=not self.closed,metadata={})
            self.ros.publisher('sources/'+stream,self.ros.types.SourceDescriptor,retained=True,depth=1).publish(descriptor_message(self.ros,row))
            rows.append(row)
        return rows

    def bind(self, run_id, epoch):
        if not isinstance(run_id,str) or not run_id or type(epoch)is not int or epoch<0:
            raise ValueError('explicit Run and epoch required')
        with self.lock:self.current=(run_id,epoch)

    def cancel(self):
        with self.lock:self.current=None

    def _error(self, request, reason):
        return self.ros.types.ModelResult(reference=self._reference('model_result'),request_id=request.request_id,run_id=request.run_id,epoch=request.epoch,
                                         rows=0,columns=14,error=str(reason)[:4096])

    def _reference(self,stream):
        with self.lock:
            self.sequences[stream]+=1
            return StreamRef(stream,self.binding['session_id'],self.clock_id,self.clock(),self.sequences[stream]).message()

    def watermarks(self):
        with self.lock:
            if self.pending is not None:raise ValueError('model request still in flight')
            return dict(self.sequences)

    def _receive(self, request):
        received=time.monotonic_ns()
        from rosidl_runtime_py.convert import message_to_ordereddict
        # CDR includes alignment padding, so serializing equal messages need
        # not produce equal bytes. Idempotency follows the declared values.
        fields=message_to_ordereddict(request)
        fields['binding_json']=json_value(request.binding_json)
        fields['observation']['pairing_json']=json_value(request.observation.pairing_json)
        fingerprint=hashlib.sha256(json.dumps(fields,sort_keys=True,ensure_ascii=False,allow_nan=False,
            separators=(',',':')).encode()).hexdigest()
        with self.lock:
            if self.closed:return
            identity=(request.run_id,request.epoch,request.request_id)
            if identity in self.completed:
                original,result=self.completed[identity]
                self.results.publish(result if original==fingerprint else self._error(request,'request identity reused with different input'));return
            if identity==self.pending:
                if self.pending_fingerprint!=fingerprint:self.results.publish(self._error(request,'request identity reused with different input'))
                return
            if self.current!=(request.run_id,request.epoch):
                self.results.publish(self._error(request,'Run or epoch is not bound'));return
            if self.pending is not None:
                self.results.publish(self._error(request,'one model request may be in flight'));return
            if json_value(request.binding_json)!=self.binding or request.session_id!=self.binding['session_id']:
                self.results.publish(self._error(request,'model binding differs'));return
            if (request.observation.run_id,request.observation.epoch)!=identity[:2]:
                self.results.publish(self._error(request,'observation belongs to another Run or epoch'));return
            self.pending=identity;self.pending_fingerprint=fingerprint
            self.thread=threading.Thread(target=self._run,args=(deepcopy(request),identity,fingerprint,received),name='ros-model-call',daemon=True)
            self.thread.start()

    def _run(self, request, identity, fingerprint, received=None):
        began=time.monotonic_ns();ids=dict(request_id=request.request_id,run_id=request.run_id,
            observation_id=request.observation.observation_id,epoch=request.epoch)
        timing.record('model.dispatch',received or began,ended_ns=began,**ids)
        result=None
        try:
            if request.valid_until_source_time_ns<=self.clock():raise ValueError('model request observation expired')
            feedback,frames=self.sources.resolve_observation(request.observation,
                timeout=max(0.,(request.valid_until_source_time_ns-self.clock())/1e9))
            resolved=time.monotonic_ns();timing.record('model.resolve_sources',began,ended_ns=resolved,**ids)
            from .ros_inputs import referenced_observation
            observation=referenced_observation(request.observation,self.sources.sources,feedback,
                frames=frames,state_to_device=self.state_to_device)
            images=observation['images']
            decision=self.stall_recovery(request)if self.stall_recovery is not None else None
            seed=request.seed
            if decision is not None:
                seed=decision['actual_seed']
                if decision['head_black']:
                    observation=dict(observation,images=dict(images,base_0_rgb=np.zeros_like(images['base_0_rgb'])))
            started=self.clock();state,prepared_images=self.prepare(observation)
            state=np.asarray(state)
            if state.shape!=(14,) or state.dtype.kind!='f' or not np.isfinite(state).all():raise ValueError('model adapter returned invalid state14')
            transforms=input_transforms(images,prepared_images,black_head=bool(decision and decision['head_black']))
            transformed=time.monotonic_ns();timing.record('model.preprocess_and_proof',resolved,ended_ns=transformed,**ids)
            self.inputs.publish(self.ros.types.ModelInput(reference=self._reference('model_input'),request_id=request.request_id,run_id=request.run_id,epoch=request.epoch,
                observation=request.observation,model_state=state.tolist(),preprocessing_json=json_text(transforms),
                metadata_json=json_text(dict(binding=self.binding,seed=seed,task=request.task,model_state_dtype=str(state.dtype),
                    **(dict(stall_recovery=decision)if decision is not None else{}),
                    applied_history_model14=np.asarray(request.applied_history).reshape(-1,14).tolist()))))
            with self.lock:
                if self.current!=identity[:2]:raise ValueError('model request revoked before inference')
            if request.valid_until_source_time_ns<=self.clock():raise ValueError('observation expired before inference')
            call_started=time.monotonic_ns()
            value=self.predict(dict(request_id=request.request_id,seed=seed,task=request.task,
                run_id=request.run_id,epoch=request.epoch,clock_domain=request.observation.state.source_clock_id,
                valid_until_source_time=request.valid_until_source_time_ns/1e9,
                observation=observation,applied_history=np.asarray(request.applied_history).reshape(-1,14)),
                (state,prepared_images))
            call_ended=time.monotonic_ns();timing.record('model.request_roundtrip',call_started,ended_ns=call_ended,**ids)
            reported=value.get('audit')or{}
            points=reported.get('timing_points')if isinstance(reported,dict)else None
            if isinstance(points,dict)and points.get('clock')=='host_monotonic_ns'and points.get('boot_id')==timing.boot_id and timing.boot_id:
                for name,left,right in [('host_prepare','host_start_ns','engine_start_ns'),('engine','engine_start_ns','engine_end_ns'),
                        ('output_to_host','engine_end_ns','host_output_ns'),('denormalize','host_output_ns','decoded_ns')]:
                    timing.record('model.worker.'+name,points.get(left),ended_ns=points.get(right),worker_pid=points.get('pid'),**ids)
            durations=reported.get('timing')if isinstance(reported,dict)else None
            for name,duration in (durations.items()if isinstance(durations,dict)else()):
                if isinstance(name,str)and name.endswith('_s')and type(duration)in(int,float)and np.isfinite(duration)and duration>=0:
                    timing.record('model.reported.'+name,call_ended-round(duration*1e9),ended_ns=call_ended,
                        timestamp_basis='reported_duration_anchored_at_reply; not_exact_stage_boundaries',**ids)
            action=np.asarray(value['action14'])
            audit=dict(value.get('audit')or{},ros_model_call_s=(self.clock()-started)/1e9)
            if decision is not None:audit['stall_recovery']=decision
            envelope={k:v for k,v in value.items()if k not in('action14','audit')}
            if action.ndim!=2 or action.size>840:raise ValueError('model output exceeds the declared action bound')
            result=self.ros.types.ModelResult(reference=self._reference('model_result'),request_id=request.request_id,run_id=request.run_id,epoch=request.epoch,
                actions=action.astype(np.float64).reshape(-1).tolist(),rows=action.shape[0],columns=action.shape[1],
                envelope_json=json_text(envelope),audit_json=json_text(audit),error='')
            if action.shape!=self.shape or not np.isfinite(action).all():raise ValueError('model output differs from the bound finite action shape')
            if self.clock()>=request.valid_until_source_time_ns:result.error='model result observation expired'
            with self.lock:
                if self.current!=identity[:2]:result.error='model result revoked'
        except Exception as error:
            if result is None:result=self._error(request,error)
            else:result.error=str(error)[:4096]
        finally:
            with self.lock:
                self.completed[identity]=(fingerprint,result)
                while len(self.completed)>256:self.completed.popitem(last=False)
                self.pending=None
            published=time.monotonic_ns();self.results.publish(result)
            timing.record('model.result_publish',published,**ids)
            timing.record('model.total',began,outcome='error'if result.error else'ok',reason=result.error[:256]or None,**ids)

    def close(self):
        with self.lock:self.closed=True;self.current=None
        if self.thread is not None:self.thread.join(5.)
        if self.thread is not None and self.thread.is_alive():raise RuntimeError('model call still in flight')
        if hasattr(self,'descriptor_device_id'):self.source_descriptors(self.descriptor_device_id)


class RosModelClient:
    def __init__(self, ros, *, binding, clock=time.monotonic_ns):
        self.ros,self.binding,self.clock=ros,deepcopy(binding),clock
        from .inference import action_shape
        self.shape=action_shape(binding.get('action_contract'))
        self.requests=ros.publisher('model/requests',ros.types.ModelRequest)
        self.changed=threading.Condition();self.pending={};self.current=None
        ros.subscribe('model/results',ros.types.ModelResult,self._receive)

    def bind(self,run_id,epoch):
        with self.changed:self.current=(run_id,epoch)

    def _receive(self,result):
        key=(result.run_id,result.epoch,result.request_id)
        with self.changed:
            if key in self.pending:self.pending[key]=result;self.changed.notify_all()

    def predict(self, request_id, observation, *, seed, task, history, valid_until_ns):
        with self.changed:
            if self.current is None:raise ValueError('model client is not bound to a Run')
            key=(*self.current,request_id)
            if self.pending:raise ValueError('one model request may be in flight')
            if valid_until_ns<=self.clock():raise ValueError('model request is already expired')
            self.pending[key]=None
            request=self.ros.types.ModelRequest(request_id=request_id,run_id=key[0],epoch=key[1],
                session_id=self.binding['session_id'],observation=observation,valid_until_source_time_ns=valid_until_ns,
                seed=seed,task=task,applied_history=np.asarray(history,dtype=np.float64).reshape(-1).tolist(),
                binding_json=json_text(self.binding))
            self.requests.publish(request)
            try:
                while self.pending[key] is None:
                    if self.current!=key[:2]:raise ValueError('model request revoked')
                    remaining=(valid_until_ns-self.clock())/1e9
                    if remaining<=0:raise TimeoutError('model result missed the observation deadline')
                    self.changed.wait(remaining)
                result=self.pending[key]
                if self.current!=key[:2] or self.clock()>=valid_until_ns:raise ValueError('late or revoked model result')
                if result.error:raise ValueError(result.error)
                if (result.rows,result.columns,len(result.actions))!=(*self.shape,self.shape[0]*self.shape[1]):raise ValueError('model result shape differs')
                action=np.asarray(result.actions,dtype=np.float64).reshape(self.shape)
                if not np.isfinite(action).all():raise ValueError('nonfinite model result')
                return dict(json_value(result.envelope_json),action14=action,audit=json_value(result.audit_json))
            finally:self.pending.pop(key,None)

    def cancel(self):
        with self.changed:self.current=None;self.changed.notify_all()
