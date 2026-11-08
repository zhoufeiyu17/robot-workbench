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
    if isinstance(value,np.generic):return plain(value.item())
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
    from .motion_reference import PreparedReference,_frozen,_extrema,JOINTS,SCHEMA
    if not isinstance(value,dict)or set(value)!={'breaks','coefficients','jaws','metadata'}:raise ValueError('declared prepared reference fields required')
    b,c,j=map(lambda key:np.asarray(value[key],dtype=np.float64),('breaks','coefficients','jaws'))
    m=deepcopy(value['metadata']);n=b.size
    if not isinstance(m,dict):raise ValueError('prepared reference metadata object required')
    shapes={(n-1,4,12)}
    if m.get('kind')in('normal_brake','prefix_then_normal_brake'):shapes.add((n-1,5,12))
    if m.get('kind')in('online_prediction_chunk','prefix_then_normal_brake'):shapes.add((n-1,6,12))
    if (not 2<=n<=(63 if m.get('kind')=='prefix_then_normal_brake'else 61) or b.ndim!=1 or c.shape not in shapes or j.shape!=(n,2)
            or not all(np.isfinite(a).all()for a in(b,c,j))or b[0]!=0. or not 0<b[-1]<=30.
            or np.any(np.diff(b)<=0)or m.get('duration_s')!=b[-1]or m.get('schema')!=SCHEMA):
        raise ValueError('bounded continuous reference required')
    identity=m.pop('reference_id',None)
    sha=hashlib.sha256(json.dumps(m,sort_keys=True).encode())
    for a in(b,c,j):sha.update(a.tobytes())
    if sha.hexdigest()!=identity:raise ValueError('prepared reference content digest differs')
    lo,hi,v,a=_extrema(c,b);lim=np.asarray(limits)
    if lim.shape!=(14,2)or not np.isfinite(lim).all()or np.any(lim[:,0]>lim[:,1]):
        raise ValueError('finite ordered SDK limits required')
    from .execution_settings import Guard,control_parameters,defaults,policy_hash
    parameters=control_parameters(defaults('inference'),business='inference')
    guard=Guard.bind(dict(revision=None,policy_hash=policy_hash(parameters),parameters=parameters),{})
    now=time.monotonic()
    dynamic=guard.evaluate(dict(rule_id='G31',signal_id=identity,
        observed=dict(velocity_limit_rad_s=float(np.max(v)),acceleration_limit_rad_s2=float(np.max(a))),
        source_time=now,checked_time=now),'legacy_reference')
    if (np.any(lo<lim[list(JOINTS),0]-1e-9)or np.any(hi>lim[list(JOINTS),1]+1e-9)
            or dynamic['verdict']=='reject' or np.any(j<lim[[6,13],0])or np.any(j>lim[[6,13],1])):
        raise ValueError('prepared reference fails command/dynamic bounds')
    m['reference_id']=identity
    return PreparedReference(_frozen(b),_frozen(c),_frozen(j),m)


class RecordingFacts:
    """Only recording lifecycle facts and Stop notification live in Owner."""
    def __init__(self,bridge,execution_instance_id=None):
        self.bridge=weakref.ref(bridge);self.ready=threading.Event();self.done=threading.Event()
        self.prepared=self.ready;self.error=None;self.controller_result=None;self.root=None;self.manifest=None
        self.alive=True
        self.execution_instance_id=execution_instance_id
        self.ingress_closed=False
        self.health=None;self.health_source=None;self.writer_identity=None
        self.ros=bool(getattr(bridge,'ros_node',None))
        if self.ros:self.prepared=threading.Event()
    def is_alive(self):return self.alive
    @property
    def thread(self):return self
    def finish(self,status,reason=None):
        bridge=self.bridge()
        if bridge:bridge.emit('recording_stop',status=status,reason=reason)
    def snapshot(self):
        state=self.health.snapshot()if self.health else {}
        return dict(state,ready=self.ready.is_set(),done=self.done.is_set(),ingress_closed=self.ingress_closed,error=self.error or state.get('error'))

    def bind_health(self,description,writer_identity):
        if self.health_source is not None:
            if self.health_source!=description or self.writer_identity!=writer_identity:raise ValueError('recording health binding changed during Run')
            return
        from .ros_data import RecordingHealth
        bridge=self.bridge();runtime=bridge.service.device.runtime
        self.health_source=deepcopy(description);self.writer_identity=deepcopy(writer_identity)
        self.health=RecordingHealth(description,capture_session_id=self.execution_instance_id,run_id=self.execution_instance_id,
            device_id=bridge.service.device.id,writer_identity=writer_identity,plane=runtime.data_plane,ros=bridge.ros_node,
            on_value=lambda ref,value:self._health(value),on_error=self._fault)

    def _health(self,value):
        if self.ingress_closed:return
        if value.get('ready')and not self.error and not self.done.is_set():self.ready.set()
        else:self.ready.clear()
        errors=value.get('source_errors')or{}
        reason=value.get('error')or next(iter(errors.values()),None)
        if reason:self._fault(reason)

    def _fault(self,reason):
        if self.ingress_closed:return
        self.error=self.error or str(reason);self.ready.clear()
        bridge=self.bridge();runner=bridge.runner()if bridge and bridge.runner else None
        if runner is not None and runner.execution_instance_id==self.execution_instance_id:
            with runner.owner.lock:runner._recording_failure(self.error)

    def close(self):
        if self.health is not None:self.health.close();self.health=None


class OwnerInference:
    def __init__(self,service,configuration,specification):
        if set(specification)!={'profile','reference','preloaded_prediction','feedforward'}:
            raise ValueError('explicit inference dependencies required')
        if specification['feedforward']is not None:
            raise ValueError('remote frozen feedforward is not qualified for this controller profile')
        from .inference_workflow import calibrated_adapter
        runtime=service.device.runtime
        if configuration.get('data_plane')!=getattr(runtime,'data_plane',None):
            raise ValueError('inference and Owner ROS data-plane configuration differs')
        inputs=runtime.control_inputs(include_feedback=False) if getattr(runtime,'data_plane',None)else runtime.control_inputs()
        self.adapter,mapping=calibrated_adapter(specification['profile'],inputs['joint_limits'],
            model_id=configuration['model_id'],disabled_grippers=configuration['disabled_grippers'])
        if mapping!=configuration['conversion_identity']['mapping']:raise ValueError('calibrated conversion changed')
        self.reference=reference_read(specification['reference'],inputs['joint_limits'])
        self.camera_bindings={role:(configuration.get('camera_capture_instances',{}).get(role),configuration.get('camera_serials',{}).get(role))
            for role in('head','left','right')}if configuration.get('camera_capture_instances')else {}
        if service.device.environment=='physical'and(not self.camera_bindings or any(not all(pair)for pair in self.camera_bindings.values())):
            raise ValueError('physical inference requires all three frozen camera identities')
        self.preloaded=deepcopy(specification['preloaded_prediction'])
        self.capture_backend=configuration.get('recording_backend')=='capture_session_v1'
        if self.capture_backend and configuration.get('data_plane')is not None:
            from .recording_process import require_recording_protocol
            require_recording_protocol()
        self.service=service;self.session_id=secrets.token_hex(16);self.runner=None
        self.lock=threading.Lock();self.observation=None;self.support=None;self.pending={}
        self.recording_ready=False;self.recording_error=None
        self.ros_node=self.ros_client=None;self.ros_error=None;self.ros_pending_observation=None
        self.ros_configuration=deepcopy(configuration);self.ros_profile=deepcopy(specification['profile'])
        if getattr(service.device.runtime,'data_plane',None) is not None:
            try:self._open_ros(configuration)
            except BaseException:
                self.close();raise

    def _open_ros(self,configuration):
        from .ros_data import RosNode,SourceDirectory,plane_config
        from .ros_inputs import RosInputs
        from .ros_inference import RosModelClient
        runtime=self.service.device.runtime;plane=plane_config(runtime.data_plane)
        if configuration.get('data_plane')!=plane:
            raise ValueError('inference and Owner must use the same declared ROS data plane')
        binding=configuration.get('action_chunk_binding')
        if not isinstance(binding,dict):raise ValueError('ROS inference requires a pinned model session binding')
        self.ros_node=RosNode('infer_control_'+self.session_id[:12],namespace=plane['namespace'],
            domain_id=plane['domain_id'],localhost_only=False)
        self.ros_directory=SourceDirectory(self.ros_node,self.service.device.id)
        from .inference_workflow import CAMERAS
        expected={'state':dict(source_instance_id=runtime.nonce,kind='state')}
        expected.update({camera:dict(source_instance_id=self.camera_bindings[role][0],kind='image')
                         for role,camera in CAMERAS.items()})
        rows=self.ros_directory.resolve(expected)
        self.ros_sources={row['stream_id']:row for row in rows}
        for role,camera in CAMERAS.items():
            if self.ros_sources[camera]['metadata'].get('serial')!=self.camera_bindings[role][1]:
                raise ValueError('ROS camera serial differs from the reviewed binding')
        self.ros_inputs=RosInputs(self.ros_node,[self.ros_sources['state']],on_sample=lambda *_:self._resolve_ros_observation())
        self.ros_client=RosModelClient(self.ros_node,binding=binding,
            clock=lambda:round(runtime.owner.control_time()*1e9))
        self.ros_node.subscribe('observations',self.ros_node.types.Observation,self._receive_ros_observation)

    def _receive_ros_observation(self,message):
        with self.lock:self.ros_pending_observation=message
        self._resolve_ros_observation()

    def _resolve_ros_observation(self):
        with self.lock:message=self.ros_pending_observation
        if message is None:return
        from .ros_inputs import referenced_observation,observation_times
        from .inference_owner import device_state
        runtime=self.service.device.runtime
        try:
            if (runtime is None or message.epoch!=runtime.owner.epoch
                    or message.model_io_digest!=self.ros_client.binding['conversion_id']
                    or message.state.source_instance_id!=runtime.nonce):
                raise ValueError('ROS observation belongs to another Owner, epoch or model conversion')
            feedback=self.ros_inputs.resolve(message.state)
            times=observation_times(message,feedback)
            observation=referenced_observation(message,self.ros_sources,feedback,state_to_device=lambda raw:device_state(
                raw['qpos'],disabled_grippers=tuple(self.ros_configuration['disabled_grippers']),
                right_gripper_device_mm=self.ros_profile['gripper']['max_opening_mm']))
            observation['feedback_sample_time_s']=message.state.source_time_ns/1e9
            observation['camera_span_s']=(max(times)-min(times))/1e9
            with self.lock:
                if self.ros_pending_observation is message:
                    self.observation=observation;self.ros_error=None;self.ros_pending_observation=None
        except (ValueError,LookupError,KeyError)as error:
            with self.lock:
                if self.ros_pending_observation is not message:return
                changed=self.ros_error!=str(error)
                self.observation=None;self.ros_error=str(error)
                if not isinstance(error,LookupError):self.ros_pending_observation=None
            if changed:
                import logging
                logging.getLogger(__name__).warning('ROS observation epoch=%s state_sequence=%s rejected: %s',
                    message.epoch,message.state.sequence,error)

    def _ros_observe(self,now):
        # No DDS calls or waits on the control tick: read the resolved view only.
        with self.lock:observation=deepcopy(self.observation)
        if observation is None:return None
        runtime=self.service.device.runtime;message=observation['ros_observation']
        if runtime is None or message.epoch!=runtime.owner.epoch:return None
        if any(self.ros_directory.values.get(key)!=source for key,source in self.ros_sources.items()):return None
        if self.ros_inputs.caches['state'].error:return None
        for rule,stamp,identity in [('G02',observation['source_time'],'model_observation'),
                ('G03',observation['feedback_sample_time_s'],'paired_feedback')]:
            if self._input_decision(rule,stamp,now,identity)['verdict']=='reject':return None
        if self._input_decision('G04',observation['source_time'],now,'camera_span',
                observed=observation.pop('camera_span_s'))['verdict']=='reject':return None
        observation.pop('feedback_sample_time_s')
        return observation

    def close(self):
        if self.ros_client is not None:self.ros_client.cancel()
        recording=getattr(getattr(self.service.device.runtime,'owner',None),'recording',None)
        if isinstance(recording,RecordingFacts)and recording.bridge()is self:recording.close()
        if self.ros_node is not None:self.ros_node.close();self.ros_node=None
        self.ros_inputs=self.ros_client=self.ros_directory=None

    def emit(self,kind,**value):
        self.service.event('owner_inference',session_id=self.session_id,kind=kind,**plain(value))

    def _input_decision(self,rule,stamp,now,identity,*,observed=None):
        from .execution_settings import Guard
        runtime=self.service.device.runtime
        runner=self.runner()if self.runner else None
        policy=getattr(runner,'guard_policy',None)
        frozen=runtime.execution_calibration or {}
        if policy is None and runner is not None and frozen.get('execution_instance_id')==runner.execution_instance_id:
            policy=frozen.get('control_policy')
        if policy is None:policy=runtime._guard_policy('inference')
        signal=dict(rule_id=rule,signal_id=identity,observed=now-stamp if observed is None else observed,source_time=stamp,checked_time=now,
                    epoch=runtime.owner.epoch)
        if runner is not None:signal['run_id']=runner.execution_instance_id
        decision=Guard.evaluate(signal,'input',policy)
        if runner is not None:runner._emit('guard_decision',decision)
        return decision

    def factory(self,owner,proxy,feedback,configuration):
        from .inference_owner import make_inference_runner
        bridge=weakref.ref(self)
        def borrow():
            value=bridge()
            if value is None:raise ValueError('inference configuration retired')
            return value
        def observe(now,fb):
            value=borrow()
            if value.ros_client is not None:return value._ros_observe(now)
            with value.lock:observation=deepcopy(value.observation)
            if observation is None:return None
            if value._input_decision('G02',observation['source_time'],now,'model_observation')['verdict']=='reject':return None
            if value._input_decision('G03',observation.pop('feedback_sample_time_s'),now,'paired_feedback')['verdict']=='reject':return None
            return observation
        def predict(request):return borrow().predict(request)
        def support(now,fb):
            value=borrow()
            with value.lock:return deepcopy(value.support)
        def event(name,value):
            bridge=borrow()
            if bridge.ros_client is not None:
                return bridge.service.device.runtime.ros_publisher.event('inference_'+name,plain(value),
                    run_id=owner.execution_instance_id,epoch=owner.epoch,source_time_ns=round(owner.control_time()*1e9))
            bridge.emit('metric',name=name,value=value)
        def record(name,value):
            bridge=borrow()
            if bridge.ros_client is None:return bridge.emit('record',name=name,value=value)
            publisher=bridge.service.device.runtime.ros_publisher
            payload=plain(value)
            if name=='prediction_received':payload.pop('prediction_model14',None)
            if name=='prediction_requested':
                payload.pop('applied_history_model14',None);payload.pop('state_device14',None)
            if isinstance(payload.get('raw'),dict)and 'qpos'in payload['raw']:
                raw=payload['raw']
                stamp=raw['source_time_ns']if raw.get('source_clock_id')==publisher.clock_id else round(raw['sample_time_s']*1e9)
                reference=publisher.reference('state',source_time_ns=stamp)
                if reference is None:raise ValueError('recorded control feedback source reference was evicted')
                payload=dict(payload,raw=dict(state_ref=reference))
            publisher.event('inference_'+name,payload,run_id=owner.execution_instance_id,
                epoch=owner.epoch,source_time_ns=round(owner.control_time()*1e9))
            if name=='finish':
                value=deepcopy(value)
                try:value['ros_final_seq']=publisher.flush()
                except Exception as error:
                    with publisher.lock:value['ros_final_seq']=dict(publisher.sequences)
                    value['recording_error']=str(error)
            if name in('prepare','finish'):bridge.emit('record',name=name,value=value)
        def prepared():
            value=borrow()
            if value.recording_error:raise ValueError(value.recording_error)
            if value.ros_client is not None:
                recording=owner.recording
                if recording.error:raise ValueError(recording.error)
                return recording.prepared.is_set()and recording.ready.is_set()
            return value.recording_ready
        capture=configuration.get('recording_backend')=='capture_session_v1'
        enabled=configuration['recording_enabled'] or capture
        self.recording_ready=False;self.recording_error=None
        runner=make_inference_runner(owner,proxy,feedback,configuration,predict=predict,observe=observe,
            support=support,adapter=self.adapter,event=event,
            record=record if enabled else None,
            record_prepared=prepared if enabled else None,
            prepared_reference=self.reference,preloaded_prediction=self.preloaded)
        if enabled:
            previous=owner.recording
            if previous is not None and not getattr(previous,'ingress_closed',False) and (not previous.done.is_set()or previous.thread.is_alive()):
                raise ValueError('previous recording ingress is not closed')
            if isinstance(previous,RecordingFacts):previous.close()
            owner.recording=RecordingFacts(self,owner.execution_instance_id)
        # Business acknowledgement gates recording completion, not Owner motion.
        self.runner=weakref.ref(runner)
        details={}
        if capture:
            frozen=self.service.device.runtime.execution_calibration.get('control_policy')
            if frozen is None:raise ValueError('common capture requires the frozen Run policy')
            details['control_policy']=deepcopy(frozen)
            runner.control_policy=deepcopy(frozen);runner.guard_policy=deepcopy(frozen)
        self.emit('created',execution_instance_id=owner.execution_instance_id,configuration=configuration,**details)
        return runner

    def predict(self,request):
        if self.ros_client is not None:
            runner=self.runner()if self.runner else None
            if runner is None:raise ValueError('inference runner unavailable')
            observation=deepcopy(request['observation']['ros_observation'])
            observation.run_id=runner.execution_instance_id;observation.epoch=runner.owner.epoch
            self.ros_client.bind(observation.run_id,observation.epoch)
            deadline=request['observation']['source_time']+runner.config['maximum_observation_age_s']
            response=self.ros_client.predict(request['request_id'],observation,seed=request['seed'],
                task=self.ros_client.binding['task'],history=request['applied_history'],
                valid_until_ns=round(deadline*1e9))
            from .inference import validate_action_chunk
            validate_action_chunk(response,binding=self.ros_client.binding,request_id=request['request_id'],
                epoch=runner.owner.epoch,clock_domain=runner.owner.execution_clock_id,now=runner.owner.control_time(),
                observation_source_time=request['observation']['source_time'],
                valid_until_source_time=round(deadline*1e9)/1e9)
            if response['observation'].get('frame_ids')!=request['observation']['frame_ids']:
                raise ValueError('action chunk frame identities differ from pending ROS observation')
            return response
        request=plain(request);identity=request['request_id'];result=queue.Queue(maxsize=1)
        runner=self.runner()if self.runner else None
        if runner is None:raise ValueError('inference runner unavailable')
        epoch,generation=runner.owner.epoch,runner.loop.generation
        context=dict(epoch=epoch,clock_domain=runner.owner.execution_clock_id,
            valid_until_source_time=request['observation']['source_time']+runner.config.get('maximum_observation_age_s',2.))
        with self.lock:
            if self.pending:raise ValueError('only one prediction may be in flight')
            self.pending[identity]=dict(queue=result,request=deepcopy(request),context=context,generation=generation,received=False)
        self.emit('query',request=request,consumer_context=context)
        try:
            deadline=time.monotonic()+2.
            while time.monotonic()<deadline:
                runner=self.runner()if self.runner else None
                if (runner is None or runner.status in('STOPPED','FAULT','READY')
                        or runner.owner.epoch!=epoch or runner.loop.generation!=generation):raise ValueError('prediction revoked')
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
            if self.ros_client is not None:raise ValueError('ROS observations cannot use the legacy business data path')
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
            for rule,stamp,identity in [('G02',value['source_time'],'model_observation'),
                    ('G03',value['feedback_sample_time_s'],'paired_feedback')]:
                decision=self._input_decision(rule,stamp,now,identity)
                if decision['verdict']=='reject':raise ValueError(rule+': fresh actual observation and feedback required')
            with self.lock:self.observation=deepcopy(value);self.support=deepcopy(args.get('support'))
        elif operation=='inference_result':
            if self.ros_client is not None:raise ValueError('ROS model results cannot use the legacy business data path')
            response=args.get('response');identity=args.get('request_id')
            with self.lock:
                pending=self.pending.get(identity)
                runner=self.runner()if self.runner else None
                if (pending is None or runner is None or runner.owner.epoch!=pending['context']['epoch']
                        or runner.loop.generation!=pending['generation']):
                    raise ValueError('late or revoked prediction rejected')
                if pending['received']:raise ValueError('prediction result already submitted')
                if response is not None:
                    binding=runner.config.get('action_chunk_binding')
                    if binding is not None:
                        from .inference import validate_action_chunk
                        validate_action_chunk(response,binding=binding,request_id=identity,epoch=runner.owner.epoch,
                            clock_domain=pending['context']['clock_domain'],now=runner.owner.control_time(),
                            observation_source_time=pending['request']['observation']['source_time'],
                            valid_until_source_time=pending['context']['valid_until_source_time'])
                        if response['observation'].get('frame_ids')!=pending['request']['observation'].get('frame_ids',{}):
                            raise ValueError('action chunk frame identities differ from pending observation')
                    else:
                        rows=np.asarray(response.get('action14'),dtype=float)
                        if response.get('schema')=='workbench.action-chunk.v1':raise ValueError('prepare trusted chunk binding before consuming envelopes')
                        if rows.shape!=(60,14)or not np.isfinite(rows).all()or response.get('request_id')!=identity:
                            raise ValueError('finite response for original prediction request required')
                try:pending['queue'].put_nowait(dict(response=deepcopy(response),error=args.get('error')))
                except queue.Full:raise ValueError('prediction result already submitted')
                pending['received']=True
        elif operation=='inference_recording':
            if type(args.get('prepared'))is not bool:raise ValueError('explicit recording readiness required')
            if type(args.get('ingress_closed'))is not bool:raise ValueError('explicit recording ingress closure required')
            runtime=self.service.device.runtime;recording=runtime.owner.recording
            if not isinstance(recording,RecordingFacts)or recording.bridge()is not self:raise ValueError('recording lease changed')
            runner=self.runner()if self.runner else None
            if (self.capture_backend and (runner is None
                    or args.get('execution_instance_id')!=runner.execution_instance_id)):
                raise ValueError('recording facts belong to another Run')
            self.recording_ready=args['prepared'];self.recording_error=args.get('error')
            recording.ingress_closed=recording.ingress_closed or args['ingress_closed']
            if recording.ingress_closed:recording.ready.clear()
            if self.ros_client is not None:
                if args.get('health_source')is not None:recording.bind_health(args['health_source'],args['writer_identity'])
                if args['prepared']:recording.prepared.set()
                if args.get('error'):recording._fault(args['error'])
                if args.get('done')is True:recording.done.set();recording.ready.clear()
                recording.alive=args.get('alive',True)
                return dict(accepted=True)
            recording.error=None if self.capture_backend else self.recording_error
            if self.capture_backend and self.recording_error:
                with runtime.owner.lock:runner._recording_failure(self.recording_error)
            if args['prepared']and not recording.ingress_closed:recording.ready.set()
            elif self.capture_backend:recording.ready.clear()
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
