"""World-memory WAM family. Raw native14/RGB in; absolute native14 targets out."""
from collections import OrderedDict, deque
from copy import copy, deepcopy
import hashlib
import json
import math
from pathlib import Path
import re
import time
import threading
import secrets

import numpy as np

from . import wam

SCHEMA='workbench.wam-mem.v1'
MEMORY_PROTOCOL='workbench.wam-memory-phase.v4'
ROLES=('checkpoint','vae','phoenix_encoder','csvae_checkpoint','text_index','text_features','source_manifest')
_SHA=re.compile('[0-9a-f]{64}')
_SOURCE_SHA256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()


def artifact_identity(value):
    if isinstance(value,str):value={'sha256':value}
    if not isinstance(value,dict):raise ValueError('artifact identity required')
    if set(value)=={'sha256'} and _SHA.fullmatch(str(value['sha256'])):return deepcopy(value)
    if (set(value)=={'source_receipt_sha256','path','bytes'} and _SHA.fullmatch(str(value['source_receipt_sha256']))
            and isinstance(value['path'],str)and not Path(value['path']).is_absolute()and '..'not in Path(value['path']).parts
            and type(value['bytes'])is int and value['bytes']>64*1024**2
            and Path(value['path']).suffix.lower()in('.pt','.pth','.ckpt','.safetensors','.bin')):
        return deepcopy(value)
    raise ValueError('artifact requires a digest or an explicit large-weight source receipt')


def metadata_weight(registry,row):
    return row.get('sha256')is None and any(isinstance(pin,dict)and pin.get('path')==row.get('path')
        and pin.get('bytes')==row.get('bytes')and artifact_identity(pin)for pin in registry.get('artifacts',{}).values())


def is_model_id(value):
    return isinstance(value,str)and re.fullmatch(r'wam-mem-[a-z0-9-]+-step[0-9]+',value)is not None


def validate_descriptor(value):
    fields={'schema','model_id','epoch','step','backend','artifacts','model_bundle_sha256','runtime','prompt',
        'inference_options','inference_capabilities','memory_protocol'}
    if (not isinstance(value,dict)or set(value) not in (fields,fields|{'training_execute_actions'}) or value['schema']!=SCHEMA
            or not is_model_id(value['model_id'])or value['backend']!='torch'or value['epoch']is not None
            or type(value['step'])is not int or value['step']<=0
            or int(value['model_id'].rsplit('-step',1)[1])!=value['step']
            or not isinstance(value['prompt'],str)or not 0<len(value['prompt'])<=4096
            or value['memory_protocol']!=MEMORY_PROTOCOL):
        raise ValueError('explicit WAM-MEM model, native target and memory contract required')
    if (not isinstance(value['artifacts'],dict)or set(value['artifacts'])!=set(ROLES)
            or not _SHA.fullmatch(str(value['model_bundle_sha256']))):
        raise ValueError('WAM-MEM producer artifact identities required')
    for pin in value['artifacts'].values():artifact_identity(pin)
    if value['runtime']!={'adapter':'world_memory_policy.v1','python_abi':'cp312','adapter_sha256':_SOURCE_SHA256}:
        raise ValueError('WAM-MEM requires the registered Torch interface')
    wam.validate_inference_options(value['inference_options'],value['inference_capabilities'])
    if 'training_execute_actions' in value:
        steps=value['training_execute_actions']
        if type(steps) is not int or not 1<=steps<=32:
            raise ValueError('training execution length must fit the 32-step prediction')
    return deepcopy(value)


def descriptor(registry,*,denoise_steps=None):
    rows=registry.get('files');pins=registry.get('artifacts')
    if not isinstance(rows,list)or not rows or not isinstance(pins,dict)or set(pins)!=set(ROLES):
        raise ValueError('complete WAM-MEM deployment artifact registry required')
    capabilities=registry.get('inference_capabilities',{'denoise_steps':{'minimum':2,'maximum':2,'default':2}})
    if denoise_steps is None:
        denoise_steps=registry.get('selected_inference_options',{}).get('denoise_steps',capabilities['denoise_steps']['default'])
    options=wam.validate_inference_options({'denoise_steps':denoise_steps},capabilities)
    canonical=json.dumps(sorted(rows,key=lambda row:row['path']),sort_keys=True,separators=(',',':')).encode()
    if hashlib.sha256(canonical).hexdigest()!=registry.get('bundle_sha256'):
        raise ValueError('WAM-MEM artifact manifest identity differs')
    training={}
    config_rows=[row for row in rows if row['path']=='metadata/training-config.json']
    if config_rows:
        from .model_install import read_json,verify_file
        path=Path(registry['root'])/'metadata/training-config.json'
        if len(config_rows)!=1 or not verify_file(path,config_rows[0]):
            raise ValueError('training configuration differs from the installed manifest')
        training['training_execute_actions']=read_json(path)['data']['execute_actions']
    return validate_descriptor(dict(schema=SCHEMA,model_id=registry['model_id'],epoch=None,step=registry['step'],backend='torch',
        artifacts={k:artifact_identity(v)for k,v in pins.items()},model_bundle_sha256=registry['bundle_sha256'],prompt=registry['instruction'],
        runtime=dict(adapter='world_memory_policy.v1',python_abi='cp312',adapter_sha256=_SOURCE_SHA256),
        memory_protocol=MEMORY_PROTOCOL,inference_options=options,inference_capabilities=deepcopy(capabilities),**training))


def variants(registry):
    default=descriptor(registry);bounds=default['inference_capabilities']['denoise_steps']
    return [dict(deepcopy(default),inference_options=dict(denoise_steps=n))
            for n in range(bounds['minimum'],bounds['maximum']+1)]


sampling_change=wam.sampling_change


def verify_registry(registry):
    """First-load small-artifact checks; reuse producer identities for large weights."""
    from .mixed366 import sha
    bundle=descriptor(registry);root=Path(registry['root']).resolve(strict=True);paths={};names=set()
    if registry.get('machine_id_sha256')!=sha('/etc/machine-id')or not str(registry.get('gpu_uuid','')).startswith('GPU-'):
        raise ValueError('registered model host and GPU required')
    for row in registry['files']:
        relative=Path(row['path']);path=(root/relative).resolve(strict=True)
        if (relative.is_absolute()or '..'in relative.parts or str(relative)in names or not path.is_relative_to(root)
                or not path.is_file()or type(row['bytes'])is not int or path.stat().st_size!=row['bytes']
                or not _SHA.fullmatch(str(row.get('sha256')))and not metadata_weight(registry,row)):
            raise ValueError('WAM-MEM artifact path, size or source identity differs')
        names.add(str(relative))
        if row['bytes']<=16*1024**2 and sha(path)!=row['sha256']:raise ValueError('WAM-MEM small artifact changed')
        if row.get('sha256'):paths.setdefault(row['sha256'],[]).append(path)
    selected={}
    for role,pin in bundle['artifacts'].items():
        identity=pin.get('sha256',pin.get('source_receipt_sha256'))
        if len(paths.get(identity,[]))!=1:raise ValueError('unique WAM-MEM source artifact required')
        path=paths[identity][0]
        if 'source_receipt_sha256'in pin:
            if path.stat().st_size>16*1024**2:raise ValueError('bounded source receipt required')
            source=json.loads(path.read_text())
            if (source.get('schema')!='workbench.model-source.v1'or source.get('model_id')!=bundle['model_id']
                    or source.get('step')!=bundle['step']or source.get('files',{}).get(pin['path'],{}).get('bytes')!=pin['bytes']
                    or pin['path']not in names):raise ValueError('large-weight source receipt differs')
            path=(root/pin['path']).resolve(strict=True)
            if not path.is_relative_to(root)or path.stat().st_size!=pin['bytes']:raise ValueError('large-weight size differs')
        selected[role]=path
    source=(root/registry['model_source']).resolve(strict=True)
    if not source.is_relative_to(root)or not(source/'world_memory/inference.py').is_file():
        raise ValueError('registered world_memory source directory required')
    selected['source']=source
    return root,selected


def worker_identity(bundle):
    value=validate_descriptor(bundle)
    return dict(status='ready',model_id=value['model_id'],epoch=None,step=value['step'],backend='torch',strict_load=True,
        checkpoint_identity=value['artifacts']['checkpoint'],model_bundle_sha256=value['model_bundle_sha256'],
        runtime=value['runtime'],memory_protocol=MEMORY_PROTOCOL,action_horizon=32,history_steps=0,action_hz=30,
        inference_steps=value['inference_options']['denoise_steps'],input_domain='recorded_native14_and_rgb',output_domain='recorded_native_state14',command_publication=False)


class Adapter(wam.Adapter):
    def model_input(self,observation):
        state=np.asarray(self.device_to_model(observation['device_state14']),dtype=np.float32)
        images={k:np.ascontiguousarray(v)for k,v in observation['images'].items()}
        validate_input(state,images)
        return state,images


def validate_input(state,images):
    state=np.asarray(state)
    if state.shape!=(14,)or state.dtype.kind!='f'or not np.isfinite(state).all():raise ValueError('finite raw native state14 required')
    if set(images)!=set(wam.CAMS):raise ValueError('all three original WAM-MEM RGB views required')
    for image in images.values():
        if not isinstance(image,np.ndarray)or image.dtype!=np.uint8 or image.ndim!=3 or image.shape[2]!=3 or min(image.shape[:2])<=0:
            raise ValueError('WAM-MEM images must be original uint8 RGB HWC')


def validate_phase(value):
    if (not isinstance(value,dict)or set(value)!={'protocol','session_id','epoch','clock_domain','start','revision'}
            or value['protocol']!=MEMORY_PROTOCOL or value['start']not in('fresh','continue')
            or any(type(value[k])is not int or value[k]<0 for k in ('epoch','revision'))
            or any(not isinstance(value[k],str)or not 0<len(value[k])<=128 for k in ('session_id','clock_domain'))):
        raise ValueError('explicit WAM-MEM v4 memory context required')
    return value


def validate_memory_sample(value, observation_time, *, required=False):
    if value is None:
        if required:raise ValueError('real sampled context metadata required')
        return None
    keys={'states14','actions14','state_times','action_times','mode','effective_index',
          'action_semantics','valid_actions','executed_action_condition'}
    if (not isinstance(value,dict) or set(value)!=keys
            or value['mode']not in ('INITIAL','MODEL_EXECUTING','TEACH')
            or value['action_semantics']!='materialized_next_observation'
            or value['executed_action_condition']is not False
            or type(value['effective_index'])is not int or value['effective_index']<0
            or type(value['valid_actions'])is not int or not 0<=value['valid_actions']<=32):
        raise ValueError('T01 real-sample contract differs')
    if type(observation_time)not in (int,float)or not math.isfinite(observation_time):
        raise ValueError('finite memory observation time required')
    count=value['valid_actions']
    for name in ('states14','actions14'):
        array=np.asarray(value[name],dtype=float)
        if array.size!=count*14 or count and array.shape!=(count,14) or not np.isfinite(array).all():
            raise ValueError('bounded real GT states/actions required')
    before=np.asarray(value['state_times'],dtype=float);after=np.asarray(value['action_times'],dtype=float)
    if (before.shape!=(count,) or after.shape!=(count,) or not np.isfinite(before).all()
            or not np.isfinite(after).all() or np.any(after<=before) or np.any(after>observation_time)
            or np.any(np.diff(before)<=0) or np.any(np.diff(after)<=0)):
        raise ValueError('GT timestamps must precede the selected real observation')
    return value


class PhaseMemory:
    """One leased recurrent policy. Only actual observations write its memory."""
    def __init__(self,policy,instruction,seed):
        self.policy,self.instruction,self.seed=policy,instruction,seed
        self.phase=None;self.revision=0;self.results=OrderedDict()
        self.origin=self.last_time=None;self.calls=self.observations=self.manual_observations=0
        self.context_cache=None;self.context_identity=None;self.context_previous_times=[]

    def _reset(self):
        self.policy.reset(control_frequency=30)
        self.origin=self.last_time=None;self.calls=0;self.observations=0;self.manual_observations=0
        self.context_cache=None;self.context_identity=None;self.context_previous_times=[]

    def clear(self):
        self._reset();self.phase=None;self.revision=0;self.results.clear()

    def snapshot(self):
        return dict(protocol=MEMORY_PROTOCOL,revision=self.revision,empty=self.observations==0,
            observations=self.observations,manual_observations=self.manual_observations,infer_calls=self.calls,
            source_time=self.last_time,origin_source_time=self.origin,
            executed_action_context=False,predictions_used_as_actions=0,
            action_context_reason='T01 materialized_next_observation training contract masks historical executed controls',
            write_policy='shared_effective_samples',
            sampling_stride=getattr(getattr(self.policy.config,'data',None),'execute_actions',32),prediction_commits_memory=False)

    def infer(self,request):
        context=validate_phase(request['memory']);operation=request.get('memory_operation','infer')
        if operation not in('infer','observe','clear'):raise ValueError('unknown memory operation')
        if operation=='observe'and context['start']!='continue':raise ValueError('manual observations preserve memory')
        phase=(context['session_id'],context['clock_domain'],context['epoch'])
        same_session=self.phase is not None and phase[0]==self.phase[0]
        if same_session and phase[1]!=self.phase[1]:raise ValueError('memory clock changed; acquire a new session')
        if same_session and phase[1]==self.phase[1]and phase[2]<self.phase[2]:raise ValueError('obsolete memory phase')
        if not same_session:
            if context['revision']!=0:raise ValueError('new memory lease starts at revision zero')
            self.revision=0;self.results.clear()
        if context['revision']!=self.revision:raise ValueError('obsolete memory revision')
        identity=request['request_id']
        if not isinstance(identity,str)or not 0<len(identity)<=128:raise ValueError('bounded request identity required')
        obs=request.get('observation');stamp=None
        sample=request.get('memory_sample')
        digest=hashlib.sha256(json.dumps([phase,context['revision'],context['start'],operation,request.get('seed'),
            self.policy.config.inference.denoising_steps],separators=(',',':')).encode())
        if operation!='clear':
            validate_input(obs['model_state14'],obs['images']);stamp=obs['source_time']
            validate_memory_sample(sample, stamp, required=operation=='observe')
            if type(stamp)not in(int,float)or not math.isfinite(stamp)or stamp<0:raise ValueError('finite source timestamp required')
            digest.update(str(stamp).encode());digest.update(np.asarray(obs['model_state14'],dtype=np.float32).tobytes())
            for name in wam.CAMS:
                digest.update(str(obs['images'][name].shape).encode());digest.update(obs['images'][name].tobytes())
        digest.update(json.dumps(sample,sort_keys=True,separators=(',',':')).encode())
        key=(phase,context['revision'],identity);signature=digest.hexdigest()
        if key in self.results:
            old,result=self.results[key]
            if old!=signature:raise ValueError('memory request identity reused with different data')
            return deepcopy(result)
        reset=(not same_session or self.phase[1]!=phase[1]
            or operation=='infer'and phase!=self.phase and context['start']=='fresh')
        if operation=='clear':
            self._reset();self.revision+=1;self.results.clear();self.phase=phase
            return dict(request_id=identity,audit=dict(memory=self.snapshot(),operation='clear',command_publication=False))
        if reset:self._reset()
        self.phase=phase
        if self.last_time is not None and (stamp < self.last_time or operation=='observe' and stamp==self.last_time):
            raise ValueError('memory observations must strictly advance; queries cannot read future memory')
        if self.origin is None:self.origin=stamp
        model_time=stamp-self.origin
        rgb={role:obs['images'][name]for role,name in zip(('main','left','right'),wam.CAMS)}
        state=np.asarray(obs['model_state14'],dtype=np.float32)
        observation_digest=hashlib.sha256(state.tobytes())
        for role in ('main','left','right'):
            observation_digest.update(str(rgb[role].shape).encode())
            observation_digest.update(rgb[role].tobytes())
        observed_identity=observation_digest.hexdigest()
        result=dict(request_id=identity)
        bootstrap=self.last_time is None
        if operation=='observe' or bootstrap:
            self.context_previous_times=list(self.policy.times)
            cache=self.policy._context(rgb,model_time,self.instruction,state)
            self.context_cache=cache;self.context_identity=observed_identity
            self.policy.memory=cache.memory;self.policy.times.append(model_time)
            self.policy.pending=None;self.policy.executed.clear()
            self.last_time=stamp;self.observations+=1
            if sample and sample['mode']=='TEACH':self.manual_observations+=1
        if operation=='infer':
            if type(request['seed'])is not int or not 0<=request['seed']<2**63:raise ValueError('uint63 sampling seed required')
            effective_seed=(request['seed']+self.calls)%(2**63);self.seed(effective_seed)
            query=copy(self.policy)
            query.times=list(self.policy.times);query.executed=[];query.pending=None
            if stamp==self.last_time:
                if observed_identity!=self.context_identity:
                    raise ValueError('same memory observation timestamp carried different data')
                query.times=list(self.context_previous_times)
                cache=self.context_cache
            else:
                cache=query._context(rgb,model_time,self.instruction,state)
            # Policy.act keeps its native denoising implementation. Its context
            # and assignments belong to this request-local policy view only.
            query._context=lambda *args,**kwargs:cache
            query.observe(rgb,model_time)
            prediction=query.act(self.instruction,state)['actions']
            if hasattr(prediction,'detach'):prediction=prediction.detach().float().cpu().numpy()
            action=np.asarray(prediction,dtype=np.float32)
            if action.shape!=(32,14)or not np.isfinite(action).all():raise ValueError('WAM-MEM requires finite absolute32x14 output')
            result['action14']=action;self.calls+=1
        result['audit']=dict(memory=dict(self.snapshot(),session_id=phase[0],clock_domain=phase[1],phase_epoch=phase[2],
            reset_applied=reset,model_time_s=model_time,start=context['start'],
            observation_kind=sample['mode']if sample else'initial'if bootstrap else'model_query',
            query_source_time=stamp,source_references=deepcopy(obs.get('frame_ids',{}))),
            operation=operation,command_publication=False,memory_sample=deepcopy(sample))
        if operation=='infer':result['audit'].update(seed=effective_seed,requested_seed=request['seed'])
        self.results[key]=(signature,deepcopy(result))
        while len(self.results)>32:self.results.popitem(last=False)
        return result


def worker_request(request):
    warm=request.get('_warmup')is True
    memory=(dict(protocol=MEMORY_PROTOCOL,session_id='internal-warmup',epoch=0,clock_domain='warmup',start='fresh',revision=0)
        if warm else request.get('memory'))
    validate_phase(memory)
    operation=request.get('memory_operation','infer')
    result=dict(request_id=request['request_id'],seed=request.get('seed',0),memory=deepcopy(memory),warmup=warm,memory_operation=operation,
        memory_sample=deepcopy(request.get('memory_sample')))
    if operation!='clear':result['observation']={k:request['observation'][k]for k in ('model_state14','images','source_time','frame_ids')if k in request['observation']}
    return result


def Client(registry,bundle):
    import sys
    return wam.Client(registry,bundle,implementation=sys.modules[__name__])


class RealTrajectorySampler:
    """Select real samples 0, stride, 2*stride across active MODEL and TEACH.

    Waiting pauses the effective index. Next-state GT links only adjacent real
    samples, while original timestamps retain all waits and source gaps.
    """
    def __init__(self, *, stride=32, frequency=30.):
        if type(stride) is not int or not 1 <= stride <= 32:
            raise ValueError('memory sampling stride must be within the model horizon')
        if type(frequency) not in (int, float) or not math.isfinite(frequency) or frequency <= 0:
            raise ValueError('finite positive memory sampling frequency required')
        self.stride = stride
        self.period = 1. / frequency
        self.previous = None
        self.history = []
        self.last_source_time = None
        self.last_grid_time = None
        self.samples = 0
        self.blocks = 0
        self.gaps = 0

    def _selected(self, observation, phase, index):
        rows = self.history
        result = dict(observation=deepcopy(observation),
                      states14=[row[0] for row in rows], actions14=[row[1] for row in rows],
                      state_times=[row[2] for row in rows], action_times=[row[3] for row in rows],
                      mode=phase, effective_index=index, action_semantics='materialized_next_observation',
                      valid_actions=len(rows), executed_action_condition=False)
        self.history = []
        self.blocks += 1
        return result

    def bootstrap(self, observation):
        if self.samples:
            return None
        self.samples = 1
        self.last_source_time = observation['source_time']
        return self._selected(observation, 'INITIAL', 0)

    def pause(self):
        self.previous = None

    def offer(self, observation, *, phase, grid_time):
        if phase not in ('MODEL_EXECUTING', 'TEACH', 'WAIT', 'HOLD'):
            raise ValueError('explicit Owner execution phase required')
        stamp = observation['source_time']
        if any(type(value) not in (int, float) or not math.isfinite(value) for value in (stamp, grid_time)):
            raise ValueError('finite original observation and grid timestamps required')
        state = np.asarray(observation['model_state14'], dtype=np.float64)
        if state.shape != (14,) or not np.isfinite(state).all():
            raise ValueError('one finite measured native14 state required')
        if self.last_source_time is not None and stamp < self.last_source_time:
            raise ValueError('memory sample source time regressed')
        if self.last_grid_time is not None and grid_time < self.last_grid_time:
            raise ValueError('memory sample grid time regressed')
        if stamp == self.last_source_time or grid_time == self.last_grid_time:
            return []
        self.last_source_time = stamp
        self.last_grid_time = grid_time
        if phase in ('WAIT', 'HOLD'):
            self.pause()
            return []
        if self.previous is not None:
            old_state, old_stamp, old_grid = self.previous
            if grid_time-old_grid <= 1.5*self.period:
                self.history.append((old_state, state.tolist(), old_stamp, stamp))
                self.history = self.history[-self.stride:]
            else:
                self.gaps += 1
        self.previous = state.tolist(), stamp, grid_time
        index = self.samples
        self.samples += 1
        if index % self.stride == 0:
            return [self._selected(observation, phase, index)]
        return []


class MemorySession:
    """One real-data sampler and serialized writer for MODEL and TEACH."""
    def __init__(self,client,adapter,clock_domain,*,resolve=None,record=None):
        self.client,self.adapter,self.clock_domain=client,adapter,clock_domain
        self.resolve,self.record=resolve,record
        self.lock=threading.RLock();self.io=threading.Lock();self.wake=threading.Event();self.stop=threading.Event()
        self.mode='HOLD';self.epoch=-1;self.generation=0;self.control_sequence=None
        self.control_event_gaps=0;self.last_control_gap=None
        self.sampling_stride=client.model_bundle.get('training_execute_actions',32)
        self.sampler=RealTrajectorySampler(stride=self.sampling_stride);self.pending=deque();self.offered=0;self.mode_events=0
        self.start='continue';self.error=None;self.audit=None;self.last_manual=None
        self.thread=threading.Thread(target=self._work,name='wam-memory-observations',daemon=True);self.thread.start()

    def mode_event(self,value):
        epoch=value.get('command_epoch')
        with self.lock:
            if type(epoch)is not int or epoch<self.epoch:return
            self.mode_events+=1;self.epoch=epoch
            self.mode=value.get('mode','HOLD')if value.get('completed')else'HOLD'

    def control_event(self,sequence):
        with self.lock:
            if self.control_sequence is not None:
                if sequence<=self.control_sequence:return False
                if sequence!=self.control_sequence+1:
                    self.control_event_gaps+=1
                    self.last_control_gap=dict(previous=self.control_sequence,received=sequence,
                        missing=sequence-self.control_sequence-1)
                    # v4 samples the authoritative activity on paired RobotState.
                    # This separate audit stream is not a memory-write input.
                    from .pipeline_timing import timing
                    timing.record('memory.control_event_gap',time.monotonic_ns(),**self.last_control_gap)
            self.control_sequence=sequence
            return True

    def source_error(self,error):
        with self.lock:self.error=str(error);self.pending.clear()
        from .pipeline_timing import timing
        timing.fault('WAM-MEM: '+str(error),epoch=self.epoch)

    def offer(self,observation,epoch):
        activity=observation.get('memory_sampling')
        if (not isinstance(activity,dict)or activity.get('schema')!='workbench.memory-activity.v1'
                or activity.get('run_id')!=observation.get('run_id')):
            return
        state=(observation['model_state14']if 'model_state14'in observation else
               self.adapter.device_to_model(observation['device_state14']))
        value=dict(observation,model_state14=np.asarray(state,dtype=float).tolist())
        with self.lock:
            if self.stop.is_set()or self.error or epoch<self.epoch:return
            selected=self.sampler.offer(value,phase=activity['phase'],grid_time=observation['sample_grid_time'])
            self.epoch=epoch
            self.mode={'MODEL_EXECUTING':'MODEL','WAIT':'MODEL','TEACH':'TEACH','HOLD':'HOLD'}[activity['phase']]
            self.offered=self.sampler.samples
            for sample in selected:
                if len(self.pending)>=8:
                    self.source_error('real memory writer queue full; no observations silently dropped')
                    return
                self.pending.append((sample,epoch,self.generation))
        if selected:self.wake.set()

    def _drain(self,until=None):
        while True:
            with self.lock:
                if not self.pending:return
                sample,epoch,generation=self.pending[0]
                if until is not None and sample['observation']['source_time']>until:return
                self.pending.popleft()
                if generation!=self.generation:continue
            observation=sample['observation']
            if self.resolve is not None:observation=self.resolve(observation)
            prepared=self.adapter.model_input(observation)
            metadata={key:value for key,value in sample.items()if key!='observation'}
            request=dict(request_id=secrets.token_hex(16),seed=0,run_id=observation.get('run_id'),epoch=epoch,
                observation=observation,applied_history=np.empty((0,14)),memory_sample=metadata,
                memory=dict(protocol=MEMORY_PROTOCOL,session_id=self.client.session_id,clock_domain=self.clock_domain,
                    epoch=epoch,revision=self.client.memory_revision,start='continue'))
            def call():return self.client.infer(request,prepared_input=prepared,memory_only=True)
            result=self.record(request,prepared,call)if self.record is not None else call()
            with self.lock:
                if generation==self.generation:
                    self.audit=deepcopy(result['audit']['memory'])
                    if sample['mode']=='TEACH':self.last_manual=observation['source_time']

    def _work(self):
        while not self.stop.is_set():
            self.wake.wait(.1);self.wake.clear()
            with self.lock:generation=self.generation
            try:
                with self.io:
                    if not self.stop.is_set():self._drain()
            except Exception as error:
                with self.lock:
                    if generation==self.generation:self.source_error(error)

    def begin(self,start):
        if start not in('fresh','continue'):raise ValueError('choose fresh or retained memory')
        with self.io:
            with self.lock:
                if self.error:raise ValueError('memory unavailable: '+self.error)
                if start=='fresh':
                    self.pending.clear();self.sampler=RealTrajectorySampler(stride=self.sampling_stride)
            if start=='continue':self._drain()
            with self.lock:self.start=start

    def finish_run(self):
        with self.io:
            self._drain()
            with self.lock:self.sampler.pause()

    def rebind_hold(self):
        with self.io:
            self._drain()
            with self.lock:
                self.mode='HOLD';self.control_sequence=None;self.sampler.pause()

    def reconfigure(self,client):
        with self.io:
            with self.lock:
                if self.error or self.mode not in ('HOLD','STOPPED'):raise ValueError('memory sampling update requires HOLD')
            self._drain()
            return self.client.reconfigure_memory(client)

    def infer(self,request,*,prepared_input=None):
        with self.io:
            with self.lock:
                if self.error:raise ValueError('memory unavailable: '+self.error)
                start=self.start;generation=self.generation
                self.sampler.bootstrap(request['observation'])
            self._drain(until=request['observation']['source_time'])
            result=self.client.infer_chunk(dict(request,memory_start=start),epoch=request['epoch'],
                clock_domain=request['clock_domain'],valid_until_source_time=request['valid_until_source_time'],prepared_input=prepared_input)
            with self.lock:
                if generation!=self.generation:raise ValueError('memory was cleared while the result was pending')
                self.audit=deepcopy(result['audit']['memory'])
            return result

    def clear(self,request_id,epoch):
        with self.lock:
            self.generation+=1;self.pending.clear();self.sampler=RealTrajectorySampler(stride=self.sampling_stride)
        with self.io:
            result=self.client.memory_clear(request_id,epoch=epoch,clock_domain=self.clock_domain)
            with self.lock:
                self.audit=deepcopy(result['audit']['memory']);self.last_manual=None;self.error=None
        return result

    def snapshot(self):
        with self.lock:
            return dict(self.audit or dict(empty=True,observations=0,manual_observations=0,infer_calls=0),
                available=not self.stop.is_set()and not self.error,error=self.error,
                pending_manual_observation=bool(self.pending),next_start=self.start,
                phase=self.mode,control_epoch=self.epoch,mode_events=self.mode_events,
                manual_offers=self.offered,effective_samples=self.sampler.samples,
                sampled_blocks=self.sampler.blocks,source_gaps=self.sampler.gaps,
                pending_real_contexts=len(self.pending),control_sequence=self.control_sequence,
                control_event_gaps=self.control_event_gaps,last_control_gap=deepcopy(self.last_control_gap),
                revision=self.client.memory_revision)

    def close(self):
        self.stop.set();self.wake.set()
        with self.lock:self.generation+=1;self.pending.clear()
        self.thread.join(5)
        if self.thread.is_alive():raise RuntimeError('memory observer still in flight')


def model_service(registry):
    from .inference_transport import ModelService
    if 'catalog_root'in registry:
        from .inference_catalog import RegisteredCatalog
        catalog=RegisteredCatalog(registry['catalog_root'])
        return ModelService(catalog.initial_client(registry),catalog=catalog.catalog,catalog_rows=catalog.rows,
            client_factory=catalog.client,client_refresh=catalog.refresh_client)
    return ModelService(Client(registry,descriptor(registry)))


def _load_policy(registry,paths):
    import sys,torch,importlib
    # Resolve the pinned encoder's lazy dependencies before loading large tensors.
    for name in ('einops','safetensors.torch'):importlib.import_module(name)
    sys.path.insert(0,str(paths['source']))
    from world_memory.identity import source_identity
    if source_identity()!=json.loads(paths['source_manifest'].read_text()):raise ValueError('world_memory source differs from validated producer')
    from world_memory.config import from_policy_architecture
    from world_memory.models.world import WorldActionModel
    from world_memory.normalization import Normalization
    from world_memory.visual_codec import build_visual_codec
    from world_memory.text_cache import load_prompts
    from world_memory.inference import Policy
    payload=torch.load(paths['checkpoint'],map_location='cpu',weights_only=False,mmap=True)
    if payload.get('format')!='world_memory_policy'or payload.get('step')!=registry['step']:
        raise ValueError('final world-memory policy identity differs')
    cfg=from_policy_architecture(payload['architecture']);d=cfg.data
    if (d.action_dim!=14 or d.state_dim!=14 or d.action_horizon!=32
            or type(d.execute_actions) is not int or not 1<=d.execute_actions<=d.action_horizon
            or d.relative_action_dims!=0 or list(d.views)!=['main','left','right']or d.image_size!=256):
        raise ValueError('unsupported temporal native14/camera/action contract')
    declared=descriptor(registry).get('training_execute_actions',32)
    if d.execute_actions!=declared:raise ValueError('checkpoint and training execution length differ')
    cfg.inference.denoising_steps=descriptor(registry)['inference_options']['denoise_steps']
    cfg.codec.checkpoint=str(paths['vae']);cfg.codec.phoenix_encoder=str(paths['phoenix_encoder'])
    cfg.codec.csvae_checkpoint=str(paths['csvae_checkpoint']);cfg.data.cache_dir=str(paths['text_index'].parent)
    cfg.text.online=False;cfg.text.packed_cache=False
    index=json.loads(paths['text_index'].read_text());instruction=registry['instruction']
    if set(index['records'])!={instruction}or paths['text_index'].parent/index['records'][instruction]['file']!=paths['text_features']:
        raise ValueError('exact frozen instruction and cache feature required')
    text=load_prompts(cfg,[instruction],'cpu',identity=registry['cached_text_identity'])
    normalization=Normalization(payload['normalization']).validate(cfg)
    with torch.device('meta'):model=WorldActionModel(cfg)
    model.load_state_dict(payload['model'],strict=True,assign=True)
    model.to(device='cuda:0',dtype=torch.bfloat16).eval().requires_grad_(False)
    codec=build_visual_codec(cfg).to('cuda:0')
    return Policy(model,codec,text,normalization,cfg,'cuda:0'),instruction


def _worker(registry,connection):
    import os,random,torch
    def seed(value):
        random.seed(value);np.random.seed(value%(2**32));torch.manual_seed(value)
    try:
        _,paths=verify_registry(registry);policy,instruction=_load_policy(registry,paths)
        memory=PhaseMemory(policy,instruction,seed);memory.clear();bundle=descriptor(registry)
        connection.send(dict(ready=True,pid=os.getpid(),adapter_source_sha256=_SOURCE_SHA256))
        while True:
            request=connection.recv()
            if request is None:return
            if request.get('operation')=='sampling':
                try:
                    options=wam.sampling_options(bundle,request)
                    policy.config.inference.denoising_steps=options['denoise_steps']
                    bundle['inference_options']=options
                    connection.send(dict(ok=True,worker_pid=os.getpid(),worker_identity=worker_identity(bundle)))
                except (ValueError,TypeError,KeyError)as error:connection.send(dict(ok=False,error=str(error)))
                continue
            began=time.monotonic();result=memory.infer(request);torch.cuda.synchronize();finished=time.monotonic()
            result['audit']['timing']=dict(total_wall_s=finished-began,engine_wall_s=finished-began,
                measurement='entire temporal Policy observe/act including visual encoding, memory and CUDA synchronization')
            result['audit']['denoise_steps']=policy.config.inference.denoising_steps
            if request['warmup']:memory.clear();result['audit']['memory']['warmup_state_cleared']=True
            connection.send(dict(ok=True,result=result))
    except BaseException as error:
        import traceback
        traceback.print_exc()
        try:connection.send(dict(error=str(error)))
        except (OSError,EOFError,BrokenPipeError):pass
    finally:connection.close()


if __name__=='__main__':
    import argparse,torch
    from multiprocessing.connection import Connection
    from .arx_output_guard import parent_death
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--worker',type=Path,required=True);parser.add_argument('--pipe-fd',type=int,required=True)
    parser.add_argument('--parent-pid',type=int,required=True);args=parser.parse_args();parent_death(args.parent_pid)
    with torch.inference_mode():_worker(json.loads(args.worker.read_text()),Connection(args.pipe_fd))
