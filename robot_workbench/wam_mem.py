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
MEMORY_PROTOCOL='workbench.wam-memory-phase.v5'
MODEL_CONTROL_HZ=30
MODEL_STEP_TIME_S=1/MODEL_CONTROL_HZ
MANUAL_STEP_TIME_S=.03
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
        raise ValueError('explicit WAM-MEM v5 memory context required')
    return value


def validate_memory_sample(value, observation_time, *, required=False):
    if value is None:
        if required:raise ValueError('real sampled context metadata required')
        return None
    keys={'states14','actions14','state_times','action_times','mode','effective_index',
          'action_semantics','valid_actions','executed_action_condition','node'}
    if (not isinstance(value,dict) or set(value)!=keys
            or value['mode']not in ('INITIAL','MODEL_EXECUTING','TEACH')
            or value['action_semantics']!='materialized_next_observation'
            or value['executed_action_condition']is not False
            or type(value['effective_index'])is not int or value['effective_index']<0
            or type(value['valid_actions'])is not int or not 0<=value['valid_actions']<=32):
        raise ValueError('explicit real-observation node contract differs')
    validate_memory_node(value['node'])
    if value['mode'] not in ('INITIAL', 'MODEL_EXECUTING' if value['node']['kind']=='MODEL' else 'TEACH'):
        raise ValueError('memory observation interface and mode differ')
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


def validate_memory_node(value):
    if (not isinstance(value,dict) or set(value)!={'kind','step','model_time_s','target_source_time'}
            or value['kind'] not in ('MODEL','TEACH')
            or type(value['step']) is not int or value['step']<0
            or any(type(value[k])not in(int,float)or not math.isfinite(value[k])or value[k]<0
                   for k in ('model_time_s','target_source_time'))):
        raise ValueError('explicit model/manual step, model time and real target time required')
    return value


def validate_memory_context(value):
    if (not isinstance(value,dict)or set(value)!={'revision','time_s'}
            or type(value['revision'])is not int or value['revision']<0
            or type(value['time_s'])not in(int,float)or not math.isfinite(value['time_s'])or value['time_s']<0):
        raise ValueError('explicit MEM snapshot and logical model time required')
    return value


def observation_sample(kind,step,model_time_s,target_source_time):
    node=validate_memory_node(dict(kind=kind,step=step,model_time_s=model_time_s,target_source_time=target_source_time))
    return dict(states14=[],actions14=[],state_times=[],action_times=[],
                mode='MODEL_EXECUTING'if kind=='MODEL'else'TEACH',effective_index=step,
                action_semantics='materialized_next_observation',valid_actions=0,
                executed_action_condition=False,node=node)


def sampling_policy(bundle,manual_stride=None):
    stride=bundle.get('training_execute_actions',32)
    return dict(protocol=MEMORY_PROTOCOL,model_submission_stride=stride,
        manual_submission_stride=stride if manual_stride is None else manual_stride,model_step_time_s=MODEL_STEP_TIME_S,
        manual_step_time_s=MANUAL_STEP_TIME_S)


def validate_state_reference(value):
    from .ros_data import StreamRef
    if not isinstance(value,dict)or set(value)!=set(StreamRef.__dataclass_fields__):
        raise ValueError('original measured state sample reference required')
    StreamRef(**value)
    return value


class MemoryRequestRejected(ValueError):
    """A fenced or duplicate request leaves the loaded policy and MEM intact."""


class PhaseMemory:
    """Explicit real-observation commits and bounded immutable query contexts."""
    def __init__(self,policy,instruction,seed):
        self.policy,self.instruction,self.seed=policy,instruction,seed
        self.phase=None;self.revision=0;self.results=OrderedDict()
        self._reset()

    def _reset(self):
        self.policy.reset(control_frequency=30)
        self.origin=self.last_time=self.last_model_time=None
        self.calls=self.observations=self.manual_observations=0
        self.context_revision=0
        self.contexts=OrderedDict();self.nodes=OrderedDict()
        self.last_steps={'MODEL':-1,'TEACH':-1}

    def clear(self):
        self._reset();self.phase=None;self.revision=0;self.results.clear()

    def snapshot(self):
        return dict(protocol=MEMORY_PROTOCOL,revision=self.revision,
            context_revision=self.context_revision,empty=self.observations==0,
            observations=self.observations,manual_observations=self.manual_observations,infer_calls=self.calls,
            source_time=self.last_time,model_time_s=self.last_model_time,origin_source_time=self.origin,
            executed_action_context=False,predictions_used_as_actions=0,
            action_context_reason='materialized_next_observation training masks historical executed controls',
            write_policy='explicit_observation_nodes',prediction_commits_memory=False)

    @staticmethod
    def _observation_identity(observation):
        digest=hashlib.sha256()
        state=np.asarray(observation['model_state14'],dtype=np.float32)
        digest.update(state.tobytes())
        for name in wam.CAMS:
            image=observation['images'][name]
            digest.update(str(image.shape).encode());digest.update(image.tobytes())
        digest.update(json.dumps(observation.get('frame_ids',{}),sort_keys=True,separators=(',',':')).encode())
        digest.update(json.dumps(observation.get('state_reference'),sort_keys=True,separators=(',',':')).encode())
        return digest.hexdigest()

    def infer(self,request):
        phase_context=validate_phase(request['memory'])
        operation=request.get('memory_operation','infer')
        if operation not in ('infer','observe','clear'):raise MemoryRequestRejected('unknown memory operation')
        if operation=='observe'and phase_context['start']!='continue':
            raise MemoryRequestRejected('explicit observations preserve the acquired memory session')
        phase=(phase_context['session_id'],phase_context['clock_domain'],phase_context['epoch'])
        same_session=self.phase is not None and phase[0]==self.phase[0]
        if same_session and phase_context['start']=='fresh'and operation!='clear':
            raise MemoryRequestRejected('use explicit memory clear to reset an acquired session')
        if same_session and phase[1]!=self.phase[1]:
            raise MemoryRequestRejected('memory clock changed; acquire a new session')
        if same_session and phase[2]<self.phase[2]:raise MemoryRequestRejected('obsolete memory phase')
        if not same_session:
            if phase_context['revision']!=0:raise MemoryRequestRejected('new memory lease starts at revision zero')
            self._reset();self.revision=0;self.results.clear()
        if phase_context['revision']!=self.revision:raise MemoryRequestRejected('obsolete memory revision')
        identity=request['request_id']
        if not isinstance(identity,str)or not 0<len(identity)<=128:raise MemoryRequestRejected('bounded request identity required')
        if operation=='clear':
            self._reset();self.revision+=1;self.results.clear();self.phase=phase
            return dict(request_id=identity,audit=dict(memory=self.snapshot(),operation='clear',command_publication=False))

        observation=request['observation'];stamp=observation['source_time']
        validate_input(observation['model_state14'],observation['images'])
        if not request.get('warmup'):
            reference=validate_state_reference(observation.get('state_reference'))
            if reference['source_clock_id']!=phase_context['clock_domain']:
                raise MemoryRequestRejected('state sample and memory clock domains differ')
        if type(stamp)not in(int,float)or not math.isfinite(stamp)or stamp<0:
            raise MemoryRequestRejected('finite original observation timestamp required')
        context=validate_memory_context(request.get('memory_context'))
        sample=request.get('memory_sample')
        validate_memory_sample(sample,stamp,required=operation=='observe')
        observed_identity=self._observation_identity(observation)
        signature_data=[phase,self.revision,operation,stamp,observed_identity,sample]
        if operation=='infer':
            signature_data += [context,request.get('seed'),self.policy.config.inference.denoising_steps]
        elif not math.isclose(sample['node']['model_time_s'],context['time_s'],abs_tol=1e-9):
            raise MemoryRequestRejected('observation node and model time differ')
        signature=hashlib.sha256(json.dumps(signature_data,sort_keys=True,separators=(',',':')).encode()).hexdigest()
        key=(phase,self.revision,identity)
        if key in self.results:
            previous,result=self.results[key]
            if previous!=signature:raise MemoryRequestRejected('memory request identity reused with different data')
            return deepcopy(result)
        rgb={role:observation['images'][name]for role,name in zip(('main','left','right'),wam.CAMS)}
        state=np.asarray(observation['model_state14'],dtype=np.float32)
        model_time=context['time_s']

        if request.get('warmup')and not self.contexts:
            sample=observation_sample('MODEL',0,model_time,stamp)
            self._commit(sample,observation,observed_identity,rgb,state,model_time)
            context=dict(context,revision=self.context_revision)

        if operation=='observe':
            node=sample['node'];node_key=(node['kind'],node['step'])
            node_signature=hashlib.sha256(json.dumps([sample,stamp,observed_identity],sort_keys=True).encode()).hexdigest()
            if node_key in self.nodes:
                previous,result=self.nodes[node_key]
                if previous!=node_signature:raise MemoryRequestRejected('memory node reused with different observation')
                self.phase=phase
                return dict(deepcopy(result),request_id=identity)
            if node['step']<=self.last_steps[node['kind']]:
                raise MemoryRequestRejected('memory node expired or regressed')
            if context['revision']!=self.context_revision:
                raise MemoryRequestRejected('observation was based on an obsolete MEM snapshot')
            if not math.isclose(node['model_time_s'],model_time,abs_tol=1e-9):
                raise MemoryRequestRejected('observation node and model time differ')
            if self.last_time is not None and stamp<self.last_time:
                raise MemoryRequestRejected('real observation source time regressed')
            if self.contexts and stamp==self.last_time and observed_identity==self.contexts[self.context_revision]['observation_identity']:
                raise MemoryRequestRejected('real observation already committed at another node')
            if self.last_model_time is not None and model_time<=self.last_model_time:
                raise MemoryRequestRejected('explicit observation model time must advance')
            self._commit(sample,observation,observed_identity,rgb,state,model_time)
            result=dict(request_id=identity,audit=dict(memory=self.snapshot(),operation='observe',
                command_publication=False,memory_sample=deepcopy(sample),
                state_reference=deepcopy(observation.get('state_reference')),
                source_references=deepcopy(observation.get('frame_ids',{}))))
        else:
            frozen=self.contexts.get(context['revision'])
            if frozen is None:raise MemoryRequestRejected('MEM context snapshot missing or expired; submit an observation first')
            if model_time<frozen['model_time']:
                raise MemoryRequestRejected('prediction cannot read future MEM')
            if type(request['seed'])is not int or not 0<=request['seed']<2**63:
                raise MemoryRequestRejected('uint63 sampling seed required')
            effective_seed=(request['seed']+self.calls)%(2**63);self.seed(effective_seed)
            query=copy(frozen['policy']);query.times=list(query.times);query.executed=[];query.pending=None
            if (stamp==frozen['source_time']and observed_identity==frozen['observation_identity']
                    and model_time==frozen['model_time']):
                query.times=list(frozen['previous_times']);cache=frozen['cache']
            else:cache=query._context(rgb,model_time,self.instruction,state)
            query._context=lambda *args,**kwargs:cache
            # Read-only queries can share a committed logical time while real
            # camera/state samples advance during WAIT or HOLD.
            query.pending=(rgb,model_time)
            prediction=query.act(self.instruction,state)['actions']
            if hasattr(prediction,'detach'):prediction=prediction.detach().float().cpu().numpy()
            action=np.asarray(prediction,dtype=np.float32)
            if action.shape!=(32,14)or not np.isfinite(action).all():
                raise ValueError('WAM-MEM requires finite absolute32x14 output')
            self.calls+=1
            result=dict(request_id=identity,action14=action,audit=dict(memory=self.snapshot(),
                operation='infer',command_publication=False,seed=effective_seed,requested_seed=request['seed'],
                context_revision=context['revision'],model_time_s=model_time,
                query_source_time=stamp,state_reference=deepcopy(observation.get('state_reference')),
                source_references=deepcopy(observation.get('frame_ids',{}))))
        self.phase=phase
        result['audit']['memory'].update(session_id=phase[0],clock_domain=phase[1],phase_epoch=phase[2],
            used_context_revision=context['revision']if operation=='infer'else self.context_revision)
        if operation=='observe':
            self.nodes[node_key]=(node_signature,deepcopy(result))
            while len(self.nodes)>128:self.nodes.popitem(last=False)
        self.results[key]=(signature,deepcopy(result))
        while len(self.results)>32:self.results.popitem(last=False)
        return result

    def _commit(self,sample,observation,identity,rgb,state,model_time):
        previous_times=list(self.policy.times)
        cache=self.policy._context(rgb,model_time,self.instruction,state)
        self.policy.memory=cache.memory;self.policy.times.append(model_time)
        self.policy.pending=None;self.policy.executed.clear()
        self.context_revision+=1;self.observations+=1
        if sample['node']['kind']=='TEACH':self.manual_observations+=1
        self.last_time=observation['source_time'];self.last_model_time=model_time
        if self.origin is None:self.origin=self.last_time
        node=sample['node'];self.last_steps[node['kind']]=node['step']
        frozen=copy(self.policy);frozen.times=list(self.policy.times)
        self.contexts[self.context_revision]=dict(policy=frozen,cache=cache,previous_times=previous_times,
            source_time=self.last_time,model_time=model_time,observation_identity=identity)
        # The current and preceding contexts cover in-flight RTC work without
        # retaining an unbounded set of GPU attention caches.
        while len(self.contexts)>2:self.contexts.popitem(last=False)


def worker_request(request):
    warm=request.get('_warmup')is True
    memory=(dict(protocol=MEMORY_PROTOCOL,session_id='internal-warmup',epoch=0,clock_domain='warmup',start='fresh',revision=0)
        if warm else request.get('memory'))
    validate_phase(memory)
    operation=request.get('memory_operation','infer')
    result=dict(request_id=request['request_id'],seed=request.get('seed',0),memory=deepcopy(memory),
        memory_context=dict(revision=0,time_s=0.)if warm else deepcopy(request.get('memory_context')),warmup=warm,memory_operation=operation,
        memory_sample=deepcopy(request.get('memory_sample')))
    if operation!='clear':result['observation']={k:request['observation'][k]for k in ('model_state14','images','source_time','frame_ids','state_reference')if k in request['observation']}
    return result


def Client(registry,bundle):
    import sys
    return wam.Client(registry,bundle,implementation=sys.modules[__name__])


class LogicalMemoryClock:
    """Owner-only progress. Physical retiming changes target stamps, never steps."""
    def __init__(self):
        self.mode='HOLD';self.last_source=None;self.segment=None;self.segment_index=0
        self.steps={'MODEL':0,'TEACH':0};self.manual_elapsed=0.;self.manual_anchor=None
        self.nodes=deque(maxlen=128)

    @property
    def model_time(self):return self.steps['MODEL']/MODEL_CONTROL_HZ+self.steps['TEACH']*MANUAL_STEP_TIME_S

    def _node(self,kind,stamp):
        self.steps[kind]+=1
        self.nodes.append(dict(kind=kind,step=self.steps[kind],model_time_s=self.model_time,target_source_time=stamp))

    def advance(self,stamp):
        if type(stamp)not in(int,float)or not math.isfinite(stamp):raise ValueError('finite memory progress source time required')
        if self.last_source is not None and stamp<self.last_source:return
        if self.mode=='MODEL'and self.segment is not None:
            points=self.segment['source_point_times_s'];origin=self.segment['start_source_s']
            while self.segment_index<len(points)and origin+points[self.segment_index]<=stamp+1e-9:
                self._node('MODEL',origin+points[self.segment_index]);self.segment_index+=1
        elif self.mode=='TEACH':
            elapsed=self.manual_elapsed+max(0.,stamp-self.manual_anchor)
            complete=math.floor((elapsed+1e-9)/MANUAL_STEP_TIME_S)
            while self.steps['TEACH']<complete:
                target=self.manual_anchor+(self.steps['TEACH']+1)*MANUAL_STEP_TIME_S-self.manual_elapsed
                self._node('TEACH',target)
        self.last_source=stamp

    def set_mode(self,mode,stamp):
        if self.last_source is not None:stamp=max(stamp,self.last_source)
        self.advance(stamp)
        if self.mode=='TEACH':self.manual_elapsed+=max(0.,stamp-self.manual_anchor)
        self.mode=mode
        self.manual_anchor=stamp if mode=='TEACH'else None
        if mode!='MODEL':self.segment=None

    def start_segment(self,segment,stamp):
        self.set_mode('MODEL',stamp)
        self.segment=deepcopy(segment);self.segment_index=0

    def snapshot(self):
        return dict(model_steps=self.steps['MODEL'],manual_steps=self.steps['TEACH'],
            model_time_s=self.model_time,nodes=list(self.nodes))


class LogicalNodeSampler:
    """Default node scheduling, separate from the public arbitrary-node APIs."""
    def __init__(self,*,model_stride=32,manual_stride=None):
        if manual_stride is None:manual_stride=model_stride
        if any(type(n)is not int or n<1 for n in(model_stride,manual_stride)):
            raise ValueError('positive logical memory submission strides required')
        self.strides={'MODEL':model_stride,'TEACH':manual_stride}
        self.base=None;self.time_origin=0.;self.last={'MODEL':0,'TEACH':0};self.blocks=0
        self.model_time_s=0.;self.samples=0;self.gaps=0

    def bootstrap(self,observation):
        if self.base is not None:return None
        activity=observation['memory_sampling']
        self._validate(activity)
        self.base={'MODEL':activity['model_steps'],'TEACH':activity['manual_steps']}
        self.time_origin=activity['model_time_s']
        kind='TEACH'if activity['phase']=='TEACH'else'MODEL'
        self.blocks+=1
        return observation_sample(kind,0,0.,observation['source_time'])

    def offer(self,activity):
        if self.base is None:return []
        self._validate(activity)
        counts={'MODEL':activity['model_steps']-self.base['MODEL'],'TEACH':activity['manual_steps']-self.base['TEACH']}
        if any(counts[k]<self.last[k]for k in counts):raise ValueError('Owner logical memory progress regressed')
        self.model_time_s=activity['model_time_s']-self.time_origin
        selected=[]
        for raw in activity['nodes']:
            node=validate_memory_node(raw);kind=node['kind'];step=node['step']-self.base[kind]
            if step>self.last[kind]and step%self.strides[kind]==0:
                expected=(self.last[kind]//self.strides[kind]+1)*self.strides[kind]
                if step!=expected:raise ValueError('logical memory node history expired; rebind or clear')
                selected.append(observation_sample(kind,step,node['model_time_s']-self.time_origin,node['target_source_time']))
                self.last[kind]=step;self.blocks+=1
        if any(counts[k]>=self.last[k]+self.strides[k]for k in counts):
            raise ValueError('Owner logical memory node missing')
        self.samples=sum(counts.values())
        return selected

    @staticmethod
    def _validate(activity):
        if (activity.get('schema')!='workbench.memory-activity.v2'
                or activity.get('phase')not in('MODEL_EXECUTING','TEACH','WAIT','HOLD')
                or any(type(activity.get(k))is not int or activity[k]<0 for k in('model_steps','manual_steps'))
                or type(activity.get('model_time_s'))not in(int,float)
                or not math.isfinite(activity['model_time_s'])
                or not math.isclose(activity['model_time_s'],activity['model_steps']/MODEL_CONTROL_HZ+activity['manual_steps']*MANUAL_STEP_TIME_S,abs_tol=1e-9)
                or not isinstance(activity.get('nodes'),list)or len(activity['nodes'])>128):
            raise ValueError('explicit Owner model/manual memory progress required')


class MemorySession:
    """Real observations selected at explicit nodes and one serialized writer."""
    def __init__(self,client,adapter,clock_domain,*,resolve=None,select=None,record=None,manual_stride=None):
        self.client,self.adapter,self.clock_domain=client,adapter,clock_domain
        self.resolve,self.select,self.record=resolve,select,record
        self.lock=threading.RLock();self.io=threading.Lock();self.wake=threading.Event();self.stop=threading.Event()
        self.prepare_io=threading.Lock()
        self.mode='HOLD';self.epoch=-1;self.generation=0;self.control_sequence=None
        self.control_event_gaps=0;self.last_control_gap=None
        policy=sampling_policy(client.model_bundle,manual_stride)
        self.sampling_stride=policy['model_submission_stride'];self.manual_stride=policy['manual_submission_stride']
        self.sampler=LogicalNodeSampler(model_stride=self.sampling_stride,manual_stride=self.manual_stride)
        self.pending=deque();self.offered=0;self.mode_events=0
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
            self.control_sequence=sequence
            return True

    def source_error(self,error):
        with self.lock:self.error=str(error);self.pending.clear()
        from .pipeline_timing import timing
        timing.fault('WAM-MEM: '+str(error),epoch=self.epoch)

    def _enqueue(self,metadata,observation,epoch,run_id=None):
        if len(self.pending)>=8:raise ValueError('real memory writer queue full; no observations silently dropped')
        self.pending.append(dict(metadata=metadata,observation=observation,epoch=epoch,
            generation=self.generation,request_id=secrets.token_hex(16),prepared=None,
            run_id=observation.get('run_id')if observation is not None else run_id))
        self.wake.set()

    def offer(self,observation,epoch):
        activity=observation.get('memory_sampling')
        if (not isinstance(activity,dict)or activity.get('schema')!='workbench.memory-activity.v2'
                or activity.get('run_id')!=observation.get('run_id')):return
        with self.lock:
            if self.stop.is_set()or self.error or epoch<self.epoch:return
            self.epoch=epoch
            self.mode={'MODEL_EXECUTING':'MODEL','WAIT':'MODEL','TEACH':'TEACH','HOLD':'HOLD'}[activity['phase']]
            try:
                if self.sampler.base is None and activity['phase']=='TEACH':
                    self._enqueue(self.sampler.bootstrap(observation),observation,epoch)
                for metadata in self.sampler.offer(activity):self._enqueue(metadata,None,epoch,observation['run_id'])
            except Exception as error:
                self.source_error(error)
                return
            self.offered=self.sampler.samples

    def _prepare_pending(self,until=None):
        with self.prepare_io:self._resolve_pending(until)

    def _resolve_pending(self,until):
        # Resolve retained RGB outside model IO, so a long prediction cannot
        # evict selected source bytes from the short ROS image cache.
        with self.lock:rows=list(self.pending)
        for row in rows:
            if until is not None and row['metadata']['node']['target_source_time']>until:return
            if row['prepared']is not None:continue
            observation=row['observation']
            if observation is None:
                if self.select is None:raise ValueError('explicit target observation selector required')
                observation=self.select(row['metadata']['node']['target_source_time'],row['epoch'],row['run_id'])
            if self.resolve is not None:observation=self.resolve(observation)
            prepared=self.adapter.model_input(observation)
            with self.lock:
                if row['generation']==self.generation:
                    row['observation']=observation;row['prepared']=prepared

    def _drain(self,until=None):
        self._prepare_pending(until)
        while True:
            with self.lock:
                if not self.pending:return
                row=self.pending[0];observation=row['observation']
                if until is not None and row['metadata']['node']['target_source_time']>until:return
                self.pending.popleft()
                if row['generation']!=self.generation:continue
            request=dict(request_id=row['request_id'],seed=0,run_id=observation.get('run_id'),epoch=row['epoch'],
                observation=observation,applied_history=np.empty((0,14)),memory_sample=row['metadata'],
                memory_context=dict(revision=self.client.memory_context_revision,time_s=row['metadata']['node']['model_time_s']),
                memory=dict(protocol=MEMORY_PROTOCOL,session_id=self.client.session_id,clock_domain=self.clock_domain,
                    epoch=row['epoch'],revision=self.client.memory_revision,start='continue'))
            def call():return self.client.infer(request,prepared_input=row['prepared'],memory_only=True)
            result=self.record(request,row['prepared'],call)if self.record is not None else call()
            with self.lock:
                if row['generation']==self.generation:
                    self.audit=deepcopy(result['audit']['memory'])
                    if row['metadata']['node']['kind']=='TEACH':self.last_manual=observation['source_time']

    def _work(self):
        while not self.stop.is_set():
            self.wake.wait(.01);self.wake.clear()
            with self.lock:generation=self.generation
            try:
                self._prepare_pending()
                if self.io.acquire(blocking=False):
                    try:
                        if not self.stop.is_set():self._drain()
                    finally:self.io.release()
            except Exception as error:
                from .observation_pairing import PairingPending
                if isinstance(error,PairingPending):continue
                with self.lock:
                    if generation==self.generation:self.source_error(error)

    def begin(self,start):
        if start not in('fresh','continue'):raise ValueError('choose fresh or retained memory')
        with self.io:
            with self.lock:
                if self.error:raise ValueError('memory unavailable: '+self.error)
                if start=='fresh'and self.client.memory_context_revision:
                    raise ValueError('use explicit memory clear before a fresh start')
            self._drain()
            with self.lock:self.start='continue'

    def finish_run(self):
        with self.io:self._drain()

    def rebind_hold(self):
        with self.io:
            self._drain()
            with self.lock:self.mode='HOLD';self.control_sequence=None

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
                generation=self.generation
                initial=self.sampler.bootstrap(request['observation'])
                if initial is not None:self._enqueue(initial,request['observation'],request['epoch'])
            # The paired state can precede the earliest RGB by a few ms. Both
            # belong to this observation; its initial MEM node must be committed.
            self._drain(until=max(request['observation']['source_time'],
                request['observation'].get('feedback_sample_time_s',request['observation']['source_time'])))
            activity=request['observation']['memory_sampling']
            logical_time=activity['model_time_s']-self.sampler.time_origin
            result=self.client.infer_chunk(dict(request,memory_context=dict(
                revision=self.client.memory_context_revision,time_s=logical_time)),epoch=request['epoch'],
                clock_domain=request['clock_domain'],valid_until_source_time=request['valid_until_source_time'],prepared_input=prepared_input)
            with self.lock:
                if generation!=self.generation:raise ValueError('memory was cleared while the result was pending')
                self.audit=deepcopy(result['audit']['memory'])
            return result

    def clear(self,request_id,epoch):
        with self.lock:
            self.generation+=1;self.pending.clear()
            self.sampler=LogicalNodeSampler(model_stride=self.sampling_stride,manual_stride=self.manual_stride)
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
                **sampling_policy(self.client.model_bundle,self.manual_stride),
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
            began=time.monotonic()
            try:result=memory.infer(request)
            except MemoryRequestRejected as error:
                connection.send(dict(ok=True,result=dict(request_id=request['request_id'],memory_rejected=str(error))))
                continue
            torch.cuda.synchronize();finished=time.monotonic()
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
