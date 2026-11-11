"""WAM model-domain preparation. No SDK, ROS source, model load or GPU on import."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import re

import numpy as np

_SOURCE_SHA256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()

SCHEMA='workbench.wam.v2'
SOURCE_SCHEMA='workbench.wam.v3'
SCHEMAS=(SCHEMA,SOURCE_SCHEMA)
NATIVE_SCHEMA='workbench.wam-native-coordinates.v1'
HORIZON=32
HZ=30
CAMS=('base_0_rgb','left_wrist_0_rgb','right_wrist_0_rgb')
SLOTS=(0,1,2,3,4,5,16,40,41,42,43,44,45,56)
PAD=tuple(i for i in range(80)if i not in SLOTS)
TORCH_VERSION='2.14.1'
MODEL_TORCH_VERSION='2.14.1+cu130'
ARTIFACT_KINDS={'checkpoint','stats','context','model_config'}
STALL_SECONDS=3.
STALL_ARM_RANGE_RAD=.02
STALL_JAW_RANGE_FRACTION=.05
STALL_COVER_CHUNKS=2


class StallRecovery:
    """Bounded executed-source-step feedback, consumed from the existing control topic."""
    def __init__(self, maximum_opening_mm, *, action_hz=HZ, feedback_max_age_s=None):
        from collections import deque
        import math
        import threading
        from .execution_settings import FEEDBACK_TIMEOUT_S
        opening=np.asarray([maximum_opening_mm[side]for side in ('left','right')],dtype=float)
        if opening.shape!=(2,)or not np.isfinite(opening).all()or np.any(opening<=0):
            raise ValueError('calibrated per-side opening required for WAM stall detection')
        self.config=dict(window_steps=math.ceil(STALL_SECONDS*action_hz),action_hz=action_hz,
            arm_range_rad=STALL_ARM_RANGE_RAD,jaw_range_mm=(opening*STALL_JAW_RANGE_FRACTION).tolist(),
            jaw_range_basis='fraction_of_calibrated_opening',jaw_range_fraction=STALL_JAW_RANGE_FRACTION,
            cover_chunks=STALL_COVER_CHUNKS,feedback_max_age_s=FEEDBACK_TIMEOUT_S if feedback_max_age_s is None else feedback_max_age_s)
        self.samples=deque(maxlen=self.config['window_steps'])
        self.lock=threading.RLock();self.segment=None;self.step=0;self.sequence=None
        self.chunks=0;self.previous_seed=None;self.last_sample=None;self.last_decision=None
        self.reset_reason='new_run'

    def reset(self, reason):
        with self.lock:
            self.samples.clear();self.segment=None;self.step=0;self.chunks=0
            self.last_sample=None;self.reset_reason=reason;self.last_decision=None

    def event(self, name, value, sequence):
        with self.lock:
            if self.sequence is not None:
                if sequence<=self.sequence:return
                if sequence!=self.sequence+1:self.reset('control_event_gap')
            self.sequence=sequence
            if name=='mode_transition' and value.get('target')!='MODEL':
                self.reset('mode_transition');return
            if name=='normal_model_segment_started':
                if self.segment and self.step<self.segment['source_steps']:
                    self.reset('incomplete_segment')
                self.segment=deepcopy(value);self.step=0;self.last_sample=None
            if name!='feedback' or self.segment is None:return
            segment=self.segment;stamp=value['source_time']
            if self.last_sample is not None and stamp<=self.last_sample:return
            spacing=segment['execution_duration_s']/segment['source_steps']
            step=min(segment['source_steps'],int((stamp-segment['start_source_s']+1e-9)/spacing))
            if step<=self.step or step<1:return
            due=segment['start_source_s']+step*spacing
            vector=np.asarray(value['device_state14'],dtype=float)
            if vector.shape!=(14,) or not np.isfinite(vector).all():
                self.reset('invalid_feedback');return
            if step!=self.step+1 or stamp-due>self.config['feedback_max_age_s']:
                self.samples.clear();self.chunks=0;self.reset_reason='executed_step_feedback_gap'
            if stamp-due<=self.config['feedback_max_age_s']:
                self.samples.append((segment['request_id'],step,stamp,vector.copy()))
            self.step=step;self.last_sample=stamp

    def choose(self, seed):
        import secrets
        with self.lock:
            full=len(self.samples)==self.config['window_steps']
            ranges=np.ptp(np.asarray([row[3]for row in self.samples]),axis=0)if self.samples else None
            stalled=bool(full and np.all(ranges[[*range(6),*range(7,13)]]<self.config['arm_range_rad'])
                and np.all(ranges[[6,13]]<self.config['jaw_range_mm']))
            actual=seed;cover=False
            if stalled:
                cover=self.chunks%(self.config['cover_chunks']+1)<self.config['cover_chunks']
                actual=secrets.randbits(63)
                while actual in (seed,self.previous_seed):actual=secrets.randbits(63)
                self.previous_seed=actual;self.chunks+=1
            else:self.chunks=0
            decision=dict(enabled=True,stalled=stalled,head_black=cover,configured_seed=seed,actual_seed=actual,
                stalled_chunk=self.chunks,observed_steps=len(self.samples),config=deepcopy(self.config),
                observed_range_device14=None if ranges is None else ranges.tolist(),reset_reason=self.reset_reason,
                first_executed_step=None if not self.samples else list(self.samples[0][:3]),
                last_executed_step=None if not self.samples else list(self.samples[-1][:3]))
            self.last_decision=decision
            return deepcopy(decision)

    def snapshot(self):
        with self.lock:return dict(config=deepcopy(self.config),last_decision=deepcopy(self.last_decision))


def is_model_id(value):
    return isinstance(value,str)and not value.startswith('wam-mem-')and re.fullmatch(r'wam-[a-z0-9-]+-step[0-9]+',value)is not None


def validate_native_profile(profile):
    from .control_plan import _read_file
    from .sdk_quiesce import load_gripper_calibration
    fields={'schema','robot_id','sdk_module','gripper_calibration','camera_serials','gripper'}
    if not isinstance(profile,dict)or set(profile)!=fields or profile['schema']!=NATIVE_SCHEMA:
        raise ValueError('WAM native deployment binding required')
    serials=profile['camera_serials'];pin=profile['sdk_module']
    if (not isinstance(serials,dict)or set(serials)!={'head','left','right'}
            or any(not isinstance(v,str)or not v for v in serials.values())or len(set(serials.values()))!=3
            or not isinstance(pin,dict)or set(pin)!={'path','sha256'}
            or hashlib.sha256(_read_file(pin['path'],64*1024*1024)).hexdigest()!=pin['sha256']):
        raise ValueError('WAM pinned SDK/camera identity differs')
    native=load_gripper_calibration(profile['gripper_calibration'],robot_id=profile['robot_id'])
    if native is None or profile['gripper']!={'max_opening_mm':{s:native['sides'][s]['width_m']*1000 for s in ('left','right')}}:
        raise ValueError('WAM per-side jaw ranges differ from the pinned calibration')
    return native


def native_binding(deployment):
    from .inference_owner import digest
    from .sdk_quiesce import load_gripper_calibration
    native=load_gripper_calibration(deployment.get('native_gripper_calibration'),robot_id=deployment['robot_id'])
    if deployment.get('disabled_grippers')!=[]or native is None:
        raise ValueError('WAM requires both measured calibrated jaws')
    profile=dict(schema=NATIVE_SCHEMA,robot_id=deployment['robot_id'],sdk_module=deepcopy(deployment['sdk_quiesce']['module']),
        gripper_calibration=deepcopy(deployment['native_gripper_calibration']),
        camera_serials={s:v['serial']for s,v in deployment['cameras'].items()},
        gripper=dict(max_opening_mm={s:native['sides'][s]['width_m']*1000 for s in ('left','right')}))
    validate_native_profile(profile)
    return profile,dict(schema=NATIVE_SCHEMA,robot_id=profile['robot_id'],camera_serials=deepcopy(profile['camera_serials']),
        calibration_profile_digest=digest(profile),coordinate_qualification='recorded_native_state14',extrinsics_consumed=False)


def validate_inference_options(options,capabilities):
    if not isinstance(capabilities,dict)or set(capabilities)!={'denoise_steps'}:
        raise ValueError('model-package denoise capability required')
    bounds=capabilities['denoise_steps']
    if (not isinstance(bounds,dict)or set(bounds)!={'minimum','maximum','default'}
            or any(type(v)is not int for v in bounds.values())
            or not 1<=bounds['minimum']<=bounds['default']<=bounds['maximum']
            or bounds['maximum']-bounds['minimum']>=64):
        raise ValueError('bounded declared denoise capability required')
    if (not isinstance(options,dict)or set(options)!={'denoise_steps'}
            or type(options['denoise_steps'])is not int
            or not bounds['minimum']<=options['denoise_steps']<=bounds['maximum']):
        raise ValueError('denoise steps outside model-package capability')
    return deepcopy(options)


def validate_descriptor(value):
    fields={'schema','model_id','epoch','step','backend','artifacts','model_bundle_sha256','runtime','normalization_json',
            'inference_options','inference_capabilities'}
    sourced=isinstance(value,dict)and value.get('schema')==SOURCE_SCHEMA
    if sourced:fields.add('checkpoint_source')
    if (not isinstance(value,dict)or set(value)!=fields or value['schema']not in SCHEMAS or not is_model_id(value['model_id'])
            or value['epoch']is not None or type(value['step'])is not int or value['step']<=0
            or int(value['model_id'].rsplit('-step',1)[1])!=value['step']or value['backend']not in('torch','anyinfer')):
        raise ValueError('explicit generic WAM checkpoint/backend descriptor required')
    artifacts=value['artifacts']
    if (not isinstance(artifacts,dict)or set(artifacts)!=ARTIFACT_KINDS
            or any(not isinstance(v,dict)or set(v)!={'sha256'}or not(k=='checkpoint'and sourced and v['sha256']is None)
                and not re.fullmatch('[a-f0-9]{64}',str(v['sha256']))for k,v in artifacts.items())):
        raise ValueError('WAM checkpoint/config/normalization/context pins required')
    if sourced:
        source=value['checkpoint_source']
        if (not isinstance(source,dict)or set(source)!={'receipt_sha256','bytes'}or type(source['bytes'])is not int
                or source['bytes']<=0 or not re.fullmatch('[a-f0-9]{64}',str(source['receipt_sha256']))):
            raise ValueError('explicit source receipt and checkpoint size required when weight SHA256 is absent')
    raw=value['normalization_json']
    if not isinstance(raw,str)or len(raw.encode())>16384 or hashlib.sha256(raw.encode()).hexdigest()!=artifacts['stats']['sha256']:
        raise ValueError('WAM original normalization payload SHA256 differs')
    normalization(json.loads(raw))
    runtime=value['runtime']
    if (not isinstance(runtime,dict)or set(runtime)!={'adapter','engine_sha256','python_abi','torch_version'}
            or runtime['adapter']!=('wam_torch_client.v2'if value['backend']=='torch'else'wam_anyinfer_client.v2')or runtime['python_abi']!='cp312'
            or runtime['torch_version']!=MODEL_TORCH_VERSION
            or not re.fullmatch('[a-f0-9]{64}',str(runtime['engine_sha256']))
            or not re.fullmatch('[a-f0-9]{64}',str(value['model_bundle_sha256']))):
        raise ValueError('WAM pinned engine/Python/Torch runtime identity required')
    validate_inference_options(value['inference_options'],value['inference_capabilities'])
    return deepcopy(value)


def worker_identity(bundle):
    value=validate_descriptor(bundle)
    pins={k:v['sha256']for k,v in value['artifacts'].items()};native=value['backend']=='torch'
    return dict(status='ready',model_id=value['model_id'],epoch=None,step=value['step'],backend=value['backend'],
        checkpoint_sha256=pins['checkpoint'],stats_sha256=pins['stats'],context_sha256=pins['context'],
        model_config_sha256=pins['model_config'],model_bundle_sha256=value['model_bundle_sha256'],runtime=value['runtime'],
        action_horizon=HORIZON,history_steps=0,action_hz=HZ,inference_steps=value['inference_options']['denoise_steps'],
        strict_load=native,load_validation='strict_pytorch_checkpoint'if native else'pinned_anyinfer_engine',command_publication=False,
        **(dict(checkpoint_source=value['checkpoint_source'])if value['schema']==SOURCE_SCHEMA else{}))


def _torch():
    import torch
    if torch.__version__.split('+')[0]!=TORCH_VERSION:raise ValueError('WAM preprocessing Torch version mismatch; expected '+TORCH_VERSION)
    return torch


def normalization(value):
    if (not isinstance(value,dict)or set(value)!={'schema','gripper_unit','train_only','state','action'}
            or value['schema']!='phoenix.joint14.normalization.v1'or value['gripper_unit']!='m'
            or value['train_only']is not True):raise ValueError('explicit WAM train-only joint14 normalization required')
    result={}
    for kind in ('state','action'):
        fields=value[kind]
        if not isinstance(fields,dict)or set(fields)!={'mean','std'}:raise ValueError('WAM mean/std required')
        mean,std=(np.asarray(fields[k],dtype=np.float32)for k in ('mean','std'))
        if mean.shape!=std.shape or mean.shape!=(14,)or not np.isfinite([mean,std]).all()or np.any(std<=0):
            raise ValueError('fourteen finite WAM mean/std values and positive std required')
        result[kind]=(mean.copy(),std.copy())
    return result


def load_normalization(path,*,expected_sha256):
    raw=Path(path).read_bytes()
    if hashlib.sha256(raw).hexdigest()!=expected_sha256:raise ValueError('WAM normalization SHA256 changed')
    return normalization(json.loads(raw))


def bf16_values(values):
    """CPU float32 -> BF16 -> float32, round-to-nearest-even, without Torch import."""
    values=np.asarray(values,dtype=np.float32)
    if not np.isfinite(values).all():raise ValueError('finite BF16 input required')
    bits=values.view(np.uint32)
    rounded=(bits+np.uint32(0x7fff)+((bits>>16)&1))&np.uint32(0xffff0000)
    result=rounded.view(np.float32)
    if not np.isfinite(result).all():raise ValueError('BF16 input overflow')
    return result.copy()


def prepare_state(state14,stats):
    state=np.asarray(state14,dtype=np.float32)
    if state.shape!=(14,)or not np.isfinite(state).all():raise ValueError('finite WAM raw joint14 required')
    mean,std=stats['state']
    return bf16_values((state-mean)/std)


def image_bits(rgb):
    """Original RGB8 -> exact reference CPU bicubic256 WAN BF16 bits, HWC."""
    torch=_torch()
    import torch.nn.functional as F
    pixels=np.asarray(rgb)
    if pixels.dtype!=np.uint8 or pixels.ndim!=3 or pixels.shape[2]!=3 or not all(pixels.shape[:2]):
        raise ValueError('original WAM RGB8 HWC image required')
    image=torch.from_numpy(np.array(pixels,copy=True,order='C')).permute(2,0,1)[None].float()/255
    image=F.interpolate(image,size=(256,256),mode='bicubic',align_corners=False).clamp(0,1)
    wan=(image*2-1).to(torch.bfloat16)
    return wan.view(torch.uint16).numpy()[0].transpose(1,2,0).copy()


def prepare_images(images):
    if set(images)!=set(CAMS):raise ValueError('WAM requires head, left-wrist and right-wrist RGB sources')
    return {role:image_bits(images[role])for role in CAMS}


class Adapter:
    """WAM input preparation over the existing recorded-native14 unit mapper."""
    def __init__(self,position_limits,maximum_opening_mm):
        from .execution_settings import MODEL_JAW_OPEN_MARGIN_MM
        self.limits=np.asarray(position_limits,dtype=float).copy();self.stats=None
        openings=np.asarray([maximum_opening_mm[s]for s in ('left','right')]if isinstance(maximum_opening_mm,dict)
                            else [maximum_opening_mm]*2,dtype=float)
        if (self.limits.shape!=(14,2)or not np.isfinite(self.limits).all()or np.any(self.limits[:,0]>self.limits[:,1])
                or openings.shape!=(2,)or not np.isfinite(openings).all()
                or np.any(openings<=MODEL_JAW_OPEN_MARGIN_MM)):raise ValueError('valid native14 limits/opening required')
        self.limits[[6,13],1]=np.minimum(self.limits[[6,13],1],openings-MODEL_JAW_OPEN_MARGIN_MM)

    def bind_normalization(self,raw,*,expected_sha256):
        if not isinstance(raw,bytes)or hashlib.sha256(raw).hexdigest()!=expected_sha256:
            raise ValueError('WAM original normalization bytes must match the pinned SHA256')
        self.stats=normalization(json.loads(raw))
        _torch()  # Resolve the exact CPU dependency before selecting Run frames.

    @staticmethod
    def device_to_model(value):
        from .inference import native_device_to_model
        return native_device_to_model(value,clip_jaws=False)

    @staticmethod
    def model_to_device(value):
        from .inference import native_model_to_device
        return native_model_to_device(value)

    def decode(self,prediction,current_device14):
        from .inference import decode_native14
        return decode_native14(prediction,current_device14,self.limits)

    def model_input(self,observation):
        if self.stats is None:raise ValueError('bind verified WAM normalization before preparing model input')
        return prepare_state(self.device_to_model(observation['device_state14']),self.stats),prepare_images(observation['images'])


def prepare_text_context(value):
    """Preserve source tokens and masks while adapting the declared runtime extent."""
    torch=_torch();context=value['context'];mask=value['context_mask']
    if context.ndim==2:context=context.unsqueeze(0)
    if mask.ndim==1:mask=mask.unsqueeze(0)
    if (context.device.type!='cpu'or mask.device.type!='cpu'or context.dtype!=torch.bfloat16
            or context.ndim!=3 or context.shape[0]!=1 or context.shape[2]!=4096
            or not 0<context.shape[1]<=200 or mask.shape!=context.shape[:2]or mask.dtype!=torch.bool
            or not mask.any()or not torch.isfinite(context).all()):
        raise ValueError('supported frozen training text context and attention mask required')
    padded=torch.zeros((1,200,4096),dtype=context.dtype)
    padded_mask=torch.zeros((1,200),dtype=torch.bool)
    padded[:,:context.shape[1]]=context;padded_mask[:,:mask.shape[1]]=mask
    padded[~padded_mask]=0
    return dict(context=padded,context_mask=padded_mask)


def network_inputs(state14,images,context,context_mask,*,seed):
    """Reconstruct CPU network tensors from the recorded effective BF16 values."""
    torch=_torch()
    if type(seed)is not int or not 0<=seed<2**63:raise ValueError('WAM request seed must be uint63')
    state=np.asarray(state14,dtype=np.float32)
    if state.shape!=(14,)or not np.array_equal(state,bf16_values(state)):
        raise ValueError('WAM state must already contain its effective BF16 normalized values')
    if set(images)!=set(CAMS):raise ValueError('exact WAM image roles required')
    if any(np.asarray(v).dtype!=np.uint16 or np.asarray(v).shape!=(256,256,3)for v in images.values()):
        raise ValueError('WAM WAN images must be uint16 BF16 bit patterns of shape256x256x3')
    if (context.device.type!='cpu'or context_mask.device.type!='cpu'
            or context.shape!=(1,200,4096)or context.dtype!=torch.bfloat16 or not torch.isfinite(context).all()
            or context_mask.shape!=(1,200)or context_mask.dtype!=torch.bool
            or not context_mask.any()or torch.count_nonzero(context[~context_mask])):
        raise ValueError('pinned WAM CPU text context/mask layout differs')
    wan=torch.from_numpy(np.stack([images[k]for k in CAMS])).view(torch.bfloat16).permute(0,3,1,2).contiguous()[None,:,:,None]
    if not torch.isfinite(wan).all()or wan.min()<-1 or wan.max()>1:raise ValueError('invalid WAM WAN image values')
    mean=torch.tensor([.485,.456,.406],dtype=torch.bfloat16).view(1,1,3,1,1,1)
    std=torch.tensor([.229,.224,.225],dtype=torch.bfloat16).view(1,1,3,1,1,1)
    phoenix=((wan.float()*.5+.5).to(torch.bfloat16)-mean)/std
    proprio=torch.zeros((1,80),dtype=torch.float32);proprio[0,list(SLOTS)]=torch.from_numpy(state)
    rng=torch.Generator(device='cpu').manual_seed(seed)
    video_noise=torch.randn((1,96,2,16,48),generator=rng).to(torch.bfloat16)
    action_noise=torch.randn((1,32,80),generator=rng).to(torch.bfloat16);action_noise[...,list(PAD)]=0
    return dict(wan_images=wan,phoenix_images=phoenix,proprio=proprio.to(torch.bfloat16),
        context=context,context_mask=context_mask,video_noise=video_noise,action_noise=action_noise)


def decode_actions(action80,stats):
    values=np.asarray(action80,dtype=np.float32)
    if values.shape!=(HORIZON,80)or not np.isfinite(values).all()or np.count_nonzero(values[:,PAD]):
        raise ValueError('WAM requires finite32x80 output with zero invalid slots')
    mean,std=stats['action']
    action=np.ascontiguousarray(values[:,SLOTS]*std+mean,dtype=np.float32)
    if not np.isfinite(action).all():raise ValueError('WAM action denormalization overflow')
    return action


def preprocessing_identity(bundle):
    bundle=validate_descriptor(bundle)
    return dict(schema=SCHEMA,camera_order=list(CAMS),image_transform='wam_bicubic256_wan_bf16',
        torch_version=TORCH_VERSION,
        color='RGB',resize=dict(mode='bicubic',align_corners=False,clamp=[0,1]),
        normalized_state_dtype='bfloat16_as_float32',slots=list(SLOTS),native_dimensions=80,
        normalization_sha256=bundle['artifacts']['stats']['sha256'],context_sha256=bundle['artifacts']['context']['sha256'],
        seed_policy='request_seed_cpu_video_then_full80_action_then_zero_pad',denoise_steps=bundle['inference_options']['denoise_steps'],
        action_horizon=HORIZON,action_hz=HZ,first_action_offset_s=1/HZ)


def descriptor(registry,*,denoise_steps=None):
    pins=registry.get('artifacts');backend=registry.get('backend')
    if not is_model_id(registry.get('model_id'))or backend not in('torch','anyinfer')or not isinstance(pins,dict)or set(pins)!=ARTIFACT_KINDS:
        raise ValueError('explicit WAM checkpoint/backend/artifact registry required')
    norms=[r for r in registry['files']if r['sha256']==pins['stats']]
    root=Path(registry['root']).resolve()
    if len(norms)!=1:raise ValueError('one original WAM normalization sidecar required')
    path=(root/norms[0]['path']).resolve(strict=True)
    if not path.is_relative_to(root)or not path.is_file()or path.stat().st_size>16384:raise ValueError('bounded registered WAM normalization required')
    capabilities=registry.get('inference_capabilities')
    if denoise_steps is None:
        denoise_steps=registry.get('selected_inference_options',{}).get('denoise_steps',
            (capabilities or{}).get('denoise_steps',{}).get('default'))
    options=validate_inference_options(dict(denoise_steps=denoise_steps),capabilities)
    source=registry.get('checkpoint_source')if pins['checkpoint']is None else None
    return validate_descriptor(dict(schema=SOURCE_SCHEMA if source is not None else SCHEMA,model_id=registry['model_id'],epoch=None,step=registry.get('step'),backend=backend,
        artifacts={k:dict(sha256=v)for k,v in pins.items()},model_bundle_sha256=registry['bundle_sha256'],
        **(dict(checkpoint_source=deepcopy(source))if source is not None else{}),
        normalization_json=path.read_text(),inference_options=options,inference_capabilities=deepcopy(capabilities),
        runtime=dict(adapter='wam_torch_client.v2'if backend=='torch'else'wam_anyinfer_client.v2',
                     engine_sha256=registry.get('engine_sha256'),python_abi='cp312',torch_version=MODEL_TORCH_VERSION)))


def variants(registry):
    default=descriptor(registry);bounds=default['inference_capabilities']['denoise_steps']
    return [dict(deepcopy(default),inference_options=dict(denoise_steps=n))
            for n in range(bounds['minimum'],bounds['maximum']+1)]


def verify_registry(registry):
    """Host/manifest and lightweight content checks; large weights use size and strict loading."""
    from .mixed366 import sha
    root=Path(registry['root']).resolve(strict=True);files=registry['files'];paths={};registered_paths=set()
    if registry.get('machine_id_sha256')!=sha('/etc/machine-id')or not str(registry.get('gpu_uuid','')).startswith('GPU-'):
        raise ValueError('registered WAM host/GPU identity required')
    canonical=json.dumps(sorted(files,key=lambda r:r['path']),sort_keys=True,separators=(',',':')).encode()
    if hashlib.sha256(canonical).hexdigest()!=registry['bundle_sha256']:raise ValueError('WAM bundle manifest changed')
    for row in files:
        path=(root/row['path']).resolve(strict=True)
        if not path.is_relative_to(root)or not path.is_file()or path.stat().st_size!=row['bytes']:
            raise ValueError('WAM registered artifact changed: '+row['path'])
        registered_paths.add(path)
        large_weight=path.suffix.lower()in ('.pt','.pth','.ckpt','.safetensors','.bin')and row['bytes']>64*1024*1024
        # Reuse the registered digest for large weights, then strictly load them.
        # Runtime code, manifests and small artifacts still receive content checks.
        if row.get('sha256')is None and not metadata_weight(registry,row):
            raise ValueError('missing artifact identity: '+row['path'])
        if not large_weight and sha(path)!=row['sha256']:
            raise ValueError('WAM registered artifact changed: '+row['path'])
        if row['sha256']is not None:paths.setdefault(row['sha256'],[]).append(path)
    bundle=descriptor(registry)
    required={**{k:v['sha256']for k,v in bundle['artifacts'].items()},'engine':bundle['runtime']['engine_sha256']}
    checkpoint=None
    if bundle.get('schema')==SOURCE_SCHEMA:
        proof=paths.get(bundle['checkpoint_source']['receipt_sha256'],[])
        if len(proof)!=1 or proof[0].stat().st_size>8*1024**2:raise ValueError('one bounded original source receipt required')
        source=json.loads(proof[0].read_text());relative=registry.get('checkpoint_path')
        if (source.get('schema')!='workbench.model-source.v1'or source.get('model_id')!=bundle['model_id']
                or source.get('step')!=bundle['step']or source.get('checkpoint_path')!=relative):
            raise ValueError('checkpoint source identity differs from the selected model')
        for row in files:
            if row['sha256']is None and source.get('files',{}).get(row['path'],{}).get('bytes')!=row['bytes']:
                raise ValueError('unhashed weight differs from its source receipt')
        rows=[row for row in files if row['path']==relative]
        if len(rows)!=1 or rows[0]['bytes']!=bundle['checkpoint_source']['bytes']:
            raise ValueError('checkpoint size differs from its declared source')
        checkpoint=root/relative
        required.pop('checkpoint')
    if any(len(paths.get(digest,[]))!=1 for digest in required.values()):raise ValueError('unique original WAM artifacts required')
    selected={name:paths[digest][0]for name,digest in required.items()}
    if checkpoint is not None:selected['checkpoint']=checkpoint
    weights=selected['checkpoint'].parent
    for name in ('phoenix_encoder.pt','csvae.pt','Wan2.2_VAE.safetensors'):
        if weights/name not in registered_paths:raise ValueError('missing registered WAM encoder: '+name)
    return root,selected


def metadata_weight(registry,row):
    """No synthetic digest: large weights may use a declared source receipt."""
    return (isinstance(registry.get('checkpoint_source'),dict)and row.get('sha256')is None
        and type(row.get('bytes'))is int and row['bytes']>64*1024**2
        and Path(row['path']).suffix.lower()in('.pt','.pth','.ckpt','.safetensors','.bin'))


def sampling_change(previous, selected):
    """Only native Torch sampling changes can reuse this loaded worker."""
    if (previous.get('backend')!='torch' or selected.get('backend')!='torch'
            or previous.get('schema')not in (*SCHEMAS,'workbench.wam-mem.v1')):return False
    return ({k:v for k,v in previous.items()if k!='inference_options'}
            =={k:v for k,v in selected.items()if k!='inference_options'})


def sampling_options(bundle,request):
    if (bundle['backend']!='torch' or set(request)!={'operation','expected_options','inference_options'}
            or request['expected_options']!=bundle['inference_options']):
        raise ValueError('sampling update binding differs')
    return validate_inference_options(request['inference_options'],bundle['inference_capabilities'])


class Client:
    """Same shared ModelService client API, in WAM's existing cp312 runtime."""
    def __init__(self,registry,bundle,*,implementation=None):
        import secrets,sys
        self.implementation=implementation or sys.modules[__name__]
        self.registry=deepcopy(registry);self.bundle=self.implementation.validate_descriptor(bundle);self.model_id=self.bundle['model_id']
        if self.bundle!=self.implementation.descriptor(registry,denoise_steps=self.bundle['inference_options']['denoise_steps']):
            raise ValueError('WAM registry and selected descriptor differ')
        self.process=self.connection=self.lock=self.process_owner=None
        self.log_path=str(Path(registry['root'])/'logs'/('wam-worker-'+secrets.token_hex(8)+'.log'))

    def worker_status(self):
        p=self.process
        return dict(worker_pid=p.pid if p else None,worker_exit_code=p.poll()if p else None,worker_alive=p is not None and p.poll()is None)

    def can_reconfigure(self,bundle):
        return self.implementation.sampling_change(self.bundle,bundle) and self.process is not None and self.process.poll()is None

    def reconfigure(self,bundle):
        bundle=self.implementation.validate_descriptor(bundle)
        if not self.can_reconfigure(bundle):raise ValueError('sampling update requires the same live Torch model')
        self.connection.send(dict(operation='sampling',expected_options=self.bundle['inference_options'],
            inference_options=bundle['inference_options']))
        if not self.connection.poll(10):raise TimeoutError('sampling update unconfirmed; do not replay')
        result=self.connection.recv()
        if result.get('ok')is not True:raise ValueError(result.get('error','sampling update failed'))
        if result.get('worker_pid')!=self.process.pid or result.get('worker_identity')!=self.implementation.worker_identity(bundle):
            raise ValueError('sampling update worker identity differs')
        self.bundle=bundle;self.registry['selected_inference_options']=deepcopy(bundle['inference_options'])
        return result['worker_identity']

    def prepare(self):
        import multiprocessing,os,subprocess,sys,threading
        from .mixed366 import gpu_lock,save
        from rule_based_workbench.launch_environment import clean_environment
        if sys.platform!='linux':raise ValueError('WAM model host must be Linux')
        if self.process is not None:
            if self.process.poll()is None:return self.implementation.worker_identity(self.bundle)
            self.close()  # This explicit prepare retires the proven-exited attempt.
        root,_=self.implementation.verify_registry(self.registry)
        # Keep the supplied bundle's interpreter convention; no parent Python,
        # ROS or CUDA environment is inherited into the compiled cp312 engine.
        python=(Path(self.registry['python'])if self.registry.get('python')else
                root/('env/bin/python'if self.bundle['backend']=='torch'else'env/anyinfer-3090-py312/bin/python'))
        if not python.is_file():raise ValueError('WAM registered cp312 interpreter is missing')
        query='import sys,sysconfig,json;print(json.dumps([list(sys.version_info[:2]),sysconfig.get_paths()["purelib"]]))'
        if self.bundle['backend']=='torch':query='import sys,json,importlib.util;from pathlib import Path;print(json.dumps([list(sys.version_info[:2]),str(Path(importlib.util.find_spec("torch").origin).parent.parent)]))'
        abi,site=json.loads(subprocess.check_output([str(python),'-I','-c',query],env=clean_environment(),timeout=10))
        if abi!=[3,12]:raise ValueError('WAM interpreter ABI differs from cp312')
        libraries=[Path(site)/'torch/lib',*sorted((Path(site)/'nvidia').glob('*/lib'))]
        source=Path(__file__).resolve().parents[1]
        env=clean_environment(dict(PYTHONPATH=str(source),CUDA_VISIBLE_DEVICES=self.registry['gpu_uuid'],
            LD_LIBRARY_PATH=os.pathsep.join(str(p)for p in libraries if p.is_dir()),
            OMP_NUM_THREADS='2',OPENBLAS_NUM_THREADS='2',MKL_NUM_THREADS='2',TOKENIZERS_PARALLELISM='false'))
        Path(self.log_path).parent.mkdir(parents=True,exist_ok=True)
        snapshot=Path(self.log_path+'.registry.json')
        save(snapshot,dict(self.registry,selected_inference_options=self.bundle['inference_options']))
        self.lock=gpu_lock(self.registry);parent,child=multiprocessing.Pipe();self.connection=parent
        try:
            with open(self.log_path,'xb')as log:
                launched=threading.Event();errors=[]
                def own_process():
                    # Linux parent-death follows the creating thread, including
                    # after the HTTP request that called prepare has returned.
                    try:
                        self.process=subprocess.Popen([str(python),'-u','-B','-m',self.implementation.__name__,'--worker',str(snapshot),
                            '--pipe-fd',str(child.fileno()),'--parent-pid',str(os.getpid())],pass_fds=(child.fileno(),),
                            stdout=log,stderr=log,env=env,start_new_session=True)
                    except BaseException as error:errors.append(error)
                    finally:launched.set()
                    process=self.process
                    if process is not None:process.wait()
                self.process_owner=threading.Thread(target=own_process,daemon=True,name='wam-worker-owner')
                self.process_owner.start();launched.wait()
                if errors:raise errors[0]
            child.close()
            if not parent.poll(300):raise TimeoutError('WAM worker load deadline; inspect worker log')
            result=parent.recv()
            if (result.get('ready')is not True or result.get('pid')!=self.process.pid
                    or result.get('adapter_source_sha256')!=self.implementation._SOURCE_SHA256):raise ValueError(result.get('error','WAM worker identity/source differs'))
            return self.implementation.worker_identity(self.bundle)
        except BaseException:
            child.close();self.close();raise

    def infer(self,request):
        if self.process is None or self.process.poll()is not None:raise ValueError('explicit WAM preparation required')
        if len(request.get('applied_history',[])):raise ValueError('WAM does not accept action history')
        # Frame/source/Run identities stay with the shared caller and recorder;
        # this model process consumes only tensors, seed and reply association.
        self.connection.send(self.implementation.worker_request(request))
        if not self.connection.poll(120):raise TimeoutError('WAM inference deadline; do not replay')
        value=self.connection.recv()
        if value.get('ok')is not True:raise ValueError(value.get('error','WAM inference failed'))
        return value['result']

    def close(self):
        import subprocess
        if self.process is not None:
            if self.process.poll()is None:
                try:self.connection.send(None)
                except (OSError,EOFError,BrokenPipeError):pass
                try:self.process.wait(15)
                except subprocess.TimeoutExpired:self.process.terminate();self.process.wait(15)
            if self.process.poll()is None:raise RuntimeError('WAM worker exit unconfirmed; GPU lock retained')
            self.process=None
        if self.process_owner is not None:self.process_owner.join(2);self.process_owner=None
        if self.connection is not None:self.connection.close();self.connection=None
        if self.lock is not None:self.lock.close();self.lock=None


def worker_request(request):
    return dict(request_id=request['request_id'],seed=request['seed'],observation=dict(
        model_state14=request['observation']['model_state14'],images=request['observation']['images']))


def model_service(registry):
    """Existing shared service protocol, with independently pinned WAM bundles."""
    import os
    from .inference_transport import ModelService
    from .inference_model import model_bundle_digest
    if 'catalog_root' in registry:
        from .inference_catalog import RegisteredCatalog
        if registry.get('additional_registries'):
            raise ValueError('catalog_root is the single model discovery source')
        directory = RegisteredCatalog(registry['catalog_root'])
        return ModelService(directory.initial_client(registry), catalog=directory.catalog,
                            catalog_rows=directory.rows, client_factory=directory.client,
                            client_refresh=directory.refresh_client)
    entries=[registry]
    for raw in registry.get('additional_registries',[]):
        path=Path(raw);info=path.stat()
        if not path.is_absolute()or path.is_symlink()or info.st_uid!=os.getuid()or info.st_mode&0o077:
            raise ValueError('private explicit WAM registry required')
        entries.append(json.loads(path.read_text()))
    catalog={};registered={}
    for entry in entries:
        for bundle in variants(entry):
            key=bundle['model_id']+'/'+bundle['backend']+'/'+str(bundle['inference_options']['denoise_steps'])
            if key in catalog:raise ValueError('duplicate WAM checkpoint/backend/configuration')
            catalog[key]=bundle;registered[model_bundle_digest(bundle['model_id'],bundle)]=entry
    def factory(bundle):
        return Client(registered[model_bundle_digest(bundle['model_id'],bundle)],bundle)
    primary=descriptor(registry)
    return ModelService(factory(primary),catalog=lambda:deepcopy(catalog),client_factory=factory)


def _torch_runner(root,paths,bundle):
    """Strictly restore the frozen training implementation; never import anyinfer."""
    import os,sys,tempfile
    source=root/'source';sys.path[:0]=[str(source/'src'),str(source)]
    os.environ.update(HF_HUB_OFFLINE='1',TRANSFORMERS_OFFLINE='1',DIFFSYNTH_SKIP_DOWNLOAD='true',
        DIFFSYNTH_MODEL_BASE_PATH=str(root/'backbones'),PHOENIX_STVAE_CHECKPOINT=str(paths['checkpoint'].parent/'csvae.pt'),
        PHOENIX_ENCODER_CHECKPOINT=str(paths['checkpoint'].parent/'phoenix_encoder.pt'))
    torch=_torch()
    from omegaconf import OmegaConf
    from hydra.utils import instantiate
    from wam.utils.config_resolvers import register_default_resolvers
    from wam.model.modules.codecs.dino_stvae.provenance import bundled_config
    from phoenix.checkpoint import _load_joint96_checkpoint
    from wam.inference.frame_joint import infer_frame_joint_window
    import wam.model.factory as factory
    register_default_resolvers();payload=torch.load(paths['checkpoint'],map_location='cpu',mmap=True,weights_only=True)
    if payload.get('step')!=bundle['step']:raise ValueError('WAM checkpoint optimizer step differs')
    config=OmegaConf.load(paths['model_config']).model
    if config.action_dit_config.training_time_rtc.enabled is not False:raise ValueError('unqualified RTC model')
    config=OmegaConf.create(OmegaConf.to_container(config,resolve=True));config.load_text_encoder=False
    config.mot_attention_backend='sdpa';config.mot_flex_masked_batches=False;config.mot_flex_training_kernel_options=None
    config.joint_visual_codec=bundled_config(OmegaConf.to_container(config.joint_visual_codec,resolve=True))
    original=factory.load_wan22_ti2v_5b_components
    def components(**kwargs):
        kwargs['skip_dit_load_from_pretrain']=True
        return original(**kwargs)
    with tempfile.TemporaryDirectory(prefix='wam-restore-',dir=root/'logs')as folder:
        registry=Path(folder)/'embodiments.json';registry.write_text(json.dumps(payload['joint96_metadata']['soft_prompt']['registry']))
        config.action_dit_config.embodiment_registry_path=str(registry);factory.load_wan22_ti2v_5b_components=components
        try:model=instantiate(config,model_dtype=torch.bfloat16,device='cpu')
        finally:factory.load_wan22_ti2v_5b_components=original
        _load_joint96_checkpoint(model,payload)
    del payload
    model.to('cuda').eval().requires_grad_(False);model.set_runtime_device('cuda')
    if 'anyinfer'in sys.modules:raise ValueError('native Torch must not import anyinfer')
    pad=torch.ones((1,80),dtype=torch.bool,device='cuda');pad[:,list(SLOTS)]=False
    def run(inputs,seed):
        value=infer_frame_joint_window(model,prompt=None,input_image=inputs['wan_images'][:,:,:,0],num_frames=9,
            action_horizon=HORIZON,proprio=inputs['proprio'],context=inputs['context'],context_mask=inputs['context_mask'],
            camera_is_pad=torch.zeros((1,3),dtype=torch.bool,device='cuda'),action_dim_is_pad=pad,proprio_dim_is_pad=pad,
            embodiment_ids=torch.tensor([2],dtype=torch.long,device='cuda'),num_inference_steps=bundle['inference_options']['denoise_steps'],
            seed=seed,decode_video=False)
        return {'actions':value['action'][None]}
    return run


def _worker(registry,connection,parent_pid):
    import importlib.machinery,importlib.util,io,os,sys,time
    try:timing_boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    except OSError:timing_boot_id=None
    try:
        root,paths=verify_registry(registry)
        bundle=descriptor(registry);torch=_torch();stats=load_normalization(paths['stats'],expected_sha256=bundle['artifacts']['stats']['sha256'])
        if str(torch.__version__)!=MODEL_TORCH_VERSION or torch.version.cuda!='13.0':
            raise ValueError('WAM model runtime requires the pinned Torch cu130 build')
        raw_context=paths['context'].read_bytes()
        if hashlib.sha256(raw_context).hexdigest()!=bundle['artifacts']['context']['sha256']:raise ValueError('WAM text context changed before use')
        context=prepare_text_context(torch.load(io.BytesIO(raw_context),map_location='cpu',weights_only=True))
        # Validate the complete cached context before allocating the engine.
        zero_images={k:np.zeros((256,256,3),np.uint16)for k in CAMS}
        network_inputs(np.zeros(14,np.float32),zero_images,context['context'],context['context_mask'],seed=0)
        free,_=torch.cuda.mem_get_info()
        if free<16*1024**3:raise ValueError('WAM requires16GiB free GPU memory before loading')
        runtime_config_sha256=bundle['artifacts']['model_config']['sha256']
        if bundle['backend']=='torch':run=_torch_runner(root,paths,bundle)
        else:
            sys.path.insert(0,str(paths['engine'].parent));spec=importlib.util.find_spec('anyinfer')
            if spec is None or not isinstance(spec.loader,importlib.machinery.ExtensionFileLoader)or Path(spec.origin).resolve()!=paths['engine']:
                raise ValueError('WAM engine import differs from the registered binary')
            import anyinfer
            import tempfile,yaml
            config=yaml.safe_load(paths['model_config'].read_text())
            config['components']['mot']['config']['num_denoise_steps']=bundle['inference_options']['denoise_steps']
            with tempfile.NamedTemporaryFile(mode='w',prefix='runtime-',suffix='.yaml',dir=root/'logs',delete=False)as configured:
                yaml.safe_dump(config,configured);config_path=Path(configured.name)
            runtime_config_sha256=hashlib.sha256(config_path.read_bytes()).hexdigest()
            engine=anyinfer.load_engine(model_type='phoenix_mwa',config=str(config_path),
                weights_dir=str(paths['checkpoint'].parent),use_cuda_graph=False,warmup=1)
            run=lambda inputs,seed:engine.run(inputs)
        text_gpu=context['context'].cuda();mask_gpu=context['context_mask'].cuda()
        connection.send(dict(ready=True,pid=os.getpid(),adapter_source_sha256=_SOURCE_SHA256))
        while True:
            request=connection.recv()
            if request is None:return
            if request.get('operation')=='sampling':
                try:
                    options=sampling_options(bundle,request)
                    # _torch_runner reads this same bundle on every call; its
                    # model and GPU tensors remain resident.
                    bundle['inference_options']=options
                    connection.send(dict(ok=True,worker_pid=os.getpid(),worker_identity=worker_identity(bundle)))
                except (ValueError,TypeError,KeyError)as error:connection.send(dict(ok=False,error=str(error)))
                continue
            start=time.monotonic();obs=request['observation']
            inputs=network_inputs(obs['model_state14'],obs['images'],context['context'],context['context_mask'],seed=request['seed'])
            hashes={k:hashlib.sha256(v.contiguous().view(torch.uint8).numpy().tobytes()).hexdigest()for k,v in inputs.items()}
            # Each request owns its buffers even if a compiled component writes
            # in place; the pinned cached context remains immutable.
            cuda_inputs={k:text_gpu.clone()if k=='context'else mask_gpu.clone()if k=='context_mask'else v.cuda()for k,v in inputs.items()}
            torch.cuda.synchronize();model_started=time.monotonic()
            outputs=run(cuda_inputs,request['seed'])
            torch.cuda.synchronize();model_finished=time.monotonic()
            raw=outputs['actions'].float().cpu().numpy()[0];output_ready=time.monotonic()
            action=decode_actions(raw,stats);decoded=time.monotonic()
            connection.send(dict(ok=True,result=dict(request_id=request['request_id'],action14=action,
                audit=dict(raw_normalized_action80=raw.tolist(),input_tensor_sha256=hashes,
                    context_sha256=bundle['artifacts']['context']['sha256'],normalization_sha256=bundle['artifacts']['stats']['sha256'],
                    denoise_steps=bundle['inference_options']['denoise_steps'],runtime_config_sha256=runtime_config_sha256,
                    seed=request['seed'],preprocessing=preprocessing_identity(bundle),
                    timing=dict(host_prepare_s=model_started-start,engine_wall_s=model_finished-model_started,
                        output_to_host_s=output_ready-model_finished,denormalize_s=decoded-output_ready,
                        total_wall_s=decoded-start,measurement='host wall; engine stage includes CUDA synchronization'),
                    timing_points=dict(clock='host_monotonic_ns',boot_id=timing_boot_id,pid=os.getpid(),
                        host_start_ns=round(start*1e9),engine_start_ns=round(model_started*1e9),
                        engine_end_ns=round(model_finished*1e9),host_output_ns=round(output_ready*1e9),decoded_ns=round(decoded*1e9)),
                    command_publication=False))))
    except BaseException as error:
        import traceback
        traceback.print_exc()
        try:connection.send(dict(error=str(error)))
        except (OSError,EOFError,BrokenPipeError):pass
    finally:connection.close()


if __name__=='__main__':
    import argparse
    from .arx_output_guard import parent_death
    from multiprocessing.connection import Connection
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--worker',type=Path,required=True);parser.add_argument('--pipe-fd',type=int,required=True)
    parser.add_argument('--parent-pid',type=int,required=True);arguments=parser.parse_args()
    parent_death(arguments.parent_pid)
    with _torch().inference_mode():
        _worker(json.loads(arguments.worker.read_text()),Connection(arguments.pipe_fd),arguments.parent_pid)
