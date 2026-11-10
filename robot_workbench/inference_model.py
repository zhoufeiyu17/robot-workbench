"""Frozen Base integrations, reusing independently pinned workers and IPC.

No model implementation or weights live here. Constructing a client is inert;
an explicit prepare operation verifies artifacts and starts the remote-host
worker in its own interpreter. Never run prepare on the developer laptop.
"""
from copy import deepcopy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import re
import threading
import time


CAMS = ('base_0_rgb', 'left_wrist_0_rgb', 'right_wrist_0_rgb')
PROMPT = 'Place the coffee cup onto the coffee machine’s tray.'
BASE379_ID = 'base379-epoch010-step29450'
BASE566_ID = 'base566-epoch007-step58982'
BASE566_PROMPT = '将纸杯放到咖啡机出水口下方'
BASE566_FAMILY_ID = '9f52982d7f5592583d36d90c48439de173aef110915b0bb658254c8fe6924449'
BASE566_MODEL_CONFIG = dict(paligemma_variant='gemma_2b', action_expert_variant='gemma_300m',
    dtype='bfloat16', image_keys=list(CAMS), disable_color_aug=False, use_joint_eef_mask=False,
    action_horizon=60, action_dim=32, rtc_max_delay=0)
MANIPULATION_SCHEMA = 'workbench.manipulation_base566.v1'
MIXED366_SCHEMA = 'workbench.mixed366_nohistory.v1'
MIXED366_ID = 'mixed366-nohistory-epoch010-step34250'
MIXED366_PINS = dict(checkpoint='d0da9cd4fd5baf5fea60e9e47acdd20c18546ccf0dd8dc8c4426d568e320495f',
    metadata='c0d43d622b2124339d5a83291651d8518f61844ff3abf4d5250a7a1f2f9b24b6',
    stats='6b51671873802e3234dd354f9a9ea9ea1117fdf21df631257e94a9f8744ce565',
    tokenizer='8986bb4f423f07f8c7f70d0dbe3526fb2316056c17bae71b1ea975e77a168fc6',
    io_contract='d21e5020f78b50e502ec34c1b953554ac87ee580d9de44690b1b347ce772638f',
    model_config='a51f0b8dfc2a9cdcead1e9548868d9f10271f3e6c8ea4fc3ba84fdc81bf7f774')
COFFEE999_ID = 'coffee999-xnorm-epoch005-step32120'
COFFEE999_PINS = dict(MIXED366_PINS,
    checkpoint='c66e97058a491fd9db6c409cfc03f006b0633abf7e28185e54bde240fc15db72',
    metadata='13277fcadae10d31bcbed7702c0eeac5cf39b31f30b221df9759b87f1bfc356b',
    stats='a1c9ba2147c28b45290cd48964476ad90b90f85379defd954d78131f3d63bb64')
NOHISTORY_MODELS = {
    MIXED366_ID: dict(epoch=10,step=34250,label='mixed366',pins=MIXED366_PINS,tasks=('coffee','lid_handoff')),
    COFFEE999_ID: dict(epoch=5,step=32120,label='coffee999-xnorm',pins=COFFEE999_PINS,tasks=('coffee',)),
}


def native_runtime(bundle):
    if not isinstance(bundle,dict):return False
    schema=bundle.get('schema')
    if schema in(MANIPULATION_SCHEMA,MIXED366_SCHEMA):return True
    if not isinstance(schema,str)or not schema.startswith('workbench.wam.'):return False
    from .wam import SCHEMAS as WAM_SCHEMAS
    return schema in WAM_SCHEMAS


def validate_native_descriptor(bundle):
    """Public checkpoint/implementation identities; host paths stay in the adapter registry."""
    from .wam import SCHEMAS as WAM_SCHEMAS,validate_descriptor
    if isinstance(bundle,dict)and bundle.get('schema')in WAM_SCHEMAS:
        return validate_descriptor(bundle)
    required={'schema','model_id','epoch','step','backend','artifacts','model_bundle_sha256','runtime'}
    if not native_runtime(bundle) or set(bundle)!=required:
        raise ValueError('explicit manipulation Base566 descriptor required')
    mixed=bundle['schema']==MIXED366_SCHEMA
    match=re.fullmatch(r'(?:mixed366-nohistory|coffee999-xnorm|base566)-epoch([0-9]+)-step([1-9][0-9]*)',str(bundle['model_id']))
    if (match is None or type(bundle['epoch'])is not int or type(bundle['step'])is not int
            or [int(x)for x in match.groups()]!=[bundle['epoch'],bundle['step']]
            or bundle['backend']not in('torch','triton')):
        raise ValueError('checkpoint and backend identity differ')
    if mixed and bundle['model_id']not in NOHISTORY_MODELS or not mixed and not bundle['model_id'].startswith('base566-'):
        raise ValueError('model family and runtime schema differ')
    def sha(value):return isinstance(value,str)and re.fullmatch('[a-f0-9]{64}',value)is not None
    artifacts=bundle['artifacts'];runtime=bundle['runtime']
    if (not isinstance(artifacts,dict) or set(artifacts)!={'checkpoint','metadata','stats','tokenizer','io_contract','model_config'}
            or any(not isinstance(v,dict) or set(v)!={'sha256'} or not sha(v['sha256']) for v in artifacts.values())
            or not sha(bundle['model_bundle_sha256'])):
        raise ValueError('public native descriptor requires actual artifact SHA256 values, without paths')
    if mixed and any(artifacts[k]['sha256']!=v for k,v in NOHISTORY_MODELS[bundle['model_id']]['pins'].items()):
        raise ValueError('mixed366 checkpoint or nohistory input contract differs')
    if not mixed and (artifacts['stats']['sha256']!=BASE566_PINS['stats'] or artifacts['tokenizer']['sha256']!=BASE566_PINS['tokenizer']):
        raise ValueError('native descriptor differs from frozen Base566 normalization/tokenizer')
    if (not isinstance(runtime,dict) or set(runtime)!={'adapter','source_manifest_sha256','frozen_model_source_sha256','triton_deployment_sha256'}
            or runtime['adapter']!=('mixed366_process_model_client.v1'if mixed else'manipulation_process_model_client.v1')
            or not all(sha(runtime[k])for k in ('source_manifest_sha256','frozen_model_source_sha256'))
            or (not sha(runtime['triton_deployment_sha256'])if bundle['backend']=='triton' else runtime['triton_deployment_sha256']is not None)):
        raise ValueError('native implementation and qualified deployment identities required')
    return deepcopy(bundle)
BASE379_PINS = dict(
    bundle='64460fbbb8324fd8ae5862aa2bb94568c3af463346c80995fe643f0a2bd05643',
    checkpoint='c9254e838362db0649833fc85fa7f9093075b290c5f81e2bccadceccd3c70b68',
    metadata='f87a485f3abcb1ea7316e6614b992c61ce6eb80d89a74c81a653cb437e7a294c',
    stats='81ba44cb89230b0e6cc4deb46be437fde96d7ce15c1b9139caa7acff4ac8ca63',
    tokenizer='8986bb4f423f07f8c7f70d0dbe3526fb2316056c17bae71b1ea975e77a168fc6',
    io_contract='62429385ea73fb08fb3725a8434eac5172618634ed8fe40977436b9dffcf38a3',
    worker='9cdcc71262ccda1f3d7358a22ec47fd0fc456ddb2599a2fc692e16835617ac91',
    process='5f52c2d8695e72cc7a48349354ff74b643c830fbe0f190c0df5767390755b914',
    history_adapter='9b301b714cd35d4ba6d1d61836a37b5359f94d869d98d37910a7208db967c8d8',
    launcher='fbe5535e388c0ee03d20026605b903c111d988da6e395761802d4d95556367c1',
)
BASE566_PINS = dict(
    checkpoint='c1bbd7c86ab478217ca985c41f4aef1a4e1752813c7051519d63c8e384cb8e80',
    metadata='bd47ae4b35a15baa8b7fdb7ecbd99cc7e8e17de498fdcb71b7b7d88027cd08b6',
    stats='84fbc17851a8d744e0b5dd8b5e5365803246b3fe6006b88813eccba78015da45',
    tokenizer=BASE379_PINS['tokenizer'],
    source_manifest='e8e2bf7fa5ee689cca0210f45a565c7bc9d613f9fc4c6c38b4dcddcfaf1e1e09',
    io_contract='3448ff67f910c4fa6d80a1c85c6d59b32dbb6b41171373dfd459526e1db787e9',
    process='974413046d65a4c47c350772a567dfe1c27420e1ec206b10fc06475116fe3446',
    worker='4f41e6e2361ce7196e82725cc1d243bbf1034d657e8f8faa605509250722d43b',
)


def model_family(model_id):
    if isinstance(model_id,str)and model_id.startswith('wam-'):
        from .wam import is_model_id
        if is_model_id(model_id):return 'wam'
    if model_id == BASE379_ID:return 'base379'
    if model_id in NOHISTORY_MODELS:return 'mixed366'
    if isinstance(model_id, str) and re.fullmatch(r'base566-epoch[0-9]{3,}-step[1-9][0-9]*', model_id):return 'base566'
    raise ValueError('unknown model family: '+str(model_id))


def action_contract(model_id=BASE379_ID, bundle=None):
    """Describe existing model values; no device mapping or execution schedule."""
    model_spec(model_id, bundle)
    family = model_family(model_id)
    return dict(schema='workbench.model-action14.v1', shape=[32 if family=='wam'else 60, 14],
        axes=[f'{side}_{axis}' for side in ('left', 'right')
              for axis in ('j1', 'j2', 'j3', 'j4', 'j5', 'j6', 'jaw')],
        units=(['rad'] * 6 + ['m']) * 2, action_type='absolute_position',
        coordinate_domain='recorded_native_state14' if family in ('mixed366','wam') else 'training_model_state14',
        jaw_semantics='recorder_endpoint_coordinate' if family in ('mixed366','wam') else
            ('continuous_sum_two_finger_travel' if family == 'base566' else 'legacy_model_jaw_coordinate'),
        normalized=False, device_mapping_applied=False,
        sampling=dict(nominal_period_s=1/30, time_origin='observation_source_time'if family=='wam'else'unassigned',
            first_sample_offset_s=1/30 if family=='wam'else None,
            basis='materialized_next_observation'if family=='wam'else'existing_workbench_nominal_grid', execution_timestamps=False),
        history_steps=0 if family in ('mixed366','wam') else 60)


def model_spec(model_id=BASE379_ID, bundle=None):
    """A checkpoint varies only within the frozen Base566 training/IO family."""
    if model_family(model_id)=='wam':
        from .wam import validate_descriptor,SCHEMA
        value=validate_descriptor(bundle)
        if value['model_id']!=model_id:raise ValueError('WAM descriptor model identity differs')
        name=model_id[4:].rsplit('-step',1)[0].upper().replace('-','_')
        return dict(id=model_id,schema=SCHEMA,epoch=None,step=value['step'],prompt='bound WAM text context',
            pins={k:v['sha256']for k,v in value['artifacts'].items()},backend=value['backend'],
            inference_options=value['inference_options'],inference_capabilities=value['inference_capabilities'],
            label=f"WAM {name} · step_{value['step']:06d} · {value['inference_options']['denoise_steps']}步去噪 · 32步输出")
    if model_id in NOHISTORY_MODELS:
        value=validate_native_descriptor(bundle)
        if value['model_id']!=model_id:raise ValueError('mixed366 descriptor required')
        registered=NOHISTORY_MODELS[model_id]
        return dict(id=model_id,schema=MIXED366_SCHEMA,epoch=registered['epoch'],step=registered['step'],prompt='coffee',tasks=registered['tasks'],
            pins={k:v['sha256']for k,v in value['artifacts'].items()},backend=value['backend'],
            label=f"{registered['label']} · 无history · epoch {registered['epoch']:03d} / step {registered['step']} · "+value['backend'])
    if model_id == BASE379_ID:
        return dict(id=model_id, schema='workbench.base379.v1', epoch=10, step=29450,
                    prompt=PROMPT, pins=BASE379_PINS, label='Base379 · epoch 010 / step 29450')
    if model_family(model_id) == 'base566':
        epoch, step = (int(value) for value in re.fullmatch(r'base566-epoch([0-9]+)-step([0-9]+)', model_id).groups())
        if native_runtime(bundle):
            value=validate_native_descriptor(bundle)
            if value['model_id']!=model_id:raise ValueError('native descriptor model identity differs')
            return dict(id=model_id,schema=MANIPULATION_SCHEMA,epoch=epoch,step=step,prompt=BASE566_PROMPT,
                        pins={k:v['sha256']for k,v in value['artifacts'].items()},backend=value['backend'],
                        label=f'Base566 · epoch {epoch:03d} / step {step} · '+value['backend'])
        pins = dict(BASE566_PINS)
        if bundle is not None:
            if bundle.get('model_id') != model_id or bundle.get('epoch') != epoch or bundle.get('step') != step:
                raise ValueError('checkpoint manifest identity differs from selected model')
            for key in ('checkpoint','metadata'):
                value = bundle.get('artifacts', {}).get(key, {}).get('sha256')
                if not isinstance(value, str) or not re.fullmatch('[0-9a-f]{64}', value):
                    raise ValueError('explicit trusted checkpoint SHA256 required: '+key)
                pins[key] = value
        elif model_id == 'base566-epoch005-step42130':
            pins.update(checkpoint='2515056ce46c64a68be7857ba345fd962901c9c984d4d83ee23619ddfc82a20c',
                        metadata='39647d960300a50419405c8733ac91f5a15e418ec82a3dd01539acf4fbe7f255')
        elif model_id != BASE566_ID:
            raise ValueError('trusted checkpoint manifest required for '+model_id)
        return dict(id=model_id, schema='workbench.base566.v1', epoch=epoch, step=step,
                    prompt=BASE566_PROMPT, pins=pins, label=f'Base566 · epoch {epoch:03d} / step {step}')
    raise ValueError('unknown pinned model identity: '+str(model_id))


def worker_identity(model_id=BASE379_ID, bundle=None):
    if model_family(model_id)=='wam':
        from .wam import worker_identity as identity
        return identity(bundle)
    spec = model_spec(model_id, bundle); pins = spec['pins']
    if model_id in NOHISTORY_MODELS:
        return dict(status='ready',model_id=model_id,epoch=spec['epoch'],step=spec['step'],backend=bundle['backend'],
            strict_load=bundle['backend']=='torch',checkpoint_sha256=pins['checkpoint'],stats_sha256=pins['stats'],
            tokenizer_sha256=pins['tokenizer'],io_contract_sha256=pins['io_contract'],
            model_bundle_sha256=bundle['model_bundle_sha256'],runtime=deepcopy(bundle['runtime']),
            action_horizon=60,history_steps=0,inference_steps=10,rtc_off=True,
            input_domain='training_raw_rgb',output_domain='recorded_native_state14',command_publication=False)
    if native_runtime(bundle):
        value=dict(status='ready',model_id=model_id,epoch=spec['epoch'],step=spec['step'],backend=bundle['backend'],
            mode='abs',strict_load=bundle['backend']=='torch',checkpoint_sha256=pins['checkpoint'],
            checkpoint_metadata_sha256=pins['metadata'],stats_sha256=pins['stats'],tokenizer_sha256=pins['tokenizer'],
            io_contract_sha256=pins['io_contract'],model_config_sha256=pins['model_config'],
            model_bundle_sha256=bundle['model_bundle_sha256'],runtime=deepcopy(bundle['runtime']),
            prompt=BASE566_PROMPT,action_horizon=60,history_steps=60,action_hz=30,inference_steps=10,
            rtc_off=True,vision_variant='original_unpatched',family_identity_sha256=BASE566_FAMILY_ID,
            history_audit_outputs=False)
        if bundle['backend']=='triton':value.update(deployment_qualified=True,load_validation='qualified_packed_artifact')
        return value
    value = dict(status='ready', epoch=spec['epoch'], step=spec['step'], mode='abs', strict_load=True,
        checkpoint_sha256=pins['checkpoint'], stats_sha256=pins['stats'], tokenizer_sha256=pins['tokenizer'],
        checkpoint_metadata_sha256=pins['metadata'], prompt=spec['prompt'], inference_steps=10,
        variant='history60', history_steps=60, world_size=16, vision_variant='unirefiner',
        rtc_off=True, rtc_prefix_steps=0, history_audit_outputs=True, rtc_audit_outputs=True)
    if model_family(model_id) == 'base566':
        value.update(model_id=model_id, vision_variant='original_unpatched', action_horizon=60, rtc_audit_outputs=False,
            jaw_encoding='continuous_sum_two_finger_travel_m', source_manifest_sha256=pins['source_manifest'],
            io_contract_sha256=pins['io_contract'], action_hz=30, execution_steps=10, family_identity_sha256=BASE566_FAMILY_ID)
    return value


def model_descriptor(model_id=BASE379_ID, bundle=None):
    spec = model_spec(model_id, bundle)
    if native_runtime(bundle):return validate_native_descriptor(bundle)
    return dict(model_id=spec['id'], epoch=spec['epoch'], step=spec['step'],
        artifacts={key:dict(sha256=value) for key,value in spec['pins'].items()})


def model_bundle_digest(model_id=BASE379_ID, bundle=None):
    return hashlib.sha256(json.dumps(model_descriptor(model_id,bundle), sort_keys=True,
        separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def validate_worker_identity(identity, model_id=BASE379_ID, *, bundle=None):
    expected = worker_identity(model_id, bundle)
    if not isinstance(identity, dict) or any(identity.get(k) != v for k,v in expected.items()):
        raise ValueError('worker identity differs from pinned '+model_id)
    if native_runtime(bundle) and bundle['backend']=='triton' and not re.fullmatch('[a-f0-9]{64}',str(identity.get('packed_sha256',''))):
        raise ValueError('qualified native packed artifact SHA256 required')
    return deepcopy(identity)


def _sha256(path):
    actual = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            actual.update(block)
    return actual.hexdigest()


def validate_base566_metadata(metadata, bundle, stats):
    identity = metadata.get('identity')
    actual = hashlib.sha256(json.dumps(identity, sort_keys=True, ensure_ascii=False,
        separators=(',', ':'), allow_nan=False).encode()).hexdigest()
    if (metadata.get('schema') != 'pi05.base566_checkpoint.v1' or metadata.get('mode') != 'train'
            or type(metadata.get('epoch')) is not int or metadata['epoch'] != bundle['epoch']
            or type(metadata.get('step')) is not int or metadata['step'] != bundle['step']
            or metadata.get('world_size') != 16 or metadata.get('identity_sha256') != BASE566_FAMILY_ID
            or actual != BASE566_FAMILY_ID or metadata.get('model_config') != BASE566_MODEL_CONFIG
            or metadata.get('stats') != stats):
        raise ValueError('checkpoint metadata differs from selected Base566 family or normalization stats')
    return metadata


def validate_bundle(bundle, *, verify_files=False):
    """Validate an explicit deployment manifest; caller supplies paths on model host."""
    if native_runtime(bundle):
        value=validate_native_descriptor(bundle)
        if verify_files:raise ValueError('native adapter must verify its registered host artifacts; public descriptors contain no paths')
        return value
    if isinstance(bundle, dict) and bundle.get('schema') == 'workbench.base566.v1':
        spec = model_spec(bundle.get('model_id'), bundle)
        fixed = dict(schema=spec['schema'], epoch=spec['epoch'], step=spec['step'], prompt=BASE566_PROMPT,
                     action_horizon=60, execution_steps=10, action_hz=30, rtc=False)
        if any(bundle.get(k) != v for k,v in fixed.items()) or bundle.get('rtc') is not False:
            raise ValueError('Base566 frozen family manifest required')
        if set(bundle.get('artifacts', {})) != set(BASE566_PINS):
            raise ValueError('exact Base566 artifacts required')
        for key, pin in spec['pins'].items():
            entry = bundle['artifacts'][key]
            if (not isinstance(entry, dict) or entry.get('sha256') != pin
                    or not isinstance(entry.get('path'), str) or not Path(entry['path']).is_absolute()):
                raise ValueError('missing or mismatched Base566 artifact: '+key)
            if 'size_bytes' in entry and (type(entry['size_bytes']) is not int or entry['size_bytes'] < 0):
                raise ValueError('invalid artifact byte count: '+key)
            if verify_files:
                if 'size_bytes' in entry and Path(entry['path']).stat().st_size != entry['size_bytes']:
                    raise ValueError('artifact size differs from manifest: '+key)
                if _sha256(entry['path']) != pin:
                    raise ValueError('artifact content differs from manifest: '+key)
        for key in ('python', 'model_source'):
            if not isinstance(bundle.get(key), str) or not Path(bundle[key]).is_absolute():
                raise ValueError('absolute model-host path required: '+key)
        if verify_files:
            validate_base566_metadata(json.loads(Path(bundle['artifacts']['metadata']['path']).read_text()), bundle,
                json.loads(Path(bundle['artifacts']['stats']['path']).read_text()))
            sources = json.loads(Path(bundle['artifacts']['source_manifest']['path']).read_text())
            root = Path(bundle['model_source']).resolve()
            for relative, entry in sources.items():
                source = Path(relative); path = (root/source).resolve()
                if (source.is_absolute() or '..' in source.parts or not path.is_relative_to(root)
                        or _sha256(path) != entry['sha256']):
                    raise ValueError('model source differs from frozen bundle: '+relative)
        return deepcopy(bundle)
    if isinstance(bundle, dict) and bundle.get('model_id', BASE379_ID) != BASE379_ID:
        raise ValueError('unknown pinned model identity')
    if (not isinstance(bundle, dict) or bundle.get('schema') != 'workbench.base379.v1' or bundle.get('epoch') != 10
            or bundle.get('step') != 29450 or bundle.get('prompt') != PROMPT
            or bundle.get('action_horizon') != 60 or bundle.get('execution_steps') != 10
            or bundle.get('action_hz') != 30 or bundle.get('rtc') is not False):
        raise ValueError('Base379 epoch010/29450, frozen prompt and 60/10@30Hz, RTC-off manifest required')
    artifacts = bundle.get('artifacts', {})
    for key in BASE379_PINS:
        entry = artifacts.get(key, {})
        digest, path = entry.get('sha256'), entry.get('path')
        if (not isinstance(digest, str) or not re.fullmatch('[0-9a-f]{64}', digest)
                or key in BASE379_PINS and digest != BASE379_PINS[key]
                or not isinstance(path, str) or not Path(path).is_absolute()):
            raise ValueError('missing or mismatched Base379 artifact: ' + key)
        if verify_files and _sha256(path) != digest:
            raise ValueError('artifact content differs from manifest: ' + key)
    for key in ('python', 'model_source', 'history_source'):
        if not isinstance(bundle.get(key), str) or not Path(bundle[key]).is_absolute():
            raise ValueError('absolute model-host path required: ' + key)
    if bundle['python'] != artifacts['launcher']['path']:
        raise ValueError('use the frozen model-env launcher to preserve the worker environment')
    if verify_files:
        original = json.loads(Path(artifacts['bundle']['path']).read_text())
        root = Path(artifacts['checkpoint']['path']).parent
        for relative, digest in original['source_identity'].items():
            source = Path(relative)
            if source.is_absolute() or '..' in source.parts or _sha256(root/source) != digest:
                raise ValueError('model source differs from frozen bundle: ' + relative)
    return deepcopy(bundle)


def letterbox_rgb224(value):
    """Frozen Base preprocessing: uint8 HWC, centered black bars, PIL bilinear."""
    import numpy as np
    from PIL import Image, ImageOps
    pixels = np.asarray(value)
    if pixels.dtype != np.uint8 or pixels.shape not in ((480, 640, 3), (224, 224, 3)):
        raise ValueError('Base requires RGB uint8 640x480 or already letterboxed 224x224')
    if pixels.shape == (224, 224, 3):
        return pixels.copy()
    output = ImageOps.pad(Image.fromarray(pixels), (224, 224), method=Image.Resampling.BILINEAR, color=(0, 0, 0))
    return np.asarray(output).copy()


class LegacyBaseAdapter:
    """Explicit left-only historical decoder plus calibrated coordinate callbacks.

    model_to_device/device_to_model are installed-machine transformations,
    including joint zero offsets and two-finger displacement versus metal gap.
    We do not assume a jaw model metre is merely 1000 device millimetres.
    """
    def __init__(self, *, model_to_device, device_to_model, position_limits,
                 closed_jaw_mm, open_jaw_mm, project_position_limits):
        import numpy as np
        limits = np.asarray(position_limits, dtype=np.float64)
        if limits.shape != (14, 2) or not np.isfinite(limits).all() or (limits[:, 0] > limits[:, 1]).any():
            raise ValueError('finite ordered 14-axis position limits required')
        if (type(project_position_limits) is not bool or not np.isfinite([closed_jaw_mm, open_jaw_mm]).all()
                or not limits[6, 0] <= closed_jaw_mm <= open_jaw_mm <= limits[6, 1]):
            raise ValueError('explicit projection policy and valid configured jaw targets required')
        self.model_to_device, self.device_to_model = model_to_device, device_to_model
        self.limits, self.closed, self.opened = limits.copy(), float(closed_jaw_mm), float(open_jaw_mm)
        self.project = project_position_limits

    def decode(self, prediction, current_device14):
        import numpy as np
        from .inference import vector14
        raw = vector14(prediction, 'prediction')
        current = vector14(current_device14, 'current device state')
        decoded = vector14(self.model_to_device(raw), 'model-to-device conversion')
        mapped = decoded.copy()
        # Historical binary-gripper and left-only behavior are explicit transforms.
        mapped[6] = self.opened if raw[6] >= .044 else self.closed
        mapped[7:] = current[7:]
        final = np.clip(mapped, self.limits[:, 0], self.limits[:, 1]) if self.project else mapped.copy()
        projections = [dict(axis=int(i), requested=float(mapped[i]), final=float(final[i]),
                            delta=float(final[i]-mapped[i]), reason='device_position_boundary')
                       for i in np.flatnonzero(mapped != final)]
        return dict(decoded_device14=decoded.tolist(), mapped_device14=mapped.tolist(), final_device14=final.tolist(),
            conversions=dict(left_jaw=dict(predicted_model_m=float(raw[6]), legacy_threshold_m=.044,
                legacy_semantic_m=.088 if raw[6] >= .044 else 0., current_target_mm=float(mapped[6]),
                configured_closed_mm=self.closed, configured_open_mm=self.opened),
                right_arm=dict(policy='hold_observed', ignored_prediction_model14=raw[7:].tolist(),
                               target_device14=mapped[7:].tolist())),
            projection_policy='recorded_position_projection' if self.project else 'record_only', projections=projections)

    def model_input(self, observation):
        import numpy as np
        from .inference import vector14
        images = observation.get('images', {})
        if set(images) != set(CAMS):
            raise ValueError('exactly three calibrated RGB camera roles required')
        return (vector14(self.device_to_model(vector14(observation['device_state14'])), 'model state').astype(np.float32),
                {name: letterbox_rgb224(images[name]) for name in CAMS})


class ContinuousBaseAdapter(LegacyBaseAdapter):
    """Base566 continuous jaw travel; only the left arm/jaw may follow predictions."""
    def __init__(self, *, model_to_device, device_to_model, position_limits,
                 project_position_limits=True, disabled_right_gripper_model_m=None):
        import numpy as np
        from .inference import vector14
        super().__init__(model_to_device=model_to_device, device_to_model=device_to_model,
            position_limits=position_limits, closed_jaw_mm=position_limits[6][0],
            open_jaw_mm=position_limits[6][1], project_position_limits=project_position_limits)
        if disabled_right_gripper_model_m is not None and (type(disabled_right_gripper_model_m) not in (int,float)
                or not np.isfinite(disabled_right_gripper_model_m) or disabled_right_gripper_model_m < 0):
            raise ValueError('explicit finite nonnegative disabled right jaw model value required')
        self.disabled_right_gripper_model_m = disabled_right_gripper_model_m
        def to_model(value):
            result = vector14(device_to_model(value)).copy()
            if self.disabled_right_gripper_model_m is not None:
                result[13] = self.disabled_right_gripper_model_m
            return result
        self.device_to_model = to_model

    def decode(self, prediction, current_device14):
        import numpy as np
        from .inference import vector14
        raw = vector14(prediction, 'prediction')
        held = vector14(current_device14, 'initial held device state')
        decoded = vector14(self.model_to_device(raw), 'model-to-device conversion')
        mapped = decoded.copy(); mapped[7:] = held[7:]
        if self.disabled_right_gripper_model_m is not None:
            mapped[13] = vector14(self.model_to_device(self.device_to_model(held)))[13]
        final = np.clip(mapped, self.limits[:,0], self.limits[:,1]) if self.project else mapped.copy()
        return dict(decoded_device14=decoded.tolist(), mapped_device14=mapped.tolist(), final_device14=final.tolist(),
            conversions=dict(left_jaw=dict(predicted_model_m=float(raw[6]), decoder='continuous_sum_two_finger_travel_m'),
                right_arm='hold_observed_at_episode_start',
                disabled_right_gripper_model_m=self.disabled_right_gripper_model_m),
            projections=[dict(axis=int(i), requested=float(mapped[i]), final=float(final[i]),
                delta=float(final[i]-mapped[i]), reason='device_position_boundary') for i in np.flatnonzero(mapped != final)])


class FrozenBaseClient:
    """Reuse the frozen Pi05Process module instead of maintaining a second wire protocol."""
    def __init__(self, bundle, *, log_path, adapter, startup_timeout_s=600., inference_timeout_s=120., env=None):
        self.bundle = validate_bundle(bundle)
        self.model_id = self.bundle.get('model_id', BASE379_ID)
        self.log_path, self.adapter = str(log_path), adapter
        self.startup_timeout, self.timeout, self.env = startup_timeout_s, inference_timeout_s, env
        self.client = None
        self._pipe_module = None
        self.identity = None
        self._query_lock = threading.Lock()

    def prepare(self):
        if self.client is not None:
            return deepcopy(self.identity)
        bundle = validate_bundle(self.bundle, verify_files=True)
        artifacts = bundle['artifacts']
        spec = importlib.util.spec_from_file_location('_workbench_frozen_pi05_process', artifacts['process']['path'])
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        command = ([bundle['python'], '-u', '-B', artifacts['worker']['path'], '--bundle-json', json.dumps(bundle)]
                   if model_family(self.model_id) == 'base566' else ['bash', bundle['python'], artifacts['worker']['path'],
            '--checkpoint', artifacts['checkpoint']['path'], '--checkpoint-sha256', BASE379_PINS['checkpoint'],
            '--checkpoint-metadata', artifacts['metadata']['path'], '--stats', artifacts['stats']['path'],
            '--tokenizer', artifacts['tokenizer']['path'], '--source', bundle['model_source'],
            '--history-source', bundle['history_source'], '--prompt', PROMPT])
        client = module.Pi05Process(command, self.log_path, startup_timeout=self.startup_timeout,
                                    timeout=self.timeout, env=self.env)
        try:
            identity = client.identity
            validate_worker_identity(identity, self.model_id, bundle=bundle)
        except BaseException:
            client.close()
            raise
        self.client, self.identity, self._pipe_module = client, deepcopy(identity), module
        return deepcopy(identity)

    def infer(self, request):
        # A stopped episode may still be draining its final worker response when
        # a newly confirmed episode starts. The single pipe stays one-consumer.
        with self._query_lock:
            return self._infer(request)

    def _infer(self, request):
        import numpy as np
        client, pipe_module = self.client, self._pipe_module
        if client is None:
            raise ValueError('explicit model preparation required before infer')
        start = time.monotonic()
        state, images = self.adapter.model_input(request['observation'])
        history = np.asarray(request['applied_history'], dtype=np.float32)
        if history.ndim != 2 or history.shape[1] != 14 or len(history) > 60 or not np.isfinite(history).all():
            raise ValueError('history must contain only up to 60 finite completed model-domain commands')
        prepared = time.monotonic()
        worker_timing = {}
        read_frame = pipe_module.read_frame if pipe_module is not None else None
        def read_with_timing(*args, **kwargs):
            metadata, arrays = read_frame(*args, **kwargs)
            timing = metadata.get('timing', {})
            if not isinstance(timing, dict):raise ValueError('worker timing object required')
            worker_timing.update(deepcopy(timing))
            if 'infer_wall_s' in metadata:worker_timing['infer_wall_s'] = metadata['infer_wall_s']
            for key, value in worker_timing.items():
                if key.endswith('_s') and value is not None and (type(value) not in (int, float)
                        or not math.isfinite(value) or value < 0):
                    raise ValueError('invalid worker timing: '+key)
            return metadata, arrays
        # Each preparation loads its own module; the query lock keeps its single
        # pipe consumer serialized. Preserve the frozen framing and validators.
        if read_frame is not None:pipe_module.read_frame = read_with_timing
        try:
            action = client.infer(state, images, seed=request['seed'], applied_history=history)
        finally:
            if read_frame is not None:pipe_module.read_frame = read_frame
        returned = time.monotonic()
        client.last_timing = dict(deepcopy(getattr(client, 'last_timing', {})),
            worker=worker_timing, host_model_input_wall_s=prepared-start,
            host_pipe_roundtrip_wall_s=returned-prepared, host_total_wall_s=returned-start)
        evidence = {key: value.tolist() for key, value in getattr(client, 'history_evidence', {}).items()}
        sampler = {key: value.tolist() if hasattr(value, 'tolist') else value
                   for key, value in getattr(client, 'rtc_evidence', {}).items()}
        return dict(request_id=request['request_id'], action14=action,
                    audit=dict(worker_identity=deepcopy(self.identity), history=evidence, sampler=sampler,
                               timing=deepcopy(client.last_timing)))

    def close(self):
        if self.client is not None:
            self.client.close()
            self.client = None
            self._pipe_module = None
