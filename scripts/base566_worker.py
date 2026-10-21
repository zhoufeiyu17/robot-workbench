#!/usr/bin/env python3
"""Frozen-family Base566 worker. Imports and CLI help never load weights."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from robot_workbench.inference_model import BASE566_PROMPT, CAMS, model_family, validate_bundle, worker_identity


def sample(state, history, images):
    import numpy as np
    state = np.asarray(state, dtype=np.float32)
    history = np.asarray(history, dtype=np.float32)
    if (state.shape != (14,) or history.ndim != 2 or history.shape[1] != 14 or len(history) > 60
            or not np.isfinite(state).all() or not np.isfinite(history).all()):
        raise ValueError('finite model-domain state14 and up to60 completed commands required')
    if set(images) != set(CAMS) or any(a.shape != (224,224,3) or a.dtype != np.uint8 for a in images.values()):
        raise ValueError('exact three letterboxed RGB uint8 cameras required')
    padded = np.zeros((60,14), np.float32); mask = np.zeros(60, bool)
    if len(history):padded[-len(history):] = history; mask[-len(history):] = True
    return dict(state=state, history=padded, history_mask=mask, images=images,
        prompt=BASE566_PROMPT, actions=np.zeros((60,14), np.float32),
        actions_mask=np.ones(60, bool), fast_eligible=True)


def load(bundle):
    if sys.platform != 'linux':
        raise RuntimeError('Base566 may load only on the allocated Linux model host')
    bundle = validate_bundle(bundle, verify_files=True)
    if model_family(bundle.get('model_id')) != 'base566':raise ValueError('Base566 family worker only')
    artifacts = bundle['artifacts']
    sys.path.insert(0, bundle['model_source'])
    import torch
    import sentencepiece
    from experiments.pi05_base566 import runtime
    from starVLA.model.modules.pi05.paligemma_with_expert import PaliGemmaWithExpertModel
    if not torch.cuda.is_available():raise RuntimeError('allocated CUDA model device required')
    metadata = json.loads(Path(artifacts['metadata']['path']).read_text())
    if (metadata.get('epoch') != bundle['epoch'] or metadata.get('step') != bundle['step']
            or metadata.get('model_config') != runtime.MODEL_CONFIG):
        raise ValueError('checkpoint metadata differs from selected checkpoint or original model config')
    model = runtime.make_model(metadata['model_config'])
    if model.paligemma_with_expert.embed_image.__func__ is not PaliGemmaWithExpertModel.embed_image:
        raise ValueError('Base566 vision embedding must remain original and unpatched')
    weights = torch.load(artifacts['checkpoint']['path'], map_location='cpu', weights_only=True, mmap=True)
    model.load_state_dict(weights, strict=True)
    actual = model.state_dict()
    if any(not torch.equal(value, actual[key]) for key,value in weights.items()):
        raise ValueError('strict-loaded checkpoint tensors differ')
    del weights, actual
    model.cuda().eval()
    stats = json.loads(Path(artifacts['stats']['path']).read_text())
    tokens = (sentencepiece.SentencePieceProcessor(model_file=artifacts['tokenizer']['path']), None)
    spec = importlib.util.spec_from_file_location('_base566_frozen_pipe', artifacts['process']['path'])
    pipe = importlib.util.module_from_spec(spec); spec.loader.exec_module(pipe)
    return model, stats, tokens, runtime, pipe


def predict(model, value, stats, tokens, runtime, seed):
    import numpy as np
    import torch
    if type(seed) is not int or not 0 <= seed < 2**63:raise ValueError('uint63 inference seed required')
    start, cpu_start = time.monotonic(), time.process_time()
    observation, _, _, history, _ = runtime.make_batch([value], stats, tokens, labels=False)
    batched = time.monotonic()
    noise = torch.randn((1,60,32), generator=torch.Generator().manual_seed(seed)).cuda()
    noise_ready = time.monotonic()
    with torch.no_grad(), torch.autocast('cuda', dtype=torch.bfloat16):
        output = model.sample_actions('cuda', observation, noise=noise, num_steps=10, **history)
    sampled = time.monotonic()
    normalized = output[0,:,:14].float().cpu().numpy()
    copied = time.monotonic()
    action = np.asarray(runtime.physical_actions(normalized, stats), dtype=np.float32)
    decoded = time.monotonic()
    encoded = runtime.encode_samples([value], stats, tokens, labels=False)
    history_normalized = np.asarray(encoded['history'][0], dtype=np.float32)
    if action.shape != (60,14) or not np.isfinite(action).all():raise ValueError('invalid continuous Base566 output')
    if history_normalized.shape != (60,14) or np.any(history_normalized[~value['history_mask']] != 0):
        raise ValueError('history padding or normalization differs from the frozen IO contract')
    # CUDA launches are asynchronous. These wall stages include host dispatch or
    # waits, and output_to_host includes the existing CPU-copy synchronization.
    timing = dict(make_batch_wall_s=batched-start, noise_wall_s=noise_ready-batched,
        sample_actions_wall_s=sampled-noise_ready, output_to_host_wall_s=copied-sampled,
        physical_actions_wall_s=decoded-copied, history_encode_wall_s=time.monotonic()-decoded,
        cpu_process_s=time.process_time()-cpu_start, gpu_compute_s=None,
        measurement='worker_host_wall_stages_cuda_async_until_output_to_host')
    return dict(action14=action, history_actions_physical=value['history'],
        history_actions_normalized=history_normalized, history_mask=value['history_mask']), timing


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bundle-json', required=True)
    args = parser.parse_args()
    bundle = validate_bundle(json.loads(args.bundle_json))
    # Keep imported libraries' stdout out of the frozen binary protocol.
    output_fd = os.dup(sys.stdout.fileno()); os.dup2(sys.stderr.fileno(), sys.stdout.fileno())
    model, stats, tokens, runtime, pipe = load(bundle)
    pipe.write_frame(output_fd, worker_identity(bundle['model_id'], bundle))
    while True:
        metadata, arrays = pipe.read_frame(sys.stdin.fileno())
        if set(metadata) != {'op','seed'} or metadata.get('op') != 'infer' or set(arrays) != {'state14','applied_history',*CAMS}:
            raise ValueError('Base566 accepts one synchronous history-only inference request')
        value = sample(arrays['state14'], arrays['applied_history'], {key:arrays[key] for key in CAMS})
        start = time.monotonic()
        result, timing = predict(model, value, stats, tokens, runtime, metadata['seed'])
        pipe.write_frame(output_fd, dict(status='ok', infer_wall_s=time.monotonic()-start, timing=timing), result)


if __name__ == '__main__':
    main()
