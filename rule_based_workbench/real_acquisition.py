"""Seal one physical observation, upload exact inputs, then pin this run's contract."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path, PurePosixPath
from uuid import uuid4

from artifact_transport import manifest, transfer as transport_transfer
from run_pipeline import absolute, below


def validate_config(config):
    pipeline = json.loads(Path(config['pipeline_config']).read_text())
    contract = json.loads(Path(pipeline['contract_path']).read_text())
    acquisition = config['acquisition']
    for name in ('profile_path', 'acquisition_binding_path'):
        if not Path(acquisition[name]).is_file():
            raise ValueError('missing physical acquisition input: ' + name)
    transport = acquisition['transport']
    root = absolute(transport['remote_root'])
    fs = contract['filesystem']
    if (below(root, pipeline['remote_run_root']) or
            not all(any(below(root, allowed) for allowed in fs.get(key, []))
                    for key in ('writable_roots', 'readonly_roots'))):
        raise ValueError('capture upload root must already be writable and declared as sealed readonly input outside the run')
    if 'endpoint' not in transport:
        raise ValueError('explicit acquisition transport endpoint required')
    return pipeline, contract


def prepare_inputs(config, output, *, read_snapshot, read_owner=None, transfer=transport_transfer, cancel=None):
    """Injected callback is the hardware boundary; never acquire/control here."""
    pipeline, contract = validate_config(config)
    acquisition = config['acquisition']
    profile = json.loads(Path(acquisition['profile_path']).read_text())
    binding = json.loads(Path(acquisition['acquisition_binding_path']).read_text())
    binding.pop('initial_state', None)  # Never reuse a previous observation's measured start.
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    name = 'physical-capture-' + uuid4().hex
    capture_dir = output / name
    from real_observation import capture, read_initial_sample, seal_initial_state
    from real_pipeline import validate_capture
    measured_start = acquisition.get('measured_initial_state', False)
    if type(measured_start) is not bool or measured_start and read_owner is None:
        raise ValueError('measured initial state requires explicit boolean and read-only owner callback')
    before = read_initial_sample(read_owner) if measured_start else None
    sealed = capture(read_snapshot, profile, binding, capture_dir,
                     **({'cancel':cancel} if cancel is not None else {}))
    if measured_start:
        sealed['initial_state'] = seal_initial_state(before, read_initial_sample(read_owner), profile)
        (capture_dir / 'acquisition-binding.json').write_text(json.dumps(sealed, indent=2, allow_nan=False))
    frames = validate_capture(capture_dir, sealed)['frames']
    names = ['profile.json', 'rgbd-manifest.json', 'acquisition-binding.json',
             *(frame['path'] for frame in frames)]
    expected = manifest(capture_dir, names)
    transport = acquisition['transport']
    remote = str(PurePosixPath(transport['remote_root']) / name)
    transfer(direction='upload', local_root=str(output), local_directory=name,
             remote_root=transport['remote_root'], remote_directory=name,
             expected=expected, endpoint=transport['endpoint'],
             **({'cancel':cancel} if cancel is not None else {}))
    # Only this new sealed input gets added. Existing roots, GPU and budgets stay exact.
    frozen = deepcopy(contract)
    pins = frozen.setdefault('provenance', {}).setdefault('input_sha256', {})
    for filename in names:
        pins[str(PurePosixPath(remote) / filename)] = hashlib.sha256((capture_dir / filename).read_bytes()).hexdigest()
    contract_path = output / 'acquisition-contract.json'
    pipeline_path = output / 'acquisition-pipeline.json'
    pipeline['contract_path'] = str(contract_path)
    pipeline_path.write_text(json.dumps(pipeline, indent=2, allow_nan=False))
    if 'config_sha256' in frozen['provenance']:
        frozen['provenance']['config_sha256'] = {pipeline_path.name: hashlib.sha256(pipeline_path.read_bytes()).hexdigest()}
    contract_path.write_text(json.dumps(frozen, indent=2, allow_nan=False))
    return dict(pipeline_config=str(pipeline_path), capture_dir=remote,
                physical_binding=remote + '/acquisition-binding.json')
