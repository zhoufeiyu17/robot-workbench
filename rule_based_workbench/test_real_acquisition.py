"""Synthetic atomic RGB-D -> sealed upload -> exact contract pins; no hardware."""
from copy import deepcopy
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch
import time

from integration import _digest
from real_acquisition import prepare_inputs, validate_config
from real_observation import capture as original_capture
from test_harness import PROFILE
from test_real_observation import fixture


def main():
    with TemporaryDirectory() as directory:
        root = Path(directory).resolve()
        remote = root / 'remote-inputs'
        remote.mkdir()
        profile = deepcopy(PROFILE)
        binding = dict(calibration_accepted=True, evidence_source='physical_rgbd', robot_id='fixture',
                       camera_serials=dict(head='h', left='l', right='r'),
                       calibration_profile_digest=_digest(profile),
                       acceptance=dict(status='user_accepted_provisional', normal_acceptance_passed=False))
        for name, value in [('profile', profile), ('binding', binding)]:
            (root / (name + '.json')).write_text(json.dumps(value))
        contract = dict(filesystem=dict(writable_roots=[str(remote)], readonly_roots=[str(remote)]),
                        resources=dict(gpu='unchanged'), provenance=dict(input_sha256={}, config_sha256={}))
        (root / 'contract.json').write_text(json.dumps(contract))
        (root / 'pipeline.json').write_text(json.dumps(dict(contract_path=str(root / 'contract.json'),remote_run_root='/separate/run')))
        config = dict(pipeline_config=str(root / 'pipeline.json'), acquisition=dict(
            profile_path=str(root / 'profile.json'), acquisition_binding_path=str(root / 'binding.json'),
            transport=dict(remote_root=str(remote), endpoint=None)))
        frames = iter(fixture(i) for i in range(9))
        def fast_capture(*args):
            clock=[0.]
            return original_capture(*args,monotonic=lambda:clock[0],sleep=lambda s:clock.__setitem__(0,clock[0]+s))
        with patch('real_observation.capture', fast_capture):
            result = prepare_inputs(config, root / 'run', read_snapshot=lambda: next(frames))
        uploaded = Path(result['capture_dir'])
        sealed = json.loads(Path(result['physical_binding']).read_text())
        assert sealed['acceptance'] == binding['acceptance']
        pipeline = json.loads(Path(result['pipeline_config']).read_text())
        frozen = json.loads(Path(pipeline['contract_path']).read_text())
        assert frozen['filesystem'] == contract['filesystem'] and frozen['resources'] == contract['resources']
        assert len(frozen['provenance']['input_sha256']) == 12
        assert len(list(uploaded.glob('*.npz'))) == 9
        assert json.loads((root / 'contract.json').read_text()) == contract
        from test_initial_state import owner
        measured = deepcopy(config)
        measured['acquisition']['measured_initial_state'] = True
        frames = iter(fixture(i) for i in range(9))
        samples = []
        def read_owner():
            value = owner(stamp=time.time_ns())
            samples.append(value)
            return value
        with patch('real_observation.capture', fast_capture):
            captured = prepare_inputs(measured, root / 'measured', read_snapshot=lambda:next(frames), read_owner=read_owner)
        start = json.loads(Path(captured['physical_binding']).read_text())['initial_state']
        assert len(samples) == 2 and start['owner_samples'][0]['pid'] == 1234
        assert start['measured_reported'] == samples[0]['physical_feedback']
        assert start['owner_samples'][1]['sample_wall_ns'] > start['owner_samples'][0]['sample_wall_ns']
        bad = deepcopy(config)
        bad['acquisition']['transport']['remote_root'] = '/outside/authorized/roots'
        try:
            validate_config(bad)
        except ValueError:
            pass
        else:
            raise AssertionError('expanded upload scope accepted')
        with patch('real_observation.capture', side_effect=ValueError('bad frame')):
            try:
                prepare_inputs(config, root / 'failed', read_snapshot=lambda:b'bad')
            except ValueError:
                pass
            else:
                raise AssertionError('failed acquisition accepted')
        assert not (root / 'failed/acquisition-pipeline.json').exists()
    print('Automatic physical observation, transfer, exact pins and no stale fallback: PASS')


if __name__ == '__main__':
    main()
