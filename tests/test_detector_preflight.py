"""CPU release failures must precede plant/C/setup; optional real-runtime fixtures."""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT/'scripts'), str(ROOT/'rule_based_workbench')]
from detector_adapter.preflight_release import preflight_release
from rule_based_workbench.launch_environment import clean_environment
import serve_physx_device
import setup_resident_collection


class PreflightOrderingTest(unittest.TestCase):
    def test_resident_and_dry_run_reject_before_output_or_isaac(self):
        with TemporaryDirectory() as temp:
            root = Path(temp);base = root/'base.json';output = root/'run'
            pipeline = dict(detection_python=sys.executable, detection_node_config=str(root/'node.json'))
            base.write_text(json.dumps(dict(pipeline_config=pipeline,
                lanes={'source': {'configuration': {'cameras': {
                    'calibration_binding': dict(a1_sha256='sealed', geometry_id='A1')}}}})))
            spec = dict(collection_setup={'detection_runtime_root': str(root/'runtime')})
            argv = ['serve', '--base', str(base), '--nominal-config', str(root/'nominal.json'),
                '--output', str(output), '--gpu', '1', '--collection-contract', str(root/'contract.json')]
            for dry_run in (False, True):
                with self.subTest(dry_run=dry_run), patch.object(sys, 'argv', argv+(['--dry-run'] if dry_run else [])), \
                        patch('robot_workbench.resident_collection.load_collection_contract', return_value=spec), \
                        patch.object(serve_physx_device, 'nominal_inputs', return_value=dict(asset_seal_sha256='sealed', asset_geometry_id='A1')), \
                        patch.object(serve_physx_device, 'preflight', return_value={}), \
                        patch('detector_adapter.preflight_release.preflight_release', side_effect=RuntimeError('release missing')) as check, \
                        patch.object(serve_physx_device, 'run') as run:
                    with self.assertRaisesRegex(RuntimeError, 'release missing'):serve_physx_device.main()
                    check.assert_called_once_with(pipeline, spec['collection_setup']['detection_runtime_root'])
                    run.assert_not_called();self.assertFalse(output.exists())
                    self.assertNotIn('isaaclab.app', sys.modules)

    def test_setup_rejects_before_preparation_and_output(self):
        with TemporaryDirectory() as temp:
            root = Path(temp).resolve();output = root/'setup'
            spec = dict(collection_setup={'detection_runtime_root': str(root/'runtime')})
            parent = dict(service={}, execution={}, base={'pipeline_config': {}})
            for run in (False, True):
                args = SimpleNamespace(resident=root, output=output, contract=root/'contract.json', run=run)
                with self.subTest(run=run), \
                        patch('robot_workbench.resident_collection.load_collection_contract', return_value=spec), \
                        patch.object(setup_resident_collection, 'live_parent', return_value=parent), \
                        patch('detector_adapter.preflight_release.preflight_release', side_effect=RuntimeError('release missing')), \
                        patch('robot_workbench.collection_setup.prepare_collection_setup') as prepare:
                    with self.assertRaisesRegex(RuntimeError, 'release missing'):setup_resident_collection.run(args)
                    prepare.assert_not_called();self.assertFalse(output.exists())


@unittest.skipUnless(os.environ.get('DETECTOR_TEST_RUNTIME_ROOT'),
    'set DETECTOR_TEST_RUNTIME_ROOT to the external detector source for real closed-release tests')
class RealReleasePreflightTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.runtime = Path(os.environ['DETECTOR_TEST_RUNTIME_ROOT']).resolve(strict=True)
        source = cls.runtime/'ros2/online_detection_ros/test/test_release_catalog.py'
        spec = importlib.util.spec_from_file_location('detector_release_fixture', source)
        cls.fixture = importlib.util.module_from_spec(spec);spec.loader.exec_module(cls.fixture)

    def fixture_config(self, root):
        fixture = self.fixture.ReleaseFixture(root/'release')
        # Fill the DEVELOPMENT fixture's optional refs so the real validator must
        # verify them too. These are tiny text fixtures, not model weights.
        for key in ('provider_config', 'sensor_profile'):
            ref = fixture.release_value[key];path = fixture.root/ref['path']
            value = json.loads(path.read_bytes())
            nested = value['detection_providers'][0]['model'] if key == 'provider_config' else value['calibration']
            payload = fixture.root/nested['path'];payload.parent.mkdir(parents=True, exist_ok=True)
            payload.write_bytes(b'CPU contract fixture')
            nested['digest'] = self.fixture.digest_file(payload)
            self.fixture.write_json(path, value);ref['digest'] = self.fixture.digest_file(path)
        self.fixture.write_json(fixture.release_path, fixture.release_value)
        node = root/'node.json'
        params = dict(release_root=str(fixture.root), release_manifest_path='release.yaml',
            release_manifest_digest=self.fixture.digest_file(fixture.release_path),
            targets_path='targets.yaml', runtime_mode='DEVELOPMENT')
        node.write_text(json.dumps({'online_detection_node': {'ros__parameters': params}}))
        return fixture, node, params, dict(detection_python=sys.executable, detection_node_config=str(node))

    def test_real_catalog_rejects_missing_root_digest_and_open_or_changed_payload(self):
        for failure, message in (('root', 'RELEASE_ROOT_MISSING'), ('digest', 'RELEASE_MANIFEST_DIGEST_MISMATCH'),
                ('payload', 'DIGEST_MISMATCH'), ('extra', 'EXTRA_FILE'), ('optional', 'ARTIFACT_NOT_VERIFIED')):
            with self.subTest(failure=failure), TemporaryDirectory() as temp:
                fixture, node, params, config = self.fixture_config(Path(temp).resolve())
                if failure == 'root':params['release_root'] += '-missing'
                elif failure == 'digest':params['release_manifest_digest'] = 'sha256:'+'0'*64
                elif failure == 'payload':
                    payload = fixture.root/'third_party/FoundationPose/weights'/self.fixture.FOUNDATIONPOSE_MODEL_FILES[0]
                    data = payload.read_bytes();payload.write_bytes(data[::-1])
                elif failure == 'extra':(fixture.root/'third_party/FoundationPose/undeclared.py').write_text('# unsealed')
                else:(fixture.root/'calibrations/camera/1.0.0').unlink()
                node.write_text(json.dumps({'online_detection_node': {'ros__parameters': params}}))
                with self.assertRaisesRegex(RuntimeError, message):preflight_release(config, self.runtime)

    def test_real_success_cleans_parent_runtime_and_keeps_release_unchanged(self):
        with TemporaryDirectory() as temp:
            fixture, node, params, config = self.fixture_config(Path(temp).resolve())
            def snapshot():return {str(p): hashlib.sha256(p.read_bytes()).hexdigest()
                for p in Path(temp).rglob('*') if p.is_file()}
            before = snapshot()
            with patch.dict(os.environ, dict(PYTHONHOME='/invalid-isaac', PYTHONPATH='/invalid-isaac', CUDA_VISIBLE_DEVICES='7')):
                receipt = preflight_release(config, self.runtime)
                self.assertEqual(os.environ['PYTHONHOME'], '/invalid-isaac')
                self.assertEqual(os.environ['CUDA_VISIBLE_DEVICES'], '7')
            self.assertEqual(receipt['release_root'], str(fixture.root.resolve()))
            self.assertEqual(receipt['release_manifest_digest'], params['release_manifest_digest'])
            self.assertGreater(receipt['verified_artifact_count'], 0)
            self.assertEqual(before, snapshot())
            # Invoke the same read-only function in a fresh CPU process with
            # import/write/network guards; guards are tests, never runtime policy.
            script = '''import sys,os,json
from pathlib import Path
sys.path.insert(0,sys.argv[1])
from detector_adapter.preflight_release import read_release
class NoGpu:
 def find_spec(self,name,*args):
  if name.split('.')[0] in {'torch','torchvision','warp','nvdiffrast','cupy','isaaclab','omni','estimater','mycpp'}:raise AssertionError('model/GPU import '+name)
sys.meta_path.insert(0,NoGpu())
def readonly(event,args):
 if event=='open' and args[2] & (os.O_WRONLY|os.O_RDWR|os.O_CREAT|os.O_TRUNC):raise AssertionError('write '+str(args[0]))
 if event.startswith(('socket.','subprocess.')) or event in {'os.mkdir','os.remove','os.rename'}:raise AssertionError(event)
sys.addaudithook(readonly)
value=read_release(Path(sys.argv[2]),Path(sys.argv[3]))
assert 'torch' not in sys.modules
assert os.environ['CUDA_VISIBLE_DEVICES']==''
print(json.dumps(value))
'''
            result = subprocess.run([sys.executable, '-I', '-B', '-c', script, str(ROOT), str(node), str(self.runtime)],
                env=clean_environment({'CUDA_VISIBLE_DEVICES': ''}), text=True, capture_output=True, timeout=30)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(json.loads(result.stdout)['release_manifest_digest'], params['release_manifest_digest'])


if __name__ == '__main__':unittest.main()
