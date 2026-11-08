"""Registered host admission and uncalibrated control refusal; no GPU/SDK."""
import hashlib
import importlib.util
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from types import SimpleNamespace

spec=importlib.util.spec_from_file_location('registered_model',Path(__file__).resolve().parents[1]/'scripts/serve_base566_model.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)

class RegisteredModelTest(unittest.TestCase):
    def test_pending_mapping_blocks_run_before_control_or_model_side_effects(self):
        from robot_workbench.service import Application
        def forbidden(*args):raise AssertionError('mapping refusal must precede all control/model work')
        flow=SimpleNamespace(bind_device=forbidden,preview_episode=forbidden,execute_review=forbidden,
                             snapshot=forbidden,control_mapping_pending='machine mapping not qualified')
        app=SimpleNamespace(operator_tasks={})
        device=SimpleNamespace(environment='physical')
        self.assertEqual(Application.inference_run_reason(app,device,flow,{}),'machine mapping not qualified')

    def test_host_binding_and_explicit_policy_are_required(self):
        with TemporaryDirectory()as directory:
            root=Path(directory);source=root/'native';(source/'scripts').mkdir(parents=True)
            (source/'scripts/serve_base566_model.py').write_text('# fixture')
            registry=dict(workspace=str(root),native_source=str(source),owner_session='test',gpu_uuid='GPU-test',models={},
                resource_policy=dict(mode='host_exclusive',machine_id_sha256=hashlib.sha256(b'host').hexdigest()))
            self.assertEqual(module.validate_registry(registry,'host'),(root.resolve(),source.resolve()))
            with self.assertRaisesRegex(ValueError,'bound to this host'):module.validate_registry(registry,'other')
            registry['resource_policy']['mode']='skip_gpu_checks'
            with self.assertRaises(ValueError):module.validate_registry(registry,'host')

if __name__=='__main__':unittest.main()
