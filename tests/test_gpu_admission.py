"""CPU-only GPU admission, desktop policy and shared-lock regression."""
import json
import hashlib
import os
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch
from types import ModuleType

from robot_workbench.gpu_admission import GpuAdmissionError, acquire, inspect, lock_path, registered_desktop


class AdmissionTests(unittest.TestCase):
    def test_native_adapter_reuses_shared_lock_and_existing_close(self):
        from scripts.serve_base566_model import create_service
        module=ModuleType('manipulation.runtime.workbench_client')
        class NativeClient:
            model_id='native-test'
            min_free_memory_mib=16000
            def close(self):self._gpu_lock.close()
        module.ResidentBase566Client=NativeClient
        with TemporaryDirectory() as folder:
            root=Path(folder);source=root/'source';(source/'scripts').mkdir(parents=True)
            (source/'scripts/serve_base566_model.py').write_text('def create_service(config): return ResidentBase566Client()\n')
            registry=dict(workspace=str(root),native_source=str(source),owner_session='test',gpu_uuid='GPU-test',models=[],
                resource_policy=dict(mode='host_exclusive',machine_id_sha256=hashlib.sha256(b'machine').hexdigest()))
            with patch.dict('sys.modules',{'manipulation.runtime.workbench_client':module}), \
                    patch('robot_workbench.gpu_admission.acquire') as acquire_gpu:
                client=create_service(registry,'machine');client._acquire_gpu()
                self.assertEqual(acquire_gpu.call_args.args[0]['gpu_uuid'],'GPU-test')
                client.close();acquire_gpu.return_value.close.assert_called_once()

    def test_distinct_models_share_the_same_gpu_lock(self):
        with TemporaryDirectory() as folder, patch.dict(os.environ, WORKBENCH_GPU_LOCK_DIR=folder), \
                patch('robot_workbench.gpu_admission.inspect', return_value={'gpu_uuid':'GPU-test'}) as probe:
            first=acquire(dict(root='/one', model_id='first', gpu_uuid='GPU-test'))
            try:
                with self.assertRaises(GpuAdmissionError) as error:
                    acquire(dict(root='/two', model_id='second', gpu_uuid='GPU-test'))
                self.assertEqual(error.exception.details['code'],'GPU_BUSY')
                self.assertEqual(error.exception.details['holder']['model_id'],'first')
                self.assertEqual(probe.call_count,1)
                other=acquire(dict(root='/two',gpu_uuid='GPU-other'))
                other.close()
            finally:first.close()
            acquire(dict(root='/two',gpu_uuid='GPU-test')).close()

    def test_desktop_policy_does_not_depend_on_checkpoint_pids(self):
        with TemporaryDirectory() as folder:
            root=Path(folder).resolve();p=root/'123';p.mkdir();exe=root/'browser';exe.touch()
            (p/'exe').symlink_to(exe);(p/'cmdline').write_bytes(b'browser\0--type=gpu-process\0')
            (p/'stat').write_text('123 (browser) '+' '.join(['S']+['0']*18+['100']))
            (p/'cgroup').write_text('0::/user.slice/user-1000.slice/user@1000.service/app.slice/app-browser-123.scope\n')
            policy=root/'policy.json';policy.write_text(json.dumps(dict(schema='workbench.gpu-admission.v1',desktop_executables=[str(exe)])));policy.chmod(0o600)
            with patch.dict(os.environ,WORKBENCH_GPU_POLICY=str(policy)):
                self.assertTrue(registered_desktop('123',{},root))
                (p/'cgroup').write_text('0::/system.slice/training.service\n')
                self.assertFalse(registered_desktop('123',{},root))

    def test_replacement_reclaims_only_its_own_memory_and_reports_foreign_process(self):
        with TemporaryDirectory() as folder, patch.dict(os.environ,WORKBENCH_GPU_LOCK_DIR=folder), \
                patch('robot_workbench.gpu_admission.subprocess.check_output',side_effect=['GPU-test, 8000','11, 14000\n12, 400\n']), \
                patch('robot_workbench.gpu_admission.registered_desktop',side_effect=[False,True]):
            value=inspect(dict(gpu_uuid='GPU-test',min_free_memory_mib=16000),replacing_pids=(11,))
            self.assertEqual(value['available_after_replace_mib'],22000)
        with TemporaryDirectory() as folder, patch.dict(os.environ,WORKBENCH_GPU_LOCK_DIR=folder), \
                patch('robot_workbench.gpu_admission.subprocess.check_output',side_effect=['GPU-test, 22000','12, 2000\n']), \
                patch('robot_workbench.gpu_admission.registered_desktop',return_value=False):
            with self.assertRaises(GpuAdmissionError) as error:inspect(dict(gpu_uuid='GPU-test'))
            self.assertEqual(error.exception.details['processes'][0]['pid'],12)
            self.assertIn('2000 MiB',str(error.exception))

    def test_failed_admission_releases_the_lock(self):
        with TemporaryDirectory() as folder, patch.dict(os.environ,WORKBENCH_GPU_LOCK_DIR=folder):
            with patch('robot_workbench.gpu_admission.inspect',side_effect=ValueError('probe failed')):
                with self.assertRaises(ValueError):acquire(dict(gpu_uuid='GPU-test'))
            with patch('robot_workbench.gpu_admission.inspect',return_value={}):
                acquire(dict(gpu_uuid='GPU-test')).close()
            self.assertEqual(lock_path('GPU-test'),Path(folder)/'GPU-test.lock')


if __name__=='__main__':unittest.main()
