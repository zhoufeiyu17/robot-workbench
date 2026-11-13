"""Runtime drift is rejected before replacing a healthy model worker."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import Mock

sys.path[:0]=[str(Path(__file__).resolve().parents[1]),str(Path(__file__).resolve().parent)]
from robot_workbench import model_runtime as runtime


class RuntimeTests(unittest.TestCase):
    def test_legacy_interpreter_resolution_requires_a_unique_environment(self):
        with TemporaryDirectory() as directory:
            root=Path(directory);first=root/'env/anyinfer-legacy/bin/python'
            first.parent.mkdir(parents=True);first.touch()
            registry=dict(root=directory,backend='anyinfer')
            self.assertEqual(runtime.interpreter(registry),first)
            other=root/'env/anyinfer-other/bin/python';other.parent.mkdir(parents=True);other.touch()
            with self.assertRaisesRegex(ValueError,'unambiguous'):runtime.interpreter(registry)
            registry['python']=str(first);self.assertEqual(runtime.interpreter(registry),first)

    def test_declared_interpreter_inventory_and_version_drift(self):
        expected=runtime.capture(sys.executable)
        self.assertEqual(runtime.verify(sys.executable,expected),expected)
        for field in ('python','architecture','packages'):
            changed=deepcopy(expected)
            if field=='packages':changed[field]['synthetic-runtime-dependency']='1.0'
            else:changed[field]='other'
            with self.subTest(field=field),self.assertRaisesRegex(ValueError,'version lock mismatch'):
                runtime.verify(sys.executable,changed)

    def test_manifest_bound_lock_rejects_missing_and_modified_content(self):
        with TemporaryDirectory() as directory:
            registry=dict(root=directory,python=sys.executable,backend='torch',files=[])
            with self.assertRaisesRegex(ValueError,'lock missing'):runtime.verify_registered(registry)
            path=Path(directory)/runtime.lock_path('torch');path.parent.mkdir()
            raw=json.dumps(runtime.capture(sys.executable)).encode();path.write_bytes(raw)
            registry['files']=[dict(path=runtime.lock_path('torch'),bytes=len(raw),sha256=hashlib.sha256(raw).hexdigest())]
            runtime.verify_registered(registry)
            path.write_bytes(raw.replace(b'"schema"',b'"xxxxxx"'))
            with self.assertRaisesRegex(ValueError,'content changed'):runtime.verify_registered(registry)

    def test_bad_candidate_lock_preserves_loaded_worker(self):
        from test_wam_mem import Worker
        from robot_workbench.inference_transport import ModelService,PROTOCOL
        from robot_workbench.inference_model import model_bundle_digest
        old=Worker();new=Worker()
        new.model_id='wam-mem-other-step1';new.bundle['model_id']=new.model_id
        new.registry=dict(root='/unused',backend='torch',files=[])
        service=ModelService(old,catalog={'old':old.bundle,'new':new.bundle},client_factory=lambda _:new)
        def request(worker):
            return dict(protocol=PROTOCOL,model_id=worker.model_id,backend='torch',conversion_id='a'*64,
                        expected_bundle_digest=model_bundle_digest(worker.model_id,worker.bundle))
        service.prepare(request(old))
        old.close=Mock()
        with self.assertRaisesRegex(ValueError,'lock missing'):service.prepare(request(new))
        self.assertIs(service.client,old);old.close.assert_not_called()
        self.assertTrue(service.status()['loaded'])


if __name__=='__main__':unittest.main()
