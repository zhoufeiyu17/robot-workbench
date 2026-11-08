"""Independent checkpoint identities; no GPU or robot access."""
from copy import deepcopy
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

import numpy as np

from robot_workbench.inference_model import (
    COFFEE999_ID, COFFEE999_PINS, MIXED366_ID, CAMS,
    model_family, model_spec, model_bundle_digest, validate_bundle, validate_worker_identity,
)
from robot_workbench.inference_transport import _validate_request, PROTOCOL
from robot_workbench.mixed366 import descriptor, Service


REG = dict(bundle_sha256='a'*64, source_manifest_sha256='b'*64, runtime_sha256='c'*64)


class Coffee999CatalogTests(unittest.TestCase):
    def bundle(self):
        return descriptor(dict(REG, model_id=COFFEE999_ID))

    def test_own_weights_stats_epoch_and_identity(self):
        b = self.bundle(); spec = model_spec(COFFEE999_ID, b)
        self.assertEqual(model_family(COFFEE999_ID), 'mixed366')
        self.assertEqual((spec['epoch'], spec['step'], spec['tasks']), (5, 32120, ('coffee',)))
        self.assertEqual(spec['pins'], COFFEE999_PINS)
        self.assertNotEqual(model_bundle_digest(COFFEE999_ID, b), model_bundle_digest(MIXED366_ID, descriptor(REG)))
        for field in ('checkpoint', 'metadata', 'stats'):
            bad = deepcopy(b); bad['artifacts'][field] = descriptor(REG)['artifacts'][field]
            with self.assertRaises(ValueError): validate_bundle(bad)
        from robot_workbench.inference_model import worker_identity
        with self.assertRaises(ValueError):
            validate_worker_identity(worker_identity(MIXED366_ID, descriptor(REG)), COFFEE999_ID, bundle=b)

    def test_only_coffee_and_empty_history_are_valid(self):
        b = self.bundle()
        metadata = dict(protocol=PROTOCOL, model_id=COFFEE999_ID, conversion_id='0'*64,
            request_id='fixture', seed=0, source_time=0., frame_ids={}, task='coffee',
            backend='torch', expected_bundle_digest=model_bundle_digest(COFFEE999_ID, b))
        arrays = dict(model_state14=np.zeros(14, np.float32), model_history=np.empty((0,14), np.float32),
            **{c:np.zeros((480,640,3), np.uint8) for c in CAMS})
        _validate_request(metadata, arrays, COFFEE999_ID, b)
        with self.assertRaises(ValueError): _validate_request(dict(metadata, task='lid_handoff'), arrays, COFFEE999_ID, b)
        with self.assertRaises(ValueError): _validate_request(metadata, dict(arrays, model_history=np.zeros((1,14))), COFFEE999_ID, b)

    def test_catalog_retains_old_model_and_adds_torch_only(self):
        with TemporaryDirectory() as tmp:
            root = Path(tmp); (root/'jobs').mkdir()
            extra = dict(REG, model_id=COFFEE999_ID, root=str(root/'new'))
            path = root/'new-registry.json'; path.write_text(json.dumps(extra)); path.chmod(0o600)
            service = Service(dict(REG, root=str(root), additional_registries=[str(path)]))
            entries = {(x['model_id'], x['backend']) for x in service.models()}
            self.assertEqual(entries, {(MIXED366_ID,'torch'), (COFFEE999_ID,'torch')})
            client = service.client_factory(self.bundle())
            self.assertEqual(client.model_id, COFFEE999_ID)
            self.assertEqual(client.registry['root'], str(root/'new'))


if __name__ == '__main__': unittest.main()
