"""Frozen research references preserve identity, bounds and chunk order."""
import hashlib
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

import numpy as np

from robot_workbench.inference_reference import load_reference,ReferencePredictions


class ReferenceTests(unittest.TestCase):
    def test_pin_bounds_and_saved_chunk(self):
        q=np.zeros((61,14));q[:,0]=np.linspace(0,.2,61)
        value=dict(schema='workbench.inference_reference.v1',duration_s=2,hz=30,
            calibration_profile_digest='a'*64,model_id='base566-epoch007-step58982',device_targets14=q.tolist())
        with tempfile.TemporaryDirectory()as folder:
            path=Path(folder)/'reference.json'
            def pin():
                raw=json.dumps(value).encode();path.write_bytes(raw)
                return dict(path=str(path),sha256=hashlib.sha256(raw).hexdigest())
            reference=load_reference(pin());provider=ReferencePredictions(reference,SimpleNamespace(device_to_model=lambda q:np.asarray(q)),pin())
            chunks=[provider.infer(dict(request_id='saved'))]
            np.testing.assert_allclose(np.concatenate([r['action14']for r in chunks]),q[1:])
            self.assertFalse(chunks[-1]['audit']['model_called'])
            held=provider.infer(dict(request_id='tail'));self.assertTrue(held['audit']['final_hold_padding'])
            np.testing.assert_allclose(held['action14'],np.repeat(q[-1:],60,axis=0))
            bad=pin();bad['sha256']='0'*64
            with self.assertRaisesRegex(ValueError,'SHA256'):load_reference(bad)
            value['device_targets14'][30][7]=.01
            with self.assertRaisesRegex(ValueError,'hold right'):load_reference(pin())
            value['device_targets14']=q.tolist();value['device_targets14'][30][0]=2.
            with self.assertRaisesRegex(ValueError,'velocity bound'):load_reference(pin())
            value['device_targets14']=q.tolist();value['device_targets14'][30][0]=float('nan')
            with self.assertRaisesRegex(ValueError,'finite'):load_reference(pin())


if __name__=='__main__':unittest.main()
