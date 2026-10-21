"""Pinned, offline model references on the existing inference execution path."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path

import numpy as np


def load_reference(pin):
    if not isinstance(pin,dict) or set(pin)!={'path','sha256'} or not Path(pin['path']).is_absolute():
        raise ValueError('absolute pinned inference reference required')
    path=Path(pin['path'])
    if path.is_symlink() or not 0<path.stat().st_size<=256*1024:
        raise ValueError('bounded regular inference reference required')
    raw=path.read_bytes()
    if hashlib.sha256(raw).hexdigest()!=pin['sha256']:raise ValueError('inference reference SHA256 changed')
    value=json.loads(raw);q=np.asarray(value.get('device_targets14'),dtype=float)
    if (value.get('schema')!='workbench.inference_reference.v1' or value.get('duration_s')!=2
            or value.get('hz')!=30 or q.shape!=(61,14) or not np.isfinite(q).all()
            or not isinstance(value.get('calibration_profile_digest'),str)
            or not isinstance(value.get('model_id'),str)):
        raise ValueError('2-second reference requires initial pose and 60 finite physical device rows at 30 Hz')
    # This first experiment preserves the existing left-only inference policy.
    if not np.allclose(q[:,7:],q[0,7:],rtol=0,atol=1e-9):
        raise ValueError('reference must hold right arm and disabled jaw')
    velocity=np.diff(q[:,:6],axis=0)*30
    if np.max(np.abs(velocity))>2.+1e-9:
        raise ValueError('reference exceeds the 2rad/s research velocity bound')
    if np.max(np.abs(np.diff(q[:,6])))*30>40.+1e-9:
        raise ValueError('reference jaw speed exceeds 40mm/s')
    return value


class ReferencePredictions:
    """Replay the saved chunk; excess queries explicitly hold the final row."""
    def __init__(self,reference,adapter,pin):
        self.rows=np.asarray([adapter.device_to_model(row) for row in reference['device_targets14'][1:]],dtype=float)
        self.pin=deepcopy(pin);self.index=0

    def infer(self,request):
        first=self.index*60;last=min(first+60,len(self.rows));self.index+=1
        rows=self.rows[first:last]
        if not len(rows):rows=np.repeat(self.rows[-1:],60,axis=0)
        return dict(request_id=request['request_id'],action14=rows.copy(),audit=dict(
            source='frozen_model_reference',reference=deepcopy(self.pin),
            reference_first_row=first+1,reference_last_row=last,
            final_hold_padding=first>=len(self.rows),model_called=False))
