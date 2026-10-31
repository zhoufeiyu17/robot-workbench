"""Explicit native-runtime identities and real HTTP round trips; no GPU or robot."""
from copy import deepcopy
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import threading
import unittest
from unittest.mock import patch
import numpy as np

from robot_workbench.inference_model import (BASE566_PINS, CAMS, MANIPULATION_SCHEMA,
    model_bundle_digest, model_descriptor, validate_bundle, validate_worker_identity, worker_identity)
from robot_workbench.inference_transport import ModelService, RemoteBaseClient, serve_model, PROTOCOL
from test_inference_transport import Adapter, request


def descriptor(backend='torch'):
    return dict(schema=MANIPULATION_SCHEMA,model_id='base566-epoch008-step67408',epoch=8,step=67408,
        backend=backend,artifacts={k:dict(sha256=BASE566_PINS[k]if k in('stats','tokenizer')else'a'*64)
            for k in('checkpoint','metadata','stats','tokenizer','io_contract','model_config')},
        model_bundle_sha256='b'*64,runtime=dict(adapter='manipulation_process_model_client.v1',
            source_manifest_sha256='c'*64,frozen_model_source_sha256='d'*64,
            triton_deployment_sha256='e'*64 if backend=='triton'else None))


class NativeRuntimeTests(unittest.TestCase):
    def test_descriptor_binds_actual_runtime_without_claiming_legacy_worker(self):
        a,b=descriptor(),descriptor('triton')
        self.assertNotEqual(model_bundle_digest(a['model_id'],a),model_bundle_digest(b['model_id'],b))
        changed=deepcopy(a);changed['runtime']['source_manifest_sha256']='f'*64
        self.assertNotEqual(model_bundle_digest(a['model_id'],a),model_bundle_digest(a['model_id'],changed))
        self.assertEqual(validate_bundle(a),a)
        with self.assertRaisesRegex(ValueError,'registered host artifacts'):validate_bundle(a,verify_files=True)
        changed['artifacts']['checkpoint']['path']='/private/weights'
        with self.assertRaises(ValueError):model_descriptor(changed['model_id'],changed)
        identity=worker_identity(b['model_id'],b);self.assertFalse(identity['strict_load']);identity['packed_sha256']='f'*64
        self.assertNotIn('process',identity);self.assertNotIn('source_manifest_sha256',identity)
        validate_worker_identity(identity,b['model_id'],bundle=b)
        identity['strict_load']=True
        with self.assertRaises(ValueError):validate_worker_identity(identity,b['model_id'],bundle=b)

    def test_same_checkpoint_modes_reuse_worker_and_reject_stale_packets_and_unload(self):
        made=[]
        class Client:
            def __init__(self,bundle):
                self.bundle=deepcopy(bundle);self.model_id=bundle['model_id'];self.alive=False;self.prepares=0;self.closes=0;self.requests=[];made.append(self)
            def prepare(self):
                self.prepares+=1;self.alive=True;identity=worker_identity(self.model_id,self.bundle)
                if self.bundle['backend']=='triton':identity['packed_sha256']='f'*64
                return identity
            def infer(self,req):
                self.requests.append(req)
                return dict(request_id=req['request_id'],action14=np.full((60,14),req['seed'],np.float32),audit=dict(seed=req['seed']))
            def worker_status(self):return dict(worker_pid=1234 if self.alive else None,worker_exit_code=None,worker_alive=self.alive)
            def close(self):self.closes+=1;self.alive=False
        catalog={x:descriptor(x)for x in('torch','triton')};service=ModelService(Client(catalog['torch']),catalog=catalog,client_factory=Client)
        with TemporaryDirectory()as folder:
            token=Path(folder)/'token';token.write_text('x'*40);server=serve_model(service,token_file=token,port=0)
            thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
            try:
                endpoint=f'http://127.0.0.1:{server.server_port}'
                clients={k:RemoteBaseClient(endpoint,token_file=token,adapter=Adapter(),conversion_identity={'device':'cpu'},model_id=v['model_id'],model_bundle=v)for k,v in catalog.items()}
                listed=clients['torch'].models();self.assertEqual({x['backend']for x in listed},{'torch','triton'})
                clients['torch'].prepare();clients['torch'].prepare();self.assertEqual(made[0].prepares,1)
                self.assertTrue(clients['torch'].status()['worker_alive'])
                for seed in (7,8):np.testing.assert_array_equal(clients['torch'].infer(request(seed=seed))['action14'],seed)
                old=made[0];clients['triton'].prepare();self.assertEqual(old.closes,1);current=made[-1]
                with self.assertRaisesRegex(RuntimeError,'backend'):clients['torch'].infer(request('stale'))
                self.assertEqual(current.closes,0)
                meta=dict(protocol=PROTOCOL,model_id=catalog['torch']['model_id'],conversion_id=clients['torch'].conversion_id,
                          expected_bundle_digest=clients['torch'].bundle_digest,backend='torch')
                with self.assertRaisesRegex(ValueError,'current model/backend'):service.unload(meta)
                self.assertEqual(current.closes,0)
                meta.update(expected_bundle_digest=clients['triton'].bundle_digest,backend='triton')
                with service.lock:
                    with self.assertRaisesRegex(ValueError,'busy'):service.unload(meta)
                response=json.loads(clients['triton']._request('/unload',json.dumps(meta).encode()))
                self.assertTrue(response['worker_exited']);self.assertFalse(service.status()['loaded']);self.assertEqual(current.closes,1)
            finally:server.shutdown();server.server_close();thread.join(2);service.close()


if __name__=='__main__':unittest.main()
