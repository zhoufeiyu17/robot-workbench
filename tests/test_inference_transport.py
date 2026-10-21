"""CPU-only real HTTP/NPZ tests; no model import, weights or GPU worker starts."""
from copy import deepcopy
from http.client import HTTPConnection
import io
import json
from pathlib import Path
import socket
from tempfile import TemporaryDirectory
import threading
import time
import unittest
from unittest.mock import patch
import zipfile

import numpy as np

from robot_workbench.inference_model import BASE379_PINS, CAMS
from robot_workbench.inference_model import BASE566_ID, model_spec, worker_identity
from robot_workbench.inference_transport import (MAX_PACKET, MODEL_ID, PROTOCOL, ModelDomainAdapter,
    ModelService, RemoteBaseClient, conversion_digest, decode_packet, encode_packet, request_packet, serve_model)


class StubModel:
    def __init__(self):
        self.prepares = self.closes = 0
        self.requests = []
        self.entered = threading.Event()
        self.release = None
        self.fail = False
    def prepare(self):
        self.prepares += 1
        return dict(epoch=10, step=29450, checkpoint_sha256=BASE379_PINS['checkpoint'], status='ready')
    def infer(self, request):
        self.requests.append(deepcopy(request));self.entered.set()
        if self.release is not None:self.release.wait(1)
        if self.fail:raise TimeoutError('owned model timed out')
        return dict(request_id=request['request_id'], action14=np.full((60, 14), request['seed'], np.float32),
                    audit=dict(history_length=len(request['applied_history'])))
    def close(self):
        self.closes += 1


class Adapter:
    def model_input(self, observation):
        return np.asarray(observation['device_state14'], np.float32)+.1, observation['images']


def request(identity='request-1', seed=7):
    return dict(request_id=identity, seed=seed, observation=dict(source_time=2., device_state14=np.zeros(14),
        images={name:np.full((224,224,3), index, np.uint8) for index,name in enumerate(CAMS)},
        frame_ids={name:'frame-'+str(index) for index,name in enumerate(CAMS)}),
        applied_history=np.arange(28, dtype=np.float32).reshape(2,14))


class InferenceTransportTests(unittest.TestCase):
    def test_base566_catalog_switch_is_explicit_serial_and_checkpoint_bound(self):
        def bundle(name):
            spec=model_spec(name)
            return dict(model_id=name,schema=spec['schema'],epoch=spec['epoch'],step=spec['step'],prompt=spec['prompt'],
                action_horizon=60,execution_steps=10,action_hz=30,rtc=False,python='/model/python',model_source='/model/code',
                artifacts={key:dict(path='/model/'+key,sha256=sha) for key,sha in spec['pins'].items()})
        fifth='base566-epoch005-step42130';catalog={name:bundle(name) for name in (BASE566_ID,fifth)}
        made=[]
        class SelectedModel(StubModel):
            def __init__(self,value):
                super().__init__();self.bundle=deepcopy(value);self.model_id=value['model_id'];made.append(self)
            def prepare(self):
                self.prepares+=1;return worker_identity(self.model_id,self.bundle)
        self.service.catalog=lambda:catalog
        self.service.client_factory=SelectedModel
        listed=self.client.list_models()
        self.assertEqual({row['model_id'] for row in listed},set(catalog))
        self.assertNotIn('/model/',json.dumps(listed),'public catalog must not include host paths')
        self.assertTrue(all(len(row['bundle_digest'])==64 for row in listed))
        self.service.catalog_rows=lambda:[dict(model_id=BASE566_ID,status='ready',selectable=True,derived_manifest=catalog[BASE566_ID]),
            dict(label='epoch008',status='downloading',selectable=False,reason='model file is incomplete')]
        self.assertEqual(self.client.models()[1]['status'],'downloading')
        self.assertNotIn('model_descriptor',self.client.models()[1])
        self.service.catalog_rows=None
        def remote(name):
            return RemoteBaseClient(self.url,token_file=self.token,adapter=Adapter(),conversion_identity=self.binding,
                model_id=name,model_bundle=catalog[name])
        seventh=remote(BASE566_ID);fifth_client=remote(fifth)
        self.assertNotEqual(seventh.conversion_id,fifth_client.conversion_id)
        seventh.prepare();self.assertEqual(self.model.closes,1)
        result=seventh.infer(request());self.assertEqual(result['audit']['transport']['model_id'],BASE566_ID)
        seventh.prepare();self.assertEqual(made[-1].prepares,1)
        with self.service.lock:
            with self.assertRaisesRegex(RuntimeError,'busy'):fifth_client.prepare()
        self.assertEqual(len(made),1);self.assertEqual(made[0].closes,0)
        fifth_client.prepare();self.assertEqual(made[0].closes,1)
        with self.assertRaisesRegex(RuntimeError,'identity'):seventh.infer(request('stale'))
        self.assertEqual(made[-1].closes,0,'stale model request must not close the current model')
        future=deepcopy(catalog[BASE566_ID]);future.update(model_id='base566-epoch008-step67408',epoch=8,step=67408)
        future['artifacts']['checkpoint']['sha256']='a'*64;future['artifacts']['metadata']['sha256']='b'*64
        catalog[future['model_id']]=future
        descriptor=next(row['model_descriptor'] for row in self.client.models() if row['model_id']==future['model_id'])
        next_client=RemoteBaseClient(self.url,token_file=self.token,adapter=Adapter(),conversion_identity=self.binding,
            model_id=future['model_id'],model_bundle=descriptor)
        self.assertEqual(next_client.prepare()['checkpoint_sha256'],'a'*64)
        with self.assertRaisesRegex(RuntimeError,'catalog'):self.client.prepare()
        self.assertEqual(next_client.infer(request('future'))['request_id'],'future')
        # Simulate a switch after decoding but before this request acquires the service lock.
        import robot_workbench.inference_transport as transport
        payload=request_packet(request('race'),Adapter(),next_client.conversion_id,future['model_id'],future)
        validate=transport._validate_request
        def switch_after_validation(*args):
            validate(*args)
            self.service.prepare(dict(protocol=PROTOCOL,model_id=fifth,conversion_id=fifth_client.conversion_id))
        with patch.object(transport,'_validate_request',side_effect=switch_after_validation):
            with self.assertRaisesRegex(ValueError,'identity'):self.service.infer(payload)
        self.assertEqual(made[-1].model_id,fifth);self.assertEqual(made[-1].closes,0)
        self.assertFalse(self.service.lock.locked())
        next_client.prepare()
        changed=deepcopy(future);changed['artifacts']['checkpoint']['sha256']='c'*64
        catalog[future['model_id']]=changed
        with self.assertRaisesRegex(RuntimeError,'bundle identity changed'):next_client.prepare()
        current=remote(future['model_id']);current.prepare()
        with self.assertRaisesRegex(RuntimeError,'bundle identity changed'):next_client.infer(request('wrong-bytes-same-step'))
        self.assertEqual(made[-1].closes,0)
        self.assertEqual(current.infer(request('current-bytes'))['request_id'],'current-bytes')

    def setUp(self):
        self.temp=TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.token=Path(self.temp.name)/'token';self.token.write_text('fixture-private-token-'+'x'*32)
        self.model=StubModel();self.service=ModelService(self.model)
        self.server=serve_model(self.service,token_file=self.token)
        self.thread=threading.Thread(target=self.server.serve_forever,daemon=True);self.thread.start()
        self.addCleanup(self.cleanup)
        self.url=f'http://127.0.0.1:{self.server.server_port}'
        self.binding=dict(device_id='cpu-fixture',calibration_snapshot='installed-1',jaw_mapping='a1')
        self.client=RemoteBaseClient(self.url,token_file=self.token,adapter=Adapter(),conversion_identity=self.binding)
    def cleanup(self):
        if self.model.release:self.model.release.set()
        self.server.shutdown();self.server.server_close();self.thread.join(2);self.service.close()

    def test_inert_prepare_once_and_roundtrip_model_domain_inputs(self):
        self.assertEqual(self.model.prepares,0)
        first=self.client.status()['fact'];second=self.client.status()['fact']
        self.assertEqual(first['instance_id'],second['instance_id']);self.assertEqual(first['revision'],second['revision'])
        self.assertFalse(self.client.status()['loaded'])
        with self.assertRaisesRegex(ValueError,'explicit remote'):self.client.infer(request())
        self.assertEqual(self.client.prepare()['checkpoint_sha256'],BASE379_PINS['checkpoint'])
        self.client.prepare();self.assertEqual(self.model.prepares,1)
        self.assertGreater(self.client.status()['fact']['revision'],first['revision'])
        self.assertEqual([row['request_id'] for row in self.model.requests],['warmup'])
        self.assertTrue(self.client.status()['warmup']['completed'])
        result=self.client.infer(request())
        self.assertEqual(result['request_id'],'request-1');self.assertEqual(result['audit']['history_length'],2)
        self.assertEqual(result['audit']['transport']['conversion_identity'],self.binding)
        received=self.model.requests[-1]
        self.assertNotIn('device_state14',received['observation'])
        np.testing.assert_array_equal(received['observation']['model_state14'],np.full(14,.1,np.float32))
        np.testing.assert_array_equal(received['applied_history'],request()['applied_history'])
        for name in CAMS:np.testing.assert_array_equal(received['observation']['images'][name],request()['observation']['images'][name])
        self.assertEqual(received['observation']['frame_ids'],request()['observation']['frame_ids'])
        self.assertEqual(result['action14'].shape,(60,14));self.assertTrue(np.all(result['action14']==7))
        self.client.close();self.assertEqual(self.model.closes,0,'remote client does not own the server process')

    def test_ready_only_binding_never_loads_or_rewarms_and_checks_worker_identity(self):
        with self.assertRaisesRegex(ValueError,'explicit preparation'):self.client.prepare(ready_only=True)
        self.assertEqual(self.model.prepares,0)
        expected=self.client.prepare();requests=len(self.model.requests)
        with patch.object(self.service,'prepare',side_effect=AssertionError('restore must not POST prepare')):
            self.assertEqual(self.client.prepare(ready_only=True),expected)
            self.assertEqual(self.model.prepares,1);self.assertEqual(len(self.model.requests),requests)
            self.service._set(worker_identity={**expected,'checkpoint_sha256':'0'*64})
            with self.assertRaises(ValueError):self.client.prepare(ready_only=True)
            self.service._set(loaded=False,status='not_loaded')
            with self.assertRaisesRegex(ValueError,'explicit preparation'):self.client.prepare(ready_only=True)
        self.assertEqual(self.model.prepares,1);self.assertEqual(self.model.closes,0)

    def test_owned_endpoint_start_is_inert_and_exits_with_its_process(self):
        from robot_workbench.inference_model import PROMPT
        from robot_workbench.inference_transport import start_model_service
        root=Path(self.temp.name)
        artifacts={key:dict(path=str(root/key),sha256=value) for key,value in BASE379_PINS.items()}
        bundle=dict(schema='workbench.base379.v1',epoch=10,step=29450,prompt=PROMPT,
            action_horizon=60,execution_steps=10,action_hz=30,rtc=False,artifacts=artifacts,
            python=artifacts['launcher']['path'],model_source=str(root/'model-source'),history_source=str(root/'history-source'))
        manifest=root/'bundle.json';manifest.write_text(json.dumps(bundle))
        process,endpoint=start_model_service(dict(model_manifest=str(manifest),gpu=0,gpu_uuid='GPU-cpu-fixture',
            min_free_memory_mib=1),root/'endpoint',self.token)
        try:
            client=RemoteBaseClient(endpoint,token_file=self.token,adapter=Adapter(),conversion_identity=self.binding)
            status=client.status();self.assertFalse(status['loaded']);self.assertEqual(status['prepare_attempts'],0)
            self.assertFalse((root/'endpoint/worker.log').exists())
        finally:
            process.terminate();process.wait(10)
        self.assertEqual(process.returncode,0)

    def test_auth_paths_conversion_and_fixed_model(self):
        wrong=Path(self.temp.name)/'wrong-token';wrong.write_text('z'*40)
        other=RemoteBaseClient(self.url,token_file=wrong,adapter=Adapter(),conversion_identity=self.binding)
        with self.assertRaisesRegex(RuntimeError,'403'):other.prepare()
        self.assertEqual(self.model.prepares,0)
        for endpoint in ('http://10.17.0.27:1234','https://127.0.0.1:1234','http://a:b@127.0.0.1:1234','http://127.0.0.1:1234/?token=x'):
            with self.assertRaises(ValueError):RemoteBaseClient(endpoint,token_file=self.token,adapter=Adapter(),conversion_identity=self.binding)
        payload=dict(protocol=PROTOCOL,model_id=MODEL_ID,conversion_id=self.client.conversion_id,model_path='/arbitrary')
        with self.assertRaisesRegex(RuntimeError,'409'):self.client._request('/prepare',json.dumps(payload).encode())
        self.assertEqual(self.model.prepares,0)
        self.client.prepare()
        with patch.object(self.model,'infer',return_value=dict(request_id='other-request',action14=np.zeros((60,14)),audit={})):
            with self.assertRaisesRegex(RuntimeError,'identity mismatch'):self.client.infer(request())
        self.assertFalse(self.client.status()['loaded']);self.assertEqual(self.model.closes,1)
        self.assertNotEqual(conversion_digest(self.binding),conversion_digest({**self.binding,'calibration_snapshot':'installed-2'}))

    def test_explicit_forwarded_host_ports_preserve_auth_and_reject_duplicate_or_unknown_hosts(self):
        def call(server,hosts,*,auth=True,path='/models',body=None):
            connection=HTTPConnection('127.0.0.1',server.server_port,timeout=2)
            try:
                connection.putrequest('POST' if body is not None else 'GET',path,skip_host=True)
                for host in hosts:connection.putheader('Host',host)
                if auth:connection.putheader('Authorization','Bearer '+self.token.read_text())
                if body is not None:connection.putheader('Content-Length',str(len(body)))
                connection.endheaders(body)
                response=connection.getresponse();response.read();return response.status
            finally:connection.close()
        self.assertEqual(call(self.server,['127.0.0.1:18875']),403,'default must not accept a forward alias')
        for ports in ((True,),(0,),(-1,),(65536,),('18875',),None):
            with self.assertRaises(ValueError):serve_model(self.service,token_file=self.token,forwarded_ports=ports)
        forwarded=serve_model(self.service,token_file=self.token,forwarded_ports=(18875,))
        thread=threading.Thread(target=forwarded.serve_forever,daemon=True);thread.start()
        try:
            for host in ('127.0.0.1:18875','localhost:18875',f'127.0.0.1:{forwarded.server_port}'):
                self.assertEqual(call(forwarded,[host]),200)
            for hosts in (['127.0.0.1:18876'],['10.17.0.27:18875'],['evil.example:18875'],
                          ['127.0.0.1:18875','127.0.0.1:18875']):
                self.assertEqual(call(forwarded,hosts),403)
            self.assertEqual(call(forwarded,['127.0.0.1:18875'],auth=False),403)
            body=json.dumps(dict(protocol=PROTOCOL,model_id=MODEL_ID,conversion_id=self.client.conversion_id)).encode()
            self.assertEqual(call(forwarded,['127.0.0.1:18875'],auth=False,path='/prepare',body=body),403)
            self.assertEqual(self.model.prepares,0)
            self.assertEqual(call(forwarded,['127.0.0.1:18875'],path='/prepare',body=body),200)
            self.assertEqual(self.model.prepares,1)
        finally:forwarded.shutdown();forwarded.server_close();thread.join(2)

    def test_npz_rejects_object_nonfinite_wrong_shape_and_expansion_before_numpy(self):
        good=request_packet(request(),Adapter(),self.client.conversion_id)
        metadata,arrays=decode_packet(good,{'model_state14','model_history',*CAMS})
        self.assertEqual(metadata['conversion_id'],self.client.conversion_id)
        for bad in (b'not zip',good+b'0'*MAX_PACKET):
            with self.assertRaises(ValueError):decode_packet(bad,set(arrays))
        with self.assertRaises(ValueError):encode_packet(metadata,{'object':np.array([{}],dtype=object)})
        bad=request();bad['observation']['device_state14'][0]=np.nan
        with self.assertRaises(ValueError):request_packet(bad,Adapter(),self.client.conversion_id)
        bad=request();bad['observation']['images'][CAMS[0]]=np.zeros((223,224,3),np.uint8)
        with self.assertRaises(ValueError):request_packet(bad,Adapter(),self.client.conversion_id)
        bad=request();bad['applied_history']=np.zeros((61,14),np.float32)
        with self.assertRaises(ValueError):request_packet(bad,Adapter(),self.client.conversion_id)
        # ZIP's advertised expansion and NPY's own shape must both be bounded.
        for declared_shape,padding in (((10**12,),b''),((MAX_PACKET+1,),b'0'*(MAX_PACKET+1))):
            array=io.BytesIO();np.lib.format.write_array_header_1_0(array,dict(descr='|u1',fortran_order=False,shape=declared_shape))
            array.write(padding);archive=io.BytesIO()
            with zipfile.ZipFile(archive,'w',compression=zipfile.ZIP_DEFLATED) as output:output.writestr('metadata.npy',array.getvalue())
            with patch('numpy.load',side_effect=AssertionError('allocation must not start')):
                with self.assertRaises(ValueError):decode_packet(archive.getvalue(),set())

    def test_timeout_late_result_cannot_become_another_request_and_calls_are_serial(self):
        self.client.prepare();self.model.release=threading.Event()
        short=RemoteBaseClient(self.url,token_file=self.token,adapter=Adapter(),conversion_identity=self.binding,inference_timeout_s=.08)
        short.prepare()
        with self.assertRaises((TimeoutError,socket.timeout)):short.infer(request('timed-out',3))
        self.assertTrue(self.model.entered.is_set())
        with self.assertRaisesRegex(RuntimeError,'busy'):self.client.infer(request('concurrent',4))
        self.model.release.set()
        deadline=time.monotonic()+2
        while self.service.status()['busy'] and time.monotonic()<deadline:time.sleep(.005)
        result=self.client.infer(request('fresh',9))
        self.assertEqual(result['request_id'],'fresh');self.assertTrue(np.all(result['action14']==9))
        self.assertEqual([row['request_id'] for row in self.model.requests],['warmup','timed-out','fresh'])
        self.model.fail=True
        with self.assertRaisesRegex(RuntimeError,'timed out'):self.client.infer(request('worker-timeout',2))
        self.assertEqual(self.model.closes,1);self.assertFalse(self.service.status()['loaded'])

    def test_remote_rejects_mismatched_conversion_and_diagnostics_retain_stderr(self):
        log=Path(self.temp.name)/'worker.log';log.write_text('startup traceback\n'+'x'*10000)
        self.model.log_path=str(log)
        self.client.prepare()
        original=self.service.infer
        def wrong_conversion(payload):
            meta,arrays=decode_packet(original(payload),{'action14'});meta['conversion_id']='0'*64
            return encode_packet(meta,arrays)
        with patch.object(self.service,'infer',side_effect=wrong_conversion):
            with self.assertRaisesRegex(ValueError,'conversion identity'):self.client.infer(request())
        status=self.client.status();self.assertEqual(status['stderr_path'],str(log))
        self.assertEqual(len(status['stderr_tail']),8192);self.assertEqual(status['worker_identity']['step'],29450)
        adapter=ModelDomainAdapter();state,images=adapter.model_input(dict(model_state14=np.zeros(14),images=request()['observation']['images']))
        self.assertEqual(state.dtype,np.float32);self.assertEqual(set(images),set(CAMS))

    def test_explicit_reprepare_uses_new_log_and_preserves_prior_evidence(self):
        base=Path(self.temp.name)/'exclusive-worker.log';self.model.log_path=str(base)
        self.service.base_log_path=str(base)
        prepare=self.model.prepare
        def prepare_with_exclusive_log():
            with Path(self.model.log_path).open('xb') as stream:stream.write(b'actual startup evidence\n')
            return prepare()
        self.model.prepare=prepare_with_exclusive_log
        self.client.prepare();original=base.read_bytes();self.client.prepare()
        self.assertEqual(self.model.prepares,1)
        self.model.fail=True
        with self.assertRaisesRegex(RuntimeError,'timed out'):self.client.infer(request())
        self.assertEqual(self.model.prepares,1,'a failed model request cannot automatically reload')
        self.model.fail=False;self.client.prepare()
        self.assertEqual(self.model.prepares,2);self.assertEqual(base.read_bytes(),original)
        alternate=Path(str(base)+'.attempt-2');self.assertEqual(alternate.read_bytes(),original)
        self.assertEqual(self.client.status()['worker_logs'],[str(base),str(alternate)])
        self.assertEqual(self.client.infer(request('reloaded'))['request_id'],'reloaded')

    def test_service_restart_preserves_existing_worker_log(self):
        base=Path(self.temp.name)/'restart-worker.log';base.write_bytes(b'prior failed startup\n')
        model=StubModel();model.log_path=str(base);prepare=model.prepare
        def exclusive():
            with Path(model.log_path).open('xb') as stream:stream.write(b'new startup\n')
            return prepare()
        model.prepare=exclusive
        service=ModelService(model)
        try:
            service.prepare(dict(protocol=PROTOCOL,model_id=MODEL_ID,conversion_id='a'*64))
            self.assertEqual(base.read_bytes(),b'prior failed startup\n')
            self.assertNotEqual(model.log_path,str(base))
            self.assertEqual(Path(model.log_path).read_bytes(),b'new startup\n')
            self.assertEqual(service.status()['worker_logs'],[model.log_path])
        finally:service.close()


if __name__=='__main__':unittest.main()
