"""E03 preprocessing against the deployed formula; CPU tensors, no model or SDK."""
from pathlib import Path
from copy import deepcopy
import sys
from tempfile import TemporaryDirectory
import threading
from types import SimpleNamespace
import unittest

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from robot_workbench import wam
import hashlib

# Checkpoint identities are fixture data; production code owns no checkpoint pins.
MODEL_ID='wam-fixture-step3190'
CAPABILITIES={'denoise_steps':{'minimum':1,'maximum':10,'default':2}}
PINS=dict(checkpoint='1'*64,stats=hashlib.sha256((ROOT/'tests/fixtures/wam-e03-normalization.json').read_bytes()).hexdigest(),
          context='2'*64,model_config='3'*64)


class WamInputWorkflowTests(unittest.TestCase):
    def test_native_training_context_is_padded_without_changing_visible_tokens(self):
        torch=wam._torch()
        context=torch.arange(128*4096,dtype=torch.float32).reshape(128,4096).to(torch.bfloat16)
        mask=torch.ones(128,dtype=torch.bool);mask[7]=False
        prepared=wam.prepare_text_context(dict(context=context,context_mask=mask))
        self.assertEqual(tuple(prepared['context'].shape),(1,200,4096))
        self.assertTrue(torch.equal(prepared['context'][0,:128][mask],context[mask]))
        self.assertEqual(int(prepared['context_mask'].sum()),127)
        self.assertEqual(int(torch.count_nonzero(prepared['context'][~prepared['context_mask']])),0)
        with self.assertRaises(ValueError):wam.prepare_text_context(dict(context=context[:,:32],context_mask=mask))

    def test_source_manifest_identity_does_not_invent_a_weight_hash(self):
        from robot_workbench.inference_model import native_runtime,validate_native_descriptor
        value=self.descriptor();value.update(schema=wam.SOURCE_SCHEMA,
            checkpoint_source={'receipt_sha256':hashlib.sha256(b'fixture provenance').hexdigest(),'bytes':12000000000})
        value['artifacts']['checkpoint']['sha256']=None
        self.assertTrue(native_runtime(value));self.assertEqual(validate_native_descriptor(value),value)
        identity=wam.worker_identity(value);self.assertIsNone(identity['checkpoint_sha256'])
        self.assertEqual(identity['checkpoint_source'],value['checkpoint_source'])
        missing=deepcopy(value);missing.pop('checkpoint_source')
        with self.assertRaises(ValueError):wam.validate_descriptor(missing)
        legacy=deepcopy(value);legacy.pop('checkpoint_source');legacy['schema']=wam.SCHEMA
        with self.assertRaises(ValueError):wam.validate_descriptor(legacy)

    def test_unhashed_weight_requires_matching_source_size_and_model_identity(self):
        import json
        from unittest.mock import patch
        with TemporaryDirectory()as temp:
            root=Path(temp).resolve();files=[];pins={}
            for name in ('checkpoint.pt','stats.json','context.pt','model_config.json','engine.py',
                         'phoenix_encoder.pt','csvae.pt','Wan2.2_VAE.safetensors'):
                p=root/name;p.write_bytes(name.encode())
                if name in('checkpoint.pt','phoenix_encoder.pt'):
                    with p.open('r+b')as stream:stream.truncate(64*1024*1024+1)
                    digest=None
                else:digest=hashlib.sha256(p.read_bytes()).hexdigest()
                files.append(dict(path=name,bytes=p.stat().st_size,sha256=digest));pins[name]=digest
            size=files[0]['bytes'];proof=dict(schema='workbench.model-source.v1',model_id=MODEL_ID,step=3190,
                checkpoint_path='checkpoint.pt',files={row['path']:{'bytes':row['bytes']}for row in files if row['sha256']is None})
            path=root/'source-receipt.json';path.write_text(json.dumps(proof));proof_sha=hashlib.sha256(path.read_bytes()).hexdigest()
            files.append(dict(path=path.name,bytes=path.stat().st_size,sha256=proof_sha))
            source={'receipt_sha256':proof_sha,'bytes':size}
            registry=dict(root=str(root),files=files,machine_id_sha256='host',gpu_uuid='GPU-fixture',checkpoint_path='checkpoint.pt',checkpoint_source=source)
            registry['bundle_sha256']=hashlib.sha256(json.dumps(sorted(files,key=lambda r:r['path']),sort_keys=True,separators=(',',':')).encode()).hexdigest()
            bundle=dict(schema=wam.SOURCE_SCHEMA,model_id=MODEL_ID,step=3190,checkpoint_source=source,
                artifacts={k:{'sha256':pins[v]}for k,v in dict(checkpoint='checkpoint.pt',stats='stats.json',context='context.pt',model_config='model_config.json').items()},runtime={'engine_sha256':pins['engine.py']})
            def sha(p):
                if str(p)=='/etc/machine-id':return'host'
                self.assertNotEqual(Path(p).name,'checkpoint.pt')
                return hashlib.sha256(Path(p).read_bytes()).hexdigest()
            with patch('robot_workbench.mixed366.sha',side_effect=sha),patch.object(wam,'descriptor',return_value=bundle):
                self.assertEqual(wam.verify_registry(registry)[1]['checkpoint'],root/'checkpoint.pt')
                changed=deepcopy(bundle);changed['step']=3191
                with patch.object(wam,'descriptor',return_value=changed),self.assertRaisesRegex(ValueError,'source identity'):
                    wam.verify_registry(registry)
                with(root/'checkpoint.pt').open('r+b')as stream:stream.truncate(size-1)
                with self.assertRaisesRegex(ValueError,'artifact changed'):wam.verify_registry(registry)

    def test_registry_uses_large_weight_metadata_but_checks_small_content_and_sizes(self):
        import json
        from unittest.mock import patch
        with TemporaryDirectory()as temp:
            root=Path(temp).resolve();files=[];pins={}
            for name in ('checkpoint.pt','stats.json','context.pt','model_config.json','engine.so',
                         'phoenix_encoder.pt','csvae.pt','Wan2.2_VAE.safetensors'):
                p=root/name;p.write_bytes(name.encode())
                if name=='checkpoint.pt':
                    with p.open('r+b')as f:f.truncate(64*1024*1024+1)
                    digest='a'*64
                else:digest=hashlib.sha256(p.read_bytes()).hexdigest()
                files.append(dict(path=name,bytes=p.stat().st_size,sha256=digest));pins[name]=digest
            def sha(path):
                if str(path)=='/etc/machine-id':return 'host'
                self.assertNotEqual(Path(path).name,'checkpoint.pt','ordinary verification must not reread the large weight')
                return hashlib.sha256(Path(path).read_bytes()).hexdigest()
            canonical=json.dumps(sorted(files,key=lambda r:r['path']),sort_keys=True,separators=(',',':')).encode()
            registry=dict(root=str(root),files=files,machine_id_sha256='host',gpu_uuid='GPU-fixture',bundle_sha256=hashlib.sha256(canonical).hexdigest())
            bundle=dict(artifacts={k:dict(sha256=pins[v])for k,v in dict(checkpoint='checkpoint.pt',stats='stats.json',context='context.pt',model_config='model_config.json').items()},runtime=dict(engine_sha256=pins['engine.so']))
            with patch('robot_workbench.mixed366.sha',side_effect=sha),patch.object(wam,'descriptor',return_value=bundle):
                self.assertEqual(wam.verify_registry(registry)[1]['checkpoint'],root/'checkpoint.pt')
                (root/'stats.json').write_bytes(b'x'*len('stats.json'))
                with self.assertRaisesRegex(ValueError,'artifact changed'):wam.verify_registry(registry)
                (root/'stats.json').write_bytes(b'stats.json')
                with (root/'checkpoint.pt').open('r+b')as f:f.truncate(3)
                with self.assertRaisesRegex(ValueError,'artifact changed'):wam.verify_registry(registry)

    def test_stall_window_counts_executed_points_then_cycles_and_recovers(self):
        from unittest.mock import patch
        recovery=wam.StallRecovery({'left':80.,'right':89.})
        sequence=0;stamp=100.
        def event(name,value):
            nonlocal sequence
            sequence+=1;recovery.event(name,value,sequence)
        def segment(request,steps=15,moving=False):
            nonlocal stamp
            start=stamp
            event('normal_model_segment_started',dict(request_id=request,start_source_s=start,
                execution_duration_s=steps/30*2,source_steps=steps))
            for step in range(1,steps+1):
                stamp=start+step/30*2
                state=np.zeros(14);state[[6,13]]=[20.,30.]
                if moving:state[0]=.03 if step%2 else-.03
                event('feedback',dict(source_time=stamp,device_state14=state.tolist()))
                # Faster control ticks and duplicate event delivery are not new model steps.
                event('feedback',dict(source_time=stamp+.001,device_state14=state.tolist()))
                recovery.event('feedback',dict(source_time=stamp+.001,device_state14=state.tolist()),sequence)
            stamp+=1.
        for i in range(5):segment(str(i))
        self.assertEqual(recovery.choose(7)['actual_seed'],7)
        segment('sixth')
        with patch('secrets.randbits',side_effect=[7,8,8,9,10,11]):
            decisions=[recovery.choose(7)for _ in range(4)]
        self.assertEqual([row['head_black']for row in decisions],[True,True,False,True])
        self.assertEqual([row['actual_seed']for row in decisions],[8,9,10,11])
        self.assertEqual(decisions[0]['config']['jaw_range_mm'],[4.,4.45])
        self.assertEqual(decisions[0]['observed_steps'],90)
        segment('moving',moving=True)
        decision=recovery.choose(7)
        self.assertFalse(decision['stalled']);self.assertEqual(decision['actual_seed'],7)
        self.assertGreater(decision['observed_range_device14'][0],.02)
        event('mode_transition',dict(target='HOLD'))
        self.assertEqual(recovery.choose(7)['observed_steps'],0)

    def test_stall_sampling_uses_local_execution_point_times(self):
        recovery=wam.StallRecovery({'left':80.,'right':89.})
        recovery.event('normal_model_segment_started',dict(request_id='local',source_steps=3,
            start_source_s=10.,execution_duration_s=1.1,source_point_times_s=[.1,1.,1.1]),1)
        recovery.event('feedback',dict(source_time=10.1,device_state14=[0.]*14),2)
        self.assertEqual(recovery.step,1)
        recovery.event('feedback',dict(source_time=10.8,device_state14=[0.]*14),3)
        self.assertEqual(recovery.step,1)
        recovery.event('feedback',dict(source_time=11.,device_state14=[0.]*14),4)
        self.assertEqual(recovery.step,2)
        recovery.event('feedback',dict(source_time=11.1,device_state14=[0.]*14),5)
        self.assertEqual(recovery.step,3);self.assertEqual(len(recovery.samples),3)

    def test_missing_feedback_and_control_event_gaps_cannot_trigger_stall(self):
        recovery=wam.StallRecovery({'left':80.,'right':89.})
        recovery.event('normal_model_segment_started',dict(request_id='one',start_source_s=10.,
            execution_duration_s=1.,source_steps=30),1)
        for step in range(1,25):recovery.event('feedback',dict(source_time=10.+step/30,device_state14=[0.]*14),step+1)
        self.assertEqual(recovery.choose(0)['observed_steps'],24)
        recovery.event('feedback',dict(source_time=10.+27/30,device_state14=[0.]*14),26)
        self.assertEqual(recovery.choose(0)['observed_steps'],1)
        recovery.event('feedback',dict(source_time=11.,device_state14=[0.]*14),28)
        self.assertEqual(recovery.choose(0)['observed_steps'],0)
        self.assertEqual(recovery.snapshot()['last_decision']['reset_reason'],'control_event_gap')

    @unittest.skipUnless(sys.platform=='linux','Linux parent-death semantics')
    def test_model_worker_survives_the_http_prepare_thread_and_closes_explicitly(self):
        import json,os,subprocess,time
        from unittest.mock import patch
        with TemporaryDirectory()as folder:
            root=Path(folder);norm=root/'normalization.json'
            norm.write_bytes((ROOT/'tests/fixtures/wam-e03-normalization.json').read_bytes())
            python=root/'env/anyinfer-3090-py312/bin/python';python.parent.mkdir(parents=True);python.symlink_to(sys.executable)
            helper=root/'cpu_worker.py'
            helper.write_text('''import os,sys
from multiprocessing.connection import Connection
from robot_workbench.arx_output_guard import parent_death
parent_death(int(sys.argv[2]))
c=Connection(int(sys.argv[1]))
c.send(dict(ready=True,pid=os.getpid(),adapter_source_sha256=sys.argv[3]))
while c.recv() is not None:pass
''')
            registry=dict(root=str(root),model_id=MODEL_ID,step=3190,backend='anyinfer',inference_capabilities=CAPABILITIES,artifacts=PINS,engine_sha256='b'*64,bundle_sha256='a'*64,gpu_uuid='GPU-CPU-fixture',files=[
                dict(path='anyinfer.cpython-312-x86_64-linux-gnu.so',sha256='b'*64),
                dict(path='normalization.json',sha256=PINS['stats'])])
            client=wam.Client(registry,wam.descriptor(registry));self.addCleanup(client.close)
            original=subprocess.Popen;errors=[]
            def popen(argv,**fields):
                if '--worker'in argv:
                    argv=[sys.executable,str(helper),argv[argv.index('--pipe-fd')+1],str(os.getpid()),wam._SOURCE_SHA256]
                return original(argv,**fields)
            def prepare():
                try:client.prepare()
                except BaseException as error:errors.append(error)
            with patch('robot_workbench.wam.verify_registry',return_value=(root,{})),\
                 patch('robot_workbench.mixed366.gpu_lock',return_value=SimpleNamespace(close=lambda:None)),\
                 patch('subprocess.check_output',return_value=json.dumps([[3,12],str(root)]).encode()),\
                 patch('subprocess.Popen',side_effect=popen):
                request=threading.Thread(target=prepare);request.start();request.join(5)
                self.assertFalse(request.is_alive());self.assertFalse(errors,errors)
                process=client.process;time.sleep(.15)
                self.assertIsNone(process.poll(),'model worker died when its prepare request thread returned')
                request=threading.Thread(target=prepare);request.start();request.join(5)
                self.assertFalse(errors,errors);self.assertIs(client.process,process)
                client.close();self.assertIsNotNone(process.poll())

    def test_native_binding_checks_real_dependencies_and_projects_each_jaw_separately(self):
        from copy import deepcopy
        from types import SimpleNamespace
        import hashlib
        from test_mixed366_control import native_mapping_fixture,jaw
        from robot_workbench.inference_workflow import calibrated_adapter,InferenceWorkflow
        from robot_workbench.inference_owner import device_limits
        with TemporaryDirectory()as folder:
            root=Path(folder);native,mapping=native_mapping_fixture(root,'arx13',(.080,.089))
            jaw.configure_mapping(mapping['path']);self.addCleanup(lambda:jaw.configure_mapping(None))
            sdk=root/'sdk.bin';sdk.write_bytes(b'CPU pin fixture, not a loaded SDK')
            deployment=dict(robot_id='arx13',disabled_grippers=[],native_gripper_calibration=native,
                sdk_quiesce=dict(module=dict(path=str(sdk),sha256=hashlib.sha256(sdk.read_bytes()).hexdigest())),
                cameras={s:dict(serial=s+'-serial')for s in ('head','left','right')})
            profile,binding=wam.native_binding(deployment)
            flow=object.__new__(InferenceWorkflow);flow.device=SimpleNamespace(id='arx13',environment='physical')
            flow.selected_model_id=MODEL_ID;flow.profile_provider=lambda:(profile,binding)
            self.assertEqual(flow._binding(),(profile,binding));self.assertFalse(binding['extrinsics_consumed'])
            limits=[[-3.,3.]for _ in range(14)];limits[6]=limits[13]=[0.,.082]
            adapter,identity=calibrated_adapter(profile,limits,model_id=MODEL_ID)
            np.testing.assert_allclose(adapter.limits[[6,13],1],[80.,89.])
            rounded=deepcopy(limits)
            rounded[6][1]=rounded[13][1]=float(np.float32(.082))
            native_limits=np.asarray(device_limits(rounded,native_coordinates=True))
            np.testing.assert_array_equal(native_limits[[6,13],1],[80.,89.])
            observed=np.zeros(14);observed[[6,13]]=[80.,89.]
            np.testing.assert_allclose(adapter.device_to_model(observed)[[6,13]],[.080,.089])
            action=np.zeros(14);action[[6,13]]=.1
            projected=adapter.decode(action,observed)
            np.testing.assert_allclose(np.asarray(projected['final_device14'])[[6,13]],[100.,100.])
            self.assertEqual(identity['native_calibration_id'],native['sha256'])
            np.testing.assert_allclose([device_limits(limits)[i][1]for i in (6,13)],[80.,83.])
            changed=deepcopy(binding);changed['robot_id']='arx6';flow.profile_provider=lambda:(profile,changed)
            with self.assertRaisesRegex(ValueError,'binding differs'):flow._binding()
            changed=deepcopy(deployment);changed['cameras']['right']['serial']='left-serial'
            with self.assertRaises(ValueError):wam.native_binding(changed)
            sdk.write_bytes(b'changed SDK bytes')
            with self.assertRaises(ValueError):wam.native_binding(deployment)

    def descriptor(self):
        return dict(schema=wam.SCHEMA,model_id=MODEL_ID,epoch=None,step=3190,backend='anyinfer',
            artifacts={k:dict(sha256=v)for k,v in PINS.items()},model_bundle_sha256='a'*64,
            normalization_json=(ROOT/'tests/fixtures/wam-e03-normalization.json').read_text(),
            inference_options={'denoise_steps':2},inference_capabilities=CAPABILITIES,
            runtime=dict(adapter='wam_anyinfer_client.v2',engine_sha256='b'*64,python_abi='cp312',torch_version=wam.MODEL_TORCH_VERSION))

    def test_same_E03_torch_and_SO_use_distinct_runtime_pins_and_same_action_contract(self):
        from robot_workbench.inference_model import action_contract,model_bundle_digest,model_spec
        so=self.descriptor();native=deepcopy(so);native.update(backend='torch',model_bundle_sha256='c'*64)
        native['artifacts']['model_config']['sha256']='12ecfe45087ec9c9f571a3d3cd2230fedbd5c4782e2cb27bffb4186f75a101b9'
        native['runtime'].update(adapter='wam_torch_client.v2',engine_sha256='e'*64)
        self.assertEqual(wam.validate_descriptor(native),native)
        self.assertNotEqual(model_bundle_digest(MODEL_ID,so),model_bundle_digest(MODEL_ID,native))
        self.assertEqual(action_contract(MODEL_ID,so),action_contract(MODEL_ID,native))
        self.assertEqual(model_spec(MODEL_ID,native)['pins']['model_config'],native['artifacts']['model_config']['sha256'])
        self.assertTrue(wam.worker_identity(native)['strict_load']);self.assertFalse(wam.worker_identity(so)['strict_load'])
        for changed in [dict(native,backend='anyinfer'),dict(so,backend='torch')]:
            with self.assertRaises(ValueError):wam.validate_descriptor(changed)

    def test_denoise_configuration_changes_runtime_identity_and_rejects_invalid_values(self):
        from robot_workbench.inference_model import model_bundle_digest,model_spec,action_contract
        base=self.descriptor()
        for steps in (1,2,10):
            bundle=deepcopy(base);bundle['inference_options']['denoise_steps']=steps
            self.assertEqual(wam.worker_identity(bundle)['inference_steps'],steps)
            self.assertEqual(wam.preprocessing_identity(bundle)['denoise_steps'],steps)
            self.assertEqual(model_spec(MODEL_ID,bundle)['inference_options']['denoise_steps'],steps)
            self.assertEqual(action_contract(MODEL_ID,bundle),action_contract(MODEL_ID,base))
            self.assertEqual(bundle['artifacts'],base['artifacts'])
            if steps!=2:self.assertNotEqual(model_bundle_digest(MODEL_ID,bundle),model_bundle_digest(MODEL_ID,base))
        for value in (0,11,True,1.5,'2'):
            bundle=deepcopy(base);bundle['inference_options']['denoise_steps']=value
            with self.assertRaises(ValueError):wam.validate_descriptor(bundle)
        old=deepcopy(base);old['schema']='workbench.wam.v1'
        with self.assertRaises(ValueError):wam.validate_descriptor(old)

    def test_torch_sampling_update_reuses_worker_and_warm_parameters(self):
        from robot_workbench.inference_transport import ModelService,PROTOCOL
        from robot_workbench.inference_model import model_bundle_digest
        base=self.descriptor();base['backend']='torch';base['runtime']['adapter']='wam_torch_client.v2'
        next_bundle=deepcopy(base);next_bundle['inference_options']['denoise_steps']=10
        changed=deepcopy(next_bundle);changed['artifacts']['checkpoint']['sha256']='f'*64
        self.assertTrue(wam.sampling_change(base,next_bundle));self.assertFalse(wam.sampling_change(base,changed))
        self.assertFalse(wam.sampling_change(self.descriptor(),next_bundle))
        class Loaded:
            model_id=MODEL_ID
            def __init__(self):self.bundle=deepcopy(base);self.prepares=0;self.closes=0;self.steps=[]
            def worker_status(self):return dict(worker_pid=4321,worker_alive=True,worker_exit_code=None)
            def can_reconfigure(self,bundle):return wam.sampling_change(self.bundle,bundle)
            def reconfigure(self,bundle):self.bundle=deepcopy(bundle);return wam.worker_identity(bundle)
            def prepare(self):self.prepares+=1;return wam.worker_identity(self.bundle)
            def infer(self,request):
                self.steps.append(self.bundle['inference_options']['denoise_steps'])
                return dict(action14=np.full((32,14),self.steps[-1],dtype=np.float32))
            def close(self):self.closes+=1
        loaded=Loaded();service=ModelService(loaded,catalog={'old':base,'new':next_bundle},
            client_factory=lambda _:self.fail('same checkpoint must not create another client'))
        def prepare(bundle):return service.prepare(dict(protocol=PROTOCOL,model_id=MODEL_ID,backend='torch',
            conversion_id='c'*64,expected_bundle_digest=model_bundle_digest(MODEL_ID,bundle)))
        prepare(base);prepare(next_bundle);prepare(next_bundle);prepare(base)
        self.assertEqual((loaded.prepares,loaded.closes,service.prepare_attempts),(1,0,1))
        self.assertEqual(loaded.steps,[2,10,2]);self.assertEqual(service.status()['worker_pid'],4321)
        self.assertEqual(service.status()['worker_identity']['inference_steps'],2)
        self.assertTrue(service.status()['warmup']['completed'])

    def test_new_checkpoint_uses_its_own_identity_statistics_and_action_contract(self):
        import json
        from robot_workbench.inference_model import action_contract,model_spec
        bundle=self.descriptor();bundle.update(model_id='wam-another-run-step42',step=42)
        stats=json.loads(bundle['normalization_json']);stats['state']['mean']=[.25]*14
        bundle['normalization_json']=json.dumps(stats)
        for kind in ('checkpoint','context','model_config'):bundle['artifacts'][kind]['sha256']='f'*64
        bundle['artifacts']['stats']['sha256']=hashlib.sha256(bundle['normalization_json'].encode()).hexdigest()
        self.assertEqual(wam.validate_descriptor(bundle),bundle)
        self.assertEqual(wam.worker_identity(bundle)['step'],42)
        self.assertEqual(model_spec(bundle['model_id'],bundle)['pins']['stats'],bundle['artifacts']['stats']['sha256'])
        self.assertEqual(action_contract(bundle['model_id'],bundle)['sampling']['basis'],'materialized_next_observation')
        np.testing.assert_array_equal(wam.prepare_state(np.full(14,.25),wam.normalization(stats)),np.zeros(14))

    def test_shared_catalog_keeps_pi_and_wam_clients_distinct_without_loading(self):
        import json
        from robot_workbench.mixed366 import Service
        from robot_workbench.inference_model import MIXED366_ID
        with TemporaryDirectory()as folder:
            root=Path(folder);metadata=root/'wam.json'
            norm=root/'wam/normalization.json';norm.parent.mkdir();norm.write_bytes((ROOT/'tests/fixtures/wam-e03-normalization.json').read_bytes())
            registry=dict(model_id=MODEL_ID,step=3190,backend='anyinfer',inference_capabilities=CAPABILITIES,artifacts=PINS,engine_sha256='b'*64,root=str(root/'wam'),bundle_sha256='a'*64,
                files=[dict(path='runtime/anyinfer.cpython-312-x86_64-linux-gnu.so',bytes=0,sha256='b'*64),
                    dict(path='normalization.json',bytes=norm.stat().st_size,sha256=PINS['stats'])])
            metadata.write_text(json.dumps(registry));metadata.chmod(0o600)
            native=deepcopy(registry);native['backend']='torch'
            native_metadata=root/'wam-torch.json';native_metadata.write_text(json.dumps(native));native_metadata.chmod(0o600)
            service=Service(dict(model_id=MIXED366_ID,root=str(root/'pi'),bundle_sha256='c'*64,
                source_manifest_sha256='d'*64,runtime_sha256='e'*64,additional_registries=[str(metadata),str(native_metadata)]))
            try:
                rows=service.models();self.assertEqual({r['model_id']for r in rows},{MIXED366_ID,MODEL_ID})
                selected=service.catalog()[MODEL_ID+'/anyinfer/2'];client=service.client_factory(selected)
                self.assertIsInstance(client,wam.Client);self.assertIsNone(client.process)
                selected=service.catalog()[MODEL_ID+'/torch/2'];client=service.client_factory(selected)
                self.assertIsInstance(client,wam.Client);self.assertEqual(client.bundle['backend'],'torch')
                self.assertFalse(service.status()['loaded']);self.assertEqual(service.model_id,MIXED366_ID)
            finally:service.close()

    def test_bound32_capability_crosses_real_http_and_model_pin_without_changing_values(self):
        from robot_workbench.inference_transport import ModelService,RemoteBaseClient,serve_model
        from robot_workbench.inference_model import action_contract,model_family,model_descriptor
        from robot_workbench.inference import validate_action_chunk
        bundle=self.descriptor();calls=[]
        class Worker:
            model_id=MODEL_ID
            def prepare(self):return wam.worker_identity(bundle)
            def infer(self,request):
                calls.append(request)
                return dict(request_id=request['request_id'],action14=np.arange(32*14,dtype=np.float32).reshape(32,14),audit={'cpu_stub':True})
            def close(self):pass
        worker=Worker();worker.bundle=bundle
        images={k:np.zeros((256,256,3),dtype=np.uint16)for k in wam.CAMS}
        adapter=SimpleNamespace(model_input=lambda obs:(np.zeros(14,np.float32),images))
        with TemporaryDirectory()as folder:
            token=Path(folder)/'auth';token.write_text('cpu-fixture-'+40*'x')
            server=serve_model(ModelService(worker),token_file=token)
            thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start();pin=None
            try:
                client=RemoteBaseClient('http://127.0.0.1:'+str(server.server_port),token_file=token,adapter=adapter,
                    conversion_identity={'cpu':'WAM'},model_id=MODEL_ID,model_bundle=bundle)
                self.assertEqual(model_family(MODEL_ID),'wam');self.assertEqual(model_descriptor(MODEL_ID,bundle),bundle)
                contract=action_contract(MODEL_ID,bundle)
                self.assertEqual(contract['shape'],[32,14]);self.assertEqual(contract['history_steps'],0)
                self.assertEqual(contract['sampling']['first_sample_offset_s'],1/30)
                client.prepare();pin=client.pin('cpu-wam-32',auto_renew=False)
                request=dict(request_id='chunk32',seed=7,observation=dict(source_time=2.,frame_ids={}),
                    applied_history=np.empty((0,14),np.float32))
                result=pin.infer_chunk(request,epoch=1,clock_domain='cpu-source',valid_until_source_time=4.5)
                binding={k:result[k]for k in ('session_id','model_binding','conversion_id','action_contract','task')}
                checked=validate_action_chunk(result,binding=binding,request_id='chunk32',epoch=1,clock_domain='cpu-source',
                    now=2.1,observation_source_time=2.,valid_until_source_time=4.5)
                np.testing.assert_array_equal(checked['action14'],np.arange(448,dtype=np.float32).reshape(32,14))
                self.assertEqual(calls[-1]['observation']['images']['base_0_rgb'].dtype,np.uint16)
                from robot_workbench.inference_owner import configuration
                from robot_workbench.execution_settings import Guard,control_parameters,defaults,policy_hash
                from test_variable_inference_steps import config
                for steps in (1,15,32):
                    candidate=config(steps);candidate.update(model_id=MODEL_ID,maximum_observation_age_s=2.5,action_chunk_binding=binding)
                    frozen=configuration(candidate)
                    self.assertEqual(frozen['execute_steps'],steps)
                    self.assertIsNone(frozen['reference_planning']['max_execution_duration_s'])
                    self.assertEqual(frozen['state_max_age_s'],.1);self.assertEqual(frozen['max_camera_span_s'],.1)
                    parameters=control_parameters(defaults('inference'),configuration=frozen,business='inference')
                    policy=dict(revision=1,parameters=parameters,policy_hash=policy_hash(parameters))
                    guard=Guard.bind(policy,{})
                    for rule,age,verdict in [('G02',2.,'pass'),('G02',2.5001,'reject'),('G03',.1001,'reject')]:
                        decision=guard.evaluate(dict(rule_id=rule,signal_id='cpu',observed=age,source_time=100.,checked_time=100.+age),'input')
                        self.assertEqual(decision['verdict'],verdict)
                candidate['execute_steps']=33
                with self.assertRaises(ValueError):configuration(candidate)
                pi=config();pi['maximum_observation_age_s']=2.5
                with self.assertRaises(ValueError):configuration(pi)
                before=len(calls);bad=dict(request,applied_history=np.zeros((1,14)))
                with self.assertRaisesRegex(RuntimeError,'history'):pin.infer(bad)
                self.assertEqual(len(calls),before)
                changed=deepcopy(bundle);changed['artifacts']['stats']['sha256']='f'*64
                with self.assertRaises(ValueError):model_descriptor(MODEL_ID,changed)
            finally:
                if pin is not None:pin.close()
                server.shutdown();server.server_close();thread.join(3);server.model_service.close()

    def stats(self):
        value=dict(schema='phoenix.joint14.normalization.v1',gripper_unit='m',train_only=True,
            state=dict(mean=np.linspace(-.2,.3,14).tolist(),std=np.linspace(.05,.8,14).tolist()),
            action=dict(mean=np.linspace(-.3,.4,14).tolist(),std=np.linspace(.1,.7,14).tolist()))
        return wam.normalization(value)

    def test_complete_cpu_input_and_output_match_reference_bit_for_bit(self):
        import torch
        import torch.nn.functional as F
        self.assertFalse(torch.cuda.is_initialized())
        stats=self.stats();raw_state=np.linspace(-.5,.8,14,dtype=np.float32)
        images={k:np.random.default_rng(i).integers(0,256,(480,640,3),dtype=np.uint8)for i,k in enumerate(wam.CAMS)}
        state=wam.prepare_state(raw_state,stats);prepared=wam.prepare_images(images)
        # This is the source panel formula, including its CPU resize, two BF16
        # roundings and full80-dimensional noise draw before invalid slots zero.
        rgb=np.stack([images[k]for k in wam.CAMS])
        rgb=torch.from_numpy(rgb).permute(0,3,1,2).float()/255
        rgb=F.interpolate(rgb,size=(256,256),mode='bicubic',align_corners=False).clamp(0,1)
        wan=(rgb*2-1)[None,:,:,None].to(torch.bfloat16)
        mean=torch.tensor([.485,.456,.406],dtype=torch.bfloat16).view(1,1,3,1,1,1)
        std=torch.tensor([.229,.224,.225],dtype=torch.bfloat16).view(1,1,3,1,1,1)
        phoenix=((wan.float()*.5+.5).to(torch.bfloat16)-mean)/std
        reference_state=torch.zeros(1,80)
        reference_state[0,list(wam.SLOTS)]=torch.from_numpy((raw_state-stats['state'][0])/stats['state'][1])
        context=torch.zeros((1,200,4096),dtype=torch.bfloat16);context[:,:128]=.25
        mask=torch.zeros((1,200),dtype=torch.bool);mask[:,:128]=True
        for seed in (7,20261830):
            actual=wam.network_inputs(state,prepared,context,mask,seed=seed)
            rng=torch.Generator(device='cpu').manual_seed(seed)
            video_noise=torch.randn((1,96,2,16,48),generator=rng).to(torch.bfloat16)
            action_noise=torch.randn((1,32,80),generator=rng).to(torch.bfloat16)
            action_noise[...,list(wam.PAD)]=0
            reference=dict(wan_images=wan,phoenix_images=phoenix,proprio=reference_state.to(torch.bfloat16),
                context=context,context_mask=mask,video_noise=video_noise,action_noise=action_noise)
            self.assertEqual(set(actual),set(reference))
            for name,tensor in actual.items():
                expected=reference[name]
                self.assertEqual(tensor.device.type,'cpu');self.assertEqual(tensor.dtype,expected.dtype)
                self.assertEqual(tuple(tensor.shape),tuple(expected.shape))
                view=torch.uint16 if tensor.dtype==torch.bfloat16 else torch.uint8
                self.assertTrue(torch.equal(tensor.contiguous().view(view),expected.contiguous().view(view)),name)
        normalized=np.arange(32*80,dtype=np.float32).reshape(32,80)/1000
        normalized[:,wam.PAD]=0
        np.testing.assert_array_equal(wam.decode_actions(normalized,stats),
            normalized[:,wam.SLOTS]*stats['action'][1]+stats['action'][0])
        # Valid text length belongs to the cached context, not one checkpoint.
        mask[:,91:]=False;context[:,91:]=0
        self.assertTrue(torch.equal(wam.network_inputs(state,prepared,context,mask,seed=7)['context_mask'],mask))
        self.assertFalse(torch.cuda.is_initialized())

    def test_invalid_layout_values_and_unpinned_stats_refuse_before_model_use(self):
        import torch
        stats=self.stats()
        ties=np.asarray([0.,-0.,1.00390625,1.01171875,-1.00390625,.1,-.1],np.float32)
        reference=torch.from_numpy(ties).to(torch.bfloat16).float().numpy()
        np.testing.assert_array_equal(wam.bf16_values(ties).view(np.uint32),reference.view(np.uint32))
        for shape in ((60,80),(32,14),(31,80)):
            with self.assertRaises(ValueError):wam.decode_actions(np.zeros(shape),stats)
        invalid=np.zeros((32,80),np.float32);invalid[0,wam.PAD[0]]=1
        with self.assertRaises(ValueError):wam.decode_actions(invalid,stats)
        invalid=np.zeros((32,80),np.float32);invalid[0,0]=np.nan
        with self.assertRaises(ValueError):wam.decode_actions(invalid,stats)
        with self.assertRaises(ValueError):wam.prepare_state([float('nan')]*14,stats)
        with TemporaryDirectory()as folder:
            path=Path(folder)/'normalization.json';path.write_text('{}')
            with self.assertRaisesRegex(ValueError,'SHA256'):wam.load_normalization(path,expected_sha256=PINS['stats'])


if __name__=='__main__':unittest.main()
