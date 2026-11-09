"""Mixed nohistory identities, raw input protocol and nonblocking startup; no GPU."""
from copy import deepcopy
import json
import hashlib
import threading
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np
from robot_workbench.inference_model import MIXED366_ID,MIXED366_SCHEMA,MIXED366_PINS,CAMS,validate_bundle,model_spec,worker_identity,model_bundle_digest
from robot_workbench.inference_transport import ModelService,PROTOCOL,_validate_request
from robot_workbench.mixed366 import descriptor,registered_desktop,Service
from scripts.start_registered_workbench import start

REG=dict(bundle_sha256='a'*64,source_manifest_sha256='b'*64,runtime_sha256='c'*64)

class MixedTest(unittest.TestCase):
    def test_added_checkpoint_creates_its_own_worker_log_directory(self):
        from robot_workbench.mixed366 import Client
        with TemporaryDirectory()as tmp:
            root=Path(tmp)/'added-checkpoint'
            client=Client(dict(root=str(root)),dict(model_id=MIXED366_ID))
            self.assertTrue(Path(client.log_path).parent.is_dir())
            self.assertIsNone(client.process)

    def test_backend_change_is_not_reported_as_a_load_failure(self):
        from robot_workbench.inference_transport import ModelSelectionChanged
        from robot_workbench.inference_workflow import InferenceWorkflow
        def changed():raise ModelSelectionChanged('explicit mode changed')
        f=SimpleNamespace(lock=threading.RLock(),model_client=SimpleNamespace(status=changed),generation=1)
        InferenceWorkflow._refresh_model_fact(f)
        self.assertEqual(f.model_service_fact['status'],'selection_changed');self.assertIs(f.model_service_fact['loaded'],False)
    def test_desktop_exception_pins_boot_process_and_command(self):
        with TemporaryDirectory()as tmp:
            p=Path(tmp);(p/'123').mkdir();(p/'sys/kernel/random').mkdir(parents=True)
            raw=b'/opt/google/chrome/chrome --type=gpu-process --x=y\0'
            (p/'123/cmdline').write_bytes(raw);(p/'123/stat').write_text('123 (chrome) '+' '.join(['S']+['0']*18+['99']))
            (p/'sys/kernel/random/boot_id').write_text('boot')
            r=dict(desktop_gpu_processes={'123':dict(exe='/opt/google/chrome/chrome',start_ticks='99',boot_id='boot',cmdline_sha256=hashlib.sha256(raw).hexdigest())})
            self.assertTrue(registered_desktop('123',r,p));self.assertFalse(registered_desktop('124',r,p))
            (p/'123/cmdline').write_bytes(raw+b'changed');self.assertFalse(registered_desktop('123',r,p))
    def test_restarted_browser_is_verified_from_executable_user_and_desktop_session(self):
        with TemporaryDirectory()as tmp:
            root=Path(tmp);p=root/'456';p.mkdir();exe=root/'chrome';exe.touch()
            (p/'exe').symlink_to(exe);(p/'cmdline').write_bytes(b'chrome\0--type=gpu-process\0')
            (p/'stat').write_text('456 (chrome) '+' '.join(['S']+['0']*18+['100']))
            (p/'cgroup').write_text('0::/user.slice/user-1000.slice/user@1000.service/session.slice/desktop.service\n')
            registry=dict(desktop_gpu_processes={'123':dict(exe=str(exe),start_ticks='old')})
            self.assertTrue(registered_desktop('456',registry,root))
            with patch('robot_workbench.mixed366.os.getuid',return_value=-1):
                self.assertFalse(registered_desktop('456',registry,root))
            (p/'cgroup').write_text('0::/system.slice/unknown.service\n')
            self.assertFalse(registered_desktop('456',registry,root))
            (p/'cmdline').write_bytes(b'python\0model.py\0')
            self.assertFalse(registered_desktop('456',registry,root))
    def test_catalog_does_not_hash_gigabytes_on_every_poll(self):
        from robot_workbench import mixed366
        with TemporaryDirectory()as tmp:
            p=Path(tmp).resolve();(p/'jobs').mkdir();(p/'triton').mkdir();(p/'model.pt').write_bytes(b'packed')
            d=dict(status='READY',bundle_sha256=REG['bundle_sha256'],packed='model.pt',packed_sha256=hashlib.sha256(b'packed').hexdigest());(p/'triton/deployment.json').write_text(json.dumps(d))
            with patch.object(mixed366,'sha',wraps=mixed366.sha)as hashed:
                s=Service(dict(REG,root=str(p)));s.models();s.models()
                self.assertEqual(sum(Path(x.args[0])==p/'model.pt'for x in hashed.call_args_list),1)
                (p/'model.pt').write_bytes(b'tampered')
                with self.assertRaises(ValueError):s.models()
    def test_identity_never_inherits_history_or_old_stats(self):
        b=descriptor(REG);self.assertEqual(validate_bundle(b),b);self.assertIn('无history',model_spec(MIXED366_ID,b)['label'])
        self.assertEqual(worker_identity(MIXED366_ID,b)['history_steps'],0)
        for kind in ['checkpoint','stats','io_contract']:
            bad=deepcopy(b);bad['artifacts'][kind]['sha256']='d'*64
            with self.assertRaises(ValueError):validate_bundle(bad)
        bad=deepcopy(b);bad['schema']='workbench.manipulation_base566.v1'
        with self.assertRaises(ValueError):validate_bundle(bad)
    def test_raw_images_and_no_history_required(self):
        b=descriptor(REG);meta=dict(protocol=PROTOCOL,model_id=MIXED366_ID,conversion_id='0'*64,request_id='cpu',seed=0,source_time=0.,frame_ids={},backend='torch',task='coffee',expected_bundle_digest=model_bundle_digest(MIXED366_ID,b))
        a=dict(model_state14=np.zeros(14,np.float32),model_history=np.empty((0,14),np.float32),**{c:np.zeros((480,640,3),np.uint8)for c in CAMS});_validate_request(meta,a,MIXED366_ID,b)
        a['model_history']=np.zeros((1,14),np.float32)
        with self.assertRaises(ValueError):_validate_request(meta,a,MIXED366_ID,b)
        a['model_history']=np.empty((0,14),np.float32);a[CAMS[0]]=np.zeros((224,224,3),np.uint8)
        with self.assertRaises(ValueError):_validate_request(meta,a,MIXED366_ID,b)
    def test_start_enqueues_once_and_waits_for_verified_readiness(self):
        config=dict(units=['robot-workbench-example.service'],url='http://127.0.0.1:18868')
        state=dict(health={'backend_reachable':True},target={'services':{'model':{'pid':5}}},model={'ready':False},operations={'prepare_model':{'allowed':True}},csrf='fixture',deployment={'sha256':'a'*64})
        completed={'status':'succeeded','verification':{'ok':True}};ready=dict(state,model={'ready':True})
        with patch('scripts.start_registered_workbench.subprocess.run')as run,patch('scripts.start_registered_workbench.http_json',side_effect=[state,{'workflow':{'manual_queue':{'available':True}}},{'job_id':'warm1'},{'status':'running'},completed,ready])as api,patch('scripts.start_registered_workbench.time.sleep'):
            result=start(config);self.assertTrue(result['ui_ready']);self.assertEqual(result['model_status'],'ready');self.assertEqual(api.call_count,6);run.assert_called_once()
        state['workflow']={'manual_queue':{'available':True}};state['active_job']='warm1';state['jobs']={'warm1':dict(job_id='warm1',request={'action':'prepare_model'})}
        with patch('scripts.start_registered_workbench.subprocess.run'),patch('scripts.start_registered_workbench.http_json',side_effect=[state,state,completed,ready])as api:
            self.assertTrue(start(config)['reused']);self.assertEqual(api.call_count,4)
    def test_warmup_uses_original_size_and_http_service_no_robot(self):
        b=descriptor(REG);requests=[]
        class C:
            model_id=MIXED366_ID;bundle=b
            def prepare(self):return worker_identity(MIXED366_ID,b)
            def infer(self,q):requests.append(q);return dict(request_id=q['request_id'],action14=np.zeros((60,14),np.float32))
            def close(self):pass
        service=ModelService(C());service.prepare(dict(protocol=PROTOCOL,model_id=MIXED366_ID,conversion_id='0'*64,expected_bundle_digest=model_bundle_digest(MIXED366_ID,b),backend='torch'))
        self.assertTrue(service.status()['loaded']);self.assertEqual(requests[0]['observation']['images'][CAMS[0]].shape,(480,640,3));self.assertEqual(len(requests[0]['applied_history']),0)

    def test_task_is_required_and_reaches_worker_through_wire(self):
        from robot_workbench.inference_transport import request_packet,ModelDomainAdapter,decode_packet
        b=descriptor(REG);requests=[]
        class Client:
            model_id=MIXED366_ID;bundle=b
            def prepare(self):return worker_identity(MIXED366_ID,b)
            def infer(self,q):requests.append(q);return dict(request_id=q['request_id'],action14=np.zeros((60,14),np.float32))
            def close(self):pass
        s=ModelService(Client());s.prepare(dict(protocol=PROTOCOL,model_id=MIXED366_ID,conversion_id='0'*64,expected_bundle_digest=model_bundle_digest(MIXED366_ID,b),backend='torch'))
        q=dict(request_id='cpu',seed=0,task='lid_handoff',applied_history=np.empty((0,14),np.float32),
            observation=dict(model_state14=np.zeros(14,np.float32),source_time=1.,images={c:np.zeros((480,640,3),np.uint8)for c in CAMS}))
        packet=request_packet(q,ModelDomainAdapter(),'0'*64,MIXED366_ID,b)
        response,_=decode_packet(s.infer(packet),{'action14'},MIXED366_ID)
        self.assertEqual(requests[-1]['task'],'lid_handoff');self.assertEqual(response['audit']['task'],'lid_handoff')
        q['task']='unknown'
        with self.assertRaises(ValueError):request_packet(q,ModelDomainAdapter(),'0'*64,MIXED366_ID,b)

if __name__=='__main__':unittest.main()
