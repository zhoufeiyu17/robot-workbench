from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import time
from types import SimpleNamespace
import unittest

from robot_workbench.inference_trace import InferenceTrace,RunArtifacts,write_json,CaptureRun
from robot_workbench.inference_history import checked_entry


def setUpModule():
    global recording_fixture
    from test_recording_process import start_recording_fixture
    recording_fixture=start_recording_fixture()


def tearDownModule():
    from test_recording_process import stop_recording_fixture
    stop_recording_fixture(recording_fixture)


class InferenceTraceTests(unittest.TestCase):
    def test_all_outcomes_and_late_events_remain_recorded(self):
        with TemporaryDirectory() as tmp:
            writer=InferenceTrace(tmp)
            for status in ('completed','failed','stopped'):
                writer.record(status,'created',{'configuration':{'token':'never-store','model_id':'m'}})
                writer.record(status,'command',{'packet':{'q_sdk14':[[0.]*14]}})
                writer.record(status,'finish',{'status':status})
                writer.record(status,'late_receipt',{'ok':True})
            writer.record('unfinished','created',{})
            writer.close()
            for status in ('completed','failed','stopped'):
                path=Path(tmp)/('inference-'+status)
                manifest=json.loads((path/'trace-manifest.json').read_text())
                self.assertEqual(manifest['terminal']['status'],status)
                self.assertEqual(manifest['event_counts']['late_receipt'],1)
                self.assertEqual(manifest['sha256'],hashlib.sha256((path/'trace.jsonl').read_bytes()).hexdigest())
                self.assertNotIn('never-store',(path/'trace.jsonl').read_text())
            self.assertEqual(json.loads((Path(tmp)/'inference-unfinished/trace-manifest.json').read_text())['status'],'incomplete')

    def test_failure_before_owner_execution_has_a_complete_receipt(self):
        with TemporaryDirectory() as tmp,ThreadPoolExecutor(max_workers=1) as pool:
            published=[];inputs=SimpleNamespace(root=Path(tmp)/'native-inputs',writer=pool,snapshot=lambda:{'error':None})
            artifacts=RunArtifacts(tmp,inputs,lambda *x:published.append(x))
            handle=artifacts.begin({'id':'job','started_wall_ns':1},{'seed':0,'token':'secret'})
            artifacts.finish(handle,{'status':'failed','error':'model not ready','finished_wall_ns':2},{'connected':False})
            artifacts.close()
            receipt,path=published[0]
            self.assertEqual(receipt['artifact_status'],'complete');self.assertEqual(receipt['trace']['status'],'not_started')
            self.assertEqual(receipt['status'],'failed');self.assertNotIn('secret',path.read_text())

    def test_inputs_are_hardlinked_into_stopped_run(self):
        with TemporaryDirectory() as tmp,ThreadPoolExecutor(max_workers=1) as pool:
            root=Path(tmp);inputs=SimpleNamespace(root=root/'native-inputs',writer=pool,snapshot=lambda:{'error':None})
            source=inputs.root/'prediction';source.mkdir(parents=True)
            for name in ('input.npz','request.json','output.npz'):(source/name).write_bytes(b'fixed')
            write_json(root/'inference-exe/trace-manifest.json',{'status':'complete','terminal':{'status':'stopped'}})
            published=[];artifacts=RunArtifacts(root,inputs,lambda *x:published.append(x))
            handle=artifacts.begin({'id':'job','started_wall_ns':1},{'seed':2})
            artifacts.bind(handle,'exe',{'model_id':'m'});artifacts.model_request(handle,'prediction')
            artifacts.finish(handle,{'status':'cancelled','finished_wall_ns':2},{'state':'STOPPED'})
            artifacts.close()
            receipt,path=published[0]
            self.assertEqual(receipt['artifact_status'],'complete')
            self.assertEqual((source/'input.npz').stat().st_ino,(root/'inference-exe/model/prediction/input.npz').stat().st_ino)

    def test_backpressure_never_blocks_producer_and_marks_incomplete(self):
        with TemporaryDirectory() as tmp:
            writer=InferenceTrace(tmp,capacity=1)
            start=time.monotonic()
            for i in range(1000):writer.record('slow','sample',{'index':i,'q':[0.]*14})
            elapsed=time.monotonic()-start
            writer.record('slow','finish',{'status':'failed'});writer.close()
            self.assertLess(elapsed,1.)
            self.assertGreater(writer.dropped['slow'],0)
            value=json.loads((Path(tmp)/'inference-slow/trace-manifest.json').read_text())
            self.assertEqual(value['status'],'incomplete')

    def test_history_accepts_only_named_debug_manifest(self):
        entry={'result':{'id':'debug-job','kind':'inference_debug'},'path':'inference-job/debug-receipt.json'}
        checked_entry(entry)
        for path in ('../secret.json','inference-job/secret.json'):
            with self.assertRaises(ValueError):checked_entry(dict(entry,path=path))

    def test_real_history_publisher_preserves_success_failure_and_stop(self):
        from robot_workbench.inference_history import InferenceHistory
        from robot_workbench.inference_workflow import InferenceWorkflow
        with TemporaryDirectory() as tmp,ThreadPoolExecutor(max_workers=1) as pool:
            root=Path(tmp);workflow=object.__new__(InferenceWorkflow)
            workflow.device=SimpleNamespace(environment='physical');workflow.history=InferenceHistory(root)
            inputs=SimpleNamespace(root=root/'native-inputs',writer=pool,snapshot=lambda:{'error':None})
            artifacts=RunArtifacts(root,inputs,workflow._debug_finished)
            trace=InferenceTrace(root)
            for status in ('completed','failed','cancelled'):
                handle=artifacts.begin({'id':status,'started_wall_ns':1},{'seed':0})
                artifacts.bind(handle,status,{'model_id':'test'})
                trace.record(status,'created',{'configuration':{'model_id':'test'}})
                trace.record(status,'finish',{'status':status})
                artifacts.finish(handle,{'status':status,'finished_wall_ns':2},{'state':'READY'})
            trace.close();artifacts.close()
            rows=workflow.history.page()['results']
            self.assertEqual({r['status'] for r in rows},{'completed','failed','cancelled'})
            self.assertTrue(all(r['details']['artifact_status']=='complete' for r in rows))
            self.assertTrue(all(workflow.history.media(r['id']).name=='debug-receipt.json' for r in rows))


class CaptureRunTests(unittest.TestCase):
    def test_all_run_outcomes_keep_model_control_and_history_together(self):
        import numpy as np
        from robot_workbench.inference_evidence import InferenceEvidence
        from robot_workbench.execution_settings import control_parameters,defaults,policy_hash
        for status in ('completed','failed','stopped'):
            with TemporaryDirectory() as tmp:
                root=Path(tmp);inputs=InferenceEvidence(root/'inputs')
                parameters=control_parameters(defaults('inference'),business='inference')
                policy=dict(revision=3,parameters=parameters,policy_hash=policy_hash(parameters));published=[]
                capture=CaptureRun('cpu-run',root/'run',{'model_id':'cpu-model','clock':{'source_clock_id':'cpu-clock'}},
                    policy,SimpleNamespace(capture_sources=lambda:[]),'fake',inputs,lambda:True,published.append)
                try:
                    owner=dict(owner_pid=1,runtime_nonce='cpu-owner',command_epoch=2)
                    capture.record('prepare',dict(capture_sequence=1,capture_source_time=1.,capture_wall_ns=1,
                        execution_owner=owner,control_clock_id='cpu-clock'))
                    self.assertIsNone(capture.error)
                    state=np.arange(14,dtype=np.float32)
                    images={'base_0_rgb':np.zeros((2,2,3),dtype=np.uint8)}
                    capture.model_begin('prediction',1.01)
                    inputs.input('prediction',state,images,{'request_id':'prediction'})
                    capture.model_end('prediction',inputs.output('prediction',{'action14':np.ones((60,14)),'audit':{}},chunk=False))
                    capture.record('feedback',dict(capture_sequence=2,capture_source_time=1.02,capture_wall_ns=2,raw={'qpos':[0.]*14}))
                    capture.record('finish',dict(status=status,reason='CPU terminal',final_capture_sequence=2,source_time=1.03))
                    self.assertTrue(capture.done.wait(15),capture.snapshot())
                    self.assertEqual(capture.manifest['recording_outcome'],'complete')
                    self.assertEqual(capture.manifest['control_terminal']['status'],status)
                    self.assertEqual(capture.manifest['streams']['control']['committed'],2)
                    row=json.loads((capture.root/'model.jsonl').read_text())
                    ref=row['payload']['files']['input.npz']['payload_ref']
                    with np.load(capture.root/ref['path']) as saved:np.testing.assert_array_equal(saved['state14'],state)
                    self.assertEqual(capture.manifest['binding']['policy_hash'],policy['policy_hash'])
                    self.assertEqual(published,[capture])
                    from robot_workbench.inference_workflow import InferenceWorkflow
                    from robot_workbench.inference_history import InferenceHistory
                    import threading
                    workflow=object.__new__(InferenceWorkflow);workflow.device=SimpleNamespace(environment='fake')
                    workflow.lock=threading.RLock();workflow.history=InferenceHistory(root)
                    run=dict(id='cpu-run',directory=capture.directory,config=capture.config,control_policy=policy)
                    if status=='failed':capture.manifest['playback_error']='CPU codec export failed'
                    workflow._finish_capture(run,capture)
                    result=workflow.history.page()['results'][0]
                    self.assertEqual(result['status'],status)
                    self.assertEqual(result['details']['artifact_status'],'complete')
                    self.assertEqual(workflow.history.media(result['id']).name,'debug-receipt.json')
                    if status=='failed':self.assertEqual(result['details']['reason'],'CPU codec export failed')
                finally:inputs.close();capture.close()

    def test_control_sequence_gap_is_not_hidden_by_successful_sealing(self):
        from robot_workbench.inference_evidence import InferenceEvidence
        from robot_workbench.execution_settings import control_parameters,defaults,policy_hash
        with TemporaryDirectory() as tmp:
            inputs=InferenceEvidence(Path(tmp)/'inputs');parameters=control_parameters(defaults('inference'),business='inference')
            capture=CaptureRun('gap-run',Path(tmp)/'run',{'model_id':'cpu','clock':{'source_clock_id':'cpu'}},
                dict(revision=0,parameters=parameters,policy_hash=policy_hash(parameters)),
                SimpleNamespace(capture_sources=lambda:[]),'fake',inputs,lambda:True,lambda _:None)
            try:
                capture.record('prepare',dict(capture_sequence=1,capture_source_time=1.,execution_owner=dict(owner_pid=1,runtime_nonce='cpu',command_epoch=1),control_clock_id='cpu'))
                capture.record('feedback',dict(capture_sequence=3,capture_source_time=1.1,raw={}))
                capture.record('finish',dict(status='completed',final_capture_sequence=3,source_time=1.2))
                self.assertTrue(capture.done.wait(10))
                self.assertEqual(capture.manifest['recording_outcome'],'incomplete')
                self.assertEqual(capture.manifest['streams']['control']['missing'],1)
            finally:inputs.close();capture.close()

    def test_business_close_without_owner_barrier_stays_incomplete(self):
        from robot_workbench.inference_evidence import InferenceEvidence
        from robot_workbench.execution_settings import control_parameters,defaults,policy_hash
        with TemporaryDirectory() as tmp:
            inputs=InferenceEvidence(Path(tmp)/'inputs');parameters=control_parameters(defaults('inference'),business='inference')
            capture=CaptureRun('lost-owner',Path(tmp)/'run',{'model_id':'cpu','clock':{'source_clock_id':'cpu'}},
                dict(revision=0,parameters=parameters,policy_hash=policy_hash(parameters)),
                SimpleNamespace(capture_sources=lambda:[]),'fake',inputs,lambda:True,lambda _:None)
            try:
                capture.record('prepare',dict(capture_sequence=1,capture_source_time=1.,execution_owner=dict(owner_pid=1,runtime_nonce='cpu',command_epoch=1),control_clock_id='cpu'))
                capture.close()
                self.assertTrue(capture.done.is_set())
                self.assertEqual(capture.manifest['recording_outcome'],'incomplete')
                self.assertEqual(capture.manifest['control_terminal']['status'],'unknown')
            finally:inputs.close();capture.close()


if __name__=='__main__':unittest.main()
