"""Real HTTP + existing preparation thread/notify; GPU steps are CPU barriers."""
from copy import deepcopy
import json
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import threading
import time
from types import SimpleNamespace as NS
import unittest
from urllib.error import HTTPError
from urllib.request import Request,build_opener,ProxyHandler
from unittest.mock import Mock,patch

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'scripts'),str(ROOT/'rule_based_workbench')]
from robot_workbench.resident_collection import CollectionPreparation
from robot_workbench.service import Application,Device,serve


class FixturePreparation(CollectionPreparation):
    def __init__(self,**kwargs):
        self.received=[];self.steps=threading.Event();self.finish=threading.Event()
        super().__init__(**kwargs)
    def _prepare(self,snapshot):
        if self.parts:self._require_idle();self._close_parts()
        self.received.append(deepcopy(snapshot))
        self._status('converting_planning_assets',output='cpu-conversion-fixture')
        if len(self.received)==1:raise RuntimeError('SRE module mismatch: CPU fixture of the actual conversion failure')
        self._status('initializing_detector',worker={'message':'正在加载检测模型'})
        self.steps.set();self.finish.wait(2)
        self._status('initializing_planner',worker={'message':'正在初始化轨迹规划器'})
        self._status('ready',error=None)


class PreparationHTTPTest(unittest.TestCase):
    def setUp(self):
        self.tmp=TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.device=Device('fake-preparation','fake',None)
        self.device.submit=Mock(side_effect=AssertionError('preparation must never submit control'))
        self.device.connect=Mock(side_effect=AssertionError('preparation must never reconnect'))
        self.application=Application({'fake':self.device})
        self.install=dict(verified=True,head_id='accepted-head',wrist_id='accepted-wrists',head_version='v000002',wrist_version='v000003')
        self.read_install=Mock(side_effect=lambda:deepcopy(self.install))
        self.preparation=FixturePreparation(contract={},output=Path(self.tmp.name),base={},device=self.device,application=self.application,
            ports={},directories={},control_url='http://127.0.0.1:1',token_file='unused',
            deadline_wall_ns=time.time_ns()+120_000_000_000,installed_snapshot=self.read_install)
        self.device.collection_preparation=self.preparation
        self.server=serve(self.application,port=0);self.thread=threading.Thread(target=self.server.serve_forever,daemon=True);self.thread.start()
        self.addCleanup(self.close)
        self.url='http://127.0.0.1:'+str(self.server.server_port);self.http=build_opener(ProxyHandler({}));self.count=0

    def close(self):
        self.preparation.finish.set();self.preparation.parts=None;self.preparation.close()
        self.server.shutdown();self.server.server_close();self.thread.join(1)

    def state(self):return json.loads(self.http.open(self.url+'/api/state?environment=fake&business=collection',timeout=1).read())

    def retry(self):
        self.count+=1
        request=Request(self.url+'/api/actions/prepare_collection',json.dumps(dict(environment='fake',business='collection',request_id=str(self.count))).encode(),
            headers={'X-CSRF-Token':self.application.csrf,'Origin':self.url,'Content-Type':'application/json'})
        with self.http.open(request,timeout=1) as response:return json.loads(response.read())

    def wait(self,predicate):
        end=time.monotonic()+2
        while time.monotonic()<end:
            value=self.state()
            if predicate(value):return value
            time.sleep(.01)
        self.fail('preparation did not converge: '+str(value))

    def failed(self):
        self.preparation.notify(self.install)
        return self.wait(lambda state:state['workflow']['status']=='failed' and not state['workflow']['busy'])

    def old_parts(self):
        from playground import Playground
        from robot_workbench.workflows import CollectionWorkflow
        playground=Playground.__new__(Playground);playground.directory=Path(self.tmp.name)/'old-plan'
        playground.directory.mkdir();playground.lock=threading.RLock()
        playground.state=dict(busy=False,plan={'validated':True,'timeline_digest':'old-timeline'})
        playground.cancel_event=threading.Event();playground.thread=None
        (playground.directory/'prepared-plan.json').write_text(json.dumps({'plan':playground.state['plan']}))
        workflow=CollectionWorkflow.__new__(CollectionWorkflow)
        workflow.playground=playground;workflow.closed=threading.Event();workflow.lock=threading.Lock();workflow.generation=0
        workflow.value=dict(status='ready',busy=False,primary_action=None);workflow.catalog=[];workflow.review_key='old-review'
        self.device.workflows['collection']=workflow
        parts=NS(playground=playground,workflow=workflow,detector=Mock(),planner=Mock())
        self.preparation.parts=parts;return parts

    def test_real_failure_is_visible_and_retry_only_reuses_current_verified_install(self):
        before=deepcopy(self.install);failed=self.failed();view=failed['workflow']
        self.assertEqual(view['stage_id'],'collection.setup.failed')
        self.assertIn('SRE module mismatch',view['message']);self.assertIn('无需重新采集',view['message'])
        self.assertEqual(view['preparation']['failed_stage'],'converting_planning_assets')
        self.assertEqual(view['primary_action']['id'],'prepare_collection');self.assertTrue(view['primary_action']['allowed'])
        self.assertFalse(failed['runtime']['connected'])
        self.retry();self.assertTrue(self.preparation.steps.wait(1))
        initializing=self.wait(lambda state:state['workflow']['stage_id']=='collection.setup.initializing_detector')
        self.assertIn('正在加载检测模型',initializing['workflow']['message']);self.assertTrue(initializing['workflow']['busy'])
        with self.assertRaises(HTTPError):self.retry()
        self.assertEqual(self.preparation.received,[before,before]);self.assertEqual(self.install,before)
        self.read_install.assert_called_once();self.device.submit.assert_not_called();self.device.connect.assert_not_called()
        self.preparation.finish.set();self.wait(lambda state:state['workflow']['stage_id']=='collection.setup.ready')
        self.assertEqual(self.preparation.deadline_wall_ns,initializing['workflow']['preparation']['resident_deadline_wall_ns'])

    def test_retry_revalidates_install_and_respects_deadline_and_existing_D_job(self):
        self.failed();self.install['verified']=False
        self.retry()
        self.wait(lambda state:(state['workflow'].get('request') or {}).get('status')=='failed')
        self.assertEqual(len(self.preparation.received),1);self.assertFalse(self.preparation.snapshot()['can_retry'])
        self.install['verified']=True
        with self.preparation.lock:self.preparation.value['installed']=deepcopy(self.install)
        self.preparation.parts=NS(playground=NS(lock=threading.Lock(),state={'busy':True}))
        with self.assertRaisesRegex(ValueError,'数采任务尚未结束'):self.preparation.retry()
        self.assertEqual(len(self.preparation.received),1)
        self.preparation.parts=None;self.preparation.deadline_wall_ns=time.time_ns()+30_000_000_000
        expired=self.state()['workflow'];self.assertFalse(expired['primary_action']['allowed'])
        with self.assertRaises(HTTPError):self.retry()
        self.device.submit.assert_not_called();self.device.connect.assert_not_called()

    def test_disconnected_old_parts_retire_without_dereferencing_runtime_holder(self):
        self.failed();parts=self.old_parts();self.preparation.identity={'old':'prepared'}
        self.preparation.contract={'collection_setup':{}}
        manifest=Path(self.tmp.name)/'calibrations/hardware/active/manifest.json'
        manifest.parent.mkdir(parents=True);manifest.write_text('{}')
        self.assertIsNone(self.device.runtime)
        with patch('robot_workbench.preview_service.resident_execution',side_effect=RuntimeError('CPU stop before conversion')):
            with self.assertRaisesRegex(RuntimeError,'CPU stop before conversion'):
                CollectionPreparation._prepare(self.preparation,self.install)
        parts.detector.close.assert_called_once();parts.planner.close.assert_called_once()
        self.assertIsNone(self.preparation.parts);self.assertIsNone(self.preparation.identity)
        self.assertFalse(parts.playground.state['plan']['validated'])
        self.device.submit.assert_not_called();self.device.connect.assert_not_called()

    def test_retry_revokes_old_D_review_and_checkpoint_and_joins_late_monitor(self):
        self.failed();parts=self.old_parts();errors=[]
        self.application.reviews[('fake','collection')]={'id':'old-review'}
        self.application.reviews[('fake','calibration')]={'keep':'C review'}
        def late_review():
            parts.workflow.closed.wait(2)
            try:self.application.add_review(self.device,'collection',{'token':'old-plan','command_epoch':1},'execute','old')
            except Exception as error:errors.append(error)
        parts.workflow.thread=threading.Thread(target=late_review);parts.workflow.thread.start()
        self.retry();self.assertTrue(self.preparation.steps.wait(1))
        self.assertFalse(parts.workflow.thread.is_alive());self.assertFalse(errors)
        self.assertNotIn(('fake','collection'),self.application.reviews)
        self.assertEqual(self.application.reviews[('fake','calibration')],{'keep':'C review'})
        saved=json.loads((parts.playground.directory/'prepared-plan.json').read_text())
        self.assertTrue(saved['invalidated_by_interrupt']);self.assertFalse(parts.playground.state['plan']['validated'])
        initializing=self.state()['workflow'];self.assertNotIn('review',initializing)
        self.preparation.finish.set();self.wait(lambda state:not self.preparation.snapshot()['busy'])
        self.assertNotIn('review',self.state()['workflow'])
        self.assertNotIn(('fake','collection'),self.application.reviews)
        self.device.submit.assert_not_called();self.device.connect.assert_not_called()


if __name__=='__main__':unittest.main()
