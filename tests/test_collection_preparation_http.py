"""Real HTTP + existing preparation thread/notify; GPU steps are CPU barriers."""
from copy import deepcopy
import hashlib
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
    # These tests exercise the plugin pipeline through an explicit test
    # management adapter. Real control admission has separate Runtime tests.
    def _require_control(self):pass
    def __init__(self,**kwargs):
        self.received=[];self.steps=threading.Event();self.finish=threading.Event();self.asset_checks=0
        super().__init__(**kwargs)
        self.collection_prepare_operation=self.managed_prepare
    def managed_prepare(self,_):
        self.device.maintenance=True
        try:return self.operations_prepare(still_current=lambda:True,lifecycle_admission=lambda action:True)
        finally:self.device.maintenance=False
    def _prepare(self,snapshot):
        if self.parts:
            self._require_idle();self.application.invalidate_collection_preparation(self.device);self._close_parts()
        self.received.append(deepcopy(snapshot))
        self._status('converting_planning_assets',output='cpu-conversion-fixture')
        if len(self.received)==1:raise RuntimeError('SRE module mismatch: CPU fixture of the actual conversion failure')
        self._status('initializing_detector',worker={'message':'正在加载检测模型'})
        self.steps.set();self.finish.wait(2)
        self._status('initializing_planner',worker={'message':'正在初始化轨迹规划器'})
        self._status('ready',error=None)

    def _verify_parts(self,current):
        # HTTP/lifecycle fixture: asset hashing has its own attachment tests.
        if not current():raise InterruptedError('CPU attachment verification revoked')
        self.asset_checks+=1


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

    def test_preparation_clock_includes_loading_and_advances_between_progress_events(self):
        prep=self.preparation
        with prep.lock:prep.busy=True;prep.started=100.
        with patch('robot_workbench.resident_collection.time.monotonic',return_value=125.):
            prep._status('converting_planning_assets')
        with patch('robot_workbench.resident_collection.time.monotonic',return_value=130.):
            first=prep.snapshot()['progress']
        self.assertEqual(first['elapsed_s'],30.);self.assertEqual(first['stage_elapsed_s'],5.)
        with patch('robot_workbench.resident_collection.time.monotonic',return_value=135.):
            prep._status('converting_planning_assets')
        with patch('robot_workbench.resident_collection.time.monotonic',return_value=136.):
            second=prep.snapshot()['progress']
        self.assertEqual(second['elapsed_s'],36.);self.assertEqual(second['stage_elapsed_s'],11.)
        with patch('robot_workbench.resident_collection.time.monotonic',return_value=140.):
            prep._status('initializing_detector',worker={'stage':'model'})
        with patch('robot_workbench.resident_collection.time.monotonic',return_value=142.):
            third=prep.snapshot()['progress']
        self.assertEqual(third['elapsed_s'],42.);self.assertEqual(third['stage_elapsed_s'],2.)
        with prep.lock:prep.busy=False

    def state(self):
        try:return json.loads(self.http.open(self.url+'/api/state?environment=fake&business=collection',timeout=1).read())
        except HTTPError as error:raise AssertionError(error.read().decode()) from error

    def retry(self,action='prepare_collection',request_id=None):
        self.count+=1
        request=Request(self.url+'/api/actions/'+action,json.dumps(dict(environment='fake',business='collection',request_id=request_id or str(self.count))).encode(),
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
        self.assertEqual(self.preparation.snapshot()['status'],'awaiting_prepare')
        self.assertEqual(self.preparation.received,[])
        self.retry()
        return self.wait(lambda state:state['workflow']['status']=='failed' and not state['workflow']['busy'])

    def connected_runtime(self,holder='collection'):
        initial=self.device.snapshot('collection')
        initial.update(connected=True,state='READY',feedback_healthy=True,automatic_enabled=False,
            runtime_nonce='cpu-owner',command_epoch=1,calibration_revision=0)
        runtime=NS(holder=holder)
        runtime.snapshot=lambda credential:{**deepcopy(initial),'holder':runtime.holder}
        self.device.runtime=runtime
        return runtime

    def test_install_notification_is_readonly_and_unified_prepare_requires_connection_and_handoff(self):
        self.preparation.notify(self.install)
        for _ in range(3):
            self.preparation.notify(self.install)
            state=self.state()
            self.assertEqual(state['workflow']['preparation']['status'],'awaiting_prepare')
            self.assertEqual(state['workflow']['collection']['warmup']['status'],'not_started')
            self.assertFalse(state['workflow']['busy'])
            self.assertEqual(self.preparation.received,[])
        self.assertTrue(self.preparation.notifications.empty());self.read_install.assert_not_called()
        self.assertFalse(state['runtime']['allowed_operations']['collection_prepare']['allowed'])
        with self.assertRaises(HTTPError):self.retry('collection_prepare')
        runtime=self.connected_runtime('calibration')
        self.assertIn('交接',self.state()['runtime']['allowed_operations']['collection_prepare']['reason'])
        with self.assertRaises(HTTPError):self.retry('collection_prepare')
        self.assertEqual(self.preparation.received,[])
        runtime.holder='collection'
        self.assertFalse(self.state()['runtime']['allowed_operations']['collection_prepare']['allowed'])
        with self.assertRaises(HTTPError):self.retry('collection_prepare')
        accepted=self.retry('prepare_collection',request_id='explicit-start')
        self.wait(lambda state:state['workflow']['status']=='failed' and not state['workflow']['busy'])
        self.assertEqual(self.retry('prepare_collection',request_id='explicit-start'),accepted)
        self.assertEqual(self.preparation.received,[self.install],'repeating one HTTP request must not repeat preparation')
        self.read_install.assert_called_once();self.device.submit.assert_not_called();self.device.connect.assert_not_called()

    def old_parts(self):
        from playground import Playground
        from robot_workbench.workflows import CollectionWorkflow
        playground=Playground.__new__(Playground);playground.directory=Path(self.tmp.name)/'old-plan'
        playground.directory.mkdir();playground.lock=threading.RLock()
        playground.state=dict(busy=False,plan={'validated':True,'timeline_digest':'old-timeline'})
        playground.cancel_event=threading.Event();playground.thread=None
        (playground.directory/'prepared-plan.json').write_text(json.dumps({'plan':playground.state['plan']}))
        workflow=CollectionWorkflow.__new__(CollectionWorkflow)
        workflow.device=self.device
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
        self.assertEqual(self.read_install.call_count,2);self.device.submit.assert_not_called();self.device.connect.assert_not_called()
        self.preparation.finish.set();self.wait(lambda state:state['workflow']['stage_id']=='collection.setup.ready')
        self.assertEqual(self.preparation.deadline_wall_ns,initializing['workflow']['preparation']['resident_deadline_wall_ns'])

    def test_persistent_preparation_http_is_readable_without_countdown_or_control(self):
        self.preparation.close()
        self.preparation=FixturePreparation(contract={'resources':{'persistent_service':True,'service_lifetime_seconds':None}},
            output=Path(self.tmp.name),base={},device=self.device,application=self.application,
            ports={},directories={},control_url='http://127.0.0.1:1',token_file='unused',
            deadline_wall_ns=None,installed_snapshot=self.read_install)
        self.device.collection_preparation=self.preparation
        for state in (self.state(),self.failed()):
            view=state['workflow'];preparation=view['preparation']
            self.assertTrue(preparation['persistent_service'])
            self.assertIsNone(preparation['remaining_wall_s']);self.assertIsNone(preparation['resident_deadline_wall_ns'])
            self.assertIn({'label':'服务期限','value':'长期运行 · 手动停止'},view['details'])
            self.assertFalse(state['runtime']['connected'])
        self.retry();self.assertTrue(self.preparation.steps.wait(1))
        initializing=self.state()
        self.assertEqual(initializing['workflow']['stage_id'],'collection.setup.initializing_detector')
        self.preparation.finish.set()
        self.wait(lambda state:state['workflow']['stage_id']=='collection.setup.ready')
        self.device.submit.assert_not_called();self.device.connect.assert_not_called()
        self.assertEqual(self.preparation.received,[self.install,self.install])

    def test_operations_do_not_retry_a_business_scene_failure(self):
        manifest=Path(self.tmp.name)/'calibrations/hardware/active/manifest.json'
        manifest.parent.mkdir(parents=True);manifest.write_text('{}')
        identity=dict(head_id=self.install['head_id'],wrist_id=self.install['wrist_id'],
            manifest_sha256=hashlib.sha256(manifest.read_bytes()).hexdigest())
        playground=NS(lock=threading.RLock(),state=dict(busy=False,status='failed',environment={'ready':False},
            error='CPU fixture: scene warmup failed',warmup={'status':'failed'},progress={}),
            cancel_event=threading.Event(),thread=None,persist=Mock())
        playground.start=Mock(side_effect=AssertionError('operations must not start business observations or warmup'))
        detector,planner=Mock(),Mock()
        detector.last_retirement=planner.last_retirement=None
        detector.status.return_value=planner.status.return_value={'status':'ready','model_initialized':True}
        detector.probe.return_value=planner.probe.return_value=dict(service={'status':'ready'},needs_warmup=False)
        parts=NS(playground=playground,workflow=NS(closed=threading.Event(),snapshot=lambda:deepcopy(playground.state),results=lambda:[]),detector=detector,planner=planner)
        self.preparation.parts=parts;self.preparation.identity=identity
        self.preparation._prepare=lambda snapshot:CollectionPreparation._prepare(self.preparation,snapshot)
        self.preparation.notify(self.install)
        with patch('local_execution.gpu_check') as gpu,patch('robot_workbench.collection_setup.prepare_collection_setup') as build:
            self.retry()
            self.wait(lambda state:not self.preparation.busy and self.application.jobs[('fake','collection')]['status']!='running')
            self.assertEqual(self.application.jobs[('fake','collection')]['status'],'completed')
            self.assertTrue(self.preparation.environment_status()['services_ready'])
            self.assertFalse(self.preparation.environment_status()['ready'])
            self.assertIn('scene warmup failed',playground.state['error'])
            self.assertIs(self.preparation.parts,parts);self.assertEqual(self.preparation.identity,identity)
            self.retry()
            self.wait(lambda state:self.preparation.snapshot()['status']=='ready' and not self.preparation.snapshot()['busy'])
            self.assertIs(self.preparation.parts,parts);self.assertEqual(self.preparation.identity,identity)
            gpu.assert_not_called();build.assert_not_called()
        playground.start.assert_not_called()
        for worker in (detector,planner):worker.close.assert_not_called();worker.start.assert_not_called()
        self.device.submit.assert_not_called();self.device.connect.assert_not_called()

    def test_ready_verification_preserves_review_checkpoint_and_workers(self):
        self.connected_runtime()
        self.preparation.notify(self.install)
        parts=self.old_parts()
        parts.playground.state.update(status='ready',environment={'ready':True},warmup={'status':'ready'})
        parts.playground.snapshot=lambda:deepcopy(parts.playground.state)
        parts.workflow.value['collection']={'warmup':{'status':'ready'}}
        parts.detector.status.return_value=parts.planner.status.return_value={'status':'ready','model_initialized':True}
        parts.detector.last_retirement=parts.planner.last_retirement=None
        parts.detector.probe.return_value=parts.planner.probe.return_value=dict(service={'status':'ready'},needs_warmup=False)
        parts.playground.start=Mock(return_value={'reused':True});parts.playground.persist=Mock()
        parts.playground.state['progress']={}
        manifest=Path(self.tmp.name)/'calibrations/hardware/active/manifest.json'
        manifest.parent.mkdir(parents=True);manifest.write_text('{}')
        self.preparation.identity=dict(head_id=self.install['head_id'],wrist_id=self.install['wrist_id'],
            manifest_sha256=hashlib.sha256(manifest.read_bytes()).hexdigest())
        with self.preparation.lock:self.preparation.value.update(status='ready',installed=deepcopy(self.install))
        self.preparation._prepare=Mock(side_effect=lambda snapshot:CollectionPreparation._prepare(self.preparation,snapshot))
        checkpoint=parts.playground.directory/'prepared-plan.json';before=checkpoint.read_bytes()
        self.application.add_review(self.device,'collection',dict(token='old-timeline',command_epoch=1,
            workflow_generation=parts.workflow.generation),'execute','existing reviewed plan')
        review=deepcopy(self.application.reviews[('fake','collection')])
        for _ in range(2):
            self.retry('prepare_collection')
            self.wait(lambda state:not self.preparation.snapshot()['busy'] and
                self.application.jobs[('fake','collection')]['status']!='running')
            self.assertEqual(self.preparation.snapshot()['status'],'ready')
            self.assertEqual(self.application.reviews.get(('fake','collection')),review)
            self.assertEqual(checkpoint.read_bytes(),before)
            self.assertTrue(parts.playground.state['plan']['validated'])
        self.preparation.notify(self.install)
        self.assertEqual(self.preparation.snapshot()['status'],'ready','repeat install notification must not reset a warm service')
        self.assertGreaterEqual(self.read_install.call_count,2,'each explicit verification still checks the current installation')
        self.assertEqual(self.preparation.asset_checks,2);self.assertTrue(self.preparation.notifications.empty())
        self.assertIs(self.preparation.parts,parts)
        for worker in (parts.detector,parts.planner):worker.close.assert_not_called();worker.start.assert_not_called()
        self.device.submit.assert_not_called();self.device.connect.assert_not_called()

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
            self.device.maintenance=True
            self.preparation._operation_current=lambda:True
            self.preparation._lifecycle_admission=lambda action:True
            with self.assertRaisesRegex(RuntimeError,'CPU stop before conversion'):
                CollectionPreparation._prepare(self.preparation,self.install)
            self.device.maintenance=False
            self.preparation._operation_current=self.preparation._lifecycle_admission=None
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
