"""Live loopback HTTP + real shared Owner. Protocol fake only, no SDK/GPU."""
from pathlib import Path
from copy import deepcopy
from http.client import HTTPConnection
import json
import socket
import sys
import threading
import time
import unittest
from tempfile import TemporaryDirectory
from urllib.request import Request,build_opener,ProxyHandler
from urllib.error import HTTPError
from unittest.mock import Mock
from unittest.mock import patch
from types import SimpleNamespace

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from robot_workbench.service import Application,Device,protocol_fake,serve,loopback_browser_hosts


class HTTPTest(unittest.TestCase):
    def test_recording_file_management_is_confirmed_scoped_and_never_connects_sdk(self):
        from test_inference_history import HistoryTests
        fixture=HistoryTests('runTest');fixture.setUp();self.addCleanup(fixture.doCleanups)
        fixture.capture('saved');fixture.capture('active',done=False)
        history=fixture.history
        self.device.workflows['inference']=SimpleNamespace(recording_files=lambda run_ids=None:
            history.recordings()if run_ids is None else history.delete_recordings(run_ids))
        url=self.url+'/api/inference-recordings?environment=fake&business=inference'
        listing=json.loads(self.http.open(url).read())['recordings']
        self.assertEqual({r['run_id']for r in listing},{'saved','active'})
        with self.assertRaises(HTTPError):self.action('inference_delete_recordings',business='inference',run_ids=['saved'],confirmed=False)
        with self.assertRaises(HTTPError):self.action('inference_delete_recordings',business='calibration',run_ids=['saved'],confirmed=True)
        with self.assertRaises(HTTPError):self.action('inference_delete_recordings',business='inference',run_ids=['saved','active'],confirmed=True)
        code,result=self.action('inference_delete_recordings',business='inference',run_ids=['saved'],confirmed=True)
        self.assertEqual(code,200);self.assertEqual(result['deleted_run_ids'],['saved'])
        self.assertEqual([r['run_id']for r in json.loads(self.http.open(url).read())['recordings']],['active'])
        self.assertIsNone(self.device.runtime)

    def test_inference_page_reuses_its_snapshot_for_action_permissions(self):
        from test_inference_workflow import InferenceWorkflowTests
        fixture=InferenceWorkflowTests('runTest');fixture.setUp()
        self.addCleanup(fixture.doCleanups)
        flow=fixture.workflow
        self.device.workflows['inference']=flow
        with patch.object(flow,'snapshot',wraps=flow.snapshot) as snapshot:
            state=self.state('inference')
        # One workflow view plus the existing execution-settings view. The
        # per-action loop must never copy the growing mode receipt ledger.
        self.assertEqual(snapshot.call_count,2)
        self.assertFalse(state['workflow']['busy'])
        self.assertFalse(state['runtime']['allowed_operations']['execute']['allowed'])

    def test_page_module_dependency_graph_loads_without_connecting_device(self):
        import re
        from urllib.parse import urljoin
        with self.http.open(self.url+'/') as response:
            html=response.read().decode()
        pending=re.findall(r'(?:src|href)="([^"]+\.(?:js|css))"',html)
        seen=set()
        while pending:
            path=pending.pop()
            if path in seen:continue
            seen.add(path)
            with self.http.open(self.url+path) as response:
                raw=response.read()
                self.assertIn(response.headers['Content-Type'],('text/javascript','application/javascript','text/css'))
            relative=path.removeprefix('/web/')
            self.assertEqual(raw,(ROOT/'web'/relative).read_bytes())
            if path.endswith('.js'):
                dependencies=re.findall(r"(?:\bfrom\s*|\bimport\s*\()\s*['\"]([^'\"]+)['\"]",raw.decode())
                pending.extend(urljoin(path,value) for value in dependencies if value.startswith('.'))
        self.assertIn('/web/physical_endpoint.js',seen)
        self.assertIn('/web/operations.js',seen)
        for path in ('/web/../AGENTS.md','/web/unknown.js','/web/vendor/unknown.js'):
            with self.assertRaises(HTTPError) as error:self.http.open(self.url+path)
            self.assertEqual(error.exception.code,404)
        self.assertIsNone(self.device.runtime)

    def test_control_policy_read_commit_conflict_and_retry(self):
        url=lambda business:self.url+'/api/control-policy?environment=fake&business='+business
        with self.assertRaises(HTTPError):self.http.open(url('calibration'))
        self.connect()
        initial=json.loads(self.http.open(url('calibration')).read())
        body=dict(environment='fake',business='calibration',request_id='policy-write',
                  expected_revision=initial['revision'],changes={'measured.limit_rad_s':1.1,'tracking.enabled':False})
        def submit(value):
            request=Request(self.url+'/api/actions/control_policy',json.dumps(value).encode(),headers={
                'Content-Type':'application/json','Origin':self.url,'X-CSRF-Token':self.app.csrf})
            return json.loads(self.http.open(request,timeout=2).read())
        result=submit(body)
        self.assertEqual(submit(body),result)
        updated=json.loads(self.http.open(url('calibration')).read())
        self.assertEqual(updated['revision'],initial['revision']+1)
        self.assertEqual(updated['parameters']['measured']['limit_rad_s'],1.1)
        self.assertNotIn('policy_id',updated)
        with self.assertRaises(HTTPError):submit(dict(body,request_id='stale-policy'))
        with self.assertRaises(HTTPError):self.action('control_policy',business='collection',expected_revision=updated['revision'],changes={'measured.limit_rad_s':1.})
        self.assertFalse(self.state()['runtime']['automatic_enabled'])

    def test_control_policy_storage_does_not_hold_the_stop_http_path(self):
        self.connect();entered=threading.Event();release=threading.Event();results=[]
        runtime=self.device.runtime;original=Path.write_text
        def write(path,*args,**kwargs):
            if path==runtime.execution_settings_path.with_suffix('.tmp'):
                entered.set()
                if not release.wait(3):raise TimeoutError('test storage barrier')
            return original(path,*args,**kwargs)
        def commit():
            try:results.append(self.action('control_policy',expected_revision=0,changes={'arrival.jaw_mm':3.}))
            except Exception as error:results.append(error)
        with patch.object(Path,'write_text',write):
            thread=threading.Thread(target=commit);thread.start()
            try:
                self.assertTrue(entered.wait(1))
                start=time.monotonic();code,_=self.action('stop')
                self.assertEqual(code,200);self.assertLess(time.monotonic()-start,.3)
                self.assertFalse(release.is_set())
            finally:release.set();thread.join(3)
        self.assertFalse(thread.is_alive());self.assertIsInstance(results[0],tuple)

    def test_disconnected_manual_definition_comes_from_model_for_all_businesses(self):
        limits=[[-1.,1.]]*12
        self.device.workflows['calibration']=SimpleNamespace(
            app=SimpleNamespace(geometry=SimpleNamespace(limits=limits)),
            snapshot=lambda:dict(busy=False,status='idle',primary_action=None),
            results=lambda:[],mesh=lambda:dict(requested=False,applied=False))
        self.device.factory=Mock(side_effect=AssertionError('model preview must not connect SDK'))
        for business in ('calibration','collection','inference','research'):
            with self.subTest(business=business):
                view=self.state(business)
                self.assertEqual(view['manual']['limits'],limits)
                self.assertEqual(len(view['manual']['joint_names']),12)
                self.assertIsNone(view['manual']['positions'],'model defaults must never appear as measured feedback')
                self.assertEqual(view['manual']['jaw_gaps_mm'],[None,None])
                self.assertFalse(view['runtime']['allowed_operations']['manual_preview']['allowed'])
                self.assertFalse(view['runtime']['allowed_operations']['manual_execute']['allowed'])
                self.assertFalse(view['runtime']['connected'])
        sdk_limits=[[-.05,3.5]]*12
        self.device.workflows['calibration'].app.manual_joint_limits=lambda:sdk_limits
        self.assertEqual(self.state('research')['manual']['limits'],sdk_limits)
        self.device.factory.assert_not_called()
        self.assertIsNone(self.device.runtime)

    def test_precision_buttons_route_once_without_replacing_business_or_connecting(self):
        study=SimpleNamespace(snapshot=lambda runtime,business:dict(configured=True,reason=None,
            execute_allowed=True,latest={'status':'completed'}),
            prepare=Mock(return_value={'message':'prepared only'}),execute=Mock(return_value={'message':'submitted'}),
            analyze=Mock(return_value={'message':'analyzed'}))
        self.device.precision_study=study
        # Numeric analysis is useful while disconnected; it must never acquire a device.
        self.action('precision_analyze',business='research')
        self.wait_precision_request()
        study.analyze.assert_called_once();self.assertIsNone(self.device.runtime)
        self.connect();self.action('handoff',target_business='research')
        before=self.state('research')['workflow']['stage_id']
        result=self.action('precision_preview',business='research',kind='trajectory',speed=.5)
        self.wait_precision_request()
        self.assertEqual(self.state('research')['workflow']['stage_id'],before)
        study.prepare.assert_called_once_with('research','trajectory',.5,None)
        study.prepare.reset_mock()
        pin=dict(path='/owned/ff.json',sha256='a'*64,size_bytes=123)
        self.action('precision_preview',business='research',kind='trajectory',speed=.5,feedforward_artifact=pin)
        self.wait_precision_request()
        study.prepare.assert_called_once_with('research','trajectory',.5,pin)
        with self.assertRaises(HTTPError):self.action('precision_preview',business='research',kind='trajectory',speed=.5,path='/arbitrary')
        self.assertFalse(self.device.snapshot('research').get('automatic_enabled'))

    def wait_precision_request(self):
        deadline=time.monotonic()+3
        while time.monotonic()<deadline:
            value=self.state('research')
            if(value['precision_study'].get('request')or{}).get('status')=='completed':return value
            time.sleep(.01)
        self.fail('research request did not complete')

    def test_shared_manual_tool_uses_current_business_and_preserves_business_task(self):
        self.connect();self.calibration_adapter()
        q=[0.01]+[0.0]*11
        for business in ('collection','inference','research'):
            with self.subTest(business=business):
                holder=self.device.runtime.holder
                deadline=time.monotonic()+3
                while not self.state(holder)['runtime']['allowed_operations']['handoff']['allowed'] and time.monotonic()<deadline:
                    time.sleep(.01)
                self.action('handoff',business=holder,target_business=business)
                self.wait(lambda value:value['runtime']['holder']==business)
                self.assertTrue(self.state(business)['runtime']['allowed_operations']['manual_preview']['allowed'])
                self.assertFalse(self.state(business)['runtime']['allowed_operations']['manual_queue']['allowed'])
                self.action('manual_preview',business=business,q12=q,speeds12=[.1]*12,jaw_gap_mm=[80,80],jaw_speed_mm_s=10)
                deadline=time.monotonic()+3
                while time.monotonic()<deadline:
                    view=self.state(business)
                    if (view['workflow'].get('request') or {}).get('status')!='running':break
                    time.sleep(.01)
                self.assertEqual(self.app.jobs[('fake',business)]['status'],'completed',self.app.jobs[('fake',business)])
                self.assertEqual(view['manual']['execution']['stage_id'],'manual.review')
                self.assertFalse(view['workflow']['business_task']['stage_id'].startswith('manual.'))
                self.assertEqual(self.device.runtime.holder,business)
                self.action('manual_execute',business=business,review_id=view['workflow']['review']['id'],physical_clearance=True)
                deadline=time.monotonic()+3
                while time.monotonic()<deadline:
                    done=self.state(business)
                    if done['manual'].get('execution',{}).get('status')=='completed':break
                    time.sleep(.01)
                self.assertEqual(done['manual']['execution']['status'],'completed',done['workflow'])
                self.assertFalse(done['workflow']['business_task']['stage_id'].startswith('manual.'))

    def test_inference_history_pages_and_old_media_are_readonly_without_device_or_model(self):
        from robot_workbench.inference_history import InferenceHistory, PAGE_RUNS
        root=Path(self.temp.name)/'infer-history';history=InferenceHistory(root)
        flow,calls,_,_=self.infer_run_fixture()
        flow.results=Mock(side_effect=AssertionError('never enumerate the full history'))
        flow.results_page=history.page;flow.media=history.media
        history.append((dict(id=f'video-{i}',run_id=f'run-{i}',kind='recording'),root/f'{i}.mp4') for i in range(40))
        video=root/'0.mp4';video.write_bytes(b'old playback bytes')
        before=history.path.read_bytes()
        latest=self.state('inference')
        self.assertEqual(latest['results_page']['total_results'],40)
        self.assertEqual(len(latest['results']),PAGE_RUNS)
        base=self.url+'/api/state?environment=fake&business=inference'
        with self.http.open(base+'&results_before='+str(latest['results_page']['older_cursor'])) as response:
            older=json.load(response)
        self.assertTrue(set(row['id'] for row in latest['results']).isdisjoint(row['id'] for row in older['results']))
        with self.http.open(base+'&results_after='+str(older['results_page']['newer_cursor'])) as response:
            self.assertEqual(json.load(response)['results'],latest['results'])
        with self.http.open(self.url+'/api/result-media?environment=fake&business=inference&result_id=video-0') as response:
            self.assertEqual(response.read(),b'old playback bytes')
        for suffix in ('results_before=0','results_before=-1','results_before=x','results_after=9223372036854775808',
                       'results_before=2&results_after=1','results_before=1&results_before=2'):
            with self.subTest(suffix=suffix),self.assertRaises(HTTPError):self.http.open(base+'&'+suffix)
        with self.assertRaises(HTTPError):self.http.open(base.replace('inference','calibration')+'&results_before=1')
        self.assertEqual(history.path.read_bytes(),before)
        self.assertIsNone(self.device.runtime);self.assertEqual(calls,[])

    def test_quality_download_is_readonly_and_bound_to_the_selected_recording(self):
        report=dict(schema='recording_quality.v1',raw_manifest_sha256='a'*64,cameras={},raw_modified=False)
        self.device.collection_history=SimpleNamespace(quality_report=lambda identity:dict(report=report,sidecar_sha256='b'*64) if identity=='known' else None)
        base=self.url+'/api/recording-quality?environment=fake&business=collection&result_id='
        with self.http.open(base+'known') as response:
            value=json.load(response)
            self.assertEqual(value['raw_manifest_sha256'],report['raw_manifest_sha256'])
            self.assertEqual(value['download_identity']['result_id'],'known')
            self.assertEqual(value['download_identity']['sidecar_sha256'],'b'*64)
            self.assertIn('attachment',response.headers['Content-Disposition'])
        for url in (base+'missing',base.replace('collection','calibration')+'known'):
            with self.assertRaises(HTTPError):self.http.open(url)
        self.assertIsNone(self.device.runtime)

    def test_calibration_can_set_inference_length_without_connecting_or_running(self):
        flow,calls,_,_=self.infer_run_fixture()
        settings=dict(execute_steps=30,revision=0,allowed=True,reason=None)
        flow.execution_settings=lambda:dict(settings)
        def update(value,revision):
            if revision!=settings['revision']:raise ValueError('revision changed')
            settings.update(execute_steps=value,revision=revision+1);flow.invalidate();return dict(settings)
        flow.set_execution_steps=update
        view=self.state('calibration')
        self.assertTrue(view['runtime']['allowed_operations']['inference_execution_settings']['allowed'])
        self.assertEqual(view['inference_execution']['execute_steps'],30)
        _,result=self.action('inference_execution_settings',business='calibration',execute_steps=45,expected_revision=0)
        self.assertEqual(result['inference_execution']['execute_steps'],45)
        self.assertIsNone(self.device.runtime);self.assertEqual(calls,[])
        self.assertEqual(self.state('inference')['inference_execution']['execute_steps'],45)
        with self.assertRaises(HTTPError):self.action('inference_execution_settings',business='calibration',execute_steps=30,expected_revision=0)

    def test_inference_state_reuses_calibration_snapshot_and_holder_snapshot(self):
        self.infer_run_fixture();self.connect()
        calibration=SimpleNamespace(snapshot=Mock(return_value=dict(busy=False,arrival_statistics={'count':3})),results=lambda:[])
        self.device.workflows['calibration']=calibration
        with patch.object(self.device,'snapshot',wraps=self.device.snapshot) as snapshots:
            state=self.app.state('fake','inference')
        self.assertEqual(calibration.snapshot.call_count,1)
        self.assertEqual(snapshots.call_count,2)
        self.assertEqual(state['arrival_statistics'],{'count':3})

    def test_run_remains_eligible_and_waits_for_normal_read_before_handoff(self):
        _,calls,_,_=self.infer_run_fixture();self.connect()
        entered=threading.Event();release=threading.Event();self.addCleanup(release.set)
        backend=self.device.runtime.owner.backend._backend;original=backend.read_calibration_state
        def read():entered.set();release.wait(1);return original()
        with patch.object(backend,'read_calibration_state',side_effect=read):
            self.assertTrue(entered.wait(1))
            state=self.state('inference')
            self.assertTrue(state['runtime']['idle_readiness']['control_cycle_in_flight'])
            self.assertTrue(state['runtime']['allowed_operations']['inference_run']['allowed'])
            body=self.infer_run_body()
            self.action('inference_run',business='inference',**body)
            deadline=time.monotonic()+.06
            while self.app.jobs[('fake','inference')].get('stage')!='handoff' and time.monotonic()<deadline:time.sleep(.001)
            self.assertEqual(self.app.jobs[('fake','inference')].get('stage'),'handoff')
            self.assertEqual(self.device.runtime.holder,'calibration');release.set()
            self.wait_infer_job('completed')
        self.assertEqual(calls,['bind','preview','execute'])

    def infer_run_fixture(self, pause=None):
        calls=[];entered=threading.Event();release=threading.Event();self.addCleanup(release.set)
        device=self.device
        class Workflow:
            generation=0
            model_id='base566-epoch007-step58982'
            digest='a'*64
            def snapshot(self):return dict(status='ready',busy=False,outcomes={},primary_action=None,
                inference=dict(selected_model_id=self.model_id,bundle_digest=self.digest,model_ready=True,
                    model=dict(id=self.model_id,loaded=True,bundle_digest=self.digest)))
            def results(self):return []
            def allowed(self,action):return dict(allowed=True,reason=None)
            def invalidate(self):self.generation+=1
            cancel=invalidate
            def refresh_models(self):calls.append('refresh');return {'models':[]}
            def select_model(self,model_id,expected_bundle_digest=None):
                if expected_bundle_digest!=self.digest:raise ValueError('digest changed')
                self.generation+=1;calls.append('select');return {'model_ready':True}
            def prepare_model(self,**_):calls.append('prepare');return {'model_ready':True}
            def step(self,name):
                calls.append(name)
                if pause==name:entered.set();release.wait(2)
            def bind_device(self):
                if device.runtime.holder!='inference':raise ValueError('not inference holder')
                self.generation+=1;generation=self.generation
                self.step('bind')
                return dict(generation=generation)
            def preview_episode(self,seed=0,duration_s=40):
                self.step('preview');self.last_review=dict(token='cpu-episode-token',seed=seed,duration_s=duration_s)
                return {'preview':dict(self.last_review)}
            def execute_review(self,token,physical_clearance=True):
                if token!='cpu-episode-token' or physical_clearance is not True:raise ValueError('invalid confirmation')
                self.step('execute');return {'accepted':True}
        flow=Workflow();self.device.workflows['inference']=flow
        return flow,calls,entered,release

    def infer_run_body(self):
        state=self.state('inference');info=state['workflow']['inference']
        return dict(seed=7,duration_s=40,model_id=info['selected_model_id'],expected_bundle_digest=info['bundle_digest'],
            expected_context=info['run_context'],state_wall_ns=state['server_wall_ns'],
            scene_reset=True,physical_clearance=True,arms_supported=True)

    def wait_infer_job(self,status=None):
        deadline=time.monotonic()+3
        while time.monotonic()<deadline:
            job=self.app.jobs.get(('fake','inference'),{})
            if job.get('status') not in (None,'running'):
                if status:self.assertEqual(job['status'],status,job)
                return job
            time.sleep(.01)
        self.fail('inference job did not finish: '+str(job))

    def test_inference_model_selection_does_not_connect_sdk(self):
        flow,calls,_,_=self.infer_run_fixture()
        for action,extra in (('inference_refresh',{}),('inference_select',dict(model_id=flow.model_id,expected_bundle_digest=flow.digest)),('inference_prepare',{})):
            self.assertTrue(self.state('inference')['runtime']['allowed_operations'][action]['allowed'])
            self.action(action,business='inference',**extra);self.wait_infer_job('completed')
            self.assertIsNone(self.device.runtime)
        self.assertEqual(calls,['refresh','select','prepare'])

    def test_one_run_hands_off_existing_connection_and_executes_once_for_one_confirmation(self):
        flow,calls,_,_=self.infer_run_fixture();self.connect();body=self.infer_run_body()
        data=dict(environment='fake',business='inference',request_id='one-reviewed-run',**body)
        def request():
            req=Request(self.url+'/api/actions/inference_run',json.dumps(data).encode(),headers={
                'Content-Type':'application/json','Origin':self.url,'X-CSRF-Token':self.app.csrf})
            with self.http.open(req,timeout=2) as response:return json.loads(response.read())
        first=request();self.assertEqual(request(),first)
        job=self.wait_infer_job('completed');self.assertEqual(request(),first)
        self.assertEqual(flow.last_review['duration_s'],40);self.assertEqual(job['result']['duration_s'],40)
        self.assertEqual(self.device.runtime.holder,'inference');self.assertEqual(calls,['bind','preview','execute'])
        self.assertFalse(self.device.snapshot('inference').get('motion'),'fixture checks orchestration without sending motion')

    def test_two_second_run_is_bound_to_review_and_idempotent(self):
        flow,calls,_,_=self.infer_run_fixture();self.connect();body=dict(self.infer_run_body(),duration_s=2)
        data=dict(environment='fake',business='inference',request_id='two-second-reviewed-run',**body)
        def request(value):
            req=Request(self.url+'/api/actions/inference_run',json.dumps(value).encode(),headers={
                'Content-Type':'application/json','Origin':self.url,'X-CSRF-Token':self.app.csrf})
            with self.http.open(req,timeout=2) as response:return json.loads(response.read())
        first=request(data);self.assertEqual(request(data),first)
        job=self.wait_infer_job('completed');self.assertEqual(request(data),first)
        self.assertEqual(calls,['bind','preview','execute']);self.assertEqual(flow.last_review['duration_s'],2)
        self.assertEqual(flow.last_review['seed'],7);self.assertEqual(job['result']['duration_s'],2)
        with self.assertRaises(HTTPError):request(dict(data,duration_s=40))
        self.assertEqual(calls.count('execute'),1)

    def test_run_default_duration_is_120_and_arbitrary_positive_durations_are_accepted(self):
        flow,calls,_,_=self.infer_run_fixture();body=self.infer_run_body()
        for duration in (0,-1,True,'1',float('nan'),float('inf')):
            with self.assertRaises(HTTPError):self.action('inference_run',business='inference',**dict(body,duration_s=duration))
        self.assertIsNone(self.device.runtime);self.assertFalse(calls)
        self.connect();body=self.infer_run_body();body.pop('duration_s');self.action('inference_run',business='inference',**body)
        job=self.wait_infer_job('completed');self.assertEqual(flow.last_review['duration_s'],120)
        self.assertEqual(job['result']['duration_s'],120)
        for duration in (5,5.125,39):
            self.action('inference_run',business='inference',**dict(self.infer_run_body(),duration_s=duration))
            job=self.wait_infer_job('completed');self.assertEqual(job['result']['duration_s'],duration)

    def test_one_second_run_rejects_a_wrong_duration_review_before_confirmation(self):
        flow,calls,_,_=self.infer_run_fixture();self.connect();original=flow.preview_episode
        def wrong(seed=0,duration_s=40):return original(seed,duration_s=40)
        flow.preview_episode=wrong
        self.action('inference_run',business='inference',**dict(self.infer_run_body(),duration_s=1))
        job=self.wait_infer_job('failed');self.assertIn('时长',job['error']);self.assertEqual(calls,['bind','preview'])

    def test_stop_during_one_second_review_still_revokes_the_run(self):
        _,calls,entered,release=self.infer_run_fixture('preview');self.connect()
        self.action('inference_run',business='inference',**dict(self.infer_run_body(),duration_s=1))
        self.assertTrue(entered.wait(1));self.action('stop',business='inference');release.set()
        self.wait_infer_job('cancelled');self.assertEqual(calls,['bind','preview'])

    def test_one_run_rejects_stale_context_missing_consent_busy_other_business_and_fault(self):
        _,calls,_,_=self.infer_run_fixture();body=self.infer_run_body()
        for change in ({'physical_clearance':False},{'scene_reset':False},{'expected_bundle_digest':'b'*64},
                       {'state_wall_ns':0},{'expected_context':{}},{'duration_s':-41}):
            with self.assertRaises(HTTPError):self.action('inference_run',business='inference',**dict(body,**change))
        self.assertIsNone(self.device.runtime);self.assertFalse(calls)
        self.connect()
        self.device.workflows['collection']=SimpleNamespace(snapshot=lambda:dict(busy=True),results=lambda:[])
        with self.assertRaises(HTTPError):self.action('inference_run',business='inference',**self.infer_run_body())
        del self.device.workflows['collection']
        original=self.device.snapshot
        with patch.object(self.device,'snapshot',side_effect=lambda business:dict(original(business),state='FAULT')):
            with self.assertRaises(HTTPError):self.action('inference_run',business='inference',**self.infer_run_body())
        self.assertEqual(self.device.runtime.holder,'calibration');self.assertFalse(calls)

    def test_stop_during_run_binding_prevents_late_preview_or_execution(self):
        _,calls,entered,release=self.infer_run_fixture('bind');self.connect()
        self.action('inference_run',business='inference',**self.infer_run_body());self.assertTrue(entered.wait(1))
        start=time.monotonic();self.action('stop',business='inference');self.assertLess(time.monotonic()-start,.2)
        release.set();self.wait_infer_job('cancelled');self.assertEqual(calls,['bind'])

    def test_operator_hold_during_review_cancels_preparation_without_motion_or_release(self):
        flow,calls,entered,release=self.infer_run_fixture('preview');self.connect()
        flow.request_operator_stop=lambda request_id:dict(accepted=True,no_op=True)
        self.action('inference_run',business='inference',**self.infer_run_body());self.assertTrue(entered.wait(1))
        state=self.device.snapshot('inference');nonce=self.device.runtime.nonce
        self.action('inference_hold',business='inference',runtime_nonce=nonce,epoch=state['command_epoch'])
        release.set();self.wait_infer_job('completed')
        self.assertEqual(calls,['bind','preview'])
        self.assertTrue(self.app.jobs[('fake','inference')]['result']['operator_stopped'])
        self.assertEqual(self.device.runtime.nonce,nonce)
        self.assertTrue(self.device.snapshot('inference')['connected'])

    def test_disconnected_run_reports_operations_gap_without_calling_factory(self):
        _,calls,_,_=self.infer_run_fixture()
        self.device.factory=Mock(side_effect=AssertionError('Run must not connect control'))
        with self.assertRaises(HTTPError) as rejected:
            self.action('inference_run',business='inference',**self.infer_run_body())
        self.assertIn('运维',json.loads(rejected.exception.read())['error'])
        self.device.factory.assert_not_called();self.assertFalse(calls);self.assertIsNone(self.device.runtime)

    def test_physical_run_never_auto_connects_even_with_support_confirmation(self):
        flow,_,_,_=self.infer_run_fixture()
        physical=self.app.devices['physical'];physical.factory=Mock(side_effect=AssertionError('must not open SDK'))
        physical.workflows['inference']=flow
        state=self.app.state('physical','inference');info=state['workflow']['inference']
        body=dict(environment='physical',business='inference',request_id='physical-consent',seed=7,duration_s=40,
            model_id=flow.model_id,expected_bundle_digest=flow.digest,expected_context=info['run_context'],
            state_wall_ns=state['server_wall_ns'],scene_reset=True,physical_clearance=True,arms_supported=False)
        for supported in (False,True):
            with self.assertRaisesRegex(ValueError,'运维'):self.app.action('inference_run',dict(body,arms_supported=supported))
        physical.factory.assert_not_called()

    def test_cancel_and_model_generation_change_revoke_pending_run_review(self):
        flow,calls,entered,release=self.infer_run_fixture('preview');self.connect()
        self.action('inference_run',business='inference',**self.infer_run_body());self.assertTrue(entered.wait(1))
        self.action('cancel',business='inference');flow.invalidate();release.set()
        self.wait_infer_job('cancelled');self.assertEqual(calls,['bind','preview'])

    def test_inference_uses_shared_holder_and_once_per_round_review(self):
        self.connect();calls=[]
        flow=SimpleNamespace(generation=0,results=lambda:[],allowed=lambda action:dict(allowed=action in ('inference_prepare','inference_preview','execute','cancel'),reason=None))
        flow.snapshot=lambda:dict(status='idle',busy=False,stage_id='inference.idle',outcomes={},
            primary_action=dict(id='inference_preview',allowed=True,label='准备本轮推理'),inference=dict(duration_s=40,seed=7,model=dict(id='base379-010-29450',loaded=True)))
        flow.invalidate=lambda:setattr(flow,'generation',flow.generation+1)
        flow.cancel=lambda:None
        def perform(action,body):
            calls.append((action,deepcopy(body)))
            if action=='inference_preview':
                owner=self.device.snapshot('inference')
                return dict(preview=dict(token='round-token',owner_pid=owner['pid'],runtime_nonce=owner['runtime_nonce'],
                    command_epoch=owner['command_epoch'],workflow_generation=flow.generation,seed=body['seed'],duration_s=40),summary='40秒；仅记录目标审计；不检查碰撞')
            return dict(message='推理请求已受理')
        flow.action=perform;self.device.workflows['inference']=flow
        self.assertFalse(self.state('inference')['runtime']['allowed_operations']['inference_preview']['allowed'])
        with self.assertRaises(HTTPError):self.action('inference_preview',business='inference',seed=7,duration_s=40)
        self.action('handoff',target_business='inference')
        self.assertEqual(self.device.runtime.holder,'inference')
        def finish():
            deadline=time.monotonic()+2
            while self.app.jobs.get(('fake','inference'),{}).get('status')=='running' and time.monotonic()<deadline:time.sleep(.01)
            self.assertEqual(self.app.jobs[('fake','inference')]['status'],'completed')
        self.action('inference_prepare',business='inference',seed=7,duration_s=40);finish()
        for action,extra in [('inference_prepare',dict(reference_feedforward_sha256='a'*64)),
                             ('inference_preview',dict(torque_bias_sdk6=[[0.]*6]))]:
            with self.assertRaises(HTTPError):self.action(action,business='inference',seed=7,duration_s=40,**extra)
        self.action('inference_preview',business='inference',seed=7,duration_s=40,reference_feedforward_sha256='a'*64);finish()
        self.assertEqual([name for name,_ in calls],['inference_prepare','inference_preview'])
        self.assertEqual(calls[-1][1]['reference_feedforward_sha256'],'a'*64)
        review=self.state('inference')['workflow']['review'];self.assertEqual(review['seed'],7);self.assertEqual(review['duration_s'],40)
        with self.assertRaises(HTTPError):self.action('execute',business='inference',review_id=review['id'],seed=8,duration_s=40,physical_clearance=True)
        with self.assertRaises(HTTPError):self.action('execute',business='inference',review_id=review['id'],seed=7,duration_s=40)
        self.action('execute',business='inference',review_id=review['id'],seed=7,duration_s=40,physical_clearance=True);finish()
        self.assertEqual(calls[-1][0],'execute');self.assertEqual(calls[-1][1]['token'],'round-token')
        with self.assertRaises(HTTPError):self.action('execute',business='inference',review_id=review['id'],seed=7,duration_s=40,physical_clearance=True)
        self.assertEqual(sum(name=='execute' for name,_ in calls),1)
        self.action('inference_preview',business='inference',seed=7,duration_s=40);finish()
        self.action('handoff',target_business='collection')
        self.assertNotIn(('fake','inference'),self.app.reviews)
        self.assertEqual(self.device.runtime.holder,'collection')
        for body in (dict(model_path='/tmp/model'),dict(duration_s=-41),dict(seed=-1),dict(seed=True)):
            with self.assertRaises(HTTPError):self.action('inference_prepare',business='inference',**body)

    def test_inference_history_shares_media_route_without_connecting(self):
        path=Path(self.temp.name)/'infer.mp4';path.write_bytes(b'0123456789')
        self.device.workflows['inference']=SimpleNamespace(snapshot=lambda:dict(status='idle',busy=False,outcomes={}),
            results=lambda:[dict(id='infer-video',kind='recording',status='valid',run_id='inference-1')],media=lambda ident:path if ident=='infer-video' else None)
        state=self.state('inference');self.assertFalse(state['runtime']['connected']);self.assertEqual(state['results'][0]['id'],'infer-video')
        url=self.url+'/api/result-media?environment=fake&business=inference&result_id=infer-video'
        with self.http.open(Request(url,headers={'Range':'bytes=2-5'})) as response:
            self.assertEqual(response.status,206);self.assertEqual(response.read(),b'2345')
        with self.assertRaises(HTTPError):self.http.open(url.replace('business=inference','business=collection'))
        self.assertIsNone(self.device.runtime)

    def test_execution_settings_are_scoped_idle_only_and_revoke_reviews(self):
        with self.assertRaises(HTTPError):self.action('execution_settings',settings={},expected_revision=0)
        self.connect()
        before=self.state()['runtime']['execution_settings'];settings=deepcopy(before['settings'])
        self.assertEqual(settings['speed_protection'],dict(enabled=True,limit_rad_s=2.5))
        legacy=deepcopy(settings);legacy.pop('speed_protection')
        with self.assertRaises(HTTPError):self.action('execution_settings',settings=legacy,expected_revision=before['revision'])
        settings['speed_protection']['limit_rad_s']=.9
        settings.update(arrival_enabled=False,tracking_enabled=False)
        settings['arrival_tolerances_rad']=[.01+i*.001 for i in range(12)]
        self.app.reviews[('fake','calibration')]={'id':'obsolete-review'}
        code,result=self.action('execution_settings',settings=settings,expected_revision=before['revision'])
        self.assertEqual(code,200);self.assertNotIn(('fake','calibration'),self.app.reviews)
        applied=self.state()['runtime']['execution_settings']
        self.assertEqual(applied['settings'],settings);self.assertEqual(applied['revision'],before['revision']+1)
        self.assertEqual(result['execution_settings']['revision'],applied['revision'])
        self.assertFalse(self.device.snapshot('calibration').get('motion'),'editing settings must not start motion')
        with self.assertRaises(HTTPError):self.action('execution_settings',settings=settings,expected_revision=before['revision'])
        with self.assertRaises(HTTPError):self.action('execution_settings',business='collection',settings=settings,expected_revision=applied['revision'])
        self.device.workflows['calibration']=SimpleNamespace(snapshot=lambda:dict(busy=True),results=lambda:[])
        self.assertFalse(self.state()['runtime']['allowed_operations']['execution_settings']['allowed'])
        with self.assertRaises(HTTPError):self.action('execution_settings',settings=settings,expected_revision=applied['revision'])
        self.device.workflows.clear()
        self.assertEqual(self.device.runtime.execution_settings_snapshot('calibration'),applied)

    def test_fake_scene_readback_is_diagnostic_only_and_requires_namespace(self):
        scene=dict(cup_position_m=[.4,.2,.78],source_wall_ns=123)
        getter=Mock(return_value=scene);self.device.fake_scene_snapshot=getter
        url=self.url+'/api/fake-scene?environment=fake&business=collection'
        self.assertEqual(json.loads(self.http.open(url).read()),scene)
        getter.assert_called_once_with();self.assertIsNone(self.device.runtime)
        for invalid in (url.replace('fake&','physical&'),url.replace('collection','calibration'),self.url+'/api/fake-scene'):
            with self.assertRaises(HTTPError) as error:self.http.open(invalid)
            self.assertEqual(error.exception.code,409)
        getter.assert_called_once_with()
        del self.device.fake_scene_snapshot
        with self.assertRaises(HTTPError) as error:self.http.open(url)
        self.assertEqual(error.exception.code,409)

    def test_cup_return_requires_new_review_and_never_starts_during_generation(self):
        self.connect();self.action('handoff',target_business='collection')
        owner=self.device.snapshot('collection');sent=[]
        self.device.cup_return_inputs={'fixture':'parent-bound'}
        self.device.collection_preparation=SimpleNamespace(snapshot=lambda:dict(status='ready',busy=False,can_retry=True,retry_reason=None,installed={'verified':True}))
        self.device.workflows['collection']=SimpleNamespace(snapshot=lambda:dict(status='failed',busy=False,outcomes={},collection={}),
            results=lambda:[],allowed=lambda action:dict(allowed=True,reason=None))
        def prepare(device,offset_xy_mm=None):
            device.cup_return_task=dict(directory='return-job',timeline_digest='return-timeline',duration_s=37.57,
                parent_plan_id='parent-plan',parent_speed_scale=.97,submitted=False,
                runtime_nonce=owner['runtime_nonce'],command_epoch=owner['command_epoch'])
            return dict(preview=dict(token='one-reviewed-return',owner_pid=owner['pid'],command_epoch=owner['command_epoch'],
                runtime_nonce=owner['runtime_nonce'],purpose='cup_return',speed_scale=.97),summary='取回杯子；无新碰撞检查')
        with patch('robot_workbench.cup_return.prepare',side_effect=prepare), \
                patch.object(self.device,'submit',side_effect=lambda business,command:sent.append(command) or {'queued':'confirm'}):
            self.action('return_cup',business='collection')
            deadline=time.monotonic()+2
            while time.monotonic()<deadline:
                view=self.state('collection')['workflow']
                if view.get('review'):break
                time.sleep(.01)
            self.assertEqual(sent,[])
            review=view['review'];self.assertEqual(review['speed_scale'],.97)
            with self.assertRaises(HTTPError):self.action('execute',business='collection',review_id=review['id'],speed_scale=.97)
            self.action('execute',business='collection',review_id=review['id'],speed_scale=.97,physical_clearance=True)
            self.assertEqual(len(sent),1);self.assertEqual(sent[0]['token'],'one-reviewed-return')
            self.assertTrue(self.device.cup_return_task['submitted'])
            with self.assertRaises(HTTPError):self.action('execute',business='collection',review_id=review['id'],speed_scale=.97,physical_clearance=True)
            self.assertEqual(len(sent),1)

    def test_readonly_history_is_available_before_collection_backend_or_connection(self):
        path=Path(self.temp.name)/'history.mp4';path.write_bytes(b'0123456789')
        row=dict(id='history-video',kind='recording',label='历史任务',media=dict(
            id='history-video',kind='replay',run_id='old-job',role='source',camera='base_0',ready=True))
        self.device.collection_history=SimpleNamespace(results=lambda:[row],media=lambda ident:path if ident=='history-video' else None)
        state=self.state('collection')
        self.assertEqual(state['results'],[row]);self.assertFalse(state['runtime']['connected'])
        self.assertEqual(self.state('calibration')['results'],[])
        url=self.url+'/api/result-media?environment=fake&business=collection&result_id=history-video'
        with self.http.open(Request(url,headers={'Range':'bytes=2-5'})) as response:
            self.assertEqual(response.status,206);self.assertEqual(response.read(),b'2345')
        for wrong in (url.replace('collection','calibration'),url.replace('history-video','unknown')):
            with self.assertRaises(HTTPError) as error:self.http.open(wrong)
            self.assertEqual(error.exception.code,404)
        with self.assertRaises(HTTPError):self.action('execute',business='collection',result_id='history-video')
        self.assertIsNone(self.device.runtime,'viewing history cannot connect or restore an execution')

    def setUp(self):
        self.temp=TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.device=protocol_fake(Path(self.temp.name))
        self.app=Application({'fake':self.device,'physical':Device('robot','physical',None)})
        self.server=serve(self.app,port=0)
        self.thread=threading.Thread(target=self.server.serve_forever,daemon=True);self.thread.start()
        self.addCleanup(self.cleanup)
        self.url='http://127.0.0.1:'+str(self.server.server_port)
        self.http=build_opener(ProxyHandler({}))
        self.counter=0

    def cleanup(self):
        if self.device.runtime:
            runtime=self.device.runtime
            runtime.owner.submit({'action':'release','confirmation':'ARMS_SUPPORTED_RELEASE'})
            self.device.thread.join(1)
            runtime.close()
        self.device.closed.set()
        self.server.shutdown();self.server.server_close();self.thread.join(1)

    def state(self,business='calibration'):
        return json.loads(self.http.open(self.url+'/api/state?environment=fake&business='+business).read())

    def action(self,action,business='calibration',**body):
        self.counter+=1
        data=dict(environment='fake',business=business,request_id=str(self.counter),**body)
        req=Request(self.url+'/api/actions/'+action,json.dumps(data).encode(),headers={
            'Content-Type':'application/json','Origin':self.url,'X-CSRF-Token':self.app.csrf})
        with self.http.open(req,timeout=2) as response:return response.status,json.loads(response.read())

    def wait(self,predicate):
        deadline=time.monotonic()+3
        while time.monotonic()<deadline:
            state=self.state()
            if predicate(state):return state
            time.sleep(.01)
        self.fail('state did not converge: '+str(state))

    def connect(self):
        code,_=self.action('connect');self.assertEqual(code,202)
        return self.wait(lambda s:s['runtime'].get('stationary'))

    def test_request_failure_preserves_running_task_and_actual_task_failure(self):
        self.connect();self.action('handoff',target_business='collection')
        value=dict(id='current-D-run',status='running',busy=True,stage_id='collection.prepare',
            message='当前环境准备仍在运行',primary_action=dict(id='generate_plan',allowed=False),outcomes={})
        workflow=SimpleNamespace(snapshot=lambda:dict(value),results=lambda:[],
            action=Mock(side_effect=ValueError('重复请求被拒绝')))
        self.device.workflows['collection']=workflow
        self.action('generate_plan',business='collection')
        end=time.monotonic()+2
        while time.monotonic()<end:
            state=self.state('collection')['workflow']
            if state.get('request',{}).get('status')=='failed':break
            time.sleep(.01)
        self.assertEqual(state['request']['error'],'重复请求被拒绝')
        self.assertEqual(state['status'],'running');self.assertTrue(state['busy'])
        self.assertEqual(state['id'],'current-D-run');self.assertEqual(state['message'],value['message'])
        value.update(status='failed',busy=False,message='实际任务失败：场景检测错误')
        state=self.state('collection')['workflow']
        self.assertEqual(state['status'],'failed');self.assertEqual(state['message'],value['message'])
        self.assertEqual(state['request']['error'],'重复请求被拒绝')

    def test_collection_review_generation_blocks_late_publication_and_execution(self):
        self.connect();self.action('handoff',target_business='collection')
        workflow=SimpleNamespace(generation=2,snapshot=lambda:dict(status='ready',busy=False,primary_action=None),results=lambda:[],action=Mock())
        self.device.workflows['collection']=workflow
        owner=self.device.snapshot('collection')
        preview=dict(token='frozen-plan',command_epoch=owner['command_epoch'],workflow_generation=1)
        self.app.add_review(self.device,'collection',preview,'execute','old snapshot')
        self.assertNotIn(('fake','collection'),self.app.reviews)
        preview['workflow_generation']=2
        self.app.add_review(self.device,'collection',preview,'execute','current snapshot')
        review=self.state('collection')['workflow']['review']
        workflow.generation=3
        self.assertNotIn('review',self.state('collection')['workflow'])
        with self.assertRaises(HTTPError) as rejected:
            self.action('execute',business='collection',review_id=review['id'],physical_clearance=True)
        self.assertIn('数采任务已变化',json.loads(rejected.exception.read())['error'])
        workflow.action.assert_not_called()

    def test_collection_execution_speed_is_server_bound_to_the_review(self):
        self.connect();self.action('handoff',target_business='collection')
        workflow=SimpleNamespace(generation=2,snapshot=lambda:dict(status='ready',busy=False,primary_action=None),
            results=lambda:[],action=Mock(return_value={'message':'CPU route fixture accepted'}))
        self.device.workflows['collection']=workflow;owner=self.device.snapshot('collection')
        self.app.add_review(self.device,'collection',dict(token='frozen-plan',command_epoch=owner['command_epoch'],
            workflow_generation=2,speed_scale=1.),'execute','执行速度 100%')
        review=self.state('collection')['workflow']['review'];self.assertEqual(review['speed_scale'],1.)
        for invalid in (.5,True,'1',None):
            with self.assertRaises(HTTPError) as error:
                self.action('execute',business='collection',review_id=review['id'],physical_clearance=True,speed_scale=invalid)
            self.assertIn('速度已变化',json.loads(error.exception.read())['error'])
        workflow.action.assert_not_called()
        self.action('execute',business='collection',review_id=review['id'],physical_clearance=True)
        self.wait(lambda _:workflow.action.called)
        self.assertEqual(workflow.action.call_args.args[1]['speed_scale'],1.)
        self.assertEqual(workflow.action.call_args.args[1]['token'],'frozen-plan')

    def test_declared_browser_port_preserves_Host_Origin_and_CSRF_checks(self):
        self.assertEqual(self.server.allowed_hosts,{f'127.0.0.1:{self.server.server_port}',f'localhost:{self.server.server_port}'})
        self.server.shutdown();self.server.server_close();self.thread.join(1)
        self.server=serve(self.app,port=0,browser_ports=[18867])
        self.thread=threading.Thread(target=self.server.serve_forever,daemon=True);self.thread.start()
        self.url='http://127.0.0.1:'+str(self.server.server_port)
        for host in ('127.0.0.1:18867','localhost:18867',f'127.0.0.1:{self.server.server_port}'):
            with self.http.open(Request(self.url+'/api/state',headers={'Host':host}),timeout=2) as response:
                state=json.loads(response.read());self.assertEqual(response.status,200)
            body=json.dumps(dict(environment='fake',business='calibration',request_id=host)).encode()
            request=Request(self.url+'/api/actions/stop',body,headers={'Content-Type':'application/json',
                'Host':host,'Origin':'http://'+host,'X-CSRF-Token':state['csrf']})
            with self.http.open(request,timeout=2) as response:self.assertEqual(response.status,200)
        undeclared_port=18868 if self.server.server_port!=18868 else 18869
        for host,origin,token in [
                ('evil.example:18867','http://evil.example:18867',self.app.csrf),
                ('0.0.0.0:18867','http://0.0.0.0:18867',self.app.csrf),
                (f'127.0.0.1:{undeclared_port}',f'http://127.0.0.1:{undeclared_port}',self.app.csrf),
                ('127.0.0.1:18867','http://localhost:18867',self.app.csrf),
                ('127.0.0.1:18867','http://127.0.0.1:'+str(self.server.server_port),self.app.csrf),
                ('127.0.0.1:18867','https://127.0.0.1:18867',self.app.csrf),
                ('127.0.0.1:18867','http://127.0.0.1:18867','wrong-token')]:
            request=Request(self.url+'/api/actions/stop',body,headers={'Content-Type':'application/json',
                'Host':host,'Origin':origin,'X-CSRF-Token':token})
            with self.subTest(host=host,origin=origin),self.assertRaises(HTTPError) as rejected:self.http.open(request,timeout=2)
            self.assertEqual(rejected.exception.code,403)
        with self.assertRaises(HTTPError) as rejected:
            self.http.open(Request(self.url+'/api/state',headers={'Host':'evil.example:18867'}),timeout=2)
        self.assertEqual(rejected.exception.code,403);self.assertIsNone(self.device.runtime)

    def test_browser_ports_are_explicit_loopback_only_and_validated_before_binding(self):
        self.assertEqual(loopback_browser_hosts([18867]),{'127.0.0.1:18867','localhost:18867'})
        invalid=(None,{},'18867',[0],[-1],[65536],[True],[18867.0],['18867'],['*'],['evil.example:18867'],['127.0.0.1:18867'])
        with patch('robot_workbench.service.ThreadingHTTPServer') as bind:
            for value in invalid:
                with self.subTest(value=value),self.assertRaisesRegex(ValueError,'browser_ports'):
                    serve(self.app,port=0,browser_ports=value)
            bind.assert_not_called()

    def test_two_Application_instances_reuse_port_with_new_identity_revision_and_CSRF(self):
        first=self.state()
        for _ in range(5):old=self.state()
        self.assertEqual(first['server_instance_id'],old['server_instance_id'])
        self.assertNotEqual(first['server_instance_id'],first['csrf'])
        port=self.server.server_port
        self.server.shutdown();self.server.server_close();self.thread.join(1)
        self.app=Application({'fake':self.device,'physical':Device('robot','physical',None)})
        self.server=serve(self.app,port=port)
        self.thread=threading.Thread(target=self.server.serve_forever,daemon=True);self.thread.start()
        fresh=self.state()
        self.assertEqual(self.server.server_port,port)
        self.assertNotEqual(fresh['server_instance_id'],old['server_instance_id'])
        self.assertLess(fresh['revision'],old['revision'])
        self.assertNotEqual(fresh['csrf'],old['csrf'])
        stale=Request(self.url+'/api/actions/stop',json.dumps(dict(environment='fake',business='calibration',request_id='old-instance')).encode(),
            headers={'Content-Type':'application/json','Origin':self.url,'X-CSRF-Token':old['csrf']})
        with self.assertRaises(HTTPError) as rejected:self.http.open(stale)
        self.assertEqual(rejected.exception.code,403)
        code,_=self.action('stop');self.assertEqual(code,200)
        self.assertIsNone(self.device.runtime,'restart checks must not start an SDK or fake control loop')

    def calibration_adapter(self):
        """Actual C HTTP client/preflight/queue plus shared Owner; geometry is a CPU fixture."""
        import numpy as np
        sys.path[:0]=[str(ROOT/'calibration'),str(ROOT/'calibration/examples/scripts/debug'),str(ROOT/'calibration/tests')]
        from test_joint_head_workbench_live_bridge import BridgeTests
        from robot_workbench.bridges import start_control_gateway
        from robot_workbench.workflows import CalibrationWorkflow
        legacy=BridgeTests();legacy.setUp();self.addCleanup(legacy.doCleanups)
        c=legacy.app;c.output=c.output.resolve();c.queue=[];c.plan=None;c.geometry_id=c.plan_geometry_id='CPU-fixture-geometry'
        c.mesh_rendering_enabled=False;c.workflow={'kind':'head','head_result_id':None};c.review_generation=0
        c.config.update(environment='fake',simulation_only=True,robot_id='fake-http-fixture')
        c.calibration_root=c.output/'calibrations';c.hardware_feedback={'owner':self.device.snapshot('calibration')}
        c.hardware_connected=lambda:self.device.snapshot('calibration').get('feedback_healthy') is True
        def measured():
            q=self.device.snapshot('calibration')['calibration_feedback']['qpos'];return np.array(q[:6]+q[7:13])
        c.current_pose=measured
        c.geometry.path_check=Mock(return_value={'ok':True})
        tokens={'calibration':'c'*40,'collection':'d'*40}
        gateway,thread=start_control_gateway(self.device,tokens)
        def cleanup():gateway.shutdown();gateway.server_close();thread.join(1)
        self.addCleanup(cleanup)
        c.control={'port':gateway.server_port};c.control_token=tokens['calibration'];c.control_http=self.http
        flow=CalibrationWorkflow.__new__(CalibrationWorkflow);flow.app=c;flow.device=self.device;flow.service=self.app
        flow.lock=threading.Lock();flow.install_lock=threading.Lock();flow.active=False;flow.catalog=[]
        flow.value=dict(busy=False,status='idle',primary_action=None,manual_queue=flow.manual_queue_state(),arrival_statistics=c.arrival_state())
        self.device.workflows['calibration']=flow
        return c,flow

    def manual_job(self,action,**body):
        self.action(action,**body)
        value=self.wait(lambda s:(s['workflow'].get('request') or {}).get('status')!='running')
        with self.app.lock:job=dict(self.app.jobs[('fake','calibration')])
        self.assertEqual(job['status'],'completed',job.get('error'))
        return value

    def test_real_http_manual_review_and_explicit_handoff(self):
        self.assertFalse(self.state()['runtime']['connected'])
        state=self.connect()
        self.assertEqual(state['runtime']['motion_status'],'no_automatic_task')
        self.assertIn('calibration_capture_ack_v1',state['runtime']['capabilities'])
        self.assertFalse(self.state('collection')['runtime']['allowed_operations']['preview']['allowed'])
        q=state['manual']['positions'][:];q[0]+=.01
        _,accepted=self.action('manual_preview',q12=q,jaw_gap_mm=[80.,80.])
        ready=self.wait(lambda s:s['workflow'].get('review'))
        self.assertEqual(ready['workflow']['review']['request_job_id'],accepted['job_id'])
        self.assertGreater(ready['workflow']['review']['total_motion_duration_s'],0.)
        self.assertEqual(ready['workflow']['review']['waypoint_jaw_gap_mm'],[[80.,80.]])
        self.assertTrue(0 < ready['workflow']['review']['expires_at_wall_ns']-ready['server_wall_ns'] <= 31e9)
        self.assertEqual(ready['workflow']['primary_action']['id'],'manual_execute')
        self.assertEqual(ready['workflow']['stage_id'],'manual.review')
        self.assertEqual(ready['workflow']['status'],'awaiting_review')
        self.assertEqual(ready['workflow']['id'],ready['workflow']['review']['id'])
        with self.assertRaises(HTTPError):
            self.action('manual_execute',review_id=ready['workflow']['review']['id'],physical_clearance=False)
        self.action('manual_execute',review_id=ready['workflow']['review']['id'],physical_clearance=True)
        running=self.wait(lambda s:s['workflow']['stage_id']=='manual.execute')
        self.assertEqual(running['workflow']['status'],'executing')
        done=self.wait(lambda s:s['workflow']['stage_id']=='manual.complete')
        self.assertEqual(done['workflow']['status'],'completed')
        self.assertIn('最后一个路径点已收到 READY',done['workflow']['message'])
        self.assertNotIn('正在执行',done['workflow']['message'])
        self.assertNotEqual(done['workflow']['primary_action']['id'],'manual_preview',
                            'disabled task placeholder must not override the separate manual preview gate')
        self.assertEqual(done['runtime']['motion']['plan_sha256'],
                         self.app.manual_tasks[('fake','calibration')]['plan_sha256'][-1])
        self.action('handoff',target_business='collection')
        self.assertEqual(self.state('collection')['runtime']['holder'],'collection')
        self.assertTrue(self.state()['runtime']['connected'])
        self.assertNotIn(('fake','calibration'),self.app.manual_tasks)
        self.assertEqual(self.state()['workflow']['stage_id'],'configuration')
        with self.assertRaises(HTTPError):self.action('manual_preview',q12=q,jaw_gap_mm=[80.,80.])

    def test_C_queue_speeds_persistence_whole_preview_and_stale_review_rejection(self):
        self.connect();c,flow=self.calibration_adapter()
        for index,speed in enumerate((.08,.04)):
            queue=self.state()['manual']['queue']
            self.manual_job('manual_queue',operation='add',queue_revision=queue['revision'],waypoint=dict(
                q=[.002*(index+1)]+[0.]*11,speeds=[speed]*12,jaw_gap_mm=[80.,80.],jaw_speed_mm_s=5.+index))
        self.assertEqual(len(json.loads((c.output/'workbench-state.json').read_text())['queue']),2)
        ready=self.manual_job('manual_queue_preview',queue_revision=self.state()['manual']['queue']['revision'])
        self.assertEqual(ready['workflow']['stage_id'],'manual.review','C idle must not override manual review')
        plans=next(iter(self.device.runtime.owner.pending.values()))[0]
        self.assertEqual(len(plans),2);self.assertEqual([p.max_velocity for p in plans],[.08,.04])
        self.assertEqual([p.jaw_speed_mm_s for p in plans],[5.,6.])
        self.assertGreaterEqual(c.geometry.path_check.call_count,4,'new queue bypassed the original C path checks')
        old=ready['workflow']['review']['id'];revision=ready['manual']['queue']['revision']
        self.manual_job('manual_queue',operation='remove',index=0,queue_revision=revision)
        self.assertNotIn(('fake','calibration'),self.app.manual_tasks)
        with self.assertRaises(HTTPError):self.action('manual_execute',review_id=old,physical_clearance=True)
        ready=self.manual_job('manual_queue_preview',queue_revision=self.state()['manual']['queue']['revision'])
        self.action('manual_execute',review_id=ready['workflow']['review']['id'],physical_clearance=True)
        done=self.wait(lambda s:s['workflow'].get('stage_id')=='manual.complete')
        self.assertAlmostEqual(done['manual']['positions'][0],.004)
        ready=self.manual_job('manual_queue_preview',queue_revision=self.state()['manual']['queue']['revision'])
        self.action('stop')
        self.assertNotIn(('fake','calibration'),self.app.manual_tasks)
        with self.assertRaises(HTTPError):self.action('manual_execute',review_id=ready['workflow']['review']['id'],physical_clearance=True)
        self.assertEqual(len(c.queue),1,'stop must not delete the persisted queue')

    def test_manual_home_targets_speed_validation_permissions_and_original_arrival_store(self):
        self.connect();c,flow=self.calibration_adapter()
        ready=self.manual_job('manual_preview',q12=[0.]*12,speeds12=[.12]*12,jaw_gap_mm=[83.,83.],jaw_speed_mm_s=7.)
        plan=next(iter(self.device.runtime.owner.pending.values()))[0][0]
        self.assertEqual(plan.max_velocity,.12);self.assertEqual(plan.jaw_speed_mm_s,7.)
        self.action('manual_execute',review_id=ready['workflow']['review']['id'],physical_clearance=True)
        self.wait(lambda s:s['runtime'].get('motion_status')=='READY')
        self.action('manual_preview',q12=[0.]*12,speeds12=[0.]*12,jaw_gap_mm=[83.,83.],jaw_speed_mm_s=7.)
        failed=self.wait(lambda s:(s['workflow'].get('request') or {}).get('status')=='failed')
        self.assertFalse(failed['workflow'].get('review'))
        event=dict(event_id='CPU-endpoint-fixture',owner_pid=11,command_epoch=2,status='success',timestamp_ns=time.time_ns(),
            plan_sha256='fixture-plan',q_target_rad=[0.]*12,q_actual_rad=[.001]*12)
        pending=dict(preview={'owner_pid':11,'command_epoch':2,'plans':[{'sha256':'fixture-plan'}]},workflow={'kind':'head'})
        c.record_arrivals({'arrival_events':[event]},pending,c.output/'batch/capture','simulated')
        flow.value['arrival_statistics']=c.arrival_state()
        report=self.state()['arrival_statistics']
        self.assertEqual(report['total_count'],1);self.assertEqual(report['joints']['left'][0]['mean_rad'],.001)
        self.assertEqual(Path(report['raw_path']),c.calibration_root/'diagnostics/arrival-errors/simulated/samples.jsonl')
        self.action('handoff',target_business='collection')
        with self.assertRaises(HTTPError):self.action('manual_queue',operation='clear',queue_revision=c.review_generation)
        self.assertFalse(self.state()['runtime']['allowed_operations']['manual_queue']['allowed'])

    def test_L1_without_C_reports_unavailable_queue_and_statistics(self):
        state=self.connect()
        self.assertFalse(state['manual']['queue']['available'])
        self.assertIn('未配置',state['arrival_statistics']['error'])
        self.assertNotIn('joints',state['arrival_statistics'])
        self.assertFalse(state['runtime']['allowed_operations']['manual_queue_preview']['allowed'])

    def test_calibration_execute_ui_shares_host_age_gate_without_polling_devices(self):
        self.connect();c,flow=self.calibration_adapter();c.pipeline_review={'kind':'head'}
        c.hardware_reader=SimpleNamespace(max_age_s=.5,read_state=Mock(side_effect=AssertionError('state polled a device')))
        c.control_request=Mock(side_effect=AssertionError('state made a control HTTP request'))
        baseline=self.device.snapshot('calibration')
        baseline['allowed_operations']['confirm']=dict(allowed=True,reason=None)
        self.app.add_review(self.device,'calibration',dict(token='UI-gate-fixture',owner_pid=baseline['pid'],
            command_epoch=baseline['command_epoch']),'execute','CPU UI readiness fixture')
        from copy import deepcopy
        age=[.501]
        def snapshot(business):
            value=deepcopy(baseline)
            value['calibration_feedback']['sample_wall_ns']=time.time_ns()-round(age[0]*1e9)
            return value
        with patch.object(self.device,'snapshot',side_effect=snapshot):
            for age[0],ready in ((.501,False),(-1.,False),(.02,True)):
                state=self.state();workflow=state['workflow'];allowed=state['runtime']['allowed_operations']
                self.assertEqual(workflow['control_readiness']['host_sample_recent'],ready)
                self.assertEqual(workflow['primary_action']['allowed'],ready)
                self.assertEqual(allowed['execute']['allowed'],ready)
                self.assertEqual(allowed['confirm']['allowed'],ready)
            self.device.workflows.pop('calibration');self.app.reviews.clear();age[0]=1.
            state=self.state()
            self.assertNotIn('control_readiness',state['workflow'])
            self.assertTrue(state['runtime']['allowed_operations']['confirm']['allowed'])
        c.hardware_reader.read_state.assert_not_called();c.control_request.assert_not_called()

    def test_independent_capture_calls_real_C_and_persists_before_shared_Owner_ACK(self):
        self.connect();c,flow=self.calibration_adapter()
        c.frames=[];c.selected_batch=None;c.evaluation=None;c.batch=None;c.calibration_previews={}
        c.store=SimpleNamespace(robot_id='fake-http-fixture');c.hardware_history=[]
        from arx_env.joint_head_workbench_hardware import HardwareReadAdapter
        c.hardware_reader=HardwareReadAdapter(c.control['port'],environment='fake')
        c.hardware_poll_lock=threading.Lock();c.hardware_history_lock=threading.Lock();c.hardware_error=None
        c.hardware_progress=dict(source=None,sdk=None,advanced=[0.,0.])
        with self.assertRaisesRegex(ValueError,'等待两臂 SDK'):c.poll_hardware()
        self.wait(lambda state:all(new>old for new,old in zip(
            state['runtime']['calibration_feedback']['sdk_timestamps'],c.hardware_progress['sdk'])))
        c.poll_hardware()
        c.control_available=lambda:True
        c.control_status={'owner':self.device.snapshot('calibration')}
        intrinsics={'head':{'serial':'head-fixture','rectified':True,'coefficients':[0.]*5}}
        cameras={'head':SimpleNamespace(read_calibration_frame=lambda:(b'fixture',{'serial':'head-fixture'}))}
        c.hardware_calibration_cameras=lambda:(cameras,intrinsics)
        c.geometry.auto_plan=Mock(return_value={'plan_complete':True,'waypoints':[
            dict(id='sample-one',q=[.003]+[0.]*11,arm='left',kind='fit',cell=[0,0],capture=True),
            dict(id='return',q=[0.]*12,arm='left',kind='transition',cell=[0,0],capture=False)]})
        c.state=lambda:{'qpos':self.state()['manual']['positions']}
        self.manual_job('plan_batch')
        deadline=time.monotonic()+2
        while c.status=='PLANNING' and time.monotonic()<deadline:time.sleep(.01)
        self.assertEqual(c.status,'READY',c.reason)
        self.assertEqual(len(c.queue),2)
        ready=self.manual_job('preview_batch',queue_revision=c.review_generation)
        pending=next(iter(self.device.runtime.owner.pending.values()))
        self.assertIsNotNone(pending[-1][0],'capture sample identity must reach the shared Owner')
        self.assertIsNone(pending[-1][1],'transition waypoint must not capture')
        self.assertEqual(ready['workflow']['primary_action']['id'],'execute')
        calls=[];original=c.control_request
        def gateway(action,body=None,**kw):
            if action=='capture-ack':
                dataset=next(c.output.glob('batch-hardware-*/capture'))
                manifest=json.loads((dataset/'dataset.json').read_text());row=manifest['frames'][0]
                self.assertTrue((dataset/row['image']).is_file())
                self.assertTrue((dataset/(body['sample_id']+'.json')).is_file())
                self.assertTrue((dataset/'camera.json').is_file())
                calls.append(body)
            return original(action,body,**kw)
        c.control_request=gateway
        def sample(*args,**kwargs):
            self.assertEqual(kwargs['environment'],'fake')
            q=self.device.snapshot('calibration')['calibration_feedback']['qpos']
            return dict(row={'camera_detections':{'head':{'24':[[1,2]]*4}}},
                rows={'head':dict(qpos=q,q_reported12=q[:6]+q[7:13],bracket_gap_s=.02,pose_span_rad=[0.]*12,metadata={'serial':'head-fixture'})},
                images={'head':b'CPU-image-fixture'},raw_images={},cameras=intrinsics)
        with patch('arx_env.joint_head_workbench_capture.capture_sample',side_effect=sample):
            self.action('execute',review_id=ready['workflow']['review']['id'],physical_clearance=True)
            deadline=time.monotonic()+5
            while time.monotonic()<deadline:
                if c.selected_batch and c.status in ('READY','REJECTED'):break
                time.sleep(.01)
        self.assertEqual(c.status,'READY',c.reason)
        self.assertEqual(len(calls),1)
        self.assertEqual(self.device.snapshot('calibration')['last_capture_ack']['sample_id'],calls[0]['sample_id'])
        completion=json.loads((c.dataset/'completion.json').read_text())
        self.assertTrue(completion['passed']);self.assertEqual(completion['captured'],1)
        source=c.selected_batch
        self.manual_job('fit_source',batch_id=None);self.assertIsNone(c.selected_batch)
        self.manual_job('fit_source',batch_id=source);self.assertEqual(c.selected_batch,source)
        fit_entered=threading.Event();c.fit_head_batch=lambda:fit_entered.set()
        self.manual_job('fit_batch',batch_id=source);self.assertTrue(fit_entered.wait(1))

    def test_manual_stage_rejects_old_runner_restart_and_new_business_task(self):
        self.connect();c,flow=self.calibration_adapter()
        ready=self.manual_job('manual_preview',q12=[.003]+[0.]*11,jaw_gap_mm=[80.,80.])
        task=dict(self.app.manual_tasks[('fake','calibration')],submitted_monotonic_s=time.monotonic())
        runtime=self.device.snapshot('calibration')
        runtime.update(motion=dict(status='READY',plan_sha256=task['plan_sha256'][-1]),
            execution_calibration=dict(plan_sha256=task['plan_sha256'],command_epoch=task['epoch'],
                accepted_monotonic_s=task['submitted_monotonic_s']-1))
        value=self.app.manual_state(task,runtime,None,False)
        self.assertEqual(value['stage_id'],'manual.confirmation','old READY cannot complete a fresh request')
        runtime['last_confirm_rejection']=dict(plan_sha256=task['plan_sha256'],command_epoch=task['epoch'],
            motion_started=False,monotonic_s=task['submitted_monotonic_s']+.1,reason='review_expired')
        rejected=self.app.manual_state(task,runtime,None,False)
        self.assertEqual(rejected['stage_id'],'manual.rejected');self.assertEqual(rejected['status'],'expired')
        self.assertFalse(rejected['busy']);self.assertIn('动作未执行',rejected['message'])
        runtime['last_confirm_rejection']['monotonic_s']=task['submitted_monotonic_s']-1
        self.assertEqual(self.app.manual_state(task,runtime,None,False)['stage_id'],'manual.confirmation')
        runtime['execution_calibration']['accepted_monotonic_s']=task['submitted_monotonic_s']+1
        self.assertEqual(self.app.manual_state(task,runtime,None,False)['stage_id'],'manual.complete')
        runtime['runtime_nonce']='restarted-runtime'
        self.assertIsNone(self.app.manual_state(task,runtime,None,False))
        self.assertIsNone(self.app.manual_state(task,self.device.snapshot('calibration'),None,True))
        flow.action=Mock(return_value={'message':'new calibration task'})
        self.manual_job('generate_plan')
        self.assertNotIn(('fake','calibration'),self.app.manual_tasks)
        self.assertFalse(self.state()['workflow'].get('review'))

    def test_stop_preempts_background_work_and_revokes_review(self):
        self.connect();entered,release=threading.Event(),threading.Event()
        class Workflow:
            def snapshot(self):return dict(status='idle',busy=False,primary_action=None)
            def results(self):return []
            def action(self,*args):entered.set();release.wait(2);return {'message':'finished'}
            def cancel(self):release.set()
            def invalidate(self):pass
        self.device.workflows['calibration']=Workflow()
        self.action('generate_plan');self.assertTrue(entered.wait(.5))
        start=time.monotonic();code,result=self.action('stop')
        self.assertLess(time.monotonic()-start,.2)
        self.assertEqual(code,200)
        self.assertIn('stop',result)
        self.assertEqual(result['stop']['firmware_ack'],'unsupported')
        self.wait(lambda s:s['runtime'].get('stop_receipt',{}).get('stop_hold_return_confirmed'))

    def test_result_inspection_is_available_without_acquiring_controller(self):
        calls=[]
        class Workflow:
            def action(self,action,body):calls.append(action);return {'message':'read-only view updated'}
        self.device.workflows['calibration']=Workflow()
        for name in ('select_result','mesh'):
            self.action(name,result_id='poor-candidate',enabled=True)
            deadline=time.monotonic()+1
            while time.monotonic()<deadline and len(calls)<1:time.sleep(.005)
            self.assertIn(name,calls)
            with self.app.lock:job=self.app.jobs[('fake','calibration')]
            while job['status']=='running' and time.monotonic()<deadline:time.sleep(.005)
            self.assertEqual(job['status'],'completed')
            calls.clear()
        self.assertIsNone(self.device.runtime)

    def test_diagnostics_separate_latched_controller_fault_from_service_thread_error(self):
        owner=self.device.snapshot('calibration')
        owner.update(state='FAULT_STOP_UNCONFIRMED',reason='SDK feedback age 0.1996s exceeds bound',feedback_healthy=True)
        with patch.object(self.device,'snapshot',return_value=owner):
            rows={row['label']:row['value'] for row in self.state()['diagnostics']}
            self.assertEqual(rows['控制器状态'],'FAULT_STOP_UNCONFIRMED')
            self.assertEqual(rows['控制器当前原因'],owner['reason'])
            self.assertEqual(rows['服务线程异常'],'未报告服务线程异常')
            self.assertNotIn('控制循环',rows)
            self.device.error='CPU service thread failure'
            rows={row['label']:row['value'] for row in self.state()['diagnostics']}
            self.assertEqual(rows['服务线程异常'],'CPU service thread failure')
            self.assertEqual(rows['控制器当前原因'],owner['reason'])

    def test_result_media_streams_ranges_without_exposing_paths(self):
        path=Path(self.temp.name)/'result.mp4';path.write_bytes(b'0123456789')
        class Workflow:
            def media(self, ident):return path if ident=='current-video' else None
        self.device.workflows['collection']=Workflow()
        url=self.url+'/api/result-media?environment=fake&business=collection&result_id='
        with self.http.open(Request(url+'current-video',headers={'Range':'bytes=2-5'})) as response:
            self.assertEqual(response.status,206)
            self.assertEqual(response.headers['Content-Range'],'bytes 2-5/10')
            self.assertEqual(response.read(),b'2345')
        for ident, headers, code in [('current-video',{'Range':'bytes=20-'},416),('../result.mp4',{},404)]:
            with self.assertRaises(HTTPError) as error:self.http.open(Request(url+ident,headers=headers))
            self.assertEqual(error.exception.code,code)
        self.assertFalse(self.device.runtime)

    def test_result_media_JSON_is_limited_to_the_registered_current_collection_plan(self):
        from robot_workbench.workflows import CollectionWorkflow
        root=Path(self.temp.name);path=root/'plan.json';payload={'timeline_digest':'current-plan','ticks':[]}
        path.write_text(json.dumps(payload));config=root/'config.json';config.write_text('{"not_a_plan":true}')
        state={'plan':{'validated':True,'timeline_digest':'current-plan','plan_path':str(path)},'can_execute':False}
        flow=CollectionWorkflow(SimpleNamespace(directory=root,snapshot=lambda:state,state=state,lock=threading.RLock(),
            initialization=lambda:dict(status='stopped')),self.device,self.app)
        flow.closed.set();flow.thread.join(1);flow.refresh()
        self.device.workflows['collection']=flow
        trajectory=flow.snapshot()['trajectory'];url=self.url+trajectory['url']
        with self.http.open(url) as response:
            self.assertEqual(response.status,200);self.assertEqual(response.headers['Content-Type'],'application/json')
            self.assertEqual(json.loads(response.read()),payload)
        with flow.lock:flow.media_files['registered-config']=config
        for suffix,code in [('registered-config',409),('../config.json',404),('unknown-plan',404)]:
            with self.assertRaises(HTTPError) as error:self.http.open(url.replace(trajectory['id'],suffix))
            self.assertEqual(error.exception.code,code)
        self.device.workflows['calibration']=flow
        with self.assertRaises(HTTPError) as error:self.http.open(url.replace('business=collection','business=calibration'))
        self.assertEqual(error.exception.code,409)
        state['plan']={};flow.refresh()
        with self.assertRaises(HTTPError) as error:self.http.open(url)
        self.assertEqual(error.exception.code,404);self.assertIsNone(self.device.runtime)

    def test_explicit_collection_steps_do_not_authorize_execution(self):
        from robot_workbench.workflows import CollectionWorkflow
        self.connect();self.action('handoff',target_business='collection')
        state=dict(status='ready',busy=False,environment={'ready':True},plan={},can_execute=False)
        playground=SimpleNamespace(directory=Path(self.temp.name),lock=threading.RLock(),state=state,snapshot=lambda:deepcopy(state),
            start=Mock(return_value={'output':'new-step','status':'accepted'}),initialization=lambda:dict(status='stopped'))
        flow=CollectionWorkflow(playground,self.device,self.app);flow.closed.set();flow.thread.join(1);flow.refresh()
        self.device.workflows['collection']=flow
        self.assertFalse(self.state('collection')['runtime']['allowed_operations']['collection_prepare']['allowed'])
        with self.assertRaises(HTTPError):self.action('collection_prepare',business='collection',speed_scale=.4)
        playground.start.assert_not_called()
        for action,expected in (('collection_plan','plan'),):
            flow.refresh();self.assertTrue(self.state('collection')['runtime']['allowed_operations'][action]['allowed'])
            code,_=self.action(action,business='collection',speed_scale=.4);self.assertEqual(code,202)
            deadline=time.monotonic()+2
            while self.app.jobs[('fake','collection')]['status']=='running' and time.monotonic()<deadline:time.sleep(.01)
            self.assertEqual(self.app.jobs[('fake','collection')]['status'],'completed')
            playground.start.assert_called_with({'action':expected,'speed_scale':.4})
        self.assertFalse(self.state('collection')['runtime']['allowed_operations']['execute']['allowed'])
        with self.assertRaises(HTTPError):self.action('execute',business='collection',review_id='not-approved',physical_clearance=True)
        self.assertEqual(playground.start.call_count,1,'step buttons must never bypass the execution review or prepare services')

    def test_calibration_JSON_export_uses_original_version_store_without_connect(self):
        sys.path[:0]=[str(ROOT/'calibration'),str(ROOT/'calibration/infra'),str(ROOT/'calibration/examples/scripts/debug'),str(ROOT/'calibration/tests')]
        from test_joint_head_workbench_library import LibraryTests
        from robot_workbench.workflows import CalibrationWorkflow
        legacy=LibraryTests();legacy.setUp();self.addCleanup(legacy.doCleanups)
        head,wrist=legacy.pair()
        legacy.app.action('/api/calibration-install',{'head_result_id':head['id'],'wrist_result_id':wrist['id']})
        flow=CalibrationWorkflow.__new__(CalibrationWorkflow);flow.lock=threading.Lock();flow.app=legacy.app
        flow.catalog=[dict(id=head['id'],kind='head'),dict(id=wrist['id'],kind='wrist')]
        self.device.workflows['calibration']=flow
        path=self.url+'/api/calibration-file?environment=fake&result_id='
        with self.http.open(path+head['id']) as response:
            self.assertIn('attachment',response.headers['Content-Disposition'])
            self.assertEqual(json.loads(response.read()),legacy.app.calibration_store_record(head['id']))
        with self.assertRaises(HTTPError):self.http.open(path+'../active/head.json')
        with self.assertRaises(HTTPError):self.http.open(path.replace('environment=fake','environment=physical')+head['id'])
        self.assertIsNone(self.device.runtime)

    def test_delete_preview_confirm_through_HTTP_uses_legacy_token_without_device_connection(self):
        sys.path[:0]=[str(ROOT/'calibration'),str(ROOT/'calibration/infra'),str(ROOT/'calibration/examples/scripts/debug'),str(ROOT/'calibration/tests')]
        from test_joint_head_workbench_library import LibraryTests
        from robot_workbench.workflows import CalibrationWorkflow
        legacy=LibraryTests();legacy.setUp();self.addCleanup(legacy.doCleanups);path=legacy.batch('delete-via-http')
        c=legacy.app;c.live=False;c.mesh_rendering_enabled=False
        flow=CalibrationWorkflow.__new__(CalibrationWorkflow);flow.lock=threading.Lock();flow.app=c
        flow.device=self.device;flow.service=self.app;flow.value={'busy':False,'status':'idle','primary_action':None}
        flow.catalog=[{'id':'capture:delete-via-http','kind':'capture'}];flow.replay_context=None;flow.replay=[];flow.media_files={}
        self.device.workflows['calibration']=flow
        ready=self.manual_job('delete_preview',scope='batch',result_id='capture:delete-via-http')
        preview=ready['workflow']['deletion_preview'];self.assertEqual(preview['image_files'],2)
        self.assertIn(preview['id'],c.delete_tokens,'UI identity must wrap the actual legacy deletion token')
        self.assertTrue(path.exists());self.assertIsNone(self.device.runtime)
        self.action('delete_confirm',delete_preview_id=preview['id'],confirmed=False)
        self.wait(lambda s:(s['workflow'].get('request') or {}).get('status')=='failed')
        self.assertTrue(path.exists())
        result=self.manual_job('delete_confirm',delete_preview_id=preview['id'],confirmed=True)
        self.assertFalse(path.exists());self.assertIsNone(result['workflow']['deletion_preview'])
        self.assertEqual(result['workflow']['deletion_result']['removed'],['delete-via-http'])
        self.assertEqual(result['results'],[]);self.assertIsNone(self.device.runtime)

    def test_csrf_namespace_and_frame_identity(self):
        with self.assertRaises(HTTPError) as error:
            self.http.open(Request(self.url+'/api/actions/stop',b'{}',headers={'Content-Type':'application/json'}))
        self.assertEqual(error.exception.code,403)
        with self.assertRaises(HTTPError):
            self.http.open(self.url+'/api/state?environment=unknown&business=calibration')
        class Media:
            def frame(self):return b'jpeg-fixture',dict(etag='"1"',frame_id='one',source_wall_ns=123)
            def snapshot(self):return dict(id='head',url='/api/media/head?environment=fake')
        self.device.media['head']=Media()
        with self.http.open(self.url+'/api/media/head?environment=fake') as reply:
            self.assertEqual(reply.headers['X-Source-Wall-Ns'],'123')
        with self.assertRaises(HTTPError) as error:
            self.http.open(Request(self.url+'/api/media/head?environment=fake',headers={'If-None-Match':'"1"'}))
        self.assertEqual(error.exception.code,304)
        self.assertFalse(self.state()['runtime']['connected'])


class HTTPTransportTest(unittest.TestCase):
    """Real TCP connections, without starting a runtime or hardware worker."""
    def setUp(self):
        self.device=Device('transport-cpu','fake',None)
        self.app=Application({'fake':self.device});self.server=serve(self.app,port=0)
        self.thread=threading.Thread(target=self.server.serve_forever,daemon=True);self.thread.start()
        self.addCleanup(lambda:(self.server.shutdown(),self.server.server_close(),self.thread.join(1)))
        self.host='127.0.0.1:'+str(self.server.server_port)

    def test_same_socket_handles_state_post_304_and_fully_consumed_error(self):
        connection=HTTPConnection('127.0.0.1',self.server.server_port,timeout=2);self.addCleanup(connection.close)
        setup=self.server.RequestHandlerClass.setup;settings=[]
        def inspect_setup(request):
            setup(request);settings.append((request.connection.gettimeout(),request.connection.getsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY)))
        with patch.object(self.server.RequestHandlerClass,'setup',inspect_setup):
            connection.request('GET','/api/state');response=connection.getresponse();state=json.loads(response.read())
        self.assertEqual(response.version,11);self.assertEqual(response.status,200);self.assertEqual(len(settings),1)
        self.assertEqual(settings[0][0],5.);self.assertTrue(settings[0][1],'TCP_NODELAY must be enabled on the accepted server socket')
        original=connection.sock
        headers={'Origin':'http://'+self.host,'X-CSRF-Token':state['csrf'],'Content-Type':'application/json'}
        body=json.dumps(dict(environment='fake',business='calibration',request_id='keep-alive-stop'))
        connection.request('POST','/api/actions/stop',body,headers);response=connection.getresponse();response.read()
        self.assertEqual(response.status,200);self.assertIs(connection.sock,original)
        self.device.media['head']=SimpleNamespace(frame=lambda:(b'jpeg',dict(etag='"frame"',frame_id='one',source_wall_ns=123)),snapshot=lambda:{})
        connection.request('GET','/api/media/head',headers={'If-None-Match':'"frame"'});response=connection.getresponse()
        self.assertEqual(response.status,304);self.assertEqual(response.read(),b'');self.assertIs(connection.sock,original)
        connection.request('POST','/api/actions/stop','{',headers);response=connection.getresponse();response.read()
        self.assertEqual(response.status,409);self.assertNotEqual(response.getheader('Connection'),'close')
        connection.request('GET','/api/state');response=connection.getresponse();response.read()
        self.assertEqual(response.status,200);self.assertIs(connection.sock,original)

    def test_unread_or_ambiguous_body_closes_without_running_embedded_request(self):
        self.app.state=Mock(wraps=self.app.state)
        poison=f'GET /api/state HTTP/1.1\r\nHost: {self.host}\r\n\r\n'.encode()
        auth=[('Origin','http://'+self.host),('X-CSRF-Token',self.app.csrf)]
        cases=[
            ('POST','/api/actions/stop',[],[('Content-Length',str(len(poison)))],403),
            ('POST','/not-an-action',auth,[('Content-Length',str(len(poison)))],404),
            ('POST','/api/actions/stop',auth,[],409),
            ('POST','/api/actions/stop',auth,[('Content-Length','-1')],409),
            ('POST','/api/actions/stop',auth,[('Content-Length',str(4*1024*1024+1))],413),
            ('POST','/api/actions/stop',auth,[('Content-Length','2'),('Content-Length',str(len(poison)))],409),
            ('POST','/api/actions/stop',auth,[('Content-Length',str(len(poison))),('Transfer-Encoding','chunked')],409),
            ('GET','/api/state',[],[('Content-Length',str(len(poison)))],409),
            ('TRACE','/api/state',[],[('Content-Length',str(len(poison)))],501),
        ]
        for method,path,authorization,framing,status in cases:
            with self.subTest(method=method,path=path,framing=framing),socket.create_connection(('127.0.0.1',self.server.server_port),timeout=2) as client:
                headers=[('Host',self.host),*authorization,*framing]
                request=f'{method} {path} HTTP/1.1\r\n'+''.join(f'{key}: {value}\r\n' for key,value in headers)+'\r\n'
                client.sendall(request.encode()+poison+poison);chunks=[]
                while True:
                    try:data=client.recv(65536)
                    except ConnectionResetError:break
                    if not data:break
                    chunks.append(data)
                reply=b''.join(chunks)
                self.assertTrue(reply.startswith(f'HTTP/1.1 {status} '.encode()),reply[:80])
                self.assertIn(b'connection: close\r\n',reply.lower());self.assertEqual(reply.count(b'HTTP/1.1 '),1)
        self.app.state.assert_not_called()

    def test_idle_keep_alive_connection_expires(self):
        self.assertEqual(self.server.RequestHandlerClass.timeout,5)
        self.server.RequestHandlerClass.timeout=.08
        connection=HTTPConnection('127.0.0.1',self.server.server_port,timeout=2);self.addCleanup(connection.close)
        connection.request('GET','/api/state');response=connection.getresponse();response.read()
        self.assertEqual(response.status,200);self.assertEqual(connection.sock.recv(1),b'')

    def test_incomplete_declared_body_closes_without_dispatch(self):
        self.app.action=Mock(wraps=self.app.action)
        with socket.create_connection(('127.0.0.1',self.server.server_port),timeout=2) as client:
            client.sendall((f'POST /api/actions/stop HTTP/1.1\r\nHost: {self.host}\r\nOrigin: http://{self.host}\r\n'
                f'X-CSRF-Token: {self.app.csrf}\r\nContent-Length: 10\r\n\r\n{{}}').encode())
            client.shutdown(socket.SHUT_WR);chunks=[]
            while True:
                data=client.recv(65536)
                if not data:break
                chunks.append(data)
            reply=b''.join(chunks)
        self.assertTrue(reply.startswith(b'HTTP/1.1 409 '));self.assertIn(b'connection: close\r\n',reply.lower())
        self.app.action.assert_not_called()


if __name__=='__main__':unittest.main()
