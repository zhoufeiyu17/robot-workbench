"""Review refresh is a read-only operation even while a pipeline awaits it."""
from pathlib import Path
from copy import deepcopy
import json
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock,patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from robot_workbench.workflows import CalibrationWorkflow,CollectionWorkflow,calibration_stage,collection_live_view


class WorkflowAdapterTest(unittest.TestCase):
    def test_research_manual_preview_reuses_calibration_geometry_and_research_credential(self):
        flow=CalibrationWorkflow.__new__(CalibrationWorkflow)
        capture={}
        def geometry_preview(body,*,submit_preview,start_q):
            capture.update(body=body,start_q=start_q);return submit_preview(body['waypoints'])
        flow.app=SimpleNamespace(hardware_trajectory_preview=geometry_preview)
        flow.device=SimpleNamespace(snapshot=lambda business:dict(calibration_feedback=dict(qpos=[0.]*14)),
            submit=Mock(return_value={'purpose':'research-manual'}))
        body=dict(q12=[.01]*12,speeds12=[.1]*12,jaw_gap_mm=[80,80],jaw_speed_mm_s=10)
        result=flow.manual_preview(body,business='research')
        self.assertEqual(result['purpose'],'research-manual');self.assertEqual(capture['start_q'],[0.]*12)
        self.assertEqual(flow.device.submit.call_args.args[0],'research')
        self.assertEqual(flow.device.submit.call_args.args[1]['action'],'preview_workbench')
        with self.assertRaisesRegex(ValueError,'single-arm'):flow.manual_preview(dict(body,active_arm='left'),business='research')

    def test_finished_calibration_does_not_keep_environment_busy_via_old_active_flag(self):
        flow=CalibrationWorkflow.__new__(CalibrationWorkflow);flow.lock=threading.Lock();flow.active=True;flow.generation=3
        for status in ('failed','stopped','completed'):
            flow.value=dict(busy=False,status=status,stage_id='wrist.plan')
            self.assertFalse(flow.environment_status()['busy'])
            flow.value['busy']=True
            self.assertTrue(flow.environment_status()['busy'],'real work or sealing must still block maintenance')
        flow.value=dict(busy=False,status='running')
        self.assertTrue(flow.environment_status()['busy'])

    def test_single_arm_manual_preview_uses_checked_control_path_without_jaw_or_other_arm(self):
        class Vector(list):
            def tolist(self):return list(self)
        flow=CalibrationWorkflow.__new__(CalibrationWorkflow)
        flow.app=SimpleNamespace(geometry=SimpleNamespace(_q=Vector),control_request=Mock(return_value={'plans':[]}))
        body=dict(active_arm='right',q12=[.1]*6+[.2,.3,.4,0.,.5,.6],speeds12=[.03]*6+[.5]*6)
        flow.manual_preview(body)
        flow.app.control_request.assert_called_once_with('motion-preview',dict(
            active_arm='right',q_active=[.2,.3,.4,0.,.5,.6],max_velocity=.5))
        for update in ({'jaw_gap_mm':[83.,83.]},{'active_arm':'both'},{'speeds12':[.03]*11+[.5]},
                       {'speeds12':[float('nan')]*12},{'speeds12':[.5001]*12}):
            with self.subTest(update=update),self.assertRaises(ValueError):flow.manual_preview(dict(body,**update))
        with self.assertRaises(ValueError):flow.manual_preview(body,queue=True)

    def test_collection_admission_uses_live_busy_in_both_cache_races(self):
        for action in ('generate_plan','execute'):
            with self.subTest(action=action):
                flow,state=self.collection()
                with flow.lock:flow.value['busy']=True
                with flow.playground.lock:state['busy']=False
                self.assertTrue(flow.allowed(action)['allowed'])
                self.assertFalse(flow.allowed('cancel')['allowed'])
                result=flow.action(action,{'token':'frozen'})
                self.assertIn('preview' if action=='generate_plan' else 'output',result)
                if action=='generate_plan':flow.playground.start.assert_not_called()
                else:flow.playground.start.assert_called_once_with({'action':'execute','speed_scale':.3})
        flow,state=self.collection()
        with flow.lock:flow.value['busy']=False
        with flow.playground.lock:state['busy']=True
        for action in ('generate_plan','execute'):
            self.assertFalse(flow.allowed(action)['allowed'])
            with self.assertRaisesRegex(ValueError,'当前数采阶段不允许此操作'):
                flow.action(action,{'token':'frozen'})
        self.assertTrue(flow.allowed('cancel')['allowed'])
        flow.action('cancel',{})
        flow.playground.start.assert_called_once_with({'action':'interrupt'})

    def frozen_service(self):
        root=Path(__file__).resolve().parents[1]
        sys.path[:0]=[str(root/'rule_based_workbench'),str(root/'tests')]
        from test_restore_prepared import RestorePreparedTest
        from robot_workbench.service import Application,Device,serve
        fixture=RestorePreparedTest();fixture.setUp();self.addCleanup(fixture.doCleanups)
        device=Device('fixture-physical','physical',None);device.runtime=SimpleNamespace(holder='collection')
        def snapshot(business):
            return dict(fixture.bridge.owner(),runtime_nonce='frozen-restore-cpu',calibration_revision=1,
                allowed_operations={name:dict(allowed=True,reason=None) for name in ('preview','confirm','handoff')})
        device.snapshot=snapshot
        device.collection_preparation=SimpleNamespace(snapshot=lambda:dict(status='failed',busy=False,
            can_retry=True,retry_reason=None,persistent_service=True))
        app=Application({'physical':device})
        flow=CollectionWorkflow(fixture.service,device,app);device.workflows['collection']=flow
        flow.closed.set();flow.thread.join(2)
        self.addCleanup(flow.closed.set)
        server=serve(app,port=0);thread=threading.Thread(target=lambda:server.serve_forever(poll_interval=.01),daemon=True);thread.start()
        app._cpu_test_http_url='http://127.0.0.1:'+str(server.server_port)
        def close_http():server.shutdown();server.server_close();thread.join(2)
        self.addCleanup(close_http)
        return app,device,flow,fixture

    def frozen_action(self,app,action,**fields):
        from urllib.request import Request,build_opener,ProxyHandler
        from urllib.error import HTTPError
        url=app._cpu_test_http_url
        body=dict(environment='physical',business='collection',request_id='frozen-'+str(time.time_ns()),**fields)
        request=Request(url+'/api/actions/'+action,json.dumps(body).encode(),headers={
            'X-CSRF-Token':app.csrf,'Origin':url,'Content-Type':'application/json'})
        try:
            with build_opener(ProxyHandler({})).open(request,timeout=3) as response:return json.loads(response.read())
        except HTTPError as error:raise ValueError(json.loads(error.read())['error']) from error

    def wait_frozen_job(self,app):
        end=time.monotonic()+3
        while time.monotonic()<end:
            job=app.jobs[('physical','collection')]
            if job['status']!='running':return job
            time.sleep(.01)
        self.fail('frozen service job did not finish')

    def test_application_restores_real_frozen_artifacts_without_warmup_and_keeps_review_gate(self):
        from robot_workbench.service import frozen_collection_scope
        app,device,flow,fixture=self.frozen_service()
        self.assertFalse(frozen_collection_scope(device))
        restore=fixture.service.restore_prepared
        def restore_with_stale_monitor():
            result=restore()
            with flow.lock:flow.value['busy']=True  # Last UI poll saw restore in progress.
            return result
        with patch.object(fixture.service,'start',side_effect=AssertionError('restoration cannot start work')),\
                patch.object(fixture.service,'restore_prepared',side_effect=restore_with_stale_monitor):
            self.frozen_action(app,'generate_plan',restore_frozen_plan=True)
            self.assertEqual(self.wait_frozen_job(app)['status'],'completed')
        self.assertTrue(frozen_collection_scope(device))
        self.assertFalse(fixture.service.snapshot()['environment']['ready'])
        self.assertEqual(device.collection_preparation.snapshot()['status'],'failed')
        review=app.reviews[('physical','collection')]
        self.assertEqual(review['token'],fixture.plan['timeline_digest'])
        flow.refresh();view=app.state('physical','collection')
        self.assertEqual(view['workflow']['review']['id'],review['id'])
        self.assertTrue(view['runtime']['allowed_operations']['execute']['allowed'])
        self.assertFalse(view['workflow']['collection']['environment']['ready'])
        self.frozen_action(app,'generate_plan')
        self.assertEqual(self.wait_frozen_job(app)['status'],'completed')
        review=app.reviews[('physical','collection')]
        with self.assertRaisesRegex(ValueError,'环境准备'):
            self.frozen_action(app,'collection_plan')
        with self.assertRaisesRegex(ValueError,'审批'):
            self.frozen_action(app,'execute',review_id='unreviewed',physical_clearance=True)
        with self.assertRaisesRegex(ValueError,'净空'):
            self.frozen_action(app,'execute',review_id=review['id'])
        with patch.object(fixture.service,'start',return_value={'output':'cpu-only-accepted'}) as start:
            self.frozen_action(app,'execute',review_id=review['id'],physical_clearance=True)
            self.assertEqual(self.wait_frozen_job(app)['status'],'completed')
        start.assert_called_once_with({'action':'execute','speed_scale':review['speed_scale']})
        fixture.bridge.control.assert_not_called();fixture.recover.assert_not_called()

    def test_application_frozen_execution_rechecks_calibration_and_consumed_checkpoint(self):
        app,device,flow,fixture=self.frozen_service()
        self.frozen_action(app,'generate_plan',restore_frozen_plan=True)
        self.assertEqual(self.wait_frozen_job(app)['status'],'completed')
        review=app.reviews[('physical','collection')]
        checkpoint=fixture.service.directory/'prepared-plan.json';saved=json.loads(checkpoint.read_text())
        consumed=deepcopy(saved);consumed.update(execution_pending=True,execution_output=str(fixture.service.directory/'jobs/unknown-execution'))
        checkpoint.write_text(json.dumps(consumed))
        with self.assertRaisesRegex(ValueError,'未执行冻结计划'):
            self.frozen_action(app,'execute',review_id=review['id'],physical_clearance=True)
        checkpoint.write_text(json.dumps(saved))
        binding_path=Path(fixture.bridge.config['trusted_binding_path']);binding=json.loads(binding_path.read_text())
        binding['calibration_profile_digest']='0'*64;binding_path.write_text(json.dumps(binding))
        with self.assertRaisesRegex(ValueError,'标定版本已改变'):
            self.frozen_action(app,'generate_plan',restore_frozen_plan=True)
        fixture.bridge.control.assert_not_called();fixture.recover.assert_not_called()

    def test_application_frozen_flag_does_not_bypass_missing_plan_drift_or_normal_planning(self):
        app,device,flow,fixture=self.frozen_service()
        checkpoint=fixture.service.directory/'prepared-plan.json'
        raw=checkpoint.read_bytes();checkpoint.unlink()
        with self.assertRaisesRegex(ValueError,'未执行冻结计划'):
            self.frozen_action(app,'generate_plan',restore_frozen_plan=True)
        checkpoint.write_bytes(raw)
        fixture.current['right_arm_joints'][0]+=.011
        with self.assertRaisesRegex(ValueError,'当前实测姿态'):
            self.frozen_action(app,'generate_plan',restore_frozen_plan=True)
        for action,fields in (('collection_plan',{'restore_frozen_plan':True}),
                ('generate_plan',{'restore_frozen_plan':True,'replan':True}),('execute',{'restore_frozen_plan':True})):
            with self.subTest(action=action),self.assertRaises(ValueError):self.frozen_action(app,action,**fields)
        self.assertEqual(app.jobs,{})
        fixture.bridge.control.assert_not_called();fixture.recover.assert_not_called()

    def test_explicit_checkpoint_restore_only_installs_a_new_review(self):
        flow,state=self.collection()
        state.update(environment={'ready':False},can_execute=False)
        def restore():state['can_execute']=True;return deepcopy(state['plan'])
        flow.playground.restore_prepared=Mock(side_effect=restore)
        result=flow.action('generate_plan',{'restore_frozen_plan':True})
        flow.playground.restore_prepared.assert_called_once()
        flow.playground.start.assert_not_called()
        self.assertEqual(result['preview']['token'],'frozen')
        self.assertEqual(result['preview']['command_epoch'],5)
        self.assertEqual(result['preview']['expires_in_s'],30)
        state['can_execute']=False;flow.playground.restore_prepared.side_effect=lambda:state['plan']
        with self.assertRaisesRegex(ValueError,'不会自动预热'):
            flow.action('generate_plan',{'restore_frozen_plan':True})
        flow.playground.start.assert_not_called()
        for action,body in (('execute',{'restore_frozen_plan':True}),
                            ('generate_plan',{'restore_frozen_plan':'yes'}),
                            ('generate_plan',{'restore_frozen_plan':True,'replan':True})):
            with self.assertRaisesRegex(ValueError,'恢复冻结计划'):flow.action(action,body)

    def test_explicit_physical_assistance_id_reaches_planning_without_changing_default_gate(self):
        flow,state=self.collection();flow.device.environment='physical'
        state.update(environment={'ready':False},busy=False)
        flow.refresh()
        with self.assertRaisesRegex(ValueError,'环境准备'):flow.action('collection_plan',{})
        with self.assertRaisesRegex(ValueError,'初始化与预热'):
            flow.action('collection_plan',{'operator_assisted_request_id':'this-accepted-scene','speed_scale':.4})
        flow.playground.start.assert_not_called();state['environment']['ready']=True
        flow.action('collection_plan',{'operator_assisted_request_id':'this-accepted-scene','speed_scale':.4})
        flow.playground.start.assert_called_once_with(dict(action='plan',speed_scale=.4,
            operator_assisted_request_id='this-accepted-scene'))
        with self.assertRaisesRegex(ValueError,'正在进行'):
            flow.action('collection_plan',{'operator_assisted_request_id':'this-accepted-scene'})
        for action,value in (('generate_plan','x'),('collection_prepare','x'),('collection_plan',''),('collection_plan',None)):
            with self.assertRaisesRegex(ValueError,'辅助定位'):
                flow.action(action,{'operator_assisted_request_id':value})
        state['busy']=False;flow.device.environment='fake'
        with self.assertRaisesRegex(ValueError,'真机'):
            flow.action('collection_plan',{'operator_assisted_request_id':'x'})

    def test_history_candidate_switch_keeps_frame_camera_and_solver_source(self):
        flow=self.calibration();a=flow.app
        a.selected_batch='solver-source'
        a.calibration_results.return_value.append(dict(id='head-second',batch='batch/capture',status='quality_failed'))
        frames=[dict(id='frame-'+str(i),image='head.png',split='holdout',valid_for_fit=False,
            detections={'24':{'corners':[]}},image_urls={'head':'/artifacts/batch/capture/head.png'}) for i in range(2)]
        a.review_batch.return_value={'frames':frames,'results':[dict(r,kind='head') for r in a.calibration_results()]}
        flow.action('select_result',dict(result_id='capture:batch/capture',frame_index=1,camera='head',overlay=True,candidate_id='head-poor'))
        flow.action('select_result',dict(result_id='capture:batch/capture',candidate_id='head-second'))
        context=flow.snapshot()['replay_context']
        self.assertEqual((context['frame_index'],context['camera'],context['candidate_id']),(1,'head','head-second'))
        self.assertEqual(context['frame']['split'],'holdout');self.assertFalse(context['frame']['valid_for_fit'])
        self.assertEqual(len(context['frames']),2);self.assertEqual(context['frame']['detections'],frames[1]['detections'])
        a.historical_overlay.assert_called_with(dict(batch_id='batch/capture',frame_index=1,camera='head',result_id='head-second'))
        self.assertEqual(a.selected_batch,'solver-source');a.action.assert_not_called()
        with self.assertRaisesRegex(ValueError,'匹配'):flow.action('select_result',dict(result_id='capture:batch/capture',candidate_id='another-batch'))

    def test_domain_views_keep_report_metrics_timing_and_source_identity(self):
        flow=self.calibration();report=dict(phase='头部：求解',steps=[dict(step='solve',half='head',result_id='head-poor',metrics={'holdout_marker_rms_px':4.2})],aborted='质量失败')
        flow.app.runtime_state.return_value=dict(status='REJECTED',pipeline=report,counts={'captured':34})
        flow.refresh();value=flow.snapshot()
        self.assertEqual(value['pipeline']['steps'],report['steps']);self.assertEqual(value['quality']['results'][0]['metrics'],report['steps'][0]['metrics'])
        value['quality']['results'][0]['metrics']['holdout_marker_rms_px']=0
        self.assertEqual(report['steps'][0]['metrics']['holdout_marker_rms_px'],4.2)
        flow,state=self.collection();source=self.root/'source.json';binding=self.root/'binding.json'
        source.write_text(json.dumps(dict(run_id='observation',scene_digest='scene',calibration_profile={'profile_id':'profile'})))
        binding.write_text(json.dumps(dict(robot_id='robot',calibration_profile_digest='profile-sha',camera_serials={'head':'camera'})))
        state['plan'].update(source_path=str(source),hardware_binding=str(binding))
        state['progress']=dict(label='检测',elapsed_s=25,stage_elapsed_s=12,estimate_s=40,estimate_source='历史参考',next_label='规划',stages=[{'label':'观测','status':'completed','elapsed_s':5,'estimate_s':4}])
        flow.refresh();domain=flow.snapshot()['collection']
        self.assertEqual(domain['timing'],state['progress']);self.assertEqual(domain['relations']['source']['run_id'],'observation')
        self.assertEqual(domain['relations']['calibration']['calibration_profile_digest'],'profile-sha')
        flow.device.collection_prepare_operation=Mock(return_value={'operation_id':'prepare-one'})
        result=flow.action('collection_prepare',{})
        self.assertEqual(result['action'],'prepare')
        flow.device.collection_prepare_operation.assert_not_called()
        self.assertTrue(flow.playground.start.call_args.kwargs['lifecycle_admission']('warmup:collection'))
        state.update(busy=False);flow.refresh();flow.action('collection_plan',{});flow.playground.start.assert_called_with({'action':'plan','speed_scale':.3})
        state.update(busy=False,environment={'ready':False});flow.refresh()
        with self.assertRaisesRegex(ValueError,'环境准备'):flow.action('collection_plan',{})

    def test_live_collection_receipts_require_current_full_binding(self):
        binding=dict(device_id='device',environment='fake',execution_instance_id='execution',runtime_nonce='owner',source_clock_id='clock',timeline_digest='parent',execution_timeline_digest='timeline')
        view=dict(id='job',status='running',stage_id='collection.execute',busy=True,execution_binding=binding,outcomes={'motion':'未执行','data':'未录制','task':'unverified'})
        runtime=dict(device_id='device',environment='fake',holder='collection',execution_instance_id='execution',runtime_nonce='owner',command_epoch=4,
            execution_calibration=dict(business='collection',execution_instance_id='execution',command_epoch=4),calibration_feedback={'source_clock_id':'clock'},
            native_task=dict(execution_instance_id='execution',runtime_nonce='owner',source_clock_id='clock',execution_timeline_digest='timeline',task_outcome='success'),
            frozen=dict(timeline_digest='parent',execution_timeline_digest='timeline',status='completed',submitted_command_ticks=10,execution_tick_count=10),
            execution_timeline=dict(parent_timeline_digest='parent',timeline_digest='timeline',command_count=10),recording=dict(done=True,manifest={'validation':{'status':'passed'}}))
        live=collection_live_view(deepcopy(view),runtime)
        self.assertEqual(live['outcomes']['motion']['status'],'completed');self.assertEqual(live['outcomes']['data']['status'],'valid')
        self.assertEqual(live['outcomes']['task']['status'],'pending','native success still requires the workflow receipt audit')
        self.assertEqual(live['progress']['current'],10)
        invalid=[('view',('execution_binding',),{}),('view',('execution_binding','execution_instance_id'),'old'),
            ('view',('execution_binding','runtime_nonce'),'old'),('view',('stage_id',),'collection.plan'),
            ('view',('status',),'completed'),('view',('busy',),False),('view',('id',),None),
            *[('runtime',(key,),'other') for key in ('runtime_nonce','execution_instance_id','device_id','environment','holder')],
            *[('runtime',('execution_calibration',key),'other') for key in ('execution_instance_id','business')],
            ('runtime',('execution_calibration','command_epoch'),3),('runtime',('command_epoch',),True),
            *[('runtime',('native_task',key),'other') for key in ('execution_instance_id','runtime_nonce','source_clock_id','execution_timeline_digest')],
            ('runtime',('calibration_feedback','source_clock_id'),'other'),
            *[('runtime',('frozen',key),'other') for key in ('timeline_digest','execution_timeline_digest')],
            *[('runtime',('execution_timeline',key),'other') for key in ('parent_timeline_digest','timeline_digest')]]
        for target,path,replacement in invalid:
            with self.subTest(target=target,path=path):
                copied_view,copied_runtime=deepcopy(view),deepcopy(runtime)
                parent=copied_view if target=='view' else copied_runtime
                for key in path[:-1]:parent=parent[key]
                parent[path[-1]]=replacement
                self.assertEqual(collection_live_view(deepcopy(copied_view),copied_runtime),copied_view)
        for count in (-1,1.5,True,11):
            malformed=deepcopy(runtime);malformed['frozen']['submitted_command_ticks']=count
            self.assertNotIn('progress',collection_live_view(deepcopy(view),malformed))
        for count in (0,-1,True,10.5,11):
            malformed=deepcopy(runtime);malformed['frozen']['execution_tick_count']=count
            self.assertNotIn('progress',collection_live_view(deepcopy(view),malformed))
        failures={name:{'status':'failed','message':'authoritative workflow failure'} for name in ('motion','data','task')}
        self.assertEqual(collection_live_view({**deepcopy(view),'outcomes':deepcopy(failures)},runtime)['outcomes'],failures)

    def setUp(self):
        folder=tempfile.TemporaryDirectory();self.addCleanup(folder.cleanup);self.root=Path(folder.name).resolve()

    def calibration(self):
        workflow=CalibrationWorkflow.__new__(CalibrationWorkflow)
        workflow.lock=threading.Lock();workflow.install_lock=threading.Lock();workflow.value={'busy':False};workflow.active=False
        workflow.preview_key=None;workflow.installed_id=None;workflow.last_catalog=-1e10
        workflow.preview_context=None;workflow.preview_failure=None;workflow.preview_pending=None;workflow.generation=0
        workflow.catalog=[];workflow.replay=[];workflow.replay_context=None;workflow.media_files={}
        self.batch=self.root/'batch/capture';self.batch.mkdir(parents=True,exist_ok=True)
        for name in ('head.png','left.png','overlay.png'):(self.batch/name).write_bytes(b'CPU fixture only')
        heads=[dict(id='head-poor',batch='batch/capture',kind='head',status='quality_failed')]
        wrists=[dict(id='wrist-poor',file='batch/capture/wrist.json',kind='wrist',pair_status='quality_failed')]
        group=dict(id='head-poor',wrist={'id':'wrist-poor'},installable=False,installed=False,reason='质量未通过，只读查看仍可用')
        workflow.app=SimpleNamespace(output=self.root,live=True,remote={},mesh_rendering_enabled=False,pipeline_review=None,
            config={'asset_geometry_id':'asset'},pipeline_run_id='run',stop=threading.Event(),
            runtime_state=Mock(return_value={'status':'READY','workflow':{'kind':'head'},'pipeline':{}}),
            calibration_results=Mock(return_value=heads),wrist_results=Mock(return_value=wrists),
            calibration_groups=Mock(return_value=[group]),history=Mock(return_value=[{'id':'batch/capture','status':'completed'}]),
            installed_pair=Mock(return_value=None),invalidate_reviews=Mock(),
            action=Mock(return_value={'message':'selected'}),
            review_batch=Mock(return_value={'frames':[{'image_urls':{'head':'/artifacts/batch/capture/head.png','left':'/artifacts/batch/capture/left.png'}}]}),
            historical_overlay=Mock(return_value={'image_url':'/artifacts/batch/capture/overlay.png'}))
        workflow.device=SimpleNamespace(environment='fake',runtime=SimpleNamespace(holder='calibration',notify_calibration_installed=Mock()))
        workflow.service=SimpleNamespace(add_review=Mock(),invalidate=Mock(),lock=threading.Lock(),reviews={('fake','calibration'):{'id':'old'}})
        return workflow

    def collection(self):
        workflow=CollectionWorkflow.__new__(CollectionWorkflow);workflow.lock=threading.Lock()
        workflow.value={'busy':False};workflow.catalog=[];workflow.review_key=None;workflow.media_files={};workflow.generation=0
        workflow.closed=threading.Event()
        state=dict(output='job-1',environment={'ready':True},busy=False,can_execute=True,status='ready',action='plan',progress={},
            plan=dict(validated=True,timeline_digest='frozen',output='job-1',duration_s=20.,speed_scale=.3),videos=[])
        def start(payload,**kwargs):
            if payload['action']=='interrupt':return {'status':'stopping'}
            state.update(output='job-next',busy=True,can_execute=False,status='running',action=payload['action'])
            return dict(status='accepted',action=payload['action'],output=state['output'])
        workflow.playground=SimpleNamespace(snapshot=Mock(side_effect=lambda:deepcopy(state)),start=Mock(side_effect=start),
            directory=self.root,media=Mock(),invalidate_checkpoint=Mock(),lock=threading.Lock(),state=state,
            initialization=lambda:dict(status='ready',planner=dict(status='ready',model_initialized=True)))
        workflow.device=SimpleNamespace(id='fixture-device',environment='fake',runtime=SimpleNamespace(holder='collection'),
            snapshot=Mock(return_value=dict(command_epoch=5,connected=True,feedback_healthy=True,stationary=True,
                automatic_enabled=False,sample_age_s=.01)))
        workflow.service=SimpleNamespace(add_review=Mock(),lock=threading.Lock(),reviews={('fake','collection'):{'id':'old'}},generations={'fake':0})
        return workflow,state

    def legacy_library(self):
        root=Path(__file__).resolve().parents[1]
        sys.path[:0]=[str(root/'calibration'),str(root/'calibration/infra'),str(root/'calibration/examples/scripts/debug'),str(root/'calibration/tests')]
        from test_joint_head_workbench_library import LibraryTests
        legacy=LibraryTests();legacy.setUp();self.addCleanup(legacy.doCleanups)
        flow=self.calibration();flow.app=legacy.app;c=legacy.app
        c.live=False;c.mesh_rendering_enabled=False;c.calibration_root=c.output/'calibration-diagnostics'
        c.runtime_state=lambda:dict(status=c.status,workflow=c.workflow,counts=c.counts(),selected_batch=c.selected_batch)
        return legacy,flow

    def test_original_delete_all_preserves_installed_group_versions_and_arrival_log(self):
        legacy,flow=self.legacy_library();c=legacy.app;head,wrist=legacy.pair();extra=legacy.batch('extra-source')
        c.action('/api/calibration-install',{'head_result_id':head['id'],'wrist_result_id':wrist['id']})
        installed=c.installed_pair();active=Path(installed['path'])
        active_bytes={str(path.relative_to(active)):path.read_bytes() for path in active.rglob('*') if path.is_file()}
        stats=c.arrival_store('simulated')
        stats.record(dict(event_id='arrival-one',batch_id='head-capture',workflow='head',status='success',
            q_target_rad=[0.]*12,q_actual_rad=[.001]*12,timestamp_ns=1,owner_pid=1,command_epoch=0,plan_sha256='a'*64))
        report=stats.summary();arrival_bytes={key:Path(report[key]).read_bytes() for key in ('raw_path','summary_path')}
        c.load_batch('extra-source');flow.refresh()
        flow.action('select_result',{'result_id':'capture:head-capture'})
        self.assertEqual(c.selected_batch,'extra-source','ordinary browsing must not select a solver source')
        flow.action('delete_preview',{'scope':'all'});preview=flow.snapshot()['deletion_preview']
        self.assertTrue(preview['valid']);self.assertIn('extra-source',preview['batches'])
        self.assertIn('所选求解批次',preview['clears']);self.assertIn('实时叠加选择',preview['clears'])
        self.assertGreaterEqual(preview['image_files'],4);self.assertTrue(preview['preserved_versions'])
        self.assertEqual(preview['preserved_installed'],installed)
        self.assertTrue(extra.exists(),'a deletion preview may not remove raw data')
        with self.assertRaisesRegex(ValueError,'明确确认'):
            flow.action('delete_confirm',{'delete_preview_id':preview['id'],'confirmed':False})
        result=flow.action('delete_confirm',{'delete_preview_id':preview['id'],'confirmed':True})
        self.assertEqual(set(result['removed']),set(preview['batches']));self.assertEqual(c.history(),[])
        self.assertIsNone(c.selected_batch);self.assertIsNone(flow.snapshot()['replay_context'])
        self.assertEqual(c.installed_pair(),installed)
        self.assertEqual({str(path.relative_to(active)):path.read_bytes() for path in active.rglob('*') if path.is_file()},active_bytes)
        self.assertEqual({key:Path(report[key]).read_bytes() for key in arrival_bytes},arrival_bytes)
        flow.last_catalog=-1e10;flow.refresh()
        self.assertTrue(all(row['kind']!='capture' for row in flow.results()))
        self.assertTrue(all(row['raw_deleted'] for row in flow.results()))
        self.assertEqual(flow.download(head['id'])['id'],head['id'])
        flow.action('select_result',{'result_id':head['id']})
        self.assertEqual(flow.snapshot()['replay_context']['frame_count'],0)
        self.assertFalse(flow.snapshot()['replay_context']['overlay_available'])
        self.assertIn('已删除',flow.snapshot()['replay_context']['message'])
        self.assertIsNone(c.selected_batch)

    def test_original_single_delete_rechecks_busy_changed_files_and_cancelled_preview(self):
        legacy,flow=self.legacy_library();c=legacy.app;path=legacy.batch('delete-one')
        flow.action('delete_preview',{'scope':'batch','result_id':'capture:delete-one'})
        preview=flow.snapshot()['deletion_preview'];self.assertEqual(preview['image_files'],2)
        for status in ('CAPTURING','FITTING'):
            c.status=status
            self.assertFalse(flow.allowed('delete_preview')['allowed'])
            with self.assertRaisesRegex(ValueError,'进行中'):
                flow.action('delete_confirm',{'delete_preview_id':preview['id'],'confirmed':True})
            self.assertTrue(path.exists())
        c.status='READY';(path/'frame-000.png').write_bytes(b'changed after preview')
        with self.assertRaisesRegex(ValueError,'文件变化'):
            flow.action('delete_confirm',{'delete_preview_id':preview['id'],'confirmed':True})
        self.assertTrue(path.exists());self.assertIsNone(flow.snapshot()['deletion_preview'])
        flow.action('delete_preview',{'scope':'batch','result_id':'capture:delete-one'})
        expired=flow.snapshot()['deletion_preview'];record=c.delete_tokens[expired['id']]
        c.delete_tokens[expired['id']]=(record[0],record[1],0.,record[3])
        self.assertFalse(flow.snapshot()['deletion_preview']['valid']);self.assertFalse(flow.allowed('delete_confirm')['allowed'])
        self.assertTrue(path.exists())
        flow.action('delete_preview',{'scope':'batch','result_id':'capture:delete-one'})
        flow.action('delete_cancel',{})
        with self.assertRaises(ValueError):flow.action('delete_confirm',{'delete_preview_id':preview['id'],'confirmed':True})
        self.assertTrue(path.exists())
        c.selected_batch='delete-one'
        with self.assertRaisesRegex(ValueError,'栏目中使用'):
            flow.action('delete_preview',{'scope':'batch','result_id':'capture:delete-one'})
        c.selected_batch=None
        flow.action('delete_preview',{'scope':'batch','result_id':'capture:delete-one'})
        preview=flow.snapshot()['deletion_preview']
        flow.action('delete_confirm',{'delete_preview_id':preview['id'],'confirmed':True})
        self.assertFalse(path.exists())
        with self.assertRaises(ValueError):flow.action('delete_confirm',{'delete_preview_id':preview['id'],'confirmed':True})

    def test_expired_batch_review_can_refresh_without_restarting_pipeline(self):
        workflow=self.calibration()
        workflow.value={'busy':True}
        workflow.app.pipeline_review={'kind':'wrist'}
        workflow.app.preview_calibration=Mock(return_value={'token':'new-token','samples':32,'total_motion_duration_s':90.})
        workflow.app.action.side_effect=AssertionError('must not restart full pipeline')
        self.assertTrue(workflow.allowed('generate_plan')['allowed'])
        result=workflow.action('generate_plan',{})
        self.assertTrue(result['review_published'])
        self.assertEqual(workflow.service.add_review.call_args.args[2]['token'],'new-token')
        workflow.app.preview_calibration.assert_called_once_with()
        workflow.app.action.assert_not_called()
        workflow.app.pipeline_review=None
        self.assertFalse(workflow.allowed('generate_plan')['allowed'])
        with self.assertRaises(ValueError):workflow.action('generate_plan',{})

    def awaiting_calibration_review(self):
        workflow=self.calibration();workflow.active=True
        workflow.app.pipeline_review={'kind':'wrist','label':'腕半场'}
        workflow.app.runtime_state.side_effect=lambda:dict(status='PIPELINE',workflow={'kind':'wrist'},
            pipeline=dict(run_id=workflow.app.pipeline_run_id,running=True,review=workflow.app.pipeline_review))
        workflow.app.preview_calibration=Mock(return_value={'token':'new-token','samples':29,'expires_in_s':30,'total_motion_duration_s':90.})
        return workflow

    def test_calibration_publication_failure_persists_until_explicit_success(self):
        for failing in ('preview','publication'):
            with self.subTest(failing=failing):
                workflow=self.awaiting_calibration_review()
                failed=(workflow.app.preview_calibration if failing=='preview' else workflow.service.add_review)
                failed.side_effect=ValueError('specific '+failing+' failure')
                workflow.refresh();workflow.refresh()
                state=workflow.snapshot();failure=state['review_error']
                self.assertEqual((state['status'],state['stage_id']),('failed','wrist.review'))
                self.assertEqual((failure['run_id'],failure['stage_id']),('run','wrist.review'))
                self.assertIn('specific '+failing+' failure',state['message'])
                self.assertIn('重新生成',state['message']);self.assertTrue(state['primary_action']['allowed'])
                self.assertIn('run',state['diagnostics'][0]['value'])
                self.assertIsNone(workflow.preview_key)
                workflow.app.preview_calibration.assert_called_once()
                with self.assertRaises(ValueError):workflow.action('generate_plan',{})
                self.assertEqual(workflow.snapshot()['status'],'failed')
                attempts=workflow.app.preview_calibration.call_count
                workflow.refresh();self.assertEqual(workflow.app.preview_calibration.call_count,attempts)
                failed.side_effect=None
                result=workflow.action('generate_plan',{})
                self.assertTrue(result['review_published'])
                self.assertIsNone(workflow.preview_failure);self.assertIsNotNone(workflow.preview_key)
                self.assertNotIn('review_error',workflow.snapshot())
                self.assertEqual(workflow.service.add_review.call_args.args[2]['workflow_generation'],workflow.generation)
                workflow.app.action.assert_not_called()  # No pipeline restart or confirmation/motion.
                workflow.refresh();self.assertEqual(workflow.app.preview_calibration.call_count,attempts+1)

    def test_cancel_and_new_run_fence_late_calibration_publication(self):
        for change in ('cancel','new_run'):
            with self.subTest(change=change):
                workflow=self.awaiting_calibration_review();entered=threading.Event();release=threading.Event();errors=[]
                def preview():
                    if threading.current_thread() is worker:
                        entered.set();self.assertTrue(release.wait(2.))
                        return {'token':'old-token','samples':29,'total_motion_duration_s':90.}
                    return {'token':'new-token','samples':29,'total_motion_duration_s':90.}
                workflow.app.preview_calibration.side_effect=preview
                def refresh():
                    try:workflow.refresh()
                    except Exception as error:errors.append(error)
                worker=threading.Thread(target=refresh);worker.start()
                try:
                    self.assertTrue(entered.wait(1.))
                    if change=='cancel':workflow.cancel()
                    else:
                        workflow.app.pipeline_run_id='new-run';workflow.refresh()
                finally:release.set();worker.join(2.)
                self.assertFalse(worker.is_alive());self.assertEqual(errors,[])
                self.assertIsNone(workflow.preview_failure)
                if change=='cancel':
                    self.assertIsNone(workflow.preview_key);workflow.service.add_review.assert_not_called()
                    self.assertNotIn('review_error',workflow.snapshot())
                else:
                    workflow.service.add_review.assert_called_once()
                    self.assertEqual(workflow.service.add_review.call_args.args[2]['token'],'new-token')
                    self.assertEqual(workflow.snapshot()['id'],'new-run')

    def test_running_calibration_description_tracks_phase_not_initial_guidance(self):
        workflow=self.calibration()
        for phase,expected in [('头部采集','正在逐点采集头部'),('腕半场采集','正在逐点采集双腕'),
                               ('头部求解','求解并验收头部'),('腕半场求解','贴纸观测求解')]:
            workflow.app.runtime_state.return_value={'status':'CAPTURING','pipeline':{'running':True,'phase':phase}}
            workflow.refresh();self.assertIn(expected,workflow.snapshot()['message'])
            self.assertNotIn('先生成计划',workflow.snapshot()['message'])
        workflow.app.runtime_state.return_value['pipeline']['aborted']='本次真实失败原因'
        workflow.refresh();self.assertEqual(workflow.snapshot()['message'],'本次真实失败原因')

    def test_calibration_preflight_details_stay_in_diagnostics_for_the_matching_run(self):
        workflow=self.calibration()
        failure=dict(run_id='run',stage_id='wrist.preflight',message='设备反馈样本已过期',
            path='/task/diagnostics/failure.json',read_diagnostics={'normalized_age_s':.7})
        workflow.app.control_preflight_failure=failure
        value=workflow.snapshot()
        self.assertEqual(value['diagnostics'][0]['value'],failure)
        self.assertNotIn('normalized_age_s',value.get('message',''))
        value['diagnostics'][0]['value']['read_diagnostics']['normalized_age_s']=99
        self.assertEqual(failure['read_diagnostics']['normalized_age_s'],.7)
        workflow.app.pipeline_run_id='new-run'
        self.assertNotIn('diagnostics',workflow.snapshot())

    def test_head_and_wrist_route_separately_and_quality_does_not_block_replay(self):
        workflow=self.calibration();workflow.refresh()
        self.assertEqual([row['kind'] for row in workflow.results()],['head','wrist','capture'])
        workflow.action('select_result',{'result_id':'wrist-poor'})
        workflow.app.action.assert_called_with('/api/wrist-select',{'result_id':'wrist-poor'})
        self.assertEqual(workflow.snapshot()['replay_context']['camera'],'left')
        media=workflow.snapshot()['replay'][0]
        self.assertEqual(workflow.media(media['id']),self.batch/'left.png')
        self.assertIn('/api/result-media?',media['url'])
        workflow.action('select_result',{'result_id':'head-poor','overlay':True})
        workflow.app.action.assert_called_with('/api/calibration',{'result_id':'head-poor'})
        workflow.app.historical_overlay.assert_called_once_with(dict(batch_id='batch/capture',frame_index=0,camera='head',result_id='head-poor'))
        workflow.app.review_batch.assert_called_with('batch/capture')
        self.assertFalse(workflow.results()[0]['install']['allowed'])
        self.assertTrue(workflow.allowed('mesh')['allowed'],'quality failed must not gate display')
        with self.assertRaisesRegex(ValueError,'质量'):workflow.action('install',{'result_id':'head-poor'})

    def test_new_adapter_calls_actual_legacy_selection_replay_and_group_install(self):
        root=Path(__file__).resolve().parents[1]
        sys.path[:0]=[str(root/'calibration'),str(root/'calibration/infra'),
                      str(root/'calibration/examples/scripts/debug'),str(root/'calibration/tests')]
        from test_joint_head_workbench_library import LibraryTests
        legacy=LibraryTests();legacy.setUp();self.addCleanup(legacy.doCleanups)
        head,wrist=legacy.pair()  # Real Workbench methods/store; fixture replaces IO, not API dispatch.
        workflow=self.calibration();workflow.app=legacy.app
        selected_before=legacy.app.selected_batch
        workflow.action('select_result',{'result_id':wrist['id']})
        self.assertEqual(legacy.app.wrist_selection_id(),wrist['id'])
        self.assertEqual(legacy.app.selected_batch,selected_before)
        workflow.action('select_result',{'result_id':head['id']})
        self.assertEqual(legacy.app.calibration['id'],head['id'])
        self.assertEqual(legacy.app.selected_batch,selected_before)
        self.assertTrue(workflow.media(workflow.snapshot()['replay'][0]['id']).is_file())
        order=[]
        workflow.device.runtime.notify_calibration_installed.side_effect=lambda identity:order.append(('revoked',identity))
        legacy.app.on_calibration_installed=workflow.calibration_installed
        original=legacy.app.action
        def observed_action(path,body):
            result=original(path,body)
            if path=='/api/calibration-install':order.append(('C_returned',None))
            return result
        legacy.app.action=observed_action
        reply=workflow.action('install',{'result_id':head['id']})
        self.assertTrue(reply['installed']['verified'])
        self.assertEqual(legacy.app.installed_pair()['head_id'],head['id'])
        self.assertEqual(legacy.app.installed_pair()['wrist_id'],wrist['id'])
        workflow.device.runtime.notify_calibration_installed.assert_called_once()
        self.assertEqual([row[0] for row in order],['revoked','C_returned'])
        self.assertLess(len(order[0][1]),512)
        workflow.calibration_installed(legacy.app.installed_pair())
        workflow.device.runtime.notify_calibration_installed.assert_called_once()
        legacy.app.runtime_state=lambda:dict(status=legacy.app.status,workflow=legacy.app.workflow,counts=legacy.app.counts(),selected_batch=legacy.app.selected_batch)
        legacy.app.arrival_state=Mock(return_value={'error':'CPU fixture has no arrival feed'})
        workflow.refresh()
        catalog={row['id']:row for row in workflow.results()}
        bundle=next(row for row in catalog.values() if row['kind']=='calibration')
        self.assertNotIn(head['id'],catalog);self.assertNotIn(wrist['id'],catalog)
        self.assertIn('metrics_rectified_px',bundle['details'])
        self.assertEqual(bundle['components'],dict(head=head['id'],wrist=wrist['id']))
        self.assertEqual(workflow.mesh_calibrations()[0]['id'],bundle['id'])
        self.assertEqual(workflow.mesh_calibrations()[0]['components'],bundle['components'])
        exported=workflow.download(bundle['id'])
        self.assertEqual(exported['schema'],'robot_workbench.calibration_bundle.v1')
        self.assertEqual(exported['records']['head']['id'],head['id'])
        self.assertEqual(exported['records']['wrist']['dependency_id'],head['id'])
        self.assertNotIn('path',exported['records']['head'])
        self.assertEqual(set(exported['records']),{'head','wrist'})
        saved_head=legacy.app.calibration_store_record(head['id'])
        saved_wrist=legacy.app.calibration_store_record(wrist['id'])
        from arx_env.joint_head_calibration_store import bundle_records
        for broken in (dict(saved_wrist,dependency_id='another-head'),dict(saved_wrist,robot_id='another-robot')):
            with self.assertRaises(ValueError):bundle_records(saved_head,broken)
        self.assertEqual(exported,workflow.download(bundle['id']))
        with patch.object(legacy.app,'action',side_effect=AssertionError('bundle replay must not change live selection')):
            selected=workflow.select_result({'result_id':bundle['id']})
        self.assertEqual(selected['replay_context']['result_id'],bundle['id'])
        self.assertEqual(workflow.snapshot()['calibration_tools']['installed']['head_id'],head['id'])
        self.assertTrue(workflow.snapshot()['calibration_tools']['installed']['verified'])
        self.assertEqual(workflow.download(head['id']),legacy.app.calibration_store_record(head['id']))
        self.assertIn('/api/calibration-file?',bundle['download_url'])
        with self.assertRaises(ValueError):workflow.download('../head.json')

    def test_original_C_workflow_fit_source_and_wrist_only_pipeline_entry(self):
        root=Path(__file__).resolve().parents[1]
        sys.path[:0]=[str(root/'calibration'),str(root/'calibration/infra'),str(root/'calibration/examples/scripts/debug'),str(root/'calibration/tests')]
        from test_joint_head_workbench_library import LibraryTests
        legacy=LibraryTests();legacy.setUp();self.addCleanup(legacy.doCleanups);legacy.populated()
        c=legacy.app;alternate=legacy.batch('alternate-head');report=deepcopy(c.fit_result)
        report['camera_T_world_optical'][0][3]=.02
        report.update(objective='sticker_corners_plus_mesh',acceptance={'image_passed':True,'image_checks':{'positive_depth':True}},
            optimizer={'success':True},metrics_rectified_px={'fit':{'corners':16,'rms_px':.5},'holdout':{'rms_px':.5}},
            mesh_metrics={'holdout':{'macro_coverage':.8,'macro_mean_penalized_px':5.}})
        (alternate/'fit.json').write_text(json.dumps(report))
        head=next(row for row in c.calibration_results() if row['batch']=='alternate-head')
        flow=self.calibration();flow.app=c
        selected=deepcopy(c.calibration)
        flow.action('configure_calibration',{'kind':'wrist','head_result_id':head['id']})
        self.assertEqual(c.workflow,{'kind':'wrist','head_result_id':head['id']})
        self.assertEqual(c.calibration,selected,'dependency selection must not change the display candidate')
        self.assertEqual(c.queue,[])
        with self.assertRaises(ValueError):flow.action('fit_source',{'batch_id':'alternate-head'})
        flow.action('fit_source',{'batch_id':'batch-live-a/capture'})
        self.assertEqual(c.selected_batch,'batch-live-a/capture')
        original_source=c.selected_batch;flow.action('select_result',{'result_id':head['id']})
        self.assertEqual(c.selected_batch,original_source,'readonly result inspection must not select a solver input')
        c.background=Mock()
        flow.action('plan_batch',{})
        self.assertEqual(c.background.call_args.args[0],'PLANNING')
        self.assertEqual(c.background.call_args.args[1],c.plan_wrists)
        with self.assertRaisesRegex(ValueError,'求解来源已变化'):flow.action('fit_batch',{'batch_id':'another-batch'})
        flow.action('fit_batch',{'batch_id':c.selected_batch})
        self.assertEqual(c.background.call_args.args[0],'FITTING')
        with patch.object(c,'fit_wrists') as solver:
            c.background.call_args.args[1]()
            solver.assert_called_once_with(allow_partial=False)
        flow.action('fit_source',{'batch_id':None});self.assertIsNone(c.selected_batch)
        with self.assertRaises(ValueError):flow.action('fit_batch',{})
        c.control_mode='hardware_sync';c._mesh_view_mode='simulation';c.hardware_connected=lambda:True
        c.hardware_feedback={'owner':{'mode':'simulation','simulation_only':True}}
        result=flow.action('generate_plan',{'mode':'wrist','head_result_id':head['id']})
        self.assertEqual(result['mode'],'wrist');self.assertEqual(result['rounds'],1)
        self.assertTrue(c.pipeline_running);self.assertEqual(c.background.call_args.args[0],'PIPELINE')
        self.assertEqual(c.workflow['head_result_id'],head['id'])
        c.pipeline_running=False
        with self.assertRaisesRegex(ValueError,'完整标定或仅重做双腕'):
            flow.action('generate_plan',{'mode':'refit','rounds':2})

    def test_batch_replay_does_not_change_solver_or_live_selection_even_while_busy(self):
        workflow=self.calibration();workflow.value={'busy':True}
        workflow.action('select_result',{'result_id':'capture:batch/capture'})
        workflow.app.action.assert_not_called()
        workflow.app.review_batch.assert_called_once_with('batch/capture')
        self.assertEqual(workflow.media(workflow.snapshot()['replay'][0]['id']),self.batch/'head.png')
        self.assertIsNone(workflow.media('../not-a-catalog-id'))
        workflow.app.review_batch.return_value['frames'][0]['image_urls']['head']='/artifacts/../escape.png'
        with self.assertRaisesRegex(ValueError,'超出'):workflow.action('select_result',{'result_id':'capture:batch/capture'})

    def test_install_uses_whole_group_and_cancel_revokes_without_claiming_stopped(self):
        workflow=self.calibration();workflow.app.calibration_groups.return_value[0].update(installable=True)
        workflow.app.installed_pair.return_value={'head_id':'head-poor','wrist_id':'wrist-poor','verified':True}
        workflow.action('install',{'result_id':'head-poor'})
        workflow.app.action.assert_called_once_with('/api/calibration-install',dict(head_result_id='head-poor',wrist_result_id='wrist-poor'))
        workflow.device.runtime.notify_calibration_installed.assert_called_once()
        workflow.value['busy']=True;workflow.action('cancel',{})
        self.assertTrue(workflow.app.stop.is_set());workflow.app.invalidate_reviews.assert_called_once()
        self.assertNotIn(('fake','calibration'),workflow.service.reviews)

    def test_stable_calibration_stages_ignore_display_wording_and_count_real_milestones(self):
        for index,kind,status in [(0,'head','PLANNING'),(1,'head','CAPTURING'),(2,'head','FITTING'),(3,'wrist','PLANNING'),(4,'wrist','CAPTURING'),(5,'wrist','FITTING')]:
            state={'status':status,'workflow':{'kind':kind},'pipeline':{'running':True,'phase':'可变的人类说明'}}
            stage,completed=calibration_stage(state)
            self.assertEqual(completed,index);self.assertNotIn('可变',stage)
        self.assertEqual(calibration_stage({'pipeline':{'review':{'kind':'wrist'}}}),('wrist.review',4))
        self.assertEqual(calibration_stage({'pipeline':{'installed':{'verified':True}}}),('calibration.complete',7))
        workflow=self.calibration();workflow.app.runtime_state.return_value={'status':'FITTING','workflow':{'kind':'wrist'},'pipeline':{'running':True}}
        workflow.refresh();self.assertEqual(workflow.snapshot()['overall_progress']['current'],5)

    def test_overall_progress_advances_with_fraction_and_never_fills_on_stop(self):
        workflow=self.calibration()
        report={'running':True,'subprogress':{'stage':2,'fraction':.2,'label':'优化迭代'}}
        workflow.app.runtime_state.return_value={'status':'FITTING','workflow':{'kind':'head'},'pipeline':report}
        workflow.refresh();first=workflow.snapshot()
        self.assertAlmostEqual(first['overall_progress']['current'],2.2)
        self.assertEqual(first['progress']['kind'],'fraction','fraction-only solver progress must survive the adapter')
        report['subprogress']['fraction']=.65;workflow.refresh()
        self.assertAlmostEqual(workflow.snapshot()['overall_progress']['current'],2.65)
        report['subprogress'].pop('fraction');workflow.refresh()
        self.assertTrue(workflow.snapshot()['overall_progress']['indeterminate'])
        self.assertTrue(workflow.snapshot()['progress']['indeterminate'])
        report.update(stopped=True,running=False);report['subprogress']['fraction']=1.
        workflow.refresh();self.assertEqual(workflow.snapshot()['overall_progress']['current'],2)
        self.assertEqual(workflow.snapshot()['status'],'stopped')
        self.assertIsNone(workflow.snapshot()['progress'])
        report.update(stopped=False,running=True,review={'kind':'wrist'})
        workflow.refresh();self.assertEqual(workflow.snapshot()['overall_progress']['current'],4)
        report.pop('review');report.update(phase='腕半场：求解')
        workflow.refresh();value=workflow.snapshot()
        self.assertEqual(value['stage_id'],'wrist.solve')
        self.assertEqual(value['overall_progress']['current'],5,'stale head fraction cannot advance a wrist stage')
        self.assertTrue(value['progress']['indeterminate'])

    def capture_progress_fixture(self):
        flow=self.calibration();flow.last_catalog=time.monotonic();flow.app.lock=threading.RLock()
        flow.app.selected_batch='head-batch/capture'
        flow.app.active_capture=dict(batch_id=flow.app.selected_batch,pipeline_run_id='run',kind='head',
            captured=32,expected_count=34)
        flow.app.batch={'completion':None}
        state=dict(status='CAPTURING',workflow={'kind':'head'},selected_batch=flow.app.selected_batch,
            counts=dict(capture_points=34,captured=32),pipeline=dict(run_id='run',running=True,phase='头部：采集',
                subprogress=dict(stage=1,completed=32,total=34,fraction=32/34,label='已采集图像')))
        flow.app.runtime_state.side_effect=lambda:deepcopy(state)
        return flow,state

    def test_failed_capture_retains_run_batch_stage_and_frozen_count_after_cleanup(self):
        flow,state=self.capture_progress_fixture()
        for terminal in ('running','rejected','cleaned'):
            if terminal=='rejected':state.update(status='REJECTED',reason='真机采集超时')
            if terminal=='cleaned':
                flow.app.pipeline_run_id=None
                state['pipeline'].update(running=False,phase=None,failed_phase='头部：采集',
                    aborted='ValueError：头部采集 失败：真机采集超时')
                # A newer acknowledged completion beats the active counter, while
                # subsequent queue editing must never rewrite this batch's total.
                flow.app.active_capture['captured']=31
                flow.app.batch={'completion':dict(passed=False,captured=32,reason='真机采集超时')}
                state['counts']['capture_points']=2
            flow.refresh();value=flow.snapshot()
            with self.subTest(terminal=terminal):
                self.assertEqual(value['id'],'run')
                self.assertEqual(value['capture_batch_id'],'head-batch/capture')
                self.assertEqual(value['stage_id'],'head.capture')
                self.assertEqual(value['title'],'头部：采集')
                self.assertEqual(value['progress']['current'],32);self.assertEqual(value['progress']['total'],34)
                self.assertAlmostEqual(value['overall_progress']['current'],1+32/34)
                self.assertEqual(value['overall_progress']['completed_stages'],1)
                self.assertEqual(value['failed_stage'],None if terminal=='running' else 'head.capture')
                if terminal!='running':
                    self.assertEqual(value['status'],'failed');self.assertIn('采集超时',value['message'])
        self.assertFalse(value['busy']);self.assertEqual(value['outcomes']['data'],'invalid')
        flow.app.history.assert_not_called();flow.app.calibration_results.assert_not_called()

    def test_terminal_capture_cold_refresh_uses_report_identity_and_matching_cached_completion(self):
        flow,state=self.capture_progress_fixture();flow.app.pipeline_run_id=None
        state.update(status='REJECTED',counts={'capture_points':99})
        state['pipeline'].update(running=False,phase=None,failed_phase='头部：采集',aborted='采集失败')
        state['pipeline'].pop('subprogress')
        flow.app.active_capture['captured']=31
        flow.app.batch={'completion':dict(passed=False,accepted=32,captured=33)}
        flow.refresh();value=flow.snapshot()
        self.assertEqual(value['id'],'run');self.assertEqual(value['progress']['current'],32)
        self.assertEqual(value['progress']['total'],34)
        # runtime_state and the later locked batch snapshot can straddle a selection.
        flow.app.selected_batch='another/capture'
        flow.app.batch={'completion':dict(passed=True,captured=34)}
        flow.refresh();value=flow.snapshot()
        self.assertEqual(value['progress']['current'],31)
        self.assertNotIn('data',value['outcomes'])
        flow.app.history.assert_not_called()

    def test_new_pipeline_does_not_inherit_failed_or_installed_prior_report(self):
        flow,state=self.capture_progress_fixture();flow.app.pipeline_run_id='new-run'
        state.update(status='REJECTED',reason='上一轮失败')
        for prior in ({'aborted':'上一轮失败','failed_phase':'头部：采集'},
                      {'installed':{'verified':True}}):
            state['pipeline']={**state['pipeline'],**prior}
            flow.refresh();value=flow.snapshot()
            with self.subTest(prior=prior):
                self.assertEqual(value['id'],'new-run');self.assertEqual(value['status'],'running')
                self.assertEqual(value['stage_id'],'head.plan');self.assertIsNone(value['capture_batch_id'])
                self.assertIsNone(value['failed_stage']);self.assertNotIn('上一轮',value['message'])
                self.assertEqual(value['overall_progress']['current'],0)
                self.assertTrue(value['progress']['indeterminate'])

    def test_independent_plan_keeps_prior_failure_out_of_current_pipeline(self):
        flow=self.calibration();flow.app.pipeline_run_id=None
        prior=dict(run_id='old-run',aborted='FileNotFoundError: old seed',failed_phase='腕半场：规划',
                   installed=dict(verified=True),steps=[])
        flow.app.runtime_state.return_value=dict(status='READY',workflow=dict(kind='wrist'),pipeline=prior)
        flow.operation=dict(id='new-plan',action='plan',kind='wrist',message='planning')
        flow.refresh();value=flow.snapshot()
        self.assertEqual(value['id'],'new-plan');self.assertEqual(value['status'],'completed')
        self.assertEqual(value['pipeline'],{});self.assertEqual(value['previous_pipeline']['aborted'],prior['aborted'])
        self.assertNotIn('installed',value['quality']);self.assertEqual(value['quality']['results'],[])
        self.assertEqual(flow.app.runtime_state.return_value['pipeline'],prior,'historical receipts remain intact')
        flow.operation=None;flow.app.restored_pipeline_report_id='old-run';flow.refresh();restored=flow.snapshot()
        self.assertEqual(restored['status'],'idle');self.assertEqual(restored['pipeline'],{})
        self.assertEqual(restored['previous_pipeline']['aborted'],prior['aborted'])
        self.assertNotIn('FileNotFoundError',restored['message'])
        flow.app.runtime_state.return_value['status']='REJECTED';flow.refresh()
        self.assertEqual(flow.snapshot()['status'],'failed','a fresh live rejection must remain visible')
        flow.app.runtime_state.return_value['status']='READY';flow.app.restored_pipeline_report_id=None;flow.refresh()
        self.assertEqual(flow.snapshot()['status'],'failed','a live quality failure must not be mistaken for a restored failure')

    def test_capture_retry_starts_at_zero_and_ignores_prior_batch_completion(self):
        flow,state=self.capture_progress_fixture()
        flow.app.batch={'completion':dict(passed=False,captured=32)}
        flow.app.active_capture.update(batch_id='retry/capture',captured=0,expected_count=12)
        flow.refresh();value=flow.snapshot()
        self.assertEqual(value['capture_batch_id'],'retry/capture')
        self.assertEqual(value['progress']['current'],0);self.assertEqual(value['progress']['total'],12)
        self.assertEqual(value['overall_progress']['current'],1)
        self.assertIsNone(value['failed_stage']);self.assertNotIn('data',value['outcomes'])
        # Neither a prior run nor a head batch may supply progress for this wrist failure.
        state.update(status='REJECTED');state['pipeline'].pop('subprogress')
        for run,phase in [('old-run','头部：采集'),('run','腕半场：采集')]:
            flow.app.active_capture['pipeline_run_id']=run;state['pipeline']['phase']=phase
            flow.refresh();value=flow.snapshot()
            self.assertIsNone(value['capture_batch_id']);self.assertIsNone(value['progress'])

    def test_original_C_terminal_report_keeps_run_id_after_live_cleanup(self):
        root=Path(__file__).resolve().parents[1]
        sys.path[:0]=[str(root/'calibration'),str(root/'calibration/infra'),
                      str(root/'calibration/examples/scripts/debug'),str(root/'calibration/tests')]
        from test_joint_head_workbench_pipeline import PipelineHost
        host=PipelineHost();seen=[]
        def fail_capture(report,label):
            seen.append(host.pipeline_run_id)
            raise ValueError('采集32/34后失败')
        host.capture_batch=fail_capture
        with self.assertRaisesRegex(ValueError,'32/34'):host.run(mode='full')
        self.assertIsNone(host.pipeline_run_id);self.assertFalse(host.pipeline_running)
        self.assertEqual(host.pipeline_report['run_id'],seen[0]);self.assertTrue(seen[0])
        self.assertIn('采集',host.pipeline_report['failed_phase'])
        self.assertIn('32/34',host.pipeline_report['aborted'])

    def test_original_C_capture_freezes_run_batch_and_expected_count_through_ack(self):
        root=Path(__file__).resolve().parents[1]
        sys.path[:0]=[str(root/'calibration'),str(root/'calibration/infra'),
                      str(root/'calibration/examples/scripts/debug'),str(root/'calibration/tests')]
        from test_joint_head_workbench_capture_flow import CaptureFlowTests
        legacy=CaptureFlowTests();legacy.setUp();self.addCleanup(legacy.doCleanups)
        c=legacy.app;c.pipeline_run_id='capture-run';original=c.control_request;initial=[]
        def gateway(action,body=None,**kwargs):
            if action=='motion-confirm':initial.append(deepcopy(c.active_capture))
            return original(action,body,**kwargs)
        c.control_request=gateway
        with patch('arx_env.joint_head_workbench_capture.capture_sample',side_effect=legacy.sample):
            legacy.confirm()
        self.assertEqual(c.status,'READY',c.reason)
        c.queue=[]
        self.assertEqual(initial[0]['captured'],0);self.assertEqual(c.active_capture['captured'],1)
        for value in (initial[0],c.active_capture):
            self.assertEqual(value['pipeline_run_id'],'capture-run')
            self.assertEqual(value['expected_count'],1);self.assertEqual(value['kind'],'head')
            self.assertEqual(value['batch_id'],c.active_capture['batch_id'])

    def test_independent_batch_stages_do_not_reuse_old_pipeline_completion(self):
        flow=self.calibration();flow.app.lock=threading.RLock();flow.app.queue=[{'q':[0.]*12,'capture':True}]
        flow.app.plan={'plan_complete':True};flow.app.workflow={'kind':'head','head_result_id':None}
        state={'status':'PLANNING','workflow':flow.app.workflow,'pipeline':{'installed':{'verified':True}},'counts':{}}
        flow.app.runtime_state.side_effect=lambda:state
        flow.operation={'id':'plan-1','kind':'head','action':'plan','message':'规划中'}
        flow.refresh();value=flow.snapshot()
        self.assertTrue(value['busy']);self.assertEqual(value['stage_id'],'head.plan')
        self.assertIsNone(value['overall_progress']);self.assertFalse(value['primary_action']['allowed'])
        state['status']='READY';flow.refresh();value=flow.snapshot()
        self.assertEqual(value['primary_action']['id'],'preview_batch');self.assertTrue(value['primary_action']['allowed'])
        flow.operation={'id':'batch/capture','kind':'head','action':'capture','message':'采集中'}
        flow.app.batch=None;state.update(status='CAPTURING',selected_batch=None)
        flow.refresh();self.assertEqual(flow.snapshot()['outcomes']['data'],'pending')
        flow.app.batch={'completion':{'passed':True}};state.update(status='READY',selected_batch='batch/capture')
        flow.refresh();value=flow.snapshot()
        self.assertEqual(value['outcomes']['data'],'valid');self.assertEqual(value['primary_action']['id'],'fit_batch')
        self.assertTrue(value['primary_action']['allowed'])
        self.assertEqual(value['calibration_tools']['fit_source_id'],'batch/capture')

    def test_expired_collection_review_refresh_is_cheap_and_replanning_is_explicit(self):
        workflow,state=self.collection();workflow.refresh()
        workflow.service.add_review.assert_not_called()
        workflow.refresh();workflow.service.add_review.assert_not_called()  # Read projection never grants a review.
        self.assertEqual(workflow.snapshot()['primary_action']['label'],'重新预览冻结计划')
        result=workflow.action('generate_plan',{})
        self.assertEqual(result['preview']['token'],'frozen');self.assertEqual(result['preview']['command_epoch'],5)
        workflow.playground.start.assert_not_called()
        workflow.action('generate_plan',{'replan':True})
        workflow.playground.start.assert_called_once_with({'action':'plan','speed_scale':.3})
        self.assertNotIn(('fake','collection'),workflow.service.reviews)

    def test_cold_business_points_to_warmup_and_missing_services_to_operations(self):
        for mode in ('fake','physical'):
            flow,state=self.collection();flow.device.environment=mode
            state.update(environment={'ready':False},plan={},can_execute=False,warmup={'status':'invalidated'})
            for services_ready in (True,False):
                with self.subTest(mode=mode,services_ready=services_ready):
                    flow.playground.initialization=lambda:dict(status='ready' if services_ready else 'stopped',
                        planner=dict(status='ready' if services_ready else 'stopped',model_initialized=False))
                    flow.refresh();value=flow.snapshot();prepare=value['collection']['actions']['prepare']
                    self.assertEqual(prepare['allowed'],services_ready)
                    reason=value['primary_action']['reason']
                    self.assertEqual(reason,value['collection']['actions']['plan']['reason'])
                    self.assertIn('初始化与预热' if services_ready else '服务运维',reason)
                    if services_ready:self.assertNotIn('运维',reason)
                    for action in ('generate_plan','collection_plan'):
                        with self.assertRaises(ValueError) as error:
                            flow.action(action,{})
                        self.assertEqual(str(error.exception),reason)
                    flow.playground.start.assert_not_called()

    def test_collection_binding_requires_this_run_instance_and_clears_on_acceptance(self):
        workflow,state=self.collection()
        state.update(action='execute',busy=True,progress={'execution_instance_id':'current-instance'},
            native_task=dict(execution_instance_id='current-instance',runtime_nonce='owner',source_clock_id='clock',
                             execution_timeline_digest='initialized-plan'))
        workflow.refresh();binding=workflow.snapshot()['execution_binding']
        self.assertEqual(binding,dict(execution_instance_id='current-instance',device_id='fixture-device',environment='fake',
            runtime_nonce='owner',source_clock_id='clock',execution_timeline_digest='initialized-plan',timeline_digest='frozen'))
        workflow.accepted({'output':'next-run'},'execute')
        self.assertIsNone(workflow.snapshot()['execution_binding'])
        for progress in ({},{'execution_instance_id':'next-instance'}):
            state['progress']=progress;workflow.refresh()
            self.assertIsNone(workflow.snapshot()['execution_binding'],'old native status cannot bind another job, even for the same plan')

    def test_collection_terminal_data_uses_sealed_manifest_not_missing_status_field(self):
        workflow,state=self.collection()
        state.update(status='failed',busy=False,action='execute')
        for recording,expected in (({},'未录制'),({'ready':True},'recording'),({'done':True},'pending'),
                ({'done':True,'manifest':{'validation':{'status':'passed'}}},'valid'),
                ({'done':True,'manifest':{'validation':{'status':'failed'}}},'invalid')):
            state['recording']=recording;workflow.refresh()
            self.assertEqual(workflow.snapshot()['outcomes']['data'],expected)
        state['recording'].update(error='camera source queue overflow')
        workflow.refresh()
        self.assertEqual(workflow.snapshot()['outcomes']['data'],
            {'status':'invalid','message':'camera source queue overflow'})

    def test_collection_terminal_title_does_not_reuse_a_running_claim(self):
        workflow,state=self.collection();state.update(action='execute',busy=False,physical_outcome='completed',task_outcome='success',
            recording={'done':True,'manifest':{'validation':{'status':'passed'}}},error='physical_export: duration differs',
            progress={'label':'视频已核验，等待 L3'})
        for status,label in [('failed','本次数采流程失败'),('stopped','本次数采流程已停止'),('completed','本次数采流程已完成')]:
            state['status']=status;workflow.refresh();value=workflow.snapshot()
            self.assertEqual(value['title'],label);self.assertEqual(value['message'],state['error'])
            self.assertEqual(value['outcomes']['motion'],'completed');self.assertEqual(value['outcomes']['data'],'valid')
            self.assertEqual(value['outcomes']['task'],'success')
        state.update(status='running',busy=True);workflow.refresh()
        self.assertEqual(workflow.snapshot()['title'],'视频已核验，等待 L3')

    def test_collection_failure_is_visible_only_for_current_failed_operation(self):
        workflow,state=self.collection()
        failure=dict(kind='motion_limit',message='左 J2 加速度超限',recovery='降低速度后重新规划',details={})
        state.update(status='failed',failure=failure,error='technical error')
        workflow.refresh();value=workflow.snapshot()
        self.assertEqual(value['failure'],failure)
        self.assertEqual(value['message'],'technical error')
        state.update(status='running',busy=True);workflow.refresh()
        self.assertIsNone(workflow.snapshot()['failure'])
        workflow.value['failure']=failure
        workflow.accepted({'output':'next'},'plan')
        self.assertIsNone(workflow.snapshot()['failure'])

    def test_collection_diagnostics_preserve_cached_evidence_without_full_frame_logs(self):
        workflow,state=self.collection();state.update(environment={'ready':False,'checks':[{'key':'camera','status':'failed','message':'source stale'}]},
            progress={'stages':[{'label':'观测','status':'failed','elapsed_s':12,'estimate_s':8,'internal':'omit'}]},
            recording={'manifest':{'validation':{'status':'failed','errors':['missing frame']},'command_count':4,'camera_frames':{'base_0':2},'files':{'large':'omit'}}},
            native_task={'task_outcome':'failed','execution_instance_id':'this-run','first_failure':{'reason':'contact','physics_step':4,'last_observation':{'full':'omit'}}},
            simulation={'status':'completed','scope':'L3_auxiliary_independent_target','task_success_evidence':False,'frames':['omit']})
        workflow.refresh();rows={row['label']:row['value'] for row in workflow.snapshot()['diagnostics']}
        self.assertEqual(rows['数采环境检查'][0]['message'],'source stale')
        self.assertEqual(rows['本轮录制封存']['validation']['errors'],['missing frame']);self.assertNotIn('files',rows['本轮录制封存'])
        self.assertEqual(rows['本轮原生任务回执']['first_failure'],{'reason':'contact','physics_step':4})
        self.assertFalse(rows['本轮辅助回放']['task_success_evidence']);self.assertNotIn('frames',rows['本轮辅助回放'])

    def test_collection_speed_preview_is_bound_and_reuses_original_retime_entry(self):
        for speed in (.3,1.,1.7):
            workflow,state=self.collection();workflow.refresh()
            result=workflow.action('generate_plan',{'speed_scale':speed})
            self.assertEqual(result['preview']['speed_scale'],speed);self.assertIn(f'{speed*100:g}%',result['summary'])
            workflow.refresh();self.assertEqual(workflow.snapshot()['speed_scale'],speed)
            workflow.playground.start.assert_not_called()
            workflow.action('execute',{'token':'frozen','speed_scale':speed})
            workflow.playground.start.assert_called_once_with({'action':'execute','speed_scale':speed})
        for value in (True,'0.5',float('nan'),float('inf'),.29,1.71):
            for action in ('collection_prepare','collection_plan','generate_plan','execute'):
                state.update(busy=False,status='ready');workflow.refresh()
                with self.assertRaisesRegex(ValueError,'30% 至 170%'):
                    workflow.action(action,{'speed_scale':value,'token':'frozen'})

    def test_collection_readonly_plan_and_replay_metadata_are_current_and_bounded(self):
        workflow,state=self.collection();plan=self.root/'plan.json';plan.write_text('{"timeline_digest":"frozen","ticks":[]}')
        image=self.root/'first.png';image.write_bytes(b'CPU readonly frame fixture')
        state['plan'].update(plan_path=str(plan),first_frame_path=str(image))
        source=self.root/'source-base_0.mp4';target=self.root/'sim-base_0.mp4'
        source.write_bytes(b'CPU source fixture');target.write_bytes(b'CPU target fixture')
        workflow.playground.media.side_effect=lambda name,version:{'video-0':source,'video-1':target}[name]
        state['videos']=[dict(url=f'/api/playground/media/video-{i}?v=job-1') for i in range(2)]
        workflow.refresh();view=workflow.snapshot()['trajectory']
        self.assertEqual(workflow.media(view['id']),plan);self.assertEqual(view['timeline_digest'],'frozen')
        self.assertEqual([row['media']['role'] for row in workflow.results()],['source','target'])
        self.assertTrue(all(row['media']['camera']=='base_0' for row in workflow.results()))
        state['plan']['plan_path']=str(self.root.parent/'outside-plan.json');workflow.refresh()
        self.assertIn('error',workflow.snapshot()['trajectory']);self.assertIsNone(workflow.media(view['id']))
        state.update(action='plan',plan={},videos=[]);workflow.refresh()
        self.assertNotIn('trajectory',workflow.snapshot());self.assertEqual(workflow.results(),[])

    def test_wrist_batch_head_replay_uses_only_its_frozen_head_dependency(self):
        legacy,workflow=self.legacy_library();head,wrist=legacy.pair();a=legacy.app
        batch=str(Path(wrist['file']).parent);path=a.output/batch
        dataset=json.loads((path/'dataset.json').read_text());dataset['frames']=[{'id':'frame','image':'head.png','q_reported_rad':[0.]*12}]
        (path/'dataset.json').write_text(json.dumps(dataset));(path/'head.png').write_bytes(b'CPU frame fixture')
        original_source=a.selected_batch;view=a.review_batch(batch)
        with patch.object(a,'historical_overlay',return_value={'image_url':view['frames'][0]['image_urls']['head']}) as overlay:
            workflow.action('select_result',{'result_id':wrist['id'],'camera':'head','overlay':True})
            overlay.assert_called_once_with(dict(batch_id=batch,frame_index=0,camera='head',result_id=head['id']))
        context=workflow.snapshot()['replay_context']
        self.assertEqual(context['batch_id'],batch);self.assertEqual(context['result_id'],wrist['id'])
        self.assertEqual(context['overlay_result_id'],head['id']);self.assertEqual(a.selected_batch,original_source)
        with patch.object(a,'wrist_head_id',return_value='unrelated-head'):
            with self.assertRaisesRegex(ValueError,'匹配'):
                workflow.action('select_result',{'result_id':wrist['id'],'camera':'head','overlay':True})
        with patch.object(a,'review_batch',return_value={**view,'results':[]}):
            with self.assertRaisesRegex(ValueError,'匹配'):
                workflow.action('select_result',{'result_id':wrist['id'],'camera':'head','overlay':True})

    def test_collection_media_is_current_and_cancel_uses_real_interrupt(self):
        workflow,state=self.collection();path=self.root/'recording.mp4';path.write_bytes(b'CPU video route fixture')
        state['videos']=[dict(label='头部回放',url='/api/playground/media/video-0?v=job-1')]
        workflow.playground.media.return_value=path;workflow.refresh()
        entry=workflow.results()[0];self.assertEqual(workflow.media(entry['id']),path)
        self.assertEqual(entry['media']['mime_type'],'video/mp4');self.assertIn('/api/result-media?',entry['media']['url'])
        workflow.playground.media.assert_called_with('video-0','job-1')
        state['videos']=[];workflow.refresh();self.assertIsNone(workflow.media(entry['id']))
        with workflow.playground.lock:state['busy']=True
        workflow.action('cancel',{})
        workflow.playground.start.assert_called_with({'action':'interrupt'})

    def test_collection_acceptance_closes_cache_gap_and_rejects_old_refresh(self):
        for action,stage in [('generate_plan','plan'),('execute','execute')]:
            with self.subTest(stage=stage):
                workflow,state=self.collection()
                if stage=='prepare':state.update(environment={'ready':False},can_execute=False,plan={'validated':False})
                entered=threading.Event();release=threading.Event();errors=[]
                def snapshot():
                    captured=deepcopy(state)
                    if threading.current_thread() is worker:
                        entered.set()
                        if not release.wait(2):raise TimeoutError('test refresh not released')
                    return captured
                def refresh():
                    try:workflow.refresh()
                    except Exception as error:errors.append(error)
                workflow.playground.snapshot.side_effect=snapshot
                worker=threading.Thread(target=refresh);worker.start()
                try:
                    self.assertTrue(entered.wait(1))
                    result=workflow.action(action,{'replan':True,'token':'frozen'})
                    value=workflow.snapshot()  # Must remain a cached, nonblocking read.
                    self.assertEqual(value['id'],result['output']);self.assertEqual(value['id'],'job-next')
                    self.assertEqual(value['stage_id'],'collection.'+stage)
                    self.assertEqual(value['status'],'running');self.assertTrue(value['busy'])
                    self.assertFalse(value['primary_action']['allowed'])
                    with self.assertRaisesRegex(ValueError,'不允许'):workflow.action('generate_plan',{})
                    self.assertNotIn(('fake','collection'),workflow.service.reviews)
                finally:release.set();worker.join(2)
                self.assertFalse(worker.is_alive());self.assertEqual(errors,[])
                self.assertEqual(workflow.snapshot(),value,'old snapshot overwrote accepted task')
                workflow.service.add_review.assert_not_called()
                state.update(busy=False,status='ready',can_execute=False)
                workflow.refresh()
                self.assertEqual(workflow.snapshot()['id'],'job-next')
                self.assertEqual(workflow.snapshot()['status'],'ready');self.assertFalse(workflow.snapshot()['busy'])

    def test_missing_collection_capability_and_interrupt_have_no_lifecycle_effects(self):
        root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root/'rule_based_workbench'))
        from playground import Playground
        from test_playground import Bridge
        for environment in ('fake','physical'):
            with self.subTest(environment=environment):
                bridge=Bridge();bridge.config['environment']=environment
                recover=Mock(side_effect=AssertionError('business restarted services'))
                playground=Playground(self.root/environment,bridge,lambda:{'status':'stopped'},recover_planner=recover)
                device=SimpleNamespace(id='fixture',environment=environment,snapshot=lambda business:{'command_epoch':1})
                manager=SimpleNamespace(lock=threading.Lock(),reviews={},add_review=Mock())
                flow=CollectionWorkflow(playground,device,manager)
                try:
                    flow.refresh()
                    self.assertFalse(flow.allowed('collection_prepare')['allowed'])
                    self.assertFalse(flow.snapshot()['collection']['actions']['prepare']['allowed'])
                    for action in ('generate_plan','collection_plan','collection_prepare'):
                        with self.assertRaises(ValueError):flow.action(action,{'replan':True})
                    with self.assertRaisesRegex(ValueError,'业务准入'):
                        playground.start({'action':'prepare','arms_supported':True})
                    playground.finish_interrupt()
                    self.assertIsNone(playground.thread);self.assertEqual(bridge.commands,[])
                    recover.assert_not_called()
                finally:flow.closed.set();flow.thread.join(2)

    def test_fixed_physical_preparation_uses_same_scoped_operation_without_connect(self):
        root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root/'rule_based_workbench'))
        from playground import Playground
        from test_playground import Bridge
        bridge=Bridge();bridge.connected=True;bridge.config['environment']='physical'
        service_state=dict(status='stopped',planner=dict(status='stopped',model_initialized=False))
        admitted=[]
        def recover(**kwargs):
            kwargs['launch'](lambda:service_state.update(status='ready',planner=dict(status='ready',model_initialized=False)))
            return dict(compatibility_path='legacy_fixed_profile',service_steps=[dict(role='planner',outcome='recovered',needs_warmup=True)])
        recovery=Mock(side_effect=recover)
        playground=Playground(self.root/'fixed',bridge,lambda:deepcopy(service_state),recover_planner=recovery)
        playground.operations_prepare_services=recovery
        device=SimpleNamespace(id='fixed',environment='physical',maintenance=True,runtime=SimpleNamespace(holder='collection'),
            snapshot=lambda business:dict(connected=True,feedback_healthy=True,stationary=True,automatic_enabled=False,sample_age_s=.01))
        flow=CollectionWorkflow(playground,device,SimpleNamespace(lock=threading.Lock(),reviews={},add_review=Mock()))
        def warmup(*args):service_state['planner']['model_initialized']=True;return {'model_initialized':True}
        pipeline=SimpleNamespace(prepare_environment=lambda config:[dict(key='cpu-dependencies',status='ready')],
            warmup=warmup,restore_plan=lambda *args:None)
        try:
            del playground.operations_prepare_services
            with self.assertRaisesRegex(ValueError,'未装配受控运维准备hook'):
                flow.operations_prepare(still_current=lambda:True,lifecycle_admission=lambda action:True)
            recovery.assert_not_called();playground.operations_prepare_services=recovery
            with patch.dict('sys.modules',playground_pipeline=pipeline):
                result=flow.operations_prepare(still_current=lambda:True,lifecycle_admission=lambda action:admitted.append(action) or True)
            self.assertTrue(result['services_ready']);self.assertFalse(result['ready'])
            self.assertFalse(flow.environment_status()['ready']);self.assertFalse(service_state['planner']['model_initialized'])
            recovery.assert_called_once();self.assertIn('start:collection',admitted)
            self.assertEqual(bridge.commands,[])
            with self.assertRaisesRegex(ValueError,'维护窗口'):
                flow.operations_prepare(still_current=lambda:False,lifecycle_admission=lambda action:True)
            self.assertEqual(recovery.call_count,1)
            current=[True];spawn=Mock()
            def stopped_hook(**kwargs):
                current[0]=False
                kwargs['launch'](spawn)
            playground.operations_prepare_services=stopped_hook
            with self.assertRaises(InterruptedError):
                flow.operations_prepare(still_current=lambda:current[0],lifecycle_admission=lambda action:True)
            spawn.assert_not_called();self.assertTrue(playground.managed_lifecycle)
            self.assertEqual(bridge.commands,[])
        finally:flow.closed.set();flow.thread.join(2)

    def test_old_monitor_failure_cannot_replace_new_task_message(self):
        workflow,state=self.collection();entered=threading.Event();release=threading.Event()
        def snapshot():
            if threading.current_thread() is worker:
                entered.set()
                if not release.wait(2):raise TimeoutError('test failure not released')
                workflow.closed.set()
                raise RuntimeError('old camera read failed')
            return deepcopy(state)
        workflow.playground.snapshot.side_effect=snapshot
        worker=threading.Thread(target=workflow.monitor);worker.start()
        try:
            self.assertTrue(entered.wait(1));workflow.action('generate_plan',{'replan':True})
            accepted=workflow.snapshot()
        finally:release.set();worker.join(2)
        self.assertFalse(worker.is_alive());self.assertEqual(workflow.snapshot(),accepted)
        self.assertNotIn('view_error',workflow.snapshot())

    def test_busy_collection_cannot_publish_review_for_retained_plan(self):
        workflow,state=self.collection();state['busy']=True
        workflow.refresh()
        self.assertFalse(workflow.snapshot()['primary_action']['allowed'])
        workflow.service.add_review.assert_not_called()

    def test_refresh_started_during_revocation_cannot_restore_review(self):
        for operation in ('invalidate','cancel'):
            with self.subTest(operation=operation):
                workflow,state=self.collection();errors=[]
                entered=threading.Event();release=threading.Event()
                sampled=threading.Event();publish=threading.Event()
                def backend(*args):
                    entered.set()
                    if not release.wait(2):raise TimeoutError('test revocation not released')
                    state.update(plan={'validated':False},can_execute=False)
                    return {'status':'ready'}
                def snapshot():
                    captured=deepcopy(state);sampled.set()
                    if not publish.wait(2):raise TimeoutError('test snapshot not released')
                    return captured
                def guarded(action):
                    try:action()
                    except Exception as error:errors.append(error)
                workflow.playground.invalidate_checkpoint.side_effect=backend
                workflow.playground.start.side_effect=backend
                workflow.playground.snapshot.side_effect=snapshot
                revoke=threading.Thread(target=guarded,args=(getattr(workflow,operation),))
                refresh=threading.Thread(target=guarded,args=(workflow.refresh,))
                revoke.start()
                try:
                    self.assertTrue(entered.wait(1));refresh.start();self.assertTrue(sampled.wait(1))
                    release.set();revoke.join(2)
                finally:
                    release.set();publish.set();revoke.join(2)
                    if refresh.ident is not None:refresh.join(2)
                self.assertFalse(revoke.is_alive());self.assertFalse(refresh.is_alive());self.assertEqual(errors,[])
                workflow.service.add_review.assert_not_called()
                self.assertIsNone(workflow.review_key)


if __name__=='__main__':unittest.main()
