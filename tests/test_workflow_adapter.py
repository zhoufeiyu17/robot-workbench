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
from robot_workbench.workflows import CalibrationWorkflow,CollectionWorkflow,calibration_stage


class WorkflowAdapterTest(unittest.TestCase):
    def setUp(self):
        folder=tempfile.TemporaryDirectory();self.addCleanup(folder.cleanup);self.root=Path(folder.name).resolve()

    def calibration(self):
        workflow=CalibrationWorkflow.__new__(CalibrationWorkflow)
        workflow.lock=threading.Lock();workflow.install_lock=threading.Lock();workflow.value={'busy':False};workflow.active=False
        workflow.preview_key=None;workflow.installed_id=None;workflow.last_catalog=-1e10
        workflow.catalog=[];workflow.replay=[];workflow.replay_context=None;workflow.media_files={}
        self.batch=self.root/'batch/capture';self.batch.mkdir(parents=True)
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
        def start(payload):
            if payload['action']=='interrupt':return {'status':'stopping'}
            state.update(output='job-next',busy=True,can_execute=False,status='running',action=payload['action'])
            return dict(status='accepted',action=payload['action'],output=state['output'])
        workflow.playground=SimpleNamespace(snapshot=Mock(side_effect=lambda:deepcopy(state)),start=Mock(side_effect=start),
            directory=self.root,media=Mock(),invalidate_checkpoint=Mock(),lock=threading.Lock(),state=state)
        workflow.device=SimpleNamespace(environment='fake',snapshot=Mock(return_value={'command_epoch':5}))
        workflow.service=SimpleNamespace(add_review=Mock(),lock=threading.Lock(),reviews={('fake','collection'):{'id':'old'}})
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
        workflow=CalibrationWorkflow.__new__(CalibrationWorkflow)
        workflow.lock=threading.Lock()
        workflow.value={'busy':True}
        workflow.app=SimpleNamespace(pipeline_review={'kind':'wrist'},
            preview_calibration=Mock(return_value={'token':'new-token','samples':32,'total_motion_duration_s':90.}),
            action=Mock(side_effect=AssertionError('must not restart full pipeline')))
        self.assertTrue(workflow.allowed('generate_plan')['allowed'])
        result=workflow.action('generate_plan',{})
        self.assertEqual(result['preview']['token'],'new-token')
        workflow.app.preview_calibration.assert_called_once_with()
        workflow.app.action.assert_not_called()
        workflow.app.pipeline_review=None
        self.assertFalse(workflow.allowed('generate_plan')['allowed'])
        with self.assertRaises(ValueError):workflow.action('generate_plan',{})

    def test_running_calibration_description_tracks_phase_not_initial_guidance(self):
        workflow=self.calibration()
        for phase,expected in [('头部采集','正在逐点采集头部'),('腕半场采集','正在逐点采集双腕'),
                               ('头部求解','求解并验收头部'),('腕半场求解','贴纸观测求解')]:
            workflow.app.runtime_state.return_value={'status':'CAPTURING','pipeline':{'running':True,'phase':phase}}
            workflow.refresh();self.assertIn(expected,workflow.snapshot()['message'])
            self.assertNotIn('先生成计划',workflow.snapshot()['message'])
        workflow.app.runtime_state.return_value['pipeline']['aborted']='本次真实失败原因'
        workflow.refresh();self.assertEqual(workflow.snapshot()['message'],'本次真实失败原因')

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
        self.assertIn('metrics_rectified_px',catalog[head['id']]['details'])
        self.assertEqual(catalog[wrist['id']]['details']['head_result_id'],head['id'])
        self.assertEqual(workflow.snapshot()['calibration_tools']['installed']['head_id'],head['id'])
        self.assertTrue(workflow.snapshot()['calibration_tools']['installed']['verified'])
        self.assertEqual(workflow.download(head['id']),legacy.app.calibration_store_record(head['id']))
        self.assertIn('/api/calibration-file?',catalog[head['id']]['download_url'])
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
        workflow.service.add_review.assert_called_once()
        workflow.refresh();workflow.service.add_review.assert_called_once()  # No polling-based TTL renewal.
        self.assertEqual(workflow.snapshot()['primary_action']['label'],'重新预览冻结计划')
        result=workflow.action('generate_plan',{})
        self.assertEqual(result['preview']['token'],'frozen');self.assertEqual(result['preview']['command_epoch'],5)
        workflow.playground.start.assert_not_called()
        workflow.action('generate_plan',{'replan':True})
        workflow.playground.start.assert_called_once_with({'action':'plan'})
        self.assertNotIn(('fake','collection'),workflow.service.reviews)

    def test_collection_media_is_current_and_cancel_uses_real_interrupt(self):
        workflow,state=self.collection();path=self.root/'recording.mp4';path.write_bytes(b'CPU video route fixture')
        state['videos']=[dict(label='头部回放',url='/api/playground/media/video-0?v=job-1')]
        workflow.playground.media.return_value=path;workflow.refresh()
        entry=workflow.results()[0];self.assertEqual(workflow.media(entry['id']),path)
        self.assertEqual(entry['media']['mime_type'],'video/mp4');self.assertIn('/api/result-media?',entry['media']['url'])
        workflow.playground.media.assert_called_with('video-0','job-1')
        state['videos']=[];workflow.refresh();self.assertIsNone(workflow.media(entry['id']))
        workflow.value['busy']=True;workflow.action('cancel',{})
        workflow.playground.start.assert_called_with({'action':'interrupt'})

    def test_collection_acceptance_closes_cache_gap_and_rejects_old_refresh(self):
        for action,stage in [('generate_plan','prepare'),('generate_plan','plan'),('execute','execute')]:
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
