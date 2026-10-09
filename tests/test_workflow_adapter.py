"""Review refresh is a read-only operation even while a pipeline awaits it."""
from pathlib import Path
from copy import deepcopy
import json
import sys
import tempfile
import threading
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
        workflow.value={'busy':False};workflow.catalog=[];workflow.review_key=None;workflow.media_files={}
        state=dict(environment={'ready':True},busy=False,can_execute=True,status='ready',action='plan',progress={},
            plan=dict(validated=True,timeline_digest='frozen',output='job-1',duration_s=20.,speed_scale=.3),videos=[])
        workflow.playground=SimpleNamespace(snapshot=Mock(side_effect=lambda:state),start=Mock(return_value={'status':'accepted'}),
            directory=self.root,media=Mock(),invalidate_checkpoint=Mock(),lock=threading.Lock(),state=state)
        workflow.device=SimpleNamespace(environment='fake',snapshot=Mock(return_value={'command_epoch':5}))
        workflow.service=SimpleNamespace(add_review=Mock(),lock=threading.Lock(),reviews={('fake','collection'):{'id':'old'}})
        return workflow,state

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


if __name__=='__main__':unittest.main()
