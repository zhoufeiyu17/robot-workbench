"""The one-click pipeline must actually fly both halves, in order, and stop on request.

The production button promises a complete calibration from nothing: plan the head capture,
fly it, solve it, then do the same for the wrist half against that head, then publish and
install the pair. What can go wrong is not arithmetic but orchestration - a step skipped, a
half solved against the wrong head, a result installed after a rejection, a stop request
ignored - so the order of calls is what is pinned here, on a host that records them and
runs the job inline instead of in a thread.
"""
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from joint_head_workbench import Workbench, PipelineStopped


class PipelineHost:
    """Just enough of the workbench to run the pipeline's own orchestration."""

    def __init__(self, **fields):
        self.calls = []
        self.events = []
        self.pipeline_report = {}
        self.status = 'READY'
        self.reason = None
        self.stop = threading.Event()
        self.lock = threading.RLock()
        self.workflow = {'kind': 'head', 'head_result_id': None}
        self.plan = {'motion_seed': 'x5-wrist-visible-seed-v7', 'min_clearance_m': 0.02}
        self.queue = [{'q': [0.] * 12, 'capture': True, 'name': 'w'}]
        self.frames = [{'valid_for_fit': True}]
        self.config = {'pipeline_auto_confirm': True}
        self.control_mode = 'hardware_sync'
        self.__dict__.update(fields)
        for name in ('pipeline_stop', 'capture_batch', 'newest_batch', 'run_calibration_pipeline'):
            setattr(self, name, getattr(Workbench, name).__get__(self))
        # Read the real retry rule rather than restating it, so the test fails if the rule
        # stops covering what it claims to.
        self.TRANSIENT_CAPTURE = Workbench.TRANSIENT_CAPTURE

    # --- host surface the pipeline leans on -----------------------------------------
    def idle(self):
        pass

    def robot_source(self):
        return 'simulated'

    def hardware_connected(self):
        return True

    def persist(self):
        self.events.append(('persist', None))

    def pipeline_progress(self, **fields):
        # Log the phase, then do what the real method does - update the report in place -
        # so the fixture cannot hide a report that the state and the job disagree about.
        self.events.append(('phase', fields.get('phase')))
        Workbench.pipeline_progress(self, **fields)

    def background(self, status, job):
        self.status = status
        self.stop.clear()
        self.events.append(('status', status))
        job()
        self.status = 'STOPPED' if self.stop.is_set() else 'READY'

    def action(self, path, body):
        self.calls.append((path, body))
        if path == '/api/workflow':
            self.workflow = dict(body)
        if path == '/api/auto-plan':
            self.events.append(('plan', 'head'))
        return {'installed': {'head_version': 'v000020', 'wrist_version': 'v000021'},
                'message': '已安装为当前有效标定：头部 ＋ 双腕一组'}

    def wait_for_step(self, timeout, label, settle=('READY', 'STOPPED')):
        self.events.append(('wait', label))

    def plan_wrists(self):
        self.events.append(('plan', 'wrist'))

    def preview_calibration(self):
        self.events.append(('preview', self.workflow['kind']))
        return {'token': 'token-%s' % self.workflow['kind']}

    def confirm_calibration(self, body):
        self.events.append(('confirm', self.workflow['kind']))
        return {'batch_id':'batch-hardware-'+self.workflow['kind']+'01/capture'}

    def counts(self):
        return {'waypoints': 35, 'capture_points': 34, 'transition_points': 1,
                'captured': 34, 'valid': 34, 'failed': 0, 'unclassified': 0}

    def history(self):
        return [{'id': 'batch-hardware-head01/capture', 'kind': 'head', 'status': 'completed',
                 'updated_at': 10, 'completion': {'passed': True}},
                {'id': 'batch-hardware-head00/capture', 'kind': 'head', 'status': 'completed',
                 'updated_at': 9, 'completion': {'passed': False}},
                {'id': 'batch-hardware-wrist01/capture', 'kind': 'wrist', 'status': 'completed',
                 'updated_at': 20, 'completion': {'passed': True}}]

    def newest_head(self, marker_frozen=False):
        self.events.append(('newest_head', marker_frozen))
        return {'id': 'head-result-1', 'batch': 'batch-hardware-head01/capture', 'file': 'fit-a.json'}

    def newest_wrist(self):
        return {'id': 'wrist-result-1', 'pair_status': 'complete',
                'left': {'holdout': {'rms_px': 1.4}}, 'right': {'holdout': {'rms_px': 1.3}}}

    def pipeline_head_metrics(self, row):
        return {'holdout_marker_rms_px': 1.124, 'holdout_mesh_coverage': 0.851,
                'holdout_mesh_px': 5.021, 'marker_mounts_frozen': [], 'mount_mode': 'free'}

    def pipeline_wrist_metrics(self, row):
        return {side: {'holdout_rms_px': row[side]['holdout']['rms_px'], 'fit_rms_px': 1.0,
                       'marker_mount_delta': None} for side in ('left', 'right')}

    def wrist_acceptance_px(self):
        return 2.5

    def run(self, **body):
        return self.run_calibration_pipeline(body)


class FullModeTest(unittest.TestCase):
    def test_wrist_capture_only_completes_without_solving_installing_or_repeating(self):
        host=PipelineHost();head=host.newest_head()
        host.calibration_results=lambda:[head]
        host.require_calibration_environment=lambda row:None
        host.require_joint_head_dependency=lambda row:None
        host.run(mode='wrist',head_result_id=head['id'],capture_only=True)
        self.assertEqual([p for p,_ in host.calls],['/api/workflow'])
        self.assertEqual([e for e in host.events if e[0]=='confirm'],[('confirm','wrist')])
        report=host.pipeline_report
        self.assertTrue(report['capture_only']);self.assertTrue(report['capture_completed'])
        self.assertEqual(report['wrist_capture'],'batch-hardware-wrist01/capture')
        self.assertEqual(report['counts']['captured'],34)
        self.assertIsNone(report['aborted']);self.assertNotIn('installed',report)
        self.assertFalse(host.pipeline_running);self.assertIsNone(host.pipeline_review)

    def test_capture_only_rejects_invalid_options_before_work(self):
        for mode,value in [('wrist',1),('wrist','true'),('wrist',None),('full',True),('refit',False)]:
            host=PipelineHost()
            with self.subTest(mode=mode,value=value),self.assertRaisesRegex(ValueError,'capture_only'):
                host.run(mode=mode,capture_only=value)
            self.assertFalse(host.calls)

    def test_capture_only_failure_is_not_reported_complete(self):
        host=PipelineHost();head=host.newest_head()
        host.calibration_results=lambda:[head]
        host.require_calibration_environment=lambda row:None
        host.require_joint_head_dependency=lambda row:None
        host.capture_batch=lambda *args:(_ for _ in ()).throw(ValueError('capture fault'))
        with self.assertRaisesRegex(ValueError,'capture fault'):
            host.run(mode='wrist',head_result_id=head['id'],capture_only=True)
        self.assertFalse(host.pipeline_report['capture_completed'])
        self.assertNotIn('installed',host.pipeline_report)
        self.assertIn('capture fault',host.pipeline_report['aborted'])

    def test_resume_wrist_reuses_explicit_head_without_head_capture_or_refit(self):
        host=PipelineHost()
        head=host.newest_head()
        host.calibration_results=lambda:[head]
        checked=[]
        host.require_calibration_environment=lambda row:checked.append(('source',row['id']))
        host.require_joint_head_dependency=lambda row:checked.append(('eligible',row['id']))
        host.run(mode='wrist',head_result_id=head['id'])
        paths=[p for p,_ in host.calls]
        self.assertEqual(paths,['/api/workflow','/api/fit-source','/api/wrist-fit','/api/calibration-install'])
        self.assertEqual(host.calls[0][1],{'kind':'wrist','head_result_id':head['id']})
        self.assertEqual([e for e in host.events if e[0]=='preview'],[('preview','wrist')])
        self.assertEqual(checked,[('source',head['id']),('eligible',head['id'])])
        self.assertTrue(host.pipeline_report['steps'][0]['reused'])
        self.assertEqual(host.pipeline_report['resumed_from'],'wrist')

    def test_resume_wrist_requires_valid_explicit_dependency_before_work(self):
        host=PipelineHost();host.calibration_results=lambda:[]
        with self.assertRaisesRegex(ValueError,'头部结果'):host.run(mode='wrist',head_result_id='missing')
        self.assertFalse(host.calls)
        head=host.newest_head();host.calibration_results=lambda:[head]
        host.require_calibration_environment=lambda row:None
        host.require_joint_head_dependency=lambda row:(_ for _ in ()).throw(ValueError('head rejected'))
        with self.assertRaisesRegex(ValueError,'head rejected'):host.run(mode='wrist',head_result_id=head['id'])
        self.assertFalse(host.calls)

    def test_both_halves_are_planned_flown_solved_and_the_pair_is_installed(self):
        host = PipelineHost()
        result = host.run(mode='full')
        self.assertEqual(result['mode'], 'full')
        paths = [path for path, _ in host.calls]
        self.assertEqual(paths, ['/api/workflow', '/api/auto-plan',
                                 '/api/fit-source', '/api/fit',
                                 '/api/workflow',
                                 '/api/fit-source', '/api/wrist-fit',
                                 '/api/calibration-install'],
                         'plan the head, fly it, solve it, then the wrist half, then install')
        self.assertEqual(host.calls[0][1], {'kind': 'head'})
        self.assertEqual(host.calls[4][1], {'kind': 'wrist', 'head_result_id': 'head-result-1'},
                         'the wrist half must depend on the head this run just solved')
        # Nothing pre-existing is reused: the head capture is the one just flown, and the
        # failed batch in the history is not the one solved.
        self.assertEqual(host.calls[2][1]['id'], 'batch-hardware-head01/capture')
        self.assertEqual(host.calls[5][1]['id'], 'batch-hardware-wrist01/capture')
        self.assertEqual([event for event in host.events if event[0] == 'preview'],
                         [('preview', 'head'), ('preview', 'wrist')])
        self.assertEqual(host.status, 'READY')

        report = host.pipeline_report
        self.assertEqual(report['mode'], 'full')
        self.assertEqual([step['step'] for step in report['steps']],
                         ['plan', 'capture', 'solve', 'wrist'])
        self.assertEqual(report['steps'][0]['capture_points'], 34)
        self.assertEqual(report['steps'][1]['batch'], 'batch-hardware-head01/capture')
        self.assertEqual(report['steps'][3]['acceptance'], {'max_holdout_rms_px': 2.5})
        self.assertEqual(report['best_pair']['worst_holdout_rms_px'], 1.4)
        self.assertEqual(report['installed']['head_version'], 'v000020')
        self.assertIsNone(report['aborted'], 'a clean full run has nothing to warn about')

    def test_a_failed_run_says_what_failed_and_where(self):
        # state.reason prefers the hardware link's own error, so a pipeline that dies must
        # keep its own account of the phase and the exception.
        host = PipelineHost()

        def preview():
            raise ValueError('请先生成标定计划')
        host.preview_calibration = preview
        with self.assertRaisesRegex(ValueError, '请先生成标定计划'):
            host.run(mode='full')
        report = host.pipeline_report
        self.assertIn('请先生成标定计划', report['aborted'])
        self.assertIn('ValueError', report['aborted'])
        self.assertEqual(report['failed_phase'], '头部：审阅与采集（34 个采样点）')
        self.assertEqual(host.calls[-1][0], '/api/auto-plan',
                         'it stopped before the head solve, and installed nothing')

    def test_the_run_needs_an_explicit_unattended_capture_authorisation(self):
        host = PipelineHost()
        host.robot_source=lambda:'physical'
        with self.assertRaisesRegex(ValueError, 'pipeline_auto_confirm'):
            host.run(mode='full')
        self.assertEqual(host.calls, [], 'nothing may move before the authorisation is read')

    def test_a_stop_during_a_step_ends_the_run_without_installing_anything(self):
        host = PipelineHost()

        def wait(timeout, label, settle=('READY', 'STOPPED')):
            host.events.append(('wait', label))
            host.stop.set()
        host.wait_for_step = wait
        host.run(mode='full')
        self.assertTrue(host.pipeline_report['stopped'])
        self.assertIn('停止', host.pipeline_report['aborted'])
        self.assertEqual([path for path, _ in host.calls if path == '/api/calibration-install'], [],
                         'a stopped run must not publish a group')
        self.assertEqual(host.pipeline_report['steps'][0]['step'], 'plan',
                         'the step that finished before the stop is still reported')

    def test_a_capture_that_dies_on_the_clock_is_retried_rather_than_failing_the_run(self):
        host = PipelineHost()
        attempts = []

        def wait(timeout, label, settle=('READY', 'STOPPED')):
            attempts.append(label)
            # Only the capture is retried: planning and solving fail for their own reasons.
            if label == '头部采集' and len(attempts) == 2:
                raise ValueError('头部采集失败：feedback bracket missing')
        host.wait_for_step = wait
        with patch.object(host.stop,'wait',return_value=False):
            host.run(mode='full')
        self.assertEqual(attempts, ['头部规划', '头部采集', '头部采集', '头部求解',
                                    '腕半场采集', '腕半场求解'],
                         'the failed head capture is flown again, and only it')
        self.assertEqual(len(host.pipeline_report['capture_retries']), 1)
        self.assertEqual(host.pipeline_report['capture_retries'][0]['batch'], '头部')
        self.assertEqual(host.pipeline_report['installed']['head_version'], 'v000020')

    def test_a_renderer_drop_is_retried_and_recorded(self):
        # The capture talks to the renderer through a tunnel; a reset says nothing about the
        # plan, so the batch is flown again, and the failure is in the report even if the
        # run later fails somewhere else entirely.
        host = PipelineHost()
        calls = []

        def wait(timeout, label, settle=('READY', 'STOPPED')):
            calls.append(label)
            if label == '头部采集' and len(calls) == 2:
                raise ValueError('DSW unavailable: [Errno 104] Connection reset by peer')
        host.wait_for_step = wait
        with patch.object(host.stop,'wait',return_value=False):
            host.run(mode='full')
        retries = host.pipeline_report['capture_retries']
        self.assertEqual(len(retries), 1)
        self.assertIn('DSW unavailable', retries[0]['reason'])
        self.assertEqual(host.pipeline_report['installed']['head_version'], 'v000020')

    def test_a_renderer_or_camera_503_is_retried(self):
        # Seen live: the preview died on urllib's HTTPError, which is an OSError rather than
        # a ValueError, so it slipped past the retry and ended the run on the first attempt.
        import urllib.error
        host = PipelineHost()
        calls = []

        def preview():
            calls.append('preview')
            if len(calls) == 1:
                raise urllib.error.HTTPError('http://127.0.0.1:8118/x', 503, 'Service Unavailable', {}, None)
            return {'token': 't'}
        host.preview_calibration = preview
        with patch.object(host.stop,'wait',return_value=False):
            host.run(mode='full')
        retries = host.pipeline_report['capture_retries']
        self.assertEqual(len(retries), 1)
        self.assertIn('503', retries[0]['reason'])
        self.assertEqual(host.pipeline_report['installed']['head_version'], 'v000020')

    def test_a_preview_failure_is_not_a_clock_race_and_is_not_retried(self):
        host = PipelineHost()

        def wait(timeout, label, settle=('READY', 'STOPPED')):
            raise ValueError('请先生成标定计划')
        host.wait_for_step = wait
        with self.assertRaisesRegex(ValueError, '请先生成标定计划'):
            host.run(mode='full')


class PhysicalReviewTest(unittest.TestCase):
    def test_physical_capture_waits_for_review_and_never_reflies_failure(self):
        host=PipelineHost(config={})
        host.robot_source=lambda:'physical'
        ready=threading.Event();errors=[]
        original=host.pipeline_progress
        def progress(**fields):
            original(**fields);ready.set()
        host.pipeline_progress=progress
        host.wait_for_step=lambda *args: (_ for _ in ()).throw(ValueError('采集 feedback bracket missing'))
        def run():
            try:host.capture_batch({},'头部')
            except Exception as exc:errors.append(exc)
        worker=threading.Thread(target=run);worker.start()
        try:
            self.assertTrue(ready.wait(2))
            self.assertFalse(any(e[0] in ('preview','confirm') for e in host.events))
            host.pipeline_review_result={'batch_id':'batch-hardware-head01/capture'}
            worker.join(2)
            self.assertFalse(worker.is_alive())
            self.assertEqual(len(errors),1)
            self.assertIsNone(host.pipeline_review)
            self.assertFalse(any(e[0]=='confirm' for e in host.events))
        finally:
            host.stop.set();worker.join(2)

    def test_stop_while_waiting_cannot_clear_stop_or_capture(self):
        host=PipelineHost(config={});host.robot_source=lambda:'physical'
        host.pipeline_progress=lambda **fields:host.stop.set()
        with self.assertRaises(PipelineStopped):host.capture_batch({},'头部')
        self.assertTrue(host.stop.is_set())
        self.assertIsNone(host.pipeline_review)
        self.assertFalse(any(e[0]=='confirm' for e in host.events))

    def test_exact_new_capture_must_pass_not_an_older_completed_batch(self):
        host=PipelineHost()
        host.confirm_calibration=lambda body:{'batch_id':'new-incomplete/capture'}
        with self.assertRaisesRegex(ValueError,'本次采集'):
            host.capture_batch({},'头部',attempts=1)

    def test_expired_preview_requires_fresh_review_without_confirming_motion(self):
        host=PipelineHost(config={})
        host.pipeline_running=True;host.pipeline_run_id='run-1'
        host.calibration_previews={'expired':{'expires':0,'workflow':host.workflow,'pipeline_run_id':'run-1'}}
        with self.assertRaisesRegex(ValueError,'过期'):
            Workbench.confirm_calibration(host,{'token':'expired','confirmation':'START_REVIEWED_SINGLE_ARM','physical_clearance':True})
        self.assertEqual(host.calibration_previews,{})
        self.assertFalse(any(e[0]=='confirm' for e in host.events))

    def test_concurrent_commands_rejected_while_subjob_status_ready(self):
        host=SimpleNamespace(mode_lock=threading.RLock(),pipeline_running=True,pipeline_thread=-1,
                             pipeline_review=None,status='READY')
        for path in ('/api/calibration-pipeline','/api/ack','/api/workflow','/api/control/motion-confirm'):
            with self.subTest(path=path),self.assertRaisesRegex(ValueError,'流水线正在运行'):
                Workbench.action(host,path,{})

    def test_physical_source_requires_explicit_owner_and_matching_result(self):
        host=SimpleNamespace(control_mode='hardware_sync',hardware_feedback={'owner':{}},
                             active_environment=lambda:'hardware')
        host.robot_source=lambda:Workbench.robot_source(host)
        host.calibration_asset_matches=lambda row:Workbench.calibration_asset_matches(host,row)
        for flag,source in ((None,'unknown'),(True,'simulated'),(False,'physical')):
            host.hardware_feedback['owner']={'simulation_only':flag}
            self.assertEqual(host.robot_source(),source)
        for row in ({'environment':'hardware'}, {'environment':'hardware','robot_source':'simulated','simulation_only':True}):
            with self.assertRaisesRegex(ValueError,'physical'):
                Workbench.require_calibration_environment(host,row)
        Workbench.require_calibration_environment(host,{'environment':'hardware','robot_source':'physical','simulation_only':False})

    def test_stop_after_wrist_solve_prevents_install(self):
        host=PipelineHost()
        original=host.pipeline_wrist_metrics
        def metrics(row):
            host.stop.set();return original(row)
        host.pipeline_wrist_metrics=metrics
        host.run(mode='full')
        self.assertTrue(host.pipeline_report['stopped'])
        self.assertNotIn('/api/calibration-install',[path for path,_ in host.calls])
        self.assertFalse(host.pipeline_running)


class StoredReportTest(unittest.TestCase):
    """The last run must survive a restart, and must never break the state write."""

    def test_a_report_is_stored_as_json_safe_data(self):
        host = SimpleNamespace(pipeline_report={'mode': 'full', 'steps': [{'step': 'plan', 'waypoints': 35}]})
        stored = Workbench.stored_pipeline_report(host)
        self.assertEqual(stored['mode'], 'full')
        self.assertEqual(stored['steps'][0]['waypoints'], 35)
        self.assertIsNot(stored, host.pipeline_report, 'the stored copy is a plain-data snapshot')

    def test_a_step_boundary_writes_the_report_through(self):
        host = SimpleNamespace(pipeline_report={}, persist=lambda: host.writes.append(1), writes=[],
                               lock=threading.RLock())
        Workbench.pipeline_progress(host, phase='头部：规划')
        self.assertEqual(host.writes, [1], 'the phase update is persisted, not just held in memory')
        self.assertEqual(host.pipeline_report['phase'], '头部：规划')

    def test_an_unstorable_report_is_dropped_rather_than_breaking_the_state_write(self):
        host = SimpleNamespace(pipeline_report={'mode': 'full', 'nan': float('nan')})
        self.assertIsNone(Workbench.stored_pipeline_report(host))
        self.assertIsNone(Workbench.stored_pipeline_report(SimpleNamespace(pipeline_report={})))
        self.assertIsNone(Workbench.stored_pipeline_report(SimpleNamespace()))


class SubprogressTest(unittest.TestCase):
    def test_measured_progress_is_monotonic_and_cannot_complete_stage(self):
        host=SimpleNamespace(pipeline_running=True,pipeline_report={},lock=threading.RLock())
        update=lambda stage,fraction:Workbench.pipeline_subprogress(host,stage,{'fraction':fraction,'label':'work'})
        update(4,.75);update(4,.5)
        self.assertEqual(host.pipeline_report['subprogress']['fraction'],.75)
        update(4,1)
        self.assertEqual(host.pipeline_report['subprogress']['fraction'],.99)
        update(5,.1)
        self.assertEqual(host.pipeline_report['subprogress']['fraction'],.1)
        update(5,float('nan'))
        self.assertEqual(host.pipeline_report['subprogress']['fraction'],.1)

    def test_capture_counts_ignore_previous_selected_batch(self):
        host=SimpleNamespace(status='CAPTURING',queue=[{}]*32,
            frames=[{'valid_for_fit':True}]*27,active_capture={'batch_id':'new/capture','captured':0})
        self.assertEqual(Workbench.counts(host)['captured'],0)
        host.active_capture['captured']=5
        self.assertEqual(Workbench.counts(host)['captured'],5)
        host.status='READY'
        self.assertEqual(Workbench.counts(host)['captured'],27)

    def test_capture_failure_keeps_actual_completed_frames(self):
        host=SimpleNamespace(status='REJECTED',reason='feedback fault',pipeline_running=True,
            pipeline_report={},lock=threading.RLock(),counts=lambda:{'captured':24,'capture_points':32})
        host.pipeline_subprogress=lambda stage,value:Workbench.pipeline_subprogress(host,stage,value)
        with self.assertRaisesRegex(ValueError,'feedback fault'):
            Workbench.wait_for_step(host,10,'腕半场采集')
        self.assertEqual(host.pipeline_report['subprogress'],
            {'stage':4,'fraction':.75,'label':'已采集图像','completed':24,'total':32})


class RefitModeTest(unittest.TestCase):
    def test_refit_needs_a_selected_capture_and_keeps_the_mount_gate(self):
        host = PipelineHost()
        host.config = {'pipeline_auto_confirm': True}
        host.dataset = None
        del host.__dict__['frames']
        host.frames = []
        with self.assertRaisesRegex(ValueError, '重解模式'):
            host.run(mode='refit', rounds=2)

    def test_an_unknown_mode_is_refused(self):
        host = PipelineHost()
        with self.assertRaisesRegex(ValueError, '未知的流水线模式'):
            host.run(mode='guess')

    def test_rounds_without_a_mode_still_means_the_refit_experiment(self):
        # `rounds` was the only knob before the full mode existed, so sending it must keep
        # selecting the experiment rather than silently planning a new capture.
        host = PipelineHost()
        host.dataset = Path('/tmp/nothing')
        host.frames = [{'valid_for_fit': True}]
        with self.assertRaisesRegex(ValueError, '重解的头部证据'):
            host.run(rounds=2)
        self.assertEqual([path for path, _ in host.calls], [],
                         'the refit checks run before the job starts, so nothing has moved')


if __name__ == '__main__':
    unittest.main()
