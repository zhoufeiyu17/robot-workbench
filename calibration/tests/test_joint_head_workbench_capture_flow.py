"""Offline capture orchestration: persistent samples precede ACK, never robot IO."""
import copy
import json
from pathlib import Path
import tempfile
import itertools
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch
from urllib.error import HTTPError

import test_joint_head_workbench_live_bridge as bridge_fixture
from joint_head_workbench import Workbench, transport_environment


class CaptureFlowTests(unittest.TestCase):
    def test_transport_environment_requires_explicit_fake_configuration(self):
        self.assertEqual(transport_environment({}),'physical')
        self.assertEqual(transport_environment({'simulation_only':True}),'physical')
        self.assertEqual(transport_environment({'environment':'fake','simulation_only':True,'robot_source':'simulated'}),'fake')
        for config in ({'environment':'fake'},{'environment':'fake','simulation_only':True,'robot_source':'physical'},
                       {'environment':'simulation'}):
            with self.assertRaises(ValueError):transport_environment(config)
        for config in ({'robot_id':'physical-a1','calibration_root':'/tmp/fake'},
                       {'robot_id':'fake-a1','calibration_root':'/var/lib/anyverse/calibration/hardware'}):
            with self.assertRaisesRegex(ValueError,'独立'):
                Workbench({'environment':'fake','simulation_only':True,**config},'/unused')

    def setUp(self):
        bridge_fixture.BridgeTests.setUp(self)
        a=self.app;a.output=a.output.resolve();a.control_mode='hardware_sync'
        a.store=SimpleNamespace(robot_id='arx6');a.geometry_id=a.plan_geometry_id='geometry'
        a.workflow={'kind':'head','head_result_id':None};a.plan=None;a.calibration_previews={}
        a.queue=[{'id':'pose-0','q':[0.]*12,'speeds':[.1]*12,'capture':True,'split':'fit','arm':'left'},
                 {'id':'return','q':[0.]*12,'speeds':[.1]*12,'capture':False}]
        a.queue=a.calibration_waypoints(a.queue)
        a.hardware_history=[];a.hardware_connected=lambda:True;a.control_available=lambda:True
        a.control_status={'owner':{'capabilities':['calibration_capture_ack_v1']}}
        a.require_real_control=lambda:a.control_status
        a.persist=Mock();a.load_batch=Mock();a.background=lambda status,fn:fn()
        class ImmediateThread:
            def __init__(self,target,**kwargs):self.target=target
            def start(self):self.target()
        worker=patch('joint_head_workbench.threading.Thread',ImmediateThread);worker.start();self.addCleanup(worker.stop)
        a.remote_request=Mock(side_effect=AssertionError('hardware capture must not call simulation'))
        self.names=['head'];self.calls=[];self.acknowledged=False
        self.intrinsics={n:{'serial':n+'-serial','rectified':True,'coefficients':[0.]*5} for n in ('head','left','right')}
        self.cameras={n:SimpleNamespace(read_calibration_frame=lambda name=n:(b'raw',{'serial':name+'-serial'})) for n in self.intrinsics}
        a.hardware_calibration_cameras=lambda:({n:self.cameras[n] for n in self.names},{n:self.intrinsics[n] for n in self.names})
        a.control_request=self.gateway

    def gateway(self,action,body=None,**kwargs):
        self.calls.append((action,copy.deepcopy(body)))
        if action=='trajectory-preview':
            self.identity=next(w['sample_id'] for w in body['waypoints'] if w['capture'])
            return {'token':'review-token','owner_pid':123,'command_epoch':4,'total_motion_duration_s':1.}
        if action=='state':return self.owner_state()
        if action=='capture-ack':
            dataset=next(self.app.output.glob('batch-hardware-*/capture'))
            manifest=json.loads((dataset/'dataset.json').read_text())
            row=manifest['frames'][0]
            self.assertTrue((dataset/(row['id']+'.json')).is_file())
            self.assertTrue((dataset/'camera.json').is_file())
            self.assertTrue(all((dataset/name).is_file() for name in row['camera_images'].values()))
            self.assertEqual(body,{'owner_pid':123,'command_epoch':4,'sample_id':self.identity})
            self.acknowledged=True
        return {'ok':True}

    def owner_state(self,**changes):
        owner={'pid':123,'command_epoch':4,'state':'READY','feedback_healthy':True,'capabilities':self.app.control_status['owner']['capabilities'],
               'automatic_enabled':not self.acknowledged,
               'motion':{'status':'READY' if self.acknowledged else 'AWAITING_CAPTURE','sample_id':self.identity,'capture_ready_wall_ns':1050000000}}
        owner.update(changes);return {'owner':owner}

    def sample(self,*args,**kwargs):
        self.assertEqual(kwargs['owner_pid'],123);self.assertEqual(kwargs['command_epoch'],4);self.assertEqual(kwargs['not_before_ns'],1050000000)
        return {'row':{'qpos':[0.]*14,'q_reported_rad':[0.]*12,'camera_detections':{n:{'24':[[1,2]]*4,'30':[[2,3]]*4} for n in self.names}},
                'rows':{n:{'qpos':[0.]*14,'q_reported12':[0.]*12,'bracket_gap_s':.02,'pose_span_rad':[0.]*12,'metadata':{'serial':n+'-serial'}} for n in self.names},
                'images':{n:b'png' for n in self.names},'raw_images':{n:b'raw jpeg' for n in self.names},
                'cameras':{n:self.intrinsics[n] for n in self.names}}

    def confirm(self):
        preview=self.app.preview_calibration()
        return self.app.confirm_calibration({'token':preview['token'],'confirmation':'START_REVIEWED_SINGLE_ARM','physical_clearance':True})

    def test_calibration_jaws_explicit_and_old_plan_rejected(self):
        self.assertTrue(all(w['jaw_gap_mm']==[80.,80.] for w in self.app.queue))
        self.app.queue[0]['jaw_gap_mm']=[54.,83.]
        with self.assertRaisesRegex(ValueError,'80 mm'):self.app.preview_calibration()
        self.assertEqual(self.calls,[])

    def fresh_control(self):
        a=self.app;now=time.monotonic()
        a.require_real_control=Workbench.require_real_control.__get__(a)
        a.control_available=Workbench.control_available.__get__(a)
        a.hardware_connected=Workbench.hardware_connected.__get__(a)
        a.hardware_poll_lock=threading.Lock();a.hardware_history_lock=threading.Lock()
        a.hardware_progress=dict(source=(123,None),sdk=[10.,20.],advanced=[now,now])
        owner=dict(pid=123,command_epoch=4,state='READY',feedback_healthy=True,
            capabilities=['calibration_capture_ack_v1'],
            calibration_feedback=dict(qpos=[0.]*14,sdk_timestamps=[10.01,20.01]))
        status=dict(motion_enabled=True,owner=copy.deepcopy(owner))
        view=dict(reachable=True,raw_owner=owner,feedback=dict(valid=True,host_sample_age_s=.01))
        a.hardware_reader=SimpleNamespace(read_state=Mock(return_value=view))
        a.control_status=dict(motion_enabled=False,owner=copy.deepcopy(owner));a.control_received=now
        a.hardware_feedback=dict(owner=owner);a.hardware_received=now-3.;a.hardware_error='old monitor failure'
        gateway=self.gateway
        def live(action,body=None,**kwargs):
            if action=='state':
                self.calls.append((action,None));return status
            return gateway(action,body,**kwargs)
        a.control_request=live;a.collect_hardware=Mock()
        return status,view

    def test_fresh_preflight_allows_preview_and_confirm_despite_stale_ui_cache(self):
        status,view=self.fresh_control();a=self.app
        self.assertFalse(a.control_available())  # The former preview/confirm veto.
        preview=a.preview_calibration()
        a.confirm_calibration(dict(token=preview['token'],confirmation='START_REVIEWED_SINGLE_ARM',physical_clearance=True))
        self.assertEqual([action for action,_ in self.calls],['state','trajectory-preview','state','motion-confirm'])
        self.assertEqual(a.hardware_reader.read_state.call_count,2)
        a.collect_hardware.assert_called_once()

    def test_fresh_preflight_rejects_live_faults_before_confirmation(self):
        for fault in ('permission','stale','frozen','bad_feedback','pid','epoch','source','read_error'):
            with self.subTest(fault=fault):
                status,view=self.fresh_control();a=self.app
                a.status='READY';preview=a.preview_calibration();self.calls.clear()
                before=set(a.output.glob('batch-hardware-*'))
                if fault=='permission':status['motion_enabled']=False
                elif fault=='stale':view['feedback']['host_sample_age_s']=.501
                elif fault=='frozen':a.hardware_progress.update(sdk=[10.01,20.01],advanced=[time.monotonic()-.501]*2)
                elif fault=='bad_feedback':view['feedback'].update(valid=False,reasons=['owner_feedback_unhealthy'])
                elif fault=='pid':status['owner']['pid']=999
                elif fault=='epoch':status['owner']['command_epoch']=5
                elif fault=='source':status['owner']['calibration_feedback']['source_clock_id']='another-plant'
                else:a.hardware_reader.read_state.side_effect=OSError('current feedback transport failed')
                with self.assertRaises((ValueError,OSError)):
                    a.confirm_calibration(dict(token=preview['token'],confirmation='START_REVIEWED_SINGLE_ARM',physical_clearance=True))
                self.assertEqual([action for action,_ in self.calls],['state'])
                self.assertEqual(set(a.output.glob('batch-hardware-*')),before)
                self.assertEqual(a.calibration_previews,{})
                a.collect_hardware.assert_not_called()

    def test_preflight_failure_preserves_exact_read_diagnostics_without_command_retry(self):
        _,view=self.fresh_control();a=self.app;a.pipeline_run_id='diagnostic-run'
        stamp=time.time_ns()
        view['feedback'].update(valid=False,reasons=['host_sample_stale_or_clock_skew'])
        first=None
        for age,message in ((.501,'已过期'),(-.001,'超前')):
            detail=dict(request_start_wall_ns=stamp-1,request_start_monotonic_s=1.,
                response_end_wall_ns=stamp,response_end_monotonic_s=1.01,
                raw_sample_wall_ns=stamp,raw_sample_wall_ns_type='int',normalize_now_ns=stamp+round(age*1e9),
                normalized_age_s=age,max_age_s=.5,branch='raw_owner',pid=123,command_epoch=4,
                clock_domain='host_wall',source_clock_id=None,source_time_ns=None,physics_step=None,
                reasons=['host_sample_stale_or_clock_skew'])
            view['read_diagnostics']=detail
            with self.assertRaisesRegex(ValueError,message) as rejected:a.require_real_control()
            self.assertNotIn('request_start_wall_ns',str(rejected.exception))
            failure=a.control_preflight_failure;path=Path(failure['path'])
            self.assertTrue(path.is_relative_to(a.output/'diagnostics'))
            self.assertEqual(json.loads(path.read_text())['read_diagnostics'],detail)
            self.assertEqual((failure['run_id'],failure['stage_id']),('diagnostic-run','head.preflight'))
            if first:self.assertEqual(first[0].read_bytes(),first[1])
            else:first=(path,path.read_bytes())
        self.assertEqual(len(list((a.output/'diagnostics').glob('control-preflight-*.json'))),2)
        self.assertEqual([action for action,_ in self.calls],['state','state'])
        a.collect_hardware.assert_not_called()

    def test_diagnostic_write_failure_keeps_original_rejection_in_memory(self):
        _,view=self.fresh_control();view['feedback'].update(valid=False,reasons=['owner_feedback_unhealthy'])
        with patch('robot_workbench.collection_setup.write',side_effect=OSError('read-only output')):
            with self.assertRaisesRegex(ValueError,'诊断文件保存失败'):self.app.require_real_control()
        self.assertIn('owner_feedback_unhealthy',self.app.control_preflight_failure['error'])
        self.assertIn('read-only output',self.app.control_preflight_failure['save_error'])
        self.assertEqual(self.calls,[('state',None)])

    def test_fake_capture_passes_source_readiness_and_keeps_wall_ack_nonce(self):
        a=self.app;a.config.update(environment='fake',simulation_only=True,robot_source='simulated')
        a.hardware_feedback={'owner':dict(mode='simulation',environment='fake',simulation_only=True)}
        a.control_status['owner']['capabilities'].append('calibration_capture_window_v2')
        original=self.gateway
        def gateway(action,body=None,**kwargs):
            if action=='state':
                state=self.owner_state();owner=state['owner']
                owner.update(mode='simulation',environment='fake',simulation_only=True,
                    calibration_feedback=dict(sample_wall_ns=2_000_000_000,clock_domain='simulation',
                        source_clock_id='fixture-plant',source_time_ns=75_000_000,physics_step=18))
                owner['motion'].update(capture_ready_source_ns=50_000_000,capture_ready_physics_step=12,
                    capture_clock_domain='simulation',capture_source_clock_id='fixture-plant')
                if self.acknowledged:owner['last_capture_ack']=dict(sample_id=self.identity,capture_ready_wall_ns=1050000000,
                    command_epoch=4,source_clock_id='fixture-plant',capture_ready_source_ns=50_000_000)
                return state
            if action=='capture-ack':
                self.assertEqual(body['capture_ready_wall_ns'],1050000000)
                self.acknowledged=True;return {'ok':True}
            return original(action,body,**kwargs)
        a.control_request=gateway
        def capture(*args,**kwargs):
            self.assertEqual(kwargs['environment'],'fake')
            self.assertEqual(kwargs['source_clock_id'],'fixture-plant')
            self.assertEqual(kwargs['not_before_source_ns'],50_000_000)
            self.assertNotIn('not_before_ns',kwargs)
            return self.sample(*args,**{**kwargs,'not_before_ns':1050000000})
        with patch('arx_env.joint_head_workbench_capture.capture_sample',side_effect=capture):self.confirm()
        self.assertEqual(a.status,'READY',a.reason)
        dataset=next(a.output.glob('batch-hardware-*/capture'))
        manifest=json.loads((dataset/'dataset.json').read_text())
        self.assertEqual(manifest['runtime_environment'],'fake')
        self.assertEqual(manifest['clock']['source_clock_id'],'fixture-plant')
        self.assertTrue(manifest['simulation_only'])
        self.assertEqual(manifest['frames'][0]['capture_ready_source_ns'],50_000_000)

    def clocked_capture(self,*,environment='fake',resident_s=300.,ready_wall_s=160.,corruption=None,pipeline=False):
        """The real C orchestration sees a healthy 0.25x source and virtual wall waits."""
        a=self.app;clock=[0.];wall_origin=1_700_000_000_000_000_000
        if environment=='fake':
            a.config.update(environment='fake',simulation_only=True,robot_source='simulated')
            a.hardware_feedback={'owner':dict(mode='simulation',environment='fake',simulation_only=True)}
            if resident_s is not None:a.config['resident_deadline_wall_ns']=wall_origin+round(resident_s*1e9)
        def wall_ns():
            rollback=1000. if corruption=='wall_rollback' and clock[0]>=50. else 0.
            return wall_origin+round((clock[0]-rollback)*1e9)
        original=self.gateway
        def gateway(action,body=None,**kwargs):
            if action=='trajectory-preview':return {**original(action,body,**kwargs),'total_motion_duration_s':40.}
            if action!='state':return original(action,body,**kwargs)
            owner=self.owner_state()['owner']
            if not self.acknowledged and clock[0]<ready_wall_s:owner['motion']['status']='RUNNING'
            if environment=='fake':
                source_ns=round(clock[0]*.25*1e9)
                if corruption=='source_regression' and clock[0]>=40.:source_ns=0
                if corruption=='epoch' and clock[0]>=40.:owner['command_epoch']+=1
                owner.update(mode='simulation',environment='fake',simulation_only=True,
                    calibration_feedback=dict(sample_wall_ns=wall_ns(),clock_domain='simulation',
                        source_clock_id='other-source' if corruption=='source_instance' and clock[0]>=40. else 'slow-fixture',
                        source_time_ns=source_ns,physics_step=source_ns*240//1_000_000_000))
                owner['motion'].update(capture_ready_source_ns=round(ready_wall_s*.25*1e9),
                    capture_ready_physics_step=round(ready_wall_s*.25*240),capture_clock_domain='simulation',capture_source_clock_id='slow-fixture')
            return {'owner':owner}
        def sample(*args,**kwargs):
            if environment=='fake':
                self.assertEqual(kwargs['not_before_source_ns'],round(ready_wall_s*.25*1e9))
                kwargs={**kwargs,'not_before_ns':1050000000}
            return self.sample(*args,**kwargs)
        def wait(unused):clock[0]+=10.;return False
        a.control_request=gateway
        with patch('joint_head_workbench.time.monotonic',side_effect=lambda:clock[0]), \
                patch('joint_head_workbench.time.time_ns',side_effect=wall_ns),patch.object(a.stop,'wait',side_effect=wait), \
                patch('arx_env.joint_head_workbench_capture.capture_sample',side_effect=sample):
            if pipeline:
                a.config['pipeline_auto_confirm']=True
                with self.assertRaises(ValueError):a.capture_batch({},'头部')
            else:self.confirm()
        dataset=next(a.output.glob('batch-hardware-*/capture'))
        return json.loads((dataset/'completion.json').read_text()),json.loads((dataset/'dataset.json').read_text())

    def test_resident_fake_capture_can_outlast_legacy_wall_formula_with_real_source_progress(self):
        completion,manifest=self.clocked_capture()
        self.assertEqual(self.app.status,'READY',self.app.reason)
        self.assertTrue(completion['passed']);self.assertEqual(completion['captured'],1)
        timing=completion['timing']
        self.assertGreater(timing['elapsed_wall_s'],timing['legacy_wall_budget_s'])
        self.assertEqual(timing['legacy_wall_budget_s'],110.)
        self.assertGreaterEqual(timing['source_span_s'],40.)
        self.assertEqual(timing['budget_origin'],'fixed_resident_wall')
        self.assertEqual(completion['runtime_environment'],'fake');self.assertEqual(manifest['environment'],'hardware')

    def test_physical_and_independent_fake_keep_finite_original_wall_timeout(self):
        for environment in ('physical','fake'):
            with self.subTest(environment=environment),tempfile.TemporaryDirectory() as root:
                self.app.output=Path(root).resolve();self.app.config.pop('resident_deadline_wall_ns',None)
                completion,manifest=self.clocked_capture(environment=environment,resident_s=None)
                self.assertFalse(completion['passed']);self.assertEqual(completion['captured'],0)
                self.assertIn(('Fake' if environment=='fake' else '真机')+'采集超时',completion['reason'])
                self.assertEqual(completion['timing']['legacy_wall_budget_s'],110.)
                self.assertEqual(completion['runtime_environment'],environment)
                self.assertEqual(manifest['failure']['runtime_environment'],environment)
                self.assertNotIn('capture-ack',[action for action,_ in self.calls])

    def test_fixed_resident_deadline_cannot_extend_on_wall_rollback_or_automatic_retry(self):
        completion,manifest=self.clocked_capture(resident_s=130.,ready_wall_s=160.,corruption='wall_rollback',pipeline=True)
        self.assertFalse(completion['passed']);self.assertIn('固定驻留',completion['reason'])
        self.assertEqual(completion['timing']['resident_budget']['deadline_monotonic_s'],85.)
        self.assertEqual(completion['timing']['elapsed_wall_s'],90.)
        self.assertEqual(completion['timing']['first_source_stamp'],[0,0])
        self.assertGreater(completion['timing']['last_source_stamp'][0],0)
        self.assertEqual([action for action,_ in self.calls].count('motion-confirm'),1)
        self.assertNotIn('capture-ack',[action for action,_ in self.calls])
        self.assertEqual(manifest['failure']['timing']['owner_pid'],123)

    def test_persistent_fake_keeps_slow_source_capture_but_bounds_each_batch(self):
        self.app.config['persistent_service']=True
        completion,_=self.clocked_capture(resident_s=None,ready_wall_s=160.)
        self.assertTrue(completion['passed'])
        self.assertEqual(completion['timing']['budget_origin'],'persistent_fake_finite_batch')
        with tempfile.TemporaryDirectory() as root:
            self.app.output=Path(root).resolve();self.acknowledged=False
            completion,_=self.clocked_capture(resident_s=None,ready_wall_s=7300.)
        self.assertFalse(completion['passed'])
        self.assertIn('采集超时',completion['reason'])
        self.assertEqual(completion['timing']['deadline_monotonic_s'],7200.)

    def test_slow_fake_still_rejects_source_regression_instance_and_epoch_changes(self):
        for corruption in ('source_regression','source_instance','epoch'):
            with self.subTest(corruption=corruption),tempfile.TemporaryDirectory() as root:
                self.app.output=Path(root).resolve();self.app.__dict__.pop('_resident_capture_budget',None)
                completion,_=self.clocked_capture(corruption=corruption)
                self.assertFalse(completion['passed']);self.assertLess(completion['timing']['elapsed_wall_s'],110.)
                self.assertIn('stop',[action for action,_ in self.calls])
                self.assertNotIn('capture-ack',[action for action,_ in self.calls])

    def test_outer_fake_wait_uses_original_resident_end_and_preserves_collector_reason(self):
        a=self.app;clock=[0.];origin=time.time_ns()
        a.config.update(environment='fake',simulation_only=True,resident_deadline_wall_ns=origin+4_000_000_000_000)
        a.status='CAPTURING';a.reason=None
        def advance(unused):
            clock[0]+=1000.
            if clock[0]>=3000.:a.status='REJECTED';a.reason='collector exact failure preserved'
        with patch('joint_head_workbench.time.monotonic',side_effect=lambda:clock[0]), \
                patch('joint_head_workbench.time.time_ns',side_effect=lambda:origin+round(clock[0]*1e9)), \
                patch('joint_head_workbench.time.sleep',side_effect=advance):
            with self.assertRaisesRegex(ValueError,'collector exact failure preserved'):a.wait_for_step(1800,'头部采集')
        self.assertEqual(clock[0],3000.)

    def test_exhausted_resident_rejects_confirmation_before_motion_or_batch_creation(self):
        a=self.app;now=time.time_ns()
        a.config.update(environment='fake',simulation_only=True,robot_source='simulated',resident_deadline_wall_ns=now+30_000_000_000)
        a.hardware_feedback={'owner':dict(mode='simulation',environment='fake',simulation_only=True)}
        with self.assertRaisesRegex(ValueError,'固定驻留'):self.confirm()
        self.assertNotIn('motion-confirm',[action for action,_ in self.calls])
        self.assertEqual(list(a.output.glob('batch-hardware-*')),[])

    def test_resident_anchor_rejects_configuration_extension(self):
        a=self.app;a.config.update(environment='fake',simulation_only=True,robot_source='simulated',
            resident_deadline_wall_ns=time.time_ns()+300_000_000_000)
        original=a.resident_capture_budget()
        a.config['resident_deadline_wall_ns']+=100_000_000_000
        with self.assertRaisesRegex(ValueError,'期限已变化'):a.resident_capture_budget()
        self.assertEqual(a._resident_capture_budget,original)

    def test_head_and_wrist_share_hardware_capture_and_persist_before_ack(self):
        for kind in ('head','wrist'):
            with self.subTest(kind=kind),tempfile.TemporaryDirectory() as root:
                a=self.app;a.output=Path(root).resolve();self.acknowledged=False;self.calls=[]
                a.workflow={'kind':kind,'head_result_id':'head-version' if kind=='wrist' else None}
                self.names=['head','left','right'] if kind=='wrist' else ['head']
                if kind=='wrist':
                    a.plan={'wrist_contract':{'environment':'hardware','head_snapshot':{'id':'head-version','objective':'sticker_corners_plus_mesh','snapshot_complete':True,'acceptance':{'image_passed':True,'image_checks':{'positive_depth':True}},'optimizer':{'success':True},'mesh_metrics':{'holdout':{'macro_coverage':.8,'macro_mean_penalized_px':5.}},'metrics_rectified_px':{'holdout':{'rms_px':1.}}},'cameras':{n:self.intrinsics[n] for n in ('left','right')}}}
                    a.queue[0].update(observer='left',target='right')
                with patch('arx_env.joint_head_workbench_capture.capture_sample',side_effect=self.sample) as sample:
                    result=self.confirm()
                self.assertEqual(result['environment'],'hardware');self.assertEqual(sample.call_count,1)
                self.assertEqual(sample.call_args.kwargs['required_markers'],{'left':[30]} if kind=='wrist' else {'head':[24]})
                dataset=next(a.output.glob('batch-hardware-*/capture'))
                manifest=json.loads((dataset/'dataset.json').read_text())
                self.assertEqual(manifest['environment'],'hardware');self.assertEqual(manifest['robot_id'],'arx6')
                self.assertEqual(len(manifest['frames']),1);self.assertEqual(manifest['frames'][0]['environment'],'hardware')
                self.assertEqual(manifest['frames'][0]['planned_id'],'pose-0')
                self.assertEqual(set(manifest['frames'][0]['camera_images']),set(self.names))
                self.assertTrue(json.loads((dataset/'completion.json').read_text())['passed'])
                self.assertEqual([x[0] for x in self.calls].count('motion-confirm'),1)
                self.assertEqual([x[0] for x in self.calls].count('capture-ack'),1)
                a.remote_request.assert_not_called()

    def test_revoked_window_retries_without_duplicate_or_stale_ack(self):
        for revoke in ('sampling_error','after_sample','ack'):
            with self.subTest(revoke=revoke),tempfile.TemporaryDirectory() as root:
                a=self.app;a.output=Path(root).resolve();self.acknowledged=False;self.calls=[]
                a.control_status['owner']['capabilities'].append('calibration_capture_window_v2')
                window=[1050000000];attempts=[];receipt={};original=self.gateway
                def state(**changes):
                    return {'owner':{'pid':123,'command_epoch':4,'state':'READY','feedback_healthy':True,'capabilities':a.control_status['owner']['capabilities'],
                        'automatic_enabled':not self.acknowledged,'last_capture_ack':dict(receipt),
                        'motion':{'status':'READY' if self.acknowledged else 'AWAITING_CAPTURE','sample_id':self.identity,'capture_ready_wall_ns':window[0]}}}
                self.owner_state=state
                def gateway(action,body=None,**kwargs):
                    if action!='capture-ack':return original(action,body)
                    self.calls.append((action,copy.deepcopy(body)))
                    dataset=next(a.output.glob('batch-hardware-*/capture'))
                    rows=json.loads((dataset/'dataset.json').read_text())['frames']
                    self.assertEqual(len(rows),1);self.assertEqual(rows[0]['capture_ready_wall_ns'],body['capture_ready_wall_ns'])
                    self.assertTrue((dataset/(self.identity+'.json')).exists())
                    if revoke=='ack' and len(attempts)==1:
                        window[0]+=1
                        receipt.update(sample_id=self.identity,capture_ready_wall_ns=window[0]-1-1)
                        raise HTTPError('http://owner/capture-ack',400,'stale capture window',{},None)
                    self.assertEqual(body['capture_ready_wall_ns'],window[0])
                    receipt.update(sample_id=self.identity,capture_ready_wall_ns=window[0]);self.acknowledged=True
                    return {'ok':True}
                a.control_request=gateway
                def sample(*args,**kwargs):
                    attempts.append(kwargs['not_before_ns'])
                    value=self.sample(*args,**{**kwargs,'not_before_ns':1050000000})
                    if len(attempts)==1 and revoke!='ack':
                        window[0]+=1
                        if revoke=='sampling_error':raise ValueError('window invalidated while bracketing')
                    return value
                with patch('arx_env.joint_head_workbench_capture.capture_sample',side_effect=sample):self.confirm()
                dataset=next(a.output.glob('batch-hardware-*/capture'));manifest=json.loads((dataset/'dataset.json').read_text())
                self.assertEqual(attempts,[1050000000,1050000001])
                self.assertEqual(len(manifest['frames']),1)
                self.assertEqual(manifest['frames'][0]['capture_ready_wall_ns'],1050000001)
                self.assertEqual(len(manifest['capture_rejections']),1)
                self.assertTrue(json.loads((dataset/'completion.json').read_text())['passed'])
                self.assertEqual(len(list(dataset.glob('*-head.png'))),1)

    def test_queued_ack_waits_for_owner_receipt_without_resampling(self):
        a=self.app;a.control_status['owner']['capabilities'].append('calibration_capture_window_v2')
        original=self.gateway;queued=[];polls=[]
        def gateway(action,body=None,**kwargs):
            if action=='capture-ack':queued.append(body);return {'ok':True}
            if action=='state' and queued:
                polls.append(True)
                if len(polls)>1:self.acknowledged=True
                state=self.owner_state()
                if self.acknowledged:state['owner']['last_capture_ack']={'sample_id':self.identity,'capture_ready_wall_ns':1050000000}
                return state
            return original(action,body)
        a.control_request=gateway
        with patch('arx_env.joint_head_workbench_capture.capture_sample',side_effect=self.sample) as sample:self.confirm()
        self.assertEqual(sample.call_count,1);self.assertEqual(len(queued),1);self.assertGreater(len(polls),1)
        dataset=next(a.output.glob('batch-hardware-*/capture'))
        self.assertTrue(json.loads((dataset/'completion.json').read_text())['passed'])

    def test_rollback_never_removes_preexisting_sample_file(self):
        a=self.app;existing=[]
        def sample(*args,**kwargs):
            dataset=next(a.output.glob('batch-hardware-*/capture'));path=dataset/(self.identity+'-head.png')
            path.write_bytes(b'existing');existing.append(path)
            return self.sample(*args,**kwargs)
        with patch('arx_env.joint_head_workbench_capture.capture_sample',side_effect=sample):self.confirm()
        self.assertEqual(a.status,'REJECTED');self.assertEqual(existing[0].read_bytes(),b'existing')
        self.assertNotIn('capture-ack',[action for action,_ in self.calls])

    def test_confirmed_frame_survives_fault_after_owner_ack(self):
        a=self.app;a.control_status['owner']['capabilities'].append('calibration_capture_window_v2')
        original=self.gateway;queued=[]
        def gateway(action,body=None,**kwargs):
            if action=='capture-ack':queued.append(body);return {'ok':True}
            if action=='state' and queued:
                return self.owner_state(command_epoch=5,state='FAULT',last_capture_ack={'sample_id':self.identity,'capture_ready_wall_ns':1050000000})
            return original(action,body)
        a.control_request=gateway
        with patch('arx_env.joint_head_workbench_capture.capture_sample',side_effect=self.sample):self.confirm()
        dataset=next(a.output.glob('batch-hardware-*/capture'))
        self.assertEqual(a.status,'REJECTED')
        self.assertEqual(len(json.loads((dataset/'dataset.json').read_text())['frames']),1)
        self.assertTrue(list(dataset.glob('*-head.png')))

    def test_unconfirmed_frame_removed_when_owner_faults(self):
        a=self.app;a.control_status['owner']['capabilities'].append('calibration_capture_window_v2')
        original=self.gateway;failed=[False]
        def gateway(action,body=None,**kwargs):
            if action=='capture-ack':failed[0]=True;return {'ok':True}
            if action=='state' and failed[0]:return self.owner_state(command_epoch=5,state='FAULT',reason='recovery timeout')
            return original(action,body)
        a.control_request=gateway
        with patch('arx_env.joint_head_workbench_capture.capture_sample',side_effect=self.sample):self.confirm()
        dataset=next(a.output.glob('batch-hardware-*/capture'));manifest=json.loads((dataset/'dataset.json').read_text())
        self.assertEqual(manifest['frames'],[]);self.assertEqual(len(manifest['capture_rejections']),1)
        self.assertFalse(list(dataset.glob('*-head.png')))
        self.assertFalse(json.loads((dataset/'completion.json').read_text())['passed'])

    def test_sampling_failure_never_acknowledges_and_stops_same_owner(self):
        with patch('arx_env.joint_head_workbench_capture.capture_sample',side_effect=RuntimeError('camera failed')):
            self.confirm()
        self.assertEqual(self.app.status,'REJECTED');self.assertIn('camera failed',self.app.reason)
        self.assertNotIn('capture-ack',[x[0] for x in self.calls]);self.assertIn(('stop',{'owner_pid':123,'command_epoch':4}),self.calls)

    def test_disk_failure_never_acknowledges(self):
        original=Path.open
        def full(path,mode='r',*args,**kwargs):
            if mode=='xb':raise OSError('disk full')
            return original(path,mode,*args,**kwargs)
        with patch('arx_env.joint_head_workbench_capture.capture_sample',side_effect=self.sample),patch.object(Path,'open',full):
            self.confirm()
        self.assertEqual(self.app.status,'REJECTED');self.assertIn('disk full',self.app.reason)
        self.assertNotIn('capture-ack',[x[0] for x in self.calls]);self.assertIn(('stop',{'owner_pid':123,'command_epoch':4}),self.calls)

    def test_owner_replacement_must_not_stop_replacement(self):
        self.owner_state=lambda **kw:{'owner':{'pid':999,'command_epoch':4,'state':'READY','feedback_healthy':True}}
        gateway=self.app.control_request
        def command(action,body=None,**kwargs):
            if action=='stop':
                self.calls.append((action,copy.deepcopy(body)))
                self.assertEqual(body,{'owner_pid':123,'command_epoch':4})
                raise ValueError('conditional stop owner or epoch changed')
            return gateway(action,body)
        self.app.control_request=command
        with patch('arx_env.joint_head_workbench_capture.capture_sample') as sample:
            self.confirm()
        self.assertEqual(self.app.status,'REJECTED');self.assertIn('身份',self.app.reason)
        sample.assert_not_called();self.assertNotIn('capture-ack',[x[0] for x in self.calls])
        self.assertIn(('stop',{'owner_pid':123,'command_epoch':4}),self.calls)

    def test_camera_role_must_match_explicit_serial_configuration(self):
        a=self.app;a.hardware_camera=self.cameras['head'];a.wrist_cameras={n:self.cameras[n] for n in ('left','right')}
        a.config={'hardware_camera_serials':{n:n+'-serial' for n in self.cameras}}
        for name,camera in self.cameras.items():
            camera.read_calibration_frame=lambda n=name:(b'raw',{'serial':n+'-serial','intrinsics':self.intrinsics[n]})
        a.workflow={'kind':'wrist','head_result_id':'head'}
        self.assertEqual(set(Workbench.hardware_calibration_cameras(a)[0]),{'head','left','right'})
        a.wrist_cameras={'left':self.cameras['right'],'right':self.cameras['left']}
        with self.assertRaisesRegex(ValueError,'配置角色'):Workbench.hardware_calibration_cameras(a)

    def test_missing_owner_ready_timestamp_fails_without_sampling(self):
        original=self.owner_state
        def state(**changes):
            value=original(**changes);value['owner']['motion'].pop('capture_ready_wall_ns');return value
        self.owner_state=state
        with patch('arx_env.joint_head_workbench_capture.capture_sample') as sample:self.confirm()
        sample.assert_not_called();self.assertEqual(self.app.status,'REJECTED')
        self.assertNotIn('capture-ack',[action for action,_ in self.calls])

    def test_hardware_disconnection_refuses_preview_without_simulation_fallback(self):
        _,view=self.fresh_control();view.update(reachable=False,error='current reader disconnected')
        with self.assertRaisesRegex(ValueError,'查看诊断'):self.app.preview_calibration()
        self.assertIn('disconnected',self.app.control_preflight_failure['error'])
        self.assertEqual(self.calls,[('state',None)]);self.app.remote_request.assert_not_called()

    def test_stop_before_collection_never_samples_or_acknowledges(self):
        gateway=self.app.control_request
        def command(action,body=None,**kwargs):
            result=gateway(action,body)
            if action=='motion-confirm':self.app.stop.set()
            return result
        self.app.control_request=command
        with patch('arx_env.joint_head_workbench_capture.capture_sample') as sample:
            self.confirm()
        self.assertEqual(self.app.status,'REJECTED');self.assertIn('停止',self.app.reason)
        sample.assert_not_called();self.assertNotIn('capture-ack',[x[0] for x in self.calls])

    def test_stop_during_sample_does_not_acknowledge_or_continue(self):
        def interrupted(*args,**kwargs):
            value=self.sample(*args,**kwargs);self.app.stop.set();return value
        with patch('arx_env.joint_head_workbench_capture.capture_sample',side_effect=interrupted):
            self.confirm()
        self.assertEqual(self.app.status,'REJECTED');self.assertIn('停止',self.app.reason)
        self.assertNotIn('capture-ack',[x[0] for x in self.calls])

    def test_stop_during_failed_sample_does_not_retry(self):
        def interrupted(*args,**kwargs):
            self.app.stop.set();raise ValueError('marker not detected')
        clock=itertools.count(100.,.25)
        with patch('arx_env.joint_head_workbench_capture.capture_sample',side_effect=interrupted) as sample,patch('joint_head_workbench.time.monotonic',side_effect=lambda:next(clock)):
            self.confirm()
        self.assertEqual(self.app.status,'REJECTED')
        self.assertEqual(sample.call_count,1)
        self.assertNotIn('capture-ack',[x[0] for x in self.calls])

    def test_manifest_write_failure_still_attempts_same_owner_stop(self):
        original=Path.write_text
        def write(path,*args,**kwargs):
            if path.name=='dataset.json.tmp':raise OSError('manifest disk full')
            return original(path,*args,**kwargs)
        with patch('arx_env.joint_head_workbench_capture.capture_sample',side_effect=self.sample),patch.object(Path,'write_text',write):
            self.confirm()
        self.assertEqual(self.app.status,'REJECTED');self.assertIn('manifest disk full',self.app.reason)
        self.assertNotIn('capture-ack',[x[0] for x in self.calls]);self.assertIn(('stop',{'owner_pid':123,'command_epoch':4}),self.calls)

    def test_confirm_does_not_recheck_idle_after_motion_has_been_submitted(self):
        a=self.app;gateway=a.control_request
        def command(action,body=None,**kwargs):
            result=gateway(action,body)
            if action=='motion-confirm':
                a.idle=Mock(side_effect=ValueError('remote now executing'))
            return result
        a.control_request=command
        a.background=lambda status,fn:Workbench.background(a,status,fn)
        class ImmediateThread:
            def __init__(self,target,**kwargs):self.target=target
            def start(self):self.target()
        with patch('joint_head_workbench.threading.Thread',ImmediateThread),patch('arx_env.joint_head_workbench_capture.capture_sample',side_effect=self.sample):
            self.confirm()
        self.assertTrue(self.acknowledged)

    def test_worker_start_failure_stops_only_reviewed_owner(self):
        class FailedThread:
            def __init__(self,target,**kwargs):pass
            def start(self):raise RuntimeError('worker cannot start')
        with patch('joint_head_workbench.threading.Thread',FailedThread),patch('arx_env.joint_head_workbench_capture.capture_sample') as sample:
            with self.assertRaisesRegex(RuntimeError,'worker cannot start'):self.confirm()
        self.assertEqual(self.app.status,'REJECTED');sample.assert_not_called()
        self.assertIn(('stop',{'owner_pid':123,'command_epoch':4}),self.calls);self.assertNotIn('capture-ack',[x[0] for x in self.calls])


if __name__=='__main__':unittest.main()
