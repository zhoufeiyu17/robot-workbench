"""Offline capture orchestration: persistent samples precede ACK, never robot IO."""
import copy
import json
from pathlib import Path
import tempfile
import itertools
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch
from urllib.error import HTTPError

import test_joint_head_workbench_live_bridge as bridge_fixture
from joint_head_workbench import Workbench


class CaptureFlowTests(unittest.TestCase):
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
        self.app.hardware_connected=lambda:False
        with self.assertRaisesRegex(ValueError,'不会回退'):self.app.preview_calibration()
        self.assertEqual(self.calls,[]);self.app.remote_request.assert_not_called()

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
