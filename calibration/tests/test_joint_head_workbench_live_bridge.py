"""Offline bridge checks: remote authority, fail closed, and explicit hardware boundary."""
import json
import hashlib
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'examples/scripts/debug'))
from joint_head_workbench import Workbench, NoRedirect, load_wrist_selection


LOADER_SHAPES=[({'id':'p','pair_status':'complete','left':{},'right':{}},True),
               ({'left':{'id':'l'},'right':{'id':'r'}},False),
               ({'id':'p','pair_status':'incomplete_left'},False),
               ({'id':'p'},False),
               ([],False),('complete',False),(None,False),({},False)]

class BridgeTests(unittest.TestCase):
    def test_ack_local_rejection_and_remote_stop_are_independent(self):
        a=self.app;a.import_error=None
        a.status='REJECTED';a.reason='wrist fit rejected';a.tokens={'stale':{}}
        a.action('/api/ack',{})
        self.assertEqual(a.status,'READY');self.assertIsNone(a.reason)
        self.assertEqual(a.tokens,{});self.assertEqual(self.calls,[])
        for remote_state in ('STOPPED','ERROR'):
            a.remote['status']=remote_state;a.status='REJECTED';a.reason='needs acknowledgement'
            a.action('/api/ack',{})
            self.assertEqual(self.calls[-1],('/ack',{}));self.assertEqual(a.status,'READY')
        a.status='REJECTED';a.remote['status']='STOPPED'
        def reject_ack(*args,**kwargs):raise ValueError('remote acknowledgement refused')
        a.remote_request=reject_ack
        with self.assertRaisesRegex(ValueError,'refused'):a.action('/api/ack',{})
        self.assertEqual(a.status,'REJECTED','Failed remote acknowledgement must not clear the local state')

    def test_paired_selection_loader_never_iterates_sides(self):
        # The selection is one paired record. Reading the persisted file as a per-side
        # mapping crashed startup with "'str' object has no attribute 'get'", so the
        # loader must accept the paired shape and discard everything else - never
        # interpret a side-keyed file as valid.
        path=self.app.output/'wrist-selected.json'
        for raw,kept in LOADER_SHAPES:
            path.write_text(json.dumps(raw))
            self.assertEqual(bool(load_wrist_selection(path)),kept,(raw,))
        path.write_text('{')                             # partial write
        self.assertEqual(load_wrist_selection(path),{})
        path.unlink()                                    # absent file
        self.assertEqual(load_wrist_selection(path),{})
        path.write_text(json.dumps({'id':'p','pair_status':'complete','left':'L','right':'R'}))
        self.assertEqual(load_wrist_selection(path)['right'],'R')
        # Startup must go through this loader: no per-side iteration may come back.
        import inspect
        source=inspect.getsource(Workbench.__init__)
        self.assertIn('load_wrist_selection',source)
        self.assertNotIn('wrist_selected.items()',source)

    def test_wrist_selection_id_is_shared_by_both_sides(self):
        a=self.app
        for choice,expected in [({'id':'p1','pair_status':'complete'},'p1'),
                                ({'left':{'id':'p1'},'right':{'id':'p1'}},None),
                                ({'id':'p1','pair_status':'incomplete_left'},'p1'),
                                ({'id':'p1'},None),({},None),(None,None)]:
            a.wrist_selected=choice
            self.assertEqual(a.wrist_selection_id(),expected,choice)

    def test_camera_frame_age_reads_whichever_basis_the_camera_gives(self):
        # A camera that computed its own age: source_age_s extended by the elapsed time.
        a=self.app
        self.assertAlmostEqual(a.camera_frame_age({'source_age_s':.25,'received_monotonic':time.monotonic()}),.25,places=2)
        # A camera that only stamps the frame: aged against the wall clock, as snapshot() does.
        age=a.camera_frame_age({'frame_timestamp_ns':time.time_ns()-250_000_000})
        self.assertTrue(.2<=age<=2.,age)
        # Neither basis present: say so instead of subtracting None from a float.
        for meta in ({'source_age_s':.5},{'received_monotonic':time.monotonic()},{}):
            with self.assertRaisesRegex(ValueError,'年龄基准'):
                a.camera_frame_age(meta)

    def test_overlay_ages_keep_the_observation_apart_from_our_render(self):
        a=self.app
        self.assertEqual(a.overlay_ages(.1,.4),(.1,.5))
        self.assertEqual(a.overlay_ages(.1,-2.),(.1,.1))
        for values in ((.1,None),('0.1',.4)):
            with self.assertRaisesRegex(ValueError,'叠加年龄'):
                a.overlay_ages(*values)

    def test_overlay_gate_judges_the_camera_not_our_render(self):
        from unittest.mock import patch
        a=self.app
        (a.output/'camera.json').write_text(json.dumps({'width':640,'height':480,'fx':1.,'fy':1.,'cx':0.,'cy':0.}))
        stamp=1_700_000_000_000_000_000
        clock=[1000.]
        a.hardware_camera=SimpleNamespace(read_frame=lambda:(b'\xff\xd8raw\xff\xd9',
            {'source_age_s':.1,'received_monotonic':1000.,'frame_timestamp_ns':stamp,'frame_seq':7}))
        a.hardware_history=[{'owner':{'pid':9,'calibration_feedback':{'qpos':[0.]*14,'sample_wall_ns':stamp+5_000_000}}}]
        a.hardware_feedback={'owner':{'pid':9}}
        a.hardware_connected=lambda:True
        a.require_remote=lambda:None
        a.config={'camera':str(a.output/'camera.json')}
        candidate={'id':'cand','joint_offsets_rad':{'left':[0.]*6,'right':[0.]*6}}
        a.calibration={'id':'cand','candidate':'joint_candidate','batch':'b','environment':'hardware','parameters':candidate}
        def slow_render(path,body=None,**kw):
            self.calls.append((path,body));clock[0]+=3.;return b'\xff\xd8mesh\xff\xd9'
        a.remote_request=slow_render
        with patch('joint_head_workbench.time.monotonic',side_effect=lambda:clock[0]):
            raw,meta=a.hardware_overlay_frame(True)
        # A 3 s render is a property of the bridge: it must not be charged to the camera.
        self.assertEqual(raw,b'\xff\xd8mesh\xff\xd9')
        self.assertEqual([call[0] for call in self.calls],['/real-overlay'])
        self.assertAlmostEqual(meta['image_age_s'],.1,places=3)
        self.assertAlmostEqual(meta['observed_age_s'],3.1,places=3)
        self.assertAlmostEqual(meta['render_ms'],3000.,places=1)
        # The camera's own age still refuses the frame, before any render is attempted.
        self.calls.clear()
        a.hardware_camera=SimpleNamespace(read_frame=lambda:(b'raw',{'source_age_s':2.5,'received_monotonic':1000.,'frame_timestamp_ns':stamp,'frame_seq':7}))
        with patch('joint_head_workbench.time.monotonic',side_effect=lambda:clock[0]):
            with self.assertRaisesRegex(ValueError,'真机源图像已过期'):
                a.hardware_overlay_frame(True)
        self.assertEqual(self.calls,[])

    def test_unplanned_motion_is_path_checked_before_the_robot_sees_it(self):
        a=self.app
        a.geometry_id='geometry';a.plan=None;a.queue=[];a.hardware_connected=lambda:True
        a.state=lambda:{'qpos':[0.]*12}
        seen=[];a.control_request=lambda action,body=None,**kwargs:seen.append((action,body)) or {'token':'t','plans':[]}
        checks=[]
        def path_check(start,target,side=None):
            checks.append((np.asarray(start).tolist(),np.asarray(target).tolist()))
            return {'ok':len(checks)>1,'fraction':0.5,'reason':'limits or conservative collision','collisions':[['left/link1','right/link2'],['left/link1','environment/chassis','intersection solver uncertain']]}
        a.geometry.with_gaps=lambda *gaps:SimpleNamespace(path_check=path_check)
        rows=[{'q':[0.]*12,'speeds':[.2]*12}]
        with self.assertRaises(ValueError) as caught:
            a.hardware_trajectory_preview({'waypoints':rows})
        message=str(caught.exception)
        self.assertIn('第 1 个路径点未通过路径检查',message)
        self.assertIn('left/link1/right/link2',message)
        self.assertIn('求解器不确定，按保守处理',message)
        self.assertEqual(seen,[],'a rejected path must never reach the control gateway')
        # The very same rows pass once the model accepts the swept path.
        a.hardware_trajectory_preview({'waypoints':rows})
        self.assertEqual([call[0] for call in seen],['trajectory-preview'])
        self.assertEqual(len(checks),2)

    def test_boundary_preview_checks_legal_path_but_owner_receives_original_request(self):
        a=self.app;a.geometry_id='geometry';a.plan=None;a.queue=[];a.hardware_connected=lambda:True
        a.geometry.limits[0]=[0.,2.]
        calls=[];checks=[]
        a.require_checked_path=lambda rows,start:checks.append((rows,start.copy()))
        def gateway(action,body,**kwargs):
            calls.append(body)
            return {'token':'bounded','plans':[{'q_target':[0.]*14}]}
        a.control_request=gateway
        for environment,tolerance in (('fake',.005),('physical',.01)):
            a.config=dict(environment=environment,simulation_only=environment=='fake',
                robot_source='simulated' if environment=='fake' else 'physical')
            raw=[0.]*12;raw[0]=-tolerance
            a.current_pose=lambda:np.asarray(raw)
            requested=raw[:];requested[0]=-tolerance/2
            value=a.hardware_trajectory_preview({'waypoints':[dict(q=requested)]})
            self.assertEqual(calls[-1]['waypoints'][0]['q'],requested)
            self.assertEqual(checks[-1][0][0]['q'][0],0.)
            self.assertEqual(checks[-1][1][0],0.)
            self.assertEqual(value['position_boundary']['start']['requested_q'],raw)
            self.assertEqual(value['position_boundary']['waypoints'][0]['command_q'][0],0.)
            self.assertTrue(value['position_boundary']['waypoints'][0]['corrections'])
            beyond=requested[:];beyond[0]=-tolerance-.0001;count=len(calls)
            with self.assertRaisesRegex(ValueError,'position-boundary'):
                a.hardware_trajectory_preview({'waypoints':[dict(q=beyond)]})
            self.assertEqual(len(calls),count)
        a.hardware_tokens.add('bounded')
        a.control_request=lambda *args,**kwargs:{'token':'bounded','plans':[{'q_target':[.1]*14}]}
        with self.assertRaisesRegex(ValueError,'合法目标与已检查'):
            a.hardware_trajectory_preview({'waypoints':[dict(q=requested)]})
        self.assertNotIn('bounded',a.hardware_tokens)

    def test_plan_backed_queue_keeps_the_checks_its_planner_already_ran(self):
        a=self.app
        a.geometry_id='geometry';a.queue=[{'q':[0.]*12,'speeds':[.2]*12}];a.plan={'plan_complete':True};a.plan_geometry_id='geometry'
        self.assertTrue(a.queue_is_plan_checked(a.queue))
        called=[]
        a.geometry.with_gaps=lambda *gaps:called.append(1) or SimpleNamespace(path_check=lambda *a,**k:{'ok':True})
        a.hardware_connected=lambda:True;a.control_request=lambda action,body=None,**kwargs:{'token':'t'}
        a.hardware_trajectory_preview({'waypoints':[{'q':[0.]*12,'speeds':[.2]*12}]})
        self.assertEqual(called,[],'a planner-produced queue must not be re-checked at execution time')
        # Any edit clears the plan, and the same rows are then checked.
        a.plan=None
        self.assertFalse(a.queue_is_plan_checked(a.queue))
        a.hardware_trajectory_preview({'waypoints':[{'q':[0.]*12,'speeds':[.2]*12}]})
        self.assertEqual(called,[1])
        # A queue planned under another geometry is not trusted either.
        a.plan={'plan_complete':True};a.plan_geometry_id='other'
        self.assertFalse(a.queue_is_plan_checked(a.queue))

    def test_execute_refuses_a_token_without_a_checked_path(self):
        a=self.app;a.q=np.zeros(12);a.jaw_mm=np.array([83.,83.])
        for token,record,message in (('missing',None,'预览已失效'),('unchecked',{'expires':time.monotonic()+60,'start':a.q.copy(),'start_jaw':a.jaw_mm.copy(),'collision_free':False},'未通过路径检查')):
            a.tokens={} if record is None else {token:record}
            with self.assertRaisesRegex(ValueError,message):a.action('/api/execute',{'token':token})

    def wrist_case(self, name='a'):
        a=self.app;a.geometry_id='geometry';a.wrist_selected={};a.control_mode='simulation'
        a.fit_result={'camera_T_world_optical':np.eye(4).tolist(),'joint_offsets_rad':{'left':[0.]*6,'right':[0.]*6},'T_link6_marker':{'24':np.eye(4).tolist(),'30':np.eye(4).tolist()}}
        snapshot=a.calibration_candidate('joint_candidate')
        contract={'head_snapshot':snapshot,'geometry_id':a.geometry_id,'nominal_mounts':{},'test_case':name}
        digest=hashlib.sha256(json.dumps(contract,sort_keys=True,allow_nan=False).encode()).hexdigest()
        dataset=a.output/('batch-live-'+name)/'capture';dataset.mkdir(parents=True)
        (dataset/'dataset.json').write_text(json.dumps({'wrist_contract':contract}))
        (dataset/'completion.json').write_text(json.dumps({'passed':True}))
        report={side:{'accepted':True,'coverage_complete':True,'observer':side,'simulation_only':True,'physical_validated':False,
                      'head_snapshot_sha256':snapshot['snapshot_sha256'],'contract_sha256':digest,'T_link6_camera':np.eye(4).tolist(),
                      'holdout':{'rms_px':1.,'positive_depth':True},'fit':{'positive_depth':True},'jacobian_rank':6,'optimizer_success':True} for side in ('left','right')}
        path=dataset/'wrist-fit-test.json';path.write_text(json.dumps(report))
        return path,contract,report

    def test_partial_wrist_catalog_rechecks_capture_provenance(self):
        from arx_env.joint_head_wrist_calibration import wrist_capture_coverage
        path,contract,report=self.wrist_case()
        contract['planned_frames']=[{'id':'observed'},{'id':'missing'}]
        dataset={'wrist_contract':contract,'frames':[{'id':'frame','planned_id':'observed'}]}
        completion={'passed':False}
        (path.parent/'dataset.json').write_text(json.dumps(dataset))
        (path.parent/'completion.json').write_text(json.dumps(completion))
        digest=hashlib.sha256(json.dumps(contract,sort_keys=True,allow_nan=False).encode()).hexdigest()
        for side in report.values():
            side.update(contract_sha256=digest,coverage_complete=False,
                        partial_capture_authorized=True,
                        capture_coverage=wrist_capture_coverage(dataset,completion))
        path.write_text(json.dumps(report))
        rows=self.app.wrist_results()
        self.assertEqual(len(rows),1)
        self.assertTrue(rows[0]['accepted'])
        self.assertEqual(rows[0]['capture_coverage']['missing_ids'],['missing'])
        report['left']['capture_coverage']['missing_ids']=[]
        path.write_text(json.dumps(report))
        self.assertEqual(self.app.wrist_results(),[])
        for side in report.values():side.pop('partial_capture_authorized')
        path.write_text(json.dumps(report))
        rows=self.app.wrist_results()
        self.assertEqual(len(rows),1)
        self.assertFalse(rows[0]['accepted'])

    def test_wrist_catalog_checks_contract_geometry_and_numerical_gate(self):
        path,contract,report=self.wrist_case();a=self.app
        self.assertEqual(len(a.wrist_results()),1)
        self.assertEqual(a.wrist_results()[0]['pair_status'],'complete')
        report['left']['holdout']['rms_px']=3.;path.write_text(json.dumps(report))
        # A rejected side no longer disappears: it is reported as an incomplete pair.
        rows=a.wrist_results()
        self.assertEqual(len(rows),1)
        self.assertEqual(rows[0]['pair_status'],'incomplete_left')
        self.assertFalse(rows[0]['left']['accepted']);self.assertTrue(rows[0]['right']['accepted'])
        self.assertTrue(rows[0]['left']['reasons'])
        contract['test_case']='modified'
        (path.parent/'dataset.json').write_text(json.dumps({'wrist_contract':contract}))
        # A contract the results no longer match keeps the record but marks it
        # incomplete, so the reason stays visible instead of vanishing.
        rows=a.wrist_results()
        self.assertEqual(len(rows),1)
        self.assertEqual(rows[0]['pair_status'],'incomplete_both')
        self.assertTrue(rows[0]['left']['reasons'] and rows[0]['right']['reasons'])
        contract['test_case']='a';(path.parent/'dataset.json').write_text(json.dumps({'wrist_contract':contract}))
        a.geometry_id='different';self.assertEqual(a.wrist_results(),[])
        contract['solver']='hybrid'
        contract['head_snapshot'].update(objective='sticker_corners_plus_mesh',acceptance={'image_passed':True,'image_checks':{'positive_depth':True}},optimizer={'success':True},mesh_metrics={'holdout':{'macro_coverage':.8,'macro_mean_penalized_px':5.}},metrics_rectified_px={'holdout':{'rms_px':1.}})
        (path.parent/'dataset.json').write_text(json.dumps({'wrist_contract':contract}))
        a.dataset=path.parent
        with self.assertRaisesRegex(ValueError,'几何'):a.fit_wrists()

    def test_wrist_selection_switches_only_display_and_clears_other_contract(self):
        self.wrist_case();a=self.app
        first=a.wrist_results()[0]
        a.action('/api/wrist-select',{'result_id':first['id']})
        self.assertEqual([p for p,_ in self.calls],['/wrist-config','/wrist-calibration','/wrist-calibration'])
        self.assertEqual(self.calls[-1][1]['contract_sha256'],first['result']['contract_sha256'])
        previous=dict(a.wrist_selected)
        self.wrist_case('b');a.wrist_selected=previous
        second=next(r for r in a.wrist_results() if 'batch-live-b' in r['file'])
        a.action('/api/wrist-select',{'result_id':second['id']})
        # Selecting one record cannot mix contracts: the choice is the pair itself.
        self.assertEqual(a.wrist_selected['id'],second['id'])
        self.assertEqual(a.wrist_selected['pair_status'],'complete')
        self.assertEqual(a.wrist_selected['contract_sha256'],second['result']['contract_sha256'])
        self.assertTrue(all(p not in ('/execute','/mirror') for p,_ in self.calls))
        a.remote['mirror']={'active':True};n=len(self.calls)
        a.action('/api/wrist-select',{'result_id':second['id']})
        self.assertEqual(len(self.calls),n+3)

    def test_wrist_restart_restores_only_same_contract_while_idle(self):
        self.wrist_case();a=self.app
        row=a.wrist_results()[0]
        a.wrist_selected={'id':row['id'],'contract_sha256':row['result']['contract_sha256'],'pair_status':'complete',
                          'file':row.get('file'),'left':row.get('left'),'right':row.get('right')}
        remote={'status':'READY','mirror':None};a.restore_wrist_display(remote)
        self.assertEqual([p for p,_ in self.calls],['/wrist-config','/wrist-calibration','/wrist-calibration'])
        remote.update(wrist_contract_sha256=a.wrist_selected['contract_sha256'],
                      wrist_calibration={s:{'id':a.wrist_selected['id']} for s in ('left','right')})
        self.calls.clear();a.restore_wrist_display(remote);self.assertEqual(self.calls,[])
        for update in ({'status':'EXECUTING'},{'status':'READY','mirror':{'active':True}}):
            a.restore_wrist_display({**remote,**update});self.assertEqual(self.calls,[])
        before=dict(a.wrist_selected)
        a.control_mode='hardware_sync';a.restore_wrist_display({'status':'READY','mirror':None});self.assertEqual(self.calls,[]);self.assertEqual(a.wrist_selected,before)

    def test_wrist_cold_restore_temporarily_clears_mirror_and_waits_for_config(self):
        self.wrist_case();a=self.app;row=a.wrist_results()[0]
        a.wrist_selected={'id':row['id'],'contract_sha256':row['result']['contract_sha256'],'pair_status':'complete'}
        selected=dict(a.wrist_selected);digest=row['result']['contract_sha256']
        states=iter([{'wrist_config_pending':True},{'wrist_config_pending':False,'wrist_contract_sha256':digest}])
        def request(path,body=None):
            self.calls.append((path,body))
            return next(states) if path=='/state' else {'queued':True}
        a.remote_request=request
        a.control_request=lambda *args:self.fail('display restoration must never command hardware')
        remote={'status':'MIRRORING','mirror':{'active':True}}
        a.restore_wrist_display(remote)
        self.assertEqual([path for path,_ in self.calls],['/mirror','/wrist-config','/wrist-calibration','/wrist-calibration','/state','/state'])
        self.assertEqual(self.calls[0],('/mirror',{'clear':True}))
        self.assertEqual(a.wrist_selected,selected)
        self.assertEqual(a.control_mode,'simulation')
        remote.update(wrist_contract_sha256=digest,wrist_calibration={side:{'id':row['id']} for side in ('left','right')})
        self.calls.clear()
        a.restore_wrist_display(remote)
        self.assertEqual(self.calls,[],'healthy mirror must not clear on every poll')

    def test_wrist_restore_skips_pipeline_owner_motion_and_capture(self):
        self.wrist_case();a=self.app;row=a.wrist_results()[0]
        a.wrist_selected={'id':row['id'],'contract_sha256':row['result']['contract_sha256'],'pair_status':'complete'}
        remote={'status':'MIRRORING','mirror':{'active':True}}
        a.pipeline_running=True;a.restore_wrist_display(remote);a.pipeline_running=False
        a.control_status={'owner':{'automatic_enabled':True}};a.restore_wrist_display(remote)
        a.control_status=None;a.hardware_feedback={'owner':{'motion':{'status':'AWAITING_CAPTURE'}}};a.restore_wrist_display(remote)
        a.hardware_feedback=None;a.restore_wrist_display({'status':'EXECUTING'})
        a.status='CAPTURING';a.restore_wrist_display(remote)
        self.assertEqual(self.calls,[])

    def test_wrist_restore_error_releases_mirror_lock_without_changing_selection(self):
        self.wrist_case();a=self.app;row=a.wrist_results()[0]
        a.wrist_selected={'id':row['id'],'contract_sha256':row['result']['contract_sha256'],'pair_status':'complete'}
        selected=dict(a.wrist_selected)
        def request(path,body=None):
            self.calls.append((path,body))
            if path=='/wrist-config':raise ValueError('renderer unavailable')
        a.remote_request=request
        with self.assertRaisesRegex(ValueError,'unavailable'):
            a.restore_wrist_display({'status':'MIRRORING','mirror':{'active':True}})
        acquired=[]
        def mirror_tick():
            with a.mode_lock:acquired.append(True)
        worker=threading.Thread(target=mirror_tick);worker.start();worker.join(1)
        self.assertEqual(acquired,[True])
        self.assertEqual(a.wrist_selected,selected)

    def test_complete_snapshot_preserves_legacy_identity_and_freezes_mounts(self):
        a=self.app
        parameters={'camera_T_world_optical':np.eye(4).tolist(),'joint_offsets_rad':{'left':[.01]*6,'right':[0.]*6}}
        legacy=hashlib.sha256(json.dumps(['test-batch','joint_candidate',parameters],sort_keys=True,allow_nan=False).encode()).hexdigest()
        mounts={'24':np.eye(4).tolist(),'30':np.eye(4).tolist()};mounts['30'][0][3]=.02
        baseline={**parameters,'T_link6_marker':{'24':np.eye(4).tolist(),'30':np.eye(4).tolist()}}
        a.fit_result={**parameters,'T_link6_marker':mounts,'fixed_joint_baseline':baseline}
        frozen=a.calibration_candidate('joint_candidate')
        self.assertEqual(frozen['id'],legacy)
        self.assertTrue(frozen['snapshot_complete']);self.assertIsNone(frozen['snapshot_error'])
        self.assertEqual(frozen['marker_ids'],{'left':24,'right':30})
        self.assertEqual(a.calibration_candidate('fixed_joint_baseline')['T_link6_marker'],baseline['T_link6_marker'])
        mounts['30'][0][3]=.03
        updated=a.calibration_candidate('joint_candidate')
        self.assertEqual(updated['id'],legacy)
        self.assertNotEqual(updated['snapshot_sha256'],frozen['snapshot_sha256'])
        self.assertEqual(frozen['T_link6_marker']['30'][0][3],.02)
        parameters['joint_offsets_rad']['left'][0]=.9
        self.assertEqual(frozen['joint_offsets_rad']['left'][0],.01)

    def test_incomplete_and_invalid_mounts_do_not_claim_complete_snapshot(self):
        a=self.app
        a.fit_result={'camera_T_world_optical':np.eye(4).tolist(),'joint_offsets_rad':{'left':[0.]*6,'right':[0.]*6}}
        missing=a.calibration_candidate('joint_candidate')
        self.assertFalse(missing['snapshot_complete']);self.assertIsNone(missing['snapshot_sha256'])
        self.assertEqual(missing['T_link6_marker'],{})
        a.fit_result['T_link6_marker']={'24':np.eye(4).tolist(),'30':np.zeros((4,4)).tolist()}
        invalid=a.calibration_candidate('joint_candidate')
        self.assertEqual(invalid['id'],missing['id']);self.assertFalse(invalid['snapshot_complete'])
        self.assertEqual(set(invalid['T_link6_marker']),{'24'})
        self.assertIn('30',invalid['snapshot_error'])

    def test_restore_legacy_parameters_never_guesses_missing_mounts(self):
        self.app.control_mode='simulation'
        a=self.app
        parameters={'id':'legacy-frozen','camera_T_world_optical':np.eye(4).tolist(),'joint_offsets_rad':{'left':[0.]*6,'right':[0.]*6}}
        choice={'id':parameters['id'],'candidate':'joint_candidate','batch':'old/capture','parameters':parameters}
        saved=a.output/'realtime-calibration.json';saved.write_text(json.dumps(choice))
        a.calibration_candidate=lambda *args:self.fail('must not read another fit to fill frozen mounts')
        a.restore_calibration()
        frozen=a.calibration['parameters']
        self.assertEqual(frozen['id'],'legacy-frozen');self.assertFalse(frozen['snapshot_complete'])
        self.assertEqual(frozen['T_link6_marker'],{});self.assertIsNone(frozen['snapshot_sha256'])
        self.assertEqual(json.loads(saved.read_text()),a.calibration)
        before=a.calibration;a.restore_calibration();self.assertEqual(a.calibration,before)

    def test_restore_complete_snapshot_checks_digest_and_retains_source_file(self):
        self.app.control_mode='simulation'
        a=self.app
        a.fit_result={'camera_T_world_optical':np.eye(4).tolist(),'joint_offsets_rad':{'left':[0.]*6,'right':[0.]*6},'T_link6_marker':{'24':np.eye(4).tolist(),'30':np.eye(4).tolist()}}
        parameters=a.calibration_candidate('joint_candidate')
        result={'id':parameters['id'],'candidate':'joint_candidate','batch':'test-batch','file':'fit-exact.json','parameters':parameters}
        a.calibration_results=lambda:[result]
        a.action('/api/calibration',{'result_id':parameters['id']})
        a.calibration=None;a.restore_calibration()
        self.assertEqual(a.calibration,result)
        saved=a.output/'realtime-calibration.json';damaged=json.loads(saved.read_text())
        damaged['parameters']['T_link6_marker']['24'][0][3]=.01
        saved.write_text(json.dumps(damaged));a.restore_calibration()
        self.assertFalse(a.calibration['parameters']['snapshot_complete'])
        self.assertIsNone(a.calibration['parameters']['snapshot_sha256'])
        self.assertIn('digest mismatch',a.calibration['parameters']['snapshot_error'])
        self.assertEqual(json.loads(saved.read_text()),damaged)
        a.restore_calibration();self.assertFalse(a.calibration['parameters']['snapshot_complete'])

    def test_catalog_excludes_failed_and_unsolved_batches(self):
        a=self.app;a.history=lambda:[{'id':v} for v in ('good','failed','empty')]
        report={'optimizer':{'success':True},'metrics_rectified_px':{'fit':{'corners':8,'rms_px':1.}},'camera_T_world_optical':np.eye(4).tolist(),'joint_offsets_rad':{'left':[0.]*6,'right':[0.]*6}}
        for name in ('good','failed','empty'):(a.output/name).mkdir()
        (a.output/'good/fit.json').write_text(json.dumps(report))
        (a.output/'failed/fit.json').write_text(json.dumps({**report,'optimizer':{'success':False}}))
        rows=a.calibration_results();self.assertEqual({r['batch'] for r in rows},{'good','failed'})
        a.selected_batch='empty';a.fit_result=None
        self.assertEqual(a.calibration_results(),rows)

    def test_realtime_candidate_keeps_source_batch(self):
        a=self.app;source=a.output/'old'/'capture';source.mkdir(parents=True)
        report={'metrics_rectified_px':{},'camera_T_world_optical':np.eye(4).tolist(),'joint_offsets_rad':{'left':[0.]*6,'right':[0.]*6}}
        (source/'fit.json').write_text(json.dumps(report))
        a.selected_batch='old/capture';a.fit_result=report
        original=a.calibration_candidate('joint_candidate')
        a.selected_batch='new/capture';a.fit_result=None
        self.assertEqual(a.calibration_candidate('joint_candidate','old/capture'),original)
        self.assertEqual(a.selected_batch,'new/capture')
        self.assertIsNone(a.fit_result)
        with self.assertRaises(ValueError):a.calibration_candidate('joint_candidate')

    def test_stream_frame_parser_and_freshness(self):
        import io
        raw,stamp=Workbench.read_stream_frame(io.BytesIO(b'--frame\r\nContent-Length: 3\r\nX-Frame-Timestamp: 12.5\r\n\r\nabc\r\n'))
        self.assertEqual((raw,stamp),(b'abc',12.5))
        with self.assertRaises(ValueError):Workbench.read_stream_frame(io.BytesIO(b'--frame\r\nContent-Length: 5\r\nX-Frame-Timestamp: 12.5\r\n\r\nabc'))
        self.app.live_frames={'actual':(raw,stamp,time.monotonic()),'preview':(raw,stamp,time.monotonic()-4)}
        self.assertEqual(self.app.live_frame('actual'),raw)
        with self.assertRaises(ValueError):self.app.live_frame('preview')
        self.app.remote['frame_timestamp_s']={'preview':12.5}
        self.assertEqual(self.app.live_frame('preview'),raw)

    def setUp(self):
        self.app=Workbench.__new__(Workbench)
        self.app.config={}
        self.app.hardware_history_lock=threading.Lock()
        a=self.app
        root=tempfile.TemporaryDirectory();self.addCleanup(root.cleanup);a.output=Path(root.name)
        a.mode_lock=threading.RLock();a.control_mode="hardware_sync";a.hardware_tokens=set()
        a.live=True;a.remote={'mode':'simulation','status':'READY','capabilities':{'jaw_gap_mm':True}};a.jaw_mm=np.array([83.,83.]);a.remote_received=time.monotonic();a.remote_error=None
        a.q=np.zeros(12);a.velocity_limits=np.ones(12);a.tokens={};a.lock=threading.RLock();a.stop=threading.Event();a.status='READY';a.importing=False;a.pending_capture=None;a.preview_revision=16;a.last_remote_timestamp=None;a.remote_advanced=0.;a.calibration=None;a.fit_result=None;a.selected_batch='test-batch'
        a.geometry=SimpleNamespace(_q=lambda q:np.array(q),limits=np.array([[-3.14,3.5]]*12))
        # The real geometry always offers both; previews now check the swept path in
        # the renderer branch too, so the fixture must model that surface.
        a.geometry.with_gaps=lambda *gaps:a.geometry
        a.geometry.path_check=lambda *args,**kwargs:{'ok':True}
        a.display=lambda q:self.fail('live preview must not compute meshes')
        self.calls=[]
        a.remote_request=lambda path,body=None,**kw:self.calls.append((path,body)) or {'capture_id':'test','revision':1}

    def test_actual_jaw_preview_maps_both_sides_without_confirm(self):
        a=self.app;calls=[]
        a.control_request=lambda action,body,**kwargs:calls.append((action,body)) or {'token':'mapped'}
        self.assertEqual(a.action('/api/hardware-jaw-preview',{'gap_mm':{'left':43,'right':43}}),{'token':'mapped'})
        self.assertEqual(calls,[('jaw-preview',{'active_arm':'both','sdk_gap_mm':{'left':39.4,'right':38.65}})])
        with self.assertRaises(ValueError):a.action('/api/hardware-jaw-preview',{'gap_mm':{'left':-1,'right':43}})
        self.assertEqual(len(calls),1)

    def test_hardware_sync_manual_queue_is_draft_without_preview_token(self):
        a=self.app
        a.hardware_connected=lambda:True
        a.hardware_sync_active=lambda:True
        a.current_pose=lambda:np.zeros(12)
        checked=[]
        a.require_checked_path=lambda rows,start:checked.append((rows,start.copy()))
        a.geometry_id='geometry';a.queue=[];a.plan=None;a.plan_geometry_id='geometry';a.persist=lambda:None
        result=a.action('/api/queue',{'q':[.1]*12,'jaw_gap_mm':[80.,80.],'speeds':[.2]*12})
        self.assertEqual(result,{'message':'已加入队列'})
        self.assertEqual(len(a.queue),1)
        self.assertEqual(a.queue[0]['jaw_gap_mm'],[80.,80.])
        self.assertEqual(len(checked),1)
        self.assertEqual(self.calls,[],'queueing must not call the renderer preview')
        self.assertEqual(a.hardware_tokens,set())

    def test_trusted_seed_legacy_capture_skips_unneeded_path_check(self):
        a=self.app;a.control_mode='simulation';a.queue=[{'q':[0.]*12,'capture':True}]
        a.plan=None;a.standard_motion_seed_active=lambda:True
        a.queue_is_plan_checked=lambda rows:False
        a.require_checked_path=lambda *args:self.fail('trusted seed must not recheck collisions')
        result=a.action('/api/batch',{})
        self.assertEqual(result['capture_id'],'test')
        self.assertEqual(self.calls[-1][0],'/execute')

    def test_hardware_workflow_status_is_not_renderer_mirror_status(self):
        a=self.app
        for name in ('hardware_error','mirror_error','reason','import_error','evaluation','batch','plan'):
            setattr(a,name,None)
        a.hardware_connected=lambda:False;a.hardware_sync_active=lambda:False;a.control_available=lambda:False
        a.hardware_camera=None;a.wrist_cameras={};a.csrf='test';a.queue=[];a.frames=[]
        a.geometry.limits=np.zeros((12,2));a.geometry.mount_chain={};a.actual={'parts':[],'camera':None}
        a.calibration_state=lambda:{};a.wrist_results=lambda:[];a.counts=lambda:{}
        a.history=lambda:[];a.library_state=lambda:{};a.diagnostics=lambda:{}
        a.remote['status']='MIRRORING'
        self.assertEqual(a.state()['status'],'READY')
        a.status='CAPTURING';self.assertEqual(a.state()['status'],'CAPTURING')
        a.status='READY';a.control_mode='simulation'
        self.assertEqual(a.state()['status'],'MIRRORING')

    def test_hardware_control_is_independent_of_dsw(self):
        a=self.app;a.hardware_feedback={'owner':{'calibration_feedback':{'qpos':[0.]*14}}};a.hardware_error=None;a.hardware_received=time.monotonic();a.remote_error='DSW disconnected'
        calls=[];a.control_request=lambda action,body,**kwargs:calls.append((action,body)) or {'ok':True}
        a.action('/api/stop',{})
        self.assertEqual(calls[-1],('stop',{}))
        a.hardware_trajectory_preview({'waypoints':[{'q':[0.]*12,'speeds':[.2]*12}]})
        self.assertEqual(calls[-1][0],'trajectory-preview')
        self.assertFalse(self.calls)
        a.hardware_received=time.monotonic()-2
        with self.assertRaisesRegex(ValueError,'断开'):a.hardware_trajectory_preview({'waypoints':[]})

    def test_mode_isolates_robot_and_invalidates_review(self):
        a=self.app
        a.hardware_feedback={'owner':{'calibration_feedback':{'qpos':[.4]*14}}};a.hardware_error=None;a.hardware_received=time.monotonic()
        hardware=[]
        a._control_request=lambda action,body=None:hardware.append((action,body)) or {'token':'reviewed'}
        a.queue=[];a.plan=None;a.plan_geometry_id='test'
        with tempfile.TemporaryDirectory() as root:
            a.output=Path(root)
            a.control_request('trajectory-preview',{})
            a.tokens['old']={}
            a.action('/api/control-mode',{'mode':'simulation'})
            self.assertFalse(a.hardware_sync_active());self.assertTrue(a.hardware_connected())
            self.assertEqual(json.loads((a.output/'workbench-state.json').read_text())['control_mode'],'simulation')
            self.assertFalse(a.tokens);self.assertFalse(a.hardware_tokens)
            self.assertEqual(self.calls[-1],('/mirror',{'clear':True}))
            for action in ('trajectory-preview','jaw-preview','motion-confirm'):
                with self.assertRaisesRegex(ValueError,'纯仿真'):a.control_request(action,{'token':'reviewed'})
            p=a.preview({'q':[.1]*12});a.action('/api/execute',{'token':p['token']})
            self.assertEqual(self.calls[-1][0],'/execute')
            a.action('/api/stop',{})
            self.assertEqual(self.calls[-1][0],'/stop')
            self.assertEqual(len(hardware),1)
            a.action('/api/control-mode',{'mode':'hardware_sync'})
            self.assertTrue(a.hardware_sync_active())
            with self.assertRaisesRegex(ValueError,'预览过期'):a.control_request('motion-confirm',{'token':'reviewed'})
            self.assertEqual(len(hardware),1,'switching back must never move the robot')
            with self.assertRaisesRegex(ValueError,'invalid control mode'):a.set_control_mode('other')
            a.remote['status']='EXECUTING'
            with self.assertRaisesRegex(ValueError,'active'):a.set_control_mode('simulation')

    def test_simulation_mode_mirror_loop_does_not_follow_connected_robot(self):
        from unittest.mock import patch
        a=self.app;a.control_mode='simulation';a.hardware_connected=lambda:True;a.mirror_error=None
        a.hardware_feedback={'owner':{'pid':123,'calibration_feedback':{'qpos':[0.]*14}},'source_age_s':0};a.hardware_received=time.monotonic()
        with patch('joint_head_workbench.time.sleep',side_effect=KeyboardInterrupt):
            with self.assertRaises(KeyboardInterrupt):a.mirror_hardware()
        self.assertEqual(self.calls,[]);self.assertIsNone(a.mirror_error)

    def test_preview_mailbox_keeps_latest_target_without_remote_wait(self):
        a=self.app;a.preview_condition=threading.Condition();a.preview_pending=None
        a.action('/api/preview-target',{'q':[.1]*12,'revision':1})
        a.action('/api/preview-target',{'q':[.2]*12,'revision':2})
        self.assertEqual(a.preview_pending['revision'],2)
        self.assertEqual(a.preview_pending['q'],[.2]*12)
        self.assertEqual(self.calls,[])

    def test_video_proxy_reconnects_after_upstream_failure(self):
        from joint_head_workbench import handler
        from unittest.mock import patch, MagicMock
        a=self.app;a.live={'url':'http://127.0.0.1:8118'};a.live_token='test'
        response=MagicMock();response.__enter__.return_value=response
        response.read1.side_effect=[b'new-frame',b'']
        a.http=MagicMock();a.http.open.side_effect=[OSError('tunnel disconnected'),response]
        h=handler(a).__new__(handler(a))
        with patch('joint_head_workbench.time.sleep'):
            stream=h.dsw_stream('actual')
            self.assertEqual(next(stream),b'new-frame')
            self.assertEqual(a.http.open.call_count,2)
            stream.close()

    def test_preview_and_motion_authority(self):
        a=self.app;a.control_mode='simulation'
        p=a.preview({'q':[.1]*12,'revision':17})
        self.assertEqual(self.calls[-1],('/preview',{'q':[.1]*12,'jaw_gap_mm':[83.,83.],'revision':17}))
        self.assertEqual(a.q.tolist(),[0.]*12)
        a.action('/api/execute',{'token':p['token']})
        self.assertEqual(self.calls[-1][0],'/execute')
        self.assertEqual(self.calls[-1][1]['expected_start'],[0.]*12)
        self.assertFalse(self.calls[-1][1]['waypoints'][0]['capture'])
        with self.assertRaises(ValueError):a.action('/api/execute',{'token':p['token']})

    def test_hardware_display_preview_never_checks_path_or_grants_authority(self):
        a=self.app;a.control_mode='hardware_sync'
        def forbidden(*args):raise AssertionError('display must not run costly path checks')
        a.geometry.with_gaps=forbidden
        p=a.preview({'q':[.1]*12})
        self.assertTrue(p['display_only']);self.assertIsNone(p['token'])
        self.assertIsNone(p['collision_free']);self.assertFalse(a.tokens)

    def test_missing_jaw_capability_rejects_instead_of_ignoring(self):
        self.app.remote['capabilities']={}
        with self.assertRaisesRegex(ValueError,'夹爪开度'):
            self.app.preview({'q':[0.]*12,'jaw_gap_mm':[0,40]})
        self.assertEqual(self.calls,[])

    def test_stale_mismatch_hardware_and_stop(self):
        a=self.app
        p=a.preview({'q':[.1]*12})
        a.q[0]=.1
        with self.assertRaises(ValueError):a.action('/api/execute',{'token':p['token']})
        a.remote_received=time.monotonic()-4
        with self.assertRaises(ValueError):a.preview({'q':[.1]*12})
        a.control_mode='simulation'
        a.action('/api/stop',{})
        self.assertEqual(self.calls[-1][0],'/stop','stop must be attempted even on stale feedback')
        a.remote_received=time.monotonic();a.remote['mode']='hardware'
        with self.assertRaises(ValueError):a.require_remote()
        with self.assertRaisesRegex(ValueError,'Hardware adapter'):Workbench({'mode':'hardware'},'/unused')

    def test_pending_capture_blocks_new_execution(self):
        a=self.app
        p=a.preview({'q':[.1]*12});a.pending_capture='previous'
        for path in ('/api/execute','/api/run-batch','/api/batch'):
            with self.assertRaisesRegex(ValueError,'上一批'):a.action(path,{'token':p['token']})
        self.assertNotIn('/execute',[call[0] for call in self.calls])
        a.control_mode='simulation'
        a.action('/api/stop',{})
        self.assertEqual(self.calls[-1][0],'/stop')

    def test_redirect_refused(self):
        from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
        from urllib.request import build_opener,ProxyHandler,Request
        from urllib.error import HTTPError
        seen=[]
        class Redirect(BaseHTTPRequestHandler):
            def log_message(self,*args):pass
            def do_GET(self):
                seen.append(self.path)
                self.send_response(302);self.send_header('Location','http://127.0.0.1:'+str(self.server.server_port)+'/credentials-must-not-arrive');self.end_headers()
        server=ThreadingHTTPServer(('127.0.0.1',0),Redirect)
        thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        try:
            opener=build_opener(ProxyHandler({}),NoRedirect())
            with self.assertRaises(HTTPError):opener.open(Request('http://127.0.0.1:'+str(server.server_port)+'/state',headers={'Authorization':'Bearer test-only'}),timeout=2)
            self.assertEqual(seen,['/state'])
        finally:server.shutdown();server.server_close();thread.join()

    def test_calibration_explicit_candidate_and_identity(self):
        self.app.control_mode='simulation'
        a=self.app
        candidate={'camera_T_world_optical':np.eye(4).tolist(),'joint_offsets_rad':{'left':[.01]*6,'right':[0.]*6}}
        a.fit_result={**candidate,'fixed_joint_baseline':{**candidate,'joint_offsets_rad':{'left':[0.]*6,'right':[0.]*6}}}
        with self.assertRaises(ValueError):a.action('/api/calibration',{})
        result={'id':a.calibration_candidate('joint_candidate')['id'],'candidate':'joint_candidate','batch':'test-batch','parameters':a.calibration_candidate('joint_candidate')}
        a.calibration_results=lambda:[result]
        before=a.q.copy();a.action('/api/calibration',{'result_id':a.calibration_results()[0]['id']})
        self.assertEqual(self.calls[-1][0],'/calibration');np.testing.assert_array_equal(a.q,before)
        self.assertEqual(a.calibration['batch'],'test-batch')
        selected=a.calibration['id'];a.remote['calibration']={'id':selected,'ready':False}
        self.assertFalse(a.calibration_state()['ready'])
        a.remote['calibration']['ready']=True;self.assertTrue(a.calibration_state()['ready'])
        a.remote['calibration']['id']='old';self.assertFalse(a.calibration_state()['ready'])
        self.assertNotEqual(a.calibration_candidate('fixed_joint_baseline')['id'],selected)
        a.action('/api/calibration',{'clear':True});self.assertIsNone(a.calibration)
        a.fit_result=None
        with self.assertRaises(ValueError):a.calibration_candidate('joint_candidate')

    def test_history_preserves_frozen_live_candidate(self):
        self.app.control_mode='simulation'
        a=self.app
        candidate={'camera_T_world_optical':np.eye(4).tolist(),'joint_offsets_rad':{'left':[.01]*6,'right':[0.]*6}}
        a.fit_result=candidate
        result={'id':a.calibration_candidate('joint_candidate')['id'],'candidate':'joint_candidate','batch':'test-batch','parameters':a.calibration_candidate('joint_candidate')}
        a.calibration_results=lambda:[result]
        a.action('/api/calibration',{'result_id':result['id']})
        frozen=a.calibration.copy();self.calls.clear()
        with tempfile.TemporaryDirectory() as root:
            a.output=Path(root).resolve();dataset=a.output/'other-batch';dataset.mkdir()
            (dataset/'completion.json').write_text(json.dumps({'captured':0}))
            a.load_batch('other-batch')
            self.assertEqual(a.selected_batch,'other-batch')
            self.assertIsNone(a.fit_result)
            self.assertEqual(a.calibration,frozen)
            self.assertEqual(self.calls,[],'historical browsing must not mutate remote rendering')
            self.assertEqual(a.calibration['batch'],'test-batch')
            a.remote['calibration']={'id':frozen['id'],'ready':True}
            self.assertTrue(a.calibration_state()['ready'])
            self.assertEqual(a.calibration_state()['available'],[])

    def test_remote_timestamp(self):
        a=self.app
        a.last_remote_timestamp=100.;a.remote_advanced=time.monotonic()-4
        a.remote_request=lambda *args,**kwargs:{'mode':'simulation','q':[0.]*12,'timestamp_s':100.}
        # One polling iteration without starting a background thread.
        from unittest.mock import patch
        with patch('joint_head_workbench.time.sleep',side_effect=KeyboardInterrupt):
            with self.assertRaises(KeyboardInterrupt):a.monitor_remote()
        self.assertIn('stale',a.remote_error)
        with self.assertRaises(ValueError):a.require_remote()

if __name__=='__main__':unittest.main()
