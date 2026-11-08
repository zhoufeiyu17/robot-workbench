"""Exit ordering, failure/cancellation boundaries and restored collector Web."""
from copy import deepcopy
from types import SimpleNamespace
import unittest
from unittest.mock import Mock,patch

from robot_workbench.management import WorkbenchManager
from robot_workbench.operations import OperationError
from scripts import workbench_deploy


class EnvironmentExitTest(unittest.TestCase):
    def test_exit_verification_keeps_released_and_wrong_collector_facts_separate(self):
        m=self.manager(False)
        m.refresh=Mock(return_value={'services':{name:dict(state='stopped',running=False)for name in m.services}})
        with patch.object(workbench_deploy,'verify_unit',return_value=dict(ActiveState='active',MainPID='7')),\
                patch.object(workbench_deploy,'http_json',return_value=dict(robot_vin='another-device',orphan_alive=False)):
            proof=WorkbenchManager.verify(m,dict(action='exit_environment',sdk_was_disconnected=True))
        self.assertFalse(proof['ok']);self.assertTrue(proof['environment_exited']);self.assertTrue(proof['sdk_release_verified'])
        self.assertFalse(proof['collection_web_ready']);self.assertFalse(proof['collection_web']['ready'])

    def test_default_home_uses_actual_calibration_run_and_fixed_recording_policy(self):
        from test_shared_target_routes import SharedTargetRoutesTest
        h=SharedTargetRoutesTest();h.setUp();self.addCleanup(h.doCleanups);h.create();made=h.capture_factory()
        factory=h.runtime.owner.recording_factory
        def ready(payload,on_fault):
            recording=factory(payload,on_fault);recording.prepared.set();recording.ready.set();return recording
        ready.recording_backend='capture_session_v1';ready.supports_deferred_preparation=True
        h.runtime.owner.recording_factory=ready
        m=object.__new__(WorkbenchManager);m.environment='physical';m.port=18878;m.config={}
        m.timeouts={'state':.1,'operation':5.};m.target=lambda:{'backend_instance':'cpu'}
        m.health_cache={'runtime':{'runtime_nonce':h.runtime.nonce}};m._request_control_stop=Mock()
        review=None;executed=False
        def transport(port,path,body=None,**kwargs):
            nonlocal review,executed
            if body is None:
                h.advance(5);runtime=h.runtime.fact_snapshot();runner=h.runtime.owner.runner
                state=dict(server_instance_id='cpu',csrf='fixture',runtime=runtime,workflow={},manual=dict(
                    jaw_gaps_mm=[0.,0.],jaw_command_ranges_mm=[[0,80],[0,80]],positions=[*h.raw.q[:6],*h.raw.q[7:13]]))
                if review is not None:state['workflow']['review']=dict(id='review',request_job_id='prep')
                if executed:state['manual']['execution']=dict(id='review',status='completed'if runner and runner.status=='READY'else'failed'if runner and runner.status=='FAULT'else'running')
                return state
            self.assertEqual(body['business'],'calibration')
            if path.endswith('/manual_preview'):
                review=h.runtime.submit(h.creds['calibration'],dict(action='preview_workbench',waypoints=[dict(
                    q12=body['q12'],speeds12=body['speeds12'],jaw_gap_mm=body['jaw_gap_mm'],jaw_speed_mm_s=body['jaw_speed_mm_s'])]))
                return {'job_id':'prep'}
            if path.endswith('/manual_execute'):h.confirm('calibration',review);executed=True;return {}
            raise AssertionError(path)
        with patch('robot_workbench.management.json_request',side_effect=transport):result=m.home_for_exit({'request_id':'home'},Mock())
        self.assertTrue(result['completed']);self.assertEqual(result['speed_rad_s'],.1);self.assertTrue(made)
        self.assertEqual(made[-1].context['business_kind'],'calibration')
        policy=made[-1].context['control_policy'];self.assertEqual(policy['parameters']['recording']['purpose_profile'],'data_required')
        self.assertEqual(policy['policy_hash'],h.runtime.execution_calibration['control_policy']['policy_hash'])
        self.assertTrue(made[-1].done.is_set());m._request_control_stop.assert_not_called()

    def test_arm_only_home_keeps_jaw_targets_out_of_the_waypoints(self):
        m=object.__new__(WorkbenchManager);m.port=18878;m.environment='physical'
        m.config={'exit_home_keep_jaws':True};m.timeouts={'state':.1,'operation':.5}
        m.target=lambda:dict(backend_instance='backend');m.health_cache={'runtime':{'runtime_nonce':'cpu'}}
        m._request_control_stop=Mock(return_value={'status':'accepted','receipt':{'stop':{'barrier_epoch':10}}})
        runtime=dict(connected=True,feedback_healthy=True,stationary=True,automatic_enabled=False,
            state='READY',runtime_nonce='cpu',command_epoch=2,holder='inference')
        state=dict(server_instance_id='backend',csrf='fixture',runtime=runtime,workflow={},manual=dict(positions=[.01]*12))
        sides=[]
        def transport(port,path,body=None,**kwargs):
            if body is None:
                if sides:
                    runtime.update(state='STOPPED_DAMPING_HOLD',stop_receipt=dict(barrier_epoch=10,prior_calls_complete=True,stop_hold_return_confirmed=True))
                return deepcopy(state)
            self.assertEqual(body['business'],'calibration')
            if path.endswith('/handoff'):runtime['holder']='calibration';return {}
            if path.endswith('/manual_preview'):
                sides.append(body['active_arm']);self.assertNotIn('jaw_gap_mm',body)
                start=0 if body['active_arm']=='left'else 6
                self.assertEqual(body['q12'][start:start+6],[0.]*6)
                self.assertEqual(body['speeds12'],[.1]*12)
                state['workflow']['review']=dict(id='review-'+sides[-1],request_job_id=sides[-1])
                return {'job_id':sides[-1]}
            if path.endswith('/manual_execute'):
                state['manual']['execution']=dict(id=body['review_id'],status='completed')
                return {}
            raise AssertionError(path)
        with patch('robot_workbench.management.json_request',side_effect=transport):
            result=m.home_for_exit({'request_id':'home'},Mock())
        self.assertEqual(sides,['left','right']);self.assertTrue(result['completed'])
        m._request_control_stop.assert_called_once_with('home-between-arms',expected_nonce='cpu')
        self.assertIn('retain_native_begin_command',result['jaw_policy'])

    def test_home_uses_new_review_native_manual_path_and_bounded_jaws(self):
        m=object.__new__(WorkbenchManager);m.port=18878;m.environment='physical';m.timeouts={'state':.1,'operation':.3}
        m.target=lambda:dict(backend_instance='backend');m.health_cache={'runtime':{'runtime_nonce':'cpu'}};m._request_control_stop=Mock()
        runtime=dict(connected=True,feedback_healthy=True,stationary=True,automatic_enabled=False,state='STOPPED_DAMPING_HOLD',runtime_nonce='cpu',command_epoch=2,holder='inference')
        state=dict(server_instance_id='backend',csrf='fixture',runtime=runtime,workflow={},manual=dict(jaw_gaps_mm=[80.2,89.1],jaw_command_ranges_mm=[[0,80],[0,89]],positions=[0.]*12))
        requests=[]
        def transport(port,path,body=None,**kw):
            if body is None:return deepcopy(state)
            requests.append((path,body))
            self.assertEqual(body['business'],'calibration')
            if path.endswith('/handoff'):runtime['holder']='calibration';return {}
            if path.endswith('/manual_preview'):
                self.assertEqual(body['q12'],[0.]*12);self.assertEqual(body['speeds12'],[.1]*12);self.assertEqual(body['jaw_gap_mm'],[80,89])
                state['workflow']['review']=dict(id='new-review',request_job_id='prep');return {'job_id':'prep'}
            if path.endswith('/manual_execute'):
                self.assertEqual(body['review_id'],'new-review');state['manual']['execution']=dict(id='new-review',status='completed');return {}
            raise AssertionError(path)
        with patch('robot_workbench.management.json_request',side_effect=transport):result=m.home_for_exit({'request_id':'test'},lambda *a,**k:None)
        self.assertTrue(result['completed']);m._request_control_stop.assert_not_called()
        self.assertEqual([x[0].split('/')[-1]for x in requests],['handoff','manual_preview','manual_execute'])

    def test_home_waits_for_recording_seal_and_timeout_never_reports_completion(self):
        for finishes in (True,False):
            with self.subTest(finishes=finishes):
                m=object.__new__(WorkbenchManager);m.port=18878;m.environment='physical'
                m.timeouts={'state':.1,'operation':.4 if finishes else .01}
                m.target=lambda:dict(backend_instance='backend')
                m.health_cache={'runtime':{'runtime_nonce':'cpu'}};m._request_control_stop=Mock()
                runtime=dict(connected=True,feedback_healthy=True,stationary=True,automatic_enabled=False,
                    state='READY',runtime_nonce='cpu',command_epoch=2,holder='calibration')
                state=dict(server_instance_id='backend',csrf='fixture',runtime=runtime,workflow={},
                    manual=dict(jaw_gaps_mm=[30.,30.],jaw_command_ranges_mm=[[0,80],[0,80]],positions=[0.]*12))
                polls=[];actions=[]
                def transport(port,path,body=None,**kwargs):
                    if body is None:
                        if 'execution'in state['manual']:
                            polls.append(True);runtime['recording']['done']=finishes and len(polls)>=3
                        return deepcopy(state)
                    actions.append(path.rsplit('/',1)[-1])
                    if path.endswith('/manual_preview'):
                        state['workflow']['review']=dict(id='review',request_job_id='prep');return {'job_id':'prep'}
                    if path.endswith('/manual_execute'):
                        state['manual']['execution']=dict(id='review',status='completed')
                        runtime['recording']=dict(done=False);return {}
                    raise AssertionError(path)
                with patch('robot_workbench.management.json_request',side_effect=transport):
                    if finishes:
                        self.assertTrue(m.home_for_exit({'request_id':'home'},Mock())['completed'])
                        self.assertGreaterEqual(len(polls),3);m._request_control_stop.assert_not_called()
                    else:
                        with self.assertRaises(OperationError)as caught:m.home_for_exit({'request_id':'home'},Mock())
                        self.assertEqual(caught.exception.code,'home_unconfirmed')
                        m._request_control_stop.assert_called_once_with('home-home-failed-stop',expected_nonce='cpu')
                self.assertEqual(actions,['manual_preview','manual_execute'])

    def manager(self,connected=True):
        m=object.__new__(WorkbenchManager);m.environment='physical';m.port=18878;m.stop_generation=0
        m.config=dict(device_id='arx13',control_owner_descriptor='/cpu/owner.json',collection_handoff=dict(unit='prometheus-arx13-web.service',files={'/cpu/unit':'a'},status_url='http://127.0.0.1:18980/api/status'))
        m.timeouts=dict(backend=.1,state=.1,operation=.2)
        m.health_cache={'runtime':dict(connected=connected,runtime_nonce='cpu'if connected else None),'busy':False}
        m.target=Mock(return_value=dict(manager_instance='manager',backend_instance='backend',deployment_sha256='a',services={}))
        m.refresh=Mock(side_effect=lambda:deepcopy(m.health_cache));m.stop_current_control=Mock(return_value={'confirmed':True})
        m.home_for_exit=Mock(return_value=dict(completed=True,runtime_nonce='cpu',command_epoch=3))
        m.rows={k:dict(state='running',running=True)for k in ('model','cameras','workbench','control_owner','triton_control')}
        m.services={k:SimpleNamespace(snapshot=lambda k=k:deepcopy(m.rows[k]))for k in m.rows};m.triton_preview=SimpleNamespace(catalog=lambda:dict(busy=False));m.calls=[]
        def run(action,request,progress):
            m.calls.append(action)
            if action=='release_control':m.health_cache['runtime'].update(connected=False,runtime_nonce=None)
            else:m.rows[{'stop_model':'model','stop_cameras':'cameras'}[action]].update(state='stopped',running=False)
            return dict(action=action)
        m._run=Mock(side_effect=run);m.verify=Mock(return_value=dict(ok=True,evidence={'release':dict(status='released',sdk_close_returned=True)}))
        def stop(role,action,progress):m.calls.append('stop_'+role);m.rows[role].update(state='stopped',running=False);return deepcopy(m.rows[role])
        m.service_action=Mock(side_effect=stop)
        return m

    def run_exit(self,m,progress=lambda *a,**k:None):
        def restored(*a,**k):
            self.assertTrue(all(x['state']=='stopped'for x in m.rows.values()))
            m.calls.append('restore_collection_web');return dict(ready=True,collection_started=False)
        with patch.object(workbench_deploy,'verify_unit',return_value={'ActiveState':'inactive'}),patch.object(workbench_deploy,'restore_collection_web',side_effect=restored),patch('robot_workbench.control_owner.ControlChannel')as channel:
            channel.return_value.call.return_value=dict(connected=False,runtime_nonce=None)
            return m.exit_environment(dict(request_id='exit-test',expected_target=m.target(),arms_supported=False),progress)

    def test_exit_requires_home_then_release_and_all_processes_before_web(self):
        m=self.manager();result=self.run_exit(m)
        m.stop_current_control.assert_called_once();m.home_for_exit.assert_called_once()
        self.assertEqual(m.calls,['release_control','stop_model','stop_cameras','stop_triton_control','stop_workbench','stop_control_owner','restore_collection_web'])
        self.assertTrue(result['environment_exited']);self.assertFalse(result['collection_started'])
        self.assertEqual(result['release_basis'],'reviewed_normal_home_completed')
        self.assertTrue(WorkbenchManager.release_verified(result['release_receipt']))

    def test_disconnected_exit_never_homes_or_reacquires(self):
        m=self.manager(False);m.rows['control_owner'].update(state='stopped',running=False)
        result=self.run_exit(m);m.home_for_exit.assert_not_called();m.stop_current_control.assert_not_called()
        self.assertTrue(result['sdk_was_disconnected']);self.assertNotIn('release_control',m.calls)

    def test_failed_home_or_unconfirmed_release_never_reaches_service_shutdown(self):
        m=self.manager();m.home_for_exit.side_effect=OperationError('home_failed','home failed')
        with self.assertRaisesRegex(OperationError,'home failed'):self.run_exit(m)
        self.assertEqual(m.calls,[])
        m=self.manager();m.verify.return_value={'ok':False}
        with self.assertRaisesRegex(OperationError,'SDK释放未确认'):self.run_exit(m)
        self.assertEqual(m.calls,['release_control'])

    def test_unknown_resource_exit_does_not_restore_collection(self):
        m=self.manager();m.verify.side_effect=[dict(ok=True,evidence={'release':dict(status='released',sdk_close_returned=True)}),dict(ok=False)]
        with self.assertRaisesRegex(OperationError,'model退出未确认'):self.run_exit(m)
        self.assertEqual(m.calls,['release_control','stop_model'])

    def test_stop_at_progress_boundary_prevents_remaining_release_steps(self):
        m=self.manager()
        def stop(phase,**kw):
            if phase=='releasing_control':raise OperationError('operator_stop','operator stopped')
        with self.assertRaisesRegex(OperationError,'operator stopped'):self.run_exit(m,stop)
        self.assertEqual(m.calls,[])

    def test_duplicate_progress_does_not_write_but_still_checks_stop(self):
        m=self.manager();calls=Mock()
        def work(action,request,progress):
            progress('waiting');progress('waiting');m.stop_generation+=1;progress('waiting')
        m._run=work
        with self.assertRaisesRegex(OperationError,'停止已撤销'):m.run('exit_environment',{},calls)
        calls.assert_called_once_with('waiting')

    def test_restore_only_starts_pinned_web_and_never_collection(self):
        pin=dict(unit='prometheus-arx13-web.service',files={'/cpu/unit':'a'},status_url='http://127.0.0.1:18980/api/status')
        inactive=dict(ActiveState='inactive',MainPID='0',ControlPID='0',ControlGroup='')
        active=dict(ActiveState='active',MainPID='7',ControlPID='0',ControlGroup='/cpu')
        with patch.object(workbench_deploy,'verify_unit',side_effect=[inactive,active]),patch.object(workbench_deploy,'http_json',return_value=dict(robot_vin='arx13',orphan_alive=False,manager_alive=False,collect_status='idle'))as http,patch.object(workbench_deploy.subprocess,'run')as command:
            result=workbench_deploy.restore_collection_web(pin,'arx13')
        command.assert_called_once_with(['systemctl','--user','start','--no-block',pin['unit']],check=True,timeout=10)
        http.assert_called_once_with(pin['status_url']);self.assertFalse(result['collection_started'])

if __name__=='__main__':unittest.main()
