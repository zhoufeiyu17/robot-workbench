"""Exit ordering, failure/cancellation boundaries and restored collector Web."""
from copy import deepcopy
from types import SimpleNamespace
import unittest
from unittest.mock import Mock,patch

from robot_workbench.management import WorkbenchManager
from robot_workbench.operations import OperationError
from scripts import workbench_deploy


class EnvironmentExitTest(unittest.TestCase):
    def test_home_uses_new_review_native_manual_path_and_bounded_jaws(self):
        m=object.__new__(WorkbenchManager);m.port=18878;m.environment='physical';m.timeouts={'state':.1,'operation':.3}
        m.target=lambda:dict(backend_instance='backend');m.health_cache={'runtime':{'runtime_nonce':'cpu'}};m._request_control_stop=Mock()
        runtime=dict(connected=True,feedback_healthy=True,stationary=True,automatic_enabled=False,state='STOPPED_DAMPING_HOLD',runtime_nonce='cpu',command_epoch=2,holder='inference')
        state=dict(server_instance_id='backend',csrf='fixture',runtime=runtime,workflow={},manual=dict(jaw_gaps_mm=[80.2,89.1],jaw_command_ranges_mm=[[0,80],[0,89]],positions=[0.]*12))
        requests=[]
        def transport(port,path,body=None,**kw):
            if body is None:return deepcopy(state)
            requests.append((path,body))
            if path.endswith('/handoff'):runtime['holder']='research';return {}
            if path.endswith('/manual_preview'):
                self.assertEqual(body['q12'],[0.]*12);self.assertEqual(body['speeds12'],[.1]*12);self.assertEqual(body['jaw_gap_mm'],[80,89])
                state['workflow']['review']=dict(id='new-review',request_job_id='prep');return {'job_id':'prep'}
            if path.endswith('/manual_execute'):
                self.assertEqual(body['review_id'],'new-review');state['manual']['execution']=dict(id='new-review',status='completed');return {}
            raise AssertionError(path)
        with patch('robot_workbench.management.json_request',side_effect=transport):result=m.home_for_exit({'request_id':'test'},lambda *a,**k:None)
        self.assertTrue(result['completed']);m._request_control_stop.assert_not_called()
        self.assertEqual([x[0].split('/')[-1]for x in requests],['handoff','manual_preview','manual_execute'])

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
