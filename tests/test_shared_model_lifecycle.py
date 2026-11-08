"""Consumer lifecycle isolation over real local HTTP; CPU fake worker only."""
from contextlib import contextmanager
from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace as NS
import threading
import unittest
from unittest.mock import Mock,patch

import test_model_session
import test_environment_exit
from robot_workbench.inference_workflow import InferenceWorkflow
from robot_workbench.management import WorkbenchManager
from robot_workbench.model_session import binding
from robot_workbench.operations import OperationError
from test_inference_transport import request
from scripts import workbench_deploy,start_registered_workbench


class SharedLifecycleTests(unittest.TestCase):
    def setUp(self):
        self.fixture=test_model_session.SessionTests();self.fixture.setUp()
        self.addCleanup(self.fixture.tearDown)
        self.a=self.fixture.pin(self.fixture.a,'workbench')
        self.b=self.fixture.pin(self.fixture.b,'collector')
        self.flow=object.__new__(InferenceWorkflow)
        self.flow.device=NS(maintenance=True);self.flow.lock=threading.RLock();self.flow.generation=0
        self.flow.client=self.a;self.flow.model_client=self.fixture.a;self.flow.model_release_unconfirmed=False
        self.flow.snapshot=lambda:{'busy':False};self.flow._revoke_device_review=Mock()

    def manager(self):
        m=test_environment_exit.EnvironmentExitTest().manager(False)
        m.config['model_lifecycle']='external';m.window=None
        m.rows['control_owner'].update(state='stopped',running=False)
        m.post=Mock(side_effect=lambda action,**kw:self.flow.operations_invalidate_model(still_current=lambda:True))
        original=m._run.side_effect
        def run(action,request,progress):
            if action=='stop_model':
                m.calls.append(action)
                return dict(action=action,stopped_model=m.stop_model('cpu-maintenance',progress))
            return original(action,request,progress)
        m._run.side_effect=run
        return m

    def test_each_consumer_can_exit_without_stopping_other_and_admin_close_invalidates_all(self):
        original=binding(self.fixture.a.status())
        self.b.close()
        self.assertEqual(self.a.infer(request('after-collector-exit'))['request_id'],'after-collector-exit')
        self.b.acquire()
        m=self.manager()
        with patch.object(workbench_deploy,'verify_unit',return_value={'ActiveState':'inactive'}),\
             patch.object(workbench_deploy,'restore_collection_web',return_value={'ready':True,'collection_started':False}),\
             patch('robot_workbench.control_owner.ControlChannel')as channel:
            channel.return_value.call.return_value=dict(connected=False,runtime_nonce=None)
            result=m.exit_environment({'request_id':'consumer-exit','expected_target':m.target()},lambda *a,**k:None)
        self.assertTrue(result['environment_exited']);self.assertTrue(m.rows['model']['running'])
        self.assertTrue(all(m.rows[name]['state']=='stopped'for name in m.managed_service_names()))
        self.assertEqual(self.fixture.a.status()['active_sessions'],1)
        self.assertEqual(self.b.infer(request('after-workbench-exit'))['request_id'],'after-workbench-exit')
        self.assertEqual(binding(self.fixture.b.status()),original);self.assertEqual(self.fixture.model.closes,0)
        self.assertNotIn(('model','stop'),[(c.args[0],c.args[1])for c in m.service_action.call_args_list])
        m.health_cache['services']=deepcopy(m.rows)
        with patch.object(workbench_deploy,'verify_unit',return_value={'ActiveState':'active','MainPID':'7'}),\
             patch.object(workbench_deploy,'http_json',return_value={'robot_vin':'arx13','orphan_alive':False}):
            self.assertTrue(WorkbenchManager.verify(m,result)['ok'])
        # This is an explicit service administrator shutdown, never a consumer operation.
        self.a.acquire()
        self.assertEqual(self.fixture.a.status()['active_sessions'],2)
        self.fixture.service.close()
        self.assertEqual(self.fixture.model.closes,1)
        for client in (self.a,self.b):
            with self.assertRaisesRegex(RuntimeError,'closed'):client.infer(request('after-admin-shutdown'))

    def test_external_process_operations_are_rejected_without_status_count_race(self):
        m=self.manager()
        for operation in ('start','stop','restart'):
            with self.subTest(operation=operation),self.assertRaisesRegex(OperationError,'独立服务管理员'):
                WorkbenchManager.service_action(m,'model',operation,Mock())
        with self.assertRaisesRegex(OperationError,'独立服务管理员'):m.prepare_model('restart_model',None,Mock(),{})
        self.assertEqual(self.fixture.a.status()['active_sessions'],2)
        self.assertEqual(self.fixture.model.closes,0);m.post.assert_not_called()

    def test_prepare_reuses_external_process_and_refuses_starting_a_missing_service(self):
        m=self.manager();m.rows['model'].update(owned=True,identity={'pid':7})
        m.wait_model=Mock();m.model_evidence=Mock(return_value={'loaded':True})
        m.post=Mock(return_value={'model_ready':True,'control_ready':False,'model_id':'cpu-only'})
        result=m.prepare_model('prepare_model','token',Mock(),{})
        self.assertTrue(result['prepared']['model_ready']);m.service_action.assert_not_called()
        m.rows['model'].update(state='stopped',running=False);m.post.reset_mock()
        with self.assertRaisesRegex(OperationError,'管理员启动'):m.prepare_model('prepare_model','token',Mock(),{})
        m.post.assert_not_called();m.service_action.assert_not_called()

    def test_missing_backend_does_not_claim_release_and_ttl_remains_server_owned(self):
        m=self.manager();result=m.stop_model(None,Mock(),backend_absent=True)
        self.assertFalse(result['binding']['invalidated'])
        self.assertEqual(result['binding']['pin_release']['status'],'unconfirmed_backend_absent')
        self.assertEqual(self.fixture.a.status()['active_sessions'],2);m.post.assert_not_called()
        self.assertEqual(self.fixture.model.closes,0)
        self.fixture.clock[0]+=31
        self.assertEqual(self.fixture.a.status()['active_sessions'],0)
        self.assertTrue(self.fixture.a.status()['loaded'])

    def test_unknown_release_is_not_relabelled_success_on_second_invalidation(self):
        m=self.manager()
        with patch.object(self.a,'close',side_effect=ConnectionError('CPU release response lost')):
            with self.assertRaises(ConnectionError):m.stop_model('token',Mock())
        with self.assertRaisesRegex(OperationError,'未确认'):m.stop_model('token',Mock())
        self.assertEqual(self.fixture.a.status()['active_sessions'],2)
        self.assertEqual(self.b.infer(request('other-consumer'))['request_id'],'other-consumer')

    def test_backend_stop_and_restart_release_pin_before_process_action(self):
        for action in ('stop','restart'):
            with self.subTest(action=action):
                if self.a.token is None:self.a.acquire()
                self.flow.client=self.a;self.flow.model_client=self.fixture.a
                m=self.manager();state={'state':'running','running':True,'owned':True,'identity':{'pid':7,'start_ticks':'7','boot_id':'cpu'}}
                def change():
                    self.assertEqual(self.fixture.a.status()['active_sessions'],1)
                    state.update(state='stopped'if action=='stop'else'running',running=action!='stop')
                    return deepcopy(state)
                m.services['workbench']=NS(snapshot=lambda:deepcopy(state),**{action:change})
                @contextmanager
                def maintenance(*args,**kwargs):yield 'cpu-maintenance'
                m.maintenance=maintenance
                result=WorkbenchManager.service_action(m,'workbench',action,Mock())
                self.assertEqual(result['shared_model_detachment']['binding']['pin_release']['status'],'released')
                self.assertEqual(self.b.infer(request('still-available-'+action))['request_id'],'still-available-'+action)

    def test_registered_start_uses_manager_ownership_and_never_starts_external_unit(self):
        profile=Path(self.fixture.temp.name)/'manager.json'
        profile.write_text(json.dumps({'model_lifecycle':'external','services':{'model':{'kind':'user_unit','unit':'robot-workbench-model.service'}}}))
        config={'manager_profile':str(profile),'units':['robot-workbench-model.service','robot-workbench-ui.service'],'url':'http://127.0.0.1:18868'}
        state={'health':{'backend_reachable':True},'target':{'services':{'model':{'pid':7}}},'model':{'ready':True},'workflow':{'manual_queue':{'available':True}}}
        with patch.object(start_registered_workbench.subprocess,'run')as command,\
             patch.object(start_registered_workbench,'http_json',return_value=state):
            self.assertTrue(start_registered_workbench.start(config)['ui_ready'])
        command.assert_called_once_with(['systemctl','--user','start','robot-workbench-ui.service'],check=True,timeout=30)
        self.assertNotIn('model',workbench_deploy.service_order({'model_lifecycle':'external','services':{'model':{}}}))
        self.assertIn('model',workbench_deploy.service_order({'services':{'model':{}}}))

    def test_external_start_rejects_missing_or_unsupported_mapping_before_any_process_call(self):
        profile=Path(self.fixture.temp.name)/'external-manager.json'
        config={'manager_profile':str(profile),'units':['robot-workbench-model.service','robot-workbench-ui.service'],'url':'http://127.0.0.1:18868'}
        for services in (None,{}, {'model':None},{'model':{}},{'model':{'kind':'user_unit'}},
                         {'model':{'kind':'user_unit','unit':''}},
                         {'model':{'kind':'user_unit','unit':'../robot-workbench-model.service'}},
                         {'model':{'kind':'docker_model','unit':'robot-workbench-model.service'}},
                         {'model':{'kind':'process','unit':'robot-workbench-model.service'}}):
            with self.subTest(services=services):
                profile.write_text(json.dumps({'model_lifecycle':'external','services':services}))
                with patch.object(start_registered_workbench.subprocess,'run')as command,\
                     patch.object(start_registered_workbench,'http_json')as http:
                    with self.assertRaisesRegex(workbench_deploy.Blocked,'explicit supported user_unit'):
                        start_registered_workbench.start(config)
                command.assert_not_called();http.assert_not_called()


if __name__=='__main__':unittest.main()
