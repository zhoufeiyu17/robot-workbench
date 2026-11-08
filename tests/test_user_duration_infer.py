"""Run orchestration only. No model, SDK, camera or GPU imports."""
from copy import deepcopy
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from unittest.mock import Mock

from robot_workbench.service import Application


class UserDurationInferTests(unittest.TestCase):
    def test_managed_model_binding_closes_before_service(self):
        from robot_workbench.management import WorkbenchManager
        manager=WorkbenchManager.__new__(WorkbenchManager)
        manager.config={'model_lifecycle':'managed'}
        manager.timeouts={'state':1.}
        manager.services={'model':SimpleNamespace(snapshot=lambda:{'running':False})}
        calls=[]
        manager.post=lambda *args,**kwargs:(calls.append('release')or {'invalidated':True,'model_ready':False})
        manager.service_action=lambda *args:(calls.append('stop')or {'state':'stopped'})
        manager.stop_model('token',lambda *args:None)
        self.assertEqual(calls,['release','stop'])
        manager.post=Mock(side_effect=ConnectionError('release unknown'))
        manager.service_action=Mock()
        with self.assertRaises(ConnectionError):manager.stop_model('token',lambda *args:None)
        manager.service_action.assert_not_called()

    def run_episode(self, *, reason='normal MODEL episode budget exhausted', stop_at=None,
                    model_failed=False, changed_identity=False, submit_error=False,
                    pending_snapshot=False, changed_nonce=False,operator_stop_at=None):
        calls=[]
        active=[True]
        runtime=object()
        device=SimpleNamespace(runtime=runtime)
        app=Application.__new__(Application)
        app.lock=threading.RLock()
        reads=[0]
        checks=[0];job={}
        def current():
            checks[0]+=1
            if (operator_stop_at=='before_model' or operator_stop_at=='admission'and checks[0]==3):
                job['operator_stop_request_id']='pedal-stop'
            return active[0]

        def snapshot():
            reads[0]+=1
            if pending_snapshot and reads[0]==1:
                return dict(id='new-execution',busy=True,
                    inference=dict(normal_mode=dict(runtime_nonce=None,runner_status=None,mode=None)))
            receipts={'initialize':dict(op='normal_initialize',completed=reads[0]>1)}
            mode='INITIALIZING' if reads[0]==1 else 'HOLD'
            for request in calls:
                receipts[request['request_id']]=dict(completed=not model_failed or request['target']=='END',
                    status='failed' if model_failed and request['target']=='MODEL' else 'completed')
            ended=bool(calls and calls[-1]['target']=='END')
            if stop_at=='initial' and reads[0]>1 or stop_at=='model' and calls:active[0]=False
            return dict(id='changed' if changed_identity and calls else 'new-execution',busy=not ended,
                inference=dict(normal_mode=dict(mode=mode,mode_session_id='mode-session',
                    runtime_nonce='other' if changed_nonce and calls else 'nonce',
                    command_epoch=7+len(calls),feedback_healthy=True,
                    transition_stage='HOLD_PENDING' if reads[0]==1 else None,mode_receipts=receipts,
                    runner_status='READY' if ended else 'RUNNING',reason=reason)))

        def submit(request):
            calls.append(deepcopy(request))
            if submit_error:raise OSError('unknown original request result')

        workflow=SimpleNamespace(snapshot=snapshot,request_normal_mode=submit)
        with patch('robot_workbench.service.time.sleep'):
            try:
                result=app.run_normal_episode(device,workflow,job,current,runtime,'nonce',120.,
                    keep_session=operator_stop_at is not None)
                return result,calls,None
            except Exception as error:
                return None,calls,error

    def test_initial_hold_model_budget_end_uses_distinct_current_requests(self):
        result,calls,error=self.run_episode()
        self.assertIsNone(error)
        self.assertEqual(result['execution_instance_id'],'new-execution')
        self.assertEqual([row['target'] for row in calls],['MODEL','END'])
        self.assertEqual([row['epoch'] for row in calls],[7,8])
        self.assertNotEqual(calls[0]['request_id'],calls[1]['request_id'])
        self.assertTrue(all(row['session_id']=='mode-session' and row['runtime_nonce']=='nonce' for row in calls))

    def test_operator_stop_before_model_admission_retains_prepared_session(self):
        for phase in ('before_model','admission'):
            with self.subTest(phase=phase):
                result,calls,error=self.run_episode(operator_stop_at=phase)
                self.assertIsNone(error);self.assertEqual(calls,[])
                self.assertTrue(result['operator_stopped']);self.assertTrue(result['recording_open'])

    def test_waits_for_first_remote_owner_snapshot_without_weakening_identity(self):
        result,calls,error=self.run_episode(pending_snapshot=True)
        self.assertIsNone(error)
        self.assertEqual(result['execution_instance_id'],'new-execution')
        self.assertEqual([row['target'] for row in calls],['MODEL','END'])
        _,calls,error=self.run_episode(pending_snapshot=True,changed_nonce=True)
        self.assertIn('执行身份已变化',str(error))
        self.assertEqual([row['target'] for row in calls],['MODEL'])

    def test_early_supply_hold_is_closed_but_reported_failed(self):
        for failed in (False,True):
            result,calls,error=self.run_episode(reason='prediction is stale',model_failed=failed)
            self.assertIsNone(result)
            self.assertEqual([row['target'] for row in calls],['MODEL','END'])
            self.assertIn('prediction is stale',str(error))

    def test_budget_too_short_for_first_segment_is_not_success(self):
        result,calls,error=self.run_episode(
            reason='complete next segment exceeds remaining episode budget',model_failed=True)
        self.assertIsNone(result)
        self.assertEqual([row['target'] for row in calls],['MODEL','END'])
        self.assertIsInstance(error,ValueError)

    def test_stop_prevents_late_model_or_end(self):
        for phase,expected in [('initial',[]),('model',['MODEL'])]:
            _,calls,error=self.run_episode(stop_at=phase)
            self.assertIsInstance(error,ValueError)
            self.assertEqual([row['target'] for row in calls],expected)

    def test_changed_execution_and_unknown_submit_are_not_retried(self):
        for options in (dict(changed_identity=True),dict(submit_error=True)):
            _,calls,error=self.run_episode(**options)
            self.assertIsNotNone(error)
            self.assertEqual(len(calls),1)

    def test_stopped_self_detach_requires_its_explicit_local_maintenance_scope(self):
        from robot_workbench import maintenance
        from robot_workbench.service import Device
        for purpose,permitted in [('stop_model',False),('stop_consumer',True)]:
            device=Device('cpu','fake',None)
            device.control_lifecycle='external'
            runtime=SimpleNamespace(nonce='nonce',release_receipt=None,
                begin_maintenance=Mock(side_effect=AssertionError('shared freeze')),
                end_maintenance=Mock(side_effect=AssertionError('shared unfreeze')),
                detach=Mock(return_value=dict(ownership_retained=False,episode_aborted=True,
                    safe_finish=False,sdk_released=False)))
            device.runtime=runtime
            app=Application({'fake':device})
            target=maintenance.target(app,device)
            opened=maintenance.operate(app,device,'begin',dict(environment='fake',request_id='open',
                purpose=purpose,expected_target=target))
            body=dict(environment='fake',request_id='detach',expected_target=target,maintenance_token=opened['token'])
            if permitted:
                maintenance.operate(app,device,'detach_stopped_consumer',body)
                runtime.detach.assert_called_once_with(abort_stopped=True)
            else:
                with self.assertRaises(ValueError):maintenance.operate(app,device,'detach_stopped_consumer',body)
                runtime.detach.assert_not_called()
            runtime.begin_maintenance.assert_not_called()

    def test_real_normal_runner_budget_reaches_end_through_run_worker(self):
        from test_normal_complete_segment import CompleteNormalTests
        fixture=CompleteNormalTests()
        fixture.setUp()
        self.addCleanup(fixture.doCleanups)
        fixture.setup_modes(recording=False,finish_kind='maintenance_home_v1',
            initial_mode='HOLD',reviewed=True,reviewed_duration_s=2.)
        runtime=fixture.runtime
        app=Application.__new__(Application)
        app.lock=threading.RLock()
        result=[]

        def snapshot():
            value=runtime.owner.runner.snapshot()
            return dict(id=runtime.owner.runner.execution_instance_id,
                busy=value['runner_status']!='READY',inference=dict(normal_mode=value))

        workflow=SimpleNamespace(snapshot=snapshot,request_normal_mode=lambda body:
            runtime.submit(fixture.creds['inference'],dict(body,action='normal_mode')))
        def run():
            try:result.append(app.run_normal_episode(SimpleNamespace(runtime=runtime),workflow,{},
                lambda:True,runtime,runtime.nonce,2.))
            except Exception as error:result.append(error)
        thread=threading.Thread(target=run,daemon=True)
        thread.start()
        fixture.until(lambda:bool(result),count=1200)
        thread.join(1)
        self.assertIsInstance(result[0],dict,str(result[0]))
        self.assertTrue(runtime.owner.runner.normal_ended)
        self.assertEqual(runtime.owner.runner.status,'READY')


if __name__=='__main__':
    unittest.main()
