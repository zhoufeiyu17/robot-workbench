"""Consumer lifecycle boundaries; current native Owner cases live in test_control_owner_process."""
from copy import deepcopy
import threading
import time
from types import SimpleNamespace as NS
from pathlib import Path
from tempfile import TemporaryDirectory
import json
import unittest
from unittest.mock import Mock,patch

import test_environment_exit
from robot_workbench.management import WorkbenchManager
from robot_workbench.operations import OperationError
from robot_workbench.device_bootstrap import physical_runtime_factory
from robot_workbench import maintenance


class WorkbenchControlLifecycleTests(unittest.TestCase):
    def test_registered_start_does_not_start_shared_owner_cameras_or_model(self):
        from scripts import start_registered_workbench as launch
        with TemporaryDirectory()as root:
            p=Path(root)/'manager.json'
            services={role:dict(kind='user_unit',unit=f'robot-workbench-{role}.service')
                for role in ('control_owner','cameras','model')}
            p.write_text(json.dumps(dict(model_lifecycle='external',control_lifecycle='external',
                control_consumer_id='C',control_owner_descriptor='/cpu/C.json',services=services)))
            config=dict(manager_profile=str(p),units=[v['unit']for v in services.values()]+['robot-workbench-ui.service'],url='http://127.0.0.1:18868')
            state=dict(health={'backend_reachable':True},target={'services':{'model':{'pid':1}}},model={'ready':True},workflow={'manual_queue':{'available':True}})
            with patch.object(launch.subprocess,'run')as command,patch.object(launch,'http_json',return_value=state):
                self.assertTrue(launch.start(config)['ui_ready'])
            command.assert_called_once_with(['systemctl','--user','start','robot-workbench-ui.service'],check=True,timeout=30)

    def manager(self,holder='D',busy=False):
        m=test_environment_exit.EnvironmentExitTest().manager()
        m.config.update(control_lifecycle='external',control_consumer_id='C',model_lifecycle='external')
        m.window=None;m.unresolved=lambda:[];m.health_lock=threading.RLock()
        m.health_cache.update(control_owner_reachable=True,backend_reachable=True,
            deployment={'files_unchanged':True},checked_wall_ns=time.time_ns(),services=m.rows,busy=busy)
        m.health_cache['runtime'].update(automatic_enabled=busy,feedback_healthy=True,
            control_owner=dict(shared_consumers=True,consumer_holder=holder,consumer_attached=holder is not None))
        m.target=Mock(side_effect=lambda:dict(runtime_nonce=m.health_cache['runtime']['runtime_nonce']))
        return m

    def test_external_exit_preserves_other_consumers_and_never_restores_collector(self):
        m=self.manager(busy=True)
        self.assertIsNone(WorkbenchManager.check(m,'exit_environment'))
        result=m.exit_environment({'request_id':'exit'},Mock())
        self.assertTrue(result['consumer_lifecycle']);self.assertEqual(m.calls,['stop_workbench'])
        self.assertEqual(m.managed_service_names(),{'workbench'})
        self.assertTrue(WorkbenchManager.verify(m,result)['ok'])
        for role in ('control_owner','cameras','model'):
            self.assertTrue(m.rows[role]['running'])
            with self.assertRaises(OperationError):WorkbenchManager.service_action(m,role,'stop',Mock())
        m.stop_current_control.assert_not_called();m.home_for_exit.assert_not_called()
        self.assertIsNotNone(WorkbenchManager.check(m,'connect_control'))
        self.assertIsNotNone(WorkbenchManager.check(m,'reclaim_control'))

    def test_own_busy_or_retained_claim_does_not_claim_success(self):
        m=self.manager('C',True)
        self.assertIsNotNone(WorkbenchManager.check(m,'exit_environment'))
        m.health_cache['busy']=False;m.health_cache['runtime']['automatic_enabled']=False
        with self.assertRaisesRegex(OperationError,'claim'):
            m.exit_environment({'request_id':'retained'},Mock())
        self.assertEqual(m.calls,['stop_workbench']);self.assertTrue(m.rows['control_owner']['running'])

    def test_same_owner_process_with_changed_sdk_or_unhealthy_feedback_cannot_verify_handoff(self):
        for field,value in (('runtime_nonce','new-runtime'),('connected',False),('feedback_healthy',False)):
            with self.subTest(field=field):
                m=self.manager()
                result=m.exit_environment({'request_id':'exit'},Mock())
                m.health_cache['runtime'][field]=value
                proof=WorkbenchManager.verify(m,result)
                self.assertFalse(proof['ok']);self.assertFalse(proof['shared_runtime_preserved'])

    def test_bootstrap_requires_dedicated_identity_and_calls_attach_not_sdk_acquire(self):
        client=NS(device_id='arx6',environment='physical',channel=NS(descriptor={}),
            attach=Mock(),acquire=Mock(side_effect=AssertionError('SDK acquire forbidden')),
            control_inputs=Mock(return_value={'cached':True}),detach=Mock())
        profile=dict(robot_id='arx6',storage_root='/tmp/cpu-only',control_owner_descriptor='/cpu/consumer.json',
            control_lifecycle='external',control_consumer_id='C')
        with self.assertRaisesRegex(ValueError,'cannot construct the SDK Owner'):
            physical_runtime_factory(profile,local_owner=True)
        with patch('robot_workbench.control_owner.ControlRuntimeClient',return_value=client):
            with self.assertRaisesRegex(ValueError,'administrator descriptor forbidden'):
                physical_runtime_factory(profile)()
            client.channel.descriptor={'shared_consumers':True,'consumer_id':'D'}
            with self.assertRaises(ValueError):physical_runtime_factory(profile)()
            client.channel.descriptor['consumer_id']='C'
            self.assertIs(physical_runtime_factory(profile)(),client)
            client.attach.assert_called_once();client.acquire.assert_not_called()
            client.attach.side_effect=PermissionError('D owns execution')
            with self.assertRaises(PermissionError):physical_runtime_factory(profile)()
            client.detach.assert_called_once()

    def test_model_maintenance_is_local_to_consumer_not_shared_owner(self):
        from robot_workbench.service import Application,Device
        device=Device('cpu','fake',None);device.control_lifecycle='external'
        runtime=NS(nonce='cpu-runtime',begin_maintenance=Mock(side_effect=AssertionError('global freeze')),
            end_maintenance=Mock(side_effect=AssertionError('global unfreeze')),release_receipt=None)
        device.runtime=runtime;app=Application({'fake':device})
        body=dict(environment='fake',request_id='begin',purpose='stop_model',expected_target=maintenance.target(app,device))
        result=maintenance.operate(app,device,'begin',body)
        self.assertEqual(result['status'],'ready')
        maintenance.operate(app,device,'end',dict(environment='fake',request_id='end',
            expected_target=maintenance.target(app,device),maintenance_token=result['token']))
        runtime.begin_maintenance.assert_not_called();runtime.end_maintenance.assert_not_called()


if __name__=='__main__':unittest.main()
