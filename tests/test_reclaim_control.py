"""One-click orchestration; device and collection transports are CPU substitutes."""
from copy import deepcopy
from types import SimpleNamespace
import unittest
from unittest.mock import Mock,patch
from pathlib import Path
from tempfile import TemporaryDirectory
import sys

from robot_workbench.management import BackendRejection,WorkbenchManager
from robot_workbench.operations import OperationError
from scripts import workbench_deploy


class ReclaimControlTests(unittest.TestCase):
    def test_idle_web_is_not_a_controller_but_sdk_and_collection_manager_are(self):
        with patch.object(sys,'path',[str(Path(__file__).resolve().parents[1]/'rule_based_workbench'),*sys.path]):
            from rule_based_workbench import real_control
        with TemporaryDirectory() as directory:
            root=Path(directory)
            for pid,argv in [(101,['python','-m','prometheus.web']),
                             (102,['python','-m','prometheus.system.collection_manager']),
                             (103,['python','/somewhere/serve_sdk_control.py']),
                             (104,['python','-m','prometheus.robots.arx5_robot_node'])]:
                file=root/str(pid)/'cmdline';file.parent.mkdir();file.write_bytes(b'\0'.join(x.encode()for x in argv))
            with patch.object(real_control,'Path',return_value=root):
                self.assertEqual(sorted(real_control.controller_conflicts()),[102,103,104])

    def test_reclaim_closes_every_registered_collector_before_sdk_acquire(self):
        manager=self.manager()
        extra=dict(unit='collector-secondary.service',files={'/cpu/secondary.unit':'b'*64},
            status_url='http://127.0.0.1:18981/api/status')
        manager.config['additional_collection_handoffs']=[extra]
        events=[]
        manager._run.side_effect=lambda *args,**kwargs:events.append('acquire')or{}
        def handoff(pin,*args,**kwargs):
            events.append(pin['unit'])
            return dict(unit=pin['unit'],exit_verified=True)
        with patch.object(workbench_deploy,'verify_unit',return_value={'ActiveState':'inactive'}),\
                patch.object(workbench_deploy,'handoff_collection',side_effect=handoff):
            result=manager.reclaim_control(dict(request_id='all',arms_supported=True),Mock())
        self.assertEqual(events,[manager.config['collection_handoff']['unit'],extra['unit'],'acquire'])
        self.assertEqual(result['reclaim']['additional_collections'][0]['unit'],extra['unit'])

    def test_extra_collector_exit_failure_keeps_sdk_unacquired(self):
        manager=self.manager()
        manager.config['additional_collection_handoffs']=[dict(unit='collector-secondary.service',
            files={'/cpu/secondary.unit':'b'*64},status_url='http://127.0.0.1:18981/api/status')]
        with patch.object(workbench_deploy,'verify_unit',return_value={'ActiveState':'inactive'}),\
                patch.object(workbench_deploy,'handoff_collection',side_effect=[{'exit_verified':True},workbench_deploy.Blocked('exit unconfirmed')]):
            with self.assertRaises(workbench_deploy.Blocked):
                manager.reclaim_control(dict(request_id='all',arms_supported=True),Mock())
        manager._run.assert_not_called()

    def test_explicit_bootstrap_conflict_is_a_known_failure(self):
        manager=self.manager();manager.stop_generation=0
        manager._run.side_effect=BackendRejection('existing hardware controller processes require owner handoff: [91]')
        with self.assertRaises(OperationError)as caught:
            WorkbenchManager.run(manager,'connect_control',{},Mock())
        self.assertEqual(caught.exception.code,'control_owner_conflict')
        self.assertFalse(caught.exception.unknown)
        manager._run.side_effect=BackendRejection('native SDK result unconfirmed')
        with self.assertRaises(BackendRejection):WorkbenchManager.run(manager,'connect_control',{},Mock())

    def test_cold_camera_source_is_verified_before_business_rebind(self):
        m=self.manager();m.config['camera_descriptor']='/cpu/cameras.json';m.timeouts['camera']=.2
        identity=dict(pid=5,start_ticks='1',boot_id='cpu',script='/cpu/camera.py')
        m.service_action=Mock(return_value=dict(identity=identity))
        m.services['cameras']=SimpleNamespace(snapshot=lambda:dict(state='running',identity=identity))
        m.post=Mock(side_effect=AssertionError('business is not running yet'))
        client=SimpleNamespace(frames={'left':None},latest_rgbd=Mock(return_value={'raw':True}))
        with patch('robot_workbench.rgbd_service.CommonCameraService.connect',side_effect=[OSError('starting'),client])as connect:
            result=m.camera_action('start',None,lambda *a,**k:None,bind=False)
        self.assertEqual(connect.call_count,2);client.latest_rgbd.assert_called_once_with('left')
        self.assertEqual(result['binding']['status'],'deferred_until_business_start');m.post.assert_not_called()

    def manager(self,*,connected=False,fault=False):
        m=object.__new__(WorkbenchManager)
        m.config=dict(device_id='arx13',control_owner_descriptor='/cpu/control.json',
            collection_handoff=dict(unit='prometheus-arx13-web.service',files={'/cpu/unit':'a'*64},status_url='http://127.0.0.1:18980/api/status'))
        m.environment='physical';m.port=18878;m.timeouts=dict(state=.1,release=.02,operation=1.)
        runtime=dict(connected=connected,runtime_nonce='cpu'if connected or fault else None,
            state='FAULT_STOP_UNCONFIRMED'if fault else'READY'if connected else'DISCONNECTED',
            automatic_enabled=False,feedback_healthy=connected,stationary=connected,
            stop_receipt=dict(stop_hold_return_confirmed=True,barrier_epoch=2))
        m.health_cache=dict(runtime=runtime,busy=False)
        m.refresh=Mock(side_effect=lambda:deepcopy(m.health_cache));m.window=None;m.unresolved=Mock(return_value=[])
        m.target=Mock(return_value=dict(manager_instance='manager',deployment_sha256='d',services={'control_owner':{'pid':1},'workbench':{'pid':2}}))
        m.services={'control_owner':SimpleNamespace(snapshot=lambda:dict(state='running'))}
        m.state=Mock(return_value=dict(csrf='cpu',server_instance_id='business',device=dict(id='arx13'),runtime=runtime))
        m._request_control_stop=Mock(return_value=dict(status='accepted',receipt=dict(stop=dict(barrier_epoch=2))))
        m._run=Mock(return_value=dict(action='recover_control',control=dict(motion_resumed=False)))
        return m

    def test_disconnected_and_fault_reuse_normal_connect_or_recover_after_handoff(self):
        for fault in (False,True):
            with self.subTest(fault=fault):
                m=self.manager(fault=fault)
                with patch.object(workbench_deploy,'verify_unit',return_value={'ActiveState':'inactive'}),patch.object(workbench_deploy,'handoff_collection',return_value={'exit_verified':True})as handoff:
                    value=m.reclaim_control(dict(request_id='one',arms_supported=True),lambda *a,**k:None)
                self.assertEqual(m._run.call_args.args[0],'recover_control'if fault else'connect_control')
                handoff.assert_called_once();m._request_control_stop.assert_not_called()
                self.assertFalse(value['reclaim']['old_motion_resumed'])

    def test_web_connect_handoff_failure_precedes_any_sdk_or_camera_start(self):
        m=self.manager();m.check=Mock(return_value=None);m.service_action=Mock()
        with patch.object(workbench_deploy,'handoff_collection',side_effect=workbench_deploy.Blocked('camera exit unconfirmed'))as handoff:
            with self.assertRaisesRegex(workbench_deploy.Blocked,'camera exit unconfirmed'):
                WorkbenchManager._run(m,'connect_control',dict(request_id='connect',expected_target=m.target(),arms_supported=True),Mock())
        handoff.assert_called_once();self.assertFalse(handoff.call_args.kwargs['stop_active'])
        m.service_action.assert_not_called()

    def test_healthy_connection_requires_fresh_stop_then_reuses_it(self):
        m=self.manager(connected=True)
        with patch.object(workbench_deploy,'verify_unit',return_value={'ActiveState':'inactive'}),patch.object(workbench_deploy,'handoff_collection',return_value={'exit_verified':True}),patch('robot_workbench.management.json_request'):
            m.reclaim_control(dict(request_id='one',arms_supported=False),lambda *a,**k:None)
        m._request_control_stop.assert_called_once_with('one-stop')
        self.assertEqual(m._run.call_args.args[0],'recover_control')
        m=self.manager(connected=True);m.health_cache['runtime']['stop_receipt']['barrier_epoch']=1
        with patch.object(workbench_deploy,'verify_unit',return_value={'ActiveState':'inactive'}),patch.object(workbench_deploy,'handoff_collection')as handoff,patch('robot_workbench.management.json_request'):
            with self.assertRaisesRegex(OperationError,'停止、静止反馈'):m.reclaim_control(dict(request_id='one'),lambda *a,**k:None)
            handoff.assert_not_called();m._run.assert_not_called()

    def test_conflicting_or_unconfirmed_collection_cannot_reach_sdk_acquire(self):
        m=self.manager(connected=True)
        with patch.object(workbench_deploy,'verify_unit',return_value={'ActiveState':'active'}),patch.object(workbench_deploy,'http_json',return_value=dict(robot_vin='arx13',manager_alive=True)):
            with self.assertRaisesRegex(OperationError,'归属冲突'):m.reclaim_control(dict(request_id='one'),lambda *a,**k:None)
        m._request_control_stop.assert_not_called();m._run.assert_not_called()
        m=self.manager()
        with patch.object(workbench_deploy,'verify_unit',return_value={'ActiveState':'inactive'}),patch.object(workbench_deploy,'handoff_collection',side_effect=workbench_deploy.Blocked('exit unconfirmed')):
            with self.assertRaises(workbench_deploy.Blocked):m.reclaim_control(dict(request_id='one'),lambda *a,**k:None)
        m._run.assert_not_called()

    def test_operator_stop_or_replaced_owner_cancels_acquisition(self):
        m=self.manager()
        def progress(*args,**kw):raise OperationError('operator_stop','operator stop')
        with patch.object(workbench_deploy,'verify_unit',return_value={'ActiveState':'inactive'}),patch.object(workbench_deploy,'handoff_collection')as handoff:
            with self.assertRaisesRegex(OperationError,'operator stop'):m.reclaim_control(dict(request_id='one'),progress)
            handoff.assert_not_called();m._run.assert_not_called()
        m=self.manager();changed=deepcopy(m.target.return_value);changed['services']['control_owner']['pid']=3
        def replace(*args,**kw):m.target.return_value=changed;return {'exit_verified':True}
        with patch.object(workbench_deploy,'verify_unit',return_value={'ActiveState':'inactive'}),patch.object(workbench_deploy,'handoff_collection',side_effect=replace):
            with self.assertRaisesRegex(OperationError,'被其他实例替换'):m.reclaim_control(dict(request_id='one'),lambda *a,**k:None)
        m._run.assert_not_called()


if __name__=='__main__':unittest.main()
