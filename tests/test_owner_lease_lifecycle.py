"""Failed management must neither hide an acquired SDK nor leak a release lease."""
from pathlib import Path
import sys,unittest
from types import SimpleNamespace
from unittest.mock import Mock,patch
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from robot_workbench.service import Device
from robot_workbench.control_owner import ControlOwnerService
from robot_workbench.maintenance import ControlActor


class OwnerLeaseLifecycleTest(unittest.TestCase):
    def test_stale_lease_rejects_acquisition_before_creating_sdk(self):
        service=object.__new__(ControlOwnerService)
        service.device=SimpleNamespace(runtime=None,connect=Mock())
        service.maintenance={'token':'old'}
        with self.assertRaisesRegex(ValueError,'prior control maintenance lease'):
            service._write('acquire',{'arms_supported':True})
        service.device.connect.assert_not_called()

    def test_management_failure_keeps_new_owner_visible_and_stops_without_reacquire(self):
        runtime=SimpleNamespace(remote_control_owner=True,nonce='current',
            begin_maintenance=Mock(side_effect=ValueError('lease collision')),
            register_business=Mock(return_value={}),request_stop=Mock())
        factory=Mock(return_value=runtime);device=Device('cpu','physical',factory);device.maintenance=True
        actor=Mock(spec=ControlActor);actor.action='connect_control';actor.current.return_value=True
        with patch.object(device,'_start_remote_monitor')as monitor:
            with self.assertRaisesRegex(ValueError,'SDK仍连接.*lease collision'):
                device.connect({'arms_supported':True},maintenance_actor=actor)
        factory.assert_called_once();runtime.request_stop.assert_called_once()
        actor.adopt.assert_called_once_with(runtime)
        monitor.assert_called_once_with(runtime);self.assertIs(device.runtime,runtime)
        self.assertTrue(device.credentials);self.assertFalse(device.connecting)

    def test_reaped_sdk_allows_retirement_maintenance_despite_an_obsolete_stop(self):
        import threading,queue
        from contextlib import nullcontext
        from robot_workbench.runtime import Runtime
        for exited in (False,True):
            r=object.__new__(Runtime);r.maintenance_active=False;r.tick_lock=threading.Lock();r.calls={}
            r._preview_boundary=nullcontext;r._release_reason=Mock(return_value='disconnected SDK cleanup')
            commands=queue.Queue();commands.put({'action':'stop'})
            r.owner=SimpleNamespace(lock=threading.RLock(),backend=SimpleNamespace(terminal_failure='exited',
                protection_snapshot=lambda exited=exited:{'worker_exit_confirmed':exited}),recording=None,
                stop_requested=True,commands=commands,pending={},inference_payloads={})
            r.preview_revision=0;r.reviewed_plans={}
            if exited:
                r.begin_maintenance();self.assertTrue(r.maintenance_active)
                self.assertEqual(commands.qsize(),1,'retirement admission must not execute the obsolete Stop')
            else:
                with self.assertRaises(ValueError):r.begin_maintenance()

    def test_release_ends_lease_before_forgetting_client_and_retains_failed_cleanup(self):
        for failure in (None,PermissionError('wrong capability')):
            with self.subTest(failure=failure):
                device=Device('cpu','fake',None)
                runtime=SimpleNamespace(monitor=Mock(),release_receipt=dict(status='released',sdk_close_returned=True),
                    end_maintenance=Mock(side_effect=failure),detach=Mock())
                device.runtime=runtime;device._start_remote_monitor(runtime);device.thread.join(2)
                self.assertFalse(device.thread.is_alive());runtime.end_maintenance.assert_called_once()
                if failure:
                    self.assertIs(device.runtime,runtime);runtime.detach.assert_not_called()
                    self.assertFalse(device.release_receipt['maintenance_cleanup_confirmed'])
                    self.assertIn('租约清理未确认',device.error)
                else:
                    self.assertIsNone(device.runtime);runtime.detach.assert_called_once()
                    self.assertTrue(device.release_receipt['maintenance_cleanup_confirmed'])


if __name__=='__main__':unittest.main()
