"""Native latch identity, retained IPC and dual-arm Stop. CPU only, no SDK imports."""
from copy import deepcopy
import sys
from types import ModuleType,SimpleNamespace as NS
import unittest
from unittest.mock import patch
import test_sdk_process as rpc_fixture
import test_prepared_reference_backend as adapter_fixture
import test_shared_target_routes as owner_fixture
from robot_workbench.sdk_process import SDKNativeFault,validate_native_faults

FAULT=dict(library_build_id='a'*64,controller_instance_id='cpu:1',reason='over_current',
    latched=True,feedback_valid=True,damping_committed=True,joint=1,observed_value=41.,latched_monotonic_ns=100)
WORKER=rpc_fixture.WORKER.replace('class Backend:', '''from types import SimpleNamespace as NS
class NativeEmergencyLatched(RuntimeError):pass
class Backend:
 _arx5=NS(NativeEmergencyLatched=NativeEmergencyLatched)
 def native_faults(self):return getattr(self,'faults',{})
''').replace("  if value.get('reject'):","  if value.get('native_latch'):\n   self.faults={'left':"+repr(FAULT)+"};raise NativeEmergencyLatched('over_current')\n  if value.get('reject'):")


class NativeFaultRPCTest(unittest.TestCase):
    fixture=rpc_fixture.SDKProcessTests.fixture
    def test_native_error_retains_ipc_for_reads_stop_release_not_software_recovery(self):
        with patch.object(rpc_fixture,'WORKER',WORKER):b=self.fixture()
        b.connect();pid=b.process.pid
        with self.assertRaises(SDKNativeFault)as caught:b.queue_reference({'native_latch':True})
        self.assertEqual(caught.exception.faults,{'left':FAULT})
        self.assertIsNone(b.failure);self.assertTrue(b.is_connected);self.assertEqual(b.process.pid,pid)
        self.assertFalse(b.read_gains()['future_queued']);b.set_damping_hold(manual=False)
        self.assertIsNone(b.process.poll());b.close()

    def test_malformed_native_report_fails_closed(self):
        bad=WORKER.replace("'joint': 1","'joint': 99")
        with patch.object(rpc_fixture,'WORKER',bad):b=self.fixture()
        b.connect()
        with self.assertRaises(RuntimeError):b.queue_reference({'native_latch':True})
        self.assertIsNotNone(b.failure);self.assertFalse(b.is_connected)

    def test_report_validation(self):
        self.assertEqual(validate_native_faults({}),{})
        for field,value in [('latched',False),('feedback_valid',1),('joint',True),('observed_value',float('nan')),
                            ('reason','unknown'),('reason',[]),('latched_monotonic_ns',0)]:
            with self.subTest(field=field),self.assertRaises(ValueError):
                validate_native_faults({'left':dict(FAULT,**{field:value})})


class NativeFaultAdapterTest(unittest.TestCase):
    def backend(self):
        class ScopedBase(adapter_fixture.Base):
            def set_damping_hold(self,**kwargs):
                for side in self.sides:
                    self._controllers[side].set_joint_cmd(adapter_fixture.State(6))
                    self._controllers[side].set_gain(adapter_fixture.Gain(6))
        module=ModuleType('arx_env.backends.arx5_backend');module.Arx5Backend=ScopedBase
        module.Arx5BackendConfig=lambda **kw:NS(**kw)
        with patch.dict(sys.modules,{'arx_env.backends.arx5_backend':module}):b=adapter_fixture.hardware_backend()
        b.output_guard=NS(sdk_contract={'library_build_id':FAULT['library_build_id']})
        c=b._controllers['left'];c.fault=False
        c.get_emergency_state=lambda:NS(**dict(FAULT,latched=c.fault))
        c.controller_instance_id=lambda:'cpu:1'
        c.set_to_damping=lambda:c.events.append('native_damping')
        return b,c

    def test_fault_arm_uses_native_damping_healthy_arm_keeps_normal_measured_stop(self):
        b,c=self.backend();c.fault=True
        def reject(*args):raise AssertionError('ordinary command attempted on latched arm')
        c.set_gain=c.set_joint_cmd=reject
        b.set_damping_hold(manual=False)
        self.assertEqual(c.events,['native_damping'])
        self.assertEqual(b._controllers['right'].events,['fixed','gain','gain'])
        self.assertEqual(b.sides,('left','right'))

    def test_latch_racing_stop_rechecks_once_and_stops_both(self):
        b,c=self.backend()
        def race(*args):c.fault=True;raise RuntimeError('native latch during legacy stop')
        c.set_joint_cmd=race;b.set_damping_hold(manual=False)
        self.assertEqual(c.events,['native_damping'])
        self.assertIn('fixed',b._controllers['right'].events)
        self.assertEqual(b.sides,('left','right'))

    def test_foreign_native_identity_not_accepted(self):
        b,c=self.backend();c.fault=True;c.controller_instance_id=lambda:'different'
        with self.assertRaisesRegex(ValueError,'identity differs'):b.native_faults()


class NativeFaultOwnerTest(unittest.TestCase):
    profile='normal_X5'
    setUp=owner_fixture.SharedTargetRoutesTest.setUp
    cleanup=owner_fixture.SharedTargetRoutesTest.cleanup
    create=owner_fixture.SharedTargetRoutesTest.create
    advance=owner_fixture.SharedTargetRoutesTest.advance
    confirm=owner_fixture.SharedTargetRoutesTest.confirm

    def test_one_incident_cancels_future_keeps_diagnostics_blocks_all_in_place_recovery(self):
        self.create();runtime=self.runtime;owner=runtime.owner
        review=runtime.submit(self.creds['calibration'],dict(action='preview_workbench',waypoints=[dict(
            q12=[.15]+[0.]*11,speeds12=[.1]*12,jaw_gap_mm=[40.,40.])]))
        self.confirm('calibration',review);self.advance(50)
        original=self.raw.read_calibration_state
        def fault_read():
            return dict(original(),native_faults={'left':deepcopy(FAULT)})
        self.raw.read_calibration_state=fault_read
        self.advance();epoch=owner.epoch;packets=len(self.raw.packets)
        self.advance(40)
        self.assertEqual(owner.control.state,owner.states.FAULT)
        self.assertTrue(owner.feedback_healthy);self.assertTrue(owner.stationary)
        self.assertEqual(owner.epoch,epoch);self.assertEqual(len(self.raw.packets),packets)
        self.assertIsNone(self.raw.window);self.assertIsNone(owner.recoverable_fault)
        events=[e for e in self.events if e[0]=='fault_diagnostic'and e[1]['diagnostic'].get('native_faults')]
        self.assertEqual(len(events),1)
        self.assertTrue(owner.last_fault_diagnostic['stop_return_confirmed'])
        self.assertFalse(owner.recovery_ready());self.assertFalse(runtime._software_stop_recovery_ready())
        owner.recoverable_fault='synthetic software reason'
        self.assertFalse(owner.recovery_ready(),'native latch must win over software eligibility')


if __name__=='__main__':unittest.main()
