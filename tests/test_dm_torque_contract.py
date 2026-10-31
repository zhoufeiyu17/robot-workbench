"""Opt-in protocol units must preserve outputs and fail closed before construction."""
import hashlib,json,struct,sys,tempfile,unittest
from pathlib import Path
from types import ModuleType,SimpleNamespace as NS
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'calibration'),str(ROOT/'calibration/infra'),str(ROOT/'tests')]
from robot_workbench.sdk_quiesce import dm_torque_units,DM_EQUIVALENT_UNITS,DM_LEGACY_TO_PROTOCOL
from test_prepared_reference_backend import Base,hardware_backend

class DMTorqueContractTests(unittest.TestCase):
    def test_pinned_machine_receipt_and_tamper(self):
        buses=dict(left='can13l',right='can13r');rows=[]
        for side,bus in buses.items():
            for motor in(5,6,7):
                for rid,v in((21,12.5),(22,30.),(23,10.)):
                    h=bytes((motor,0,0x33,rid));rows.append(dict(side=side,bus=bus,motor_id=motor,register=rid,query_count=1,request_can_id=0x7ff,request_data=(h+bytes(4)).hex(),responses=[dict(can_id=0,data=(h+struct.pack('<f',v)).hex(),value=v)]))
        with tempfile.TemporaryDirectory()as root:
            p=Path(root)/'ranges.json';p.write_text(json.dumps(dict(robot_id='arx13',can_interfaces=buses,queries=rows,complete=True,parameter_writes=0,enable_commands=0,motion_commands=0)))
            c=dict(mode=DM_EQUIVALENT_UNITS,register_receipt=dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest()))
            self.assertEqual(dm_torque_units(c,can_interfaces=buses,robot_id='arx13'),DM_EQUIVALENT_UNITS)
            for robot in('arx6',None):
                with self.assertRaises(ValueError):dm_torque_units(c,can_interfaces=buses,robot_id=robot)
            p.write_text('{}')
            with self.assertRaisesRegex(ValueError,'hash'):dm_torque_units(c,can_interfaces=buses,robot_id='arx13')
        self.assertIsNone(dm_torque_units(None))

    def test_constructor_opt_in_and_legacy_default(self):
        calls=[];native=NS(MotorType=NS(EC_A4310='ec',DM_J4310='dm',NONE='none'))
        native.Arx5JointController=lambda r,c,b:calls.append((r.dm_protocol_torque_equivalent,r.gripper_motor_type,b))
        class ImportBase(Base):
            def _import_arx5(self):return native
        m=ModuleType('arx_env.backends.arx5_backend');m.Arx5Backend=ImportBase;m.Arx5BackendConfig=lambda **k:NS(**k)
        with patch.dict(sys.modules,{'arx_env.backends.arx5_backend':m}),patch('robot_workbench.arx_output_guard.require_guard',return_value=NS(sdk_contract={'pinned':True})),patch('robot_workbench.sdk_quiesce.verify_module'):
            b=hardware_backend(can_interfaces=dict(left='can13l',right='can13r'),dm_joint_velocity_wire_scale=1.5,dm_torque_unit_contract=DM_EQUIVALENT_UNITS);sdk=b._import_arx5()
            def robot():return NS(robot_model='X5',motor_id=[1,2,4,5,6,7],motor_type=['ec']*3+['dm']*3,gripper_motor_type='dm',dm_joint_velocity_wire_scale=1.,dm_protocol_torque_equivalent=False)
            for bus in('can13l','can13r'):sdk.Arx5JointController(robot(),None,bus)
            self.assertEqual(calls,[(True,'dm','can13l'),(True,'dm','can13r')])
            old=robot();del old.dm_protocol_torque_equivalent
            with self.assertRaisesRegex(ValueError,'default-off'):sdk.Arx5JointController(old,None,'can13l')
            with self.assertRaises(ValueError):hardware_backend(dm_torque_unit_contract=DM_EQUIVALENT_UNITS)
            self.assertIs(hardware_backend()._import_arx5(),native)

    def test_equivalent_ff_limits_and_inverse(self):
        for old in(-18.,-2.,-.01,0.,.01,2.,18.):
            new=old*DM_LEGACY_TO_PROTOCOL
            self.assertAlmostEqual(new*1.8,old/.424,places=12)
            self.assertAlmostEqual(new*.7632,old,places=12)

if __name__=='__main__':unittest.main()
