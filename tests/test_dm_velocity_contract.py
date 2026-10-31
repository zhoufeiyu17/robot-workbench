"""Physical range evidence and pre-constructor opt-in; no CAN/controller construction."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import struct
import sys
import tempfile
from types import ModuleType,SimpleNamespace as NS
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'calibration'),str(ROOT/'calibration/infra'),str(ROOT/'tests')]
from robot_workbench.sdk_quiesce import dm_joint_velocity_wire_scale
from examples.scripts.debug.joint_head_real_owner import hardware_backend
from test_prepared_reference_backend import Base


class DMVelocityContractTests(unittest.TestCase):
    def test_complete_raw_register_evidence_and_corruption(self):
        rows=[]
        for side,bus in (('left','can3'),('right','can1')):
            for motor in (5,6,7):
                for rid,value in ((21,12.5),(22,30.),(23,10.)):
                    header=bytes([motor,0,0x33,rid])
                    rows.append(dict(side=side,bus=bus,motor_id=motor,register=rid,
                        query_count=1,request_can_id=0x7ff,request_data=(header+bytes(4)).hex(),
                        responses=[dict(can_id=0,data=(header+struct.pack('<f',value)).hex(),value=value)]))
        receipt=dict(queries=rows,complete=True,parameter_writes=0,enable_commands=0,motion_commands=0)
        with tempfile.TemporaryDirectory()as root:
            path=Path(root)/'receipt.json'
            def pin(value):
                raw=json.dumps(value).encode();path.write_bytes(raw)
                return dict(vendor_vmax_rad_s=45.,register_receipt=dict(path=str(path),sha256=hashlib.sha256(raw).hexdigest()))
            self.assertEqual(dm_joint_velocity_wire_scale(pin(receipt)),1.5)
            self.assertIsNone(dm_joint_velocity_wire_scale(None))
            second=deepcopy(receipt);second.update(robot_id='arx13',can_interfaces=dict(left='can13l',right='can13r'))
            for row in second['queries']:row['bus']=second['can_interfaces'][row['side']]
            self.assertEqual(dm_joint_velocity_wire_scale(pin(second),can_interfaces=second['can_interfaces'],robot_id='arx13'),1.5)
            with self.assertRaises(ValueError):dm_joint_velocity_wire_scale(pin(second),can_interfaces=second['can_interfaces'],robot_id='arx6')
            for change in (
                lambda x:x['queries'].pop(),lambda x:x['queries'].__setitem__(1,x['queries'][0]),
                lambda x:x['queries'][1]['responses'][0].update(value=45.),
                lambda x:x['queries'][1].update(request_data='0500551600000000'),
                lambda x:x.update(parameter_writes=1),lambda x:x['queries'][1].update(bus='can1')):
                bad=deepcopy(receipt);change(bad)
                with self.assertRaises(ValueError):dm_joint_velocity_wire_scale(pin(bad))
            contract=pin(receipt);path.write_text('{}')
            with self.assertRaisesRegex(ValueError,'hash'):dm_joint_velocity_wire_scale(contract)

    def test_shared_factory_applies_scale_before_native_constructor_and_preserves_jaw_policy(self):
        native=NS(MotorType=NS(EC_A4310='ec',DM_J4310='dm',NONE='none'));calls=[]
        native.Arx5JointController=lambda robot,controller,interface:calls.append((robot.dm_joint_velocity_wire_scale,robot.gripper_motor_type,interface))
        class ImportBase(Base):
            def _import_arx5(self):return native
        module=ModuleType('arx_env.backends.arx5_backend');module.Arx5Backend=ImportBase;module.Arx5BackendConfig=lambda **kw:NS(**kw)
        with patch.dict(sys.modules,{'arx_env.backends.arx5_backend':module}), \
                patch('robot_workbench.arx_output_guard.require_guard',return_value=NS(sdk_contract={'pinned':True})), \
                patch('robot_workbench.sdk_quiesce.verify_module'):
            b=hardware_backend(disabled_grippers=('right',),dm_joint_velocity_wire_scale=1.5);sdk=b._import_arx5()
            def robot():return NS(robot_model='X5',motor_id=[1,2,4,5,6,7],motor_type=['ec']*3+['dm']*3,
                dm_joint_velocity_wire_scale=1.,gripper_motor_type='dm')
            sdk.Arx5JointController(robot(),None,b.config.left_can)
            sdk.Arx5JointController(robot(),None,b.config.right_can)
            self.assertEqual(calls,[(1.5,'dm',b.config.left_can),(1.5,'none',b.config.right_can)])
            old=robot();del old.dm_joint_velocity_wire_scale
            with self.assertRaisesRegex(ValueError,'does not match'):sdk.Arx5JointController(old,None,b.config.left_can)
            wrong=robot();wrong.motor_id[3]=8
            with self.assertRaisesRegex(ValueError,'does not match'):sdk.Arx5JointController(wrong,None,b.config.left_can)
            self.assertEqual(len(calls),2,'incompatible configuration must fail before native construction')
            legacy=hardware_backend();self.assertIs(legacy._import_arx5(),native)
            for value in (True,0.,float('nan'),2.):
                with self.assertRaises(ValueError):hardware_backend(dm_joint_velocity_wire_scale=value)


if __name__=='__main__':unittest.main()
