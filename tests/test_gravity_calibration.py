"""Gravity-only coordinate mapping, source binding and actual adapter selection."""
from copy import deepcopy
import hashlib,json
from pathlib import Path
from tempfile import TemporaryDirectory
from types import ModuleType,SimpleNamespace as NS
import sys,unittest
from unittest.mock import patch
import numpy as np
from robot_workbench.gravity_calibration import bake_joint_offsets,load_gravity_calibration
from robot_workbench.motion_reference import load_feedforward
from arx_env.joint_head_isaac_fit import Model
from test_prepared_reference_backend import Base,hardware_backend


class GravityCalibrationTests(unittest.TestCase):
    def fixture(self,root):
        def put(name,raw):
            path=Path(root)/name;path.write_bytes(raw);return dict(path=str(path),sha256=hashlib.sha256(raw).hexdigest())
        def js(name,value):return put(name,json.dumps(value).encode())
        raw=('<robot name="X5"><link name="base_link"/>'+''.join(
            f'<link name="link{i}"/><joint name="joint{i}" type="revolute"><parent link="'+('base_link'if i==1 else f'link{i-1}')+
            f'"/><child link="link{i}"/><origin xyz=".1 .02 .03" rpy=".1 -.2 .3"/><axis xyz="'+('0 1 0'if i%2 else'1 0 0')+'"/></joint>'for i in range(1,7))+'</robot>').encode()
        offsets={'left':[.01,-.02,.03,-.01,.02,-.03],'right':[-.02,.01,-.01,.03,-.02,.01]}
        head=dict(kind='head',id='h',robot_id='r',payload=dict(row=dict(parameters=dict(joint_offsets_rad=offsets))))
        wrist=dict(kind='wrist',id='w',robot_id='r');manifest=dict(robot_id='r',head={'id':'h'},wrist={'id':'w'})
        sources=dict(head=js('head.json',head),wrist=js('wrist.json',wrist),manifest=js('manifest.json',manifest),sdk_model=put('nominal.urdf',raw),calibration_model=put('calibration.urdf',raw))
        value=dict(schema='sdk_gravity_calibration.v1',robot_id='r',source_pins=sources,joint_offsets_rad=offsets,
            models={s:put(s+'.urdf',bake_joint_offsets(raw,o))for s,o in offsets.items()},
            convention='q_gravity_model=q_sdk_feedback+joint_offsets_rad',gravity_vector=[0.,0.,-9.807])
        return value,js('gravity.json',value),js

    def test_offsets_applied_exactly_once_and_sources_bound(self):
        self.assertIsNone(load_gravity_calibration(None))
        with TemporaryDirectory()as root:
            value,pin,js=self.fixture(root);result=load_gravity_calibration(pin);self.assertEqual(result['id'],pin['sha256'])
            names=[f'joint{i}'for i in range(1,7)];original=Model(value['source_pins']['sdk_model']['path'],names)
            rng=np.random.default_rng(27)
            for side,offsets in value['joint_offsets_rad'].items():
                calibrated=Model(value['models'][side]['path'],names)
                for q in rng.uniform(-1,1,(30,6)):
                    a,b=calibrated.frames(q),original.frames(q+offsets)
                    for name in a:np.testing.assert_allclose(a[name],b[name],atol=2e-14,rtol=0)
            wrong=deepcopy(value);wrong['models']['left']=value['models']['right']
            with self.assertRaisesRegex(ValueError,'single application'):load_gravity_calibration(js('wrong.json',wrong))
            wrong=deepcopy(value);wrong['joint_offsets_rad']['left'][0]*=-1
            with self.assertRaisesRegex(ValueError,'installed'):load_gravity_calibration(js('wrong.json',wrong))
            Path(value['source_pins']['head']['path']).write_text('{}')
            with self.assertRaisesRegex(ValueError,'changed'):load_gravity_calibration(pin)

    def test_adapter_uses_each_arm_model_and_old_FF_cannot_cross(self):
        with TemporaryDirectory()as root:
            value,pin,js=self.fixture(root);record=load_gravity_calibration(pin);calls=[]
            native=NS(MotorType=NS(EC_A4310='ec',DM_J4310='dm',NONE='none'))
            native.Arx5JointController=lambda r,c,b:calls.append((r.urdf_path,r.gripper_motor_type,b))
            class ImportBase(Base):
                def _import_arx5(self):return native
            module=ModuleType('arx_env.backends.arx5_backend');module.Arx5Backend=ImportBase;module.Arx5BackendConfig=lambda **kw:NS(**kw)
            with patch.dict(sys.modules,{'arx_env.backends.arx5_backend':module}),patch('robot_workbench.arx_output_guard.require_guard',return_value=NS(sdk_contract={'pin':True})),patch('robot_workbench.sdk_quiesce.verify_module'):
                b=hardware_backend(disabled_grippers=('right',),gravity_calibration=record);sdk=b._import_arx5()
                def robot():return NS(robot_model='X5',urdf_path=value['source_pins']['sdk_model']['path'],gravity_vector=[0.,0.,-9.807],gripper_motor_type='dm')
                sdk.Arx5JointController(robot(),None,b.config.left_can);sdk.Arx5JointController(robot(),None,b.config.right_can)
                self.assertEqual(calls,[(value['models']['left']['path'],'dm',b.config.left_can),(value['models']['right']['path'],'none',b.config.right_can)])
                self.assertEqual(b.gravity_calibration_id,pin['sha256'])
                wrong=robot();wrong.urdf_path='unqualified.urdf'
                with self.assertRaisesRegex(ValueError,'model differs'):sdk.Arx5JointController(wrong,None,b.config.left_can)
            ff=dict(schema='reference_feedforward.v1',source_sha256='a'*64,controller_profile='normal_X5',phase=[0.,1.],torque_bias_sdk6=[[0.]*6]*2,provenance={})
            def write():
                raw=json.dumps(ff).encode();sha=hashlib.sha256(raw).hexdigest();(Path(root)/(sha+'.json')).write_bytes(raw);return sha
            old=write();load_feedforward(root,old,'a'*64)
            with self.assertRaisesRegex(ValueError,'gravity calibration'):load_feedforward(root,old,'a'*64,gravity_calibration_id=pin['sha256'])
            ff['provenance']['gravity_calibration_id']=pin['sha256'];new=write();load_feedforward(root,new,'a'*64,gravity_calibration_id=pin['sha256'])
            with self.assertRaisesRegex(ValueError,'gravity calibration'):load_feedforward(root,new,'a'*64)


if __name__=='__main__':unittest.main()
