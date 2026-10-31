"""Unit migration boundaries; no hardware constructors or CAN."""
from copy import deepcopy
import hashlib,json
from pathlib import Path
from tempfile import TemporaryDirectory
from types import ModuleType,SimpleNamespace as NS
import sys,unittest
from unittest.mock import patch
import numpy as np
from robot_workbench.sdk_quiesce import ec_torque_units,EC_STATIC_UNITS,EC_J2_EQUIVALENT_UNITS,EC_J2_GRAVITY_SCALES
from robot_workbench.motion_reference import load_feedforward
from robot_workbench.target_interpreter import TargetInterpreter
import test_target_interpreter as interpreter_fixture
from test_prepared_reference_backend import Base,hardware_backend

class ECTorqueContractTests(unittest.TestCase):
    def test_static_reference_is_constant_bounded_and_separate_from_50s_source(self):
        import io
        from robot_workbench.control_reference import decode
        b=np.array([0.,12.]);c=np.zeros((1,4,12));j=np.full((2,2),.04)
        m=dict(study_schema='workbench.control_reference.v1',schema='prepared_motion_reference.v1',
            study_kind='static_unit_probe',duration_s=12.,source_range_s=[0.,12.],speed_multiplier=None,event_times_s=[0.,12.])
        def raw():
            meta=deepcopy(m);h=hashlib.sha256(json.dumps(meta,sort_keys=True).encode())
            for arr in (b,c,j):h.update(arr.tobytes())
            meta['reference_id']=h.hexdigest();f=io.BytesIO();np.savez(f,breaks=b,coefficients=c,jaws=j,metadata=np.array(json.dumps(meta)));return f.getvalue()
        limits=[[-3,3]]*14;limits[6]=limits[13]=[0,.082]
        decode(raw(),limits)
        c[0,2,1]=.001
        with self.assertRaisesRegex(ValueError,'static unit'):decode(raw(),limits)
        c[0,2,1]=0.;j[1,0]=.05
        with self.assertRaisesRegex(ValueError,'static unit'):decode(raw(),limits)
        j[:]=.04;m['study_kind']='unknown'
        with self.assertRaises(ValueError):decode(raw(),limits)

    def test_scope_and_parameter_receipt(self):
        self.assertIsNone(ec_torque_units(None))
        for bad in ({},dict(mode='global'),dict(mode=EC_STATIC_UNITS,register_receipt={'path':'relative','sha256':'a'*64})):
            with self.assertRaises(ValueError):ec_torque_units(bad)
        with TemporaryDirectory()as root:
            p=Path(root)/'receipt.json';p.write_text('{}')
            with self.assertRaisesRegex(ValueError,'hash'):
                ec_torque_units(dict(mode=EC_STATIC_UNITS,register_receipt=dict(path=str(p),sha256='bfac89fa11b94a12e56887d071813685a166cb66e79e86ee56f4d1d9098f1a5d')))

    def test_constructor_selection_is_left_j2_j3_only_and_preserves_jaw(self):
        calls=[];native=NS(MotorType=NS(EC_A4310='ec',DM_J4310='dm',NONE='none'))
        native.Arx5JointController=lambda r,c,b:calls.append((r.ec_torque_protocol_joints,r.ec_protocol_gravity_scale,r.gripper_motor_type,b))
        class ImportBase(Base):
            def _import_arx5(self):return native
        module=ModuleType('arx_env.backends.arx5_backend');module.Arx5Backend=ImportBase;module.Arx5BackendConfig=lambda **kw:NS(**kw)
        with patch.dict(sys.modules,{'arx_env.backends.arx5_backend':module}),patch('robot_workbench.arx_output_guard.require_guard',return_value=NS(sdk_contract={'pin':True})),patch('robot_workbench.sdk_quiesce.verify_module'):
            b=hardware_backend(disabled_grippers=('right',),ec_torque_unit_contract=EC_STATIC_UNITS);sdk=b._import_arx5()
            def robot():return NS(robot_model='X5',motor_id=[1,2,4,5,6,7],motor_type=['ec']*3+['dm']*3,ec_torque_protocol_joints=[],ec_protocol_gravity_scale=1.,gripper_motor_type='dm')
            sdk.Arx5JointController(robot(),None,b.config.left_can);sdk.Arx5JointController(robot(),None,b.config.right_can)
            self.assertEqual(calls,[([1,2],1/1.4,'dm',b.config.left_can),([],1.,'none',b.config.right_can)])
            wrong=robot();wrong.ec_torque_protocol_joints=[0]
            with self.assertRaisesRegex(ValueError,'does not match'):sdk.Arx5JointController(wrong,None,b.config.left_can)
            self.assertEqual(len(calls),2)

    def test_equivalent_j2_units_retain_calibrated_models_and_inert_profile_validation(self):
        from test_gravity_calibration import GravityCalibrationTests
        from test_device_bootstrap import BootstrapTests
        from robot_workbench.device_bootstrap import read_profile
        with TemporaryDirectory()as root:
            value,pin,_=GravityCalibrationTests().fixture(root)
            from robot_workbench.gravity_calibration import load_gravity_calibration
            gravity=load_gravity_calibration(pin);calls=[]
            native=NS(MotorType=NS(EC_A4310='ec',DM_J4310='dm',NONE='none'))
            native.Arx5JointController=lambda r,c,b:calls.append((r.urdf_path,r.ec_torque_protocol_joints,r.ec_protocol_gravity_scale,r.gripper_motor_type))
            class ImportBase(Base):
                def _import_arx5(self):return native
            module=ModuleType('arx_env.backends.arx5_backend');module.Arx5Backend=ImportBase;module.Arx5BackendConfig=lambda **kw:NS(**kw)
            with patch.dict(sys.modules,{'arx_env.backends.arx5_backend':module}),patch('robot_workbench.arx_output_guard.require_guard',return_value=NS(sdk_contract={'pin':True})),patch('robot_workbench.sdk_quiesce.verify_module'):
                def robot():return NS(robot_model='X5',urdf_path=value['source_pins']['sdk_model']['path'],gravity_vector=[0.,0.,-9.807],motor_id=[1,2,4,5,6,7],motor_type=['ec']*3+['dm']*3,ec_torque_protocol_joints=[],ec_protocol_gravity_scale=1.,gripper_motor_type='dm')
                for mode,scale in EC_J2_GRAVITY_SCALES.items():
                    calls.clear();b=hardware_backend(disabled_grippers=('right',),ec_torque_unit_contract=mode,gravity_calibration=gravity);sdk=b._import_arx5()
                    sdk.Arx5JointController(robot(),None,b.config.left_can);sdk.Arx5JointController(robot(),None,b.config.right_can)
                    self.assertEqual(calls,[(value['models']['left']['path'],[1],scale,'dm'),(value['models']['right']['path'],[],1.,'none')])
                    if mode!=EC_J2_EQUIVALENT_UNITS:
                        with self.assertRaisesRegex(ValueError,'requires calibrated'):hardware_backend(ec_torque_unit_contract=mode)
                with self.assertRaises(ValueError):hardware_backend(ec_torque_unit_contract=EC_STATIC_UNITS,gravity_calibration=gravity)
            fixture=BootstrapTests();fixture.setUp()
            try:
                profile=dict(fixture.profile,native_reference_api='preserve_reference_v1',prepared_reference_enabled=True,target_interpreter_profile='x5_control50_damping_v1',sdk_quiesce={'pin':'inert fixture'},sdk_gravity_calibration=pin,ec_torque_contract={'mode':EC_J2_EQUIVALENT_UNITS})
                with patch('robot_workbench.sdk_quiesce.ec_torque_units',side_effect=lambda c:None if c is None else c['mode']),patch('subprocess.Popen',side_effect=AssertionError('inert only')):
                    for mode in EC_J2_GRAVITY_SCALES:
                        profile['ec_torque_contract']['mode']=mode;fixture.path.write_text(json.dumps(profile));self.assertEqual(read_profile(fixture.path)['sdk_gravity_calibration'],pin)
                        if mode!=EC_J2_EQUIVALENT_UNITS:
                            bad=deepcopy(profile);bad.pop('sdk_gravity_calibration');fixture.path.write_text(json.dumps(bad))
                            with self.assertRaisesRegex(ValueError,'requires calibrated'):read_profile(fixture.path)
                    profile['ec_torque_contract']['mode']=EC_STATIC_UNITS;fixture.path.write_text(json.dumps(profile))
                    with self.assertRaisesRegex(ValueError,'reviewed J2'):read_profile(fixture.path)
            finally:fixture.doCleanups()

    def test_feedforward_cannot_cross_unit_contracts(self):
        value=dict(schema='reference_feedforward.v1',source_sha256='a'*64,controller_profile='normal_X5',phase=[0.,1.],torque_bias_sdk6=[[0.]*6]*2,provenance={})
        with TemporaryDirectory()as root:
            def write(v):
                raw=json.dumps(v).encode();sha=hashlib.sha256(raw).hexdigest();(Path(root)/(sha+'.json')).write_bytes(raw);return sha
            old=write(value)
            load_feedforward(root,old,'a'*64)
            with self.assertRaisesRegex(ValueError,'unit contract'):load_feedforward(root,old,'a'*64,torque_unit_contract=EC_STATIC_UNITS)
            value['provenance']['torque_unit_contract']=EC_STATIC_UNITS;new=write(value)
            load_feedforward(root,new,'a'*64,torque_unit_contract=EC_STATIC_UNITS)
            with self.assertRaisesRegex(ValueError,'unit contract'):load_feedforward(root,new,'a'*64)

    def test_static_candidate_rejects_old_integral_defaults_and_unbound_bias(self):
        reference,_=interpreter_fixture.TargetInterpreterTests().point()
        backend=NS(queue_reference=lambda p:None,torque_unit_contract=EC_STATIC_UNITS)
        args=dict(active_arm='both',start_s=100.,epoch=1,current_epoch=lambda:1,gate=lambda:None,clock=lambda:100.,profile='x5_control50_damping_v1',integral=True)
        with self.assertRaisesRegex(ValueError,'Ki0'):TargetInterpreter(backend,reference,**args)
        args['integral_gains']={'left':[400.,0.,0.,160.,160.,80.],'right':[50.,50.,60.,20.,20.,10.]}
        follower=TargetInterpreter(backend,reference,**args)
        np.testing.assert_equal(follower.integrals['left'].gains[1:3],[0.,0.])
        with self.assertRaisesRegex(ValueError,'unit contract'):TargetInterpreter(backend,reference,**args,feedforward={'provenance':{}})

if __name__=='__main__':unittest.main()
