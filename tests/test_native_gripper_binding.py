"""Machine endpoint binding, native pre-constructor configuration and jaw units."""
from copy import deepcopy
import hashlib,json,sys,tempfile,unittest
from pathlib import Path
from types import ModuleType,SimpleNamespace as NS
from unittest.mock import patch,Mock

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'calibration'),str(ROOT/'calibration/infra'),str(ROOT/'tests')]
from robot_workbench.sdk_quiesce import load_gripper_calibration
from arx_env import joint_head_jaw_mapping as jaw
from examples.scripts.debug import joint_head_real_owner as ref
from test_prepared_reference_backend import Base


def fixture(root):
    root=Path(root)
    def put(name,value):
        p=root/name;p.write_text(json.dumps(value));return dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest())
    sides=dict(left=dict(open_readout=-3.426222629129473,closed_readout=-.1623178454261076,width_m=.08),
               right=dict(open_readout=-3.521210040436408,closed_readout=-.032997634851605895,width_m=.089))
    source=dict(id='arx13',left_can='can13l',right_can='can13r',left_model='X5',right_model='X5')
    for side,row in sides.items():source.update({side+'_gripper_'+k:v for k,v in row.items()})
    value=dict(schema='native_gripper_endpoints.v1',robot_id='arx13',source=put('machine.json',source),sdk_width_m=.082,sides=sides)
    pin=put('endpoints.json',value)
    mapping=put('mapping.json',dict(schema_version='native_gripper_mapping.v1',robot_id='arx13',native_gripper_calibration=pin))
    return value,pin,mapping,put


class NativeGripperBindingTest(unittest.TestCase):
    def tearDown(self):jaw.configure_mapping(None)

    def test_sdk_worker_loads_the_same_mapping_before_backend_construction(self):
        from scripts.serve_sdk_control import main
        with tempfile.TemporaryDirectory()as root:
            _,pin,mapping,_=fixture(root)
            profile=dict(robot_id='arx13',native_gripper_calibration=pin,jaw_mapping_file=mapping['path'],
                legacy_source='/cpu/legacy',can_interfaces=dict(left='can13l',right='can13r'))
            def construct(**kwargs):
                self.assertEqual(jaw.mapping_identity()['native_calibration_id'],pin['sha256'])
                self.assertAlmostEqual(jaw.hold_command_sdk('left',80.27115474520804),.082)
                return Mock()
            with patch.object(sys,'argv',['worker','--profile','/cpu/profile','--record','/cpu/record','--fd','3','--epoch-fd','4','--owner','5','--child']),patch(
                    'robot_workbench.device_bootstrap.read_profile',return_value=profile),patch(
                    'robot_workbench.arx_output_guard.bind_child'),patch('scripts.serve_sdk_control.socket.socket'),patch(
                    'scripts.serve_sdk_control.os.set_inheritable'),patch('scripts.serve_sdk_control.mmap.mmap'),patch(
                    'robot_workbench.sdk_process.serve_worker'),patch.object(ref,'legacy_modules'),patch.object(ref,'hardware_backend',side_effect=construct):
                self.assertEqual(main(),0)

    def test_observed_full_open_noise_changes_only_the_hold_command(self):
        with tempfile.TemporaryDirectory()as root:
            _,pin,mapping,_=fixture(root);value=load_gripper_calibration(pin)
            jaw.configure_mapping(mapping['path'])
            module=ModuleType('arx_env.backends.arx5_backend');module.Arx5Backend=Base;module.Arx5BackendConfig=lambda **kw:NS(**kw)
            with patch.dict(sys.modules,{'arx_env.backends.arx5_backend':module}):
                backend=ref.hardware_backend(gripper_calibration=value)
                legacy=ref.hardware_backend()
            measured=.08227793361383824
            self.assertAlmostEqual(backend._stop_gripper_command('left',measured),.082)
            self.assertEqual(backend._stop_jaw_projection['left']['raw_sdk_m'],measured)
            self.assertTrue(backend._stop_jaw_projection['left']['command_clipped'])
            self.assertEqual(legacy._stop_gripper_command('left',measured),measured)
            with self.assertRaises(ValueError):backend._stop_gripper_command('left',.09)

    def test_source_identity_units_and_full_open_hold(self):
        from arx_env.joint_head_real_motion import feedback_to_command,make_plan
        with tempfile.TemporaryDirectory()as root:
            value,pin,mapping,put=fixture(root)
            actual=load_gripper_calibration(pin,robot_id='arx13')
            self.assertEqual(actual['sides'],value['sides'])
            with self.assertRaises(ValueError):load_gripper_calibration(pin,robot_id='arx6')
            jaw.configure_mapping(mapping['path'])
            self.assertEqual([jaw.command_max_mm(s)for s in ('left','right')],[80.,89.])
            from arx_env.joint_head_workbench_geometry import jaw_gaps
            self.assertEqual(jaw_gaps([80.,89.]).tolist(),[80.,89.])
            for gaps in ([80.01,89.],[80.,89.01],[-.01,0.]):
                with self.assertRaises(ValueError):jaw_gaps(gaps)
            self.assertEqual([[jaw.feedback_min_mm(s),jaw.feedback_max_mm(s)]for s in ('left','right')],[[-3.,83.],[-3.,92.]])
            for side,maximum in [('left',80.),('right',89.)]:
                self.assertAlmostEqual(jaw.command_sdk(side,maximum),.082)
                self.assertAlmostEqual(jaw.hold_command_sdk(side,maximum),.082)
                self.assertAlmostEqual(jaw.command_gap_mm(side,.041),maximum/2)
                with self.assertRaises(ValueError):jaw.command_sdk(side,maximum+.01)
                self.assertEqual(jaw.hold_command_sdk(side,-3.),0.)
                self.assertAlmostEqual(jaw.hold_command_sdk(side,maximum+3.),.082)
                for gap in (-3.000001,maximum+3.000001):
                    with self.assertRaises(ValueError):jaw.hold_command_sdk(side,gap)
            raw=[0.]*14;raw[6]=raw[13]=.082;limits=[[-3.,3.]]*14;limits[6]=limits[13]=[0.,.082]
            self.assertEqual(list(feedback_to_command(raw,limits)),raw)
            self.assertEqual(jaw.feedback_gaps(raw),[80.,89.])
            plan=make_plan('both',raw,raw,limits,kind='both_home')
            self.assertEqual(list(plan.q_target),raw)
            from real_control import classes
            classes(ref,jaw_mapping_file=mapping['path'],device_id='arx13')
            with self.assertRaisesRegex(ValueError,'another physical device'):
                classes(ref,jaw_mapping_file=mapping['path'],device_id='arx6')
            stale=deepcopy(value);stale['sides']['left']['open_readout']=-4
            with self.assertRaises(ValueError):load_gripper_calibration(put('wrong.json',stale))
            Path(value['source']['path']).write_text('{}')
            with self.assertRaisesRegex(ValueError,'changed'):load_gripper_calibration(pin)

    def test_endpoint_override_precedes_native_constructor(self):
        with tempfile.TemporaryDirectory()as root:
            _,pin,_,_=fixture(root);value=load_gripper_calibration(pin);calls=[]
            sdk=NS(MotorType=NS(DM_J4310='dm',NONE='none'))
            sdk.Arx5JointController=lambda r,c,b:calls.append((b,r.gripper_closed_readout,r.gripper_open_readout,r.gripper_width))
            class ImportBase(Base):
                def _import_arx5(self):return sdk
            module=ModuleType('arx_env.backends.arx5_backend');module.Arx5Backend=ImportBase;module.Arx5BackendConfig=lambda **kw:NS(**kw)
            with patch.dict(sys.modules,{'arx_env.backends.arx5_backend':module}),patch('robot_workbench.arx_output_guard.require_guard',return_value=NS(sdk_contract={'pin':True})),patch('robot_workbench.sdk_quiesce.verify_module'):
                backend=ref.hardware_backend(can_interfaces=dict(left='can13l',right='can13r'),gripper_calibration=value)
                native=backend._import_arx5()
                for side,bus in [('left','can13l'),('right','can13r')]:
                    r=NS(robot_model='X5',gripper_closed_readout=0.,gripper_width=.088)
                    native.Arx5JointController(r,None,bus);row=value['sides'][side]
                    self.assertEqual(calls[-1],(bus,row['closed_readout'],row['open_readout'],.082))
                with self.assertRaisesRegex(ValueError,'capability'):
                    native.Arx5JointController(NS(robot_model='X5'),None,'can13l')
                self.assertEqual(len(calls),2)

if __name__=='__main__':unittest.main()
