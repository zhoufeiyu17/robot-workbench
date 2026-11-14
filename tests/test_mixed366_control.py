"""Native bimanual coordinates, RGB provenance and nohistory requests; CPU only."""
from copy import deepcopy
import hashlib,json,sys,tempfile,time,unittest
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'calibration/infra')]
from robot_workbench.mixed366_control import SCHEMA,ENDPOINTS,SERIALS,load_binding,adapter
from arx_env import joint_head_jaw_mapping as jaw


def native_mapping_fixture(root,robot_id='arx6',widths=(.083,.083)):
    root=Path(root);root.mkdir(parents=True,exist_ok=True)
    def put(name,value):
        p=root/name;p.write_text(json.dumps(value));return dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest())
    source=dict(id=robot_id,left_can='can3'if robot_id=='arx6'else'can13l',right_can='can1'if robot_id=='arx6'else'can13r',left_model='X5',right_model='X5')
    sides={s:dict(row,width_m=width)for(s,row),width in zip(ENDPOINTS.items(),widths)}
    for side,row in sides.items():source.update({side+'_gripper_'+k:v for k,v in row.items()})
    native=put('endpoints.json',dict(schema='native_gripper_endpoints.v1',robot_id=robot_id,source=put('machine.json',source),sdk_width_m=.082,sides=sides))
    mapping=put('mapping.json',dict(schema_version='native_gripper_mapping.v1',robot_id=robot_id,native_gripper_calibration=native))
    return native,mapping


class NativeTest(unittest.TestCase):
    def tearDown(self):jaw.configure_mapping(None)

    def test_recorded_native_roundtrip_both_arms_and_full_command_range(self):
        with tempfile.TemporaryDirectory()as directory:
            root=Path(directory)
            def put(name,v):
                p=root/name;p.write_text(json.dumps(v));return dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest())
            native,mapping=native_mapping_fixture(root);jaw.configure_mapping(mapping['path'])
            sdk=put('sdk.so',{'cpu_only':True})
            profile=dict(schema=SCHEMA,robot_id='arx6',sdk_module=sdk,gripper_calibration=native,camera_serials=SERIALS,gripper={'max_opening_mm':83.})
            deployment=dict(robot_id='arx6',disabled_grippers=[],sdk_quiesce={'module':sdk},native_gripper_calibration=native,cameras={k:{'serial':v}for k,v in SERIALS.items()})
            loaded,binding=load_binding(put('binding.json',profile),deployment)
            limits=[[-3.,3.]for _ in range(14)];limits[6]=limits[13]=[0.,.082]
            a,identity=adapter(loaded,limits,[])
            raw=np.linspace(-.5,.5,14);raw[[6,13]]=[23.,76.]
            model=a.device_to_model(raw);np.testing.assert_allclose(model[[6,13]],[.023,.076]);np.testing.assert_allclose(a.model_to_device(model),raw)
            prediction=model.copy();prediction[7]=1.2;prediction[13]=.1
            decoded=a.decode(prediction,raw);self.assertEqual(decoded['final_device14'][7],1.2);self.assertEqual(decoded['final_device14'][13],100.)
            self.assertEqual(decoded['projections'],[]);self.assertEqual(identity['history_steps'],0)
            self.assertEqual(identity['command_jaw_limits_mm'],[0.,83.])
            np.testing.assert_allclose(a.limits[[6,13],1],[83.,83.])
            from robot_workbench.inference_model import CAMS
            images={c:np.zeros((480,640,3),np.uint8)for c in CAMS}
            state,result=a.model_input(dict(device_state14=raw,images=images));self.assertIs(result,images)
            images[CAMS[0]]=np.zeros((224,224,3),np.uint8)
            with self.assertRaises(ValueError):a.model_input(dict(device_state14=raw,images=images))
            bad=deepcopy(deployment);bad['disabled_grippers']=['right']
            with self.assertRaises(ValueError):load_binding(put('binding.json',profile),bad)

    def test_online_reference_retains_both_arm_predictions(self):
        from robot_workbench.motion_reference import prepare_reference
        jaw.configure_mapping(None);limits=[[-3.,3.]for _ in range(14)];limits[6]=limits[13]=[0.,.08]
        initial=np.zeros(14);target=initial.copy();target[0]=.001;target[7]=-.002
        reference=prepare_reference(dict(device_targets14=[initial.tolist(),target.tolist()],duration_s=.5,
            kind='online_prediction_chunk',active_arms=['left','right']),initial,limits,source_sha256='a'*64)
        q,_,_=reference.sample([reference.duration_s]);self.assertAlmostEqual(q[0,7],-.002)
        self.assertEqual(reference.metadata['active_arms'],['left','right'])

    def test_nohistory_loop_has_no_completed_command_prefix(self):
        from robot_workbench.inference import InferenceLoop
        requests=[]
        def predict(q):requests.append(q);return dict(request_id=q['request_id'],action14=np.zeros((60,14)))
        loop=InferenceLoop(predict=predict,decode=lambda x,s:{'final_device14':x},device_to_model=lambda x:x,history_steps=0)
        loop.start(epoch=1,source_time=0.);loop.history.append(np.ones(14))
        loop._launch(dict(source_time=0.,device_state14=np.zeros(14)))
        deadline=time.monotonic()+1
        while not requests and time.monotonic()<deadline:time.sleep(.001)
        self.assertEqual(requests[0]['applied_history'].shape,(0,14))


if __name__=='__main__':unittest.main()
