"""Native ROS model transport with a CPU-only deterministic model callable."""
from pathlib import Path
import os
import sys
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch
import weakref
import numpy as np

ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'tests')]
from robot_workbench.ros_data import RosNode,StreamRef
from robot_workbench.ros_inputs import RosInputs
from robot_workbench.ros_inference import RosModelClient,RosModelWorker,reconstruct_input
from test_ros_inputs import sources


@unittest.skipUnless(os.environ.get('WORKBENCH_NATIVE_ROS_TEST')=='1','native ROS DDS needs isolated Linux')
class RosModelTests(unittest.TestCase):
    def setUp(self):
        from sensor_msgs.msg import Image
        self.nodes=[RosNode(name,namespace='/wb_cpu_inputs',domain_id=126)for name in('source','infer','control','record')]
        self.addCleanup(self.close)
        source,infer,control,record=self.nodes
        self.sources=sources()
        for row in self.sources:
            if row['kind']=='image':row['metadata'].update(width=640,height=480)
        pubs={r['stream_id']:source.publisher(r['topic'],Image if r['kind']=='image'else source.types.RobotState)for r in self.sources}
        self.inputs=RosInputs(infer,self.sources);self.record_inputs=RosInputs(record,self.sources)
        self.used=[];self.results=[];record.subscribe('model/inputs',record.types.ModelInput,self.used.append)
        record.subscribe('model/results',record.types.ModelResult,self.results.append)
        self.prepares=[];self.calls=[];self.fail=False;self.block=None;self.entered=threading.Event()
        def prepare(obs):
            self.prepares.append(obs)
            from robot_workbench.inference_model import letterbox_rgb224
            return np.asarray(obs['device_state14'],dtype=np.float32),{k:letterbox_rgb224(v)for k,v in obs['images'].items()}
        def predict(request,prepared):
            self.calls.append(request['request_id']);self.entered.set()
            if self.block is not None:self.block.wait(3)
            if self.fail:raise ValueError('cpu model failure')
            return dict(request_id=request['request_id'],action14=np.arange(840,dtype=float).reshape(60,14)/1000.,audit={'cpu_stub':True})
        from robot_workbench.inference_model import action_contract
        self.binding={'session_id':'model_session','model_binding':{'id':'cpu'},'conversion_id':'cpu_conversion','action_contract':action_contract(),'task':'coffee'}
        self.worker=RosModelWorker(infer,self.inputs,prepare,predict,binding=self.binding);self.worker.bind('run',1)
        self.client=RosModelClient(control,binding=self.binding);self.client.bind('run',1)
        self.wait(lambda:all(p.get_subscription_count()==2 for p in pubs.values()) and self.client.requests.get_subscription_count()==1
            and self.worker.inputs.get_subscription_count()==1 and self.worker.results.get_subscription_count()==2)
        now=time.monotonic_ns()
        for role in ('head','left','right'):
            msg=Image(height=480,width=640,encoding='rgb8',step=1920,data=bytes([3,6,9])*(640*480))
            msg.header.stamp.sec=now//10**9;msg.header.stamp.nanosec=now%10**9;pubs[role].publish(msg)
        ref=StreamRef('state','cpu_source','simulation',now,1)
        pubs['state'].publish(source.types.RobotState(reference=ref.message(),positions=[.1]*14,
            velocities=[0.]*14,efforts=[],metadata_json='{}'))
        self.wait(lambda:all(c.history()for c in self.inputs.caches.values()) and all(c.history()for c in self.record_inputs.caches.values()))
        self.observation=self.inputs.observation({r:self.inputs.latest(r)[0]for r in('head','left','right')},ref,
            run_id='run',epoch=1,model_io_digest='io',source_time_ns=now,pairing={})

    def close(self):
        if self.block:self.block.set()
        if hasattr(self,'worker'):self.worker.close()
        for node in reversed(self.nodes):node.close()

    def wait(self,condition):
        deadline=time.monotonic()+8
        while not condition() and time.monotonic()<deadline:time.sleep(.01)
        self.assertTrue(condition())

    def predict(self,identity='request',deadline=None):
        return self.client.predict(identity,self.observation,seed=3,task='coffee',history=[],
            valid_until_ns=deadline or time.monotonic_ns()+2_000_000_000)

    def test_references_reconstruct_exact_input_without_image_topic_copies(self):
        deadline=time.monotonic_ns()+2_000_000_000
        result=self.predict(deadline=deadline);self.wait(lambda:len(self.used)==1)
        state,images=reconstruct_input(self.used[0],self.record_inputs)
        self.assertTrue(np.array_equal(state,np.asarray([.1]*14,dtype=np.float32).astype(np.float64)))
        self.assertTrue(all(v.shape==(224,224,3)for v in images.values()))
        self.assertTrue(np.array_equal(result['action14'],np.arange(840).reshape(60,14)/1000.))
        self.assertEqual(self.calls,['request']);self.assertEqual(len(self.prepares),1)
        again=self.predict(deadline=deadline)
        self.assertTrue(np.array_equal(again['action14'],result['action14']))
        self.assertEqual(self.calls,['request'],'idempotent replay cannot run the model again')
        self.assertEqual(len(self.used),1,'idempotent replay cannot duplicate actual model input')
        for row in self.sources:self.assertEqual(self.nodes[0].node.count_publishers(row['topic']),1)
        self.assertFalse(hasattr(self.used[0],'images'))

    def test_failed_model_still_publishes_actual_input_and_error(self):
        self.fail=True
        with self.assertRaisesRegex(ValueError,'cpu model failure'):self.predict()
        self.wait(lambda:len(self.used)==1 and len(self.results)==1)
        self.assertEqual(self.results[0].error,'cpu model failure')
        reconstruct_input(self.used[0],self.record_inputs)

    def test_cancelled_result_cannot_be_applied(self):
        self.block=threading.Event();errors=[]
        def call():
            try:self.predict()
            except Exception as error:errors.append(error)
        thread=threading.Thread(target=call);thread.start()
        self.assertTrue(self.entered.wait(5));self.client.cancel();self.worker.cancel();self.block.set();thread.join(5)
        self.assertEqual(len(errors),1);self.assertIn('revoked',str(errors[0]))
        self.wait(lambda:bool(self.results));self.assertIn('revoked',self.results[0].error)


@unittest.skipUnless(os.environ.get('WORKBENCH_NATIVE_ROS_TEST')=='1','native ROS DDS needs isolated Linux')
class OwnerRosTests(unittest.TestCase):
    def test_owner_consumes_references_and_calls_model_without_business_query(self):
        from sensor_msgs.msg import Image
        from robot_workbench.ros_data import descriptor_message
        from robot_workbench.owner_inference import OwnerInference
        from robot_workbench.inference_owner import device_state
        from robot_workbench.execution_settings import defaults,control_parameters,policy_hash
        from robot_workbench.inference_workflow import CAMERAS,InferenceWorkflow
        from robot_workbench.observation_pairing import TimestampPairer
        plane=dict(schema='workbench.ros-data-plane.v1',namespace='/wb_owner_ros',domain_id=126)
        nodes=[RosNode(name,namespace=plane['namespace'],domain_id=126)for name in('source','model')]
        bridge=worker=None
        try:
            source,model_node=nodes;rows=[];publishers={};declarations=[]
            for role,stream in [*CAMERAS.items(),('state','state')]:
                image=stream!='state'
                row=dict(device_id='cpu_robot',stream_id=stream,source_instance_id='camera'if image else'owner',
                    source_clock_id='owner_clock',topic=plane['namespace']+'/'+stream,
                    message_type='sensor_msgs/msg/Image'if image else'workbench_interfaces/msg/RobotState',
                    kind='image'if image else'state',units='raw',coordinate_frame=role,
                    nominal_hz=30. if image else 100.,available=True,
                    metadata=dict(width=2,height=2,serial=role,environment='physical')if image else{})
                rows.append(row)
                publisher=source.publisher('sources/'+stream,source.types.SourceDescriptor,retained=True,depth=1)
                publisher.publish(descriptor_message(source,row));declarations.append(publisher)
                publishers[stream]=source.publisher(row['topic'],Image if image else source.types.RobotState)
            parameters=control_parameters(defaults('inference'),business='inference')
            policy=dict(revision=0,policy_hash=policy_hash(parameters),parameters=parameters)
            runtime=SimpleNamespace(data_plane=plane,nonce='owner',execution_calibration=None,
                owner=SimpleNamespace(epoch=1,runtime_nonce='owner',execution_clock_id='owner_clock',control_time=time.monotonic),
                control_inputs=lambda **kw:dict(joint_limits=[[-3.,3.]]*14,command_epoch=1,runtime_nonce='owner',
                    calibration_feedback=None,feedback_healthy=True,control_time_s=time.monotonic(),disabled_grippers=[]),
                _guard_policy=lambda _:policy)
            emitted=[]
            service=SimpleNamespace(device=SimpleNamespace(id='cpu_robot',environment='physical',runtime=runtime),
                event=lambda name,**kw:emitted.append((name,kw)))
            contract=dict(shape=[60,14],axes=[f'{s}_{a}'for s in('left','right')for a in('j1','j2','j3','j4','j5','j6','jaw')],
                units=(['rad']*6+['m'])*2,action_type='absolute_position',normalized=False,device_mapping_applied=False)
            binding=dict(session_id='model_session',model_binding={},conversion_id='conversion',action_contract=contract,task='coffee')
            configuration=dict(model_id='cpu',disabled_grippers=[],conversion_identity={'mapping':{}},
                camera_capture_instances={r:'camera'for r in CAMERAS},camera_serials={r:r for r in CAMERAS},
                data_plane=plane,action_chunk_binding=binding)
            with patch('robot_workbench.inference_workflow.calibrated_adapter',return_value=(object(),{})):
                bridge=OwnerInference(service,configuration,dict(profile={'gripper':{'max_opening_mm':83.}},
                    reference=None,preloaded_prediction=None,feedforward=None))
            model_inputs=RosInputs(model_node,rows);used=[]
            convert=lambda raw:device_state(raw['qpos'],disabled_grippers=(),right_gripper_device_mm=83.)
            def predict(request,prepared):
                used.append((request,prepared))
                return dict(binding,schema='workbench.action-chunk.v1',request_id=request['request_id'],epoch=request['epoch'],
                    action14=np.zeros((60,14)),audit={},execution_authorized=False,
                    observation=dict(source_time=request['observation']['source_time'],clock_domain=request['clock_domain'],
                        frame_ids=request['observation']['frame_ids']),
                    validity=dict(valid_until_source_time=request['valid_until_source_time'],clock_domain=request['clock_domain']))
            worker=RosModelWorker(model_node,model_inputs,lambda obs:(np.asarray(obs['device_state14']),obs['images']),
                predict,binding=binding,state_to_device=convert,clock_id='owner_clock')
            worker.bind('run',1)
            class Runner:pass
            runner=Runner();runner.owner=runtime.owner;runner.execution_instance_id='run';runner.guard_policy=policy
            runner.config={'maximum_observation_age_s':.5};runner._emit=lambda *args:None
            bridge.runner=weakref.ref(runner)
            observation_publisher=model_node.publisher('observations',model_node.types.Observation)
            deadline=time.monotonic()+5
            while (any(p.get_subscription_count()<1 for p in publishers.values()) or
                   observation_publisher.get_subscription_count()!=1 or worker.results.get_subscription_count()!=1)and time.monotonic()<deadline:time.sleep(.01)
            positions=[0.]*14;positions[6]=.03;positions[13]=.04
            def publish(sequence):
                now=time.monotonic_ns()
                for stream,publisher in publishers.items():
                    if stream=='state':
                        from robot_workbench.ros_data import json_text
                        reference=StreamRef('state','owner','owner_clock',now,sequence)
                        publisher.publish(source.types.RobotState(reference=reference.message(),positions=positions,
                            velocities=[0.]*14,efforts=[],metadata_json=json_text(dict(sample_time_s=now/1e9,
                                sample_wall_ns=time.time_ns(),command_epoch=1,runtime_nonce='owner'))))
                    else:
                        msg=Image(height=2,width=2,encoding='rgb8',step=6,data=bytes(range(12)))
                        msg.header.stamp.sec=now//10**9;msg.header.stamp.nanosec=now%10**9;publisher.publish(msg)
            publish(1)
            deadline=time.monotonic()+2
            while (not all(c.history()for c in model_inputs.caches.values())or not bridge.ros_inputs.caches['state'].samples)and time.monotonic()<deadline:time.sleep(.005)
            time.sleep(.04);publish(2)
            deadline=time.monotonic()+1
            while (model_inputs.latest('state')[0].sequence!=2 or bridge.ros_inputs.latest('state')[0].sequence!=2)and time.monotonic()<deadline:time.sleep(.005)
            # Exercise the actual workflow source selector, not a hand-built message.
            flow=object.__new__(InferenceWorkflow);flow.device=service.device;flow.ros_plane=plane
            flow.ros_inputs=model_inputs;flow.ros_observations=observation_publisher
            flow.client=SimpleNamespace(conversion_id='conversion');flow.disabled_grippers=()
            flow.lock=threading.RLock();flow.observation_lock=threading.Lock();flow.generation=1;flow.current=None
            flow.normal_modes=None;flow.ros_worker=None
            flow.event=None;flow._input_default_policy=policy;flow._sync_control_policy=lambda:None
            flow.prepared=dict(binding_digest='binding',profile={'gripper':{'max_opening_mm':83.}},capture_instances=None,disabled_grippers=[])
            flow.observation_pairer=TimestampPairer();flow.cameras=SimpleNamespace()
            with patch.object(runtime,'control_inputs',side_effect=AssertionError('live ROS input must not wait on status RPC')):
                flow._refresh_observation()
            self.assertNotIn('images',flow.cache['observation'],'workflow selection must not forward pixels')
            deadline=time.monotonic()+1
            while bridge.observation is None and time.monotonic()<deadline:time.sleep(.005)
            observation=bridge._ros_observe(time.monotonic());self.assertIsNotNone(observation,bridge.ros_error)
            # Independent reliable topics do not guarantee state-before-selection
            # delivery. Retain this exact reference until that state arrives.
            from copy import deepcopy
            from robot_workbench.ros_data import json_text
            entered=threading.Event();release=threading.Event();self.addCleanup(release.set)
            original_receive=bridge.ros_inputs._receive
            def delayed_receive(stream,message):
                if message.reference.sequence==3:entered.set();release.wait(1)
                return original_receive(stream,message)
            def state_message(sequence):
                stamp=time.monotonic_ns()
                return source.types.RobotState(reference=StreamRef('state','owner','owner_clock',stamp,sequence).message(),
                    positions=positions,velocities=[0.]*14,efforts=[],metadata_json=json_text(dict(sample_time_s=stamp/1e9,
                        sample_wall_ns=time.time_ns(),command_epoch=1,runtime_nonce='owner')))
            bridge.ros_inputs._receive=delayed_receive
            publishers['state'].publish(state_message(3));self.assertTrue(entered.wait(1))
            pending=deepcopy(observation['ros_observation'])
            for sequence in range(4,15):
                row=state_message(sequence);publishers['state'].publish(row)
                if sequence==8:pending.state=row.reference
                time.sleep(.001)
            bridge._receive_ros_observation(pending);self.assertIsNone(bridge.observation)
            release.set()
            deadline=time.monotonic()+.4
            while bridge.observation is None and time.monotonic()<deadline:time.sleep(.005)
            observation=bridge._ros_observe(time.monotonic());self.assertIsNotNone(observation,bridge.ros_error)
            self.assertEqual(observation['ros_observation'].state.sequence,8)
            self.assertNotIn('images',observation,'Owner must not consume camera payloads')
            result=bridge.predict(dict(request_id='request',seed=1,observation=observation,applied_history=[]))
            self.assertEqual(result['action14'].shape,(60,14));self.assertEqual(len(used),1)
            self.assertEqual(used[0][0]['observation']['frame_ids'],observation['frame_ids'])
            np.testing.assert_array_equal(used[0][1][0],observation['device_state14'])
            self.assertFalse(any(row[1].get('kind')=='query'for row in emitted))
            self.assertEqual(len(bridge.ros_inputs.sources),1,'control subscribes only to raw state')
            for row in rows:self.assertEqual(source.node.count_publishers(row['topic']),1)
            with self.assertRaisesRegex(ValueError,'legacy'):bridge.update('inference_observation',{})
            runtime.owner.epoch=2
            self.assertIsNone(bridge._ros_observe(time.monotonic()))
        finally:
            if bridge is not None:bridge.close()
            if worker is not None:worker.close()
            for node in reversed(nodes):node.close()


if __name__=='__main__':unittest.main()
