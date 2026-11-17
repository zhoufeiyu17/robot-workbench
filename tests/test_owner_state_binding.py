"""Owner state reader lifecycle over native DDS with synthetic CPU sources."""
from copy import deepcopy
import os
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


class OwnerBindingLifecycleTests(unittest.TestCase):
    def test_acquire_retires_old_inference_before_creating_the_new_runtime(self):
        from unittest.mock import Mock
        from robot_workbench.control_owner import ControlOwnerService
        service = object.__new__(ControlOwnerService)
        service.device = SimpleNamespace(runtime=None)
        service.maintenance = None
        bridge = Mock()
        service.inference = bridge
        def connect(args):
            self.assertEqual(args, {'arms_supported': True})
            self.assertIsNone(service.inference)
            bridge.close.assert_called_once_with()
        service.device.connect = connect
        service.inspect = lambda: {'connected': True}
        self.assertEqual(service._write('acquire', {'arms_supported': True}), {'connected': True})


@unittest.skipUnless(os.environ.get('WORKBENCH_NATIVE_ROS_TEST') == '1',
                     'native ROS DDS needs isolated Linux')
class OwnerStateBindingTests(unittest.TestCase):
    def setUp(self):
        from robot_workbench.ros_data import RosNode, descriptor_message
        from robot_workbench.inference_workflow import CAMERAS
        self.plane = dict(schema='workbench.ros-data-plane.v1',
                          namespace='/wb_state_rebind', domain_id=127)
        self.anchor = RosNode('camera_declarations', namespace=self.plane['namespace'], domain_id=127)
        self.service = SimpleNamespace(
            device=SimpleNamespace(id='cpu_robot', environment='physical', runtime=None),
            inference=None)
        self.bridge = self.producer = self.publisher = None
        self.stopped = None
        self.addCleanup(self.close)
        for role, stream in CAMERAS.items():
            row = dict(device_id='cpu_robot', stream_id=stream, source_instance_id='camera',
                       source_clock_id='host_wall', topic=self.plane['namespace']+'/'+stream,
                       message_type='sensor_msgs/msg/Image', kind='image', units='rgb8',
                       coordinate_frame=role, nominal_hz=30., available=True,
                       metadata=dict(width=2, height=2, serial=role))
            self.anchor.publisher('sources/'+stream, self.anchor.types.SourceDescriptor,
                                  retained=True, depth=1).publish(descriptor_message(self.anchor, row))
        self.configuration = dict(model_id='cpu', disabled_grippers=[], conversion_identity={'mapping': {}},
            camera_capture_instances={role: 'camera' for role in CAMERAS},
            camera_serials={role: role for role in CAMERAS}, data_plane=self.plane,
            action_chunk_binding=dict(session_id='cpu_model', conversion_id='cpu_conversion',
                                      action_contract={'shape': [32, 14]}))

    def connect(self, index):
        from robot_workbench.ros_data import RosNode
        from robot_workbench.ros_driver import RosDriverPublisher
        self.disconnect()
        nonce = 'synthetic_owner_'+str(index)
        self.producer = RosNode('control_'+nonce, namespace=self.plane['namespace'], domain_id=127)
        self.publisher = RosDriverPublisher(self.producer, device_id='cpu_robot', instance_id=nonce,
            clock_id='owner:'+nonce, state_topic=self.plane['namespace']+'/state',
            event_topic=self.plane['namespace']+'/control')
        self.service.device.runtime = SimpleNamespace(data_plane=self.plane, nonce=nonce,
            owner=SimpleNamespace(control_time=time.monotonic, epoch=1), ros_publisher=self.publisher,
            control_inputs=lambda **kw: dict(joint_limits=[[-3., 3.]]*14))
        self.stopped = threading.Event()
        def publish():
            while not self.stopped.wait(.01):
                self.publisher._enqueue('state', time.monotonic_ns(), dict(
                    positions=[0.]*14, velocities=[0.]*14, efforts=[],
                    metadata=dict(sample_wall_ns=time.time_ns(), diagnostic_padding='x'*16000)))
        self.worker = threading.Thread(target=publish)
        self.worker.start()

    def bind(self):
        from robot_workbench.owner_inference import OwnerInference
        with patch('robot_workbench.inference_workflow.calibrated_adapter', return_value=(object(), {})):
            candidate = OwnerInference(self.service, deepcopy(self.configuration),
                dict(profile={'gripper': {'max_opening_mm': 83.}}, reference=None,
                     preloaded_prediction=None, feedforward=None))
        previous = self.bridge
        self.service.inference = self.bridge = candidate
        if previous is not None:
            previous.close()
        return candidate

    def disconnect(self):
        if self.stopped is not None:
            self.stopped.set()
            self.worker.join(2)
            self.stopped = None
        if self.publisher is not None:
            self.publisher.close()
            self.publisher = None
        if self.producer is not None:
            self.producer.close()
            self.producer = None

    def close(self):
        self.disconnect()
        if self.bridge is not None:
            self.bridge.close()
        self.anchor.close()

    def test_repeated_owner_replacement_receives_current_state(self):
        import gc
        enabled = gc.isenabled()
        gc.disable()
        try:
            for index in range(20):
                with self.subTest(index=index):
                    self.connect(index)
                    bridge = self.bind()
                    cache = self.publisher.state_cache
                    self.assertIs(bridge.state_cache, cache)
                    with cache.lock:
                        reference = next(reversed(cache.samples))
                    self.assertEqual(reference.source_instance_id, self.service.device.runtime.nonce)
                    self.bind()
                    self.assertIs(self.bridge.state_cache, cache)
        finally:
            if enabled:
                gc.enable()

    def test_owner_does_not_subscribe_to_its_state_payload(self):
        from robot_workbench.ros_data import RosNode
        self.connect('no_loopback')
        subscribe = RosNode.subscribe
        def forbid_loopback(node, topic, message_type, callback, **kwargs):
            if message_type is node.types.RobotState:
                self.fail('Owner must consume producer history without an internal DDS state reader')
            return subscribe(node, topic, message_type, callback, **kwargs)
        with patch.object(RosNode, 'subscribe', forbid_loopback):
            bridge = self.bind()
        self.assertTrue(bridge.state_cache.samples)

    def test_producer_cache_matches_external_dds_and_is_committed_before_publication(self):
        from robot_workbench.ros_data import RosNode
        from robot_workbench.ros_inputs import RosInputs
        self.connect('exact')
        bridge = self.bind()
        node = RosNode('external_reader', namespace=self.plane['namespace'], domain_id=127)
        try:
            inputs = RosInputs(node, [self.publisher.state_cache.descriptor])
            cache = inputs.caches['state']
            with cache.changed:
                self.assertTrue(cache.changed.wait_for(lambda: bool(cache.samples), timeout=2))
            reference, raw = inputs.latest('state')
            self.assertEqual(bridge.state_cache.get(reference), raw)
            seen = []
            publish = self.publisher.state.publish
            def verify_before_publish(message):
                from robot_workbench.ros_data import StreamRef
                ref = StreamRef.from_message(message.reference)
                seen.append(ref)
                self.assertEqual(bridge.state_cache.get(ref)['qpos'], list(message.positions))
                return publish(message)
            with patch.object(self.publisher.state, 'publish', verify_before_publish):
                deadline = time.monotonic()+1
                while not seen and time.monotonic() < deadline:
                    time.sleep(.01)
            self.assertTrue(seen)
        finally:
            node.close()

    def test_stalled_producer_does_not_qualify_from_old_samples(self):
        self.connect('stalled')
        bridge = self.bind()
        cache = bridge.state_cache
        self.stopped.set()
        self.worker.join(2)
        self.publisher.flush()
        with self.assertRaisesRegex(TimeoutError, 'producer history stopped advancing'):
            self.bind()
        self.assertIs(self.service.inference, bridge)
        self.assertIsNone(cache.error)
        self.connect('recovered')
        self.assertIsNot(self.bind().state_cache, cache)
        self.assertIsNotNone(cache.error)

    def test_source_identity_mismatch_is_rejected(self):
        self.connect('identity')
        self.bind()
        self.publisher.state_cache.descriptor['source_instance_id'] = 'different_owner'
        with self.assertRaisesRegex(ValueError, 'history differs from the declared source'):
            self.bind()

    def test_retiring_a_bridge_preserves_the_producer_history(self):
        self.connect('retirement')
        bridge = self.bind()
        cache = bridge.state_cache
        bridge.close()
        self.service.inference = self.bridge = None
        self.assertIsNone(cache.error)
        self.assertFalse(self.publisher.state_listeners)
        self.assertIs(self.bind().state_cache, cache)
        self.assertEqual(cache.max_samples, 256)

    def test_future_reference_is_woken_by_production_and_old_references_are_not_substituted(self):
        from robot_workbench.inference_workflow import CAMERAS
        from robot_workbench.ros_data import StreamRef
        self.connect('future')
        bridge = self.bind()
        self.stopped.set()
        self.worker.join(2)
        self.publisher.flush()
        stamp, wall = time.monotonic_ns(), time.time_ns()
        reference = StreamRef('state', self.service.device.runtime.nonce,
            self.publisher.clock_id, stamp, self.publisher.sequences['state']+1)
        message = bridge.ros_node.types.Observation(observation_id='future', run_id='cpu', epoch=1,
            state=reference.message(), source_time_ns=stamp, model_io_digest='cpu_conversion',
            frames=[StreamRef(stream, 'camera', 'host_wall', wall, wall).message()
                    for stream in CAMERAS.values()], camera_roles=list(CAMERAS.values()), pairing_json='{}')
        with patch('robot_workbench.inference_owner.device_state', side_effect=lambda value, **kw: value):
            bridge._receive_ros_observation(message)
            self.assertIsNone(bridge.observation)
            self.assertIs(bridge.ros_pending_observation, message)
            self.publisher._enqueue('state', stamp, dict(positions=[0.]*14, velocities=[0.]*14,
                efforts=[], metadata={'sample_wall_ns': wall}))
            self.publisher.flush()
            self.assertIsNotNone(bridge.observation, bridge.ros_error)
            self.assertEqual(bridge.observation['ros_observation'].state.sequence, reference.sequence)
            message.epoch = 2
            bridge._receive_ros_observation(message)
            self.assertIsNone(bridge.observation)
            self.assertIn('another Owner, epoch or model conversion', bridge.ros_error)
        for index in range(300):
            self.publisher._enqueue('state', stamp+index+1, dict(positions=[0.]*14,
                velocities=[0.]*14, efforts=[], metadata={}))
        self.publisher.flush()
        self.assertEqual(len(bridge.state_cache.samples), 256)
        with self.assertRaisesRegex(ValueError, 'evicted'):
            bridge.state_cache.get(reference)


if __name__ == '__main__':
    unittest.main()
