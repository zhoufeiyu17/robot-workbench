"""Direct image/state subscriptions and shared-reference observations."""
from pathlib import Path
import os
import sys
import time
import json
import threading
import urllib.request
from types import SimpleNamespace
import unittest

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from robot_workbench.ros_inputs import RosInputs,image_sample,state_sample
from robot_workbench.ros_data import StreamRef


def sources():
    rows=[]
    for role in ('head','left','right','state'):
        image=role!='state'
        rows.append(dict(device_id='cpu_robot',stream_id=role,source_instance_id='cpu_source',
            source_clock_id='simulation',topic='/wb_cpu_inputs/'+role,
            message_type='sensor_msgs/msg/Image'if image else'workbench_interfaces/msg/RobotState',
            kind='image'if image else'state',units='rgb8'if image else'raw_sdk',
            coordinate_frame=role,nominal_hz=30. if image else 100.,available=True,
            metadata={'width':2,'height':2,'environment':'fake'}if image else{}))
    return rows


class SourceDecoding(unittest.TestCase):
    @unittest.skipUnless(os.environ.get('WORKBENCH_NATIVE_ROS_TEST')=='1','native ROS DDS needs isolated Linux')
    def test_state_delivery_survives_a_short_consumer_pause_with_small_image_history(self):
        from robot_workbench.ros_data import RosNode
        producer=RosNode('paused_state_source',namespace='/wb_cpu_inputs',domain_id=126)
        consumer=RosNode('paused_state_reader',namespace='/wb_cpu_inputs',domain_id=126)
        entered=threading.Event();resume=threading.Event();received=[]
        try:
            source=sources()[-1];publisher=producer.publisher(source['topic'],producer.types.RobotState)
            def sample(ref,value):
                received.append(ref.sequence)
                if ref.sequence==1:entered.set();resume.wait(3)
            inputs=RosInputs(consumer,[source],max_samples=16,on_sample=sample)
            deadline=time.monotonic()+5
            while publisher.get_subscription_count()!=1:
                self.assertLess(time.monotonic(),deadline);time.sleep(.01)
            def publish(sequence):
                ref=StreamRef('state','cpu_source','simulation',sequence*10_000_000,sequence)
                publisher.publish(producer.types.RobotState(reference=ref.message(),positions=[0.]*14,
                    velocities=[0.]*14,metadata_json='{}'))
            publish(1);self.assertTrue(entered.wait(3))
            for sequence in range(2,41):publish(sequence);time.sleep(.01)
            resume.set();deadline=time.monotonic()+3
            while not received or received[-1]!=40:
                self.assertLess(time.monotonic(),deadline);time.sleep(.01)
            self.assertEqual(received,list(range(1,41)))
            self.assertLessEqual(len(inputs.caches['state'].history()),16)
        finally:resume.set();consumer.close();producer.close()

    @unittest.skipUnless(os.environ.get('WORKBENCH_NATIVE_ROS_TEST')=='1','native ROS DDS needs isolated Linux')
    def test_original_physical_rgbd_publisher_preserves_hardware_metadata(self):
        import tempfile
        from real_rgbd import RosCapture,Frames,pair_metadata
        from robot_workbench.ros_data import RosNode,SourceDirectory
        from robot_workbench.ros_inputs import RosCameraView
        from real_observation import decode_snapshot
        plane=dict(schema='workbench.ros-data-plane.v1',namespace='/wb_original_physical',domain_id=126)
        with tempfile.TemporaryDirectory()as folder:
            root=Path(folder);intr=dict(serial='123',width=2,height=2,fx=10.,fy=10.,cx=1.,cy=1.,coefficients=[],distortion_model='none',rectified=False)
            path=root/'intrinsics.json';path.write_text(json.dumps(intr))
            source=RosCapture(dict(data_plane=plane,robot_id='cpu',cameras={'head':dict(serial='123',intrinsics=str(path))}),'head',root,Frames())
            receiver=RosNode('physical_reader',namespace=plane['namespace'],domain_id=126)
            try:
                def publish(number):
                    stamp=time.time_ns()
                    status=pair_metadata(serial='123',camera_name='head',color_number=number,depth_number=number+10,
                        color_ms=number*33.,depth_ms=number*33.+1.,color_domain='hardware_clock',depth_domain='hardware_clock',
                        host_receive_ns=stamp,depth_scale_m=.001,intrinsics=intr)
                    status['host_receive_ns']=stamp
                    source.publish('base_0',status,bytes(range(12)),bytes([244,1])*4,width=2,height=2)
                    return stamp
                publish(1)
                rows=SourceDirectory(receiver,'cpu').resolve({'base_0':dict(kind='image')})
                saved=[];inputs=RosInputs(receiver,rows,include_depth=True,on_sample=lambda ref,value:saved.append((ref,value)))
                deadline=time.monotonic()+5;number=1
                while not saved:
                    number+=1;publish(number);time.sleep(.02);self.assertLess(time.monotonic(),deadline)
                ref,value=saved[-1];meta=value['metadata']
                self.assertEqual(meta['host_receive_ns'],ref.source_time_ns)
                self.assertEqual(meta['frame_number'],number);self.assertEqual(meta['depth_frame_number'],number+10)
                self.assertEqual(meta['device_timestamp_ms'],number*33.)
                self.assertEqual(meta['depth_device_timestamp_ms'],number*33.+1.)
                self.assertEqual(meta['timestamp_domain'],'hardware_clock')
                self.assertEqual(value['rgb'],bytes(range(12)));self.assertEqual(value['depth'],bytes([244,1])*4)
                self.assertNotIn('physics_step',meta);self.assertNotIn('owner_pid',meta)
                self.assertEqual(source.ros.node.count_publishers(plane['namespace']+'/base_0'),1)
                # The real acquisition decoder sees the original hardware times,
                # not synthetic device timestamps derived from the ROS header.
                view=object.__new__(RosCameraView);view.environment='physical';view.intrinsics={'base_0':intr}
                view.sources={'base_0':rows[0]}
                from robot_workbench.rgbd_service import _Preview
                view.frames={'base_0':_Preview()};view._receive(ref,value)
                from real_rgbd import snapshot_bundle
                frame=view.frames['base_0'].snapshot(formats=('color_png','depth_png'))
                decoded=decode_snapshot(snapshot_bundle(frame['status'],frame['color_png'],frame['depth_png']),'123',
                    dict(frames=dict(environment_from_camera=dict(position=[0,0,0],orientation_wxyz=[1,0,0,0]))))
                self.assertEqual(decoded[0]['depth_frame_number'],number+10)
                changed=dict(intr,fx=11.)
                status=pair_metadata(serial='123',camera_name='head',color_number=9,depth_number=19,color_ms=100.,depth_ms=101.,
                    color_domain='hardware_clock',depth_domain='hardware_clock',host_receive_ns=time.time_ns(),depth_scale_m=.001,intrinsics=changed)
                with self.assertRaisesRegex(ValueError,'intrinsics'):source.publish('base_0',status,bytes(12),bytes(8),width=2,height=2)
                source.close()
                deadline=time.monotonic()+3
                while not inputs.caches['base_0'].error:
                    self.assertLess(time.monotonic(),deadline);time.sleep(.01)
            finally:
                if not source.ros.closed:source.close()
                receiver.close()

    def test_depth16_padding_and_byte_order_preserve_original_values(self):
        source=dict(sources()[0]);source['metadata']=dict(source['metadata'],pixel_format='depth16le')
        message=SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=1,nanosec=7)),
            width=2,height=2,encoding='16UC1',is_bigendian=True,step=6,
            data=bytes([0,1,1,244,99,99,3,232,255,255,99,99]))
        ref,value,size=image_sample(message,source)
        self.assertEqual(value['depth'],bytes([1,0,244,1,232,3,255,255]))
        self.assertIsNone(value['rgb']);self.assertEqual(ref.source_time_ns,1_000_000_007)
        message.encoding='rgb8'
        with self.assertRaises(ValueError):image_sample(message,source)

    @unittest.skipUnless(os.environ.get('WORKBENCH_NATIVE_ROS_TEST')=='1','native ROS DDS needs isolated Linux')
    def test_rgbd_pairs_only_identical_source_stamps_without_model_depth_subscription(self):
        from sensor_msgs.msg import Image
        from robot_workbench.ros_data import RosNode,descriptor_message
        producer=RosNode('rgbd_source',namespace='/wb_cpu_inputs',domain_id=126)
        receiver=RosNode('rgbd_receiver',namespace='/wb_cpu_inputs',domain_id=126)
        model=RosNode('rgb_model',namespace='/wb_cpu_inputs',domain_id=126)
        try:
            rgb=sources()[0];rgb['metadata'].update(serial='camera',depth_stream_id='head_depth')
            depth=dict(rgb,stream_id='head_depth',topic='/wb_cpu_inputs/head_depth',units='depth16le',
                metadata=dict(width=2,height=2,serial='camera',environment='fake',pixel_format='depth16le',
                              depth_scale_m=.001,depth_aligned_to='color'))
            writer=producer.publisher(rgb['topic'],Image);depth_writer=producer.publisher(depth['topic'],Image)
            declaration=producer.publisher('sources/head_depth',producer.types.SourceDescriptor,retained=True,depth=1)
            declaration.publish(descriptor_message(producer,depth))
            recorded=[];inputs=RosInputs(receiver,[rgb],include_depth=True,max_samples=2,on_sample=lambda ref,value:recorded.append((ref,value)))
            model_inputs=RosInputs(model,[rgb])
            self.assertNotIn('head_depth',model_inputs.sources)
            deadline=time.monotonic()+5
            while writer.get_subscription_count()!=2 or depth_writer.get_subscription_count()!=1:
                self.assertLess(time.monotonic(),deadline);time.sleep(.01)
            def publish(target,stamp,encoding,data,step):
                message=Image(height=2,width=2,encoding=encoding,step=step,data=data)
                message.header.stamp.sec=1;message.header.stamp.nanosec=stamp;target.publish(message)
            for stamp in (1,2,3):publish(writer,stamp,'rgb8',bytes(range(12)),6)
            deadline=time.monotonic()+3
            while not inputs.caches['head'].samples or inputs.latest('head')[0].source_time_ns!=1_000_000_003:
                self.assertLess(time.monotonic(),deadline);time.sleep(.01)
            self.assertIsNone(inputs.caches['head'].error,'pre-join RGB history must stay bounded without poisoning the source')
            self.assertLessEqual(len(inputs.pending_rgbd),2)
            publish(depth_writer,3,'16UC1',bytes([1,0,2,0,3,0,4,0]),4)
            deadline=time.monotonic()+3
            while len(recorded)<1:
                self.assertLess(time.monotonic(),deadline);time.sleep(.01)
            ref,value=recorded[0]
            self.assertEqual(value['rgb'],bytes(range(12)))
            self.assertEqual(value['depth'],bytes([1,0,2,0,3,0,4,0]))
            self.assertEqual(value['metadata']['depth_reference']['source_time_ns'],ref.source_time_ns)
            self.assertFalse(inputs.pending_rgbd,'pre-join incomplete frames are outside the first complete-pair boundary')
            publish(writer,4,'rgb8',bytes(range(12)),6)
            publish(depth_writer,5,'16UC1',bytes(8),4)
            deadline=time.monotonic()+3
            while len(inputs.caches['head_depth'].history())<2:
                self.assertLess(time.monotonic(),deadline);time.sleep(.01)
            self.assertEqual(len(recorded),1,'a nearby depth sample must not replace the matching one')
            self.assertTrue(any(r.source_time_ns==1_000_000_004 for r in inputs.pending_rgbd))
            self.assertEqual(producer.node.count_publishers(rgb['topic']),1)
            self.assertEqual(producer.node.count_publishers(depth['topic']),1)
        finally:model.close();receiver.close();producer.close()

    @unittest.skipUnless(os.environ.get('WORKBENCH_NATIVE_ROS_TEST')=='1','native ROS DDS needs isolated Linux')
    def test_image_subscription_matches_the_actual_best_effort_publisher(self):
        from sensor_msgs.msg import Image
        from robot_workbench.ros_data import RosNode
        source=RosNode('best_effort_source',namespace='/wb_cpu_inputs',domain_id=126)
        consumer=RosNode('best_effort_consumer',namespace='/wb_cpu_inputs',domain_id=126)
        try:
            descriptor=sources()[0]
            publisher=source.node.create_publisher(Image,descriptor['topic'],source.qos(reliable=False))
            inputs=RosInputs(consumer,[descriptor])
            self.assertEqual(inputs.reliability['head'],'best_effort')
            deadline=time.monotonic()+3
            while publisher.get_subscription_count()!=1 and time.monotonic()<deadline:time.sleep(.01)
            message=Image(height=2,width=2,encoding='rgb8',step=6,data=bytes(range(12)))
            message.header.stamp.sec=1;publisher.publish(message)
            deadline=time.monotonic()+3
            while not inputs.caches['head'].samples and time.monotonic()<deadline:time.sleep(.01)
            self.assertEqual(inputs.latest('head')[1]['rgb'],bytes(range(12)))
        finally:consumer.close();source.close()

    @unittest.skipUnless(os.environ.get('WORKBENCH_NATIVE_ROS_TEST')=='1','native ROS DDS needs isolated Linux')
    def test_preview_subscribes_original_images_and_keeps_source_stamp(self):
        from sensor_msgs.msg import Image
        from robot_workbench.ros_data import RosNode,descriptor_message
        from robot_workbench.ros_inputs import RosCameraView
        plane=dict(schema='workbench.ros-data-plane.v1',namespace='/wb_preview',domain_id=126)
        node=RosNode('preview_source',namespace=plane['namespace'],domain_id=126)
        view=None;stop=threading.Event();thread=None
        try:
            intrinsics={name:dict(serial=name,width=2,height=2)for name in('base_0','left_wrist_0','right_wrist_0')}
            pubs={};declared={};descriptors={}
            for name,k in intrinsics.items():
                descriptors[name]=dict(device_id='cpu_robot',stream_id=name,source_instance_id='source',source_clock_id='host_wall',
                    topic=plane['namespace']+'/'+name,message_type='sensor_msgs/msg/Image',kind='image',units='rgb8',
                    coordinate_frame=name,nominal_hz=30.,available=True,metadata=dict(k,environment='physical'))
                declared[name]=node.publisher('sources/'+name,node.types.SourceDescriptor,retained=True,depth=1)
                declared[name].publish(descriptor_message(node,descriptors[name]))
                pubs[name]=node.publisher(descriptors[name]['topic'],Image)
            def produce():
                while not stop.wait(.02):
                    now=time.time_ns()
                    for p in pubs.values():
                        image=Image(width=2,height=2,step=6,encoding='rgb8',data=bytes(range(12)))
                        image.header.stamp.sec=now//10**9;image.header.stamp.nanosec=now%10**9;p.publish(image)
            view=RosCameraView(plane,'cpu_robot',intrinsics,'physical')
            thread=threading.Thread(target=produce);thread.start();ports=view.start_servers()
            deadline=time.monotonic()+5
            while not view.frames['base_0'].snapshot(formats=())['status'].get('ready')and time.monotonic()<deadline:time.sleep(.01)
            with urllib.request.urlopen('http://127.0.0.1:'+str(ports['head'])+'/frame.jpg')as response:
                metadata=json.loads(response.headers['X-Camera-Metadata']);jpeg=response.read()
            self.assertTrue(jpeg.startswith(b'\xff\xd8'))
            self.assertEqual(metadata['source_time_ns'],metadata['host_receive_ns'])
            self.assertEqual(metadata['capture_instance_id'],'source')
            self.assertEqual(view.channels,{})
            for name in pubs:self.assertEqual(node.node.count_publishers(descriptors[name]['topic']),1)
            changed=dict(descriptors['base_0'],available=False)
            declared['base_0'].publish(descriptor_message(node,changed))
            deadline=time.monotonic()+3
            while view.frames['base_0'].snapshot(formats=())['status'].get('ready')and time.monotonic()<deadline:time.sleep(.01)
            self.assertFalse(view.frames['base_0'].snapshot(formats=())['status'].get('ready'))
            declared['base_0'].publish(descriptor_message(node,descriptors['base_0']))
            time.sleep(.1)
            self.assertFalse(view.frames['base_0'].snapshot(formats=())['status'].get('ready'),'same source returning also requires explicit rebind')
            self.assertTrue(view.reconnect(max_age_s=.5)['reconnected'])
            self.assertEqual(view.frames['base_0'].snapshot(formats=())['status']['capture_instance_id'],'source')
            declared['base_0'].publish(descriptor_message(node,changed))
            deadline=time.monotonic()+3
            while view.frames['base_0'].snapshot(formats=())['status'].get('ready')and time.monotonic()<deadline:time.sleep(.01)
            declared['base_0'].publish(descriptor_message(node,dict(descriptors['base_0'],source_instance_id='replacement')))
            time.sleep(.1)
            self.assertFalse(view.frames['base_0'].snapshot(formats=())['status'].get('ready'),'source replacement needs explicit rebind')
            self.assertTrue(view.reconnect(max_age_s=.5)['reconnected'])
            self.assertEqual(view.frames['base_0'].snapshot(formats=())['status']['capture_instance_id'],'replacement')
        finally:
            stop.set()
            if thread:thread.join(2)
            if view:view.close()
            node.close()

    def test_image_stamp_pixels_and_padding_are_original(self):
        stamp=SimpleNamespace(sec=1,nanosec=7)
        message=SimpleNamespace(header=SimpleNamespace(stamp=stamp),width=2,height=2,
            encoding='bgr8',step=8,data=bytes([1,2,3,4,5,6,99,99]*2))
        ref,value,size=image_sample(message,sources()[0])
        self.assertEqual(ref.source_time_ns,1_000_000_007)
        self.assertEqual(value['rgb'],bytes([3,2,1,6,5,4]*2))
        self.assertEqual(value['metadata']['original_encoding'],'bgr8')
        message.width=3
        with self.assertRaises(ValueError):image_sample(message,sources()[0])

    def test_state_metadata_cannot_replace_canonical_values(self):
        reference=SimpleNamespace(**StreamRef('state','cpu_source','simulation',7,1).__dict__)
        value=SimpleNamespace(reference=reference,positions=[0.]*14,velocities=[0.]*14,efforts=[],metadata_json='{}')
        ref,state,_=state_sample(value,sources()[-1]);self.assertEqual(state['qpos'],[0.]*14)
        value.metadata_json='{"qpos":[123]}'
        with self.assertRaises(ValueError):state_sample(value,sources()[-1])

    @unittest.skipUnless(os.environ.get('WORKBENCH_NATIVE_ROS_TEST')=='1','native ROS DDS needs isolated Linux')
    def test_one_publication_is_shared_by_input_and_record_subscribers(self):
        from sensor_msgs.msg import Image
        from robot_workbench.ros_data import RosNode
        nodes=[RosNode(name,namespace='/wb_cpu_inputs',domain_id=126)for name in('source','input','recorder')]
        try:
            source,input_node,record_node=nodes
            publishers={row['stream_id']:source.publisher(row['topic'],Image if row['kind']=='image'else source.types.RobotState)for row in sources()}
            inputs=RosInputs(input_node,sources());recorded=[]
            recorder=RosInputs(record_node,sources(),on_sample=lambda ref,value:recorded.append(ref))
            deadline=time.monotonic()+10
            while not all(p.get_subscription_count()==2 for p in publishers.values())and time.monotonic()<deadline:time.sleep(.02)
            self.assertTrue(all(p.get_subscription_count()==2 for p in publishers.values()))
            for role in ('head','left','right'):
                msg=Image(height=2,width=2,encoding='rgb8',step=6,data=bytes(range(12)))
                msg.header.stamp.sec=1;msg.header.stamp.nanosec=7
                publishers[role].publish(msg)
            ref=StreamRef('state','cpu_source','simulation',1_000_000_007,1)
            publishers['state'].publish(source.types.RobotState(reference=ref.message(),positions=[.1]*14,
                velocities=[0.]*14,efforts=[],metadata_json='{}'))
            deadline=time.monotonic()+5
            while len(recorded)!=4 and time.monotonic()<deadline:time.sleep(.01)
            self.assertEqual(len(recorded),4)
            frames={role:inputs.latest(role)[0]for role in('head','left','right')}
            observation=inputs.observation(frames,ref,run_id='run',epoch=1,model_io_digest='io',
                source_time_ns=ref.source_time_ns,pairing={})
            state,images=recorder.resolve_observation(observation)
            self.assertEqual(state['qpos'],[.1]*14)
            self.assertTrue(all(value['rgb']==bytes(range(12))for value in images.values()))
            self.assertFalse(hasattr(observation,'images'),'selection messages must not copy image payloads')
            self.assertEqual(len(observation.frames),3)
            for row in sources():self.assertEqual(source.node.count_publishers(row['topic']),1)
            self.assertEqual(list(input_node.errors),[]);self.assertEqual(list(record_node.errors),[])
            # A second DDS writer cannot silently take over the same topic/source.
            replacement=RosNode('replacement',namespace='/wb_cpu_inputs',domain_id=126);nodes.append(replacement)
            writer=replacement.publisher('/wb_cpu_inputs/state',replacement.types.RobotState)
            deadline=time.monotonic()+5
            while writer.get_subscription_count()!=2 and time.monotonic()<deadline:time.sleep(.02)
            writer.publish(source.types.RobotState(reference=StreamRef('state','cpu_source','simulation',1_000_000_008,2).message(),
                positions=[.2]*14,velocities=[0.]*14,efforts=[],metadata_json='{}'))
            deadline=time.monotonic()+5
            while not inputs.caches['state'].error and time.monotonic()<deadline:time.sleep(.01)
            with self.assertRaisesRegex(ValueError,'publisher'):inputs.latest('state')
        finally:
            for node in reversed(nodes):node.close()


if __name__=='__main__':unittest.main()
