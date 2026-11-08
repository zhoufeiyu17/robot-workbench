"""Nonblocking ROS publication from the unique SDK/plant owner."""
from copy import deepcopy
from collections import OrderedDict
import queue
import threading
import time

from .ros_data import BoundedQueue, StreamRef, bounded_value, descriptor_message, json_text, MAX_METADATA_BYTES


class RosDriverPublisher:
    """The Owner submits immutable bounded facts; DDS never runs on its tick."""
    def __init__(self, ros, *, device_id, instance_id, clock_id, state_topic, event_topic, max_bytes=64*1024*1024):
        self.ros,self.device_id,self.instance_id,self.clock_id=ros,device_id,instance_id,clock_id
        self.state=ros.publisher(state_topic,ros.types.RobotState)
        # Preparation can release a burst of already buffered control facts.
        # Keep that burst within the existing byte budget, not the ROS default32.
        self.events=ros.publisher(event_topic,ros.types.DataEvent,depth=max(1,max_bytes//MAX_METADATA_BYTES))
        self.sources={}
        self.queue=BoundedQueue(max_bytes);self.lock=threading.RLock();self.closed=False
        self.sequences={'state':0,'control':0};self.references={};self.last_sample=None;self.error=None;self.dropped=0
        self.state_references=OrderedDict()
        self.source_values=[dict(device_id=device_id,stream_id=stream,source_instance_id=instance_id,
            source_clock_id=clock_id,topic=topic,message_type='workbench_interfaces/msg/'+kind,kind=source_kind,
            units=units,coordinate_frame='device',nominal_hz=hz,available=stream!='state',metadata={})
            for stream,topic,kind,source_kind,units,hz in (
                ('state',state_topic,'RobotState','state',{'positions':'SDK_rad_and_m','velocities':'SDK_rad_s_and_m_s'},100.),
                ('control',event_topic,'DataEvent','event','original_event_units',0.))]
        self.thread=threading.Thread(target=self._run,name='ros-driver-publisher',daemon=True);self.thread.start()
        for value in self.source_values:
            publisher=ros.publisher('sources/'+value['stream_id'],ros.types.SourceDescriptor,retained=True,depth=1)
            self.sources[value['stream_id']]=publisher;publisher.publish(descriptor_message(ros,value))

    def _enqueue(self, stream, stamp_ns, value):
        with self.lock:
            if self.closed:return False
            self.sequences[stream]+=1
            ref=StreamRef(stream,self.instance_id,self.clock_id,stamp_ns,self.sequences[stream])
            self.references[stream]=ref
            if stream=='state':
                self.state_references[stamp_ns]=ref
                while len(self.state_references)>256:self.state_references.popitem(last=False)
            try:
                snapshot,cost=bounded_value(value)
                self.queue.put_nowait((cost+1024,(stream,ref,snapshot)))
                return True
            except (ValueError,queue.Full)as error:
                self.error=(stream+('/'+str(value.get('event'))if stream=='control'else'')+': '+
                    (str(error)or'ROS source publication queue full'));self.dropped+=1;return False

    def publish_state(self, feedback):
        stamp=(feedback['source_time_ns']if feedback.get('source_clock_id')==self.clock_id
               else round(feedback['sample_time_s']*1e9))
        with self.lock:
            if self.last_sample==stamp:return True
            self.last_sample=stamp
        q,v=([float(x)for x in feedback[key]]for key in('qpos','qvel'))
        raw=feedback.get('raw_state',{});arms=raw.get('arms',{})
        efforts=[value for side in('left','right')for value in arms.get(side,{}).get('effort',[])]
        validity=[value is not None for value in efforts]
        metadata={k:deepcopy(feedback[k])for k in ('sample_time_s','sample_wall_ns','sdk_timestamps',
            'gripper_feedback_valid','gripper_feedback_reason','runtime_nonce','command_epoch','source_call_id',
            'clock_domain','source_clock_id','source_time_ns','physics_step','step_kind','environment','simulation_only')if k in feedback}
        metadata['effort_validity']=validity
        metadata['raw_metadata']={k:deepcopy(raw[k])for k in ('schema','status','joint_units','jaw_units','effort_units','pairing','host_time_scope','jaw_calibration_id')if k in raw}
        metadata['per_arm_samples']={side:{k:deepcopy(value)for k,value in arm.items()
            if k not in('qpos','qvel','effort')}for side,arm in arms.items()}
        return self._enqueue('state',stamp,dict(positions=q,velocities=v,
            efforts=[0. if value is None else value for value in efforts],metadata=metadata))

    def event(self, name, value, *, run_id='', epoch=0, source_time_ns=None):
        return self._enqueue('control',time.monotonic_ns()if source_time_ns is None else source_time_ns,
            dict(event=name,payload=value,run_id=run_id,epoch=epoch))

    def _run(self):
        while True:
            item=self.queue.get()
            try:
                if item is None:return
                stream,ref,value=item
                if stream=='state':
                    message=self.ros.types.RobotState(reference=ref.message(),positions=list(value['positions']),
                        velocities=list(value['velocities']),efforts=list(value['efforts']),metadata_json=json_text(value['metadata']))
                    self.state.publish(message)
                    if not self.source_values[0]['available']:
                        self.source_values[0]['available']=True
                        self.sources['state'].publish(descriptor_message(self.ros,self.source_values[0]))
                else:
                    self.events.publish(self.ros.types.DataEvent(reference=ref.message(),run_id=value['run_id'],
                        epoch=value['epoch'],event=value['event'],payload_json=json_text(value['payload'])))
            except Exception as error:
                with self.lock:self.error=str(error);self.dropped+=1
            finally:self.queue.task_done()

    def flush(self, timeout=5.):
        deadline=time.monotonic()+timeout
        while self.queue.unfinished_tasks:
            if time.monotonic()>=deadline:raise TimeoutError('ROS source publication is still draining')
            time.sleep(.002)
        with self.lock:
            if self.error:raise ValueError(self.error)
            return dict(self.sequences)

    def reference(self,stream,*,source_time_ns=None):
        with self.lock:
            value=self.state_references.get(source_time_ns)if stream=='state'and source_time_ns is not None else self.references.get(stream)
            return None if value is None else dict(value.__dict__)

    def close(self):
        with self.lock:
            if self.closed:return
            self.closed=True
        self.queue.put((1,None),timeout=5.);self.thread.join(5.)
        if self.thread.is_alive():raise TimeoutError('ROS publisher worker has not exited')
        for value in self.source_values:self.sources[value['stream_id']].publish(descriptor_message(self.ros,dict(value,available=False)))
