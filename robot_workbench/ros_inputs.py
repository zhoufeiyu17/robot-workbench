"""Direct source subscriptions shared by input, inference and recorder processes.

This module never republishes a camera frame or a state sample. Model requests
refer to samples in these bounded histories; a missing sample is an error.
"""
from copy import deepcopy
from collections import OrderedDict
import hashlib
import math
import re
import secrets
import threading
import time

from .ros_data import MAX_CACHE_SAMPLES, MAX_METADATA_BYTES, SampleCache, StreamRef, descriptor_value, json_text, json_value, source_descriptor
from .pipeline_timing import timing


def observation_times(message, feedback):
    """Map selected original stamps using the selected state clock pair only."""
    state=StreamRef.from_message(message.state);times=[]
    for raw in message.frames:
        ref=StreamRef.from_message(raw)
        if ref.source_clock_id==state.source_clock_id:stamp=ref.source_time_ns
        elif ref.source_clock_id=='host_wall' and type(feedback.get('sample_wall_ns'))is int:
            stamp=state.source_time_ns+ref.source_time_ns-feedback['sample_wall_ns']
        else:raise ValueError('selected sources have no declared common clock mapping')
        times.append(stamp)
    if len(times)!=3 or message.source_time_ns!=min(times):
        raise ValueError('observation source time differs from its original selected samples')
    return times


def referenced_observation(message, sources, feedback, *, frames=None, state_to_device=None):
    """One deterministic view of a selection for control and model consumers.

    Camera identities come from the original source, never subscriber arrival
    clocks. Control leaves frames unset and does not subscribe to image bytes.
    """
    if len(message.frames)!=3 or len(set(message.camera_roles))!=3:
        raise ValueError('three distinct selected camera roles required')
    observation_times(message,feedback)
    identities={}
    for role,raw in zip(message.camera_roles,message.frames):
        ref=StreamRef.from_message(raw);source=sources[ref.stream_id]
        if source['kind']!='image' or any(getattr(ref,k)!=source[k] for k in
                ('source_instance_id','source_clock_id')):
            raise ValueError('selected camera differs from the bound original source')
        identities[role]=dict(serial=source['metadata'].get('serial'),capture_instance_id=ref.source_instance_id,
            frame_number=ref.sequence,source_time_ns=ref.source_time_ns,source_clock_id=ref.source_clock_id)
        if ref.source_clock_id=='host_wall':identities[role]['host_receive_ns']=ref.source_time_ns
    value=dict(source_time=message.source_time_ns/1e9,run_id=message.run_id,
        state_reference=StreamRef.from_message(message.state).__dict__,
        device_state14=state_to_device(feedback) if state_to_device else feedback['qpos'],
        frame_ids=identities,ros_observation=message)
    if 'memory_sampling'in feedback:
        value.update(memory_sampling=deepcopy(feedback['memory_sampling']),
            feedback_sample_time_s=message.state.source_time_ns/1e9)
    if frames is not None:
        import numpy as np
        if set(frames)!=set(identities):raise ValueError('selected frame roles differ')
        value['images']={role:np.frombuffer(frame['rgb'],dtype=np.uint8).reshape(frame['height'],frame['width'],3)
                         for role,frame in frames.items()}
    return value


def image_sample(message, source):
    """Decode the original ROS Image without changing its source timestamp."""
    stamp = int(message.header.stamp.sec)*10**9 + int(message.header.stamp.nanosec)
    meta = source['metadata']; width, height = int(message.width), int(message.height)
    depth=meta.get('pixel_format')=='depth16le';channels=2 if depth else 3
    encodings=('16UC1',)if depth else('rgb8','bgr8')
    if ((width, height) != (meta['width'], meta['height']) or message.encoding not in encodings
            or not width*channels <= message.step <= width*channels+4096 or len(message.data) != message.step*height):
        raise ValueError('ROS image differs from the declared raw image source')
    # A native Image has no sequence field. Its original source timestamp is
    # the sample identity; do not invent a subscriber-local frame counter.
    reference = StreamRef(source['stream_id'], source['source_instance_id'], source['source_clock_id'], stamp, stamp)
    raw = bytes(message.data)
    pixels = b''.join(raw[row*message.step:row*message.step+width*channels] for row in range(height))
    if depth and message.is_bigendian:
        little=bytearray(len(pixels));little[0::2]=pixels[1::2];little[1::2]=pixels[0::2];pixels=bytes(little)
    if message.encoding == 'bgr8':
        rgb = bytearray(len(pixels)); rgb[0::3] = pixels[2::3]; rgb[1::3] = pixels[1::3]; rgb[2::3] = pixels[0::3]
        pixels = bytes(rgb)
    metadata = dict(meta, camera=source['stream_id'], capture_instance_id=source['source_instance_id'],
        frame_number=stamp, host_receive_ns=stamp if source['source_clock_id']=='host_wall' else None,
        source_time_ns=stamp, source_clock_id=source['source_clock_id'],
        subscriber_receive_ns=time.time_ns(), encoding='depth16le'if depth else'rgb8', original_encoding=message.encoding,
        ros_reference=dict(reference.__dict__))
    return reference, dict(metadata=metadata, rgb=None if depth else pixels, width=width, height=height,
                          depth=pixels if depth else None), len(pixels)+4096


def state_sample(message, source):
    reference = StreamRef.from_message(message.reference)
    position, velocity, effort = map(list, (message.positions, message.velocities, message.efforts))
    if len(position)!=14 or len(velocity)!=14 or len(effort) not in (0,14):
        raise ValueError('ROS state must contain fourteen declared axes')
    if not all(math.isfinite(x) for x in position+velocity+effort):
        raise ValueError('nonfinite ROS state')
    metadata = json_value(message.metadata_json)
    if not isinstance(metadata, dict) or any(k in metadata for k in ('qpos','qvel','effort')):
        raise ValueError('state metadata cannot override canonical state arrays')
    value = dict(metadata, qpos=position, qvel=velocity, effort=effort,
                 source_time_ns=reference.source_time_ns, source_clock_id=reference.source_clock_id,
                 ros_reference=dict(reference.__dict__))
    return reference, value, len(message.metadata_json.encode())+14*8*3+1024


class RosInputs:
    def __init__(self, ros, descriptors, *, on_sample=None, max_samples=64, max_bytes=64*1024*1024,subscription_depth=4,include_depth=False):
        from sensor_msgs.msg import Image
        from rclpy.qos import ReliabilityPolicy
        self.ros = ros; self.sources = {}; self.caches = {}; self.writers = {}; self.on_sample = on_sample
        self.consumer=ros.node.get_name()
        self.clock_samples={};self.image_arrivals={}
        self.reliability={};self.depth_for={};self.rgb_for={};self.pending_rgbd=set();self.last_pairs={}
        self.raw_caches={};self.metadata_caches={};self.metadata_writers={};self.pending_metadata=set();self.gaps={}
        descriptors=list(descriptors)
        if type(include_depth)is not bool:raise ValueError('explicit RGB-D subscription mode required')
        if include_depth:
            from .ros_data import SourceDirectory
            for rgb in list(descriptors):
                stream=rgb['metadata'].get('depth_stream_id')if rgb['kind']=='image'else None
                if stream is None:continue
                depth=SourceDirectory(ros,rgb['device_id']).resolve({stream:dict(kind='image',
                    source_instance_id=rgb['source_instance_id'],source_clock_id=rgb['source_clock_id'])})[0]
                meta=depth['metadata']
                if (meta.get('pixel_format')!='depth16le' or meta.get('depth_aligned_to')!='color'
                        or type(meta.get('depth_scale_m'))not in(int,float) or not math.isfinite(meta['depth_scale_m'])
                        or meta['depth_scale_m']<=0 or any(meta.get(k)!=rgb['metadata'].get(k)for k in('width','height','serial'))):
                    raise ValueError('aligned depth source differs from its original RGB binding')
                if stream in self.rgb_for:raise ValueError('one original depth stream per RGB source required')
                self.depth_for[rgb['stream_id']]=stream;self.rgb_for[stream]=rgb['stream_id'];descriptors.append(depth)
        if type(subscription_depth)is not int or not 1<=subscription_depth<=256:raise ValueError('bounded ROS subscription depth required')
        for row in descriptors:
            source = source_descriptor(row); stream = source['stream_id']
            if stream in self.sources or source['kind'] not in ('image','state'):
                raise ValueError('unique raw image/state source bindings required')
            expected = 'sensor_msgs/msg/Image' if source['kind']=='image' else 'workbench_interfaces/msg/RobotState'
            if source['message_type'] != expected or not source['available']:
                raise ValueError('source type or readiness differs from its binding')
            deadline=time.monotonic()+2.
            publishers=ros.node.get_publishers_info_by_topic(source['topic'])
            while not publishers and time.monotonic()<deadline:
                time.sleep(.01);publishers=ros.node.get_publishers_info_by_topic(source['topic'])
            if len(publishers)!=1:raise ValueError('one live ROS publisher required before source binding: '+stream)
            offered=publishers[0].qos_profile.reliability
            if offered not in (ReliabilityPolicy.RELIABLE,ReliabilityPolicy.BEST_EFFORT):
                raise ValueError('resolved ROS source reliability required')
            reliable=offered==ReliabilityPolicy.RELIABLE
            if source['kind']=='state'and not reliable:raise ValueError('canonical state source must offer reliable delivery')
            self.reliability[stream]='reliable'if reliable else'best_effort'
            self.writers[stream]=bytes(publishers[0].endpoint_gid)
            declared=source['metadata'].get('publisher_gid')
            if declared is not None and self.writers[stream].hex()!=declared:
                raise ValueError('raw ROS publisher differs from registered source')
            self.sources[stream] = source
            self.caches[stream] = SampleCache(source, max_samples=max_samples, max_bytes=max_bytes)
            clock_topic=source['metadata'].get('timing_topic')
            if source['metadata'].get('timing_boot_id')!=timing.boot_id:clock_topic=None
            if clock_topic is not None:
                if not isinstance(clock_topic,str)or not re.fullmatch(r'/[A-Za-z_][A-Za-z0-9_/]*',clock_topic):
                    raise ValueError('absolute camera timing topic required')
                from std_msgs.msg import String
                self.clock_samples[stream]=OrderedDict();self.image_arrivals[stream]=OrderedDict()
                ros.subscribe(clock_topic,String,lambda msg,s=stream:self._receive_clock(s,msg),depth=32)
            topic=source['metadata'].get('metadata_topic')
            if topic is not None:
                if source['kind']!='image'or source['metadata'].get('pixel_format')=='depth16le'or not isinstance(topic,str)or not re.fullmatch(r'/[A-Za-z_][A-Za-z0-9_/]*',topic):
                    raise ValueError('original RGB metadata requires an explicit ROS topic')
                deadline=time.monotonic()+2.
                writers=ros.node.get_publishers_info_by_topic(topic)
                while not writers and time.monotonic()<deadline:
                    time.sleep(.01);writers=ros.node.get_publishers_info_by_topic(topic)
                if len(writers)!=1 or writers[0].qos_profile.reliability!=ReliabilityPolicy.RELIABLE:
                    raise ValueError('one reliable original metadata publisher required')
                self.metadata_writers[stream]=bytes(writers[0].endpoint_gid)
                self.raw_caches[stream]=SampleCache(source,max_samples=max_samples,max_bytes=max_bytes)
                self.metadata_caches[stream]=SampleCache(source,max_samples=max_samples,max_bytes=max_bytes)
                ros.subscribe(topic,ros.types.DataEvent,lambda msg,s=stream:self._receive_metadata(s,msg),depth=subscription_depth)
            callback = lambda msg,stream=stream:self._receive(stream,msg)
            # Image retention is not a state delivery budget: sixteen samples
            # cover only160ms at100Hz. Keep the existing bounded state history
            # through short executor pauses without enlarging image caches.
            state_depth=max(max_samples,min(MAX_CACHE_SAMPLES,max(1,max_bytes//MAX_METADATA_BYTES)))
            ros.subscribe(source['topic'], Image if source['kind']=='image' else ros.types.RobotState,
                          callback, depth=state_depth if source['kind']=='state'else subscription_depth,reliable=reliable)
            ros.subscribe('sources/'+stream,ros.types.SourceDescriptor,self._source_changed,retained=True,depth=1)

    def _source_changed(self,message):
        value=descriptor_value(message);source=self.sources.get(value['stream_id'])
        if source is None or value['device_id']!=source['device_id']:return
        if any(value[k]!=source[k]for k in('source_instance_id','source_clock_id','topic','message_type','units','metadata'))or not value['available']:
            cache=self.caches[value['stream_id']]
            with cache.changed:
                cache.error='ROS source declaration changed; explicit rebind required';cache.changed.notify_all()

    def _receive(self, stream, message):
        began=time.monotonic_ns()
        source, cache = self.sources[stream], self.caches[stream]
        publishers = self.ros.node.get_publishers_info_by_topic(source['topic'])
        timing.record('input.publisher_lookup.'+stream,began)
        if len(publishers) != 1:
            cache.error = 'raw source has ambiguous ROS publishers'; raise ValueError(cache.error)
        writer = bytes(publishers[0].endpoint_gid)
        previous = self.writers.get(stream)
        if previous is None:
            declared = source['metadata'].get('publisher_gid')
            if declared is not None and writer.hex() != declared:
                cache.error = 'raw ROS publisher differs from registered source'; raise ValueError(cache.error)
            self.writers[stream] = writer
        elif writer != previous:
            cache.error = 'raw ROS publisher restarted; explicit source rebind required'; raise ValueError(cache.error)
        decoded=time.monotonic_ns()
        reference, value, size = (image_sample if source['kind']=='image' else state_sample)(message, source)
        identity=dict(consumer=self.consumer,stream_id=stream,source_instance_id=reference.source_instance_id,
            source_time_ns=reference.source_time_ns,source_sequence=reference.sequence)
        timing.record('input.decode.'+stream,decoded,**identity)
        if source['kind']=='image':
            if source['source_clock_id']=='host_wall':
                age=time.time_ns()-reference.source_time_ns
                timing.record('camera.source_to_decoded.'+stream,time.monotonic_ns()-age,**identity)
            if stream in self.image_arrivals:
                self.image_arrivals[stream][reference.source_time_ns]=began
                while len(self.image_arrivals[stream])>128:self.image_arrivals[stream].popitem(last=False)
                self._join_timing(stream,reference.source_time_ns)
        if stream in self.raw_caches:
            raw=self.raw_caches[stream]
            if reference in raw.samples:value['metadata']['subscriber_receive_ns']=raw.get(reference)['metadata']['subscriber_receive_ns']
            if not raw.put(reference,value,size):return
            self.pending_metadata.add(reference)
            pending=[r for r in self.pending_metadata if r.stream_id==stream]
            if len(pending)>raw.max_samples:
                if cache.samples:
                    self.gaps[stream]=self.gaps.get(stream,0)+1
                self.pending_metadata.remove(min(pending,key=lambda r:r.sequence))
            self._join_metadata(stream,reference)
            return
        # Subscriber arrival time is diagnostic, not part of sample equality.
        if reference in cache.samples:
            old = cache.get(reference)
            if source['kind']=='image':value['metadata']['subscriber_receive_ns']=old['metadata']['subscriber_receive_ns']
        self._accept_sample(stream,reference,value,size)

    def _receive_clock(self,stream,message):
        # Diagnostics never change sample readiness, timestamps, or cache identity.
        try:
            if len(message.data)>8192:raise ValueError('camera timing message too large')
            row=json_value(message.data)
            if row.get('kind')!='frame':return
            stamp=row['stamp_ns'];data=row['data'];received=data['recv_mono_ns'];published=data['publish_done_mono_ns']
            if (type(stamp)is not int or type(received)is not int or type(published)is not int
                    or not 0<received<=published<=time.monotonic_ns()):
                raise ValueError('camera timing clock is not this host monotonic clock')
            self.clock_samples[stream][stamp]=(received,published)
            while len(self.clock_samples[stream])>128:self.clock_samples[stream].popitem(last=False)
            wake=data.get('loop_wake_mono_ns')
            if type(wake)is int and 0<wake<=received:
                timing.record('camera.capture_wait.'+stream,wake,ended_ns=received,stream_id=stream,source_time_ns=stamp)
            timing.record('camera.publisher.'+stream,received,ended_ns=published,stream_id=stream,source_time_ns=stamp)
            self._join_timing(stream,stamp)
        except (ValueError,KeyError,TypeError)as error:
            now=time.monotonic_ns();timing.record('camera.timing_invalid',now,outcome='error',stream_id=stream,reason=str(error)[:256])

    def _join_timing(self,stream,stamp):
        clock=self.clock_samples[stream].get(stamp);arrival=self.image_arrivals[stream].get(stamp)
        if clock is None or arrival is None:return
        # Publish return is not a send timestamp: delivery may overlap publishing.
        timing.record('camera.source_to_callback.'+stream,clock[0],ended_ns=arrival,
            stream_id=stream,source_time_ns=stamp)
        timing.record('camera.publish_return_to_callback.'+stream,min(clock[1],arrival),ended_ns=arrival,
            stream_id=stream,source_time_ns=stamp,publish_return_overlapped=arrival<clock[1],
            signed_gap_ns=arrival-clock[1])
        self.clock_samples[stream].pop(stamp,None);self.image_arrivals[stream].pop(stamp,None)

    def _receive_metadata(self,stream,message):
        source=self.sources[stream];cache=self.caches[stream]
        try:
            writers=self.ros.node.get_publishers_info_by_topic(source['metadata']['metadata_topic'])
            if len(writers)!=1 or bytes(writers[0].endpoint_gid)!=self.metadata_writers[stream]:
                raise ValueError('original frame metadata publisher changed')
            ref=StreamRef.from_message(message.reference)
            if message.event!='frame_metadata' or ref.sequence!=ref.source_time_ns:
                raise ValueError('frame metadata must reference the original image sample')
            value=json_value(message.payload_json)
            if self.sources[stream]['metadata'].get('environment')=='physical':
                expected={'host_receive_ns','frame_number','depth_frame_number','device_timestamp_ms','depth_device_timestamp_ms','timestamp_domain'}
                if (not isinstance(value,dict)or set(value)!=expected or value['host_receive_ns']!=ref.source_time_ns
                        or any(type(value[k])is not int or value[k]<0 for k in ('host_receive_ns','frame_number','depth_frame_number'))
                        or any(type(value[k])not in (int,float)or not math.isfinite(value[k])or value[k]<0 for k in ('device_timestamp_ms','depth_device_timestamp_ms'))
                        or not isinstance(value['timestamp_domain'],str)or not 0<len(value['timestamp_domain'])<=128
                        or abs(value['device_timestamp_ms']-value['depth_device_timestamp_ms'])>1000/self.sources[stream]['nominal_hz']):
                    raise ValueError('invalid original physical frame metadata')
                if self.metadata_caches[stream].put(ref,value,len(message.payload_json.encode())+1024):self._join_metadata(stream,ref)
                return
            expected={'host_receive_ns','physics_step','frame_number','owner_available','owner_pid','command_epoch','runtime_nonce'}
            if (not isinstance(value,dict)or set(value)!=expected or type(value['host_receive_ns'])is not int
                    or value['host_receive_ns']<=0 or type(value['frame_number'])is not int or value['frame_number']<0
                    or type(value['physics_step'])is not int or value['physics_step']<0
                    or type(value['owner_available'])is not bool):raise ValueError('invalid original frame metadata')
            if (value['owner_pid']is not None and (type(value['owner_pid'])is not int or value['owner_pid']<=0)
                    or value['command_epoch']is not None and (type(value['command_epoch'])is not int or value['command_epoch']<0)
                    or value['runtime_nonce']is not None and (not isinstance(value['runtime_nonce'],str)or not 1<=len(value['runtime_nonce'])<=200)):
                raise ValueError('invalid frame sampling owner identity')
            if value['owner_available']and any(value[k]is None for k in ('owner_pid','command_epoch','runtime_nonce')):
                raise ValueError('available frame owner identity is incomplete')
            if self.metadata_caches[stream].put(ref,value,len(message.payload_json.encode())+1024):self._join_metadata(stream,ref)
        except ValueError as error:
            cache.error=str(error);raise

    def _join_metadata(self,stream,ref):
        try:
            value=self.raw_caches[stream].get(ref);meta=self.metadata_caches[stream].get(ref)
        except LookupError:return
        except ValueError as error:
            if 'evicted' in str(error):
                if self.caches[stream].samples:self.gaps[stream]=self.gaps.get(stream,0)+1
                return
            raise
        cache=self.caches[stream]
        if not cache.samples:self.pending_metadata={r for r in self.pending_metadata if r.stream_id!=stream or r.sequence>=ref.sequence}
        self.pending_metadata.discard(ref)
        value=dict(value,metadata=dict(value['metadata'],**meta))
        self._accept_sample(stream,ref,value,len(value['rgb'])+4096)

    def _accept_sample(self,stream,reference,value,size):
        began=time.monotonic_ns()
        cache=self.caches[stream]
        if not cache.put(reference,value,size):return
        timing.record('input.cache_put.'+stream,began,consumer=self.consumer,source_time_ns=reference.source_time_ns,source_sequence=reference.sequence)
        if stream in self.depth_for:
            self.pending_rgbd.add(reference)
            if sum(r.stream_id==stream for r in self.pending_rgbd)>cache.max_samples:
                if stream in self.last_pairs:
                    self.gaps[stream]=self.gaps.get(stream,0)+1
                # Discovery may deliver RGB before the depth subscription is
                # matched. Coverage starts at the first complete pair.
                self.pending_rgbd.remove(min((r for r in self.pending_rgbd if r.stream_id==stream),key=lambda r:r.sequence))
        rgb=self.rgb_for.get(stream,stream)
        if rgb in self.depth_for:
            source=self.sources.get(rgb)
            if source is None:return
            reference=StreamRef(rgb,source['source_instance_id'],source['source_clock_id'],reference.source_time_ns,reference.sequence)
            try:value=self.rgbd(reference)
            except LookupError:return
            previous=self.last_pairs.get(rgb)
            if previous is not None and reference.sequence<=previous.sequence:return
            if previous is None:
                self.pending_rgbd={r for r in self.pending_rgbd if r.stream_id!=rgb or r.sequence>=reference.sequence}
            self.last_pairs[rgb]=reference;self.pending_rgbd.discard(reference)
        if self.on_sample is not None:self.on_sample(reference,value)

    def rgbd(self,reference):
        """Resolve only the original aligned depth sample with the same stamp."""
        value=self.resolve(reference);stream=self.depth_for.get(reference.stream_id)
        if stream is None:return value
        if stream not in self.sources:raise LookupError('aligned depth source is not yet bound')
        source=self.sources[stream]
        ref=StreamRef(stream,source['source_instance_id'],source['source_clock_id'],reference.source_time_ns,reference.sequence)
        depth=self.resolve(ref)
        meta=dict(value['metadata'],depth_reference=dict(ref.__dict__),rgbd_paired=True,pixel_format='rgb8-depth16le',
            depth_scale_m=source['metadata']['depth_scale_m'],depth_aligned_to='color',depth_frame_number=value['metadata'].get('depth_frame_number',value['metadata']['frame_number']),
            depth_encoding='uint16_png',device_timestamp_ms=value['metadata'].get('device_timestamp_ms',reference.source_time_ns/1e6),
            depth_device_timestamp_ms=value['metadata'].get('depth_device_timestamp_ms',ref.source_time_ns/1e6),
            color_timestamp_ms=value['metadata'].get('device_timestamp_ms',reference.source_time_ns/1e6),
            depth_timestamp_ms=value['metadata'].get('depth_device_timestamp_ms',ref.source_time_ns/1e6),
            color_timestamp_domain=value['metadata'].get('timestamp_domain',reference.source_clock_id),
            depth_timestamp_domain=value['metadata'].get('timestamp_domain',ref.source_clock_id))
        return dict(value,depth=depth['depth'],metadata=meta)

    def latest(self, stream):
        history = self.caches[stream].history()
        if not history: raise LookupError('ROS source has not produced a sample: '+stream)
        return history[-1]

    def resolve(self, reference, *, timeout=0.):
        ref = reference if isinstance(reference,StreamRef) else StreamRef.from_message(reference)
        if ref.stream_id not in self.caches: raise ValueError('unbound ROS source reference')
        return self.caches[ref.stream_id].get(ref, timeout=timeout)

    def paired(self, frame_streams, pairer, *, state_stream='state',target_ns=None):
        """Select directly from the original histories, independent of model family."""
        states=self.caches[state_stream].history()
        if not states:raise LookupError('ROS state has not arrived')
        latest,feedback=states[-1];histories={}
        cutoff=latest.source_time_ns-500_000_000
        for role,stream in frame_streams.items():
            samples=[]
            for ref,value in self.caches[stream].history():
                if ref.source_clock_id==latest.source_clock_id:stamp=ref.source_time_ns
                elif ref.source_clock_id=='host_wall' and type(feedback.get('sample_wall_ns'))is int:
                    stamp=latest.source_time_ns+ref.source_time_ns-feedback['sample_wall_ns']
                else:raise ValueError('camera/state clocks lack a declared mapping')
                if stamp>=cutoff:samples.append((stamp,value))
            histories[role]=samples
        selected,state,pairing=pairer.select(histories,[(ref.source_time_ns,value)for ref,value in states if ref.source_time_ns>=cutoff],target_ns=target_ns)
        pairing['clock_id']=latest.source_clock_id
        pairing['target_source_ns']=pairing.pop('target_wall_ns')
        pairing['origin_source_ns']=pairing.pop('origin_wall_ns')
        pairing['decision_clock']='host_wall_seconds; matching stamps in declared source clock'
        refs={role:StreamRef(**value['metadata']['ros_reference'])for role,value in selected.items()}
        state_ref=StreamRef(**state['ros_reference'])
        # Recompute the selection's age using that exact state, not the latest one.
        stamps=[ref.source_time_ns if ref.source_clock_id==state_ref.source_clock_id else
                state_ref.source_time_ns+ref.source_time_ns-state['sample_wall_ns']for ref in refs.values()]
        return refs,state_ref,min(stamps),pairing

    def observation(self, frames, state, *, run_id, epoch, model_io_digest, source_time_ns, pairing):
        """Publishable selection metadata, with no image/state payload copies."""
        frame_refs = {role:ref if isinstance(ref,StreamRef) else StreamRef.from_message(ref) for role,ref in frames.items()}
        state_ref = state if isinstance(state,StreamRef) else StreamRef.from_message(state)
        if len(frame_refs)!=3 or type(epoch)is not int or epoch<0:
            raise ValueError('three camera selections and an execution epoch required')
        self.resolve(state_ref)
        for ref in frame_refs.values():self.resolve(ref)
        identity = hashlib.sha256(json_text(dict(state=state_ref.__dict__,
            frames={k:v.__dict__ for k,v in sorted(frame_refs.items())},io=model_io_digest)).encode()).hexdigest()
        roles=sorted(frame_refs)
        return self.ros.types.Observation(observation_id=identity,run_id=run_id,epoch=epoch,
            state=state_ref.message(),frames=[frame_refs[role].message() for role in roles],camera_roles=roles,
            source_time_ns=source_time_ns,model_io_digest=model_io_digest,pairing_json=json_text(pairing))

    def resolve_observation(self, observation, *, timeout=0.):
        if len(observation.frames)!=len(observation.camera_roles) or len(set(observation.camera_roles))!=3:
            raise ValueError('observation camera role mapping is incomplete')
        deadline=time.monotonic()+timeout
        state=deepcopy(self.resolve(observation.state,timeout=max(0.,deadline-time.monotonic())))
        images={}
        for role,ref in zip(observation.camera_roles,observation.frames):
            images[role]=self.resolve(ref,timeout=max(0.,deadline-time.monotonic()))
        return state,images


class RosCameraView:
    """Derived preview only; original frames stay on their registered ROS topics."""
    def __init__(self,plane,device_id,intrinsics,environment):
        from .ros_data import RosNode,SourceDirectory,plane_config
        from .rgbd_service import _Preview
        if environment not in('fake','physical')or set(intrinsics)!={'base_0','left_wrist_0','right_wrist_0'}:
            raise ValueError('three declared camera intrinsics and environment required')
        self.plane=plane_config(plane);self.device_id=device_id;self.intrinsics=deepcopy(intrinsics)
        self.environment=environment;self.generation=0;self.process=None;self.descriptor_path=None
        self.descriptor=dict(schema='workbench.ros-camera-view.v1',instance_id=secrets.token_hex(16),
            environment=environment,intrinsics=self.intrinsics,data_plane=self.plane)
        self.frames={name:_Preview()for name in intrinsics};self.channels={};self.servers={}
        self.inputs=None;self.sources={};self.error=None;self.lock=threading.RLock()
        self.closed=threading.Event();self.rebind=threading.Event();self.reconnecting=False
        self.ros=RosNode('camera_view_'+self.descriptor['instance_id'][:12],namespace=self.plane['namespace'],
            domain_id=self.plane['domain_id'],localhost_only=False)
        self.directory=SourceDirectory(self.ros,device_id)
        self.thread=threading.Thread(target=self._monitor,name='ros-camera-view',daemon=True);self.thread.start()

    def _unavailable(self,reason):
        self.error=str(reason)
        for frame in self.frames.values():
            with frame.lock:frame.pixels=None;frame.cached=None
            frame.frames.publish(status=dict(ready=False,error=self.error))

    def _bind(self):
        rows=self.directory.resolve({name:dict(kind='image')for name in self.intrinsics},timeout=.1)
        for row in rows:
            expected=self.intrinsics[row['stream_id']];metadata=row['metadata']
            if any(metadata.get(key)!=expected[key]for key in('serial','width','height'))or metadata.get('environment')!=self.environment:
                raise ValueError('ROS preview source differs from registered camera intrinsics')
        self.sources={row['stream_id']:row for row in rows}
        self.inputs=RosInputs(self.ros,rows,on_sample=self._receive,max_samples=4,subscription_depth=1,include_depth=True)
        self.sources=dict(self.inputs.sources)
        self.directory.resolve({k:dict(kind='image')for k in self.sources})
        self.reconnecting=False;self.error=None

    def _receive(self,reference,value):
        metadata=dict(value['metadata'],ready=True,raw=True,annotated=False,
            pixel_format='rgb8-depth16le'if value.get('depth')is not None else'rgb8',rgbd_paired=value.get('depth')is not None,
            pair_id=reference.source_instance_id+':'+str(reference.source_time_ns),
            intrinsics=self.intrinsics[reference.stream_id],capture_fps=self.sources[reference.stream_id]['nominal_hz'])
        # Receive time controls display freshness only; it never changes the
        # original image header, source clock or selection reference.
        metadata['preview_receive_ns']=metadata['subscriber_receive_ns']
        self.frames[reference.stream_id].publish(dict(value,metadata=metadata))

    def _monitor(self):
        while not self.closed.is_set():
            try:
                if self.rebind.is_set():
                    from .ros_data import RosNode,SourceDirectory
                    self.ros.close()
                    self.ros=RosNode('camera_view_'+self.descriptor['instance_id'][:12],namespace=self.plane['namespace'],
                        domain_id=self.plane['domain_id'],localhost_only=False)
                    self.directory=SourceDirectory(self.ros,self.device_id);self.inputs=None;self.sources={}
                    self.rebind.clear()
                if self.inputs is None:self._bind()
                else:
                    for name,cache in self.inputs.caches.items():
                        if cache.error:raise ValueError(cache.error)
                        if self.directory.values.get(name)!=self.sources[name]:raise ValueError('ROS preview source changed; explicit rebind required')
            except (ValueError,LookupError,TimeoutError)as error:self._unavailable(error)
            self.closed.wait(.05)

    def start_servers(self,ports=None):
        from http.server import ThreadingHTTPServer
        from real_rgbd import make_handler
        for role,name in {'head':'base_0','left':'left_wrist_0','right':'right_wrist_0'}.items():
            if role not in self.servers:
                server=ThreadingHTTPServer(('127.0.0.1',(ports or{}).get(role,0)),make_handler(self.frames[name]))
                server.daemon_threads=True;self.servers[role]=server
                threading.Thread(target=server.serve_forever,daemon=True,name='ros-preview-'+role).start()
        return {role:server.server_port for role,server in self.servers.items()}

    def latest_rgbd(self,camera):
        if camera not in self.frames:raise ValueError('unregistered ROS camera')
        frame=self.frames[camera]
        with frame.lock:
            if frame.pixels is None:raise LookupError('ROS camera has no complete frame')
            value=deepcopy(frame.pixels)
        status=frame.snapshot(formats=())['status']
        if not status.get('ready'):raise ValueError(status.get('error')or'ROS camera unavailable')
        return dict(value,source_age_s=status.get('source_age_s'))

    def snapshot(self):
        with self.lock:
            return dict(instance_id=self.descriptor['instance_id'],environment=self.environment,
                capture_instances={name:row['source_instance_id']for name,row in self.sources.items()if name in self.intrinsics},
                recording_channels={},source_clock_id=next(iter(self.sources.values()))['source_clock_id']if self.sources else None,
                observed_intrinsics=deepcopy(self.intrinsics),error=self.error)

    def reconnect_state(self):
        return dict(self.snapshot(),supported=True,generation=self.generation,reconnecting=self.reconnecting,
            active_calls=0,recording_leases={})

    def reconnect(self,*,max_age_s=2.,allow_current=False):
        if type(max_age_s)not in(int,float)or not 0<max_age_s<=30 or type(allow_current)is not bool:
            raise ValueError('bounded camera age and explicit rebind mode required')
        if self.reconnecting:raise ValueError('preview source rebind already in progress')
        self.reconnecting=True;self.generation+=1;self.rebind.set()
        deadline=time.monotonic()+5
        while time.monotonic()<deadline:
            if not self.reconnecting and self.inputs is not None:
                try:
                    for frame in self.frames.values():
                        state=frame.snapshot(formats=())['status']
                        age=state.get('source_age_s')
                        if age is None:age=state.get('preview_age_s')
                        if not state.get('ready')or age is None or not 0<=age<=max_age_s:break
                    else:return dict(reconnected=True,**self.reconnect_state())
                except ValueError:pass
            time.sleep(.02)
        raise ValueError(self.error or 'ROS preview source rebind timed out')

    def close(self):
        if self.closed.is_set():return
        self.closed.set();self.thread.join(2)
        self.ros.close()
        for frame in self.frames.values():
            with frame.condition:frame.frames.closed=True;frame.condition.notify_all()
        for server in self.servers.values():server.shutdown();server.server_close()


class RosStateView:
    """Read-only compatibility projection of the exact referenced state sample."""
    def __init__(self,plane,device_id,owner_instance,environment):
        from .ros_data import RosNode,SourceDirectory,plane_config
        self.plane=plane_config(plane);self.owner_instance=owner_instance;self.environment=environment
        self.ros=RosNode('state_view_'+secrets.token_hex(6),namespace=self.plane['namespace'],
            domain_id=self.plane['domain_id'],localhost_only=False)
        self.directory=SourceDirectory(self.ros,device_id);self.inputs=None

    def resolve(self,reference):
        if not isinstance(reference,dict):raise LookupError('Owner has no published ROS state reference')
        ref=StreamRef(**reference)
        if ref.source_instance_id!=self.owner_instance:raise ValueError('ROS state belongs to another Owner')
        if self.inputs is None:
            sources=self.directory.resolve({'state':dict(source_instance_id=self.owner_instance,kind='state')},timeout=.1)
            self.inputs=RosInputs(self.ros,sources)
        value=deepcopy(self.inputs.resolve(ref,timeout=.1))
        # Legacy UI/calibration readers use their original clock vocabulary;
        # ros_reference retains the exact transport clock and sample identity.
        value['source_clock_id']=None if self.environment=='physical'else ref.source_clock_id
        value['clock_domain']='host_wall'if self.environment=='physical'else'simulation'
        return value

    def close(self):self.ros.close()
