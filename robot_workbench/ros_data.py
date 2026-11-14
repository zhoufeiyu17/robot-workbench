"""ROS data-plane ownership and bounded local caches. No SDK or file transport.

Image topics carry the original sensor_msgs/Image once. All selections use
source-bound references. A missing ROS runtime is an error, never a fallback.
"""
from collections import OrderedDict, deque
from copy import deepcopy
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import sys
import re
import threading
import time


MAX_METADATA_BYTES = 512 * 1024
MAX_CACHE_BYTES = 64 * 1024 * 1024
MAX_CACHE_SAMPLES = 64
RECORDING_HEALTH_FIELDS=frozenset(('state','prepared','ready','degraded','missing_streams','available_streams',
    'streams','source_errors','data_queue','heartbeat_monotonic_s','error','recording_activity'))
_NAME = re.compile(r'[A-Za-z0-9_][A-Za-z0-9_.:-]{0,199}')
_TOPIC = re.compile(r'/[A-Za-z_][A-Za-z0-9_/]*')
_MESSAGE_NAMES=('StreamRef','SourceDescriptor','RobotState','Observation','ModelRequest','ModelInput','ModelResult','DataEvent')
# Exact field names, types and bounds of ros/workbench_interfaces/msg/*.msg.
_MESSAGE_LAYOUT_SHA256='39c50d57a46f51feb1f2e0956015f3e1b80b3240ba8e628d205091c0a613bcd9'


def image_transport_environment():
    """Apply the packaged image transport default to a newly launched publisher."""
    environment=dict(os.environ)
    if not any(environment.get(key)for key in ('FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_DEFAULT_PROFILES_FILE')):
        profile=Path(__file__).resolve().parents[1]/'configs/dds-image-transport.xml'
        if not profile.is_file():raise ValueError('packaged DDS image transport profile is missing')
        environment['FASTRTPS_DEFAULT_PROFILES_FILE']=str(profile)
    return environment


def require_runtime():
    """Check the CPU ROS dependency before any device backend is constructed."""
    try:
        import rclpy
        from workbench_interfaces import msg
        layout={name:getattr(msg,name).get_fields_and_field_types()for name in _MESSAGE_NAMES}
        actual=hashlib.sha256(json.dumps(layout,sort_keys=True,separators=(',',':')).encode()).hexdigest()
        if actual!=_MESSAGE_LAYOUT_SHA256:raise ValueError('workbench_interfaces message layout version mismatch')
    except (ImportError,OSError,AttributeError,ValueError) as error:
        raise RuntimeError('ROS2 runtime and exact workbench_interfaces message layout are required; no legacy fallback')from error


def plane_config(value):
    if (not isinstance(value,dict) or set(value)!={'schema','namespace','domain_id'}
            or value['schema']!='workbench.ros-data-plane.v1'
            or not isinstance(value['namespace'],str) or not _TOPIC.fullmatch(value['namespace'])
            or value['namespace'].endswith('/') or '//'in value['namespace']
            or type(value['domain_id'])is not int or not 0<=value['domain_id']<=232):
        raise ValueError('explicit ROS data-plane schema, namespace and domain required')
    return dict(value)


def bounded_value(value, remaining=MAX_METADATA_BYTES-256, depth=0):
    """Detach bounded JSON values without serializing them on the producer."""
    if depth > 16:raise ValueError('capture payload nesting exceeds bound')
    if value is None or type(value) is bool:result,size=value,max(5,sys.getsizeof(value))
    elif type(value) is int:
        if value.bit_length()>64:raise ValueError('capture integer exceeds 64 bits')
        result,size=value,max(22,sys.getsizeof(value))
    elif type(value) is float:
        if not math.isfinite(value):raise ValueError('capture numbers must be finite')
        result,size=value,max(32,sys.getsizeof(value))
    elif isinstance(value,str):result,size=value,max(6*len(value)+2,sys.getsizeof(value))
    elif type(value) in (list,tuple,dict):
        size=2;result={} if type(value) is dict else []
        items=value.items() if type(value) is dict else enumerate(value)
        for key,item in items:
            if type(value) is dict:
                if not isinstance(key,str) or not 0<len(key)<=128:raise ValueError('bounded capture field name required')
                size+=max(6*len(key)+3,sys.getsizeof(key))
            copied,cost=bounded_value(item,remaining-size,depth+1);size+=cost+1
            if size>remaining:raise ValueError('capture payload exceeds message bound; use a payload reference')
            if type(value) is dict:result[key]=copied
            else:result.append(copied)
        if type(value) is not dict:result=tuple(result)
        size+=sys.getsizeof(result)
    else:raise ValueError('capture payload requires JSON values or an artifact reference; received '+type(value).__name__)
    if size>remaining:raise ValueError('capture payload exceeds message bound; use a payload reference')
    return result,size


class BoundedQueue(queue.Queue):
    def __init__(self,max_bytes):
        super().__init__();self.max_bytes=max_bytes;self.queued_bytes=0;self.peak_bytes=0

    def _put(self,item):
        size,value=item
        if self.queued_bytes+size>self.max_bytes:raise queue.Full
        self.queued_bytes+=size;self.peak_bytes=max(self.peak_bytes,self.queued_bytes)
        super()._put((size,value,time.monotonic()))

    def _get(self):
        size,value,_=super()._get();self.queued_bytes-=size
        return value

    def snapshot(self):
        with self.mutex:return dict(queued_bytes=self.queued_bytes,max_bytes=self.max_bytes,
                                   peak_bytes=self.peak_bytes,depth=len(self.queue),
                                   oldest_age_s=max(0.,time.monotonic()-self.queue[0][2])if self.queue else 0.)



def json_text(value):
    raw = json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(',', ':'))
    if len(raw.encode()) > MAX_METADATA_BYTES:
        raise ValueError('ROS metadata exceeds the existing 512 KiB message budget')
    return raw


def json_value(raw):
    if not isinstance(raw, str) or len(raw.encode()) > MAX_METADATA_BYTES:
        raise ValueError('bounded ROS metadata required')
    return json.loads(raw, parse_constant=lambda value: (_ for _ in ()).throw(ValueError('nonfinite ROS metadata')))


@dataclass(frozen=True)
class StreamRef:
    stream_id: str
    source_instance_id: str
    source_clock_id: str
    source_time_ns: int
    sequence: int

    def __post_init__(self):
        if any(not isinstance(v, str) or not _NAME.fullmatch(v) for v in
               (self.stream_id, self.source_instance_id, self.source_clock_id)):
            raise ValueError('explicit stream, source instance and source clock required')
        if (type(self.source_time_ns) is not int or not 0 <= self.source_time_ns < 2**63
                or type(self.sequence) is not int or not 0 <= self.sequence < 2**64):
            raise ValueError('bounded original sample time and sequence required')

    @classmethod
    def from_message(cls, value):
        return cls(*(getattr(value, key) for key in cls.__dataclass_fields__))

    def message(self):
        from workbench_interfaces.msg import StreamRef as Message
        return Message(**self.__dict__)


def source_descriptor(value):
    fields = {'device_id', 'stream_id', 'source_instance_id', 'source_clock_id', 'topic',
              'message_type', 'kind', 'units', 'coordinate_frame', 'nominal_hz', 'available', 'metadata'}
    if not isinstance(value, dict) or set(value) != fields:
        raise ValueError('complete explicit ROS source descriptor required')
    for key in ('device_id', 'stream_id', 'source_instance_id', 'source_clock_id'):
        if not isinstance(value[key], str) or not _NAME.fullmatch(value[key]):
            raise ValueError('invalid ROS source identity: ' + key)
    if not isinstance(value['topic'], str) or not _TOPIC.fullmatch(value['topic']) or '//' in value['topic']:
        raise ValueError('absolute resolved ROS topic required')
    if value['kind'] not in ('image', 'state', 'observation', 'model_request', 'model_input', 'model_result', 'event'):
        raise ValueError('unknown source kind')
    if type(value['available']) is not bool or not isinstance(value['coordinate_frame'], str):
        raise ValueError('explicit source validity and coordinate frame required')
    hz = value['nominal_hz']
    if type(hz) not in (int, float) or not math.isfinite(hz) or hz < 0:
        raise ValueError('finite nonnegative source rate required')
    if not isinstance(value['message_type'], str) or not re.fullmatch(r'[a-z][a-z0-9_]*/msg/[A-Za-z][A-Za-z0-9_]*', value['message_type']):
        raise ValueError('explicit ROS message type required')
    json_text(value['units']); json_text(value['metadata'])
    return deepcopy(value)


def descriptor_message(ros, value):
    value=source_descriptor(value)
    value['nominal_hz']=float(value['nominal_hz'])
    return ros.types.SourceDescriptor(**{k:v for k,v in value.items()if k not in('units','metadata')},
        units_json=json_text(value['units']),metadata_json=json_text(value['metadata']))


def descriptor_value(message):
    return source_descriptor(dict(device_id=message.device_id,stream_id=message.stream_id,
        source_instance_id=message.source_instance_id,source_clock_id=message.source_clock_id,
        topic=message.topic,message_type=message.message_type,kind=message.kind,
        units=json_value(message.units_json),coordinate_frame=message.coordinate_frame,
        nominal_hz=message.nominal_hz,available=message.available,metadata=json_value(message.metadata_json)))


class SourceDirectory:
    """ROS source declarations; lookup never changes an existing consumer binding."""
    def __init__(self,ros,device_id):
        self.ros,self.device_id=ros,device_id;self.values={};self.changed=threading.Condition();self.subscribed=set()

    def _receive(self,message):
        value=descriptor_value(message)
        if value['device_id']!=self.device_id:return
        with self.changed:self.values[value['stream_id']]=value;self.changed.notify_all()

    def resolve(self, expected, timeout=5.):
        for stream in expected:
            if stream not in self.subscribed:
                self.ros.subscribe('sources/'+stream,self.ros.types.SourceDescriptor,self._receive,retained=True,depth=1)
                self.subscribed.add(stream)
        deadline=time.monotonic()+timeout
        with self.changed:
            while True:
                values=[]
                for stream,fields in expected.items():
                    value=self.values.get(stream)
                    if value is None or not value['available']:break
                    if any(value.get(k)!=v for k,v in fields.items()):
                        raise ValueError('ROS source declaration differs: '+stream)
                    values.append(value)
                else:return deepcopy(values)
                remaining=deadline-time.monotonic()
                if remaining<=0:raise TimeoutError('required ROS sources are not declared and available')
                self.changed.wait(remaining)


class SampleCache:
    """One source-bound history; never substitute another publisher or latest sample."""
    def __init__(self, descriptor, *, max_samples=MAX_CACHE_SAMPLES, max_bytes=MAX_CACHE_BYTES):
        self.descriptor = source_descriptor(descriptor)
        if type(max_samples) is not int or max_samples < 1 or type(max_bytes) is not int or max_bytes < 1:
            raise ValueError('positive bounded sample cache required')
        self.max_samples, self.max_bytes = max_samples, max_bytes
        self.samples = OrderedDict(); self.bytes = 0; self.error = None
        self.lock = threading.RLock(); self.changed = threading.Condition(self.lock)

    def put(self, reference, value, size):
        with self.changed:
            if self.error: raise ValueError(self.error)
            if any(getattr(reference, key) != self.descriptor[key] for key in
                   ('stream_id', 'source_instance_id', 'source_clock_id')):
                self.error = 'ROS sample source identity changed; explicit rebind required'
                self.changed.notify_all(); raise ValueError(self.error)
            if type(size) is not int or not 0 < size <= self.max_bytes:
                raise ValueError('sample exceeds its bounded cache')
            if reference in self.samples:
                if self.samples[reference][0] != value:
                    self.error = 'same ROS sample identity carried different data'
                    self.changed.notify_all(); raise ValueError(self.error)
                return False
            if self.samples:
                previous = next(reversed(self.samples))
                if reference.sequence <= previous.sequence or reference.source_time_ns <= previous.source_time_ns:
                    self.error = 'ROS source sequence or timestamp regressed'
                    self.changed.notify_all(); raise ValueError(self.error)
            self.samples[reference] = (value, size); self.bytes += size
            while len(self.samples) > self.max_samples or self.bytes > self.max_bytes:
                _, (_, cost) = self.samples.popitem(last=False); self.bytes -= cost
            self.changed.notify_all(); return True

    def get(self, reference, timeout=0.):
        deadline = time.monotonic() + timeout
        with self.changed:
            while True:
                if self.error: raise ValueError(self.error)
                if reference in self.samples: return self.samples[reference][0]
                if self.samples and reference.sequence < next(iter(self.samples)).sequence:
                    raise ValueError('referenced ROS sample was evicted; no latest-frame substitution')
                remaining = deadline - time.monotonic()
                if remaining <= 0: raise LookupError('referenced ROS sample is not available')
                self.changed.wait(remaining)

    def history(self):
        with self.lock:
            if self.error: raise ValueError(self.error)
            return tuple((ref, value) for ref, (value, _) in self.samples.items())


class RosNode:
    """Owned ROS context/executor. No daemon, device acquisition or ROS fallback."""
    _contexts = {}
    _context_lock = threading.RLock()

    def __init__(self, name, *, namespace, domain_id, localhost_only=True):
        require_runtime()
        if (not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', name)
                or not _TOPIC.fullmatch(namespace) or type(domain_id) is not int or not 0 <= domain_id <= 232):
            raise ValueError('explicit node, namespace and ROS domain required')
        if localhost_only and os.environ.get('ROS_LOCALHOST_ONLY') != '1':
            raise ValueError('ROS_LOCALHOST_ONLY=1 required before creating the isolated ROS context')
        try:
            import rclpy
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from workbench_interfaces import msg
        except ImportError as error:
            raise RuntimeError('ROS2 runtime and built workbench_interfaces are required; no legacy fallback') from error
        self.types = msg; self.closed = False; self.context_key = (os.getpid(), domain_id)
        # ROS components in one process share its participant/context. Creating
        # concurrent Humble participants for each logical component is unsafe.
        with self._context_lock:
            entry = self._contexts.get(self.context_key)
            if entry is None:
                context = Context(); rclpy.init(context=context, domain_id=domain_id)
                entry = self._contexts[self.context_key] = [context, 0]
            self.context = entry[0]; entry[1] += 1
        self.node = rclpy.create_node(name, namespace=namespace, context=self.context,
                                     enable_rosout=False, start_parameter_services=False)
        class TimedExecutor(SingleThreadedExecutor):
            # The pinned Humble executor discards take_message()[1]. Preserve it
            # for our diagnostic wrappers while retaining direct subscriptions.
            def _take_subscription(self, subscription):
                if not getattr(subscription.callback, '_workbench_timing', False):
                    return super()._take_subscription(subscription)
                with subscription.handle:
                    return subscription.handle.take_message(subscription.msg_type, subscription.raw)

            async def _execute_subscription(self, subscription, taken):
                if not getattr(subscription.callback, '_workbench_timing', False):
                    return await super()._execute_subscription(subscription, taken)
                if taken is not None:
                    subscription.callback(taken[0], taken[1])

        self.executor = TimedExecutor(context=self.context); self.executor.add_node(self.node)
        self.errors = deque(maxlen=16); self.publishers = {}; self.subscriptions = []
        self.thread = threading.Thread(target=self.executor.spin, name=name+'-ros', daemon=True)
        self.thread.start()

    @staticmethod
    def qos(*, retained=False, depth=32, reliable=True):
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
        return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=depth,
                          reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
                          durability=DurabilityPolicy.TRANSIENT_LOCAL if retained else DurabilityPolicy.VOLATILE)

    def publisher(self, topic, message_type, *, retained=False, depth=32):
        key = (topic, message_type)
        if key not in self.publishers:
            self.publishers[key] = self.node.create_publisher(message_type, topic, self.qos(retained=retained, depth=depth))
        return self.publishers[key]

    def subscribe(self, topic, message_type, callback, *, retained=False, depth=32,reliable=True):
        from .pipeline_timing import timing
        from .runtime_monitor import probes
        consumer=self.node.get_name()
        def receive(message, info=None):
            began=time.monotonic_ns();wall=time.time_ns();outcome='ok'
            ref=getattr(message,'reference',None)
            header=getattr(message,'header',None)
            source_stamp=(int(header.stamp.sec)*10**9+int(header.stamp.nanosec))if header is not None else getattr(ref,'source_time_ns',None)
            identity=dict(consumer=consumer,topic=topic,
                request_id=getattr(message,'request_id',None),observation_id=getattr(message,'observation_id',None),
                stream_id=getattr(ref,'stream_id',None),source_time_ns=source_stamp,
                source_sequence=getattr(ref,'sequence',None),
                source_instance_id=getattr(ref,'source_instance_id',None))
            received=info.get('received_timestamp',0)if isinstance(info,dict)else getattr(info,'received_timestamp',0)
            probe_channel = 'callback/' + consumer
            probes.mark(probe_channel, now_ns=began, active=True, stage=message_type.__name__,
                        topic=topic, source_ns=source_stamp, source_sequence=identity['source_sequence'],
                        rmw_received_wall_ns=received or None, callback_wall_ns=wall)
            if type(received)is int and received>0 and 0<=wall-received<=60_000_000_000:
                timing.record('ros.middleware_to_callback.'+message_type.__name__,began-(wall-received),
                    ended_ns=began,**identity)
            try: callback(message)
            except Exception as error:
                outcome='error';self.errors.append(str(error))
            finally:
                ended = time.monotonic_ns()
                probes.mark(probe_channel, now_ns=ended, force=ended-began>=25_000_000,
                            active=False, stage=message_type.__name__, topic=topic,
                            duration_ns=ended-began, source_ns=source_stamp,
                            source_sequence=identity['source_sequence'], outcome=outcome)
                if ended-began >= 100_000_000:
                    probes.fault('callback_slow', consumer=consumer, topic=topic,
                                 callback_type=message_type.__name__, duration_ns=ended-began)
                timing.record('ros.callback.'+message_type.__name__,began,outcome=outcome,
                    rmw_received_wall_ns=received or None,**identity)
        receive._workbench_timing=True
        sub = self.node.create_subscription(message_type, topic, receive, self.qos(retained=retained, depth=depth,reliable=reliable))
        self.subscriptions.append(sub); return sub

    def close(self):
        if self.closed: return
        self.closed = True; self.executor.shutdown(timeout_sec=5.)
        self.thread.join(5.)
        if self.thread.is_alive(): raise RuntimeError('ROS executor did not exit')
        self.node.destroy_node()
        # Destroying DDS entities does not clear their Python callbacks. Break
        # subscriber -> callback -> RosNode cycles while Owner GC is disabled.
        for subscription in self.subscriptions:subscription.callback=lambda message:None
        self.subscriptions.clear();self.publishers.clear()
        with self._context_lock:
            entry=self._contexts[self.context_key];entry[1]-=1
            if not entry[1]:
                self.context.shutdown();del self._contexts[self.context_key]


class RecordingHealth:
    """Pinned original health source shared by recorder clients and Owner/UI."""
    def __init__(self,description,*,capture_session_id,device_id,writer_identity,plane,on_value=None,on_error=None,run_id=None,ros=None):
        if (not isinstance(description,dict)or set(description)!={'kind','source','namespace','domain_id'}
                or description['kind']!='ros'or any(description[key]!=plane[key]for key in ('namespace','domain_id'))):
            raise ValueError('registered ROS recording health source required')
        source=source_descriptor(description['source']);meta=source['metadata']
        if (source['kind']!='event'or source['message_type']!='workbench_interfaces/msg/DataEvent'
                or source['device_id']!=device_id or source['source_clock_id']!='host_monotonic'
                or source['units']!='capture_health'or meta.get('capture_session_id')!=capture_session_id):
            raise ValueError('recording health belongs to another source/Run')
        writer=meta.get('writer_identity')or{}
        if set(writer)!={'pid','start_ticks','boot_id'}or any(writer_identity.get(k)!=v for k,v in writer.items()):
            raise ValueError('recording health publisher differs from the actual writer')
        instance=':'.join(str(writer[k])for k in ('pid','start_ticks','boot_id'))
        if source['source_instance_id']!=instance:raise ValueError('recording health source birth identity differs')
        gid=meta.get('publisher_gid')
        if not isinstance(gid,str)or not re.fullmatch('[a-f0-9]{32,64}',gid):raise ValueError('recording health publisher GID required')
        self.description=deepcopy(description);self.source=source;self.run_id=run_id
        self.on_value,self.on_error=on_value,on_error
        self.lock=threading.RLock();self.reference=None;self.value={};self.error=None;self.closed=False
        self.owned_node=ros is None
        self.ros=ros or RosNode('recording_health_'+str(os.getpid())+'_'+str(id(self)),
            namespace=description['namespace'],domain_id=description['domain_id'],localhost_only=False)
        try:
            deadline=time.monotonic()+2.
            while True:
                publishers=self.ros.node.get_publishers_info_by_topic(source['topic'])
                if publishers or time.monotonic()>=deadline:break
                time.sleep(.01)
            if len(publishers)!=1 or bytes(publishers[0].endpoint_gid).hex()!=gid:
                raise ValueError('one original recording health publisher required before binding')
        except BaseException:
            if self.owned_node:self.ros.close()
            raise
        def receive(message):
            try:
                with self.lock:
                    if self.closed:return
                    publishers=self.ros.node.get_publishers_info_by_topic(source['topic'])
                # Discovery can remove a normally sealed writer before its last
                # retained sample is dispatched. Do not accept an unverified
                # sample or invent a failure; actual exit/seal proof is RPC.
                if not publishers:return
                if len(publishers)!=1 or bytes(publishers[0].endpoint_gid).hex()!=gid:
                    raise ValueError('original recording health publisher changed or disappeared')
                self._receive(message,bytes(publishers[0].endpoint_gid))
            except Exception as error:
                with self.lock:
                    if self.closed:return
                    first=self.error is None;self.error=self.error or str(error);callback=self.on_error
                if first and callback is not None:callback(str(error))
        self.subscription=self.ros.node.create_subscription(self.ros.types.DataEvent,source['topic'],receive,self.ros.qos(retained=True,depth=1))
        self.ros.subscriptions.append(self.subscription)

    def bind_run(self,run_id):
        with self.lock:
            if self.run_id not in(None,run_id):raise ValueError('recording health Run binding changed')
            self.run_id=run_id

    def _receive(self,message,gid):
        source=self.source;ref=StreamRef.from_message(message.reference)
        if (message.event!='recording_health'or gid.hex()!=source['metadata']['publisher_gid']
                or any(getattr(ref,key)!=source[key]for key in ('stream_id','source_instance_id','source_clock_id'))):
            raise ValueError('original recording health publisher/reference changed')
        value=json_value(message.payload_json)
        if (not isinstance(value,dict)or not {'state','prepared','ready','streams','heartbeat_monotonic_s'}<=set(value)
                or set(value)-RECORDING_HEALTH_FIELDS or any(type(value[k])is not bool for k in ('prepared','ready'))
                or value['state']not in('preparing','prepared','bound','recording','sealing','complete','incomplete','failed','recovering')
                or type(value['heartbeat_monotonic_s'])not in(int,float)or not math.isfinite(value['heartbeat_monotonic_s'])
                or round(value['heartbeat_monotonic_s']*1e9)!=ref.source_time_ns):raise ValueError('invalid recording health values')
        for key in ('streams','source_errors','data_queue'):
            if key in value and not isinstance(value[key],dict):raise ValueError('invalid recording health '+key)
        for key in ('missing_streams','available_streams'):
            if key in value and (not isinstance(value[key],list)or len(value[key])>64
                    or any(not isinstance(v,str)or not _NAME.fullmatch(v)for v in value[key])):
                raise ValueError('invalid recording health '+key)
        if any(not isinstance(v,str)for v in value.get('source_errors',{}).values()):raise ValueError('invalid recording source errors')
        if value.get('error')is not None and not isinstance(value['error'],str):raise ValueError('invalid recording health error')
        if 'recording_activity'in value:
            from .recording_activity import activity_summary
            activity_summary(value['recording_activity'])
        with self.lock:
            if self.closed or self.error:return
            if self.run_id is not None and message.run_id not in('',self.run_id):raise ValueError('recording health belongs to another Run')
            old=self.reference
            if old is not None:
                if ref.sequence<old.sequence:return
                if ref.sequence==old.sequence:
                    if value!=self.value:raise ValueError('same recording health reference carried different values')
                    return
                if ref.source_time_ns<old.source_time_ns:raise ValueError('recording health source time regressed')
            self.reference=ref;self.value=deepcopy(value);callback=self.on_value
        if callback is not None:callback(ref,value)

    def snapshot(self):
        with self.lock:
            value=deepcopy(self.value)
            if self.error:value.update(error=self.error,ready=False,degraded=True)
            return value

    def close(self):
        with self.lock:
            if self.closed:return
            self.closed=True
        node=self.ros;self.ros=None;self.on_value=self.on_error=None
        if self.owned_node:node.close()
        elif not node.closed:
            node.node.destroy_subscription(self.subscription)
            if self.subscription in node.subscriptions:node.subscriptions.remove(self.subscription)
        self.subscription=None
