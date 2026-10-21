"""One CPU camera backend for physical and fake raw ingress, over bounded Unix IPC."""
from copy import deepcopy
import errno
import hmac
import io
import json
import os
from pathlib import Path
import secrets
import socket
import socketserver
import struct
import subprocess
import sys
import threading
import time

from robot_workbench.rgbd_channel import MemoryRgbdBudget, MemoryRgbdChannel
from robot_workbench.sample_clock import clock_spec, sample_stamp, assert_progress

CAMERAS={'head':'base_0','left':'left_wrist_0','right':'right_wrist_0'}
MAX_HEADER=64*1024
MAX_BODY=32*1024**2


def start_capture_sources(profile, service, output):
    """Reuse the physical raw ingress; never construct an arm controller."""
    root=Path(__file__).resolve().parents[1];output=Path(output)
    descriptor=Path(service.directories['base_0']).parent/'camera-service.json'
    env=dict(os.environ,PYTHONPATH=os.pathsep.join([str(root),str(root/'rule_based_workbench'),
        str(root/'calibration/infra'),os.environ.get('PYTHONPATH','')]))
    processes=[]
    try:
        for role,row in profile['cameras'].items():
            with (output/('capture-'+role+'.log')).open('xb') as log:
                processes.append(subprocess.Popen([sys.executable,'-B',str(root/'rule_based_workbench/real_rgbd.py'),
                    '--serial',row['serial'],'--camera-name',role,'--service-descriptor',str(descriptor),
                    '--output',str(output/('capture-'+role))],env=env,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT))
        return processes
    except BaseException:
        for process in processes:
            process.terminate();process.wait(timeout=5)
        raise


def _address(value):
    return '\0'+value[1:] if value.startswith('@') else value


def _read(sock,size):
    out=bytearray()
    while len(out)<size:
        block=sock.recv(min(size-len(out),1024*1024))
        if not block:raise ConnectionError('camera IPC message truncated')
        out.extend(block)
    return bytes(out)


def _send(sock,value):
    value=dict(value);parts={key:value.pop(key) for key in list(value) if isinstance(value[key],bytes)}
    value['_binary']={key:len(blob) for key,blob in parts.items()}
    header=json.dumps(value,allow_nan=False,separators=(',',':')).encode()
    if len(header)>MAX_HEADER or sum(map(len,parts.values()))>MAX_BODY:
        raise ValueError('camera IPC message exceeds bounded frame size')
    sock.sendall(struct.pack('!I',len(header))+header)
    for blob in parts.values():sock.sendall(blob)


def _receive(sock):
    length=struct.unpack('!I',_read(sock,4))[0]
    if not 0<length<=MAX_HEADER:raise ValueError('camera IPC header exceeds limit')
    value=json.loads(_read(sock,length));parts=value.pop('_binary',{})
    if (not isinstance(value,dict) or not isinstance(parts,dict) or len(parts)>6
            or any(type(n) is not int or n<0 for n in parts.values()) or sum(parts.values())>MAX_BODY
            or any(key in value for key in parts)):
        raise ValueError('invalid bounded camera IPC payload')
    value.update({key:_read(sock,size) for key,size in parts.items()})
    return value


class ChannelClient:
    """RecordingSession's existing channel operations, with no in-process shortcut."""
    def __init__(self,service,camera):
        self.service,self.camera=service,camera;self.cached_token=None;self.cached_snapshot={}
    def _call(self,method,*args,**kwargs):
        return self.service.call('channel',camera=self.camera,method=method,args=list(args),kwargs=kwargs)['value']
    def begin(self,lease):
        return self._call('begin',lease)
    def token(self):return self._call('token')
    def current_lease(self):return self._call('current_lease')
    def renew(self,*args):return self._call('renew',*args)
    def fail(self,reason,*,sampled_token):return self._call('fail',reason,sampled_token=sampled_token)
    def latest(self,token):return self._call('latest',token)
    def ack(self,token,pair_id):return self._call('ack',token,pair_id)
    def snapshot(self,token=None):
        return self._call('snapshot',token)
    def end(self,token):
        return self._call('end',token)
    def stop_after(self,token,endpoint_ns):
        return self._call('stop_after',token,endpoint_ns)
    def close(self):return self._call('close')
    def _frame(self,method,token):
        frame=self.service.call('frame',camera=self.camera,method=method,token=token).get('value')
        return frame
    def peek(self,token):return self._frame('peek',token)
    def pop(self,token):return self._frame('pop',token)


class FrameClient:
    def __init__(self,service,camera):
        self.service,self.camera=service,camera
        self.stop_requested=threading.Event();self.stop_signal=None
    def snapshot(self,formats=None):return self.service.call('preview',camera=self.camera,formats=formats)
    def publish(self,**frame):
        if set(frame)!={'status'}:raise ValueError('raw pixels must enter the common camera publisher')
        return self.service.call('status',camera=self.camera,**frame)
    def request_stop(self,signum,_frame=None):self.stop_signal=signum;self.stop_requested.set()


class CommonCameraService:
    def __init__(self,descriptor,*,process=None):
        self.descriptor=deepcopy(descriptor);self.process=process
        self.frames={camera:FrameClient(self,camera) for camera in descriptor['intrinsics']}
        self.channels={camera:ChannelClient(self,camera) for camera in descriptor['intrinsics']}
        self.directories=descriptor['directories']
        self.pending_failures={};self.failure_lock=threading.Lock();self.failure_worker=None
        self.binding_lock=threading.Lock();self.active_calls=0;self.reconnecting=False;self.generation=0
        self.descriptor_path=None;self.service_pid=None;self.observed_intrinsics={};self.source_clock_id=None;self.capture_instances={}
        self.known_tokens=set();self.retired_tokens=set()

    @classmethod
    def connect(cls,descriptor_path):
        path=Path(descriptor_path).absolute()
        descriptor=json.loads(path.read_text())
        instance=cls(descriptor)
        if instance.call('identity')['instance_id']!=descriptor['instance_id']:
            raise ValueError('camera service instance changed')
        instance.descriptor_path=path
        instance.call('snapshot')
        return instance

    @classmethod
    def start(cls,intrinsics,root,environment,*,budget_bytes=1024**3):
        from launch_environment import clean_environment
        if environment not in ('fake','physical') or set(intrinsics)!=set(CAMERAS.values()):
            raise ValueError('explicit environment and three configured camera identities required')
        if (any(not isinstance(k,dict) or not isinstance(k.get('serial'),str) or not k['serial']
                or any(type(k.get(axis)) is not int or k[axis]<=0 for axis in ('width','height'))
                or k['width']*k['height']*3>16*1024**2 for k in intrinsics.values())
                or len({k['serial'] for k in intrinsics.values()})!=3):
            raise ValueError('three distinct serials and bounded raw image dimensions required')
        root=Path(root);root.mkdir(parents=True,exist_ok=False)
        instance=secrets.token_hex(16)
        endpoint='@robot-rgbd-'+instance if sys.platform.startswith('linux') else str(root/'camera.sock')
        if len(endpoint.encode())>=104:raise ValueError('camera IPC path must be shorter than 104 bytes')
        descriptor=dict(schema='shared_rgbd_service.v1',instance_id=instance,secret=secrets.token_urlsafe(32),
            endpoint=endpoint,environment=environment,intrinsics=intrinsics,budget_bytes=budget_bytes,
            parent_pid=os.getpid(),directories={camera:str(root/camera) for camera in intrinsics})
        path=root/'camera-service.json'
        with os.fdopen(os.open(path,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600),'w') as stream:
            json.dump(descriptor,stream,allow_nan=False)
        for camera in intrinsics:
            directory=root/camera;directory.mkdir()
            (directory/'recording-service.json').write_text(json.dumps(dict(schema='rgbd_lease.v1',
                transport='unix_rgbd',descriptor=str(path),camera=camera)))
        code=Path(__file__).resolve().parents[1]
        paths=[str(code),str(code/'rule_based_workbench'),*sys.path]
        env=clean_environment(dict(PYTHONPATH=os.pathsep.join(dict.fromkeys(paths)),
            PYTHONDONTWRITEBYTECODE='1',CUDA_VISIBLE_DEVICES='',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1'))
        with (root/'camera-service.log').open('xb') as log:
            process=subprocess.Popen([sys.executable,'-u','-B','-m','robot_workbench.rgbd_service',str(path)],
                env=env,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT)
        client=cls(descriptor,process=process);deadline=time.monotonic()+10
        try:
            while True:
                try:client.call('identity');break
                except (OSError,ConnectionError):
                    if process.poll() is not None or time.monotonic()>=deadline:
                        raise RuntimeError('shared camera service failed to start; see '+str(root/'camera-service.log'))
                    time.sleep(.02)
            return client
        except BaseException:
            process.terminate();process.wait(timeout=5);raise

    def call(self,action,*,_generation=None,**fields):
        with self.binding_lock:
            if self.reconnecting:raise ValueError('camera service reconnect in progress')
            if _generation is not None and _generation!=self.generation:
                raise ValueError('camera operation belongs to a previous service binding')
            if action=='channel' and fields.get('method')=='begin':
                token=fields['args'][0]['run_id']
                if token in self.retired_tokens:raise ValueError('previous camera service recording lease cannot resume')
                if self.channels[fields['camera']].cached_token is not None:
                    raise ValueError('camera recording channel is already leased or its begin reply is unknown')
                self.known_tokens.add(token)
                # A lost begin reply is ambiguous: retain the lease until explicit end succeeds.
                self.channels[fields['camera']].cached_token=token
            self.active_calls+=1;descriptor=self.descriptor
        try:
            value=self._request(descriptor,action,**fields)
            with self.binding_lock:self._remember(action,fields,value)
            return value
        finally:
            with self.binding_lock:self.active_calls-=1

    def _remember(self,action,fields,value):
        if action in ('identity','snapshot'):self.service_pid=value.get('service_pid',self.service_pid)
        if action=='snapshot':
            self.observed_intrinsics.update(deepcopy(value.get('observed_intrinsics',{})))
            self.source_clock_id=value.get('source_clock_id')
            self.capture_instances=dict(value.get('capture_instances',{}))
        camera=fields.get('camera');channel=self.channels.get(camera)
        if channel is None:return
        method=fields.get('method');result=value.get('value')
        if action=='channel':
            if method=='end' and channel.cached_token==fields['args'][0]:channel.cached_token=None
            if method=='close':channel.cached_token=None
            if method=='snapshot':channel.cached_snapshot=result
        if action=='frame' and result is not None:channel.cached_snapshot=result.pop('_channel_snapshot')
        metadata=value.get('metadata') if action=='latest' else value.get('status')
        if isinstance(metadata,dict) and metadata.get('intrinsics'):
            self.observed_intrinsics[camera]=deepcopy(metadata['intrinsics'])
            if 'source_clock_id' in metadata:self.source_clock_id=metadata['source_clock_id']
            if 'capture_instance_id' in metadata:self.capture_instances[camera]=metadata['capture_instance_id']

    @staticmethod
    def _request(descriptor,action,**fields):
        deadline=time.monotonic()+5
        def remaining():
            seconds=deadline-time.monotonic()
            if seconds<=0:raise TimeoutError('camera IPC request exceeded its 5 second budget')
            return seconds
        while True:
            with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as sock:
                sock.settimeout(remaining())
                try:sock.connect(_address(descriptor['endpoint']))
                except BlockingIOError as error:
                    if error.errno not in (errno.EAGAIN,errno.EWOULDBLOCK):raise
                else:
                    # Only connect can retry: publish/pop/ack may mutate state
                    # after any request bytes have been sent, even without a reply.
                    sock.settimeout(remaining())
                    _send(sock,dict(action=action,secret=descriptor['secret'],
                        expected_instance_id=descriptor.get('instance_id'),**fields))
                    sock.settimeout(remaining());value=_receive(sock)
                    break
            time.sleep(min(.005,remaining()))
        if 'error' in value:raise ValueError(value['error'])
        if value.get('_frame'):
            value={'value':{key:item for key,item in value.items() if key!='_frame'}}
        return value

    def reconnect_state(self):
        """Local diagnostic state; reconnect itself also checks remote recording leases."""
        with self.binding_lock:
            return dict(instance_id=self.descriptor.get('instance_id'),service_pid=self.service_pid,
                environment=self.descriptor.get('environment'),generation=self.generation,
                supported=self.descriptor_path is not None and self.process is None,
                reconnecting=self.reconnecting,active_calls=self.active_calls,
                recording_leases={camera:channel.cached_token for camera,channel in self.channels.items()
                                  if channel.cached_token is not None})

    def reconnect(self,*,max_age_s=2.,allow_current=False):
        """Explicit idle-only replacement, using only the originally trusted descriptor path.

        The caller must hold the device-wide business activity gate. No operation or
        recording lease is replayed; failed validation leaves this binding untouched.
        """
        if type(max_age_s) not in (int,float) or not 0<max_age_s<=30:
            raise ValueError('bounded camera freshness age required')
        if type(allow_current) is not bool:raise ValueError('explicit boolean allow_current required')
        with self.binding_lock:
            if self.descriptor_path is None or self.process is not None:
                raise ValueError('reconnect requires an attached trusted descriptor')
            if self.reconnecting or self.active_calls:
                raise ValueError('camera operations are active; reconnect requires idle clients')
            if any(channel.cached_token is not None for channel in self.channels.values()):
                raise ValueError('cannot reconnect while a recording lease remains')
            self.reconnecting=True;old=self.descriptor
            old_identity=dict(instance_id=old['instance_id'],service_pid=self.service_pid,
                              capture_instances=dict(self.capture_instances))
        try:
            try:
                previous=self._request(old,'snapshot')
            except (OSError,ConnectionError,ValueError):
                # A live but unreachable server may still hold another client's lease.
                if self.service_pid is None:raise ValueError('old camera service lease state is unknown') from None
                try:os.kill(self.service_pid,0)
                except ProcessLookupError:previous={}
                else:raise ValueError('old camera service is unavailable; cannot establish idle lease state') from None
            if any(row.get('run_id') for row in previous.get('recording_channels',{}).values()):
                raise ValueError('cannot reconnect while a recording lease remains')
            old_identity['capture_instances']=previous.get('capture_instances',old_identity['capture_instances'])
            try:candidate=type(self).connect(self.descriptor_path)
            except Exception:raise ValueError('replacement camera descriptor or service unavailable') from None
            new=candidate.descriptor
            same=new['instance_id']==old['instance_id']
            if same and not allow_current:raise ValueError('camera service instance has not changed')
            if new.get('environment')!=old.get('environment') or new.get('intrinsics')!=old.get('intrinsics'):
                raise ValueError('camera environment, serial or intrinsics changed; new device setup required')
            before=candidate.snapshot()
            if any(row.get('run_id') for row in before['recording_channels'].values()):
                raise ValueError('replacement camera service has an active recording lease')
            observed=previous.get('observed_intrinsics',self.observed_intrinsics)
            source_clock=previous.get('source_clock_id',self.source_clock_id)
            if old['environment']=='fake' and source_clock is None:
                raise ValueError('original camera source clock is unverified; new device setup required')
            cameras={}
            for camera in self.frames:
                frame=candidate.latest_rgbd(camera,max_age_s=max_age_s);meta=frame['metadata'];intr=meta['intrinsics']
                width,height=new['intrinsics'][camera]['width'],new['intrinsics'][camera]['height']
                if (meta.get('environment')!=old['environment'] or meta.get('serial')!=old['intrinsics'][camera]['serial']
                        or meta.get('capture_instance_id')!=before['capture_instances'].get(camera)
                        or not isinstance(frame.get('rgb'),bytes) or len(frame['rgb'])!=width*height*3
                        or not isinstance(frame.get('depth'),bytes) or len(frame['depth'])!=width*height*2
                        or (frame.get('width'),frame.get('height'))!=(width,height)
                        or meta.get('ready') is not True or meta.get('raw') is not True
                        or meta.get('annotated') is not False or meta.get('rgbd_paired') is not True
                        or meta.get('depth_aligned_to')!='color'):
                    raise ValueError('replacement camera raw RGB-D identity is invalid')
                if camera in observed and observed[camera]!=intr:
                    raise ValueError('observed camera intrinsics changed; new device setup required')
                clock=clock_spec(meta['environment'],clock_domain=meta.get('clock_domain'),source_clock_id=meta.get('source_clock_id'))
                sample_stamp(meta,clock)
                if source_clock is not None and clock['source_clock_id']!=source_clock:
                    raise ValueError('camera source clock changed; new device setup required')
                cameras[camera]=dict(capture_instance_id=meta['capture_instance_id'],serial=meta['serial'],
                    frame_number=meta['frame_number'],pair_id=meta['pair_id'],host_receive_ns=meta['host_receive_ns'],
                    source_age_s=frame['source_age_s'],clock=clock,rgb_bytes=len(frame['rgb']),depth_bytes=len(frame['depth']),
                    healthy=True)
            after=candidate.snapshot()
            if (after['capture_instances']!=before['capture_instances'] or
                    any(row.get('run_id') for row in after['recording_channels'].values())):
                raise ValueError('replacement camera source or recording lease changed during validation')
            for row in cameras.values():
                row['source_age_s']=(time.time_ns()-row['host_receive_ns'])/1e9
                if not 0<=row['source_age_s']<=max_age_s:raise ValueError('replacement camera frame stale during validation')
            if not same:
                with self.binding_lock:
                    self.descriptor=deepcopy(new);self.directories.clear();self.directories.update(new['directories'])
                    self.service_pid=candidate.service_pid;self.observed_intrinsics=candidate.observed_intrinsics
                    self.source_clock_id=candidate.source_clock_id;self.generation+=1
                    self.capture_instances=candidate.capture_instances
                    self.retired_tokens.update(self.known_tokens)
                    for channel in self.channels.values():channel.cached_snapshot={}
            return dict(reconnected=not same,already_bound=same,old=old_identity,new=dict(instance_id=new['instance_id'],service_pid=self.service_pid),
                environment=new['environment'],generation=self.generation,cameras=cameras,recording_leases_restored=False)
        finally:
            with self.binding_lock:self.reconnecting=False

    def publish(self,camera,metadata,rgb,depth,*,width,height,sampled_token):
        return self.call('publish',camera=camera,metadata=metadata,rgb=rgb,depth=depth,
                         width=width,height=height,sampled_token=sampled_token)

    def fail_async(self,reason,tokens):
        """Source/Owner loops never wait for an unavailable camera service."""
        with self.binding_lock:generation=self.generation
        with self.failure_lock:
            self.pending_failures.update({camera:(reason,token,generation) for camera,token in tokens.items() if token})
            if self.failure_worker and self.failure_worker.is_alive():return
            def flush():
                while True:
                    with self.failure_lock:
                        if not self.pending_failures:return
                        camera,(message,token,generation)=self.pending_failures.popitem()
                    try:self.call('channel',camera=camera,method='fail',args=[message],
                                  kwargs=dict(sampled_token=token),_generation=generation)
                    except (OSError,ValueError,ConnectionError):pass
            self.failure_worker=threading.Thread(target=flush,daemon=True,name='camera-failure-notice')
            self.failure_worker.start()

    def latest_rgbd(self,camera,*,max_age_s=2.):
        """Atomic pair for inference; pixel arrays are unannotated and unrectified."""
        return self.call('latest',camera=camera,max_age_s=max_age_s)

    def rebind_camera(self,camera,expected_capture_instance_id):
        return self.call('rebind',camera=camera,expected_capture_instance_id=expected_capture_instance_id)

    def start_servers(self,ports=None):return self.call('http',ports=ports or dict.fromkeys(CAMERAS,0))['ports']
    def snapshot(self):return self.call('snapshot')
    def close(self):
        if self.process is None:return  # Attached clients never stop a shared server.
        try:self.call('close')
        except (OSError,ValueError,ConnectionError):pass
        try:self.process.wait(timeout=5)
        except subprocess.TimeoutExpired:self.process.terminate();self.process.wait(timeout=5)


class _Preview:
    def __init__(self):
        from real_rgbd import Frames
        self.frames=Frames();self.condition=self.frames.condition
        self.pixels=None;self.cached=None;self.lock=threading.Lock();self.encoder_lock=threading.Lock()
    @property
    def latest(self):return self.frames.latest
    @property
    def closed(self):return self.frames.closed
    def publish(self,value):
        with self.lock:self.pixels=value;self.cached=None
        self.frames.publish(status=value['metadata'])
    def snapshot(self,formats=None):
        from PIL import Image
        formats=('jpeg','color_png','depth_png') if formats is None else tuple(formats)
        if not set(formats)<={'jpeg','color_png','depth_png'}:raise ValueError('known camera image formats required')
        with self.encoder_lock:
            with self.lock:value,cached=self.pixels,self.cached
            if value is None:return self.frames.snapshot()
            cached=dict(cached or {})
            if not set(formats)<=set(cached):
                size=(value['width'],value['height'])
                rgb=Image.frombytes('RGB',size,value['rgb']);depth=Image.frombytes('I;16',size,value['depth'])
                out={}
                for key,image,fmt,kwargs in [('jpeg',rgb,'JPEG',dict(quality=95,subsampling=0)),
                        ('color_png',rgb,'PNG',dict(compress_level=1)),('depth_png',depth,'PNG',dict(compress_level=1))]:
                    if key in formats and key not in cached:
                        stream=io.BytesIO();image.save(stream,format=fmt,**kwargs);out[key]=stream.getvalue()
                cached.update(out)
                with self.lock:
                    if self.pixels is value:self.cached=cached
            status=dict(value['metadata']);age=(time.time_ns()-int(status['host_receive_ns']))/1e9
            status['source_age_s']=max(0.,age)
            if age>2:status.update(ready=False,error='camera frame stale')
            return dict(status=status,**{key:cached[key] for key in formats})


class _Backend:
    def __init__(self,config):
        self.config=config;self.budget=MemoryRgbdBudget(config['budget_bytes'])
        self.channels={camera:MemoryRgbdChannel(lease_guard=lambda _:True,budget=self.budget) for camera in config['intrinsics']}
        self.previews={camera:_Preview() for camera in self.channels}
        self.progress={};self.stream_ids={};self.observed_intrinsics={};self.http={};self.http_lock=threading.Lock();self.closed=threading.Event()
        self.camera_locks={camera:threading.Lock() for camera in self.channels}
        self.source_clock_id=None;self.identity_lock=threading.Lock()

    def dispatch(self,request):
        if not hmac.compare_digest(request.pop('secret',''),self.config['secret']):raise PermissionError('camera service identity required')
        if request.pop('expected_instance_id',self.config['instance_id'])!=self.config['instance_id']:
            raise ValueError('camera service instance changed')
        action=request.pop('action');camera=request.get('camera')
        if action=='identity':return dict(instance_id=self.config['instance_id'],service_pid=os.getpid())
        if action=='close':self.closed.set();return dict(closed=True)
        if action=='snapshot':return dict(instance_id=self.config['instance_id'],service_pid=os.getpid(),environment=self.config['environment'],
            recording_budget=self.budget.snapshot(),recording_channels={k:v.snapshot() for k,v in self.channels.items()},
            capture_instances=dict(self.stream_ids),source_clock_id=self.source_clock_id,
            observed_intrinsics=deepcopy(self.observed_intrinsics))
        if action=='http':
            from http.server import ThreadingHTTPServer
            from real_rgbd import make_handler
            ports=request['ports']
            if set(ports)!=set(CAMERAS):raise ValueError('three camera preview ports required')
            with self.http_lock:
                for role,name in CAMERAS.items():
                    if role not in self.http:
                        server=ThreadingHTTPServer(('127.0.0.1',ports[role]),make_handler(self.previews[name]));server.daemon_threads=True
                        threading.Thread(target=server.serve_forever,daemon=True).start();self.http[role]=server
                return dict(ports={role:server.server_port for role,server in self.http.items()})
        if camera not in self.channels:raise ValueError('unknown configured camera')
        channel=self.channels[camera];preview=self.previews[camera]
        if action=='rebind':
            with self.camera_locks[camera]:
                if channel.current_lease() is not None:raise ValueError('cannot rebind a camera during recording')
                if self.stream_ids.get(camera)!=request['expected_capture_instance_id']:raise ValueError('camera source changed before rebind')
                self.stream_ids.pop(camera,None);self.progress.pop(camera,None);self.observed_intrinsics.pop(camera,None)
                with preview.lock:preview.pixels=preview.cached=None
                preview.frames.publish(status=dict(ready=False,phase='awaiting_explicit_capture_rebind'))
                return dict(rebound=True)
        if action=='channel':
            method=request['method']
            if method not in ('begin','token','current_lease','renew','fail','latest','snapshot','end','close','ack','stop_after'):
                raise ValueError('unknown recording channel operation')
            return dict(value=getattr(channel,method)(*request['args'],**request['kwargs']))
        if action=='frame':
            if request['method'] not in ('peek','pop'):raise ValueError('unknown recording frame operation')
            frame=getattr(channel,request['method'])(request['token'])
            return dict(_frame=True,_channel_snapshot=channel.snapshot(),**frame) if frame is not None else dict(value=None)
        if action=='preview':return preview.snapshot(request.get('formats'))
        if action=='latest':
            with preview.lock:
                if preview.pixels is None:raise ValueError('camera has no raw RGB-D pair yet')
                value=dict(preview.pixels)
            limit=request.get('max_age_s',2.)
            if type(limit) not in (int,float) or not 0<limit<=30:raise ValueError('bounded camera freshness age required')
            age=(time.time_ns()-int(value['metadata']['host_receive_ns']))/1e9
            if not 0<=age<=limit:raise ValueError('raw camera frame stale or host clock inconsistent')
            return dict(value,source_age_s=age)
        if action=='status':
            if not request['status'].get('ready'):
                with preview.lock:preview.pixels=preview.cached=None
            else:
                with preview.lock:
                    if preview.pixels and request['status'].get('pair_id')==preview.pixels['metadata'].get('pair_id'):
                        preview.pixels=dict(preview.pixels,metadata=deepcopy(request['status']))
            preview.frames.publish(status=request['status']);return dict(accepted=True)
        if action!='publish':raise ValueError('unknown camera IPC action')
        with self.camera_locks[camera]:
            meta=deepcopy(request['metadata']);k=self.config['intrinsics'][camera]
            intr=meta.get('intrinsics',{})
            if (meta.get('environment')!=self.config['environment'] or meta.get('serial')!=k['serial']
                    or (intr.get('width'),intr.get('height'))!=(k['width'],k['height'])
                    or (request['width'],request['height'])!=(k['width'],k['height'])
                    or not isinstance(meta.get('capture_instance_id'),str) or not meta['capture_instance_id']):
                raise ValueError('raw camera identity, intrinsics or dimensions changed')
            if camera in self.observed_intrinsics and self.observed_intrinsics[camera]!=intr:
                raise ValueError('camera intrinsics changed inside one capture stream')
            if (meta.get('ready') is not True or meta.get('raw') is not True or meta.get('annotated') is not False
                    or meta.get('rgbd_paired') is not True or meta.get('depth_aligned_to')!='color'):
                raise ValueError('raw unannotated aligned RGB-D pair required')
            if any(key in k and (type(intr.get(key)) not in (int,float) or abs(intr[key]-k[key])>1e-6)
                   for key in ('fx','fy','cx','cy')):
                raise ValueError('camera intrinsics differ from configured capture identity')
            from real_rgbd import pair_metadata
            pair_metadata(serial=meta['serial'],camera_name=meta['camera_name'],color_number=meta['frame_number'],
                depth_number=meta['depth_frame_number'],color_ms=meta['device_timestamp_ms'],depth_ms=meta['depth_device_timestamp_ms'],
                color_domain=meta['timestamp_domain'],depth_domain=meta['timestamp_domain'],host_receive_ns=int(meta['host_receive_ns']),
                depth_scale_m=meta['depth_scale_m'],intrinsics=intr,fps=meta['capture_fps'])
            identity=meta['capture_instance_id']
            if camera in self.stream_ids and self.stream_ids[camera]!=identity:
                raise ValueError('camera capture instance changed; explicit service rebind required')
            clock=clock_spec(meta['environment'],clock_domain=meta.get('clock_domain'),source_clock_id=meta.get('source_clock_id'))
            stamp=sample_stamp(dict(meta,host_receive_ns=int(meta['host_receive_ns'])),clock)
            if not 0<int(meta['host_receive_ns'])<2**63 or not 0<=stamp[0]<2**63:
                raise ValueError('camera timestamps exceed bounded nanosecond domain')
            assert_progress(self.progress.get(camera),stamp)
            rgb,depth=request['rgb'],request['depth'];width,height=request['width'],request['height']
            if (not isinstance(rgb,bytes) or not isinstance(depth,bytes) or len(rgb)!=width*height*3
                    or len(depth)!=width*height*2 or max(len(rgb),len(depth))>16*1024**2):
                raise ValueError('raw RGB8/depth16le payload size differs from configured camera')
            if meta['environment']=='fake':
                with self.identity_lock:
                    if self.source_clock_id not in (None,clock['source_clock_id']):raise ValueError('cameras belong to different simulation source clocks')
                    self.source_clock_id=clock['source_clock_id']
            self.stream_ids[camera]=identity;self.progress[camera]=stamp;self.observed_intrinsics[camera]=intr
            meta['rgbd_ingress_format']='rgb8-depth16le'
            source_valid=meta.get('data_valid',True)
            accepted=channel.write_pixels(meta,rgb,depth,width=width,height=height,sampled_token=request['sampled_token'])
            # A recording overflow does not invalidate the current sensor image
            # used by preview/inference; its recording_error remains visible.
            preview.publish(dict(metadata=dict(meta,data_valid=source_valid),rgb=rgb,depth=depth,width=width,height=height,pixel_format='rgb8-depth16le'))
            return dict(recorded=accepted,status=meta,channel=channel.snapshot())


def _serve(path):
    config=json.loads(Path(path).read_text());backend=_Backend(config)
    slots=threading.BoundedSemaphore(16)
    class Handler(socketserver.BaseRequestHandler):
        def handle(self):
            self.request.settimeout(5)
            try:_send(self.request,backend.dispatch(_receive(self.request)))
            except Exception as error:
                try:_send(self.request,dict(error=f'{type(error).__name__}: {error}'))
                except OSError:pass
    class Server(socketserver.ThreadingMixIn,socketserver.UnixStreamServer):
        daemon_threads=True
        def process_request(self,request,address):
            if not slots.acquire(blocking=False):request.close();return
            super().process_request(request,address)
        def process_request_thread(self,request,address):
            try:super().process_request_thread(request,address)
            finally:slots.release()
    endpoint=_address(config['endpoint'])
    with Server(endpoint,Handler) as server:
        if not config['endpoint'].startswith('@'):os.chmod(endpoint,0o600)
        threading.Thread(target=server.serve_forever,daemon=True).start()
        while not backend.closed.wait(.2):
            if os.getppid()!=config['parent_pid']:break
        server.shutdown()
        for camera in backend.channels.values():camera.close()
        for server in backend.http.values():server.shutdown();server.server_close()
    if not config['endpoint'].startswith('@'):Path(endpoint).unlink(missing_ok=True)


if __name__=='__main__':_serve(sys.argv[1])
