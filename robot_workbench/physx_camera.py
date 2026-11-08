"""Three cached RGB-D streams from one caller-owned PhysX plant.

Only collect() may touch the plant, on its main thread. HTTP and encoding never
step physics; preview frames stay in memory and recording uses bounded leases.
"""
from collections import deque
from concurrent.futures import ThreadPoolExecutor, wait
from copy import deepcopy
import json
import os
from pathlib import Path
import queue
import secrets
import threading
import time

CAMERAS = {'head':'base_0', 'left':'left_wrist_0', 'right':'right_wrist_0'}


def camera_intrinsics(rig):
    result = {}
    for role, name in CAMERAS.items():
        value = rig['device_calibrations'][name]
        coefficients = value['distortion']['coefficients']
        if len(coefficients) != 5 or any(v != 0 for v in coefficients):
            raise ValueError('this fake raw pinhole producer requires zero-distortion rig')
        result[name] = dict(serial=value['serial'], width=value['width'], height=value['height'],
            **value['intrinsics'], coefficients=list(coefficients), rectified=False,
            distortion_model='distortion.inverse_brown_conrady', dictionary='DICT_4X4_50',
            left_id=24, right_id=30, black_size_m=.015)
    if len({row['serial'] for row in result.values()}) != 3:
        raise ValueError('three distinct camera identities required')
    return result


class CameraProducer:
    def __init__(self, intrinsics, root, *, queue_capacity=8, min_free_bytes=2*1024**3,recording_guard=None,
                 memory_recording=True,recording_budget_bytes=1024**3,ros_plane=None,device_id=None):
        if set(intrinsics) != set(CAMERAS.values()) or type(queue_capacity) is not int or queue_capacity < 1:
            raise ValueError('three cameras and a positive bounded queue required')
        self.intrinsics = deepcopy(intrinsics)
        self.root = Path(root)
        self.frames, self.spools, self.directories, self.channels = {}, {}, {}, {}
        self.recording_guard=recording_guard;self.channel_status={}
        self.ros_plane=ros_plane;self.ros=None;self.ros_sources={};self._backend=None
        if type(memory_recording) is not bool or (ros_plane is None and memory_recording and not callable(recording_guard)):
            raise ValueError('same-process recording requires a live Owner recording guard')
        if ros_plane is not None:
            from .ros_data import plane_config
            from .ros_inputs import RosCameraView
            self.ros_plane=plane_config(ros_plane)
            if not isinstance(device_id,str)or not device_id:raise ValueError('ROS fake source requires its registered device identity')
            self.device_id=device_id;self._open_ros()
            self.common=RosCameraView(self.ros_plane,device_id,intrinsics,'fake')
            self.service_owner=None;self.descriptor_path=None;self.publisher=self
            self.frames=self.common.frames
        else:
            from real_rgbd import RawSpool
            from robot_workbench.rgbd_service import CommonCameraService
            self.service_owner=CommonCameraService.start(intrinsics,self.root,'fake',budget_bytes=recording_budget_bytes)
            self.descriptor_path=self.root/'camera-service.json'
            self.common=CommonCameraService.connect(self.descriptor_path)
            self.publisher=self.common;self.frames,self.directories=self.common.frames,self.common.directories
            if memory_recording:self.channels=self.common.channels
            for name in (() if memory_recording else CAMERAS.values()):
                self.spools[name]=RawSpool(self.root/name,min_free_bytes=min_free_bytes,lease_guard=recording_guard)
        self.publish_generation=0;self.ports=None
        self.sampling_lock=threading.RLock();self.restart_lock=threading.Lock();self.paused=False
        self.queue = queue.Queue(maxsize=queue_capacity)
        self.closed = threading.Event()
        self.thread_id = threading.get_ident()
        self.lock = threading.Lock()
        self.drops = deque(maxlen=512)
        self.drop_history_before_ns = 0
        self.dropped_batches = 0
        self.error = None
        self.last_drop = self.last_publish = None
        self.maximum_queue_wait_s = self.maximum_publish_s = 0.
        self.last_step = None
        self.frame_sequence = 0
        self.clock_id = None
        self.encoders = ThreadPoolExecutor(max_workers=3, thread_name_prefix='rgbd-camera')
        self.worker = threading.Thread(target=self._encode_loop, daemon=True, name='physx-rgbd-encoder')
        self.worker.start()

    def _open_ros(self):
        from sensor_msgs.msg import Image
        from .ros_data import RosNode
        self.descriptor=dict(instance_id=secrets.token_hex(16),environment='fake',data_plane=self.ros_plane)
        self.ros=RosNode('fake_camera_'+self.descriptor['instance_id'][:12],namespace=self.ros_plane['namespace'],
            domain_id=self.ros_plane['domain_id'],localhost_only=False)
        self.ros_sources={}
        self.ros_images={name:self.ros.publisher(self.ros_plane['namespace']+'/'+name,Image)
            for camera in CAMERAS.values()for name in (camera,camera+'_depth')}
        self.ros_declarations={name:self.ros.publisher('sources/'+name,self.ros.types.SourceDescriptor,retained=True,depth=1)
            for name in self.ros_images}
        self.ros_metadata={name:self.ros.publisher(self.ros_plane['namespace']+'/'+name+'/metadata',self.ros.types.DataEvent)
            for name in CAMERAS.values()}

    def _close_ros(self):
        if self.ros is None or self.ros.closed:return
        from .ros_data import descriptor_message
        for name,value in self.ros_sources.items():
            self.ros_declarations[name].publish(descriptor_message(self.ros,dict(value,available=False)))
        self.ros.close()

    def _publish_ros(self,packet,name,rgb,depth):
        from sensor_msgs.msg import Image
        from .ros_data import descriptor_message,StreamRef,json_text
        stamp=packet['feedback']['source_time_ns'];k=self.intrinsics[name]
        for stream,encoding,pixels,width in ((name,'rgb8',rgb.tobytes(),3),(name+'_depth','16UC1',depth.astype('<u2',copy=False).tobytes(),2)):
            if stream not in self.ros_sources:
                meta=dict(k,environment='fake',clock_domain='simulation',pixel_format='rgb8'if width==3 else'depth16le',
                    simulation_only=True,evidence_source='fake_device_rgbd',timestamp_domain='simulation',
                    timestamp_kind='simulation_step',camera_name=next(role for role,camera in CAMERAS.items()if camera==name))
                if width==3:meta.update(depth_stream_id=name+'_depth',intrinsics=dict(k),
                    metadata_topic=self.ros_plane['namespace']+'/'+name+'/metadata')
                else:meta.update(depth_scale_m=.001,depth_aligned_to='color')
                value=dict(device_id=self.device_id,stream_id=stream,source_instance_id=self.descriptor['instance_id'],
                    source_clock_id=packet['feedback']['source_clock_id'],topic=self.ros_plane['namespace']+'/'+stream,
                    message_type='sensor_msgs/msg/Image',kind='image',units=meta['pixel_format'],coordinate_frame=name,
                    nominal_hz=30.,available=True,metadata=meta)
                self.ros_sources[stream]=value;self.ros_declarations[stream].publish(descriptor_message(self.ros,value))
            frame=Image(width=k['width'],height=k['height'],encoding=encoding,step=k['width']*width,data=pixels)
            frame.header.stamp.sec=stamp//10**9;frame.header.stamp.nanosec=stamp%10**9;frame.header.frame_id=name
            self.ros_images[stream].publish(frame)
        actor=packet['actor']
        metadata=dict(host_receive_ns=packet['wall_ns'],physics_step=packet['feedback']['physics_step'],
            frame_number=packet['frame_sequence'],owner_available=actor.get('owner_available')is True,
            owner_pid=actor.get('owner_pid'),command_epoch=actor.get('command_epoch'),runtime_nonce=actor.get('runtime_nonce'))
        self.ros_metadata[name].publish(self.ros.types.DataEvent(
            reference=StreamRef(name,self.descriptor['instance_id'],packet['feedback']['source_clock_id'],stamp,stamp).message(),
            run_id='',epoch=actor.get('command_epoch')or 0,event='frame_metadata',payload_json=json_text(metadata)))
        return dict(transport='ros',recorded=False,source_time_ns=stamp)

    def start_servers(self, ports=None):
        self.ports=self.common.start_servers(ports)
        return self.ports

    def collect(self, plant, backend, actor):
        """Call after this step's native render, never from an HTTP handler."""
        if threading.get_ident() != self.thread_id:
            raise RuntimeError('only the PhysX main thread may sample cameras')
        with self.sampling_lock:
            return self._collect(plant,backend,actor)

    def _collect(self,plant,backend,actor):
        if self.closed.is_set() or self.paused:
            return False
        self._backend=backend
        feedback = backend.read_calibration_state()
        step = feedback['physics_step']
        if self.clock_id is not None and self.clock_id != feedback['source_clock_id']:
            raise ValueError('a resident producer cannot switch or reset its plant clock')
        if self.last_step is not None and step <= self.last_step:
            if step == self.last_step:return False
            raise ValueError('camera source physics step regressed')
        self.clock_id, self.last_step = feedback['source_clock_id'], step
        sampled_wall_ns = time.time_ns()
        images = plant.cameras(render=False)
        actual, velocity = plant.read()
        if actual != backend.actual or velocity != backend.velocity:
            raise RuntimeError('rendered camera and actual feedback do not describe the same completed plant step')
        packet = dict(feedback=deepcopy(feedback), actor=deepcopy(actor), wall_ns=sampled_wall_ns,
                      frame_sequence=self.frame_sequence,
                      camera_service=self.publisher,publish_generation=self.publish_generation,
                      recording_tokens={name: channel.cached_token for name, channel in self.channels.items()},
                      images={name: {key: values[key].copy() for key in ('rgb','depth')}
                              for name, values in images.items()})
        self.frame_sequence += 1
        packet['queued_monotonic_s'] = time.monotonic()
        try:
            self.queue.put_nowait(packet)
            return True
        except queue.Full:
            self._record_drop(packet, 'camera source queue overflow')
            return False

    def _record_drop(self, packet, reason):
        with self.lock:
            if len(self.drops) == self.drops.maxlen:self.drop_history_before_ns = max(self.drop_history_before_ns,self.drops[0]['sample_wall_ns'])
            self.dropped_batches += 1
            self.last_drop = dict(reason=reason, sample_wall_ns=packet['wall_ns'], detected_wall_ns=time.time_ns(),
                frame_sequence=packet['frame_sequence'], queue_size=self.queue.qsize(), queue_capacity=self.queue.maxsize,
                **{key:packet['feedback'][key] for key in ('source_clock_id','source_time_ns','physics_step')},
                last_publish=deepcopy(self.last_publish))
            self.drops.append(self.last_drop)
        if self.ros_plane is not None:
            with self.lock:self.error=reason
        elif self.common:self.common.fail_async(reason+' during recording lease',packet.get('recording_tokens',{}))
        else:
            for name, channel in self.channels.items():
                channel.fail(reason+' during recording lease', sampled_token=packet.get('recording_tokens', {}).get(name))

    def _lease_drop_error(self, spool):
        if hasattr(spool, 'current_lease'):
            lease = spool.current_lease()
            if lease is None:return None
        else:
            request = spool.root/'recording-request.json'
            try:lease=json.loads(request.read_text())
            except FileNotFoundError:return None
        if spool.lease_guard is not None and not spool.lease_guard(lease):return None
        start, end = lease.get('admission_start_wall_ns',lease.get('start_wall_ns')), lease.get('deadline_wall_ns')
        if type(start) is not int or type(end) is not int:return 'invalid recording lease'
        with self.lock:
            # ponytail: bounded drop history; older leases are unverifiable rather than silently complete.
            if start <= self.drop_history_before_ns:return 'camera source drop history unavailable for recording lease'
            dropped = next((row for row in self.drops if start <= row['sample_wall_ns'] < end),None)
        return dropped['reason']+' during recording lease' if dropped else None

    def _encode_loop(self):
        while not self.closed.is_set() or not self.queue.empty():
            try:packet = self.queue.get(timeout=.1)
            except queue.Empty:continue
            started=time.monotonic();queue_wait=started-packet['queued_monotonic_s']
            cameras=None
            try:cameras=self._publish(packet)
            except Exception as error:
                self._record_drop(packet, f'{type(error).__name__}: {error}')
                with self.lock:self.error = f'{type(error).__name__}: {error}'
                if self.ros_plane is not None:self.common._unavailable(self.error)
                else:
                    for frame in self.frames.values():
                        try:frame.publish(status=dict(ready=False, data_valid=False, error=self.error))
                        except (OSError,ConnectionError,ValueError):pass
            finally:
                elapsed=time.monotonic()-started
                with self.lock:
                    self.last_publish=dict(physics_step=packet['feedback']['physics_step'],queue_wait_s=queue_wait,
                        publish_wall_s=elapsed,cameras=cameras)
                    self.maximum_queue_wait_s=max(self.maximum_queue_wait_s,queue_wait)
                    self.maximum_publish_s=max(self.maximum_publish_s,elapsed)
                self.queue.task_done()

    def _publish(self, packet):
        # Each camera keeps its own ordered spool; three encoders share only the
        # immutable sampled packet. No GPU/physics work moves off the main thread.
        if packet['publish_generation']!=self.publish_generation:
            raise ValueError('camera packet belongs to a previous source service')
        futures = [self.encoders.submit(self._publish_camera, packet, role, name)
                   for role, name in CAMERAS.items()]
        wait(futures)
        return {name:future.result() for name,future in zip(CAMERAS.values(),futures)}

    def _publish_camera(self, packet, role, name):
        import numpy as np
        from real_rgbd import pair_metadata
        from .capture_diagnostics import unsupported
        started=time.monotonic()
        feedback, actor = packet['feedback'], packet['actor']
        source_fields = {key: feedback[key] for key in
            ('clock_domain','source_clock_id','source_time_ns','physics_step')}
        image = packet['images'][name]
        rgb, depth = image['rgb'], image['depth']
        k = self.intrinsics[name]
        if rgb.shape != (k['height'],k['width'],3) or rgb.dtype != np.uint8 or depth.shape != rgb.shape[:2]:
            raise ValueError('plant RGB-D shape/dtype differs from camera intrinsics')
        valid = np.isfinite(depth) & (depth > 0) & (depth <= 65.535)
        quantized = np.rint(np.where(valid,depth,0)/.001).astype(np.uint16)
        if self.ros_plane is not None:return self._publish_ros(packet,name,rgb,quantized)
        number = packet['frame_sequence']
        ms = feedback['source_time_ns']/1e6
        status = pair_metadata(serial=k['serial'], camera_name=role, color_number=number,
            depth_number=number, color_ms=ms, depth_ms=ms, color_domain='simulation',
            depth_domain='simulation', host_receive_ns=packet['wall_ns'], depth_scale_m=.001,
            intrinsics=k, fps=30)
        status.update(**source_fields, host_receive_ns=packet['wall_ns'], timestamp_kind='simulation_step',
            capture_instance_id=packet['camera_service'].descriptor['instance_id'],
            environment='fake', simulation_only=True, evidence_source='fake_device_rgbd',
            owner_available=actor.get('owner_available') is True, owner_pid=actor.get('owner_pid'),
            command_epoch=actor.get('command_epoch'), runtime_nonce=actor.get('runtime_nonce'),
            sampling_actor_scope='identity at camera sampling, not encode/HTTP completion',
            pair_id=f"{feedback['source_clock_id']}:{name}:{feedback['physics_step']}",
            frame_readback=dict(**source_fields, qpos=list(feedback['qpos']), qvel=list(feedback['qvel']),
                sample_wall_ns=feedback['sample_wall_ns'], environment='fake', simulation_only=True,
                evidence_source=feedback['evidence_source'], owner_feedback_event=False),
            note='zero-distortion simulated raw pinhole; separate Owner history pairing is required',
            data_valid=True, camera_sample_hz=30,capture_diagnostics=unsupported())
        token=packet['recording_tokens'].get(name)
        if token:
            lease=self.channels[name].current_lease()
            if lease is None or not self.recording_guard(lease):token=None
        response=packet['camera_service'].publish(name,status,rgb.tobytes(),quantized.astype('<u2',copy=False).tobytes(),
            width=k['width'],height=k['height'],sampled_token=token)
        if self.channels:
            with self.lock:self.channel_status[name]=response['channel']
            self.channels[name].cached_snapshot=response['channel']
            return dict(encode_wall_s=0.,lease_check_wall_s=0.,spool_write_wall_s=0.,
                total_wall_s=time.monotonic()-started,transport='unix_rgbd',recorded=response['recorded'])
        spool = self.spools[name]
        lease_started=time.monotonic()
        error = self._lease_drop_error(spool)
        spool_started=time.monotonic()
        if error:status.update(data_valid=False, recording_error=error)
        with self.lock:status['dropped_preview_batches'] = self.dropped_batches
        try:
            encoded=self.frames[name].snapshot(formats=('color_png','depth_png'))
            spool.write(status,encoded['color_png'],encoded['depth_png'])
        except Exception as exc:
            error = error or str(exc)
        error = error or status.get('recording_error')
        spool_finished=time.monotonic()
        if error:
            spool.error = error
            status.update(data_valid=False, recording_enabled=False, recording_error=error)
        self.frames[name].publish(status=status)
        return dict(encode_wall_s=0.,lease_check_wall_s=spool_started-lease_started,
                    spool_write_wall_s=spool_finished-spool_started,total_wall_s=time.monotonic()-started)

    def snapshot(self):
        with self.lock:
            return dict(source_clock_id=self.clock_id, last_submitted_physics_step=self.last_step,
                        dropped_batches=self.dropped_batches, error=self.error,
                        queue_size=self.queue.qsize(), queue_capacity=self.queue.maxsize,worker_alive=self.worker.is_alive(),
                        last_drop=deepcopy(self.last_drop),last_publish=deepcopy(self.last_publish),
                        maximum_queue_wait_s=self.maximum_queue_wait_s,maximum_publish_s=self.maximum_publish_s,
                        recording_channels=deepcopy(self.channel_status))

    def _fresh_frames(self,service,timeout_s):
        deadline=time.monotonic()+timeout_s
        while True:
            try:
                frames={camera:service.latest_rgbd(camera) for camera in self.frames}
                if any(frame['metadata'].get('source_clock_id')!=self.clock_id for frame in frames.values()):
                    raise ValueError('camera frames belong to another plant clock')
                return frames
            except ValueError as error:
                if str(error)!='ValueError: camera has no raw RGB-D pair yet':raise
            if threading.get_ident()==self.thread_id:
                raise RuntimeError('camera startup must leave the PhysX main thread free to sample')
            if time.monotonic()>=deadline:raise TimeoutError('camera service has no complete fresh RGB-D first-frame set')
            time.sleep(.02)

    def start(self,progress=lambda phase, fraction=None, detail=None:None,*,timeout_s=20.):
        if self.ros_plane is not None:
            if self.paused or self.ros.closed:return self.restart(progress,timeout_s=timeout_s)
            return self._ros_ready(timeout_s)
        if (self.service_owner.process.poll() is not None or self.paused or
                self.common.descriptor['instance_id']!=self.service_owner.descriptor['instance_id']):
            return self.restart(progress,timeout_s=timeout_s)
        progress('verifying_camera_frames')
        cameras={}
        for camera,frame in self._fresh_frames(self.common,timeout_s).items():
            meta=frame['metadata']
            cameras[camera]=dict(capture_instance_id=meta['capture_instance_id'],serial=meta['serial'],
                frame_number=meta['frame_number'],pair_id=meta['pair_id'],host_receive_ns=meta['host_receive_ns'],
                source_age_s=frame['source_age_s'],rgb_bytes=len(frame['rgb']),depth_bytes=len(frame['depth']),
                clock={key:meta[key] for key in ('environment','clock_domain','source_clock_id')},healthy=True)
        return dict(already_running=True,instance_id=self.common.descriptor['instance_id'],
                    service_pid=self.service_owner.process.pid,cameras=cameras)

    def stop(self,progress=lambda phase, fraction=None, detail=None:None,*,timeout_s=20.):
        """Stop our CPU camera service under the caller's device maintenance gate.

        Sampling stays paused after a close request, including an unknown exit.
        The encoder, plant clock and control connection remain caller-owned.
        """
        if self.ros_plane is not None:return self._ros_maintenance(False,progress,timeout_s)
        if threading.get_ident()==self.thread_id:
            raise RuntimeError('camera stop must leave the PhysX main thread free to sample')
        if type(timeout_s) not in (int,float) or not 0<timeout_s<=120:
            raise ValueError('camera stop requires a bounded positive drain timeout')
        if not self.channels:raise ValueError('camera stop requires common memory recording channels')
        if not self.restart_lock.acquire(blocking=False):raise ValueError('camera maintenance is already running')
        was_paused=self.paused;close_requested=False
        try:
            if self.closed.is_set():raise ValueError('camera producer is closed')
            if self.common.reconnect_state()['recording_leases']:
                raise ValueError('cannot stop cameras while a recording lease remains')
            progress('pausing_camera_samples')
            with self.sampling_lock:self.paused=True
            deadline=time.monotonic()+timeout_s
            with self.queue.all_tasks_done:
                while self.queue.unfinished_tasks:
                    remaining=deadline-time.monotonic()
                    if remaining<=0:raise TimeoutError('camera publishing did not become idle; old service retained')
                    self.queue.all_tasks_done.wait(remaining)
            owner=self.service_owner;already_stopped=owner.process.poll() is not None
            if not already_stopped:
                state=owner.snapshot()
                if any(row['run_id'] for row in state['recording_channels'].values()):
                    raise ValueError('cannot stop cameras while a recording lease remains')
            if self.common.reconnect_state()['recording_leases']:
                raise ValueError('cannot stop cameras while a recording lease remains')
            progress('stopping_camera_service')
            close_requested=True
            if not already_stopped:owner.close()
            if owner.process.poll() is None:
                raise TimeoutError('camera service exit remains unknown; sampling stays paused')
            return dict(stopped=True,exit_verified=True,already_stopped=already_stopped,
                        instance_id=owner.descriptor['instance_id'],service_pid=owner.process.pid,
                        source_clock_id=self.clock_id,preview_ports=self.ports,
                        plant_preserved=True,control_reconnected=False)
        except BaseException:
            if not close_requested:
                with self.sampling_lock:self.paused=was_paused
            raise
        finally:self.restart_lock.release()

    def restart(self,progress=lambda phase, fraction=None, detail=None:None,*,timeout_s=20.):
        """Replace only our CPU service; the caller holds the device maintenance gate.

        Run on the maintenance worker: the existing main loop alone samples plant
        frames after resume. Each queued packet retains its original service target.
        """
        if self.ros_plane is not None:return self._ros_maintenance(True,progress,timeout_s)
        from robot_workbench.rgbd_service import CommonCameraService
        if threading.get_ident()==self.thread_id:
            raise RuntimeError('camera restart must leave the PhysX main thread free to sample')
        if not self.channels:raise ValueError('camera restart requires common memory recording channels')
        if not self.restart_lock.acquire(blocking=False):raise ValueError('camera restart is already running')
        replacement=None;old_stopped=False;was_paused=self.paused
        try:
            if self.closed.is_set():raise ValueError('camera producer is closed')
            if self.common.reconnect_state()['recording_leases']:
                raise ValueError('cannot restart cameras while a recording lease remains')
            progress('pausing_camera_samples')
            with self.sampling_lock:self.paused=True
            deadline=time.monotonic()+timeout_s
            with self.queue.all_tasks_done:
                while self.queue.unfinished_tasks:
                    remaining=deadline-time.monotonic()
                    if remaining<=0:raise TimeoutError('camera publishing did not become idle; old service retained')
                    self.queue.all_tasks_done.wait(remaining)
            old=self.service_owner
            if old.process.poll() is None:
                state=old.snapshot()
                if any(row['run_id'] for row in state['recording_channels'].values()):
                    raise ValueError('cannot restart cameras while a recording lease remains')
            if self.common.reconnect_state()['recording_leases']:
                raise ValueError('cannot restart cameras while a recording lease remains')
            if not self.clock_id:raise ValueError('camera source clock must be sampled before restart')
            progress('starting_camera_service')
            replacement=CommonCameraService.start(self.intrinsics,
                self.root.parent/('camera-'+secrets.token_hex(6)),'fake',
                budget_bytes=old.descriptor['budget_bytes'])
            old.close();old_stopped=True;self.service_owner=replacement
            if self.ports is not None:
                if replacement.start_servers(self.ports)!=self.ports:
                    raise ValueError('replacement camera preview ports changed')
            staged=self.descriptor_path.with_name('.camera-service-'+secrets.token_hex(6)+'.json')
            try:
                with os.fdopen(os.open(staged,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600),'w') as stream:
                    json.dump(replacement.descriptor,stream,allow_nan=False)
                os.replace(staged,self.descriptor_path)
            finally:staged.unlink(missing_ok=True)
            progress('waiting_for_fresh_camera_frames')
            with self.sampling_lock:
                self.publisher=replacement;self.publish_generation+=1;self.paused=False
            self._fresh_frames(replacement,timeout_s)
            progress('rebinding_camera_clients')
            deadline=time.monotonic()+timeout_s
            while True:
                try:receipt=self.common.reconnect();break
                except ValueError as error:
                    if str(error)!='camera operations are active; reconnect requires idle clients' or time.monotonic()>=deadline:
                        raise
                    time.sleep(.02)  # Only idle admission retries; no IPC mutation is replayed.
            with self.sampling_lock:self.publisher=self.common
            with self.lock:self.error=None;self.channel_status={}
            return dict(receipt,source_clock_id=self.clock_id,preview_ports=self.ports,
                        plant_preserved=True,control_reconnected=False)
        except BaseException:
            if not old_stopped:
                if replacement is not None:replacement.close()
                with self.sampling_lock:self.paused=was_paused
            else:
                with self.sampling_lock:self.paused=True
            raise
        finally:self.restart_lock.release()

    def _ros_ready(self,timeout_s):
        deadline=time.monotonic()+timeout_s
        while time.monotonic()<deadline:
            try:
                cameras={name:self.common.latest_rgbd(name)for name in CAMERAS.values()}
                if all(frame['metadata']['capture_instance_id']==self.descriptor['instance_id']for frame in cameras.values()):
                    return dict(already_running=True,instance_id=self.descriptor['instance_id'],service_pid=os.getpid(),
                        source_available=True,process_preserved=True,cameras={name:dict(healthy=True,
                            capture_instance_id=frame['metadata']['capture_instance_id'])for name,frame in cameras.items()})
            except (LookupError,ValueError):pass
            if threading.get_ident()==self.thread_id:raise RuntimeError('camera readiness must leave the PhysX main thread free to sample')
            time.sleep(.01)
        raise TimeoutError('ROS camera source has no complete fresh RGB-D frame set')

    def _ros_maintenance(self,restart,progress,timeout_s):
        if threading.get_ident()==self.thread_id:raise RuntimeError('camera maintenance must leave the PhysX main thread free to sample')
        if type(timeout_s)not in(int,float)or not 0<timeout_s<=120:raise ValueError('bounded camera drain timeout required')
        if not self.restart_lock.acquire(blocking=False):raise ValueError('camera maintenance is already running')
        try:
            owner=getattr(self._backend,'owner',None);recording=getattr(owner,'recording',None)
            if recording is not None and not recording.done.is_set():raise ValueError('cannot stop cameras while recording is active')
            if self.closed.is_set():raise ValueError('camera producer is closed')
            progress('pausing_camera_samples')
            with self.sampling_lock:self.paused=True
            deadline=time.monotonic()+timeout_s
            with self.queue.all_tasks_done:
                while self.queue.unfinished_tasks:
                    remaining=deadline-time.monotonic()
                    if remaining<=0:raise TimeoutError('camera publishing has not drained; source retained')
                    self.queue.all_tasks_done.wait(remaining)
            old=self.descriptor['instance_id'];self._close_ros()
            if not restart:return dict(stopped=True,exit_verified=False,source_available=False,
                process_preserved=True,service_pid=os.getpid(),instance_id=old,source_clock_id=self.clock_id)
            progress('starting_camera_source');self._open_ros();self.publish_generation+=1
            with self.sampling_lock:self.paused=False
            self.common.reconnect();result=self._ros_ready(timeout_s)
            with self.lock:self.error=None
            return dict(result,reconnected=True,previous_instance_id=old,source_clock_id=self.clock_id,
                plant_preserved=True,control_reconnected=False,preview_ports=self.ports)
        finally:self.restart_lock.release()

    def close(self):
        self.closed.set()
        self.worker.join(timeout=5)
        self.encoders.shutdown(wait=False, cancel_futures=True)
        if self.ros_plane is not None:
            if self.worker.is_alive():raise TimeoutError('ROS camera source worker still draining')
            self._close_ros();self.common.close()
        else:self.service_owner.close()
