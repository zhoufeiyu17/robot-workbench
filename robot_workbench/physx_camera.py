"""Three cached RGB-D streams from one caller-owned PhysX plant.

Only collect() may touch the plant, on its main thread. HTTP and encoding never
step physics; preview frames stay in memory and recording uses bounded leases.
"""
from collections import deque
from concurrent.futures import ThreadPoolExecutor, wait
from copy import deepcopy
from http.server import ThreadingHTTPServer
import io
import json
from pathlib import Path
import queue
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
    def __init__(self, intrinsics, root, *, queue_capacity=8, min_free_bytes=2*1024**3,recording_guard=None):
        from real_rgbd import Frames, RawSpool
        if set(intrinsics) != set(CAMERAS.values()) or type(queue_capacity) is not int or queue_capacity < 1:
            raise ValueError('three cameras and a positive bounded queue required')
        self.intrinsics = deepcopy(intrinsics)
        self.root = Path(root)
        self.frames, self.spools, self.directories = {}, {}, {}
        for name in CAMERAS.values():
            path = self.root/name
            path.mkdir(parents=True, exist_ok=False)
            self.frames[name] = Frames()
            self.spools[name] = RawSpool(path, min_free_bytes=min_free_bytes,lease_guard=recording_guard)
            self.directories[name] = str(path)
        self.queue = queue.Queue(maxsize=queue_capacity)
        self.closed = threading.Event()
        self.thread_id = threading.get_ident()
        self.lock = threading.Lock()
        self.drops = deque(maxlen=512)
        self.drop_history_before_ns = 0
        self.dropped_batches = 0
        self.error = None
        self.last_step = None
        self.frame_sequence = 0
        self.clock_id = None
        self.servers = []
        self.encoders = ThreadPoolExecutor(max_workers=3, thread_name_prefix='rgbd-camera')
        self.worker = threading.Thread(target=self._encode_loop, daemon=True, name='physx-rgbd-encoder')
        self.worker.start()

    def start_servers(self, ports=None):
        from real_rgbd import make_handler
        ports = ports or dict.fromkeys(CAMERAS, 0)
        if set(ports) != set(CAMERAS):
            raise ValueError('head/left/right loopback ports required')
        result = {}
        for role, name in CAMERAS.items():
            server = ThreadingHTTPServer(('127.0.0.1', ports[role]), make_handler(self.frames[name]))
            server.daemon_threads = True
            threading.Thread(target=server.serve_forever, daemon=True, name='physx-camera-'+role).start()
            self.servers.append(server)
            result[role] = server.server_port
        return result

    def collect(self, plant, backend, actor):
        """Call after this step's native render, never from an HTTP handler."""
        if threading.get_ident() != self.thread_id:
            raise RuntimeError('only the PhysX main thread may sample cameras')
        if self.closed.is_set():
            return False
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
                      images={name: {key: values[key].copy() for key in ('rgb','depth')}
                              for name, values in images.items()})
        self.frame_sequence += 1
        try:
            self.queue.put_nowait(packet)
            return True
        except queue.Full:
            self._record_drop(sampled_wall_ns)
            return False

    def _record_drop(self, wall_ns):
        with self.lock:
            if len(self.drops) == self.drops.maxlen:self.drop_history_before_ns = max(self.drop_history_before_ns,self.drops[0])
            self.drops.append(wall_ns)
            self.dropped_batches += 1

    def _lease_drop_error(self, spool):
        request = spool.root/'recording-request.json'
        try:lease=json.loads(request.read_text())
        except FileNotFoundError:return None
        if spool.lease_guard is not None and not spool.lease_guard(lease):return None
        start, end = lease.get('admission_start_wall_ns',lease.get('start_wall_ns')), lease.get('deadline_wall_ns')
        if type(start) is not int or type(end) is not int:return 'invalid recording lease'
        with self.lock:
            # ponytail: bounded drop history; older leases are unverifiable rather than silently complete.
            dropped = start <= self.drop_history_before_ns or any(start <= stamp < end for stamp in self.drops)
        return 'camera source queue overflow during recording lease' if dropped else None

    def _encode_loop(self):
        while not self.closed.is_set() or not self.queue.empty():
            try:packet = self.queue.get(timeout=.1)
            except queue.Empty:continue
            try:self._publish(packet)
            except Exception as error:
                self._record_drop(packet['wall_ns'])
                with self.lock:self.error = f'{type(error).__name__}: {error}'
                for frame in self.frames.values():
                    frame.publish(status=dict(ready=False, data_valid=False, error=self.error))
            finally:self.queue.task_done()

    def _publish(self, packet):
        # Each camera keeps its own ordered spool; three encoders share only the
        # immutable sampled packet. No GPU/physics work moves off the main thread.
        futures = [self.encoders.submit(self._publish_camera, packet, role, name)
                   for role, name in CAMERAS.items()]
        wait(futures)
        for future in futures:future.result()

    def _publish_camera(self, packet, role, name):
        import numpy as np
        from PIL import Image
        from real_rgbd import pair_metadata
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
        png, zpng, jpeg = io.BytesIO(), io.BytesIO(), io.BytesIO()
        Image.fromarray(rgb).save(png, format='PNG', compress_level=1)
        Image.fromarray(quantized).save(zpng, format='PNG', compress_level=1)
        Image.fromarray(rgb).save(jpeg, format='JPEG', quality=95, subsampling=0)
        number = packet['frame_sequence']
        ms = feedback['source_time_ns']/1e6
        status = pair_metadata(serial=k['serial'], camera_name=role, color_number=number,
            depth_number=number, color_ms=ms, depth_ms=ms, color_domain='simulation',
            depth_domain='simulation', host_receive_ns=packet['wall_ns'], depth_scale_m=.001,
            intrinsics=k, fps=30)
        status.update(**source_fields, host_receive_ns=packet['wall_ns'], timestamp_kind='simulation_step',
            environment='fake', simulation_only=True, evidence_source='fake_device_rgbd',
            owner_available=actor.get('owner_available') is True, owner_pid=actor.get('owner_pid'),
            command_epoch=actor.get('command_epoch'), runtime_nonce=actor.get('runtime_nonce'),
            sampling_actor_scope='identity at camera sampling, not encode/HTTP completion',
            pair_id=f"{feedback['source_clock_id']}:{name}:{feedback['physics_step']}",
            frame_readback=dict(**source_fields, qpos=list(feedback['qpos']), qvel=list(feedback['qvel']),
                sample_wall_ns=feedback['sample_wall_ns'], environment='fake', simulation_only=True,
                evidence_source=feedback['evidence_source'], owner_feedback_event=False),
            note='zero-distortion simulated raw pinhole; separate Owner history pairing is required',
            data_valid=True, camera_sample_hz=30)
        spool = self.spools[name]
        error = self._lease_drop_error(spool)
        if error:status.update(data_valid=False, recording_error=error)
        try:
            spool.write(status, png.getvalue(), zpng.getvalue())
        except Exception as exc:
            error = str(exc)
        if error:
            spool.error = error
            status.update(data_valid=False, recording_enabled=False, recording_error=error)
        with self.lock:status['dropped_preview_batches'] = self.dropped_batches
        self.frames[name].publish(status=status, jpeg=jpeg.getvalue(),
                                 color_png=png.getvalue(), depth_png=zpng.getvalue())

    def snapshot(self):
        with self.lock:
            return dict(source_clock_id=self.clock_id, last_submitted_physics_step=self.last_step,
                        dropped_batches=self.dropped_batches, error=self.error,
                        queue_size=self.queue.qsize(), worker_alive=self.worker.is_alive())

    def close(self):
        self.closed.set()
        self.worker.join(timeout=5)
        self.encoders.shutdown(wait=False, cancel_futures=True)
        for frame in self.frames.values():
            with frame.condition:
                frame.closed = True
                frame.condition.notify_all()
        for server in self.servers:
            server.shutdown();server.server_close()
