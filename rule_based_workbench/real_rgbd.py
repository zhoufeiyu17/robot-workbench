"""RealSense raw ingress to the shared camera backend; one SDK owner per serial."""
import argparse
import io
import json
import math
from pathlib import Path
import signal
import sys
import shutil
import uuid
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit
import zipfile


def paired_timestamps(color_ms,depth_ms,color_domain,depth_domain,fps):
    return (color_domain==depth_domain and all(math.isfinite(x)and x>=0 for x in (color_ms,depth_ms))
        and abs(color_ms-depth_ms)<=1000/fps)


def pair_metadata(*, serial, camera_name, color_number, depth_number,
                  color_ms, depth_ms, color_domain, depth_domain,
                  host_receive_ns, depth_scale_m, intrinsics, fps=30):
    if (not serial or not all(type(n) is int and n >= 0 for n in (color_number, depth_number))
            or not all(math.isfinite(x) and x >= 0 for x in (color_ms, depth_ms))
            or not math.isfinite(depth_scale_m) or depth_scale_m <= 0
            or not math.isfinite(fps) or fps <= 0 or host_receive_ns <= 0):
        raise ValueError('invalid RGB-D frame metadata')
    if not paired_timestamps(color_ms,depth_ms,color_domain,depth_domain,fps):
        raise ValueError('RGB and depth timestamps do not form a contemporaneous pair')
    return dict(ready=True, serial=serial, camera_name=camera_name,
                frame_number=color_number, depth_frame_number=depth_number,
                pair_id=f'{serial}:{color_number}:{depth_number}:{host_receive_ns}',
                host_receive_ns=str(host_receive_ns), device_timestamp_ms=color_ms,
                depth_device_timestamp_ms=depth_ms, timestamp_domain=color_domain,
                pair_skew_ms=abs(color_ms-depth_ms), capture_fps=fps,
                intrinsics=intrinsics, depth_scale_m=depth_scale_m,
                depth_aligned_to='color', depth_encoding='uint16_png',
                depth_zero_is_invalid=True, raw=True, annotated=False,
                image_domain='raw', rgbd_paired=True, synchronized=False,
                joint_feedback_available=False, calibration_sample=False,
                motion_commanded=False, timestamp_kind='host_receive')


def snapshot_bundle(metadata, color_png, depth_png):
    """Single response/record is the pairing boundary; separate GETs are not."""
    buffer = io.BytesIO()
    with zipfile.ZipFile(buffer, 'w', compression=zipfile.ZIP_STORED) as archive:
        archive.writestr('metadata.json', json.dumps(metadata, allow_nan=False))
        archive.writestr('color.png', color_png)
        archive.writestr('depth.png', depth_png)
    return buffer.getvalue()


class RawSpool:
    """Local exclusive recording lease; idle preview never saves frame files."""
    def __init__(self, root, *, clock_ns=time.time_ns, min_free_bytes=2*1024**3,lease_guard=None):
        self.root = Path(root)
        self.clock_ns, self.min_free_bytes = clock_ns, min_free_bytes
        self.lease_guard=lease_guard
        self.active = None
        self.count = self.size = 0
        self.error = None
        (self.root/'recording-service.json').write_text(json.dumps({'schema': 'rgbd_lease.v1'}))

    def write(self, status, color, depth):
        request = self.root/'recording-request.json'
        try:lease=json.loads(request.read_text())
        except FileNotFoundError:lease=None
        now = self.clock_ns()
        if lease:
            token = lease.get('run_id')
            start, end = lease.get('start_wall_ns'), lease.get('deadline_wall_ns')
            admission=lease.get('admission_start_wall_ns',start)
            if (not isinstance(token, str) or len(token) != 32 or any(c not in '0123456789abcdef' for c in token)
                    or type(start) is not int or type(end) is not int or not 0 < end-start <= 600_000_000_000
                    or type(admission) is not int or not 0<admission<=start):
                raise ValueError('invalid bounded recording lease')
            if not start <= now < end:
                lease = None
            elif self.lease_guard is not None and (not self.lease_guard(lease)
                    or int(status['host_receive_ns'])<admission
                    or any(status.get(key)!=value for key,value in lease.get('execution_owner',{}).items())):
                # Fake-only permission at consumption, including delayed encodes
                # and an os.link which completed after its execution was revoked.
                lease=None
        if lease and self.active != lease['run_id']:
            self.active = lease['run_id']
            self.count = self.size = 0
            self.error = None
            (self.root/self.active).mkdir(exist_ok=False)
        if not lease:
            self.active = None
        status.update(recording_enabled=bool(lease) and not self.error,
                      recorded_frames=self.count, recording_error=status.get('recording_error') or self.error,
                      recording_run_id=self.active,
                      recording_duration_s=(lease['deadline_wall_ns']-lease['start_wall_ns'])/1e9 if lease else 0,
                      recording_deadline_wall_ns=lease['deadline_wall_ns'] if lease else None)
        if lease and not self.error:
            blob = snapshot_bundle(status, color, depth)
            if shutil.disk_usage(self.root).free < self.min_free_bytes+len(blob) or self.size+len(blob)>4*1024**3:
                self.error = 'raw recording disk reserve or 4 GiB session limit reached'
                status.update(recording_enabled=False, recording_error=status.get('recording_error') or self.error)
                return
            self.count += 1
            destination = self.root/self.active/f'{self.count:09d}.zip'
            temporary = destination.with_suffix('.tmp')
            temporary.write_bytes(blob)
            temporary.replace(destination)
            self.size += len(blob)
            status['recorded_frames'] = self.count


class Frames:
    def __init__(self):
        self.condition = threading.Condition()
        self.latest = {'status': {'ready': False}}
        self.closed = False
        self.stop_requested = threading.Event()
        self.stop_signal = None

    def request_stop(self, signum, _frame=None):
        self.stop_signal = signum
        self.stop_requested.set()

    def publish(self, **frame):
        with self.condition:
            self.latest = frame
            self.condition.notify_all()

    def snapshot(self,formats=None):
        with self.condition:
            frame = dict(self.latest)
            status = dict(frame['status'])
        if status.get('host_receive_ns'):
            status['source_age_s'] = max(0, (time.time_ns()-int(status['host_receive_ns']))/1e9)
            if status['source_age_s'] > 2:
                status.update(ready=False, error='camera frame stale')
        return {**frame, 'status': status}


def make_handler(frames):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *unused):
            pass

        def do_GET(self):
            path = urlsplit(self.path).path
            if path == '/stream.mjpg':
                self.connection.settimeout(3)
                self.send_response(200)
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                previous = None
                try:
                    while not frames.closed:
                        with frames.condition:
                            frames.condition.wait_for(lambda: frames.closed or frames.latest['status'].get('pair_id') != previous, 2)
                        frame = frames.snapshot(formats=('jpeg',))
                        status = frame['status']
                        if not status.get('ready') or status.get('pair_id') == previous:
                            return
                        previous = status['pair_id']
                        jpeg = frame['jpeg']
                        self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                                         + str(len(jpeg)).encode() + b'\r\nX-Camera-Metadata: '
                                         + json.dumps(status, separators=(',', ':')).encode()
                                         + b'\r\n\r\n' + jpeg + b'\r\n')
                        self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError, TimeoutError):
                    pass
                return
            formats=(() if path=='/status' else ('color_png','depth_png') if path=='/snapshot.zip' else
                     ('depth_png',) if path=='/depth.png' else ('jpeg',))
            frame = frames.snapshot(formats=formats)
            status = frame['status']
            if path == '/status':
                payload, mime = json.dumps(status).encode(), 'application/json'
            elif path in ('/frame.jpg', '/raw-frame.jpg', '/depth.png', '/snapshot.zip'):
                if not status.get('ready'):
                    self.send_error(503, 'RGB-D frame unavailable or stale')
                    return
                if path == '/snapshot.zip':
                    payload, mime = snapshot_bundle(status, frame['color_png'], frame['depth_png']), 'application/zip'
                elif path == '/depth.png':
                    payload, mime = frame['depth_png'], 'image/png'
                else:
                    payload, mime = frame['jpeg'], 'image/jpeg'
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header('Content-Type', mime)
            self.send_header('Content-Length', str(len(payload)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Camera-Metadata', json.dumps(status, separators=(',', ':')))
            self.end_headers()
            try:
                self.wfile.write(payload)
            except (BrokenPipeError, ConnectionResetError):
                pass
    return Handler


class FrameProgress:
    """Skip reused framesets, but reject counter rollback or a stalled pair."""
    def __init__(self, now):
        self.seen = self.published = None
        self.last_fresh = now
        self.last_rejection=None

    def accept(self, color, depth, now, *, timestamps=None):
        pair = (color, depth)
        if self.seen is not None and any(a < b for a, b in zip(pair, self.seen)):
            raise RuntimeError('camera frame number regressed')
        self.seen = pair
        if now - self.last_fresh >= 3:
            raise RuntimeError('camera produced no fresh RGB-D pair for 3 seconds')
        if timestamps is not None and not paired_timestamps(*timestamps):
            self.last_rejection='timestamp_mismatch'
            return False
        if self.published is not None and any(a == b for a, b in zip(pair, self.published)):
            self.last_rejection='reused_pair'
            return False
        self.last_rejection=None
        self.published, self.last_fresh = pair, now
        return True


def _frame_counters(pair):
    rgb,z=pair.get_color_frame(),pair.get_depth_frame()
    return dict(rgb_number=rgb.get_frame_number() if rgb else None,
        depth_number=z.get_frame_number() if z else None,
        rgb_device_ms=rgb.get_timestamp() if rgb else None,
        depth_device_ms=z.get_timestamp() if z else None)


def capture(args, frames):
    import cv2
    import numpy as np
    import pyrealsense2 as rs
    import fcntl
    from robot_workbench.rgbd_service import CommonCameraService,CAMERAS
    from robot_workbench.capture_diagnostics import CaptureDiagnostics
    service=CommonCameraService.connect(args.service_descriptor)
    if service.descriptor['environment']!='physical':raise ValueError('physical camera service required')
    camera=CAMERAS[args.camera_name]
    if service.descriptor['intrinsics'][camera]['serial']!=args.serial:raise ValueError('camera serial differs from shared service')
    capture_instance=uuid.uuid4().hex
    diagnostics=CaptureDiagnostics(args.output,capture_instance)
    frames.capture_diagnostics=diagnostics

    pipeline, config = rs.pipeline(), rs.config()
    config.enable_device(args.serial)
    # Same RGB mode and conversion as calibration; add depth in this producer.
    config.enable_stream(rs.stream.color, 640, 480, rs.format.yuyv, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    frames.publish(status={'ready': False, 'phase': 'starting_camera', 'serial': args.serial})
    sdk_lease=(Path(service.directories[camera])/'capture.lock').open('a')
    started=False
    try:
        fcntl.flock(sdk_lease,fcntl.LOCK_EX|fcntl.LOCK_NB)
        # Native bounded FIFO retains frames across short align/publish delays;
        # pipeline.wait_for_frames() otherwise exposes only its latest frameset.
        ingress=rs.frame_queue(32,True)
        profile=pipeline.start(config,ingress);started=True
        scale = profile.get_device().first_depth_sensor().get_depth_scale()
        align = rs.align(rs.stream.color)
        last_color = last_depth = None
        color_missing = depth_missing = count = 0
        for warmup in range(20):
            if frames.stop_requested.is_set():
                return
            frames.publish(status={'ready': False, 'phase': 'warming_up', 'serial': args.serial,
                                   'warmup_frames': warmup, 'warmup_target_frames': 20})
            ingress.wait_for_frame(3000).as_frameset()
        rate_start = time.monotonic()
        progress = FrameProgress(rate_start)
        reused_framesets = 0
        deadline = rate_start+args.duration_s if args.duration_s else math.inf
        while not frames.stop_requested.is_set() and time.monotonic() < deadline:
            sampled_token=service.channels[camera].token()
            diagnostics.active_stage='wait'
            before_wait=time.monotonic_ns()
            try:
                received = ingress.wait_for_frame(3000).as_frameset()
                stamp = time.time_ns()
            finally:
                waited=time.monotonic_ns()-before_wait;diagnostics.stage('wait',waited)
            raw_counters=_frame_counters(received)
            diagnostics.active_stage='align'
            before_align=time.monotonic_ns()
            try:aligned = align.process(received)
            finally:
                aligned_ns=time.monotonic_ns()-before_align;diagnostics.stage('align',aligned_ns)
            color, depth = aligned.get_color_frame(), aligned.get_depth_frame()
            if not color or not depth:
                diagnostics.attempt(raw_counters,_frame_counters(aligned),False,'missing_aligned_frame',host_receive_ns=stamp)
                raise RuntimeError('missing aligned RGB-D frame')
            number, depth_number = color.get_frame_number(), depth.get_frame_number()
            diagnostics.active_stage='pair_filter'
            try:accepted=progress.accept(number,depth_number,time.monotonic(),timestamps=(
                color.get_timestamp(),depth.get_timestamp(),str(color.get_frame_timestamp_domain()),
                str(depth.get_frame_timestamp_domain()),30))
            except RuntimeError:
                diagnostics.attempt(raw_counters,_frame_counters(aligned),False,'counter_regressed_or_stalled',host_receive_ns=stamp)
                raise
            diagnostics.attempt(raw_counters,_frame_counters(aligned),accepted,'accepted' if accepted else progress.last_rejection,host_receive_ns=stamp,
                stage_ns=dict(wait=waited,align=aligned_ns))
            if not accepted:
                if progress.last_rejection=='reused_pair':reused_framesets += 1
                elif sampled_token:
                    service.channels[camera].fail('RGB-D timestamp mismatch; invalid recording pair',sampled_token=sampled_token)
                diagnostics.active_stage=None
                continue
            if last_color is not None:
                color_missing += max(0, number-last_color-1)
                depth_missing += max(0, depth_number-last_depth-1)
            last_color, last_depth = number, depth_number
            intr = color.profile.as_video_stream_profile().get_intrinsics()
            intrinsic = dict(serial=args.serial,width=intr.width, height=intr.height, fx=intr.fx, fy=intr.fy,
                             cx=intr.ppx, cy=intr.ppy, distortion_model=str(intr.model),
                             coefficients=list(intr.coeffs), rectified=False)
            status = pair_metadata(serial=args.serial, camera_name=args.camera_name,
                                   color_number=number, depth_number=depth_number,
                                   color_ms=color.get_timestamp(), depth_ms=depth.get_timestamp(),
                                   color_domain=str(color.get_frame_timestamp_domain()),
                                   depth_domain=str(depth.get_frame_timestamp_domain()),
                                   host_receive_ns=stamp, depth_scale_m=scale, intrinsics=intrinsic)
            before_convert=time.monotonic_ns()
            diagnostics.active_stage='convert'
            raw = np.asanyarray(color.get_data()).view(np.uint8).reshape(intr.height, intr.width, 2)
            rgb = cv2.cvtColor(raw, cv2.COLOR_YUV2RGB_YUY2)
            depth_array = np.asanyarray(depth.get_data())
            if depth_array.dtype != np.uint16 or depth_array.shape != rgb.shape[:2]:
                raise RuntimeError('depth must be aligned uint16 with color dimensions')
            rgb_payload=rgb.tobytes();depth_payload=depth_array.astype('<u2',copy=False).tobytes()
            diagnostics.stage('convert',time.monotonic_ns()-before_convert)
            diagnostics.active_stage=None
            count += 1
            status.update(phase='capturing', reused_framesets=reused_framesets, delivered_fps=count/(time.monotonic()-rate_start),
                color_missing_frames=color_missing,depth_missing_frames=depth_missing,
                capture_instance_id=capture_instance,environment='physical',simulation_only=False,
                evidence_source='physical_rgbd',clock_domain='host_wall',source_clock_id=None,
                host_receive_ns=stamp,data_valid=True,capture_diagnostics=diagnostics.summary())
            event=diagnostics.take_event()
            if event:status['capture_gap_evidence']=event
            before_publish=time.monotonic_ns()
            diagnostics.active_stage='publish'
            try:
                service.publish(camera,status,rgb_payload,depth_payload,
                    width=intr.width,height=intr.height,sampled_token=sampled_token)
            finally:
                diagnostics.stage('publish',time.monotonic_ns()-before_publish)
                del rgb_payload,depth_payload
            diagnostics.active_stage=None
    finally:
        stop_reason='capture_exception' if sys.exc_info()[0] else 'stop_requested' if frames.stop_requested.is_set() else 'duration_complete'
        try:
            if started:pipeline.stop()
        finally:
            diagnostics.close(reason=stop_reason)
            sdk_lease.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--serial', required=True)
    parser.add_argument('--camera-name', required=True, choices=('head', 'left', 'right'))
    parser.add_argument('--port', type=int,default=None)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration-s', type=float, default=0)
    parser.add_argument('--service-descriptor',type=Path,required=True,
                        help='the same shared three-camera service descriptor for all serials')
    args = parser.parse_args()
    if (not args.serial.isascii() or not args.serial.isdecimal() or args.port is not None and not 1024 <= args.port <= 65535
            or not math.isfinite(args.duration_s) or args.duration_s < 0):
        parser.error('decimal serial, unprivileged port and nonnegative duration required')
    args.output.mkdir(parents=True, exist_ok=False)
    from robot_workbench.rgbd_service import CommonCameraService,CAMERAS
    service=CommonCameraService.connect(args.service_descriptor)
    ports=service.start_servers({role:args.port if role==args.camera_name and args.port else 0 for role in CAMERAS})
    if args.port and ports[args.camera_name]!=args.port:raise ValueError('preview port differs from shared camera service')
    frames=service.frames[CAMERAS[args.camera_name]]
    previous_handlers = {sig: signal.signal(sig, frames.request_stop) for sig in (signal.SIGTERM, signal.SIGINT)}
    try:
        capture(args, frames)
    except Exception as exc:
        frames.publish(status={**frames.snapshot()['status'], 'ready': False,
                               'error': f'{type(exc).__name__}: {exc}'})
        raise
    finally:
        diagnostics=getattr(frames,'capture_diagnostics',None)
        if diagnostics is not None:diagnostics.close()
        final = {**frames.snapshot()['status'], 'ready': False, 'phase': 'stopped',
                 'stop_signal': frames.stop_signal, 'stopped_wall_ns': str(time.time_ns())}
        if diagnostics is not None:final['capture_diagnostics']=diagnostics.summary()
        frames.publish(status=final)
        (args.output/'status.json').write_text(json.dumps(final, indent=2))
        for sig, handler in previous_handlers.items():
            signal.signal(sig, handler)


if __name__ == '__main__':
    main()
