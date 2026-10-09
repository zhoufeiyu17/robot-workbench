"""Read-only workbench view of an existing owner; no SDK or command endpoint."""
import json
import math
from pathlib import Path
import threading
import time
from urllib.request import HTTPRedirectHandler, ProxyHandler, build_opener
from robot_workbench.sample_clock import assert_progress, clock_spec, sample_stamp


def camera_process_inventory(serials, *, proc_root='/proc'):
    """Linux process evidence only; no command lines, device access or alive guesses.

    Call from a cached monitor (e.g. every two seconds), never per HTTP request.
    A serial not present in a candidate producer's argv remains unknown.
    """
    serials=list(serials)
    if any(not isinstance(serial,str) or not serial.isascii() or not serial.isdecimal() for serial in serials):
        raise ValueError('camera serials must be decimal strings')
    result={'observed_at_ns':time.time_ns(),'processes':[],
            'cameras':{serial:{'process_alive':None,'processes':[]} for serial in serials},
            'source':'linux_proc_readonly','scan_complete':True}
    try:
        entries=list(Path(proc_root).iterdir())
    except OSError:
        result.update(scan_complete=False,error='process inventory unavailable')
        return result
    producer_scripts={'camera-stream-v2.py','camera-stream.py','joint_head_real_preview.py'}
    for path in entries:
        if not path.name.isdecimal():continue
        try:
            args=(path/'cmdline').read_bytes().split(b'\0')
            text=[arg.decode('utf-8',errors='replace') for arg in args if arg]
            modules=[text[i+1] for i,arg in enumerate(text[:-1]) if arg=='-m']
            producer=any(module=='prometheus.web.preview' for module in modules) or any(Path(arg).name in producer_scripts for arg in text)
            if not producer:continue
            stat=(path/'stat').read_text();fields=stat[stat.rfind(')')+2:].split()
            if fields[0] in ('Z','X'):continue
            matched=[serial for serial in serials if serial in text or any(arg.endswith('='+serial) for arg in text)]
            row={'pid':int(path.name),'comm':(path/'comm').read_text().strip(),
                 'start_ticks':int(fields[19]),'serial_match':matched}
            result['processes'].append(row)
            for serial in matched:
                result['cameras'][serial]['process_alive']=True
                result['cameras'][serial]['processes'].append(row)
        except PermissionError:
            result['scan_complete']=False
        except (OSError,ValueError,IndexError):
            continue  # Processes may exit during this observational scan.
    result['processes'].sort(key=lambda row:row['pid'])
    return result


def _number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def normalize_owner_state(owner, *, now_ns, previous=None, max_age_s=.5, environment='physical', source_clock_id=None):
    """Host age assumes synchronized clocks; SDK stamps have separate side epochs.

    `previous` is the last normalized result, never a controller target. PID is
    the current owner's only process identity; it is not a durable boot UUID.
    """
    if not isinstance(owner, dict):
        raise ValueError('owner state must be an object')
    if environment not in ('physical','fake'):
        raise ValueError('trusted physical/fake environment required')
    if not _number(max_age_s) or max_age_s <= 0 or type(now_ns) is not int:
        raise ValueError('positive max age and integer wall clock required')
    identity = {key: owner.get(key) for key in ('pid', 'mode', 'command_epoch', 'state')}
    identity['protocol'] = owner.get('motion_protocol')
    identity.update(environment=owner.get('environment'),simulation_only=owner.get('simulation_only'),
                    evidence_source=owner.get('evidence_source'))
    reasons = []
    if type(identity['pid']) is not int or identity['pid'] <= 0:
        reasons.append('owner_pid_invalid')
    if type(identity['command_epoch']) is not int or identity['command_epoch'] < 0:
        reasons.append('command_epoch_invalid')
    if environment=='fake':
        if owner.get('mode')!='simulation' or owner.get('environment')!='fake' or owner.get('simulation_only') is not True:
            reasons.append('owner_is_not_explicit_fake')
    elif (identity['mode']!='hardware' or owner.get('environment') not in (None,'physical')
          or owner.get('simulation_only') is True or owner.get('physical_hardware_connected') is False):
        reasons.append('owner_is_not_hardware')
    if identity['protocol'] != 'reviewed-single-arm-v1':
        reasons.append('owner_protocol_incompatible')
    if owner.get('connected') is not True:
        reasons.append('owner_disconnected')
    if owner.get('feedback_healthy') is not True:
        reasons.append('owner_feedback_unhealthy')
    feedback = owner.get('calibration_feedback')
    feedback = feedback if isinstance(feedback, dict) else {}
    clock=None;source_stamp=None
    try:
        clock=clock_spec(environment,clock_domain=feedback.get('clock_domain'),
                         source_clock_id=source_clock_id or feedback.get('source_clock_id'))
        source_stamp=sample_stamp(feedback,clock,wall_key='sample_wall_ns')
        if previous and environment=='fake':
            prior=previous['feedback']
            if prior.get('source_clock_id')!=clock['source_clock_id']:
                raise ValueError('simulation source_clock_id changed')
            if type(prior.get('source_time_ns')) is int and type(prior.get('physics_step')) is int:
                assert_progress((prior['source_time_ns'],prior['physics_step']),source_stamp,allow_same=True)
    except (ValueError,KeyError,TypeError) as exc:
        reasons.append('source_clock_invalid: '+str(exc))
    q, stamps = feedback.get('qpos'), feedback.get('sdk_timestamps')
    q_valid = isinstance(q, list) and len(q) == 14 and all(_number(x) for x in q)
    sdk_valid = isinstance(stamps, list) and len(stamps) == 2 and all(_number(x) and x >= 0 for x in stamps)
    if not q_valid:
        reasons.append('raw14_qpos_invalid')
    if not sdk_valid:
        reasons.append('sdk_timestamps_invalid')
    wall = feedback.get('sample_wall_ns')
    age = (now_ns-wall)/1e9 if type(wall) is int else None
    recent = age is not None and 0 <= age <= max_age_s
    if not recent:
        reasons.append('host_sample_stale_or_clock_skew')
    sdk = dict(zip(('left', 'right'), stamps)) if sdk_valid else {'left': None, 'right': None}
    advanced = {'left': None, 'right': None}
    if previous:
        before = previous['owner']
        if any(identity[key] != before[key] for key in ('pid', 'mode', 'protocol', 'command_epoch')):
            reasons.append('owner_identity_or_epoch_changed')
        else:
            prior = previous['feedback']
            old_wall = prior['sample_wall_ns']
            if type(wall) is int and type(old_wall) is int and wall < old_wall:
                reasons.append('host_sample_time_regressed')
            for side in ('left', 'right'):
                old, new = prior['sdk_timestamps_s'][side], sdk[side]
                if old is not None and new is not None:
                    advanced[side] = new > old
                    if new < old:
                        reasons.append(f'{side}_sdk_clock_regressed')
    valid = not reasons
    return dict(
        backend='simulation' if environment=='fake' else 'hardware', environment=environment,
        source='existing_owner_raw_feedback', reachable=True,
        raw_owner=owner,
        owner=identity, q_actual_rad=q[:6]+q[7:13] if valid else None,
        jaw_positions_m=dict(left=q[6], right=q[13]) if valid else None,
        feedback=dict(valid=valid, reasons=reasons, sample_wall_ns=wall if type(wall) is int else None,
                      clock_domain=clock['clock_domain'] if clock else None,
                      source_clock_id=clock['source_clock_id'] if clock else None,
                      source_time_ns=source_stamp[0] if source_stamp and environment=='fake' else None,
                      physics_step=source_stamp[1] if source_stamp else None,
                      host_sample_age_s=age, host_sample_recent=recent,
                      sdk_timestamps_s=sdk, sdk_advanced=advanced,
                      owner_feedback_healthy=owner.get('feedback_healthy') is True,
                      per_motor_freshness=None,
                      note='Host age requires clock synchronization; per-side SDK progress is not per-motor freshness.'),
        pedal=owner.get('pedal'), fault=dict(state=identity['state'], reason=owner.get('reason'),
                                          recoverable=owner.get('recoverable_fault')),
        motion=owner.get('motion'), stationary=owner.get('stationary') is True,
        capabilities=dict(read_state=True, execute_queue=False, stop=False, capture=False),
        missing_capabilities=['reviewed_queue_to_single_arm_protocol', 'typed_stop_integration',
                              'synchronized_camera_capture', 'per_motor_freshness'],
    )


class _NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        raise ValueError('hardware read redirects are forbidden')


class HardwareReadAdapter:
    """Explicit loopback tunnel only. Construction performs no I/O."""
    def __init__(self, owner_port, *, timeout_s=.8, max_age_s=.5, environment='physical'):
        if type(owner_port) is not int or not 1 <= owner_port <= 65535:
            raise ValueError('owner port must be an integer in 1..65535')
        if not _number(timeout_s) or timeout_s <= 0 or not _number(max_age_s) or max_age_s <= 0:
            raise ValueError('positive timeout and max age required')
        if environment not in ('physical','fake'):raise ValueError('trusted physical/fake environment required')
        self.environment,self.source_clock_id,self.source_stamp=environment,None,None
        self.url = f'http://127.0.0.1:{owner_port}/state'
        self.timeout_s, self.max_age_s = timeout_s, max_age_s
        self.http = build_opener(ProxyHandler({}), _NoRedirect())
        self.previous = None
        self.lock = threading.Lock()

    def read_state(self):
        with self.lock:
            try:
                started=time.monotonic()
                with self.http.open(self.url, timeout=self.timeout_s) as response:
                    payload = response.read(4*1024*1024+1)
                if len(payload) > 4*1024*1024:
                    raise ValueError('owner state reply too large')
                value=json.loads(payload)
                now=time.time_ns()
                if value.get("source") == "read-only-status-gateway":
                    age=value["source_age_s"]
                    if not _number(age) or age<0:raise ValueError("invalid source age")
                    value=value["owner"]
                    now=value["calibration_feedback"]["sample_wall_ns"]+int((age+time.monotonic()-started)*1e9)
                result = normalize_owner_state(value, now_ns=now,
                                               previous=self.previous, max_age_s=self.max_age_s,
                                               environment=self.environment,source_clock_id=self.source_clock_id)
                if result['feedback']['valid'] and self.environment=='fake':
                    point=(result['feedback']['source_time_ns'],result['feedback']['physics_step'])
                    try:assert_progress(self.source_stamp,point,allow_same=True)
                    except ValueError as exc:
                        result['feedback']['valid']=False
                        result['feedback']['reasons'].append('source_clock_invalid: '+str(exc))
                        result.update(q_actual_rad=None,jaw_positions_m=None)
                    else:self.source_clock_id,self.source_stamp=result['feedback']['source_clock_id'],point
                self.previous = result
                return result
            except (OSError, ValueError) as exc:
                self.previous = None
                result = normalize_owner_state({}, now_ns=time.time_ns(), max_age_s=self.max_age_s,
                                               environment=self.environment,source_clock_id=self.source_clock_id)
                result.update(reachable=False, error=f'{type(exc).__name__}: {exc}')
                return result


class HardwareCameraAdapter:
    """Read an already-running real_preview service; never start a camera.

    Current sources attach atomic frame metadata and expose continuous MJPEG.
    Older sources use bracketing reads. Neither supplies synchronized joints or
    an exposure timestamp; do not invent either.
    """
    def __init__(self, camera_port, *, timeout_s=.8, max_age_s=2., environment='physical'):
        if type(camera_port) is not int or not 1 <= camera_port <= 65535:
            raise ValueError('camera port must be an integer in 1..65535')
        if not all(_number(x) and x > 0 for x in (timeout_s, max_age_s)):
            raise ValueError('positive timeout and max age required')
        if environment not in ('physical','fake'):raise ValueError('trusted physical/fake environment required')
        self.environment,self.source_clock_id,self.source_stamp=environment,None,None
        self.base = f'http://127.0.0.1:{camera_port}'
        self.timeout_s, self.max_age_s = timeout_s, max_age_s
        self.http = build_opener(ProxyHandler({}), _NoRedirect())
        self.lock = threading.Lock()
        self.last_failure = None
        self.last_success_ns = None
        self.latest = dict(ready=False, reachable=False, frame_seq=None, frame_timestamp_ns=None,
                           q_actual_rad=None, synchronized=False, note='Camera service has not been read.')

    def _get(self, path, limit):
        with self.http.open(self.base+path, timeout=self.timeout_s) as response:
            data = response.read(limit+1)
        if len(data) > limit:
            raise ValueError('camera reply too large')
        return data

    def _status(self):
        status = json.loads(self._get('/status', 65536))
        if not isinstance(status, dict) or status.get('ready') is not True:
            raise ValueError('camera frame not ready')
        if type(status.get('frame_number')) is not int or status['frame_number'] < 0:
            raise ValueError('camera frame number missing or invalid')
        return status

    def snapshot(self):
        with self.lock:
            result = dict(self.latest)
            stamp = result.get('frame_timestamp_ns')
            if result.get('received_monotonic') is not None:
                result['age_s']=result['source_age_s']+time.monotonic()-result['received_monotonic']
                if result['age_s']>self.max_age_s:result.update(ready=False,error='camera frame stale')
            elif stamp is not None:
                result['age_s'] = (time.time_ns()-stamp)/1e9
                if not 0 <= result['age_s'] <= self.max_age_s:
                    result.update(ready=False, error='camera frame stale or host clocks disagree')
            if result.get('error') and self.latest.get('ready') and not result.get('ready'):
                self.last_failure = dict(reason=result['error'],at_ns=time.time_ns())
                self.latest.update(ready=False,error=result['error'])
            result.update(last_failure=dict(self.last_failure) if self.last_failure else None,
                          last_success_ns=self.last_success_ns,process_alive=None,
                          frame_readable=result.get('ready') is True,
                          monitoring_source=self.base+'/frame.jpg')
        return result

    def poll_state(self):
        """Verify JPEG readability without requiring owner state or camera control."""
        try:
            self.read_frame()
        except (OSError, ValueError):
            pass
        return self.snapshot()

    def monitor(self, stop_event, interval_s=1.):
        """Caller-owned background read loop; never starts or acquires a camera."""
        if not _number(interval_s) or interval_s <= 0:
            raise ValueError('positive camera monitor interval required')
        while not stop_event.is_set():
            self.poll_state()
            if stop_event.wait(interval_s):
                break

    def _accept_frame(self, jpeg, status):
        if not isinstance(status, dict) or status.get('ready') is not True:
            raise ValueError('camera frame not ready')
        if type(status.get('frame_number')) is not int or status['frame_number'] < 0:
            raise ValueError('camera frame number missing or invalid')
        if not jpeg.startswith(b'\xff\xd8') or not jpeg.endswith(b'\xff\xd9'):
            raise ValueError('camera reply is not a complete JPEG')
        raw_stamp = status.get('host_receive_ns')
        stamp = int(raw_stamp) if (type(raw_stamp) is int or isinstance(raw_stamp, str) and raw_stamp.isdecimal()) else None
        if stamp is not None and stamp <= 0:
            raise ValueError('invalid camera host timestamp')
        if self.environment=='fake':
            if status.get('timestamp_kind')!='simulation_step':raise ValueError('fake camera requires simulation_step timestamp')
            clock=clock_spec('fake',clock_domain=status.get('clock_domain'),
                             source_clock_id=self.source_clock_id or status.get('source_clock_id'))
            source_stamp=sample_stamp({**status,'host_receive_ns':stamp},clock)
            assert_progress(self.source_stamp,source_stamp,allow_same=True)
        else:
            # Older physical previews may omit wall timestamps, but never use a
            # simulation source as an implicit physical camera fallback.
            if (status.get('environment') not in (None,'physical') or status.get('simulation_only') is True
                    or status.get('timestamp_kind')=='simulation_step' or status.get('clock_domain') not in (None,'host_wall') or status.get('source_clock_id') is not None):
                raise ValueError('simulation camera cannot authorize physical capture')
        age = status.get('source_age_s',(time.time_ns()-stamp)/1e9 if stamp is not None else None)
        if age is not None and not _number(age):raise ValueError('invalid camera age')
        if age is not None and not 0 <= age <= self.max_age_s:
            raise ValueError('camera frame stale or host clocks disagree')
        if self.environment=='fake':self.source_clock_id,self.source_stamp=clock['source_clock_id'],source_stamp
        device = status.get('device_timestamp_ms')
        self.last_success_ns = time.time_ns()
        self.latest = dict(ready=True, reachable=True, source_age_s=age,received_monotonic=time.monotonic() if 'source_age_s' in status else None,frame_seq=status['frame_number'],
                           frame_timestamp_ns=stamp, timestamp_kind='simulation_step' if self.environment=='fake' else 'host_receive', age_s=age,
                           environment=self.environment,clock_domain='simulation' if self.environment=='fake' else 'host_wall',
                           source_clock_id=self.source_clock_id,source_time_ns=self.source_stamp[0] if self.source_stamp else None,
                           physics_step=self.source_stamp[1] if self.source_stamp else None,
                           device_timestamp_ms=device if _number(device) and device >= 0 else None,
                           delivered_fps=status.get('delivered_fps'), capture_fps=status.get('capture_fps'),
                           timestamp_domain=status.get('timestamp_domain'), metadata_matched=True,
                           q_actual_rad=None, synchronized=False, calibration_sample=False,
                           note='Annotated raw camera preview; no paired joints or exposure time. Host age requires synchronized clocks.'
                                if stamp is not None else 'Camera host timestamp absent; freshness unknown. No paired joints.')
        return jpeg, dict(self.latest)

    def _failed(self, exc):
        self.last_failure = dict(reason=f'{type(exc).__name__}: {exc}',at_ns=time.time_ns())
        self.latest = dict(ready=False, reachable=False, frame_seq=None, frame_timestamp_ns=None,
                           q_actual_rad=None, synchronized=False, error=self.last_failure['reason'])

    def read_frame(self):
        with self.lock:
            try:
                with self.http.open(self.base+'/frame.jpg', timeout=self.timeout_s) as response:
                    jpeg = response.read(8*1024*1024+1)
                    metadata = response.headers.get('X-Camera-Metadata')
                if len(jpeg) > 8*1024*1024:
                    raise ValueError('camera reply too large')
                if metadata is not None:
                    if len(metadata) > 65536:
                        raise ValueError('camera metadata too large')
                    return self._accept_frame(jpeg, json.loads(metadata))
                # Older deployed cameras do not include atomic metadata headers.
                for _ in range(2):
                    before = self._status()
                    jpeg = self._get('/frame.jpg', 8*1024*1024)
                    status = self._status()
                    if all(before.get(k) == status.get(k) for k in ('frame_number', 'host_receive_ns', 'device_timestamp_ms', 'timestamp_domain')):
                        return self._accept_frame(jpeg, status)
                raise ValueError('camera frame changed during metadata read; retry next poll')
            except (OSError, ValueError) as exc:
                self._failed(exc)
                raise

    def read_calibration_frame(self):
        """Atomic unannotated endpoint only; old placement JPEGs are never a fallback."""
        with self.lock:
            try:
                with self.http.open(self.base+'/raw-frame.jpg',timeout=self.timeout_s) as response:
                    jpeg=response.read(8*1024*1024+1)
                    header=response.headers.get('X-Camera-Metadata')
                if len(jpeg)>8*1024*1024 or header is None or len(header)>65536:
                    raise ValueError('bounded atomic raw frame metadata required')
                status=json.loads(header)
                if status.get('raw') is not True or status.get('annotated') is not False or status.get('image_domain')!='raw':
                    raise ValueError('explicit unannotated raw camera frame required')
                # RGB-D producers bind the serial at the atomic frame level.
                # Preserve an explicit inner serial so capture rejects mismatches.
                if isinstance(status.get('intrinsics'),dict) and isinstance(status.get('serial'),str) and status['serial']:
                    status['intrinsics']={'serial':status['serial'],**status['intrinsics']}
                self._accept_frame(jpeg,status)
                return jpeg,{**status,'metadata_matched':True}
            except (OSError,ValueError) as exc:
                self._failed(exc)
                raise

    def iter_frames(self):
        """One connection per viewer, with bounded atomic JPEG/metadata parts."""
        try:
            with self.http.open(self.base+'/stream.mjpg', timeout=max(3., self.timeout_s)) as response:
                if response.headers.get('Content-Type', '').split(';')[0] != 'multipart/x-mixed-replace':
                    raise ValueError('camera stream is not MJPEG')
                while True:
                    boundary = response.readline(128)
                    if boundary == b'\r\n':
                        boundary = response.readline(128)
                    if boundary != b'--frame\r\n':
                        raise ValueError('camera stream ended or invalid boundary')
                    headers, size = {}, 0
                    while True:
                        line = response.readline(65537)
                        size += len(line)
                        if size > 65536 or not line:
                            raise ValueError('camera stream headers too large or incomplete')
                        if line == b'\r\n':
                            break
                        key, separator, value = line.decode('ascii').partition(':')
                        if not separator:
                            raise ValueError('invalid camera stream header')
                        headers[key.lower()] = value.strip()
                    length = int(headers.get('content-length', '0'))
                    if not 0 < length <= 8*1024*1024:
                        raise ValueError('camera stream frame too large or empty')
                    jpeg = response.read(length)
                    if len(jpeg) != length:
                        raise ValueError('camera stream frame incomplete')
                    status = json.loads(headers.get('x-camera-metadata', '{}'))
                    with self.lock:
                        result = self._accept_frame(jpeg, status)
                    yield result
        except (OSError, ValueError) as exc:
            with self.lock:
                self._failed(exc)
            raise
