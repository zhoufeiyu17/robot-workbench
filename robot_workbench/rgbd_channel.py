"""Bounded same-process RGB-D delivery; no filesystem work on the camera path."""
from collections import deque
from copy import deepcopy
import json
import threading
import time
from robot_workbench.sample_clock import owner_identity


class MemoryRgbdBudget:
    """One byte budget shared by all three recording subscriptions."""
    def __init__(self, max_bytes=1024**3):
        if type(max_bytes) is not int or max_bytes < 1:
            raise ValueError('positive recording memory byte budget required')
        self.max_bytes = max_bytes
        self.size = self.peak_size = 0
        self.lock = threading.RLock()

    def snapshot(self):
        with self.lock:
            return dict(queued_bytes=self.size, peak_bytes=self.peak_size, max_bytes=self.max_bytes,
                        accounting='payload_bytes_plus_metadata_json', scope='shared_camera_subscriptions')


class MemoryRgbdChannel:
    def __init__(self, *, lease_guard, budget=None, clock_ns=time.time_ns):
        if not callable(lease_guard) or budget is not None and not isinstance(budget, MemoryRgbdBudget):
            raise ValueError('recording guard and shared memory budget required')
        self.lease_guard, self.clock_ns = lease_guard, clock_ns
        self.budget = budget if budget is not None else MemoryRgbdBudget()
        self.lock = self.budget.lock
        self.frames = deque()
        self.lease = None
        self.closed = False
        self.error = None
        self.count = self.size = self.peak_size = self.peak_frames = 0
        self.head = None
        self.admission_endpoint = None
        self.admission_closed = False
        self.after_endpoint_frames = 0

    @staticmethod
    def _validate(lease):
        token = lease.get('run_id')
        start, end = lease.get('start_wall_ns'), lease.get('deadline_wall_ns')
        admission = lease.get('admission_start_wall_ns', start)
        if (not isinstance(token, str) or len(token) != 32 or any(c not in '0123456789abcdef' for c in token)
                or type(start) is not int or type(end) is not int or not 0 < end-start <= 600_000_000_000
                or type(admission) is not int or not 0 < admission <= start
                or not isinstance(lease.get('execution_instance_id'), str) or not lease['execution_instance_id']
                or not isinstance(lease.get('execution_owner'), dict) or not lease['execution_owner']):
            raise ValueError('invalid bounded same-process recording lease')
        if owner_identity(lease['execution_owner']) != lease['execution_owner']:
            raise ValueError('one execution Owner identity required')
        if lease.get('environment','fake') not in ('physical','fake'):
            raise ValueError('explicit known recording environment required')
        if lease.get('identity_scope','execution_epoch') not in ('execution_epoch','recording_run'):
            raise ValueError('known recording identity scope required')

    def begin(self, lease):
        self._validate(lease)
        with self.lock:
            if self.closed or self.lease is not None:
                raise ValueError('camera recording channel is closed or already leased')
            if not lease['start_wall_ns'] <= self.clock_ns() < lease['deadline_wall_ns']:
                raise ValueError('recording lease is not currently valid')
            self.lease = deepcopy(lease)
            self.frames.clear()
            self.error = self.head = None
            self.count = self.size = self.peak_size = self.peak_frames = 0
            self.admission_endpoint = None
            self.admission_closed = False
            self.after_endpoint_frames = 0

    def token(self):
        with self.lock:
            return self.lease['run_id'] if self.lease else None

    def current_lease(self):
        with self.lock:
            return deepcopy(self.lease)

    def renew(self, token, previous_deadline, new_deadline):
        with self.lock:
            lease = self._owned(token)
            now = self.clock_ns()
            if (lease['deadline_wall_ns'] != previous_deadline or not lease['start_wall_ns'] <= now < previous_deadline
                    or not previous_deadline < new_deadline <= now+600_000_000_000):
                raise ValueError('recording lease ownership/window changed; expired leases cannot resume')
            if not self.lease_guard(lease):
                return False
            self.lease = dict(lease, start_wall_ns=now, deadline_wall_ns=new_deadline,
                              admission_start_wall_ns=lease.get('admission_start_wall_ns', lease['start_wall_ns']))
        return True

    def _owned(self, token):
        if self.lease is None or token != self.lease['run_id']:
            raise ValueError('camera recording channel belongs to another execution')
        return self.lease

    def fail(self, reason, *, sampled_token):
        with self.lock:
            if self.lease and self.lease['run_id'] == sampled_token:
                self.error = self.error or str(reason)

    def _endpoint_reached(self,metadata):
        if self.admission_endpoint is None or metadata is None:return False
        key='source_time_ns' if self.lease.get('environment','fake')=='fake' else 'host_receive_ns'
        stamp=metadata.get(key)
        if key=='host_receive_ns' and isinstance(stamp,str) and stamp.isascii() and stamp.isdigit():stamp=int(stamp)
        if type(stamp) is not int:raise ValueError('recording endpoint requires the original integer source timestamp')
        return stamp>=self.admission_endpoint

    def stop_after(self,token,endpoint_ns):
        """Stop admission after one bracketing frame; preserve every queued pair."""
        if type(endpoint_ns) is not int or endpoint_ns<0:raise ValueError('nonnegative source endpoint required')
        with self.lock:
            self._owned(token)
            if self.admission_endpoint not in (None,endpoint_ns):raise ValueError('recording endpoint cannot change')
            self.admission_endpoint=endpoint_ns
            self.admission_closed=self.admission_closed or endpoint_ns==0 or self._endpoint_reached(self.head)
            return dict(endpoint_ns=endpoint_ns,admission_closed=self.admission_closed,queued_frames=len(self.frames))

    def write(self, status, color, depth, *, sampled_token):
        """Legacy lossless frame input; new camera services submit raw pixels."""
        if not isinstance(color, bytes) or not isinstance(depth, bytes):
            raise ValueError('immutable encoded RGB-D bytes required')
        return self._write(status, dict(color_png=color,depth_png=depth), sampled_token=sampled_token)

    def write_pixels(self,status,rgb,depth,*,width,height,sampled_token):
        if (type(width) is not int or type(height) is not int or min(width,height)<=0
                or not isinstance(rgb,bytes) or depth is not None and not isinstance(depth,bytes)
                or len(rgb)!=width*height*3 or depth is not None and len(depth)!=width*height*2
                or max(len(rgb),len(depth)if depth is not None else 0)>16*1024**2):
            raise ValueError('bounded immutable RGB8 and aligned little-endian depth16 pixels required')
        return self._write(status,dict(rgb=rgb,depth=depth,width=width,height=height,
            pixel_format='rgb8'if depth is None else'rgb8-depth16le'),sampled_token=sampled_token)

    def _write(self,status,payload,*,sampled_token):
        lease = self.current_lease()
        environment=lease.get('environment','fake') if lease else None
        owner=lease['execution_owner'] if lease else {}
        same_owner=all(status.get(key)==value for key,value in owner.items())
        if lease and lease.get('identity_scope')=='recording_run':
            same_owner=(all(status.get(key)==owner[key] for key in ('owner_pid','runtime_nonce'))
                and type(status.get('command_epoch')) is int and status['command_epoch']>=owner['command_epoch'])
        admitted = (lease is not None and sampled_token == lease['run_id']
                    and self.lease_guard(lease)
                    and int(status['host_receive_ns']) >= lease.get('admission_start_wall_ns', lease['start_wall_ns'])
                    and status.get('environment','fake')==environment
                    and (environment=='physical' or same_owner))
        with self.lock:
            admitted = (admitted and self.lease is not None and self.lease['run_id'] == sampled_token
                        and self.lease['start_wall_ns'] <= self.clock_ns() < self.lease['deadline_wall_ns'])
            if admitted and self.admission_closed:
                self.after_endpoint_frames+=1
                admitted=False
            current = self.lease if admitted else None
            status.update(recording_enabled=bool(current) and not self.error,
                          recorded_frames=self.count if current else 0,
                          recording_error=(status.get('recording_error') or self.error) if current else None,
                          recording_run_id=current['run_id'] if current else None,
                          recording_duration_s=(current['deadline_wall_ns']-current['start_wall_ns'])/1e9 if current else 0,
                          recording_deadline_wall_ns=current['deadline_wall_ns'] if current else None)
            if not admitted:
                return False
            self.error = self.error or status.get('recording_error')
            if environment=='physical':
                status.update(recording_execution_owner=deepcopy(current['execution_owner']),
                              recording_identity_scope='lease association; not simultaneous Owner feedback')
            if current.get('identity_scope')=='recording_run':
                status.update(recording_execution_owner=deepcopy(current['execution_owner']),recording_identity_scope='recording_run')
            metadata = deepcopy(status)
            metadata['recorded_frames'] = self.count+1
            closes_after=self._endpoint_reached(metadata)
            size = sum(len(v) for v in payload.values() if isinstance(v,bytes))+len(json.dumps(metadata, allow_nan=False).encode())
            if self.budget.size+size > self.budget.max_bytes:
                self.error = self.error or ('bounded camera recording memory queue overflow: shared byte budget '
                    f'{self.budget.size} + {size} > {self.budget.max_bytes} bytes')
            if self.error:
                status.update(data_valid=False, recording_enabled=False, recording_error=self.error)
                return False
            self.count += 1
            status['recorded_frames'] = self.count
            metadata['recorded_frames'] = self.count
            self.frames.append(dict(metadata=metadata, **payload, size=size))
            self.size += size
            self.budget.size += size
            self.budget.peak_size = max(self.budget.peak_size, self.budget.size)
            self.peak_size = max(self.peak_size, self.size)
            self.peak_frames = max(self.peak_frames, len(self.frames))
            self.head = metadata
            if closes_after:self.admission_closed=True
            return True

    def peek(self, token):
        with self.lock:
            self._owned(token)
            if self.error:
                raise ValueError(self.error)
            return deepcopy(self.frames[0]) if self.frames else None

    def pop(self, token):
        with self.lock:
            self._owned(token)
            if self.error:
                raise ValueError(self.error)
            if not self.frames:
                return None
            frame = self.frames.popleft()
            self.size -= frame['size']
            self.budget.size -= frame['size']
            return dict(frame, metadata=deepcopy(frame['metadata']))

    def ack(self,token,pair_id):
        with self.lock:
            self._owned(token)
            if not self.frames or self.frames[0]['metadata'].get('pair_id')!=pair_id:
                raise ValueError('camera acknowledgement differs from queued head')
            self.pop(token)

    def latest(self, token):
        with self.lock:
            self._owned(token)
            if self.error:
                raise ValueError(self.error)
            return deepcopy(self.head)

    def end(self, token):
        with self.lock:
            if self.lease is None or self.lease['run_id'] != token:
                return False
            self.lease = None
            self.frames.clear()
            self.budget.size -= self.size
            self.size = 0
        return True

    def snapshot(self, token=None):
        with self.lock:
            if token is not None:
                self._owned(token)
            return dict(transport='bounded_memory', run_id=self.lease['run_id'] if self.lease else None,
                        queued_frames=len(self.frames), queued_bytes=self.size, capacity=None,
                        shared_budget=self.budget.snapshot(), peak_frames=self.peak_frames, peak_bytes=self.peak_size,
                        published_frames=self.count, error=self.error, closed=self.closed,
                        admission_endpoint_ns=self.admission_endpoint,admission_closed=self.admission_closed,
                        outside_endpoint_frames=self.after_endpoint_frames,
                        latest_metadata=deepcopy(self.head))

    def close(self):
        with self.lock:
            self.closed = True
            self.lease = None
            self.frames.clear()
            self.budget.size -= self.size
            self.size = 0
