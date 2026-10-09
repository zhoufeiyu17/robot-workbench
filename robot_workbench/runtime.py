"""One managed device owner; server-side business credentials, no HTTP or SDK loader.

The embedding service supplies the reviewed backend factory/initialization and
runs tick() on its sole control thread. A local lock excludes cooperating owners,
not arbitrary external CAN programs. SDK return evidence is not firmware ACK.
"""
from copy import copy, deepcopy
import fcntl
import hashlib
import os
from pathlib import Path
import queue
import secrets
import stat
import threading
import time


BUSINESSES = {'calibration', 'collection'}
CALIBRATION_PREVIEWS = {'preview', 'preview_home', 'preview_arm_home', 'preview_jaw_target',
                        'preview_both_home', 'preview_workbench'}
COLLECTION_PREVIEWS = {'preview_frozen', 'preview_start_alignment'}


class _Backend:
    """Track entry/return of real backend calls without locking around native IO."""
    def __init__(self, backend, runtime):
        object.__setattr__(self, '_backend', backend)
        object.__setattr__(self, '_runtime', runtime)

    def __setattr__(self, name, value):
        setattr(self._backend, name, value)

    def __getattr__(self, name):
        value = getattr(self._backend, name)
        if name not in {'connect', 'close', 'begin', 'send', 'set_damping_hold',
                         'read_calibration_state', 'get_bimanual_joint_position_limits', 'read_gains'}:
            return value
        def invoke(*args, **kwargs):
            return self._runtime._backend_call(name, value, *args, **kwargs)
        return invoke


class Runtime:
    def __init__(self, ref, safety, backend_factory, *, device_id, environment, lock_root,
                 initial_holder='calibration', initialize_backend=None, **owner_options):
        if environment not in {'fake', 'physical'} or initial_holder not in BUSINESSES:
            raise ValueError('explicit fake/physical environment and registered business required')
        if not isinstance(device_id, str) or not device_id.strip():
            raise ValueError('nonempty device identity required')
        hardware = environment == 'physical'
        if 'hardware' in owner_options and owner_options.pop('hardware') != hardware:
            raise ValueError('hardware flag differs from runtime namespace')
        self.device_id, self.environment = device_id, environment
        self.nonce, self.holder = secrets.token_hex(24), initial_holder
        self.lock, self.tick_lock = threading.RLock(), threading.Lock()
        self.clients, self.calls, self.hold = {}, {}, {'left': None, 'right': None}
        self.hold_returns = {'left': None, 'right': None}
        self.call_sequence, self.preview_revision = 0, 0
        self.last_return, self.stop_receipt, self.detached_hold = None, None, None
        self.initialization_error = None
        self.owner = None
        root = Path(lock_root).resolve(); root.mkdir(parents=True, exist_ok=True)
        identity = hashlib.sha256((environment+'\0'+device_id).encode()).hexdigest()
        self.lock_path = root/(identity+'.lock')
        self.fd = os.open(self.lock_path, os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW, 0o600)
        try:
            if not stat.S_ISREG(os.fstat(self.fd).st_mode):
                raise ValueError('device lock must be a regular file')
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            from real_control import classes
            collection, _, _ = classes(ref)
            runtime = self
            class ManagedOwner(collection):
                def start_motion(self, plan, feedback, *, reviewed_deadline=None):
                    runtime.detached_hold = None
                    if plan.sha256 in self.frozen_payloads or plan.sha256 in self.alignment_plans:
                        return super().start_motion(plan, feedback, reviewed_deadline=reviewed_deadline)
                    return ref.Owner.start_motion(self, plan, feedback, reviewed_deadline=reviewed_deadline)

                def begin_collection(self, measured):
                    # Includes the physical collection implementation's direct gain calls.
                    return runtime._backend_call('begin_collection', super().begin_collection, measured)

            self.ref, self.collection = ref, collection
            backend = _Backend(backend_factory(), self)
            self.owner = ManagedOwner(backend, safety, hardware=hardware, **owner_options)
            if initialize_backend is not None:
                # Caller owns connection/support authorization. No implicit connect/hold.
                with self.tick_lock:
                    try:
                        initialize_backend(backend)
                    except Exception as error:
                        # Retain ownership after a failed native initialization;
                        # never free the lock while compensation may still be live.
                        self.initialization_error = str(error)
                        self.owner.control.state = self.owner.states.FAULT
                        self.owner.control.reason = 'initialization failed: '+str(error)
        except BaseException:
            os.close(self.fd); self.fd = None
            raise

    def register_business(self, business):
        """Trusted server bootstrap only; never expose registration as a public route."""
        if business not in BUSINESSES:
            raise ValueError('unknown business')
        with self.lock:
            if business in self.clients:
                raise ValueError('business already registered')
            token = secrets.token_urlsafe(32)
            self.clients[business] = token
            return dict(business=business, token=token, runtime_nonce=self.nonce,
                        environment=self.environment, device_id=self.device_id)

    def _authenticate(self, credential):
        if self.fd is None:
            raise PermissionError('runtime closed; reconnect through the device lifecycle')
        if not isinstance(credential, dict):
            raise PermissionError('registered server identity required')
        business = credential.get('business')
        expected = self.clients.get(business)
        token = credential.get('token')
        if (not expected or not isinstance(token, str) or not secrets.compare_digest(token, expected)
                or credential.get('runtime_nonce') != self.nonce
                or credential.get('environment') != self.environment or credential.get('device_id') != self.device_id):
            raise PermissionError('business credential belongs to another runtime/device/environment')
        return business

    def _backend_call(self, name, function, *args, **kwargs):
        with self.lock:
            if name in {'begin', 'send', 'begin_collection'}:
                self.owner.command_gate()
            self.call_sequence += 1
            call = dict(id=self.call_sequence, operation=name, started_monotonic_s=time.monotonic(),
                        epoch=self.owner.epoch if self.owner else None)
            self.calls[call['id']] = call
        try:
            result = function(*args, **kwargs)
        except BaseException as error:
            with self.lock:
                call['error'] = str(error)
                if name in {'begin', 'send', 'begin_collection', 'set_damping_hold', 'close'}:
                    self.hold = dict(left=None, right=None)
                    self.hold_returns = dict(left=None, right=None)
            raise
        else:
            with self.lock:
                if name == 'set_damping_hold':
                    self.hold = dict.fromkeys(('left', 'right'), 'manual_damping' if kwargs.get('manual') else 'damping')
                    self.hold_returns = dict.fromkeys(('left', 'right'), call)
                elif name == 'begin':
                    self.hold[args[0]] = 'position'
                    self.hold_returns[args[0]] = call
                elif name == 'begin_collection':
                    self.hold = dict.fromkeys(('left', 'right'), 'position')
                    self.hold_returns = dict.fromkeys(('left', 'right'), call)
                elif name == 'close':
                    self.hold = dict(left=None, right=None)
                    self.hold_returns = dict(left=None, right=None)
            return result
        finally:
            with self.lock:
                call['returned_monotonic_s'] = time.monotonic()
                self.last_return = call
                self.calls.pop(call['id'])

    def _idle_reason(self):
        owner = self.owner
        if self.tick_lock.locked() or self.calls:
            return 'control tick or SDK call is still in flight'
        if not owner.backend.is_connected:
            return 'device disconnected'
        if owner.stop_requested or owner.release_requested or not owner.commands.empty():
            return 'control command still pending'
        if (owner.runner is not None and owner.runner.status not in {'READY', 'STOPPED'}
                or owner.sequence_remaining or owner.capture_sample):
            return 'motion or capture is still active'
        if owner.control.state not in (owner.states.READY, owner.states.STOPPED, owner.states.MANUAL):
            return 'holding/control state is not confirmed'
        if any(mode is None for mode in self.hold.values()):
            return 'no successful SDK holding-mode return evidence'
        feedback = owner.cache.get('calibration_feedback') or {}
        if (not owner.feedback_healthy or not owner.stationary
                or not 0 <= owner.control_time()-feedback.get('sample_time_s', -1) <= .1):
            return 'fresh healthy stationary feedback required'
        if owner.pedal_snapshot()[1:] != (False, True):
            return 'pedal must be released and healthy'
        recording = owner.recording
        if recording is not None:
            if not recording.done.is_set() or getattr(recording, 'thread', None) is None or recording.thread.is_alive():
                return 'recording writer has not exited after sealing'
        return None

    def submit(self, credential, command):
        if not isinstance(command, dict):
            raise ValueError('command object required')
        action = command.get('action')
        with self.lock:
            business = self._authenticate(credential)
            if action == 'stop':
                result = self.ref.Owner.submit(self.owner, command)
                self.preview_revision += 1
                self.stop_receipt = dict(accepted_monotonic_s=time.monotonic(), barrier_epoch=self.owner.epoch,
                    in_flight_calls=deepcopy(list(self.calls.values())), prior_calls_complete=not self.calls,
                    stop_hold_return_confirmed=False,
                    firmware_ack='unsupported', physical_stop='unverified')
                recording = self.owner.recording
                if recording is not None:
                    # Writer bookkeeping is outside request/control locks. A stuck
                    # writer blocks handoff, never stop acceptance or motor holding.
                    def finish():
                        try:
                            recording.finish('stopped', 'operator stop')
                        except Exception as error:
                            self.owner.recording_failed(error)
                    threading.Thread(target=finish, daemon=True).start()
                return dict(result, stop=deepcopy(self.stop_receipt))
            if business != self.holder:
                raise PermissionError('请先交接控制权限：当前由 '+self.holder+' 持有')
            previews = CALIBRATION_PREVIEWS if business == 'calibration' else COLLECTION_PREVIEWS
            if action in previews:
                reason = self._idle_reason()
                if reason:
                    raise ValueError(reason)
                self.preview_revision += 1
                revision, epoch = self.preview_revision, self.owner.epoch
                pedal = self.owner.pedal_snapshot()
                with self.owner.lock:
                    # A compiler snapshot has no control loop and never touches SDK.
                    compiler = copy(self.owner)
                    compiler.lock = threading.RLock()
                    compiler.cache = deepcopy(self.owner.cache)
                    compiler.pending, compiler.frozen_payloads, compiler.alignment_plans = {}, {}, {}
                    compiler.commands = queue.Queue(maxsize=16)
                method = self.ref.Owner.submit if business == 'calibration' else self.collection.submit
            else:
                if action not in {'confirm', 'capture_ack', 'recover', 'release', 'simulate_pedal'}:
                    raise ValueError('action unavailable for this business')
                if action == 'capture_ack' and business != 'calibration':
                    raise ValueError('capture acknowledgement belongs to calibration')
                if action in {'confirm', 'release', 'recover', 'simulate_pedal'}:
                    self.preview_revision += 1
                method = self.ref.Owner.submit if business == 'calibration' else self.collection.submit
                return method(self.owner, command)
        # No runtime/owner lock around full plan validation and hashing.
        result = method(compiler, command)
        with self.lock, self.owner.lock:
            if (self.holder != business or self.owner.epoch != epoch or self.preview_revision != revision
                    or self._idle_reason() or self.owner.cache.get('joint_limits') != compiler.cache.get('joint_limits')
                    or self.owner.pedal_snapshot() != pedal):
                raise ValueError('preview revoked during compilation; request a new preview')
            self.owner.pending = compiler.pending
            self.owner.frozen_payloads = compiler.frozen_payloads
            self.owner.alignment_plans = compiler.alignment_plans
            self.owner.review_wall_deadline = compiler.review_wall_deadline
            return dict(result, business=business, runtime_nonce=self.nonce, environment=self.environment)

    def handoff(self, credential, target):
        with self.lock:
            business = self._authenticate(credential)
            if business != self.holder or target not in self.clients or target == business:
                raise PermissionError('current holder must explicitly hand off to another registered business')
            reason = self._idle_reason()
            if reason:
                raise ValueError(reason)
            if not self.tick_lock.acquire(blocking=False):
                raise ValueError('control tick is in flight')
            try:
                with self.owner.lock:
                    self.owner.invalidate()
                    self.preview_revision += 1
                    self.owner.frozen_payloads.clear(); self.owner.alignment_plans.clear()
                    # READY runners only monitor endpoint hold. Retire their old
                    # policy/epoch; the SDK retains its already submitted setpoint.
                    self.owner.runner = None
                    self.owner.active_epoch = None
                    self.detached_hold = dict(modes=deepcopy(self.hold), epoch=self.owner.epoch,
                                              sdk_returns=deepcopy(self.hold_returns),
                                              evidence='completed SDK calls; firmware ACK unsupported')
                    self.holder = target
                    self.owner.cache.update(command_epoch=self.owner.epoch, motion=None, automatic_enabled=False)
                    self.owner.cache.pop('frozen', None)
                    return dict(holder=target, command_epoch=self.owner.epoch, hold=deepcopy(self.detached_hold))
            finally:
                self.tick_lock.release()

    def tick(self):
        if not self.tick_lock.acquire(blocking=False):
            raise RuntimeError('only one control tick may run at a time')
        try:
            if self.fd is None:
                raise RuntimeError('runtime closed')
            result = self.owner.tick()
            with self.lock:
                if self.stop_receipt is not None:
                    receipt = self.stop_receipt
                    stopped = self.owner.control.state in (self.owner.states.STOPPED, self.owner.states.MANUAL)
                    if (stopped and not self.calls and all(row is not None
                            and row['operation'] == 'set_damping_hold' and 'error' not in row
                            and row.get('started_monotonic_s', -1) >= receipt['accepted_monotonic_s']
                            and 'returned_monotonic_s' in row for row in self.hold_returns.values())):
                        receipt.update(stop_hold_return_confirmed=True, sdk_hold_returns=deepcopy(self.hold_returns))
            return result
        finally:
            self.tick_lock.release()

    def snapshot(self, credential):
        with self.lock:
            business = self._authenticate(credential)
            reason = '请先交接控制权限：当前由 '+self.holder+' 持有' if business != self.holder else self._idle_reason()
            # Use the cache: a stalled SDK/recording call must not block status.
            with self.owner.lock:
                state = deepcopy(self.owner.cache)
            state.update(device_id=self.device_id, environment=self.environment, runtime_nonce=self.nonce,
                holder=self.holder, command_epoch=self.owner.epoch, hold=deepcopy(self.hold),
                sdk_hold_returns=deepcopy(self.hold_returns),
                sdk_calls_in_flight=deepcopy(list(self.calls.values())), last_sdk_return=deepcopy(self.last_return),
                detached_hold=deepcopy(self.detached_hold), stop_receipt=deepcopy(self.stop_receipt),
                initialization_error=self.initialization_error,
                allowed_operations={'stop':dict(allowed=True, reason=None),
                    'preview':dict(allowed=reason is None, reason=reason),
                    'handoff':dict(allowed=reason is None, reason=reason),
                    'confirm':dict(allowed=reason is None and bool(self.owner.pending),
                                   reason=reason or (None if self.owner.pending else 'generate and review a fresh plan'))})
            return state

    def close(self):
        """Release the managed lock only after an explicitly authorized disconnect."""
        with self.lock:
            if self.tick_lock.locked() or self.calls or self.owner.backend.is_connected:
                raise RuntimeError('explicit supported release and completed SDK calls required before closing runtime')
            if self.fd is not None:
                os.close(self.fd); self.fd = None
