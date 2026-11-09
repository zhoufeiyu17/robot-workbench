"""Serialized deployment-owned operations, independent of runtime safety stop.

Only trusted deployment code registers callbacks. Requests cannot supply paths,
commands or arbitrary callback arguments; registered bounded parameters only. ``check`` is read-only and returns a reason or
None; ``run`` receives (request, progress, cancellation_event); ``verify`` must
return a JSON object with ``ok is True`` before an operation can succeed.

Safety stop must keep using its independent runtime path: this queue can wait
on a slow deployment callback. Closing this service never stops a runtime/SDK.
"""
from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
import fcntl
import json
import math
import os
from pathlib import Path
import re
import tempfile
import threading
import time
from typing import Callable
import uuid


class OperationError(ValueError):
    def __init__(self, code, message, *, unknown=False, details=None):
        self.code = code
        self.unknown = unknown
        self.details = _json(details) if details is not None else None
        if self.details is not None and len(json.dumps(self.details).encode())>8192:
            raise ValueError('operation diagnostic exceeds 8KiB')
        super().__init__(message)


def prepare_capture(root, *, request_id, run_id, business_kind, environment, policy,
                    source_descriptors, still_current, execution_owner=None, execution_context=None):
    """SC05: qualify and bind one recorder; never start control or capture sources."""
    from .execution_settings import Guard,validate_control_parameters,policy_hash
    from .recording_process import CaptureSession
    if environment not in ('fake','physical') or business_kind not in ('calibration','collection','inference','research'):
        raise OperationError('capture_context_invalid','Known business and environment required.')
    if not isinstance(run_id,str) or not 0<len(run_id)<=200 or not callable(still_current):
        raise OperationError('capture_context_invalid','Run identity and current-context verifier required.')
    if (not isinstance(policy,dict) or not {'revision','policy_hash','parameters'}<=policy.keys()
            or type(policy['revision']) is not int or policy['revision']<0):
        raise OperationError('capture_policy_invalid','Frozen Owner policy required.')
    policy=deepcopy(policy)
    parameters=validate_control_parameters(policy['parameters'],environment=environment,business=business_kind)
    if policy_hash(parameters)!=policy['policy_hash']:
        raise OperationError('capture_policy_changed','Frozen policy contents and hash differ; refresh the Run.')
    frozen=dict(revision=policy['revision'],policy_hash=policy['policy_hash'],parameters=parameters)
    recording=parameters['recording']
    if recording['purpose_profile'] is None:
        raise OperationError('capture_purpose_required','Research Run must explicitly select its recording purpose.')
    expected=dict(run_id=run_id,business_kind=business_kind,policy_hash=frozen['policy_hash'])
    if execution_context is not None:expected.update(control_policy=_json(frozen),execution_context=_json(execution_context))
    if execution_owner is not None:expected['execution_owner']=deepcopy(execution_owner)
    def current():
        value=still_current()
        if type(value) is not bool:raise OperationError('capture_context_invalid','Current-context check must return a boolean.')
        return value
    def receipt(session, state, *, ready=False, decision=None, reason=None):
        actual=session.snapshot() if session else {}
        return dict(capture_session_id=session.capture_session_id if session else None,state=state,ready=ready,
                    decision=decision,reason=reason,policy_hash=frozen['policy_hash'],
                    binding=deepcopy(actual.get('binding')),missing_streams=list(actual.get('missing_streams',[])),
                    degraded=bool(actual.get('degraded')))
    if not current():return None,receipt(None,'cancelled',reason='Run context was revoked before preparation.')
    session=CaptureSession.create(root,request_id=request_id,source_descriptors=source_descriptors,
        purpose_profile=recording['purpose_profile'],resource_budget=recording['resource_budget'])
    try:state=session.prepare()
    except (OSError,ValueError,TimeoutError) as error:
        return session,receipt(session,'unknown',reason=str(error))
    if state.get('read_only'):
        return session,receipt(session,'unknown',reason='Existing writer is read-only; inspect its original receipt.')
    if all(s.get('source_ref',{}).get('kind')=='ros'for s in source_descriptors):
        if not isinstance(state.get('health_source'),dict):
            return session,receipt(session,'unknown',reason='Original ROS recording health binding is unavailable.')
        expected['health_source']=deepcopy(state['health_source'])
    if state.get('binding') not in (None,expected):
        raise OperationError('capture_binding_changed','Request is already bound to a different Run; original capture preserved.')
    def finish_candidate(status,reason):
        session.finish(dict(session.last_sequence),dict(status=status,reason=reason),
            **({'source_endpoints':{key:dict(source_clock_id=row['source_clock_id'],source_time=0.) for key,row in session.camera_sources.items()}}
               if session.camera_sources else {}))
    def cancelled():
        if session.snapshot().get('state') in ('prepared','bound'):
            finish_candidate('cancelled','Run context revoked during capture preparation')
        return session,receipt(session,'cancelled',reason='Run context revoked during capture preparation.')
    try:allowed=current()
    except Exception as error:return session,receipt(session,'unknown',reason=str(error))
    if not allowed:return cancelled()
    failures=len(state.get('missing_streams',[]))+int(bool(state.get('error')))
    if not state.get('ready') and not failures:failures=1
    now=time.monotonic()
    decision=Guard.evaluate(dict(rule_id='G35',signal_id=session.capture_session_id,observed=failures,
        source_time=state.get('heartbeat_monotonic_s',now),checked_time=now,run_id=run_id,request_id=request_id),
        'capture_prepare',frozen)
    if decision['verdict']=='reject':
        finish_candidate('rejected','Recording readiness failed')
        return session,receipt(session,'rejected',decision=decision,reason='Required recording sources/resources are unavailable.')
    try:bound=session.bind(**expected)
    except (OSError,ValueError,TimeoutError) as error:
        return session,receipt(session,'unknown',decision=decision,reason=str(error))
    if bound.get('binding')!=expected:
        return session,receipt(session,'unknown',decision=decision,reason='Capture binding was not confirmed.')
    try:allowed=current()
    except Exception as error:return session,receipt(session,'unknown',reason=str(error))
    if not allowed:return cancelled()
    return session,receipt(session,'bound',ready=True,decision=decision)


@dataclass(frozen=True)
class Operation:
    name: str
    label: str
    impact: str
    check: Callable
    run: Callable
    verify: Callable
    requires_support: bool | Callable = False
    cancellable: bool = False
    timeout_s: float | None = None
    parameter_keys: tuple[str, ...] = ()
    use_case_id: str | None = None

    def support_required(self, request):
        value=self.requires_support(deepcopy(request))if callable(self.requires_support)else self.requires_support
        if type(value)is not bool:raise OperationError('invalid_check','Support policy must return a boolean.')
        return value


def _json(value):
    """Copy only recordable JSON; reject NaN and aliases into service state."""
    return json.loads(json.dumps(value, allow_nan=False, sort_keys=True))


def _identity(value):
    if (not isinstance(value, dict) or value.get('environment') not in ('fake', 'physical')
            or not all(isinstance(key, str) for key in value)):
        raise OperationError('target_invalid', 'explicit fake/physical target identity required')
    return _json(value)


def _same(left, right):
    return json.dumps(left, sort_keys=True) == json.dumps(right, sort_keys=True)


def _receipt_fields(job):
    """Derive the common receipt without changing legacy status or identities."""
    status = job['status']
    outcome = 'unknown' if status in ('unknown', 'timeout_unknown', 'interrupted_unknown') else (
        'cancelling' if status == 'cancel_requested' else status)
    if outcome == 'running':
        outcome = {'preflight': 'checking', 'verify': 'verifying'}.get(job['phase'], outcome)
    actions = ['inspect']
    if (status in ('queued', 'running', 'timeout_unknown') and job.get('cancellable')
            and not job.get('cancellation_requested') and job.get('finished_at_ns') is None):
        actions.append('cancel')
    return dict(outcome=outcome, unknown_reason=(job.get('error') or {}).get('message') if outcome == 'unknown' else None,
                allowed_next_actions=actions)


def job_summary(job):
    """Polling carries bounded progress; the immutable full receipt stays queryable."""
    if job is None:return None
    row={key:deepcopy(job.get(key)) for key in ('job_id','request','label','status','phase','progress',
        'created_at_ns','updated_at_ns','finished_at_ns','timed_out','cancellation_requested','cancellable',
        'use_case_id','outcome','unknown_reason','allowed_next_actions')}
    request=row['request']
    if isinstance(request.get('expected_target'),dict):
        request['expected_target']={key:value for key,value in request['expected_target'].items() if key in
            ('environment','device_id','manager_instance','backend_instance','deployment_sha256','runtime_nonce','command_epoch')}
    row['verification'] = (
        {key: job['verification'][key] for key in ('ok', 'consumer_detached', 'independent_services_preserved', 'shared_runtime_preserved',
            'environment_exited', 'sdk_release_verified', 'collection_web_ready')
         if key in job['verification']}
        if isinstance(job.get('verification'), dict) else None
    )
    result = job.get('result')
    next_url = result.get('next_url') if isinstance(result, dict) else None
    if isinstance(next_url, str) and len(next_url) <= 2048:
        row['result'] = {'next_url': next_url}
    if isinstance(result,dict) and type(result.get('manager_exit_pending'))is bool:
        row.setdefault('result',{})['manager_exit_pending']=result['manager_exit_pending']
    if job.get('error'):
        row['error']={key:str(job['error'].get(key,''))[:1024] for key in ('code','message')}
    detail=job.get('detail')
    if detail is not None and len(json.dumps(detail,ensure_ascii=False))<=1024:row['detail']=deepcopy(detail)
    elif isinstance(detail,dict)and isinstance(detail.get('step_refs'),list):
        refs=[ref for ref in detail['step_refs'][:32]if isinstance(ref,str)and re.fullmatch(r'(?:UC|SC)\d{2}-S\d{2}',ref)]
        if refs:row['detail']={'step_refs':refs}
    row['detail_url']='/api/operations/jobs/'+job['job_id']
    return row


class OperationService:
    def __init__(self, operations, *, target, receipt_dir):
        rows = list(operations.values() if isinstance(operations, dict) else operations)
        self.operations = {row.name: row for row in rows}
        if len(self.operations) != len(rows):
            raise ValueError('duplicate operation name')
        for row in rows:
            if (not re.fullmatch(r'[a-z][a-z0-9_]{0,63}', row.name)
                    or not row.label or not row.impact
                    or not all(callable(fn) for fn in (row.check, row.run, row.verify))
                    or not(type(row.requires_support)is bool or callable(row.requires_support)) or type(row.cancellable) is not bool
                    or not isinstance(row.parameter_keys, tuple) or any(not isinstance(key, str) for key in row.parameter_keys)
                    or row.use_case_id is not None and (not isinstance(row.use_case_id, str)
                        or not re.fullmatch(r'UC[0-9]{2}', row.use_case_id))
                    or row.timeout_s is not None and (type(row.timeout_s) not in (int, float)
                        or not math.isfinite(row.timeout_s) or row.timeout_s <= 0)):
                raise ValueError('invalid trusted operation registration')
        self.target = target
        self.directory = Path(receipt_dir)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.lock = threading.RLock()
        self.jobs = {}
        self.active = None
        self.closed = False
        self.storage_error = None
        self.cancel_event = None
        self.thread = None
        self._fd = os.open(self.directory/'manager.lock', os.O_CREAT | os.O_RDWR, 0o600)
        try:
            fcntl.flock(self._fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self._restore()
        except BaseException:
            os.close(self._fd)
            self._fd = None
            raise

    def _restore(self):
        for path in sorted(self.directory.glob('job-*.json')):
            job = json.loads(path.read_text())
            if (job.get('schema') != 'robot_workbench.operation.v1'
                    or not isinstance(job.get('request'), dict)
                    or path.name != 'job-'+job.get('job_id', '')+'.json'):
                raise ValueError('invalid operation receipt: '+path.name)
            request_id = job['request']['request_id']
            if request_id in self.jobs:
                raise ValueError('duplicate request identity in operation receipts')
            if job['status'] in ('queued', 'running', 'cancel_requested', 'timeout_unknown'):
                job.update(status='interrupted_unknown', phase='recovery',
                    updated_at_ns=time.time_ns(), error=dict(code='manager_interrupted',
                    message='Manager restarted; external operation state is unknown. No automatic retry.'))
                self._save(job)
            job.update(_receipt_fields(job))
            self.jobs[request_id] = job

    def _save(self, job):
        job.update(_receipt_fields(job))
        filename = self.directory/('job-'+job['job_id']+'.json')
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(mode='w', dir=self.directory, prefix='.receipt-',
                                             delete=False, encoding='utf-8') as stream:
                temporary = Path(stream.name)
                json.dump(job, stream, allow_nan=False, indent=2)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, filename)
            directory_fd = os.open(self.directory, os.O_RDONLY)
            try:
                os.fsync(directory_fd)
            finally:
                os.close(directory_fd)
        except BaseException as exc:
            self.storage_error = str(exc)
            raise
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)

    def _preflight(self, operation, request):
        try:actual=_identity(self.target())
        except Exception as error:
            raise OperationError('target_unknown','Target state could not be verified; no external operation began.',
                unknown=True,details=dict(phase='preflight',exception=type(error).__name__))from None
        if not _same(actual, request['expected_target']):
            raise OperationError('target_changed', 'Target identity changed; refresh and review the operation.')
        if operation.support_required(request) and request['arms_supported'] is not True:
            raise OperationError('support_required', 'Confirm both arms are supported for this operation and target.')
        reason = operation.check(deepcopy(request))
        if reason is not None:
            if not isinstance(reason, str) or not reason:
                raise OperationError('invalid_check', 'Operation check must return None or a nonempty reason.')
            raise OperationError('precondition_failed', reason)

    def snapshot(self,*,summary=False):
        with self.lock:
            target_error = None
            try:
                target = _identity(self.target())
            except Exception as exc:
                target, target_error = None, str(exc)
            busy = self.active is not None
            operations = {}
            recent = sorted(self.jobs.values(), key=lambda row: row['created_at_ns'], reverse=True)
            copy_job=job_summary if summary else deepcopy
            for name, operation in self.operations.items():
                support=operation.requires_support if type(operation.requires_support)is bool else True
                reason = (self.storage_error or ('Manager is closed.' if self.closed else None)
                          or ('Another management operation is still in flight.' if busy else None)
                          or target_error)
                if reason is None:
                    try:
                        # Support is separately confirmed on each start, never granted by a GET.
                        request = dict(action=name, request_id='snapshot', expected_target=target,
                                       arms_supported=False)
                        support=operation.support_required(request)
                        reason = operation.check(deepcopy(request))
                        if reason is not None and (not isinstance(reason, str) or not reason):
                            reason = 'Invalid operation check result.'
                    except Exception as exc:
                        reason = str(exc) or type(exc).__name__
                latest = next((row for row in recent if row['request']['action'] == name), None)
                operations[name] = dict(name=name, label=operation.label, impact=operation.impact,
                    use_case_id=operation.use_case_id,
                    allowed=reason is None, disabled_reason=reason, requires_support=support,
                    cancellable=operation.cancellable, latest_job=copy_job(latest))
            return dict(schema='robot_workbench.operations.v1', target=target, target_error=target_error,
                operations=operations, busy=busy, active_job=copy_job(self.jobs.get(self.active)),
                recent_jobs=[copy_job(row) for row in recent[:20]], storage_error=self.storage_error,
                safety_stop_path='independent_runtime_stop',
                recovery_requires_live_preflight=True)

    def start(self, action, request_id, expected_target, arms_supported=False, parameters=None):
        if not isinstance(action, str) or action not in self.operations:
            raise OperationError('unknown_operation', 'Operation is not registered by this deployment.')
        if not isinstance(request_id, str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,128}', request_id):
            raise OperationError('request_invalid', 'A stable request_id of 1–128 letters, digits, _ or - is required.')
        if type(arms_supported) is not bool:
            raise OperationError('request_invalid', 'arms_supported must be a boolean.')
        request = dict(action=action, request_id=request_id,
                       expected_target=_identity(expected_target), arms_supported=arms_supported)
        if parameters is not None:
            if (not isinstance(parameters, dict) or set(parameters)-set(self.operations[action].parameter_keys)
                    or len(json.dumps(parameters, allow_nan=False)) > 4096):
                raise OperationError('request_invalid', 'Operation parameters must be a bounded object.')
            request['parameters'] = _json(parameters)
        with self.lock:
            previous = self.jobs.get(request_id)
            if previous is not None:
                if not _same(previous['request'], request):
                    raise OperationError('idempotency_conflict', 'request_id is already bound to a different request.')
                return deepcopy(previous)
            if self.closed or self.storage_error:
                raise OperationError('manager_unavailable', self.storage_error or 'Manager is closed.')
            if self.active is not None:
                raise OperationError('operation_busy', 'Another management operation is still in flight.')
            operation = self.operations[action]
            self._preflight(operation, request)
            now = time.time_ns()
            job = dict(schema='robot_workbench.operation.v1', job_id=uuid.uuid4().hex,
                use_case_id=operation.use_case_id,
                request=request, label=operation.label, impact=operation.impact,
                status='queued', phase='queued', progress=0., detail=None,
                created_at_ns=now, updated_at_ns=now, finished_at_ns=None,
                result=None, verification=None, error=None, timed_out=False,
                cancellation_requested=False, cancellable=operation.cancellable,
                events=[dict(phase='queued', at_ns=now, monotonic_ns=time.monotonic_ns())])
            self._save(job)  # No side effect is allowed before durable request admission.
            self.jobs[request_id] = job
            self.active = request_id
            self.cancel_event = threading.Event()
            self.thread = threading.Thread(target=self._execute, args=(operation, job, self.cancel_event),
                                           name='management-'+action, daemon=True)
            self.thread.start()
            return deepcopy(job)

    def _update(self, job, **changes):
        with self.lock:
            if 'phase' in changes and changes['phase'] != job['phase']:
                stamp = time.monotonic_ns()
                job['events'][-1]['finished_monotonic_ns'] = stamp
                job['events'].append(dict(phase=changes['phase'], at_ns=time.time_ns(), monotonic_ns=stamp))
            if 'detail' in changes:
                job['events'][-1]['detail'] = deepcopy(changes['detail'])
            if changes.get('finished_at_ns') is not None:
                job['events'][-1]['finished_monotonic_ns'] = time.monotonic_ns()
            job.update(changes, updated_at_ns=time.time_ns())
            self._save(job)

    def _timeout(self, job, cancel_event):
        with self.lock:
            if self.active != job['request']['request_id'] or job['finished_at_ns'] is not None:
                return
            if job['cancellable']:
                cancel_event.set()
            self._update(job, status='timeout_unknown', timed_out=True,
                cancellation_requested=job['cancellation_requested'] or job['cancellable'],
                error=dict(code='operation_timeout', message='Deadline passed; external state is unknown. '
                           'The callback has not been killed and no new operation may start yet.'))

    def _execute(self, operation, job, cancel_event):
        timer = None
        effects_started = False
        def progress(phase, fraction=None, detail=None):
            if not isinstance(phase, str) or not phase or len(phase) > 128:
                raise ValueError('progress phase must be nonempty text of at most 128 characters')
            changes = dict(phase=phase, detail=_json(detail))
            if fraction is not None:
                if type(fraction) not in (int, float) or not math.isfinite(fraction) or not 0 <= fraction <= 1:
                    raise ValueError('progress fraction must be in [0, 1]')
                changes['progress'] = fraction
            self._update(job, **changes)
        try:
            self._update(job, status='running', phase='preflight')
            if operation.timeout_s is not None:
                timer = threading.Timer(operation.timeout_s, self._timeout, args=(job, cancel_event))
                timer.daemon = True
                timer.start()
            self._preflight(operation, job['request'])
            if job['timed_out']:
                raise OperationError('timeout_before_run', 'Deadline passed before side effects began.')
            if cancel_event.is_set():
                raise OperationError('cancelled_before_run', 'Cancellation acknowledged before side effects began.')
            self._update(job, phase='run')
            effects_started = True
            result = _json(operation.run(deepcopy(job['request']), progress, cancel_event))
            self._update(job, phase='verify', result=result)
            verification = _json(operation.verify(deepcopy(job['request']), deepcopy(result)))
            if not isinstance(verification, dict) or type(verification.get('ok')) is not bool:
                raise OperationError('invalid_verification', 'Verification must return an explicit boolean ok.', unknown=True)
            self._update(job, verification=verification)
            if verification['ok'] is not True:
                raise OperationError('verification_unknown' if verification.get('unknown') else 'verification_failed',
                    str(verification.get('reason') or 'Postcondition not verified.'), unknown=verification.get('unknown') is True)
            # A cooperative callback may acknowledge cancellation, but only verify
            # can establish its postcondition. An event alone never means stopped.
            cancelled = (job['cancellation_requested'] and isinstance(result, dict)
                         and result.get('cancelled') is True and verification.get('cancelled') is True)
            self._update(job, status='cancelled' if cancelled else 'succeeded', phase='complete',
                         progress=1., error=None, finished_at_ns=time.time_ns())
        except Exception as exc:
            code = getattr(exc, 'code', type(exc).__name__)
            # Only an explicit operation error establishes a known failure after
            # effects begin. Transport/parser/verification failures leave the
            # external result unproven and must never qualify a blind retry.
            unknown = exc.unknown if isinstance(exc, OperationError) else effects_started
            try:
                error=dict(code=code,message=str(exc))
                if getattr(exc,'details',None) is not None:error['details']=deepcopy(exc.details)
                self._update(job, status='cancelled' if code == 'cancelled_before_run' else 'unknown' if unknown else 'failed',
                    error=error, finished_at_ns=time.time_ns())
            except Exception:
                # Fail closed on an unrecordable outcome; snapshot exposes the
                # storage error and the last durable receipt recovers as unknown.
                pass
        finally:
            if timer is not None:
                timer.cancel()
            with self.lock:
                self.active = None
                self.cancel_event = None
                if self.closed:
                    self._unlock()

    def cancel(self, request_id):
        with self.lock:
            job = self.jobs.get(request_id)
            if job is None:
                raise OperationError('unknown_job', 'Unknown operation request.')
            if self.active != request_id or job['finished_at_ns'] is not None:
                raise OperationError('not_running', 'No live callback can acknowledge cancellation for this job.')
            if not job['cancellable']:
                raise OperationError('not_cancellable', 'This operation cannot be cancelled; no SDK process was killed.')
            if not job['cancellation_requested']:
                self._update(job, cancellation_requested=True,
                             status='timeout_unknown' if job['timed_out'] else 'cancel_requested')
                self.cancel_event.set()
            return deepcopy(job)

    def _unlock(self):
        if self._fd is not None:
            os.close(self._fd)
            self._fd = None

    def close(self):
        """Reject new work; let an existing callback finish without killing it."""
        with self.lock:
            self.closed = True
            if self.active is None:
                self._unlock()
