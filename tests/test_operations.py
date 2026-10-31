"""Management jobs use CPU callbacks; no runtime, SDK or subprocess is started."""
from copy import deepcopy
import json
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import threading
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from robot_workbench.operations import Operation, OperationError, OperationService


TARGET = dict(environment='physical', device_id='cpu-fixture', deployment_id='reviewed-v1')


class OperationTests(unittest.TestCase):
    def test_support_policy_is_live_boolean_and_never_granted_by_snapshot(self):
        required=[False];calls=[]
        service=self.service(requires_support=lambda request:required[0],run=lambda *args:calls.append(True)or{})
        self.assertFalse(service.snapshot()['operations']['restart']['requires_support'])
        required[0]=True
        self.assertTrue(service.snapshot()['operations']['restart']['requires_support'])
        with self.assertRaisesRegex(OperationError,'supported'):service.start('restart','no-support',TARGET)
        self.assertFalse(calls)
        required[0]=False;service.start('restart','request-1',TARGET)
        self.assertEqual(self.wait(service)['status'],'succeeded');self.assertEqual(calls,[True])

    def test_invalid_support_policy_fails_closed(self):
        service=self.service(requires_support=lambda request:'false')
        self.assertFalse(service.snapshot()['operations']['restart']['allowed'])
        with self.assertRaisesRegex(OperationError,'boolean'):service.start('restart','bad-policy',TARGET)

    def test_failed_receipt_keeps_the_individual_validation_evidence(self):
        detail=dict(checks={'same_runtime':True,'fresh_publication':False},state='FAULT_STOP_UNCONFIRMED')
        def run(request,progress,cancel):raise OperationError('recovery_unconfirmed','injected stale reply',unknown=True,details=detail)
        service=self.service(run=run)
        service.start('restart',request_id='request-1',expected_target=self.target)
        result=self.wait(service)
        self.assertEqual(result['status'],'unknown');self.assertEqual(result['error']['details'],detail)
        saved=json.loads(next(Path(self.directory.name).glob('job-*.json')).read_text())
        self.assertEqual(saved['error']['details'],detail)

    def setUp(self):
        self.directory = TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.target = deepcopy(TARGET)
        self.release = threading.Event()
        self.addCleanup(self.release.set)
        self.services = []
        self.addCleanup(lambda: [service.close() for service in self.services])

    def service(self, *, check=lambda request: None, run=lambda request, progress, cancel: {},
                verify=lambda request, result: {'ok': True}, requires_support=False,
                cancellable=False, timeout_s=None):
        operation = Operation('restart', 'Restart worker', 'Restart only the registered worker',
                              check, run, verify, requires_support, cancellable, timeout_s)
        service = OperationService([operation], target=lambda: deepcopy(self.target),
                                   receipt_dir=self.directory.name)
        self.services.append(service)
        return service

    def wait(self, service, request_id='request-1'):
        deadline = time.monotonic()+2
        while service.snapshot()['busy'] and time.monotonic() < deadline:
            time.sleep(.005)
        self.assertFalse(service.snapshot()['busy'])
        return next(job for job in service.snapshot()['recent_jobs'] if job['request']['request_id'] == request_id)

    def test_idempotency_binds_payload_and_survives_restart(self):
        calls = []
        service = self.service(run=lambda request, progress, cancel: calls.append(request) or {'started': True})
        admitted = service.start('restart', 'request-1', TARGET)
        job = self.wait(service)
        self.assertEqual(job['status'], 'succeeded')
        self.assertEqual(service.start('restart', 'request-1', TARGET)['job_id'], admitted['job_id'])
        with self.assertRaisesRegex(OperationError, 'different request'):
            service.start('restart', 'request-1', TARGET, True)
        service.close()
        restored = self.service(run=lambda *args: self.fail('must not rerun'))
        self.assertEqual(restored.start('restart', 'request-1', TARGET)['job_id'], admitted['job_id'])
        self.assertEqual(len(calls), 1)

    def test_concurrent_starts_are_exclusive_and_duplicate_is_safe(self):
        entered = threading.Event()
        def run(request, progress, cancel):
            entered.set()
            self.release.wait(2)
            return {}
        service = self.service(run=run)
        first = service.start('restart', 'request-1', TARGET)
        self.assertTrue(entered.wait(1))
        self.assertEqual(service.start('restart', 'request-1', TARGET)['job_id'], first['job_id'])
        errors = []
        def start(index):
            try:
                service.start('restart', 'other-'+str(index), TARGET)
            except OperationError as exc:
                errors.append(exc.code)
        threads = [threading.Thread(target=start, args=(index,)) for index in range(6)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
        self.assertEqual(errors, ['operation_busy']*6)
        self.assertFalse(service.snapshot()['operations']['restart']['allowed'])
        self.release.set()
        self.wait(service)

    def test_preflight_is_rechecked_before_effects(self):
        checks = []
        def check(request):
            checks.append(request)
            return 'Lease changed' if len(checks) > 1 else None
        service = self.service(check=check, run=lambda *args: self.fail('preflight must block effects'))
        service.start('restart', 'request-1', TARGET)
        job = self.wait(service)
        self.assertEqual(job['status'], 'failed')
        self.assertEqual(job['error']['code'], 'precondition_failed')
        self.assertEqual(job['phase'], 'preflight')

    def test_identity_is_bound_and_rechecked(self):
        def check(request):
            self.target['deployment_id'] = 'replacement'
        service = self.service(check=check, run=lambda *args: self.fail('identity changed'))
        service.start('restart', 'request-1', TARGET)
        job = self.wait(service)
        self.assertEqual(job['error']['code'], 'target_changed')
        with self.assertRaises(OperationError):
            service.start('restart', 'request-2', TARGET)

    def test_postcondition_failure_is_not_success_or_stopped(self):
        service = self.service(verify=lambda request, result: {'ok': False, 'reason': 'Worker still unhealthy'})
        service.start('restart', 'request-1', TARGET)
        job = self.wait(service)
        self.assertEqual(job['status'], 'failed')
        self.assertEqual(job['phase'], 'verify')
        self.assertEqual(job['error']['code'], 'verification_failed')
        self.assertFalse(job['verification']['ok'])

    def test_only_explicit_boolean_verification_can_succeed(self):
        service = self.service(verify=lambda request, result: {'ok': 1})
        service.start('restart', 'request-1', TARGET)
        self.assertEqual(self.wait(service)['error']['code'], 'invalid_verification')

    def test_unknown_verification_remains_unknown(self):
        service = self.service(verify=lambda request, result: {'ok': False, 'unknown': True, 'reason': 'Feedback unavailable'})
        service.start('restart', 'request-1', TARGET)
        self.assertEqual(self.wait(service)['status'], 'unknown')

    def test_parameters_are_registered_and_part_of_idempotency(self):
        service = self.service()
        with self.assertRaises(OperationError):
            service.start('restart', 'request-1', TARGET, parameters={'command': 'anything'})
        service.close()
        operation = Operation('prepare', 'Prepare', 'Selected model only', lambda request: None,
            lambda request, progress, cancel: request['parameters'], lambda request, result: {'ok': True},
            parameter_keys=('model_id',))
        service = OperationService([operation], target=lambda: TARGET, receipt_dir=self.directory.name)
        self.services.append(service)
        first = service.start('prepare', 'request-1', TARGET, parameters={'model_id': 'selected'})
        self.assertEqual(self.wait(service)['result'], {'model_id': 'selected'})
        self.assertEqual(service.start('prepare', 'request-1', TARGET, parameters={'model_id': 'selected'})['job_id'], first['job_id'])
        with self.assertRaises(OperationError):
            service.start('prepare', 'request-1', TARGET, parameters={'model_id': 'changed'})

    def test_fixed_request_interface_rejects_command_and_path_arguments(self):
        service = self.service()
        for extra in ({'shell': 'anything'}, {'path': '/tmp/anything'}, {'params': {'force': True}}):
            with self.assertRaises(TypeError):
                service.start('restart', 'request-1', TARGET, **extra)
        for action in ('not_registered', '../restart'):
            with self.assertRaises(OperationError):
                service.start(action, 'request-1', TARGET)
        self.assertEqual(service.snapshot()['recent_jobs'], [])

    def test_restart_recovers_unknown_without_replay(self):
        service = self.service()
        service.start('restart', 'request-1', TARGET)
        job = self.wait(service)
        service.close()
        path = Path(self.directory.name)/('job-'+job['job_id']+'.json')
        receipt = json.loads(path.read_text())
        receipt.update(status='running', phase='run', finished_at_ns=None)
        path.write_text(json.dumps(receipt))
        restored = self.service(run=lambda *args: self.fail('restart must not replay'))
        recovered = restored.start('restart', 'request-1', TARGET)
        self.assertEqual(recovered['status'], 'interrupted_unknown')
        self.assertIsNone(recovered['finished_at_ns'])
        self.assertEqual(json.loads(path.read_text())['status'], 'interrupted_unknown')

    def test_support_confirmation_has_operation_request_and_target_scope(self):
        service = self.service(requires_support=True)
        with self.assertRaises(OperationError) as caught:
            service.start('restart', 'request-1', TARGET)
        self.assertEqual(caught.exception.code, 'support_required')
        service.start('restart', 'request-1', TARGET, True)
        self.wait(service)
        with self.assertRaises(OperationError):
            service.start('restart', 'request-2', TARGET)
        wrong_target = dict(TARGET, environment='fake')
        with self.assertRaises(OperationError):
            service.start('restart', 'request-2', wrong_target, True)
        with self.assertRaises(OperationError):
            service.start('restart', 'request-2', TARGET, 'true')

    def test_timeout_keeps_exclusivity_until_late_result_is_verified(self):
        entered = threading.Event()
        def run(request, progress, cancel):
            progress('waiting_for_worker', .25, {'scope': 'registered-worker'})
            entered.set()
            self.release.wait(2)
            return {'worker': 'ready'}
        service = self.service(run=run, timeout_s=.03)
        service.start('restart', 'request-1', TARGET)
        self.assertTrue(entered.wait(1))
        deadline = time.monotonic()+1
        while not service.snapshot()['active_job']['timed_out'] and time.monotonic() < deadline:
            time.sleep(.005)
        state = service.snapshot()
        self.assertTrue(state['busy'])
        self.assertEqual(state['active_job']['status'], 'timeout_unknown')
        with self.assertRaises(OperationError):
            service.start('restart', 'request-2', TARGET)
        self.release.set()
        job = self.wait(service)
        self.assertEqual(job['status'], 'succeeded')
        self.assertTrue(job['timed_out'])
        self.assertTrue(job['verification']['ok'])

    def test_cancellation_requires_callback_and_verified_acknowledgement(self):
        entered = threading.Event()
        def run(request, progress, cancel):
            entered.set()
            cancel.wait(2)
            return {'cancelled': True}
        service = self.service(run=run, cancellable=True,
                               verify=lambda request, result: {'ok': True, 'cancelled': True})
        service.start('restart', 'request-1', TARGET)
        self.assertTrue(entered.wait(1))
        self.assertTrue(service.cancel('request-1')['cancellation_requested'])
        self.assertEqual(self.wait(service)['status'], 'cancelled')

    def test_cancel_cannot_reopen_a_finished_receipt_during_worker_teardown(self):
        service = self.service(cancellable=True)
        service.start('restart', 'request-1', TARGET)
        self.assertEqual(self.wait(service)['status'], 'succeeded')
        with service.lock:
            service.active = 'request-1'  # Simulate the short completion/finally interval.
            try:
                with self.assertRaises(OperationError):
                    service.cancel('request-1')
                self.assertEqual(service.jobs['request-1']['status'], 'succeeded')
            finally:
                service.active = None

    def test_close_and_refused_cancel_do_not_kill_worker_or_release_process_lock(self):
        entered = threading.Event()
        def run(request, progress, cancel):
            entered.set()
            self.release.wait(2)
            self.assertFalse(cancel.is_set())
            return {}
        service = self.service(run=run)
        service.start('restart', 'request-1', TARGET)
        self.assertTrue(entered.wait(1))
        with self.assertRaisesRegex(OperationError, 'cannot be cancelled'):
            service.cancel('request-1')
        service.close()
        with self.assertRaises(BlockingIOError):
            self.service()
        self.release.set()
        self.assertEqual(self.wait(service)['status'], 'succeeded')
        self.service().close()


if __name__ == '__main__':
    unittest.main()
