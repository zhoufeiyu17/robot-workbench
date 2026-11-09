"""The browser switch consumes the same verified manager job as the normal UI."""
from contextlib import contextmanager
from copy import deepcopy
import unittest
from unittest.mock import Mock,patch

import test_environment_exit as exit_fixture
from robot_workbench.management import WorkbenchManager
from robot_workbench.operations import job_summary


class UserWorkbenchHandoffTests(unittest.TestCase):
    def test_platform_start_is_after_consumer_exit_and_before_navigation(self):
        manager=exit_fixture.EnvironmentExitTest().manager()
        manager.config.update(control_lifecycle='external',model_lifecycle='external',
            control_consumer_id='C',consumer_handoff_url='http://collector.invalid:18080/',
            consumer_handoff_web={'registered':'fixture'})
        manager.health_cache.update(control_owner_reachable=True)
        manager.health_cache['runtime'].update(control_owner={'consumer_holder':None},feedback_healthy=True)
        manager.target=Mock(return_value={'runtime_nonce':'cpu'})
        def ready(*args,**kwargs):
            self.assertEqual(manager.calls,['stop_workbench'])
            return dict(ready=True,collection_started=False)
        with patch('scripts.workbench_deploy.restore_collection_web',side_effect=ready)as start:
            result=manager.exit_environment({'request_id':'platform'},Mock())
            self.assertTrue(result['collection_web']['ready'])
            self.assertEqual(result['next_url'],manager.config['consumer_handoff_url'])
            start.assert_called_once()

    def test_peer_url_rejects_invalid_or_nonconsumer_configuration_early(self):
        base = dict(schema='robot_workbench.manager.v1', environment='physical', device_id='arx6',
                    backend_port=18878, state_root='/tmp/handoff-config-only', control_lifecycle='external',
                    control_consumer_id='workbench-C', control_owner_descriptor='/cpu/C.json', model_lifecycle='external')
        for value in ('javascript:alert(1)', 'file:///tmp/a', 'http://user:password@host/',
                      'http://host:bad/', 'http://host/ with-space', '', 123):
            with self.subTest(value=value), self.assertRaises(ValueError):
                WorkbenchManager(dict(base, consumer_handoff_url=value), monitor=False)
        with self.assertRaises(ValueError):
            WorkbenchManager(dict(base, control_lifecycle='managed', consumer_handoff_url='http://host/'), monitor=False)

    def test_verified_consumer_exit_returns_only_the_configured_destination(self):
        manager = exit_fixture.EnvironmentExitTest().manager()
        manager.config.update(control_lifecycle='external', control_consumer_id='workbench-C',
                              consumer_handoff_url='http://collector.invalid:18080/', model_lifecycle='external')
        manager.health_cache.update(control_owner_reachable=True)
        manager.health_cache['runtime']['control_owner'] = dict(consumer_holder=None)
        manager.health_cache['runtime']['feedback_healthy'] = True
        manager.target = Mock(return_value={'runtime_nonce': 'cpu'})
        result = manager.exit_environment( {'request_id': 'switch'}, Mock())
        self.assertEqual(result['next_url'], manager.config['consumer_handoff_url'])
        self.assertEqual(manager.calls, ['stop_workbench'])
        manager.home_for_exit.assert_not_called()
        manager.stop_current_control.assert_not_called()

    def test_poll_summary_carries_detach_proof_and_link_without_full_private_result(self):
        job = dict(job_id='switch-job', request={'action': 'exit_environment', 'request_id': 'switch'},
                   status='succeeded', verification={'ok': True, 'consumer_detached': True,
                   'independent_services_preserved': True, 'private': 'not-public'},
                   result={'next_url': 'http://collector.invalid:18080/', 'manager_exit_pending': True, 'private': 'not-public'})
        summary = job_summary(job)
        self.assertEqual(summary['result'], {'next_url': 'http://collector.invalid:18080/', 'manager_exit_pending': True})
        self.assertEqual(summary['verification'], dict(ok=True, consumer_detached=True, independent_services_preserved=True))
        self.assertNotIn('not-public', str(summary))
        for result in (None, 'completed', 1, ['completed']):
            with self.subTest(result=result):
                self.assertNotIn('result', job_summary(dict(job, result=result)))

    def test_cold_consumer_rebind_keeps_the_running_models_bundle_identity(self):
        manager = exit_fixture.EnvironmentExitTest().manager()
        manager.config['model_port'] = 18925
        manager.health_cache['model_service'] = dict(loaded=True, status='ready', model_id='selected',
            bundle_digest='a'*64, warmup={'completed': True})
        manager.model_evidence = Mock(return_value={'worker_pid': 42})
        binding = manager.warm_model_binding()
        manager.model_evidence.assert_called_once_with('selected', 'a'*64)
        self.assertEqual(binding['bundle_digest'], 'a'*64)

    def test_external_rebind_uses_local_model_maintenance_and_ready_only_identity(self):
        manager = exit_fixture.EnvironmentExitTest().manager()
        manager.config['control_lifecycle'] = 'external'
        calls = []
        @contextmanager
        def maintenance(purpose):
            calls.append(purpose)
            yield 'local-capability'
        manager.maintenance = maintenance
        manager.state = Mock(return_value=dict(model={'selected_model_id': 'selected', 'bundle_digest': 'a'*64},
            runtime={'pid': 7, 'runtime_nonce': 'runtime', 'automatic_enabled': False}))
        evidence = {'service_identity': {'pid': 20}, 'worker_pid': 21}
        manager.model_evidence = Mock(return_value=evidence)
        manager.post = Mock(return_value={'model_ready': True, 'control_ready': False})
        result = manager.restore_warm_model_binding(dict(model_id='selected', bundle_digest='a'*64,
            evidence=deepcopy(evidence)), Mock(), expected_owner={'pid': 7, 'runtime_nonce': 'runtime'})
        self.assertEqual(calls, ['prepare_model'])
        self.assertEqual(result['status'], 'restored')
        self.assertEqual(manager.post.call_args.kwargs['expected_model_id'], 'selected')


if __name__ == '__main__':
    unittest.main()
