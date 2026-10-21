"""Environment facts use no business credential and perform no lifecycle action."""
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from urllib.error import HTTPError

sys.path[:0]=[str(Path(__file__).resolve().parents[1]),str(Path(__file__).resolve().parent)]
from test_http_service import HTTPTest


class EnvironmentStateTests(HTTPTest):
    def get(self,path):
        with self.http.open(self.url+path,timeout=2) as response:return json.loads(response.read())

    def test_environment_http_has_shared_facts_without_business_or_connection(self):
        state=self.get('/api/environment?environment=fake')
        self.assertEqual(state['schema'],'robot_workbench.environment.v1')
        self.assertFalse(state['runtime']['connected']);self.assertFalse(state['activity']['busy'])
        self.assertEqual(state['device']['environment'],'fake');self.assertIsNone(self.device.runtime)
        self.assertEqual(set(state)&{'runtime','activity','maintenance','model','collection','cameras'},
            {'runtime','activity','maintenance','model','collection','cameras'})
        for path in ('/api/environment','/api/environment?environment=wrong','/api/environment?environment=fake&business=inference'):
            with self.assertRaises(HTTPError):self.get(path)

    def test_connected_shared_snapshot_does_not_authenticate_a_business(self):
        self.connect();runtime=self.device.runtime
        with patch.object(runtime,'_authenticate',side_effect=AssertionError('facts must not impersonate a holder')):
            facts=self.app.environment_state('fake')
        self.assertTrue(facts['runtime']['connected'])
        self.assertEqual(facts['runtime']['holder'],runtime.holder)
        self.assertNotIn('allowed_operations',facts['runtime'])
        self.assertIn('control_operations',facts['runtime'])

    def test_model_and_collection_use_environment_hooks_and_unknown_stays_unknown(self):
        self.device.workflows['inference']=SimpleNamespace(snapshot=lambda:dict(busy=False),
            environment_status=lambda:dict(status='unknown',loaded=None,ready=None,selected_model_id='selected'))
        self.device.collection_preparation=SimpleNamespace(snapshot=lambda:dict(status='waiting_calibration',busy=False))
        facts=self.app.environment_state('fake')
        self.assertIsNone(facts['model']['ready']);self.assertIsNone(facts['model']['loaded'])
        self.assertEqual(facts['collection']['status'],'waiting_calibration')
        self.assertFalse(facts['runtime']['connected'])


def load_tests(loader,tests,pattern):
    return unittest.TestSuite(EnvironmentStateTests(name) for name in EnvironmentStateTests.__dict__ if name.startswith('test_'))


if __name__=='__main__':unittest.main()
