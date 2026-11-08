"""Startup pedal routing only; all controllers are explicit CPU fixtures."""
import json
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock,patch

import test_device_bootstrap as bootstrap_fixture
from robot_workbench.device_bootstrap import read_profile,physical_runtime_factory


class PedalInputModeTests(unittest.TestCase):
    setUp=bootstrap_fixture.BootstrapTests.setUp

    def acquire(self,profile,listener=None):
        backend=Mock();runtime=NS(owner=NS(backend=backend,listener=listener,
            control=NS(state='ready',reason=None),states=NS(FAULT='fault')))
        with patch.dict(sys.modules,{'arx_env.pedal_stop':None}),\
             patch('real_control.controller_conflicts',return_value=[]),\
             patch('robot_workbench.device_bootstrap.preflight',return_value={'configuration_ready':True}),\
             patch('examples.scripts.debug.joint_head_real_owner.legacy_modules',return_value=NS()),\
             patch('robot_workbench.runtime.Runtime',return_value=runtime):
            self.assertIs(physical_runtime_factory(profile)(),runtime)
        return runtime,backend

    def test_explicit_external_intents_never_imports_or_constructs_legacy_listener(self):
        value=dict(self.profile,pedal_input_mode='external_intents')
        self.path.write_text(json.dumps(value));self.assertEqual(read_profile(self.path)['pedal_input_mode'],'external_intents')
        runtime,backend=self.acquire(value)
        self.assertIsNone(runtime.owner.listener);self.assertEqual(runtime.owner.control.state,'ready')
        backend.connect.assert_called_once_with();backend.set_damping_hold.assert_called_once_with(manual=True)
        backend.send.assert_not_called()

    def test_invalid_or_dual_pedal_profiles_fail_before_runtime_construction(self):
        invalid=[dict(pedal_input_mode=v)for v in (None,True,[],{},'disabled','external')]
        invalid+=[dict(pedal_input_mode='external_intents',pedal_stop_key_code=v)for v in (5,46,None,0)]
        for fields in invalid:
            with self.subTest(fields=fields):
                value=dict(self.profile,**fields);self.path.write_text(json.dumps(value))
                with self.assertRaises(ValueError):read_profile(self.path)
                with patch('robot_workbench.runtime.Runtime')as construct:
                    with self.assertRaises(ValueError):physical_runtime_factory(value)
                    construct.assert_not_called()

    def test_unexpected_preexisting_listener_is_not_silently_removed(self):
        listener=object();runtime,backend=self.acquire(dict(self.profile,pedal_input_mode='external_intents'),listener)
        self.assertIs(runtime.owner.listener,listener)
        self.assertEqual(runtime.owner.control.state,'fault')
        self.assertIn('no legacy',runtime.initialization_error)
        backend.connect.assert_not_called()

    def test_independent_runtime_stop_still_reaches_both_holds_without_listener(self):
        from test_shared_target_routes import SharedTargetRoutesTest
        fixture=SharedTargetRoutesTest();fixture.profile='normal_X5';fixture.setUp();self.addCleanup(fixture.doCleanups)
        fixture.create();self.assertIsNone(fixture.runtime.owner.listener)
        receipt=fixture.runtime.request_stop();self.assertTrue(receipt['stop']['prior_calls_complete'])
        fixture.advance(10)
        self.assertTrue(fixture.runtime.stop_receipt['stop_hold_return_confirmed'])
        self.assertEqual(set(fixture.runtime.stop_receipt['sdk_hold_returns']),{'left','right'})
        self.assertTrue(fixture.raw.is_connected)


if __name__=='__main__':unittest.main()
