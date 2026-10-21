"""CLI assembly stays inert for SDK/model startup; all effects are CPU fakes."""
import contextlib
import importlib.util
import io
import json
import os
from pathlib import Path
import sys
import time
import threading
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import Mock,patch

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
spec=importlib.util.spec_from_file_location('physical_cli_cpu_fixture',ROOT/'scripts/serve_physical_device.py')
cli=importlib.util.module_from_spec(spec);spec.loader.exec_module(cli)


class ServePhysicalDeviceTests(unittest.TestCase):
    def test_explicit_shutdown_requires_verified_guard_retirement_or_normal_release(self):
        runtime=SimpleNamespace(owner=SimpleNamespace(backend=SimpleNamespace(terminal_failure=None)),
            retire_protected=Mock(return_value={'worker_exit_confirmed':True,'sdk_close_confirmed':False}))
        device=SimpleNamespace(runtime=runtime,lock=threading.Lock(),closed=threading.Event(),credentials={'old':'credential'})
        self.assertFalse(cli.retire_exited_control(device));runtime.retire_protected.assert_not_called()
        runtime.owner.backend.terminal_failure='heartbeat lost'
        runtime.retire_protected.side_effect=ValueError('worker exit not confirmed')
        self.assertFalse(cli.retire_exited_control(device));self.assertIs(device.runtime,runtime)
        self.assertFalse(device.closed.is_set())
        runtime.retire_protected.side_effect=None
        self.assertTrue(cli.retire_exited_control(device));self.assertIsNone(device.runtime)
        self.assertTrue(device.closed.is_set());self.assertEqual(device.credentials,{})
        self.assertFalse(device.protected_shutdown_retirement['sdk_close_confirmed'])

    def test_preflight_never_builds_or_supervises_hardware_or_models(self):
        with patch.object(sys,'argv',['serve','--profile','/cpu/profile.json','--preflight']),\
                patch.object(cli,'read_profile',return_value={'robot_id':'arx6'}),\
                patch.object(cli,'preflight',return_value={'configuration_ready':True,'models_started':False}),\
                patch.object(cli,'build_physical_application',side_effect=AssertionError('preflight cannot start service')),\
                patch('subprocess.Popen',side_effect=AssertionError('preflight cannot spawn')):
            with contextlib.redirect_stdout(io.StringIO()):self.assertEqual(cli.main(),0)

    def test_started_service_receipt_exposes_model_only_preparation_without_autostart(self):
        with TemporaryDirectory() as temporary:
            output=Path(temporary)/'service'
            profile=dict(robot_id='arx6',storage_root=temporary,
                inference={'model_endpoint':'http://127.0.0.1:49379'})
            inference=SimpleNamespace(selected_model_id='base566-epoch007-step58982',
                catalog_root='/approved/checkpoints',disabled_grippers=('right',))
            deployment=SimpleNamespace(device=SimpleNamespace(runtime=None),application=object(),
                camera_ports=dict(head=1,left=2,right=3),inference=inference)
            def build(*args,**kwargs):output.mkdir();return deployment
            server=Mock(server_port=18868)
            with patch.object(sys,'argv',['serve','--profile','/cpu/profile.json','--output',str(output),'--supervised-child']),\
                    patch.object(cli,'read_profile',return_value=profile),\
                    patch.object(cli,'preflight',return_value={'configuration_ready':True}),\
                    patch.object(cli,'build_physical_application',side_effect=build) as assembled,\
                    patch.object(cli,'close_deployment') as close,patch.object(cli.signal,'signal'),\
                    patch('robot_workbench.arx_output_guard.bind_child',side_effect=AssertionError('HTTP must not be guard child')),\
                    patch('robot_workbench.arx_output_guard.supervise',side_effect=AssertionError('HTTP must survive SDK latch')),\
                    patch('robot_workbench.service.serve',return_value=server),\
                    contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(cli.main(),0)
            self.assertFalse(assembled.call_args.kwargs['start_cameras'])
            self.assertIsNone(deployment.device.runtime)
            close.assert_called_once_with(deployment)
            receipt=json.loads((output/'service.json').read_text())
            self.assertFalse(receipt['hardware_connected']);self.assertFalse(receipt['model_loaded'])
            self.assertEqual(receipt['inference']['model_id'],'base566-epoch007-step58982')
            self.assertEqual(receipt['inference']['disabled_grippers'],['right'])
            self.assertFalse(receipt['inference']['model_service_started_by_workbench'])
            self.assertFalse(receipt['inference']['model_prepare_requires_device_connection'])
            self.assertFalse(receipt['inference']['control_ready'])


if __name__=='__main__':unittest.main()
