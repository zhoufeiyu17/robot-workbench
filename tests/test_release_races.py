"""CPU-only release races through the real Runtime/Owner; no SDK or GPU loading."""
import ast
from copy import deepcopy
import inspect
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'tests')]
from test_owner_clock import ClockBackend, calibration, jaw
from examples.scripts.debug.joint_head_fake_hardware import FakeSafety
from robot_workbench.runtime import Runtime
from robot_workbench.service import Device


class ReleaseRaceTests(unittest.TestCase):
    def setUp(self):
        jaw.configure_mapping(None)
        self.directory=TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.runtimes=[]
        self.cleanup_receipts=[]
        self.now=10.
        self.addCleanup(self.cleanup)

    def cleanup(self):
        for runtime in self.runtimes:
            # Preserve the tested failure before explicitly disposing only this
            # test's CPU fake. Cleanup is not evidence of a confirmed user release.
            self.cleanup_receipts.append(deepcopy(runtime.release_receipt))
            if runtime.fd is not None:
                runtime.owner.recording=None
                runtime.owner.submit(dict(action='release',confirmation='ARMS_SUPPORTED_RELEASE'))
                runtime.tick()
                runtime.close()

    def create(self, holder='calibration'):
        runtime=Runtime(calibration,FakeSafety,
            lambda:ClockBackend(lambda:self.now,source_clock_id='release-race-cpu-clock'),
            device_id='release-race-'+str(len(self.runtimes)),environment='fake',
            lock_root=self.directory.name,initial_holder=holder,control_clock=lambda:self.now,
            initialize_backend=lambda backend:backend.set_damping_hold(manual=True))
        self.runtimes.append(runtime)
        device=Device(runtime.device_id,'fake',None)
        device.runtime=runtime
        device.credentials={name:runtime.register_business(name) for name in ('calibration','collection')}
        for _ in range(42):self.tick(runtime)
        return runtime,device

    def tick(self,runtime):
        self.now+=.01
        return runtime.tick()

    def test_private_gateway_release_failure_is_recorded_once_without_retry(self):
        # Both existing C/D /operator bridges dispatch through Device.submit.
        for business in ('calibration','collection'):
            with self.subTest(business=business):
                runtime,device=self.create(business)
                backend=runtime.owner.backend._backend
                with patch.object(backend,'close',side_effect=OSError('fixture close refused')) as close:
                    result=device.submit(business,dict(action='release',confirmation='ARMS_SUPPORTED_RELEASE'))
                    self.assertEqual(result['release_receipt']['status'],'requested')
                    for _ in range(10):self.tick(runtime)
                    self.assertEqual(close.call_count,1,'private bridge release automatically retried SDK.close')
                    self.assertEqual(runtime.release_receipt['status'],'unconfirmed')
                    self.assertFalse(runtime.release_receipt['sdk_close_returned'])
                    self.assertFalse(runtime.owner.release_requested)
                    self.assertIsNotNone(runtime.fd,'unconfirmed close must retain the device lock')
                    self.assertIn('fixture close refused',runtime.release_receipt['message'])

    def test_release_completion_during_dispatch_never_retargets_a_new_runtime(self):
        for command in ({'action':'stop'},{'action':'preview_home','active_arm':'left'}):
            with self.subTest(action=command['action']):
                runtime,device=self.create()
                replacement=SimpleNamespace(submit=Mock(side_effect=AssertionError('sent to replacement runtime')))
                lines,start=inspect.getsourcelines(Device.submit)
                dispatch=start+next(i for i,line in enumerate(lines)
                                    if '.submit(' in line and 'return' in line)
                reached=threading.Event()
                def complete_release(frame,event,arg):
                    if frame.f_code is Device.submit.__code__ and event=='line' and frame.f_lineno==dispatch and not reached.is_set():
                        reached.set()
                        runtime.owner.submit(dict(action='release',confirmation='ARMS_SUPPORTED_RELEASE'))
                        self.tick(runtime);runtime.close()
                        device.runtime=replacement
                        device.credentials={}
                    return complete_release
                previous=sys.gettrace()
                try:
                    sys.settrace(complete_release)
                    if command['action']=='stop':
                        result=device.submit('calibration',command)
                        self.assertTrue(result.get('message'))
                        self.assertTrue(device.stop_latched)
                    else:
                        with self.assertRaises((ValueError,PermissionError)):
                            device.submit('calibration',command)
                finally:sys.settrace(previous)
                self.assertTrue(reached.is_set(),'release/dispatch barrier was not exercised')
                replacement.submit.assert_not_called()

    def test_partial_close_error_is_not_retried_by_resident_cleanup(self):
        # Compile the actual resident cleanup only: importing/running its Isaac
        # startup would defeat a CPU regression and duplicate cleanup would drift.
        path=ROOT/'scripts/serve_physx_device.py'
        tree=ast.parse(path.read_text(),filename=str(path))
        run=next(node for node in tree.body if isinstance(node,ast.FunctionDef) and node.name=='run')
        release=next(node for node in ast.walk(run) if isinstance(node,ast.FunctionDef) and node.name=='release')
        final=next(node for node in ast.walk(run) if isinstance(node,ast.Try)
                   and any(isinstance(row,ast.If) and ast.unparse(row.test)=='device and device.runtime'
                           for row in node.finalbody))
        cleanup=next(row for row in final.finalbody if isinstance(row,ast.If)
                     and ast.unparse(row.test)=='device and device.runtime')
        for entry in ('resident_finally','cancelled_connection'):
            with self.subTest(entry=entry):
                runtime,device=self.create()
                backend=runtime.owner.backend._backend
                original_close=backend.close
                calls=[]
                def partial_close():
                    calls.append(True);original_close()
                    if len(calls)==1:raise OSError('fixture disconnected then raised')
                events=[]
                scope=dict(device=device,task_observer=None,
                           event=lambda *args,**kwargs:events.append((args,kwargs)))
                with patch.object(backend,'close',partial_close):
                    runtime.request_release(device.credentials['calibration'])
                    self.tick(runtime)
                    self.assertFalse(backend.is_connected)
                    self.assertEqual(runtime.release_receipt['status'],'unconfirmed')
                    with self.assertRaisesRegex(RuntimeError,'unconfirmed'):runtime.close()
                    before=deepcopy(runtime.release_receipt)
                    nodes=[release]+([cleanup] if entry=='resident_finally' else [])
                    exec(compile(ast.Module(body=nodes,type_ignores=[]),str(path),'exec'),scope)
                    if entry=='cancelled_connection':
                        with self.assertRaisesRegex(RuntimeError,'unconfirmed'):
                            scope['release'](runtime)
                    self.assertEqual(len(calls),1,'resident cleanup retried a failed SDK close')
                    self.assertEqual(runtime.release_receipt['status'],'unconfirmed')
                    self.assertFalse(runtime.release_receipt['sdk_close_returned'])
                    self.assertIsNotNone(runtime.fd,'unknown SDK result must not release the lock')
                    self.assertEqual(runtime.release_receipt.get('requested_wall_ns'),before.get('requested_wall_ns'))


if __name__=='__main__':unittest.main()
