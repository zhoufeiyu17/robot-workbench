"""Explicit supported release through HTTP; protocol fake, never hardware."""
import json
import sys
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
from urllib.error import HTTPError

sys.path[:0]=[str(Path(__file__).resolve().parent),str(Path(__file__).resolve().parents[1])]
from test_http_service import HTTPTest


class ReleaseTest(HTTPTest):
    def cleanup(self):
        runtime=self.device.runtime
        if runtime:
            runtime.owner.submit({'action':'release','confirmation':'ARMS_SUPPORTED_RELEASE'})
            self.device.thread.join(2)
            if runtime.fd is not None:runtime.close()
        self.device.closed.set()
        self.server.shutdown();self.server.server_close();self.thread.join(1)

    def test_release_reconnect_revokes_old_review_without_automatic_motion(self):
        before=self.connect();old=self.device.runtime
        old.notify_calibration_installed('same-device-installed-C')
        self.action('manual_preview',q12=[.02]*12,jaw_gap_mm=[80.,80.])
        review=self.wait(lambda s:s['workflow'].get('review'))['workflow']['review']['id']
        code,response=self.action('release')
        self.assertEqual(code,200);self.assertEqual(response['release_receipt']['status'],'requested')
        state=self.wait(lambda s:s['runtime'].get('release_receipt',{}).get('status')=='released')
        self.assertFalse(state['runtime']['connected']);self.assertIsNone(old.fd)
        self.assertTrue(state['runtime']['release_receipt']['sdk_close_returned'])
        new=self.connect()
        self.assertNotEqual(before['runtime']['runtime_nonce'],new['runtime']['runtime_nonce'])
        self.assertEqual(new['runtime']['calibration_snapshot_id'],'same-device-installed-C')
        self.assertFalse(new['runtime']['automatic_enabled'])
        with self.assertRaises(HTTPError):self.action('manual_execute',review_id=review,physical_clearance=True)

    def test_nonholder_busy_workflow_and_active_capture_refuse_release(self):
        self.connect();runtime=self.device.runtime
        with self.assertRaises(HTTPError) as error:self.action('release','collection')
        self.assertIn('交接',json.loads(error.exception.read())['error'])
        self.device.workflows['calibration']=SimpleNamespace(snapshot=lambda:dict(busy=True))
        with self.assertRaises(HTTPError):self.action('release')
        self.device.workflows.clear()
        with runtime.lock:
            runtime.owner.capture_sample='in-progress'
            with self.assertRaises(ValueError):runtime.request_release(self.device.credentials['calibration'])
            runtime.owner.capture_sample=None
        self.assertFalse(runtime.owner.release_requested)

    def test_physical_release_requires_current_support_confirmation(self):
        with self.assertRaises(ValueError) as error:
            self.app.action('release',dict(environment='physical',business='calibration',request_id='unsupported'))
        self.assertIn('双臂已可靠支撑',str(error.exception))
        with self.assertRaises(ValueError) as error:
            self.app.action('release',dict(environment='physical',business='calibration',request_id='supported',arms_supported=True))
        self.assertEqual(str(error.exception),'设备未连接')

    def test_blocked_sdk_close_keeps_lock_status_and_stop_available(self):
        self.connect();runtime=self.device.runtime;backend=runtime.owner.backend._backend
        entered,proceed=threading.Event(),threading.Event();original=backend.close
        def close():
            entered.set()
            if not proceed.wait(5):raise TimeoutError('test close barrier')
            original()
        with patch.object(backend,'close',close):
            try:
                self.action('release');self.assertTrue(entered.wait(1))
                runtime.release_receipt['requested_monotonic_s']-=2.1
                state=self.state()['runtime'];self.assertEqual(state['release_receipt']['status'],'unconfirmed')
                self.assertFalse(state['allowed_operations']['connect']['allowed']);self.assertIsNotNone(runtime.fd)
                started=time.monotonic();self.action('stop');self.assertLess(time.monotonic()-started,.3)
                code,_=self.action('connect');self.assertEqual(code,202)
                self.wait(lambda s:(s['workflow'].get('request') or {}).get('status')=='failed')
                self.assertIs(self.device.runtime,runtime)
            finally:proceed.set()
            self.wait(lambda s:s['runtime'].get('release_receipt',{}).get('status')=='released')

    def test_sdk_close_failure_is_visible_and_not_automatically_retried(self):
        self.connect();runtime=self.device.runtime;backend=runtime.owner.backend._backend
        with patch.object(backend,'close',side_effect=OSError('close refused')) as close:
            self.action('release')
            state=self.wait(lambda s:s['runtime'].get('release_receipt',{}).get('status')=='unconfirmed')
            time.sleep(.08)
            self.assertEqual(close.call_count,1)
            self.assertIn('close refused',state['runtime']['release_receipt']['message'])
            self.assertIsNotNone(runtime.fd);self.assertIs(self.device.runtime,runtime)
            self.assertFalse(state['runtime']['allowed_operations']['connect']['allowed'])
        self.action('release')
        self.wait(lambda s:s['runtime'].get('release_receipt',{}).get('status')=='released')

    def test_unsealed_recording_refuses_release(self):
        self.connect();runtime=self.device.runtime
        with runtime.lock:
            runtime.owner.recording=SimpleNamespace(done=threading.Event(),thread=None)
            try:
                with self.assertRaisesRegex(ValueError,'录制'):runtime.request_release(self.device.credentials['calibration'])
            finally:runtime.owner.recording=None

    def test_external_plant_loop_can_close_runtime_before_device_thread_exits(self):
        def drive(runtime, closed):
            while runtime.tick():
                closed.wait(.01)
            runtime.close()
            self.device.runtime=None;self.device.credentials={}
            # Resident PhysX keeps the plant alive for cameras and a later explicit connection.
            runtime.owner.backend._backend.is_connected=True
            closed.set()
        self.device.drive=drive
        self.connect();self.action('release')
        state=self.wait(lambda s:(s['runtime'].get('release_receipt') or {}).get('status')=='released')
        self.assertFalse(state['runtime']['connected'])
        self.assertIsNone(self.device.error)


def load_tests(loader,tests,pattern):
    return unittest.TestSuite(ReleaseTest(name) for name in ReleaseTest.__dict__ if name.startswith('test_'))


if __name__=='__main__':unittest.main()
