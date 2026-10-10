"""Unified display HTTP path stays independent of control and workflow jobs."""
import json
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import threading
import time
from types import SimpleNamespace
import unittest
from urllib.error import HTTPError
from urllib.request import Request,build_opener,ProxyHandler
from unittest.mock import Mock

ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'rule_based_workbench'),str(ROOT/'calibration/infra')]
from robot_workbench.service import Application,Device,protocol_fake,serve
from robot_workbench.target_preview import TargetPreview
from robot_workbench.preview_service import PreviewService


class PreviewHTTPTests(unittest.TestCase):
    def setUp(self):
        self.tmp=TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.device=protocol_fake(Path(self.tmp.name));self.preview=TargetPreview(SimpleNamespace(),'r'*32,mesh_inputs=Mock())
        self.preview.last_health=time.monotonic();self.device.preview=self.preview
        self.app=Application({'fake':self.device,'physical':Device('physical','physical',None)})
        self.server=serve(self.app,port=0);self.thread=threading.Thread(target=self.server.serve_forever,daemon=True);self.thread.start()
        self.addCleanup(lambda:(self.server.shutdown(),self.server.server_close(),self.thread.join(1)))
        self.url='http://127.0.0.1:'+str(self.server.server_port);self.http=build_opener(ProxyHandler({}));self.counter=0

    def post(self,action,**fields):
        self.counter+=1
        body=dict(environment='fake',business='collection',request_id=str(self.counter),**fields)
        request=Request(self.url+'/api/actions/'+action,json.dumps(body).encode(),
            headers={'X-CSRF-Token':self.app.csrf,'Origin':self.url,'Content-Type':'application/json'})
        with self.http.open(request,timeout=1) as response:return json.loads(response.read())

    def state(self):return json.loads(self.http.open(self.url+'/api/state?environment=fake&business=collection',timeout=1).read())

    def frame(self,view,metadata):
        self.preview.frames[view]=(b'cpu-jpeg',dict(renderer_instance_id='r'*32,frame_id='one',source_timestamp_ns=123,
            client_revision=self.preview.revisions['target'],client_view_revision=self.preview.revisions['view'],**metadata),time.monotonic())

    def test_display_request_skips_motion_permission_and_busy_business_queue(self):
        self.app.jobs[('fake','collection')]={'status':'running'}
        self.app.reviews[('fake','collection')]={'test':'frozen-motion-review'}
        result=self.post('preview_target',q12=[.1]*12,jaw_gap_mm=[80.,80.])
        self.assertEqual(result['client_revision'],1);self.assertIsNone(result['token'])
        self.assertEqual(self.app.reviews[('fake','collection')],{'test':'frozen-motion-review'})
        self.app.reviews.clear();self.app.jobs.clear()
        self.frame('target',{})
        value=self.state()['preview'];self.assertTrue(value['frames']['target']['ready'])
        with self.http.open(self.url+value['frames']['target']['url']) as response:
            metadata=json.loads(response.headers['X-Frame-Metadata']);self.assertEqual(response.read(),b'cpu-jpeg')
        self.assertEqual(metadata['revision'],1);self.assertEqual(metadata['source_wall_ns'],123)
        self.assertEqual(metadata['environment'],'fake');self.assertEqual(metadata['renderer_id'],'r'*32)
        self.post('preview_overview',yaw=.2,pitch=.4,zoom=1.)
        # A late frame for the same target but old orbit must not resurrect.
        self.frame('target',{});self.preview.frames['target'][1]['client_view_revision']=0
        with self.assertRaises(HTTPError):self.http.open(self.url+value['frames']['target']['url'])
        self.post('stop')
        self.assertTrue(self.device.stop_latched)

    def test_mesh_metadata_keeps_actual_wall_time_and_candidate_identity(self):
        self.post('preview_mesh',enabled=True,camera='left',candidate_id='wrist-candidate')
        self.frame('mesh',dict(revision=1,camera='left',candidate_id='wrist-candidate',source_wall_ns=456,
            source_time_ns=8_000_000,observed_age_s=.01,environment='fake'))
        value=self.state()['preview'];self.assertTrue(value['mesh']['applied'])
        with self.http.open(self.url+value['frames']['mesh']['url']) as response:metadata=json.loads(response.headers['X-Frame-Metadata'])
        self.assertEqual(metadata['source_wall_ns'],456);self.assertEqual(metadata['source_time_ns'],8_000_000)
        self.assertEqual(metadata['camera'],'left');self.assertEqual(metadata['candidate_id'],'wrist-candidate')
        self.post('preview_mesh',enabled=False,camera='left',candidate_id='wrist-candidate')
        with self.assertRaises(HTTPError):self.http.open(self.url+value['frames']['mesh']['url'])
        with self.assertRaises(HTTPError):self.post('preview_recover')

    def test_completed_target_keeps_its_revision_while_newer_target_is_pending(self):
        self.post('preview_target',q12=[.1]*12,jaw_gap_mm=[80.,80.]);self.frame('target',{})
        before=self.state()['preview'];self.post('preview_target',q12=[.2]*12,jaw_gap_mm=[80.,80.])
        value=self.state()['preview']
        self.assertEqual(value['revisions']['target'],2);self.assertEqual(value['frames']['target']['revision'],1)
        self.assertTrue(value['frames']['target']['ready']);self.assertEqual(value['display_context'],before['display_context'])
        with self.http.open(self.url+value['frames']['target']['url']) as response:metadata=json.loads(response.headers['X-Frame-Metadata'])
        self.assertEqual(metadata['revision'],1,'a completed frame must never be relabeled as the latest request')
        self.assertEqual(metadata['display_context'],value['display_context']);self.assertEqual(metadata['server_instance_id'],self.app.server_instance_id)

    def test_owner_epoch_and_installation_change_require_a_new_target(self):
        original=self.device.snapshot;identity={}
        self.device.snapshot=lambda business:dict(original(business),**identity)
        self.post('preview_target',q12=[.1]*12,jaw_gap_mm=[80.,80.]);self.frame('target',{})
        previous=self.state()['preview']
        for field,new in [('runtime_nonce','new-runtime'),('command_epoch',2),('calibration_revision',2),('calibration_snapshot_id','new-install')]:
            identity[field]=new;value=self.state()['preview']
            self.assertNotEqual(value['display_context'],previous['display_context'])
            self.assertEqual(value['target_floor'],value['revisions']['target']+1);self.assertFalse(value['frames']['target']['ready'])
            with self.assertRaises(HTTPError):self.http.open(self.url+value['frames']['target']['url'])
            result=self.post('preview_target',q12=[.2]*12,jaw_gap_mm=[80.,80.])
            self.assertEqual(result['display_context'],value['display_context'])
            with self.assertRaises(HTTPError):self.http.open(self.url+value['frames']['target']['url'])
            self.frame('target',{})
            with self.http.open(self.url+value['frames']['target']['url']) as response:metadata=json.loads(response.headers['X-Frame-Metadata'])
            self.assertEqual(metadata['revision'],result['client_revision']);self.assertEqual(metadata['display_context'],value['display_context'])
            previous=value

    def test_recovery_and_health_do_not_wait_for_display_startup(self):
        provider=PreviewService.__new__(PreviewService)
        provider.lock=threading.Lock();provider.closed=threading.Event();provider.thread=None;provider.process=None
        provider.adapter=self.preview;provider.deadline=time.time_ns()+120_000_000_000;provider.status='ready';provider.error=None
        gate=threading.Event();provider._start=lambda:gate.wait(2)
        self.device.preview=provider
        try:
            result=self.post('preview_recover');self.assertTrue(result['queued'])
            start=time.monotonic();self.state();self.post('stop')
            self.assertLess(time.monotonic()-start,.3)
            self.assertFalse(self.state()['runtime']['allowed_operations']['preview_recover']['allowed'])
        finally:gate.set();provider.thread.join(1)


if __name__=='__main__':unittest.main()
