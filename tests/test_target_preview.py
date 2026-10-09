"""Real loopback C renderer protocol with CPU pixels; no Isaac/GPU/Owner startup."""
from copy import deepcopy
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'calibration'),str(ROOT/'calibration/infra')]
from examples.scripts.debug.joint_head_workbench_live import Session,serve
from robot_workbench.target_preview import RendererClient,TargetPreview,PendingFrame,observed_mesh


class Geometry:
    velocity_limits=np.ones(12);limits=np.array([[-1,1]]*12)
    k=dict(width=4,height=3,fx=2.,fy=2.,cx=2.,cy=1.5);marker_size=.015


class TargetPreviewTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.root=Path(self.tmp.name);self.token=self.root/'token';self.token.write_text('x'*40)

    def wait(self,predicate):
        deadline=time.monotonic()+3
        while time.monotonic()<deadline:
            if predicate():return
            time.sleep(.01)
        self.fail('CPU preview condition did not complete')

    def test_actual_http_frames_bind_target_revision_and_never_mutate_actual_pose(self):
        session=Session(Geometry(),self.root/'renderer',{})
        started,release,closed=threading.Event(),threading.Event(),threading.Event()
        class Renderer:
            def __init__(self):self.view=None
            def set_overview_view(self,view):self.view=view
            def render(self,q,gaps=None):
                if q[0]==.1 and not release.is_set():started.set();release.wait(2)
                return json.dumps(dict(q=q.tolist(),view=self.view)).encode(),b'head'
        renderer=Renderer();session.tick(renderer,0)
        server=serve(session,'x'*40,0)
        client=RendererClient('http://127.0.0.1:'+str(server.server_port),self.token)
        preview=TargetPreview(client,session.renderer_instance_id)
        def render_loop():
            while not closed.is_set():session.tick(renderer,.02);closed.wait(.01)
        worker=threading.Thread(target=render_loop);worker.start();preview.start()
        try:
            preview.target_pose([.1]*12,[80.,80.]);self.assertTrue(started.wait(2))
            before=time.monotonic();self.assertTrue(preview.state()['display_only'])
            self.assertLess(time.monotonic()-before,.05)
            current=preview.target_pose([.2]*12,[80.,80.])
            with self.assertRaises(PendingFrame):preview.media('target')
            release.set()
            self.wait(lambda:preview.state()['frames'].get('target',{}).get('client_revision')==current['client_revision'])
            raw,meta=preview.media('target')
            self.assertEqual(json.loads(raw)['q'],[.2]*12)
            self.assertEqual(meta['q_reported_rad'],[.2]*12)
            self.assertEqual(meta['renderer_instance_id'],session.renderer_instance_id)
            self.assertEqual(meta['source'],'kinematic_target');self.assertFalse(meta['measurement_qualified'])
            np.testing.assert_array_equal(session.actual_q,np.zeros(12));np.testing.assert_array_equal(session.q,np.zeros(12))
            old_stamp=meta['source_timestamp_ns']
            preview.overview(.3,.4,1.2)
            self.wait(lambda:preview.state()['frames'].get('target',{}).get('overview_revision')==1)
            raw,meta=preview.media('target')
            self.assertEqual(json.loads(raw)['view']['yaw'],.3)
            self.assertGreater(meta['source_timestamp_ns'],old_stamp)
            with self.assertRaises(PendingFrame):
                client.request('/frame.jpg?view=preview&target_revision=999',frame=True)
            with self.assertRaises(ValueError):client.request('/execute',{})
            session.renderer_instance_id='another-process'
            self.wait(lambda:bool(preview.state()['error']))
        finally:
            release.set();preview.close();closed.set();worker.join(2);server.shutdown();server.server_close()

    def inputs(self):
        wall=time.time_ns();step=8;stamp=step*1_000_000_000//240
        shared=dict(environment='fake',clock_domain='simulation',source_clock_id='resident-actual',source_time_ns=stamp,physics_step=step)
        meta=dict(shared,host_receive_ns=wall,pair_id='resident-actual:head:8')
        feedback=dict(shared,sample_wall_ns=wall,qpos=[0.]*14)
        candidate=dict(id='poor-but-viewable',camera_T_world_optical=np.eye(4).tolist(),
            joint_offsets_rad=dict(left=[0.]*6,right=[0.]*6),acceptance={'passed':False})
        return b'actual-camera-jpeg',meta,feedback,Geometry.k,candidate

    def test_candidate_mesh_uses_actual_pixels_and_closing_switch_stops_jobs(self):
        started,release=threading.Event(),threading.Event();calls=[]
        inputs=self.inputs();original=deepcopy(inputs)
        def request(path,body=None,**kwargs):
            calls.append((path,body))
            if path=='/state':return dict(renderer_instance_id='preview',mesh_rendering_enabled=False,preview_revision=-1,overview_revision=0)
            if path.startswith('/frame.jpg'):raise PendingFrame('no display fixture')
            if path=='/real-overlay':started.set();release.wait(2);return b'overlay'
            raise AssertionError(path)
        reader=Mock(return_value=inputs)
        adapter=TargetPreview(SimpleNamespace(request=request),'preview',mesh_inputs=reader)
        self.assertFalse(calls,'construction must not start a renderer or network work')
        adapter.start()
        try:
            adapter.mesh(True,candidate_id='poor-but-viewable');self.assertTrue(started.wait(2))
            before=time.monotonic();adapter.mesh(False);adapter.state()
            self.assertLess(time.monotonic()-before,.05)
            release.set();time.sleep(.25)
            self.assertEqual(sum(path=='/real-overlay' for path,_ in calls),1)
            with self.assertRaises(PendingFrame):adapter.media('mesh')
            payload=next(body for path,body in calls if path=='/real-overlay')
            import base64
            self.assertEqual(base64.b64decode(payload['jpeg_base64']),original[0])
            self.assertEqual(payload['qpos'],original[2]['qpos']);self.assertEqual(inputs,original)
            self.assertFalse(payload['candidate']['acceptance']['passed'],'display must not add an installation gate')
        finally:release.set();adapter.close()

    def test_mesh_requires_actual_source_clock_and_frame_identity(self):
        client=SimpleNamespace(request=Mock(return_value=b'overlay'))
        inputs=list(self.inputs())
        raw,metadata=observed_mesh(client,inputs,'poor-but-viewable')
        self.assertEqual(raw,b'overlay');self.assertEqual(metadata['source_clock_id'],'resident-actual')
        for field,value in (('source_clock_id','target-renderer'),('source_time_ns',123)):
            bad=deepcopy(inputs);bad[2][field]=value
            before=client.request.call_count
            with self.assertRaises(ValueError):observed_mesh(client,bad,'poor-but-viewable')
            self.assertEqual(client.request.call_count,before)
        bad=deepcopy(inputs);bad[1].pop('pair_id')
        with self.assertRaises(ValueError):observed_mesh(client,bad,'poor-but-viewable')


if __name__=='__main__':unittest.main()
