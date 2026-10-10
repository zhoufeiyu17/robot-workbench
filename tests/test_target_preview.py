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
from urllib.parse import parse_qs,urlsplit
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

    def test_unchanged_frame_skips_jpeg_without_skipping_new_target_or_view(self):
        remote=dict(renderer_instance_id='preview',mesh_rendering_enabled=False,preview_revision=0,overview_revision=0,
                    frame_timestamp_s={'actual':1.,'preview':1.})
        reads=[];rounds=[]
        def request(path,body=None,**kwargs):
            if path=='/state':return deepcopy(remote)
            if path=='/preview':remote['preview_revision']=body['revision'];return {}
            if path=='/overview-view':remote['overview_revision']=body['revision'];return {}
            query=parse_qs(urlsplit(path).query);view=query['view'][0];reads.append(view)
            return b'jpeg',dict(schema='target_preview_frame.v1',frame_id=view+str(remote['frame_timestamp_s'][view]),
                renderer_instance_id='preview',view=view,source='kinematic_target' if view=='preview' else 'kinematic_overview',
                overview_revision=remote['overview_revision'],target_revision=remote['preview_revision'] if view=='preview' else None,
                source_timestamp_ns=int(remote['frame_timestamp_s'][view]*1_000_000_000))
        adapter=TargetPreview(SimpleNamespace(request=request),'preview');adapter.target_pose([.1]*12,[80.,80.])
        def next_iteration(delay):
            self.assertLessEqual(delay,.025)  # Leave headroom above the required 20 fps.
            rounds.append(len(reads))
            if len(rounds)==2:remote['frame_timestamp_s']['actual']=2.
            elif len(rounds)==3:adapter.overview(.3,.4,1.)
            elif len(rounds)==4:adapter.target_pose([.2]*12,[80.,80.])
            elif len(rounds)==5:adapter.closed.set()
        adapter.changed.wait=next_iteration
        adapter._preview_loop()
        self.assertEqual(rounds,[2,2,3,5,6])
        self.assertEqual(reads,['actual','preview','actual','actual','preview','preview'])
        self.assertEqual(adapter.frames['target'][1]['client_revision'],2)
        self.assertEqual(adapter.frames['target'][1]['client_view_revision'],1)
        mesh=TargetPreview(SimpleNamespace(),'preview')
        def stop_mesh(delay):
            self.assertLessEqual(delay,.05)  # Leave headroom above the required 10 fps.
            mesh.closed.set()
        mesh.closed.wait=stop_mesh;mesh._mesh_loop()

    def test_delayed_renderer_completes_during_continuous_latest_target_updates(self):
        pending=None;published=None;rounds=0;sent=[];seen=[];requested={}
        remote=dict(renderer_instance_id='preview',mesh_rendering_enabled=False,preview_revision=0,overview_revision=0)
        def edit():
            revision=adapter.target_pose([adapter.revisions['target']/100]*12,[80.,80.])['client_revision']
            requested[revision]=adapter.requested_wall_ns['target']
        def request(path,body=None,**kwargs):
            nonlocal pending,published
            if path=='/state':return deepcopy(remote)
            if path=='/preview':
                self.assertIsNone(pending,'a same-view target must finish before the next renderer request')
                pending=dict(body,polls=0);sent.append(body['q'][0]);return {}
            query=parse_qs(urlsplit(path).query)
            if query['view'][0]=='actual':raise PendingFrame('overview omitted from this fixture')
            if pending:
                pending['polls']+=1
                if pending['polls']<3:raise PendingFrame('delayed renderer')
                remote['preview_revision']=pending['revision']
                published=deepcopy(pending);pending=None
                edit()  # A new input during the read must not relabel or discard the completed pixels.
            if not published:raise PendingFrame('no completed target')
            stamp=time.time_ns()
            return json.dumps(published['q']).encode(),dict(schema='target_preview_frame.v1',frame_id='frame-'+str(published['revision']),
                renderer_instance_id='preview',view='preview',source='kinematic_target',overview_revision=0,
                target_revision=published['revision'],source_timestamp_ns=stamp)
        adapter=TargetPreview(SimpleNamespace(request=request),'preview');edit()
        def next_iteration(delay):
            nonlocal rounds
            rounds+=1
            if 'target' in adapter.frames:
                raw,meta=adapter.media('target');revision=meta['client_revision']
                self.assertEqual(json.loads(raw),[(revision-1)/100]*12)
                self.assertEqual(meta['client_requested_wall_ns'],requested[revision])
                self.assertLess(revision,adapter.revisions['target'])
                seen.append(revision)
                before=meta['frame_id'];edit()
                self.assertEqual(adapter.media('target')[1]['frame_id'],before,'new input must retain the completed frame')
            else:edit()
            if rounds==12:adapter.closed.set()
        adapter.changed.wait=next_iteration;adapter._preview_loop()
        self.assertIsNone(adapter.error)
        self.assertEqual(len(sent),4);self.assertEqual(len(set(seen)),4)
        self.assertEqual(seen,sorted(seen));self.assertLess(len(sent),adapter.revisions['target'])

    def test_new_view_interrupts_pending_target_and_rejects_old_generations(self):
        rounds=0;sent=[];returned=[];pending=None;wrong='view'
        remote=dict(renderer_instance_id='preview',mesh_rendering_enabled=False,preview_revision=0,overview_revision=0)
        def request(path,body=None,**kwargs):
            nonlocal pending
            if path=='/state':return deepcopy(remote)
            if path=='/overview-view':remote['overview_revision']=body['revision'];return {}
            if path=='/preview':pending=deepcopy(body);sent.append((body['revision'],remote['overview_revision']));return {}
            query=parse_qs(urlsplit(path).query)
            if query['view'][0]=='actual' or rounds==0:raise PendingFrame('old target never completes')
            meta=dict(schema='target_preview_frame.v1',frame_id='frame-'+str(pending['revision']),
                renderer_instance_id='wrong' if wrong=='renderer' else 'preview',view='preview',source='kinematic_target',
                overview_revision=0 if wrong=='view' else remote['overview_revision'],target_revision=pending['revision'],
                source_timestamp_ns=1 if wrong=='timestamp' else time.time_ns())
            returned.append(deepcopy(meta));return b'jpeg',meta
        adapter=TargetPreview(SimpleNamespace(request=request),'preview');adapter.target_pose([.1]*12,[80.,80.])
        def next_iteration(delay):
            nonlocal rounds,wrong
            rounds+=1
            if rounds==1:adapter.overview(.3,.4,1.)
            elif rounds in (2,3):
                self.assertNotIn('target',adapter.frames)
                self.assertIn('matching atomic display metadata',adapter.error)
                wrong='renderer' if rounds==2 else None
            elif rounds==4:
                self.assertIsNone(adapter.error);self.assertEqual(adapter.media('target')[1]['client_view_revision'],1)
                self.assertEqual(adapter.media('target')[1]['client_revision'],1)
                wrong='timestamp'
            elif rounds==5:
                self.assertIn('timestamp moved backwards',adapter.error)
                self.assertEqual(adapter.media('target')[1],dict(returned[-2],client_revision=1,client_view_revision=1,
                    client_requested_wall_ns=adapter.requested_wall_ns['view'],
                    display_latency_ms=(returned[-2]['source_timestamp_ns']-adapter.requested_wall_ns['view'])/1e6))
                remote['renderer_instance_id']='replacement'
            else:
                self.assertNotIn('target',adapter.frames)
                self.assertIn('explicit rebind required',adapter.error);adapter.closed.set()
        adapter.changed.wait=next_iteration;adapter._preview_loop()
        self.assertEqual(sent,[(1,0),(2,1)],'new view resends latest pose without waiting on old-view completion')


if __name__=='__main__':unittest.main()
