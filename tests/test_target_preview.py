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
from unittest.mock import Mock,patch
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'calibration'),str(ROOT/'calibration/infra')]
from examples.scripts.debug.joint_head_workbench_live import Session,serve
from robot_workbench.target_preview import RendererClient,TargetPreview,PendingFrame,observed_mesh,observed_mesh_group


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
        session=Session(Geometry(),self.root/'renderer',{'display_only':True})
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
            self.wait(lambda:preview.state()['frames'].get('overview',{}).get('overview_revision')==1)
            self.assertNotIn('target',preview.state()['frames'])
            raw,meta=preview.media('overview')
            self.assertEqual(json.loads(raw)['view']['yaw'],.3)
            self.assertGreater(meta['source_timestamp_ns'],old_stamp)
            preview.target_pose([.3]*12,[80.,80.])
            self.wait(lambda:preview.state()['frames'].get('target',{}).get('client_revision')==3)
            self.assertEqual(json.loads(preview.media('target')[0])['q'],[.3]*12)
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

    def test_three_camera_mesh_retains_per_camera_sources_and_rejects_bad_group(self):
        from io import BytesIO
        from PIL import Image
        inputs={name:deepcopy(self.inputs()) for name in ('head','left','right')}
        for name,packet in inputs.items():
            packet[1]['pair_id']='frame-'+name
            packet[4]['id']='head-cal' if name=='head' else 'wrist-cal'
            if name!='head':packet[4]['head_result_id']='head-cal'
        def render(path,body,**kwargs):
            self.assertEqual(path,'/real-overlay')
            image=Image.new('RGB',(64,48),'red' if body['candidate']['id']=='head-cal' else 'blue')
            output=BytesIO();image.save(output,format='JPEG');return output.getvalue()
        raw,meta=observed_mesh_group(SimpleNamespace(request=render),inputs,'wrist-cal')
        with Image.open(BytesIO(raw)) as image:self.assertEqual(image.size,(192,72))
        self.assertEqual(meta['head_candidate_id'],'head-cal');self.assertEqual(meta['wrist_candidate_id'],'wrist-cal')
        self.assertEqual([m['source_frame_id'] for m in meta['cameras']],['frame-head','frame-left','frame-right'])
        self.assertFalse(meta['synchronized_capture']);self.assertFalse(meta['measurement_qualified'])
        bad=deepcopy(inputs);bad['right'][4]['id']='unrelated'
        with self.assertRaises(ValueError):observed_mesh_group(SimpleNamespace(request=render),bad,'wrist-cal')
        bad=deepcopy(inputs);bad['right'][4]['head_result_id']='wrong-head'
        with self.assertRaises(ValueError):observed_mesh_group(SimpleNamespace(request=render),bad,'wrist-cal')
        adapter=TargetPreview(SimpleNamespace(),'renderer',mesh_inputs=lambda *_:inputs)
        adapter.mesh(True,camera='all',candidate_id='wrist-cal')
        adapter.frames['mesh']=(raw,dict(meta,revision=1),time.monotonic())
        self.assertEqual(adapter.media('mesh')[1]['camera'],'all')
        adapter.mesh(False,camera='all',candidate_id='wrist-cal')
        with self.assertRaises(PendingFrame):adapter.media('mesh')

    def test_unchanged_frame_skips_jpeg_without_skipping_new_target_or_view(self):
        remote=dict(renderer_instance_id='preview',mesh_rendering_enabled=False,preview_revision=0,overview_revision=0,
                    frame_timestamp_s={'actual':1.,'preview':1.})
        reads=[];rounds=[]
        def request(path,body=None,**kwargs):
            if path=='/state':return deepcopy(remote)
            if path=='/preview':remote['preview_revision']=body['revision'];return {}
            if path=='/overview-view':remote['overview_revision']=body['revision'];return {}
            if path=='/display-view':return {}
            query=parse_qs(urlsplit(path).query);view=query['view'][0];reads.append(view)
            self.assertEqual(query.get('wait_ms'),['100'] if view=='preview' else None)
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
        self.assertEqual(rounds,[1,1,1,2,3])
        self.assertEqual(reads,['preview','actual','preview'])
        self.assertEqual(adapter.frames['target'][1]['client_revision'],2)
        self.assertEqual(adapter.frames['target'][1]['client_view_revision'],1)
        mesh=TargetPreview(SimpleNamespace(),'preview')
        def stop_mesh(delay):
            self.assertLessEqual(delay,.05)  # Leave headroom above the required 10 fps.
            mesh.closed.set()
        mesh.closed.wait=stop_mesh;mesh._mesh_loop()

    def test_three_fifo_slots_exact_reads_and_presentation_ack(self):
        pending=None;waits=[];sent=[];requested={}
        remote=dict(renderer_instance_id='preview',mesh_rendering_enabled=False,preview_revision=0,overview_revision=0)
        def request(path,body=None,**kwargs):
            nonlocal pending
            if path=='/state':return deepcopy(remote)
            if path=='/preview':
                self.assertIsNone(pending,'a same-view target must finish before the next renderer request')
                pending=dict(body,polls=0);sent.append((body['q'][0],len(waits)));return {}
            query=parse_qs(urlsplit(path).query)
            self.assertEqual(query['view'],['preview'])
            self.assertEqual(query['wait_ms'],['100'])
            pending['polls']+=1
            if pending['polls']<2:raise PendingFrame('delayed renderer')
            published=pending;pending=None;remote['preview_revision']=published['revision']
            return json.dumps(published['q']).encode(),dict(schema='target_preview_frame.v1',frame_id='frame-'+str(published['revision']),
                renderer_instance_id='preview',view='preview',source='kinematic_target',overview_revision=0,
                target_revision=published['revision'],source_timestamp_ns=time.time_ns())
        adapter=TargetPreview(SimpleNamespace(request=request),'preview');adapter.bind_context('context')
        identity=dict(display_context='context',renderer_instance_id='preview',view_revision=0)
        for revision in range(1,4):
            self.assertEqual(adapter.target_pose([revision/100]*12,[80.,80.],**identity)['client_revision'],revision)
            requested[revision]=adapter.requested_wall_ns['target']
        self.assertEqual(adapter.state()['queue']['occupied'],3)
        with self.assertRaisesRegex(ValueError,'queue is full'):adapter.target_pose([.04]*12,[80.,80.],**identity)
        with self.assertRaises(PendingFrame):adapter.media('target',2,**identity)
        def next_iteration(delay):
            waits.append(delay)
            if adapter.highest_completed_revision==3:adapter.closed.set()
        adapter.changed.wait=next_iteration;adapter._preview_loop();adapter.closed.clear()
        self.assertIsNone(adapter.error);self.assertEqual(sent,[(.01,0),(.02,1),(.03,2)])
        for revision in range(1,4):
            raw,meta=adapter.media('target',revision,**identity)
            self.assertEqual(json.loads(raw),[revision/100]*12)
            self.assertEqual(meta['client_revision'],revision)
            self.assertEqual(meta['client_requested_wall_ns'],requested[revision])
            self.assertLessEqual(requested[revision],meta['adapter_submit_started_wall_ns'])
            self.assertLessEqual(meta['adapter_submit_started_wall_ns'],meta['source_timestamp_ns'])
            self.assertLessEqual(meta['source_timestamp_ns'],meta['adapter_frame_received_wall_ns'])
        self.assertEqual(adapter.state()['queue']['occupied'],3,'GET must not consume completed slots')
        for fields,watermark in (({},1),(dict(identity,display_context='old'),1),
                                 (dict(identity,renderer_instance_id='old'),1),(dict(identity,view_revision=1),1),(identity,4)):
            with self.assertRaises(ValueError):adapter.target_pose([.04]*12,[80.,80.],**fields,presented_through_revision=watermark)
            self.assertEqual(adapter.state()['queue']['occupied'],3)
        adapter.target_pose([.04]*12,[80.,80.],**identity,presented_through_revision=2)
        adapter.target_pose([.05]*12,[80.,80.],**identity,presented_through_revision=2)  # Repeated watermark is harmless.
        self.assertEqual([slot['revision'] for slot in adapter.slots],[3,4,5])
        with self.assertRaises(PendingFrame):adapter.media('target',2,**identity)
        with self.assertRaises(PendingFrame):adapter.media('target',4,**identity)
        self.assertEqual(adapter.media('target')[1]['client_revision'],3)
        adapter.target_pose([.06]*12,[80.,80.],**identity,presented_through_revision=3)
        self.assertEqual([slot['revision'] for slot in adapter.slots],[4,5,6])
        self.assertEqual(adapter.media('target')[1]['client_revision'],3,'ACK retains latest successful static pixels')
        deadline=max(slot['deadline'] for slot in adapter.slots)
        with patch('robot_workbench.target_preview.time.monotonic',return_value=deadline+.001):
            self.assertEqual(adapter.state()['queue']['occupied'],0)
            self.assertEqual(adapter.media('target')[1]['client_revision'],3)
            with self.assertRaisesRegex(PendingFrame,'expired'):adapter.media('target',4,**identity)
        self.assertEqual([item['status'] for item in adapter.state()['failures']],['expired']*3)

    def test_context_and_view_fence_inflight_pixels_and_old_ack(self):
        for fence in ('context','view'):
            with self.subTest(fence=fence):
                remote=dict(renderer_instance_id='preview',mesh_rendering_enabled=False,preview_revision=0,overview_revision=0)
                pending=None;sent=[];fenced=False
                adapter=None
                def identity():return dict(display_context=adapter.display_context,renderer_instance_id='preview',view_revision=adapter.revisions['view'])
                def request(path,body=None,**kwargs):
                    nonlocal pending,fenced
                    if path=='/state':return deepcopy(remote)
                    if path=='/display-view':return {}
                    if path=='/overview-view':remote['overview_revision']=body['revision'];return {}
                    if path=='/preview':pending=deepcopy(body);sent.append(body['q'][0]);return {}
                    if not fenced:
                        if fence=='context':adapter.bind_context('next')
                        else:adapter.overview(.3,.4,1.)
                        adapter.target_pose([.2]*12,[80.,80.],**identity());fenced=True
                    return b'jpeg',dict(schema='target_preview_frame.v1',frame_id='frame-'+str(pending['revision']),
                        renderer_instance_id='preview',view='preview',source='kinematic_target',overview_revision=remote['overview_revision'],
                        target_revision=pending['revision'],source_timestamp_ns=time.time_ns())
                adapter=TargetPreview(SimpleNamespace(request=request),'preview');adapter.bind_context('first')
                old=identity();adapter.target_pose([.1]*12,[80.,80.],**old)
                def next_iteration(delay):
                    if adapter.highest_completed_revision:
                        adapter.closed.set();return
                    self.assertNotIn('target',adapter.frames)
                    with self.assertRaises(ValueError):adapter.target_pose([.9]*12,[80.,80.],**old,presented_through_revision=1)
                    with self.assertRaisesRegex(ValueError,'watermark'):adapter.target_pose([.9]*12,[80.,80.],**identity(),presented_through_revision=1)
                    self.assertEqual([slot['revision'] for slot in adapter.slots],[2])
                adapter.changed.wait=next_iteration;adapter._preview_loop();adapter.closed.clear()
                self.assertEqual(sent,[.1,.2]);self.assertEqual(adapter.media('target',2,**identity())[1]['client_revision'],2)
                self.assertEqual(adapter.state()['failures'][0]['status'],'cancelled')
                for index in range(40):
                    adapter.bind_context('context-'+str(index));adapter.target_pose([0.]*12,[80.,80.])
                self.assertEqual(len(adapter.state()['failures']),32)

    def test_exact_wait_releases_lock_and_context_change_wakes_reader(self):
        adapter=TargetPreview(SimpleNamespace(),'preview');adapter.bind_context('first')
        identity=dict(display_context='first',renderer_instance_id='preview',view_revision=0)
        adapter.target_pose([.1]*12,[80.,80.],**identity)
        with self.assertRaises(ValueError):adapter.media('target',1)
        with self.assertRaises(ValueError):adapter.media('target',1,**identity,wait_ms=101)
        with self.assertRaises(ValueError):adapter.media('target',None,wait_ms=10)
        waiting=threading.Event();original=adapter.ready.wait;result=[]
        def wait(timeout):
            self.assertLessEqual(timeout,.1);waiting.set();return original(timeout)
        adapter.ready.wait=wait
        def read():
            try:result.append(adapter.media('target',1,**identity,wait_ms=100))
            except ValueError as error:result.append(str(error))
        worker=threading.Thread(target=read);worker.start()
        try:
            self.assertTrue(waiting.wait(1));started=time.monotonic()
            adapter.state();adapter.target_pose([.2]*12,[80.,80.],**identity)
            self.assertLess(time.monotonic()-started,.05)
            adapter.bind_context('next');worker.join(.2)
            self.assertFalse(worker.is_alive());self.assertIn('generation changed',result[0])
        finally:adapter.close();worker.join(1)

    def test_completed_rendering_and_accepted_slots_expire_without_losing_latest(self):
        pending=None
        def request(path,body=None,**kwargs):
            nonlocal pending
            if path=='/state':return dict(renderer_instance_id='preview',mesh_rendering_enabled=False,preview_revision=0,overview_revision=0)
            if path=='/preview':pending=body;return {}
            if pending['revision']>1:raise PendingFrame('second target is still rendering')
            return b'first',dict(schema='target_preview_frame.v1',frame_id='first',renderer_instance_id='preview',
                view='preview',source='kinematic_target',overview_revision=0,target_revision=1,source_timestamp_ns=time.time_ns())
        adapter=TargetPreview(SimpleNamespace(request=request),'preview');adapter.bind_context('context')
        identity=dict(display_context='context',renderer_instance_id='preview',view_revision=0)
        for index in range(3):adapter.target_pose([index/100]*12,[80.,80.],**identity)
        adapter.changed.wait=lambda delay:adapter.closed.set()
        adapter._preview_loop();adapter.closed.clear()
        self.assertEqual([slot['status'] for slot in adapter.slots],['completed','rendering','accepted'])
        self.assertTrue(all(2400<slot['remaining_ms']<=2500 for slot in adapter.state()['queue']['slots']))
        with patch('robot_workbench.target_preview.time.monotonic',return_value=max(slot['deadline'] for slot in adapter.slots)+.001):
            self.assertEqual(adapter.state()['queue']['occupied'],0)
            self.assertEqual(adapter.media('target')[0],b'first')
            for revision in (1,2,3):
                with self.assertRaisesRegex(PendingFrame,'expired'):adapter.media('target',revision,**identity)
        self.assertEqual([failure['client_revision'] for failure in adapter.state()['failures']],[1,2,3])

    def test_exact_wait_wakes_with_its_completed_frame(self):
        pending=None;publish=threading.Event();waiting=threading.Event();result=[]
        def request(path,body=None,**kwargs):
            nonlocal pending
            if path=='/state':return dict(renderer_instance_id='preview',mesh_rendering_enabled=False,preview_revision=0,overview_revision=0)
            if path=='/preview':pending=body;return {}
            if not publish.is_set():raise PendingFrame('renderer not complete')
            return b'exact',dict(schema='target_preview_frame.v1',frame_id='exact',renderer_instance_id='preview',
                view='preview',source='kinematic_target',overview_revision=0,target_revision=pending['revision'],source_timestamp_ns=time.time_ns())
        adapter=TargetPreview(SimpleNamespace(request=request),'preview');adapter.bind_context('context')
        identity=dict(display_context='context',renderer_instance_id='preview',view_revision=0)
        adapter.target_pose([.1]*12,[80.,80.],**identity)
        original=adapter.ready.wait
        def wait(timeout):waiting.set();return original(timeout)
        adapter.ready.wait=wait
        reader=threading.Thread(target=lambda:result.append(adapter.media('target',1,**identity,wait_ms=100)))
        reader.start()
        try:
            self.assertTrue(waiting.wait(1));publish.set();adapter.start();reader.join(.2)
            self.assertFalse(reader.is_alive());self.assertEqual(result[0][0],b'exact')
            self.assertEqual(result[0][1]['client_revision'],1)
            self.assertEqual(adapter.state()['queue']['occupied'],1)
        finally:adapter.close();reader.join(1)

    def test_renderer_frame_wait_does_not_hold_state_or_presentation_ack_lock(self):
        waiting,release=threading.Event(),threading.Event();pending=None;sent=[]
        def request(path,body=None,**kwargs):
            nonlocal pending
            if path=='/state':return dict(renderer_instance_id='preview',mesh_rendering_enabled=False,preview_revision=0,overview_revision=0)
            if path=='/preview':pending=body;sent.append(body['revision']);return {}
            query=parse_qs(urlsplit(path).query);self.assertEqual(query['wait_ms'],['100'])
            if pending['revision']==2:
                waiting.set()
                if not release.wait(int(query['wait_ms'][0])/1000):raise PendingFrame('bounded renderer wait expired')
            return b'frame',dict(schema='target_preview_frame.v1',frame_id='frame-'+str(pending['revision']),renderer_instance_id='preview',
                view='preview',source='kinematic_target',overview_revision=0,target_revision=pending['revision'],source_timestamp_ns=time.time_ns())
        adapter=TargetPreview(SimpleNamespace(request=request),'preview');adapter.bind_context('context')
        identity=dict(display_context='context',renderer_instance_id='preview',view_revision=0)
        adapter.target_pose([.1]*12,[80.,80.],**identity);adapter.target_pose([.2]*12,[80.,80.],**identity);adapter.start()
        try:
            self.assertTrue(waiting.wait(1));started=time.monotonic()
            self.assertEqual(adapter.state()['queue']['occupied'],2)
            adapter.target_pose([.3]*12,[80.,80.],**identity,presented_through_revision=1)
            self.assertEqual([slot['revision'] for slot in adapter.slots],[2,3])
            self.assertEqual(adapter.media('target')[1]['client_revision'],1)
            self.assertLess(time.monotonic()-started,.05,'renderer HTTP wait must never hold the adapter lock')
            release.set();self.wait(lambda:adapter.state()['queue']['highest_completed_revision']==3)
            self.assertEqual(sent,[1,2,3])
        finally:release.set();adapter.close()

    def test_renderer_metadata_and_timestamp_regression_remain_rejected(self):
        rounds=0;pending=None;wrong='view'
        remote=dict(renderer_instance_id='preview',mesh_rendering_enabled=False,preview_revision=0,overview_revision=0)
        def request(path,body=None,**kwargs):
            nonlocal pending
            if path=='/state':return deepcopy(remote)
            if path=='/preview':pending=deepcopy(body);return {}
            meta=dict(schema='target_preview_frame.v1',frame_id='frame-'+str(pending['revision']),
                renderer_instance_id='wrong' if wrong=='renderer' else 'preview',view='preview',source='kinematic_target',
                overview_revision=1 if wrong=='view' else 0,target_revision=pending['revision'],
                source_timestamp_ns=1 if wrong=='timestamp' else time.time_ns())
            return b'jpeg',meta
        adapter=TargetPreview(SimpleNamespace(request=request),'preview');adapter.bind_context('context')
        identity=dict(display_context='context',renderer_instance_id='preview',view_revision=0)
        adapter.target_pose([.1]*12,[80.,80.],**identity)
        def next_iteration(delay):
            nonlocal rounds,wrong
            rounds+=1
            if rounds in (1,2):
                self.assertNotIn('target',adapter.frames)
                self.assertIn('matching atomic display metadata',adapter.error)
                wrong='renderer' if rounds==1 else None
            elif rounds==3:
                self.assertIsNone(adapter.error)
                self.assertEqual(adapter.media('target')[1]['client_revision'],1)
                wrong='timestamp';adapter.target_pose([.2]*12,[80.,80.],**identity)
            elif rounds==4:
                self.assertIn('timestamp moved backwards',adapter.error)
                self.assertEqual(adapter.media('target')[1]['client_revision'],1)
                remote['renderer_instance_id']='replacement'
            else:
                self.assertNotIn('target',adapter.frames)
                self.assertEqual(adapter.state()['queue']['occupied'],0)
                with self.assertRaisesRegex(ValueError,'rebind'):adapter.target_pose([.3]*12,[80.,80.],**identity,presented_through_revision=1)
                self.assertIn('explicit rebind required',adapter.error);adapter.closed.set()
        adapter.changed.wait=next_iteration;adapter._preview_loop()


if __name__=='__main__':unittest.main()
