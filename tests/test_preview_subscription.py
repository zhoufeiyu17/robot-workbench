"""Display-only view subscriptions with CPU pixels; ordinary renderer behavior stays intact."""
from pathlib import Path
import sys
import threading
import time
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import Mock,patch
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'calibration'),str(ROOT/'calibration/infra')]
from examples.scripts.debug.joint_head_workbench_live import Session,serve,run_render_loop
from robot_workbench.target_preview import RendererClient,PendingFrame


class Geometry:
    velocity_limits=np.ones(12);limits=np.array([[-1,1]]*12)
    k=dict(width=4,height=3,fx=2.,fy=2.,cx=2.,cy=1.5);marker_size=.015


class Renderer:
    def __init__(self):self.calls=[];self.products={}
    def actual_updates(self,enabled):self.products['actual']=enabled
    def preview_updates(self,enabled):self.products['preview']=enabled
    def set_overview_view(self,view):pass
    def render(self,q,gaps=None,preview=False):self.calls.append('target' if preview else 'overview');return b'jpeg',b'head'
    def render_pair(self,*args):raise AssertionError('a hidden view must never consume a paired render')


class PreviewSubscriptionTest(unittest.TestCase):
    def setUp(self):
        self.tmp=TemporaryDirectory();self.addCleanup(self.tmp.cleanup)

    def test_dedicated_target_does_not_render_unsubscribed_overview_or_idle_products(self):
        session=Session(Geometry(),Path(self.tmp.name)/'dedicated',{'display_only':True,'mesh_rendering_enabled':False})
        renderer=Renderer();session.tick(renderer,0)
        self.assertEqual(renderer.calls,[]);self.assertEqual(renderer.products,{'actual':False,'preview':False})
        session.command('/preview',{'q':[.1]*12,'jaw_gap_mm':[80.,80.],'revision':1});session.tick(renderer,.033)
        self.assertEqual(renderer.calls,['target']);self.assertEqual(session.frame_metadata['preview']['target_revision'],1)
        np.testing.assert_array_equal(session.actual_q,np.zeros(12),'target-only rendering leaves the overview state untouched')
        self.assertEqual(session.status,'READY','the first successful dedicated target establishes renderer readiness')
        session.tick(renderer,.033);self.assertEqual(renderer.products,{'actual':False,'preview':False})
        session.command('/display-view',{'view':'overview'});session.tick(renderer,.033)
        self.assertEqual(renderer.calls,['target','overview']);self.assertFalse(renderer.products['preview'])
        np.testing.assert_array_equal(session.actual_q,[.1]*12,'display-only overview shows the last target')
        self.assertEqual(session.frame_metadata['actual']['source'],'kinematic_overview')
        self.assertFalse(session.frame_metadata['actual']['measurement_qualified'])
        session.command('/display-view',{'view':'target'});session.command('/preview',{'q':[.2]*12,'jaw_gap_mm':[80.,80.],'revision':2});session.tick(renderer,.033)
        self.assertEqual(renderer.calls,['target','overview','target']);self.assertFalse(renderer.products['actual'])
        np.testing.assert_array_equal(session.actual_q,[.1]*12,'a subsequent target does not render an unsubscribed overview')
        with self.assertRaises(ValueError):session.command('/execute',{})

    def test_ordinary_renderer_rejects_subscription_and_keeps_actual_rendering(self):
        session=Session(Geometry(),Path(self.tmp.name)/'ordinary',{'mesh_rendering_enabled':False});renderer=Renderer()
        with self.assertRaises(ValueError):session.command('/display-view',{'view':'target'})
        session.tick(renderer,0);self.assertEqual(renderer.calls,['overview'])
        session.actual_render_started=None;session.tick(renderer,.033);self.assertEqual(renderer.calls,['overview','overview'])
        self.assertIsNone(session.active_display_view)

    def test_target_loop_wakes_on_input_and_does_not_insert_idle_update_or_sleep_between_targets(self):
        session=Session(Geometry(),Path(self.tmp.name)/'wake',{'display_only':True,'mesh_rendering_enabled':False});renderer=Renderer()
        entered=threading.Event();original_wait=session.wait_display_work;original_render=renderer.render;errors=[]
        def wait(timeout=.05):entered.set();return original_wait(timeout)
        def render(q,gaps=None,preview=False):
            result=original_render(q,gaps,preview)
            if len(renderer.calls)==1:session.command('/preview',{'q':[.2]*12,'jaw_gap_mm':[80.,80.],'revision':2})
            return result
        session.wait_display_work=wait;renderer.render=render
        app=SimpleNamespace(is_running=lambda:len(renderer.calls)<2,update=Mock())
        def run():
            try:run_render_loop(app,session,renderer,time.monotonic()+2)
            except Exception as error:errors.append(error)
        with patch('examples.scripts.debug.joint_head_workbench_live.time.sleep') as sleep:
            worker=threading.Thread(target=run);worker.start()
            try:
                self.assertTrue(entered.wait(1));session.command('/preview',{'q':[.1]*12,'jaw_gap_mm':[80.,80.],'revision':1})
            finally:worker.join(2)
            self.assertFalse(worker.is_alive());sleep.assert_not_called()
        self.assertEqual(errors,[]);app.update.assert_not_called();self.assertEqual(renderer.calls,['target','target'])
        ordinary=Session(Geometry(),Path(self.tmp.name)/'ordinary-loop',{'mesh_rendering_enabled':False})
        app=SimpleNamespace(is_running=Mock(side_effect=[True,False]),update=Mock())
        with patch('examples.scripts.debug.joint_head_workbench_live.time.sleep') as sleep:
            run_render_loop(app,ordinary,Renderer(),time.monotonic()+1)
            self.assertEqual(sleep.call_count,1,'ordinary C keeps its original paced loop')

    def test_exact_renderer_wait_wakes_on_publication_and_rejects_view_change(self):
        session=Session(Geometry(),Path(self.tmp.name)/'http-wait',{'display_only':True,'mesh_rendering_enabled':False});renderer=Renderer()
        token=Path(self.tmp.name)/'token';token.write_text('x'*40);server=serve(session,'x'*40,0)
        client=RendererClient('http://127.0.0.1:'+str(server.server_port),token)
        entered=threading.Event();original=session.frame_ready.wait;results=[]
        def wait(timeout):self.assertLessEqual(timeout,.1);entered.set();return original(timeout)
        session.frame_ready.wait=wait
        def read(revision):
            try:results.append(client.request('/frame.jpg?view=preview&renderer_instance_id='+session.renderer_instance_id+'&target_revision='+str(revision)+'&overview_revision=0&wait_ms=100',frame=True))
            except Exception as error:results.append(error)
        session.command('/preview',{'q':[.1]*12,'jaw_gap_mm':[80.,80.],'revision':1})
        reader=threading.Thread(target=read,args=(1,));reader.start()
        try:
            self.assertTrue(entered.wait(1));started=time.monotonic();client.request('/state')
            self.assertLess(time.monotonic()-started,.05,'renderer state must not wait for a frame waiter')
            session.tick(renderer,0);reader.join(.3);self.assertFalse(reader.is_alive())
            raw,metadata=results[0];self.assertEqual(raw,b'jpeg');self.assertEqual(metadata['target_revision'],1)
            self.assertLessEqual(metadata['renderer_received_wall_ns'],metadata['renderer_render_started_wall_ns'])
            self.assertLessEqual(metadata['renderer_render_started_wall_ns'],metadata['source_timestamp_ns'])
            entered.clear();results.clear();session.command('/preview',{'q':[.2]*12,'jaw_gap_mm':[80.,80.],'revision':2})
            reader=threading.Thread(target=read,args=(2,));reader.start();self.assertTrue(entered.wait(1))
            session.command('/display-view',{'view':'overview'});reader.join(.3)
            self.assertFalse(reader.is_alive());self.assertIsInstance(results[0],PendingFrame,'old target pixels cannot be served after the view subscription changes')
            with self.assertRaises(ValueError):client.request('/frame.jpg?view=preview&wait_ms=101',frame=True)
        finally:
            reader.join(1);server.shutdown();server.server_close()


if __name__=='__main__':unittest.main()
