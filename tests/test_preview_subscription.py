"""Display-only view subscriptions with CPU pixels; ordinary renderer behavior stays intact."""
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import unittest
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'calibration'),str(ROOT/'calibration/infra')]
from examples.scripts.debug.joint_head_workbench_live import Session


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
        session.tick(renderer,.033);self.assertEqual(renderer.products,{'actual':False,'preview':False})
        session.command('/display-view',{'view':'overview'});session.tick(renderer,.033)
        self.assertEqual(renderer.calls,['target','overview']);self.assertFalse(renderer.products['preview'])
        session.command('/display-view',{'view':'target'});session.command('/preview',{'q':[.2]*12,'jaw_gap_mm':[80.,80.],'revision':2});session.tick(renderer,.033)
        self.assertEqual(renderer.calls,['target','overview','target']);self.assertFalse(renderer.products['actual'])
        np.testing.assert_array_equal(session.actual_q,np.zeros(12))
        with self.assertRaises(ValueError):session.command('/execute',{})

    def test_ordinary_renderer_rejects_subscription_and_keeps_actual_rendering(self):
        session=Session(Geometry(),Path(self.tmp.name)/'ordinary',{'mesh_rendering_enabled':False});renderer=Renderer()
        with self.assertRaises(ValueError):session.command('/display-view',{'view':'target'})
        session.tick(renderer,0);self.assertEqual(renderer.calls,['overview'])
        session.actual_render_started=None;session.tick(renderer,.033);self.assertEqual(renderer.calls,['overview','overview'])
        self.assertIsNone(session.active_display_view)


if __name__=='__main__':unittest.main()
