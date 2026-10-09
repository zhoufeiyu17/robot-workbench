"""Operational status remains readable while planning/catalog locks are held."""
import http.client
import json
import threading
import time
import unittest
from http.server import ThreadingHTTPServer
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

from joint_head_workbench import Workbench, handler


class RuntimeStateTests(unittest.TestCase):
    def setUp(self):
        a=self.app=Workbench.__new__(Workbench)
        a.lock=threading.Lock();a.mode_lock=threading.Lock()
        a.live=True;a.control_mode='hardware_sync';a.config={'simulation_only':True}
        a.remote={'status':'READY','timestamp_s':1.,'render':{},'capture_manifest':['large']}
        a.remote_error=None;a.remote_received=time.monotonic()
        owner={'pid':12,'command_epoch':3,'state':'READY','connected':True,
               'simulation_only':True,'physical_hardware_connected':False,'evidence_source':'fake',
               'calibration_feedback':{'qpos':[0.]*14},'arrival_events':[{'large':'history'}]*500}
        a.hardware_feedback={'owner':owner,'source_age_s':.01}
        a.hardware_received=time.monotonic();a.hardware_error=None;a.mirror_error=None
        a.control_status={'owner':owner,'motion_enabled':True};a.control_received=time.monotonic()
        a.q=np.zeros(12);a.jaw_mm=np.array([80.,80.]);a.velocity_limits=np.ones(12)
        a.geometry=SimpleNamespace(limits=np.array([[-3.,3.]]*12))
        a.status='READY';a.reason=None;a.import_error=None;a.csrf='request-token'
        a.queue=[{'capture':True}];a.frames=[];a.selected_batch='batch'
        a.pipeline_running=True;a.pipeline_review={'kind':'wrist'}
        a.pipeline_report={'phase':'腕部：采集','steps':[{'step':'capture','half':'head'}],
                           'subprogress':{'stage':4,'fraction':.5,'completed':16,'total':32}}
        a.motion_preflight={'request_id':'r','status':'checking','stage':'正在检查路径'}
        a.review_generation=7
        for name in ('state','history','calibration_state','library_state','arrival_state','wrist_results'):
            setattr(a,name,Mock(side_effect=AssertionError('catalog must not be read')))

    def test_runtime_reads_no_global_locks_or_catalogs_and_omits_feedback_history(self):
        a=self.app
        with a.lock,a.mode_lock,patch('joint_head_workbench.read',side_effect=AssertionError('disk read')):
            result=[]
            thread=threading.Thread(target=lambda:result.append(a.runtime_state()),daemon=True)
            thread.start();thread.join(.5)
            self.assertFalse(thread.is_alive(),'operational state waited for a planning/page lock')
        thread.join(1)
        s=result[0]
        self.assertEqual((s['robot_control'],s['robot_feedback']),('owned_by_workbench','fresh'))
        self.assertEqual(s['review_generation'],7)
        self.assertEqual(s['motion_preflight']['stage'],'正在检查路径')
        self.assertEqual(s['pipeline']['subprogress']['completed'],16)
        self.assertEqual(s['counts']['capture_points'],1)
        self.assertEqual(s['hardware_feedback']['owner']['simulation_only'],True)
        self.assertNotIn('arrival_events',s['hardware_feedback']['owner'])
        self.assertNotIn('calibration_feedback',s['hardware_feedback']['owner'])
        self.assertNotIn('capture_manifest',s['remote'])
        for key in ('queue','history','calibration','library','frames','actual_parts'):
            self.assertNotIn(key,s)
        for name in ('state','history','calibration_state','library_state','arrival_state','wrist_results'):
            getattr(a,name).assert_not_called()
        a.pipeline_report['steps'].append({'step':'capture','half':'wrist'})
        a.motion_preflight['status']='ready'
        self.assertEqual(len(s['pipeline']['steps']),1)
        self.assertEqual(s['motion_preflight']['status'],'checking')

    def test_stale_feedback_never_looks_controllable(self):
        self.app.hardware_received=time.monotonic()-1
        s=self.app.runtime_state()
        self.assertFalse(s['hardware_sync_active'])
        self.assertFalse(s['hardware_control_connected'])
        self.assertIsNone(s['hardware_feedback'])
        self.assertEqual(s['execution_blocker']['code'],'feedback_stale')

    def test_http_runtime_route_uses_cached_snapshot_without_full_state(self):
        server=ThreadingHTTPServer(('127.0.0.1',0),handler(self.app))
        thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        try:
            with self.app.lock,self.app.mode_lock:
                c=http.client.HTTPConnection('127.0.0.1',server.server_port,timeout=1)
                c.request('GET','/api/runtime-state');r=c.getresponse();s=json.loads(r.read());c.close()
            self.assertEqual(r.status,200)
            self.assertEqual(s['runtime']['name'],'physical')
            self.assertEqual(s['review_generation'],7)
            self.app.state.assert_not_called()
        finally:
            server.shutdown();server.server_close();thread.join()


if __name__=='__main__':unittest.main()
