"""Two runtime namespaces on one HTTP server; no robot or renderer connections."""
import http.client
import json
from pathlib import Path
import tempfile
import threading
from types import SimpleNamespace
import unittest
from http.server import ThreadingHTTPServer

from joint_head_workbench import Workbench, handler


class Runtime(Workbench):
    def __init__(self, root, name):
        self.name=name;self.output=(Path(root)/name).resolve();self.output.mkdir()
        (self.output/'frame.png').write_bytes(name.encode())
        self.calibration_root=Path(root)/(name+'-calibrations')
        self.csrf=name+'-request-token';self.config={'dsw_recovery_guide':'recover '+name}
        self.mode_lock=threading.RLock();self.status='READY';self.pipeline_running=False
        self.control_mode='hardware_sync';self.control=True
        self.owner={'motion':{'status':'READY'}};self.calls=[]
        self.tokens={'old':{}};self.hardware_tokens={'old'};self.calibration_previews={'old':{}}
        self.hardware_camera=SimpleNamespace(iter_frames=lambda:iter([(name.encode(),{})]))
    def state(self):
        return {'name':self.name,'csrf_token':self.csrf,'frame':'/artifacts/frame.png','nested':[{'url':'/api/hardware-frame'}]}
    def idle(self):
        if self.status=='RUNNING':raise ValueError('job is active')
    def control_request(self,path):
        self.calls.append(('control',path));return {'owner':self.owner}
    def set_control_mode(self,mode):
        self.calls.append(('mode',mode));self.control_mode=mode;return {'control_mode':mode}
    def action(self,path,body):
        self.calls.append((path,body));return {'name':self.name,'path':path}


class RoutingTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.physical=Runtime(self.temp.name,'physical');self.fake=Runtime(self.temp.name,'fake')
        self.server=ThreadingHTTPServer(('127.0.0.1',0),handler(self.physical,self.fake))
        self.thread=threading.Thread(target=self.server.serve_forever,daemon=True);self.thread.start()
    def tearDown(self):
        self.server.shutdown();self.server.server_close();self.thread.join();self.temp.cleanup()
    def request(self,path,body=None,token=None):
        connection=http.client.HTTPConnection('127.0.0.1',self.server.server_port,timeout=3)
        headers={'X-CSRF-Token':token or ''}
        connection.request('POST' if body is not None else 'GET',path,json.dumps(body) if body is not None else None,headers)
        response=connection.getresponse();data=response.read();status=response.status
        if response.getheader('Content-Type')=='application/json':data=json.loads(data)
        connection.close();return status,data
    def test_preview_etag_skips_unchanged_frame_without_hiding_new_pixels(self):
        self.fake.require_remote=lambda:None
        pixels=[b'jpeg-preview']
        self.fake.live_frame=lambda view:pixels[0]
        def frame(tag=None):
            c=http.client.HTTPConnection('127.0.0.1',self.server.server_port,timeout=3)
            c.request('GET','/fake/api/live-frame?view=preview',headers={'If-None-Match':tag} if tag else {})
            r=c.getresponse();answer=r.status,r.getheader('ETag'),r.read();c.close();return answer
        status,tag,data=frame()
        self.assertEqual((status,data),(200,pixels[0]))
        self.assertEqual(frame(tag),(304,tag,b''))
        pixels[0]=b'next-target'
        status,new_tag,data=frame(tag)
        self.assertEqual((status,data),(200,pixels[0]));self.assertNotEqual(tag,new_tag)

    def test_default_and_prefixed_state_and_artifacts_are_isolated(self):
        for prefix,name in (('', 'physical'),('/physical','physical'),('/fake','fake')):
            status,data=self.request(prefix+'/api/state')
            self.assertEqual(status,200);self.assertEqual(data['name'],name)
            self.assertEqual(data['frame'],'/'+name+'/artifacts/frame.png')
            self.assertEqual(data['nested'][0]['url'],'/'+name+'/api/hardware-frame')
            self.assertEqual(data['dsw_recovery_guide'],'recover '+name)
            self.assertEqual(self.request(prefix+'/artifacts/frame.png'),(200,name.encode()))
            status,stream=self.request(prefix+'/api/hardware-stream')
            self.assertEqual(status,200);self.assertIn(name.encode(),stream)
    def test_pages_rewrite_urls_and_inject_matching_context(self):
        for path,name in (('/','physical'),('/physical/','physical'),('/fake/','fake')):
            status,page=self.request(path);self.assertEqual(status,200)
            self.assertIn(('window.__WORKBENCH_RUNTIME__={"name": "'+name+'"').encode(),page)
            self.assertIn(('/'+name+'/api/state').encode(),page)
            self.assertNotIn(b"fetch('/api/",page)
            if name=='fake':
                self.assertIn(b'/fake/api/fake-controller-restart-preview',page)
                self.assertNotIn(b'/fake/fake/api/',page)
    def test_cross_namespace_tokens_and_old_unprefixed_fake_posts_fail(self):
        for path,token in (('/physical/api/test',self.fake.csrf),('/fake/api/test',self.physical.csrf),('/api/test',self.fake.csrf)):
            self.assertEqual(self.request(path,{},token)[0],403)
        self.assertEqual(self.physical.calls+self.fake.calls,[])
        self.assertEqual(self.request('/fake/api/test',{},self.fake.csrf)[1]['name'],'fake')

    def test_fake_restart_routes_never_reach_physical_runtime(self):
        calls=[]
        self.fake.fake_controller_restart_preview=lambda body:(calls.append(('preview',body)) or {'token':'one'})
        self.fake.fake_controller_restart_confirm=lambda body:(calls.append(('confirm',body)) or {'owner_pid':456})
        for path in ('fake-controller-restart-preview','fake-controller-restart-confirm'):
            self.assertEqual(self.request('/physical/api/'+path,{},self.physical.csrf)[0],400)
            self.assertEqual(self.request('/api/'+path,{},self.physical.csrf)[0],400)
            self.assertEqual(self.request('/fake/api/'+path,{},self.physical.csrf)[0],403)
        self.assertEqual(calls,[])
        self.assertEqual(self.request('/fake/api/fake-controller-restart-preview',{},self.fake.csrf)[1]['token'],'one')
        self.assertEqual(self.request('/fake/api/fake-controller-restart-confirm',{'token':'one'},self.fake.csrf)[1]['owner_pid'],456)
        self.assertEqual(calls,[('preview',{}),('confirm',{'token':'one'})])
    def test_runtime_switch_uses_source_token_target_mode_and_only_reads_owner(self):
        status,data=self.request('/physical/api/runtime-switch',{'runtime':'fake','mode':'hardware_sync'},self.physical.csrf)
        self.assertEqual(status,200);self.assertEqual(data['url'],'/fake/')
        self.assertEqual(self.fake.control_mode,'hardware_sync')
        self.assertEqual(self.physical.control_mode,'hardware_sync')
        self.assertEqual(self.physical.calls,[('control','state')])
        self.assertEqual(self.fake.calls,[('control','state'),('mode','hardware_sync')])
        self.assertEqual(self.physical.calibration_previews,{})
        self.assertEqual(self.fake.tokens,{})
        self.assertEqual(self.request('/fake/api/test',{},self.physical.csrf)[0],403)
    def test_active_either_side_blocks_switch_including_ready_pipeline(self):
        for instance in (self.physical,self.fake):
            for field,value in (('status','RUNNING'),('pipeline_running',True),('owner',{'automatic_enabled':True}),('owner',{'motion':{'status':'AWAITING_CAPTURE'}}),('owner',{'state':'VERIFYING'}),('owner',{})):
                old=getattr(instance,field);setattr(instance,field,value)
                try:
                    self.assertEqual(self.request('/physical/api/runtime-switch',{'runtime':'fake','mode':'hardware_sync'},self.physical.csrf)[0],400)
                    self.assertEqual(self.fake.control_mode,'hardware_sync')
                finally:setattr(instance,field,old)
    def test_invalid_runtime_and_physical_simulation_refused(self):
        for body in ({'runtime':'other','mode':'hardware_sync'},{'runtime':'physical','mode':'simulation'},{'runtime':'fake','mode':'other'},{'runtime':'fake','mode':'simulation'}):
            self.assertEqual(self.request('/physical/api/runtime-switch',body,self.physical.csrf)[0],400)
        self.assertEqual(self.request('/physical/api/control-mode',{'mode':'simulation'},self.physical.csrf)[0],400)
        self.assertEqual(self.fake.calls,[],'Rejected legacy mode must not reach the fake owner')
    def test_unstarted_owner_requires_empty_process_inspection(self):
        calls=[]
        processes=[]
        def gateway(path,body=None):
            calls.append(path)
            return ({'reachable':False,'owner':{},'starting_pid':None,'motion_enabled':False}
                    if path=='state' else {'processes':processes})
        self.physical.control_request=gateway
        body={'runtime':'fake','mode':'hardware_sync'}
        self.assertEqual(self.request('/physical/api/runtime-switch',body,self.physical.csrf)[0],200)
        self.assertEqual(calls,['state','takeover-preview'])
        processes.append({'pid':123})
        self.assertEqual(self.request('/physical/api/runtime-switch',body,self.physical.csrf)[0],400)
        self.assertNotIn('takeover-confirm',calls)

    def test_unreachable_gateway_blocks_switch_but_not_fake_namespace(self):
        self.physical.control_request=lambda *args:(_ for _ in ()).throw(ValueError('gateway unavailable'))
        self.assertEqual(self.request('/physical/api/runtime-switch',{'runtime':'fake','mode':'hardware_sync'},self.physical.csrf)[0],400)
        self.assertEqual(self.request('/fake/api/state')[0],200)
        self.assertEqual(self.request('/fake/')[0],200)

    def test_disconnected_known_owner_does_not_block_fake_development(self):
        self.physical.owner={'connected':False,'automatic_enabled':False,'state':'DISCONNECTED'}
        self.assertEqual(self.request('/physical/api/runtime-switch',{'runtime':'fake','mode':'hardware_sync'},self.physical.csrf)[0],200)

    def test_stopped_damping_hold_can_switch_without_recovering_owner(self):
        self.fake.status='STOPPED'
        self.fake.owner={'state':'STOPPED_DAMPING_HOLD','automatic_enabled':False,'motion':{'status':'STOPPED'}}
        status,data=self.request('/physical/api/runtime-switch',{'runtime':'fake','mode':'hardware_sync'},self.physical.csrf)
        self.assertEqual(status,200)
        self.assertEqual(self.fake.owner['state'],'STOPPED_DAMPING_HOLD')
        self.assertEqual(self.fake.calls,[('control','state'),('mode','hardware_sync')])

    def test_artifact_cannot_escape_namespace(self):
        self.assertEqual(self.request('/fake/artifacts/../physical/frame.png')[0],404)

    def test_export_does_not_rewrite_hashed_record_content(self):
        record={'version':'v000001','payload':{'url':'/api/internal','image':'/artifacts/frame.png'},'sha256':'unchanged'}
        self.fake.calibration_store_record=lambda ident:record
        self.assertEqual(self.request('/fake/api/calibration-file?id=x'),(200,record))

    def test_swapped_config_sources_refused_before_instances_start(self):
        from unittest.mock import patch
        from joint_head_workbench import main
        physical=Path(self.temp.name)/'physical.json';fake=Path(self.temp.name)/'fake.json'
        physical.write_text(json.dumps({'simulation_only':True}))
        fake.write_text(json.dumps({'simulation_only':False}))
        args=['workbench','--config',str(physical),'--output',str(self.physical.output),
              '--fake-config',str(fake),'--fake-output',str(self.fake.output)]
        with patch('sys.argv',args),patch('joint_head_workbench.Workbench') as create,patch('sys.stderr'):
            with self.assertRaises(SystemExit):main()
            create.assert_not_called()

    def test_duplicate_instance_or_shared_output_refused(self):
        with self.assertRaises(ValueError):handler(self.physical,self.physical)
        self.fake.calibration_root=self.physical.calibration_root
        with self.assertRaises(ValueError):handler(self.physical,self.fake)
        self.fake.calibration_root=Path(self.temp.name)/'other'
        self.fake.output=self.physical.output
        with self.assertRaises(ValueError):handler(self.physical,self.fake)


if __name__=='__main__':unittest.main()
