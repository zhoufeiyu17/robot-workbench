"""CPU HTTP registration/routing; fixture servers never access a robot or SSH."""
from http.client import HTTPConnection
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
import json
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import threading
import unittest
from urllib.parse import parse_qs,urlsplit

ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from serve_lan_gateway import serve
from robot_workbench.physical_endpoint import PhysicalEndpoint


def upstream(environment):
    node=dict(id='A1-cpu' if environment=='physical' else 'fake-cpu',environment=environment,
              connected=False,state_fail=False,gets=[],actions=[])
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*_):pass
        def send(self,code,value,kind='application/json',headers=None):
            raw=value if isinstance(value,bytes) else json.dumps(value).encode()
            self.send_response(code);self.send_header('Content-Type',kind);self.send_header('Content-Length',str(len(raw)))
            for key,value in (headers or {}).items():self.send_header(key,value)
            self.end_headers();self.wfile.write(raw)
        def do_GET(self):
            node['gets'].append(self.path);path=urlsplit(self.path);query=parse_qs(path.query)
            if path.path=='/api/session':return self.send(200,{'status':'ready'})
            if path.path=='/api/state':
                if node['state_fail']:return self.send(503,{'error':'fixture state unavailable'})
                selected=query.get('environment',[environment])[0]
                valid=selected==environment
                ident=node['id'] if valid else 'physical-unconfigured'
                value=dict(schema='robot-workbench-ui.v1',server_instance_id=node['id']+'-server',revision=1,csrf=environment+'-token',
                    device=dict(id=ident,label=ident,environment=selected),
                    runtime=dict(connected=node['connected'],mode='hardware' if environment=='physical' else 'simulation',
                        state='READY' if node['connected'] else 'DISCONNECTED',holder='calibration' if node['connected'] else None,
                        allowed_operations={'connect':{'allowed':valid and not node['connected']}}),workflow={'busy':False},
                    media=[{'url':'/api/media/head?environment='+selected}],
                    results=[{'media':{'url':'/api/result-media?environment='+selected+'&result_id=video'}}])
                return self.send(200,value)
            if path.path=='/api/media/head':return self.send(200,b'\xff\xd8cpu\xff\xd9','image/jpeg')
            if path.path=='/api/result-media':
                value=b'0123456789'
                if self.headers.get('Range')=='bytes=2-5':return self.send(206,value[2:6],'video/mp4',{'Content-Range':'bytes 2-5/10'})
                return self.send(200,value,'video/mp4')
            return self.send(200,b'<html><body><nav class="tabs"></nav></body></html>','text/html')
        def do_POST(self):
            body=json.loads(self.rfile.read(int(self.headers['Content-Length'])))
            if self.headers.get('X-CSRF-Token')!=environment+'-token':return self.send(403,{'error':'wrong upstream csrf'})
            if self.path=='/api/actions/connect' and body.get('arms_supported') is not True:
                return self.send(409,{'error':'explicit support required'})
            node['actions'].append(dict(path=self.path,body=body,host=self.headers['Host'],origin=self.headers.get('Origin')))
            if self.path=='/api/actions/connect':node['connected']=True
            if node.get('hold_response'):
                entered,release=node['hold_response'];entered.set();release.wait(2.)
            return self.send(200,{'accepted':self.path})
    server=ThreadingHTTPServer(('127.0.0.1',0),Handler);server.daemon_threads=True
    thread=threading.Thread(target=lambda:server.serve_forever(poll_interval=.01),daemon=True);thread.start()
    return server,thread,node


class EndpointTests(unittest.TestCase):
    def setUp(self):
        self.temp=TemporaryDirectory();self.addCleanup(self.temp.cleanup);self.root=Path(self.temp.name)
        self.fake,self.fake_thread,self.fake_node=upstream('fake')
        self.physical,self.physical_thread,self.node=upstream('physical')
        self.path=self.root/'physical.json'
        self.gateway=serve(listen='127.0.0.1',port=0,upstream_port=self.fake.server_port,
            physical_config=self.path,physical_assets=ROOT/'web',camera_config=self.root/'cameras.json',camera_assets=ROOT/'web')
        self.thread=threading.Thread(target=lambda:self.gateway.serve_forever(poll_interval=.01),daemon=True);self.thread.start()
        self.addCleanup(self.close)
        self.host=self.gateway.public_authority
        _,state=self.request('/api/physical-endpoint');self.token=state['csrf']
        self.endpoint=dict(address='127.0.0.1',port=self.physical.server_port)

    def close(self):
        for server,thread in ((self.gateway,self.thread),(self.fake,self.fake_thread),(self.physical,self.physical_thread)):
            server.shutdown();server.server_close();thread.join(1)

    def request(self,path,body=None,headers=None):
        connection=HTTPConnection('127.0.0.1',self.gateway.server_port,timeout=3)
        fields={'Host':self.host,'Origin':'http://'+self.host,**(headers or {})}
        data=None
        if body is not None:fields['Content-Type']='application/json';data=json.dumps(body).encode()
        connection.request('POST' if body is not None else 'GET',path,data,fields)
        response=connection.getresponse();raw=response.read();kind=response.getheader('Content-Type','');code=response.status
        connection.close()
        return code,json.loads(raw) if kind.startswith('application/json') else raw

    def configure(self):
        code,probe=self.manage(dict(operation='probe',endpoint=self.endpoint));self.assertEqual(code,200,probe)
        code,result=self.manage(dict(operation='save',endpoint=self.endpoint,expected_device_id=probe['identity']['device_id'],expected_route_id=None))
        self.assertEqual(code,200,result);return result['endpoint']

    def manage(self,value):
        return self.request('/api/physical-endpoint',value,{'X-Physical-Endpoint-Token':self.token})

    def test_registration_reads_only_and_bound_actions_keep_original_confirmation_and_csrf(self):
        endpoint=self.configure();self.assertEqual(self.node['actions'],[])
        code,state=self.request('/api/state?environment=physical&business=calibration')
        self.assertEqual(code,200);prefix='/device-physical/'+endpoint['route_id']
        self.assertEqual(state['gateway_route']['prefix'],prefix)
        self.assertTrue(state['media'][0]['url'].startswith(prefix))
        body=dict(environment='physical',business='calibration',request_id='cpu')
        self.assertEqual(self.request('/api/actions/connect',body,{'X-CSRF-Token':'physical-token'})[0],409)
        self.assertEqual(self.request(prefix+'/api/actions/connect',body,{'X-CSRF-Token':'physical-token'})[0],409)
        self.assertEqual(self.node['actions'],[])
        self.assertEqual(self.request(prefix+'/api/actions/connect',dict(body,arms_supported=True),{'X-CSRF-Token':'wrong'})[0],403)
        self.assertEqual(self.request(prefix+'/api/actions/connect',dict(body,arms_supported=True),{'X-CSRF-Token':'physical-token'})[0],200)
        self.assertEqual(len(self.node['actions']),1)
        self.assertEqual(self.node['actions'][0]['host'],f'127.0.0.1:{self.physical.server_port}')
        self.assertNotIn('physical-token',self.path.read_text())

    def test_fake_session_and_site_camera_routes_stay_separate_and_physical_media_ranges_work(self):
        endpoint=self.configure();prefix='/device-physical/'+endpoint['route_id']
        self.assertEqual(self.request('/api/state?environment=fake')[1]['device']['id'],'fake-cpu')
        self.assertEqual(self.request('/api/session')[1],{'status':'ready'})
        self.assertEqual(self.request('/api/site-cameras')[0],200)
        self.assertEqual(self.request(prefix+'/api/media/head?environment=physical')[1],b'\xff\xd8cpu\xff\xd9')
        self.assertEqual(self.request(prefix+'/api/result-media?environment=physical&result_id=video',headers={'Range':'bytes=2-5'}),(206,b'2345'))
        html=self.request('/')[1]
        self.assertIn(b'/web/site_cameras.js',html);self.assertIn(b'/web/physical_endpoint.js',html)
        self.assertEqual(self.request('/web/physical_endpoint.js')[0],200)

    def test_active_owner_blocks_rebinding_and_old_generations_never_control_replacement(self):
        old=self.configure();self.node['connected']=True
        self.assertEqual(self.manage(dict(operation='remove',expected_route_id=old['route_id']))[0],409)
        self.assertEqual(self.request('/api/physical-endpoint')[1]['endpoint'],old)
        self.node['connected']=False
        self.assertEqual(self.manage(dict(operation='remove',expected_route_id=old['route_id']))[0],200)
        new=self.configure();self.assertNotEqual(old['route_id'],new['route_id'])
        body=dict(environment='physical',business='calibration',request_id='old')
        self.assertEqual(self.request('/device-physical/'+old['route_id']+'/api/actions/stop',body,{'X-CSRF-Token':'physical-token'})[0],409)
        self.assertEqual(self.node['actions'],[])
        restored=PhysicalEndpoint(self.path,ROOT/'web',allow_loopback=True)
        self.assertEqual(restored.snapshot()['endpoint'],new)

    def test_unknown_retains_address_and_explicit_unbind_never_claims_stop_or_release(self):
        endpoint=self.configure();self.node['state_fail']=True
        self.assertEqual(self.manage(dict(operation='remove',expected_route_id=endpoint['route_id']))[0],409)
        self.assertEqual(self.request('/api/physical-endpoint')[1]['endpoint'],endpoint)
        body=dict(operation='forget_unreachable',expected_route_id=endpoint['route_id'],confirmation='wrong')
        self.assertEqual(self.manage(body)[0],409)
        code,result=self.manage(dict(body,confirmation='FORGET_UNREACHABLE_SERVICE_NOT_STOPPED'))
        self.assertEqual(code,200);self.assertIsNone(result['endpoint'])
        self.assertFalse(result['last_detached']['controller_stopped']);self.assertFalse(result['last_detached']['controller_released'])
        self.assertEqual(self.node['actions'],[])

    def test_stop_does_not_wait_for_broken_state_preflight_but_new_control_does_not_forward(self):
        endpoint=self.configure();self.node['state_fail']=True;prefix='/device-physical/'+endpoint['route_id']
        body=dict(environment='physical',business='calibration',request_id='stop')
        self.assertEqual(self.request(prefix+'/api/actions/stop',body,{'X-CSRF-Token':'physical-token'})[0],200)
        self.assertEqual(self.request(prefix+'/api/actions/connect',dict(body,arms_supported=True),{'X-CSRF-Token':'physical-token'})[0],409)
        self.assertEqual([row['path'] for row in self.node['actions']],['/api/actions/stop'])

    def test_invalid_addresses_placeholder_self_proxy_and_identity_change_fail_closed(self):
        for address in ('8.8.8.8','169.254.169.254','example.com'):
            self.assertEqual(self.manage(dict(operation='probe',endpoint=dict(self.endpoint,address=address)))[0],409)
        self.assertEqual(self.manage(dict(operation='probe',endpoint=dict(address='127.0.0.1',port=self.fake.server_port)))[0],409)
        self.assertEqual(self.manage(dict(operation='probe',endpoint=dict(address='127.0.0.1',port=self.gateway.server_port)))[0],409)
        endpoint=self.configure();self.node['id']='another-robot'
        body=dict(environment='physical',business='calibration',request_id='cpu',arms_supported=True)
        self.assertEqual(self.request('/device-physical/'+endpoint['route_id']+'/api/actions/connect',body,{'X-CSRF-Token':'physical-token'})[0],409)
        self.assertEqual(self.node['actions'],[])

    def test_in_flight_request_and_generation_recheck_close_endpoint_change_races(self):
        endpoint=self.configure();entered,release=threading.Event(),threading.Event()
        self.node['hold_response']=(entered,release);result=[]
        def call():
            result.append(self.request('/device-physical/'+endpoint['route_id']+'/api/actions/inference_refresh',
                dict(environment='physical',business='inference',request_id='in-flight'),{'X-CSRF-Token':'physical-token'}))
        thread=threading.Thread(target=call);thread.start()
        try:
            self.assertTrue(entered.wait(1.))
            code,value=self.manage(dict(operation='remove',expected_route_id=endpoint['route_id']))
            self.assertEqual(code,409);self.assertIn('在途',value['error'])
        finally:release.set();thread.join(2.)
        self.assertEqual(result[0][0],200)
        self.assertEqual(self.manage(dict(operation='remove',expected_route_id=endpoint['route_id']))[0],200)
        registry=PhysicalEndpoint(self.path,ROOT/'web',allow_loopback=True)
        with self.assertRaisesRegex(ValueError,'绑定已变化'):registry.begin_post(endpoint)


if __name__=='__main__':unittest.main()
