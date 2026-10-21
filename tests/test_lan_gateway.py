"""Real loopback HTTP proxy checks; no DSW, robot or external network."""
from copy import deepcopy
from http.client import HTTPConnection,IncompleteRead
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
import json
from pathlib import Path
import socket
import sys
import threading
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'scripts'))
import serve_lan_gateway as gateway

VIDEO=bytes(range(256))*8192
JPEG=b'\xff\xd8fixture-jpeg\xff\xd9'


class GatewayTest(unittest.TestCase):
    def setUp(self):
        self.seen=[]
        seen=self.seen
        class Upstream(BaseHTTPRequestHandler):
            protocol_version='HTTP/1.1'
            def log_message(self,*args):pass
            def send(self,code,raw,kind='application/json',extra=()):
                self.send_response(code);self.send_header('Content-Type',kind);self.send_header('Content-Length',str(len(raw)))
                for name,value in extra:self.send_header(name,value)
                self.end_headers()
                try:self.wfile.write(raw)
                except OSError:pass
            def do_GET(self):
                seen.append((self.command,self.path,list(self.headers.items())))
                if self.path=='/api/session':return self.send(200,b'{"csrf":"health-token-must-not-leak","status":"ready"}')
                if self.path=='/jpeg':return self.send(200,JPEG,'image/jpeg',[('ETag','"image-one"')])
                if self.path=='/video':
                    headers=[('Accept-Ranges','bytes'),('Last-Modified','Sun, 11 Oct 2026 10:00:00 GMT')]
                    if self.headers.get('Range'):
                        start,end=map(int,self.headers['Range'].removeprefix('bytes=').split('-'))
                        return self.send(206,VIDEO[start:end+1],'video/mp4',headers+[('Content-Range',f'bytes {start}-{end}/{len(VIDEO)}')])
                    return self.send(200,VIDEO,'video/mp4',headers)
                if self.path in ('/chunked','/broken-chunk'):
                    self.send_response(200);self.send_header('Transfer-Encoding','chunked');self.send_header('Connection','close, X-Internal')
                    self.send_header('X-Internal','private-hop');self.end_headers();self.close_connection=True
                    self.wfile.write(b'5\r\nhello\r\n6\r\n world\r\n')
                    if self.path=='/chunked':self.wfile.write(b'0\r\n\r\n')
                    return
                if self.path=='/truncated':
                    self.send_response(200);self.send_header('Content-Length','100');self.end_headers()
                    self.wfile.write(b'short');self.close_connection=True;return
                if self.path=='/bad-length':
                    self.send_response(200);self.send_header('Content-Length','0');self.send_header('Content-Length','1');self.end_headers()
                    self.close_connection=True;return
                if self.path=='/error':return self.send(503,b'{"error":"fixture unavailable"}')
                return self.send(200,json.dumps({'path':self.path,'host':self.headers['Host'],'origin':self.headers.get('Origin')}).encode(),
                    extra=[('Connection','close, X-Internal'),('X-Internal','private-hop')])
            def do_POST(self):
                seen.append((self.command,self.path,list(self.headers.items())))
                body=self.rfile.read(int(self.headers['Content-Length']))
                if self.headers.get('X-CSRF-Token')!='fixture-token':return self.send(403,b'{"error":"invalid token"}')
                return self.send(200,body,self.headers.get('Content-Type','application/octet-stream'),
                    [('X-Observed-Host',self.headers['Host']),('X-Observed-Origin',self.headers.get('Origin',''))])
        self.upstream=ThreadingHTTPServer(('127.0.0.1',0),Upstream);self.upstream.daemon_threads=True
        self.upstream_thread=threading.Thread(target=self.upstream.serve_forever,daemon=True);self.upstream_thread.start()
        self.server=gateway.serve(listen='127.0.0.1',port=0,upstream_port=self.upstream.server_port)
        self.thread=threading.Thread(target=self.server.serve_forever,daemon=True);self.thread.start()
        self.addCleanup(self.close)
        self.host=self.server.public_authority;self.origin='http://'+self.host

    def close(self):
        self.server.shutdown();self.server.server_close();self.thread.join(1)
        self.upstream.shutdown();self.upstream.server_close();self.upstream_thread.join(1)

    def request(self,method='GET',path='/json',*,headers=None,body=None):
        connection=HTTPConnection('127.0.0.1',self.server.server_port,timeout=3)
        self.addCleanup(connection.close)
        connection.request(method,path,body,{'Host':self.host,**(headers or {})})
        response=connection.getresponse()
        return response.status,dict(response.getheaders()),response.read()

    def raw(self,method='POST',path='/post',headers=(),body=b'',incomplete=False):
        connection=HTTPConnection('127.0.0.1',self.server.server_port,timeout=3)
        self.addCleanup(connection.close)
        connection.putrequest(method,path,skip_host=True,skip_accept_encoding=True)
        for key,value in headers:connection.putheader(key,value)
        connection.endheaders(body)
        if incomplete:connection.sock.shutdown(socket.SHUT_WR)
        response=connection.getresponse();return response.status,response.read()

    def test_json_jpeg_range_head_and_post_keep_protocol_fields(self):
        code,headers,body=self.request(headers={'Origin':self.origin,'Connection':'close, X-Hop','X-Hop':'discard','X-Request-ID':'trace-one'})
        self.assertEqual(code,200);self.assertNotIn('X-Internal',headers)
        self.assertEqual(json.loads(body),{'path':'/json','host':'127.0.0.1:43759','origin':'http://127.0.0.1:43759'})
        sent=dict(self.seen[-1][2]);self.assertNotIn('X-Hop',sent);self.assertEqual(sent['X-Request-ID'],'trace-one')
        code,headers,body=self.request(path='/jpeg');self.assertEqual(body,JPEG);self.assertEqual(headers['ETag'],'"image-one"')
        self.assertEqual(headers['Content-Type'],'image/jpeg')
        code,headers,body=self.request(path='/video',headers={'Range':'bytes=101-100100'})
        self.assertEqual(code,206);self.assertEqual(body,VIDEO[101:100101]);self.assertEqual(headers['Content-Range'],f'bytes 101-100100/{len(VIDEO)}')
        self.assertEqual(headers['Accept-Ranges'],'bytes');self.assertEqual(int(headers['Content-Length']),100000)
        code,headers,body=self.request('HEAD','/video',headers={'Range':'bytes=101-200'})
        self.assertEqual(code,206);self.assertEqual(body,b'');self.assertEqual(int(headers['Content-Length']),100)
        self.assertEqual(self.seen[-1][0],'GET','HEAD works with the existing GET-only manager')
        payload=b'{"request_id":"fixture","nested":{"value":1}}'
        code,headers,body=self.request('POST','/post',headers={'Origin':self.origin,'X-CSRF-Token':'fixture-token','Content-Type':'application/json'},body=payload)
        self.assertEqual(code,200);self.assertEqual(body,payload)
        self.assertEqual(headers['X-Observed-Host'],'127.0.0.1:43759');self.assertEqual(headers['X-Observed-Origin'],'http://127.0.0.1:43759')
        sent=dict(self.seen[-1][2]);self.assertEqual(sent['X-CSRF-Token'],'fixture-token');self.assertEqual(sent['Content-Type'],'application/json')
        self.assertEqual(self.request('POST','/post',headers={'X-CSRF-Token':'wrong'},body=b'{}')[0],403,'upstream CSRF rejection is preserved')

    def test_strict_host_origin_target_and_request_framing(self):
        for headers in ([('Host',self.host),('Host',self.host)],[('Host','evil.test')],[],
                [('Host',self.host),('Origin',self.origin),('Origin',self.origin)],
                [('Host',self.host),('Origin','http://evil.test')],[('Host',self.host),('Origin','null')]):
            with self.subTest(headers=headers):self.assertEqual(self.raw('GET',headers=headers)[0],403)
        for path in ('http://evil.test/json','//evil.test/json','//[','/json#fragment','/\\evil.test'):
            with self.subTest(path=path):self.assertEqual(self.raw('GET',path,[('Host',self.host)])[0],400)
        for fields in ([],[('Content-Length','2'),('Content-Length','2')],[('Content-Length','-1')],
                       [('Content-Length','2'),('Transfer-Encoding','chunked')],[('Transfer-Encoding','identity')],
                       [('Content-Length','2'),('Connection','X-CSRF-Token')]):
            with self.subTest(fields=fields):self.assertEqual(self.raw(headers=[('Host',self.host),*fields])[0],400)
        for size in ('0',str(gateway.BODY_LIMIT+1)):
            self.assertEqual(self.raw(headers=[('Host',self.host),('Content-Length',size)])[0],413)
        self.assertEqual(self.raw(headers=[('Host',self.host),('Content-Length','10')],body=b'{}',incomplete=True)[0],400)
        for method in ('GET','HEAD'):
            self.assertEqual(self.raw(method,headers=[('Host',self.host),('Content-Length','2')],body=b'{}')[0],400)
        self.assertEqual(self.request('PUT')[0],405);self.assertEqual(self.request('OPTIONS')[0],405)
        self.assertEqual(self.seen,[],'all invalid requests must be rejected before touching the upstream')

    def test_bounded_streaming_chunked_and_exact_body_limit(self):
        reads=[]
        class CheckedConnection(HTTPConnection):
            def getresponse(connection):
                response=super().getresponse();read=response.read
                def bounded(size=None):
                    self.assertIsInstance(size,int);self.assertGreater(size,0);self.assertLessEqual(size,gateway.CHUNK_SIZE)
                    reads.append(size);return read(size)
                response.read=bounded;return response
        with patch.object(gateway,'HTTPConnection',CheckedConnection):
            code,headers,body=self.request(path='/video');self.assertEqual(code,200);self.assertEqual(body,VIDEO)
        self.assertGreater(len(reads),10)
        code,headers,body=self.request(path='/chunked');self.assertEqual(code,200);self.assertEqual(body,b'hello world')
        self.assertNotIn('X-Internal',headers);self.assertEqual(headers['Transfer-Encoding'],'chunked')
        payload=b'x'*gateway.BODY_LIMIT
        code,headers,body=self.request('POST','/post',headers={'X-CSRF-Token':'fixture-token'},body=payload)
        self.assertEqual(code,200);self.assertEqual(body,payload)

    def test_upstream_errors_and_truncation_cannot_appear_successful(self):
        self.assertEqual(self.request(path='/error')[0],503)
        self.assertEqual(self.request(path='/bad-length')[0],502)
        for path in ('/truncated','/broken-chunk'):
            with self.subTest(path=path),self.assertRaises(IncompleteRead):self.request(path=path)
        self.upstream.shutdown();self.upstream.server_close();self.upstream_thread.join(1)
        self.assertEqual(self.request()[0],503)
        code,headers,body=self.request(path='/healthz');self.assertEqual(code,503)
        self.assertEqual(json.loads(body),{'gateway':'ready','upstream_reachable':False,'upstream_http_status':None})

    def test_health_contains_no_upstream_body_or_token(self):
        code,headers,body=self.request(path='/healthz');self.assertEqual(code,200)
        self.assertEqual(json.loads(body),{'gateway':'ready','upstream_reachable':True,'upstream_http_status':200})
        self.assertNotIn(b'csrf',body);self.assertNotIn(b'health-token',body)
        self.assertEqual(self.request('HEAD','/healthz')[2],b'')
        before=len(self.seen);self.assertEqual(self.request('POST','/healthz',body=b'{}')[0],405);self.assertEqual(len(self.seen),before)

    def test_bind_and_upstream_are_fixed_explicit_loopback_routes(self):
        for kwargs in ({'listen':'0.0.0.0'},{'listen':'example.com'},{'listen':'8.8.8.8'},
                       {'upstream_authority':'evil.test:43759'},{'upstream_authority':'127.0.0.1:0'},
                       {'upstream_port':0}):
            with self.subTest(kwargs=kwargs),self.assertRaises(ValueError):gateway.serve(**kwargs)

    def test_optional_frozen_frontend_keeps_api_proxy_and_existing_module_routes(self):
        for path in ('/','/web/app.js','/web/style.css'):
            self.assertEqual(json.loads(self.request(path=path)[2])['path'],path)
        with TemporaryDirectory() as temporary:
            root=Path(temporary);web=root/'web';web.mkdir()
            bodies={'index.html':b'<!doctype html><title>frozen shared workbench</title>',
                    'app.js':b"import './session.js';",'style.css':b'body { color: black; }',
                    'session.js':b'export const current = true;',
                    'site_cameras.js':b'fallback camera module',
                    'physical_endpoint.js':b'fallback physical module'}
            for name,raw in bodies.items():(web/name).write_bytes(raw)
            camera_assets=root/'camera-assets';camera_assets.mkdir()
            (camera_assets/'site_cameras.js').write_bytes(b'camera route wins')
            (camera_assets/'physical_endpoint.js').write_bytes(b'physical route wins')
            camera_config=root/'cameras.json';camera_config.write_text('{"version":1,"devices":[]}')
            self.server.shutdown();self.server.server_close();self.thread.join(1)
            self.server=gateway.serve(listen='127.0.0.1',port=0,upstream_port=self.upstream.server_port,
                workbench_assets=web,camera_config=camera_config,camera_assets=camera_assets,
                physical_config=root/'physical.json',physical_assets=camera_assets)
            self.thread=threading.Thread(target=self.server.serve_forever,daemon=True);self.thread.start()
            self.host=self.server.public_authority;self.origin='http://'+self.host
            before=len(self.seen)
            for path,name,kind in [('/','index.html','text/html'),('/index.html','index.html','text/html'),
                    ('/web/index.html','index.html','text/html'),('/web/app.js?v=2','app.js','text/javascript'),
                    ('/web/style.css','style.css','text/css'),('/web/session.js','session.js','text/javascript')]:
                code,headers,body=self.request(path=path)
                self.assertEqual(code,200);self.assertEqual(body,bodies[name])
                self.assertEqual(headers['Content-Type'],kind+'; charset=utf-8')
                self.assertEqual(headers['Cache-Control'],'no-store')
                head_code,head_headers,head_body=self.request('HEAD',path)
                self.assertEqual(head_code,200);self.assertEqual(head_body,b'')
                self.assertEqual(int(head_headers['Content-Length']),len(body))
            self.assertEqual(self.request(path='/web/site_cameras.js')[2],b'camera route wins')
            self.assertEqual(self.request(path='/web/physical_endpoint.js')[2],b'physical route wins')
            (root/'secret.json').write_text('must never serve')
            for path in ('/web/../secret.json','/web/%2e%2e/secret.json','/web/app.js/../../secret.json','/web/unknown.js'):
                self.assertEqual(self.request(path=path)[0],404)
            self.assertEqual(self.request(path='/',headers={'Host':'evil.test'})[0],403)
            self.assertEqual(self.request('POST','/web/app.js',body=b'{}')[0],405)
            self.assertEqual(len(self.seen),before)
            for path in ('/api/state?environment=fake','/api/session','/session'):
                self.assertEqual(self.request(path=path)[0],200)
                self.assertEqual(self.seen[-1][1],path)
            payload=b'{"action":"fixture"}'
            self.assertEqual(self.request('POST','/api/action',headers={'X-CSRF-Token':'fixture-token'},body=payload)[2],payload)
            self.assertEqual(self.seen[-1][:2],('POST','/api/action'))
            # The bundle is frozen at startup, so edits cannot create a mixed frontend.
            (web/'app.js').write_bytes(b'not yet published')
            self.assertEqual(self.request(path='/web/app.js')[2],bodies['app.js'])
            self.upstream.shutdown();self.upstream.server_close();self.upstream_thread.join(1)
            self.assertEqual(self.request(path='/')[2],bodies['index.html'])
            self.assertEqual(self.request(path='/api/session')[0],503)
            (web/'style.css').unlink();(web/'style.css').symlink_to(root/'secret.json')
            with self.assertRaises(ValueError):gateway.serve(workbench_assets=web)
            with self.assertRaises(ValueError):gateway.serve(workbench_assets='relative/web')


class ManagedGatewayTest(unittest.TestCase):
    def setUp(self):
        self.temp=TemporaryDirectory();self.addCleanup(self.temp.cleanup);self.root=Path(self.temp.name).resolve()
        self.nodes={};self.events=[];self.servers=[]
        for environment,digest in [('fake','a'*64),('physical','b'*64)]:
            state=dict(schema='robot_workbench.environment.v1',csrf=environment+'-manager-token',selected_profile_id=environment+'-profile',
                target=dict(environment=environment,device_id=environment+'-device',manager_instance=environment+'-manager'),
                deployment=dict(sha256=digest,files_unchanged=True),runtime=dict(connected=False),health=dict(busy=False,maintenance=None),
                operations={},active_job=None,recent_jobs=[])
            self.nodes[environment]=state
            server=ThreadingHTTPServer(('127.0.0.1',0),self.handler(environment))
            thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start();self.servers.append((server,thread))
        registry=dict(schema='physical_endpoint_registry.v1',endpoint=dict(address='127.0.0.1',port=self.servers[1][0].server_port,
            route_id='f'*32,device_id='physical-device',label='Physical test'))
        self.registry=self.root/'physical.json';self.registry.write_text(json.dumps(registry));self.original_registry=self.registry.read_bytes()
        self.profiles=self.root/'profiles.json';self.profiles.write_text(json.dumps([dict(id=env+'-profile',environment=env,device_id=env+'-device',
            label=env,manager_url='/',robot_address='127.0.0.1' if env=='physical' else '',configured=True) for env in self.nodes]))
        self.server=gateway.serve(listen='127.0.0.1',port=0,upstream_port=self.servers[0][0].server_port,
            upstream_authority='127.0.0.1:'+str(self.servers[0][0].server_port),physical_config=self.registry,manager_profiles=self.profiles)
        self.thread=threading.Thread(target=self.server.serve_forever,daemon=True);self.thread.start()
        self.addCleanup(self.cleanup)

    def handler(self,environment):
        owner=self
        class Manager(BaseHTTPRequestHandler):
            def log_message(self,*args):pass
            def send(self,code,value,kind='application/json'):
                raw=json.dumps(value).encode() if not isinstance(value,bytes) else value
                self.send_response(code);self.send_header('Content-Type',kind);self.send_header('Content-Length',str(len(raw)))
                self.send_header('X-Workbench-Deployment',owner.nodes[environment]['deployment']['sha256']);self.end_headers();self.wfile.write(raw)
            def do_GET(self):
                from urllib.parse import urlsplit
                path=urlsplit(self.path).path;state=owner.nodes[environment]
                if path=='/api/environment':return self.send(200,state)
                if path in ('/','/index.html','/operations','/session'):
                    return self.send(200,b'<html><head><link href="/web/style.css"></head><body><script type="module" src="/web/app.js"></script><script type="module">import x from "/web/operations.js";</script></body></html>','text/html')
                if path.startswith('/web/'):
                    return self.send(200,('const version="'+state['deployment']['sha256']+'"; import "./operations.js";').encode(),'text/javascript')
                if path=='/api/state':return self.send(200,dict(schema='robot-workbench-ui.v1',server_instance_id='physical-backend',csrf='physical-business-token',
                    device=dict(id='physical-device',environment='physical'),runtime=dict(environment='physical',mode='hardware',connected=False,allowed_operations={}),workflow=dict(busy=False)))
                return self.send(200,dict(environment=environment,path=self.path))
            def do_POST(self):
                body=json.loads(self.rfile.read(int(self.headers['Content-Length'])))
                state=owner.nodes[environment];business=self.path.startswith('/api/actions/')
                expected=environment+'-business-token' if business else state['csrf']
                if self.headers.get('X-CSRF-Token')!=expected:return self.send(403,{'error':'wrong upstream csrf'})
                if not self.path.startswith(('/api/operations/stop','/api/actions/stop')) and self.headers.get('X-Workbench-Deployment')!=state['deployment']['sha256']:
                    return self.send(409,{'error':'wrong upstream deployment'})
                owner.events.append(dict(environment=environment,path=self.path,body=body,headers=dict(self.headers)))
                return self.send(200,dict(status='accepted',environment=environment))
        return Manager

    def cleanup(self):
        self.server.shutdown();self.server.server_close();self.thread.join(1)
        for server,thread in self.servers:server.shutdown();server.server_close();thread.join(1)

    def request(self,path,body=None,*,environment='fake',headers=None):
        connection=HTTPConnection('127.0.0.1',self.server.server_port,timeout=4)
        state=self.nodes[environment];extra={}
        if body is not None:extra={'Content-Type':'application/json','Origin':'http://'+self.server.public_authority,
            'X-CSRF-Token':state['csrf'],'X-Workbench-Deployment':state['deployment']['sha256']}
        connection.request('POST' if body is not None else 'GET',path,json.dumps(body) if body is not None else None,
            {'Host':self.server.public_authority,**extra,**(headers or {})})
        response=connection.getresponse();result=(response.status,dict(response.getheaders()),response.read());connection.close();return result

    def test_environment_bound_modules_and_neutral_facts_never_mix_managers(self):
        for environment in self.nodes:
            code,headers,body=self.request('/?environment='+environment)
            self.assertEqual(code,200);self.assertIn(('/manager/'+environment+'/web/app.js').encode(),body)
            self.assertIn(('/manager/'+environment+'/web/operations.js').encode(),body)
            code,headers,body=self.request('/manager/'+environment+'/web/app.js')
            self.assertEqual(code,200);self.assertEqual(headers['X-Workbench-Deployment'],self.nodes[environment]['deployment']['sha256'])
            self.assertIn(self.nodes[environment]['deployment']['sha256'].encode(),body);self.assertIn(b'./operations.js',body)
            code,_,body=self.request('/api/environment?environment='+environment)
            value=json.loads(body);self.assertEqual(value['target']['environment'],environment)
            self.assertEqual([row['id'] for row in value['profiles']],['fake-profile','physical-profile'])
            self.assertEqual(value['gateway_routing']['asset_prefix'],'/manager/'+environment+'/web/')
        self.assertEqual(self.request('/web/physical_endpoint.js')[0],409)
        self.assertEqual(self.request('/manager/fake/api/operations/action',{})[0],404)
        self.assertEqual(self.request('/manager/fake/web/app.js',{})[0],405)
        self.assertEqual(self.request('/manager/fake/web/../../secret')[0],404)
        self.assertEqual(self.events,[])

    def test_operation_authority_and_independent_stop_stay_with_selected_manager(self):
        for environment in self.nodes:
            value=dict(action='prepare_model',request_id='one',expected_target=self.nodes[environment]['target'])
            self.assertEqual(self.request('/api/operations/action?environment='+environment,value,environment=environment)[0],200)
            self.assertEqual(self.events[-1]['environment'],environment)
            self.assertEqual(self.events[-1]['headers']['X-Workbench-Deployment'],self.nodes[environment]['deployment']['sha256'])
        count=len(self.events)
        self.assertEqual(self.request('/api/operations/action?environment=physical',{},headers={'X-CSRF-Token':'physical-manager-token'})[0],409)
        self.nodes['physical']['schema']='old-version'
        self.assertEqual(self.request('/api/operations/action?environment=physical',{},environment='physical')[0],409)
        self.assertEqual(len(self.events),count,'wrong loaded page or schema cannot forward a write')
        self.nodes['physical']['active_job']={'status':'running'}
        self.assertEqual(self.request('/api/operations/stop?environment=physical',{'expected_device_id':'physical-device','request_id':'stop'},environment='physical',headers={'X-Workbench-Deployment':'stale'})[0],200)
        self.assertEqual(self.events[-1]['path'],'/api/operations/stop?environment=physical')
        self.nodes['fake']['schema']='old-version';count=len(self.events)
        self.assertEqual(self.request('/api/actions/connect',{'environment':'fake'},headers={'X-CSRF-Token':'fake-business-token'})[0],409)
        self.assertEqual(self.request('/api/session/start',{'request_id':'old'})[0],410)
        self.assertEqual(len(self.events),count,'legacy business/session writes cannot bypass the managed schema gate')

    def test_cross_profile_target_is_readonly_and_requires_current_control_released(self):
        body=dict(profile_id='physical-profile',expected_target=self.nodes['fake']['target'],robot_address='127.0.0.1')
        code,_,raw=self.request('/api/operations/target?environment=fake',body);self.assertEqual(code,200)
        value=json.loads(raw);self.assertEqual(value['redirect_url'],'http://'+self.server.public_authority+'/?environment=physical')
        self.assertFalse(value['controller_acquired']);self.assertEqual(value['environment']['target']['device_id'],'physical-device')
        self.nodes['fake']['runtime']['connected']=True
        self.assertEqual(self.request('/api/operations/target?environment=fake',body)[0],409)
        self.nodes['fake']['runtime']['connected']=None
        self.assertEqual(self.request('/api/operations/target?environment=fake',body)[0],409)
        self.nodes['fake']['runtime']['connected']=False;body['robot_address']='10.99.99.99'
        self.assertEqual(self.request('/api/operations/target?environment=fake',body)[0],409)
        self.assertEqual(self.request('/api/physical-endpoint',{'operation':'save'})[0],410)
        self.assertEqual(self.registry.read_bytes(),self.original_registry);self.assertEqual(self.events,[])

    def test_verified_stopped_backend_can_switch_without_starting_or_releasing_control(self):
        state=self.nodes['fake'];state['runtime']={};state['recovery']={'required':False}
        state['health']=dict(backend_reachable=False,busy=None,maintenance=None,services={'workbench':{'state':'stopped','running':False,'owned':True}})
        stopped=deepcopy(state)
        body=dict(profile_id='physical-profile',expected_target=state['target'])
        code,_,raw=self.request('/api/operations/target?environment=fake',body)
        self.assertEqual(code,200);self.assertFalse(json.loads(raw)['controller_acquired'])
        for change in (
                {'health':{'backend_reachable':True}},
                {'health':{'services':{'workbench':{'state':'running','running':True}}}},
                {'health':{'services':{'workbench':{'state':'unknown','running':False}}}},
                {'health':{'services':{'workbench':{'state':'stopped','running':None}}}},
                {'health':{'services':{'workbench':{'state':'stopped','running':True}}}},
                {'health':{'busy':True}},{'runtime':{'connected':True}},
                {'recovery':{'required':True}},{'recovery':{}},
                {'active_job':{'status':'running'}},{'health':{'maintenance':{'status':'active'}}},
                {'maintenance':{'status':'active'}},{'runtime':{'maintenance_active':True}}):
            with self.subTest(change=change):
                self.nodes['fake']=deepcopy(stopped)
                for key,value in change.items():
                    if key=='health':self.nodes['fake'][key].update(value)
                    else:self.nodes['fake'][key]=value
                self.assertEqual(self.request('/api/operations/target?environment=fake',body)[0],409)
        self.assertEqual(self.registry.read_bytes(),self.original_registry);self.assertEqual(self.events,[],'target switching must not start, stop, release or connect any service')

    def test_legacy_physical_business_binding_keeps_its_identity_fence(self):
        code,_,raw=self.request('/api/state?environment=physical');self.assertEqual(code,200)
        value=json.loads(raw);self.assertEqual(value['gateway_route']['route_id'],'f'*32)
        path=value['gateway_route']['prefix']+'/api/actions/stop?environment=physical'
        self.assertEqual(self.request(path,{'environment':'physical'},environment='physical',headers={'X-CSRF-Token':'physical-business-token'})[0],200)
        self.assertEqual(self.events[-1]['environment'],'physical')
        self.assertEqual(self.request(path.replace('f'*32,'e'*32),{'environment':'physical'},environment='physical')[0],409)

    def test_managed_mode_refuses_static_overrides_and_wrong_device_identity(self):
        for field in ('workbench_assets','physical_assets'):
            with self.assertRaisesRegex(ValueError,'remove'):gateway.serve(manager_profiles=self.profiles,**{field:self.root})
        self.nodes['physical']['target']['device_id']='wrong-device'
        self.assertEqual(self.request('/api/environment?environment=physical')[0],409)
        self.assertEqual(self.events,[])


if __name__=='__main__':unittest.main()
