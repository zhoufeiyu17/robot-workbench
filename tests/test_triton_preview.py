"""Actual HTTP proxy contract with a CPU-only service fixture; SDK never created."""
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import threading
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.error import HTTPError
from urllib.request import Request, urlopen
from unittest.mock import Mock

from robot_workbench.service import Application, Device, serve
from robot_workbench.triton_preview import TritonPreviewClient, validate_config


class TritonPreviewTests(unittest.TestCase):
    def setUp(self):
        self.temp = TemporaryDirectory(); self.calls = []
        self.catalog = dict(sources=[dict(id='source1',display_name='Source',status='AVAILABLE',selectable=True)],
            models=[dict(id='bad',display_name='Failed conversion',status='FAILED',selectable=True,deployment_sha256='a'*64),
                    dict(id='good',display_name='Ready conversion',status='READY',selectable=True,deployment_sha256='b'*64)])
        self.job = dict(job_id='job1',status='SUCCEEDED',result=dict(hardware_control=False,command_publication=False,
            prediction=dict(shape=[60,14],joint_units='rad',gripper_units='m',values=[[0.]*14 for _ in range(60)])))
        owner=self
        class Upstream(BaseHTTPRequestHandler):
            def log_message(self,*args):pass
            def do_GET(self):
                assert self.headers['Authorization']=='Bearer fixture-token'
                data=owner.catalog if self.path=='/v1/catalog' else owner.job
                raw=json.dumps(data).encode();self.send_response(200);self.end_headers();self.wfile.write(raw)
            def do_POST(self):
                assert self.headers['Authorization']=='Bearer fixture-token'
                owner.calls.append((self.path,json.loads(self.rfile.read(int(self.headers['Content-Length'])))))
                self.send_response(202);self.end_headers();self.wfile.write(b'{"job_id":"job1","status":"QUEUED"}')
        self.up=ThreadingHTTPServer(('127.0.0.1',0),Upstream)
        self.threads=[]
        token=Path(self.temp.name)/'token';token.write_text('fixture-token');token.chmod(0o600)
        self.client=TritonPreviewClient(dict(endpoint=f'http://127.0.0.1:{self.up.server_port}',token_file=str(token)))
        self.device=Device('arx13','physical',Mock(side_effect=AssertionError('SDK must remain untouched')))
        self.device.triton_preview=self.client
        self.app=Application({'physical':self.device});self.server=serve(self.app,port=0)
        for server in (self.up,self.server):
            t=threading.Thread(target=server.serve_forever,daemon=True);t.start();self.threads.append(t)
        self.base=f'http://127.0.0.1:{self.server.server_port}'

    def tearDown(self):
        for server in (self.server,self.up):server.shutdown();server.server_close()
        for t in self.threads:t.join(2)
        self.temp.cleanup()

    def get(self,path):return json.load(urlopen(self.base+path,timeout=3))

    def post(self,action,**kwargs):
        body=dict(environment='physical',business='inference',request_id='req1',**kwargs)
        return json.load(urlopen(Request(self.base+'/api/triton/'+action,json.dumps(body).encode(),
            headers={'Content-Type':'application/json','X-CSRF-Token':self.app.csrf}),timeout=3))

    def test_catalog_and_jobs_are_readonly_and_failed_cannot_infer(self):
        view=self.get('/api/triton/catalog?environment=physical')
        self.assertFalse(view['models'][0]['selectable']);self.assertTrue(view['models'][1]['selectable'])
        self.assertNotIn('fixture-token',json.dumps(view))
        with self.assertRaises(HTTPError)as error:self.post('infer',model_id='bad')
        self.assertEqual(error.exception.code,409);self.assertEqual(self.calls,[])
        self.assertEqual(self.get('/api/triton/jobs/job1?environment=physical')['result']['prediction']['shape'],[60,14])
        self.device.factory.assert_not_called();self.assertIsNone(self.device.runtime)

    def test_starts_only_registered_ids_without_motion_and_enforces_csrf(self):
        self.assertEqual(self.post('conversions',source_id='source1')['status'],'QUEUED')
        self.assertEqual(self.post('infer',model_id='good')['job_id'],'job1')
        self.assertEqual([x[0]for x in self.calls],['/v1/conversions','/v1/infer'])
        self.assertEqual(self.calls[-1][1],dict(model_id='good',request_id='req1'))
        for action,body in [('infer',dict(model_id='good',path='/arbitrary')),('conversions',dict(source_id='../bad'))]:
            with self.assertRaises(HTTPError):self.post(action,**body)
        with self.assertRaises(HTTPError)as error:
            urlopen(Request(self.base+'/api/triton/infer',b'{}',headers={'Content-Type':'application/json'}))
        self.assertEqual(error.exception.code,403);self.assertEqual(len(self.calls),2)
        self.device.factory.assert_not_called()

    def test_malformed_preview_private_config_and_maintenance_fail_closed(self):
        self.job['result']['hardware_control']=True
        with self.assertRaises(ValueError):self.client.job('job1')
        self.device.maintenance=True
        with self.assertRaises(HTTPError):self.post('infer',model_id='good')
        self.assertEqual(self.calls,[])
        for endpoint in ('https://example.com','http://127.0.0.1:1234/arbitrary','http://user@127.0.0.1:1234'):
            with self.assertRaises(ValueError):validate_config(dict(endpoint=endpoint,token_file='/private/token'))
        Path(self.client.config['token_file']).chmod(0o644)
        with self.assertRaises(ValueError):self.client.catalog()


if __name__=='__main__':unittest.main()
