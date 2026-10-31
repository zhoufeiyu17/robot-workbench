"""Existing exact-revision display protocol using actual CPU mesh rendering."""
from pathlib import Path
import json,sys,tempfile,time,threading,unittest
from http.client import HTTPConnection
from urllib.parse import urlencode
from types import SimpleNamespace
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'calibration'),str(ROOT/'calibration/infra')]
from robot_workbench.cpu_preview import MeshRenderer
from robot_workbench.target_preview import RendererClient,TargetPreview
from examples.scripts.debug.joint_head_workbench_live import Session,serve


class CpuPreviewTests(unittest.TestCase):
    def test_asymmetric_sides_are_not_mirrored_and_use_neutral_colors(self):
        class Geometry:
            triangle=np.array([[0,-.08,-.08],[0,.08,-.08],[0,0,.08]])
            side_meshes=dict(left=[('link6',triangle,np.array([[0,1,2]]))],right=[('link6',triangle,np.array([[0,1,2]]))])
            static_meshes=[('base',np.array([[-.2,-.4,-.2],[.2,-.4,-.2],[.2,.4,-.2],[-.2,.4,-.2]]),np.array([[0,1,2],[0,2,3]]))]
            def with_gaps(self,*gaps):
                self.gaps=gaps;return self
            def frames(self,q):
                result={}
                for side,index,y in [('left',0,.3),('right',6,-.3)]:
                    transform=np.eye(4);transform[1,3]=y;transform[2,3]=q[index]+self.gaps[index//6]/1000
                    result[side]=dict(link6=transform)
                return result
        renderer=MeshRenderer(Geometry());renderer.view=dict(yaw=0,pitch=0,zoom=1,pan_y=0)
        _,base=renderer.render([0.]*12,[0.,0.])
        self.assertTrue(np.all(base[:,:,0]==base[:,:,1])and np.all(base[:,:,1]==base[:,:,2]))
        self.assertTrue(np.all(base[470,630]==255))
        for index,gap in [(0,False),(6,False),(0,True),(6,True)]:
            q=[0.]*12;gaps=[0.,0.]
            if gap:gaps[index//6]=83.
            else:q[index]=.1
            _,changed=renderer.render(q,gaps);ys,xs=np.where(np.any(changed!=base,axis=2))
            self.assertGreater(len(xs),50)
            self.assertTrue(np.all(xs>320)if index==0 else np.all(xs<320),'left/right input must affect its unmirrored side only')
        renderer.view['top_view']=True
        _,split=renderer.render([0.]*12,[83.,0.]);self.assertEqual(split.shape,(480,1280,3))
        self.assertIs(renderer.render([0.]*12,[83.,0.])[1],split,'an unchanged display reuses its completed image')
        top_image=renderer.top_image
        self.assertGreater(np.count_nonzero(split[40:,642:]<220),50)
        renderer.view['yaw']=.8;_,turned=renderer.render([0.]*12,[83.,0.])
        self.assertIs(renderer.top_image,top_image,'orbiting must not rerender a fixed top camera')
        self.assertTrue(np.array_equal(split[:,642:],turned[:,642:]),'top view stays fixed while the main view orbits')
        self.assertFalse(np.array_equal(split[:,:638],turned[:,:638]))
        renderer.render([.1]+[0.]*11,[83.,0.]);self.assertIsNot(renderer.top_image,top_image,'changing pose invalidates both views')

    def test_target_changes_pixels_and_exact_pose_without_motion_or_collision(self):
        class Geometry:
            velocity_limits=np.ones(12);limits=np.array([[-2.,2.]]*12)
            k=dict(width=640,height=480,fx=520,fy=520,cx=320,cy=240)
            bases={side:np.eye(4)for side in('left','right')}
            triangle=np.array([[0,-.1,0],[0,.1,0],[0,0,.2]])
            side_meshes=dict(left=[('link6',triangle,np.array([[0,1,2]]))],right=[('link6',triangle,np.array([[0,1,2]]))])
            def with_gaps(self,*args):return self
            def frames(self,q):
                T=np.eye(4);T[2,3]=q[0]
                return {side:dict(link6=T)for side in('left','right')}
            def preview(self,*args):raise AssertionError('display must not plan or collide')
        with tempfile.TemporaryDirectory()as temporary:
            root=Path(temporary);token=root/'token';token.write_text('x'*40);token.chmod(0o600)
            geometry=Geometry();session=Session(geometry,root/'session',dict(display_only=True))
            renderer=MeshRenderer(geometry);closed=threading.Event()
            server=serve(session,'x'*40,0)
            def render():
                while not closed.is_set():session.tick(renderer,.02);closed.wait(.01)
            worker=threading.Thread(target=render);worker.start()
            preview=TargetPreview(RendererClient('http://127.0.0.1:'+str(server.server_port),token),session.renderer_instance_id)
            preview.start();preview.bind_context('test')
            try:
                raws=[]
                for value in(0.,.3):
                    result=preview.target_pose([value]+[0.]*11,[70.,70.])
                    end=time.monotonic()+3
                    while time.monotonic()<end and preview.state()['queue']['highest_completed_revision']<result['client_revision']:time.sleep(.01)
                    raw,meta=preview.media('target',result['client_revision'],display_context='test',
                        renderer_instance_id=session.renderer_instance_id,view_revision=0)
                    self.assertEqual(meta['q_reported_rad'][0],value)
                    self.assertEqual(cv2.imdecode(np.frombuffer(raw,np.uint8),1).shape,(480,640,3))
                    raws.append(raw)
                self.assertTrue(raws[0]!=raws[1],'target pose must change rendered pixels');self.assertEqual(session.q.tolist(),[0.]*12)
                self.assertFalse(session.state()['capabilities']['execute'])
                preview.overview(.4,.4,1.,top_view=True)
                end=time.monotonic()+3
                while time.monotonic()<end and not preview.state()['frames'].get('overview',{}).get('top_view'):time.sleep(.01)
                raw,metadata=preview.media('overview');self.assertTrue(metadata['top_view'])
                self.assertEqual(metadata['q_reported_rad'],[.3]+[0.]*11,'both views show the last requested target, not a different zero pose')
                self.assertEqual(cv2.imdecode(np.frombuffer(raw,np.uint8),1).shape,(480,1280,3))
                preview.close()
                from robot_workbench.service import Application,Device,serve as serve_application
                from robot_workbench.management import serve as serve_manager
                descriptor=root/'display.json'
                descriptor.write_text(json.dumps(dict(schema='workbench.cpu_target_display.v1',device_id='cpu',
                    url='http://127.0.0.1:'+str(server.server_port),token_file=str(token),
                    renderer_instance_id=session.renderer_instance_id,joint_limits=geometry.limits.tolist())))
                descriptor.chmod(0o600)
                def forbidden():raise AssertionError('display accessed a controller')
                device=Device('cpu','physical',forbidden);app=Application({'physical':device})
                candidates=[dict(id='saved-wrist',kind='wrist',mesh_selectable=False,mesh_reason='geometry differs',
                    details=dict(environment='hardware',robot_source='physical',simulation_only=False,head_result_id='saved-head'))]
                device.workflows['calibration']=SimpleNamespace(snapshot=lambda:dict(busy=False,status='idle'),
                    results=lambda:[],mesh_candidates=lambda:candidates,mesh_calibrations=lambda:[dict(id='one-complete-group',kind='calibration')])
                backend=serve_application(app,port=0);backend_thread=threading.Thread(target=backend.serve_forever);backend_thread.start()
                manager=SimpleNamespace(config=dict(device_id='cpu',target_display_descriptor=str(descriptor)),
                    port=backend.server_port,deployment=SimpleNamespace(sha256='a'*64),backend_auth=dict(csrf=app.csrf),require_deployment=lambda headers:None)
                gateway=serve_manager(manager,host='127.0.0.1',port=0);gateway_thread=threading.Thread(target=gateway.serve_forever);gateway_thread.start()
                def request(path,body=None,csrf=None):
                    connection=HTTPConnection('127.0.0.1',gateway.server_port)
                    try:
                        headers={'Content-Type':'application/json','Origin':'http://127.0.0.1:'+str(gateway.server_port)}
                        if csrf:headers['X-CSRF-Token']=csrf
                        connection.request('POST'if body else'GET',path,json.dumps(body)if body else None,headers)
                        response=connection.getresponse();raw=response.read()
                        return response.status,raw,dict(response.getheaders())
                    finally:connection.close()
                try:
                    status,raw,_=request('/api/state?environment=physical&business=calibration');self.assertEqual(status,200,raw);state=json.loads(raw)
                    self.assertFalse(state['runtime']['connected']);self.assertTrue(state['preview']['capabilities']['target'])
                    self.assertIn('mesh',state['preview']['sources'])
                    self.assertEqual(state['preview']['mesh_candidates'],candidates,'independent target display must preserve the real mesh catalog')
                    self.assertEqual(state['preview']['mesh_calibrations'],[dict(id='one-complete-group',kind='calibration')])
                    current=state['preview']
                    body=dict(environment='physical',business='calibration',request_id='display-only',q12=[.1]+[0.]*11,jaw_gap_mm=[70.,70.],
                        display_context=current['display_context'],renderer_id=current['renderer_id'],view_revision=0)
                    self.assertEqual(request('/api/actions/preview_target',body,csrf='wrong')[0],403)
                    status,raw,_=request('/api/actions/preview_target',body,csrf=app.csrf);self.assertEqual(status,200,raw)
                    revision=json.loads(raw)['client_revision'];end=time.monotonic()+3
                    url='/api/preview-media?'+urlencode(dict(environment='physical',view='target',revision=revision,
                        display_context=current['display_context'],renderer_id=current['renderer_id'],view_revision=0,wait_ms=100))
                    while time.monotonic()<end:
                        status,raw,headers=request(url)
                        if status==200:break
                    self.assertEqual(status,200,raw)
                    metadata=json.loads(headers['X-Frame-Metadata'])
                    self.assertEqual(metadata['q_reported_rad'][0],.1)
                    self.assertEqual(metadata['server_instance_id'],current['server_instance_id'])
                    self.assertIsNone(device.runtime);self.assertIsNone(device.thread)
                finally:
                    if getattr(manager,'target_display',None):manager.target_display.close()
                    gateway.shutdown();gateway.server_close();gateway_thread.join(2)
                    backend.shutdown();backend.server_close();backend_thread.join(2)
            finally:
                preview.close();closed.set();worker.join(2);server.shutdown();server.server_close()

if __name__=='__main__':unittest.main()
