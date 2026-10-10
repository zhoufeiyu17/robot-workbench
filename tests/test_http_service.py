"""Live loopback HTTP + real shared Owner. Protocol fake only, no SDK/GPU."""
from pathlib import Path
from http.client import HTTPConnection
import json
import socket
import sys
import threading
import time
import unittest
from tempfile import TemporaryDirectory
from urllib.request import Request,build_opener,ProxyHandler
from urllib.error import HTTPError
from unittest.mock import Mock
from unittest.mock import patch
from types import SimpleNamespace

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from robot_workbench.service import Application,Device,protocol_fake,serve,loopback_browser_hosts


class HTTPTest(unittest.TestCase):
    def setUp(self):
        self.temp=TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.device=protocol_fake(Path(self.temp.name))
        self.app=Application({'fake':self.device,'physical':Device('robot','physical',None)})
        self.server=serve(self.app,port=0)
        self.thread=threading.Thread(target=self.server.serve_forever,daemon=True);self.thread.start()
        self.addCleanup(self.cleanup)
        self.url='http://127.0.0.1:'+str(self.server.server_port)
        self.http=build_opener(ProxyHandler({}))
        self.counter=0

    def cleanup(self):
        if self.device.runtime:
            runtime=self.device.runtime
            runtime.owner.submit({'action':'release','confirmation':'ARMS_SUPPORTED_RELEASE'})
            self.device.thread.join(1)
            runtime.close()
        self.device.closed.set()
        self.server.shutdown();self.server.server_close();self.thread.join(1)

    def state(self,business='calibration'):
        return json.loads(self.http.open(self.url+'/api/state?environment=fake&business='+business).read())

    def action(self,action,business='calibration',**body):
        self.counter+=1
        data=dict(environment='fake',business=business,request_id=str(self.counter),**body)
        req=Request(self.url+'/api/actions/'+action,json.dumps(data).encode(),headers={
            'Content-Type':'application/json','Origin':self.url,'X-CSRF-Token':self.app.csrf})
        with self.http.open(req,timeout=2) as response:return response.status,json.loads(response.read())

    def wait(self,predicate):
        deadline=time.monotonic()+3
        while time.monotonic()<deadline:
            state=self.state()
            if predicate(state):return state
            time.sleep(.01)
        self.fail('state did not converge: '+str(state))

    def connect(self):
        code,_=self.action('connect');self.assertEqual(code,202)
        return self.wait(lambda s:s['runtime'].get('stationary'))

    def test_request_failure_preserves_running_task_and_actual_task_failure(self):
        self.connect();self.action('handoff',target_business='collection')
        value=dict(id='current-D-run',status='running',busy=True,stage_id='collection.prepare',
            message='当前环境准备仍在运行',primary_action=dict(id='generate_plan',allowed=False),outcomes={})
        workflow=SimpleNamespace(snapshot=lambda:dict(value),results=lambda:[],
            action=Mock(side_effect=ValueError('重复请求被拒绝')))
        self.device.workflows['collection']=workflow
        self.action('generate_plan',business='collection')
        end=time.monotonic()+2
        while time.monotonic()<end:
            state=self.state('collection')['workflow']
            if state.get('request',{}).get('status')=='failed':break
            time.sleep(.01)
        self.assertEqual(state['request']['error'],'重复请求被拒绝')
        self.assertEqual(state['status'],'running');self.assertTrue(state['busy'])
        self.assertEqual(state['id'],'current-D-run');self.assertEqual(state['message'],value['message'])
        value.update(status='failed',busy=False,message='实际任务失败：场景检测错误')
        state=self.state('collection')['workflow']
        self.assertEqual(state['status'],'failed');self.assertEqual(state['message'],value['message'])
        self.assertEqual(state['request']['error'],'重复请求被拒绝')

    def test_collection_review_generation_blocks_late_publication_and_execution(self):
        self.connect();self.action('handoff',target_business='collection')
        workflow=SimpleNamespace(generation=2,snapshot=lambda:dict(status='ready',busy=False,primary_action=None),results=lambda:[],action=Mock())
        self.device.workflows['collection']=workflow
        owner=self.device.snapshot('collection')
        preview=dict(token='frozen-plan',command_epoch=owner['command_epoch'],workflow_generation=1)
        self.app.add_review(self.device,'collection',preview,'execute','old snapshot')
        self.assertNotIn(('fake','collection'),self.app.reviews)
        preview['workflow_generation']=2
        self.app.add_review(self.device,'collection',preview,'execute','current snapshot')
        review=self.state('collection')['workflow']['review']
        workflow.generation=3
        self.assertNotIn('review',self.state('collection')['workflow'])
        with self.assertRaises(HTTPError) as rejected:
            self.action('execute',business='collection',review_id=review['id'],physical_clearance=True)
        self.assertIn('数采任务已变化',json.loads(rejected.exception.read())['error'])
        workflow.action.assert_not_called()

    def test_collection_execution_speed_is_server_bound_to_the_review(self):
        self.connect();self.action('handoff',target_business='collection')
        workflow=SimpleNamespace(generation=2,snapshot=lambda:dict(status='ready',busy=False,primary_action=None),
            results=lambda:[],action=Mock(return_value={'message':'CPU route fixture accepted'}))
        self.device.workflows['collection']=workflow;owner=self.device.snapshot('collection')
        self.app.add_review(self.device,'collection',dict(token='frozen-plan',command_epoch=owner['command_epoch'],
            workflow_generation=2,speed_scale=1.),'execute','执行速度 100%')
        review=self.state('collection')['workflow']['review'];self.assertEqual(review['speed_scale'],1.)
        for invalid in (.5,True,'1',None):
            with self.assertRaises(HTTPError) as error:
                self.action('execute',business='collection',review_id=review['id'],physical_clearance=True,speed_scale=invalid)
            self.assertIn('速度已变化',json.loads(error.exception.read())['error'])
        workflow.action.assert_not_called()
        self.action('execute',business='collection',review_id=review['id'],physical_clearance=True)
        self.wait(lambda _:workflow.action.called)
        self.assertEqual(workflow.action.call_args.args[1]['speed_scale'],1.)
        self.assertEqual(workflow.action.call_args.args[1]['token'],'frozen-plan')

    def test_declared_browser_port_preserves_Host_Origin_and_CSRF_checks(self):
        self.assertEqual(self.server.allowed_hosts,{f'127.0.0.1:{self.server.server_port}',f'localhost:{self.server.server_port}'})
        self.server.shutdown();self.server.server_close();self.thread.join(1)
        self.server=serve(self.app,port=0,browser_ports=[18867])
        self.thread=threading.Thread(target=self.server.serve_forever,daemon=True);self.thread.start()
        self.url='http://127.0.0.1:'+str(self.server.server_port)
        for host in ('127.0.0.1:18867','localhost:18867',f'127.0.0.1:{self.server.server_port}'):
            with self.http.open(Request(self.url+'/api/state',headers={'Host':host}),timeout=2) as response:
                state=json.loads(response.read());self.assertEqual(response.status,200)
            body=json.dumps(dict(environment='fake',business='calibration',request_id=host)).encode()
            request=Request(self.url+'/api/actions/stop',body,headers={'Content-Type':'application/json',
                'Host':host,'Origin':'http://'+host,'X-CSRF-Token':state['csrf']})
            with self.http.open(request,timeout=2) as response:self.assertEqual(response.status,200)
        undeclared_port=18868 if self.server.server_port!=18868 else 18869
        for host,origin,token in [
                ('evil.example:18867','http://evil.example:18867',self.app.csrf),
                ('0.0.0.0:18867','http://0.0.0.0:18867',self.app.csrf),
                (f'127.0.0.1:{undeclared_port}',f'http://127.0.0.1:{undeclared_port}',self.app.csrf),
                ('127.0.0.1:18867','http://localhost:18867',self.app.csrf),
                ('127.0.0.1:18867','http://127.0.0.1:'+str(self.server.server_port),self.app.csrf),
                ('127.0.0.1:18867','https://127.0.0.1:18867',self.app.csrf),
                ('127.0.0.1:18867','http://127.0.0.1:18867','wrong-token')]:
            request=Request(self.url+'/api/actions/stop',body,headers={'Content-Type':'application/json',
                'Host':host,'Origin':origin,'X-CSRF-Token':token})
            with self.subTest(host=host,origin=origin),self.assertRaises(HTTPError) as rejected:self.http.open(request,timeout=2)
            self.assertEqual(rejected.exception.code,403)
        with self.assertRaises(HTTPError) as rejected:
            self.http.open(Request(self.url+'/api/state',headers={'Host':'evil.example:18867'}),timeout=2)
        self.assertEqual(rejected.exception.code,403);self.assertIsNone(self.device.runtime)

    def test_browser_ports_are_explicit_loopback_only_and_validated_before_binding(self):
        self.assertEqual(loopback_browser_hosts([18867]),{'127.0.0.1:18867','localhost:18867'})
        invalid=(None,{},'18867',[0],[-1],[65536],[True],[18867.0],['18867'],['*'],['evil.example:18867'],['127.0.0.1:18867'])
        with patch('robot_workbench.service.ThreadingHTTPServer') as bind:
            for value in invalid:
                with self.subTest(value=value),self.assertRaisesRegex(ValueError,'browser_ports'):
                    serve(self.app,port=0,browser_ports=value)
            bind.assert_not_called()

    def test_two_Application_instances_reuse_port_with_new_identity_revision_and_CSRF(self):
        first=self.state()
        for _ in range(5):old=self.state()
        self.assertEqual(first['server_instance_id'],old['server_instance_id'])
        self.assertNotEqual(first['server_instance_id'],first['csrf'])
        port=self.server.server_port
        self.server.shutdown();self.server.server_close();self.thread.join(1)
        self.app=Application({'fake':self.device,'physical':Device('robot','physical',None)})
        self.server=serve(self.app,port=port)
        self.thread=threading.Thread(target=self.server.serve_forever,daemon=True);self.thread.start()
        fresh=self.state()
        self.assertEqual(self.server.server_port,port)
        self.assertNotEqual(fresh['server_instance_id'],old['server_instance_id'])
        self.assertLess(fresh['revision'],old['revision'])
        self.assertNotEqual(fresh['csrf'],old['csrf'])
        stale=Request(self.url+'/api/actions/stop',json.dumps(dict(environment='fake',business='calibration',request_id='old-instance')).encode(),
            headers={'Content-Type':'application/json','Origin':self.url,'X-CSRF-Token':old['csrf']})
        with self.assertRaises(HTTPError) as rejected:self.http.open(stale)
        self.assertEqual(rejected.exception.code,403)
        code,_=self.action('stop');self.assertEqual(code,200)
        self.assertIsNone(self.device.runtime,'restart checks must not start an SDK or fake control loop')

    def calibration_adapter(self):
        """Actual C HTTP client/preflight/queue plus shared Owner; geometry is a CPU fixture."""
        import numpy as np
        sys.path[:0]=[str(ROOT/'calibration'),str(ROOT/'calibration/examples/scripts/debug'),str(ROOT/'calibration/tests')]
        from test_joint_head_workbench_live_bridge import BridgeTests
        from robot_workbench.bridges import start_control_gateway
        from robot_workbench.workflows import CalibrationWorkflow
        legacy=BridgeTests();legacy.setUp();self.addCleanup(legacy.doCleanups)
        c=legacy.app;c.output=c.output.resolve();c.queue=[];c.plan=None;c.geometry_id=c.plan_geometry_id='CPU-fixture-geometry'
        c.mesh_rendering_enabled=False;c.workflow={'kind':'head','head_result_id':None};c.review_generation=0
        c.config.update(environment='fake',simulation_only=True,robot_id='fake-http-fixture')
        c.calibration_root=c.output/'calibrations';c.hardware_feedback={'owner':self.device.snapshot('calibration')}
        c.hardware_connected=lambda:self.device.snapshot('calibration').get('feedback_healthy') is True
        def measured():
            q=self.device.snapshot('calibration')['calibration_feedback']['qpos'];return np.array(q[:6]+q[7:13])
        c.current_pose=measured
        c.geometry.path_check=Mock(return_value={'ok':True})
        tokens={'calibration':'c'*40,'collection':'d'*40}
        gateway,thread=start_control_gateway(self.device,tokens)
        def cleanup():gateway.shutdown();gateway.server_close();thread.join(1)
        self.addCleanup(cleanup)
        c.control={'port':gateway.server_port};c.control_token=tokens['calibration'];c.control_http=self.http
        flow=CalibrationWorkflow.__new__(CalibrationWorkflow);flow.app=c;flow.device=self.device;flow.service=self.app
        flow.lock=threading.Lock();flow.install_lock=threading.Lock();flow.active=False;flow.catalog=[]
        flow.value=dict(busy=False,status='idle',primary_action=None,manual_queue=flow.manual_queue_state(),arrival_statistics=c.arrival_state())
        self.device.workflows['calibration']=flow
        return c,flow

    def manual_job(self,action,**body):
        self.action(action,**body)
        value=self.wait(lambda s:(s['workflow'].get('request') or {}).get('status')!='running')
        with self.app.lock:job=dict(self.app.jobs[('fake','calibration')])
        self.assertEqual(job['status'],'completed',job.get('error'))
        return value

    def test_real_http_manual_review_and_explicit_handoff(self):
        self.assertFalse(self.state()['runtime']['connected'])
        state=self.connect()
        self.assertEqual(state['runtime']['motion_status'],'no_automatic_task')
        self.assertIn('calibration_capture_ack_v1',state['runtime']['capabilities'])
        self.assertFalse(self.state('collection')['runtime']['allowed_operations']['preview']['allowed'])
        q=state['manual']['positions'][:];q[0]+=.01
        self.action('manual_preview',q12=q,jaw_gap_mm=[80.,80.])
        ready=self.wait(lambda s:s['workflow'].get('review'))
        self.assertEqual(ready['workflow']['primary_action']['id'],'manual_execute')
        self.assertEqual(ready['workflow']['stage_id'],'manual.review')
        self.assertEqual(ready['workflow']['status'],'awaiting_review')
        self.assertEqual(ready['workflow']['id'],ready['workflow']['review']['id'])
        with self.assertRaises(HTTPError):
            self.action('manual_execute',review_id=ready['workflow']['review']['id'],physical_clearance=False)
        self.action('manual_execute',review_id=ready['workflow']['review']['id'],physical_clearance=True)
        running=self.wait(lambda s:s['workflow']['stage_id']=='manual.execute')
        self.assertEqual(running['workflow']['status'],'executing')
        done=self.wait(lambda s:s['workflow']['stage_id']=='manual.complete')
        self.assertEqual(done['workflow']['status'],'completed')
        self.assertIn('最后一个路径点已收到 READY',done['workflow']['message'])
        self.assertNotIn('正在执行',done['workflow']['message'])
        self.assertNotEqual(done['workflow']['primary_action']['id'],'manual_preview',
                            'disabled task placeholder must not override the separate manual preview gate')
        self.assertEqual(done['runtime']['motion']['plan_sha256'],
                         self.app.manual_tasks[('fake','calibration')]['plan_sha256'][-1])
        self.action('handoff',target_business='collection')
        self.assertEqual(self.state('collection')['runtime']['holder'],'collection')
        self.assertTrue(self.state()['runtime']['connected'])
        self.assertNotIn(('fake','calibration'),self.app.manual_tasks)
        self.assertEqual(self.state()['workflow']['stage_id'],'configuration')
        with self.assertRaises(HTTPError):self.action('manual_preview',q12=q,jaw_gap_mm=[80.,80.])

    def test_C_queue_speeds_persistence_whole_preview_and_stale_review_rejection(self):
        self.connect();c,flow=self.calibration_adapter()
        for index,speed in enumerate((.08,.04)):
            queue=self.state()['manual']['queue']
            self.manual_job('manual_queue',operation='add',queue_revision=queue['revision'],waypoint=dict(
                q=[.002*(index+1)]+[0.]*11,speeds=[speed]*12,jaw_gap_mm=[80.,80.],jaw_speed_mm_s=5.+index))
        self.assertEqual(len(json.loads((c.output/'workbench-state.json').read_text())['queue']),2)
        ready=self.manual_job('manual_queue_preview',queue_revision=self.state()['manual']['queue']['revision'])
        self.assertEqual(ready['workflow']['stage_id'],'manual.review','C idle must not override manual review')
        plans=next(iter(self.device.runtime.owner.pending.values()))[0]
        self.assertEqual(len(plans),2);self.assertEqual([p.max_velocity for p in plans],[.08,.04])
        self.assertEqual([p.jaw_speed_mm_s for p in plans],[5.,6.])
        self.assertGreaterEqual(c.geometry.path_check.call_count,4,'new queue bypassed the original C path checks')
        old=ready['workflow']['review']['id'];revision=ready['manual']['queue']['revision']
        self.manual_job('manual_queue',operation='remove',index=0,queue_revision=revision)
        self.assertNotIn(('fake','calibration'),self.app.manual_tasks)
        with self.assertRaises(HTTPError):self.action('manual_execute',review_id=old,physical_clearance=True)
        ready=self.manual_job('manual_queue_preview',queue_revision=self.state()['manual']['queue']['revision'])
        self.action('manual_execute',review_id=ready['workflow']['review']['id'],physical_clearance=True)
        done=self.wait(lambda s:s['workflow'].get('stage_id')=='manual.complete')
        self.assertAlmostEqual(done['manual']['positions'][0],.004)
        ready=self.manual_job('manual_queue_preview',queue_revision=self.state()['manual']['queue']['revision'])
        self.action('stop')
        self.assertNotIn(('fake','calibration'),self.app.manual_tasks)
        with self.assertRaises(HTTPError):self.action('manual_execute',review_id=ready['workflow']['review']['id'],physical_clearance=True)
        self.assertEqual(len(c.queue),1,'stop must not delete the persisted queue')

    def test_manual_home_targets_speed_validation_permissions_and_original_arrival_store(self):
        self.connect();c,flow=self.calibration_adapter()
        ready=self.manual_job('manual_preview',q12=[0.]*12,speeds12=[.12]*12,jaw_gap_mm=[83.,83.],jaw_speed_mm_s=7.)
        plan=next(iter(self.device.runtime.owner.pending.values()))[0][0]
        self.assertEqual(plan.max_velocity,.12);self.assertEqual(plan.jaw_speed_mm_s,7.)
        self.action('manual_execute',review_id=ready['workflow']['review']['id'],physical_clearance=True)
        self.wait(lambda s:s['runtime'].get('motion_status')=='READY')
        self.action('manual_preview',q12=[0.]*12,speeds12=[0.]*12,jaw_gap_mm=[83.,83.],jaw_speed_mm_s=7.)
        failed=self.wait(lambda s:(s['workflow'].get('request') or {}).get('status')=='failed')
        self.assertFalse(failed['workflow'].get('review'))
        event=dict(event_id='CPU-endpoint-fixture',owner_pid=11,command_epoch=2,status='success',timestamp_ns=time.time_ns(),
            plan_sha256='fixture-plan',q_target_rad=[0.]*12,q_actual_rad=[.001]*12)
        pending=dict(preview={'owner_pid':11,'command_epoch':2,'plans':[{'sha256':'fixture-plan'}]},workflow={'kind':'head'})
        c.record_arrivals({'arrival_events':[event]},pending,c.output/'batch/capture','simulated')
        flow.value['arrival_statistics']=c.arrival_state()
        report=self.state()['arrival_statistics']
        self.assertEqual(report['total_count'],1);self.assertEqual(report['joints']['left'][0]['mean_rad'],.001)
        self.assertEqual(Path(report['raw_path']),c.calibration_root/'diagnostics/arrival-errors/simulated/samples.jsonl')
        self.action('handoff',target_business='collection')
        with self.assertRaises(HTTPError):self.action('manual_queue',operation='clear',queue_revision=c.review_generation)
        self.assertFalse(self.state()['runtime']['allowed_operations']['manual_queue']['allowed'])

    def test_L1_without_C_reports_unavailable_queue_and_statistics(self):
        state=self.connect()
        self.assertFalse(state['manual']['queue']['available'])
        self.assertIn('未配置',state['arrival_statistics']['error'])
        self.assertNotIn('joints',state['arrival_statistics'])
        self.assertFalse(state['runtime']['allowed_operations']['manual_queue_preview']['allowed'])

    def test_independent_capture_calls_real_C_and_persists_before_shared_Owner_ACK(self):
        self.connect();c,flow=self.calibration_adapter()
        c.frames=[];c.selected_batch=None;c.evaluation=None;c.batch=None;c.calibration_previews={}
        c.store=SimpleNamespace(robot_id='fake-http-fixture');c.hardware_history=[]
        c.control_available=lambda:True
        c.control_status={'owner':self.device.snapshot('calibration')}
        intrinsics={'head':{'serial':'head-fixture','rectified':True,'coefficients':[0.]*5}}
        cameras={'head':SimpleNamespace(read_calibration_frame=lambda:(b'fixture',{'serial':'head-fixture'}))}
        c.hardware_calibration_cameras=lambda:(cameras,intrinsics)
        c.geometry.auto_plan=Mock(return_value={'plan_complete':True,'waypoints':[
            dict(id='sample-one',q=[.003]+[0.]*11,arm='left',kind='fit',cell=[0,0],capture=True),
            dict(id='return',q=[0.]*12,arm='left',kind='transition',cell=[0,0],capture=False)]})
        c.state=lambda:{'qpos':self.state()['manual']['positions']}
        self.manual_job('plan_batch')
        deadline=time.monotonic()+2
        while c.status=='PLANNING' and time.monotonic()<deadline:time.sleep(.01)
        self.assertEqual(c.status,'READY',c.reason)
        self.assertEqual(len(c.queue),2)
        ready=self.manual_job('preview_batch',queue_revision=c.review_generation)
        pending=next(iter(self.device.runtime.owner.pending.values()))
        self.assertIsNotNone(pending[-1][0],'capture sample identity must reach the shared Owner')
        self.assertIsNone(pending[-1][1],'transition waypoint must not capture')
        self.assertEqual(ready['workflow']['primary_action']['id'],'execute')
        calls=[];original=c.control_request
        def gateway(action,body=None,**kw):
            if action=='capture-ack':
                dataset=next(c.output.glob('batch-hardware-*/capture'))
                manifest=json.loads((dataset/'dataset.json').read_text());row=manifest['frames'][0]
                self.assertTrue((dataset/row['image']).is_file())
                self.assertTrue((dataset/(body['sample_id']+'.json')).is_file())
                self.assertTrue((dataset/'camera.json').is_file())
                calls.append(body)
            return original(action,body,**kw)
        c.control_request=gateway
        def sample(*args,**kwargs):
            self.assertEqual(kwargs['environment'],'fake')
            q=self.device.snapshot('calibration')['calibration_feedback']['qpos']
            return dict(row={'camera_detections':{'head':{'24':[[1,2]]*4}}},
                rows={'head':dict(qpos=q,q_reported12=q[:6]+q[7:13],bracket_gap_s=.02,pose_span_rad=[0.]*12,metadata={'serial':'head-fixture'})},
                images={'head':b'CPU-image-fixture'},raw_images={},cameras=intrinsics)
        with patch('arx_env.joint_head_workbench_capture.capture_sample',side_effect=sample):
            self.action('execute',review_id=ready['workflow']['review']['id'],physical_clearance=True)
            deadline=time.monotonic()+5
            while time.monotonic()<deadline:
                if c.selected_batch and c.status in ('READY','REJECTED'):break
                time.sleep(.01)
        self.assertEqual(c.status,'READY',c.reason)
        self.assertEqual(len(calls),1)
        self.assertEqual(self.device.snapshot('calibration')['last_capture_ack']['sample_id'],calls[0]['sample_id'])
        completion=json.loads((c.dataset/'completion.json').read_text())
        self.assertTrue(completion['passed']);self.assertEqual(completion['captured'],1)
        source=c.selected_batch
        self.manual_job('fit_source',batch_id=None);self.assertIsNone(c.selected_batch)
        self.manual_job('fit_source',batch_id=source);self.assertEqual(c.selected_batch,source)
        fit_entered=threading.Event();c.fit_head_batch=lambda:fit_entered.set()
        self.manual_job('fit_batch',batch_id=source);self.assertTrue(fit_entered.wait(1))

    def test_manual_stage_rejects_old_runner_restart_and_new_business_task(self):
        self.connect();c,flow=self.calibration_adapter()
        ready=self.manual_job('manual_preview',q12=[.003]+[0.]*11,jaw_gap_mm=[80.,80.])
        task=dict(self.app.manual_tasks[('fake','calibration')],submitted_monotonic_s=time.monotonic())
        runtime=self.device.snapshot('calibration')
        runtime.update(motion=dict(status='READY',plan_sha256=task['plan_sha256'][-1]),
            execution_calibration=dict(plan_sha256=task['plan_sha256'],command_epoch=task['epoch'],
                accepted_monotonic_s=task['submitted_monotonic_s']-1))
        value=self.app.manual_state(task,runtime,None,False)
        self.assertEqual(value['stage_id'],'manual.confirmation','old READY cannot complete a fresh request')
        runtime['execution_calibration']['accepted_monotonic_s']=task['submitted_monotonic_s']+1
        self.assertEqual(self.app.manual_state(task,runtime,None,False)['stage_id'],'manual.complete')
        runtime['runtime_nonce']='restarted-runtime'
        self.assertIsNone(self.app.manual_state(task,runtime,None,False))
        self.assertIsNone(self.app.manual_state(task,self.device.snapshot('calibration'),None,True))
        flow.action=Mock(return_value={'message':'new calibration task'})
        self.manual_job('generate_plan')
        self.assertNotIn(('fake','calibration'),self.app.manual_tasks)
        self.assertFalse(self.state()['workflow'].get('review'))

    def test_stop_preempts_background_work_and_revokes_review(self):
        self.connect();entered,release=threading.Event(),threading.Event()
        class Workflow:
            def snapshot(self):return dict(status='idle',busy=False,primary_action=None)
            def results(self):return []
            def action(self,*args):entered.set();release.wait(2);return {'message':'finished'}
            def cancel(self):release.set()
            def invalidate(self):pass
        self.device.workflows['calibration']=Workflow()
        self.action('generate_plan');self.assertTrue(entered.wait(.5))
        start=time.monotonic();code,result=self.action('stop')
        self.assertLess(time.monotonic()-start,.2)
        self.assertEqual(code,200)
        self.assertIn('stop',result)
        self.assertEqual(result['stop']['firmware_ack'],'unsupported')
        self.wait(lambda s:s['runtime'].get('stop_receipt',{}).get('stop_hold_return_confirmed'))

    def test_result_inspection_is_available_without_acquiring_controller(self):
        calls=[]
        class Workflow:
            def action(self,action,body):calls.append(action);return {'message':'read-only view updated'}
        self.device.workflows['calibration']=Workflow()
        for name in ('select_result','mesh'):
            self.action(name,result_id='poor-candidate',enabled=True)
            deadline=time.monotonic()+1
            while time.monotonic()<deadline and len(calls)<1:time.sleep(.005)
            self.assertIn(name,calls)
            with self.app.lock:job=self.app.jobs[('fake','calibration')]
            while job['status']=='running' and time.monotonic()<deadline:time.sleep(.005)
            self.assertEqual(job['status'],'completed')
            calls.clear()
        self.assertIsNone(self.device.runtime)

    def test_diagnostics_separate_latched_controller_fault_from_service_thread_error(self):
        owner=self.device.snapshot('calibration')
        owner.update(state='FAULT_STOP_UNCONFIRMED',reason='SDK feedback age 0.1996s exceeds bound',feedback_healthy=True)
        with patch.object(self.device,'snapshot',return_value=owner):
            rows={row['label']:row['value'] for row in self.state()['diagnostics']}
            self.assertEqual(rows['控制器状态'],'FAULT_STOP_UNCONFIRMED')
            self.assertEqual(rows['控制器当前原因'],owner['reason'])
            self.assertEqual(rows['服务线程异常'],'未报告服务线程异常')
            self.assertNotIn('控制循环',rows)
            self.device.error='CPU service thread failure'
            rows={row['label']:row['value'] for row in self.state()['diagnostics']}
            self.assertEqual(rows['服务线程异常'],'CPU service thread failure')
            self.assertEqual(rows['控制器当前原因'],owner['reason'])

    def test_result_media_streams_ranges_without_exposing_paths(self):
        path=Path(self.temp.name)/'result.mp4';path.write_bytes(b'0123456789')
        class Workflow:
            def media(self, ident):return path if ident=='current-video' else None
        self.device.workflows['collection']=Workflow()
        url=self.url+'/api/result-media?environment=fake&business=collection&result_id='
        with self.http.open(Request(url+'current-video',headers={'Range':'bytes=2-5'})) as response:
            self.assertEqual(response.status,206)
            self.assertEqual(response.headers['Content-Range'],'bytes 2-5/10')
            self.assertEqual(response.read(),b'2345')
        for ident, headers, code in [('current-video',{'Range':'bytes=20-'},416),('../result.mp4',{},404)]:
            with self.assertRaises(HTTPError) as error:self.http.open(Request(url+ident,headers=headers))
            self.assertEqual(error.exception.code,code)
        self.assertFalse(self.device.runtime)

    def test_result_media_JSON_is_limited_to_the_registered_current_collection_plan(self):
        from robot_workbench.workflows import CollectionWorkflow
        root=Path(self.temp.name);path=root/'plan.json';payload={'timeline_digest':'current-plan','ticks':[]}
        path.write_text(json.dumps(payload));config=root/'config.json';config.write_text('{"not_a_plan":true}')
        state={'plan':{'validated':True,'timeline_digest':'current-plan','plan_path':str(path)},'can_execute':False}
        flow=CollectionWorkflow(SimpleNamespace(directory=root,snapshot=lambda:state),self.device,self.app)
        flow.closed.set();flow.thread.join(1);flow.refresh()
        self.device.workflows['collection']=flow
        trajectory=flow.snapshot()['trajectory'];url=self.url+trajectory['url']
        with self.http.open(url) as response:
            self.assertEqual(response.status,200);self.assertEqual(response.headers['Content-Type'],'application/json')
            self.assertEqual(json.loads(response.read()),payload)
        with flow.lock:flow.media_files['registered-config']=config
        for suffix,code in [('registered-config',409),('../config.json',404),('unknown-plan',404)]:
            with self.assertRaises(HTTPError) as error:self.http.open(url.replace(trajectory['id'],suffix))
            self.assertEqual(error.exception.code,code)
        self.device.workflows['calibration']=flow
        with self.assertRaises(HTTPError) as error:self.http.open(url.replace('business=collection','business=calibration'))
        self.assertEqual(error.exception.code,409)
        state['plan']={};flow.refresh()
        with self.assertRaises(HTTPError) as error:self.http.open(url)
        self.assertEqual(error.exception.code,404);self.assertIsNone(self.device.runtime)

    def test_calibration_JSON_export_uses_original_version_store_without_connect(self):
        sys.path[:0]=[str(ROOT/'calibration'),str(ROOT/'calibration/infra'),str(ROOT/'calibration/examples/scripts/debug'),str(ROOT/'calibration/tests')]
        from test_joint_head_workbench_library import LibraryTests
        from robot_workbench.workflows import CalibrationWorkflow
        legacy=LibraryTests();legacy.setUp();self.addCleanup(legacy.doCleanups)
        head,wrist=legacy.pair()
        legacy.app.action('/api/calibration-install',{'head_result_id':head['id'],'wrist_result_id':wrist['id']})
        flow=CalibrationWorkflow.__new__(CalibrationWorkflow);flow.lock=threading.Lock();flow.app=legacy.app
        flow.catalog=[dict(id=head['id'],kind='head'),dict(id=wrist['id'],kind='wrist')]
        self.device.workflows['calibration']=flow
        path=self.url+'/api/calibration-file?environment=fake&result_id='
        with self.http.open(path+head['id']) as response:
            self.assertIn('attachment',response.headers['Content-Disposition'])
            self.assertEqual(json.loads(response.read()),legacy.app.calibration_store_record(head['id']))
        with self.assertRaises(HTTPError):self.http.open(path+'../active/head.json')
        with self.assertRaises(HTTPError):self.http.open(path.replace('environment=fake','environment=physical')+head['id'])
        self.assertIsNone(self.device.runtime)

    def test_delete_preview_confirm_through_HTTP_uses_legacy_token_without_device_connection(self):
        sys.path[:0]=[str(ROOT/'calibration'),str(ROOT/'calibration/infra'),str(ROOT/'calibration/examples/scripts/debug'),str(ROOT/'calibration/tests')]
        from test_joint_head_workbench_library import LibraryTests
        from robot_workbench.workflows import CalibrationWorkflow
        legacy=LibraryTests();legacy.setUp();self.addCleanup(legacy.doCleanups);path=legacy.batch('delete-via-http')
        c=legacy.app;c.live=False;c.mesh_rendering_enabled=False
        flow=CalibrationWorkflow.__new__(CalibrationWorkflow);flow.lock=threading.Lock();flow.app=c
        flow.device=self.device;flow.service=self.app;flow.value={'busy':False,'status':'idle','primary_action':None}
        flow.catalog=[{'id':'capture:delete-via-http','kind':'capture'}];flow.replay_context=None;flow.replay=[];flow.media_files={}
        self.device.workflows['calibration']=flow
        ready=self.manual_job('delete_preview',scope='batch',result_id='capture:delete-via-http')
        preview=ready['workflow']['deletion_preview'];self.assertEqual(preview['image_files'],2)
        self.assertIn(preview['id'],c.delete_tokens,'UI identity must wrap the actual legacy deletion token')
        self.assertTrue(path.exists());self.assertIsNone(self.device.runtime)
        self.action('delete_confirm',delete_preview_id=preview['id'],confirmed=False)
        self.wait(lambda s:(s['workflow'].get('request') or {}).get('status')=='failed')
        self.assertTrue(path.exists())
        result=self.manual_job('delete_confirm',delete_preview_id=preview['id'],confirmed=True)
        self.assertFalse(path.exists());self.assertIsNone(result['workflow']['deletion_preview'])
        self.assertEqual(result['workflow']['deletion_result']['removed'],['delete-via-http'])
        self.assertEqual(result['results'],[]);self.assertIsNone(self.device.runtime)

    def test_csrf_namespace_and_frame_identity(self):
        with self.assertRaises(HTTPError) as error:
            self.http.open(Request(self.url+'/api/actions/stop',b'{}',headers={'Content-Type':'application/json'}))
        self.assertEqual(error.exception.code,403)
        with self.assertRaises(HTTPError):
            self.http.open(self.url+'/api/state?environment=unknown&business=calibration')
        class Media:
            def frame(self):return b'jpeg-fixture',dict(etag='"1"',frame_id='one',source_wall_ns=123)
            def snapshot(self):return dict(id='head',url='/api/media/head?environment=fake')
        self.device.media['head']=Media()
        with self.http.open(self.url+'/api/media/head?environment=fake') as reply:
            self.assertEqual(reply.headers['X-Source-Wall-Ns'],'123')
        with self.assertRaises(HTTPError) as error:
            self.http.open(Request(self.url+'/api/media/head?environment=fake',headers={'If-None-Match':'"1"'}))
        self.assertEqual(error.exception.code,304)
        self.assertFalse(self.state()['runtime']['connected'])


class HTTPTransportTest(unittest.TestCase):
    """Real TCP connections, without starting a runtime or hardware worker."""
    def setUp(self):
        self.device=Device('transport-cpu','fake',None)
        self.app=Application({'fake':self.device});self.server=serve(self.app,port=0)
        self.thread=threading.Thread(target=self.server.serve_forever,daemon=True);self.thread.start()
        self.addCleanup(lambda:(self.server.shutdown(),self.server.server_close(),self.thread.join(1)))
        self.host='127.0.0.1:'+str(self.server.server_port)

    def test_same_socket_handles_state_post_304_and_fully_consumed_error(self):
        connection=HTTPConnection('127.0.0.1',self.server.server_port,timeout=2);self.addCleanup(connection.close)
        setup=self.server.RequestHandlerClass.setup;settings=[]
        def inspect_setup(request):
            setup(request);settings.append((request.connection.gettimeout(),request.connection.getsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY)))
        with patch.object(self.server.RequestHandlerClass,'setup',inspect_setup):
            connection.request('GET','/api/state');response=connection.getresponse();state=json.loads(response.read())
        self.assertEqual(response.version,11);self.assertEqual(response.status,200);self.assertEqual(len(settings),1)
        self.assertEqual(settings[0][0],5.);self.assertTrue(settings[0][1],'TCP_NODELAY must be enabled on the accepted server socket')
        original=connection.sock
        headers={'Origin':'http://'+self.host,'X-CSRF-Token':state['csrf'],'Content-Type':'application/json'}
        body=json.dumps(dict(environment='fake',business='calibration',request_id='keep-alive-stop'))
        connection.request('POST','/api/actions/stop',body,headers);response=connection.getresponse();response.read()
        self.assertEqual(response.status,200);self.assertIs(connection.sock,original)
        self.device.media['head']=SimpleNamespace(frame=lambda:(b'jpeg',dict(etag='"frame"',frame_id='one',source_wall_ns=123)),snapshot=lambda:{})
        connection.request('GET','/api/media/head',headers={'If-None-Match':'"frame"'});response=connection.getresponse()
        self.assertEqual(response.status,304);self.assertEqual(response.read(),b'');self.assertIs(connection.sock,original)
        connection.request('POST','/api/actions/stop','{',headers);response=connection.getresponse();response.read()
        self.assertEqual(response.status,409);self.assertNotEqual(response.getheader('Connection'),'close')
        connection.request('GET','/api/state');response=connection.getresponse();response.read()
        self.assertEqual(response.status,200);self.assertIs(connection.sock,original)

    def test_unread_or_ambiguous_body_closes_without_running_embedded_request(self):
        self.app.state=Mock(wraps=self.app.state)
        poison=f'GET /api/state HTTP/1.1\r\nHost: {self.host}\r\n\r\n'.encode()
        auth=[('Origin','http://'+self.host),('X-CSRF-Token',self.app.csrf)]
        cases=[
            ('POST','/api/actions/stop',[],[('Content-Length',str(len(poison)))],403),
            ('POST','/not-an-action',auth,[('Content-Length',str(len(poison)))],404),
            ('POST','/api/actions/stop',auth,[],409),
            ('POST','/api/actions/stop',auth,[('Content-Length','-1')],409),
            ('POST','/api/actions/stop',auth,[('Content-Length',str(4*1024*1024+1))],413),
            ('POST','/api/actions/stop',auth,[('Content-Length','2'),('Content-Length',str(len(poison)))],409),
            ('POST','/api/actions/stop',auth,[('Content-Length',str(len(poison))),('Transfer-Encoding','chunked')],409),
            ('GET','/api/state',[],[('Content-Length',str(len(poison)))],409),
            ('TRACE','/api/state',[],[('Content-Length',str(len(poison)))],501),
        ]
        for method,path,authorization,framing,status in cases:
            with self.subTest(method=method,path=path,framing=framing),socket.create_connection(('127.0.0.1',self.server.server_port),timeout=2) as client:
                headers=[('Host',self.host),*authorization,*framing]
                request=f'{method} {path} HTTP/1.1\r\n'+''.join(f'{key}: {value}\r\n' for key,value in headers)+'\r\n'
                client.sendall(request.encode()+poison+poison);chunks=[]
                while True:
                    try:data=client.recv(65536)
                    except ConnectionResetError:break
                    if not data:break
                    chunks.append(data)
                reply=b''.join(chunks)
                self.assertTrue(reply.startswith(f'HTTP/1.1 {status} '.encode()),reply[:80])
                self.assertIn(b'connection: close\r\n',reply.lower());self.assertEqual(reply.count(b'HTTP/1.1 '),1)
        self.app.state.assert_not_called()

    def test_idle_keep_alive_connection_expires(self):
        self.assertEqual(self.server.RequestHandlerClass.timeout,5)
        self.server.RequestHandlerClass.timeout=.08
        connection=HTTPConnection('127.0.0.1',self.server.server_port,timeout=2);self.addCleanup(connection.close)
        connection.request('GET','/api/state');response=connection.getresponse();response.read()
        self.assertEqual(response.status,200);self.assertEqual(connection.sock.recv(1),b'')

    def test_incomplete_declared_body_closes_without_dispatch(self):
        self.app.action=Mock(wraps=self.app.action)
        with socket.create_connection(('127.0.0.1',self.server.server_port),timeout=2) as client:
            client.sendall((f'POST /api/actions/stop HTTP/1.1\r\nHost: {self.host}\r\nOrigin: http://{self.host}\r\n'
                f'X-CSRF-Token: {self.app.csrf}\r\nContent-Length: 10\r\n\r\n{{}}').encode())
            client.shutdown(socket.SHUT_WR);chunks=[]
            while True:
                data=client.recv(65536)
                if not data:break
                chunks.append(data)
            reply=b''.join(chunks)
        self.assertTrue(reply.startswith(b'HTTP/1.1 409 '));self.assertIn(b'connection: close\r\n',reply.lower())
        self.app.action.assert_not_called()


if __name__=='__main__':unittest.main()
