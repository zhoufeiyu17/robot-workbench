"""Actual loopback HTTP and CPU subprocess fixtures; no SDK, model or Isaac runs."""
from copy import deepcopy
import base64
from http.client import HTTPConnection
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import secrets
import socket
import struct
import subprocess
import sys
from tempfile import TemporaryDirectory
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock,patch

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from robot_workbench.management import WorkbenchManager, serve, json_request, private_save
from robot_workbench.machine_profile import Deployment


def free_port():
    with socket.socket() as sock:
        sock.bind(('127.0.0.1',0));return sock.getsockname()[1]


def cpu_fixture(kind,port,environment,model_port):
    """A test-owned protocol endpoint, not a physical/fake robot runtime."""
    instance=secrets.token_hex(12)
    value=dict(connected=False,reset_state=False,state_delay=0.,camera_delay=0.,stale_frames=False,
        lose_begin=False,release_reset=False,end_fail=False,loaded=False,warmup=False,wrong_pid=False,
        maintenance=None,events=[],requests=0,model_generation=0,camera_instance='cpu-cameras',begin_cache={},
        media_missing={},media_attempts={},invalid_image=False,
        camera_stopped=False,connect_delay=0.,connect_partial=False,stop_requested=False,
        inspect_runtime_same=True,release_unconfirmed=False,hold_invalid=False,recoverable_fault=None,
        model_prepare_failure=False,stderr_tail='',selected_model_id='cpu-test-model',bundle_digest='a'*64,call_results={},lose_action=None)
    csrf='backend-cpu-'+instance
    def target():return dict(server_instance_id=instance,device_id='cpu-device',runtime_nonce='cpu-runtime' if value['connected'] else None)
    def runtime():return dict(connected=value['connected'],pid=os.getpid(),holder='inference',
        state=value.get('nonrecoverable_fault') or ('FAULT' if value['recoverable_fault'] else 'READY' if value['connected'] else 'DISCONNECTED'),
        automatic_enabled=False,feedback_healthy=value['connected'],stationary=value['connected'],sample_age_s=.01,
        runtime_nonce=target()['runtime_nonce'],release_receipt=value.get('release_receipt'),recoverable_fault=value['recoverable_fault'])
    class Handler(BaseHTTPRequestHandler):
        protocol_version='HTTP/1.1'
        def log_message(self,*args):pass
        def send(self,code,body,headers=None):
            call=getattr(self,'maintenance_call',None)
            if call:
                action,request=call
                canonical={'release':'release_control','ops_maintenance_end':'ops_maintenance_end'}.get(action,action)
                value['call_results'][request['request_id']]=dict(action=canonical,body=request,
                    status='completed' if code==200 else 'failed',response=body,error=body.get('error') if code!=200 else None)
                if value['lose_action']==action:
                    value['lose_action']=None;self.connection.close();self.close_connection=True;return
            raw=json.dumps(body).encode() if isinstance(body,dict) else body
            self.send_response(code);self.send_header('Content-Length',str(len(raw)));self.send_header('Connection','close')
            for key,item in (headers or {'Content-Type':'application/json'}).items():self.send_header(key,item)
            self.end_headers();self.close_connection=True
            try:
                if self.command!='HEAD':self.wfile.write(raw)
            except (BrokenPipeError,ConnectionResetError):pass
        def do_GET(self):
            value['requests']+=1
            if self.path=='/fixture':return self.send(200,{key:item for key,item in value.items() if key not in ('begin_cache','call_results')})
            if self.path=='/fixture/exit-zero':
                self.send(200,{'exiting':True});threading.Thread(target=server.shutdown,daemon=True).start();return
            if self.path=='/fixture/reset':self.connection.close();self.close_connection=True;return
            if self.path=='/fixture/big':return self.send(200,b'x'*(2*1024*1024),{'Content-Type':'application/octet-stream'})
            if self.path=='/status':
                return self.send(200,dict(service_pid=os.getpid()+(1 if value['wrong_pid'] else 0),worker_pid=os.getpid() if value['loaded'] else None,model_id=value['selected_model_id'],
                    loaded=value['loaded'],status='ready' if value['loaded'] else 'error' if value['stderr_tail'] else 'not_loaded',
                    warmup=dict(completed=value['warmup'],input='CPU fixture only; no model execution'),stderr_tail=value['stderr_tail']))
            if self.path.startswith(('/api/state','/api/environment')):
                if value['reset_state']:self.connection.close();self.close_connection=True;return
                time.sleep(value['state_delay'])
                public={key:item for key,item in (value['maintenance'] or {}).items() if key not in ('token','body')}
                state=dict(schema='robot-workbench-ui.v1',server_instance_id=instance,csrf=csrf,
                    device=dict(id='cpu-device',environment=environment),runtime=runtime(),
                    workflow=dict(busy=False,inference=dict(generation=value['model_generation'],selected_model_id=value['selected_model_id'],
                        bundle_digest=value['bundle_digest'],model=dict(id=value['selected_model_id'],loaded=value['loaded']))),
                    maintenance=public or None,media=[])
                if self.path.startswith('/api/environment'):
                    state.update(schema='robot_workbench.environment.v1',activity={'busy':False},
                        model=dict(id=value['selected_model_id'],selected_model_id=value['selected_model_id'],bundle_digest=value['bundle_digest'],
                            generation=value['model_generation'],loaded=value['loaded'],model_ready=value['loaded'],control_ready=False),
                        collection={'ready':value.get('collection_ready',False),'services':value.get('collection_services',{}),
                            'installation_identity':{'manifest_sha256':'cpu-fixture'} if value.get('collection_services') else None},cameras=[])
                    state.pop('workflow');state.pop('media')
                return self.send(200,state)
            if self.path.startswith('/api/media/'):
                if value['camera_stopped']:return self.send(503,{'error':'camera source stopped'})
                role=self.path.split('/')[3].split('?')[0]
                value['media_attempts'][role]=value['media_attempts'].get(role,0)+1
                if value['media_attempts'][role]<=value['media_missing'].get(role,0):
                    return self.send(503,{'raw_ready':True,'error':'preview image not encoded yet'})
                stamp=time.time_ns()-(10_000_000_000 if value['stale_frames'] else 0)
                image=base64.b64decode('iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/x8AAwMCAO+aB5kAAAAASUVORK5CYII=')
                if value['invalid_image']:image=b'not-an-image'
                return self.send(200,image,{'Content-Type':'image/png','X-Frame-Id':str(stamp),
                    'X-Source-Wall-Ns':str(stamp),'ETag':'"fixture-frame"'})
            if self.path.startswith('/api/result-media'):
                headers={'Content-Type':'video/mp4','Accept-Ranges':'bytes','ETag':'"fixture-video"','X-Frame-Metadata':'{"fixture":true}'}
                if self.headers.get('Range')=='bytes=2-5':headers['Content-Range']='bytes 2-5/10';return self.send(206,b'2345',headers)
                return self.send(200,b'0123456789',headers)
            return self.send(404,{'error':'fixture route missing'})
        do_HEAD=do_GET
        def do_POST(self):
            value['requests']+=1
            body=json.loads(self.rfile.read(int(self.headers['Content-Length'])))
            if self.path=='/fixture':value.update(body);return self.send(200,{'updated':True})
            if self.path=='/warm':value.update(loaded=True,warmup=True);value['events'].append('warm');return self.send(200,{'ready':True})
            if (self.headers.get('Host')!='127.0.0.1:'+str(port) or self.headers.get('Origin')!='http://127.0.0.1:'+str(port)
                    or self.headers.get('X-CSRF-Token')!=csrf):return self.send(403,{'error':'fixture backend origin or token mismatch'})
            action=self.path.rsplit('/',1)[-1]
            if self.path.startswith('/api/maintenance/'):
                if 'business' in body:return self.send(409,{'error':'management must not impersonate business'})
                expected=body.get('expected_target') or {}
                if expected.get('server_instance_id')!=instance or expected.get('device_id')!='cpu-device':return self.send(409,{'error':'target changed'})
                action={'begin':'ops_maintenance_begin','end':'ops_maintenance_end','release_control':'release',
                    'start_cameras':'ops_start_cameras','restart_cameras':'ops_restart_cameras',
                    'rebind_cameras':'ops_rebind_cameras','prepare_model':'ops_prepare_model'}.get(action,action)
                if action=='receipt':
                    prior=value['call_results'].get(body['lookup_request_id'])
                    if not prior:return self.send(200,dict(status='running',request_id=body['lookup_request_id'],target=target()))
                    if prior['body'].get('maintenance_token')!=body.get('maintenance_token'):return self.send(409,{'error':'wrong receipt token'})
                    return self.send(200,dict({k:v for k,v in prior.items() if k!='body'},request_id=body['lookup_request_id'],target=target()))
                if action not in ('stop','inspect') and expected!=target() and not (
                        action=='ops_maintenance_begin' and body['request_id'] in value['begin_cache']):
                    return self.send(409,{'error':'runtime nonce changed'})
            value['events'].append(action)
            if self.path.startswith('/api/maintenance/') and action not in ('ops_maintenance_begin','inspect','stop'):
                self.maintenance_call=(action,deepcopy(body))
            if action=='stop':
                value['stop_requested']=True
                return self.send(200,{'stopped':True,'origin':self.headers['Origin'],'host':self.headers['Host']})
            if action=='ops_maintenance_begin':
                if body['request_id'] in value['begin_cache']:return self.send(200,value['begin_cache'][body['request_id']])
                if value['maintenance']:return self.send(409,{'error':'existing fixture maintenance'})
                value['maintenance']=dict(token='private-maintenance-'+instance,purpose=body['purpose'],status='ready',active_action=None,body=body,target=target())
                value['stop_requested']=False
                result=dict(token=value['maintenance']['token'],status='ready')
                value['begin_cache'][body['request_id']]=result
                if value.get('recovery_changes_after_begin'):
                    value['recoverable_fault']=None;value['nonrecoverable_fault']='FAULT_STOP_UNCONFIRMED'
                if value['lose_begin']:value['lose_begin']=False;self.connection.close();self.close_connection=True;return
                return self.send(200,result)
            window=value['maintenance']
            if not window or body.get('maintenance_token')!=window['token']:return self.send(409,{'error':'fixture maintenance token mismatch'})
            if action=='inspect':
                return self.send(200,dict(target=target(),runtime_same=value['inspect_runtime_same'] and target()==window['target'],
                    active_action=window['active_action'],status=window['status'],phase='fixture',error=None))
            if action=='ops_maintenance_end':
                if window.get('active_action') or value['end_fail']:return self.send(409,{'error':'fixture maintenance still active'})
                value['maintenance']=None;return self.send(200,{'ended':True,'motion_resumed':False})
            if action=='release':
                if environment=='physical' and body.get('arms_supported') is not True:return self.send(409,{'error':'support required'})
                if value['release_reset']:value['reset_state']=True
                else:value['connected']=False
                value['release_receipt']={'status':'unknown' if value['release_unconfirmed'] else 'released','sdk_close_returned':not value['release_unconfirmed']}
                window['target']=dict(server_instance_id=instance,device_id='cpu-device',runtime_nonce=None)
                return self.send(200,{'connected':False,'release_receipt':value['release_receipt'],'target':window['target']})
            if action in ('connect_control','recover_control'):
                if action=='connect_control' and environment=='physical' and body.get('arms_supported') is not True:return self.send(409,{'error':'support required'})
                if action=='recover_control' and not(value['connected']and value['recoverable_fault']):return self.send(409,{'error':'in-place recovery unavailable'})
                value['connected']=True;window['target']=target();window['active_action']=action
                time.sleep(value['connect_delay']);window['active_action']=None
                if value['connect_partial'] or value['stop_requested']:
                    window['status']='failed';return self.send(409,{'error':'partial connection revoked; runtime retained'})
                value['recoverable_fault']=None
                value['nonrecoverable_fault']=None
                return self.send(200,dict(runtime=runtime(),target=target(),**(
                    dict(connected=True) if action=='connect_control' else dict(recovered=True,motion_resumed=False))))
            if action=='verify_hold':
                return self.send(200,dict(schema='workbench.fixed_hold_verification.v1',status='completed',verified=True,
                    duration_s=5.,held_source_s=4.9 if value['hold_invalid'] else 5.,held_wall_s=.1,
                    target_source='fresh_joint_feedback_and_bounded_calibrated_jaw_command',joint_offsets_applied=False,requested_target=None,
                    jaw_feedback_mm=[0.,0.],jaw_command_mm=[0.,0.],jaw_command_clipped={'left':False,'right':False},
                    fixed_target=[0.]*14,begin_feedback_qpos=[0.]*14,samples=[{'qpos':[0.]*14}],sample_count=1,
                    sdk_returns=[dict(operation='send',returned_monotonic_s=time.monotonic())],sdk_send_count=1,
                    stop_receipt={'stop_hold_return_confirmed':True},target=target(),cpu_fixture=True))
            if action=='stop_cameras':
                value['camera_stopped']=True
                return self.send(200,dict(stopped=True,exit_verified=True,service_pid=os.getpid(),plant_preserved=True,control_reconnected=False))
            if action=='invalidate_model':
                value['loaded']=False;value['warmup']=False;value['model_generation']+=1
                return self.send(200,dict(invalidated=True,model_ready=False,worker_lifecycle_changed=False))
            if action=='prepare_collection':
                role=body.get('service_role')
                if role:
                    state=value['collection_services'][role];state.update(pid=state['pid']+10,process_start_ticks=state['process_start_ticks']+10,model_initialized=False)
                    proof=deepcopy(state)
                    if value.get('bad_role_proof'):proof['pid']+=1
                    value['collection_ready']=False
                    return self.send(200,dict(service_role=role,service_ready=True,ready=False,needs_warmup=True,
                        service_step=dict(role=role,outcome='restarted',evidence={'service':proof}),cpu_fixture=True))
                value['collection_ready']=True;return self.send(200,dict(ready=True,cpu_fixture=True))
            if action in ('ops_start_cameras','ops_restart_cameras','ops_rebind_cameras'):
                value['camera_stopped']=False
                window['active_action']=action
                time.sleep(value['camera_delay']);window['active_action']=None
                if action=='ops_restart_cameras':value['camera_instance']=secrets.token_hex(8)
                return self.send(200,dict(already_running=action=='ops_start_cameras',instance_id=value['camera_instance'],
                    cameras={name:dict(healthy=True) for name in ('base_0','left_wrist_0','right_wrist_0')}))
            if action=='ops_prepare_model':
                if value['model_prepare_failure']:return self.send(409,{'error':'model worker exited before ready'})
                if 'expected_model_id' in body:
                    status=json_request(model_port,'/status')
                    if body['expected_model_id']!=value['selected_model_id'] or status['model_id']!=value['selected_model_id'] or status['loaded'] is not True or status['warmup']['completed'] is not True:
                        return self.send(409,{'error':'warm binding identity differs'})
                else:json_request(model_port,'/warm',{})
                value['loaded']=True;value['model_generation']+=1
                return self.send(200,dict(model_ready=True,control_ready=False,model_id=value['selected_model_id'],generation=value['model_generation']))
            return self.send(404,{'error':'fixture action missing'})
    server=ThreadingHTTPServer(('127.0.0.1',port),Handler);server.daemon_threads=True;server.serve_forever()


class CpuProcessService:
    """Own a real CPU fixture via Popen; do not invent platform start_ticks."""
    def __init__(self,kind,environment,model_port=0):
        self.kind,self.environment,self.model_port=kind,environment,model_port
        self.port=free_port();self.child=None;self.starts=0;self.stops=0;self.last_events=[]
    def snapshot(self):
        running=self.child is not None and self.child.poll() is None
        return dict(state='running' if running else 'stopped',running=running,owned=self.child is not None,
            identity=dict(pid=self.child.pid,script=str(Path(__file__).resolve()),process_state='fixture') if self.child else None,
            reason='test-owned CPU HTTP fixture; no production process identity claim')
    def start(self):
        if self.snapshot()['running']:return dict(self.snapshot(),reused=True)
        self.starts+=1
        self.child=subprocess.Popen([sys.executable,'-B',str(Path(__file__).resolve()),'--http-fixture',
            self.kind,str(self.port),self.environment,str(self.model_port)],stdin=subprocess.DEVNULL,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
        deadline=time.monotonic()+3
        while time.monotonic()<deadline:
            try:json_request(self.port,'/fixture',timeout=.1);return self.snapshot()
            except (OSError,ValueError):time.sleep(.01)
        raise AssertionError('CPU fixture did not start')
    def stop(self):
        if self.snapshot()['running']:
            self.last_events=json_request(self.port,'/fixture')['events'];self.stops+=1
            self.child.terminate();self.child.wait(timeout=3)
        return self.snapshot()
    def restart(self):self.stop();return self.start()
    def track_child(self,pid):
        if self.child is not None and self.child.poll() is None and self.child.pid==pid:return self.snapshot()
        return dict(state='unknown',owned=False,reason='CPU fixture only tracks its directly spawned PID')


class ManagementTests(unittest.TestCase):
    def test_actual_business_manager_and_page_environment_client(self):
        from robot_workbench.service import Application, protocol_fake, serve as serve_business
        device=protocol_fake(Path(self.directory.name)/'actual-business')
        backend=serve_business(Application({'fake':device}),port=0)
        thread=threading.Thread(target=backend.serve_forever,daemon=True);thread.start()
        try:
            h=self.harness('fake',services=self.h['services'],config_overrides={
                'device_id':device.id,'backend_port':backend.server_port})
            query='/api/environment?environment=fake&view=summary'
            raw=HTTPConnection('127.0.0.1',backend.server_port,timeout=3)
            try:
                raw.request('GET',query);response=raw.getresponse();response.read()
                self.assertEqual(response.status,409,'the business projection is not the public manager endpoint')
            finally:raw.close()
            code,state,_=self.request(query,h=h)
            self.assertEqual(code,200,state)
            self.assertEqual(state['projection'],'manager_environment_summary')
            self.assertEqual(state['target']['device_id'],device.id)
            self.assertFalse(state['runtime']['connected'])
            self.assertTrue(state['operations']['status_check']['allowed'])
            # Execute the real served JS client against the actual HTTP producer.
            program="""
                import assert from 'node:assert/strict';
                const origin=process.argv[1];
                const source=await (await fetch(origin+'/web/operations.js')).text();
                const {createOperationClient}=await import('data:text/javascript;base64,'+Buffer.from(source).toString('base64'));
                const client=createOperationClient({fetch:(url,options)=>{
                    assert(!options?.method||options.method==='GET');return fetch(new URL(url,origin),options);
                }});
                const state=await client.read('fake');
                assert.equal(client.compatible(state),true);
                assert.equal(state.runtime.connected,false);
                assert.equal(state.target.environment,'fake');
                assert(state.operations.status_check.allowed);
            """
            result=subprocess.run(['node','--input-type=module','-e',program,
                'http://127.0.0.1:'+str(h['port'])],capture_output=True,text=True,timeout=10)
            self.assertEqual(result.returncode,0,result.stderr[-2000:])
            code,_,headers=self.request('/?business=inference',h=h)
            self.assertEqual(code,303)
            self.assertEqual(headers['Location'],'/?business=inference&environment=fake')
            self.assertIsNone(device.runtime,'opening the complete page path must never connect hardware')
        finally:
            device.closed.set();backend.shutdown();backend.server_close();thread.join(1)

    def test_recovery_scope_accepts_registered_recorder_and_rejects_unknown_service(self):
        m=object.__new__(WorkbenchManager);m.services={name:object()for name in ('workbench','recording','cameras','model')}
        m.window={'affected_services':['workbench','recording']};m.unresolved=lambda:[];m.operations=SimpleNamespace(jobs={})
        self.assertEqual(m.maintenance_services({}),{'workbench','recording'})
        m.window={'affected_services':['workbench','foreign']}
        with self.assertRaisesRegex(ValueError,'作用域'):m.maintenance_services({})

    def test_recording_shutdown_refusal_precedes_any_process_stop(self):
        m=object.__new__(WorkbenchManager);m.config={};m.window=None;m.timeouts={'state':.1}
        service=Mock();service.snapshot.return_value=dict(running=True,state='running');m.services={'recording':service}
        with patch.dict(os.environ,WORKBENCH_RECORDING_SERVICE='/registered/service.json'),patch(
                'robot_workbench.recording_process.recording_call',side_effect=ValueError('sessions must seal')):
            with self.assertRaisesRegex(ValueError,'must seal'):m.service_action('recording','stop',lambda *a:None)
        service.stop.assert_not_called()

    def test_physical_source_wait_and_model_prepare_overlap_but_maintenance_rpc_does_not(self):
        from robot_workbench.operations import OperationError
        for failure in (None,'camera','model'):
            with self.subTest(failure=failure):
                m=object.__new__(WorkbenchManager);m.config={};events=[]
                source_started=threading.Event();model_finished=threading.Event()
                def camera(action,token,progress,*,bind):
                    self.assertFalse(bind);source_started.set()
                    self.assertTrue(model_finished.wait(1))
                    if failure=='camera':raise OperationError('camera_start_failed','CPU source failure')
                    events.append('source_ready');return dict(service={'pid':42})
                def model(*args,**kwargs):
                    self.assertTrue(source_started.wait(1),'source wait did not overlap model preparation')
                    events.append('model_finished');model_finished.set()
                    if failure=='model':raise OperationError('model_prepare_failed','CPU model failure')
                    return dict(prepared={'control_ready':False})
                def post(*args,**kwargs):
                    self.assertEqual(events,['model_finished','source_ready']);return {'bound':True}
                m.camera_action=camera;m.prepare_model=model;m.post=Mock(side_effect=post)
                if failure:
                    with self.assertRaises(OperationError):m._prepare_observation_and_model('cpu',Mock(),{'pid':41})
                    m.post.assert_not_called()
                else:
                    result=m._prepare_observation_and_model('cpu',Mock(),{'pid':41})
                    self.assertEqual(result['cameras']['binding'],{'bound':True})
                    m.post.assert_called_once_with('ops_rebind_cameras',token='cpu',timeout=15)

    def test_environment_model_process_starts_before_camera_wait_without_retry_or_early_warmup(self):
        from robot_workbench.operations import OperationError
        for case in ('ready','camera_failure','model_exit','stop'):
            with self.subTest(case=case):
                h=self.harness('fake');m=h['manager'];model=h['services']['model']
                model.stop();m.refresh();starts=model.starts;seen=[]
                def camera(*args,**kwargs):
                    self.assertTrue(model.snapshot()['running'],'camera wait began before independent model startup')
                    self.assertFalse(json_request(model.port,'/fixture')['loaded'],'model warmed before camera binding')
                    seen.append(True)
                    if case=='camera_failure':raise OperationError('camera_start_failed','CPU camera not ready')
                    if case=='model_exit':model.stop()
                    if case=='stop':m.stop_generation+=1
                    return dict(already_running=True,cameras={})
                request=dict(request_id='parallel-prepare',expected_target=m.target(),parameters={'capabilities':['observe','inference']})
                with patch.object(m,'camera_action',side_effect=camera),patch.object(m,'service_action',wraps=m.service_action)as actions:
                    if case!='ready':
                        with self.assertRaises(OperationError)as caught:m.run('prepare_environment',request,Mock())
                        self.assertEqual(caught.exception.code,dict(camera_failure='camera_start_failed',
                            model_exit='model_start_failed',stop='operator_stop')[case])
                    else:
                        result=m.run('prepare_environment',request,Mock())
                        self.assertTrue(result['model']['prepared']['model_ready'])
                self.assertEqual(seen,[True]);self.assertEqual(model.starts,starts+1)
                self.assertEqual([(call.args[0],call.args[1])for call in actions.call_args_list],[('model','start')])
                self.assertEqual(model.snapshot()['running'],case!='model_exit','partial software preparation must retain its real state')
                state=json_request(h['services']['workbench'].port,'/fixture')
                self.assertFalse(state['connected']);self.assertEqual(state['loaded'],case=='ready')

    def test_unverified_environment_process_and_owner_states_remain_unknown(self):
        from robot_workbench.operations import OperationError
        manager=object.__new__(WorkbenchManager)
        manager.health_lock=threading.Lock();manager.config=dict(control_lifecycle='external')
        healthy=dict(deployment=dict(files_unchanged=True),checked_wall_ns=time.time_ns(),services={'workbench':{'state':'running'}})
        for health,code in [(dict(healthy,checked_wall_ns=0),'environment_state_unknown'),
                (dict(healthy,services={'workbench':{'state':'unknown'}}),'process_identity_unknown'),
                (dict(healthy,control_owner_reachable=False),'owner_state_unknown')]:
            manager.health_cache=health
            with self.assertRaises(OperationError)as caught:manager.check('connect_control')
            self.assertTrue(caught.exception.unknown);self.assertEqual(caught.exception.code,code)
    def test_recover_nonrecoverable_fault_requires_verified_release_before_reconnect(self):
        for unconfirmed in (False,True):
            with self.subTest(release_unconfirmed=unconfirmed):
                h,cameras,client=self.camera_reset_fixture()
                self.configure(h=h,connected=True,nonrecoverable_fault='FAULT_STOP_UNCONFIRMED',release_unconfirmed=unconfirmed)
                code,rejected,_=self.start('recover_control',h=h,support=False)
                self.assertEqual(code,409,rejected)
                self.assertNotIn('release',json_request(h['services']['workbench'].port,'/fixture')['events'])
                with patch('robot_workbench.rgbd_service.CommonCameraService.connect',return_value=client):
                    code,job,_=self.start('recover_control',h=h,support=True);self.assertEqual(code,200,job)
                    result=self.wait_job(job,h)
                events=json_request(h['services']['workbench'].port,'/fixture')['events']
                self.assertIn('release',events,result);self.assertNotIn('recover_control',events,'hard fault is never merely acknowledged')
                if unconfirmed:
                    self.assertIn(result['status'],('failed','unknown'),result)
                    self.assertNotIn('connect_control',events,'unknown SDK close must never reconnect')
                else:
                    self.assertEqual(result['status'],'succeeded',result);self.assertTrue(result['verification']['ok'])
                    self.assertLess(events.index('release'),events.index('connect_control'))
                    self.assertTrue(result['result']['release']['release_receipt']['sdk_close_returned'])
                    self.assertTrue(result['result']['control']['sdk_reconnected']);self.assertFalse(result['result']['control']['motion_resumed'])
                    self.assertEqual({r['service']for r in result['result']['dependencies']},{'workbench','cameras'})
                self.assertEqual(h['services']['workbench'].starts,1);self.assertEqual(h['services']['workbench'].stops,0)
                self.assertEqual(h['services']['model'].starts,1);self.assertEqual(h['services']['model'].stops,0)

    def test_service_prepare_verifies_declared_processes_without_claiming_business_ready(self):
        from robot_workbench.management import stable_identity
        manager=self.h['manager'];health=manager.refresh()
        before=dict(backend_instance=health['backend_instance'],services={'workbench':stable_identity(health['services']['workbench']['identity'])})
        for environment in ('fake','physical'):
            roles=['detector','planner']+(['scene']if environment=='fake'else[])
            services={role:dict(status='ready',pid=100+index,process_start_ticks=str(200+index))for index,role in enumerate(roles)}
            for hot in (False,True):
                collection=dict(services_ready=True,ready=hot,needs_warmup=not hot,declared_roles=roles,services=services)
                result=dict(action='prepare_collection',before=before,collection=dict(collection,
                    service_steps=[dict(role=role,evidence={'service':row})for role,row in services.items()]))
                with patch.object(manager,'refresh',return_value=dict(health,collection=collection)):
                    self.assertTrue(manager.verify(result)['ok'])
                    wrong=deepcopy(result);wrong['collection']['service_steps'][0]['evidence']['service']['pid']+=1
                    self.assertFalse(manager.verify(wrong)['ok'])
                    missing=deepcopy(result);missing['collection']['service_steps'].pop()
                    self.assertFalse(manager.verify(missing)['ok'])
                    legacy=dict(result,collection={'ready':True})
                    self.assertFalse(manager.verify(legacy)['ok'])

    def test_triton_proxy_is_model_only_and_requires_manager_identity(self):
        m=self.h['manager'];m.triton_preview=Mock()
        m.triton_preview.catalog.return_value=dict(configured=True,available=True,sources=[],models=[])
        m.triton_preview.start.return_value=dict(job_id='cpu-job',status='QUEUED')
        code,body,_=self.request('/api/triton/catalog?environment=fake');self.assertEqual(code,200);self.assertEqual(body['models'],[])
        body=dict(environment='fake',business='inference',model_id='cpu-model',request_id='cpu-request')
        self.assertEqual(self.request('/api/triton/infer',body)[0],403)
        headers={'Content-Type':'application/json','X-CSRF-Token':m.csrf,'X-Workbench-Deployment':m.deployment.sha256}
        code,reply,_=self.request('/api/triton/infer',body,headers=headers)
        self.assertEqual(code,202,reply);m.triton_preview.start.assert_called_once_with('infer',dict(model_id='cpu-model',request_id='cpu-request'))
        self.assertFalse(m.state()['runtime']['connected'])
        self.assertEqual(self.request('/api/triton/catalog?environment=physical')[0],409)

    def setUp(self):
        self.directory=TemporaryDirectory();self.addCleanup(self.directory.cleanup)
        self.harnesses=[]
        self.addCleanup(self.cleanup)
        self.h=self.harness('fake')
    def cleanup(self):
        for h in reversed(self.harnesses):
            h['server'].shutdown();h['server'].server_close();h['manager'].close()
            for service in h['services'].values():service.stop()
    def harness(self,environment,root=None,services=None,config_overrides=None):
        if services is None:
            model=CpuProcessService('model',environment);model.start()
            backend=CpuProcessService('backend',environment,model.port);backend.start()
            services={'workbench':backend,'model':model}
        config=dict(schema='robot_workbench.manager.v1',environment=environment,device_id='cpu-device',
            state_root=str(root or Path(self.directory.name)/(environment+'-'+secrets.token_hex(4))),
            backend_port=services['workbench'].port,model_port=services['model'].port,
            timeouts=dict(state=.35,backend=2.,camera=.5,release=.5,operation=5.))
        config.update(deepcopy(config_overrides or {}))
        source=Path(self.directory.name)/'source-fixture'/'web';source.mkdir(parents=True,exist_ok=True)
        (source/'operations.js').write_text('// isolated CPU deployment fixture\n')
        with patch('robot_workbench.management.Deployment',side_effect=lambda root,registered:Deployment(source.parent,{})):
            manager=WorkbenchManager(config,services=services,monitor=False)
        server=serve(manager,host='127.0.0.1',port=0)
        threading.Thread(target=server.serve_forever,daemon=True).start()
        h=dict(manager=manager,server=server,services=services,port=server.server_port,config=config)
        self.harnesses.append(h);return h
    def request(self,path,body=None,*,h=None,headers=None,method=None):
        h=h or self.h;connection=HTTPConnection('127.0.0.1',h['port'],timeout=4)
        try:
            raw=json.dumps(body).encode() if body is not None else None
            connection.request(method or ('POST' if body is not None else 'GET'),path,raw,headers or {})
            response=connection.getresponse();data=response.read();meta=dict(response.getheaders())
            try:data=json.loads(data)
            except (ValueError,UnicodeError):pass
            return response.status,data,meta
        finally:connection.close()
    def configure(self,*,h=None,service='workbench',**values):
        h=h or self.h;return json_request(h['services'][service].port,'/fixture',values)
    def start(self,action,*,h=None,support=False):
        h=h or self.h;h['manager'].refresh();state=h['manager'].snapshot()
        body=dict(action=action,request_id=secrets.token_hex(8),expected_target=state['target'],arms_supported=support)
        return self.request('/api/operations/action?environment='+h['manager'].environment,body,h=h,
            headers={'Content-Type':'application/json','X-CSRF-Token':state['csrf'],
                     'X-Workbench-Deployment':state['deployment']['sha256'],'Origin':'http://127.0.0.1:'+str(h['port'])})
    def wait_job(self,job,h=None):
        h=h or self.h;deadline=time.monotonic()+5
        while time.monotonic()<deadline:
            result=h['manager'].operations.jobs[job['request']['request_id']]
            if result.get('finished_at_ns') is not None:return deepcopy(result)
            time.sleep(.01)
        self.fail('management job did not finish')

    def test_shared_environments_release_restart_and_do_not_end_old_token_on_new_backend(self):
        for h in (self.h,self.camera_reset_fixture()[0]):
            self.configure(h=h,connected=True)
            old=json_request(h['services']['workbench'].port,'/api/state')['server_instance_id']
            status,job,_=self.start('restart_workbench',h=h,support=True);self.assertEqual(status,200,job)
            result=self.wait_job(job,h);self.assertEqual(result['status'],'succeeded',result)
            service=h['services']['workbench'];new=json_request(service.port,'/api/state')
            self.assertNotEqual(new['server_instance_id'],old);self.assertFalse(new['runtime']['connected'])
            self.assertIn('release',service.last_events);self.assertEqual(service.stops,1)
            self.assertNotIn('ops_maintenance_end',json_request(service.port,'/fixture')['events'])
            self.assertIsNone(h['manager'].window)
            public=json.dumps(h['manager'].snapshot())
            self.assertNotIn('private-maintenance-',public)
            for path in (Path(h['config']['state_root'])/'operations').glob('job-*.json'):
                self.assertNotIn('private-maintenance-',path.read_text())

    def test_deployment_handshake_rejects_stale_pages_without_polling_source_files(self):
        manager=self.h['manager'];state=manager.snapshot();version=state['deployment']['sha256']
        status,deployment,_=self.request('/api/deployment');self.assertEqual(status,200)
        self.assertEqual(deployment['sha256'],version)
        status,source,headers=self.request('/web/operations.js')
        self.assertEqual(status,200);self.assertIn(version.encode(),source)
        self.assertNotIn(b'__WORKBENCH_DEPLOYMENT_SHA__',source)
        self.assertEqual(headers['X-Workbench-Deployment'],version)
        body=dict(action='start_cameras',request_id='stale-page',expected_target=state['target'],arms_supported=False)
        authorization={'Content-Type':'application/json','X-CSRF-Token':state['csrf']}
        for supplied in ({},{'X-Workbench-Deployment':'b'*64}):
            status,error,_=self.request('/api/operations/action',body,headers=dict(authorization,**supplied))
            self.assertEqual(status,409);self.assertEqual(error['code'],'deployment_mismatch')
        self.assertEqual(manager.operations.jobs,{})
        status,error,_=self.request('/api/actions/preview',{},headers={'Content-Type':'application/json','X-Workbench-Deployment':'b'*64})
        self.assertEqual(status,409);self.assertEqual(error['code'],'deployment_mismatch')
        body.update(action='status_check',request_id='read-only-stale-page')
        status,job,_=self.request('/api/operations/action',body,headers=authorization)
        self.assertEqual(status,200);self.assertEqual(self.wait_job(job)['status'],'succeeded')
        backend=json_request(self.h['services']['workbench'].port,'/api/state')
        status,result,_=self.request('/api/actions/stop',dict(environment='fake',business='inference',request_id='stale-stop'),
            headers={'Content-Type':'application/json','Origin':'http://127.0.0.1:'+str(self.h['port']),
                     'X-CSRF-Token':backend['csrf'],'X-Workbench-Deployment':'b'*64})
        self.assertEqual(status,200);self.assertTrue(result['stopped'])
        source=Path(self.directory.name)/'source-fixture/web/operations.js';source.write_text('// changed deployed bytes\n')
        self.assertEqual(self.request('/api/operations')[0],200)
        status,job,_=self.start('start_cameras')
        self.assertEqual(status,200,job);self.assertEqual(self.wait_job(job)['status'],'succeeded')
        self.assertNotIn('files_unchanged',manager.snapshot()['deployment'])

    def test_profile_identity_and_history_do_not_claim_live_hardware_or_current_version(self):
        status,job,_=self.start('status_check');self.assertEqual(status,200)
        old=self.wait_job(job)['request']['expected_target']['deployment_sha256']
        source=Path(self.directory.name)/'new-deployment/web';source.mkdir(parents=True)
        (source/'operations.js').write_text('// another release\n')
        manager=self.h['manager'];manager.deployment=Deployment(source.parent,{})
        manager.refresh();snapshot=manager.snapshot()
        self.assertNotEqual(snapshot['deployment']['sha256'],old)
        self.assertEqual(snapshot['recent_jobs'][0]['request']['expected_target']['deployment_sha256'],old)
        self.assertEqual(snapshot['machine_profile']['identity'],dict(device_id='cpu-device',environment='fake'))
        self.assertFalse(snapshot['machine_profile']['hardware_verified'])
        self.h['services']['workbench'].stop();manager.refresh();snapshot=manager.snapshot()
        self.assertFalse(snapshot['health']['backend_reachable'])
        self.assertFalse(snapshot['current_capabilities']['model_loaded'])
        self.assertEqual(snapshot['recent_jobs'][0]['status'],'succeeded')

    def test_independent_warm_model_survives_backend_unbound_or_unreachable(self):
        self.configure(service='model',loaded=True,warmup=True,stderr_tail='private-fixture-log-must-not-publish')
        manager=self.h['manager'];manager.refresh();snapshot=manager.snapshot()
        self.assertFalse(snapshot['health']['model']['loaded'])
        self.assertTrue(snapshot['current_capabilities']['model_loaded'])
        self.assertFalse(snapshot['current_capabilities']['workflow_model_bound'])
        self.h['services']['workbench'].stop();manager.refresh();snapshot=manager.snapshot()
        self.assertFalse(snapshot['health']['backend_reachable'])
        self.assertTrue(snapshot['current_capabilities']['model_loaded'])
        self.assertIsNone(snapshot['current_capabilities']['workflow_model_bound'])
        self.assertIsNone(snapshot['current_capabilities']['control_connected'])
        self.assertTrue({'status','model_id','loaded','warmup','service_pid','worker_pid','observed_wall_ns'}<=set(snapshot['health']['model_service']))
        self.assertNotIn('private-fixture',json.dumps(snapshot))

    def test_model_pid_mismatch_is_unknown_and_never_inferred_from_workflow_ready(self):
        self.configure(loaded=True);self.configure(service='model',loaded=True,warmup=True,wrong_pid=True)
        manager=self.h['manager'];manager.refresh();snapshot=manager.snapshot()
        self.assertTrue(snapshot['health']['model']['loaded'])
        self.assertIsNone(snapshot['current_capabilities']['model_loaded'])
        self.assertEqual(snapshot['health']['model_service']['status'],'unknown')

    def test_independent_ui_and_operations_survive_backend_exit(self):
        self.h['services']['workbench'].stop();self.h['manager'].refresh()
        for path in ('/?environment=fake','/operations?environment=fake','/web/operations.js','/api/operations?environment=fake'):
            self.assertEqual(self.request(path)[0],200,path)
        self.assertEqual(self.request('/api/state?environment=fake')[0],503)
        status,job,_=self.start('restart_workbench');self.assertEqual(status,200,job)
        self.assertEqual(self.wait_job(job)['status'],'succeeded')

    def test_backend_reload_rebinds_same_warm_worker_without_reloading_or_connecting(self):
        self.configure(loaded=True);self.configure(service='model',loaded=True,warmup=True)
        model=self.h['services']['model'];before=json_request(model.port,'/status');starts=model.starts
        _,job,_=self.start('restart_workbench');result=self.wait_job(job)
        self.assertEqual(result['status'],'succeeded',result)
        after=json_request(model.port,'/status');state=self.h['manager'].state()
        self.assertEqual(after['worker_pid'],before['worker_pid']);self.assertEqual(model.starts,starts);self.assertEqual(model.stops,0)
        self.assertEqual(json_request(model.port,'/fixture')['events'],[],'no model load request')
        self.assertTrue(state['model']['loaded']);self.assertFalse(state['runtime']['connected'])
        self.assertIsNone(state['maintenance']);self.assertIsNone(self.h['manager'].window)
        events=json_request(self.h['services']['workbench'].port,'/fixture')['events']
        self.assertEqual(events,['ops_maintenance_begin','ops_prepare_model','ops_maintenance_end'])
        # A different profile default must not silently replace the old selection.
        self.configure(selected_model_id='previous-selected');self.configure(service='model',selected_model_id='previous-selected')
        _,job,_=self.start('restart_workbench');result=self.wait_job(job)
        self.assertEqual(result['status'],'failed');self.assertIn('显式选择原模型',json.dumps(result,ensure_ascii=False))
        self.assertEqual(json_request(model.port,'/status')['worker_pid'],before['worker_pid'])
        self.assertEqual(json_request(model.port,'/fixture')['events'],[])

    def test_backend_reload_does_not_load_cold_or_unconfigured_model(self):
        model=self.h['services']['model']
        for configured in (True,False):
            with self.subTest(configured=configured):
                if not configured:
                    self.h['manager'].services={'workbench':self.h['services']['workbench']}
                    self.h['manager'].config.pop('model_port')
                _,job,_=self.start('restart_workbench');result=self.wait_job(job)
                self.assertEqual(result['status'],'succeeded',result)
                self.assertFalse(json_request(model.port,'/status')['loaded'])
                self.assertEqual(json_request(model.port,'/fixture')['events'],[])
                self.assertNotIn('ops_prepare_model',json_request(self.h['services']['workbench'].port,'/fixture')['events'])
                self.assertFalse(self.h['manager'].state()['runtime']['connected'])

    def test_http_unreachable_while_process_alive_never_stops_it(self):
        self.configure(reset_state=True)
        status,result,_=self.start('restart_workbench')
        self.assertEqual(status,409,result);self.assertEqual(self.h['services']['workbench'].stops,0)
        self.assertTrue(self.h['services']['workbench'].snapshot()['running'])

    def test_lost_release_state_keeps_freeze_until_explicit_recovery(self):
        self.configure(connected=True,release_reset=True)
        status,job,_=self.start('restart_workbench');self.assertEqual(status,200,job)
        result=self.wait_job(job);self.assertEqual(result['status'],'unknown')
        self.assertEqual(self.h['services']['workbench'].stops,0);self.assertIsNotNone(self.h['manager'].window)
        self.assertEqual(os.stat(self.h['manager'].window_path).st_mode&0o777,0o600)
        self.configure(reset_state=False,release_reset=False,connected=False)
        self.assertEqual(self.start('start_cameras')[0],409)
        status,job,_=self.start('recover_maintenance');self.assertEqual(status,200,job)
        self.assertEqual(self.wait_job(job)['status'],'succeeded');self.assertIsNone(self.h['manager'].window)

    def test_lost_begin_reply_reuses_private_request_identity_on_recovery(self):
        self.configure(lose_begin=True)
        status,job,_=self.start('start_cameras');self.assertEqual(status,200,job)
        self.assertEqual(self.wait_job(job)['status'],'unknown');self.assertIsNone(self.h['manager'].window['token'])
        begin=self.h['manager'].window['begin_body']['request_id']
        status,job,_=self.start('recover_maintenance');self.assertEqual(status,200,job)
        self.assertEqual(self.wait_job(job)['status'],'succeeded')
        events=json_request(self.h['services']['workbench'].port,'/fixture')['events']
        self.assertEqual(events.count('ops_maintenance_begin'),2)
        self.assertEqual(len(begin),32)

    def test_operation_failure_is_preserved_when_maintenance_cleanup_also_fails(self):
        self.configure(end_fail=True)
        manager=self.h['manager']
        with self.assertRaisesRegex(ValueError,'杯位波动.*维护收尾尚未确认.*fixture maintenance still active') as caught:
            with manager.maintenance('prepare_collection'):
                raise ValueError('杯位波动6.44mm超过6mm')
        self.assertTrue(caught.exception.unknown)
        self.assertIsNotNone(manager.window)
        self.configure(end_fail=False)
        manager.finish_window()
        self.assertIsNone(manager.window)

    def test_private_claim_survives_manager_restart_and_recovery_is_visible(self):
        self.configure(lose_begin=True)
        _,job,_=self.start('start_cameras');self.assertEqual(self.wait_job(job)['status'],'unknown')
        root=Path(self.h['config']['state_root']);before=json.loads((root/'maintenance-private.json').read_text())
        self.h['server'].shutdown();self.h['server'].server_close();self.h['manager'].close()
        restored=self.harness('fake',root=root,services=self.h['services'])
        self.assertEqual(restored['manager'].window['begin_body'],before['begin_body'])
        state=restored['manager'].snapshot()
        self.assertIn('recover_maintenance',state['operations']);self.assertTrue(state['operations']['recover_maintenance']['allowed'])
        self.assertFalse(state['operations']['start_cameras']['allowed'])
        _,job,_=self.start('recover_maintenance',h=restored);self.assertEqual(self.wait_job(job,restored)['status'],'succeeded')
        self.assertIsNone(json.loads((root/'maintenance-private.json').read_text()))

    def test_configuration_rejects_nonlocal_ports_and_untrusted_descriptor_shapes(self):
        for changes in ({'model_port':'http://elsewhere:1234'},{'model_port':self.h['config']['backend_port']},
                        {'camera_descriptor':'../camera.json'},{'camera_descriptor':{'path':'/tmp/camera'}},
                        {'timeouts':{'state':float('nan')}},{'backend_attempt_pointer':'relative/current.json'},
                        {'backend_attempt_pointer':'/trusted/backend/../current.json'},{'backend_attempt_pointer':'/trusted/backend/log.json'}):
            with self.assertRaises(ValueError):WorkbenchManager(dict(self.h['config'],**changes),services=self.h['services'],monitor=False)

    def attempt_fixture(self,error="No module named 'robot_sim'"):
        root=(Path(self.directory.name)/'backend').resolve();output=root/'attempt-123';output.mkdir(parents=True)
        started=time.time_ns();identity=dict(pid=self.h['services']['workbench'].child.pid,start_ticks='123',
            output=str(output),started_wall_ns=started,argv=['never-publish','--token','private-fixture-value'])
        (root/'current.json').write_text(json.dumps(identity))
        exited=dict(identity,status='exited',exit_code=0,finished_wall_ns=started+1)
        (root/'attempt-123-exit.json').write_text(json.dumps(exited))
        (output/'final.json').write_text(json.dumps(dict(status='failed',error=error,traceback='private-fixture-traceback')))
        manager=self.h['manager'];manager.backend_attempt_pointer=root/'current.json';manager.backend_attempt_root=root
        return root,identity

    def test_zero_process_exit_still_fails_fast_on_owned_attempt_final_failure(self):
        root,identity=self.attempt_fixture();service=self.h['services']['workbench'];manager=self.h['manager']
        json_request(service.port,'/fixture/exit-zero');self.assertEqual(service.child.wait(3),0)
        phases=[];began=time.monotonic()
        with self.assertRaisesRegex(ValueError,"No module named 'robot_sim'") as caught:
            manager.wait_backend(lambda phase,**values:phases.append((phase,values)),
                started_service=dict(identity=service.snapshot()['identity'],startup_admitted_wall_ns=identity['started_wall_ns']-1))
        self.assertEqual(caught.exception.code,'backend_start_failed')
        self.assertLess(time.monotonic()-began,.4)
        self.assertEqual(phases[0][0],'backend_start_failed')
        self.assertEqual(set(phases[0][1]['detail']),{'status','error'})
        manager.refresh();public=json.dumps(manager.snapshot())
        self.assertIn('robot_sim',public);self.assertNotIn('private-fixture',public)
        self.assertEqual(manager.snapshot()['health']['backend_attempt']['status'],'failed')
        (root/'attempt-123/final.json').write_text(json.dumps(dict(status='failed',error='Authorization: Bearer private-fixture-token')))
        self.assertNotIn('private-fixture',json.dumps(manager.backend_attempt(service.snapshot())))

    def test_unknown_exit_or_old_attempt_waits_and_reports_unknown_instead_of_final_failure(self):
        root,identity=self.attempt_fixture();manager=self.h['manager'];service=self.h['services']['workbench']
        original=service.snapshot();manager.timeouts['backend']=.12
        states=[dict(original,state='unknown',running=True,children=[{'pid':9876}]),
                dict(original,state='stopped',running=False,owned=False),
                dict(original,state='stopped',running=False,identity=dict(original['identity'],pid=1))]
        for observed in states:
            with self.subTest(observed=observed['state']),patch.object(manager,'state',side_effect=ConnectionError('not ready')),\
                    patch.object(service,'snapshot',return_value=observed):
                began=time.monotonic()
                with self.assertRaisesRegex(ValueError,'状态未知') as caught:
                    manager.wait_backend(started_service=dict(identity=original['identity']))
                self.assertEqual(caught.exception.code,'backend_start_unknown');self.assertGreaterEqual(time.monotonic()-began,.12)
        stopped=dict(original,state='stopped',running=False)
        with patch.object(manager,'state',side_effect=ConnectionError('not ready')),patch.object(service,'snapshot',return_value=stopped):
            with self.assertRaisesRegex(ValueError,'旧启动'):
                manager.wait_backend(started_service=dict(identity=original['identity'],startup_admitted_wall_ns=identity['started_wall_ns']+1))
            exited=dict(identity,status='unknown',exit_code=None,finished_wall_ns=time.time_ns())
            (root/'attempt-123-exit.json').write_text(json.dumps(exited))
            with self.assertRaisesRegex(ValueError,'尚不可核验'):manager.wait_backend()

    def test_live_backend_ignores_old_final_failure_and_pointer_escape_never_reads_target(self):
        root,identity=self.attempt_fixture();manager=self.h['manager'];state=manager.state()
        with patch.object(manager,'state',side_effect=[ConnectionError('starting'),ConnectionError('starting'),state]) as ready:
            self.assertEqual(manager.wait_backend(),state);self.assertEqual(ready.call_count,3)
        outside=Path(self.directory.name)/'outside';outside.mkdir();(outside/'final.json').write_text('{"status":"failed","error":"outside-private-value"}')
        (root/'current.json').write_text(json.dumps(dict(identity,output=str(outside))))
        observed=dict(self.h['services']['workbench'].snapshot(),state='stopped',running=False)
        result=manager.backend_attempt(observed)
        self.assertEqual(result['status'],'unknown');self.assertNotIn('outside-private-value',json.dumps(result))

    def test_camera_start_is_reusable_but_stale_frames_do_not_verify(self):
        for _ in range(2):
            status,job,_=self.start('start_cameras');self.assertEqual(status,200,job)
            result=self.wait_job(job);self.assertEqual(result['status'],'succeeded',result)
            self.assertEqual(set(result['verification']['evidence']['cameras']),{'head','left','right'})
        self.configure(stale_frames=True)
        _,job,_=self.start('start_cameras');result=self.wait_job(job)
        self.assertEqual(result['status'],'failed');self.assertFalse(result['verification']['ok'])

    def test_camera_http_first_frames_wait_without_repeating_action(self):
        self.configure(media_missing={'head':2,'left':1,'right':1})
        manager=self.h['manager']
        with patch.object(manager,'refresh',wraps=manager.refresh) as refresh:
            _,job,_=self.start('restart_cameras');result=self.wait_job(job)
        self.assertEqual(result['status'],'succeeded',result)
        frames=result['verification']['evidence']['cameras']
        self.assertEqual(set(frames),{'head','left','right'})
        self.assertTrue(all(row['bytes']>0 and 0<=row['source_age_s']<=2 for row in frames.values()))
        observed=json_request(self.h['services']['workbench'].port,'/fixture')
        self.assertEqual(observed['events'].count('ops_restart_cameras'),1)
        self.assertGreaterEqual(observed['media_attempts']['head'],3)
        self.assertGreaterEqual(observed['media_attempts']['left'],2)
        self.assertGreaterEqual(observed['media_attempts']['right'],2)
        self.assertGreaterEqual(refresh.call_count,2)
        self.assertGreaterEqual(result['verification']['health']['checked_wall_ns'],max(row['host_receive_ns'] for row in frames.values()))

    def test_camera_http_missing_invalid_and_stale_stay_failed_until_deadline(self):
        manager=self.h['manager'];manager.timeouts['camera']=.16
        for changes in ({'media_missing':{'head':1000}},{'invalid_image':True},{'stale_frames':True}):
            with self.subTest(changes=changes):
                self.configure(media_missing={},media_attempts={},invalid_image=False,stale_frames=False)
                self.configure(**changes);began=time.monotonic()
                with self.assertRaisesRegex(TimeoutError,'HTTP图像未在等待时限内就绪'):manager.camera_evidence()
                self.assertGreaterEqual(time.monotonic()-began,.15)
                self.assertLess(time.monotonic()-began,.8)
                observed=json_request(self.h['services']['workbench'].port,'/fixture')
                self.assertGreaterEqual(observed['media_attempts']['head'],2)
                self.assertEqual(observed['events'],[],'frame polling must not issue any camera action')

    def test_model_restart_checks_owned_pid_and_real_warmup_contract_without_infer(self):
        prior=self.h['services']['model'].child.pid
        status,job,_=self.start('restart_model');self.assertEqual(status,200,job)
        result=self.wait_job(job);self.assertEqual(result['status'],'succeeded',result)
        self.assertNotEqual(self.h['services']['model'].child.pid,prior)
        self.assertTrue(result['verification']['evidence']['model']['warmup']['completed'])
        self.assertEqual(json_request(self.h['services']['model'].port,'/fixture')['events'],['warm'])
        self.configure(service='model',wrong_pid=True)
        _,job,_=self.start('prepare_model');self.assertEqual(self.wait_job(job)['status'],'failed')

    def test_model_start_waits_for_matching_http_endpoint_before_warmup(self):
        model=self.h['services']['model'];attempts=[];ready=[]
        def delayed(port,path,*args,**kwargs):
            if port==model.port and path=='/status':
                attempts.append(path)
                if len(attempts)<3:raise ConnectionRefusedError('model HTTP is still starting')
                value=json_request(port,path,*args,**kwargs)
                ready.append(value['loaded']);return value
            if path=='/api/maintenance/prepare_model':
                self.assertGreaterEqual(len(attempts),3);self.assertEqual(ready,[False])
            return json_request(port,path,*args,**kwargs)
        with patch('robot_workbench.management.json_request',side_effect=delayed):
            status,job,_=self.start('prepare_model');self.assertEqual(status,200)
            result=self.wait_job(job)
        self.assertEqual(result['status'],'succeeded',result)
        phases=[event['phase'] for event in result['events']]
        self.assertLess(phases.index('waiting_model_endpoint'),phases.index('warming_model'))

    def test_model_start_reports_owned_exit_early_but_never_adopts_unknown_process(self):
        manager=self.h['manager'];model=self.h['services']['model'];started=model.snapshot();model.stop()
        began=time.monotonic();phases=[]
        with self.assertRaisesRegex(ValueError,'已退出') as caught:
            manager.wait_model(started,lambda phase,**detail:phases.append(phase))
        self.assertEqual(caught.exception.code,'model_start_failed');self.assertLess(time.monotonic()-began,.2)
        self.assertEqual(phases,['waiting_model_endpoint','model_start_failed'])
        manager.timeouts['model']=.12
        with patch.object(model,'snapshot',return_value=dict(started,state='unknown',owned=False)),\
                patch('robot_workbench.management.json_request') as read:
            began=time.monotonic()
            with self.assertRaisesRegex(ValueError,'身份尚未核验') as caught:manager.wait_model(started,lambda phase:None)
            self.assertEqual(caught.exception.code,'model_start_unknown');self.assertGreaterEqual(time.monotonic()-began,.12)
            read.assert_not_called()

    def test_model_prepare_failure_adds_only_last_exception_line_from_owned_status(self):
        self.configure(model_prepare_failure=True)
        self.configure(service='model',stderr_tail='private-fixture-noise\n File "/private/config", line 1\nValueError: old error\nRuntimeError: missing transformer configuration\n')
        _,job,_=self.start('prepare_model');result=self.wait_job(job)
        self.assertEqual(result['error']['code'],'model_prepare_failed')
        self.assertIn('RuntimeError: missing transformer configuration',result['error']['message'])
        self.assertNotIn('private-fixture',json.dumps(result));self.assertNotIn('/private/config',json.dumps(result))
        self.assertNotIn('old error',json.dumps(result))
        self.configure(service='model',stderr_tail='RuntimeError: API_TOKEN=private-fixture-secret\n')
        _,job,_=self.start('prepare_model');result=self.wait_job(job)
        self.assertNotIn('private-fixture',json.dumps(result));self.assertIn('凭据',result['error']['message'])

    def camera_reset_fixture(self):
        h=self.harness('physical');root=Path(h['config']['state_root'])/'camera-source';root.mkdir()
        cameras=CpuProcessService('cameras','physical');cameras.start();h['services']['cameras']=cameras
        profile=root/'profile.json';profile.write_text(json.dumps(dict(environment='physical',robot_id='cpu-device',
            cameras={role:{'serial':str(100+i)} for i,role in enumerate(('head','left','right'))})))
        descriptor=root/'current.json';descriptor.write_text(json.dumps(dict(environment='physical',secret='private-fixture-descriptor',
            intrinsics={camera:{'serial':str(100+i)} for i,camera in enumerate(('base_0','left_wrist_0','right_wrist_0'))})))
        cameras.spec=SimpleNamespace(kind='process',expected_script=str(ROOT/'scripts/serve_camera_sources.py'),env=None,
            command=[sys.executable,'-B',str(ROOT/'scripts/serve_camera_sources.py'),'--profile',str(profile),'--root',str(root)])
        h['manager'].config['camera_descriptor']=str(descriptor)
        client=Mock();client.frames=dict.fromkeys(('base_0','left_wrist_0','right_wrist_0'))
        client.snapshot.return_value=dict(recording_channels={camera:{'run_id':None} for camera in client.frames})
        self.configure(h=h,connected=True)
        return h,cameras,client

    def test_physical_fixed_role_reset_pauses_three_streams_and_preserves_control(self):
        h,cameras,client=self.camera_reset_fixture();before=h['services']['workbench'].child.pid
        reply=dict(status='enumerated',role='left',serial='101',reset_calls=1,disappearance_observed=True,
                   raw_frames_verified=False,token='private-fixture-must-not-leak')
        with patch('robot_workbench.rgbd_service.CommonCameraService.connect',return_value=client),\
                patch('robot_workbench.management.subprocess.run',return_value=SimpleNamespace(returncode=0,stdout=json.dumps(reply),stderr='')) as reset:
            status,job,_=self.start('reset_camera_left',h=h);self.assertEqual(status,200,job)
            result=self.wait_job(job,h)
        self.assertEqual(result['status'],'succeeded',result)
        command=reset.call_args.args[0];self.assertEqual(command[-2:],['--reset-role','left']);self.assertNotIn('--serial',command)
        self.assertEqual(reset.call_count,1);self.assertEqual(cameras.stops,1);self.assertEqual(cameras.starts,2)
        self.assertEqual(client.latest_rgbd.call_count,3)
        self.assertEqual(h['services']['workbench'].child.pid,before);self.assertEqual(h['services']['workbench'].stops,0)
        self.assertTrue(h['manager'].snapshot()['health']['runtime']['connected'])
        self.assertNotIn('private-fixture',json.dumps(result))
        self.assertEqual(set(result['verification']['evidence']['cameras']),{'head','left','right'})

    def test_ros_camera_profile_admits_complete_connect_and_reset_without_legacy_descriptor(self):
        h,cameras,_=self.camera_reset_fixture();manager=h['manager']
        path=Path(cameras.spec.command[cameras.spec.command.index('--profile')+1])
        profile=json.loads(path.read_text())
        profile['data_plane']=dict(schema='workbench.ros-data-plane.v1',namespace='/cpu',domain_id=126)
        for role,item in profile['cameras'].items():
            intrinsic=path.parent/(role+'-intrinsics.json')
            intrinsic.write_text(json.dumps(dict(serial=item['serial'],width=2,height=2)))
            item['intrinsics']=str(intrinsic)
        path.write_text(json.dumps(profile))
        manager.config.pop('camera_descriptor')
        manager.config['publication_files']={'physical_profile':str(path)}
        self.configure(h=h,connected=False)
        view=Mock()
        with patch('robot_workbench.ros_inputs.RosCameraView',return_value=view),\
                patch('robot_workbench.rgbd_service.CommonCameraService.connect',side_effect=AssertionError('legacy IPC used')):
            status,job,_=self.start('connect_control',h=h,support=True)
            self.assertEqual(status,200,job)
            result=self.wait_job(job,h);self.assertEqual(result['status'],'succeeded',result)
            self.assertTrue(manager.snapshot()['health']['runtime']['connected'])
            reply=dict(status='enumerated',role='left',serial='101',reset_calls=1,disappearance_observed=True)
            with patch.dict(os.environ,WORKBENCH_RECORDING_SERVICE='/cpu/recording.json'),\
                    patch('robot_workbench.recording_process.recording_call',return_value={'can_stop':True}),\
                    patch('robot_workbench.management.subprocess.run',return_value=SimpleNamespace(returncode=0,stdout=json.dumps(reply),stderr='')):
                status,job,_=self.start('reset_camera_left',h=h)
                self.assertEqual(status,200,job)
                result=self.wait_job(job,h);self.assertEqual(result['status'],'succeeded',result)
            self.configure(h=h,connected=False)
            status,job,_=self.start('recover_control',h=h,support=True)
            self.assertEqual(status,200,job)
            result=self.wait_job(job,h);self.assertEqual(result['status'],'succeeded',result)
        self.assertEqual(cameras.stops,1)
        self.assertEqual(view.close.call_count,3)
        profile['robot_id']='another-device';path.write_text(json.dumps(profile))
        self.configure(h=h,connected=False)
        starts=cameras.starts
        status,_,_=self.start('connect_control',h=h,support=True)
        self.assertEqual(status,409);self.assertEqual(cameras.starts,starts)

    def test_camera_owned_exit_reports_source_error_without_waiting_or_retrying(self):
        h,cameras,_=self.camera_reset_fixture();manager=h['manager'];started=cameras.snapshot()
        stopped=dict(started,state='stopped',running=False)
        path=Path(manager.config['camera_descriptor']).parent/'state.json'
        path.write_text(json.dumps(dict(pid=started['identity']['pid'],phase='error',error="left: Frame didn't arrive within 3000")))
        with patch.object(manager,'service_action',return_value=started) as action,patch.object(cameras,'snapshot',return_value=stopped),\
                patch('robot_workbench.rgbd_service.CommonCameraService.connect',side_effect=ConnectionRefusedError):
            began=time.monotonic()
            with self.assertRaisesRegex(ValueError,'left: Frame') as caught:manager.camera_action('start','fixture-token',lambda *_:None)
            self.assertEqual(caught.exception.code,'camera_start_failed');self.assertLess(time.monotonic()-began,.2)
            action.assert_called_once()
            self.assertIn('left: Frame',manager.refresh()['services']['cameras']['error'])
        path.write_text(json.dumps(dict(pid=started['identity']['pid']+1,phase='error',error='wrong owner')))
        self.assertIsNone(manager.camera_source_error(stopped))
        self.assertIsNone(manager.camera_source_error(started))

    def test_reset_is_unsupported_for_fake_and_recording_or_uncertain_reset_never_retries(self):
        snapshot=self.h['manager'].snapshot()
        for role in ('head','left','right'):
            row=snapshot['operations']['reset_camera_'+role];self.assertFalse(row['allowed']);self.assertIn('Fake',row['disabled_reason'])
        self.assertEqual(self.start('reset_camera_head')[0],409)
        h,cameras,client=self.camera_reset_fixture()
        client.snapshot.return_value['recording_channels']['base_0']['run_id']='active-lease'
        with patch('robot_workbench.rgbd_service.CommonCameraService.connect',return_value=client),\
                patch('robot_workbench.management.subprocess.run') as reset:
            _,job,_=self.start('reset_camera_head',h=h);result=self.wait_job(job,h)
            self.assertIn('录制租约',result['error']['message']);reset.assert_not_called();self.assertEqual(cameras.stops,0)
        client.snapshot.return_value['recording_channels']['base_0']['run_id']=None
        with patch('robot_workbench.rgbd_service.CommonCameraService.connect',return_value=client),\
                patch('robot_workbench.management.subprocess.run',return_value=SimpleNamespace(returncode=1,stdout='',stderr='RuntimeError: reset outcome uncertain')) as reset:
            _,job,_=self.start('reset_camera_head',h=h);result=self.wait_job(job,h)
        self.assertEqual(result['error']['code'],'camera_reset_unknown');self.assertEqual(reset.call_count,1)
        self.assertFalse(cameras.snapshot()['running']);self.assertEqual(cameras.starts,1)
        self.assertEqual(h['services']['workbench'].stops,0)

    def test_reset_recovery_failure_preserves_single_reset_evidence_and_process_facts(self):
        h,cameras,client=self.camera_reset_fixture();manager=h['manager']
        evidence=dict(status='enumerated',role='right',serial='102',reset_calls=1,
                      disappearance_observed=True,raw_frames_verified=False)
        with patch('robot_workbench.rgbd_service.CommonCameraService.connect',return_value=client),\
                patch('robot_workbench.management.subprocess.run',return_value=SimpleNamespace(returncode=0,stdout=json.dumps(evidence),stderr='')) as reset,\
                patch.object(manager,'camera_action',side_effect=ValueError('no new raw frames')):
            _,job,_=self.start('reset_camera_right',h=h);result=self.wait_job(job,h)
        self.assertEqual(result['error']['code'],'camera_reset_recovery_unknown')
        self.assertEqual(result['detail']['reset']['reset_calls'],1);self.assertEqual(reset.call_count,1)
        self.assertFalse(result['detail']['camera_service']['running']);self.assertEqual(cameras.starts,1)
        self.assertEqual(h['services']['workbench'].stops,0)
        state=manager.snapshot()
        body=dict(action='reset_camera_right',request_id='forged-serial',expected_target=state['target'],arms_supported=False,serial='outside-device')
        status,_,_=self.request('/api/operations/action',body,h=h,headers={'Content-Type':'application/json',
            'X-CSRF-Token':state['csrf'],'X-Workbench-Deployment':state['deployment']['sha256']})
        self.assertEqual(status,409)

    def test_unknown_history_is_read_only_until_explicit_recovery(self):
        _,job,_=self.start('start_cameras');result=self.wait_job(job)
        root=Path(self.h['config']['state_root']);path=root/'operations'/('job-'+result['job_id']+'.json')
        self.h['server'].shutdown();self.h['server'].server_close();self.h['manager'].close()
        result.update(status='running',phase='run',finished_at_ns=None);private_save(path,result)
        restored=self.harness('fake',root=root,services=self.h['services'])
        self.assertEqual(self.start('start_cameras',h=restored)[0],409)
        _,inspection,_=self.start('status_check',h=restored);self.assertEqual(self.wait_job(inspection,restored)['status'],'succeeded')
        self.assertTrue(restored['manager'].unresolved())
        _,recovery,_=self.start('recover_maintenance',h=restored);self.assertEqual(self.wait_job(recovery,restored)['status'],'succeeded')
        self.assertFalse(restored['manager'].unresolved());self.assertEqual(json.loads(path.read_text())['status'],'interrupted_unknown')

    def test_old_callback_must_finish_before_recovery_can_end_window(self):
        self.configure(camera_delay=.8)
        status,job,_=self.start('start_cameras');self.assertEqual(status,200)
        deadline=time.monotonic()+1
        while not self.h['manager'].window and time.monotonic()<deadline:time.sleep(.01)
        self.assertEqual(self.start('recover_maintenance')[0],409)
        result=self.wait_job(job)
        self.assertEqual(result['status'],'unknown')
        self.assertIn('维护收尾尚未确认',result['error']['message'])
        self.assertIsNotNone(self.h['manager'].window)
        time.sleep(.35)
        status,recovery,_=self.start('recover_maintenance');self.assertEqual(status,200,recovery)
        self.assertEqual(self.wait_job(recovery)['status'],'succeeded')

    def test_warning_ack_is_durable_scoped_and_does_not_recover_control(self):
        manager=self.h['manager'];state=manager.snapshot()
        event=dict(id='cpu-heartbeat-fault',reason='SDK owner heartbeat lost',wall_ns=time.time_ns())
        manager.control_alerts.observe(dict(event=event,active_anomaly=True))
        headers={'Content-Type':'application/json','X-CSRF-Token':state['csrf'],
                 'X-Workbench-Deployment':state['deployment']['sha256'],'Origin':'http://127.0.0.1:'+str(self.h['port'])}
        body=dict(event_id=event['id'],environment='fake',device_id='cpu-device')
        before=json_request(self.h['services']['workbench'].port,'/fixture')
        path='/api/operations/acknowledge-control-warning?environment=fake'
        self.assertEqual(self.request(path,body,headers={**headers,'X-CSRF-Token':'wrong'})[0],403)
        self.assertNotEqual(self.request(path,{**body,'device_id':'wrong'},headers=headers)[0],200)
        self.assertEqual(len(manager.snapshot()['control_alerts']['warnings']),1)
        status,result,_=self.request(path,body,headers=headers)
        self.assertEqual(status,200,result);self.assertFalse(result['motion_resumed']);self.assertFalse(result['control_recovered'])
        self.assertEqual(manager.snapshot()['control_alerts']['warnings'],[])
        saved=json.loads((manager.root/'control-alerts.json').read_text())
        self.assertTrue(saved['events'][event['id']]['acknowledged_wall_ns'])
        after=json_request(self.h['services']['workbench'].port,'/fixture')
        self.assertEqual(after['events'],before['events'])
        self.assertEqual(self.h['services']['workbench'].starts,1)

    def test_proxy_preserves_origin_csrf_range_and_media_metadata(self):
        status,state,_=self.request('/api/state?environment=fake');self.assertEqual(status,200)
        status,result,_=self.request('/api/actions/stop',dict(environment='fake',business='inference',request_id='stop-fixture'),
            headers={'Origin':'http://127.0.0.1:'+str(self.h['port']),'X-CSRF-Token':state['csrf'],'Content-Type':'application/json'})
        self.assertEqual(status,200,result);self.assertEqual(result['origin'],'http://127.0.0.1:'+str(self.h['services']['workbench'].port))
        status,body,headers=self.request('/api/result-media?environment=fake',headers={'Range':'bytes=2-5'})
        self.assertEqual((status,body),(206,2345));self.assertEqual(headers['Content-Range'],'bytes 2-5/10')
        self.assertEqual(headers['Accept-Ranges'],'bytes');self.assertEqual(headers['ETag'],'"fixture-video"')
        self.assertIn('X-Frame-Metadata',headers)
        self.assertEqual(self.request('/api/result-media',method='HEAD')[2]['Content-Length'],'10')

    def test_slow_status_poll_does_not_block_independent_stop_or_operations(self):
        state=json_request(self.h['services']['workbench'].port,'/api/state')
        self.configure(state_delay=.8)
        thread=threading.Thread(target=lambda:self.request('/api/state?environment=fake'));thread.start();time.sleep(.08)
        began=time.monotonic()
        status,result,_=self.request('/api/actions/stop',dict(environment='fake',business='inference',request_id='parallel-stop'),
            headers={'Origin':'http://127.0.0.1:'+str(self.h['port']),'X-CSRF-Token':state['csrf'],'Content-Type':'application/json'})
        self.assertEqual(status,200,result);self.assertLess(time.monotonic()-began,.4)
        began=time.monotonic();self.assertEqual(self.request('/api/operations')[0],200);self.assertLess(time.monotonic()-began,.4)
        thread.join(2);self.assertFalse(thread.is_alive())

    def test_bad_framing_duplicate_host_origin_and_ssrf_targets_are_rejected(self):
        host='127.0.0.1:'+str(self.h['port'])
        cases=[('GET /api/state HTTP/1.1',f'Host: {host}\r\nHost: {host}',403),
               ('GET /api/state HTTP/1.1',f'Host: {host}\r\nOrigin: http://{host}\r\nOrigin: http://evil',403),
               ('GET http://127.0.0.1:9/api/state HTTP/1.1',f'Host: {host}',400),
               ('GET //127.0.0.1:9/api/state HTTP/1.1',f'Host: {host}',400),
               ('GET /api/state HTTP/1.1',f'Host: {host}\r\nContent-Length: 1',400),
               ('POST /api/actions/stop HTTP/1.1',f'Host: {host}\r\nTransfer-Encoding: chunked\r\nContent-Length: 2',400),
               ('POST /api/actions/stop HTTP/1.1',f'Host: {host}\r\nExpect: 100-continue\r\nContent-Length: 2',417),
               ('POST /api/actions/stop HTTP/1.1',f'Host: {host}\r\nContent-Length: 2\r\nContent-Length: 2',400),
               ('POST /api/actions/stop HTTP/1.1',f'Host: {host}\r\nContent-Length: 999999999',400)]
        for first,headers,expected in cases:
            with socket.create_connection(('127.0.0.1',self.h['port']),timeout=2) as sock:
                sock.sendall((first+'\r\n'+headers+'\r\n\r\n').encode())
                response=sock.recv(4096);self.assertIn((' '+str(expected)+' ').encode(),response.split(b'\r\n')[0],first)
        self.assertEqual(self.request('/web/../robot_workbench/management.py')[0],404)
        self.assertEqual(self.request('/api/operations?environment=fake&url=http://evil')[0],409)

    def test_connection_reset_and_abandoned_stream_do_not_kill_management(self):
        self.assertEqual(self.request('/fixture/reset')[0],503)
        sock=socket.create_connection(('127.0.0.1',self.h['port']),timeout=2)
        sock.sendall(('GET /fixture/big HTTP/1.1\r\nHost: 127.0.0.1:'+str(self.h['port'])+'\r\n\r\n').encode())
        sock.recv(256);sock.setsockopt(socket.SOL_SOCKET,socket.SO_LINGER,struct.pack('ii',1,0));sock.close()
        self.assertEqual(self.request('/api/operations')[0],200)

    def test_target_ignores_volatile_process_state_and_verify_rejects_same_backend(self):
        before=self.h['manager'].target()
        with self.h['manager'].health_lock:
            self.h['manager'].health_cache['services']['workbench']['identity']['process_state']='other-fixture-state'
        self.assertEqual(self.h['manager'].target(),before)
        result=dict(action='restart_workbench',before=before,backend=before['backend_instance'])
        self.assertFalse(self.h['manager'].verify(result)['ok'])

    def test_claimed_exit_is_rechecked_before_starting_replacement(self):
        service=self.h['services']['workbench'];original=service.snapshot()
        with patch.object(service,'stop',return_value=dict(original,state='stopped',running=False)):
            _,job,_=self.start('restart_workbench');result=self.wait_job(job)
        self.assertEqual(result['status'],'unknown');self.assertEqual(service.starts,1)
        self.assertTrue(service.snapshot()['running'])

    def test_live_model_with_unreadable_status_is_not_stopped(self):
        model=self.h['services']['model'];prior=model.child.pid
        with patch('robot_workbench.management.json_request',wraps=json_request) as read:
            def guarded(port,path,*args,**kwargs):
                if port==model.port and path=='/status':raise ConnectionError('fixture model API unavailable')
                return json_request(port,path,*args,**kwargs)
            read.side_effect=guarded
            _,job,_=self.start('restart_model');self.assertEqual(self.wait_job(job)['status'],'unknown')
        self.assertEqual(model.child.pid,prior);self.assertEqual(model.stops,0)

    def test_stop_cameras_uses_mode_adapter_and_never_waits_for_new_frames(self):
        for h in (self.h,self.harness('physical')):
            with self.subTest(environment=h['manager'].environment):
                self.configure(h=h,connected=True)
                if h['manager'].environment=='physical':
                    cameras=CpuProcessService('cameras','physical');cameras.start();h['services']['cameras']=cameras
                before={name:service.child.pid for name,service in h['services'].items() if name!='cameras'}
                with patch.object(h['manager'],'camera_evidence',side_effect=AssertionError('stop waited for frames')):
                    status,job,_=self.start('stop_cameras',h=h);self.assertEqual(status,200,job)
                    result=self.wait_job(job,h);self.assertEqual(result['status'],'succeeded',result)
                state=json_request(h['services']['workbench'].port,'/fixture')
                self.assertTrue(state['connected'])
                if h['manager'].environment=='fake':self.assertTrue(state['camera_stopped']);self.assertIn('stop_cameras',state['events'])
                else:self.assertEqual(h['services']['cameras'].stops,1)
                self.assertEqual({name:h['services'][name].child.pid for name in before},before)

    def test_stop_model_checks_worker_and_only_invalidates_local_binding(self):
        self.configure(connected=True)
        _,prepared,_=self.start('prepare_model');self.assertEqual(self.wait_job(prepared)['status'],'succeeded')
        model=self.h['services']['model'];backend=self.h['services']['workbench'];backend_pid=backend.child.pid
        camera=json_request(backend.port,'/fixture')['camera_instance']
        with patch.object(self.h['manager'],'camera_evidence',side_effect=AssertionError('model stop touched frames')):
            _,job,_=self.start('stop_model');result=self.wait_job(job)
        self.assertEqual(result['status'],'succeeded',result);self.assertEqual(model.stops,1);self.assertEqual(model.starts,1)
        state=json_request(backend.port,'/fixture')
        self.assertFalse(state['loaded']);self.assertTrue(state['connected']);self.assertEqual(state['camera_instance'],camera)
        self.assertEqual(backend.child.pid,backend_pid);self.assertIn('invalidate_model',state['events'])
        _,job,_=self.start('stop_model');self.assertEqual(self.wait_job(job)['status'],'succeeded')
        self.assertEqual(model.starts,1);self.assertEqual(model.stops,1)
        other=self.harness('fake');self.configure(h=other,service='model',loaded=True,wrong_pid=True)
        _,job,_=self.start('stop_model',h=other);result=self.wait_job(job,other)
        self.assertNotEqual(result['status'],'succeeded');self.assertEqual(other['services']['model'].stops,0)
        self.assertNotIn('invalidate_model',json_request(other['services']['workbench'].port,'/fixture')['events'])
        exited=self.harness('fake');model=exited['services']['model'];original=json_request
        def exited_status(port,path,*args,**kwargs):
            result=original(port,path,*args,**kwargs)
            if port==model.port and path=='/status':result.update(worker_pid=999999999,worker_exit_code=1,loaded=False,status='error')
            return result
        with patch('robot_workbench.management.json_request',side_effect=exited_status),\
                patch.object(model,'track_child',side_effect=AssertionError('already exited worker must not be signalled')):
            _,job,_=self.start('stop_model',h=exited);result=self.wait_job(job,exited)
        self.assertEqual(result['status'],'succeeded',result);self.assertEqual(model.stops,1)

    def test_stop_workbench_requires_real_release_and_preserves_independent_services(self):
        h=self.harness('physical');self.configure(h=h,connected=True)
        camera=CpuProcessService('cameras','physical');camera.start();h['services']['cameras']=camera
        backend=h['services']['workbench'];preserved={name:h['services'][name].child.pid for name in ('model','cameras')}
        self.assertEqual(self.start('stop_workbench',h=h)[0],409)
        _,job,_=self.start('stop_workbench',h=h,support=True);result=self.wait_job(job,h)
        self.assertEqual(result['status'],'succeeded',result);self.assertFalse(backend.snapshot()['running'])
        self.assertEqual(backend.starts,1);self.assertEqual(backend.stops,1);self.assertIn('release',backend.last_events)
        self.assertTrue(result['verification']['evidence']['release']['release_receipt']['sdk_close_returned'])
        self.assertNotIn('ops_maintenance_end',backend.last_events)
        self.assertEqual({name:h['services'][name].child.pid for name in preserved},preserved)
        _,job,_=self.start('stop_workbench',h=h,support=True);self.assertEqual(self.wait_job(job,h)['status'],'succeeded')
        self.assertEqual(backend.starts,1);self.assertEqual(backend.stops,1)
        failed=self.h;self.configure(h=failed,connected=True,release_unconfirmed=True)
        _,job,_=self.start('stop_workbench',h=failed);result=self.wait_job(job,failed)
        self.assertEqual(result['status'],'unknown',result);self.assertEqual(failed['services']['workbench'].stops,0)
        self.assertIsNotNone(failed['manager'].window)

    def test_partial_connect_stop_requires_explicit_same_ticket_inspection(self):
        for recovery in ('recover_maintenance','recover_control'):
            with self.subTest(recovery=recovery):
                h=self.harness('fake');self.configure(h=h,connect_delay=.3)
                _,job,_=self.start('connect_control',h=h)
                deadline=time.monotonic()+2
                while time.monotonic()<deadline:
                    state=json_request(h['services']['workbench'].port,'/fixture')
                    if (state.get('maintenance') or {}).get('active_action')=='connect_control':break
                    time.sleep(.01)
                self.assertEqual((state.get('maintenance') or {}).get('active_action'),'connect_control')
                status,_,_=self.request('/api/operations/stop',dict(expected_device_id='cpu-device',request_id='race-stop'),h=h,
                    headers={'Content-Type':'application/json','X-CSRF-Token':h['manager'].csrf,'Origin':'http://127.0.0.1:'+str(h['port'])})
                self.assertEqual(status,200);self.assertEqual(self.wait_job(job,h)['status'],'unknown')
                self.assertIsNone(h['manager'].window['expected_target']['runtime_nonce'])
                events=json_request(h['services']['workbench'].port,'/fixture')['events']
                self.assertNotIn('inspect',events);self.assertNotIn('ops_maintenance_end',events)
                self.configure(h=h,inspect_runtime_same=False)
                _,bad,_=self.start(recovery,h=h);self.assertEqual(self.wait_job(bad,h)['status'],'unknown')
                self.assertIsNone(h['manager'].window['expected_target']['runtime_nonce'])
                self.configure(h=h,inspect_runtime_same=True)
                _,resolved,_=self.start(recovery,h=h);result=self.wait_job(resolved,h)
                self.assertEqual(result['status'],'succeeded',result);self.assertIsNone(h['manager'].window)
                state=json_request(h['services']['workbench'].port,'/fixture')
                self.assertEqual(state['events'].count('connect_control'),1);self.assertNotIn('recover_control',state['events'])
                self.assertIn('inspect',state['events']);self.assertEqual(state['events'].count('ops_maintenance_end'),1)
                self.assertTrue(state['connected']);self.assertFalse(result['result']['motion_resumed'])
                if recovery=='recover_control':self.assertFalse(result['result']['control']['sdk_reconnected'])

    def test_control_connection_observes_first_and_checks_release_and_hold_receipts(self):
        self.configure(stale_frames=True)
        _,job,_=self.start('connect_control');self.assertEqual(self.wait_job(job)['status'],'failed')
        self.assertNotIn('connect_control',json_request(self.h['services']['workbench'].port,'/fixture')['events'])
        self.configure(stale_frames=False)
        _,job,_=self.start('connect_control');self.assertEqual(self.wait_job(job)['status'],'succeeded')
        _,job,_=self.start('verify_hold');result=self.wait_job(job);self.assertEqual(result['status'],'succeeded',result)
        self.assertTrue(result['verification']['evidence']['hold']['stop_receipt']['stop_hold_return_confirmed'])
        self.configure(hold_invalid=True)
        _,job,_=self.start('verify_hold');self.assertEqual(self.wait_job(job)['status'],'failed')
        _,job,_=self.start('release_control');result=self.wait_job(job);self.assertEqual(result['status'],'succeeded',result)
        self.assertTrue(result['verification']['evidence']['release']['sdk_close_returned'])

    def test_fixed_role_restarts_verify_identity_and_do_not_claim_collection_ready(self):
        for environment in ('fake','physical'):
            h=self.h if environment=='fake'else self.harness(environment)
            services={role:dict(status='ready',pid=100+i,process_start_ticks=100+i,model_initialized=True)
                      for i,role in enumerate(('detector','planner','scene'))}
            self.configure(h=h,connected=True,collection_services=services,collection_ready=True)
            for role in services:
                with self.subTest(environment=environment,role=role):
                    before=json_request(h['services']['workbench'].port,'/fixture')['collection_services']
                    status,job,_=self.start('restart_'+role,h=h);self.assertEqual(status,200,job)
                    result=self.wait_job(job,h);self.assertEqual(result['status'],'succeeded',result)
                    after=json_request(h['services']['workbench'].port,'/fixture')['collection_services']
                    for sibling in services:
                        if sibling!=role:self.assertEqual(before[sibling],after[sibling])
                    self.assertEqual(after[role]['pid'],before[role]['pid']+10)
                    self.assertFalse(result['verification']['health']['collection']['ready'])
                    self.assertTrue(result['result']['collection']['needs_warmup'])
            self.configure(h=h,bad_role_proof=True)
            _,job,_=self.start('restart_planner',h=h);self.assertEqual(self.wait_job(job,h)['status'],'failed')

    def test_unknown_independent_services_do_not_block_scoped_operations_or_turn_green(self):
        h,cameras,client=self.camera_reset_fixture();model=h['services']['model']
        unknown=dict(model.snapshot(),state='unknown',owned=False,reason='fixture /proc unreadable')
        with patch.object(model,'snapshot',return_value=unknown),patch('robot_workbench.rgbd_service.CommonCameraService.connect',return_value=client):
            _,job,_=self.start('restart_cameras',h=h);result=self.wait_job(job,h)
            self.assertEqual(result['status'],'succeeded',result)
            proof=result['verification']['evidence']
            self.assertEqual(proof['independent_services']['model']['status'],'not_verified')
            self.assertIsNone(proof['independent_services_preserved'])
            self.assertEqual({row['service'] for row in result['result']['dependencies']},{'workbench','cameras'})
        self.assertEqual(model.starts,1);self.assertEqual(model.stops,0)
        unknown=dict(cameras.snapshot(),state='unknown',owned=False)
        with patch.object(cameras,'snapshot',return_value=unknown):
            _,job,_=self.start('prepare_model',h=h);result=self.wait_job(job,h)
            self.assertEqual(result['status'],'succeeded',result)
            self.assertEqual(result['verification']['evidence']['independent_services']['cameras']['status'],'not_verified')
        self.assertEqual(cameras.starts,2);self.assertEqual(cameras.stops,1)

    def test_unknown_target_cannot_start_second_instance_and_unknown_workbench_keeps_stop(self):
        h,cameras,_=self.camera_reset_fixture();unknown=dict(cameras.snapshot(),state='unknown',owned=False)
        # This is the existing ManagedService contract: unreadable identity returns unknown, with no spawn.
        with patch.object(cameras,'snapshot',return_value=unknown),patch.object(cameras,'start',return_value=unknown) as start:
            _,job,_=self.start('start_cameras',h=h);result=self.wait_job(job,h)
            self.assertEqual(result['status'],'unknown',result);self.assertEqual(start.call_count,1)
            self.assertEqual(cameras.starts,1);self.assertEqual(cameras.stops,0)
        manager=self.h['manager'];backend=self.h['services']['workbench']
        with patch.object(backend,'snapshot',return_value=dict(backend.snapshot(),state='unknown',owned=False)):
            status,error,_=self.start('prepare_model');self.assertEqual(status,409,error)
            _,job,_=self.start('status_check');self.assertEqual(self.wait_job(job)['status'],'succeeded')
            self.assertTrue(manager.stop(dict(expected_device_id='cpu-device',request_id='unknown-stop'))['receipt']['stopped'])
        self.assertEqual(backend.stops,0)

    def test_recovery_uses_ticket_scope_and_actual_reconnect_branch(self):
        h,cameras,_=self.camera_reset_fixture();manager=h['manager'];model=h['services']['model']
        with manager.maintenance('start_cameras'):
            with patch.object(model,'snapshot',return_value=dict(model.snapshot(),state='unknown',owned=False)):
                manager.refresh();self.assertIsNone(manager.check('recover_maintenance'))
            with patch.object(cameras,'snapshot',return_value=dict(cameras.snapshot(),state='unknown',owned=False)):
                manager.refresh();self.assertIn('作用域',manager.check('recover_maintenance'))
            window=deepcopy(manager.window);legacy=deepcopy(window)
            legacy['begin_body']['purpose']='prepare_environment';legacy.pop('affected_services')
            manager.window=legacy
            with patch.object(model,'snapshot',return_value=dict(model.snapshot(),state='unknown',owned=False)):
                manager.refresh();self.assertIn('作用域',manager.check('recover_maintenance'))
            manager.window=dict(window,affected_services=['arbitrary-service'])
            self.assertIn('作用域损坏',manager.check('recover_maintenance'));manager.window=window
        with patch.object(cameras,'snapshot',return_value=dict(cameras.snapshot(),state='unknown',owned=False)):
            status,job,_=self.start('recover_control',h=h,support=True);self.assertEqual(status,200,job);result=self.wait_job(job,h)
            self.assertEqual(result['status'],'succeeded',result)
            self.assertTrue(result['result']['control']['already_operable'])
            self.assertEqual({r['service'] for r in result['result']['dependencies']},{'workbench'})
            self.configure(h=h,connected=False);manager.refresh()
            self.assertEqual(manager.service_dependencies('recover_control',manager.health_cache),{'workbench','cameras'})
        self.assertEqual(cameras.starts,1);self.assertEqual(cameras.stops,0)

    def test_role_restart_ignores_unrelated_unknown_and_rejects_legacy_profile(self):
        model=self.h['services']['model']
        self.configure(connected=True)
        status,error,_=self.start('restart_planner');self.assertEqual(status,409,error)
        self.assertIn('固定配置',error['error'])
        services={role:dict(status='unknown' if role=='scene' else 'ready',pid=100+i,process_start_ticks=100+i)
            for i,role in enumerate(('detector','planner','scene'))}
        self.configure(collection_services=services)
        with patch.object(model,'snapshot',return_value=dict(model.snapshot(),state='unknown',owned=False)):
            _,job,_=self.start('restart_planner');result=self.wait_job(job)
            self.assertEqual(result['status'],'succeeded',result)
            self.assertFalse(result['result']['collection']['ready'])
            self.assertEqual(result['verification']['evidence']['independent_services']['model']['status'],'not_verified')
        after=json_request(self.h['services']['workbench'].port,'/fixture')['collection_services']
        self.assertEqual(after['scene'],services['scene']);self.assertEqual(model.stops,0)

    def test_undeclared_resident_role_is_explicit_and_does_not_prepare(self):
        services={role:dict(status='ready',pid=100+i,process_start_ticks=100+i)
            for i,role in enumerate(('detector','planner'))}
        self.configure(connected=True,collection_services=services)
        status,error,_=self.start('restart_scene')
        self.assertEqual(status,409);self.assertIn('未启用常驻场景服务',error['error'])
        self.assertEqual(json_request(self.h['services']['workbench'].port,'/fixture')['collection_services'],services)

    def test_recovery_classifies_healthy_fault_disconnected_and_unknown_without_model_loading(self):
        self.configure(connected=True)
        _,job,_=self.start('recover_control');result=self.wait_job(job);self.assertEqual(result['status'],'succeeded',result)
        self.assertTrue(result['result']['control']['already_operable'])
        self.assertEqual(json_request(self.h['services']['workbench'].port,'/fixture')['events'],[])
        self.configure(recoverable_fault='tracking')
        _,job,_=self.start('recover_control');result=self.wait_job(job);self.assertEqual(result['status'],'succeeded',result)
        self.assertTrue(result['result']['control']['recovered']);self.assertFalse(result['result']['control']['motion_resumed'])
        for cold in (False,True):
            with self.subTest(cold=cold):
                h=self.harness('fake');backend=h['services']['workbench']
                if cold:backend.stop()
                _,job,_=self.start('recover_control',h=h);result=self.wait_job(job,h)
                self.assertEqual(result['status'],'succeeded',result);self.assertTrue(result['result']['control']['reconnected'])
                state=json_request(backend.port,'/fixture');self.assertTrue(state['connected'])
                self.assertIn('ops_start_cameras',state['events']);self.assertNotIn('ops_prepare_model',state['events'])
                self.assertFalse(json_request(h['services']['model'].port,'/fixture')['loaded'])
        unknown=self.harness('fake');self.configure(h=unknown,reset_state=True)
        _,job,_=self.start('recover_control',h=unknown);result=self.wait_job(job,unknown)
        self.assertEqual(result['status'],'unknown',result)
        self.assertEqual(unknown['services']['workbench'].starts,1);self.assertEqual(unknown['services']['workbench'].stops,0)

    def test_physical_in_place_recovery_needs_no_support_but_never_reconnects_without_it(self):
        h=self.harness('physical');self.configure(h=h,connected=True,recoverable_fault='confirmed software cancellation')
        manager=h['manager'];manager.refresh()
        self.assertFalse(manager.snapshot()['operations']['recover_control']['requires_support'])
        status,job,_=self.start('recover_control',h=h,support=False);self.assertEqual(status,200,job)
        result=self.wait_job(job,h);self.assertEqual(result['status'],'succeeded',result)
        self.assertTrue(result['result']['control']['recovered'])
        events=json_request(h['services']['workbench'].port,'/fixture')['events']
        self.assertIn('recover_control',events);self.assertNotIn('release',events);self.assertNotIn('connect_control',events)
        self.configure(h=h,connected=False);manager.refresh()
        self.assertTrue(manager.snapshot()['operations']['recover_control']['requires_support'])
        status,result,_=self.start('recover_control',h=h,support=False);self.assertNotEqual(status,200)
        self.assertEqual(result.get('code'),'support_required',result)

    def test_in_place_recovery_does_not_fall_back_to_release_after_conditions_change(self):
        h=self.harness('physical')
        self.configure(h=h,connected=True,recoverable_fault='confirmed software cancellation',recovery_changes_after_begin=True)
        status,job,_=self.start('recover_control',h=h,support=False);self.assertEqual(status,200,job)
        result=self.wait_job(job,h);self.assertEqual(result['status'],'failed',result)
        self.assertEqual(result['error']['code'],'support_required')
        events=json_request(h['services']['workbench'].port,'/fixture')['events']
        for name in ('release','connect_control','recover_control','ops_start_cameras'):self.assertNotIn(name,events)

    def test_legacy_session_routes_cannot_reach_a_second_lifecycle_writer(self):
        state=self.h['manager'].snapshot();before=json_request(self.h['services']['workbench'].port,'/fixture')['events']
        for action in ('start','stop'):
            status,value,_=self.request('/api/session/'+action,{},headers={'Content-Type':'application/json',
                'X-CSRF-Token':state['csrf'],'X-Workbench-Deployment':state['deployment']['sha256']})
            self.assertEqual(status,409,value);self.assertEqual(value['code'],'operation_required')
        status,value,_=self.request('/api/session');self.assertEqual(status,200);self.assertTrue(value['managed']);self.assertEqual(value['allowed'],{})
        self.assertEqual(json_request(self.h['services']['workbench'].port,'/fixture')['events'],before)
        self.assertEqual(self.h['services']['workbench'].starts,1);self.assertEqual(self.h['services']['workbench'].stops,0)

    def test_idle_gpu_profile_requires_explicit_model_stop_without_cold_backend_deadlock(self):
        h=self.harness('fake',config_overrides={'backend_requires_idle_gpu':True})
        manager=h['manager'];backend=h['services']['workbench'];model=h['services']['model']
        self.configure(h=h,service='model',loaded=True,warmup=True)
        status,error,_=self.start('restart_workbench',h=h)
        self.assertEqual(status,409,error);self.assertIn('空闲GPU',error['error'])
        self.assertIn('先显式停止模型',manager.snapshot()['operations']['restart_workbench']['impact'])
        self.assertEqual(backend.stops,0);self.assertEqual(model.stops,0)
        backend.stop()
        for action in ('prepare_environment','connect_control','recover_control','start_cameras','restart_cameras'):
            status,error,_=self.start(action,h=h);self.assertEqual(status,409,error)
            self.assertIn('停止模型',error['error'])
        self.assertEqual(backend.starts,1)
        _,job,_=self.start('stop_model',h=h);result=self.wait_job(job,h)
        self.assertEqual(result['status'],'succeeded',result)
        self.assertEqual(result['result']['stopped_model']['binding']['status'],'not_applicable')
        self.assertEqual(model.stops,1);self.assertEqual(model.starts,1);self.assertEqual(backend.starts,1)
        _,job,_=self.start('connect_control',h=h);result=self.wait_job(job,h)
        self.assertEqual(result['status'],'succeeded',result);self.assertEqual(backend.starts,2);self.assertEqual(model.starts,1)
        self.assertTrue(json_request(backend.port,'/fixture')['connected'])

    def test_independent_camera_stop_after_known_backend_exit_does_not_start_backend(self):
        h=self.harness('physical');backend=h['services']['workbench']
        cameras=CpuProcessService('cameras','physical');cameras.start();h['services']['cameras']=cameras
        backend.stop()
        _,job,_=self.start('stop_cameras',h=h);result=self.wait_job(job,h)
        self.assertEqual(result['status'],'succeeded',result);self.assertTrue(result['result']['backend_absent'])
        self.assertEqual(cameras.stops,1);self.assertEqual(backend.starts,1);self.assertEqual(h['services']['model'].stops,0)
        self.configure(reset_state=True)
        self.assertEqual(self.start('stop_model')[0],409)
        self.assertEqual(self.h['services']['model'].stops,0)

    def test_camera_start_and_restart_restore_only_an_exited_backend_without_connecting(self):
        for action in ('start_cameras','restart_cameras'):
            with self.subTest(action=action):
                h=self.harness('fake');backend=h['services']['workbench'];backend.stop()
                status,job,_=self.start(action,h=h);self.assertEqual(status,200,job)
                result=self.wait_job(job,h);self.assertEqual(result['status'],'succeeded',result)
                self.assertIn('observation_backend_start',result['result'])
                value=json_request(backend.port,'/fixture');self.assertFalse(value['connected'])
                self.assertNotIn('ops_connect',value['events']);self.assertNotIn('connect',value['events'])
                self.assertNotIn('ops_prepare_model',value['events'])
                self.assertEqual(value['events'].count('ops_'+action),1)
                self.assertEqual(h['services']['model'].stops,0)
                self.configure(h=h,reset_state=True)
                self.assertEqual(self.start(action,h=h)[0],409,'unknown live backend is never replaced')


class CameraSourceStopTest(unittest.TestCase):
    def test_source_stop_does_not_claim_the_plant_process_exited(self):
        from robot_workbench.management import _camera_stop_verified
        source=dict(stopped=True,exit_verified=False,source_available=False,process_preserved=True)
        self.assertTrue(_camera_stop_verified(source))
        self.assertTrue(_camera_stop_verified(dict(stopped=True,exit_verified=True)))
        for changed in ({'source_available':True},{'process_preserved':False},{'stopped':False},{'exit_verified':None}):
            self.assertFalse(_camera_stop_verified(dict(source,**changed)))


class RosCameraManagementTest(unittest.TestCase):
    def test_readiness_uses_registered_profile_ros_and_closes_only_its_subscriber(self):
        with TemporaryDirectory()as temp:
            root=Path(temp);profile=dict(environment='physical',robot_id='cpu',
                data_plane=dict(schema='workbench.ros-data-plane.v1',namespace='/cpu',domain_id=126),cameras={})
            for i,role in enumerate(('head','left','right')):
                path=root/(role+'.json');path.write_text(json.dumps(dict(serial=str(i),width=2,height=2)))
                profile['cameras'][role]=dict(serial=str(i),intrinsics=str(path))
            path=root/'profile.json';path.write_text(json.dumps(profile))
            manager=object.__new__(WorkbenchManager);manager.environment='physical';manager.timeouts={'camera':.1}
            manager.config=dict(device_id='cpu',publication_files={'physical_profile':str(path)},native_rgb={'directory':'unused'})
            manager.services={'cameras':Mock()};manager.service_action=Mock(return_value={'state':'running'})
            manager.post=Mock(return_value={'reconnected':True});view=Mock()
            with patch('robot_workbench.ros_inputs.RosCameraView',return_value=view)as create,\
                    patch('robot_workbench.rgbd_service.CommonCameraService.connect',side_effect=AssertionError('no IPC')),\
                    patch('robot_workbench.prometheus_rgb.RGBClient',side_effect=AssertionError('no raw files')):
                result=manager.camera_action('start','token',lambda *a,**k:None)
            self.assertTrue(result['binding']['reconnected']);view.close.assert_called_once()
            self.assertEqual(view.latest_rgbd.call_count,3);self.assertEqual(create.call_args.args[0],profile['data_plane'])
            manager.service_action.reset_mock()
            profile['robot_id']='another-device';path.write_text(json.dumps(profile))
            with self.assertRaisesRegex(ValueError,'another device'):
                manager.camera_action('start','token',lambda *a,**k:None)
            manager.service_action.assert_not_called()


if __name__=='__main__':
    if len(sys.argv)>1 and sys.argv[1]=='--http-fixture':
        cpu_fixture(sys.argv[2],int(sys.argv[3]),sys.argv[4],int(sys.argv[5]))
    else:unittest.main()
