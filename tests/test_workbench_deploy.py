"""Startup fault matrix: no robot SDK, no CAN, no real service changes."""
import importlib.util
import json
import os
from pathlib import Path
import socket
import sys
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import patch

script=Path(__file__).resolve().parent/'workbench_deploy.py'
if not script.is_file():script=Path(__file__).resolve().parents[1]/'scripts/workbench_deploy.py'
spec=importlib.util.spec_from_file_location('deploy',script)
d=importlib.util.module_from_spec(spec);spec.loader.exec_module(d)


class Unit:
    def __init__(self,role,events,running=False,fail=False):
        self.role,self.events,self.running,self.fail=role,events,running,fail
    def snapshot(self):
        return dict(state='running' if self.running else 'stopped',owned=True,identity=dict(pid=42))
    def start(self):
        self.events.append(self.role)
        if self.fail:return dict(state='unknown',owned=False)
        self.running=True;return self.snapshot()


def observation(state='DISCONNECTED',connected=False):
    return dict(runtime=dict(connected=connected,state=state,runtime_nonce='live' if connected else None,
        holder='research' if connected else None,command_epoch=3,automatic_enabled=False,feedback_healthy=connected,stationary=connected),
        backend_instance='test',cameras_ready=True,target_model_ready=True,manual_ready=True,
        active_job=False,maintenance=False,business_busy=False)


class StartupTest(unittest.TestCase):
    def test_camera_rebind_recovers_stale_consumers_once_without_starting_hardware(self):
        stale=dict(media=[dict(id=role,ready=False,error='source unavailable',frame_id='old-frame')for role in ('head','left','right')])
        ready=dict(media=[dict(id=role,ready=True)for role in ('head','left','right')])
        environment=dict(operations={'rebind_cameras':{'allowed':True}},target={'backend_instance':'same'},
            csrf='fixture',deployment={'sha256':'a'*64})
        for outcome in ('succeeded','failed','denied','still_unready'):
            current=json.loads(json.dumps(environment))
            if outcome=='denied':current['operations']['rebind_cameras']=dict(allowed=False,disabled_reason='business is active')
            result=dict(status='failed'if outcome=='failed'else'succeeded',verification={'ok':outcome!='failed'})
            with self.subTest(outcome=outcome),patch.object(d,'api',side_effect=[current,result,stale if outcome=='still_unready'else ready]),\
                    patch.object(d,'http_json',return_value={'job_id':'one'})as post:
                if outcome=='succeeded':self.assertEqual(d.ensure_camera_inputs(self.m,state=stale),ready)
                else:
                    with self.assertRaises(d.Blocked):d.ensure_camera_inputs(self.m,state=stale)
                self.assertEqual(post.call_count,0 if outcome=='denied'else 1)
                if post.called:
                    body=post.call_args.args[1];self.assertEqual(body['action'],'rebind_cameras');self.assertFalse(body['arms_supported'])

    def test_initial_camera_discovery_waits_and_healthy_binding_is_reused(self):
        waiting=dict(media=[dict(id=role,ready=False,frame_id=None)for role in ('head','left','right')])
        ready=dict(media=[dict(id=role,ready=True)for role in ('head','left','right')])
        with patch.object(d,'api',return_value=ready)as read,patch.object(d,'http_json')as post,patch.object(d.time,'sleep'):
            self.assertEqual(d.ensure_camera_inputs(self.m,state=waiting),ready);read.assert_called_once();post.assert_not_called()
        with patch.object(d,'api')as read,patch.object(d,'http_json')as post:
            self.assertEqual(d.ensure_camera_inputs(self.m,state=ready),ready);read.assert_not_called();post.assert_not_called()

    def test_sealed_start_repairs_invalid_binding_in_an_existing_business(self):
        stale=observation();stale['cameras_ready']=False
        with patch.object(d,'observe',side_effect=[stale,stale,observation()]),patch.object(d,'ensure_camera_inputs',create=True)as rebind:
            result=d.start(self.m,self.units(True))
        self.assertTrue(result['checks']['cameras_ready']);rebind.assert_called_once();self.assertEqual(self.events,[])

    def test_native_camera_uses_its_live_registered_recording_descriptor(self):
        self.assertEqual(d.camera_descriptor({'native_rgb':{'directory':'/dev/shm/cpu'}},{}),'/dev/shm/cpu/recording-camera-service.json')
        self.assertEqual(d.camera_descriptor({}, {'camera_service_descriptor':'/registered/rgbd.json'}),'/registered/rgbd.json')
        self.assertIsNone(d.camera_descriptor({},{}))

    def test_live_camera_owner_descriptor_is_not_a_static_release_input(self):
        with TemporaryDirectory()as tmp:
            root=Path(tmp);entry=root/'serve_prometheus_rgb.py';entry.write_text('pass\n')
            live=root/'owner.json';live.write_text('{}');profile=root/'profile.json';profile.write_text('{}')
            self.assertEqual(list(d.command_inputs(['/usr/bin/python3',str(entry),'--owner',str(live),'--profile',str(profile)])),[str(entry),str(profile)])
            camera=root/'camera_owner.py';camera.write_text('pass\n')
            self.assertEqual(list(d.command_inputs(['/usr/bin/python3',str(camera),'--observe-existing','--state',str(live)])),[str(camera)])

    def test_seal_existing_user_unit_model_keeps_endpoint_and_credential_binding(self):
        with TemporaryDirectory()as tmp:
            root=Path(tmp);unit=root/'model.service';script=root/'serve_mixed366_model.py';credential=root/'model-token'
            unit.write_text(f'ExecStart=/usr/bin/python3 {script} --registry {root}/registry.json --token-file {credential} --port 18925\n')
            spec=dict(kind='user_unit',unit='robot-workbench-model.service',expected_script=str(script),identity_path=str(root/'identity.json'))
            manager=dict(model_port=18925,services={'model':spec});business=dict(inference=dict(model_endpoint='http://127.0.0.1:18925',model_token_file=str(credential)))
            with patch.object(d,'unit_pin',return_value=dict(unit=spec['unit'],files={str(unit):'a'*64})):
                self.assertEqual(d.seal_model_service(manager,business)['kind'],'user_unit')
                for body in (dict(business,inference=dict(business['inference'],model_token_file='/different')),dict(business,inference=dict(business['inference'],model_endpoint='http://127.0.0.1:18926'))):
                    with self.assertRaises(d.Blocked):d.seal_model_service(manager,body)

    def setUp(self):
        self.temp=TemporaryDirectory();self.addCleanup(self.temp.cleanup);self.root=Path(self.temp.name)
        self.m=dict(root=str(self.root),device_id='arx13',url='http://10.17.32.3:18868',
            owner_descriptor=str(self.root/'owner/control-owner.json'),camera_descriptor=str(self.root/'camera.json'),
            conflicts=[],services={k:dict(unit=k+'.service',expected_script='/none/'+k+'.py')for k in d.ORDER})
        d.private_write(self.m['camera_descriptor'],dict(parent_pid=42))
        Path(self.m['owner_descriptor']).parent.mkdir(mode=0o700)
        d.private_write(self.m['owner_descriptor'],dict(identity=dict(pid=42)))
        self.events=[]
    def units(self,running=False):return {k:Unit(k,self.events,running)for k in d.ORDER}
    def test_recording_starts_before_consumers_and_checks_its_own_pid(self):
        units=self.units();units['recording']=Unit('recording',self.events)
        self.m['services']['recording']=dict(unit='recording.service',expected_script='/none/recording.py')
        with patch.dict(os.environ,WORKBENCH_RECORDING_SERVICE='/registered/service.json'),patch.object(
                d,'observe',return_value=observation()),patch.object(d,'old_owner_files'),patch(
                'robot_workbench.control_owner.ControlChannel') as channel,patch(
                'robot_workbench.recording_process.recording_call',return_value=dict(ready=True,pid=42)) as call:
            channel.return_value.call.return_value={'connected':False}
            self.assertTrue(d.start(self.m,units)['ok'])
        self.assertEqual(self.events,['recording',*d.ORDER]);call.assert_called_with('/registered/service.json','inspect',timeout=2)
    def test_cold_start_order_and_new_owner_is_inert(self):
        units=self.units()
        with patch.object(d,'observe',return_value=observation()),patch.object(d,'old_owner_files')as cleanup,patch('robot_workbench.control_owner.ControlChannel')as channel:
            channel.return_value.call.return_value={'connected':False}
            result=d.start(self.m,units)
        self.assertTrue(result['ok']);self.assertEqual(self.events,list(d.ORDER));self.assertEqual(result['sdk_acquisitions'],0)
        channel.return_value.call.assert_called_once_with('inspect');self.assertEqual(cleanup.call_count,2);cleanup.assert_called_with(self.m,apply=True)

    def test_ros_cold_start_uses_live_frames_without_legacy_camera_descriptor(self):
        self.m['camera_descriptor']=None
        with patch.object(d,'observe',return_value=observation()),patch.object(d,'old_owner_files'),\
                patch('robot_workbench.control_owner.ControlChannel') as channel:
            channel.return_value.call.return_value={'connected':False}
            result=d.start(self.m,self.units())
        self.assertTrue(result['checks']['cameras_ready'])
        self.assertEqual(result['sdk_acquisitions'],0)
    def test_repeated_start_preserves_live_control_without_lifecycle_calls(self):
        with patch.object(d,'observe',return_value=observation('READY',True)):
            result=d.start(self.m,self.units(True))
        self.assertEqual(self.events,[]);self.assertTrue(result['ok']);self.assertEqual(result['control']['runtime_nonce'],'live')

    def test_missing_business_is_started_without_reacquiring_existing_idle_owner(self):
        units=self.units(True);units['workbench'].running=False
        live=observation('STOPPED_DAMPING_HOLD',True)
        with patch.object(d,'observe',return_value=live),patch('robot_workbench.control_owner.ControlChannel')as channel:
            channel.return_value.call.return_value=live['runtime']
            result=d.start(self.m,units)
        self.assertEqual(self.events,['workbench']);self.assertEqual(result['control']['runtime_nonce'],'live')
        self.assertEqual([x.args[0]for x in channel.return_value.call.call_args_list],['inspect','snapshot'])

    def test_configured_triton_is_started_before_manager_and_requires_readiness(self):
        self.m['services']['triton_control']=dict(unit='triton.service',expected_script='/none/triton.py')
        units=self.units();units['triton_control']=Unit('triton_control',self.events)
        live=observation();live['triton_ready']=False
        with patch.object(d,'old_owner_files'),patch('robot_workbench.control_owner.ControlChannel')as channel,patch.object(d,'observe',return_value=live):
            channel.return_value.call.return_value={'connected':False}
            with self.assertRaisesRegex(d.Blocked,'Triton tools did not become ready'):
                d.start(self.m,units,timeout=.01)
        self.assertEqual(self.events,['control_owner','cameras','workbench','triton_control','manager'])
        live['triton_ready']=True
        with patch.object(d,'observe',return_value=live):
            result=d.start(self.m,units)
        self.assertTrue(result['ok']);self.assertEqual(result['actions'],[]);self.assertEqual(result['sdk_acquisitions'],0)
    def test_fault_stays_visible_no_reconnect_or_restart(self):
        with patch.object(d,'observe',return_value=observation('FAULT_STOP_UNCONFIRMED')):
            result=d.start(self.m,self.units(True))
        self.assertTrue(result['ok']);self.assertTrue(result['services_ready']);self.assertFalse(result['control_ready']);self.assertEqual(self.events,[])

    def test_model_load_required_and_idempotent_start_preserves_controller(self):
        self.m['model_port']=18925;self.m['services']['model']=dict(kind='docker_model')
        units=self.units(True);units['model']=Unit('model',self.events)
        live=observation('READY',True);live['model_ready']=False
        ready=json.loads(json.dumps(live));ready['model_ready']=True;ready['runtime']['command_epoch']+=1
        with patch.object(d,'observe',side_effect=[live,live,ready]),patch.object(d,'warm_model',return_value={'reused':False}):
            result=d.start(self.m,units)
        self.assertEqual(self.events,['model']);self.assertTrue(result['model_ready']);self.assertEqual(result['control']['runtime_nonce'],'live')
        with patch.object(d,'observe',return_value=ready),patch.object(d,'warm_model',return_value={'reused':True}):
            self.assertEqual(d.start(self.m,units)['actions'],[])

    def test_model_prepare_submits_once_and_does_not_turn_failure_into_success(self):
        self.m['model_port']=18925
        env=dict(health={'backend_reachable':True},model={'ready':False},operations={'prepare_model':{'allowed':True}},target={'device_id':'arx13'},csrf='fixture',deployment={'sha256':'a'*64})
        with patch.object(d,'api',side_effect=[env,{'status':'failed','error':{'message':'load failed'}}]),patch.object(d,'http_json',return_value={'job_id':'one'})as post:
            with self.assertRaisesRegex(d.Blocked,'load failed'):d.warm_model(self.m)
        post.assert_called_once();self.assertEqual(post.call_args.args[1]['action'],'prepare_model')
        self.assertFalse(post.call_args.args[1]['arms_supported'])

    def test_model_warmup_waits_for_initial_manager_observation(self):
        env=dict(health={'backend_reachable':True},model={'ready':False},operations={'prepare_model':{'allowed':True}},target={'device_id':'arx13'},csrf='fixture',deployment={'sha256':'a'*64})
        initial=dict(env,health={'backend_reachable':False},operations={'prepare_model':{'allowed':False}})
        with patch.object(d,'api',side_effect=[initial,env,{'status':'succeeded','verification':{'ok':True}}]),patch.object(d,'http_json',return_value={'job_id':'one'})as post,patch.object(d.time,'sleep'):
            self.assertEqual(d.warm_model(self.m)['job_id'],'one')
        post.assert_called_once()

    def test_model_warmup_never_uses_previous_container_identity(self):
        env=dict(health={'backend_reachable':True},model={'ready':False},operations={'prepare_model':{'allowed':True}},target={'services':{'model':{'pid':2}}},csrf='fixture',deployment={'sha256':'a'*64})
        old=dict(env,model={'ready':True},target={'services':{'model':{'pid':1}}})
        with patch.object(d,'api',side_effect=[old,env,{'status':'succeeded','verification':{'ok':True}}]),patch.object(d,'http_json',return_value={'job_id':'new'})as post,patch.object(d.time,'sleep'):
            d.warm_model(self.m,expected_model={'pid':2})
        self.assertEqual(post.call_args.args[1]['expected_target']['services']['model']['pid'],2)

    def test_model_ready_from_previous_business_is_not_reused(self):
        old=dict(health={'backend_reachable':True},model={'ready':True},target={'backend_instance':'old'})
        ready=dict(old,target={'backend_instance':'new'})
        with patch.object(d,'api',side_effect=[old,ready])as read,patch.object(d,'http_json')as post,patch.object(d.time,'sleep'):
            self.assertTrue(d.warm_model(self.m,expected_backend='new')['reused'])
        self.assertEqual(read.call_count,2);post.assert_not_called()
    def test_partial_start_failure_does_not_repeat_or_stop_other_services(self):
        units=self.units();units['cameras'].fail=True
        with patch.object(d,'old_owner_files'),patch('robot_workbench.control_owner.ControlChannel')as channel:
            channel.return_value.call.return_value={'connected':False}
            with self.assertRaisesRegex(d.Blocked,'startup not confirmed'):d.start(self.m,units)
        self.assertEqual(self.events,['control_owner','cameras'])
    def test_running_action_and_unknown_identity_block_before_any_start(self):
        live=observation('READY',True);live['business_busy']=True
        with patch.object(d,'observe',return_value=live):
            with self.assertRaises(d.Blocked):d.start(self.m,self.units(True))
        units=self.units();units['control_owner'].snapshot=lambda:dict(state='unknown')
        with self.assertRaises(d.Blocked):d.start(self.m,units)
        self.assertEqual(self.events,[])
    def test_collection_handoff_requires_explicit_flag_and_idle_and_no_control_fault(self):
        self.m['conflicts']=[dict(unit='manual.service',status_url='http://127.0.0.1:18980/api/status')]
        active=dict(ActiveState='active',MainPID='7',ControlPID='0')
        with patch.object(d,'unit_state',return_value=active),patch.object(d,'http_json',return_value=dict(manager_alive=True,collect_status='recording')),patch.object(d.subprocess,'run')as run:
            for handoff in (False,True):
                with self.assertRaises(d.Blocked):d.start(self.m,self.units(),handoff=handoff)
            run.assert_not_called()
        with patch.object(d,'unit_state',return_value=active),patch.object(d,'observe',return_value=observation('FAULT_STOP_UNCONFIRMED')),patch.object(d.subprocess,'run')as run:
            with self.assertRaises(d.Blocked):d.start(self.m,self.units(True),handoff=True)
            run.assert_not_called()
    def test_idle_collection_handoff_is_only_declared_stop_then_ordered_start(self):
        pin=dict(unit='manual.service',files={'/unit':'a'*64},status_url='http://127.0.0.1:18980/api/status');self.m['conflicts']=[pin]
        active=dict(ActiveState='active',MainPID='7',ControlPID='0',SendSIGKILL='no',ControlGroup='/missing-cpu-fixture')
        idle=dict(robot_vin='arx13',manager_alive=False,orphan_alive=False,teleop=None,collect_status='stopped',collection=dict(phase='stopped',safe_ok=True))
        with patch.object(d,'verify_unit',return_value=active),patch.object(d,'unit_state',side_effect=[active,dict(ActiveState='inactive',MainPID='0')]),patch.object(d,'http_json',return_value=idle),patch.object(d.subprocess,'run')as run,patch.object(d,'observe',return_value=observation()),patch.object(d,'old_owner_files'),patch('robot_workbench.control_owner.ControlChannel')as channel:
            channel.return_value.call.return_value={'connected':False};result=d.start(self.m,self.units(),handoff=True)
        self.assertEqual(run.call_args.args[0],['systemctl','--user','stop','--no-block','manual.service']);self.assertEqual(result['paused_collection_units'],['manual.service'])
    def test_stale_owner_blocks_before_stopping_collection(self):
        self.m['conflicts']=[dict(unit='manual.service',status_url='http://127.0.0.1:18980/api/status')]
        with patch.object(d,'old_owner_files',side_effect=d.Blocked('same-boot')),patch.object(d.subprocess,'run')as run:
            with self.assertRaisesRegex(d.Blocked,'same-boot'):d.start(self.m,self.units(),handoff=True)
            run.assert_not_called()
        self.assertEqual(self.events,[])

    def test_active_collection_uses_normal_stop_and_verifies_exit_without_kill(self):
        pin=dict(unit='prometheus-arx13-web.service',files={'/unit':'a'*64},status_url='http://127.0.0.1:18980/api/status')
        active=dict(ActiveState='active',MainPID='7',ControlPID='0',SendSIGKILL='no',ControlGroup='/missing-cpu-fixture')
        running=dict(robot_vin='arx13',manager_alive=True,orphan_alive=False,teleop=None,collect_status='running',collection=dict(session_id='cpu',phase='recording'))
        stopped=dict(running,manager_alive=False,collect_status='stopped',collection=dict(session_id='cpu',phase='stopped',safe_ok=True))
        with patch.object(d,'verify_unit',return_value=active),patch.object(d,'unit_state',return_value=dict(ActiveState='inactive',MainPID='0')),patch.object(d,'http_json',side_effect=[running,{'collect_status':'stopping'},stopped])as api,patch.object(d.subprocess,'run')as run:
            result=d.handoff_collection(pin,'arx13',stop_active=True)
        self.assertTrue(result['exit_verified']);self.assertTrue(result['may_have_returned_home'])
        self.assertEqual(api.call_args_list[1].args,('http://127.0.0.1:18980/api/collect/stop',{}))
        run.assert_called_once_with(['systemctl','--user','stop','--no-block',pin['unit']],check=True,timeout=10)
        for bad in (dict(running,robot_vin='other'),dict(running,orphan_alive=True),dict(running,teleop={'active':True}),dict(stopped,collection=dict(phase='stopped',safe_ok=False))):
            with self.subTest(bad=bad),patch.object(d,'verify_unit',return_value=active),patch.object(d,'http_json',return_value=bad),patch.object(d.subprocess,'run')as mutation:
                with self.assertRaises(d.Blocked):d.handoff_collection(pin,'arx13',stop_active=True)
                mutation.assert_not_called()

    def test_handoff_releases_independent_camera_even_when_collector_web_is_stopped(self):
        camera=dict(unit='collector-images@bench.service',files={'/cpu/camera.unit':'a'*64},
            state_path='/cpu/camera.json',cameras={'head':'fixture-camera'})
        pin=dict(unit='collector-web.service',files={'/cpu/web.unit':'b'*64},
            status_url='http://127.0.0.1:8080/api/status',camera_owner=camera)
        inactive=dict(ActiveState='inactive',MainPID='0',ControlPID='0',ControlGroup='')
        active=dict(ActiveState='active',MainPID='7',ControlPID='0',SendSIGKILL='no',ControlGroup='/missing-camera-fixture')
        owner=dict(pid=7,instance='source',profile={'cameras':camera['cameras']})
        for web_running in (False,True):
            with self.subTest(web_running=web_running):
                def unit(value):return active if value['unit']==camera['unit']or web_running else inactive
                idle=dict(robot_vin='cpu',manager_alive=False,orphan_alive=False,teleop=None,collect_status='idle')
                with patch.object(d,'verify_unit',side_effect=unit),patch.object(d,'unit_state',return_value=inactive),\
                        patch.object(d,'http_json',return_value=idle),patch('robot_workbench.camera_owner.read_owner',return_value=owner),\
                        patch.object(d.subprocess,'run')as stop:
                    result=d.handoff_collection(pin,'cpu')
                self.assertTrue(result['camera_owner']['exit_verified'])
                self.assertEqual([x.args[0][-1]for x in stop.call_args_list],
                    ([pin['unit']]if web_running else[])+[camera['unit']])
        with patch.object(d,'verify_unit',side_effect=lambda value:active if value['unit']==camera['unit']else inactive),\
                patch('robot_workbench.camera_owner.read_owner',return_value=dict(owner,pid=8)),patch.object(d.subprocess,'run')as stop:
            with self.assertRaisesRegex(d.Blocked,'identity/profile'):d.handoff_collection(pin,'cpu')
            stop.assert_not_called()
        with patch.object(d,'verify_unit',side_effect=lambda value:active if value['unit']==camera['unit']else inactive),\
                patch('robot_workbench.camera_owner.read_owner',return_value=owner),patch.object(d,'unit_state',return_value=active),\
                patch.object(d.time,'monotonic',side_effect=[0.,0.,1.]),patch.object(d.time,'sleep'),patch.object(d.subprocess,'run')as stop:
            with self.assertRaisesRegex(d.Blocked,'camera exit unconfirmed'):d.handoff_collection(pin,'cpu',timeout=.1)
            self.assertEqual(stop.call_count,1)

    def test_stopped_collection_without_episode_has_terminal_components_not_safe_result(self):
        pin=dict(unit='collector.service',files={'/unit':'a'*64},status_url='http://127.0.0.1:8080/api/status')
        unit=dict(ActiveState='active',MainPID='7',ControlPID='0',SendSIGKILL='no',ControlGroup='/missing-cpu-fixture')
        terminal=dict(schema_version=1,phase='stopped',safe_ok=None,error='',
            components=dict(robot='stopped',cameras='stopped',tactile='disabled',event='stopped',pedal='stopped'))
        status=dict(robot_vin='cpu',manager_alive=False,orphan_alive=False,teleop=None,collect_status='stopped',collection=terminal)
        with patch.object(d,'verify_unit',return_value=unit),patch.object(d,'unit_state',return_value=dict(ActiveState='inactive',MainPID='0')),\
                patch.object(d,'http_json',return_value=status),patch.object(d.subprocess,'run')as stop:
            result=d.handoff_collection(pin,'cpu')
        self.assertTrue(result['exit_verified']);self.assertFalse(result['stop_requested']);stop.assert_called_once()
        failed=dict(terminal,phase='failed',end_reason='startup_failed',episode_index=None,recording_started_at=None,
            error='robot exited before ready',components=dict(robot='stopped',cameras='waiting',tactile='waiting',event='waiting',pedal='waiting'))
        with patch.object(d,'verify_unit',return_value=unit),patch.object(d,'unit_state',return_value=dict(ActiveState='inactive',MainPID='0')),\
                patch.object(d,'http_json',return_value=dict(status,collection=failed)),patch.object(d.subprocess,'run'):
            result=d.handoff_collection(pin,'cpu')
        self.assertTrue(result['exit_verified']);self.assertEqual(result['previous_collection_phase'],'failed')
        for bad in (dict(terminal,schema_version=0),dict(terminal,components={}),dict(terminal,error='shutdown failed'),
                    dict(terminal,components=dict(terminal['components'],robot='running')),dict(terminal,safe_ok=False),
                    {k:v for k,v in terminal.items()if k!='safe_ok'},dict(failed,episode_index=1),
                    dict(failed,components=dict(failed['components'],robot='waiting'))):
            with self.subTest(bad=bad),patch.object(d,'verify_unit',return_value=unit),\
                    patch.object(d,'http_json',return_value=dict(status,collection=bad)),patch.object(d.subprocess,'run')as stop:
                with self.assertRaises(d.Blocked):d.handoff_collection(pin,'cpu')
                stop.assert_not_called()

    def test_collection_handoff_stop_cancels_future_steps(self):
        pin=dict(unit='prometheus-arx13-web.service',files={'/unit':'a'*64},status_url='http://127.0.0.1:18980/api/status')
        active=dict(ActiveState='active',MainPID='7',ControlPID='0',SendSIGKILL='no',ControlGroup='/missing-cpu-fixture')
        running=dict(robot_vin='arx13',manager_alive=True,orphan_alive=False,teleop=None,collection=dict(session_id='cpu'))
        def cancelled(phase,**kw):
            if phase=='waiting_collection_exit':raise RuntimeError('operator stop')
        with patch.object(d,'verify_unit',return_value=active),patch.object(d,'http_json',side_effect=[running,{}])as api,patch.object(d.subprocess,'run')as mutation:
            with self.assertRaisesRegex(RuntimeError,'operator stop'):d.handoff_collection(pin,'arx13',stop_active=True,progress=cancelled)
            self.assertEqual(api.call_count,2);mutation.assert_not_called()
    def test_startup_checks_live_units_without_rescanning_source_or_dropin_files(self):
        pin=dict(unit='x.service',files={'/x':'a'})
        with patch.object(d,'unit_pin',side_effect=AssertionError('no repeated file scan')),patch.object(d,'unit_state',return_value={'NeedDaemonReload':'no'}):
            self.assertEqual(d.verify_unit(pin),{'NeedDaemonReload':'no'})
        with patch.object(d,'unit_state',return_value={'NeedDaemonReload':'yes'}):
            with self.assertRaisesRegex(d.Blocked,'not loaded'):d.verify_unit(pin)
        p=self.root/'x.py';p.write_text('before');self.m['files']={str(p):d.sha(p)};p.write_text('after')
        self.m.update(services={},conflicts=[],can_interfaces={})
        with patch.object(d,'sha',side_effect=AssertionError('no repeated file hash')),patch.object(d,'services',return_value={}):
            self.assertEqual(d.preflight(self.m),{})
        self.assertEqual(self.events,[])
    def test_dependencies_pin_static_graph_without_reading_secret_or_mutable_descriptor(self):
        child=self.root/'child.json';child.write_text('{}');token=self.root/'token';token.write_text('private')
        desc=self.root/'current.json';desc.write_text('{}');p=self.root/'root.json'
        p.write_text(json.dumps(dict(calibration_config=str(child),token_file=str(token),camera_service_descriptor=str(desc),backend_attempt_pointer=str(desc))))
        files={};d.dependencies(p,self.root,files)
        self.assertEqual(set(files),{str(p),str(child)})
    def test_prestart_output_receipt_is_not_a_static_command_input(self):
        script=self.root/'prestart.py';script.write_text('# CPU preflight')
        profile=self.root/'profile.json';profile.write_text('{}')
        output=self.root/'model-prestart.json';output.write_text('{"instance":1}')
        argv=['python',str(script),'--profile',str(profile),'--output',str(output)]
        self.assertEqual(list(d.command_inputs(argv)),[str(script),str(profile)])
        output.write_text('{"instance":2}')
        self.assertEqual(list(d.command_inputs(argv)),[str(script),str(profile)])
        self.assertEqual(list(d.command_inputs(['python',str(script),'--output='+str(output)])),[str(script)])
    @unittest.skipUnless(sys.platform=='linux','real /proc boot identity and Unix socket on Linux')
    def test_old_boot_quarantine_preserves_evidence_and_rejects_same_boot(self):
        root=self.root/'owner'
        token=root/'control-token';d.private_write(token,{'secret':'unused'})
        sock=socket.socket(socket.AF_UNIX);sock.bind(str(root/'control.sock'));sock.close();(root/'control.sock').chmod(0o600)
        descriptor=Path(self.m['owner_descriptor'])
        value=dict(device_id='arx13',socket_path=str(root/'control.sock'),token_file=str(token),identity=dict(uid=os.getuid(),boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip()))
        descriptor.write_text(json.dumps(value))
        with self.assertRaisesRegex(d.Blocked,'same-boot'):d.old_owner_files(self.m)
        value['identity']['boot_id']='previous-boot';descriptor.write_text(json.dumps(value))
        (root/'control.sock').unlink()
        with socket.socket(socket.AF_UNIX)as live:
            live.bind(str(root/'control.sock'));live.listen();(root/'control.sock').chmod(0o600)
            with self.assertRaisesRegex(d.Blocked,'registered in the kernel'):d.old_owner_files(self.m,apply=True)
            self.assertTrue(token.exists());self.assertTrue(descriptor.exists())
        self.assertEqual(len(d.old_owner_files(self.m)),2)
        with patch.object(d,'unit_state',return_value={'MainPID':'0'}):d.old_owner_files(self.m,apply=True)
        archived=list(root.glob('prior-boot-*'));self.assertEqual(len(archived),1);self.assertTrue((archived[0]/'control-token').is_file());self.assertFalse(token.exists())
    def test_launch_serialization(self):
        with d.startup_lock(self.root/'lock'):
            with self.assertRaises(BlockingIOError):
                with d.startup_lock(self.root/'lock'):pass

class StartupOperationTest(unittest.TestCase):
    def ready(self):
        return dict(health={'backend_reachable':True},operations={'prepare_model':{'allowed':True}},
            target={'device_id':'fixture'},csrf='test',deployment={'sha256':'test'})

    def test_explicit_pre_execution_refusal_is_retried_with_new_request(self):
        states=[self.ready(),dict(status='failed',error={'code':'precondition_changed'}),
            self.ready(),dict(status='succeeded',verification={'ok':True})]
        with patch.object(d,'api',side_effect=states),patch.object(d,'http_json',side_effect=[{'job_id':'first'},{'job_id':'second'}])as submit,patch.object(d.time,'sleep'):
            result=d.startup_operation({'url':'http://localhost:1234'},'prepare_model')
        self.assertEqual(result['job_id'],'second')
        self.assertNotEqual(submit.call_args_list[0].args[1]['request_id'],submit.call_args_list[1].args[1]['request_id'])

    def test_unknown_result_is_not_replayed(self):
        with patch.object(d,'api',side_effect=[self.ready(),dict(status='unknown',unknown_reason='reply missing')]),patch.object(
                d,'http_json',return_value={'job_id':'only'})as submit:
            with self.assertRaisesRegex(d.Blocked,'unconfirmed'):d.startup_operation({'url':'http://localhost:1234'},'prepare_model')
        self.assertEqual(submit.call_count,1)

    def test_matching_running_operation_is_joined_without_submission(self):
        state=self.ready();state.update(active_job=dict(job_id='existing',request={'action':'prepare_model','parameters':{}}))
        state['operations']['prepare_model']['allowed']=False
        with patch.object(d,'api',side_effect=[state,dict(status='succeeded',verification={'ok':True})]),patch.object(d,'http_json')as submit:
            self.assertEqual(d.startup_operation({'url':'http://localhost:1234'},'prepare_model')['job_id'],'existing')
        submit.assert_not_called()


if __name__=='__main__':unittest.main()
