import unittest
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch,MagicMock
from scripts.start_registered_workbench import start,acquire_control,open_page
from scripts.workbench_deploy import Blocked


class ManualStartupTests(unittest.TestCase):
    def test_webpage_opens_before_service_preparation_and_model_wait(self):
        config=dict(units=['demo-ui@bench.service'],ui_unit='demo-ui@bench.service',url='http://127.0.0.1:18868')
        response=MagicMock();response.__enter__.return_value.status=200
        with patch('scripts.start_registered_workbench.build_opener')as opener,patch('scripts.start_registered_workbench.subprocess.run')as command,\
                patch.dict('os.environ',{'LD_LIBRARY_PATH':'/fixture/robot/lib','LD_PRELOAD':'/fixture/preload.so','DISPLAY':':fixture'}):
            opener.return_value.open.return_value=response
            self.assertLess(open_page(config)['open_completed_after_s'],5)
            self.assertEqual([x.args[0][0]for x in command.call_args_list],['systemctl','xdg-open'])
            browser_env=command.call_args_list[-1].kwargs['env']
            self.assertNotIn('LD_LIBRARY_PATH',browser_env)
            self.assertNotIn('LD_PRELOAD',browser_env)
            self.assertNotIn('PYTHONPATH',browser_env)
            self.assertEqual(browser_env['DISPLAY'],':fixture')

    def test_declared_pedal_requires_a_live_heartbeat_before_startup_succeeds(self):
        with TemporaryDirectory()as folder:
            profile=Path(folder)/'manager.json';profile.write_text(json.dumps(dict(services={'pedal':{}})))
            config=dict(manager_profile=str(profile),units=[],url='http://127.0.0.1:18868')
            state=dict(health={'backend_reachable':True},target={'services':{'model':{'pid':7}}},model={'ready':True})
            for ready in (True,False):
                pedal=dict(workflow={'inference':{'operator_input':dict(state='ready'if ready else'unavailable',error='missing heartbeat')}})
                with patch('scripts.start_registered_workbench.http_json',side_effect=[state,{'workflow':{'manual_queue':{'available':True}}},pedal]),\
                        patch('scripts.start_registered_workbench.time.monotonic',side_effect=[0,0,0,11]):
                    if ready:self.assertTrue(start(config)['ui_ready'])
                    else:
                        with self.assertRaisesRegex(Blocked,'missing heartbeat'):start(config)

    def test_operator_launch_uses_existing_takeover_once_and_reuses_healthy_control(self):
        healthy=dict(connected=True,state='READY',runtime_nonce='new',feedback_healthy=True,
            stationary=True,automatic_enabled=False,sample_age_s=0.,command_epoch=1)
        config=dict(url='http://127.0.0.1:18868')
        with patch('scripts.start_registered_workbench.http_json',side_effect=[{'runtime':{'connected':False}}, {'runtime':healthy}]),\
                patch('scripts.start_registered_workbench.startup_operation',return_value={'job_id':'takeover'})as operation:
            result=acquire_control(config)
        self.assertEqual(result['sdk_acquisitions'],1)
        operation.assert_called_once_with(config,'reclaim_control',arms_supported=True,timeout=300)
        with patch('scripts.start_registered_workbench.http_json',return_value={'runtime':healthy}),\
                patch('scripts.start_registered_workbench.startup_operation')as operation:
            self.assertEqual(acquire_control(config)['sdk_acquisitions'],0);operation.assert_not_called()

    def test_auto_connect_defers_collection_shutdown_to_the_existing_reclaim_operation(self):
        with TemporaryDirectory()as folder:
            profile=Path(folder)/'manager.json';profile.write_text(json.dumps(dict(services={'control_owner':{},'cameras':{'unit':'registered-images.service'}},
                control_lifecycle='managed',collection_handoff={'registered':True},device_id='cpu')))
            config=dict(manager_profile=str(profile),connect_control=True,units=['registered-images.service','robot-workbench-ui.service'],url='http://127.0.0.1:18868')
            state=dict(health={'backend_reachable':True},target={'services':{'model':{'pid':7}}},model={'ready':True})
            with patch('scripts.start_registered_workbench.subprocess.run')as service,patch('scripts.start_registered_workbench.handoff_collection')as handoff,\
                    patch('scripts.start_registered_workbench.acquire_control',return_value=dict(sdk_acquisitions=1,reused=False,job_id='takeover'))as acquire,\
                    patch('scripts.start_registered_workbench.http_json',side_effect=[state,state,{'workflow':{'manual_queue':{'available':True}}}]):
                result=start(config)
            handoff.assert_not_called();acquire.assert_called_once_with(config);self.assertEqual(result['sdk_acquisitions'],1)
            self.assertEqual([call.args[0][-1]for call in service.call_args_list],['robot-workbench-ui.service'],'camera opens only inside the subsequent verified handoff')

    def test_idle_collection_web_handoff_precedes_start_and_never_stops_active_collection(self):
        with TemporaryDirectory() as folder:
            profile=Path(folder)/'manager.json';pin={'unit':'registered-collector'}
            profile.write_text(json.dumps(dict(services={},device_id='cpu',collection_handoff=pin)))
            config=dict(manager_profile=str(profile),units=['robot-workbench-ui.service'],url='http://127.0.0.1:18868')
            state=dict(health={'backend_reachable':True},target={'services':{'model':{'pid':7}}},model={'ready':True})
            with patch('scripts.start_registered_workbench.handoff_collection',side_effect=Blocked('collection is active'))as handoff,\
                    patch('scripts.start_registered_workbench.subprocess.run')as command:
                with self.assertRaisesRegex(Blocked,'collection is active'):start(config)
                handoff.assert_called_once_with(pin,'cpu',stop_active=False);command.assert_not_called()
            events=[]
            with patch('scripts.start_registered_workbench.handoff_collection',side_effect=lambda *a,**kw:events.append('idle_web_stopped')),\
                    patch('scripts.start_registered_workbench.subprocess.run',side_effect=lambda *a,**kw:events.append('software_started')),\
                    patch('scripts.start_registered_workbench.http_json',side_effect=[state,{'workflow':{'manual_queue':{'available':True}}}]):
                self.assertEqual(start(config)['sdk_acquisitions'],0)
            self.assertEqual(events,['idle_web_stopped','software_started'])

    def test_registered_target_and_all_camera_inputs_are_ready_before_return(self):
        with TemporaryDirectory() as folder:
            profile=Path(folder)/'manager.json'
            profile.write_text(json.dumps(dict(services={},target_model_manifest='/registered/model.json')))
            config=dict(manager_profile=str(profile),units=['robot-workbench-ui.service'],url='http://127.0.0.1:18868')
            state=dict(health={'backend_reachable':True},target={'services':{'model':{'pid':7}}},model={'ready':True})
            calibration=dict(workflow={'manual_queue':{'available':True}},preview={'webgl_model':{'model_id':'registered'}},
                media=[dict(id=role,ready=False)for role in ('head','left','right')])
            ready=dict(calibration,media=[dict(id=role,ready=True)for role in ('head','left','right')])
            with patch('scripts.start_registered_workbench.subprocess.run'),patch(
                    'scripts.start_registered_workbench.http_json',side_effect=[state,calibration,state])as api,\
                    patch('scripts.workbench_deploy.api',return_value=ready)as camera_read,\
                    patch('scripts.start_registered_workbench.time.sleep'):
                self.assertEqual(start(config)['model_status'],'ready')
                self.assertEqual(api.call_count,3)
                camera_read.assert_called_once()

    def test_failed_warmup_is_not_reported_as_a_successful_desktop_start(self):
        state=dict(health={'backend_reachable':True},target={'services':{'model':{'pid':7}}},
            model={'ready':False},active_job=dict(job_id='warm1',request={'action':'prepare_model'}))
        config=dict(units=['robot-workbench-test.service'],url='http://127.0.0.1:18868')
        with patch('scripts.start_registered_workbench.subprocess.run'),patch(
                'scripts.start_registered_workbench.http_json',side_effect=[state,
                    {'workflow':{'manual_queue':{'available':True}}},{'status':'failed'}])as api:
            with self.assertRaisesRegex(Blocked,'preparation failed'):start(config)
        self.assertEqual(api.call_count,3)

    def test_registered_recording_starts_first_and_requires_its_own_readiness(self):
        with TemporaryDirectory()as tmp:
            profile=Path(tmp)/'manager.json'
            profile.write_text(json.dumps(dict(services={'recording':dict(kind='user_unit',unit='demo-recorder.service')})))
            config=dict(manager_profile=str(profile),units=['robot-workbench-business.service'],url='http://127.0.0.1:18868')
            for ready in (True,False):
                state=dict(health={'backend_reachable':True,'recording_service':{'ready':ready}},target={'services':{'model':{'pid':7}}},model={'ready':True})
                with patch('scripts.start_registered_workbench.subprocess.run')as command,patch(
                        'scripts.start_registered_workbench.http_json',side_effect=[state,{'workflow':{'manual_queue':{'available':True}}}]):
                    if ready:self.assertTrue(start(config)['ui_ready'])
                    else:
                        with self.assertRaisesRegex(Blocked,'recording service readiness'):start(config)
                self.assertEqual([call.args[0][-1]for call in command.call_args_list],
                    ['demo-recorder.service','robot-workbench-business.service'])

    def test_missing_planner_is_not_reported_as_complete_startup(self):
        state=dict(health={'backend_reachable':True},target={'services':{'model':{'pid':7}}},model={'ready':True})
        config=dict(units=['robot-workbench-test.service'],url='http://127.0.0.1:18868')
        with patch('scripts.start_registered_workbench.subprocess.run'),patch(
                'scripts.start_registered_workbench.http_json',side_effect=[state,{'workflow':{}}])as api:
            with self.assertRaisesRegex(Blocked,'manual planning'):start(config)
            self.assertEqual(api.call_count,2)


if __name__=='__main__':unittest.main()
