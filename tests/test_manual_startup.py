import unittest
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch
from scripts.start_registered_workbench import start
from scripts.workbench_deploy import Blocked


class ManualStartupTests(unittest.TestCase):
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
                    'scripts.start_registered_workbench.http_json',side_effect=[state,calibration,ready])as api,\
                    patch('scripts.start_registered_workbench.time.sleep'):
                self.assertEqual(start(config)['model_status'],'ready')
                self.assertEqual(api.call_count,3)

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
            profile.write_text(json.dumps(dict(services={'recording':dict(kind='user_unit',unit='robot-workbench-recording.service')})))
            config=dict(manager_profile=str(profile),units=['robot-workbench-business.service'],url='http://127.0.0.1:18868')
            for ready in (True,False):
                state=dict(health={'backend_reachable':True,'recording_service':{'ready':ready}},target={'services':{'model':{'pid':7}}},model={'ready':True})
                with patch('scripts.start_registered_workbench.subprocess.run')as command,patch(
                        'scripts.start_registered_workbench.http_json',side_effect=[state,{'workflow':{'manual_queue':{'available':True}}}]):
                    if ready:self.assertTrue(start(config)['ui_ready'])
                    else:
                        with self.assertRaisesRegex(Blocked,'recording service readiness'):start(config)
                self.assertEqual([call.args[0][-1]for call in command.call_args_list],
                    ['robot-workbench-recording.service','robot-workbench-business.service'])

    def test_missing_planner_is_not_reported_as_complete_startup(self):
        state=dict(health={'backend_reachable':True},target={'services':{'model':{'pid':7}}},model={'ready':True})
        config=dict(units=['robot-workbench-test.service'],url='http://127.0.0.1:18868')
        with patch('scripts.start_registered_workbench.subprocess.run'),patch(
                'scripts.start_registered_workbench.http_json',side_effect=[state,{'workflow':{}}])as api:
            with self.assertRaisesRegex(Blocked,'manual planning'):start(config)
            self.assertEqual(api.call_count,2)


if __name__=='__main__':unittest.main()
