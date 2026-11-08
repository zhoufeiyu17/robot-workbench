"""Pure environment presentation: current facts never become execution authority."""
from copy import deepcopy
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from robot_workbench.environment import describe,validate_profiles


class EnvironmentPresentationTests(unittest.TestCase):
    def setUp(self):
        self.config=dict(environment='fake',device_id='robot',selected_profile_id='registered')
        self.profiles=validate_profiles([dict(id='registered',label='CPU fixture',environment='fake',device_id='robot',
            robot_address='10.0.0.2',compute_address='10.0.0.3',manager_url='/',configured=True,topology='fullfake_dsw')])
        self.view=dict(target=dict(device_id='robot',environment='fake'),current_capabilities={'camera_frames':True},
            machine_profile={'declared':{'hardware':{'robot_model':'fixture-robot'}}},health=dict(backend_reachable=True,
                checked_wall_ns=123,activity={'busy':False,'businesses':{name:{'busy':False} for name in ('calibration','collection','inference')}},
                runtime=dict(connected=True,state='READY',feedback_healthy=True,hold={'left':'position','right':'manual_damping'}),
                model={'status':'ready','model_ready':True,'selected_model_id':'fixture-checkpoint'},
                collection={'status':'ready','ready':True,'services':{
                    'detector':{'status':'ready','service_pid':1},'planner':{'status':'ready','model_initialized':True,'service_pid':2},
                    'scene':{'status':'busy','model_initialized':True,'service_pid':3}}},
                services={'workbench':{'state':'running'}},resources={'gpu':{'status':'observed','cards':[{'uuid':'fixture','utilization_percent':0}]},
                    'reservation':{'status':'declared','record':'existing-record'}}))
    def render(self):return describe(self.config,self.view,self.profiles)

    def test_resident_recording_health_is_separate_from_per_run_readiness(self):
        self.view['health']['services']['recording']={'state':'running'}
        self.view['health']['recording_service']={'ready':True,'pid':123,'active_sessions':[]}
        dependency=next(row for row in self.render()['dependencies']if row['id']=='recording')
        self.assertEqual(dependency['status'],'ready');self.assertIn('每个Run另行',dependency['expected'])
        self.view['health']['recording_service']={'ready':False,'status':'unknown','error':'IPC unavailable'}
        dependency=next(row for row in self.render()['dependencies']if row['id']=='recording')
        self.assertEqual(dependency['status'],'unknown');self.assertEqual(dependency['reason'],'IPC unavailable')

    def test_dead_or_unknown_worker_cannot_be_overridden_by_cached_binding(self):
        self.view['health']['services']['model']={'state':'running'}
        base=dict(model_id='fixture-checkpoint',loaded=True,status='ready',warmup={'completed':True})
        self.view['health']['model_service']=base
        self.assertTrue(self.render()['capabilities']['inference']['ready'])
        for service in ({**base,'loaded':False,'status':'error'},
                        {**base,'worker_pid':None},
                        {**base,'model_id':'other-model'},
                        {**base,'warmup':{'completed':False}},
                        dict(status='unknown',loaded=None)):
            self.view['health']['model_service']=service
            value=self.render()
            self.assertIsNot(value['capabilities']['inference']['ready'],True)
            self.assertTrue(value['capabilities']['calibration']['ready'])
            self.assertTrue(value['model']['readiness']['workspace_bound'])

    def test_mixed_control_publication_is_not_current_control_readiness(self):
        self.view['health']['runtime']['control_coherence']={'consistent':False}
        self.assertFalse(self.render()['capabilities']['control']['ready'])

    def test_unknown_backend_and_unconfigured_businesses_never_turn_green(self):
        for reachable in (False,None):
            with self.subTest(reachable=reachable):
                self.view['health']['backend_reachable']=reachable
                value=self.render()
                self.assertTrue(all(row['status']=='unknown' for row in value['capabilities'].values()))
                self.assertEqual(value['execution_admission']['status'],'unknown')
                self.assertEqual(value['model'],{});self.assertEqual(value['collection'],{})
        self.view['health']['backend_reachable']=True
        self.view['health']['activity']['businesses']={}
        self.view['health']['collection']={'status':'unconfigured','ready':False}
        self.profiles[0]['capabilities']=['calibration','collection','inference']
        value=self.render()
        for name in ('calibration','collection','inference'):
            self.assertEqual(value['capabilities'][name]['status'],'unsupported')
        del self.view['health']['activity']['businesses']
        self.assertEqual(self.render()['capabilities']['calibration']['status'],'unknown')

    def test_current_faults_unknown_state_and_missing_hold_do_not_claim_control(self):
        healthy=deepcopy(self.view['health']['runtime'])
        for change in ({'state':'FAULT_STOP_UNCONFIRMED'},{'state':'CONTROL_FAULT_LATCHED'},
                {'state':'EMERGENCY_STOP'},{'state':'new_unknown_state'},{'hold':{}},{'feedback_healthy':'true'},
                {'recoverable_fault':'tracking failed'},{'initialization_error':'native init failed'},
                {'release_receipt':{'status':'unconfirmed','message':'close not confirmed'}},
                {'output_guard':{'state':'latched','reason':'CAN feedback lost'}}):
            with self.subTest(change=change):
                self.view['health']['runtime']={**healthy,**change}
                value=self.render()
                self.assertNotEqual(value['capabilities']['control']['status'],'ready')
                self.assertNotEqual(value['capabilities']['calibration']['status'],'ready')
        self.view['health']['runtime']={**healthy,'output_guard':{'state':'active','last_latch':{'reason':'historical'}}}
        self.assertEqual(self.render()['capabilities']['control']['status'],'ready','historical latch is not current fault evidence')

    def test_first_calibration_does_not_require_model_or_existing_calibration(self):
        self.view['health']['model']={'status':'not_loaded','model_ready':False}
        self.view['health']['collection']={'status':'awaiting_calibration','ready':False}
        value=self.render()
        self.assertEqual(value['capabilities']['calibration']['status'],'ready')
        self.assertEqual(value['capabilities']['collection']['status'],'blocked')
        self.assertEqual(value['capabilities']['inference']['status'],'blocked')
        self.view['health']['model']={'status':'unknown','model_ready':True}
        self.assertEqual(self.render()['capabilities']['inference']['status'],'unknown')

    def test_busy_and_maintenance_are_execution_barriers_not_lost_dependencies(self):
        value=self.render()
        self.assertTrue(all(row['ready'] for row in value['capabilities'].values()))
        self.assertEqual(value['execution_admission']['status'],'requires_review')
        self.assertFalse(value['execution_admission']['authorization_evaluated'])
        self.view['health']['activity']['busy']=True
        value=self.render()
        self.assertTrue(value['capabilities']['control']['ready']);self.assertTrue(value['capabilities']['calibration']['ready'])
        self.assertEqual(value['execution_admission']['status'],'blocked')
        self.view['health']['activity']['busy']=False;self.view['health']['maintenance']={'status':'failed','purpose':'verify_hold'}
        value=self.render()
        self.assertTrue(value['capabilities']['control']['ready'])
        self.assertEqual(value['execution_admission']['status'],'blocked');self.assertIn('维护',value['execution_admission']['reason'])

    def test_existing_collection_services_are_exposed_and_failures_override_stale_ready(self):
        value=self.render()
        self.assertEqual(value['services']['planner'],self.view['health']['collection']['services']['planner'])
        dependencies={row['id']:row for row in value['dependencies']}
        self.assertEqual(dependencies['collection_scene']['status'],'ready')
        self.assertIsNone(dependencies['collection']['reason'],'ready collection must not ask the operator to prepare it again')
        self.view['health']['collection']['services']['planner'].update(status='error',error='worker failed')
        value=self.render();self.assertEqual(value['capabilities']['collection']['status'],'blocked')
        self.assertIn('请在运维准备数采资产与常驻服务',next(row for row in value['dependencies'] if row['id']=='collection')['reason'])
        self.assertEqual(next(row for row in value['dependencies'] if row['id']=='collection_planner')['reason'],'worker failed')
        self.view['health']['collection']['services']['planner']={'status':'unknown'}
        self.assertEqual(self.render()['capabilities']['collection']['status'],'unknown')
        self.view['health']['activity']['businesses']={}
        self.assertEqual(self.render()['capabilities']['collection']['support'],'supported','preparation can precede workflow installation')

    def test_prerequisites_keep_resource_network_and_hardware_claims_as_declarations(self):
        value=self.render();rows={row['id']:row for row in value['dependencies']}
        self.assertIn('Isaac',rows['fake_runtime']['expected']);self.assertEqual(rows['fake_runtime']['status'],'unknown')
        self.assertIn('USD',rows['fake_assets']['expected']);self.assertNotEqual(rows['fake_assets']['status'],'ready')
        self.assertEqual(rows['network']['actual']['compute_address'],'10.0.0.3');self.assertEqual(rows['network']['status'],'declared')
        self.assertEqual(rows['compute_resources']['actual'],self.view['health']['resources'])
        self.assertNotEqual(rows['compute_resources']['status'],'ready')
        self.config['environment']='physical';rows={row['id']:row for row in self.render()['dependencies']}
        self.assertNotIn('fake_runtime',rows);self.assertIn('CAN',rows['device_adapter']['expected'])
        self.assertIn('供电',rows['onsite']['expected']);self.assertEqual(rows['onsite']['status'],'operator_confirmation')

    def test_presentation_is_pure_and_preserves_input_facts(self):
        original=deepcopy((self.config,self.view,self.profiles))
        with patch('builtins.open',side_effect=AssertionError('no file IO')),patch('socket.socket',side_effect=AssertionError('no network IO')),\
                patch('subprocess.Popen',side_effect=AssertionError('no process IO')):
            value=self.render()
        self.assertEqual((self.config,self.view,self.profiles),original)
        value['services']['planner']['status']='changed';self.assertEqual(self.view['health']['collection']['services']['planner']['status'],'ready')
        self.assertEqual(value['profiles'],self.profiles);self.assertEqual(value['selected_profile_id'],'registered')


if __name__=='__main__':unittest.main()
