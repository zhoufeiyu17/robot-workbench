"""Independent manager HTTP contracts using owned CPU protocol subprocesses."""
from copy import deepcopy
from pathlib import Path
import sys
import time
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'tests')]
import test_management as fixture
from robot_workbench import management
from robot_workbench.environment import validate_profiles


class EnvironmentManagerTests(unittest.TestCase):
    def test_summary_deduplicates_jobs_and_aliases_and_detail_route_keeps_evidence(self):
        import json
        status,job,_=self.action('status_check',request_id='summary-check')
        self.assertEqual(status,200);self.fixture.wait_job(job)
        manager=self.h['manager'];manager.refresh()
        with manager.operations.lock:
            stored=manager.operations.jobs['summary-check'];stored['result']['cold']='x'*500000
        status,summary,_=self.fixture.request('/api/environment?environment=fake&view=summary',h=self.h)
        self.assertEqual(status,200);self.assertEqual(summary['projection'],'manager_environment_summary')
        self.assertIsInstance(summary['recent_jobs'][0],str)
        self.assertEqual(summary['operations']['status_check']['latest_job'],job['job_id'])
        self.assertNotIn('runtime',summary['health']);self.assertNotIn('result',summary['jobs'][job['job_id']])
        self.assertLess(len(json.dumps(summary)),50000)
        status,detail,_=self.fixture.request('/api/operations/jobs/'+job['job_id']+'?environment=fake',h=self.h)
        self.assertEqual(status,200);self.assertEqual(len(detail['result']['cold']),500000)

    def setUp(self):
        # Composition intentionally avoids re-running ManagementTests' suite.
        self.fixture=fixture.ManagementTests();self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups);self.h=self.fixture.h

    def headers(self,h=None,*,deployment=None):
        h=h or self.h;state=h['manager'].snapshot()
        return {'Content-Type':'application/json','X-CSRF-Token':state['csrf'],
            'X-Workbench-Deployment':deployment or state['deployment']['sha256'],
            'Origin':'http://127.0.0.1:'+str(h['port'])}

    def post(self,path,body,*,h=None,deployment=None):
        h=h or self.h
        return self.fixture.request(path,body,h=h,headers=self.headers(h,deployment=deployment))

    def backend(self,h=None,*,model=False):
        h=h or self.h
        return fixture.json_request(h['services']['model' if model else 'workbench'].port,'/fixture')

    def action(self,name,*,parameters=None,request_id='operation',h=None):
        h=h or self.h;h['manager'].refresh()
        body=dict(action=name,request_id=request_id,expected_target=h['manager'].target(),arms_supported=False)
        if parameters is not None:body['parameters']=parameters
        status,job,_=self.post('/api/operations/action',body,h=h)
        return status,job,body

    def profile(self,name,h=None,**values):
        h=h or self.h
        return dict(id=name,label=name,environment=h['manager'].environment,device_id='cpu-device',
            manager_url='/',robot_address='10.0.0.2',compute_address='10.0.0.3',**values)

    def probe(self,**selection):
        return self.post('/api/operations/target',dict(expected_target=self.h['manager'].target(),**selection))

    def test_environment_get_reads_common_facts_without_workflow_or_lifecycle(self):
        for h in (self.h,self.fixture.harness('physical')):
            with self.subTest(environment=h['manager'].environment):
                before=self.backend(h);counts={name:(s.starts,s.stops) for name,s in h['services'].items()}
                paths=[];original=management.json_request
                def read(port,path,*args,**kwargs):
                    if port==h['services']['workbench'].port:
                        paths.append(path)
                        if path.startswith('/api/state'):raise AssertionError('manager parsed a business workflow snapshot')
                    return original(port,path,*args,**kwargs)
                with patch.object(management,'json_request',side_effect=read):
                    h['manager'].refresh()
                    status,value,_=self.fixture.request('/api/environment',h=h)
                self.assertEqual(status,200,value);self.assertEqual(value['schema'],'robot_workbench.environment.v1')
                self.assertIn('/api/environment?environment='+h['manager'].environment,paths)
                self.assertNotIn('workflow',value)
                self.assertEqual(set(value['capabilities']),{'observe','control','calibration','collection','inference'})
                for row in value['capabilities'].values():
                    self.assertIn(row['status'],('ready','blocked','unknown','unsupported'))
                    self.assertIsInstance(row['checked_wall_ns'],int);self.assertEqual(row['identity'],value['target'])
                self.assertEqual({row['category'] for row in value['dependencies']},{'preinstalled','preparable','onsite'})
                self.assertTrue(value['profiles']);self.assertFalse(value['model']['loaded'])
                self.assertEqual(self.backend(h)['events'],before['events'])
                self.assertEqual({name:(s.starts,s.stops) for name,s in h['services'].items()},counts)
                self.assertFalse(self.backend(h)['connected']);self.assertFalse(self.backend(h,model=True)['loaded'])

    def test_current_profile_check_and_unknown_address_never_acquire(self):
        manager=self.h['manager'];manager.profiles=validate_profiles([self.profile('current')])
        before=self.backend()['events']
        status,value,_=self.probe(profile_id='current',robot_address='10.0.0.2')
        self.assertEqual(status,200,value);self.assertEqual(value['target'],manager.target())
        status,error,_=self.probe(robot_address='10.9.9.9')
        self.assertEqual(status,409,error);self.assertIn('唯一已登记档案',error['error'])
        stale=deepcopy(manager.target());stale['manager_instance']='stale-manager'
        status,error,_=self.post('/api/operations/target',dict(expected_target=stale,profile_id='current'))
        self.assertEqual(status,409,error)
        self.assertEqual(self.backend()['events'],before);self.assertFalse(self.backend()['connected'])

    def test_registered_cross_manager_redirect_is_readonly_and_checks_device_identity(self):
        other=self.fixture.harness('physical');remote=other['manager']
        row=self.profile('other',other);row['manager_url']='http://127.0.0.1:'+str(other['port'])
        self.h['manager'].profiles=validate_profiles([row])
        before=self.backend(other)['events']
        with patch.object(remote,'environment_snapshot',wraps=remote.environment_snapshot) as read:
            status,value,_=self.probe(profile_id='other')
            self.assertEqual(status,200,value);self.assertEqual(value['redirect_url'],row['manager_url'])
            self.assertFalse(value['controller_acquired']);read.assert_called_once()
            row['device_id']='wrong-device';self.h['manager'].profiles=validate_profiles([row])
            status,error,_=self.probe(profile_id='other')
            self.assertEqual(status,409,error);self.assertIn('身份',error['error'])
        self.assertEqual(self.backend(other)['events'],before)
        self.assertFalse(self.backend(other)['connected']);self.assertFalse(self.backend()['connected'])

    def test_same_ip_requires_registered_instance_selection_without_guessing(self):
        other=self.fixture.harness('physical');local=self.profile('first');remote=self.profile('second',other)
        remote['manager_url']='http://127.0.0.1:'+str(other['port'])
        self.h['manager'].profiles=validate_profiles([local,remote])
        with patch.object(other['manager'],'environment_snapshot',wraps=other['manager'].environment_snapshot) as read:
            status,error,_=self.probe(compute_address='10.0.0.3')
            self.assertEqual(status,409,error);self.assertIn('选择对应实例',error['error']);read.assert_not_called()
            status,value,_=self.probe(compute_address='10.0.0.3',profile_id='second')
            self.assertEqual(status,200,value);read.assert_called_once()
        self.assertEqual(self.backend()['events'],[]);self.assertEqual(self.backend(other)['events'],[])

    def test_independent_stop_ignores_stale_deployment_and_operation_busy(self):
        self.fixture.configure(camera_delay=.6)
        status,job,_=self.action('restart_cameras',request_id='busy-camera')
        self.assertEqual(status,200,job)
        deadline=time.monotonic()+2
        while time.monotonic()<deadline:
            state=self.backend()
            if (state.get('maintenance') or {}).get('active_action'):break
            time.sleep(.01)
        self.assertIsNotNone(self.h['manager'].operations.active)
        started=time.monotonic()
        status,stopped,_=self.post('/api/operations/stop',dict(expected_device_id='cpu-device',request_id='stop-now'),deployment='0'*64)
        self.assertEqual(status,200,stopped);self.assertEqual(stopped['status'],'accepted')
        self.assertLess(time.monotonic()-started,.5);self.assertIn('stop',self.backend()['events'])
        before=self.backend()['events'].count('stop')
        status,error,_=self.post('/api/operations/stop',dict(expected_device_id='other-device',request_id='wrong-stop'),deployment='0'*64)
        self.assertEqual(status,409,error);self.assertEqual(self.backend()['events'].count('stop'),before)
        self.fixture.wait_job(job)

    def test_stop_reports_unknown_when_transport_is_unavailable(self):
        self.h['services']['workbench'].stop()
        self.h['manager'].refresh()
        status,value,_=self.post('/api/operations/stop',dict(expected_device_id='cpu-device',request_id='unreachable-stop'),deployment='0'*64)
        self.assertEqual(status,200,value);self.assertEqual(value['status'],'unknown')
        self.assertIs(value['stopped'],False);self.assertNotIn('receipt',value)

    def test_operation_parameters_are_registered_and_bound_to_idempotency(self):
        parameters=dict(model_id='cpu-test-model',expected_bundle_digest='a'*64)
        status,job,body=self.action('prepare_model',parameters=parameters,request_id='select-once')
        self.assertEqual(status,200,job)
        result=self.fixture.wait_job(job);self.assertEqual(result['status'],'succeeded',result)
        self.assertEqual(result['request']['parameters'],parameters)
        warmed=self.backend(model=True)['events'].count('warm')
        status,repeated,_=self.post('/api/operations/action',body)
        self.assertEqual(status,200,repeated);self.assertEqual(repeated['job_id'],job['job_id'])
        self.assertEqual(self.backend(model=True)['events'].count('warm'),warmed)
        conflict=deepcopy(body);conflict['parameters']['expected_bundle_digest']='b'*64
        status,error,_=self.post('/api/operations/action',conflict)
        self.assertEqual(status,409,error);self.assertIn('request_id',error['error'])
        for action,params in [('prepare_model',{'command':'unregistered'}),('status_check',parameters),
                              ('prepare_environment',{'capabilities':['observe','unknown']})]:
            status,error,_=self.action(action,parameters=params,request_id='invalid-'+action)
            self.assertEqual(status,409,error)
        self.assertEqual(self.backend(model=True)['events'].count('warm'),warmed)

    def test_environment_prepare_defaults_to_observe_and_model_loading_is_explicit(self):
        starts={name:service.starts for name,service in self.h['services'].items()}
        status,job,_=self.action('prepare_environment',request_id='observe-only')
        self.assertEqual(status,200,job)
        result=self.fixture.wait_job(job);self.assertEqual(result['status'],'succeeded',result)
        self.assertIn('ops_start_cameras',self.backend()['events'])
        self.assertNotIn('ops_prepare_model',self.backend()['events']);self.assertFalse(self.backend(model=True)['loaded'])
        self.assertFalse(self.backend()['connected'])
        status,job,_=self.action('prepare_model',request_id='explicit-model')
        self.assertEqual(status,200,job)
        result=self.fixture.wait_job(job);self.assertEqual(result['status'],'succeeded',result)
        self.assertEqual(self.backend()['events'].count('ops_prepare_model'),1)
        self.assertEqual(self.backend(model=True)['events'].count('warm'),1)
        self.assertTrue(self.backend(model=True)['loaded']);self.assertFalse(self.backend()['connected'])
        self.assertEqual({name:service.starts for name,service in self.h['services'].items()},starts)


if __name__=='__main__':unittest.main()
