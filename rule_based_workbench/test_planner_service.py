"""Owned planner IPC, formal identity and GPU admission checks, without CUDA."""
from copy import deepcopy
import io
import json
import os
from pathlib import Path
import subprocess
import sys
from tempfile import TemporaryDirectory
from types import SimpleNamespace as NS
from unittest.mock import patch
from uuid import uuid4

from curobo_worker import request_service,Worker
from planner_service import INSTALL_SPEC,OWNED_GPU_CHECK,PlannerService,startup_config,validate_config,validate_spec,private_spec_arguments,load_private_spec
from run_pipeline import PATH_KEYS,PIPELINE_LANES,build_plan,load_config


def main():
    from motiongen_motion import cuda_graph_counter
    class Graph:
        def capture_begin(self,*args,**kwargs):return args,kwargs
        def replay(self):return 7
    module=NS(cuda=NS(CUDAGraph=Graph));calls=cuda_graph_counter(module)
    assert cuda_graph_counter(module) is calls
    graph=Graph();assert graph.capture_begin(1,pool=2)==((1,),{'pool':2}) and graph.replay()==7
    assert calls=={'captures':1,'replays':1}
    with TemporaryDirectory() as temporary:
        local=Path(temporary); output='/mnt/data/feiyu.zhou/runs/robot_sim/planner-test'
        contract=dict(schema_version='anyverse.dsw-codex-experiment.v1',identity={'session_id':'test-session'},
            filesystem=dict(readonly_roots=['/remote/input'],writable_roots=[output],allow_overwrite=False,allow_delete=False),
            resources=dict(lanes={v['instance_id']:[v['gpu']] for v in PIPELINE_LANES.values()},
                max_gpu_count_per_instance=1,max_workers_per_instance=2,max_cpu_threads=4,
                require_gpu_idle_check=True,max_wall_time_minutes=20),
            provenance=dict(code_sha256={'/remote/input/workbench/'+name:'a'*64 for name in
                ('curobo_worker.py','planner_service.py','motiongen_plan.py','motiongen_motion.py','gpu_acceptance.py',
                 'curobo_robot_model.py','cup_contact.py','cup_envelope.py','region_self_collision.py')}))
        config={key:'/remote/input/'+key for key in PATH_KEYS}
        config.update(workbench_code='/remote/input/workbench',contract_path=str(local/'contract.json'),
            remote_run_root=output,opening_mm=83.,finger_envelope_sha256='a'*64,planning_backend='motiongen',curobo_root='/remote/input/curobo')
        (local/'contract.json').write_text(json.dumps(contract));(local/'config.json').write_text(json.dumps(config))
        prepared=startup_config(local/'config.json',local)
        current,frozen,_=load_config(prepared);spec=current['planner_service']
        long_contract=deepcopy(contract);long_contract['resources']['service_lifetime_seconds']=172800
        (local/'long-contract.json').write_text(json.dumps(long_contract))
        (local/'long-config.json').write_text(json.dumps(dict(config,contract_path=str(local/'long-contract.json'))))
        long=startup_config(local/'long-config.json',local)
        assert load_config(long)[0]['planner_service']['max_lifetime_seconds']==172800
        assert load_config(long)[1]['resources']['max_wall_time_minutes']==20
        # Explicit remote persistence preserves the dual-DSW lanes and finite per-job budgets.
        persistent_contract=deepcopy(contract)
        persistent_contract['resources'].update(persistent_service=True,service_lifetime_seconds=None)
        runtime='/remote/input/runtime'
        persistent_contract['provenance']['code_sha256'].update({runtime+'/scripts/run_online_detection_episode.py':'b'*64,
            runtime+'/ros2/online_detection_ros/node.py':'c'*64})
        persistent_file=local/'persistent-contract.json';persistent_file.write_text(json.dumps(persistent_contract))
        persistent_config=dict(config,contract_path=str(persistent_file),detection_runtime_root=runtime)
        persistent_input=local/'persistent.json';persistent_input.write_text(json.dumps(persistent_config))
        persistent=startup_config(persistent_input,local)
        remote,remote_contract,_=load_config(persistent)
        assert remote['planner_service']['persistent_service'] is True
        assert remote['planner_service']['max_lifetime_seconds'] is None and 'execution' not in remote
        assert remote_contract['resources']['lanes']==contract['resources']['lanes']
        assert remote_contract['resources']['max_wall_time_minutes']==20
        remote_stages={s['name']:s for s in build_plan(remote,remote_contract)}
        assert 'ROBOT_WORKBENCH_DETECTOR_RUNTIME_ROOT='+runtime in remote_stages['detect']['argv'][-1]
        for change in ({'service_lifetime_seconds':7200}, {'persistent_service':False},
                       {'require_gpu_idle_check':False}, {'max_wall_time_minutes':None},
                       {'lanes':{'foreign':[0]}}):
            bad=deepcopy(persistent_contract);bad['resources'].update(change)
            persistent_file.write_text(json.dumps(bad))
            try:load_config(persistent_input)
            except ValueError:pass
            else:raise AssertionError('remote persistent contract weakened required boundaries')
        persistent_file.write_text(json.dumps(persistent_contract))
        for root in ('/outside/runtime','/remote/input/unpinned-runtime'):
            persistent_input.write_text(json.dumps(dict(persistent_config,detection_runtime_root=root)))
            try:load_config(persistent_input)
            except ValueError:pass
            else:raise AssertionError('remote detector runtime escaped declared SHA-pinned roots')
        persistent_input.write_text(json.dumps(persistent_config))
        state=dict(remote['planner_service'],status='ready',pid=123,process_start_ticks='999',model_initialized=False)
        process=NS(stdin=io.StringIO(),stdout=io.StringIO(json.dumps({'type':'service-state','service':state})+'\n'),poll=lambda:None,wait=lambda timeout:0)
        persistent_manager=PlannerService()
        with patch('subprocess.Popen',return_value=process) as spawn:
            persistent_manager.start(persistent,local/'persistent-manager');persistent_manager.thread.join(timeout=2)
        assert 'persistent hold-stdin' in spawn.call_args.args[0][-1]
        assert 'selected GPU is not idle' in spawn.call_args.args[0][-1]
        persistent_manager.close();assert process.stdin.closed
        private=json.loads((prepared.parent/'planner-spec.json').read_text())
        assert 'owner_token' not in spec
        assert spec['session_id']=='test-session' and current['remote_run_root'].startswith(spec['allowed_output_root']+'/')
        stages={s['name']:s for s in build_plan(current,frozen)}
        assert 'curobo_worker.py' in stages['motiongen-plan']['argv'][-1] and '--request' in stages['motiongen-plan']['argv'][-1]
        assert 'selected GPU has an unowned process' in stages['replay-target']['argv'][-1]
        assert all(private['owner_token'] not in ' '.join(s['argv']) for s in stages.values()),'capability exposed in stage argv'
        public=dict(spec,status='ready',pid=123,process_start_ticks='999',model_initialized=False)
        fake=NS(stdin=io.StringIO(),stdout=io.StringIO(json.dumps({'type':'service-state','service':public})+'\n'),poll=lambda:None,wait=lambda timeout:0)
        manager=PlannerService()
        with patch('subprocess.Popen',return_value=fake) as spawn:
            manager.start(prepared,local/'manager');manager.thread.join(timeout=2)
        startup=spawn.call_args.args[0][-1]
        assert ' 1>&2 && ' in startup,'GPU preflight stdout contaminates worker protocol'
        assert private['owner_token'] not in startup and manager.status()['status']=='ready'
        manager.close();assert private['owner_token'] not in (local/'manager/planner-service.log').read_text()
        fresh=NS(stdin=io.StringIO(),stdout=io.StringIO(json.dumps({'type':'service-state','service':public})+'\n'),poll=lambda:None,wait=lambda timeout:0)
        with patch('subprocess.Popen',return_value=fresh) as spawn,patch('detector_service.wait_for_gpu_idle',return_value=True) as wait:
            manager.recover(timeout=2)
        wait.assert_called_once();assert wait.call_args.args==('dsw-scene',)
        assert manager.status()['status']=='ready' and 'resume' in spawn.call_args.args[0][-1]
        with patch.object(manager,'close') as close,patch.object(manager,'start') as start:
            manager.recover(timeout=2)
            with patch.object(manager,'status',side_effect=[dict(status='busy'),dict(status='ready',model_initialized=True)]),patch('time.sleep'):
                assert manager.recover(timeout=2)['model_initialized'] is True
            close.assert_not_called();start.assert_not_called()
        # A busy/ghost release wait failure must not launch a replacement model.
        manager.state=dict(status='failed',error='worker exited',model_initialized=False)
        with patch('detector_service.wait_for_gpu_idle',side_effect=RuntimeError('release wait expired')),patch('subprocess.Popen') as spawn:
            try:manager.recover(timeout=2)
            except RuntimeError as error:assert 'release wait expired' in str(error)
            else:raise AssertionError('failed GPU release admitted a new worker')
            spawn.assert_not_called()
        assert manager.status()['status']=='stopped'
        manager.close()
        shared=startup_config(local/'config.json',local,shared_config=prepared)
        second,second_contract,_=load_config(shared)
        assert second['planner_service']==spec and second['remote_run_root']!=current['remote_run_root']
        assert (shared.parent/'planner-spec.json').read_bytes()==(prepared.parent/'planner-spec.json').read_bytes()
        from app import shared_startup_config
        real={'preparation':{'pipeline_config':str(local/'config.json'),'acquisition':{'keep':'camera config'}},
              'playground':{'pipeline_config':str(shared),'enabled':True}}
        app_config=shared_startup_config(local/'config.json',local/'app-start',real)
        app_service=load_config(app_config)[0]['planner_service']
        assert all(load_config(Path(real[k]['pipeline_config']))[0]['planner_service']==app_service
                   for k in ('preparation','playground'))
        assert real['preparation']['acquisition']=={'keep':'camera config'}
        different=deepcopy(contract);different['provenance']['code_sha256']['/remote/input/workbench/curobo_worker.py']='b'*64
        (local/'different-contract.json').write_text(json.dumps(different))
        different_config=dict(config,contract_path=str(local/'different-contract.json'))
        (local/'different.json').write_text(json.dumps(different_config))
        try:startup_config(local/'different.json',local,shared_config=prepared)
        except ValueError as error:assert '身份不一致' in str(error)
        else:raise AssertionError('incompatible modes shared one solver identity')
        for change in ({'max_lifetime_seconds':0},{'session_id':'foreign'},{'socket':'/tmp/escape'},
                       {'allowed_output_root':'/mnt/data/elsewhere'},{'identity':'b'*64}):
            bad=deepcopy(current);bad['planner_service'].update(change)
            try:validate_config(bad,frozen)
            except ValueError:pass
            else:raise AssertionError('unsafe planner scope accepted')
        # A formal request cannot convert a physical or fixture source into simulation.
        worker=Worker.__new__(Worker);worker.formal=True;worker.engine_cache={};worker.native_code=worker.assets=None
        (local/'input.json').write_text('{}')
        (local/'result.json').write_text('{"status":"geometry-inputs-ready"}')
        request=dict(input=local/'input.json')
        provenance={'input_sha256':'a'*64,'source_sha256':'b'*64}
        with patch('playground_geometry.load',return_value=('planner','observation','data')) as load:
            assert worker.load_geometry(request,{},provenance)[-1] is False
            assert worker.load_geometry(request,{},provenance)[-1] is True and load.call_count==1
            (local/'result.json').write_text('{"status":"failed"}')
            assert worker.load_geometry(request,{},provenance)[-1] is False and load.call_count==2
            assert worker.load_geometry(request,{},dict(provenance,source_sha256='c'*64))[-1] is False
        for index,source in enumerate(({'schema_version':'physical_collection_source.v1','lane':'physical-real'},
                {'schema_version':'dual_gpu_pipeline.v1','lane':'simulated-real','synthetic_fixture':True})):
            (local/'source.json').write_text(json.dumps(source))
            with patch('playground_geometry.load',side_effect=AssertionError('invalid formal source reached geometry')):
                result=worker.plan(dict(input=local/'input.json',source=local/'source.json',output=local/str(index),speed_scale=1.))
            expected='accepted real calibration receipt' if index==0 else 'non-fixture simulation source'
            assert not result['passed'] and expected in result['error']
            assert not (local/str(index)/'plan.json').exists()
        from test_physical_motiongen import fake_inputs
        physical,_,binding=fake_inputs()
        (local/'source.json').write_text(json.dumps(physical));(local/'binding.json').write_text(json.dumps(binding))
        request=dict(input=local/'input.json',source=local/'source.json',physical_binding=local/'binding.json',
                     output=local/'physical-valid',speed_scale=.3)
        with patch.object(worker,'load_geometry',side_effect=ValueError('validated physical input reached geometry')) as loaded:
            result=worker.plan(request)
        assert loaded.call_count==1 and 'validated physical input reached geometry' in result['error']
        bad=deepcopy(binding);bad['source_digest']='wrong';(local/'binding.json').write_text(json.dumps(bad))
        request['output']=local/'physical-invalid'
        with patch.object(worker,'load_geometry',side_effect=AssertionError('bad binding reached geometry')) as loaded:
            result=worker.plan(request)
        assert not loaded.called and 'observation differs' in result['error']
        (local/'binding.json').write_text(json.dumps(binding));request['output']=local/'warm-valid'
        with patch.object(worker,'warmup',return_value=dict(model_initialized=True,model_key='model',policy_planned=False)) as initialized, \
                patch.object(worker,'plan',side_effect=AssertionError('warmup planned a policy')):
            result=worker.warmup_request(request)
        assert initialized.call_count==1 and result['status']=='motiongen-warm-ready' and result['motion_requested'] is False

        state=dict(**private,status='ready',pid=123,process_start_ticks='999',model_initialized=False)
        rows=['']
        def read_text(path,*args,**kwargs):
            if path.name=='status.json':return json.dumps(state)
            if path.name=='stat':return '123 (planner spaces) '+' '.join(['S']+['0']*18+['999']+['0']*3)
            raise AssertionError(str(path))
        def output(args,**kwargs):return 'GPU-a, 65000\n' if '-i' in args else rows[0]
        def admission():
            with patch('planner_service.load_private_spec',return_value=private),patch('pathlib.Path.read_text',read_text),patch('pathlib.Path.read_bytes',return_value=b'python\0/remote/input/workbench/curobo_worker.py\0--service-spec-file\0'),patch('subprocess.check_output',output),patch('sys.argv',['check',json.dumps(private_spec_arguments(current))]):
                exec(compile(OWNED_GPU_CHECK,'admission','exec'),{})
        admission();state['model_initialized']=True;rows[0]='GPU-a, 123, 4000\n';admission()
        for invalid in ('','GPU-a, 777, 1\n','GPU-a, 123, 20000\n','GPU-a, 123, 4000\nGPU-a, 777, 1\n'):
            rows[0]=invalid
            try:admission()
            except AssertionError:pass
            else:raise AssertionError('unowned/missing/overbudget GPU process accepted')

        # Exercise the actual Unix socket server with a non-planning worker.
        reads=local/'reads';reads.mkdir();outputs=local/'outputs';outputs.mkdir()
        for name in ('planning-input.json','result.json','source.json'):(reads/name).write_text('{}')
        ipc=dict(private,root='/tmp/rbw-planner-'+uuid4().hex[:16],allowed_read_roots=[str(reads)],allowed_output_root=str(outputs),
                 persistent_service=True,max_lifetime_seconds=None)
        ipc['socket']=ipc['root']+'/p';validate_spec(ipc,private=True)
        payload=json.dumps(ipc,sort_keys=True,separators=(',',':')).encode()
        specfile=outputs/'planner-spec.json';specfile.write_bytes(payload);specfile.chmod(0o600)
        from run_pipeline import digest
        assert load_private_spec(specfile,digest(payload))==ipc
        # Reuse only a dead, identically owned runtime scope; keep old outputs.
        root=Path(ipc['root']);root.mkdir()
        state=dict(ipc,pid=999999999,process_start_ticks='999',status='stopped')
        (root/'status.json').write_text(json.dumps(state));(root/'p').write_text('old socket')
        (outputs/'historical-result.json').write_text('{}')
        def restart_scope():
            with patch('sys.stdin',NS(buffer=io.BytesIO(payload+b'\n'))),patch('sys.argv',['install','[]',digest(payload),'resume']):
                exec(compile(INSTALL_SPEC,'install','exec'),{})
        foreign=dict(state,owner_token='foreign');(root/'status.json').write_text(json.dumps(foreign))
        try:restart_scope()
        except AssertionError:pass
        else:raise AssertionError('foreign planner scope deleted')
        assert (root/'p').exists()
        (root/'status.json').write_text(json.dumps(state));(root/'unknown').write_text('keep')
        try:restart_scope()
        except AssertionError:pass
        else:raise AssertionError('unknown runtime files deleted')
        assert (root/'unknown').exists();(root/'unknown').unlink()
        restart_scope();assert not root.exists() and (outputs/'historical-result.json').exists()
        specfile.chmod(0o644)
        try:load_private_spec(specfile,digest(payload))
        except ValueError:pass
        else:raise AssertionError('public capability file accepted')
        specfile.chmod(0o600)
        program='''import json,sys\nfrom pathlib import Path\nfrom unittest.mock import patch\nfrom curobo_worker import serve\nclass Fake:\n def __init__(self):self.engine_cache={};self.calls=0\n def warmup_request(self,r):\n  r['output'].mkdir();self.engine_cache.update(engine=True,key='cpu-model')\n  value=dict(type='result',passed=True,status='motiongen-warm-ready',output=str(r['output']),model_initialized=True,policy_planned=False,motion_requested=False)\n  (r['output']/'result.json').write_text(json.dumps(value));return value\n def plan(self,r):\n  self.calls+=1;r['output'].mkdir();self.engine_cache['engine']=True\n  value=dict(type='result',passed=True,status='motiongen-plan-ready',output=str(r['output']),calls=self.calls)\n  (r['output']/'result.json').write_text(json.dumps(value));return value\ns=json.loads(sys.argv[1])\nwith patch('curobo_worker.process_start_ticks',return_value='999'):serve(Fake(),s,tuple(Path(p).resolve() for p in s['allowed_read_roots']),Path(s['allowed_output_root']).resolve(),dict(type='ready',passed=True))\n'''
        child=subprocess.Popen([sys.executable,'-B','-c',program,json.dumps(ipc)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,
            env=dict(os.environ,PYTHONPATH=str(Path(__file__).parent)))
        try:
            line=child.stdout.readline();assert line,child.stderr.read()
            first=json.loads(line);assert first['service']['status']=='ready' and 'owner_token' not in first['service']
            ready=json.loads(child.stdout.readline());assert ready['passed']
            request=dict(input=str(reads/'planning-input.json'),source=str(reads/'source.json'),output=str(outputs/'one'))
            warmed=request_service(ipc['socket'],ipc['owner_token'],dict(request,operation='warmup',output=str(outputs/'warm')),timeout=5)
            assert warmed['model_initialized'] and warmed['policy_planned'] is False and warmed['motion_requested'] is False
            assert json.loads((Path(ipc['root'])/'status.json').read_text())['model_initialized'] is True
            assert request_service(ipc['socket'],ipc['owner_token'],request,timeout=5)['calls']==1
            request['output']=str(outputs/'two')
            assert request_service(ipc['socket'],ipc['owner_token'],request,timeout=5)['calls']==2
            request['output']=str(outputs/'forbidden')
            try:request_service(ipc['socket'],'foreign',request,timeout=5)
            except ValueError:pass
            else:raise AssertionError('foreign owner accepted')
            assert not Path(request['output']).exists()
            child.stdin.close();child.stdin=None
            _,errors=child.communicate(timeout=5)
            assert child.returncode==0,errors
            assert not Path(ipc['socket']).exists()
            assert json.loads((Path(ipc['root'])/'status.json').read_text())['status']=='stopped'
        finally:
            if child.poll() is None:child.kill();child.communicate()
            for file in Path(ipc['root']).glob('*'):file.unlink()
            Path(ipc['root']).rmdir()
        manager=PlannerService();process=NS(stdin=io.StringIO(),wait=lambda timeout:0)
        manager.process=process;manager.close();assert process.stdin.closed
        # Busy cancellation is the supervisor's responsibility, not the socket
        # server's next idle select. Exercise its real child group without GPU.
        from detector_service import SUPERVISOR
        import time
        workload='import json,os,pathlib,sys,time; pathlib.Path(sys.argv[1]).write_text(json.dumps({"pid":os.getpid()})); time.sleep(30)'
        for lifetime in ('20','persistent'):
            busy=local/('busy-'+lifetime+'.json')
            supervisor=subprocess.Popen([sys.executable,'-B','-c',SUPERVISOR,json.dumps([sys.executable,'-B','-c',workload,str(busy)]),lifetime,'hold-stdin'],stdin=subprocess.PIPE)
            try:
                deadline=time.monotonic()+5
                while not busy.exists() and time.monotonic()<deadline:time.sleep(.01)
                assert busy.exists(),'busy child never started'
                pid=json.loads(busy.read_text())['pid']
                supervisor.stdin.close();supervisor.wait(timeout=5)
                try:os.kill(pid,0)
                except ProcessLookupError:pass
                else:raise AssertionError('busy owned child survived stdin cancellation')
            finally:
                if supervisor.poll() is None:supervisor.kill();supervisor.wait()
    assert 'torch' not in sys.modules
    print('planner formal source, frozen scope, IPC reuse/EOF, and owned GPU admission checks passed')


if __name__=='__main__':main()
