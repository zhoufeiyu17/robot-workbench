"""CPU-only local/legacy routing and resource admission; never invokes a GPU."""
from copy import deepcopy
import hashlib
import io
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
from tempfile import TemporaryDirectory
import threading
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

from local_execution import TARGET,validate,gpu_check
from run_pipeline import PATH_KEYS,PIPELINE_LANES,transport,ssh,isaac_environment,load_config,build_plan,bounded_command,pinned_files,digest
from playground_pipeline import native_stage,geometry_stage,fetch
from planner_service import startup_config,validate_config
from launch_environment import clean_command

WB=Path(__file__).resolve().parent


class LocalExecutionTest(unittest.TestCase):
    def setUp(self):
        self.tmp=TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.root=Path(self.tmp.name).resolve();self.proc=self.root/'proc';self.proc.mkdir()
        script=self.root/'serve_physx_device.py';script.write_text('# CPU fixture, not a running plant\n')
        plant_scope='/mnt/data/feiyu.zhou/runs/robot-workbench/resident-fixture'
        self.execution=dict(TARGET,session_id='cpu-session',max_processes=4,min_free_memory_mib=1024,
            plant=dict(session_id='cpu-session',pid=101,process_start_ticks='123',scope=plant_scope,
                script=str(script),script_sha256=hashlib.sha256(script.read_bytes()).hexdigest(),
                argv=['python3',str(script),'--output',plant_scope,'--gpu','1'],max_memory_mib=12000))
        remote=plant_scope+'/collection/jobs'
        self.contract=dict(schema_version='anyverse.dsw-codex-experiment.v1',identity={'session_id':'cpu-session'},
            execution=deepcopy(self.execution),resources=dict(lanes={TARGET['instance_id']:[1]},max_gpu_count_per_instance=1,
                max_workers_per_instance=4,max_cpu_threads=4,require_gpu_idle_check=True,max_wall_time_minutes=20),
            filesystem=dict(readonly_roots=['/remote/input',str(self.root),str(WB)],writable_roots=[remote,plant_scope,str(self.root)],allow_overwrite=False,allow_delete=False),
            provenance=dict(code_sha256={str(WB/name):'a'*64 for name in ('curobo_worker.py','planner_service.py','motiongen_plan.py',
                'motiongen_motion.py','gpu_acceptance.py','curobo_robot_model.py','cup_contact.py','cup_envelope.py','region_self_collision.py')}))
        self.contract['provenance']['code_sha256'][str(script)]=self.execution['plant']['script_sha256']
        self.config={key:'/remote/input/'+key for key in PATH_KEYS}
        self.config.update(execution=deepcopy(self.execution),environment='fake',contract_path=str(self.root/'contract.json'),
            resident_deadline_wall_ns=9_000_000_000_000_000_000,
            workbench_code=str(WB),remote_run_root=remote,planning_backend='motiongen',curobo_root='/remote/input/curobo',
            opening_mm=83.,finger_envelope_sha256='a'*64)
        self.contract['resources']['resident_deadline_wall_ns']=self.config['resident_deadline_wall_ns']
        self.config['detection_runtime_root']='/remote/input/detector-runtime'
        self.contract['provenance']['code_sha256'].update({self.config['detection_runtime_root']+p:'d'*64 for p in
            ('/scripts/run_online_detection_episode.py','/ros2/online_detection_ros/fixture.py')})
        self.save()
        self.process(101,self.execution['plant']['argv'])

    def save(self):
        (self.root/'contract.json').write_text(json.dumps(self.contract))
        (self.root/'config.json').write_text(json.dumps(self.config))

    def process(self,pid,argv,*,session=None,start='123',mask='1',extra_env=None):
        root=self.proc/str(pid);root.mkdir(exist_ok=True)
        (root/'stat').write_text(str(pid)+' (fixture) '+' '.join(['S']+['0']*18+[start]))
        (root/'cmdline').write_bytes(b'\0'.join(v.encode() for v in argv)+b'\0')
        (root/'environ').write_text('CUDA_VISIBLE_DEVICES='+mask+'\0'+('ROBOT_WORKBENCH_SESSION_ID='+session+'\0' if session else '')+
            ''.join(k+'='+v+'\0' for k,v in (extra_env or {}).items()))

    def query(self,pids=(101,),*,uuid=TARGET['gpu_uuid'],free=40000,memory=1000):
        def result(argv,**kwargs):
            self.assertEqual(argv,['nvidia-smi','-i','1','-q','-x'])
            rows=''.join(f'<process_info><pid>{pid}</pid><type>C+G</type><used_memory>{memory} MiB</used_memory></process_info>' for pid in pids)
            return f'<nvidia_smi_log><gpu><uuid>{uuid}</uuid><fb_memory_usage><free>{free} MiB</free></fb_memory_usage><processes>{rows}</processes></gpu></nvidia_smi_log>'
        return result

    def test_legacy_shape_and_explicit_local_never_fall_back(self):
        legacy={k:v for k,v in self.config.items() if k!='execution'}
        self.assertEqual(transport(legacy,'dsw-scene','command'),ssh('dsw-scene','command'))
        self.assertEqual(isaac_environment(legacy,'dsw-scene')['CUDA_VISIBLE_DEVICES'],'0')
        self.assertEqual(isaac_environment(self.config,'dsw-scene')['CUDA_VISIBLE_DEVICES'],'1')
        self.assertEqual(isaac_environment(self.config,'dsw-scene')['ROBOT_WORKBENCH_SESSION_ID'],'cpu-session')
        cfg,contract,_=load_config(self.root/'config.json')
        stages=build_plan(cfg,contract)
        detection=next(stage for stage in stages if stage['name']=='detect')
        self.assertIn('ROBOT_WORKBENCH_DETECTOR_RUNTIME_ROOT=/remote/input/detector-runtime',detection['argv'][-1])
        self.assertTrue(all(any(value.endswith('/owned_process.py') for value in s['argv']) for s in stages))
        for stage in stages:
            self.assertNotIn('root@47.116.111.83',stage['argv'])
            self.assertNotIn('HostKeyAlias',str(stage['argv']))
        with patch('run_pipeline.ssh',side_effect=AssertionError('SSH fallback')):
            self.assertEqual(transport(cfg,'dsw-scene','command'),clean_command(['bash','-c','command']))
            with self.assertRaises(ValueError):transport({**cfg,'execution':{**cfg['execution'],'gpu_index':0}},'dsw-scene','command')
        native=native_stage(cfg,'geometry-inputs',cfg['remote_run_root']+'/scene')
        self.assertIn('--gpu 1',native['argv'][-1]);self.assertIn(TARGET['instance_id'],native['argv'][-1])
        self.assertIn('--execution-json',native['argv'][-1])
        plan=geometry_stage(cfg,cfg['remote_run_root']+'/plan')
        self.assertIn('CUDA_VISIBLE_DEVICES=1',plan['argv'][-1])
        bounded,seconds=bounded_command(native,100)
        self.assertEqual(bounded[bounded.index('--timeout')+1],str(seconds))

    def test_local_fetch_uses_local_transport(self):
        source=self.root/'source';source.write_bytes(b'owned output')
        with patch('run_pipeline.ssh',side_effect=AssertionError('SSH fallback')):
            fetch(str(source),self.root/'copy',execution_config=self.config)
        self.assertEqual((self.root/'copy').read_bytes(),source.read_bytes())

    def test_local_output_is_an_explicit_strict_child_of_its_plant_scope(self):
        scope=str(self.root/'resident');output=scope+'/collection/jobs'
        for value in (self.config['execution'],self.contract['execution']):
            value['plant']['scope']=scope
            value['plant']['argv'][value['plant']['argv'].index('--output')+1]=scope
        self.config['remote_run_root']=output
        self.contract['filesystem']['writable_roots']=[scope,output]
        self.save()
        cfg,_,_=load_config(self.root/'config.json')
        self.assertEqual(cfg['remote_run_root'],output)
        for invalid in (scope,scope+'-sibling/jobs',str(self.root/'another-run')):
            with self.subTest(invalid=invalid):
                self.config['remote_run_root']=invalid
                self.contract['filesystem']['writable_roots']=[scope,invalid]
                self.save()
                with self.assertRaisesRegex(ValueError,'strict child'):load_config(self.root/'config.json')
        self.config['remote_run_root']=output;self.contract['filesystem']['writable_roots']=[scope];self.save()
        with self.assertRaisesRegex(ValueError,'non-overwriting output root'):load_config(self.root/'config.json')
        # The same local path is still rejected when no explicit local deployment exists.
        self.config.pop('execution');self.contract.pop('execution');self.save()
        with self.assertRaisesRegex(ValueError,'remote output must be'):load_config(self.root/'config.json')

    def test_exact_session_scope_uuid_pid_mask_and_budgets(self):
        result=gpu_check(self.config,proc_root=self.proc,query=self.query())
        self.assertEqual(result['gpu'],1)
        for kwargs in ({'pids':(101,999)},{'uuid':'other-card'},{'free':1},{'memory':13000}):
            with self.subTest(kwargs=kwargs),self.assertRaises(ValueError):gpu_check(self.config,proc_root=self.proc,query=self.query(**kwargs))
        self.process(101,self.execution['plant']['argv'],start='124')
        with self.assertRaises(ValueError):gpu_check(self.config,proc_root=self.proc,query=self.query())
        self.process(101,self.execution['plant']['argv'],mask='0')
        with self.assertRaises(ValueError):gpu_check(self.config,proc_root=self.proc,query=self.query())
        bad=deepcopy(self.contract);bad['identity']['session_id']='other-session'
        with self.assertRaises(ValueError):validate(self.config,bad)
        self.contract['resources']['resident_deadline_wall_ns']-=1;self.save()
        with self.assertRaisesRegex(ValueError,'same frozen resident wall deadline'):load_config(self.root/'config.json')

    def test_frozen_service_identity_ignores_new_frames_but_not_code_or_model(self):
        prepared=startup_config(self.root/'config.json',self.root/'startup')
        cfg,contract,_=load_config(prepared)
        private=json.loads((prepared.parent/'planner-spec.json').read_text())
        self.assertLess((prepared.parent/'planner-spec.json').stat().st_size,65536)
        self.assertEqual(private['compute_resource'],TARGET)
        contract['provenance'].setdefault('input_sha256',{})['/remote/input/fresh-capture/frame.npz']='b'*64
        validate_config(cfg,contract)
        broken=deepcopy(contract);broken['provenance']['code_sha256'][str(WB/'motiongen_plan.py')]='c'*64
        with self.assertRaises(ValueError):validate_config(cfg,broken)
        with self.assertRaises(ValueError):validate_config({**cfg,'assets':'/remote/input/new-install'},contract)
        # The legacy remote private spec and identity calculation remain v1.
        self.config.pop('execution');self.contract.pop('execution')
        old_root=self.config['remote_run_root'];self.config['remote_run_root']='/mnt/data/feiyu.zhou/runs/robot_sim/legacy-test'
        self.contract['filesystem']['writable_roots']=[self.config['remote_run_root'] if p==old_root else p for p in self.contract['filesystem']['writable_roots']]
        self.contract['resources'].update(max_workers_per_instance=2,lanes={v['instance_id']:[v['gpu']] for v in PIPELINE_LANES.values()})
        self.save();legacy=startup_config(self.root/'config.json',self.root/'legacy')
        old,old_contract,_=load_config(legacy)
        self.assertEqual(old['planner_service']['identity'],digest(json.dumps(pinned_files(self.contract),sort_keys=True).encode()))
        self.assertNotIn('input_pins_sha256',old['planner_service'])
        self.assertNotIn('compute_resource',old['planner_service'])

    def test_only_verified_private_worker_peers_are_allowed(self):
        cfg=deepcopy(self.config)
        root=self.root/'detector';root.mkdir()
        spec=dict(root=str(root),socket=str(root/'s'),owner_token='private-fixture-token',identity={'code':'pinned'},
            allowed_read_roots=[str(self.root)],allowed_output_root=str(self.root/'jobs'),max_memory_mib=8000,min_free_memory_mib=2000,
            max_lifetime_seconds=1200)
        cfg['detector_service']=spec
        state=dict(spec,pid=202,process_start_ticks='123',service_root=str(root),status='ready',gpu=0,physical_gpu=1,logical_cuda_index=0)
        (root/'status.json').write_text(json.dumps(state))
        argv=['python3',cfg['detection_script'],'--serve','--service-root',str(root),'--owner-token',spec['owner_token']]
        self.process(202,argv,session='cpu-session')
        from local_execution import detector_environment
        env=detector_environment(cfg)
        from detector_service import launch_command
        command=launch_command(cfg)
        self.assertIn('ROBOT_WORKBENCH_DETECTOR_RUNTIME_ROOT',command)
        self.assertIn('CUDA_VISIBLE_DEVICES=1',command)
        self.process(202,argv,session='cpu-session',extra_env=env)
        result=gpu_check(cfg,require='detector',proc_root=self.proc,query=self.query((101,202)))
        self.assertEqual({row['pid'] for row in result['processes']},{101,202})
        self.process(202,argv,session='another-session',extra_env=env)
        with self.assertRaises(ValueError):gpu_check(cfg,require='detector',proc_root=self.proc,query=self.query((101,202)))
        self.process(202,argv,session='cpu-session',extra_env=env)
        state['owner_token']='another-private-scope';(root/'status.json').write_text(json.dumps(state))
        with self.assertRaises(ValueError):gpu_check(cfg,require='detector',proc_root=self.proc,query=self.query((101,202)))

    def test_private_planner_lazy_cuda_context_and_foreign_scope(self):
        cfg=deepcopy(self.config);root=self.root/'planner';root.mkdir()
        private=dict(root=str(root),socket=str(root/'s'),session_id='cpu-session',owner_token='private-planner-token',
            identity='a'*64,allowed_read_roots=[str(self.root)],allowed_output_root=str(self.root/'jobs'),
            max_memory_mib=8000,min_free_memory_mib=2000,max_lifetime_seconds=1200)
        spec={k:v for k,v in private.items() if k!='owner_token'};spec['spec_sha256']='b'*64
        cfg['planner_service']=spec
        state=dict(private,spec_sha256=spec['spec_sha256'],status='ready',pid=303,process_start_ticks='123',model_initialized=False)
        (root/'status.json').write_text(json.dumps(state))
        argv=['python3',cfg['workbench_code']+'/curobo_worker.py','--service-spec-file',spec['allowed_output_root']+'/planner-spec.json']
        self.process(303,argv,session='cpu-session')
        with patch('planner_service.load_private_spec',return_value=private):
            gpu_check(cfg,require='planner',proc_root=self.proc,query=self.query())
            state['model_initialized']=True;(root/'status.json').write_text(json.dumps(state))
            with self.assertRaises(ValueError):gpu_check(cfg,require='planner',proc_root=self.proc,query=self.query())
            gpu_check(cfg,require='planner',proc_root=self.proc,query=self.query((101,303)))
        with patch('planner_service.load_private_spec',return_value=dict(private,session_id='foreign')):
            with self.assertRaises(ValueError):gpu_check(cfg,require='planner',proc_root=self.proc,query=self.query((101,303)))

    def test_local_cancel_reaps_only_owned_cpu_process(self):
        from cancellable_process import run
        mark=self.root/'child.pid';cancel=threading.Event();outcome=[]
        script='import os,time,pathlib;pathlib.Path('+repr(str(mark))+').write_text(str(os.getpid()));time.sleep(60)'
        argv=transport(self.config,'dsw-scene',shlex.join([sys.executable,'-c',script]),timeout=60)
        def work():
            try:run(argv,cancel=cancel.is_set,timeout=60)
            except InterruptedError:outcome.append('cancelled')
        unrelated=subprocess.Popen([sys.executable,'-c','import time;time.sleep(60)'])
        worker=threading.Thread(target=work);worker.start()
        try:
            deadline=time.monotonic()+3
            while not mark.exists() and time.monotonic()<deadline:time.sleep(.01)
            self.assertTrue(mark.exists());pid=int(mark.read_text());cancel.set();worker.join(5)
            self.assertFalse(worker.is_alive());self.assertEqual(outcome,['cancelled'])
            status=subprocess.run(['ps','-o','stat=','-p',str(pid)],capture_output=True,text=True).stdout.strip()
            self.assertTrue(not status or status.startswith('Z'))
            self.assertIsNone(unrelated.poll())
        finally:
            cancel.set();worker.join(5);unrelated.terminate();unrelated.wait(5)

    def test_local_worker_supervisor_keeps_eof_and_terminate_cleanup(self):
        from detector_service import LOCAL_SUPERVISOR
        for mode,stop in (('no-stdin','eof'),('hold-stdin','terminate')):
            marker=self.root/(mode+'.pid')
            script='import os,time,pathlib;pathlib.Path('+repr(str(marker))+').write_text(str(os.getpid()));time.sleep(60)'
            supervisor=subprocess.Popen([sys.executable,'-c',LOCAL_SUPERVISOR,
                json.dumps([sys.executable,'-c',script]),'60',mode,str(WB)],stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
            try:
                deadline=time.monotonic()+3
                while not marker.exists() and time.monotonic()<deadline:time.sleep(.01)
                self.assertTrue(marker.exists())
                if stop=='eof':supervisor.stdin.close()
                else:supervisor.terminate()
                supervisor.wait(5)
                status=subprocess.run(['ps','-o','stat=','-p',marker.read_text()],capture_output=True,text=True).stdout.strip()
                self.assertTrue(not status or status.startswith('Z'))
            finally:
                if not supervisor.stdin.closed:supervisor.stdin.close()
                if supervisor.poll() is None:supervisor.terminate();supervisor.wait(25)
                supervisor.stderr.close()

    def test_five_roles_require_exact_preview_and_auxiliary_registration(self):
        from local_execution import role_spec_digest
        execution=deepcopy(self.execution);scope=self.root/'resident';scope.mkdir()
        execution['plant']['scope']=str(scope);execution['plant']['argv'][3]=str(scope)
        preview=scope/'preview';preview.mkdir();config=preview/'config.json';config.write_text('{}')
        script=self.root/'joint_head_workbench_live.py';script.write_text('# renderer fixture\n')
        auxiliary=self.root/'playground_sim.py';auxiliary.write_text('# auxiliary fixture\n')
        digest=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
        execution.update(max_processes=5,roles=dict(preview=dict(root=str(preview),registration=str(scope/'preview.json'),
            script=str(script),script_sha256=digest(script),config_path=str(config),config_sha256=digest(config),
            token_file=str(preview/'token'),port=19001,max_memory_mib=8000),
            auxiliary=dict(root=str(scope),registration=str(scope/'auxiliary.json'),entrypoints={str(auxiliary):digest(auxiliary)},max_memory_mib=9000)))
        cfg=dict(self.config,execution=execution)
        self.process(101,execution['plant']['argv'])
        def register(role,pid,entry,options,instance=None):
            spec=execution['roles'][role]
            argv=['python3',str(entry),*options,'--execution-json',json.dumps(execution)]
            self.process(pid,argv,session='cpu-session')
            state=dict(role=role,session_id='cpu-session',spec_sha256=role_spec_digest(spec),pid=pid,
                process_start_ticks='123',argv=argv,entrypoint=str(entry),entrypoint_sha256=digest(entry),
                config_sha256=spec.get('config_sha256'),renderer_instance_id=instance,status='ready')
            Path(spec['registration']).write_text(json.dumps(state));return state
        state=register('preview',404,script,['--config',str(config),'--output',str(preview/'frames'),
            '--token-file',str(preview/'token'),'--port','19001','--render-gpu','1','--duration-s','600','--display-only'],'f'*32)
        register('auxiliary',405,auxiliary,['--output',str(scope/'replay'),'--gpu','1'])
        result=gpu_check(cfg,require='preview',proc_root=self.proc,query=self.query((101,404,405)))
        self.assertEqual({row['pid'] for row in result['processes']},{101,404,405})
        with self.assertRaises(ValueError):gpu_check(cfg,proc_root=self.proc,query=self.query((101,404,405,999)))
        self.process(404,state['argv'],session='foreign')
        with self.assertRaises(ValueError):gpu_check(cfg,proc_root=self.proc,query=self.query((101,404,405)))
        self.process(404,state['argv'],session='cpu-session')
        config.write_text('{"changed":true}')
        with self.assertRaises(ValueError):gpu_check(cfg,proc_root=self.proc,query=self.query((101,404,405)))
        bad=deepcopy(execution);bad['max_processes']=4
        with self.assertRaises(ValueError):validate({'execution':bad})

    def test_asset_conversion_uses_physical_gpu_one_with_logical_cuda_zero(self):
        from a1_isaac_assets import conversion_device,main
        self.assertEqual(conversion_device(0)['active_gpu'],0)
        options=conversion_device(1)
        self.assertEqual(options['active_gpu'],1);self.assertEqual(options['device'],'cuda:0')
        self.assertIn('activeGpu=1',options['kit_args'])
        called=[]
        def launcher(**kwargs):called.append(kwargs);raise RuntimeError('stop before any Isaac import')
        with patch('a1_isaac_assets.prepare',return_value={}),patch.dict('sys.modules',{'isaaclab.app':NS(AppLauncher=launcher)}),\
             patch.dict(os.environ,{'CUDA_VISIBLE_DEVICES':'0'}),patch.object(sys,'argv',['convert','--bundle','/unused','--base-assets','/unused','--output','/unused','--gpu','1']):
            with self.assertRaisesRegex(RuntimeError,'stop before'):main()
            self.assertEqual(os.environ['CUDA_VISIBLE_DEVICES'],'1')
        self.assertEqual(called,[options])


if __name__=='__main__':unittest.main()
