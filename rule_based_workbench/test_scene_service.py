"""CPU protocol/reset binding checks; no Isaac, GPU, plant, or network routes."""
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import shutil
import shlex
import subprocess
import sys
from tempfile import TemporaryDirectory
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock,patch
from uuid import uuid4

from curobo_worker import request_service
from scene_worker import Worker,request_paths,reset_suite


class SceneServiceTest(unittest.TestCase):
    def test_reset_identity_is_request_scoped_and_original_inputs_are_unchanged(self):
        source={'run_id':'observed-1'};binding={'calibration':'installed'}
        suite={'episodes':[{'episode_id':'physical-observed','machine_asset':'machine-v1'}]}
        original=deepcopy((source,suite,binding))
        first,one=reset_suite(source,suite,binding,'/owned/first')
        second,two=reset_suite(source,suite,binding,'/owned/second')
        third,three=reset_suite(dict(source,run_id='observed-2'),suite,binding,'/owned/third')
        self.assertEqual((source,suite,binding),original)
        self.assertEqual(one['input_digest'],two['input_digest']);self.assertNotEqual(one['input_digest'],three['input_digest'])
        self.assertEqual(len({row['reset_episode_id'] for row in (one,two,three)}),3)
        for current,row in ((first,one),(second,two),(third,three)):
            self.assertEqual(row['source_episode_id'],'physical-observed')
            self.assertEqual(current['episodes'][0]['episode_id'],row['reset_episode_id'])

    @unittest.skipUnless(os.environ.get('ROBOT_WORKBENCH_TEST_NATIVE_CODE'),'explicit frozen native source needed for its actual reset guard')
    def test_actual_native_reset_guard_accepts_distinct_internal_ids_without_clearing_state(self):
        from playground_geometry import configure_native
        with patch.dict(sys.modules):
            configure_native(Path(os.environ['ROBOT_WORKBENCH_TEST_NATIVE_CODE']))
            from robot_sim.tasks.manager_based.coffee_make.generalization_execution import ControlSpec,GuardedExecution
            from robot_sim.specs.action import ac_one_dual_arm_zero_home_action_spec,ArmCommand,DualArmCommand
            spec=ac_one_dual_arm_zero_home_action_spec();hold=spec.encode(DualArmCommand(ArmCommand((0.,)*6,True),ArmCommand((0.,)*6,True)))
            loop=GuardedExecution(ControlSpec(240,8,2.,5.,300.),spec)
            source={'run_id':'observed-1'};suite={'episodes':[{'episode_id':'physical-observed','machine_asset':'machine-v1'}]}
            loop.reset('physical-observed',hold)
            with self.assertRaisesRegex(ValueError,'new nonempty episode identity'):loop.reset('physical-observed',hold)
            # New observations may intentionally keep the external episode ID;
            # retries at distinct output roots still require distinct internal resets.
            for index,current in enumerate((source,source,dict(source,run_id='observed-2'))):
                internal,identity=reset_suite(current,suite,{},'/owned/request-'+str(index))
                loop.reset(internal['episodes'][0]['episode_id'],hold)
                self.assertEqual(loop.episode_id,identity['reset_episode_id']);self.assertEqual(loop.physics_steps,0)
                with self.assertRaisesRegex(ValueError,'new nonempty episode identity'):loop.reset(loop.episode_id,hold)

    def test_generated_scene_rpc_stage_obeys_real_bounded_command_contract(self):
        from test_local_execution import LocalExecutionTest
        from playground_pipeline import native_stage
        from run_pipeline import bounded_command
        fixture=LocalExecutionTest('test_frozen_service_identity_ignores_new_frames_but_not_code_or_model')
        fixture.setUp();self.addCleanup(fixture.doCleanups)
        cfg=deepcopy(fixture.config);cfg['scene_service']=dict(root='/tmp/rbw-scene-fixture',spec_sha256='a'*64)
        for local in (True,False):
            candidate=deepcopy(cfg)
            if not local:candidate.pop('execution')
            stage=native_stage(candidate,'geometry-inputs',candidate['remote_run_root']+'/scene',timeout_s=600)
            bounded,seconds=bounded_command(stage,125)
            self.assertEqual(seconds,80)
            remote=shlex.split(bounded[-1]);self.assertEqual(remote[:6],['timeout','-k','30','80','bash','-c'])
            client=shlex.split(remote[-1]);request=json.loads(client[client.index('--request')+1])
            self.assertEqual(request['output']+'/result.json',stage['result_path'])
            self.assertIn(candidate['workbench_code']+'/scene_worker.py',client)
            self.assertEqual(client[client.index('--service-spec-file')+1],'/tmp/rbw-scene-fixture/private-spec.json')
            self.assertNotIn('--gpu',client);self.assertNotIn('--execution-json',client)
            self.assertEqual(stage['label'],'更新本轮场景与首帧 · 常驻服务')
            if local:
                self.assertEqual(bounded[bounded.index('--timeout')+1],'80')
                self.assertEqual(bounded[bounded.index('--failure-result')+1],stage['result_path'])
                self.assertEqual(stage['execution_config'],dict(execution=candidate['execution'],workbench_code=candidate['workbench_code']))
                self.assertEqual(stage['terminate_grace_s'],25);self.assertNotIn('ssh',bounded)
            else:self.assertEqual(bounded[0],'ssh')

    def test_real_startup_pin_allows_only_exact_file_without_expanding_rpc_scope(self):
        from test_local_execution import LocalExecutionTest,WB
        from planner_service import startup_config as planner_startup
        from scene_service import startup_config as scene_startup,SceneService
        from scene_worker import main
        from run_pipeline import load_config
        fixture=LocalExecutionTest('test_frozen_service_identity_ignores_new_frames_but_not_code_or_model')
        fixture.setUp();self.addCleanup(fixture.doCleanups)
        cfg,contract=fixture.config,fixture.contract;scope=fixture.root/'plant';scope.mkdir()
        execution=deepcopy(fixture.execution);execution['max_processes']=6
        execution['plant']['scope']=str(scope);execution['plant']['argv'][3]=str(scope)
        preview=scope/'preview';preview.mkdir()
        execution['roles']=dict(preview=dict(root=str(preview),registration=str(scope/'preview.json'),
            script=str(WB/'playground_sim.py'),script_sha256='a'*64,config_path=str(preview/'config.json'),config_sha256='a'*64,
            token_file=str(preview/'token'),port=19001,max_memory_mib=8192),
            **{role:dict(root=str(scope),registration=str(scope/(role+'.json')),entrypoints={str(WB/script):'a'*64},max_memory_mib=12288)
               for role,script in [('auxiliary','playground_sim.py'),('scene','scene_worker.py')]})
        cfg.update(execution=execution,remote_run_root=str(scope/'collection/jobs'))
        contract.update(execution=deepcopy(execution));contract['resources']['max_workers_per_instance']=6
        contract['filesystem']['writable_roots']=[str(scope),cfg['remote_run_root'],str(fixture.root)]
        contract['provenance']['code_sha256'].update({str(WB/name):'a'*64 for name in ('scene_service.py','scene_worker.py','playground_sim.py')})
        with TemporaryDirectory() as other:
            external=Path(other).resolve();finger=external/'finger-envelope.json';finger.write_text('{"frozen":true}')
            sibling=external/'not-authorized.json';sibling.write_bytes(finger.read_bytes())
            digest=hashlib.sha256(finger.read_bytes()).hexdigest()
            cfg.update(finger_envelope=str(finger),finger_envelope_sha256=digest)
            contract['filesystem']['readonly_roots'].append(str(finger))
            contract['provenance']['input_sha256']={str(finger):digest}
            fixture.save()
            planner=planner_startup(fixture.root/'config.json',fixture.root/'start')
            candidate=scene_startup(planner,fixture.root/'start');actual,_,_=load_config(candidate)
            spec=actual['scene_service'];self.assertEqual(spec['allowed_read_roots'],actual['planner_service']['allowed_read_roots'])
            self.assertNotIn(str(finger),spec['allowed_read_roots']);self.assertNotIn(str(external),spec['allowed_read_roots'])
            self.assertEqual(spec['finger_envelope'],dict(path=str(finger),sha256=digest))
            runtime=Path(spec['root']);runtime.mkdir(mode=0o700);self.addCleanup(shutil.rmtree,runtime)
            private=runtime/'private-spec.json';private.write_bytes((candidate.parent/'scene-spec.json').read_bytes());private.chmod(0o600)
            output=Path(spec['allowed_output_root']);output.mkdir(parents=True)
            argv=['scene_worker.py','--service-spec-file',str(private),'--spec-sha256',spec['spec_sha256'],
                '--assets',actual['assets'],'--native-code',actual['native_code'],'--finger-envelope',str(finger),
                '--finger-envelope-sha256',digest,'--output',str(output/'valid'),
                '--gpu','1','--instance',execution['instance_id'],'--execution-json',json.dumps(execution),'--environment','fake']
            # Only stop at the native process boundary: parsers, spec SHA/path,
            # startup config derivation and input filesystem checks are real.
            with patch('scene_worker.serve',return_value=0) as serve,patch('local_execution.register_scene',return_value=Mock()),patch('signal.signal'):
                with patch('sys.argv',argv):self.assertEqual(main(),0)
                self.assertEqual(serve.call_args.args[0].options.finger_envelope,finger)
                changes=[('--finger-envelope',str(sibling)),('--finger-envelope-sha256','0'*64),
                    ('--assets',str(fixture.root/'config.json')),('--gpu','0'),('--instance','other-instance')]
                for index,(flag,value) in enumerate(changes):
                    bad=list(argv);bad[bad.index(flag)+1]=value;destination=output/f'bad-{index}';bad[bad.index('--output')+1]=str(destination)
                    with self.subTest(flag=flag),patch('sys.argv',bad),self.assertRaises((ValueError,FileNotFoundError)):main()
                    self.assertFalse(destination.exists())
                finger.write_text('{"tampered":true}')
                with patch('sys.argv',argv),self.assertRaisesRegex(ValueError,'exact startup pin'):main()
                finger.write_bytes(sibling.read_bytes())
                self.assertEqual(serve.call_count,1)
            rpc=dict(source=str(finger),suite=str(sibling),physical_binding=str(sibling),output=str(output/'rpc'))
            with self.assertRaisesRegex(ValueError,'outside allowed roots'):
                request_paths(json.dumps(rpc),tuple(Path(p) for p in spec['allowed_read_roots']),output)
            bad_cfg=json.loads(planner.read_text());bad_cfg['finger_envelope']=str(sibling)
            bad_path=planner.parent/'unapproved.json';bad_path.write_text(json.dumps(bad_cfg))
            with self.assertRaises(ValueError):scene_startup(bad_path,fixture.root/'rejected')
        service=SceneService();service.process=NS(poll=lambda:1)
        self.assertIn('场景服务已退出',service.status()['error']);self.assertNotIn('规划',service.status()['error'])

    def test_reuses_loaded_scene_but_rebuilds_current_readback_and_frame(self):
        with TemporaryDirectory() as folder:
            root=Path(folder);envelope=root/'envelope';envelope.write_bytes(b'frozen')
            options=NS(environment='fake',finger_envelope=envelope,finger_envelope_sha256=hashlib.sha256(b'frozen').hexdigest(),
                assets=root/'assets',cup_contact_preset=None)
            worker=Worker(options);worker.identity={'pid':123,'identity':'model'}
            source=dict(run_id='first',calibration_profile={'id':'calibrated-model'})
            suite=dict(episodes=[dict(episode_id='physical-observed',machine_asset='machine-v1',pose=[1,2,3])])
            files={key:root/(key+'.json') for key in ('source','suite','physical_binding')}
            def inputs():
                for name,value in [('source',source),('suite',suite),('physical_binding',{})]:files[name].write_text(json.dumps(value))
            inputs();planner=object();rendered=[]
            def render(args,current,*_):
                rendered.append(current['run_id']);(args.output/'first-frame.png').write_bytes(current['run_id'].encode())
            with patch('playground_sim.validate_inputs'), \
                 patch('playground_sim.setup',return_value=(object(),object(),object(),object(),object(),object(),{},{})) as setup, \
                 patch('playground_sim.reset_scene',return_value=(object(),{})) as reset, \
                 patch('playground_sim.first_frame',side_effect=render), \
                 patch('playground_sim.plan_episode',return_value=planner) as export:
                first=worker.prepare(dict(files,output=root/'first'))
                source['run_id']='second';suite['episodes'][0]['pose']=[4,5,6];inputs()
                second=worker.prepare(dict(files,output=root/'second'))
                self.assertTrue(first['passed'] and second['passed']);self.assertFalse(first['scene_reused']);self.assertTrue(second['scene_reused'])
                self.assertEqual(setup.call_count,1);self.assertEqual(reset.call_count,1)
                cold_id=setup.call_args.args[2]['episodes'][0]['episode_id'];hot_id=reset.call_args.args[2]['episodes'][0]['episode_id']
                self.assertNotEqual(cold_id,hot_id);self.assertNotEqual(cold_id,'physical-observed')
                self.assertEqual(first['source_episode_id'],second['source_episode_id'])
                self.assertEqual(json.loads(files['suite'].read_text()),suite)
                self.assertIs(export.call_args.kwargs['planner'],planner)
                self.assertEqual(rendered,['first','second']);self.assertNotEqual(first['source_digest'],second['source_digest'])
                self.assertEqual((root/'second/first-frame.png').read_bytes(),b'second')
                source['calibration_profile']['id']='changed-model';inputs()
                rejected=worker.prepare(dict(files,output=root/'changed'))
                self.assertFalse(rejected['passed']);self.assertIn('model/calibration changed',rejected['error'])
                self.assertEqual(reset.call_count,1);self.assertEqual(rendered,['first','second'])

    def test_private_socket_scope_and_repeated_requests(self):
        with TemporaryDirectory() as folder:
            root=Path(folder).resolve();reads=root/'reads';reads.mkdir();out=root/'out';out.mkdir()
            for name in ('source','suite','physical_binding'):(reads/(name+'.json')).write_text('{}')
            runtime=Path('/tmp')/('rbw-scene-'+uuid4().hex[:20]);runtime.mkdir(mode=0o700)
            spec=dict(root=str(runtime),socket=str(runtime/'p'),owner_token=uuid4().hex,session_id='scene-test',identity='a'*64,
                allowed_read_roots=[str(reads)],allowed_output_root=str(out),max_memory_mib=12288,min_free_memory_mib=16384,
                max_lifetime_seconds=30,finger_envelope=dict(path=str(reads/'source.json'),sha256=hashlib.sha256(b'{}').hexdigest()))
            payload=json.dumps(spec,sort_keys=True,separators=(',',':')).encode();spec_sha256=hashlib.sha256(payload).hexdigest()
            private=runtime/'private-spec.json';private.write_bytes(payload);private.chmod(0o600)
            program='''import json,sys
from unittest.mock import patch
from scene_worker import serve
class Fake:
 def __init__(self):self.model_key=None;self.calls=0
 def prepare(self,r):
  self.calls+=1;self.model_key='model';r['output'].mkdir()
  return dict(status='geometry-inputs-ready',passed=True,output=str(r['output']),calls=self.calls)
 def close(self):pass
with patch('scene_worker.process_start_ticks',return_value='123'):serve(Fake(),json.loads(sys.argv[1]),'b'*64)
'''
            child=subprocess.Popen([sys.executable,'-B','-c',program,json.dumps(spec)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,
                text=True,env=dict(os.environ,PYTHONPATH=str(Path(__file__).parent)))
            try:
                ready=json.loads(child.stdout.readline());self.assertNotIn('owner_token',ready['service']);self.assertFalse(ready['service']['model_initialized'])
                request={name:str(reads/(name+'.json')) for name in ('source','suite','physical_binding')}
                request['output']=str(out/'one')
                self.assertEqual(request_service(spec['socket'],spec['owner_token'],request,timeout=5)['calls'],1)
                request['output']=str(out/'two')
                self.assertEqual(request_service(spec['socket'],spec['owner_token'],request,timeout=5)['calls'],2)
                self.assertTrue(json.loads((runtime/'status.json').read_text())['model_initialized'])
                request['output']=str(out/'cli')
                cli=subprocess.run([sys.executable,'-B',str(Path(__file__).with_name('scene_worker.py')),
                    '--service-spec-file',str(private),'--spec-sha256',spec_sha256,'--request',json.dumps(request)],
                    capture_output=True,text=True,timeout=5)
                self.assertEqual(cli.returncode,0,cli.stderr);self.assertEqual(json.loads(cli.stdout)['calls'],3)
                request['output']=str(out/'forbidden')
                with self.assertRaises(ValueError):request_service(spec['socket'],'other-owner',request,timeout=5)
                self.assertFalse(Path(request['output']).exists())
                with self.assertRaises(ValueError):request_paths(json.dumps(dict(request,action='execute')),(reads,),out)
                (reads/'escape.json').symlink_to(root/'private.json');(root/'private.json').write_text('{}')
                with self.assertRaises(ValueError):request_paths(json.dumps(dict(request,source=str(reads/'escape.json'))),(reads,),out)
                child.stdin.close();child.stdin=None;_,errors=child.communicate(timeout=5)
                self.assertEqual(child.returncode,0,errors);self.assertFalse((runtime/'p').exists())
            finally:
                if child.poll() is None:child.kill();child.communicate()
                shutil.rmtree(runtime)

    def test_hot_reset_rebinds_full_start_and_preserves_configured_machine_baseline(self):
        import numpy as np
        from contract import demo_state
        from playground_sim import reset_scene
        from test_playground_sim import NativeSpec,action_module
        from integration import _digest
        spec=NativeSpec();state=demo_state();state['left_arm_joints'][0]=.2
        state['right_arm_joints'][0]=.3
        original=NS(machine_asset='machine-v1',metadata={'initial_robot_state':demo_state()})
        suite={'episodes':[dict(episode_id='physical-observed',machine_asset='machine-v1',metadata=dict(initial_robot_state=state,initial_robot_state_digest=_digest(state)))]}
        def joints(value):
            return {**{side+'joint'+str(i):q for side,key,start in [('left_','left_arm_joints',1),('right_','right_arm_joints',11)]
                        for i,q in enumerate(value[key],start)},
                    'left_joint7':value['left_gripper']/2000,'left_joint8':value['left_gripper']/2000,
                    'right_joint17':value['right_gripper']/2000,'right_joint18':value['right_gripper']/2000}
        names=list(joints(state));robot=NS(data=NS(default_joint_pos=np.zeros((1,16)),default_joint_vel=np.ones((1,16))),
            find_joints=lambda requested,**_:([names.index(n) for n in requested],requested))
        runtime=NS(spec=NS(episode=original),gripper_mode='distance',action=spec,evaluator=NS(),physical_identity={},loop=NS(physics_steps=0),_render=Mock())
        runtime.read=lambda _: (NS(arm_positions=list(runtime.home_positions)),None)
        env=NS(scene={'robot':robot},device='cpu',cfg=NS(scene=NS(robot=NS(init_state=NS(joint_pos={})))),reset=Mock(return_value=(None,{'reset':'readback'})))
        modules={'torch':NS(tensor=lambda value,**_:np.asarray(value)),
            'robot_sim.specs.action':action_module(spec),
            'robot_sim.tasks.manager_based.coffee_make.episode_scene':NS(EpisodeScene=NS(from_dict=lambda value:NS(**value))),
            'robot_sim.tasks.manager_based.coffee_make.episode_randomization':NS(scene_build_key=lambda episode:episode.machine_asset),
            'robot_sim.tasks.manager_based.coffee_make.initial_robot_state':NS(initial_robot_state=lambda episode,opening:deepcopy(episode.metadata['initial_robot_state']),
                joint_positions=joints,arm_positions=lambda value:tuple(value['left_arm_joints']+value['right_arm_joints']),gripper_mode=lambda episode:'distance')}
        with TemporaryDirectory() as folder,patch.dict(sys.modules,modules),patch('playground_sim.source_environment',return_value='fake'):
            args=NS(output=Path(folder));source=dict(initial_state={'model':state})
            internal,args.scene_reset_identity=reset_suite(source,suite,{},args.output)
            episode,_=reset_scene(args,source,internal,env,runtime,NS(open_position_m=.0415))
            self.assertIs(runtime.spec.episode,original)
            self.assertEqual(runtime.initial_robot_state,state);self.assertEqual(runtime.physical_identity['initial_robot_state_digest'],_digest(state))
            np.testing.assert_allclose(robot.data.default_joint_pos[0],[joints(state)[name] for name in names])
            self.assertFalse(robot.data.default_joint_vel.any());self.assertEqual(env.reset.call_args.kwargs,{'options':{'episode':episode}})
            normalized=json.loads((args.output/'initial-normalization.json').read_text())
            self.assertEqual({key:normalized[key] for key in args.scene_reset_identity},args.scene_reset_identity)
            self.assertEqual(episode.episode_id,args.scene_reset_identity['reset_episode_id'])
            runtime.read=lambda _: (NS(arm_positions=[q+.006 for q in runtime.home_positions]),None)
            args.output=Path(folder)/'drift';args.output.mkdir()
            with self.assertRaisesRegex(ValueError,'reset differs'):reset_scene(args,source,internal,env,runtime,NS(open_position_m=.0415))

    def test_pipeline_warmup_never_invokes_policy_plan(self):
        import playground_pipeline as pipeline
        with TemporaryDirectory() as folder:
            out=Path(folder);cfg={'remote_run_root':'/owned/current'}
            result=dict(status='motiongen-warm-ready',model_initialized=True,model_key='model',policy_planned=False,motion_requested=False)
            with patch.object(pipeline,'prepare_scene',return_value=(cfg,{},dict(pipeline_config='config'),[],'fake')) as fresh, \
                 patch.object(pipeline,'geometry_stage',return_value={'name':'warmup'}) as stage, \
                 patch.object(pipeline,'run_stages'),patch.object(pipeline,'plan',side_effect=AssertionError('policy planning called')), \
                 patch.object(pipeline,'fetch',side_effect=lambda remote,path,**kw:Path(path).write_text(json.dumps(result))):
                self.assertTrue(pipeline.warmup({},out,object())['model_initialized'])
                fresh.assert_called_once();self.assertEqual(stage.call_args.kwargs,{'operation':'warmup'})


if __name__=='__main__':unittest.main()
