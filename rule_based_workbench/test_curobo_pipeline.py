"""Fake-only backend wiring checks; no SSH, GPU, simulation, or robot services."""
from copy import deepcopy
import json
from pathlib import Path
import shlex
import sys
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest.mock import Mock, patch

import playground_geometry as geometry
import playground_pipeline as pipeline
import playground_plan
import frozen_executor
from integration import PIPELINE_LANES
from run_pipeline import PATH_KEYS, load_config


def main():
    with TemporaryDirectory() as directory:
        root=Path(directory)
        remote='/mnt/data/feiyu.zhou/runs/robot_sim/curobo-fake'
        contract=dict(schema_version='anyverse.dsw-codex-experiment.v1',
            filesystem=dict(readonly_roots=['/inputs'],writable_roots=[remote],allow_overwrite=False,allow_delete=False),
            resources=dict(lanes={v['instance_id']:[v['gpu']] for v in PIPELINE_LANES.values()},
                max_gpu_count_per_instance=1,max_cpu_threads=4,require_gpu_idle_check=True,max_wall_time_minutes=20))
        contract_path=root/'contract.json';contract_path.write_text(json.dumps(contract))
        config={key:'/inputs/'+key for key in PATH_KEYS}
        config.update(contract_path=str(contract_path),remote_run_root=remote,
                      finger_envelope_sha256='a'*64,opening_mm=83.)
        config_path=root/'config.json'
        def configured(change):
            config_path.write_text(json.dumps({**config,**change}))
            return load_config(config_path)[0]
        native=configured({})
        assert 'planning_backend' not in native, 'native config compatibility changed'
        gpu=configured(dict(planning_backend='curobo',curobo_root='/inputs/curobo'))
        for change in (dict(planning_backend='other'),dict(planning_backend='curobo'),
                       dict(planning_backend='curobo',curobo_root='relative'),
                       dict(planning_backend='curobo',curobo_root='/inputs-escape/curobo'),
                       dict(planning_backend='curobo',curobo_root='/inputs/../escape'),
                       dict(planning_backend='curobo',curobo_root=remote+'/curobo')):
            try:configured(change)
            except ValueError:pass
            else:raise AssertionError('invalid backend/root accepted: '+repr(change))
        with patch.object(pipeline,'gpu_check_command',return_value='GPU_IDLE_CHECK') as idle:
            normal=pipeline.geometry_stage(native,'/out/native')
            accelerated=pipeline.geometry_stage(gpu,'/out/gpu')
            cached=pipeline.geometry_stage(gpu,'/out/cached',initial='/checked/initial-plan.json')
            assert idle.call_count==1
        def env(stage):
            shell=shlex.split(stage['argv'][-1])[-1]
            argv=shlex.split(shell.split(' && ')[-1])
            from launch_environment import HOST_PYTHON
            values={p.split('=',1)[0]:p.split('=',1)[1] for p in argv[:argv.index(HOST_PYTHON)] if '=' in p}
            return shell,argv,values
        shell,argv,values=env(accelerated)
        assert values['CUDA_VISIBLE_DEVICES']=='0' and shell.startswith('GPU_IDLE_CHECK && ')
        assert values['PYTHONPATH'].startswith('/inputs/curobo/python:/inputs/curobo/deps:')
        assert values['LD_LIBRARY_PATH']=='/inputs/curobo/cuda-overlay/lib64'
        assert values['TORCH_EXTENSIONS_DIR'].startswith(remote+'/cache/')
        assert values['WARP_CACHE_PATH'].startswith(remote+'/cache/')
        assert argv[argv.index('--backend')+1]=='curobo'
        assert accelerated['label']=='GPU 碰撞检查与规则轨迹规划'
        for stage in (normal,cached):
            shell,argv,values=env(stage)
            assert values['CUDA_VISIBLE_DEVICES']=='' and 'GPU_IDLE_CHECK' not in shell
            assert '/inputs/curobo' not in values['PYTHONPATH'] and 'CUDA_HOME' not in values
        assert normal['label']=='CPU 轨迹规划与碰撞检查'
        assert cached['label']=='复用已验证轨迹并调整速度'

        with patch.object(pipeline,'load_config',return_value=(gpu,contract,{})), \
             patch('cancellable_process.run',return_value=SimpleNamespace(returncode=0)) as calls:
            checks=pipeline.prepare_environment(dict(pipeline_config=str(config_path)))
            assert len(checks)==2 and '尚未常驻预热' in checks[1]['message']
            assert 'cuRobo 依赖可读' in checks[1]['message']
            assert '/inputs/curobo/python/curobo/__init__.py' in calls.call_args.args[0][-1]
            assert '/inputs/curobo' not in calls.call_args_list[0].args[0][-1]
        events=[]
        with patch.object(pipeline,'bounded_command',return_value=(['fake'],1)), \
             patch('cancellable_process.run',return_value=SimpleNamespace(returncode=0)), \
             patch.object(pipeline,'fetch',side_effect=lambda remote,local,**kw:
                   Path(local).write_text(json.dumps(dict(status='geometric-plan-ready')))):
            pipeline.run_stages([dict(name='detect'),accelerated],root/'progress',contract,
                                lambda label,**fields:events.append(dict(label=label,**fields)))
        assert events[0]['next_label']=='GPU 碰撞检查与规则轨迹规划'
        assert events[-1]['stages'][-1]['label']=='GPU 碰撞检查与规则轨迹规划'

        source=root/'source.json';source.write_text('{}')
        input_path=root/'planning-input.json';input_path.write_text('{}')
        base=dict(timeline_digest='fixed-ticks',simulation={'validated':False,'outcome':'not_run'},
                  planner_id='existing-planner',native_identity={'identity':'unchanged'},
                  planning_validation={'schema_version':'geometric_path_validation.v1','passed':True})
        fake_planner=SimpleNamespace(geo=SimpleNamespace(whole=SimpleNamespace(world=[('/machine',None,None,None)])),
                                     initial=Mock(return_value=dict(passed=True)))
        data={'native_identity':{'acceptance':{'edge_margin_m':.004}}}
        engine=Mock(return_value=SimpleNamespace(object_ids=('/machine',)))
        wrapper=Mock(return_value='wrapped-native-exact')
        install=Mock()
        with patch.object(geometry,'load',return_value=(fake_planner,'observation',data)) as load, \
             patch.object(playground_plan,'build',return_value=deepcopy(base)), \
             patch.object(playground_plan,'retime',side_effect=lambda base,source,speed:deepcopy(base)), \
             patch.object(frozen_executor,'validate_plan_evidence'), \
             patch.dict(sys.modules,dict(curobo=SimpleNamespace(__version__='0.7.8'),
                 curobo_collision=SimpleNamespace(CuroboWorldBroadphase=engine,
                     __file__=str(Path(__file__).with_name('curobo_collision.py'))),
                 curobo_whole_geometry=SimpleNamespace(CuroboWholeGeometry=wrapper,
                     __file__=str(Path(__file__).with_name('curobo_whole_geometry.py'))),
                 curobo_relative_geometry=SimpleNamespace(install=install, initial=lambda p,o,e:p.initial(o,e),
                     __file__=str(Path(__file__).with_name('curobo_relative_geometry.py'))))):
            args=SimpleNamespace(input=input_path,source=source,native_code=root,assets=root,initial=None,
                                 speed_scale=.3,backend='curobo',output=root/'fresh-plan')
            geometry.main(args)
            assert engine.call_count==wrapper.call_count==install.call_count==load.call_count==1
            assert fake_planner.geo.whole=='wrapped-native-exact'
            plan=json.loads((args.output/'plan.json').read_text())
            identity=plan['planning_backend']
            assert identity['backend']=='curobo_world_broadphase_native_exact' and identity['gpu_ik'] is False
            assert identity['curobo_version']=='0.7.8' and len(identity['code_sha256'])==3
            assert plan['planning_validation']['backend']==identity
            assert plan['planner_id']==base['planner_id'] and plan['native_identity']==base['native_identity']
            args.initial=args.output/'initial-plan.json';args.output=root/'retimed'
            # Any accidental geometry or GPU initialization during speed-only reuse fails this check.
            load.side_effect=AssertionError('cached retime rebuilt native geometry')
            engine.side_effect=AssertionError('cached retime initialized GPU')
            geometry.main(args)
            assert engine.call_count==1 and load.call_count==1
            receipt=json.loads((args.output/'result.json').read_text())
            assert receipt['cached_retime'] and receipt['backend']=='curobo_world_broadphase_native_exact'
            assert receipt['gpu_ik'] is False and receipt['timings_seconds']['path_build_and_audit']==0.
            assert receipt['gpu_used'] is False and receipt['operation']=='cached_retime'
            assert json.loads((args.output/'plan.json').read_text())['planning_backend']==identity
            # Current GPU configuration cannot relabel a historical native audit.
            native_folder=root/'native-base';native_folder.mkdir()
            native_base=deepcopy(base)
            native_identity=dict(backend='native',gpu_ik=False)
            native_base['planning_backend']=native_identity
            native_base['planning_validation']['backend']=native_identity
            (native_folder/'base-plan.json').write_text(json.dumps(native_base))
            args.initial=native_folder/'initial-plan.json';args.output=root/'native-cached'
            geometry.main(args)
            historical=json.loads((args.output/'plan.json').read_text())
            receipt=json.loads((args.output/'result.json').read_text())
            assert historical['planning_backend']==historical['planning_validation']['backend']==native_identity
            assert receipt['backend']=='native' and receipt['requested_backend']=='curobo' and receipt['gpu_used'] is False
            args.initial=root/'missing'/'initial-plan.json';args.output=root/'missing-cache'
            try:geometry.main(args)
            except ValueError as error:assert 'cached base-plan.json required' in str(error)
            else:raise AssertionError('missing cuRobo cache silently replanned on CPU/GPU')
            assert load.call_count==engine.call_count==1
    print('optional cuRobo pipeline fake checks passed; no GPU, SSH or hardware executed')


if __name__=='__main__':
    main()
