"""Native training sidecars resolve releases without loading checkpoint payloads."""
import hashlib
import json
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from robot_workbench.model_install import resolve_training_release,write_json


class ProducerTests(unittest.TestCase):
    def setUp(self):
        temporary=TemporaryDirectory();self.addCleanup(temporary.cleanup)
        self.root=Path(temporary.name);self.experiment=self.root/'experiment';self.experiment.mkdir()

    def file(self,path,data=b'synthetic'):
        path=self.root/path;path.parent.mkdir(parents=True,exist_ok=True)
        path.write_bytes(data);return path

    def sha(self,path):return hashlib.sha256(path.read_bytes()).hexdigest()

    def test_wam_uses_current_contract_and_committed_checkpoint(self):
        run=self.experiment;checkpoint=run/'outputs/formal/checkpoints/step_000042'
        weight=self.file('experiment/outputs/formal/checkpoints/step_000042/weights.pt')
        source=self.root/'frozen-source';module=self.file('frozen-source/src/wam/inference/frame_joint.py',b'# frozen\n')
        empty=self.file('frozen-source/src/wam/__init__.py',b'')
        stats=self.file('stats.json',b'{"current":"normalization"}')
        model=self.file('model.yaml',b'model: {}\n');context=self.file('context.pt')
        index=self.root/'text-index.json';write_json(index,dict(prompts={'prompt':dict(path=str(context),sha256=self.sha(context))}))
        pins={k:self.sha(p)for k,p in [('normalization',stats),('model_definition',model),('text_index',index)]}
        native=dict(schema='phoenix.training_run.v1',assets=pins)
        write_json(run/'contract.json',dict(runtime=dict(source=str(source)),normalization=str(stats),
            model_definition=str(model),text_index=str(index),text_pack=str(context)))
        write_json(run/'frozen-inputs.json',{str(module):self.sha(module),str(empty):self.sha(empty)})
        write_json(checkpoint/'trainer_state.json',dict(step=42,epoch=2,contract=native))
        marker=dict(schema='phoenix.training_state.v1',step=42,contract=native,
            files={'weights.pt':dict(bytes=weight.stat().st_size,sha256=self.sha(weight))})
        write_json(checkpoint/'COMPLETE.json',marker)
        write_json(run/'outputs/formal/latest.json',dict(step=42,path=str(checkpoint)))
        release,sources=resolve_training_release('wam',run,'latest',self.root,self.root/'release',runtime_id='runtime-v1')
        self.assertEqual(release['step'],42);self.assertEqual(release['producer']['epoch'],2)
        self.assertEqual(sources['normalization.json'],stats)
        self.assertEqual(release['backends']['torch']['artifacts']['context'],'context.pt')
        self.assertEqual(next(r['bytes']for r in release['files']if r['path'].endswith('__init__.py')),0)
        marker['step']=43;write_json(checkpoint/'COMPLETE.json',marker)
        with self.assertRaisesRegex(ValueError,'trainer state'):
            resolve_training_release('wam',run,checkpoint,self.root,self.root/'bad',runtime_id='runtime-v1')

    def test_memory_uses_native_source_map_exact_prompt_and_codec_paths(self):
        run=self.experiment/'train';checkpoint=run/'step_00000042'
        weight=self.file('experiment/train/step_00000042/weights.pt')
        source=self.file('experiment/source/src/world_memory/inference.py',b'# actual package\n')
        assets={role:self.file('shared/'+name,role.encode())for role,name in (
            ('vae','vae.pt'),('phoenix_encoder','encoder.pt'),('csvae_checkpoint','csvae.pt'))}
        cache=self.root/'shared/text';feature=self.file('shared/text/feature.pt',b'text features')
        write_json(cache/'complete.json',dict(identity={'encoder':'fixture'},records={'pick cup':dict(file='feature.pt',sha256=self.sha(feature))}))
        dependencies={role:[dict(path=str(path),size=path.stat().st_size,sha256=self.sha(path))]for role,path in assets.items()}
        dependencies.update(source={'inference.py':self.sha(source)},text_cache=dict(path=str(cache/'complete.json'),
            size=(cache/'complete.json').stat().st_size,sha256=self.sha(cache/'complete.json')))
        data=dict(state_dim=14,action_dim=14,action_horizon=32,control_frequency=30,relative_action_dims=0)
        config=dict(codec=dict(checkpoint=str(assets['vae']),phoenix_encoder=str(assets['phoenix_encoder']),
            csvae_checkpoint=str(assets['csvae_checkpoint'])),data=dict(data,cache_dir=str(cache)))
        write_json(run/'config.json',config);write_json(run/'run.json',dict(dependencies=dependencies))
        write_json(checkpoint/'complete.json',dict(schema=1,step=42,contract={'data':data},dependencies=dependencies,
            files={'weights.pt':dict(size=weight.stat().st_size,sha256=self.sha(weight))}))
        release,sources=resolve_training_release('wam-mem',self.experiment,'latest',self.root,self.root/'release',runtime_id='memory-v4')
        backend=release['backends']['torch']
        self.assertEqual(backend['instruction'],'pick cup');self.assertEqual(backend['cached_text_identity'],{'encoder':'fixture'})
        self.assertEqual(json.loads(sources['source-manifest.json'].read_text()),dependencies['source'])
        self.assertNotIn('stats',backend['artifacts'])
        (checkpoint/'complete.json').unlink()
        write_json(checkpoint/'export.json', dict(format='world_memory_policy', weights_only=True,
            step=42, size=weight.stat().st_size, sha256=self.sha(weight)))
        exported, _ = resolve_training_release('wam-mem', self.experiment, 'latest', self.root,
            self.root/'weights-only-release', runtime_id='memory-v4')
        self.assertEqual(exported['model_id'], release['model_id'])
        self.assertEqual(exported['producer']['completion_kind'], 'export.json')
        self.assertEqual(exported['backends'], release['backends'])
        source.write_text('# altered\n')
        with self.assertRaisesRegex(ValueError,'content differs'):
            resolve_training_release('wam-mem',self.experiment,checkpoint,self.root,self.root/'bad',runtime_id='memory-v4')


if __name__=='__main__':unittest.main()
