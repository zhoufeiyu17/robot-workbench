"""CPU-only catalog checks use tiny synthetic bytes, never model weights."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from robot_workbench import inference_catalog as catalog
from robot_workbench import inference_model as model


def data(value):
    return json.dumps(value,sort_keys=True,ensure_ascii=False,separators=(',',':')).encode()


def sha(value):return hashlib.sha256(value).hexdigest()


class InferenceCatalogTests(unittest.TestCase):
    def setUp(self):
        temporary=TemporaryDirectory();self.addCleanup(temporary.cleanup)
        self.root=Path(temporary.name).resolve();self.checkpoints=self.root/'checkpoints';self.checkpoints.mkdir()
        self.assets=self.root/'approved-family';self.assets.mkdir()
        self.source=self.assets/'source';self.source.mkdir();(self.source/'runtime.py').write_bytes(b'# inert fixture\n')
        self.identity={'fixture':'approved family, CPU only'}
        self.family_id=sha(data(self.identity));self.stats={'schema':'fixture-stats','state':{'q01':[0.]*14,'q99':[1.]*14}}
        values={'stats':data(self.stats),'tokenizer':b'no real tokenizer',
            'source_manifest':data({'runtime.py':{'sha256':sha((self.source/'runtime.py').read_bytes()),'size_bytes':16}}),
            'io_contract':data({'fixture':'approved IO'}),'process':b'# no process launched\n','worker':b'# no worker launched\n'}
        pins={name:sha(content) for name,content in values.items()}
        patches=[patch.dict(model.BASE566_PINS,pins),patch.object(model,'BASE566_FAMILY_ID',self.family_id)]
        for item in patches:item.start();self.addCleanup(item.stop)
        artifacts={}
        for name,content in values.items():
            path=self.assets/name;path.write_bytes(content);artifacts[name]={'path':str(path),'sha256':sha(content)}
        self.weights=b'tiny synthetic state dictionary bytes; not a model'
        artifacts.update(checkpoint={'path':str(self.root/'not-yet-downloaded/model.pt'),
            'sha256':model.BASE566_PINS['checkpoint'],'size_bytes':len(self.weights)},
            metadata={'path':str(self.root/'not-yet-downloaded/metadata.json'),'sha256':model.BASE566_PINS['metadata']})
        self.template=dict(schema='workbench.base566.v1',model_id=model.BASE566_ID,epoch=7,step=58982,
            prompt=model.BASE566_PROMPT,action_horizon=60,execution_steps=10,action_hz=30,rtc=False,
            python=str(Path(sys.executable).resolve()),model_source=str(self.source),artifacts=artifacts)

    def checkpoint(self,epoch=5,step=42130,name=None,changes=None):
        directory=self.checkpoints/(name or f'epoch_{epoch:03d}');directory.mkdir()
        metadata=dict(schema='pi05.base566_checkpoint.v1',mode='train',epoch=epoch,step=step,world_size=16,
            identity=self.identity,identity_sha256=self.family_id,model_config=model.BASE566_MODEL_CONFIG,
            stats=self.stats,fixed_input_sha256='a'*64)
        metadata.update(changes or {})
        (directory/'model.pt').write_bytes(self.weights)
        self.seal(directory,metadata)
        return directory

    def seal(self,directory,metadata):
        encoded=data(metadata);(directory/'metadata.json').write_bytes(encoded)
        complete=dict(complete=True,mode='train',epoch=metadata['epoch'],step=metadata['step'],world_size=16,
            identity_sha256=self.family_id,fixed_input_sha256=metadata['fixed_input_sha256'],
            model_sha256=sha(self.weights),metadata_sha256=sha(encoded),
            files={'model.pt':sha(self.weights),'metadata.json':sha(encoded),
                   'training-state-rank-00.pt':'b'*64,'train-fit/rank-00.npz':'c'*64})
        (directory/'complete.json').write_bytes(data(complete))

    def entries(self):return catalog.scan_catalog(self.checkpoints,self.template)

    def test_epochs_and_future_epoch_derive_only_trusted_paths_without_opening_weights(self):
        first=self.checkpoint(changes={'python':'/untrusted/run.py','model_source':'/untrusted/source'})
        self.checkpoint(7,58982);self.checkpoint(12,101112)
        original=Path.open
        def guarded(path,*args,**kwargs):
            if path.name=='model.pt':raise AssertionError('listing read model weights')
            return original(path,*args,**kwargs)
        with patch.object(Path,'open',guarded):
            rows=self.entries()
            manifests=catalog.catalog_manifests(self.checkpoints,self.template)
        self.assertEqual([row['epoch'] for row in rows],[12,7,5])
        self.assertTrue(all(row['selectable'] and not row['weight_content_verified'] for row in rows))
        bundle=manifests['base566-epoch005-step42130']
        self.assertEqual(bundle['artifacts']['checkpoint']['path'],str(first/'model.pt'))
        self.assertEqual(bundle['model_source'],str(self.source))
        self.assertEqual(bundle['artifacts']['stats'],self.template['artifacts']['stats'])
        self.assertEqual(catalog.resolve_checkpoint(self.checkpoints,self.template,bundle['model_id']),bundle)
        self.assertFalse((first/'training-state-rank-00.pt').exists(),'inference does not require optimizer shards')

    def test_download_missing_pairing_and_size_states_are_not_selectable(self):
        partial=self.checkpoint(1,100);(partial/'model.pt.partial').write_bytes(b'ongoing')
        absent=self.checkpoint(2,200);(absent/'complete.json').unlink()
        missing=self.checkpoint(3,300);(missing/'metadata.json').unlink()
        short=self.checkpoint(4,400);(short/'model.pt').write_bytes(b'short')
        unfinished=self.checkpoint(6,600)
        complete=json.loads((unfinished/'complete.json').read_bytes());complete['complete']=False
        (unfinished/'complete.json').write_bytes(data(complete))
        self.assertEqual({row['directory_name']:row['status'] for row in self.entries()},
            {'epoch_001':'downloading','epoch_002':'downloading','epoch_003':'missing_assets',
             'epoch_004':'downloading','epoch_006':'downloading'})
        self.assertEqual(catalog.catalog_manifests(self.checkpoints,self.template),{})

    def test_family_model_stats_and_complete_hash_mismatch_are_rejected(self):
        bad_identity=deepcopy(self.identity);bad_identity['different']=True
        self.checkpoint(1,100,changes={'identity':bad_identity})
        config=deepcopy(model.BASE566_MODEL_CONFIG);config['rtc_max_delay']=8
        self.checkpoint(2,200,changes={'model_config':config})
        self.checkpoint(3,300,changes={'stats':{'wrong':'normalization'}})
        altered=self.checkpoint(4,400);(altered/'metadata.json').write_bytes(b'{}')
        duplicate=self.checkpoint(5,500)
        complete=json.loads((duplicate/'complete.json').read_bytes());complete['files']['model.pt']='f'*64
        (duplicate/'complete.json').write_bytes(data(complete))
        self.assertTrue(all(row['status']=='incompatible' and not row['selectable'] for row in self.entries()))

    def test_symlink_escape_and_duplicate_identity_are_rejected(self):
        outside=self.root/'outside';outside.mkdir();(self.checkpoints/'escape').symlink_to(outside,target_is_directory=True)
        linked=self.checkpoint(1,100);(linked/'model.pt').unlink();(linked/'model.pt').symlink_to(self.assets/'worker')
        escaped=self.checkpoint(2,200)
        complete=json.loads((escaped/'complete.json').read_bytes());complete['files']['../../outside.py']='d'*64
        (escaped/'complete.json').write_bytes(data(complete))
        self.checkpoint(5,42130,'copy_a');self.checkpoint(5,42130,'copy_b')
        self.assertTrue(all(row['status']=='incompatible' and not row['selectable'] for row in self.entries()))
        with self.assertRaisesRegex(ValueError,'同一模型身份'):
            catalog.resolve_checkpoint(self.checkpoints,self.template,'base566-epoch005-step42130')

    def test_missing_family_assets_and_full_load_hash_gate(self):
        directory=self.checkpoint()
        bundle=catalog.resolve_checkpoint(self.checkpoints,self.template,'base566-epoch005-step42130')
        model.validate_bundle(bundle,verify_files=True)
        (directory/'model.pt').write_bytes(b'x'*len(self.weights))
        self.assertTrue(self.entries()[0]['selectable'],'listing does not claim full weight hash verification')
        with self.assertRaisesRegex(ValueError,'artifact content differs'):
            model.validate_bundle(bundle,verify_files=True)
        (self.assets/'stats').unlink()
        self.assertEqual(self.entries()[0]['status'],'missing_assets')


if __name__=='__main__':unittest.main()
