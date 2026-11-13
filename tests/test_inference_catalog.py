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


class RegisteredCatalogTests(unittest.TestCase):
    def setUp(self):
        temporary = TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name).resolve()
        self.catalog_root = self.root / 'catalog'
        self.catalog_root.mkdir()

    def register(self, identity='wam-fixture-step42', backend='torch'):
        assets = self.root / (identity + '-' + backend)
        assets.mkdir(exist_ok=True)
        files = []
        pins = {}
        for kind in ('checkpoint', 'stats', 'context', 'model_config', 'engine'):
            content = ((ROOT / 'tests/fixtures/wam-e03-normalization.json').read_bytes()
                       if kind == 'stats' else (kind + identity + backend).encode())
            path = assets / kind
            path.write_bytes(content)
            pins[kind] = sha(content)
            files.append(dict(path=kind, bytes=len(content), sha256=pins[kind]))
        registry = dict(model_id=identity, step=int(identity.rsplit('step', 1)[1]), backend=backend,
            root=str(assets), files=files, bundle_sha256=sha(data(sorted(files, key=lambda row: row['path']))),
            engine_sha256=pins.pop('engine'), artifacts=pins,
            inference_capabilities={'denoise_steps': {'minimum': 1, 'maximum': 3, 'default': 2}})
        manifest = self.catalog_root / identity / backend / 'manifest.json'
        manifest.parent.mkdir(parents=True, exist_ok=True)
        manifest.write_text(json.dumps(registry))
        manifest.chmod(0o600)
        return manifest, registry

    def test_directory_discovery_refreshes_and_preserves_loaded_identity_over_http(self):
        import threading
        from robot_workbench import wam
        from robot_workbench.inference_transport import RemoteBaseClient, serve_model, PROTOCOL
        manifest, registry = self.register()
        from robot_workbench import model_runtime
        lock_name=model_runtime.lock_path('torch')
        lock_file=Path(registry['root'])/lock_name
        lock_file.parent.mkdir()
        lock_bytes=data(model_runtime.capture(sys.executable))
        lock_file.write_bytes(lock_bytes)
        registry['python']=sys.executable
        registry['files'].append(dict(path=lock_name,bytes=len(lock_bytes),sha256=sha(lock_bytes)))
        registry['bundle_sha256']=sha(data(sorted(registry['files'],key=lambda row:row['path'])))
        manifest.write_text(json.dumps(registry))
        closed = []

        class Worker:
            def __init__(self, registry, bundle):
                self.registry = deepcopy(registry)
                self.bundle = deepcopy(bundle)
                self.model_id = bundle['model_id']
            def prepare(self):
                return wam.worker_identity(self.bundle)
            def infer(self, request):
                import numpy as np
                return dict(action14=np.zeros((32, 14), np.float32))
            def close(self):
                closed.append(self.bundle)

        with patch('robot_workbench.wam.Client', Worker):
            service = wam.model_service(dict(registry, catalog_root=str(self.catalog_root)))
            token = self.root / 'token'
            token.write_text('cpu-test-' + 'x'*40)
            server = serve_model(service, token_file=token)
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            self.addCleanup(service.close)
            self.addCleanup(server.server_close)
            self.addCleanup(server.shutdown)
            bundle = deepcopy(service.model_bundle)
            client = RemoteBaseClient('http://127.0.0.1:' + str(server.server_port),
                token_file=token, model_id=bundle['model_id'], model_bundle=bundle,
                adapter=None, conversion_identity={'cpu': 'directory-catalog'})
            initial_root = self.root / 'moved-before-first-prepare'
            Path(registry['root']).rename(initial_root)
            registry['root'] = str(initial_root)
            manifest.write_text(json.dumps(registry))
            client.prepare()
            self.assertEqual(service.client.registry['root'], str(initial_root))
            closed.clear()
            original = service.client
            self.assertEqual(len(client.models()), 3)
            added, _ = self.register('wam-new-training-step77', 'anyinfer')
            rows = client.models()
            self.assertEqual(len(rows), 6)
            self.assertIn('wam-new-training-step77', {r['model_id'] for r in rows})
            self.assertIs(service.client, original)
            added.unlink()
            rows = client.models()
            self.assertEqual(sum(r['selectable'] for r in rows), 3)
            self.assertTrue(any('manifest' in r.get('reason', '') for r in rows))
            replacement = deepcopy(registry)
            replacement['inference_capabilities']['denoise_steps']['maximum'] = 2
            manifest.write_text(json.dumps(replacement))
            self.assertEqual(sum(r['selectable'] for r in client.models()), 2)
            with self.assertRaisesRegex((ValueError, RuntimeError), 'absent|changed'):
                client.prepare()
            self.assertIs(service.client, original)
            self.assertEqual(service.model_bundle, bundle)
            self.assertEqual(closed, [])
            relocated = self.root / 'relocated-assets'
            Path(registry['root']).rename(relocated)
            registry['root'] = str(relocated)
            manifest.write_text(json.dumps(registry))
            client.prepare()
            self.assertIs(service.client, original)
            service.unload(dict(protocol=PROTOCOL, model_id=client.model_id,
                conversion_id=client.conversion_id, expected_bundle_digest=client.bundle_digest,
                backend=bundle['backend']))
            client.prepare()
            self.assertEqual(service.client.registry['root'], str(relocated))
            self.assertIsNot(service.client, original)

    def test_scans_only_metadata_and_exposes_invalid_assets_and_duplicates(self):
        manifest, registry = self.register()
        directory = catalog.RegisteredCatalog(self.catalog_root)
        original_open = Path.open
        def metadata_only(path, *args, **kwargs):
            self.assertIn(path.name, ('manifest.json', 'stats'))
            return original_open(path, *args, **kwargs)
        with patch.object(Path, 'open', metadata_only):
            self.assertEqual(len(directory.catalog()), 3)
        (Path(registry['root']) / 'checkpoint').unlink()
        rows = directory.rows()
        self.assertFalse(any(r['selectable'] for r in rows))
        self.assertIn('checkpoint', rows[0]['reason'])
        self.register()
        duplicate = self.catalog_root / 'duplicate' / 'torch' / 'manifest.json'
        duplicate.parent.mkdir(parents=True)
        duplicate.write_bytes(manifest.read_bytes())
        duplicate.chmod(0o600)
        self.assertTrue(all(not r['selectable'] for r in directory.rows()))
        self.assertTrue(all('duplicate' in r['reason'] for r in directory.rows()))
        duplicate.write_text('{')
        self.assertEqual(sum(r['selectable'] for r in directory.rows()), 3)
        registry['files'][0]['path'] = '../outside'
        manifest.write_text(json.dumps(registry))
        self.assertFalse(any(r['selectable'] for r in directory.rows()))

    def test_startup_reads_assets_and_configuration_only_from_the_catalog_manifest(self):
        from robot_workbench import wam
        manifest, registry = self.register()
        moved = self.root / 'relocated-before-service-start'
        Path(registry['root']).rename(moved)
        registry['root'] = str(moved)
        manifest.write_text(json.dumps(registry))
        selection = dict(model_id=registry['model_id'], backend='torch',
                         catalog_root=str(self.catalog_root), selected_inference_options={'denoise_steps': 3})
        service = wam.model_service(selection)
        self.addCleanup(service.close)
        self.assertEqual(service.client.registry['root'], str(moved))
        self.assertEqual(service.model_bundle['inference_options']['denoise_steps'], 3)
        self.assertIsNone(service.client.process)
        with self.assertRaisesRegex(ValueError, 'initial model'):
            wam.model_service(dict(selection, model_id='wam-missing-step1'))

    def test_pi_and_wam_share_the_directory_and_use_their_existing_loaders(self):
        from robot_workbench import mixed366, wam
        self.register()
        assets = self.root / 'pi'
        assets.mkdir()
        content = b'CPU artifact fixture'
        (assets / 'artifact').write_bytes(content)
        registry = dict(model_id=model.MIXED366_ID, root=str(assets), backend='torch',
            files=[dict(path='artifact', bytes=len(content), sha256=sha(content))],
            bundle_sha256='a'*64, source_manifest_sha256='b'*64, runtime_sha256='c'*64)
        manifest = self.catalog_root / registry['model_id'] / 'torch' / 'manifest.json'
        manifest.parent.mkdir(parents=True)
        manifest.write_text(json.dumps(registry));manifest.chmod(0o600)
        service = mixed366.Service(dict(registry, catalog_root=str(self.catalog_root)))
        self.addCleanup(service.close)
        rows = service.models()
        self.assertEqual({row['model_id'] for row in rows}, {model.MIXED366_ID, 'wam-fixture-step42'})
        self.assertIsInstance(service.client, mixed366.Client)
        bundle = next(row['model_descriptor'] for row in rows if row['model_id'].startswith('wam-'))
        self.assertIsInstance(service.client_factory(bundle), wam.Client)
        packed = assets / 'triton/packed.pt'
        packed.parent.mkdir();packed.write_bytes(b'CPU packed fixture')
        (packed.parent / 'deployment.json').write_text(json.dumps(dict(status='READY',
            packed='triton/packed.pt', packed_sha256=sha(packed.read_bytes()), bundle_sha256='a'*64)))
        triton = self.catalog_root / registry['model_id'] / 'triton' / 'manifest.json'
        triton.parent.mkdir();triton.write_text(json.dumps(dict(registry, backend='triton')));triton.chmod(0o600)
        rows = service.models()
        bundle = next(row['model_descriptor'] for row in rows if row['backend'] == 'triton')
        self.assertEqual(service.client_factory(bundle).packed, str(packed))
        self.assertTrue(any(row['id'].startswith('triton-') for row in service.tool_catalog()['models']))

    def test_unsafe_and_unfinished_directories_are_visible_and_unselectable(self):
        manifest, registry = self.register()
        directory = catalog.RegisteredCatalog(self.catalog_root)
        manifest.chmod(0o644)
        self.assertFalse(directory.rows()[0]['selectable'])
        manifest.chmod(0o600)
        (self.catalog_root / 'empty').mkdir()
        (self.catalog_root / 'linked').symlink_to(manifest.parent.parent)
        rows = directory.rows()
        self.assertEqual(sum(row['selectable'] for row in rows), 3)
        self.assertEqual(sum(not row['selectable'] for row in rows), 2)
        registry['backend'] = 'triton'
        manifest.write_text(json.dumps(registry))
        self.assertFalse(any(row['selectable'] for row in directory.rows()))

    def test_same_size_asset_corruption_is_rejected_by_load_time_content_verification(self):
        from robot_workbench import mixed366, wam
        manifest, registry = self.register()
        assets = Path(registry['root'])
        for name in ('phoenix_encoder.pt', 'csvae.pt', 'Wan2.2_VAE.safetensors'):
            content = ('CPU fixture ' + name).encode()
            (assets / name).write_bytes(content)
            registry['files'].append(dict(path=name, bytes=len(content), sha256=sha(content)))
        registry.update(machine_id_sha256='d'*64, gpu_uuid='GPU-cpu-fixture',
            bundle_sha256=sha(data(sorted(registry['files'], key=lambda row: row['path']))))
        manifest.write_text(json.dumps(registry))
        original_sha = mixed366.sha
        with patch.object(mixed366, 'sha', side_effect=lambda path:
                          'd'*64 if str(path) == '/etc/machine-id' else original_sha(path)):
            wam.verify_registry(registry)
            checkpoint = assets / 'checkpoint'
            checkpoint.write_bytes(b'x' * checkpoint.stat().st_size)
            self.assertTrue(all(row['selectable'] for row in catalog.RegisteredCatalog(self.catalog_root).rows()))
            with self.assertRaisesRegex(ValueError, 'artifact changed: checkpoint'):
                wam.verify_registry(registry)


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
