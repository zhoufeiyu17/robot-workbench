"""Tiny producer artifacts exercise install/retry/publication without SDK or GPU use."""
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import shutil
import sys
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from robot_workbench import model_install as install
from robot_workbench.inference_catalog import RegisteredCatalog


@unittest.skipUnless(sys.platform == 'linux', 'target-side Linux installer')
class InstallTests(unittest.TestCase):
    def setUp(self):
        temporary = TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.source = self.root/'producer'
        self.source.mkdir()
        self.target = dict(schema=install.TARGET_SCHEMA, asset_root=str(self.root/'assets'),
            catalog_root=str(self.root/'catalog'), gpu_uuid='GPU-00000000-0000-0000-0000-000000000001',
            machine_id_sha256=hashlib.sha256(Path('/etc/machine-id').read_bytes()).hexdigest(),
            runtimes={name:dict(family=name, backend='torch', python=sys.executable)for name in ('wam','wam-mem')})
        self.downloads = []

    def release(self, family='wam', name='fixture', step=42):
        roles = ('checkpoint','stats','context','model_config','engine') if family=='wam' else (
            'checkpoint','vae','phoenix_encoder','csvae_checkpoint','text_index','text_features','source_manifest')
        artifacts = {}; files = []
        for role in roles:
            filename = role+'.pt' if role in ('checkpoint','vae','phoenix_encoder','csvae_checkpoint','text_features') else role+'.json'
            content = ((ROOT/'tests/fixtures/wam-e03-normalization.json').read_bytes() if role=='stats' else
                       (role+'-'+family+'-'+name).encode())
            (self.source/filename).write_bytes(content)
            artifacts[role] = filename
            files.append(dict(path=filename))
        spec = dict(model_id=family+'-'+name+'-step'+str(step), step=step,
            producer=dict(training_run='synthetic-run', checkpoint_complete=True), files=files,
            backends={'torch':dict(runtime_id=family,artifacts=artifacts,
                inference_capabilities={'denoise_steps':{'minimum':1,'maximum':10,'default':2}})})
        if family=='wam-mem':
            path=self.source/'source/world_memory/inference.py';path.parent.mkdir(parents=True);path.write_text('# synthetic source\n')
            spec['files'].append(dict(path=str(path.relative_to(self.source))))
            spec['backends']['torch'].update(model_source='source',instruction='synthetic instruction',cached_text_identity={})
        else:
            for name in ('phoenix_encoder.pt','csvae.pt','Wan2.2_VAE.safetensors'):
                (self.source/name).write_bytes(('synthetic-'+name).encode())
                spec['files'].append(dict(path=name))
        return install.export_release(spec,self.source,self.root/'release.json')

    def download(self,row,destination):
        self.downloads.append(row['path'])
        shutil.copyfile(self.source/row['path'],destination)

    def test_both_torch_families_are_discovered_with_all_sampling_variants(self):
        for family in ('wam','wam-mem'):
            release=self.release(family)
            first=install.install_release(release,self.target,self.download)
            count=len(self.downloads)
            second=install.install_release(release,self.target,self.download)
            self.assertFalse(first['reused']);self.assertTrue(second['reused'])
            self.assertEqual(len(self.downloads),count)
            rows=[r for r in RegisteredCatalog(self.target['catalog_root']).rows()if r['model_id']==release['model_id']]
            self.assertEqual(len(rows),10);self.assertTrue(all(r['selectable']for r in rows))
            self.assertEqual({r['manifest']['inference_options']['denoise_steps']for r in rows},set(range(1,11)))
            self.assertEqual(rows[0]['registry']['python'],sys.executable)

    def test_interrupted_transfer_never_publishes_and_retry_reuses_completed_files(self):
        release=self.release();failed=release['files'][2]['path']
        def interrupted(row,destination):
            if row['path']==failed:
                destination.write_bytes(b'partial')
                raise OSError('synthetic interruption')
            self.download(row,destination)
        with self.assertRaisesRegex(OSError,'interruption'):install.install_release(release,self.target,interrupted)
        self.assertFalse((Path(self.target['catalog_root'])/release['model_id']).exists())
        prior=self.downloads[:]
        install.install_release(release,self.target,self.download)
        self.assertTrue(all(self.downloads.count(name)==1 for name in prior))

    def test_concurrent_repeat_has_one_publisher_and_conflicting_identity_is_rejected(self):
        release=self.release()
        with ThreadPoolExecutor(max_workers=2)as workers:
            results=list(workers.map(lambda _:install.install_release(release,self.target,self.download),range(2)))
        self.assertEqual(sorted(r['reused']for r in results),[False,True])
        changed=deepcopy(release);changed['producer']['training_run']='different-run'
        with self.assertRaisesRegex(ValueError,'different assets'):install.install_release(changed,self.target,self.download)

    def test_invalid_runtime_and_artifact_escape_are_rejected_before_download(self):
        release=self.release();bad=deepcopy(self.target);bad['runtimes']['wam']['family']='wam-mem'
        with self.assertRaisesRegex(ValueError,'runtime family'):install.install_release(release,bad,self.download)
        self.assertEqual(self.downloads,[])
        release['files'][0]['path']='../escape'
        with self.assertRaises(ValueError):install.install_release(release,self.target,self.download)

    def test_large_weight_checks_never_read_the_weight_or_its_partial_file(self):
        row=dict(path='weight.pt',bytes=install.WEIGHT_BYTES+1,sha256=None,object='weight.pt')
        path=self.root/'download.incoming'
        with path.open('wb')as stream:stream.truncate(row['bytes'])
        with patch.object(Path,'read_bytes',side_effect=AssertionError('large weight read forbidden')):
            self.assertTrue(install.verify_file(path,row))

    def test_same_family_new_normalization_is_not_compared_with_an_old_checkpoint(self):
        release=self.release()
        stats=self.source/release['backends']['torch']['artifacts']['stats']
        value=json.loads(stats.read_text())
        # JSON whitespace changes its digest while retaining the declared normalization contract.
        stats.write_text(json.dumps(value,indent=3)+'\n')
        spec={k:v for k,v in release.items()if k not in ('schema','complete')}
        updated=install.export_release(spec,self.source,self.root/'updated.json')
        self.assertNotEqual(updated['files'],release['files'])
        self.assertEqual(install.install_release(updated,self.target,self.download)['status'],'installed')


if __name__=='__main__':unittest.main()
