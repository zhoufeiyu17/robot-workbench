import json
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from arx_env.joint_head_calibration_store import CalibrationStore


class CalibrationStoreTest(unittest.TestCase):
    def test_source_partition_preserves_bytes_and_rejects_relabel(self):
        sim=self.store.register('head','sim',{},'sim-batch')
        real=self.store.register('head','real',{},'real-batch',environment='hardware')
        before={r['id']:Path(r['path']).read_bytes() for r in (sim,real)}
        self.store.split_legacy();self.store.split_legacy()
        self.assertEqual(self.store.list(),[])
        for env,identity in [('simulation','sim'),('hardware','real')]:
            store=CalibrationStore(self.root/env,'robot-1',env)
            self.assertEqual(Path(store.get(identity)['path']).read_bytes(),before[identity])
            with self.assertRaises(ValueError):store.register('head',identity,{},'batch',environment='hardware' if env=='simulation' else 'simulation')

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name).resolve()/'versions'
        self.store = CalibrationStore(self.root,'robot-1')

    def test_persistence_dedup_conflict_and_capture_independence(self):
        raw=self.root.parent/'raw.json';raw.write_text('raw input')
        payload={'T':[1,2,3]}
        first=self.store.register('head','head-1',payload,str(raw),validation={'passed':True})
        payload['T'][0]=9
        self.assertEqual(first['payload']['T'],[1,2,3])
        raw.unlink()
        reopened=CalibrationStore(self.root,'robot-1')
        self.assertEqual(reopened.get('head-1'),first)
        self.assertEqual(reopened.register('head','head-1',{'T':[1,2,3]},'other'),first)
        second=reopened.register('left','left-1',{'T':[4]},'batch',dependency_id='head-1')
        self.assertEqual([r['version'] for r in reopened.list()],['v000001','v000002'])
        self.assertEqual(second['dependency_id'],'head-1')
        for kind,payload in [('head',{'T':[9]}),('right',{'T':[1,2,3]})]:
            with self.assertRaises(ValueError):reopened.register(kind,'head-1',payload,'batch')
        with self.assertRaises(ValueError):reopened.get('missing')
        with self.assertRaises(ValueError):CalibrationStore(self.root,'robot-2').list()

    def test_tamper_is_rejected_on_every_read_and_registration(self):
        record=self.store.register('head','head-1',{'value':1},'batch')
        path=Path(record['path']);stored=json.loads(path.read_text());stored['payload']['value']=2
        path.write_text(json.dumps(stored))
        for action in (self.store.list,lambda:self.store.get('head-1'),
                       lambda:self.store.register('right','right-1',{},'batch')):
            with self.assertRaises(ValueError):action()

    def test_traversal_symlinks_and_non_json_values_rejected(self):
        with self.assertRaises(ValueError):CalibrationStore('relative','robot-1')
        with self.assertRaises(ValueError):CalibrationStore(self.root/'..'/'escaped','robot-1')
        for value in ('../escape','a/b','a\\b','..','', '/absolute'):
            with self.assertRaises(ValueError):self.store.register('head',value,{},'batch')
            with self.assertRaises(ValueError):self.store.get(value)
        with self.assertRaises(ValueError):self.store.register('head','bad',{'value':float('nan')},'batch')
        alias=self.root.parent/'alias';alias.symlink_to(self.root,target_is_directory=True)
        with self.assertRaises(ValueError):CalibrationStore(alias/'child','robot-1')
        target=self.root.parent/'outside.json';target.write_text('{}')
        (self.root/'v000001.json').symlink_to(target)
        with self.assertRaises(ValueError):self.store.list()
        self.assertEqual(target.read_text(),'{}')

    def test_threads_share_one_monotonic_sequence_and_dedup(self):
        def register(i):
            return CalibrationStore(self.root,'robot-1').register(('head','left','right')[i%3],f'id-{i}',{'i':i},'batch')
        with ThreadPoolExecutor(max_workers=6) as pool:
            records=list(pool.map(register,list(range(12))*2))
        self.assertEqual(len(self.store.list()),12)
        self.assertEqual(sorted({r['version'] for r in records}),[f'v{i:06d}' for i in range(1,13)])
        self.assertEqual(list(self.root.glob('.pending-*')),[])

    def test_atomic_publication_never_overwrites_existing_target(self):
        import os
        original_link=os.link
        def competing_writer(source,target):
            Path(target).write_bytes(b'preexisting version')
            original_link(source,target)
        with patch('arx_env.joint_head_calibration_store.os.link',side_effect=competing_writer):
            with self.assertRaises(FileExistsError):self.store.register('head','head-1',{},'batch')
        self.assertEqual((self.root/'v000001.json').read_bytes(),b'preexisting version')
        self.assertEqual(list(self.root.glob('.pending-*')),[])


if __name__ == '__main__':
    unittest.main()
