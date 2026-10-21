"""Content-fenced Store cache: hot reads avoid JSON/canonical work, never byte checks."""
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from arx_env.joint_head_calibration_store import CalibrationStore


class StoreCacheTest(unittest.TestCase):
    def setUp(self):
        self.temporary=tempfile.TemporaryDirectory();self.addCleanup(self.temporary.cleanup)
        self.root=Path(self.temporary.name).resolve()/'store'
        self.store=CalibrationStore(self.root,'robot','simulation')
        self.head=self.store.register('head','head',{'value':1},'batch')
        self.wrist=self.store.register('wrist','wrist',{'value':2},'batch',dependency_id='head')
        self.store.install(self.head,self.wrist,geometry_id='geometry')
        self.store.list();self.store.active()

    def test_warm_reads_keep_bytes_checks_without_json_or_canonical_and_copy_results(self):
        import arx_env.joint_head_calibration_store as module
        original_open=os.open;opened=[]
        def opened_file(path,*args,**kwargs):
            if Path(path).name.startswith('v'):opened.append(Path(path).name)
            return original_open(path,*args,**kwargs)
        with patch.object(module.json,'loads',side_effect=AssertionError('hot reads must reuse validated parse')),\
                patch.object(module,'_canonical',side_effect=AssertionError('hot reads must reuse validated canonical hash')),\
                patch.object(module.os,'open',side_effect=opened_file):
            first=self.store.get('head');first['payload']['value']=999
            self.assertEqual(opened,['v000001.json'],'get checks only its target once the ID index is warm')
            self.assertEqual(self.store.get('head')['payload']['value'],1)
            rows=self.store.list();rows[0]['payload']['value']=999
            self.assertEqual(self.store.list()[0]['payload']['value'],1)
            active=self.store.active();active['acceptance']['injected']=True
            self.assertNotIn('injected',self.store.active()['acceptance'])

    def test_same_size_rewrite_with_identical_stat_is_detected_before_cached_use(self):
        path=Path(self.head['path']);before=path.stat();raw=path.read_bytes();changed=raw.replace(b'"value":1',b'"value":9')
        self.assertNotEqual(raw,changed);self.assertEqual(len(raw),len(changed))
        # Model the observed ARX case where even ctime/mtime/size/inode repeat.
        with patch.object(self.store,'_stamp',return_value=('identical-stat',)):
            self.assertTrue(self.store.active()['verified'])
            path.write_bytes(changed);os.utime(path,ns=(before.st_atime_ns,before.st_mtime_ns))
            with self.assertRaises(ValueError):self.store.get('head')
            with self.assertRaises(ValueError):self.store.list()
            self.assertFalse(self.store.active()['verified'])
        # Copying the same tampering into active cannot bypass the record hash.
        (self.root/'active/head.json').write_bytes(changed)
        self.assertFalse(self.store.active()['verified'])

    def test_external_register_replace_delete_and_explicit_invalidation(self):
        other=CalibrationStore(self.root,'robot','simulation')
        newer=other.register('head','next',{'value':3},'next-batch');generation=self.store.generation
        self.assertEqual(self.store.get('next'),newer);self.assertGreater(self.store.generation,generation)
        Path(newer['path']).unlink()
        with self.assertRaises(ValueError):self.store.get('next')
        self.store.invalidate();self.assertTrue(self.store.active()['verified'])
        # Legitimate same-name byte replacement is revalidated, not stale cached.
        payload=json.loads(Path(self.head['path']).read_text());payload['payload']['value']=7
        import hashlib
        from arx_env.joint_head_calibration_store import _canonical
        payload.pop('sha256');payload['sha256']=hashlib.sha256(_canonical(payload)).hexdigest()
        Path(self.head['path']).write_bytes(_canonical(payload))
        self.assertEqual(self.store.get('head')['payload']['value'],7)
        self.assertFalse(self.store.active()['verified'])

    def test_install_qualifies_actual_bytes_and_manifest_changes_invalidate(self):
        manifest=self.root/'active/manifest.json';value=json.loads(manifest.read_text());value['notes']='changed outside Store'
        manifest.write_text(json.dumps(value));self.assertEqual(self.store.active()['notes'],value['notes'])
        generation=self.store.generation
        self.store.register('left','candidate',{},'batch');self.assertGreater(self.store.generation,generation)
        self.assertTrue(self.store.active()['verified'])
        path=Path(self.head['path']);path.write_bytes(path.read_bytes().replace(b'"value":1',b'"value":9'))
        with self.assertRaises(ValueError):self.store.install(self.head,self.wrist,geometry_id='geometry')


if __name__=='__main__':unittest.main()
