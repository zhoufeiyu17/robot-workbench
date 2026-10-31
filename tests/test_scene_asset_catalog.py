"""Actual catalog, HTTP and preparation thread; all compute/control steps are CPU fixtures."""
from copy import deepcopy
import hashlib,json
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import threading
import unittest
from unittest.mock import Mock,patch
from urllib.request import build_opener,ProxyHandler
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'rule_based_workbench'),str(ROOT/'calibration/infra')]
from robot_workbench.scene_asset_catalog import SceneAssetCatalog
from robot_workbench.device_bootstrap import RemoteCollectionPreparation
from robot_workbench.resident_collection import CollectionPreparation
from robot_workbench.service import Device,Application,serve


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def catalog_fixture(root):
    image=root/'preview.png';image.write_bytes(b'\x89PNG\r\n\x1a\nCPU-only image-file boundary fixture')
    template=root/'profile.json';template.write_text('{"cpu_fixture":true}')
    base=dict(compute=dict(endpoint={'host':'cpu-fixture-only'},deployment_path='/cpu/legacy.json',deployment_sha256='a'*64),
        registration=str(root/'registration.json'),profile_template=str(template))
    assets={key:dict(kind=kind,label=label,asset_id=key,revision='cpu-v1',dimensions_m=size,
        preview=dict(file=image.name,sha256=sha(image)))for key,kind,label,size in (
            ('machine','machine','咖啡机',[.23,.43,.345]),('open','cup','无盖杯',[.079,.079,.1105]),
            ('lid','cup','带盖杯',[.082,.082,.124]))}
    pairs={key:dict(machine='machine',cup=cup,asset_set_digest=digest*64,
        compute=dict(base['compute'],deployment_path='/cpu/'+key+'.json'),
        profile_template=str(template),profile_template_sha256=sha(template))for key,cup,digest in (
            ('machine-open','open','b'),('machine-lid','lid','c'))}
    value=dict(schema='workbench.scene_asset_catalog.v1',assets=assets,pairs=pairs)
    path=root/'catalog.json';path.write_text(json.dumps(value))
    return dict(path=str(path),sha256=sha(path)),base,value


class CPUPreparation(RemoteCollectionPreparation):
    def _require_control(self):pass
    def installation(self,snapshot):return dict(head_id='h',wrist_id='w',manifest_sha256='d'*64)
    def _prepare(self,snapshot):
        self.received.append(deepcopy(self.details));self._status('converting_planning_assets')
        if self.fail_next:
            self.fail_next=False;raise ValueError('CPU fixture: new pair conversion failed')
        self._status('ready',error=None)
    def environment_status(self):return dict(services_ready=True,ready=False,needs_warmup=True)


class CatalogTest(unittest.TestCase):
    def setUp(self):
        self.temp=TemporaryDirectory();self.addCleanup(self.temp.cleanup);self.root=Path(self.temp.name).resolve()
        self.reference,self.base,self.value=catalog_fixture(self.root)
        self.catalog=SceneAssetCatalog(self.reference,self.base)
        self.device=Device('cpu-assets','fake',None);self.app=Application({'fake':self.device})
        self.device.connect=Mock(side_effect=AssertionError('asset selection must not connect SDK'))
        self.device.submit=Mock(side_effect=AssertionError('asset selection must not issue robot commands'))
        install=dict(verified=True,head_id='h',wrist_id='w')
        self.prep=CPUPreparation(details=dict(self.base,asset_catalog=self.reference),calibration=None,
            calibration_config=self.root/'calibration.json',contract={'resources':dict(persistent_service=True,
                service_lifetime_seconds=None,resident_deadline_wall_ns=None)},output=self.root,base={},device=self.device,
            application=self.app,ports={},directories={},control_url='http://127.0.0.1:1',token_file='unused',
            deadline_wall_ns=None,installed_snapshot=lambda:deepcopy(install))
        self.prep.received=[];self.prep.fail_next=False;self.device.collection_preparation=self.prep
        self.prep.notify(install);self.addCleanup(self.prep.close)

    def prepare(self,pair):
        self.device.maintenance=True
        try:return self.prep.operations_prepare(still_current=lambda:True,lifecycle_admission=lambda role:True,
            asset_pair_id=pair,expected_catalog_digest=self.reference['sha256'])
        finally:self.device.maintenance=False

    def test_directory_only_exposes_ids_and_preview_never_changes_control(self):
        public=self.catalog.public('fake');serialized=json.dumps(public)
        self.assertNotIn('deployment_path',serialized);self.assertNotIn('profile_template',serialized)
        self.assertNotIn(str(self.root),serialized)
        server=serve(self.app,port=0);thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        try:
            opener=build_opener(ProxyHandler({}));base=f'http://127.0.0.1:{server.server_port}'
            with opener.open(base+'/api/state?environment=fake&business=collection')as response:
                state=json.load(response)
            self.assertEqual(len(state['scene_assets']['assets']),3)
            with opener.open(base+public['assets'][0]['preview_url'])as response:
                self.assertEqual(response.headers['Content-Type'],'image/png')
                self.assertTrue(response.read().startswith(b'\x89PNG'))
            self.assertEqual(self.prep.received,[])
            self.device.connect.assert_not_called();self.device.submit.assert_not_called()
        finally:server.shutdown();server.server_close();thread.join(2)

    def test_apply_requires_registered_ids_and_current_directory(self):
        before=deepcopy(self.prep.details)
        for pair,digest in [('not-registered',self.reference['sha256']),('machine-open','f'*64)]:
            with self.assertRaises(ValueError):self.prep.validate_asset_request(pair,digest)
        self.assertEqual(self.prep.details,before);self.assertEqual(self.prep.received,[])
        with self.assertRaisesRegex(ValueError,'请先选择'):self.prep.validate_asset_request()
        self.prepare('machine-open')
        state=self.prep.snapshot();self.assertEqual(state['applied_asset_pair_id'],'machine-open')
        self.assertEqual(self.prep.received[0]['compute']['deployment_path'],'/cpu/machine-open.json')
        with patch.object(CollectionPreparation,'_reuse_parts',return_value=True)as reuse:
            self.assertTrue(self.prep._reuse_parts({},lambda:True))
            self.prep.value['requested_asset_pair_id']='machine-lid'
            self.assertFalse(self.prep._reuse_parts({},lambda:True));self.assertEqual(reuse.call_count,1)
        self.device.connect.assert_not_called();self.device.submit.assert_not_called()

    def test_failed_new_pair_keeps_previous_applied_identity_and_blocks_busy_switch(self):
        self.prepare('machine-open');before=deepcopy(self.prep.details)
        with patch.object(self.device,'snapshot',return_value={'automatic_enabled':True}):
            with self.assertRaisesRegex(ValueError,'动作或录制'):self.prepare('machine-lid')
        self.assertEqual(self.prep.details,before)
        self.prep.fail_next=True
        with self.assertRaisesRegex(ValueError,'new pair conversion failed'):self.prepare('machine-lid')
        state=self.prep.snapshot();self.assertEqual(state['status'],'failed')
        self.assertEqual(state['applied_asset_pair_id'],'machine-open')
        self.assertEqual(state['requested_asset_pair_id'],'machine-lid')
        self.prepare('machine-lid');self.assertEqual(self.prep.snapshot()['applied_asset_pair_id'],'machine-lid')

    def test_path_escape_preview_tamper_and_changed_template_are_rejected(self):
        image=self.root/'preview.png';image.write_bytes(b'changed')
        with self.assertRaisesRegex(ValueError,'预览文件或摘要'):self.catalog.preview('machine',self.reference['sha256'])
        template=self.root/'profile.json';template.write_text('changed')
        with self.assertRaisesRegex(ValueError,'接口模板已变化'):self.catalog.resolve('machine-open',self.reference['sha256'])
        value=deepcopy(self.value);value['assets']['machine']['preview']['file']='../outside.png'
        path=self.root/'bad-catalog.json';path.write_text(json.dumps(value))
        with self.assertRaisesRegex(ValueError,'目录内固定图片'):
            SceneAssetCatalog(dict(path=str(path),sha256=sha(path)),self.base)


if __name__=='__main__':unittest.main()
