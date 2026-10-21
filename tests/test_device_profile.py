"""Shared installed-profile construction with synthetic physical-interface records.

Uses the actual C store and A1 builder; no hardware observations or GPU qualification.
"""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'tests'),str(ROOT/'calibration/infra'),str(ROOT/'rule_based_workbench')]
sys.path.insert(0,str(ROOT/'calibration/examples/scripts/debug'))
from arx_env.joint_head_calibration_store import CalibrationStore
from joint_head_workbench import Workbench
from a1_cal import canonical
from robot_workbench.calibration_profile import build_device_profile,build_fake_profile,load_device_profile,installed_profile_provider
from test_warm_diagnostic import nominal,sha
from test_fake_calibration_profile import fixture,write
from test_harness import PROFILE


class DeviceProfileTests(unittest.TestCase):
    def setUp(self):
        self.temp=TemporaryDirectory();self.addCleanup(self.temp.cleanup);self.root=Path(self.temp.name).resolve()
        a1=nominal(self.root/'a1');contract=json.loads((a1/'calibration-input.json').read_text())
        self.capture=self.root/'captures';self.capture.mkdir();self.service=self.root/'service';self.service.mkdir()
        geometry,seal=contract['geometry_id'],sha(a1/'SHA256SUMS.json')
        robot='A1-physical-interface-cpu-fixture';flags=dict(robot_source='physical',simulation_only=False)
        clock=dict(environment='physical',clock_domain='host_wall',source_clock_id=None)
        cameras={role:dict(serial=str(800+index),width=640,height=480,fx=400.,fy=401.,cx=320.,cy=240.,
            coefficients=[0.]*5,distortion_model='inverse_brown_conrady',rectified=False)
            for index,role in enumerate(('head','left','right'))}
        transform=np.eye(4).tolist()
        parameters=dict(joint_offsets_rad=dict(left=[.01]*6,right=[-.02]*6),camera_T_world_optical=transform,
                        asset_geometry_id=geometry,asset_seal_sha256=seal)
        wrist_contract=dict(geometry_id='cpu-physical-workbench',asset_geometry_id=geometry,asset_seal_sha256=seal,
            head_snapshot={'id':'head'},cameras={role:cameras[role] for role in ('left','right')})
        store=CalibrationStore(self.root/'installed/hardware',robot,'hardware')
        head=store.register('head','head',{'row':dict(parameters=parameters,**flags)},'batch-head/capture',environment='hardware')
        wrist=store.register('wrist','wrist',{'row':dict(pair_status='complete',asset_geometry_id=geometry,
            asset_seal_sha256=seal,**flags,left={'T_link6_camera':transform},right={'T_link6_camera':transform}),
            'contract':wrist_contract},'batch-wrist/capture',environment='hardware',dependency_id=head['id'])
        for kind in ('head','wrist'):
            data=dict(environment='hardware',runtime_environment='physical',robot_id=robot,asset_geometry_id=geometry,
                asset_seal_sha256=seal,frames=[{'id':'synthetic-physical-interface-cpu-fixture'}],
                camera=cameras['head'],clock=clock,**flags)
            if kind=='wrist':data['wrist_contract']=wrist_contract
            write(self.capture/('batch-'+kind)/'capture/dataset.json',data)
            write(self.capture/('batch-'+kind)/'capture/completion.json',{'passed':True,'cpu_fixture_only':True})
        store.install(head,wrist,geometry_id='cpu-physical-workbench',robot_source='physical',
                      acceptance={'status':'synthetic_cpu_contract_fixture_only'})
        camera=self.root/'head-camera.json';write(camera,cameras['head'])
        self.config=dict(environment='physical',runtime_environment='physical',robot_id=robot,**flags,model_family='ac_one',clock=clock,
            calibration_root=str(self.root/'installed'),urdf=str(a1/'robot.urdf'),camera=str(camera),
            jaw_mapping_file=str(a1/'jaw-mapping.json'),asset_geometry_id=geometry,asset_seal_sha256=seal,
            hardware_camera_serials={role:value['serial'] for role,value in cameras.items()})
        self.path=self.service/'calibration-config.json';write(self.path,self.config)
        self.expected=dict(head_id=head['id'],wrist_id=wrist['id'],manifest_sha256=sha(self.root/'installed/hardware/active/manifest.json'))
        self.registration=dict(length_unit='m',T_environment_calibration_world=np.eye(4).tolist())

    def build(self,name='profile'):
        return build_device_profile(self.path,self.capture,PROFILE,self.registration,self.root/name,expected_install=self.expected)

    @staticmethod
    def workbench(config,capture):
        app=Workbench.__new__(Workbench);app.config=config;app.output=capture
        app.control_mode='hardware_sync';app.geometry_id='updated-software-and-planning-policy'
        app.hardware_store=CalibrationStore(Path(config['calibration_root'])/'hardware',config['robot_id'],'hardware')
        return app

    def test_software_drift_preserves_installed_pair_in_both_modes_but_not_old_plan(self):
        fake=self.root/'fake';fake.mkdir()
        capture,active,config,expected,registration=fixture(fake,self.root/'a1')
        for mode,cfg,frames,receipt,config_path,destination in (
                ('physical',self.config,self.capture,self.expected,self.path,self.root/'physical-profile'),
                ('fake',config,capture,expected,fake/'calibration-config.json',fake/'profile')):
            with self.subTest(mode=mode):
                app=self.workbench(cfg,frames);installed=Path(cfg['calibration_root'])/'hardware/active'
                before={path:path.read_bytes() for path in installed.glob('*.json')}
                snapshot=app.installed_pair()
                self.assertTrue(snapshot['verified'],snapshot)
                self.assertFalse(snapshot['workbench_fingerprint_matches'])
                self.assertEqual(snapshot['geometry_id'],json.loads(before[installed/'manifest.json'])['geometry_id'])
                app.plan={'plan_complete':True};app.plan_geometry_id=snapshot['geometry_id'];app.queue=[]
                self.assertFalse(app.queue_is_plan_checked([]),'old plans still require current software/policy checks')
                result=build_device_profile(config_path,frames,PROFILE,registration,destination,expected_install=receipt)
                self.assertEqual(result['simulation_only'],mode=='fake')
                self.assertEqual(before,{path:path.read_bytes() for path in before})

    def test_installed_pair_keeps_asset_source_group_and_byte_guards_after_cache_warmup(self):
        app=self.workbench(self.config,self.capture);self.assertTrue(app.installed_pair()['verified'])
        for field,value in [('asset_geometry_id','different-asset'),('asset_seal_sha256','different-seal'),
                            ('environment','fake')]:
            with self.subTest(field=field):
                original=app.config[field];app.config[field]=value
                self.assertFalse(app.installed_pair()['verified'])
                app.config[field]=original;self.assertTrue(app.installed_pair()['verified'])
        active=app.hardware_store.root/'active';manifest_path=active/'manifest.json'
        manifest_bytes=manifest_path.read_bytes();manifest=json.loads(manifest_bytes)
        for change in ({'geometry_id':'other-group-geometry'},{'robot_id':'another-robot'},
                       {'environment':'simulation'},{'robot_source':'simulated'},
                       {'wrist':{**manifest['wrist'],'dependency_id':'another-head'}}):
            with self.subTest(change=change):
                write(manifest_path,{**manifest,**change})
                self.assertFalse(app.installed_pair()['verified'])
                manifest_path.write_bytes(manifest_bytes);self.assertTrue(app.installed_pair()['verified'])
        wrist_path=active/'wrist.json';raw=wrist_path.read_bytes();wrist=json.loads(raw)
        published=app.hardware_store.root/(wrist['version']+'.json')
        for target,key,value in [('contract','geometry_id','other-software'),('contract','head_snapshot',{'id':'other-head'}),
                                 ('row','asset_seal_sha256','wrong-seal'),('row','robot_source','simulated')]:
            with self.subTest(target=target,key=key):
                changed=deepcopy(wrist);changed['payload'][target][key]=value
                changed.pop('sha256');changed['sha256']=hashlib.sha256(canonical(changed)).hexdigest()
                write(wrist_path,changed);write(published,changed)
                write(manifest_path,{**manifest,'wrist':{**manifest['wrist'],'sha256':changed['sha256']}})
                self.assertFalse(app.installed_pair()['verified'],'even self-consistent file digests cannot change group identity')
                wrist_path.write_bytes(raw);published.write_bytes(raw);manifest_path.write_bytes(manifest_bytes)
                self.assertTrue(app.installed_pair()['verified'])
        wrist_path.write_bytes(raw+b' ')
        self.assertFalse(app.installed_pair()['verified'],'installed bytes must still equal the versioned record')

    def test_software_drift_does_not_bypass_three_camera_identity_in_either_mode(self):
        fake=self.root/'fake';fake.mkdir()
        capture,_,config,expected,registration=fixture(fake,self.root/'a1')
        for mode,cfg,frames,receipt,config_path in (
                ('physical',self.config,self.capture,self.expected,self.path),
                ('fake',config,capture,expected,fake/'calibration-config.json')):
            app=self.workbench(cfg,frames)
            self.assertTrue(app.installed_pair()['verified'])
            for role in ('head','left','right'):
                with self.subTest(mode=mode,role=role):
                    wrong=deepcopy(cfg);wrong['hardware_camera_serials'][role]='another-sensor';write(config_path,wrong)
                    with self.assertRaisesRegex(ValueError,'camera identities'):
                        build_device_profile(config_path,frames,PROFILE,registration,
                            self.root/(mode+'-'+role),expected_install=receipt)
                    write(config_path,cfg)

    def test_physical_and_fake_use_same_offset_equations_and_loader_without_gpu(self):
        result=self.build();self.assertFalse(result['simulation_only']);self.assertFalse(result['runtime_allowed'])
        profile,binding=load_device_profile(Path(result['profile_path']),environment='physical',robot_id=self.config['robot_id'])
        self.assertEqual(profile['source']['kind'],'installed_physical_calibration')
        self.assertEqual(profile['joints']['zero_offsets_rad'],[.01]*6+[-.02]*6)
        self.assertEqual(binding['evidence_source'],'physical_rgbd')
        fake=self.root/'fake';fake.mkdir()
        capture,_,_,expected,registration=fixture(fake,self.root/'a1')
        fake_result=build_fake_profile(fake/'calibration-config.json',capture,PROFILE,registration,fake/'profile',expected_install=expected)
        fake_profile,_=load_device_profile(fake_result['profile_path'],environment='fake')
        self.assertEqual(profile['joints'],fake_profile['joints'])
        self.assertNotIn('isaaclab',sys.modules);self.assertNotIn('torch',sys.modules)

    def test_shared_inference_provider_reuses_then_rebuilds_for_current_installation(self):
        template=self.root/'template.json';write(template,PROFILE)
        registration=self.root/'registration.json';write(registration,self.registration)
        installed=dict(verified=True,head_id=self.expected['head_id'],wrist_id=self.expected['wrist_id'])
        provider=installed_profile_provider(config_path=self.path,capture_root=self.capture,registration=registration,
            profile_template=template,output=self.service,installed_snapshot=lambda:deepcopy(installed),environment='physical')
        with patch('robot_workbench.calibration_profile.build_device_profile',wraps=build_device_profile) as build:
            first=provider();self.assertEqual(provider(),first);build.assert_called_once()
            manifest=self.root/'installed/hardware/active/manifest.json'
            manifest.write_bytes(manifest.read_bytes()+b' ')
            second=provider();self.assertEqual(build.call_count,2)
            self.assertNotEqual(first[1]['calibration_snapshot_id'],second[1]['calibration_snapshot_id'])
            installed['verified']=False
            with self.assertRaisesRegex(ValueError,'正式安装'):provider()
            self.assertEqual(build.call_count,2)
        self.assertNotIn('isaaclab',sys.modules);self.assertNotIn('torch',sys.modules)

    def test_cross_environment_inputs_and_stale_artifacts_rejected(self):
        with self.assertRaisesRegex(ValueError,'fake'):
            build_fake_profile(self.path,self.capture,PROFILE,self.registration,self.root/'bad',expected_install=self.expected)
        path=self.capture/'batch-head/capture/dataset.json';original=path.read_bytes();data=json.loads(original)
        data.update(runtime_environment='fake',simulation_only=True,robot_source='simulated');write(path,data)
        with self.assertRaises(ValueError):self.build('fake-capture')
        path.write_bytes(original)
        result=self.build()
        with self.assertRaises(ValueError):load_device_profile(result['profile_path'],environment='fake')
        with self.assertRaises(ValueError):load_device_profile(result['profile_path'],environment='physical',robot_id='another-device')
        path.write_bytes(original+b' ')
        with self.assertRaisesRegex(ValueError,'input changed'):load_device_profile(result['profile_path'],environment='physical')

    def test_legacy_physical_capture_is_read_without_rewriting_or_inventing_clocks(self):
        originals={}
        for kind in ('head','wrist'):
            path=self.capture/('batch-'+kind)/'capture/dataset.json';data=json.loads(path.read_text())
            data.pop('runtime_environment');data.pop('clock');write(path,data);originals[path]=path.read_bytes()
        result=self.build('legacy');receipt=json.loads((Path(result['output'])/'bridge.json').read_text())
        legacy=receipt['legacy_capture_schema']
        self.assertEqual(set(legacy['captures']),{'head','wrist'})
        self.assertFalse(legacy['original_datasets_modified']);self.assertFalse(legacy['original_timestamps_modified'])
        self.assertFalse(legacy['captures']['head']['clock_field_present'])
        self.assertEqual(receipt['calibration_capture_clock'],dict(environment='physical',clock_domain='host_wall',source_clock_id=None))
        for path,raw in originals.items():self.assertEqual(path.read_bytes(),raw)
        path=self.capture/'batch-head/capture/dataset.json';legacy_data=json.loads(originals[path])
        for index,change in enumerate(({'runtime_environment':None},{'runtime_environment':'fake'},
                {'environment':'simulation'},{'robot_id':'other'},{'simulation_only':True},{'robot_source':'simulated'})):
            write(path,{**legacy_data,**change})
            with self.assertRaises(ValueError):self.build('contradiction-'+str(index))
        path.write_bytes(originals[path])
        fake=self.root/'fake-legacy';fake.mkdir()
        capture,_,_,expected,registration=fixture(fake,self.root/'a1')
        path=next(capture.glob('*/capture/dataset.json'));data=json.loads(path.read_text());data.pop('runtime_environment');write(path,data)
        with self.assertRaises(ValueError):
            build_fake_profile(fake/'calibration-config.json',capture,PROFILE,registration,fake/'profile',expected_install=expected)


if __name__=='__main__':unittest.main()
