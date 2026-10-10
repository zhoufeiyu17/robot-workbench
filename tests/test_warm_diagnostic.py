"""CPU-only pinned warm-source reuse, using real C store and profile construction."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import threading
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'scripts'),str(ROOT/'calibration/infra'),str(ROOT/'rule_based_workbench')]
from a1_cal import canonical
from model_adapt import seal,verify
from test_fake_calibration_profile import fixture,write
from test_harness import PROFILE
from robot_workbench.calibration_profile import (build_fake_profile,validate_diagnostic_source,
    diagnostic_snapshot,diagnostic_metadata,diagnostic_mesh_view)
from robot_workbench.resident_collection import CollectionPreparation


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def nominal(root):
    root.mkdir();order=[f'{side}_joint{i}' for side,indices in [('left',range(1,7)),('right',range(11,17))] for i in indices]
    joints=[f'<joint name="{name}" type="revolute"><parent link="base_link"/><child link="{name}_link"/><origin xyz="0 0 0" rpy="0 0 0"/></joint>' for name in order]
    parents=dict(head='base_link',left='left_link6',right='right_link16')
    for side,prefix in [('head','top'),('left','left_wrist'),('right','right_wrist')]:
        joints.extend([f'<joint name="{prefix}_camera_link_joint" type="fixed"><parent link="{parents[side]}"/><child link="{prefix}_camera_link"/><origin xyz="0 0 0" rpy="0 0 0"/></joint>',
            f'<joint name="{prefix}_camera_rgb_optical_frame_joint" type="fixed"><parent link="{prefix}_camera_link"/><child link="{prefix}_optical"/></joint>'])
    (root/'robot.urdf').write_text('<robot name="cpu-only-a1">'+''.join(joints)+'</robot>')
    geometry={'robot.urdf':sha(root/'robot.urdf')}
    write(root/'calibration-input.json',dict(schema='robot_calibration_input.v1',geometry_id=hashlib.sha256(canonical(geometry)).hexdigest(),
        geometry_files=geometry,joint_order=order,joint_signs=[1]*12,camera_parents=parents,zero_offsets_rad=None,
        units={'length':'m','joint_angle':'rad','public_jaw_gap':'mm'}))
    write(root/'manifest.json',dict(calibration_status='required_head_then_wrists'))
    write(root/'jaw-mapping.json',{'cpu_fixture_only':True})
    seal(root);verify(root)
    return root


class WarmDiagnosticTest(unittest.TestCase):
    def setUp(self):
        self.temp=TemporaryDirectory();self.addCleanup(self.temp.cleanup);self.root=Path(self.temp.name).resolve()
        self.origin=self.root/'origin';self.origin.mkdir();self.current=self.root/'current';self.current.mkdir()
        self.capture,self.active,self.config,self.expected,self.registration=fixture(self.origin,nominal(self.root/'a1'))
        self.base=dict(pipeline_config=dict(assets='/fixed/assets',native_code='/fixed/native'),initial_suite={'episodes':[{'seed':0}]},
            native_identity={'physical_configuration':{'fixture':'fixed-physical-identity'}},
            lanes={'source':{'raw_provenance':{'source_files':{'source/fixture.py':'a'*64}},'configuration':{'cameras':{'rig':'fixed'}}}})
        self.source_base=self.root/'source-base.json';write(self.source_base,self.base)
        write(self.root/'nominal-config.json',self.config)
        write(self.origin/'run.json',{'base_sha256':sha(self.source_base),'nominal_config_sha256':sha(self.root/'nominal-config.json')})
        self.markers=[dict(id=24,print_surface_offset_m=.0001)]
        write(self.origin/'truth-fixture.json',{'markers':self.markers})
        write(self.origin/'plant-fixture.json',dict(source_clock_id=self.config['clock']['source_clock_id'],joint_offsets_rad=[0.]*12,metal_gap_offset_mm=.008))
        write(self.origin/'registration.json',self.registration)
        self.spec=dict(mode='warm_diagnostic',source_run=str(self.origin),source_base_manifest=str(self.source_base),
                       expected_install=deepcopy(self.expected),input_sha256={})
        self.repin()
        self.new=deepcopy(self.config);self.new.update(robot_id='fake-current-new-instance',calibration_root=str(self.current/'calibrations'),
            clock=dict(environment='fake',clock_domain='simulation',source_clock_id='new-current-clock'),camera=str(self.current/'camera-head.json'))
        write(self.current/'camera-head.json',json.loads((self.origin/'camera-head.json').read_text()))
        wrist=json.loads((self.active/'wrist.json').read_text())['payload']['contract']
        self.new['hardware_wrist_camera_files']={}
        for side in ('left','right'):
            path=self.current/('camera-'+side+'.json');write(path,wrist['cameras'][side]);self.new['hardware_wrist_camera_files'][side]=str(path)
        write(self.current/'calibration-config.json',self.new)
        (self.current/'calibration-workbench').mkdir()

    def repin(self):
        paths=[self.source_base,self.root/'nominal-config.json',*[p for p in self.origin.rglob('*.json') if '.active.' not in str(p)],
               *[p for p in (self.root/'a1').rglob('*') if p.is_file()]]
        self.spec['input_sha256']={str(path):sha(path) for path in paths}

    def validate(self,**changes):
        return validate_diagnostic_source(changes.get('spec',self.spec),changes.get('base',self.base),
            changes.get('nominal',self.config),changes.get('markers',self.markers),changes.get('output',self.current))

    def build(self,name='derived',**changes):
        source=changes['source'] if 'source' in changes else self.validate()
        return build_fake_profile(self.current/'calibration-config.json',self.current/'calibration-workbench',PROFILE,
            self.registration,self.current/name,expected_install=changes.get('expected',self.expected),
            diagnostic_source=source)

    def test_real_profile_keeps_origin_and_new_identity_separate_without_current_install(self):
        before={path:sha(path) for path in self.spec['input_sha256']}
        source=self.validate();result=self.build(source=source);output=Path(result['output'])
        binding=json.loads((output/'acquisition-binding.json').read_text());profile=json.loads((output/'profile.json').read_text())
        bridge=json.loads((output/'bridge.json').read_text());model=json.loads((output/'planning-model.json').read_text())
        self.assertEqual(binding['robot_id'],self.new['robot_id']);self.assertNotIn('clock',binding)
        self.assertNotIn('capture_owner',binding);self.assertNotIn('initial_state',binding)
        self.assertEqual(binding['diagnostic_origin']['origin_robot_id'],self.config['robot_id'])
        self.assertEqual(binding['diagnostic_origin']['origin_clock'],self.config['clock'])
        self.assertEqual(profile['physical_calibration']['robot_id'],self.config['robot_id'])
        self.assertEqual(bridge['observation_clock'],self.new['clock']);self.assertEqual(bridge['calibration_capture_clock'],self.config['clock'])
        self.assertEqual(profile['joints']['zero_offsets_rad'],[.01]*6+[-.02]*6)
        self.assertFalse(result['fresh_C_completed']);self.assertFalse(binding['fresh_C_completed'])
        self.assertFalse(model['joint_zero_offsets_baked_into_urdf']);self.assertEqual(model['urdf'],str(output/'model/robot.urdf'))
        self.assertFalse((self.current/'calibrations/hardware/active').exists())
        graph='\n'.join(path.read_text() for path in output.rglob('*.json'))
        for forbidden in ('truth-fixture.json','plant-fixture.json','plant_joint_offsets_rad',str(self.source_base)):
            self.assertNotIn(forbidden,graph)
        admission=json.loads((output/'diagnostic-admission.json').read_text())
        self.assertEqual(admission['admission_evidence_files'],len(source['input_sha256']))
        self.assertEqual(bridge['diagnostic_admission_sha256'],sha(output/'diagnostic-admission.json'))
        self.assertNotIn(str(self.origin/'truth-fixture.json'),bridge['input_sha256'])
        for kind in ('head','wrist','manifest'):
            self.assertEqual(json.loads((output/'model/calibration_sources'/(kind+'.json')).read_text()),json.loads((self.active/(kind+'.json')).read_text()))
        self.assertEqual(before,{path:sha(path) for path in before})
        self.assertNotIn('plant_joint_offsets_rad',self.new);self.assertNotIn('diagnostic_calibration_source',self.new)
        self.assertNotIn('isaaclab',sys.modules)

    def test_default_cold_profile_still_uses_current_C_identity(self):
        result=build_fake_profile(self.origin/'calibration-config.json',self.capture,PROFILE,self.registration,
                                 self.origin/'cold-derived',expected_install=self.expected)
        binding=json.loads((Path(result['output'])/'acquisition-binding.json').read_text())
        self.assertEqual(binding['robot_id'],self.config['robot_id'])
        self.assertNotIn('diagnostic_origin',binding);self.assertNotIn('fresh_C_completed',result)

    def test_source_file_pin_missing_or_changed_rejected_before_build(self):
        path=self.capture/'batch-head/capture/dataset.json';original=path.read_bytes();path.write_bytes(original+b' ')
        with self.assertRaisesRegex(ValueError,'pin changed'):self.validate()
        path.write_bytes(original)
        spec=deepcopy(self.spec);spec['input_sha256'].pop(str(path))
        with self.assertRaisesRegex(ValueError,'omits'):self.validate(spec=spec)
        source=self.validate();path.write_bytes(original+b' ')
        with self.assertRaisesRegex(ValueError,'pin changed'):self.build(source=source)
        self.assertFalse((self.current/'derived').exists())

    def test_claimed_new_geometry_hash_cannot_override_original_asset_seal(self):
        path=self.root/'a1/robot.urdf';path.write_text(path.read_text()+'\n')
        self.spec['input_sha256'][str(path)]=sha(path)
        with self.assertRaisesRegex(ValueError,'frozen model digest mismatch'):self.validate()
        self.assertFalse((self.current/'derived').exists())

    def test_different_fixture_group_source_identity_and_head_dependency_rejected(self):
        for key,value in [('plant_joint_offsets_rad',[.003]+[0.]*11),('initial_suite',{'episodes':[{'seed':1}]})]:
            base=deepcopy(self.base);base[key]=value
            with self.assertRaisesRegex(ValueError,'fixture'):self.validate(base=base)
        with self.assertRaisesRegex(ValueError,'printed-marker'):self.validate(markers=[])
        spec=deepcopy(self.spec);spec['expected_install']['head_id']='another-head'
        with self.assertRaisesRegex(ValueError,'group'):self.validate(spec=spec)
        spec=deepcopy(self.spec);spec['mode']='physical'
        with self.assertRaisesRegex(ValueError,'explicit'):self.validate(spec=spec)
        path=self.active/'wrist.json';row=json.loads(path.read_text());row['dependency_id']='another-head';row.pop('sha256');row['sha256']=hashlib.sha256(canonical(row)).hexdigest();write(path,row)
        manifest=json.loads((self.active/'manifest.json').read_text());manifest['wrist']['sha256']=row['sha256'];write(self.active/'manifest.json',manifest)
        self.spec['expected_install']['manifest_sha256']=sha(self.active/'manifest.json');self.repin()
        with self.assertRaisesRegex(ValueError,'depend'):self.validate()

    def test_fresh_clock_robot_intrinsics_and_registration_required(self):
        source=self.validate()
        for key,value in [('robot_id',self.config['robot_id']),('clock',self.config['clock']),('environment','physical')]:
            current={**self.new,key:value};write(self.current/'calibration-config.json',current)
            with self.assertRaisesRegex(ValueError,'new fake'):self.build(source=source)
        write(self.current/'calibration-config.json',self.new)
        camera=json.loads((self.current/'camera-left.json').read_text());camera['fx']+=1;write(self.current/'camera-left.json',camera)
        with self.assertRaisesRegex(ValueError,'intrinsics'):self.build(source=source)
        camera['fx']-=1;write(self.current/'camera-left.json',camera)
        self.registration['T_environment_calibration_world'][0][3]+=.001
        with self.assertRaisesRegex(ValueError,'registration'):self.build(source=source)

    def test_validated_origin_context_cannot_be_relabelled_in_memory(self):
        source=self.validate();source['origin_clock']['source_clock_id']='fabricated-origin-clock'
        with self.assertRaisesRegex(ValueError,'context changed'):self.build(source=source)
        source=self.validate();source['records']['head']['payload']['row']['parameters']['joint_offsets_rad']['left'][0]=.3
        with self.assertRaisesRegex(ValueError,'context changed'):self.build(source=source)

    def test_origin_capture_clock_is_not_relabelled(self):
        path=self.capture/'batch-head/capture/dataset.json';row=json.loads(path.read_text());row['clock']=self.new['clock'];write(path,row);self.repin()
        with self.assertRaisesRegex(ValueError,'completed current-run fake capture'):self.validate()

    def test_readonly_mesh_candidates_and_new_C_install_invalidate_diagnostic(self):
        class Workbench:
            geometry='current-geometry';geometry_id='current-workbench-geometry'
            calibration_snapshot=staticmethod(lambda row:row)
            def wrist_overlay_error(self,row):
                assert self.wrist_contract(row)['head_snapshot']['id']=='current-head'
                return None
        source=self.validate();view,rows=diagnostic_mesh_view(Workbench(),source)
        self.assertEqual(view.geometry,'current-geometry');self.assertIsNone(view.wrist_overlay_error(view.wrist_results()[0]))
        self.assertEqual({row['kind'] for row in rows},{'head','wrist'})
        self.assertTrue(all('install' not in row and 'download_url' not in row for row in rows))
        device=NS(diagnostic_calibration=diagnostic_metadata(source),diagnostic_mesh_candidates=rows)
        invalidated=[];app=NS(invalidate_collection_preparation=lambda value:invalidated.append(value))
        with patch.object(CollectionPreparation,'_run',lambda self:None):
            prep=CollectionPreparation(contract={},output=self.current,base=self.base,device=device,application=app,ports={},directories={},
                control_url='http://127.0.0.1:1',token_file='unused',deadline_wall_ns=time.time_ns()+60_000_000_000,
                diagnostic_source=source,installed_snapshot=lambda:diagnostic_snapshot(source))
        prep.notify(diagnostic_snapshot(source));self.assertFalse(prep.diagnostic_invalidated)
        prep.notify(dict(verified=True,head_id='new-head',wrist_id='new-wrist'))
        self.assertTrue(prep.diagnostic_invalidated);self.assertEqual(prep.snapshot()['status'],'failed')
        self.assertEqual(device.diagnostic_mesh_candidates,[]);self.assertTrue(device.diagnostic_calibration['invalidated'])
        self.assertEqual(invalidated,[device])
        prep._status('ready');self.assertEqual(prep.snapshot()['status'],'failed')


if __name__=='__main__':unittest.main()
