"""CPU contract fixtures, including real native identity and SupportBounds consumers.

A1_NATIVE_SUPPORT_SOURCE optionally names the readonly coffee_make source copy.
CUDA model allocation is a test double; no geometric/GPU qualification is claimed.
"""
from copy import deepcopy
import importlib
import json
import os
from pathlib import Path
import shutil
import sys
from tempfile import TemporaryDirectory
from types import ModuleType,SimpleNamespace as NS
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET
import numpy as np

import a1_asset_derivation as derivation
from a1_asset_validation import validate_assets,ROBOT
from gpu_acceptance import SupportBounds,same_support_source
from curobo_planning import robot_config


def fixture(root):
    source=root/'source'/'robot.urdf';source.parent.mkdir();(source.parent/'meshes').mkdir()
    tree=ET.Element('robot',name='cpu-contract-fixture')
    names=['base_link',*[f'{side}_link{i+offset}' for side,offset in (('left',0),('right',10)) for i in range(1,9)],
        *[prefix+'_camera_link' for prefix in ('top','left_wrist','right_wrist')],
        *[name.removesuffix('_joint') for name in derivation.OPTICAL]]
    meshes={}
    for name in names:
        link=ET.SubElement(tree,'link',name=name)
        if name in derivation.LINKS:
            mesh=source.parent/'meshes'/(name+'.stl');mesh.write_bytes(('CPU identity fixture '+name).encode())
            visual=ET.SubElement(link,'visual');ET.SubElement(visual,'origin',xyz='0 0 0',rpy='0 0 0')
            ET.SubElement(ET.SubElement(visual,'geometry'),'mesh',filename='meshes/'+mesh.name,scale='1 1 1')
            meshes[name]=derivation.sha(mesh)
    for side,offset in (('left',0),('right',10)):
        for index in range(1,9):
            name=f'{side}_joint{index+offset}';kind='revolute' if index<=6 else 'prismatic'
            joint=ET.SubElement(tree,'joint',name=name,type=kind)
            parent='base_link' if index==1 else f'{side}_link{offset+(index-1 if index<=6 else 6)}'
            ET.SubElement(joint,'parent',link=parent);ET.SubElement(joint,'child',link=f'{side}_link{index+offset}')
            ET.SubElement(joint,'origin',xyz='0 0 0',rpy='0 0 0');ET.SubElement(joint,'axis',xyz='0 0 1')
            ET.SubElement(joint,'limit',lower='-2' if index<=6 else '0',upper='2' if index<=6 else '.05',velocity='2',effort='10')
    for name in derivation.OPTICAL:
        child=name.removesuffix('_joint');parent=child.removesuffix('_rgb_optical_frame')+'_link'
        joint=ET.SubElement(tree,'joint',name=name,type='fixed')
        ET.SubElement(joint,'parent',link=parent);ET.SubElement(joint,'child',link=child)
        ET.SubElement(joint,'origin',xyz='0 0 0',rpy='0 0 0')
    ET.ElementTree(tree).write(source)
    policy=dict(asset_geometry_id=derivation.GEOMETRY_ID,max_interference_m=.0012,
        separation_direction_link2=[0.,0.,1.],front_support_x_min_m=-.1,
        claw_support_component_seed_faces={'link2':37470,'link5':14791},authorization='CPU test policy fixture')
    source.with_name('a1_support_contact.json').write_bytes(derivation.encoded(dict(schema='a1_local_support.v1',
        urdf_sha256=derivation.sha(source),policy=policy,mesh_sha256=meshes)))
    native=root/'native';algorithm_root=native/'source/robot_sim/robot_sim/tasks/manager_based/coffee_make';algorithm_root.mkdir(parents=True)
    actual=os.environ.get('A1_NATIVE_SUPPORT_SOURCE')
    for name in derivation.ALGORITHMS:
        (algorithm_root/name).write_bytes((Path(actual)/name).read_bytes() if actual else ('# CPU algorithm '+name).encode())
    identity=derivation.support_identity(source,native)
    positions={joint.get('name'):0. for joint in tree.findall('joint') if joint.get('type')!='fixed'}
    _,kinematics=robot_config(source,positions,positions)
    domain={name:[value['lower'],value['upper']] for name,value in kinematics['joints'].items()}
    certs=root/'certificates';certs.mkdir()
    for name in derivation.CERTIFICATES:
        data=dict(urdf_sha256=identity['urdf_sha256'],source_parts_sha256='a'*64,unchanged_threshold=.0012,
            checks=[dict(reason='continuous_interval_separation',joint_domain=domain,minimum_separation_lower_bound_m=.001)])
        if name in ('a1_support_domain.json','a1_support_near.json'):data['support_identity']=identity
        if name=='a1_support_near.json':data.update(checks=[],passed=False,complete_domain_certified=False)
        (certs/name).write_bytes(derivation.encoded(data))
    target=root/'target'/'robot.urdf';shutil.copytree(source.parent,target.parent)
    new=ET.parse(target)
    for joint in new.getroot().findall('joint'):
        if joint.get('name') in derivation.OPTICAL:joint.find('origin').set('xyz','.001 .002 .003')
    new.write(target)
    return source,target,native,certs,positions


def publish(source,target,native,certs):
    sidecar,files,proof=derivation.derive(source,target,target,native,certs)
    target.with_name('a1_support_contact.json').write_bytes(sidecar)
    directory=target.parent/'derived-certificates';directory.mkdir()
    for name,data in files.items():(directory/name).write_bytes(data)
    target.with_name(derivation.PROOF).write_bytes(derivation.encoded(proof))
    return proof


class AssetDerivationTest(unittest.TestCase):
    def test_cup_reference_rebinds_only_urdf_and_preserves_accepted_pose(self):
        import motiongen_targets as targets
        with TemporaryDirectory() as temp:
            source,target,native,certs,_=fixture(Path(temp));proof=publish(source,target,native,certs)
            accepted={**deepcopy(targets.REFERENCE),'urdf_sha256':derivation.sha(source)}
            before=deepcopy(accepted)
            ref=derivation.derive_cup_reference(accepted,target,certificate_root=certs)
            self.assertEqual(accepted,before)
            self.assertEqual(ref['urdf_sha256'],derivation.sha(target))
            evidence=ref.pop('calibration_derivation')
            self.assertEqual(evidence['proof_sha256'],derivation.sha(target.with_name(derivation.PROOF)))
            self.assertEqual(evidence['source_reference_sha256'],derivation.value_sha(accepted))
            self.assertEqual(evidence['mechanical_identity_sha256'],proof['mechanical_identity_sha256'])
            ref['urdf_sha256']=accepted['urdf_sha256'];self.assertEqual(ref,accepted)
            # The archived optical proof format remains verifiable from its
            # original native source entry point; no historical artifact rewrite.
            source.with_name('acone_no_handle_d405.urdf').write_bytes(source.read_bytes())
            proof.pop('source_urdf_path');target.with_name(derivation.PROOF).write_bytes(derivation.encoded(proof))
            self.assertEqual(derivation.derive_cup_reference(accepted,target,certificate_root=certs)['urdf_sha256'],derivation.sha(target))

    def test_actual_task_reference_guard_still_rejects_cup_native_and_envelope_changes(self):
        import motiongen_targets as targets
        with TemporaryDirectory() as temp:
            source,target,native,certs,_=fixture(Path(temp));publish(source,target,native,certs)
            accepted={**deepcopy(targets.REFERENCE),'urdf_sha256':derivation.sha(source)}
            planner=NS(cup=NS(profile_sha256=accepted['cup_profile_sha256']),
                identity={key:accepted[key] for key in ('native_geometry_sha256','envelope_sha256')})
            data=dict(urdf_sha256=derivation.sha(target));original=derivation.derive_cup_reference
            def checked(reference,urdf):return original(reference,urdf,certificate_root=certs)
            with patch.object(targets,'REFERENCE',accepted),patch.object(derivation,'derive_cup_reference',side_effect=checked):
                ref,identity=targets.checked_reference(planner,data,urdf_path=target)
                self.assertEqual(identity['urdf_sha256'],derivation.sha(target));self.assertEqual(ref['source'],accepted['source'])
                self.assertIn('calibration_derivation',ref)
                with self.assertRaisesRegex(ValueError,'does not match'):targets.checked_reference(planner,data)
                for field in ('cup_profile_sha256','native_geometry_sha256','envelope_sha256'):
                    changed=deepcopy(planner)
                    if field=='cup_profile_sha256':changed.cup.profile_sha256='f'*64
                    else:changed.identity[field]='f'*64
                    with self.subTest(field=field),self.assertRaisesRegex(ValueError,'does not match'):
                        targets.checked_reference(changed,data,urdf_path=target)
                # A caller cannot ask the unchanged strict guard to accept the
                # old reference simply because an optical proof exists nearby.
                with self.assertRaisesRegex(ValueError,'does not match'):
                    targets.checked_reference(planner,data,reference=accepted,urdf_path=target)

    def test_cup_reference_rejects_changed_mechanics_or_damaged_proof(self):
        from motiongen_targets import REFERENCE
        for change in ('joint','mesh','finger','missing_proof','proof','certificate','sidecar','source'):
            with self.subTest(change=change),TemporaryDirectory() as temp:
                source,target,native,certs,_=fixture(Path(temp));publish(source,target,native,certs)
                accepted={**deepcopy(REFERENCE),'urdf_sha256':derivation.sha(source)}
                if change in ('joint','finger'):
                    tree=ET.parse(target)
                    if change=='joint':tree.find("joint[@name='left_joint2']/origin").set('xyz','.001 0 0')
                    else:
                        visual=ET.SubElement(tree.find("link[@name='left_link7']"),'visual')
                        ET.SubElement(ET.SubElement(visual,'geometry'),'box',size='.1 .1 .1')
                    tree.write(target)
                elif change=='mesh':(target.parent/'meshes/left_link2.stl').write_bytes(b'changed support')
                elif change=='missing_proof':target.with_name(derivation.PROOF).unlink()
                elif change=='proof':
                    path=target.with_name(derivation.PROOF);value=json.loads(path.read_text());value['mechanical_identity_sha256']='f'*64
                    path.write_bytes(derivation.encoded(value))
                elif change=='certificate':
                    path=target.parent/'derived-certificates'/derivation.CERTIFICATES[0];path.write_bytes(path.read_bytes()+b'\n')
                elif change=='sidecar':
                    path=target.with_name('a1_support_contact.json');value=json.loads(path.read_text());value['policy']['max_interference_m']=.002
                    path.write_bytes(derivation.encoded(value))
                else:source.write_bytes(source.read_bytes()+b'\n')
                with self.assertRaises(ValueError):derivation.derive_cup_reference(accepted,target,certificate_root=certs)

    def test_real_asset_prepare_derives_proofs_and_rejects_mechanical_change_before_output(self):
        from a1_isaac_assets import prepare
        from model_adapt import seal
        with TemporaryDirectory() as temp:
            root=Path(temp);source,bundle_urdf,native,certs,_=fixture(root)
            base=root/'base';base_urdf=base/(ROBOT+'source/acone/urdf/acone_no_handle_d405.urdf')
            shutil.copytree(source.parent,base_urdf.parent);(base_urdf.parent/'robot.urdf').rename(base_urdf)
            rig=dict(camera_proxy_size_m=[.025,.045,.045],device_calibrations={},cameras=[dict(parent=parent,device_calibration_ref=device)
                for parent,device in [('base_link','base_0'),('left_link6','left_wrist_0'),('right_link16','right_wrist_0')]])
            (base/(ROBOT+'d405_rig.json')).write_bytes(derivation.encoded(rig))
            cup=base/'objects/manual-cup/collision.usd';cup.parent.mkdir(parents=True);cup.write_bytes(b'accepted manual cup box')
            k=dict(serial='cpu',width=4,height=3,fx=100.,fy=100.,cx=2.,cy=1.,coefficients=[0.]*5,distortion_model='inverse_brown_conrady')
            profile=dict(profile_id='cpu-a1-derived',model_binding=dict(geometry_id=derivation.GEOMETRY_ID,urdf_sha256=derivation.sha(bundle_urdf)),
                physical_calibration=dict(T_world_head_optical=np.eye(4).tolist(),
                    T_link6_wrist_optical={side:np.eye(4).tolist() for side in ('left','right')},intrinsics={side:k for side in ('head','left','right')}))
            (bundle_urdf.parent/'profile.json').write_bytes(derivation.encoded(profile));seal(bundle_urdf.parent)
            before={path:path.read_bytes() for path in base.rglob('*') if path.is_file()};original_derive=derivation.derive
            def with_fixture_certificates(*args):return original_derive(*args,certificate_root=certs)
            with patch('a1_asset_derivation.derive',side_effect=with_fixture_certificates):
                binding=prepare(bundle_urdf.parent,base,root/'output',native_code=native)
                target=Path(binding['urdf'])
                self.assertEqual(derivation.sha(target.with_name('a1_support_contact.json')),binding['support_derivation']['sidecar_sha256'])
                self.assertEqual(len(list((target.parent/'derived-certificates').glob('*.json'))),6)
                # The production base is a copied overlay with a manual cup.
                Path(binding['usd']).write_bytes(b'CPU converted USD fixture')
                copied=root/'copied-base';shutil.copytree(root/'output',copied,symlinks=True)
                copied_before={p:p.read_bytes() for p in copied.rglob('*') if p.is_file()}
                tree=ET.parse(bundle_urdf);tree.find(f"joint[@name='{derivation.OPTICAL[0]}']/origin").set('xyz','.004 .005 .006');tree.write(bundle_urdf)
                profile['model_binding']['urdf_sha256']=derivation.sha(bundle_urdf)
                (bundle_urdf.parent/'profile.json').write_bytes(derivation.encoded(profile))
                (bundle_urdf.parent/'SHA256SUMS.json').unlink();seal(bundle_urdf.parent)
                rebound=prepare(bundle_urdf.parent,copied,root/'rebound',native_code=native)
                latest=Path(rebound['urdf']);proof=json.loads(latest.with_name(derivation.PROOF).read_bytes())
                self.assertEqual(proof['source_urdf_sha256'],derivation.sha(base_urdf))
                self.assertEqual(Path(proof['source_urdf_path']),base_urdf.resolve())
                self.assertNotEqual(proof['derived_urdf_sha256'],derivation.sha(target))
                self.assertEqual((root/'rebound'/cup.relative_to(base)).read_bytes(),cup.read_bytes())
                self.assertEqual(copied_before,{p:p.read_bytes() for p in copied.rglob('*') if p.is_file()})
                reference=derivation.derive_cup_reference({'urdf_sha256':derivation.sha(base_urdf)},latest,certificate_root=certs)
                self.assertEqual(reference['urdf_sha256'],derivation.sha(latest))
                self.assertEqual(len(list((latest.parent/'derived-certificates').glob('*.json'))),6)
                for name in derivation.CERTIFICATES:derivation.certificate_path(name,latest)
                tree=ET.parse(bundle_urdf);tree.find("joint[@name='left_joint2']/origin").set('xyz','.001 0 0');tree.write(bundle_urdf)
                profile['model_binding']['urdf_sha256']=derivation.sha(bundle_urdf)
                (bundle_urdf.parent/'profile.json').write_bytes(derivation.encoded(profile))
                (bundle_urdf.parent/'SHA256SUMS.json').unlink();seal(bundle_urdf.parent)
                for input_base in (base,copied):
                    with self.assertRaisesRegex(ValueError,'mechanical'):prepare(bundle_urdf.parent,input_base,root/'rejected',native_code=native)
                    self.assertFalse((root/'rejected').exists())
            self.assertEqual(before,{path:path.read_bytes() for path in base.rglob('*') if path.is_file()})

    def test_legacy_derived_base_rebinds_directly_to_original_reviewed_source(self):
        with TemporaryDirectory() as temp:
            root=Path(temp);source,target,native,certs,_=fixture(root)
            source=source.rename(source.with_name('acone_no_handle_d405.urdf'))
            proof=publish(source,target,native,certs);proof.pop('source_urdf_path')
            for name in derivation.CERTIFICATES:
                path=target.parent/'derived-certificates'/name;data=json.loads(path.read_bytes())
                data['calibration_derivation'].pop('source_urdf_path');path.write_bytes(derivation.encoded(data))
                proof['certificates'][name]['derived_sha256']=derivation.sha(path)
            target.with_name(derivation.PROOF).write_bytes(derivation.encoded(proof))
            _,_,new=derivation.derive(target,target,root/'next/robot.urdf',native,certs)
            self.assertEqual(new['source_urdf_path'],str(source.resolve()))
            self.assertEqual(new['source_urdf_sha256'],derivation.sha(source))

    def test_rederivation_rejects_tampering_even_with_self_consistent_local_hashes(self):
        changes=('joint','mesh','source_mesh','source_urdf','source_sidecar','source_certificate','algorithm',
            'proof','parts','source_pin','sidecar_policy','certificate_payload','certificate_parts',
            'proof_symlink','sidecar_symlink','certificate_symlink','certificate_directory_symlink','missing_proof')
        for change in changes:
            with self.subTest(change=change),TemporaryDirectory() as temp:
                root=Path(temp);source,target,native,certs,_=fixture(root);proof=publish(source,target,native,certs)
                proof_path=target.with_name(derivation.PROOF);sidecar=target.with_name('a1_support_contact.json')
                cert=target.parent/'derived-certificates'/derivation.CERTIFICATES[0]
                if change=='joint':
                    tree=ET.parse(target);tree.find("joint[@name='left_joint2']/origin").set('xyz','.001 0 0');tree.write(target)
                    proof['derived_urdf_sha256']=derivation.sha(target)
                    data=json.loads(sidecar.read_bytes());data['urdf_sha256']=derivation.sha(target)
                    sidecar.write_bytes(derivation.encoded(data));proof['derived_sidecar_sha256']=derivation.sha(sidecar)
                elif change in ('mesh','source_mesh'):
                    ((target if change=='mesh' else source).parent/'meshes/left_link2.stl').write_bytes(b'unreviewed mesh')
                elif change in ('source_urdf','source_sidecar','source_certificate','algorithm'):
                    path={'source_urdf':source,'source_sidecar':source.with_name('a1_support_contact.json'),
                        'source_certificate':certs/derivation.CERTIFICATES[0],
                        'algorithm':native/'source/robot_sim/robot_sim/tasks/manager_based/coffee_make'/derivation.ALGORITHMS[0]}[change]
                    path.write_bytes(path.read_bytes()+b'\n')
                elif change in ('proof','parts'):proof['mechanical_identity_sha256' if change=='proof' else 'source_parts_sha256']='f'*64
                elif change=='source_pin':proof['certificates'][derivation.CERTIFICATES[0]]['source_sha256']='f'*64
                elif change=='sidecar_policy':
                    data=json.loads(sidecar.read_bytes());data['policy']['max_interference_m']=.002
                    sidecar.write_bytes(derivation.encoded(data));proof['derived_sidecar_sha256']=derivation.sha(sidecar)
                elif change in ('certificate_payload','certificate_parts'):
                    data=json.loads(cert.read_bytes())
                    data['unchanged_threshold' if change=='certificate_payload' else 'source_parts_sha256']=.1 if change=='certificate_payload' else 'f'*64
                    cert.write_bytes(derivation.encoded(data));proof['certificates'][cert.name]['derived_sha256']=derivation.sha(cert)
                proof_path.write_bytes(derivation.encoded(proof))
                if change.endswith('_symlink'):
                    path={'proof_symlink':proof_path,'sidecar_symlink':sidecar,'certificate_symlink':cert,
                        'certificate_directory_symlink':cert.parent}[change]
                    external=root/'external';path.rename(external);path.symlink_to(external,target_is_directory=external.is_dir())
                elif change=='missing_proof':proof_path.unlink()
                with self.assertRaises(ValueError):derivation.derive(target,target,root/'rejected/robot.urdf',native,certs)
                self.assertFalse((root/'rejected').exists())

    def test_optical_only_proof_preserves_every_certificate_field_except_bound_identity(self):
        with TemporaryDirectory() as temp:
            source,target,native,certs,_=fixture(Path(temp));before={path:path.read_bytes() for path in source.parent.rglob('*') if path.is_file()}
            proof=publish(source,target,native,certs)
            self.assertEqual(proof['mechanical_identity_sha256'],derivation.mechanical_identity(source))
            self.assertEqual(proof['mechanical_identity_sha256'],derivation.mechanical_identity(target))
            for name in derivation.CERTIFICATES:
                old=json.loads((certs/name).read_bytes());new=json.loads(derivation.certificate_path(name,target).read_bytes())
                provenance=new.pop('calibration_derivation')
                self.assertEqual(provenance['source_certificate_sha256'],derivation.sha(certs/name))
                new['urdf_sha256']=old['urdf_sha256']
                if 'support_identity' in old:
                    for key in ('urdf_sha256','sidecar_path','sidecar_sha256'):new['support_identity'][key]=old['support_identity'][key]
                self.assertEqual(new,old)
            self.assertEqual(before,{path:path.read_bytes() for path in source.parent.rglob('*') if path.is_file()})
            self.assertEqual(derivation.certificate_path('a1_support_domain.json',source),Path(derivation.__file__).with_name('a1_support_domain.json'))

    def test_mechanical_mesh_parent_and_baseline_changes_are_rejected_before_publication(self):
        for change in ('joint','support_origin','mesh','leaf_geometry','optical_parent','optical_origin_extra','baseline_policy','baseline_urdf','baseline_certificate'):
            with self.subTest(change=change),TemporaryDirectory() as temp:
                source,target,native,certs,_=fixture(Path(temp));tree=ET.parse(target)
                if change=='mesh':(target.parent/'meshes/left_link2.stl').write_bytes(b'changed mechanical mesh')
                elif change=='baseline_policy':
                    path=source.with_name('a1_support_contact.json');data=json.loads(path.read_text());data['policy']['max_interference_m']=.002;path.write_bytes(derivation.encoded(data))
                elif change=='baseline_urdf':source.write_bytes(source.read_bytes()+b'\n')
                elif change=='baseline_certificate':
                    path=certs/'a1_fixed_domain.json';data=json.loads(path.read_text());data['urdf_sha256']='f'*64;path.write_bytes(derivation.encoded(data))
                else:
                    if change=='joint':tree.find("joint[@name='left_joint2']/origin").set('xyz','.001 0 0')
                    elif change=='support_origin':tree.find("link[@name='left_link2']/visual/origin").set('xyz','.001 0 0')
                    elif change=='leaf_geometry':ET.SubElement(tree.find("link[@name='top_camera_rgb_optical_frame']"),'visual')
                    elif change=='optical_origin_extra':tree.find("joint[@name='top_camera_rgb_optical_frame_joint']/origin").set('unreviewed','extra')
                    else:tree.find("joint[@name='top_camera_rgb_optical_frame_joint']/parent").set('link','left_link2')
                    tree.write(target)
                with self.assertRaises(ValueError):derivation.derive(source,target,target,native,certs)
                self.assertFalse(target.with_name(derivation.PROOF).exists())

    def test_derived_certificate_payload_and_sidecar_tampering_are_rejected(self):
        for name in (*derivation.CERTIFICATES,'a1_support_contact.json'):
            with self.subTest(name=name),TemporaryDirectory() as temp:
                source,target,native,certs,_=fixture(Path(temp));publish(source,target,native,certs)
                path=target.parent/name if name=='a1_support_contact.json' else target.parent/'derived-certificates'/name
                path.write_bytes(path.read_bytes()+b'\n')
                with self.assertRaises(ValueError):derivation.certificate_path(name if name in derivation.CERTIFICATES else derivation.CERTIFICATES[0],target)

    def test_asset_validation_requires_pinned_sidecar_proof_and_all_six_certificates(self):
        with TemporaryDirectory() as temp:
            root=Path(temp);source,target,native,certs,_=fixture(root)
            assets=root/'assets';urdf=assets/(ROBOT+'source/acone/urdf/a1-cal.urdf')
            shutil.copytree(target.parent,urdf.parent);(urdf.parent/'robot.urdf').rename(urdf)
            proof=publish(source,urdf,native,certs)
            usd=assets/(ROBOT+'sim/a1-cal.usd');usd.parent.mkdir(parents=True);usd.write_text('CPU USD identity fixture only')
            for name in ('acone','acone_no_handle','acone_no_handle_d405'):urdf.with_name(name+'.urdf').symlink_to(urdf.name)
            for name in ('ac_one','ac_one_no_handle','ac_one_no_handle_d405'):usd.with_name(name+'.usd').symlink_to(usd.name)
            model=dict(urdf_sha256=derivation.sha(urdf),geometry_id=derivation.GEOMETRY_ID)
            profile=dict(profile_id='cpu-derived-profile',model_binding=model)
            binding=dict(schema='a1_isaac_assets.v1',conversion_complete=True,profile_id=profile['profile_id'],model_binding=model,
                urdf=str(urdf),usd=str(usd),usd_sha256=derivation.sha(usd),support_derivation=dict(
                    proof_sha256=derivation.sha(urdf.with_name(derivation.PROOF)),sidecar_sha256=derivation.sha(urdf.with_name('a1_support_contact.json')),
                    mechanical_identity_sha256=proof['mechanical_identity_sha256']))
            (assets/'a1-binding.json').write_bytes(derivation.encoded(binding))
            (assets/(ROBOT+'d405_rig.json')).write_bytes(derivation.encoded(dict(id=profile['profile_id'],calibration_binding=model)))
            pins={str(path):derivation.sha(path) for path in assets.rglob('*') if path.is_file()}
            self.assertIn('support_derivation',validate_assets(assets,profile,pins)['asset_sha256'])
            for path in [urdf.with_name('a1_support_contact.json'),urdf.with_name(derivation.PROOF),
                         *[urdf.parent/'derived-certificates'/name for name in derivation.CERTIFICATES]]:
                missing=dict(pins);missing.pop(str(path))
                with self.subTest(path=path),self.assertRaises(ValueError):validate_assets(assets,profile,missing)

    @unittest.skipUnless(os.environ.get('A1_NATIVE_SUPPORT_SOURCE'),'set A1_NATIVE_SUPPORT_SOURCE for the real native CPU consumer')
    def test_real_native_support_identity_and_supportbounds_cpu_identity_gates(self):
        package=ModuleType('native_support_contract_fixture');package.__path__=[os.environ['A1_NATIVE_SUPPORT_SOURCE']]
        with patch.dict(sys.modules,{'native_support_contract_fixture':package}):
            actual=importlib.import_module('native_support_contract_fixture.a1_support_contact')
        with TemporaryDirectory() as temp:
            source,target,native,certs,positions=fixture(Path(temp));proof=publish(source,target,native,certs)
            identity=actual.support_identity(target)
            self.assertTrue(same_support_source(identity,proof['derived_support_identity']))
            support=NS(identity=identity,meshes={},faces={},pairs=())
            planner=NS(support_contact=support,home_state=positions,fk=NS(movable=positions))
            engine=NS(torch=None,tensor_args=NS(to_device=lambda value:value),config={'kinematics':{'lock_joints':positions}})
            cuda_modules={'curobo.types.robot':NS(RobotConfig=NS(from_dict=lambda cfg,args:NS(kinematics=cfg['kinematics']))),
                'curobo.cuda_robot_model.cuda_robot_model':NS(CudaRobotModel=lambda cfg:NS(joint_names=cfg['cspace']['joint_names']))}
            with patch.dict(sys.modules,cuda_modules):
                checked=SupportBounds(engine,planner,target)
                self.assertEqual(checked.certificate['urdf_sha256'],derivation.sha(target))
                support.identity={**identity,'sidecar_sha256':'changed'}
                with self.assertRaisesRegex(ValueError,'support domain source identity'):SupportBounds(engine,planner,target)
            path=target.with_name('a1_support_contact.json');data=json.loads(path.read_text());data['policy']['max_interference_m']=.002
            path.write_bytes(derivation.encoded(data))
            with self.assertRaisesRegex(ValueError,'unreviewed or mismatched'):actual.support_identity(target)


if __name__=='__main__':unittest.main()
