"""CPU contract fixture using the real C store; requires a sealed nominal --a1.

No images are fitted here: passing proves provenance/conversion checks, not the
accuracy of calibration, planning, or a resident PhysX execution.
"""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import tempfile

import numpy as np
from arx_env.joint_head_calibration_store import CalibrationStore
from a1_cal import canonical, read_active
from integration import _digest
from joint_space import convert_state
from model_adapt import digest, verify
from physical_profile import profile_from_calibration
from robot_workbench.calibration_profile import build_fake_profile
from test_harness import PROFILE
from contract import demo_state


def write(path,value):
    path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(canonical(value)+b'\n')


def fixture(root,a1):
    contract=json.loads((a1/'calibration-input.json').read_text());geometry=contract['geometry_id'];seal=digest(a1/'SHA256SUMS.json')
    capture=root/'calibration-workbench';capture.mkdir()
    robot='fake-cpu-profile-fixture'
    k=dict(width=640,height=480,fx=400.,fy=401.,cx=320.,cy=240.,coefficients=[0.]*5,
           distortion_model='inverse_brown_conrady',rectified=False)
    cameras={side:dict(k,serial='fixture-'+side) for side in ('head','left','right')}
    transform=np.eye(4);transform[:3,3]=[.1,.2,.9]
    flags=dict(robot_source='simulated',simulation_only=True)
    clock=dict(environment='fake',clock_domain='simulation',source_clock_id='cpu-current-plant')
    parameters=dict(joint_offsets_rad=dict(left=[.01]*6,right=[-.02]*6),camera_T_world_optical=transform.tolist(),
        asset_geometry_id=geometry,asset_seal_sha256=seal)
    wrist_contract=dict(geometry_id='current-workbench-geometry',asset_geometry_id=geometry,asset_seal_sha256=seal,
        head_snapshot={'id':'current-head'},cameras={side:cameras[side] for side in ('left','right')})
    store=CalibrationStore(root/'calibrations'/'hardware',robot,'hardware')
    head=store.register('head','current-head',{'row':dict(parameters=parameters,**flags)},'batch-head/capture',environment='hardware')
    wrist=store.register('wrist','current-wrist',{'row':dict(pair_status='complete',asset_geometry_id=geometry,
        asset_seal_sha256=seal,**flags,**{side:dict(T_link6_camera=transform.tolist()) for side in ('left','right')}),
        'contract':wrist_contract},'batch-wrist/capture',environment='hardware',dependency_id=head['id'])
    for kind in ('head','wrist'):
        data=dict(environment='hardware',runtime_environment='fake',robot_id=robot,asset_geometry_id=geometry,
            asset_seal_sha256=seal,frames=[{'id':'synthetic-contract-fixture'}],camera=cameras['head'],clock=clock,**flags)
        if kind=='wrist':data['wrist_contract']=wrist_contract
        write(capture/('batch-'+kind)/'capture'/'dataset.json',data)
        write(capture/('batch-'+kind)/'capture'/'completion.json',{'passed':True})
    store.install(head,wrist,geometry_id='current-workbench-geometry',robot_source='simulated',
        acceptance={'status':'synthetic_cpu_contract_fixture_only'})
    write(root/'camera-head.json',cameras['head'])
    config=dict(environment='fake',runtime_environment='fake',robot_id=robot,**flags,model_family='ac_one',clock=clock,
        calibration_root=str(root/'calibrations'),urdf=str(a1/'robot.urdf'),camera=str(root/'camera-head.json'),
        jaw_mapping_file=str(a1/'jaw-mapping.json'),asset_geometry_id=geometry,asset_seal_sha256=seal,
        hardware_camera_serials={side:row['serial'] for side,row in cameras.items()})
    write(root/'calibration-config.json',config)
    active=root/'calibrations'/'hardware'/'active'
    expected=dict(head_id=head['id'],wrist_id=wrist['id'],manifest_sha256=digest(active/'manifest.json'))
    registration=dict(length_unit='m',T_environment_calibration_world=np.eye(4).tolist())
    return capture,active,config,expected,registration


def check(a1):
    a1=Path(a1).resolve();verify(a1);before=digest(a1/'SHA256SUMS.json')
    with tempfile.TemporaryDirectory() as folder:
        root=Path(folder).resolve();capture,active,config,expected,registration=fixture(root,a1)
        def build(name,*,receipt=None,registration_value=None):
            return build_fake_profile(root/'calibration-config.json',capture,PROFILE,
                registration if registration_value is None else registration_value,root/name,
                expected_install=expected if receipt is None else receipt)
        def rejects(call):
            try:call()
            except (ValueError,FileNotFoundError):pass
            else:raise AssertionError('invalid calibration bridge input accepted')
        rejects(lambda:build('wrong-receipt',receipt={**expected,'manifest_sha256':'0'*64}))
        rejects(lambda:build('wrong-units',registration_value={**registration,'length_unit':'mm'}))
        saved=(active/'manifest.json').read_bytes();(active/'manifest.json').unlink()
        rejects(lambda:build('empty'));assert not (root/'empty').exists();(active/'manifest.json').write_bytes(saved)
        saved_wrist=(active/'wrist.json').read_bytes();record=json.loads(saved_wrist)
        record['dependency_id']='another-head';record.pop('sha256');record['sha256']=hashlib.sha256(canonical(record)).hexdigest()
        write(active/'wrist.json',record);manifest=json.loads(saved);manifest['wrist']['sha256']=record['sha256'];write(active/'manifest.json',manifest)
        rejects(lambda:build('wrong-pair',receipt={**expected,'manifest_sha256':digest(active/'manifest.json')}))
        (active/'wrist.json').write_bytes(saved_wrist);(active/'manifest.json').write_bytes(saved)
        path=capture/'batch-head'/'capture'/'dataset.json';data=path.read_bytes();old=json.loads(data);old.pop('runtime_environment');write(path,old)
        rejects(lambda:build('old-batch'));path.write_bytes(data)
        old=json.loads(data);old['clock']['source_clock_id']='another-plant';write(path,old)
        rejects(lambda:build('other-plant'));path.write_bytes(data)
        original_config=deepcopy(config);config['calibration_root']=str(root.parent/'physical-calibration');write(root/'calibration-config.json',config)
        rejects(lambda:build('cross-root'));write(root/'calibration-config.json',original_config)
        result=build('new-profile');assert result['simulation_only'] and not result['runtime_allowed']
        profile=json.loads((root/'new-profile/profile.json').read_text())
        binding=json.loads((root/'new-profile/acquisition-binding.json').read_text())
        planner=json.loads((root/'new-profile/planning-model.json').read_text())
        assert profile['source']['kind']=='installed_fake_calibration' and profile['source']['simulation_only'] is True
        assert profile['source']['runtime_environment']=='fake' and profile['source']['robot_source']=='simulated'
        assert binding['calibration_profile_digest']==_digest(profile)
        assert binding['active_sha256']['manifest.json']==expected['manifest_sha256']
        assert binding['head_id']=='current-head' and binding['wrist_id']=='current-wrist'
        assert planner['joint_zero_offsets_baked_into_urdf'] is False and planner['actual_plant_unchanged'] is True
        assert planner['zero_offsets_rad']==[.01]*6+[-.02]*6
        reported=demo_state();model=convert_state(reported,profile,to_reported=False)
        np.testing.assert_allclose(model['left_arm_joints'],np.array(reported['left_arm_joints'])+.01)
        np.testing.assert_allclose(model['right_arm_joints'],np.array(reported['right_arm_joints'])-.02)
        restored=convert_state(model,profile,to_reported=True)
        for side in ('left','right'):np.testing.assert_allclose(restored[side+'_arm_joints'],reported[side+'_arm_joints'],atol=1e-15)
        records=read_active(active,config['asset_geometry_id'],asset_seal=before,environment='fake')
        rejects(lambda:read_active(active,config['asset_geometry_id'],asset_seal=before))
        rejects(lambda:profile_from_calibration(profile['physical_calibration'],profile['physical_calibration']['intrinsics'],
            records['manifest'],PROFILE,np.eye(4)))
        calibrated_input={**original_config,'urdf':str(root/'new-profile/model/robot.urdf'),
                          'asset_seal_sha256':digest(root/'new-profile/model/SHA256SUMS.json')}
        write(root/'calibration-config.json',calibrated_input)
        rejects(lambda:build('double-calibrated-model'))
        write(root/'calibration-config.json',original_config)
        verify(root/'new-profile/model');assert not any(name in __import__('sys').modules for name in ('isaaclab','pxr'))
    assert digest(a1/'SHA256SUMS.json')==before;verify(a1)
    print('PASS fake installed-profile bridge: real C store, empty/wrong group rejection, current-run capture provenance, nominal URDF + offset once, explicit registration, fake never physical; CPU contract fixture only.')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--a1',type=Path,required=True)
    check(parser.parse_args().a1)
