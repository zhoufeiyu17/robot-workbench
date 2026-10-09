"""Native A1-cal fake-input integration; never loads SDK or contacts hardware."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

import numpy as np
from a1_cal import build, canonical, read_active
from joint_space import convert_state, freeze_reported, validate
from model_adapt import digest, verify
from physical_profile import camera_offsets
from real_contract import validate_real_plan
from test_harness import PROFILE
from test_frozen_executor import fixture


def inputs(root, a1):
    active = root/'active'; active.mkdir()
    identity = json.loads((a1/'calibration-input.json').read_text())['geometry_id']
    k = dict(width=640, height=480, fx=400., fy=401., cx=320., cy=240., coefficients=[0.]*5,
             distortion_model='inverse_brown_conrady', rectified=False)
    cameras = {s: dict(k, serial='fixture-'+s) for s in ('head','left','right')}
    T = np.eye(4); T[:3,3] = [.12,.23,.91]
    head = dict(schema='joint-head-calibration-store-v1', kind='head', id='fake-head', version='v000001',
                environment='simulation', robot_id='fake-robot', payload={'row': dict(robot_source='simulated', simulation_only=True,
                parameters=dict(joint_offsets_rad={'left':[.01]*6,'right':[-.02]*6}, camera_T_world_optical=T.tolist()))})
    wrist = dict(schema='joint-head-calibration-store-v1', kind='wrist', id='fake-wrist', version='v000002',
                 environment='simulation', robot_id='fake-robot', dependency_id=head['id'],
                 payload={'row':dict(robot_source='simulated', simulation_only=True,
                                    **{s:{'T_link6_camera':T.tolist()} for s in ('left','right')}),
                          'contract':{'cameras':{s:cameras[s] for s in ('left','right')}}})
    for row in (head['payload']['row']['parameters'], wrist['payload']['row'], wrist['payload']['contract']):
        row.update(asset_geometry_id=identity, asset_seal_sha256=digest(a1/'SHA256SUMS.json'))
    wrist['payload']['contract']['geometry_id'] = 'fixture-workbench-geometry'
    for record in (head,wrist): record['sha256'] = hashlib.sha256(canonical(record)).hexdigest()
    manifest = dict(schema='joint-head-calibration-active-v1', geometry_id='fixture-workbench-geometry', robot_id='fake-robot',
                    robot_source='simulated', environment='simulation', simulation_only=True,
                    acceptance={'status':'fixture-only'},
                    **{s:{key:record[key] for key in ('id','version','sha256')} for s,record in (('head',head),('wrist',wrist))})
    for name, value in (('head',head),('wrist',wrist),('manifest',manifest)):
        (active/(name+'.json')).write_bytes(canonical(value))
    return active, cameras['head'], identity


def main(a1=None):
    a1 = Path(a1) if a1 is not None else Path(__file__).parent/'model_assets/ac_one_A1_pending_calibration_r2'
    before = digest(a1/'SHA256SUMS.json')
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        active, k, identity = inputs(root, a1)
        output = root/'A1-cal-fake'
        result = build(a1, active, k, PROFILE, np.eye(4), output, fixture=True)
        assert result['simulation_only'] and result['runtime_allowed'] is False
        verify(output)
        profile = json.loads((output/'profile.json').read_text())
        assert profile['source']['simulation_only'] is True
        assert profile['model_binding']['geometry_id'] == identity
        assert profile['gripper']['max_opening_mm'] == 83
        original, adapted = (ET.parse(p/'robot.urdf').getroot() for p in (a1,output))
        for old, new in zip(original.findall('joint'), adapted.findall('joint')):
            if 'rgb_optical_frame' not in old.get('name'):
                assert ET.tostring(old) == ET.tostring(new)
        for old, new in zip(original.findall('link'), adapted.findall('link')):
            assert ET.tostring(old) == ET.tostring(new)
        for mesh in (a1/'meshes').iterdir(): assert digest(mesh) == digest(output/'meshes'/mesh.name)
        source, plan = fixture()
        source.update(schema_version='physical_collection_source.v1', calibration_profile=profile)
        plan['native_actions'] = [[0.]*14 for _ in plan['ticks']]
        old = deepcopy(plan['ticks'][0]['target'])
        freeze_reported(plan, source); validate(plan, source)
        recovered = convert_state(plan['ticks'][0]['target'], profile, to_reported=False)
        for side in ('left','right'):
            np.testing.assert_allclose(recovered[side+'_arm_joints'], old[side+'_arm_joints'], atol=1e-14)
        parents = {s:dict(parent=s, T_parent_reference=np.eye(4).tolist()) for s in ('head','left','right')}
        assert set(camera_offsets(profile, parents)) == {'base_0','left_wrist_0','right_wrist_0'}
        source.pop('gpu', None); source.pop('instance_id', None)
        source.update(lane='physical-real', adapter='foundationpose', evidence_source='physical_rgbd',
                      robot_id='fake-robot', camera_serials=dict(head='h',left='l',right='r'))
        binding = dict(calibration_accepted=True, evidence_source='physical_rgbd', robot_id='fake-robot',
                       camera_serials=source['camera_serials'])
        try: validate_real_plan(plan, source, binding)
        except ValueError as error: assert 'fixture calibration' in str(error), str(error)
        else: raise AssertionError('fixture reached physical authorization')
        for action in (lambda: read_active(active, identity), lambda: read_active(active, 'old-X5', fixture=True),
                       lambda: freeze_reported(plan,source),
                       lambda: build(a1,active,k,PROFILE,np.eye(4),output,fixture=True)):
            try: action()
            except (ValueError, FileExistsError): pass
            else: raise AssertionError('invalid input accepted')
        head = json.loads((active/'head.json').read_text()); head['payload']['row']['parameters']['joint_offsets_rad']['left'][0] = 1.
        (active/'head.json').write_bytes(canonical(head))
        try: read_active(active,identity,fixture=True)
        except ValueError: pass
        else: raise AssertionError('modified calibration accepted')
    assert before == digest(a1/'SHA256SUMS.json')
    print('A1-cal: sealed native geometry, optical-only update, profile, model/reported roundtrip, provenance rejection PASS')


if __name__ == '__main__':
    import argparse
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--a1',type=Path)
    main(parser.parse_args().a1)
