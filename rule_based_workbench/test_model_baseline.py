"""Read-only validation of a generated baseline; no robot/Isaac runtime."""
import argparse,json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from arx_env.joint_head_isaac_fit import Model
from arx_env.urdf_mesh import origin
from arx_env.joint_head_workbench_geometry import stl_vertices,hand_template
from model_baseline import sha


def validate(candidate):
    candidate=Path(candidate).resolve();manifest=json.loads((candidate/'manifest.json').read_text())
    assert manifest['urdf_sha256']==sha(candidate/'robot.urdf')
    for name,digest in manifest['sources'].items():
        path=(candidate/name).resolve();assert path.is_relative_to(candidate) and sha(path)==digest
    for part in manifest['parts']:
        path=(candidate/part['path']).resolve();assert path.is_relative_to(candidate) and sha(path)==part['sha256']
    tree=ET.parse(candidate/'robot.urdf').getroot();x5tree=ET.parse(candidate/'sources/x5.urdf').getroot();native=ET.parse(candidate/'sources/ac_one.urdf').getroot()
    names=[f'left_joint{i}' for i in range(1,7)]+[f'right_joint1{i}' for i in range(1,7)]
    robot=Model(candidate/'robot.urdf',names);single=Model(candidate/'sources/x5.urdf',[f'joint{i}' for i in range(1,7)])
    worst=0.
    for q in np.vstack([np.zeros(12),np.random.default_rng(7).uniform(-1,1,(100,12))]):
        actual=robot.frames(q)
        for side,index in [('left',0),('right',6)]:
            expected=single.frames(q[index:index+6]);base=np.array(manifest['T_world_arm_base'][side])
            for i in range(1,7):
                name=f'{side}_link{i if side=="left" else 10+i}';error=float(np.max(abs(actual[name]-base@expected[f'link{i}'])));worst=max(worst,error);assert error<1e-12
    for side in ['left','right']:
        n=lambda i:f'{side}_joint{i if side=="left" else 10+i}'
        assert np.allclose(origin(tree.find(f"joint[@name='{n(1)}']/origin")),origin(native.find(f"joint[@name='{n(1)}']/origin")),atol=1e-12)
        for i in range(1,6):
            link=f'{side}_link{i if side=="left" else 10+i}'
            for tag in ['mass','inertia']:
                assert tree.find(f"link[@name='{link}']/inertial/{tag}").attrib==x5tree.find(f"link[@name='link{i}']/inertial/{tag}").attrib
    parts={p['name']:p for p in manifest['parts']}
    for side in ['left','right']:
        assert parts[side+'_handle']['triangles']==4810
        for gap in [0.,.030,.083]:
            metal_planes=[];rubber_planes=[]
            for sign,i in [(1,7),(-1,8)]:
                joint=f'{side}_joint{i if side=="left" else 10+i}';o=origin(tree.find(f"joint[@name='{joint}']/origin"))[:3,3]
                assert float(tree.find(f"joint[@name='{joint}']/limit").get('upper'))==.0415
                for prefix,planes in [('hand',metal_planes),('rubber',rubber_planes)]:
                    v=stl_vertices(candidate/parts[f'{side}_{prefix}_{i}']['path']);v+=o;v[:,1]+=sign*gap/2
                    planes.append(sign*float((sign*v[:,1]).min()))
            assert abs((metal_planes[0]-metal_planes[1])-gap)<6e-8
            assert abs((rubber_planes[0]-rubber_planes[1])-(gap-.003))<6e-8
    assert manifest['markers_rendered'] is False and manifest['hardware_validated'] is False
    assert not manifest['gripper']['sdk_mapping_included']
    assert manifest['geometry_only'] and manifest['runtime_allowed'] is False
    for side in ['left','right']:
        for i in (6,7,8):
            link=f'{side}_link{i if side=="left" else 10+i}'
            assert tree.find(f"link[@name='{link}']/inertial") is None
    assert all(not row['valid'] for row in manifest['inertia_checks'])
    assert all((candidate/e.get('filename')).resolve().is_relative_to(candidate) for e in tree.findall('.//mesh'))
    print(json.dumps(dict(status='PASS',poses=101,max_fk_matrix_error=worst,metal_gaps_mm=[0,30,83],rubber_gaps_mm=[-3,27,80],handle_faces_each=4810,source_hashes=True,hardware_validated=False)))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('candidate',type=Path);validate(p.parse_args().candidate)
