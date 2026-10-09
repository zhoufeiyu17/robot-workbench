"""Offline candidate FK/mesh/inertia checks against frozen calibration geometry."""
import argparse
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import xml.etree.ElementTree as ET
import numpy as np
from arx_env.joint_head_isaac_fit import Model
from arx_env.joint_head_workbench_geometry import stl_vertices
from arx_env.urdf_mesh import origin
from calibrated_mesh import CalibratedMesh
from calibrated_urdf import build,sha


def validate(snapshot,native,candidate):
    mesh=CalibratedMesh(snapshot);geometry=mesh.geometry
    manifest=json.loads((candidate/'manifest.json').read_text())
    assert manifest['urdf_sha256']==sha(candidate/'robot.urdf')
    original=ET.parse(native).getroot();root=ET.parse(candidate/'robot.urdf').getroot()
    assert {e.get('name') for e in root.findall('link')}=={e.get('name') for e in original.findall('link')}
    assert {e.get('name') for e in root.findall('joint')}=={e.get('name') for e in original.findall('joint')}
    names=[f'left_joint{i}' for i in range(1,7)]+[f'right_joint1{i}' for i in range(1,7)]
    model=Model(candidate/'robot.urdf',names)
    worst=0.
    for q in np.vstack([np.zeros(12),np.random.default_rng(42).uniform(-1,1,(100,12))]):
        actual=model.frames(q);expected=geometry.frames(q)
        for side in ('left','right'):
            for i in range(1,7):
                name=f'{side}_link{i if side=="left" else 10+i}'
                difference=float(np.abs(actual[name]-expected[side][f'link{i}']).max())
                worst=max(worst,difference);assert difference<1e-12,(name,difference)
            optical=actual[side+'_wrist_camera_rgb_optical_frame']
            assert np.allclose(optical,expected[side]['link6']@mesh.wrist_cameras[side],atol=1e-12)
        assert np.allclose(actual['top_camera_rgb_optical_frame'],mesh.head_camera,atol=1e-12)
    # Absolute geometry pins: full triangles, no source traversal, no external mesh references.
    for part in manifest['parts']:
        p=(candidate/part['path']).resolve()
        assert p.is_relative_to(candidate.resolve()) and sha(p)==part['sha256']
        vertices=stl_vertices(p)
        assert len(vertices)==part['triangles']*3 and np.isfinite(vertices).all()
    assert len(manifest['parts'])==26, len(manifest['parts'])
    for node in root.findall('.//mesh'):
        assert (candidate/node.get('filename')).resolve().is_relative_to(candidate.resolve())
    for link in root.findall('link'):
        before=original.find(f"link[@name='{link.get('name')}']/inertial")
        after=link.find('inertial')
        if before is None:assert after is None;continue
        assert before.find('mass').attrib==after.find('mass').attrib
        rotated=link.get('name') in {'left_link3','left_link4','left_link5','right_link13','right_link14','right_link15'}
        expected_origin=(np.diag([1.,-1.,-1.,1.]) if rotated else np.eye(4))@origin(before.find('origin'))
        assert np.allclose(origin(after.find('origin')),expected_origin,atol=1e-12)
        def tensor(row):
            a=row.find('inertia').attrib
            mat=np.array([[float(a['ixx']),float(a['ixy']),float(a['ixz'])],
                          [float(a['ixy']),float(a['iyy']),float(a['iyz'])],
                          [float(a['ixz']),float(a['iyz']),float(a['izz'])]])
            r=origin(row.find('origin'))[:3,:3]
            return r@mat@r.T
        assert np.allclose(np.linalg.eigvalsh(tensor(before)),np.linalg.eigvalsh(tensor(after)),atol=1e-12)
    # Full nominal external shape is retained, including static camera/housings.
    for side in ('left','right'):
        parts={x['name']:x for x in manifest['parts']}
        for name,vertices,_ in geometry.side_meshes[side]:
            if name=='link6':continue
            exported=stl_vertices(candidate/parts[side+'_'+name]['path'])
            if name=='base_link':
                t=geometry.bases[side];vertices=vertices@t[:3,:3].T+t[:3,3]
            assert np.allclose(exported,vertices,atol=6e-8,rtol=0),name
    for side in ('left','right'):
        for gap in (0.,.03,.083):
            for sign,i in ((0,6),(1,7),(-1,8)):
                exported=stl_vertices(candidate/parts[f'{side}_hand_{i}']['path'])
                if sign:
                    number=i if side=='left' else 10+i
                    joint=root.find(f"joint[@name='{side}_joint{number}']")
                    exported+=origin(joint.find('origin'))[:3,3]
                    exported[:,1]+=sign*gap/2
                hand=geometry.hand
                face_mask=hand['signs'][hand['faces'][:,0]]==sign
                expected=hand['vertices'][hand['faces'][face_mask]].reshape(-1,3).copy()
                expected[:,1]+=sign*(gap-hand['nominal_gap_m'])/2
                assert np.allclose(exported,expected,atol=6e-8,rtol=0)
    assert manifest['markers_rendered'] is False and manifest['hardware_validated'] is False
    print(json.dumps(dict(status='PASS',poses=101,links_per_pose=12,max_fk_matrix_error=worst,
                          mass_preserved=True,inertia_eigenvalues_preserved=True,mesh_parts=len(manifest['parts']),hardware_validated=False)))


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--snapshot',type=Path,required=True);p.add_argument('--native-urdf',type=Path,required=True)
    p.add_argument('--candidate',type=Path)
    args=p.parse_args()
    if args.candidate:validate(args.snapshot,args.native_urdf,args.candidate)
    else:
        with TemporaryDirectory() as temporary:
            candidate=Path(temporary)/'candidate'
            build(args.snapshot,args.native_urdf,candidate)
            validate(args.snapshot,args.native_urdf,candidate)
