"""CPU source identity, unchanged FK/dynamics and measured geometric-gap audit."""
import argparse,json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from arx_env.joint_head_isaac_fit import Model
from arx_env.urdf_mesh import origin
from arx_env.joint_head_workbench_geometry import stl_vertices
from robot_sim.asset_cfg.robots.ac_one_finger_contact import ac_one_finger_contact_spec
from ac_one_baseline import sha


def validate(candidate):
    candidate=Path(candidate).resolve();m=json.loads((candidate/'manifest.json').read_text());root=ET.parse(candidate/'robot.urdf').getroot();native=ET.parse(candidate/'sources/ac_one.urdf').getroot()
    assert sha(candidate/'robot.urdf')==m['urdf_sha256']
    for name,digest in m['sources'].items():assert (candidate/name).resolve().is_relative_to(candidate) and sha(candidate/name)==digest
    for part in m['parts']:assert (candidate/part['path']).resolve().is_relative_to(candidate) and sha(candidate/part['path'])==part['sha256']
    for old in native.findall('joint'):
        new=root.find(f"joint[@name='{old.get('name')}']")
        for tag in ('origin','axis','parent','child'):assert old.find(tag).attrib==new.find(tag).attrib
    for old in native.findall('link'):
        new=root.find(f"link[@name='{old.get('name')}']")
        for tag in ('origin','mass','inertia'):assert old.find('inertial/'+tag).attrib==new.find('inertial/'+tag).attrib
    names=[f'left_joint{i}' for i in range(1,7)]+[f'right_joint1{i}' for i in range(1,7)]
    a=Model(candidate/'robot.urdf',names);b=Model(candidate/'sources/ac_one.urdf',names);worst=0.
    for q in np.vstack([np.zeros(12),np.random.default_rng(3).uniform(-1,1,(100,12))]):
        fa,fb=a.frames(q),b.frames(q)
        for name in fb:
            error=float(np.abs(fa[name]-fb[name]).max());worst=max(worst,error);assert error<1e-12
    parts={p['name']:p for p in m['parts']}
    for side in ('left','right'):assert parts[side+'_handle']['triangles']==4810
    qmax=m['gripper']['symmetric_joint_upper_m'];metal=[];rubber=[]
    for pad in ac_one_finger_contact_spec().pads[:2]:
        j=root.find(f"joint[@name='{pad.link_name.replace('link','joint')}']");assert abs(float(j.find('limit').get('upper'))-qmax)<1e-15
        sign=1 if pad.link_name=='left_link7' else -1
        y=origin(j.find('origin'))[1,3]+sign*qmax
        metal.append(y+sign*min(sign*p[1] for p in pad.metal_surface_polygon_m))
        points=stl_vertices(candidate/parts[pad.link_name+'_rubber']['path']);rubber.append(y+sign*float((sign*points[:,1]).min()))
    assert abs(metal[0]-metal[1]-.083)<1e-12
    assert abs(rubber[0]-rubber[1]-.080)<4e-9
    for camera in ('top','left_wrist','right_wrist'):assert not parts[camera+'_camera_housing']['collision']
    base=stl_vertices(candidate/parts['chassis']['path']);original=stl_vertices(candidate/'sources/base_link.STL')
    assert np.array_equal(base[:len(original)],original), 'original chassis/mast/seat triangles must be preserved'
    assert base[:,2].max()>.53 and base[:,2].max()<.54
    assert all('serial' not in json.dumps(json.loads((candidate/name).read_text())).lower() for name in m['sources'] if name.endswith('.json'))
    print(json.dumps(dict(status='PASS',poses=101,max_fk_error=worst,original_inertials_unchanged=True,metal_gap_mm=83,rubber_gap_mm=80,qmax_m=qmax,original_chassis_preserved=True,source_hashes=True,hardware_validated=False)))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('candidate',type=Path);validate(p.parse_args().candidate)
