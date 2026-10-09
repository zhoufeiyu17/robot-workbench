"""Freeze an all-AC-One baseline with separate removable handles; CPU only."""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from arx_env.urdf_mesh import origin,transform
from arx_env.joint_head_workbench_geometry import box_triangles
from robot_sim.asset_cfg.robots.ac_one_finger_contact import ac_one_finger_contact_spec,_pad_prism_mesh
from calibrated_urdf import write_stl,set_origin
from model_baseline import source_bytes,handle_vertices,sha


def triangles(data):
    dtype=np.dtype([('n','<f4',(3,)),('v','<f4',(3,3)),('a','<u2')])
    n=int.from_bytes(data[80:84],'little')
    if len(data)!=84+50*n:raise ValueError('invalid STL')
    return np.frombuffer(data,dtype=dtype,offset=84)['v'].astype(float).reshape(-1,3)


def build(native_urdf,rig_path,lfs_root,chassis,output):
    native_urdf,output=Path(native_urdf).resolve(),Path(output).resolve()
    if sha(native_urdf)!='779204e9f4f7ed81deacaf9d81414978d110786debbaaba50119112ebe6068a4':raise ValueError('original AC-One identity mismatch')
    root=ET.parse(native_urdf).getroot();root.set('name','ac_one_baseline_v1')
    links={x.get('name'):x for x in root.findall('link')};joints={x.get('name'):x for x in root.findall('joint')}
    output.mkdir(parents=True,exist_ok=False);(output/'meshes').mkdir();(output/'sources').mkdir()
    sources={};parts=[]
    def pin(name,data):
        relative='sources/'+name;(output/relative).write_bytes(data);sources[relative]=hashlib.sha256(data).hexdigest()
    pin('ac_one.urdf',native_urdf.read_bytes());pin('chassis_preserved_seat.stl',Path(chassis).read_bytes())
    rig=json.loads(Path(rig_path).read_text())
    # Baseline freezes only nominal installation, not old camera serials/intrinsics.
    nominal={k:rig[k] for k in ('id','units','transform_convention','mast','camera_proxy_size_m','optical_origin_note')}
    nominal['cameras']=[{k:c[k] for k in ('id','parent','xyz','rpy')} for c in rig['cameras']]
    pin('d405_nominal_installation.json',(json.dumps(nominal,indent=2)+'\n').encode())
    palette={'silver':(.57,.59,.60,1),'graphite':(.012,.015,.013,1),'rubber':(.012,.014,.013,1)}
    for name,rgba in palette.items():
        material=ET.SubElement(root,'material',name='AC1_'+name);ET.SubElement(material,'color',rgba=' '.join(map(str,rgba)))
    def attach(name,link,v,f,color='silver',collision=True):
        relative='meshes/'+name+'.stl';write_stl(output/relative,v,f)
        for kind in (('visual','collision') if collision else ('visual',)):
            shape=ET.SubElement(links[link],kind,name=name);ET.SubElement(ET.SubElement(shape,'geometry'),'mesh',filename=relative,scale='1 1 1')
            if kind=='visual':ET.SubElement(shape,'material',name='AC1_'+color)
        parts.append(dict(name=name,link=link,path=relative,sha256=sha(output/relative),triangles=len(f),collision=collision))
    for name,link in list(links.items()):
        visual=link.find('visual');mesh=visual.find('geometry/mesh');original=native_urdf.parent.parent/'meshes'/Path(mesh.get('filename')).name
        data=source_bytes(original,lfs_root);pin(name+'.STL',data)
        t=origin(visual.find('origin'));scale=np.fromstring(mesh.get('scale','1 1 1'),sep=' ')
        if not np.allclose(t,np.eye(4)) or not np.array_equal(scale,[1,1,1]):raise ValueError('unreviewed source visual transform')
        for child in list(link):
            if child.tag in ('visual','collision'):link.remove(child)
        v=triangles(data)
        if name=='base_link':v=triangles(Path(chassis).read_bytes())
        if name in ('left_link6','right_link16'):
            handle=handle_vertices(data)
            # Exact triangle records/order of the source-derived no-handle file.
            nohandle=original.with_name(original.stem+'_no_handle.STL')
            palmdata=source_bytes(nohandle,lfs_root);palm=triangles(palmdata)
            allfaces={tuple(row) for row in v.reshape(-1,9)}
            assert len(palm)//3==18166 and len(handle)//3==4810
            assert {tuple(row) for row in palm.reshape(-1,9)}|{tuple(row) for row in handle.reshape(-1,9)}==allfaces
            attach('left_handle' if name=='left_link6' else 'right_handle',name,handle,np.arange(len(handle)).reshape(-1,3),'graphite')
            v=palm
        attach('chassis' if name=='base_link' else name,name,v,np.arange(len(v)).reshape(-1,3),'graphite' if name in ('base_link','left_link1','right_link11','left_link5','right_link15','left_link6','right_link16') else 'silver')
    contact=ac_one_finger_contact_spec();gap_offsets={}
    for pad in contact.pads:
        i=int(pad.link_name.split('link')[1]);joint=joints[pad.link_name.replace('link','joint')];joint.find('limit').set('upper','.0415')
        vertices,counts,indices=_pad_prism_mesh(pad,contact.rubber_thickness_m)
        faces=[];offset=0
        for count in counts:
            polygon=indices[offset:offset+count];offset+=count
            faces.extend((polygon[0],polygon[j],polygon[j+1]) for j in range(1,count-1))
        attach(pad.link_name+'_rubber',pad.link_name,np.array(vertices),np.array(faces),'rubber')
        y=origin(joint.find('origin'))[1,3]
        sign=1 if i in (7,17) else -1
        gap_offsets[pad.link_name]=y+sign*min(sign*p[1] for p in pad.metal_surface_polygon_m)
    zero_gap=gap_offsets['left_link7']-gap_offsets['left_link8']
    if abs(zero_gap-(gap_offsets['right_link17']-gap_offsets['right_link18']))>1e-12:raise ValueError('asymmetric gap relation')
    qmax=(.083-zero_gap)/2
    for name in gap_offsets:joints[name.replace('link','joint')].find('limit').set('upper',format(qmax,'.17g'))
    for camera in nominal['cameras']:
        name=camera['id'];link=name+'_camera_link';optical=name+'_camera_rgb_optical_frame'
        links[link]=ET.SubElement(root,'link',name=link);links[optical]=ET.SubElement(root,'link',name=optical)
        for child,parent,t in [(link,camera['parent'],transform(camera['xyz'],camera['rpy'])),(optical,link,transform((.0125,0,0),(-np.pi/2,0,-np.pi/2)))]:
            joint=ET.SubElement(root,'joint',name=child+'_joint',type='fixed');ET.SubElement(joint,'parent',link=parent);ET.SubElement(joint,'child',link=child);set_origin(joint,t)
        v=box_triangles(nominal['camera_proxy_size_m'],np.eye(4));attach(name+'_camera_housing',link,v,np.arange(len(v)).reshape(-1,3),collision=False)
    ET.indent(root);ET.ElementTree(root).write(output/'robot.urdf',encoding='utf-8',xml_declaration=True)
    manifest=dict(schema='robot_model_baseline.v1',id='ac_one_baseline_v1',hardware_validated=False,geometry_only=False,runtime_allowed=False,
        runtime_status='native inertials preserved; modified mast/pads retain existing negligible-mass assumption; new composed asset needs USD/collision smoke validation',
        urdf_sha256=sha(output/'robot.urdf'),sources=sources,parts=parts,markers_rendered=False,
        arm_model='original AC-One meshes, joint origins/axes/limits and inertials; finger upper computed from exact 83 mm metal gap',
        gripper=dict(nominal_max_metal_gap_m=.083,geometry_max_metal_gap_m=.083,geometry_max_rubber_gap_m=.080,
                     symmetric_joint_upper_m=qmax,minimum_metal_gap_at_original_q_zero_m=zero_gap,metal_gap_zero_offset_m=zero_gap,metal_gap_formula='q_positive + q_negative + metal_gap_zero_offset_m',
                     rubber_thickness_per_side_m=.0015,rubber_gap_formula='metal_gap_m - 0.003',sdk_mapping_included=False,
                     contact_configuration_id=contact.configuration_id,contact_configuration_digest=contact.digest(),nominal_gap_tolerance_m=.00001),
        materials=dict(palette=palette,pbr=dict(silver=[.82,.29],graphite=[.15,.42],rubber=[0.,.62]),assignment='explicit per-part AC-One palette'),
        dynamics='all original AC-One inertials untouched; no incompatible X5/finger split; added pads/camera shells/mast extension no remeasured mass',
        cameras='nominal AC-One installation only, visual-only shells; no device serials/intrinsics/zero-offsets/physical optical calibration applied',
        handles='source component separated without changing vertices; removable by deleting named visual/collision only')
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2,allow_nan=False)+'\n');return manifest


if __name__=='__main__':
    p=argparse.ArgumentParser()
    for key in ('native-urdf','rig-path','lfs-root','chassis','output'):p.add_argument('--'+key,type=Path,required=True)
    a=p.parse_args();r=build(a.native_urdf,a.rig_path,a.lfs_root,a.chassis,a.output);print(json.dumps({'id':r['id'],'parts':len(r['parts']),'gripper':r['gripper']}))
