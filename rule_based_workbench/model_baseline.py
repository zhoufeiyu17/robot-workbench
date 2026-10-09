"""Offline X5-arm / AC-One-installation baseline builder; never applies calibration."""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from arx_env.urdf_mesh import origin,transform
from arx_env.joint_head_workbench_geometry import stl_vertices,hand_template,box_triangles
from calibrated_gripper import split_hand_meshes
from calibrated_urdf import write_stl,set_origin


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def source_bytes(path,lfs_root):
    data=Path(path).read_bytes()
    if data.startswith(b'version https://git-lfs.github.com/spec/v1'):
        fields=dict(line.split(' ',1) for line in data.decode().splitlines())
        digest=fields['oid'].removeprefix('sha256:')
        if len(digest)!=64 or any(c not in '0123456789abcdef' for c in digest):raise ValueError('invalid LFS digest')
        data=(Path(lfs_root)/digest[:2]/digest[2:4]/digest).read_bytes()
        if hashlib.sha256(data).hexdigest()!=digest or len(data)!=int(fields['size']):raise ValueError('LFS content mismatch')
    return data


def handle_vertices(data):
    n=struct.unpack_from('<I',data,80)[0]
    if n!=22976 or len(data)!=84+50*n:raise ValueError('unexpected original AC-One palm mesh')
    dtype=np.dtype([('normal','<f4',(3,)),('v','<f4',(3,3)),('attr','<u2')])
    vertices=np.frombuffer(data,dtype=dtype,offset=84)['v'].astype(float)
    unique,inverse=np.unique(np.round(vertices.reshape(-1,3),7),axis=0,return_inverse=True)
    faces=inverse.reshape(-1,3)
    edges=np.concatenate([faces[:,[0,1]],faces[:,[1,2]],faces[:,[2,0]]])
    graph=coo_matrix((np.ones(len(edges)),(edges[:,0],edges[:,1])),shape=(len(unique),len(unique)))
    _,labels=connected_components(graph,directed=False)
    found=[]
    for group in np.unique(labels):
        selection=labels[faces[:,0]]==group
        if selection.sum()==4810 and vertices[selection].min(axis=(0,1))[2]<-.14:found.append(vertices[selection])
    if len(found)!=1:raise ValueError('expected exactly one original handle component')
    return found[0].reshape(-1,3)


def pad_mesh(hand,sign,origin_xyz):
    """Extrude exactly the closest planar X5 inner metal surface toward the cup."""
    vertices,faces=hand['vertices'],hand['faces']
    triangles=vertices[faces[hand['signs'][faces[:,0]]==sign]]
    plane=float((sign*triangles[:,:,1]).min())
    triangles=triangles[np.all(abs(sign*triangles[:,:,1]-plane)<1e-6,axis=1)]
    if len(triangles)!=2:raise ValueError('expected reviewed two-triangle X5 fingertip plane')
    points,inverse=np.unique(np.round(triangles.reshape(-1,3),8),axis=0,return_inverse=True)
    faces=inverse.reshape(-1,3)
    # Orient the metal-backed face outward, away from the extruded rubber volume.
    for row in faces:
        if np.cross(points[row[1]]-points[row[0]],points[row[2]]-points[row[0]])[1]*sign<0:row[1],row[2]=row[2],row[1]
    count=len(points);outer=points.copy();outer[:,1]-=sign*.0015
    allpoints=np.vstack([points,outer]);result=[*faces.tolist(),*(faces[:,::-1]+count).tolist()]
    edges={}
    for a,b,c in faces:
        for edge in ((a,b),(b,c),(c,a)):edges.setdefault(tuple(sorted(edge)),[]).append(edge)
    for rows in edges.values():
        if len(rows)==1:
            a,b=rows[0];result.extend([[b,a,a+count],[b,a+count,b+count]])
    allpoints-=origin_xyz;allpoints[:,1]-=sign*hand['nominal_gap_m']/2
    return allpoints,np.asarray(result,int),dict(plane_y_m=sign*plane,metal_bounds_m=[points.min(0).tolist(),points.max(0).tolist()],thickness_m=.0015)


def subtract_finger_inertias(whole, fingers):
    def moments(element, mount):
        mass=float(element.find('mass').get('value'));t=mount@origin(element.find('origin'))
        a=element.find('inertia').attrib
        inertia=np.array([[float(a['ixx']),float(a['ixy']),float(a['ixz'])],
                          [float(a['ixy']),float(a['iyy']),float(a['iyz'])],
                          [float(a['ixz']),float(a['iyz']),float(a['izz'])]])
        c=t[:3,3];r=t[:3,:3]
        return mass,c,r@inertia@r.T+mass*((c@c)*np.eye(3)-np.outer(c,c))
    mass,c,tensor=moments(whole,np.eye(4));first=mass*c
    for element,mount in fingers:
        fm,fc,fi=moments(element,mount);mass-=fm;first-=fm*fc;tensor-=fi
    if mass<=0:return dict(valid=False,palm_mass_kg=mass,reason='nonpositive residual mass')
    c=first/mass;tensor-=mass*((c@c)*np.eye(3)-np.outer(c,c))
    eigen=np.linalg.eigvalsh(tensor)
    return dict(valid=bool(np.all(eigen>0)),palm_mass_kg=mass,palm_com_m=c.tolist(),
                palm_inertia_eigenvalues_kg_m2=eigen.tolist(),reason='source X5 whole-hand minus assumed native AC-One finger inertias')


def build(snapshot,native_urdf,rig_path,lfs_root,chassis,output):
    snapshot,native_urdf,output=Path(snapshot).resolve(),Path(native_urdf).resolve(),Path(output).resolve()
    pins=json.loads((snapshot/'snapshot-sha256.json').read_text())
    manifest=json.loads((snapshot/'assets/mesh_manifest.json').read_text())
    urdf=snapshot/'assets/urdf.urdf'
    paths=[urdf,snapshot/'assets/mesh_manifest.json',*[snapshot/'assets'/p for p in manifest['model']['meshes'].values()]]
    for path in paths:
        if not path.resolve().is_relative_to(snapshot) or pins.get(str(path.relative_to(snapshot)))!=sha(path):raise ValueError('untrusted X5 source')
    if sha(native_urdf)!='779204e9f4f7ed81deacaf9d81414978d110786debbaaba50119112ebe6068a4':raise ValueError('original AC-One URDF identity mismatch')
    single=ET.parse(urdf).getroot();root=ET.parse(native_urdf).getroot();root.set('name','ac_one_x5_baseline_v1')
    links={e.get('name'):e for e in root.findall('link')};joints={e.get('name'):e for e in root.findall('joint')}
    output.mkdir(parents=True,exist_ok=False);(output/'meshes').mkdir();(output/'sources').mkdir()
    sources={}
    def pin(name,data):
        (output/'sources'/name).write_bytes(data);sources['sources/'+name]=hashlib.sha256(data).hexdigest()
    pin('x5.urdf',urdf.read_bytes());pin('ac_one.urdf',native_urdf.read_bytes());pin('d405_rig.json',Path(rig_path).read_bytes());pin('chassis.stl',Path(chassis).read_bytes())
    local=[]
    for link in single.findall('link'):
        for visual in link.findall('visual'):
            node=visual.find('geometry/mesh')
            if node is None:continue
            name=link.get('name');path=snapshot/'assets'/manifest['model']['meshes'][name]
            pin('x5_'+name+'.stl',path.read_bytes())
            v=stl_vertices(path)*np.fromstring(node.get('scale','1 1 1'),sep=' ');t=origin(visual.find('origin'));v=v@t[:3,:3].T+t[:3,3]
            local.append((name,v,np.arange(len(v)).reshape(-1,3)))
    hand=hand_template(local)
    for link in links.values():
        for child in list(link):
            if child.tag in ('visual','collision'):link.remove(child)
    palette={'silver':(.57,.59,.60,1),'graphite':(.012,.015,.013,1),'rubber':(.012,.014,.013,1)}
    for name,rgba in palette.items():
        mat=ET.SubElement(root,'material',name='AC1_'+name);ET.SubElement(mat,'color',rgba=' '.join(map(str,rgba)))
    parts=[];pads=[];bases={};inertia_checks=[]
    def attach(name,link,v,f,color='silver'):
        relative='meshes/'+name+'.stl';write_stl(output/relative,v,f)
        for kind in ('visual','collision'):
            shape=ET.SubElement(links[link],kind,name=name);ET.SubElement(ET.SubElement(shape,'geometry'),'mesh',filename=relative,scale='1 1 1')
            if kind=='visual':ET.SubElement(shape,'material',name='AC1_'+color)
        parts.append(dict(name=name,link=link,path=relative,sha256=sha(output/relative),triangles=len(f)))
    for side in ('left','right'):
        lname=lambda i:f'{side}_link{i if side=="left" else 10+i}'
        jname=lambda i:f'{side}_joint{i if side=="left" else 10+i}'
        mount=origin(joints[jname(1)].find('origin'))@np.linalg.inv(origin(single.find("joint[@name='joint1']/origin")))
        bases[side]=mount.tolist()
        for i in range(1,7):
            source=single.find(f"joint[@name='joint{i}']");target=joints[jname(i)]
            t=origin(source.find('origin'));set_origin(target,mount@t if i==1 else t)
            target.find('axis').set('xyz',source.find('axis').get('xyz'))
            target.find('limit').attrib.update(source.find('limit').attrib)
            inertial=links[lname(i)].find('inertial')
            if inertial is not None:links[lname(i)].remove(inertial)
            links[lname(i)].append(deepcopy(single.find(f"link[@name='link{i}']/inertial")))
        origins={sign:np.fromstring(joints[jname(i)].find('origin').get('xyz'),sep=' ') for sign,i in ((1,7),(-1,8))}
        for i in (7,8):joints[jname(i)].find('limit').set('upper','.0415')
        finger_inertias=[]
        for sign,i in ((1,7),(-1,8)):
            mount=origin(joints[jname(i)].find('origin'));mount[1,3]+=sign*hand['nominal_gap_m']/2
            finger_inertias.append((links[lname(i)].find('inertial'),mount))
        check=subtract_finger_inertias(single.find("link[@name='link6']/inertial"),finger_inertias)
        inertia_checks.append(dict(side=side,**check))
        # These source inertias cannot define a valid split hand. Geometry-only
        # output deliberately contains no invented palm/finger physical values.
        for i in (6,7,8):
            inertial=links[lname(i)].find('inertial')
            if inertial is not None:links[lname(i)].remove(inertial)
        for sign,(v,f) in split_hand_meshes(hand,origins).items():attach(side+'_hand_'+str({0:6,1:7,-1:8}[sign]),lname({0:6,1:7,-1:8}[sign]),v,f,'graphite' if not sign else 'silver')
        for sign,i in ((1,7),(-1,8)):
            v,f,meta=pad_mesh(hand,sign,origins[sign]);attach(side+'_rubber_'+str(i),lname(i),v,f,'rubber');pads.append(dict(side=side,sign=sign,**meta))
        for name,v,f in local:
            if name=='link6':continue
            if name=='base_link':v=v@mount[:3,:3].T+mount[:3,3];dest='base_link'
            else:dest=lname(int(name[4:]))
            attach(side+'_'+name,dest,v,f,'graphite' if name in ('base_link','link1','link5') else 'silver')
        source=native_urdf.parent.parent/'meshes'/('left_link6.STL' if side=='left' else 'right_link16.STL')
        data=source_bytes(source,lfs_root)
        expected={'left':'07a5153ca15aaf6db69f3b3b27d276fec46069c2eb036e4917c99983dbb62bb8','right':'8580dc2d57b01f10edc254baffb8cf7c3f1f3ef0a72e3cfc4c1e7bed95567735'}
        if hashlib.sha256(data).hexdigest()!=expected[side]:raise ValueError('original handle mesh identity mismatch')
        pin(side+'_ac_original_link6.stl',data);v=handle_vertices(data)
        attach(side+'_handle',lname(6),v,np.arange(len(v)).reshape(-1,3),'graphite')
    v=stl_vertices(chassis);attach('chassis','base_link',v,np.arange(len(v)).reshape(-1,3),'graphite')
    rig=json.loads(Path(rig_path).read_text())
    for camera in rig['cameras']:
        name=camera['id'];link=name+'_camera_link';optical=name+'_camera_rgb_optical_frame'
        links[link]=ET.SubElement(root,'link',name=link);links[optical]=ET.SubElement(root,'link',name=optical)
        for child,parent,t in [(link,camera['parent'],transform(camera['xyz'],camera['rpy'])),(optical,link,transform((.0125,0,0),(-np.pi/2,0,-np.pi/2)))]:
            joint=ET.SubElement(root,'joint',name=child+'_joint',type='fixed');ET.SubElement(joint,'parent',link=parent);ET.SubElement(joint,'child',link=child);set_origin(joint,t)
        v=box_triangles(rig['camera_proxy_size_m'],np.eye(4));attach(name+'_camera_housing',link,v,np.arange(len(v)).reshape(-1,3))
    ET.indent(root);ET.ElementTree(root).write(output/'robot.urdf',encoding='utf-8',xml_declaration=True)
    result=dict(schema='robot_model_baseline.v1',id='ac_one_x5_baseline_v1',hardware_validated=False,geometry_only=True,runtime_allowed=False,urdf_sha256=sha(output/'robot.urdf'),sources=sources,parts=parts,
        arm_model='frozen X5 origins/axes/visual meshes; AC-One first-joint installation unchanged',T_world_arm_base=bases,
        handles='original AC-One component in nominal link6 coordinates; mounting interface not remeasured',markers_rendered=False,
        gripper=dict(max_metal_gap_m=.083,rubber_thickness_per_side_m=.0015,rubber_gap_formula='metal_gap_m - 0.003',sdk_mapping_included=False,pad_source=pads),
        materials=dict(palette=palette,pbr=dict(silver=[.82,.29],graphite=[.15,.42],rubber=[0.,.62]),assignment='AC-One palette with explicit per-part assignment; X5 topology differs, old face subsets not reused'),
        dynamics='X5 source link1..5 inertials retained; split palm/finger inertials omitted because cross-model subtraction is not positive definite; sources preserved; DO NOT IMPORT FOR DYNAMICS',inertia_checks=inertia_checks,
        cameras='AC-One nominal rig only; no serial-specific intrinsics, joint zero offsets or measured optical calibration applied')
    (output/'manifest.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n');return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('snapshot','native-urdf','rig-path','lfs-root','chassis','output'):p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args();r=build(a.snapshot,a.native_urdf,a.rig_path,a.lfs_root,a.chassis,a.output);print(json.dumps({'id':r['id'],'parts':len(r['parts']),'urdf_sha256':r['urdf_sha256']}))
