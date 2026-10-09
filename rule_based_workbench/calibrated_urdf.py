"""Build an isolated, hash-pinned URDF from the calibration geometry snapshot.

No zero offsets are baked in, and no existing asset or running simulation is changed.
Dynamics are inherited from the native model, not physically re-estimated.
"""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
from arx_env.urdf_mesh import origin
from calibrated_mesh import CalibratedMesh
from calibrated_gripper import split_hand_meshes


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def set_origin(element, matrix):
    old = element.find('origin')
    if old is not None:
        element.remove(old)
    ET.SubElement(element, 'origin', xyz=' '.join(format(v, '.17g') for v in matrix[:3,3]),
                  rpy=' '.join(format(v, '.17g') for v in Rotation.from_matrix(matrix[:3,:3]).as_euler('xyz')))


def write_stl(path, vertices, faces):
    triangles = np.asarray(vertices, float)[faces]
    normal = np.cross(triangles[:,1]-triangles[:,0], triangles[:,2]-triangles[:,0])
    length = np.linalg.norm(normal, axis=1)
    normal = np.divide(normal, length[:,None], out=np.zeros_like(normal), where=length[:,None]>0)
    data = np.zeros(len(triangles), dtype=[('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attr','<u2')])
    data['normal'], data['vertices'] = normal, triangles
    with Path(path).open('xb') as stream:
        stream.write(b'calibration snapshot candidate; metres'.ljust(80,b'\0'))
        stream.write(struct.pack('<I',len(triangles)));stream.write(data.tobytes())


def build(snapshot, native_urdf, output):
    snapshot, native_urdf, output = Path(snapshot).resolve(), Path(native_urdf).resolve(), Path(output).resolve()
    mesh = CalibratedMesh(snapshot)  # Verifies every source geometry/calibration pin.
    geometry = mesh.geometry
    root = ET.parse(native_urdf).getroot()
    single = ET.parse(geometry.config['urdf']).getroot()
    output.mkdir(parents=True, exist_ok=False)
    (output/'meshes').mkdir()
    links = {item.get('name'):item for item in root.findall('link')}
    joints = {item.get('name'):item for item in root.findall('joint')}
    originals = {name:deepcopy(link.find('inertial')) for name,link in links.items()}
    # Replace all visual/collision shapes, including stale native camera housings.
    for link in links.values():
        for child in list(link):
            if child.tag in ('visual','collision'):link.remove(child)
    parts = []
    def attach(name, link, vertices, faces):
        relative = 'meshes/'+name+'.stl'
        write_stl(output/relative, vertices, faces)
        for kind in ('visual','collision'):
            shape=ET.SubElement(links[link],kind,name=name)
            ET.SubElement(ET.SubElement(shape,'geometry'),'mesh',filename=relative,scale='1 1 1')
        parts.append(dict(name=name,link=link,path=relative,sha256=sha(output/relative),triangles=len(faces)))
    flip=np.diag([1.,-1.,-1.,1.])
    for side in ('left','right'):
        def joint_name(i):return f'{side}_joint{i if side=="left" else 10+i}'
        def link_name(i):return f'{side}_link{i if side=="left" else 10+i}'
        for i in range(1,7):
            native=joints[joint_name(i)];cal=single.find(f"joint[@name='joint{i}']")
            transform=origin(cal.find('origin'))
            if i==1:transform=geometry.bases[side]@transform
            set_origin(native,transform)
            native.find('axis').set('xyz',cal.find('axis').get('xyz'))
            # Native link3..5 use Rx(pi)-different frames. Re-express the same
            # mass/inertia frame; the inertia tensor's own coordinates stay intact.
            inertia=links[link_name(i)].find('inertial')
            if i in (3,4,5) and inertia is not None:
                set_origin(inertia,flip@origin(inertia.find('origin')))
        origins={sign:np.fromstring(joints[joint_name(i)].find('origin').get('xyz'),sep=' ')
                 for sign,i in ((1,7),(-1,8))}
        for sign,i in ((1,7),(-1,8)):
            row=joints[joint_name(i)]
            if not np.allclose(origin(row.find('origin'))[:3,:3],np.eye(3)):
                raise ValueError('finger joint origin rotation requires explicit adaptation')
            if not np.array_equal(np.fromstring(row.find('axis').get('xyz'),sep=' '),[0,sign,0]):
                raise ValueError('unexpected native finger axis')
            row.find('limit').set('upper','.0415')
        hand=split_hand_meshes(geometry.hand,origins)
        for sign,(vertices,faces) in hand.items():
            i={0:6,1:7,-1:8}[sign]
            attach(f'{side}_hand_{i}',link_name(i),vertices,faces)
        for name,vertices,faces in geometry.side_meshes[side]:
            if name=='link6':continue
            if name=='base_link':
                t=geometry.bases[side];vertices=vertices@t[:3,:3].T+t[:3,3];link='base_link'
            elif name.startswith('link'):
                link=link_name(int(name[4:]))
            else:
                link=link_name(6)  # Existing nominal camera housings/supports.
            # Geometry faces are display-sampled; all source triangles are required.
            attach(side+'_'+name,link,vertices,np.arange(len(vertices)).reshape(-1,3))
    for name,vertices,faces in geometry.static_meshes:
        attach('static_'+name,'base_link',vertices,np.arange(len(vertices)).reshape(-1,3))
    # Optical poses are calibrated independently of unchanged housing geometry.
    for name,pose in [('top',mesh.head_camera),('left_wrist',mesh.wrist_cameras['left']),('right_wrist',mesh.wrist_cameras['right'])]:
        set_origin(joints[name+'_camera_link_joint'],pose)
        set_origin(joints[name+'_camera_rgb_optical_frame_joint'],np.eye(4))
    ET.indent(root)
    ET.ElementTree(root).write(output/'robot.urdf',encoding='utf-8',xml_declaration=True)
    for name,inertial in originals.items():
        current=links[name].find('inertial')
        if inertial is not None:
            if (inertial.find('mass').attrib != current.find('mass').attrib or
                    inertial.find('inertia').attrib != current.find('inertia').attrib):
                raise ValueError('native physical mass/inertia values changed')
    manifest=dict(schema='calibrated_urdf_candidate.v1',hardware_validated=False,
        native_urdf_sha256=sha(native_urdf),snapshot_manifest_sha256=sha(snapshot/'snapshot-sha256.json'),
        urdf_sha256=sha(output/'robot.urdf'),parts=parts,calibration=mesh.calibration(),
        joint_space='model; zero offsets not baked in',gripper_max_opening_mm=83.,
        dynamics='native masses/inertia reused; link3..5 inertia frames rotated Rx(pi); not physically calibrated',
        collision='full calibration visual meshes, including nominal proxies; Isaac convex decomposition unvalidated',
        housing_policy='unchanged calibration snapshot',markers_rendered=False,
        validation_scope='offline candidate; not deployed or accepted for physical execution')
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2,allow_nan=False)+'\n')
    return manifest


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for key in ('snapshot','native-urdf','output'):parser.add_argument('--'+key,type=Path,required=True)
    args=parser.parse_args()
    print(json.dumps(build(args.snapshot,args.native_urdf,args.output),indent=2))
