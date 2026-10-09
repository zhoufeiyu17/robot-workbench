"""Blender-only offline URDF reference and sticker-placement renderer."""
import argparse
import json
from pathlib import Path
import struct
import sys
import xml.etree.ElementTree as ET

import bpy
from mathutils import Matrix, Vector, Euler
import numpy as np


def stl(path):
    data=path.read_bytes();n=struct.unpack_from('<I',data,80)[0]
    if len(data)!=84+50*n:raise ValueError('binary STL required')
    d=np.dtype([('normal','<f4',(3,)),('v','<f4',(3,3)),('attr','<u2')])
    v=np.frombuffer(data,dtype=d,offset=84)['v'].reshape(-1,3)
    unique,index=np.unique(v,axis=0,return_inverse=True)
    return unique,index.reshape(-1,3)


def pose(node):
    if node is None:return Matrix.Identity(4)
    xyz=list(map(float,node.get('xyz','0 0 0').split()))
    rpy=list(map(float,node.get('rpy','0 0 0').split()))
    return Matrix.Translation(Vector(xyz)) @ Euler(rpy,'XYZ').to_matrix().to_4x4()


def material(name,color):
    m=bpy.data.materials.new(name);m.diffuse_color=(*color,1);return m


def mesh_object(name,v,f,mat,world):
    mesh=bpy.data.meshes.new(name);mesh.from_pydata(v.tolist(),[],f.tolist());mesh.update()
    obj=bpy.data.objects.new(name,mesh);bpy.context.collection.objects.link(obj)
    obj.matrix_world=world;obj.data.materials.append(mat);return obj


def box(name,size,world,mat):
    bpy.ops.mesh.primitive_cube_add(size=1);o=bpy.context.object;o.name=name
    o.matrix_world=world;o.scale=size;o.data.materials.append(mat);return o


def render(model,output):
    model,output=Path(model),Path(output);output.mkdir(parents=True,exist_ok=True)
    bpy.ops.object.select_all(action='SELECT');bpy.ops.object.delete(use_global=False)
    tree=ET.parse(model/'robot.urdf').getroot()
    model_manifest=json.loads((model/'manifest.json').read_text())
    grip=model_manifest.get('gripper',{})
    jaw_q=grip.get('symmetric_joint_upper_m',grip.get('max_joint_position_m',.0415))
    mats={}
    for m in tree.findall('material'):
        color=m.find('color');rgba=[.6,.65,.7,1] if color is None else list(map(float,color.get('rgba').split()))
        mats[m.get('name')]=material(m.get('name'),rgba[:3])
    fallback=material('default',(0.56,.61,.67))
    frames={'base_link':Matrix.Identity(4)}
    offsets=[0.]*12
    if (model/'adaptation.json').exists():offsets=json.loads((model/'adaptation.json').read_text())['zero_offsets_rad']
    sample=[0.,1.,1.,0.,0.,0.]*2
    for j in tree.findall('joint'):
        p,c=j.find('parent').get('link'),j.find('child').get('link')
        # URDF declarations are ordered in the generated assets.
        if p not in frames:raise ValueError('unordered or missing parent '+p)
        local=pose(j.find('origin'));kind=j.get('type')
        if kind in ('revolute','continuous'):
            name=j.get('name');side=0 if name.startswith('left_') else 1
            i=int(name.split('joint')[1])-(0 if side==0 else 10)-1
            axis=Vector(map(float,j.find('axis').get('xyz').split()))
            local=local@Matrix.Rotation(sample[side*6+i]+offsets[side*6+i],4,axis)
        elif kind=='prismatic':local=local@Matrix.Translation(Vector(map(float,j.find('axis').get('xyz').split()))*jaw_q)
        frames[c]=frames[p]@local
    for link in tree.findall('link'):
        for visual in link.findall('visual'):
            mesh=visual.find('geometry/mesh')
            if mesh is None:continue
            path=(model/mesh.get('filename')).resolve()
            if not path.is_relative_to(model.resolve()):raise ValueError('mesh path escapes model')
            v,f=stl(path);v=v*np.array(list(map(float,mesh.get('scale','1 1 1').split())))
            node=visual.find('material');m=fallback if node is None else mats.get(node.get('name'),fallback)
            mesh_object(visual.get('name',link.get('name')),v,f,m,frames[link.get('name')]@pose(visual.find('origin')))
    scene=bpy.context.scene;scene.render.engine='BLENDER_WORKBENCH'
    scene.display.shading.light='STUDIO';scene.display.shading.color_type='MATERIAL'
    scene.display.shading.show_shadows=True;scene.display.shading.show_cavity=True
    scene.display.shading.cavity_type='BOTH';scene.display.shading.background_type='WORLD'
    scene.world.color=(.86,.89,.93);scene.view_settings.view_transform='Standard'
    scene.render.resolution_x=1500;scene.render.resolution_y=1200;scene.render.resolution_percentage=100
    scene.render.image_settings.file_format='PNG';scene.render.film_transparent=False
    bpy.ops.object.camera_add();cam=bpy.context.object;scene.camera=cam;cam.data.type='ORTHO'
    def shot(name,target,eye,scale):
        cam.location=eye;cam.rotation_euler=(Vector(target)-cam.location).to_track_quat('-Z','Y').to_euler();cam.data.ortho_scale=scale
        scene.render.filepath=str(output/name);bpy.ops.render.render(write_still=True)
    shot('overview.png',(.1,0,.22),(1.3,-1.5,1.1),1.1)
    bpy.ops.wm.save_as_mainfile(filepath=str(output/'reference.blend'))
    hidden=[]
    for obj in scene.objects:
        if obj.type=='MESH' and obj.name not in ('chassis','top_camera_housing'):
            obj.hide_render=True;hidden.append(obj)
    seat_path=output.parent/'native-seat.stl'
    seat=None
    if seat_path.exists():
        v,f=stl(seat_path);center=v.mean(0);v=center+(v-center)*1.0002
        seat=mesh_object('original_seat_highlight',v,f,material('original seat diagnostic',(.95,.48,.06)),Matrix.Identity(4))
    shot('mast.png',(.045,0,.29),(.95,-1.0,.52),.66)
    if seat:seat.hide_render=True
    for obj in hidden:obj.hide_render=False
    # Demo only: nominal placement centred on the rigid housing's top face.
    white=material('demo_paper',(1,1,1));black=material('demo_print',(0.015,.02,.025))
    demos=[]
    for side,marker in [('left',24),('right',30)]:
        frame=frames[side+'_wrist_camera_link']
        demos.append(box('DEMO_PAPER_'+str(marker),(.02,.02,.0002),frame@Matrix.Translation((0,0,.0227)),white))
        demos.append(box('DEMO_MARKER_'+str(marker),(.015,.015,.0002),frame@Matrix.Translation((0,0,.0229)),black))
        # Readable location label, not a printable/scannable ArUco pattern.
        curve=bpy.data.curves.new('ID_'+str(marker),'FONT');curve.body=str(marker);curve.align_x='CENTER';curve.align_y='CENTER';curve.size=.007
        text=bpy.data.objects.new('DEMO_ID_'+str(marker),curve);bpy.context.collection.objects.link(text)
        text.matrix_world=frame@Matrix.Translation((0,0,.0231));text.data.materials.append(white);demos.append(text)
        t=frame.translation
        shot('sticker_'+side+'.png',t,t+frame.to_3x3()@Vector((.15,-.17,.22)),.21)
    for ob in demos:ob.hide_render=True
    audit=dict(sample_reported_joint_rad=sample,max_metal_gap_mm=83,pad_thickness_mm=1.5,
               current_pose_is_demo=True,sticker_demo_only=True,marker_ids={'left':24,'right':30},
               marker_black_square_mm=15,paper_square_mm=20,
               marker_locations='nominal housing top face; actual mounting solved during calibration; demo is not printable ArUco',
               model=str(model),zero_offsets_applied=offsets)
    (output/'render-info.json').write_text(json.dumps(audit,indent=2)+'\n')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--model',required=True);p.add_argument('--output',required=True)
    a=p.parse_args(sys.argv[sys.argv.index('--')+1:]);render(a.model,a.output)
