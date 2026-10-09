"""Offline X5 mesh browser using frozen calibration and measured frame feedback."""
import argparse,json,shutil,xml.etree.ElementTree as ET
from pathlib import Path
import cv2
import numpy as np
from joint_head_real_fit import OLD,Model,rectify
from arx_env.urdf_mesh import origin
from arx_env.joint_head_markerless import raster_depth

def hand_template(meshes):
    matches = [(vertices, faces) for name, vertices, faces in meshes if name == "link6"]
    if len(matches) != 1:
        raise ValueError("expected exactly one link6 visual mesh")
    vertices, faces = matches[0]
    unique, inverse = np.unique(vertices, axis=0, return_inverse=True)
    merged_faces = inverse[faces]
    parent = np.arange(len(unique))

    def find(index):
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = parent[index]
        return index

    def union(a, b):
        a, b = find(a), find(b)
        if a != b:
            parent[b] = a

    for triangle in merged_faces:
        union(triangle[0], triangle[1])
        union(triangle[1], triangle[2])
    roots = np.array([find(index) for index in range(len(unique))])
    signs = np.zeros(len(unique), dtype=np.int8)
    positive_min, negative_max = [], []
    for root in np.unique(roots):
        indices = np.flatnonzero(roots == root)
        points = unique[indices]
        lo, hi = points.min(0), points.max(0)
        if lo[1] >= .025 and hi[0] >= .06:
            signs[indices] = 1
            positive_min.append(lo[1])
        elif hi[1] <= -.025 and hi[0] >= .06:
            signs[indices] = -1
            negative_max.append(hi[1])
    if not positive_min or not negative_max:
        raise ValueError("link6 finger components are not separable")
    nominal_gap = min(positive_min) - max(negative_max)
    if not .04 <= nominal_gap <= .08:
        raise ValueError(f"unexpected nominal finger gap: {nominal_gap}")
    return {"vertices": vertices, "faces": faces, "signs": signs[inverse], "nominal_gap_m": float(nominal_gap)}


def main():
 p=argparse.ArgumentParser();p.add_argument('--dataset',type=Path,required=True);p.add_argument('--candidate',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
 a.output.mkdir(parents=True,exist_ok=True)
 urdf=OLD/'calibration-base-xyz-round1/dataset-v2/model/model.urdf';model=Model(urdf,[f'joint{i}' for i in range(1,7)])
 mesh_manifest=json.loads((urdf.parent.parent/'dataset.json').read_text())['model']['meshes']
 bases=json.loads((OLD/'calibration-motion-design/scene-seed.json').read_text())['T_world_arm_base']
 candidate=json.loads(a.candidate.read_text());camera=np.array(candidate['T_world_optical']);k=json.loads((a.dataset/'camera.json').read_text());meshes=[]
 for link in ET.parse(urdf).getroot().findall('link'):
  for visual in link.findall('visual'):
   m=visual.find('geometry/mesh')
   if m is None:continue
   file=urdf.parent.parent/mesh_manifest[link.get('name')];raw=file.read_bytes();n=int.from_bytes(raw[80:84],'little')
   dt=np.dtype([('normal','<f4',(3,)),('v','<f4',(3,3)),('attr','<u2')])
   if len(raw)==84+50*n:tri=np.frombuffer(raw,dtype=dt,offset=84)['v'].astype(float)
   else:tri=np.array([list(map(float,line.split()[1:])) for line in raw.decode().splitlines() if line.strip().startswith('vertex ')]).reshape(-1,3,3)
   v=tri.reshape(-1,3)*np.fromstring(m.get('scale','1 1 1'),sep=' ');T=origin(visual.find('origin'));v=v@T[:3,:3].T+T[:3,3];meshes.append((link.get('name'),v,np.arange(len(v)).reshape(-1,3)))
 template=hand_template(meshes)
 print('nominal_gap_m',template['nominal_gap_m'],flush=True)
 h,w=k['height'],k['width'];yy,xx=np.mgrid[:h,:w];rect=rectify(np.c_[xx.ravel(),yy.ravel()],k).reshape(h,w,2).astype(np.float32)
 rows=[]
 for file in sorted(a.dataset.glob('*.json')):
  row=json.loads(file.read_text())
  if 'qpos' not in row or 'image' not in row:continue
  layers=[]
  for side,j in [('left',0),('right',7)]:
   fk=model.frames(np.array(row['qpos'][j:j+6]));parts=[]
   for name,v,f in meshes:
    if name=='link6':
     v=v.copy();gap=.083 if side=='left' else row['qpos'][13];v[:,1]+=template['signs']*(gap-template['nominal_gap_m'])/2
    T=np.array(bases[side])@fk[name];world=v@T[:3,:3].T+T[:3,3];pc=(world-camera[:3,3])@camera[:3,:3];parts.append((pc,f))
   depth=raster_depth(parts,k,(h,w));mask=np.isfinite(depth).astype(np.uint8)*255
   layers.append(cv2.remap(mask,rect[:,:,0],rect[:,:,1],cv2.INTER_NEAREST,borderMode=cv2.BORDER_CONSTANT))
  rgba=np.zeros((h,w,4),np.uint8)
  for mask,color in zip(layers,[(255,190,20),(40,140,255)]):
   rgba[mask>0]=[*color,160]
  edge=np.zeros_like(rgba)
  for mask,color in zip(layers,[(255,190,20),(40,140,255)]):
   contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE);cv2.drawContours(edge,contours,-1,(*color,255),1)
  name=row['id'];shutil.copyfile(a.dataset/row['image'],a.output/(name+'.png'));cv2.imwrite(str(a.output/(name+'-mesh.png')),rgba);cv2.imwrite(str(a.output/(name+'-edge.png')),edge)
  rows.append(dict(id=name,split=row['split'],q=row['qpos'],left_gap_mm=83.,right_gap_mm=round(row['qpos'][13]*1000,2)))
  print(name,flush=True)
 data=json.dumps(rows)
 html='''<!doctype html><meta charset="utf-8"><title>X5 真机 mesh 叠加</title><style>body{background:#15202b;color:#eee;font:16px system-ui;margin:24px}button,select,input{font:inherit;margin:8px}#view{position:relative;width:960px;max-width:100%;aspect-ratio:4/3}img{position:absolute;width:100%;height:100%;image-rendering:auto}pre{white-space:pre-wrap}small{color:#bcc9d5}</style><h2>真实 X5 · 头相机 mesh 叠加浏览</h2><p>冻结相机候选＋采样实际关节；左臂青色，右臂橙色。离线采集回放，不控制真机。</p><button id="prev">上一张</button><select id="frame" aria-label="采样帧"></select><button id="next">下一张</button><label>透明度<input id="alpha" type="range" min="0" max="1" step=".05" value=".5"></label><label><input id="outline" type="checkbox" checked>轮廓</label><div id="view"><img id="photo"><img id="mesh"><img id="edge"></div><p id="info"></p><small>模型是原始 X5 URDF，未包含腕相机外壳；夹指开度已适配：左侧使用历史尺量内间距 83 mm（本次未复量），右侧使用 SDK 开度近似；开度与模型误差仍需单独检查。原图保留 inverse-Brown 畸变，mesh 投影已映射到原图坐标。名义底座安装与 15 mm 黑框尺寸仍是假设。</small><details><summary>实际关节反馈</summary><pre id="q"></pre></details><script>const rows=DATA;const $=id=>document.getElementById(id);rows.forEach((r,i)=>$('frame').add(new Option(r.split+' / '+r.id,i)));function draw(){const r=rows[+$('frame').value];$('photo').src=r.id+'.png';$('mesh').src=r.id+'-mesh.png?v=opening-fixed-2';$('edge').src=r.id+'-edge.png?v=opening-fixed-2';$('mesh').style.opacity=$('alpha').value;$('edge').hidden=!$('outline').checked;$('info').textContent=(+$('frame').value+1)+' / '+rows.length+' · '+r.split+' · 显示内间距：左 '+r.left_gap_mm+' mm（历史尺量），右 '+r.right_gap_mm+' mm（SDK 近似）';$('q').textContent=JSON.stringify(r.q,null,2)}$('frame').onchange=$('alpha').oninput=$('outline').onchange=draw;$('prev').onclick=()=>{$('frame').selectedIndex=Math.max(0,$('frame').selectedIndex-1);draw()};$('next').onclick=()=>{$('frame').selectedIndex=Math.min(rows.length-1,$('frame').selectedIndex+1);draw()};draw();</script>'''.replace('DATA',data)
 (a.output/'index.html').write_text(html);(a.output/'manifest.json').write_text(json.dumps({'candidate':candidate,'frames':rows,'model':str(urdf)},indent=2))
if __name__=='__main__':main()
