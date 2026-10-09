"""Export nominal AC-One visual STL and box geometry. No simulator truth."""
import argparse,json,hashlib
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
p=argparse.ArgumentParser();p.add_argument('--urdf',type=Path,required=True);p.add_argument('--package',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
assert not a.output.exists()
out={};names=[];counts={};hashes={}
for link in ET.parse(a.urdf).getroot().findall('link'):
 vs=[];fs=[];offset=0
 for vis in link.findall('visual'):
  mesh=vis.find('geometry/mesh');box=vis.find('geometry/box')
  if mesh is not None:
   uri=mesh.get('filename');assert uri.startswith('package://acone/'),uri
   path=a.package/uri.removeprefix('package://acone/');raw=path.read_bytes();hashes[path.name]=hashlib.sha256(raw).hexdigest()
   n=int.from_bytes(raw[80:84],'little');dtype=np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attribute','<u2')])
   if len(raw)==84+50*n:tri=np.frombuffer(raw,dtype=dtype,offset=84)['vertices'].astype(float)
   else:tri=np.array([list(map(float,line.split()[1:])) for line in raw.decode().splitlines() if line.strip().startswith('vertex ')]).reshape(-1,3,3)
   vertices,inverse=np.unique(tri.reshape(-1,3),axis=0,return_inverse=True);faces=inverse.reshape(-1,3)
   vertices*=np.fromstring(mesh.get('scale','1 1 1'),sep=' ')
  elif box is not None:
   size=np.fromstring(box.get('size'),sep=' ')
   if size.shape!=(3,) or not np.isfinite(size).all() or (size<=0).any():raise ValueError('invalid visual box')
   vertices=np.array([[-1,-1,-1],[1,-1,-1],[1,1,-1],[-1,1,-1],[-1,-1,1],[1,-1,1],[1,1,1],[-1,1,1]])*size/2
   faces=np.array([[0,2,1],[0,3,2],[4,5,6],[4,6,7],[0,1,5],[0,5,4],[1,2,6],[1,6,5],[2,3,7],[2,7,6],[3,0,4],[3,4,7]])
  else:raise ValueError('unsupported visual primitive; refusing incomplete occlusion geometry')
  origin=vis.find('origin')
  if origin is not None:vertices=vertices@Rotation.from_euler('xyz',np.fromstring(origin.get('rpy','0 0 0'),sep=' ')).as_matrix().T+np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
  vs.append(vertices);fs.append(faces+offset);offset+=len(vertices)
 if vs:
  i=len(names);name=link.get('name');names.append(name);out[f'vertices_{i}']=np.concatenate(vs);out[f'faces_{i}']=np.concatenate(fs);counts[name]=[offset,len(out[f'faces_{i}'])]
out['names']=np.array(names);np.savez_compressed(a.output,**out);a.output.with_suffix('.json').write_text(json.dumps({'urdf_sha256':hashlib.sha256(a.urdf.read_bytes()).hexdigest(),'mesh_sha256':hashes,'counts':counts},indent=2));print(json.dumps(counts))
