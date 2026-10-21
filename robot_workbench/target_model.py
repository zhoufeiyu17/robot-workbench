"""Sealed display geometry only: compact buffers and the native URDF FK tree."""
from pathlib import Path
import gzip,hashlib,io,json,os,stat
import numpy as np

SCHEMA='workbench.target_model.v1'
MAX_BYTES=64*1024*1024

def export_model(geometry,output,device_id):
    output=Path(output);output.mkdir(mode=0o700,parents=True,exist_ok=True)
    model=geometry.model;meshes=[];buffers=[];offset=0
    for side in ('left','right'):
        aliases={alias:native for native,alias in geometry.aliases[side].items()}
        for name,vertices,faces in geometry.full_side_meshes[side]:meshes.append((aliases[name],vertices[faces]))
    meshes.extend((None,vertices[faces])for _,vertices,faces in geometry.static_meshes)
    rows=[]
    for link,triangles in meshes:
        positions=np.asarray(triangles,dtype='<f4').reshape(-1,3)
        normal=np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0])
        normal/=np.maximum(np.linalg.norm(normal,axis=1,keepdims=True),1e-12)
        normals=np.asarray(np.repeat(normal,3,axis=0),dtype='<f4')
        raw=positions.tobytes()+normals.tobytes();buffers.append(raw)
        rows.append(dict(link=link,offset=offset,normal_offset=offset+positions.nbytes,count=len(positions)))
        offset+=len(raw)
    binary=b''.join(buffers)
    if len(binary)>MAX_BYTES:raise ValueError('display mesh exceeds bounded binary size')
    zero=geometry.frames(np.zeros(12));points=[]
    for side in ('left','right'):
        for name,vertices,_ in geometry.full_side_meshes[side]:
            T=zero[side][name];points.append(vertices@T[:3,:3].T+T[:3,3])
    points.extend(vertices for _,vertices,_ in geometry.static_meshes)
    bounds=[np.min([v.min(axis=0)for v in points],axis=0).tolist(),np.max([v.max(axis=0)for v in points],axis=0).tolist()]
    metadata=dict(schema=SCHEMA,device_id=device_id,root=model.root,base=model.base,q_names=model.names,
        joints=[dict(name=n,parent=p,child=c,origin=T.tolist(),axis=a.tolist(),kind=k)for n,p,c,T,a,k in model.joints],
        mimics=model.mimics,gap_joints=[dict(name=f'{side}_joint{i+off}',side=j,scale=.0005,offset=-.000004)
            for j,(side,off)in enumerate((('left',0),('right',10)))for i in (7,8)],
        meshes=rows,bounds=bounds,binary_bytes=len(binary),binary_sha256=hashlib.sha256(binary).hexdigest(),
        urdf_sha256=hashlib.sha256(Path(geometry.config['urdf']).read_bytes()).hexdigest(),
        display_only=True,measurement_qualified=False)
    canonical=json.dumps(metadata,sort_keys=True,separators=(',',':')).encode();model_id=hashlib.sha256(canonical).hexdigest()
    metadata['model_id']=model_id;raw=json.dumps(metadata,separators=(',',':')).encode();packed=gzip.compress(binary,mtime=0)
    for name,data in [('model.json',raw),('model.bin.gz',packed)]:
        with os.fdopen(os.open(output/name,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600),'wb')as f:f.write(data)
    manifest=dict(schema=SCHEMA,device_id=device_id,model_id=model_id,metadata_sha256=hashlib.sha256(raw).hexdigest(),compressed_bytes=len(packed),metadata='model.json',binary='model.bin.gz')
    with os.fdopen(os.open(output/'manifest.json',os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600),'w')as f:json.dump(manifest,f)
    return manifest


class ModelAssets:
    def __init__(self,path,device_id):
        path=Path(path)
        def read(file,limit):
            info=file.lstat()
            if not stat.S_ISREG(info.st_mode)or info.st_uid!=os.getuid()or info.st_mode&0o077 or not 0<info.st_size<=limit:
                raise ValueError('private owned bounded display asset required')
            return file.read_bytes()
        if not path.is_absolute():raise ValueError('absolute display manifest required')
        manifest=json.loads(read(path,16384));self.model_id=manifest['model_id']
        if manifest.get('schema')!=SCHEMA or manifest.get('device_id')!=device_id or manifest.get('metadata')!='model.json'or manifest.get('binary')!='model.bin.gz':
            raise ValueError('display asset identity differs')
        self.metadata_raw=read(path.parent/'model.json',256*1024)
        if hashlib.sha256(self.metadata_raw).hexdigest()!=manifest['metadata_sha256']:raise ValueError('display metadata digest differs')
        self.metadata=json.loads(self.metadata_raw);self.binary=read(path.parent/'model.bin.gz',16*1024*1024)
        if len(self.binary)!=manifest['compressed_bytes']:raise ValueError('compressed display byte count differs')
        with gzip.GzipFile(fileobj=io.BytesIO(self.binary))as f:raw=f.read(MAX_BYTES+1)
        if (len(raw)>MAX_BYTES or self.metadata.get('model_id')!=self.model_id or self.metadata.get('device_id')!=device_id
                or len(raw)!=self.metadata.get('binary_bytes')or hashlib.sha256(raw).hexdigest()!=self.metadata.get('binary_sha256')):
            raise ValueError('display binary integrity differs')
        self.public=dict(model_id=self.model_id,url='/api/target-model?environment=physical',
            binary_url='/api/target-model.bin?environment=physical&model_id='+self.model_id)
