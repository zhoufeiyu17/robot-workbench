"""Trusted asset choices for collection; browser requests contain IDs only."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import re
from urllib.parse import urlencode


def file_sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def identifier(value):
    if not isinstance(value,str)or not re.fullmatch(r'[A-Za-z0-9_.-]{1,128}',value):
        raise ValueError('场景资产ID无效')
    return value


class SceneAssetCatalog:
    def __init__(self,reference,base_details):
        if (not isinstance(reference,dict)or set(reference)!={'path','sha256'}
                or not re.fullmatch('[a-f0-9]{64}',str(reference.get('sha256','')))):
            raise ValueError('场景资产目录需要固定路径和摘要')
        self.path=Path(reference['path']);self.digest=reference['sha256']
        if (not self.path.is_absolute()or self.path.is_symlink()or not self.path.is_file()
                or self.path.stat().st_size>1024*1024 or file_sha(self.path)!=self.digest):
            raise ValueError('场景资产目录文件或摘要不匹配')
        value=json.loads(self.path.read_bytes())
        if (not isinstance(value,dict)or set(value)!={'schema','assets','pairs'}
                or value['schema']!='workbench.scene_asset_catalog.v1'
                or not isinstance(value['assets'],dict)or not 1<=len(value['assets'])<=128
                or not isinstance(value['pairs'],dict)or not 1<=len(value['pairs'])<=256):
            raise ValueError('场景资产目录格式无效')
        self.assets,self.pairs=value['assets'],value['pairs'];self.base=deepcopy(base_details)
        for key,row in self.assets.items():
            identifier(key)
            if (not isinstance(row,dict)or set(row)!={'kind','label','asset_id','revision','dimensions_m','preview'}
                    or row['kind']not in ('machine','cup')or any(not isinstance(row[k],str)or not 1<=len(row[k])<=256 for k in ('label','asset_id','revision'))):
                raise ValueError('场景资产名称、类型或版本无效')
            dimensions=row['dimensions_m']
            if (not isinstance(dimensions,list)or len(dimensions)!=3
                    or any(type(x)not in (int,float)or not 0<x<10 for x in dimensions)):
                raise ValueError('场景资产尺寸必须是有限的米制三维尺寸')
            preview=row['preview']
            if not isinstance(preview,dict)or set(preview)!={'file','sha256'}:raise ValueError('资产预览必须固定文件和摘要')
            self._preview_path(preview)
        for key,row in self.pairs.items():
            identifier(key)
            required={'machine','cup','asset_set_digest'}
            optional={'compute','profile_template','profile_template_sha256','reason'}
            if not isinstance(row,dict)or not required<=set(row)or set(row)-required-optional:
                raise ValueError('场景资产组合格式无效')
            if any(row.get(k)not in self.assets or self.assets[row[k]]['kind']!=k for k in ('machine','cup')):
                raise ValueError('资产组合必须引用目录中的咖啡机和杯子')
            if not re.fullmatch('[a-f0-9]{64}',str(row['asset_set_digest'])):raise ValueError('资产组合摘要无效')
            configured={'compute','profile_template','profile_template_sha256'}&set(row)
            if configured and configured!={'compute','profile_template','profile_template_sha256'}:
                raise ValueError('资产组合需要完整计算部署和接口模板')
            if configured:
                compute=row['compute']
                if (not isinstance(compute,dict)or set(compute)!={'endpoint','deployment_path','deployment_sha256'}
                        or compute['endpoint']!=self.base['compute']['endpoint']
                        or not isinstance(compute['deployment_path'],str)or not Path(compute['deployment_path']).is_absolute()
                        or not re.fullmatch('[a-f0-9]{64}',str(compute['deployment_sha256']))
                        or not isinstance(row['profile_template'],str)or not Path(row['profile_template']).is_absolute()
                        or not re.fullmatch('[a-f0-9]{64}',str(row['profile_template_sha256']))):
                    raise ValueError('资产组合只能使用当前登记的计算端点和固定部署')

    def _preview_path(self,row):
        relative=Path(row['file']);path=self.path.parent/relative
        if (relative.is_absolute()or '..'in relative.parts or path.is_symlink()
                or not path.resolve().is_relative_to(self.path.parent.resolve())
                or path.suffix.lower()not in ('.png','.jpg','.jpeg')
                or not re.fullmatch('[a-f0-9]{64}',str(row['sha256']))):
            raise ValueError('资产预览必须是目录内固定图片')
        return path

    def public(self,environment,*,requested=None,applied=None):
        assets=[]
        for key,row in self.assets.items():
            assets.append({k:deepcopy(row[k])for k in ('kind','label','asset_id','revision','dimensions_m')}|dict(id=key,
                preview_url='/api/scene-assets/preview?'+urlencode(dict(environment=environment,asset_id=key,catalog_digest=self.digest))))
        pairs=[dict(id=key,machine=row['machine'],cup=row['cup'],asset_set_digest=row['asset_set_digest'],
            can_prepare='compute'in row,reason=row.get('reason')or(None if 'compute'in row else'此组合的计算部署尚未登记'))for key,row in self.pairs.items()]
        return dict(catalog_digest=self.digest,assets=assets,pairs=pairs,requested_pair_id=requested,applied_pair_id=applied)

    def resolve(self,pair_id,expected_digest):
        if expected_digest!=self.digest or file_sha(self.path)!=self.digest:
            raise ValueError('场景资产目录已变化，请刷新后重新选择')
        row=self.pairs.get(identifier(pair_id))
        if row is None:raise ValueError('所选资产组合未登记')
        if 'compute'not in row:raise ValueError(row.get('reason')or'此组合的计算部署尚未登记')
        path=Path(row['profile_template'])
        if path.is_symlink()or not path.is_file()or file_sha(path)!=row['profile_template_sha256']:
            raise ValueError('所选资产接口模板已变化')
        return dict(compute=deepcopy(row['compute']),registration=self.base['registration'],profile_template=str(path))

    def preview(self,asset_id,expected_digest):
        if expected_digest!=self.digest:raise ValueError('资产预览属于旧目录')
        row=self.assets.get(identifier(asset_id))
        if row is None:raise ValueError('资产预览未登记')
        path=self._preview_path(row['preview'])
        if not path.is_file()or path.stat().st_size>4*1024*1024 or file_sha(path)!=row['preview']['sha256']:
            raise ValueError('资产预览文件或摘要已变化')
        raw=path.read_bytes()
        kind='image/png'if raw.startswith(b'\x89PNG\r\n\x1a\n')else'image/jpeg'if raw.startswith(b'\xff\xd8')else None
        if kind is None:raise ValueError('资产预览图片格式无效')
        return raw,kind
