"""Same-origin routing for an independent, display-only CPU renderer."""
from copy import deepcopy
import json,math,os,secrets,stat
from pathlib import Path
from urllib.parse import parse_qs,urlsplit

from .service import Application,Device
from .target_preview import RendererClient,TargetPreview


class TargetDisplayGateway:
    def __init__(self,path,device_id):
        path=Path(path);info=path.stat()
        if not path.is_absolute()or path.is_symlink()or not stat.S_ISREG(info.st_mode)or info.st_uid!=os.getuid()or info.st_mode&0o077 or info.st_size>16384:
            raise ValueError('private owned target display descriptor required')
        value=json.loads(path.read_text())
        if value.get('schema')!='workbench.cpu_target_display.v1'or value.get('device_id')!=device_id:
            raise ValueError('target display device identity differs')
        address=urlsplit(value['url']);limits=value['joint_limits'];token=Path(value['token_file']);secret=token.stat()
        if (address.scheme!='http'or address.hostname!='127.0.0.1'or address.path or address.query or address.fragment
                or address.username or address.password or not address.port or not token.is_absolute()or token.is_symlink()
                or secret.st_uid!=os.getuid()or secret.st_mode&0o077 or not stat.S_ISREG(secret.st_mode)
                or len(limits)!=12 or any(not isinstance(pair,list)or len(pair)!=2 or not all(type(v)in(int,float)and math.isfinite(v)for v in pair)
                    or pair[0]>pair[1]for pair in limits)):
            raise ValueError('bounded loopback display and explicit model bounds required')
        self.client=RendererClient(value['url'],value['token_file'])
        def validate(q,gaps):
            if any(abs(v)>2*math.pi for v in q)or any(not 0<=v<=83 for v in gaps):
                raise ValueError('target display requires bounded angles and 0–83 mm jaw gaps; no control permission is granted')
        self.device=Device(device_id,'physical',None)
        self.device.preview=TargetPreview(self.client,value['renderer_instance_id'],validate_pose=validate)
        self.app=Application({'physical':self.device});self.device.preview.start()

    def augment(self,state):
        actual=state['preview'];display=self.app.preview_state(self.device)
        display['server_instance_id']=self.app.server_instance_id
        display.update(capabilities=dict(target=True,overview=True,top_view=True,mesh=actual.get('capabilities',{}).get('mesh',False)),
            sources=dict(mesh=actual),mesh=actual.get('mesh',{}))
        display['frames']['mesh']=actual.get('frames',{}).get('mesh',{})
        display['revisions']['mesh']=actual.get('revisions',{}).get('mesh',0)
        for name in ('mesh_candidates','mesh_calibrations','mesh_catalog_error'):
            if name in actual:display[name]=deepcopy(actual[name])
        state['preview']=display
        for name in('target','overview'):
            state['runtime']['allowed_operations']['preview_'+name]=dict(allowed=bool(display['connected']),
                reason=None if display['connected']else display.get('error')or'独立目标显示尚未就绪')
        state['runtime']['allowed_operations']['preview_recover']=dict(allowed=False,reason='目标显示由独立CPU服务承载；不恢复设备控制')
        return state

    def action(self,name,body):
        if name not in('preview_target','preview_overview'):raise ValueError('display-only action required')
        return self.app.action(name,body)

    def media(self,query):
        if query.get('environment')!=['physical']or query.get('view')not in(['target'],['overview']):
            raise ValueError('physical target/overview display required')
        exact={}
        if 'revision'in query:
            exact=dict(revision=int(query['revision'][0]),display_context=query.get('display_context',[None])[0],
                renderer_id=query.get('renderer_id',[None])[0],view_revision=int(query.get('view_revision',['-1'])[0]),wait_ms=int(query.get('wait_ms',['0'])[0]))
        return self.app.preview_media('physical',query['view'][0],**exact)

    def close(self):self.device.preview.close()


def route(manager,handler,parsed,body):
    """Only display routes and display fields; control/camera responses pass through."""
    descriptor=manager.config.get('target_display_descriptor')
    model_manifest=manager.config.get('target_model_manifest')
    if not descriptor and not model_manifest:return False
    query=parse_qs(parsed.query,keep_blank_values=True);path=parsed.path
    post=handler.command=='POST'and path in('/api/actions/preview_target','/api/actions/preview_overview')
    read=handler.command in('GET','HEAD')
    state=read and path=='/api/state'and query.get('environment')==['physical']
    model=read and path in('/api/target-model','/api/target-model.bin')and query.get('environment')==['physical']
    media=read and path=='/api/preview-media'and query.get('environment')==['physical']and query.get('view')in(['target'],['overview'])
    if not(state or media or post or model):return False
    assets=getattr(manager,'target_model_assets',None)
    if model_manifest and assets is None:
        from .target_model import ModelAssets
        try:assets=ModelAssets(model_manifest,manager.config['device_id']);manager.target_model_assets=assets
        except (OSError,ValueError,KeyError)as error:
            manager.target_model_error=str(error)
            if model:raise ValueError('display model unavailable: '+str(error))from None
    if model:
        if assets is None:raise ValueError('GPU display model is not configured')
        if path.endswith('.bin'):
            if set(query)!={'environment','model_id'}or query['model_id']!=[assets.model_id]:raise ValueError('display model version differs')
            handler.send(200,assets.binary,'application/octet-stream',headers={'Content-Encoding':'gzip','X-Model-SHA256':assets.metadata['binary_sha256']});return True
        if set(query)!={'environment'}:raise ValueError('unknown model query')
        handler.send(200,dict(assets.metadata,**assets.public));return True
    if state:
        from .management import json_request
        value=json_request(manager.port,handler.path,timeout=5)
        if value.get('device',{}).get('id')!=manager.config['device_id']:raise ValueError('display page device differs')
    display=getattr(manager,'target_display',None)
    if descriptor and display is None:
        try:
            display=TargetDisplayGateway(descriptor,manager.config['device_id']);manager.target_display=display
        except (OSError,ValueError)as error:
            manager.target_display_error=str(error)
            if not state:raise ValueError('独立目标显示不可用：'+str(error))from None
    if state:
        if display is not None:value=display.augment(value)
        if assets is not None:
            value['preview']['webgl_model']=assets.public
            value['preview'].setdefault('capabilities',{}).update(target=True,overview=True,top_view=True)
            for name in('preview_target','preview_overview'):value['runtime']['allowed_operations'][name]=dict(allowed=True,reason=None)
        elif model_manifest:value['preview']['webgl_error']=getattr(manager,'target_model_error','display model unavailable')
        handler.send(200,value);return True
    if display is None:raise ValueError('CPU fallback display unavailable')
    if post:
        value=json.loads(body);auth=manager.backend_auth or{}
        if not isinstance(value,dict)or value.get('environment')!='physical':return False
        tokens=handler.headers.get_all('X-CSRF-Token',[])
        if len(tokens)!=1 or not auth.get('csrf')or not secrets.compare_digest(tokens[0],auth['csrf']):
            handler.send(403,{'error':'invalid display authorization'});return True
        handler.send(200,display.action(path.rsplit('/',1)[-1],value));return True
    raw,metadata=display.media(query)
    handler.send(200,raw,'image/jpeg',headers={'X-Frame-Metadata':json.dumps(metadata,allow_nan=False,separators=(',',':'))})
    return True
