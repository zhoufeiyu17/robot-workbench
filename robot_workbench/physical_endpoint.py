"""One explicitly selected physical HTTP service at the LAN gateway.

Registration only reads service state. It never deploys software, acquires an
Owner, releases a controller, changes SSH routes or sends motion commands.
"""
from copy import deepcopy
from http.client import HTTPConnection, HTTPException
from ipaddress import IPv4Address, IPv4Network
import json
import os
from pathlib import Path
import re
import secrets
import threading
import time
from urllib.parse import parse_qs, urlsplit

from .site_cameras import CameraDirectory

NETWORKS=tuple(map(IPv4Network,('10.0.0.0/8','172.16.0.0/12','192.168.0.0/16')))
PREFIX='/device-physical/'
STATE_LIMIT=2*1024*1024


def validate_endpoint(value, *, allow_loopback=False):
    if not isinstance(value,dict) or set(value)!={'address','port'}:
        raise ValueError('真机服务只接受内网 IPv4 地址和端口')
    if not isinstance(value['address'],str):raise ValueError('请输入内网 IPv4 地址文本')
    try:address=IPv4Address(value['address'])
    except (ValueError,TypeError):raise ValueError('请输入有效的内网 IPv4 地址') from None
    if (not any(address in network for network in NETWORKS)
            and not (allow_loopback and address.is_loopback)):
        raise ValueError('只允许明确的内网服务地址')
    if type(value['port']) is not int or not 1024<=value['port']<=65535:
        raise ValueError('真机服务端口须在 1024–65535 之间')
    return dict(address=str(address),port=value['port'])


def verify_state(state, *, expected_device_id=None):
    if (not isinstance(state,dict) or state.get('schema')!='robot-workbench-ui.v1'
            or not isinstance(state.get('server_instance_id'),str) or not state['server_instance_id']
            or not isinstance(state.get('device'),dict) or state['device'].get('environment')!='physical'):
        raise ValueError('目标不是兼容的物理真机工作台服务')
    device=state['device'];runtime=state.get('runtime') or {}
    if runtime.get('environment','physical')!='physical' or runtime.get('simulation_only') is True:
        raise ValueError('仿真设备状态不能作为物理真机服务接入')
    ident=device.get('id')
    if (not isinstance(ident,str) or not ident.strip() or ident=='physical-unconfigured' or ident.startswith('fake-')
            or expected_device_id is not None and ident!=expected_device_id):
        raise ValueError('真机设备身份不匹配；请核对服务主机并重新预检')
    configured=(runtime.get('connected') is True and runtime.get('mode')=='hardware'
        or (runtime.get('allowed_operations',{}).get('connect') or {}).get('allowed') is True)
    if not configured:
        raise ValueError('目标只提供占位入口或设备仍在连接收尾，尚不能确认控制适配器')
    return dict(device_id=ident,label=device.get('label') or ident,server_instance_id=state['server_instance_id'],
        connected=runtime.get('connected') is True,holder=runtime.get('holder'),state=runtime.get('state'),
        busy=bool(state.get('workflow',{}).get('busy') or runtime.get('automatic_enabled')),
        controller_acquired=False,motion_started=False)


def read_state(endpoint, *, business='calibration', expected_device_id=None, timeout=2.):
    connection=HTTPConnection(endpoint['address'],endpoint['port'],timeout=timeout)
    try:
        connection.request('GET','/api/state?environment=physical&business='+business,
                           headers={'Connection':'close','Accept':'application/json'})
        response=connection.getresponse()
        if response.status!=200:raise ValueError('真机服务返回 HTTP '+str(response.status))
        if not response.getheader('Content-Type','').startswith('application/json'):
            raise ValueError('真机服务没有返回 JSON 状态契约')
        raw=response.read(STATE_LIMIT+1)
        if len(raw)>STATE_LIMIT:raise ValueError('真机状态超过大小限制')
        state=json.loads(raw)
        return verify_state(state,expected_device_id=expected_device_id),state
    except (OSError,HTTPException) as exc:
        raise ConnectionError('真机工作台不可达，尚未停止或释放任何控制器') from exc
    finally:connection.close()


def annotate_state(state, endpoint):
    verify_state(state,expected_device_id=endpoint['device_id'])
    prefix=PREFIX+endpoint['route_id']
    def rewrite(value):
        if isinstance(value,dict):return {key:rewrite(item) for key,item in value.items()}
        if isinstance(value,list):return [rewrite(item) for item in value]
        if isinstance(value,str) and value.startswith('/api/'):
            return prefix+value
        return value
    result=rewrite(state)
    result['gateway_route']=dict(route_id=endpoint['route_id'],prefix=prefix,
        device_id=endpoint['device_id'],address=endpoint['address'],port=endpoint['port'])
    return result


class PhysicalEndpoint:
    send=staticmethod(CameraDirectory.send)

    def __init__(self,path,assets,*,allow_loopback=False):
        self.path,self.assets=Path(path),Path(assets)
        self.allow_loopback=allow_loopback
        self.lock=threading.Lock();self.mutations=threading.Lock();self.csrf=secrets.token_urlsafe(32)
        self.gateway_authority=None
        self.in_flight=0;self.changing=False
        self.current=None;self.detached=None
        if self.path.exists():
            saved=json.loads(self.path.read_text())
            if saved.get('schema')!='physical_endpoint_registry.v1':raise ValueError('unknown physical endpoint registry')
            self.current=saved.get('endpoint');self.detached=saved.get('last_detached')
            if self.current:
                validate_endpoint({k:self.current[k] for k in ('address','port')},allow_loopback=allow_loopback)
                if (not re.fullmatch('[0-9a-f]{32}',self.current.get('route_id',''))
                        or not isinstance(self.current.get('device_id'),str) or not self.current['device_id']):
                    raise ValueError('invalid persisted physical endpoint identity')

    def snapshot(self):
        with self.lock:
            return dict(endpoint=deepcopy(self.current),last_detached=deepcopy(self.detached),csrf=self.csrf,
                        scope='already_deployed_http_service_only',deployment_automatic=False)

    def _save(self,current,detached):
        with self.lock:
            if self.in_flight:raise ValueError('仍有真机请求在途，确认请求结束后再更换页面绑定')
            self.changing=True
        try:
            self.path.parent.mkdir(parents=True,exist_ok=True)
            temporary=self.path.with_name(self.path.name+'.'+secrets.token_hex(6)+'.tmp')
            with os.fdopen(os.open(temporary,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600),'w') as stream:
                json.dump(dict(schema='physical_endpoint_registry.v1',endpoint=current,last_detached=detached),stream,allow_nan=False,indent=2)
                stream.flush();os.fsync(stream.fileno())
            os.replace(temporary,self.path)
            with self.lock:self.current,self.detached=deepcopy(current),deepcopy(detached)
        finally:
            with self.lock:self.changing=False

    def begin_post(self,endpoint):
        with self.lock:
            if self.changing or self.current is None or endpoint['route_id']!=self.current['route_id']:
                raise ValueError('真机绑定已变化，请求未下发；请重新核验页面')
            self.in_flight+=1

    def end_post(self):
        with self.lock:self.in_flight-=1

    def _endpoint(self,value):
        endpoint=validate_endpoint(value,allow_loopback=self.allow_loopback)
        if f"{endpoint['address']}:{endpoint['port']}"==self.gateway_authority:
            raise ValueError('不能把当前网关设为自己的真机服务')
        return endpoint

    def change(self,body):
        if not isinstance(body,dict):raise ValueError('endpoint operation object required')
        operation=body.get('operation')
        if operation=='probe':
            if set(body)!={'operation','endpoint'}:raise ValueError('invalid endpoint probe')
            endpoint=self._endpoint(body['endpoint'])
            identity,_=read_state(endpoint)
            return dict(endpoint=endpoint,identity=identity,verified=True,controller_acquired=False)
        if operation not in ('save','remove','forget_unreachable'):raise ValueError('unknown endpoint operation')
        with self.mutations:
            current=self.snapshot()['endpoint'];old_id=current['route_id'] if current else None
            if body.get('expected_route_id')!=old_id:raise ValueError('页面绑定已变化，请刷新后重试')
            if operation=='save':
                if set(body)!={'operation','endpoint','expected_device_id','expected_route_id'}:
                    raise ValueError('save requires reviewed endpoint and device identity')
                endpoint=self._endpoint(body['endpoint'])
                identity,_=read_state(endpoint,expected_device_id=body['expected_device_id'])
                if current and all(current[key]==endpoint[key] for key in ('address','port')):
                    if identity['device_id']!=current['device_id']:raise ValueError('同一地址的设备身份已变化，请先核对旧绑定')
                    return self.snapshot()
            else:
                allowed={'operation','expected_route_id'}|({'confirmation'} if operation=='forget_unreachable' else set())
                if set(body)!=allowed:raise ValueError('invalid unbind operation')
            if current:
                try:
                    identities=[read_state(current,business=name,expected_device_id=current['device_id'])[0]
                                for name in ('calibration','collection','inference')]
                except (OSError,ValueError,ConnectionError):
                    if operation!='forget_unreachable' or body.get('confirmation')!='FORGET_UNREACHABLE_SERVICE_NOT_STOPPED':
                        raise ValueError('旧真机服务状态未知。保留原地址；如确需解除页面绑定，请明确确认远端控制不会因此停止或释放') from None
                    detached=dict(current,verification='unknown',controller_stopped=False,controller_released=False,
                                  detached_wall_ns=time.time_ns())
                    self._save(None,detached)
                    return self.snapshot()
                if operation=='forget_unreachable':raise ValueError('旧服务已可达，请按实际状态释放后正常解除绑定')
                if any(row['connected'] or row['busy'] for row in identities):
                    raise ValueError('旧真机仍连接或业务仍在进行；请先在原服务明确停止、支撑并释放，再更换地址')
            if operation=='save':
                endpoint.update(route_id=secrets.token_hex(16),device_id=identity['device_id'],label=identity['label'])
                self._save(endpoint,self.detached)
            else:self._save(None,self.detached)
            return self.snapshot()

    def resolve(self,path,method,body=None):
        parsed=urlsplit(path);query=parse_qs(parsed.query)
        prefix_match=re.fullmatch(PREFIX+r'([0-9a-f]{32})(/api/[^#]*)',parsed.path)
        endpoint=self.snapshot()['endpoint']
        if endpoint is not None:self._endpoint({key:endpoint[key] for key in ('address','port')})
        payload=None
        if body and parsed.path.startswith(('/api/actions/',PREFIX)):
            try:payload=json.loads(body)
            except (ValueError,TypeError):raise ValueError('physical actions require JSON') from None
        environment=(payload or {}).get('environment') if isinstance(payload,dict) else None
        values=query.get('environment',[])
        if len(values)>1 or values and environment is not None and values[0]!=environment:
            raise ValueError('ambiguous device environment')
        environment=environment or (values[0] if values else None)
        if prefix_match:
            if endpoint is None or prefix_match[1]!=endpoint['route_id']:
                raise ValueError('真机服务绑定已变化，旧请求已拒绝；请重新核验页面状态')
            if environment!='physical':raise ValueError('bound physical route requires physical environment')
            target=prefix_match[2]+('?' + parsed.query if parsed.query else '')
        elif parsed.path.startswith(PREFIX):
            raise ValueError('invalid bound physical route')
        elif environment=='physical' and endpoint is not None:
            if method not in ('GET','HEAD') or parsed.path!='/api/state':
                raise ValueError('真机请求缺少当前绑定身份，请刷新工作台状态')
            target=path
        else:return None
        if target.split('?',1)[0].startswith('/api/actions/') and method=='POST' and (payload or {}).get('action')!='stop' and not target.split('?',1)[0].endswith('/stop'):
            read_state(endpoint,expected_device_id=endpoint['device_id'])
        return endpoint,target

    def route(self,handler,body):
        path=urlsplit(handler.path).path
        if path=='/web/physical_endpoint.js':
            if handler.command not in ('GET','HEAD'):return self.send(handler,405,{'error':'read-only asset'})
            return self.send(handler,200,(self.assets/'physical_endpoint.js').read_bytes(),'text/javascript; charset=utf-8')
        if path!='/api/physical-endpoint':return False
        try:
            if handler.command in ('GET','HEAD'):
                value=self.snapshot()
            elif handler.command=='POST':
                if not secrets.compare_digest(handler.headers.get('X-Physical-Endpoint-Token',''),self.csrf):
                    return self.send(handler,403,{'error':'请重新读取真机地址配置后操作'})
                value=self.change(json.loads(body))
            else:return self.send(handler,405,{'error':'method not allowed'})
            return self.send(handler,200,value)
        except (ValueError,KeyError,TypeError) as exc:return self.send(handler,409,{'error':str(exc)})
        except (OSError,HTTPException) as exc:return self.send(handler,503,{'error':str(exc)})
