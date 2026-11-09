#!/usr/bin/env python3
"""A fixed-loopback upstream for the workbench's explicit LAN entry."""
import argparse
from http.client import HTTPConnection, HTTPException, IncompleteRead
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from ipaddress import IPv4Address
import json
from pathlib import Path
import re
import secrets
from urllib.parse import parse_qs,urlsplit

BODY_LIMIT=4*1024*1024
CHUNK_SIZE=65536
HOP_HEADERS={'connection','keep-alive','proxy-authenticate','proxy-authorization',
             'te','trailer','transfer-encoding','upgrade','proxy-connection'}


def hop_headers(headers):
    return HOP_HEADERS|{name.strip().lower() for value in headers.get_all('Connection',[])
                       for name in value.split(',') if name.strip()}


def serve(*,listen='10.17.0.27',port=18866,upstream_port=18867,
          upstream_authority='127.0.0.1:43759',upstream_timeout=15,
          camera_config=None,camera_assets=None,physical_config=None,physical_assets=None,
          workbench_assets=None,manager_profiles=None):
    address=IPv4Address(listen)
    if address.is_unspecified or address.is_multicast or not (address.is_private or address.is_loopback):
        raise ValueError('listen must be one explicit private or loopback IPv4 address')
    if type(port) is not int or not 0<=port<=65535 or type(upstream_port) is not int or not 1<=upstream_port<=65535:
        raise ValueError('invalid listen or upstream port')
    host,separator,authority_port=upstream_authority.rpartition(':')
    if (host!='127.0.0.1' or not separator or not authority_port.isascii() or not authority_port.isdigit()
            or not 1<=int(authority_port)<=65535 or str(int(authority_port))!=authority_port):
        raise ValueError('upstream authority must be 127.0.0.1:<port>')
    profiles=None
    if manager_profiles is not None:
        import sys
        sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
        from robot_workbench.environment import validate_profiles
        path=Path(manager_profiles)
        if not path.is_absolute() or path.is_symlink() or not path.is_file() or path.stat().st_size>65536:
            raise ValueError('managed gateway requires a bounded trusted profiles file')
        rows=validate_profiles(json.loads(path.read_text()))
        if not rows or len({row['environment'] for row in rows})!=len(rows) or any(row.get('configured') is not True for row in rows):
            raise ValueError('exactly one configured manager per registered environment required')
        if workbench_assets is not None or physical_assets is not None:
            raise ValueError('managed gateway assets come from each manager; remove workbench-assets and physical-assets overrides')
        profiles={row['environment']:row for row in rows}
    cameras=None
    if camera_config is not None:
        import sys
        sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
        from robot_workbench.site_cameras import CameraDirectory
        cameras=CameraDirectory(camera_config,camera_assets or Path(__file__).resolve().parents[1]/'web',allow_loopback=address.is_loopback)
    physical=None
    if physical_config is not None:
        import sys
        sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
        from robot_workbench.physical_endpoint import PhysicalEndpoint
        physical=PhysicalEndpoint(physical_config,physical_assets or Path(__file__).resolve().parents[1]/'web',allow_loopback=address.is_loopback)
    assets={}
    if workbench_assets is not None:
        import sys
        sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
        from robot_workbench.site_cameras import CameraDirectory
        root=Path(workbench_assets)
        if not root.is_absolute() or not root.is_dir():
            raise ValueError('workbench assets require an absolute existing directory')
        root=root.resolve()
        for name in ('index.html','app.js','style.css','session.js','site_cameras.js','physical_endpoint.js'):
            file=root/name
            if not file.is_file() or file.is_symlink() or file.stat().st_size>BODY_LIMIT:
                raise ValueError('workbench assets require bounded regular files: '+name)
            kind=('text/html' if name.endswith('.html') else 'text/css' if name.endswith('.css') else 'text/javascript')+'; charset=utf-8'
            assets['/web/'+name]=(file.read_bytes(),kind)
        assets['/']=assets['/index.html']=assets['/web/index.html']

    def manager_route(environment):
        if environment not in profiles:raise ValueError('所选环境尚未登记兼容管理服务')
        if environment=='fake':return '127.0.0.1',upstream_port,upstream_authority
        endpoint=physical.snapshot()['endpoint'] if physical is not None else None
        if (not endpoint or endpoint['device_id']!=profiles[environment]['device_id']
                or profiles[environment].get('robot_address') not in (None,'',endpoint['address'])):
            raise ValueError('真机管理服务与既有绑定身份不匹配；未改变绑定')
        return endpoint['address'],endpoint['port'],f"{endpoint['address']}:{endpoint['port']}"

    def read_manager(environment):
        destination,port,authority=manager_route(environment)
        connection=HTTPConnection(destination,port,timeout=min(3,upstream_timeout))
        try:
            connection.request('GET','/api/environment?environment='+environment,headers={'Host':authority,'Connection':'close'})
            response=connection.getresponse();raw=response.read(BODY_LIMIT+1)
            if response.status!=200 or len(raw)>BODY_LIMIT:raise ValueError('所选目标尚未提供兼容的统一运维接口')
            value=json.loads(raw)
            if not isinstance(value,dict) or not isinstance(value.get('target'),dict) or not isinstance(value.get('deployment'),dict):
                raise ValueError('管理服务未返回受支持的环境对象')
            deployment=value['deployment'].get('sha256')
            if (value.get('schema')!='robot_workbench.environment.v1' or not isinstance(value.get('operations'),dict)
                    or value.get('target',{}).get('environment')!=environment
                    or value['target'].get('device_id')!=profiles[environment]['device_id']
                    or value.get('selected_profile_id')!=profiles[environment]['id']
                    or not isinstance(value.get('csrf'),str) or not value['csrf']
                    or not isinstance(deployment,str) or not re.fullmatch('[a-f0-9]{64}',deployment)
                    or response.getheader('X-Workbench-Deployment')!=deployment):
                raise ValueError('管理服务schema、设备、档案或部署版本不匹配；未接入控制')
            return value
        finally:connection.close()

    def public_state(value):
        value['profiles']=[dict(row,manager_url='/') for row in profiles.values()]
        value['gateway_routing']=dict(schema='robot_workbench.gateway-routing.v1',
            environment=value['target']['environment'],asset_prefix='/manager/'+value['target']['environment']+'/web/')
        return value

    def managed_physical_route(path,method,body):
        parsed=urlsplit(path);query=parse_qs(parsed.query);endpoint=physical.snapshot()['endpoint']
        prefix=re.fullmatch(r'/device-physical/([a-f0-9]{32})(/api/[^#]*)',parsed.path)
        payload=json.loads(body) if body and parsed.path.startswith(('/api/actions/','/device-physical/')) else {}
        environment=payload.get('environment') if isinstance(payload,dict) else None
        values=query.get('environment',[])
        if len(values)>1 or values and environment is not None and values[0]!=environment:raise ValueError('ambiguous device environment')
        environment=environment or (values[0] if values else None)
        if prefix:
            if endpoint is None or prefix[1]!=endpoint['route_id'] or environment!='physical':
                raise ValueError('真机绑定身份或环境已变化，请重新读取状态')
            target=prefix[2]+('?' + parsed.query if parsed.query else '')
        elif parsed.path.startswith('/device-physical/'):raise ValueError('invalid bound physical route')
        elif environment=='physical':
            if method not in ('GET','HEAD') or parsed.path!='/api/state':raise ValueError('真机请求缺少当前绑定身份')
            target=path
        else:return None
        manager_route('physical')  # Existing registry, never caller-supplied network coordinates.
        return endpoint,target

    def managed_physical_state(value,endpoint):
        if (not isinstance(value,dict) or not isinstance(value.get('device'),dict) or not isinstance(value.get('runtime'),dict)
                or value.get('schema')!='robot-workbench-ui.v1' or not isinstance(value.get('server_instance_id'),str) or not value['server_instance_id']
                or value.get('device',{}).get('id')!=endpoint['device_id']
                or value['device'].get('environment')!='physical'
                or value['runtime'].get('environment','physical')!='physical'
                or value.get('runtime',{}).get('simulation_only') is True):
            raise ValueError('业务状态与已登记真机身份不匹配')
        prefix='/device-physical/'+endpoint['route_id']
        def rewrite(item):
            if isinstance(item,dict):return {key:rewrite(row) for key,row in item.items()}
            if isinstance(item,list):return [rewrite(row) for row in item]
            return prefix+item if isinstance(item,str) and item.startswith('/api/') else item
        result=rewrite(value);result['gateway_route']=dict(endpoint,prefix=prefix)
        return result

    class Handler(BaseHTTPRequestHandler):
        protocol_version='HTTP/1.1';timeout=5;disable_nagle_algorithm=True
        def log_message(self,*args):pass
        def reply(self,code,value,*,deployment=None):
            raw=json.dumps(value,allow_nan=False,separators=(',',':')).encode()
            self.close_connection=True
            self.send_response(code);self.send_header('Content-Type','application/json')
            if deployment:self.send_header('X-Workbench-Deployment',deployment)
            self.send_header('Content-Length',str(len(raw)));self.send_header('Cache-Control','no-store')
            self.send_header('Connection','close');self.end_headers()
            if getattr(self,'command',None)!='HEAD':
                try:self.wfile.write(raw)
                except OSError:pass
        def send_error(self,code,message=None,explain=None):
            self.reply(405 if code==501 else code,{'error':'method not allowed' if code==501 else 'invalid HTTP request'})
        def handle_expect_100(self):
            self.reply(417,{'error':'Expect is not supported'});return False
        def valid_request(self):
            hosts=self.headers.get_all('Host',[]);origins=self.headers.get_all('Origin',[])
            if hosts!=[self.server.public_authority] or len(origins)>1 or origins and origins!=['http://'+hosts[0]]:
                self.reply(403,{'error':'invalid Host or Origin'});return False
            # BaseHTTPRequestHandler normalizes leading //; inspect the original
            # target so an absolute/proxy-form request cannot hide that prefix.
            parts=self.requestline.split()
            target=parts[1] if len(parts)==3 else ''
            try:parsed=urlsplit(target)
            except ValueError:
                self.reply(400,{'error':'invalid request target'});return False
            if (not target.startswith('/') or target.startswith('//') or '\\' in target or not target.isascii()
                    or parsed.scheme or parsed.netloc or parsed.fragment):
                self.reply(400,{'error':'origin-form request target required'});return False
            if self.headers.get_all('Transfer-Encoding',[]) or self.headers.get_all('Expect',[]):
                self.reply(400,{'error':'Transfer-Encoding and Expect are not supported'});return False
            nominated=hop_headers(self.headers)-HOP_HEADERS
            if nominated&{'host','origin','content-length','x-csrf-token','x-physical-endpoint-token'}:
                self.reply(400,{'error':'request identity or framing cannot be hop-by-hop'});return False
            return True
        def health(self):
            connection=HTTPConnection('127.0.0.1',upstream_port,timeout=min(2,upstream_timeout))
            response=None
            try:
                connection.request('GET','/api/session',headers={'Host':upstream_authority,'Connection':'close'})
                response=connection.getresponse()
                self.reply(200 if response.status==200 else 503,dict(gateway='ready',upstream_reachable=True,upstream_http_status=response.status))
            except (OSError,HTTPException):
                self.reply(503,dict(gateway='ready',upstream_reachable=False,upstream_http_status=None))
            finally:
                if response is not None:response.close()
                connection.close()
        def managed_proxy(self,body):
            parsed=urlsplit(self.path);path=parsed.path;query=parse_qs(parsed.query,keep_blank_values=True)
            asset=re.fullmatch(r'/manager/(fake|physical)(/web/[A-Za-z0-9_-]+\.(?:js|mjs|css|html|svg|png|jpg|ico))',path)
            pages=('/', '/index.html','/operations','/session','/web/index.html')
            operation=path.startswith('/api/operations')
            if path in ('/api/session/start','/api/session/stop') and self.command=='POST':
                self.reply(410,dict(error='旧入口只读；启停统一由服务运维执行',code='operation_required'));return True
            if path=='/api/physical-endpoint' and self.command=='POST':
                self.reply(410,dict(error='统一运维模式不修改全局设备绑定，请检查已登记环境档案',code='operation_required'));return True
            if path.startswith('/manager/') and not asset:
                self.reply(404,{'error':'unknown environment asset'});return True
            if path.startswith('/web/') and path not in pages and not asset:
                if path=='/web/site_cameras.js':return False
                self.reply(409,{'error':'请从带环境的工作台入口重新加载，静态资源不得跨管理器混用'});return True
            if not (asset or path in pages or path=='/api/environment' or operation):return False
            try:
                values=query.get('environment',[])
                if len(values)>1 or values and values[0] not in ('fake','physical'):
                    raise ValueError('明确的单一environment参数必需')
                environment=asset[1] if asset else values[0] if values else 'fake'
                if asset and values and values!=[environment]:raise ValueError('资源环境与查询不匹配')
                if operation and not values:raise ValueError('运维请求必须明确environment')
                if (operation or path=='/api/environment') and set(query)-{'environment'}:raise ValueError('unknown management query')
                if (asset or path in pages) and self.command not in ('GET','HEAD'):
                    self.reply(405,{'error':'environment assets are read-only'});return True
                destination,destination_port,authority=manager_route(environment)
                if path=='/api/environment':
                    if self.command not in ('GET','HEAD'):raise ValueError('环境事实仅支持只读请求')
                    state=public_state(read_manager(environment))
                    self.reply(200,state,deployment=state['deployment']['sha256']);return True
                if operation and self.command=='POST' and path!='/api/operations/stop':
                    if self.headers.get('Content-Type','').split(';')[0].strip().lower()!='application/json':
                        raise ValueError('JSON operation body required')
                    value=json.loads(body)
                    if not isinstance(value,dict):raise ValueError('operation object required')
                    state=read_manager(environment)
                    tokens=self.headers.get_all('X-CSRF-Token',[])
                    if len(tokens)!=1 or not secrets.compare_digest(tokens[0],state['csrf']):
                        self.reply(403,{'error':'invalid manager token'});return True
                    if not(path=='/api/operations/action' and value.get('action')=='status_check'):
                        if self.headers.get_all('X-Workbench-Deployment',[])!=[state['deployment']['sha256']]:
                            raise ValueError('页面与当前管理器版本不匹配，请重新加载；独立Stop仍可请求')
                    if path=='/api/operations/target':
                        if (set(value)-{'profile_id','robot_address','compute_address','expected_target'}
                                or value.get('expected_target')!=state['target']):
                            raise ValueError('当前环境身份已变化，请重新核验')
                        selected=next((row for row in profiles.values() if row['id']==value.get('profile_id')),None)
                        if selected is None or any(value.get(field) and value[field]!=selected.get(field) for field in ('robot_address','compute_address')):
                            raise ValueError('地址未匹配唯一已登记档案；不会猜测端口或创建路由')
                        if selected['environment']==environment:
                            self.reply(200,public_state(state),deployment=state['deployment']['sha256']);return True
                        health=state.get('health') or {};runtime=state.get('runtime') or {}
                        workbench=(health.get('services') or {}).get('workbench') or {};recovery=state.get('recovery') or {}
                        stopped=(health.get('backend_reachable') is False and workbench.get('state')=='stopped'
                            and workbench.get('running') is False and recovery.get('required') is False)
                        if (state.get('active_job') or health.get('maintenance') or state.get('maintenance')
                                or runtime.get('maintenance_active') or recovery.get('required')
                                or runtime.get('connected') is True or health.get('busy') is True
                                or not stopped and (runtime.get('connected') is not False or health.get('busy') is not False)):
                            raise ValueError('请先核验、停止任务并在运维释放当前设备，再切换环境；不会自动释放')
                        target=public_state(read_manager(selected['environment']))
                        self.reply(200,dict(redirect_url='http://'+self.server.public_authority+'/?environment='+selected['environment'],
                            environment=target,controller_acquired=False),deployment=state['deployment']['sha256']);return True
                target=(asset[2] if asset else path)+('?' + parsed.query if parsed.query else '')
                connection=HTTPConnection(destination,destination_port,timeout=upstream_timeout)
                try:
                    headers={k:v for k,v in self.headers.items() if k.lower() not in hop_headers(self.headers)|{'host','origin'}}
                    headers.update(Host=authority,Connection='close')
                    if self.headers.get('Origin'):headers['Origin']='http://'+authority
                    connection.request('GET' if self.command=='HEAD' else self.command,target,body,headers)
                    response=connection.getresponse();raw=response.read(BODY_LIMIT+1)
                    if len(raw)>BODY_LIMIT:raise ValueError('manager response exceeds bounded size')
                    deployment=response.getheader('X-Workbench-Deployment','')
                    if response.status==200 and not re.fullmatch('[a-f0-9]{64}',deployment):
                        raise ValueError('目标未返回受支持的部署版本，请核验管理服务')
                    kind=response.getheader('Content-Type','application/json')
                    if path in pages and response.status==200:
                        if not kind.startswith('text/html'):raise ValueError('管理服务页面契约不匹配')
                        if cameras is not None:
                            from robot_workbench.site_cameras import inject_tab
                            raw=inject_tab(raw)
                        raw=raw.replace(b'/web/',('/manager/'+environment+'/web/').encode())
                    if asset and response.status==200 and b'__WORKBENCH_DEPLOYMENT_SHA__' in raw:
                        raise ValueError('静态页面版本尚未由对应管理器绑定，请重新部署')
                    self.send_response(response.status);self.send_header('Content-Type',kind)
                    self.send_header('Content-Length',str(len(raw)));self.send_header('Cache-Control','no-store')
                    self.send_header('X-Content-Type-Options','nosniff');self.send_header('Connection','close')
                    if re.fullmatch('[a-f0-9]{64}',deployment):self.send_header('X-Workbench-Deployment',deployment)
                    self.end_headers();self.close_connection=True
                    if self.command!='HEAD':
                        try:self.wfile.write(raw)
                        except (BrokenPipeError,ConnectionResetError):pass
                finally:connection.close()
                return True
            except (OSError,HTTPException):self.reply(503,{'error':'所选管理服务不可达；未切换设备或启动服务'})
            except (ValueError,TypeError,KeyError) as error:self.reply(409,{'error':str(error)})
            return True
        def proxy(self,body=None):
            if profiles is not None and self.managed_proxy(body):return
            if cameras is not None and cameras.route(self,body):return
            if physical is not None and physical.route(self,body):return
            path=urlsplit(self.path).path
            if assets and (path in assets or path.startswith('/web/')):
                if self.command not in ('GET','HEAD'):
                    return self.reply(405,{'error':'workbench assets are read-only'})
                if path not in assets:return self.reply(404,{'error':'unknown workbench asset'})
                raw,kind=assets[path]
                return CameraDirectory.send(self,200,raw,kind)
            if urlsplit(self.path).path=='/healthz':
                return self.health() if self.command in ('GET','HEAD') else self.reply(405,{'error':'healthz is read-only'})
            destination,target,authority,route='127.0.0.1',self.path,upstream_authority,None
            destination_port=upstream_port
            try:
                if physical is not None:route=(managed_physical_route(self.path,self.command,body)
                    if profiles is not None else physical.resolve(self.path,self.command,body))
            except (OSError,HTTPException):return self.reply(503,{'error':'真机服务无法核验；未发送控制命令'})
            except (ValueError,TypeError,KeyError) as error:return self.reply(409,{'error':str(error)})
            if route is not None:
                endpoint,target=route
                destination,destination_port=endpoint['address'],endpoint['port']
                authority=f'{destination}:{destination_port}'
            if profiles is not None and self.command=='POST' and not urlsplit(target).path.endswith('/stop'):
                try:
                    state=read_manager('physical' if route is not None else 'fake')
                    if self.headers.get_all('X-Workbench-Deployment',[])!=[state['deployment']['sha256']]:
                        raise ValueError('业务页面与对应管理器版本不匹配，请重新加载')
                except (OSError,HTTPException):return self.reply(503,{'error':'管理服务无法核验，未提交业务请求'})
                except (ValueError,TypeError,KeyError) as error:return self.reply(409,{'error':str(error)})
            physical_post=route is not None and self.command=='POST'
            if physical_post:
                try:physical.begin_post(endpoint)
                except ValueError as error:return self.reply(409,{'error':str(error)})
            connection=HTTPConnection(destination,destination_port,timeout=upstream_timeout)
            started=False;response=None
            try:
                # The frozen manager implements GET/POST only. HEAD borrows the
                # same read-only GET headers and closes without relaying its body.
                connection.putrequest('GET' if self.command=='HEAD' else self.command,target,skip_host=True,skip_accept_encoding=True)
                blocked=hop_headers(self.headers)|{'host','origin'}
                for name,value in self.headers.items():
                    if name.lower() not in blocked:connection.putheader(name,value)
                connection.putheader('Host',authority)
                if self.headers.get_all('Origin',[]):connection.putheader('Origin','http://'+authority)
                connection.putheader('Connection','close');connection.endheaders(body)
                response=connection.getresponse()
                lengths=response.headers.get_all('Content-Length',[])
                transfers=response.headers.get_all('Transfer-Encoding',[])
                if (len(lengths)>1 or lengths and (not lengths[0].isascii() or not lengths[0].isdigit())
                        or transfers and (len(transfers)!=1 or transfers[0].lower()!='chunked' or lengths)
                        or response.status<200):
                    raise HTTPException('invalid upstream response framing')
                if route is not None and response.status==200 and urlsplit(target).path=='/api/state':
                    from robot_workbench.physical_endpoint import annotate_state,STATE_LIMIT
                    raw=response.read(STATE_LIMIT+1)
                    if len(raw)>STATE_LIMIT or lengths and len(raw)!=int(lengths[0]):
                        raise HTTPException('invalid physical state response')
                    value=managed_physical_state(json.loads(raw),endpoint) if profiles is not None else annotate_state(json.loads(raw),endpoint)
                    return physical.send(self,200,value)
                if ((cameras is not None or physical is not None) and response.status==200 and self.command in ('GET','HEAD')
                        and urlsplit(self.path).path in ('/','/index.html','/web/index.html','/session')
                        and response.getheader('Content-Type','').startswith('text/html')):
                    raw=response.read(1024*1024+1)
                    if len(raw)>1024*1024 or lengths and len(raw)!=int(lengths[0]):
                        raise HTTPException('invalid workbench HTML response')
                    if cameras is not None:
                        from robot_workbench.site_cameras import inject_tab
                        raw=inject_tab(raw)
                    if physical is not None and b'src="/web/physical_endpoint.js"' not in raw:
                        raw=raw.replace(b'</body>',b'<script type="module" src="/web/physical_endpoint.js"></script></body>')
                    return (cameras or physical).send(self,200,raw,'text/html; charset=utf-8')
                length=int(lengths[0]) if lengths else None
                no_body=self.command=='HEAD' or response.status in (204,304)
                chunked=length is None and not no_body
                self.send_response_only(response.status);started=True
                blocked=hop_headers(response.headers)|{'content-length'}
                for name,value in response.getheaders():
                    if name.lower() not in blocked:self.send_header(name,value)
                if length is not None:self.send_header('Content-Length',str(length))
                if chunked:self.send_header('Transfer-Encoding','chunked')
                if self.close_connection:self.send_header('Connection','close')
                self.end_headers()
                if not no_body:
                    sent=0
                    while data:=response.read(CHUNK_SIZE):
                        if chunked:self.wfile.write(f'{len(data):x}\r\n'.encode())
                        self.wfile.write(data)
                        if chunked:self.wfile.write(b'\r\n')
                        sent+=len(data)
                    if length is not None and sent!=length:raise IncompleteRead(b'',length-sent)
                    if chunked:self.wfile.write(b'0\r\n\r\n')
            except (OSError,HTTPException,ValueError) as error:
                self.close_connection=True
                if not started:self.reply(503 if isinstance(error,OSError) else 502,{'error':'workbench upstream unavailable'})
                # Headers already sent: leave the body incomplete; never append
                # a success terminator or disguise an upstream failure as JSON.
            finally:
                if response is not None:response.close()
                connection.close()
                if physical_post:physical.end_post()
        def read_only(self):
            if not self.valid_request():return
            if self.headers.get_all('Content-Length',[]) not in ([],['0']):
                return self.reply(400,{'error':'GET and HEAD bodies are not supported'})
            self.proxy()
        do_GET=read_only
        do_HEAD=read_only
        def do_POST(self):
            keep_alive=not self.close_connection;self.close_connection=True
            if not self.valid_request():return
            lengths=self.headers.get_all('Content-Length',[])
            if len(lengths)!=1 or not lengths[0].isascii() or not lengths[0].isdigit():
                return self.reply(400,{'error':'one explicit Content-Length is required'})
            try:size=int(lengths[0])
            except ValueError:return self.reply(400,{'error':'invalid Content-Length'})
            if not 0<size<=BODY_LIMIT:return self.reply(413,{'error':'request body must be between 1 byte and 4 MiB'})
            try:body=self.rfile.read(size)
            except OSError:return self.reply(408,{'error':'request body timed out'})
            if len(body)!=size:return self.reply(400,{'error':'incomplete request body'})
            self.close_connection=not keep_alive;self.proxy(body)

    server=ThreadingHTTPServer((str(address),port),Handler);server.daemon_threads=True
    server.public_authority=f'{address}:{server.server_port}'
    if physical is not None:physical.gateway_authority=server.public_authority
    if address.is_loopback and server.server_port==upstream_port:
        server.server_close();raise ValueError('gateway cannot proxy to itself')
    return server


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--listen',default='10.17.0.27')
    parser.add_argument('--port',type=int,default=18866)
    parser.add_argument('--upstream-port',type=int,default=18867)
    parser.add_argument('--upstream-authority',default='127.0.0.1:43759')
    parser.add_argument('--camera-config',type=Path)
    parser.add_argument('--camera-assets',type=Path)
    parser.add_argument('--physical-config',type=Path,help='persistent registration of an explicitly selected existing physical HTTP service')
    parser.add_argument('--physical-assets',type=Path)
    parser.add_argument('--workbench-assets',type=Path,help='frozen shared frontend directory, independent of the compute backend')
    parser.add_argument('--manager-profiles',type=Path,help='trusted fake/physical profiles; manager-bound assets and read-only target discovery')
    args=parser.parse_args();server=serve(**vars(args))
    print('Workbench LAN gateway: http://'+server.public_authority,flush=True)
    try:server.serve_forever()
    except KeyboardInterrupt:pass
    finally:server.server_close()
