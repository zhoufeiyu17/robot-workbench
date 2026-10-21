"""Read-only ARX camera directory at the LAN gateway; independent of device Owners."""
from copy import deepcopy
import hashlib
from http.client import HTTPConnection, HTTPException
from ipaddress import IPv4Address, IPv4Network
import json
import math
from pathlib import Path
import re
import secrets
import threading
from urllib.parse import parse_qs, urlsplit

CAMERAS = {'base_0_rgb': '头部相机', 'left_wrist_0_rgb': '左腕相机', 'right_wrist_0_rgb': '右腕相机'}
NETWORKS = tuple(map(IPv4Network, ('10.0.0.0/8', '172.16.0.0/12', '192.168.0.0/16')))
ID = re.compile(r'[a-z0-9][a-z0-9_-]{0,63}')


def camera_lifecycle(address,port,*,start=False):
    """Fixed camera-only lifecycle endpoint; never proxy arbitrary operations."""
    connection=HTTPConnection(address,port,timeout=5)
    try:
        connection.request('POST' if start else 'GET','/start' if start else '/service',
            body=b'{}' if start else None,headers={'X-Camera-Start':'1','Content-Type':'application/json'} if start else {})
        response=connection.getresponse();raw=response.read(65537)
        if len(raw)>65536:raise ValueError('camera lifecycle response exceeds limit')
        if response.status not in (200,202):raise ValueError('图像进程管理不可用（HTTP '+str(response.status)+'）')
        value=json.loads(raw)
        if not isinstance(value,dict):raise ValueError('invalid camera lifecycle status')
        return {key:value[key] for key in ('available','phase','message','error') if key in value}
    finally:connection.close()


def read_camera(address, port, path, *, timeout=2):
    """Fixed camera resources only; no redirects, proxies, credentials or controls."""
    if path != '/streams' and path not in ('/frame/' + key + '.jpg' for key in CAMERAS):
        raise ValueError('unknown camera resource')
    connection = HTTPConnection(address, port, timeout=timeout)
    try:
        connection.request('GET', path, headers={'Connection': 'close'})
        response = connection.getresponse()
        if response.status != 200:
            raise ValueError('相机预览桥返回 HTTP ' + str(response.status))
        limit = 65536 if path == '/streams' else 2 * 1024 * 1024
        data = response.read(limit + 1)
        if len(data) > limit:
            raise ValueError('相机响应超出大小限制')
        if path == '/streams':
            value = json.loads(data)
            if not isinstance(value, dict) or not isinstance(value.get('streams'), list):
                raise ValueError('相机预览桥状态格式不匹配')
            rows = []
            for key in CAMERAS:
                row = next((r for r in value['streams'] if isinstance(r, dict) and r.get('key') == key), {})
                age, fps = row.get('last_frame_age_s'), row.get('fps')
                age = float(age) if type(age) in (int, float) and math.isfinite(age) and age >= 0 else None
                fps = float(fps) if type(fps) in (int, float) and math.isfinite(fps) and fps >= 0 else None
                status = 'live' if row.get('status') == 'live' and age is not None and age <= 1 else 'stale' if row else 'missing'
                source_state = row.get('source_state', 'unknown')
                if source_state not in ('running', 'starting', 'stopped', 'error', 'unknown'):
                    source_state = 'unknown'
                if source_state in ('stopped', 'error'):
                    status = 'stopped'
                rows.append(dict(key=key, label=CAMERAS[key], status=status, age_s=age, fps=fps, source_state=source_state))
            return rows
        if not data.startswith(b'\xff\xd8') or not response.getheader('Content-Type', '').lower().startswith('image/jpeg'):
            raise ValueError('相机预览桥未返回 JPEG')
        return data
    except HTTPException as error:
        raise OSError('相机 HTTP 响应不完整') from error
    finally:
        connection.close()


def inject_tab(html):
    """Add an independent view to either the frozen or current workbench UI."""
    marker = b'<nav class="tabs"'
    start = html.find(marker)
    if start < 0:
        return html.replace(b'</body>', '<p style="padding:16px"><a href="/site-cameras">现场相机 · 独立只读监控</a></p></body>'.encode())
    if b'data-tab="site-cameras"' not in html:
        end = html.find(b'</nav>', start)
        if end < 0:
            return html
        html = html[:end] + '<button data-tab="site-cameras">现场相机</button>'.encode() + html[end:]
    if b'src="/web/site_cameras.js"' not in html:
        html = html.replace(b'</body>', b'<script type="module" src="/web/site_cameras.js"></script></body>')
    return html


class CameraDirectory:
    def __init__(self, config, assets, *, allow_loopback=False):
        self.path, self.assets = Path(config), Path(assets)
        self.allow_loopback = allow_loopback
        self.lock = threading.RLock()
        self.csrf = secrets.token_urlsafe(32)
        value = json.loads(self.path.read_text()) if self.path.exists() else {'version': 1, 'devices': []}
        if value.get('version') != 1 or not isinstance(value.get('devices'), list):
            raise ValueError('invalid site camera directory')
        self.devices = [self.validate(row) for row in value['devices']]
        if len({row['id'] for row in self.devices}) != len(self.devices):
            raise ValueError('duplicate ARX camera identity')

    def validate(self, row):
        if not isinstance(row, dict) or set(row) - {'id', 'label', 'address', 'port'}:
            raise ValueError('设备只接受名称、内网 IPv4 和预览桥端口')
        label, address, port = row.get('label'), row.get('address'), row.get('port', 18873)
        if not isinstance(label, str) or not 1 <= len(label.strip()) <= 60 or any(ord(c) < 32 for c in label):
            raise ValueError('请输入 1–60 字的设备名称')
        try:
            ip = IPv4Address(address)
        except (ValueError, TypeError):
            raise ValueError('请输入有效的内网 IPv4 地址') from None
        if not any(ip in network for network in NETWORKS) and not (self.allow_loopback and ip.is_loopback):
            raise ValueError('仅允许内网设备地址')
        if type(port) is not int or not 1024 <= port <= 65535:
            raise ValueError('预览桥端口须为 1024–65535')
        ident = row.get('id') or secrets.token_hex(12)
        if not isinstance(ident, str) or not ID.fullmatch(ident):
            raise ValueError('invalid camera device identity')
        return dict(id=ident, label=label.strip(), address=str(ip), port=port)

    @staticmethod
    def public(row):
        return dict(row, source=hashlib.sha256((row['address'] + ':' + str(row['port'])).encode()).hexdigest()[:20])

    def save(self, body):
        if not isinstance(body, dict) or set(body) != {'operation', 'device'} or body['operation'] not in ('save', 'remove'):
            raise ValueError('invalid camera directory operation')
        with self.lock:
            rows = deepcopy(self.devices)
            if body['operation'] == 'save':
                row = self.validate(body['device'])
                if any(r['id'] != row['id'] and (r['address'], r['port']) == (row['address'], row['port']) for r in rows):
                    raise ValueError('该地址已在设备列表中，请编辑已有设备')
                rows = [r for r in rows if r['id'] != row['id']] + [row]
                if len(rows) > 64:
                    raise ValueError('现场相机最多配置 64 台设备')
            else:
                if not isinstance(body['device'], dict) or set(body['device']) != {'id'}:
                    raise ValueError('remove requires one device identity')
                rows = [r for r in rows if r['id'] != body['device']['id']]
            self.path.parent.mkdir(parents=True, exist_ok=True)
            temporary = self.path.with_name(self.path.name + '.tmp')
            temporary.write_text(json.dumps({'version': 1, 'devices': rows}, ensure_ascii=False, indent=2) + '\n')
            temporary.replace(self.path)
            self.devices = rows

    def route(self, handler, body):
        parsed = urlsplit(handler.path)
        path = parsed.path
        if path == '/web/site_cameras.js':
            if handler.command not in ('GET', 'HEAD'):
                return self.send(handler, 405, {'error': 'read-only resource'})
            return self.send(handler, 200, (self.assets / 'site_cameras.js').read_bytes(), 'text/javascript; charset=utf-8')
        if path == '/site-cameras':
            if handler.command not in ('GET', 'HEAD'):
                return self.send(handler, 405, {'error': 'read-only resource'})
            html = '<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>统一工作台 · 现场相机</title><body><main><nav class="tabs"><a href="/">工作台主页</a><button data-tab="site-cameras" aria-current="page">现场相机</button></nav></main><script type="module" src="/web/site_cameras.js"></script></body></html>'
            return self.send(handler, 200, html.encode(), 'text/html; charset=utf-8')
        if not path.startswith('/api/site-cameras'):
            return False
        try:
            if path == '/api/site-cameras':
                if handler.command == 'POST':
                    if not secrets.compare_digest(handler.headers.get('X-Site-Camera-Token', ''), self.csrf):
                        return self.send(handler, 403, {'error': '请重新读取设备列表后再保存'})
                    self.save(json.loads(body))
                elif handler.command not in ('GET', 'HEAD'):
                    return self.send(handler, 405, {'error': 'method not allowed'})
                with self.lock:
                    value = dict(devices=[self.public(r) for r in self.devices], csrf=self.csrf)
                return self.send(handler, 200, value)
            start_match=re.fullmatch(r'/api/site-cameras/([a-z0-9_-]+)/start',path)
            if start_match and handler.command=='POST':
                if not secrets.compare_digest(handler.headers.get('X-Site-Camera-Token',''),self.csrf):
                    return self.send(handler,403,dict(error='请重新读取设备列表后再启动图像'))
                request=json.loads(body)
                with self.lock:
                    row=next((self.public(r) for r in self.devices if r['id']==start_match[1]),None)
                    if row is None:return self.send(handler,404,dict(error='监控设备已移除'))
                    if request!={'source':row['source']}:
                        return self.send(handler,409,dict(error='设备地址已变化，请重新读取后启动'))
                    result=camera_lifecycle(row['address'],row['port'],start=True)
                return self.send(handler,202,result)
            if handler.command not in ('GET', 'HEAD'):
                return self.send(handler, 405, {'error': 'camera view is read-only'})
            match = re.fullmatch(r'/api/site-cameras/([a-z0-9_-]+)/(?:(streams)|(frame)/([a-z0-9_]+)\.jpg)', path)
            if not match:
                return self.send(handler, 404, {'error': 'unknown camera resource'})
            with self.lock:
                row = next((self.public(r) for r in self.devices if r['id'] == match[1]), None)
            if row is None:
                return self.send(handler, 404, {'error': '设备配置已移除，请重新读取列表'})
            if match[2]:
                try:
                    streams = read_camera(row['address'], row['port'], '/streams')
                    value = dict(device=row, reachable=True, streams=streams)
                    try:value['lifecycle']=camera_lifecycle(row['address'],row['port'])
                    except (OSError,ValueError,HTTPException):value['lifecycle']=dict(available=False)
                except (OSError, ValueError) as error:
                    value = dict(device=row, reachable=False, streams=[], error='相机预览桥不可用：' + str(error))
                return self.send(handler, 200, value)
            key = match[4]
            if key not in CAMERAS:
                return self.send(handler, 404, {'error': 'unknown camera'})
            if parse_qs(parsed.query).get('source') != [row['source']]:
                return self.send(handler, 409, {'error': '设备地址已变化，请重新读取相机状态'})
            data = read_camera(row['address'], row['port'], '/frame/' + key + '.jpg')
            return self.send(handler, 200, data, 'image/jpeg', {'X-Camera-Source': row['source']})
        except (ValueError, KeyError, TypeError) as error:
            return self.send(handler, 400, {'error': str(error)})
        except OSError as error:
            return self.send(handler, 503, {'error': '相机暂不可用：' + str(error)})

    @staticmethod
    def send(handler, status, value, kind='application/json', headers=None):
        raw = json.dumps(value, ensure_ascii=False, allow_nan=False).encode() if kind == 'application/json' else value
        handler.send_response(status)
        handler.send_header('Content-Type', kind)
        handler.send_header('Content-Length', str(len(raw)))
        handler.send_header('Cache-Control', 'no-store')
        handler.send_header('X-Content-Type-Options', 'nosniff')
        for key, value in (headers or {}).items():
            handler.send_header(key, value)
        handler.end_headers()
        if handler.command != 'HEAD':
            try:
                handler.wfile.write(raw)
            except OSError:
                pass
        return True
