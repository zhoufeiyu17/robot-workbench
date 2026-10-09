"""Existing business transports bound to the same managed device, no SDK imports."""
from copy import deepcopy
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
import json
import re
import secrets
import threading
import time


def calibration_gateway(device):
    from arx_env.joint_head_workbench_control import Bridge
    class SharedBridge(Bridge):
        def request(self,path,body=None):
            if path=='/state' and body is None:return device.snapshot('calibration')
            if path=='/operator' and body is not None:return device.submit('calibration',body)
            raise ValueError('unknown shared owner request')

        def acquire(self,body):
            if body!={'confirmation':'ARMS_SUPPORTED_CONNECT'}:
                raise ValueError('explicit supported-arms confirmation required')
            return device.connect({'arms_supported':True})

        def takeover_preview(self,body):
            raise ValueError('请在统一页显式交接控制权限；不停止或替换其他控制器')

        def takeover_confirm(self,body):
            raise ValueError('旧进程接管入口已停用；请在统一页显式交接控制权限')

        def status(self):
            result=super().status()
            if device.runtime and device.runtime.holder!='calibration':
                result.update(motion_enabled=False,motion_disabled_reason='请先交接控制权限：当前由 '+device.runtime.holder+' 持有')
            return result
    return SharedBridge()


def start_control_gateway(device, token, *, port=0):
    """One loopback transport exposes existing C gateway and D owner shapes.

    GET /state is local read-only feedback for the legacy calibration adapter.
    Every write requires a private registered business credential. Operator UI
    credentials are separate and cannot call this transport.
    """
    bridge=calibration_gateway(device)
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def send(self,code,value):
            raw=json.dumps(value,ensure_ascii=False,allow_nan=False).encode()
            self.send_response(code);self.send_header('Content-Type','application/json')
            self.send_header('Content-Length',str(len(raw)));self.end_headers()
            try:self.wfile.write(raw)
            except (BrokenPipeError,ConnectionResetError):pass
        def host_ok(self):
            return self.headers.get('Host') in (f'127.0.0.1:{self.server.server_port}',f'localhost:{self.server.server_port}')
        def business(self):
            value=self.headers.get('Authorization','').removeprefix('Bearer ')
            for business,secret in token.items():
                if secrets.compare_digest(value,secret):return business
            raise PermissionError('private business credential required')
        def do_GET(self):
            if not self.host_ok():return self.send(403,{'error':'invalid host'})
            try:
                if self.path=='/state':return self.send(200,device.snapshot('collection'))
                if self.business()!='calibration':raise PermissionError('calibration route requires calibration identity')
                if self.path=='/api/state':return self.send(200,bridge.status())
                if self.path=='/api/feedback':return self.send(200,bridge.feedback())
                self.send(404,{'error':'not found'})
            except PermissionError as error:self.send(403,{'error':str(error)})
            except Exception as error:self.send(409,{'error':str(error)})
        def do_POST(self):
            if not self.host_ok():return self.send(403,{'error':'invalid host'})
            try:
                business=self.business()
                length=int(self.headers.get('Content-Length','0'))
                if not 0<length<=4*1024*1024:raise ValueError('small JSON body required')
                body=json.loads(self.rfile.read(length))
                if self.path=='/operator' and business=='collection':
                    if body.get('action')=='acquire':
                        if body!={'action':'acquire','confirmation':'ARMS_SUPPORTED_CONNECT'}:
                            raise ValueError('explicit supported-arms confirmation required')
                        result=device.connect({'arms_supported':True})
                    else:result=device.submit(business,body)
                elif business=='calibration':result=bridge.command(self.path,body)
                else:raise PermissionError('business route mismatch')
                self.send(200,result)
            except PermissionError as error:self.send(403,{'error':str(error)})
            except Exception as error:self.send(409,{'error':str(error)})
    if set(token)!={'calibration','collection'} or len(set(token.values()))!=2 or any(len(t)<32 for t in token.values()):
        raise ValueError('two distinct private business credentials required')
    server=ThreadingHTTPServer(('127.0.0.1',port),Handler);server.daemon_threads=True
    thread=threading.Thread(target=server.serve_forever,daemon=True,name='shared-control-gateway')
    thread.start()
    return server,thread


class CameraMedia:
    """Read an existing shared producer; preview never starts recording or sensors."""
    def __init__(self,name,label,read_frame,*,environment,kind='actual'):
        self.name,self.label,self.read_frame=name,label,read_frame
        self.environment,self.kind=environment,kind
        self.last=dict(ready=False,frame_id=None,source_wall_ns=None)
        self.lock=threading.Lock()
        self.raw=None
        self.closed=threading.Event()
        self.thread=threading.Thread(target=self.monitor,daemon=True,name='media-'+name)
        self.thread.start()

    def refresh(self):
        raw,metadata=self.read_frame()
        frame_id=metadata.get('pair_id',metadata.get('frame_number'))
        stamp=metadata.get('host_receive_ns',metadata.get('frame_timestamp_ns'))
        if frame_id is None or stamp is None:raise ValueError('frame identity and source timestamp required')
        if not re.fullmatch(r'[A-Za-z0-9_.:-]{1,200}',str(frame_id)) or type(stamp) is bool or int(stamp)<=0:
            raise ValueError('invalid frame identity or actual wall timestamp')
        # The stamp comes from the producer. Request completion is never a new frame.
        info=dict(frame_id=str(frame_id),source_wall_ns=int(stamp),etag='"'+str(frame_id).replace('"','')+'"')
        with self.lock:
            if self.last.get('frame_id')==info['frame_id'] and self.last.get('source_wall_ns')!=info['source_wall_ns']:
                raise ValueError('same frame cannot acquire a newer source timestamp')
            self.raw=raw;self.last=dict(ready=True,**info)

    def monitor(self):
        while not self.closed.is_set():
            started=time.monotonic()
            try:self.refresh()
            except Exception as error:
                with self.lock:self.last.update(ready=False,error=str(error))
            self.closed.wait(max(.001,.04-(time.monotonic()-started)))

    def frame(self):
        with self.lock:
            if self.raw is None or not self.last['ready']:
                raise ValueError(self.last.get('error','camera producer has no frame'))
            return self.raw,dict(self.last)

    def snapshot(self):
        with self.lock:result=deepcopy(self.last)
        return dict(result,id=self.name,label=self.label,kind=self.kind,poll_hz=25,
                    url='/api/media/'+self.name+'?environment='+self.environment)
