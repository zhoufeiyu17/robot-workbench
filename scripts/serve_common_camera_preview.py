#!/usr/bin/env python3
"""Read-only site preview of an already running CommonCameraService."""
import argparse
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from ipaddress import IPv4Address
import math
from pathlib import Path
import sys
import subprocess
import time
from urllib.parse import urlsplit

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from robot_workbench.rgbd_service import CommonCameraService
from robot_workbench.site_cameras import CAMERAS, CameraDirectory

SOURCES = {key: key.removesuffix('_rgb') for key in CAMERAS}


def snapshot(client, key):
    """Preserve source identity/time; fetching a frame never makes it fresh."""
    frame = client.frames[SOURCES[key]].snapshot(formats=['jpeg'])
    status = frame.get('status', {})
    received = status.get('host_receive_ns')
    age = (time.time_ns() - received) / 1e9 if type(received) is int and received > 0 else None
    if age is not None and (not math.isfinite(age) or age < 0):
        age = None
    jpeg = frame.get('jpeg')
    valid_image = isinstance(jpeg, bytes) and jpeg.startswith(b'\xff\xd8') and len(jpeg) <= 2 * 1024 * 1024
    live = status.get('ready') is True and valid_image and age is not None and age <= 1
    source_state = ('running' if live else 'error' if status.get('error') and received is None
                    else 'starting' if received is None else 'unknown')
    fps = status.get('capture_fps')
    fps = fps if type(fps) in (int, float) and math.isfinite(fps) and fps >= 0 else None
    row = dict(key=key, label=CAMERAS[key], status='live' if live else 'stale',
               source_state=source_state, last_frame_age_s=age, fps=fps)
    # Only fixed non-secret metadata leaves the IPC boundary.
    for field in ('host_receive_ns', 'source_time_ns', 'frame_number', 'pair_id',
                  'capture_instance_id', 'source_clock_id'):
        value = status.get(field)
        if type(value) in (str, int) and (not isinstance(value, str) or len(value) <= 256):
            row[field] = value
    return row, jpeg if live else None


def serve(descriptor, listen='127.0.0.1', port=18874, *, client=None, source_unit=None, source_state=None):
    path = Path(descriptor)
    if not path.is_absolute():
        raise ValueError('absolute CommonCameraService descriptor path required')
    address = IPv4Address(listen)
    if not address.is_loopback:
        raise ValueError('common camera preview requires a loopback IPv4 listener')
    if type(port) is not int or not 0 <= port <= 65535:
        raise ValueError('invalid preview port')
    if source_unit not in (None,'robot-workbench-rgbd.service'):
        raise ValueError('only the camera-only source unit can be started')
    client = client if client is not None else None if source_unit else CommonCameraService.connect(path)
    if client is not None and not set(SOURCES.values()) <= set(client.frames):
        raise ValueError('three configured common camera sources required')
    cached=[None,client]
    def current_client():
        if source_unit:
            stamp=path.stat().st_mtime_ns
            if cached[0]!=stamp:
                cached[:]=[stamp,CommonCameraService.connect(path)]
        return cached[1]

    def lifecycle():
        if source_unit is None:return dict(available=False)
        unit=subprocess.run(['systemctl','--user','is-active',source_unit],capture_output=True,text=True,timeout=2).stdout.strip()
        try:value=json.loads(Path(source_state).read_text())
        except (OSError,ValueError,TypeError):value={}
        phase=value.get('phase','stopped') if unit=='active' else 'starting' if unit=='activating' else 'error' if unit=='failed' else 'stopped'
        return dict(available=True,phase=phase,error=value.get('error') if phase=='error' else None,
                    message='仅启动相机采集与共享图像服务，不连接机械臂控制')

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def reply(self, code, value, kind='application/json'):
            self.close_connection = True
            return CameraDirectory.send(self, code, value, kind)

        def do_GET(self):
            if self.headers.get_all('Host', []) != [f'{address}:{self.server.server_port}']:
                return self.reply(403, {'error': 'invalid host'})
            parsed = urlsplit(self.path)
            if (parsed.netloc or parsed.scheme or parsed.query or parsed.fragment
                    or self.headers.get_all('Content-Length', []) not in ([], ['0'])
                    or self.headers.get('Transfer-Encoding')):
                return self.reply(400, {'error': 'origin-form read-only request required'})
            if parsed.path=='/service':return self.reply(200,lifecycle())
            if parsed.path == '/streams':
                rows = []
                for key in SOURCES:
                    try:
                        row, _ = snapshot(current_client(), key)
                    except (OSError, ValueError, KeyError, TypeError):
                        row = dict(key=key, label=CAMERAS[key], status='stale', source_state='error',
                                   last_frame_age_s=None, fps=None)
                    rows.append(row)
                return self.reply(200, dict(readonly=True, backend='common_rgbd', streams=rows))
            key = next((key for key in SOURCES if parsed.path == '/frame/' + key + '.jpg'), None)
            if key is None:
                return self.reply(404, {'error': 'read-only camera resources only'})
            try:
                _, jpeg = snapshot(current_client(), key)
                if jpeg is not None:
                    return self.reply(200, jpeg, 'image/jpeg')
            except (OSError, ValueError, KeyError, TypeError):
                pass
            return self.reply(503, {'error': 'common camera frame unavailable or stale'})

        do_HEAD = do_GET

        def do_POST(self):
            if source_unit is None or self.path!='/start':
                return self.reply(405, {'error': 'read-only camera preview'})
            if (self.headers.get_all('Host',[])!=[f'{address}:{self.server.server_port}']
                    or self.headers.get('X-Camera-Start')!='1' or self.headers.get('Origin')
                    or self.headers.get('Content-Length')!='2' or self.headers.get('Transfer-Encoding')):
                return self.reply(403,dict(error='invalid camera-only start request'))
            if self.rfile.read(2)!=b'{}':return self.reply(400,dict(error='no camera start parameters accepted'))
            try:
                result=subprocess.run(['systemctl','--user','start','--no-block',source_unit],capture_output=True,text=True,timeout=3)
                if result.returncode:raise ValueError('相机服务启动失败：'+result.stderr[:300])
                return self.reply(202,dict(available=True,phase='starting',message='正在启动并连接图像进程，请等待首帧'))
            except (OSError,ValueError,subprocess.TimeoutExpired) as error:
                return self.reply(503,dict(error=str(error)))

        def do_PUT(self):self.reply(405,dict(error='method not allowed'))
        do_PATCH = do_DELETE = do_PUT

    server = ThreadingHTTPServer((str(address), port), Handler)
    server.daemon_threads = True
    return server


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--descriptor', required=True)
    parser.add_argument('--listen', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=18874)
    parser.add_argument('--source-unit',choices=['robot-workbench-rgbd.service'])
    parser.add_argument('--source-state',type=Path)
    args = parser.parse_args()
    server = serve(**vars(args))
    print('Common camera preview listening:', server.server_address, flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
