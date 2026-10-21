#!/usr/bin/env python3
"""LAN read-only adapter for an existing robot-local Prometheus preview bridge."""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from ipaddress import IPv4Address
import json
from pathlib import Path
import sys
from urllib.parse import urlsplit

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from robot_workbench.site_cameras import CAMERAS, NETWORKS, read_camera, camera_lifecycle


def serve(listen, port=18873, upstream_port=8081, owner_state=None, lifecycle=False):
    address = IPv4Address(listen)
    if not address.is_loopback and not any(address in network for network in NETWORKS):
        raise ValueError('explicit private or loopback listen IPv4 required')
    if type(port) is not int or not 0 <= port <= 65535 or type(upstream_port) is not int or not 1024 <= upstream_port <= 65535:
        raise ValueError('invalid camera bridge port')

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def reply(self, code, value, kind='application/json'):
            raw = json.dumps(value, ensure_ascii=False, allow_nan=False).encode() if kind == 'application/json' else value
            self.close_connection = True
            self.send_response(code)
            self.send_header('Content-Type', kind)
            self.send_header('Content-Length', str(len(raw)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Connection', 'close')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.end_headers()
            if self.command != 'HEAD':
                try:
                    self.wfile.write(raw)
                except OSError:
                    pass

        def do_GET(self):
            if self.headers.get_all('Host', []) != [f'{address}:{self.server.server_port}']:
                return self.reply(403, {'error': 'invalid host'})
            parsed = urlsplit(self.path)
            if parsed.netloc or parsed.scheme or self.headers.get_all('Content-Length', []) not in ([], ['0']) or self.headers.get('Transfer-Encoding'):
                return self.reply(400, {'error': 'origin-form read-only request required'})
            path = parsed.path
            if path=='/service' and lifecycle:
                try:return self.reply(200,camera_lifecycle('127.0.0.1',upstream_port))
                except (OSError,ValueError) as error:return self.reply(503,dict(error=str(error)))
            if path != '/streams' and path not in ('/frame/' + k + '.jpg' for k in CAMERAS):
                return self.reply(404, {'error': 'read-only camera resources only'})
            try:
                data = read_camera('127.0.0.1', upstream_port, path)
                if path == '/streams':
                    source_state = 'unknown'
                    if owner_state:
                        try:
                            from robot_workbench.camera_owner import read_owner
                            source_state = read_owner(owner_state, allow_stopped=True)['state']
                        except (OSError, ValueError, KeyError):
                            source_state = 'error'
                    return self.reply(200, dict(readonly=True, streams=[dict(key=row['key'], label=row['label'],
                        status=row['status'], source_state=source_state if owner_state else row['source_state'],
                        last_frame_age_s=row['age_s'], fps=row['fps']) for row in data]))
                return self.reply(200, data, 'image/jpeg')
            except (OSError, ValueError) as error:
                return self.reply(503, {'error': 'existing camera preview unavailable: ' + str(error)})

        do_HEAD = do_GET

        def do_POST(self):
            if not lifecycle or self.path!='/start':return self.reply(405, {'error': 'read-only camera bridge'})
            if (self.headers.get_all('Host',[])!=[f'{address}:{self.server.server_port}']
                    or self.headers.get('X-Camera-Start')!='1' or self.headers.get('Origin')
                    or self.headers.get('Content-Length')!='2' or self.headers.get('Transfer-Encoding')):
                return self.reply(403,dict(error='invalid camera-only start request'))
            if self.rfile.read(2)!=b'{}':return self.reply(400,dict(error='no start parameters accepted'))
            try:return self.reply(202,camera_lifecycle('127.0.0.1',upstream_port,start=True))
            except (OSError,ValueError) as error:return self.reply(503,dict(error=str(error)))

    server = ThreadingHTTPServer((str(address), port), Handler)
    server.daemon_threads = True
    return server


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--listen', required=True)
    parser.add_argument('--port', type=int, default=18873)
    parser.add_argument('--upstream-port', type=int, default=8081)
    parser.add_argument('--owner-state')
    parser.add_argument('--lifecycle',action='store_true',help='enable only the fixed camera-only start endpoint')
    args = parser.parse_args()
    server = serve(**vars(args))
    print('Read-only camera bridge listening:', server.server_address, flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
