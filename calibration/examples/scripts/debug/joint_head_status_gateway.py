"""Loopback read-only status gateway. No SDK, commands, or process control."""
import argparse
import json
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.request import build_opener, ProxyHandler, HTTPRedirectHandler

class NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, *args):
        raise ValueError('redirect forbidden')

def handler(owner_port):
    http=build_opener(ProxyHandler({}),NoRedirect())
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def do_GET(self):
            code=200
            try:
                if self.path!='/state':raise ValueError('only /state is available')
                with http.open(f'http://127.0.0.1:{owner_port}/state',timeout=.5) as r:raw=r.read(65537)
                if len(raw)>65536:raise ValueError('oversized state')
                owner=json.loads(raw)
                stamp=owner['calibration_feedback']['sample_wall_ns']
                if type(stamp) is not int:raise ValueError('invalid timestamp')
                value=dict(source='read-only-status-gateway',owner=owner,source_age_s=(time.time_ns()-stamp)/1e9)
            except Exception as e:code=503;value={'error':str(e)}
            raw=json.dumps(value,allow_nan=False).encode()
            self.send_response(code);self.send_header('Content-Type','application/json');self.send_header('Content-Length',str(len(raw)));self.send_header('Cache-Control','no-store');self.end_headers();self.wfile.write(raw)
        def do_POST(self):self.send_error(405,'read-only gateway')
    return Handler

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--port',type=int,default=8120);p.add_argument('--owner-port',type=int,default=8098);a=p.parse_args()
    ThreadingHTTPServer(('127.0.0.1',a.port),handler(a.owner_port)).serve_forever()
