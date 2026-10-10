"""Local HTTP mode isolation checks; no hardware connection."""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from threading import Thread
import time
from unittest.mock import Mock, patch
from urllib.error import URLError
import app
from real_bridge import RealBridge, CAMERAS
from test_app import serving


def transport_checks():
    calls = []
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args): pass
        def do_GET(self):
            calls.append((self.command, self.path))
            time.sleep(2.2)
            raw = b'{"token":"review"}'
            self.send_response(200)
            self.send_header('Content-Length', str(len(raw)))
            self.end_headers()
            try: self.wfile.write(raw)
            except (BrokenPipeError, ConnectionResetError): pass
        def do_POST(self):
            body = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
            calls.append(body['action'])
            self.do_GET()
    server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    thread = Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        with TemporaryDirectory() as root:
            token = Path(root)/'token'
            token.write_text('private-token-'*3)
            url = f'http://127.0.0.1:{server.server_port}'
            bridge = RealBridge(dict(token_file=str(token), control_url=url,
                                     cameras={camera:dict(url=url) for camera in CAMERAS}))
            bridge.inputs = Mock(return_value=({'secret_source':'private-source'}, {'secret_plan':'private-plan'}))
            started = time.monotonic()
            assert bridge.control({'action':'preview_frozen'}) == {'token':'review'}
            assert time.monotonic()-started > 2
            for invoke, expected in ((bridge.owner, 'GET /state action=none budget=2s'),
                                     (lambda: bridge.control({'action':'stop'}), 'POST /operator action=stop budget=2s')):
                try: invoke()
                except TimeoutError as error: assert str(error) == 'robot service timed out: '+expected
                else: raise AssertionError('ordinary request exceeded its short socket budget')
            assert calls == ['preview_frozen', ('POST','/operator'), ('GET','/state'), 'stop', ('POST','/operator')]
            for error in (TimeoutError('private-source private-plan '+bridge.token), URLError(TimeoutError(bridge.token))):
                with patch.object(bridge.opener, 'open', side_effect=error) as opened:
                    try: bridge.control({'action':'preview_frozen'})
                    except TimeoutError as failure:
                        assert str(failure) == 'robot service timed out: POST /operator action=preview_frozen budget=15s'
                    else: raise AssertionError('preview timeout was swallowed')
                    opened.assert_called_once()
                    assert opened.call_args.kwargs == {'timeout':15}
    finally:
        server.shutdown(); server.server_close(); thread.join()
    print('Long preview, short state/stop, labelled timeout and no retries: PASS')


def main():
    transport_checks()
    original = app.WORKBENCH_MODE, app.REAL_BRIDGE
    try:
        app.WORKBENCH_MODE = 'dual_dsw'
        app.REAL_BRIDGE = Mock()
        app.REAL_BRIDGE.config = {}
        app.REAL_BRIDGE.owner.return_value = {'automatic_enabled': False}
        with serving(app.WorkbenchState()) as request:
            mode = request('GET', '/api/mode')
            headers = {'X-Workbench-CSRF': mode['csrf_token']}
            request('POST', '/api/mode', {'mode': 'real'}, expected=403)
            request('POST', '/api/mode', {'mode': 'real'}, headers={**headers, 'Origin': 'http://evil.example'}, expected=403)
            request('POST', '/api/mode', {'mode': 'real'}, headers=headers)
            assert not app.REAL_BRIDGE.control.called
            request('POST', '/api/pipeline/run', {}, expected=403)
            app.REAL_BRIDGE.workflow_status.return_value = {
                'pipeline': {'schema_version': 'physical_collection_pipeline.v1'},
                'telemetry': {'status': 'unavailable'}, 'calibration_profile': None,
                'scene': None, 'trajectory': None, 'pipeline_configured': False, 'workflow': {}}
            real = request('GET', '/api/state?mode=real')
            assert real['mode'] == 'real' and real['scene'] is None
            dual = request('GET', '/api/state?mode=dual_dsw')
            assert dual['pipeline'] != real['pipeline']
            request('GET', '/api/state?mode=unknown', expected=400)
            request('GET', '/api/state?mode=real&mode=dual_dsw', expected=400)
            request('GET', '/api/live-preview/source.jpg?mode=dual_dsw&camera=unknown', expected=400)
            app.REAL_BRIDGE.prepare_collection.return_value = {'status': 'awaiting-physical-confirmation', 'preview': {'token': 'test'}}
            result = request('POST', '/api/pipeline/run', {'mode':'real'}, headers=headers, expected=202)
            assert result['status'] == 'awaiting-physical-confirmation'
            assert app.REAL_BRIDGE.prepare_collection.call_count == 1
            request('POST', '/api/pipeline/run', {'mode':'dual_dsw'}, headers=headers, expected=400)
            request('POST', '/api/pipeline/source', {}, expected=400)
            assert not app.REAL_BRIDGE.control.called
            app.REAL_BRIDGE.owner.return_value = {'automatic_enabled': True}
            request('POST', '/api/mode', {'mode': 'dual_dsw'}, headers=headers, expected=400)
            app.REAL_BRIDGE.owner.return_value = {'automatic_enabled': False}
            request('POST', '/api/mode', {'mode': 'dual_dsw'}, headers=headers)
            request('POST', '/api/real/control', {'action': 'acquire', 'confirmation': 'ARMS_SUPPORTED_CONNECT'}, headers=headers, expected=400)
            assert not app.REAL_BRIDGE.control.called
        print('Mode isolation, CSRF, no implicit acquisition and busy guard: PASS')
    finally:
        app.WORKBENCH_MODE, app.REAL_BRIDGE = original


if __name__ == '__main__':
    main()
