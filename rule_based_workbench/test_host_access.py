"""Robot-host allowlist, CSRF and offline launcher checks; CPU only."""
from http.client import HTTPConnection
from http.server import ThreadingHTTPServer
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from threading import Thread
from unittest.mock import patch
import app
from real_bridge import RealBridge
from start_robot_host_offline import prepare


def main():
    server = ThreadingHTTPServer(('127.0.0.1', 0), app.Handler)
    server.allowed_hosts = ('10.17.32.5',)
    thread = Thread(target=server.serve_forever, daemon=True)
    thread.start()
    host = f'10.17.32.5:{server.server_port}'

    def request(method, path, headers, expected):
        connection = HTTPConnection(*server.server_address, timeout=3)
        try:
            connection.request(method, path, body='{}' if method == 'POST' else None,
                               headers={'Content-Type': 'application/json', **headers})
            response = connection.getresponse()
            body = response.read()
            assert response.status == expected, (response.status, body)
            return json.loads(body) if body else None
        finally:
            connection.close()

    try:
        token = request('GET', '/api/mode', {'Host': host}, 200)['csrf_token']
        request('HEAD', '/api/playback/source-base_0.mp4', {'Host': 'evil.example'}, 403)
        request('POST', '/api/pipeline/source', {}, 403)
        request('GET', '/api/mode', {'Host': 'evil.example'}, 403)
        request('GET', '/api/mode', {'Host': host, 'Origin': 'http://evil.example'}, 403)
        request('GET', '/api/mode', {'Host': host, 'Origin': 'null'}, 403)
        for route in ('/api/mode', '/api/real/control', '/api/pipeline/run', '/api/pipeline/source', '/api/command'):
            request('POST', route, {'Host': host}, 403)
            request('POST', route, {'Host': host, 'Origin': 'http://evil.example', 'X-Workbench-CSRF': token}, 403)
        request('POST', '/api/mode', {'Host': host, 'Origin': 'http://' + host, 'X-Workbench-CSRF': token}, 400)
        request('POST', '/api/pipeline/run', {'Host': 'evil.example'}, 403)
    finally:
        server.shutdown()
        server.server_close()
        thread.join()
    with TemporaryDirectory() as root:
        command = prepare(root, '10.17.32.5', 8119)
        assert '--pipeline-config' not in command
        config = json.loads((Path(root) / 'offline-real-config.json').read_text())
        bridge = RealBridge(config)
        with patch.object(bridge.opener, 'open', side_effect=AssertionError('hardware touched')):
            assert bridge.owner()['hardware_access_enabled'] is False
            assert bridge.status()['hardware_access_enabled'] is False
            for action in (lambda: bridge.camera('base_0', 'status'), lambda: bridge.camera_stream('base_0')):
                try:
                    action()
                except ValueError:
                    pass
                else:
                    raise AssertionError('offline camera access permitted')
    print('Robot host allowlist, same origin, CSRF and offline deployment: PASS')


if __name__ == '__main__':
    main()
