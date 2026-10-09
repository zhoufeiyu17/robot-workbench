#!/usr/bin/env python3
"""Read-only local review of captured Isaac RGB, PhysX motion and calibration."""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import mimetypes
from pathlib import Path
from urllib.parse import quote, unquote, urlsplit


def read_json(path):
    def reject(value):
        raise ValueError('non-finite JSON: ' + value)
    return json.loads(path.read_text(), parse_constant=reject)


def select(value, keys):
    return {key: value[key] for key in keys.split() if key in value}


def local_file(root, name):
    path = (root / name).resolve()
    if not path.is_relative_to(root) or not path.is_file():
        raise ValueError('artifact outside dataset or missing')
    return path


def review(root):
    data = read_json(local_file(root, 'dataset.json'))
    if data.get('schema') != 'joint-head-isaac-v1' or len(data['joint_names']) != 12:
        raise ValueError('expected joint-head-isaac-v1 with 12 ordered joints')
    def image_url(name):
        path = local_file(root, name)
        if path.suffix.lower() not in ('.png', '.jpg', '.jpeg', '.gif'):
            raise ValueError('expected raster image')
        return '/images/' + quote(path.relative_to(root).as_posix(), safe='/')
    frames = []
    for item in data['frames']:
        row = select(item, 'id split q_reported_rad velocity_rad_s target_reported_rad tracking_error_rad motion sim_time_s physics_step')
        row['image'] = image_url(item['image'])
        frames.append(row)
    reports = []
    for path in sorted(root.glob('fit-*.json'), key=lambda p: p.stat().st_mtime, reverse=True):
        fit = read_json(local_file(root, path.name))
        reports.append({'file': path.name, **select(fit, 'status metrics observability per_link per_frame macro_metrics coverage joint_offsets_rad optimizer_success optimizer_message at_parameter_bound positive_depth used_truth_in_fit physical_validated applied acceptance frames_without_detections dataset_sha256')})
    preview = read_json(local_file(root, 'preview.json')) if (root / 'preview.json').exists() else {}
    score = read_json(local_file(root, 'truth-score.json')) if (root / 'truth-score.json').exists() else None
    # Only post-fit error scores are exposed. Raw hidden truth has no HTTP route.
    if score is not None:
        score = select(score, 'camera_translation_error_mm camera_rotation_error_deg joint_offset_errors_deg max_joint_offset_error_deg')
    return {'dataset': root.name, **select(data, 'source geometry_equivalence joint_names'),
            'frames': frames, 'reports': reports, 'truth_score': score,
            'preview': {'frames': [image_url(p) for p in preview.get('frames', [])], 'fps': preview.get('fps', 2)},
            'overlays': [{'name': p.relative_to(root).as_posix(), 'url': image_url(p.relative_to(root).as_posix())}
                         for p in sorted(root.rglob('*overlay*.png'))],
            'trajectory_gif': image_url('trajectory.gif') if (root / 'trajectory.gif').exists() else None,
            'simulation_only': True, 'hardware_connected': False}


def handler(root):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def send(self, value, status=200, content_type='application/json; charset=utf-8'):
            raw = (json.dumps(value, ensure_ascii=False, allow_nan=False) + '\n').encode() if content_type.startswith('application/json') else value
            self.send_response(status)
            for key, value in [('Content-Type', content_type), ('Content-Length', str(len(raw))),
                               ('Cache-Control', 'no-store'), ('X-Content-Type-Options', 'nosniff'),
                               ('X-Frame-Options', 'DENY')]:
                self.send_header(key, value)
            self.end_headers()
            self.wfile.write(raw)

        def do_GET(self):
            port = self.server.server_port
            if self.headers.get('Host') not in (f'127.0.0.1:{port}', f'localhost:{port}'):
                return self.send({'error': 'loopback Host required'}, 403)
            try:
                active_root = root() if callable(root) else root
                path = urlsplit(self.path).path
                if path == '/':
                    return self.send(Path(__file__).with_suffix('.html').read_bytes(), content_type='text/html; charset=utf-8')
                if path == '/api/review':
                    return self.send(review(active_root))
                if path.startswith('/images/'):
                    file = local_file(active_root, unquote(path[len('/images/'):]))
                    if file.suffix.lower() not in ('.png', '.jpg', '.jpeg', '.gif'):
                        raise ValueError('only raster images are served')
                    return self.send(file.read_bytes(), content_type=mimetypes.guess_type(file.name)[0])
                return self.send({'error': 'not found'}, 404)
            except (OSError, ValueError, KeyError, TypeError) as exc:
                return self.send({'error': str(exc)}, 400)
    return Handler


def self_test():
    """HTTP contract test; temporary bytes are not simulator visual evidence."""
    import http.client
    import tempfile
    import threading
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp).resolve() / 'run'
        root.mkdir()
        (root / 'frame.png').write_bytes(b'test-only')
        (Path(tmp) / 'outside.png').write_bytes(b'private')
        (root / 'escape.png').symlink_to(Path(tmp) / 'outside.png')
        (root / 'truth.json').write_text('{"secret": "withheld"}')
        (root / 'dataset.json').write_text(json.dumps({'schema': 'joint-head-isaac-v1', 'joint_names': list(range(12)),
                                                     'frames': [{'id': 'one', 'image': 'frame.png'}]}))
        server = ThreadingHTTPServer(('127.0.0.1', 0), handler(root))
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            connection = http.client.HTTPConnection('127.0.0.1', server.server_port)
            for path, expected in [('/', 200), ('/api/review', 200), ('/images/frame.png', 200),
                                   ('/images/%2e%2e/outside.png', 400), ('/images/escape.png', 400),
                                   ('/images/truth.json', 400), ('/truth.json', 404)]:
                connection.request('GET', path)
                response = connection.getresponse()
                body = response.read()
                assert response.status == expected, (path, response.status)
                assert b'withheld' not in body
            connection.request('GET', '/', headers={'Host': 'evil.example'})
            response = connection.getresponse()
            assert response.status == 403
            response.read()
            (root / 'fit-bad.json').write_text('{"metrics": {"value": NaN}}')
            connection.request('GET', '/api/review')
            response = connection.getresponse()
            assert response.status == 400
            response.read()
            connection.close()
        finally:
            server.shutdown()
            server.server_close()
            thread.join()
    print('PASS: HTTP review, image access, traversal/symlink/hidden-truth rejection, Host and finite JSON')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path)
    parser.add_argument('--port', type=int, default=8110)
    parser.add_argument('--self-test', action='store_true')
    args = parser.parse_args()
    if args.self_test:
        return self_test()
    if args.dataset is None or not 1024 <= args.port <= 65535:
        parser.error('--dataset and a port in 1024..65535 are required')
    root = args.dataset.resolve(strict=True)
    review(root)
    with ThreadingHTTPServer(('127.0.0.1', args.port), handler(root)) as server:
        print(f'Isaac evidence review: http://127.0.0.1:{args.port} (read-only; simulation only)', flush=True)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
