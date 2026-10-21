"""Task-owned, serial Unix-socket service for the frozen RGB-D detection adapter."""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import signal
import socket
import sys
import threading
import time
from types import SimpleNamespace


def digest(path):
    return 'sha256:' + hashlib.sha256(Path(path).read_bytes()).hexdigest()


def receive(connection):
    data = bytearray()
    while b'\n' not in data:
        part = connection.recv(4096)
        if not part:
            raise ConnectionError('incomplete service message')
        data.extend(part)
        if len(data) > 65536:
            raise ValueError('service message exceeds 64 KiB')
    value = json.loads(data.split(b'\n', 1)[0])
    if not isinstance(value, dict):
        raise ValueError('service message must be an object')
    return value


def send(connection, value):
    connection.sendall(json.dumps(value, allow_nan=False).encode() + b'\n')


def validate_request(payload, state, adapter, context):
    if payload.get('owner_token') != state['owner_token']:
        raise ValueError('service owner token mismatch')
    expected = payload.get('expected_identity')
    if not isinstance(expected, dict) or expected != state['identity']:
        raise ValueError('service identity mismatch')
    manifest = Path(payload['manifest'])
    output = Path(payload['run_dir'])
    if not manifest.is_absolute() or not output.is_absolute():
        raise ValueError('manifest and run-dir must be absolute')
    if output.exists() or output.is_symlink():
        raise ValueError('run-dir must not already exist')
    if output.parent.resolve(strict=True) != output.parent or not output.parent.is_dir():
        raise ValueError('run-dir parent must be a canonical existing directory')
    if output.resolve().parent != output.parent or not output.is_relative_to(Path(state['allowed_output_root'])):
        raise ValueError('run-dir must be inside the allowed output root')
    manifest = manifest.resolve(strict=True)
    read_roots = [Path(value) for value in state['allowed_read_roots']]
    if not any(manifest.is_relative_to(root) for root in read_roots):
        raise ValueError('manifest must be inside an allowed read root')
    capture = adapter.load_manifest(manifest, **({'streaming':True} if payload.get('stream_id') else {}))
    if payload.get('stream_id') and len(capture['frames']) != 1:
        raise ValueError('stream request must contain exactly one captured frame')
    if capture['frame_id'] != context['catalog'].runtime_config.sensor_profile.optical_frame:
        raise ValueError('manifest optical frame differs from frozen release')
    for row in capture['frames']:
        frame = (manifest.parent / row['path']).resolve(strict=True)
        if not any(frame.is_relative_to(root) for root in read_roots):
            raise ValueError('frame must be inside an allowed read root')
        adapter.read_frame(frame)
    return manifest, output


def serve(args, adapter):
    if sys.platform != 'linux' or os.environ.get('CUDA_VISIBLE_DEVICES') not in ('0','1'):
        raise RuntimeError('service requires an explicitly selected single Linux CUDA GPU 0 or 1')
    if not args.service_root or not args.socket or not args.owner_token or not args.node_config:
        raise ValueError('serve requires service-root, socket, owner-token and node-config')
    if not args.allowed_read_root or not args.allowed_output_root:
        raise ValueError('serve requires allowed-read-root and allowed-output-root')
    for boundary in [*args.allowed_read_root, args.allowed_output_root]:
        boundary = Path(boundary)
        if not boundary.is_absolute() or boundary.resolve(strict=True) != boundary or not boundary.is_dir():
            raise ValueError('allowed roots must be absolute canonical existing directories')
    root, address = Path(args.service_root), Path(args.socket)
    if not root.is_absolute() or not address.is_absolute() or address.parent != root:
        raise ValueError('socket must be directly inside the absolute unique service root')
    if len(os.fsencode(address)) >= 104:
        raise ValueError('Unix socket path is too long')
    if root.parent.resolve(strict=True) != root.parent:
        raise ValueError('service root parent must be canonical')
    root.mkdir(mode=0o700, exist_ok=False)
    started = time.perf_counter()
    stop = threading.Event()
    lock = threading.Lock()
    state = dict(status='starting', stage='runtime_imports_and_cuda', message='加载 CUDA 与检测运行时',
                 progress=0, eta_s=155.0, error=None, pid=os.getpid(), gpu=0,
                 physical_gpu=int(os.environ['CUDA_VISIBLE_DEVICES']),logical_cuda_index=0,
                 process_start_ticks=int(Path('/proc/self/stat').read_text().rsplit(')', 1)[1].split()[19]),
                 boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
                 owner_token=args.owner_token, socket=str(address), service_root=str(root),
                 allowed_read_roots=[str(path) for path in args.allowed_read_root],
                 allowed_output_root=str(args.allowed_output_root),
                 prepare_count=0, completed_requests=0,
                 identity=dict(adapter_sha256=digest(adapter.__file__), service_sha256=digest(__file__),
                               node_config_sha256=digest(args.node_config)))

    def publish():
        with lock:
            value = dict(state, elapsed_s=time.perf_counter() - started)
            if value['status'] == 'starting':
                value['eta_s'] = max(0., 155.0 - value['elapsed_s'])
            init_status = 'ready' if value['status'] == 'busy' else value['status']
            value['initialization'] = {key: value[key] for key in
                                       ('stage', 'message', 'progress', 'elapsed_s', 'eta_s', 'error')}
            value['initialization']['status'] = init_status
            encoded = json.dumps(value, allow_nan=False)
            temporary = root / 'status.json.partial'
            temporary.write_text(encoded + '\n')
            temporary.replace(root / 'status.json')
            print(encoded, flush=True)

    phases = {'runtime_imports_and_cuda': ('catalog_assets_runtime_validation', 16),
              'catalog_assets_runtime_validation': ('configure_environment', 17),
              'configure_environment': ('render_batch_install', 18),
              'render_batch_install': ('prepare_application', 67),
              'prepare_application': ('ready', 100)}
    messages = {'catalog_assets_runtime_validation': '校验模型资产与发布配置',
                'configure_environment': '配置检测运行环境与缓存',
                'render_batch_install': '导入渲染依赖',
                'prepare_application': '加载检测与姿态模型',
                'ready': '验证检测会话重置能力'}

    def advance(row):
        next_phase, percent = phases.get(row['stage'], (row['stage'], None))
        with lock:
            state.update(stage=next_phase, progress=percent,
                         message=messages.get(next_phase, next_phase))
        publish()

    def heartbeat():
        while not stop.wait(1):
            publish()

    def terminate(signum, frame):
        raise KeyboardInterrupt('service owner requested stop')

    signal.signal(signal.SIGTERM, terminate)
    context = None
    thread = threading.Thread(target=heartbeat, daemon=True)
    listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    publish()
    thread.start()
    try:
        listener.bind(str(address))
        os.chmod(address, 0o600)
        listener.listen(2)
        timing = {'segments': [], '_callback': advance}
        init_args = SimpleNamespace(**vars(args))
        init_args.run_dir = root / 'runtime'
        init_args.run_dir.mkdir()
        init_args.node_config = Path(args.node_config).resolve(strict=True)
        context, _ = adapter.initialize_detection(init_args, {}, timing, time.perf_counter())
        state['prepare_count'] += 1
        # Validate supported lifecycle before advertising readiness, without model reload.
        context['runtime'] = context['runtime'].new_session(timeout=12)
        (root / 'startup-timing.json').write_text(json.dumps({'segments': timing['segments']}, indent=2))
        active_stream = None
        stream_identity = None
        state.update(status='ready', stage='ready', message='检测模型已就绪', progress=100, eta_s=0,
                     model_initialization_count=model_count(context))
        publish()
        while True:
            connection, _ = listener.accept()
            with connection:
                connection.settimeout(600)
                try:
                    payload = receive(connection)
                    stream_id = payload.get('stream_id')
                    if stream_id is not None:
                        if (not isinstance(stream_id, str) or len(stream_id) != 32 or
                                any(c not in '0123456789abcdef' for c in stream_id) or
                                payload.get('owner_token') != state['owner_token'] or
                                payload.get('expected_identity') != state['identity']):
                            raise ValueError('stream ownership/identity mismatch')
                        if payload.get('stream_end') is True:
                            if active_stream != stream_id:
                                raise ValueError('cannot end a different acquisition stream')
                            context['runtime'] = context['runtime'].new_session(timeout=12)
                            context.pop('_stream_source',None)
                            active_stream = stream_identity = None
                            send(connection, {'exit_code':0, 'cleanup_complete':True, 'stream_id':stream_id})
                            continue
                        if type(payload.get('stream_begin')) is not bool:
                            raise ValueError('explicit stream begin flag required')
                        if not payload['stream_begin'] and active_stream != stream_id:
                            raise ValueError('frame belongs to another acquisition stream')
                        timeout = payload.get('result_timeout_s')
                        if type(timeout) not in (int,float) or not 0 < timeout <= 120:
                            raise ValueError('bounded streaming result timeout required')
                    if stream_id is None and active_stream is not None:
                        raise ValueError('live acquisition owns detector tracking')
                    manifest, output = validate_request(payload, state, adapter, context)
                except (ValueError, KeyError, TypeError, OSError) as error:
                    send(connection, {'exit_code': 2, 'error': str(error), 'service_status': 'ready'})
                    continue
                state.update(status='busy', stage='detect', message='检测采集场景', active_run_dir=str(output))
                publish()
                request_args = SimpleNamespace(**vars(args))
                request_args.manifest, request_args.run_dir = manifest, output
                request_args.dry_run = False
                request_args.streaming = stream_id is not None
                if stream_id is not None:
                    if payload['stream_begin']:
                        # A new acquisition supersedes an abandoned cancelled stream,
                        # after its last in-flight request has finished serially.
                        context['runtime'] = context['runtime'].new_session(timeout=12)
                        context.pop('_stream_source',None)
                        active_stream = stream_id
                        stream_identity = None
                    request_args.result_timeout_s = payload['result_timeout_s']
                try:
                    summary = adapter.run(request_args, prepared=context)
                    summary['service'] = {key: state[key] for key in
                                          ('pid', 'process_start_ticks', 'prepare_count', 'identity')}
                    summary['service']['model_initialization_count'] = model_count(context)
                    if stream_id is not None:
                        if stream_identity is None:
                            stream_identity = summary['tracking_session_id']
                        elif stream_identity != summary['tracking_session_id']:
                            raise RuntimeError('tracking session changed inside acquisition')
                        summary['stream_id'] = stream_id
                    adapter.write_json(output / 'summary.json', summary)
                except Exception as error:
                    state.update(status='failed', stage='detect', error=str(error), message='检测服务失败')
                    publish()
                    send(connection, {'exit_code': 1, 'error': str(error), 'service_status': 'failed'})
                    raise
                state.update(status='ready', stage='ready', message='检测模型已就绪',
                             completed_requests=state['completed_requests'] + 1, active_run_dir=None)
                publish()
                try:
                    response = {'exit_code': 0, 'summary_path': str(output / 'summary.json'),
                                'tracking_session_id': summary['tracking_session_id'], 'service_status': 'ready'}
                    if stream_id is not None:
                        observations = [json.loads(line) for line in (output/'observations.jsonl').read_text().splitlines()]
                        if len(observations) != 1:
                            raise ValueError('stream request must contain exactly one frame')
                        response.update(stream_id=stream_id, observation={k:v for k,v in observations[0].items()
                            if k != 'diagnostics'}, summary=summary)
                    send(connection, response)
                except OSError:
                    pass  # Completed evidence remains available if the client disconnected.
    except BaseException as error:
        state.update(status='failed', stage='stopped', error=str(error), message='检测服务已停止')
        raise
    finally:
        stop.set()
        thread.join(timeout=2)
        listener.close()
        if context is not None:
            state['cleanup_complete'] = context['runtime'].close(timeout=12)
        publish()


def model_count(context):
    # Read-only diagnostics from the exact configured FoundationPose stage.
    stage = context['runtime']._processing._pose_stage
    return stage._provider.shared_runtime_initialization_count


def request(args):
    if not args.socket or not args.manifest or not args.run_dir or not args.owner_token or not args.expected_identity:
        raise ValueError('request requires socket, manifest, run-dir, owner-token and expected-identity')
    payload = dict(owner_token=args.owner_token, manifest=str(Path(args.manifest).resolve()),
                   run_dir=str(Path(args.run_dir).absolute()))
    payload['expected_identity'] = json.loads(args.expected_identity)
    if not isinstance(payload['expected_identity'], dict):
        raise ValueError('expected-identity must be an object')
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
        connection.settimeout(600)
        connection.connect(str(args.socket))
        send(connection, payload)
        result = receive(connection)
    print(json.dumps(result), flush=True)
    if result['exit_code']:
        raise SystemExit(result['exit_code'])
    return result
