"""Lossless RGB-D segments. Source timestamps live in metadata, never video PTS.

Only atomic *.commit.json files publish segments. A stopped writer leaves its
partial files for diagnosis; recovery exposes the committed prefix, not a
complete episode. Callers supply RGB8/depth16 PNGs or immutable RGB8/depth16le pixels.
"""
from collections import deque, OrderedDict
from concurrent.futures import ThreadPoolExecutor, TimeoutError as FutureTimeout
import hashlib
import io
import json
import os
from pathlib import Path
import re
import select
import shutil
import struct
import subprocess
import threading
import time
import zlib


_LIMIT = 16 * 1024 * 1024
_SCHEMA = 'rgbd-segment-v1'
_RGB_SCHEMA = 'rgb-segment-v1'
# Encoding reaches the source rate with two workers; decoding needs four for
# this FFV1 slice grid. Avoid oversubscribing both stages during segment overlap.
_DEPTH_ENCODE_THREADS = 2
_DEPTH_DECODE_THREADS = 4
_PROBE_CACHE = OrderedDict()
_PROBE_LOCK = threading.Lock()


def _shape(blob, depth=False):
    if not isinstance(blob, bytes) or not 33 <= len(blob) <= _LIMIT or blob[:8] != b'\x89PNG\r\n\x1a\n':
        raise ValueError('bounded PNG payload required')
    width, height, bits, kind, compression, filtering, interlace = struct.unpack('!IIBBBBB', blob[16:29])
    if (blob[12:16] != b'IHDR' or bits != (16 if depth else 8) or kind != (0 if depth else 2)
            or compression or filtering or interlace or min(width, height) < 1
            or width * height * (2 if depth else 3) > _LIMIT):
        raise ValueError('noninterlaced RGB8 / grayscale depth16 PNG required; alpha is not discarded')
    return width, height


def _pixel_shape(rgb, depth, width, height):
    if (type(width) is not int or type(height) is not int or min(width, height) < 1
            or width * height * 3 > _LIMIT):
        raise ValueError('bounded positive integer RGB-D dimensions required')
    if (not isinstance(rgb, bytes) or not isinstance(depth, bytes)
            or len(rgb) != width * height * 3 or len(depth) != width * height * 2):
        raise ValueError('immutable RGB8 / little-endian depth16 bytes must match dimensions')
    return width, height


def _digest(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return {'size_bytes': path.stat().st_size, 'sha256': digest.hexdigest()}


def _safe(root, name):
    path = Path(name)
    if not isinstance(name, str) or not path.parts or path.is_absolute() or '..' in path.parts or str(path) != name:
        raise ValueError('invalid segment path')
    current = Path(root)
    if current.is_symlink():
        raise ValueError('symlink segment root rejected')
    for part in path.parts:
        current = current / part
        if current.is_symlink():
            raise ValueError('symlink segment path rejected')
    if not current.is_file():
        raise ValueError('regular segment file required: ' + name)
    return current


class _Child:
    """Own exactly one codec process; bound its stderr, IO waits and lifetime."""
    def __init__(self, argv, timeout_s):
        from launch_environment import clean_environment
        argv=[shutil.which(argv[0])or argv[0],*argv[1:]]
        self.timeout_s = timeout_s
        self.process = subprocess.Popen(argv, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                        bufsize=0, close_fds=True, env=clean_environment())
        self.errors = deque(maxlen=4)
        self.drain = threading.Thread(target=self._stderr, daemon=True)
        self.drain.start()
        os.set_blocking(self.process.stdin.fileno(), False)
        os.set_blocking(self.process.stdout.fileno(), False)

    def _stderr(self):
        try:
            for block in iter(lambda: self.process.stderr.read(4096), b''):
                self.errors.append(block)
        except (ValueError, OSError):
            pass  # Concurrent abort already closed this owned child's pipe.

    def _error(self, message):
        return RuntimeError(message + ': ' + b''.join(self.errors).decode(errors='replace')[-8192:])

    def write(self, blob):
        view = memoryview(blob)
        deadline = time.monotonic() + self.timeout_s
        while view:
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not select.select([], [self.process.stdin], [], remaining)[1]:
                raise TimeoutError('segment encoder write timeout')
            try:
                view = view[os.write(self.process.stdin.fileno(), view):]
            except BrokenPipeError as exc:
                raise self._error('segment encoder exited') from exc

    def read(self, size, *, eof=False):
        result = bytearray()
        deadline = time.monotonic() + self.timeout_s
        while len(result) < size:
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not select.select([self.process.stdout], [], [], remaining)[0]:
                raise TimeoutError('segment decoder read timeout')
            block = os.read(self.process.stdout.fileno(), min(size - len(result), 1024 * 1024))
            if not block:
                if eof and not result:
                    return b''
                raise self._error('segment decoder truncated frame')
            result.extend(block)
        return bytes(result)

    def finish(self):
        if not self.process.stdin.closed:
            self.process.stdin.close()
        try:
            code = self.process.wait(timeout=self.timeout_s)
        except subprocess.TimeoutExpired:
            self.abort()
            raise TimeoutError('segment codec exit timeout') from None
        self.drain.join(timeout=1)
        self.process.stdout.close()
        self.process.stderr.close()
        if code:
            raise self._error('segment codec failed')

    def abort(self):
        if self.process.poll() is None:
            try:
                self.process.terminate()
            except ProcessLookupError:
                pass
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=2)
        if not self.process.stdin.closed:
            self.process.stdin.close()
        self.drain.join(timeout=1)
        self.process.stdout.close()
        self.process.stderr.close()


def _probe(path, ffprobe, timeout_s, *, factory=_Child, release=lambda child: None, probe_cache=None):
    # Always reread the bytes. Only the expensive decode/count is reusable;
    # stat/mtime alone cannot authorize a previously verified segment.
    pin=_digest(Path(path));tool=str(Path(shutil.which(ffprobe)or ffprobe).resolve())
    key=(tool,pin['size_bytes'],pin['sha256'])
    with _PROBE_LOCK:
        cached=(probe_cache or {}).get(key)or _PROBE_CACHE.get(key)
        if cached is not None:
            if probe_cache is not None:probe_cache[key]=dict(cached)
            if key in _PROBE_CACHE:_PROBE_CACHE.move_to_end(key)
            return dict(cached)
    child = factory([ffprobe, '-v', 'error', '-threads', str(_DEPTH_DECODE_THREADS if Path(path).suffix=='.mkv'else 1), '-count_frames', '-select_streams', 'v:0', '-show_entries',
                    'stream=width,height,nb_read_frames,codec_name,pix_fmt', '-of', 'json', str(path)], timeout_s)
    try:
        child.process.stdin.close()
        output = bytearray()
        while block := child.read(1, eof=True):
            output.extend(block)
            if len(output) > 4096:
                raise ValueError('segment probe output exceeds limit')
        child.finish()
        streams = json.loads(output)['streams']
        if len(streams) != 1:
            raise ValueError('one video stream required')
        if _digest(Path(path))!=pin:raise ValueError('segment changed during codec verification')
        with _PROBE_LOCK:
            if probe_cache is not None:probe_cache[key]=dict(streams[0])
            _PROBE_CACHE[key]=dict(streams[0]);_PROBE_CACHE.move_to_end(key)
            while len(_PROBE_CACHE)>128:_PROBE_CACHE.popitem(last=False)
        return streams[0]
    finally:
        child.abort()
        release(child)


def _check_probe(path, manifest, kind, ffprobe, timeout_s, **kwargs):
    stream = _probe(path, ffprobe, timeout_s, **kwargs)
    if (int(stream['width']) != manifest['width'] or int(stream['height']) != manifest['height']
            or int(stream['nb_read_frames']) != manifest['frame_count']
            or stream['codec_name'] != ('h264' if kind == 'rgb' else 'ffv1')
            or stream['pix_fmt'] not in (('gbrp',) if kind == 'rgb' else ('gray16le',))):
        raise ValueError('segment codec, frame count or dimensions differ')


class SegmentWriter:
    def __init__(self, root, camera, segment_frames=120, ffmpeg='ffmpeg', ffprobe='ffprobe', timeout_s=30,
                 commit_executor=None, pixel_format='rgb8-depth16le',probe_cache=None):
        started = time.monotonic()
        if not isinstance(camera, str) or not re.fullmatch(r'[A-Za-z0-9_-]+', camera):
            raise ValueError('safe camera name required')
        if type(segment_frames) is not int or not 1 <= segment_frames <= 120 or not 0 < timeout_s <= 300:
            raise ValueError('bounded segment size / timeout required')
        self.root, self.camera = Path(root), camera
        if pixel_format not in ('rgb8','rgb8-depth16le'):raise ValueError('declared segment pixel format required')
        self.kinds=('rgb',) if pixel_format=='rgb8' else ('rgb','depth')
        self.schema=_RGB_SCHEMA if pixel_format=='rgb8' else _SCHEMA
        self.root.mkdir(parents=True, exist_ok=True)
        if self.root.is_symlink():
            raise ValueError('symlink segment root rejected')
        self.directory = self.root / camera
        self.directory.mkdir(exist_ok=False)
        self.segment_frames, self.ffmpeg, self.ffprobe, self.timeout_s = segment_frames, ffmpeg, ffprobe, timeout_s
        self.probe_cache=probe_cache
        self.segments, self.children = [], {}
        self.meta = None
        self.closed = self.aborted = False
        self.count = self.issued_index = 0
        self.shape = None
        self._pending = deque()
        self._lock = threading.RLock()
        self._owned_children = set()
        self._owns_executor = commit_executor is None
        self._executor = commit_executor or ThreadPoolExecutor(max_workers=1, thread_name_prefix='rgbd-commit')
        self.timings = dict(commit_segments=0, commit_wall_s=0., commit_peak_wall_s=0.,
                            pending_wait_wall_s=0., pending_wait_peak_wall_s=0.,
                            codec_finalize_wall_s=0., probe_wall_s=0., persist_hash_wall_s=0., last_commit=None,
                            directory_setup_wall_s=time.monotonic()-started, first_frame=None)

    def _check_active(self):
        if self.aborted:
            raise ValueError('aborted segment writer cannot seal; recover its committed prefix')

    def _spawn(self, argv, timeout_s):
        with self._lock:
            self._check_active()
            child = _Child(argv, timeout_s)
            self._owned_children.add(child)
            return child

    def _forget(self, child):
        with self._lock:
            self._owned_children.discard(child)

    def _start(self, shape):
        self.shape, self.count = shape, 0
        self.pixel_hashes = {kind: hashlib.sha256() for kind in self.kinds}
        self.stem = f'{self.issued_index:06d}'
        self.issued_index += 1
        self.paths = {kind: self.directory / f'{self.stem}.{kind}.partial.{extension}'
                      for kind, extension in [('rgb', 'mp4'), ('depth', 'mkv'), ('frames', 'jsonl')] if kind in (*self.kinds,'frames')}
        detail = self.timings['first_frame'] if self.stem == '000000' else None
        started = time.monotonic()
        if detail is not None:detail['stage'] = 'metadata_open'
        self.meta = self.paths['frames'].open('xb')
        if detail is not None:detail['metadata_open_wall_s'] = time.monotonic()-started
        for kind in self.kinds:
            # Avoid FFV1's tiny-frame slice-grid row loss. FFmpeg ffv1enc.c's
            # level-3 constraint rejects one slice above 352x288 (CIF); retain
            # its automatic production grid for those larger dimensions.
            codec = (['-c:v', 'libx264rgb', '-crf', '0', '-preset', 'ultrafast', '-tune', 'zerolatency', '-pix_fmt', 'rgb24',
                      '-g', str(self.segment_frames)] if kind == 'rgb' else
                     ['-c:v', 'ffv1', '-level', '3', '-pix_fmt', 'gray16le',
                      '-slices', '1' if shape[0] <= 352 and shape[1] <= 288 else '0', '-slicecrc', '1'])
            argv = [self.ffmpeg, '-nostdin', '-v', 'error', '-n', '-threads', '1', '-filter_threads', '1', '-f', 'rawvideo',
                    '-pixel_format', 'rgb24' if kind == 'rgb' else 'gray16le',
                    '-video_size', f'{shape[0]}x{shape[1]}', '-framerate', '30', '-i', 'pipe:0',
                    '-an', '-threads', str(_DEPTH_ENCODE_THREADS if kind=='depth'else 1), *codec, str(self.paths[kind])]
            started = time.monotonic()
            if detail is not None:detail['stage'] = kind+'_spawn'
            self.children[kind] = self._spawn(argv, self.timeout_s)
            if detail is not None:detail[kind+'_spawn_wall_s'] = time.monotonic()-started

    def _collect_pending(self, *, wait=False):
        future = self.pending
        if future is None or not wait and not future.done():
            return
        started = time.monotonic()
        try:
            manifest = future.result(timeout=self.timeout_s if wait else 0)
        except FutureTimeout:
            if future.done():
                raise  # The commit itself failed with TimeoutError.
            raise TimeoutError('pending segment verification exceeded commit wait budget') from None
        finally:
            elapsed = time.monotonic() - started
            if wait:
                with self._lock:
                    self.timings['pending_wait_wall_s'] += elapsed
                    self.timings['pending_wait_peak_wall_s'] = max(self.timings['pending_wait_peak_wall_s'], elapsed)
        self.segments.append(manifest)
        self._pending.popleft()

    @property
    def pending(self):return self._pending[0][0]if self._pending else None

    @property
    def pending_segment(self):return self._pending[0][1]if self._pending else None

    def add(self, meta, color_png, depth_png):
        if self.closed:
            raise ValueError('segment writer closed')
        shape = _shape(color_png)
        if self.kinds==('rgb',):
            if depth_png is not None:raise ValueError('RGB-only source cannot contain depth')
        elif _shape(depth_png, True) != shape:
            raise ValueError('RGB and depth dimensions differ')
        return self._add_frame(meta, color_png, depth_png, shape, input_format='png')

    def add_pixels(self, meta, rgb_bytes, depth_u16le_bytes, width, height):
        """Write packed RGB uint8 and aligned little-endian uint16 depth unchanged."""
        if self.closed:
            raise ValueError('segment writer closed')
        if self.kinds==('rgb',):
            if (depth_u16le_bytes is not None or type(width)is not int or type(height)is not int
                    or min(width,height)<=0 or width*height*3>_LIMIT or not isinstance(rgb_bytes,bytes)
                    or len(rgb_bytes)!=width*height*3):raise ValueError('bounded RGB-only pixels required')
            shape=width,height
        else:shape = _pixel_shape(rgb_bytes, depth_u16le_bytes, width, height)
        return self._add_frame(meta, rgb_bytes, depth_u16le_bytes, shape, input_format='rgb8_depth16le')

    def _add_frame(self, meta, color, depth, shape, *, input_format):
        row = json.dumps(meta, allow_nan=False, separators=(',', ':')).encode() + b'\n'
        if not isinstance(meta, dict) or len(row) > 64 * 1024:
            raise ValueError('bounded object frame metadata required')
        if self.shape is not None and self.shape != shape:
            raise ValueError('camera dimensions changed')
        started = time.monotonic()
        first = not self.issued_index
        detail = dict(stage='input_ready', input_format=input_format, status='started',
                      started_wall_ns=time.time_ns()) if first else None
        if first:self.timings['first_frame'] = detail
        try:
            self._collect_pending()
            if input_format == 'png':
                if first:detail['stage'] = 'png_decode'
                from PIL import Image
                import numpy as np
                with Image.open(io.BytesIO(color)) as image:
                    color = image.tobytes()
                if 'depth'in self.kinds:
                    with Image.open(io.BytesIO(depth)) as image:
                        depth = np.asarray(image, dtype='<u2').tobytes()
                    _pixel_shape(color, depth, *shape)
                if first:detail['png_decode_wall_s'] = time.monotonic()-started
            if not self.children:
                self._start(shape)
            reference = {'segment': f'{self.camera}/{self.stem}.commit.json', 'frame': self.count}
            before = time.monotonic()
            if first:detail['stage'] = 'rgb_pipe_write'
            self.children['rgb'].write(color)
            if first:detail['rgb_pipe_write_wall_s'] = time.monotonic()-before
            before = time.monotonic()
            if first:detail['stage'] = 'depth_pipe_write'
            if 'depth'in self.kinds:self.children['depth'].write(depth)
            if first:detail['depth_pipe_write_wall_s'] = time.monotonic()-before
            before = time.monotonic()
            if first:detail['stage'] = 'metadata_append'
            self.pixel_hashes['rgb'].update(color)
            if 'depth'in self.kinds:self.pixel_hashes['depth'].update(depth)
            self.meta.write(row)
            if first:detail['metadata_append_wall_s'] = time.monotonic()-before
            self.count += 1
            if self.count == self.segment_frames:
                self._queue_commit()
            if first:detail.update(status='written',stage='complete')
            return reference
        except BaseException as error:
            if first:detail.update(status='failed',error=str(error))
            self.abort()
            raise
        finally:
            if first:detail.update(total_wall_s=time.monotonic()-started,finished_wall_ns=time.time_ns())

    def wait_committed(self):
        """Wait for the already detached segment; keep the active segment open."""
        self._check_active()
        try:
            while self._pending:self._collect_pending(wait=True)
            return list(self.segments)
        except BaseException:
            self.abort()
            raise

    def flush_segment(self):
        """Commit a paused capture's tail while keeping this writer reusable."""
        self._check_active()
        if self.children:self._queue_commit()
        return self.wait_committed()

    def _queue_commit(self):
        # At most two queued/running commits per camera. A slower disk cannot
        # turn closed codec instances or segment jobs into an unbounded queue.
        self._collect_pending()
        if len(self._pending)>=2:self._collect_pending(wait=True)
        with self._lock:
            self._check_active()
            manifest = {'schema': self.schema, 'camera': self.camera, 'segment': int(self.stem),
                        'frame_count': self.count, 'width': self.shape[0], 'height': self.shape[1],
                        'metadata_time_authoritative': True, 'files': {},
                        'decoded_pixel_sha256': {kind: digest.hexdigest() for kind, digest in self.pixel_hashes.items()}}
            snapshot = dict(manifest=manifest, paths=self.paths, children=self.children, meta=self.meta,
                            enqueued_monotonic_s=time.monotonic())
            for child in self.children.values():
                child.process.stdin.close()
            self.children, self.meta, self.count = {}, None, 0
            self._pending.append((self._executor.submit(self._commit, snapshot),snapshot))

    def _commit(self, snapshot):
        started = time.monotonic()
        manifest, paths, metadata = snapshot['manifest'], snapshot['paths'], snapshot['meta']
        detail = dict(segment=manifest['segment'], queue_wait_wall_s=started - snapshot['enqueued_monotonic_s'],
                      codec_finalize_wall_s=0., probe_wall_s=0., persist_hash_wall_s=0., status='failed')
        try:
            self._check_active()
            before = time.monotonic()
            for child in snapshot['children'].values():
                child.finish()
                self._forget(child)
            detail['codec_finalize_wall_s'] = time.monotonic() - before
            before = time.monotonic()
            for kind in self.kinds:
                self._check_active()
                _check_probe(paths[kind], manifest, kind, self.ffprobe, self.timeout_s,
                             factory=self._spawn, release=self._forget,probe_cache=self.probe_cache)
            detail['probe_wall_s'] = time.monotonic() - before
            before = time.monotonic()
            metadata.flush()
            os.fsync(metadata.fileno())
            metadata.close()
            for kind, path in paths.items():
                self._check_active()
                with path.open('rb') as stream:
                    os.fsync(stream.fileno())
                destination = path.with_name(path.name.replace('.partial.', '.'))
                path.rename(destination)
                manifest['files'][kind] = {'path': str(destination.relative_to(self.root)), **_digest(destination)}
            partial = self.directory / f"{manifest['segment']:06d}.commit.partial.json"
            with partial.open('x') as stream:
                json.dump(manifest, stream, allow_nan=False, separators=(',', ':'))
                stream.flush()
                os.fsync(stream.fileno())
            # Abort and publication share a lock: after abort begins, this
            # writer cannot promote another partial segment into the prefix.
            with self._lock:
                self._check_active()
                partial.rename(self.directory / f"{manifest['segment']:06d}.commit.json")
            fd = os.open(self.directory, os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(fd)
            finally:
                os.close(fd)
            detail['persist_hash_wall_s'] = time.monotonic() - before
            detail['status'] = 'committed'
            return manifest
        finally:
            for child in snapshot['children'].values():
                child.abort()
                self._forget(child)
            if not metadata.closed:
                metadata.close()
            detail['commit_wall_s'] = time.monotonic() - started
            with self._lock:
                self.timings['commit_segments'] += detail['status'] == 'committed'
                self.timings['commit_wall_s'] += detail['commit_wall_s']
                self.timings['commit_peak_wall_s'] = max(self.timings['commit_peak_wall_s'], detail['commit_wall_s'])
                for key in ('codec_finalize_wall_s', 'probe_wall_s', 'persist_hash_wall_s'):
                    self.timings[key] += detail[key]
                self.timings['last_commit'] = detail

    def finish(self):
        self._check_active()
        if self.closed:
            return list(self.segments)
        try:
            if self.children:
                self._queue_commit()
            while self._pending:self._collect_pending(wait=True)
            self.closed = True
            if self._owns_executor:
                self._executor.shutdown(wait=True)
            return list(self.segments)
        except BaseException:
            self.abort()
            raise

    def abort(self):
        with self._lock:
            self.closed = self.aborted = True
            pending=tuple(self._pending)
            for future,_ in pending:future.cancel()
            children = tuple(self._owned_children)
        for child in children:
            child.abort()
            self._forget(child)
        if self.meta is not None and not self.meta.closed:
            self.meta.close()
        # Do not close the worker's metadata stream underneath a running write.
        for future,snapshot in pending:
            if future.cancelled():
                if not snapshot['meta'].closed:snapshot['meta'].close()
            else:
                try:future.result(timeout=self.timeout_s)
                except FutureTimeout:
                    if not future.done():raise TimeoutError('segment commit cleanup timed out; retained incomplete files') from None
                except BaseException:pass  # Preserve the caller's original error.
        if self._owns_executor:
            self._executor.shutdown(wait=False, cancel_futures=True)


def verify_segment(root, segment, *, ffprobe='ffprobe', timeout_s=30, probe_cache=None):
    path = _safe(root, segment)
    if not re.fullmatch(r'[A-Za-z0-9_-]+/[0-9]{6}\.commit\.json', segment) or path.stat().st_size > 64 * 1024:
        raise ValueError('bounded committed segment manifest required')
    manifest = json.loads(path.read_bytes())
    kinds={'rgb'}if manifest.get('schema')==_RGB_SCHEMA else {'rgb','depth'}
    if (manifest.get('schema') not in (_SCHEMA,_RGB_SCHEMA) or type(manifest.get('frame_count')) is not int
            or not 1 <= manifest['frame_count'] <= 120 or set(manifest.get('files', {})) != kinds|{'frames'}
            or manifest.get('metadata_time_authoritative') is not True
            or not isinstance(manifest.get('decoded_pixel_sha256'), dict)
            or set(manifest['decoded_pixel_sha256']) != kinds
            or any(not isinstance(value, str) or not re.fullmatch('[0-9a-f]{64}', value)
                   for value in manifest['decoded_pixel_sha256'].values())
            or type(manifest.get('segment')) is not int or manifest['segment'] < 0
            or segment != f"{manifest.get('camera')}/{manifest['segment']:06d}.commit.json"
            or any(type(manifest.get(key)) is not int or manifest[key] < 1 for key in ('width', 'height'))
            or manifest['width'] * manifest['height'] * 3 > _LIMIT):
        raise ValueError('invalid segment manifest')
    for kind, pin in manifest['files'].items():
        extension = {'rgb': 'mp4', 'depth': 'mkv', 'frames': 'jsonl'}[kind]
        expected = f"{manifest['camera']}/{manifest['segment']:06d}.{kind}.{extension}"
        if pin.get('path') != expected:
            raise ValueError('segment member path differs')
        member = _safe(root, pin['path'])
        if _digest(member) != {key: pin.get(key) for key in ('size_bytes', 'sha256')}:
            raise ValueError('segment file hash/size differs')
        if kind != 'frames':
            _check_probe(member, manifest, kind, ffprobe, timeout_s, probe_cache=probe_cache)
    with _safe(root, manifest['files']['frames']['path']).open('rb') as stream:
        for _ in range(manifest['frame_count']):
            row = stream.readline(64 * 1024 + 1)
            if len(row) > 64 * 1024 or not row.endswith(b'\n') or not isinstance(json.loads(row), dict):
                raise ValueError('invalid segment metadata frame')
        if stream.read(1):
            raise ValueError('segment metadata frame count differs')
    return manifest


def _png(raw, width, height, depth):
    if depth:
        swapped = bytearray(len(raw))
        swapped[::2], swapped[1::2] = raw[1::2], raw[::2]
        raw = swapped
    stride = width * (2 if depth else 3)
    pixels = b''.join(b'\0' + raw[index:index + stride] for index in range(0, len(raw), stride))
    def chunk(kind, body):
        return struct.pack('!I', len(body)) + kind + body + struct.pack('!I', zlib.crc32(kind + body))
    return (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('!IIBBBBB', width, height, 16 if depth else 8,
                                                               0 if depth else 2, 0, 0, 0))
            + chunk(b'IDAT', zlib.compress(pixels, 1)) + chunk(b'IEND', b''))


def iter_segment(root, segment, *, ffmpeg='ffmpeg', ffprobe='ffprobe', timeout_s=30, png=True, probe_cache=None):
    """Verify once, then decode one paired frame at a time (never whole video)."""
    manifest = verify_segment(root, segment, ffprobe=ffprobe, timeout_s=timeout_s, probe_cache=probe_cache)
    children = {}
    kinds=('rgb',)if manifest['schema']==_RGB_SCHEMA else ('rgb','depth')
    pixel_hashes = {kind: hashlib.sha256() for kind in kinds}
    try:
        for kind, pixel_format in [('rgb', 'rgb24'), ('depth', 'gray16le')]:
            if kind not in kinds:continue
            path = _safe(root, manifest['files'][kind]['path'])
            children[kind] = _Child([ffmpeg, '-nostdin', '-v', 'error', '-threads', str(_DEPTH_DECODE_THREADS if kind=='depth'else 1), '-filter_threads', '1', '-i', str(path),
                                     '-an', '-threads', '1', '-f', 'rawvideo', '-pix_fmt', pixel_format,
                                     'pipe:1'], timeout_s)
            children[kind].process.stdin.close()
        width, height = manifest['width'], manifest['height']
        with _safe(root, manifest['files']['frames']['path']).open('rb') as metadata:
            for _ in range(manifest['frame_count']):
                meta = json.loads(metadata.readline(64 * 1024 + 1))
                color = children['rgb'].read(width * height * 3)
                depth = children['depth'].read(width * height * 2)if 'depth'in children else None
                pixel_hashes['rgb'].update(color)
                if depth is not None:pixel_hashes['depth'].update(depth)
                yield meta, _png(color, width, height, False) if png else color, _png(depth, width, height, True) if png and depth is not None else depth
        for child in children.values():
            if child.read(1, eof=True):
                raise ValueError('segment contains unexpected extra frames')
            child.finish()
        if {kind: digest.hexdigest() for kind, digest in pixel_hashes.items()} != manifest['decoded_pixel_sha256']:
            raise ValueError('segment decoded pixels differ from original')
    finally:
        for child in children.values():
            child.abort()


def recover(root, camera, *, ffprobe='ffprobe', timeout_s=30):
    """Read-only recovery. A committed prefix alone never qualifies an episode."""
    if not isinstance(camera, str) or not re.fullmatch(r'[A-Za-z0-9_-]+', camera):
        raise ValueError('safe camera name required')
    directory = Path(root) / camera
    if directory.is_symlink() or Path(root).is_symlink():
        raise ValueError('symlink segment root rejected')
    if not directory.exists():
        return {'status':'incomplete','episode_complete':False,'segments':[],
                'frame_count':0,'uncommitted_files':[],'errors':['camera stream not created']}
    commits, errors = [], []
    candidates = sorted(directory.glob('*.commit.json'))
    for index, path in enumerate(candidates):
        if path.name != f'{index:06d}.commit.json':
            errors.append('non-contiguous committed segment prefix')
            break
        try:
            commits.append(verify_segment(root, f'{camera}/{path.name}', ffprobe=ffprobe, timeout_s=timeout_s))
        except (OSError, ValueError, RuntimeError, TimeoutError) as exc:
            errors.append(str(exc))
            break
    known = {Path(pin['path']).name for segment in commits for pin in segment['files'].values()}
    known.update(f"{segment['segment']:06d}.commit.json" for segment in commits)
    uncommitted = sorted(path.name for path in directory.iterdir() if path.name not in known)
    return {'status': 'recoverable' if commits else 'incomplete', 'episode_complete': False,
            'segments': commits, 'frame_count': sum(item['frame_count'] for item in commits),
            'uncommitted_files': uncommitted, 'errors': errors}
