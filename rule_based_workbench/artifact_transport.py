"""Receipt-bound artifact transport. No hardware access; raw stays on its host.

Only explicitly listed files move. SSH uses a fixed Python receiver and encoded
JSON, never caller-supplied shell commands. Failed destinations retain a partial
marker; retries accept only an already complete, byte-identical destination.
Trusted roots must be owned by this task (no concurrent untrusted writers).
"""
import base64
import hashlib
import json
from pathlib import Path, PurePosixPath
import re
import shlex
import shutil
import subprocess
import sys
import tarfile
import tempfile

RECEIPT = '.transport.json'
PARTIAL = '.transport.partial'


def relative(name):
    path = PurePosixPath(name)
    if (not isinstance(name, str) or not path.parts or path.is_absolute()
            or str(path) != name or any(p in ('.', '..') for p in path.parts)
            or path.parts[0] in (RECEIPT, PARTIAL)):
        raise ValueError('artifact path must be canonical, relative and non-reserved')
    return path


def contained(root, name=None):
    root = Path(root)
    if not root.is_absolute() or '..' in root.parts:
        raise ValueError('trusted root must be absolute')
    path = root if name is None else root / relative(name)
    for ancestor in (path, *path.parents):
        if ancestor.is_symlink():
            raise ValueError('symbolic links are forbidden')
    if not root.is_dir():
        raise ValueError('trusted root must exist')
    return path


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def manifest(root, names):
    files = {}
    for name in names:
        path = contained(root, name)
        if name in files or not path.is_file():
            raise ValueError('distinct regular files required')
        files[name] = dict(size=path.stat().st_size, sha256=digest(path))
    value = dict(schema_version='artifact_transport.v1', files=files)
    validate_manifest(value)
    return value


def validate_manifest(value):
    if value.get('schema_version') != 'artifact_transport.v1' or not isinstance(value.get('files'), dict) or not value['files']:
        raise ValueError('nonempty artifact manifest required')
    for name, row in value['files'].items():
        relative(name)
        if (type(row.get('size')) is not int or row['size'] < 0
                or not isinstance(row.get('sha256'), str)
                or not re.fullmatch('[0-9a-f]{64}', row['sha256'])):
            raise ValueError('invalid file size or digest')
    return value


def verify(root, expected):
    validate_manifest(expected)
    if manifest(root, expected['files']) != expected:
        raise ValueError('artifact digest mismatch')


def pack(root, expected, stream):
    verify(root, expected)
    with tarfile.open(fileobj=stream, mode='w|') as archive:
        for name, row in expected['files'].items():
            path = contained(root, name)
            info = tarfile.TarInfo(name)
            info.size = row['size']
            info.mode = 0o600
            with path.open('rb') as source:
                archive.addfile(info, source)


def unpack(root, destination, expected, stream):
    validate_manifest(expected)
    target = contained(root, destination)
    if target.exists():
        if (target / PARTIAL).exists():
            raise ValueError('partial destination retained; choose a new destination')
        receipt = target / RECEIPT
        if receipt.is_symlink() or not receipt.is_file() or json.loads(receipt.read_text()) != expected:
            raise ValueError('refusing to overwrite existing destination')
        verify(target, expected)
        # Drain a repeated upload so SSH does not report a closed input channel.
        while stream.read(1024 * 1024):
            pass
        return target
    # Parents must already exist in the task-owned root; exclusive mkdir reserves
    # this destination before any bytes arrive, including concurrent retries.
    target.mkdir()
    (target / PARTIAL).write_text('transfer incomplete\n')
    seen = set()
    with tarfile.open(fileobj=stream, mode='r|*') as archive:
        for member in archive:
            relative(member.name)
            row = expected['files'].get(member.name)
            if not member.isfile() or row is None or member.name in seen or member.size != row['size']:
                raise ValueError('unexpected, duplicate or non-regular archive entry')
            seen.add(member.name)
            output = contained(target, member.name)
            output.parent.mkdir(parents=True, exist_ok=True)
            with archive.extractfile(member) as source, output.open('xb') as sink:
                shutil.copyfileobj(source, sink, length=1024 * 1024)
            if digest(output) != row['sha256']:
                raise ValueError('artifact digest mismatch')
    if seen != set(expected['files']):
        raise ValueError('archive missing expected files')
    verify(target, expected)
    (target / RECEIPT).write_text(json.dumps(expected, sort_keys=True))
    (target / PARTIAL).unlink()
    return target


def _worker(request):
    if request['operation'] == 'receive':
        unpack(request['root'], request['directory'], request['manifest'], sys.stdin.buffer)
    elif request['operation'] == 'send':
        pack(contained(request['root'], request['directory']), request['manifest'], sys.stdout.buffer)
    else:
        raise ValueError('unknown operation')


def ssh_argv(endpoint, request):
    host, port, key = endpoint['host'], endpoint['port'], endpoint['key']
    if not isinstance(host, str) or not re.fullmatch(r'[A-Za-z0-9_][A-Za-z0-9_.@:-]*', host):
        raise ValueError('explicit SSH host required')
    if type(port) is not int or not 1 <= port <= 65535 or not Path(key).is_absolute():
        raise ValueError('explicit SSH port and absolute key required')
    source = base64.b64encode(Path(__file__).read_bytes()).decode('ascii')
    data = base64.b64encode(json.dumps(request).encode()).decode('ascii')
    code = f"import base64; exec(compile(base64.b64decode('{source}'),'<transport>','exec')); _worker(json.loads(base64.b64decode('{data}')))"
    return ['ssh', '-T', '-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=yes',
            '-o', 'ConnectTimeout=15', '-p', str(port), '-i', key, '--', host,
            shlex.join(['python3', '-B', '-c', code])]


def transfer(*, direction, local_root, local_directory, remote_root,
             remote_directory, expected, endpoint=None, timeout=600, cancel=None):
    """upload/download; endpoint=None is the local-directory transport test double.

    For upload local_directory is a sealed capture; download selects only target
    artifacts. expected must come from a trusted manifest, not from the archive.
    """
    validate_manifest(expected)
    if direction not in ('upload', 'download'):
        raise ValueError('direction must be upload or download')
    relative(local_directory)
    relative(remote_directory)
    request = dict(operation='receive' if direction == 'upload' else 'send',
                   root=str(remote_root), directory=remote_directory, manifest=expected)
    # ponytail: disk-spooled archive bounds RAM; streaming can replace it if measured
    # temporary storage becomes the bottleneck. No model weights belong here.
    with tempfile.TemporaryFile() as archive:
        if direction == 'upload':
            pack(contained(local_root, local_directory), expected, archive)
            archive.seek(0)
            if endpoint is None:
                return unpack(remote_root, remote_directory, expected, archive)
            from cancellable_process import run
            run(ssh_argv(endpoint, request), stdin=archive, stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE, check=True, timeout=timeout, cancel=cancel)
            return str(PurePosixPath(remote_root) / remote_directory)
        if endpoint is None:
            pack(contained(remote_root, remote_directory), expected, archive)
        else:
            from cancellable_process import run
            run(ssh_argv(endpoint, request), stdout=archive, stderr=subprocess.PIPE,
                check=True, timeout=timeout, cancel=cancel)
        archive.seek(0)
        return unpack(local_root, local_directory, expected, archive)
