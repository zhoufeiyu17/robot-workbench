#!/usr/bin/env python3
"""Bounded Shanghai simulator worker for the local preview harness. No robot IO."""
import argparse
import hashlib
import json
import os
import re
from pathlib import Path
import shlex
import subprocess
import tarfile
import time
import uuid


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', type=Path, required=True, help='trusted local deployment JSON')
    p.add_argument('--plan', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    plan = json.loads(args.plan.read_text())
    if not plan.get('stations') or len(plan['stations']) > 60:
        raise ValueError('bounded nonempty simulator station list required')
    config = json.loads(args.config.read_text())
    required = ('host', 'port', 'ssh_key', 'root', 'source', 'assets', 'python_exe', 'isaac_python', 'runner', 'runner_sha256')
    if set(config) != set(required):
        raise ValueError('deployment config requires exactly: ' + ', '.join(required))
    host = config['host']
    if not isinstance(host, str) or not re.fullmatch(r'[A-Za-z0-9_][A-Za-z0-9_.-]*@[A-Za-z0-9][A-Za-z0-9.-]*', host):
        raise ValueError('host must be user@hostname')
    if type(config['port']) is not int or not 1 <= config['port'] <= 65535:
        raise ValueError('invalid SSH port')
    for field in ('root', 'source', 'assets', 'python_exe', 'isaac_python', 'runner'):
        value = config[field]
        if not isinstance(value, str) or not value.startswith('/') or '..' in Path(value).parts or not re.fullmatch(r'/[A-Za-z0-9_./-]+', value):
            raise ValueError('remote paths must be absolute: ' + field)
    if not re.fullmatch('[0-9a-f]{64}', config['runner_sha256']):
        raise ValueError('runner_sha256 must pin the deployed capture file')
    root, source, assets = (config[k] for k in ('root', 'source', 'assets'))
    if not Path(config['runner']).is_relative_to(root):
        raise ValueError('runner must be within the task root')
    remote = root + '/harness-' + uuid.uuid4().hex[:12]
    key = str(Path(config['ssh_key']).expanduser().resolve(strict=True))
    opts = ['-i', key, '-o', 'IdentitiesOnly=yes', '-o', 'BatchMode=yes',
            '-o', 'StrictHostKeyChecking=yes', '-o', 'ConnectTimeout=10']
    ssh = ['ssh', *opts, '-p', str(config['port']), host]
    scp = ['scp', *opts, '-P', str(config['port'])]
    def call(command):
        return subprocess.check_output([*ssh, command], text=True)
    activity = call('nvidia-smi --query-compute-apps=pid --format=csv,noheader').strip()
    if activity:
        raise RuntimeError('GPU process present; isolated preview refused')
    expected_runner = config['runner_sha256']
    if call('sha256sum ' + shlex.quote(config['runner'])).split()[0] != expected_runner:
        raise RuntimeError('Deployed simulator runner changed')
    call('mkdir ' + shlex.quote(remote))
    subprocess.run([*scp, str(args.plan), host + ':' + remote + '/plan.json'], check=True)
    env = {'ROBOT_SIM_ASSET_ROOT': assets, 'PYTHONDONTWRITEBYTECODE': '1',
           'PYTHONPATH': root + '/deps:' + source + '/deps:' + source + '/source/robot_sim',
           'PYTHONEXE': config['python_exe'],
           'OMP_NUM_THREADS': '4', 'CUDA_VISIBLE_DEVICES': '0',
           'XDG_CACHE_HOME': root + '/cache', 'TORCH_EXTENSIONS_DIR': root + '/cache/torch'}
    argv = ['timeout', '--signal=TERM', '--kill-after=15s', '1800s',
            config['isaac_python'], '-B', config['runner'],
            '--source', source, '--assets', assets, '--plan', remote + '/plan.json',
            '--output', remote + '/capture']
    command = 'export ' + ' '.join(k + '=' + shlex.quote(v) for k, v in env.items()) + '; '
    command += shlex.join(argv) + ' > ' + shlex.quote(remote + '/worker.log') + ' 2>&1'
    started = time.time()
    result = subprocess.run([*ssh, command])
    completion = json.loads(call('cat ' + shlex.quote(remote + '/capture/completion.json')))
    args.output.mkdir(parents=True, exist_ok=False)
    # Retrieve only light images, structured state and preview JPEGs. The huge
    # archival GIF and full color duplicates remain on DSW.
    archive = args.output / 'evidence.tar'
    with archive.open('xb') as stream:
        subprocess.run([*ssh, 'cd ' + shlex.quote(remote + '/capture')
                        + ' && tar cf - -- *.json *pinhole.png robot.urdf preview-*.jpg'], stdout=stream, check=True)
    with tarfile.open(archive) as bundle:
        for member in bundle:
            if not member.isfile() or Path(member.name).name != member.name or member.size > 64_000_000:
                raise ValueError('unexpected evidence archive member')
            with (args.output / member.name).open('xb') as stream:
                stream.write(bundle.extractfile(member).read())
    (args.output / 'worker.json').write_text(json.dumps({'remote': remote,
        'elapsed_seconds': time.time() - started, 'local_pid': os.getpid(),
        'exit_code': result.returncode, 'plan_sha256': hashlib.sha256(args.plan.read_bytes()).hexdigest()}, indent=2))
    if result.returncode or completion.get('passed') is not True:
        raise RuntimeError('Simulator failed; inspect remote worker.log')
    print(json.dumps({'passed': True, 'output': str(args.output), 'remote': remote}), flush=True)


if __name__ == '__main__':
    main()
