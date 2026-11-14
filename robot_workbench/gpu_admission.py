"""Host-local GPU admission shared by model loaders and conversion jobs."""
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import stat
import subprocess


class GpuAdmissionError(ValueError):
    def __init__(self, code, message, **details):
        self.code = code
        self.details = dict(code=code, **details)
        super().__init__(message)


def desktop_policy(registry):
    path = Path(os.environ.get('WORKBENCH_GPU_POLICY', Path.home()/'.config/robot-workbench/gpu-admission.json'))
    if not path.exists():
        return {row.get('exe') for row in (registry.get('desktop_gpu_processes') or {}).values()}
    info = path.lstat()
    if not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid() or info.st_mode & 0o077 or info.st_size > 65536:
        raise ValueError('GPU admission policy must be a private owned regular file')
    value = json.loads(path.read_text())
    if (set(value) != {'schema', 'desktop_executables'} or value['schema'] != 'workbench.gpu-admission.v1'
            or not isinstance(value['desktop_executables'], list) or len(value['desktop_executables']) > 32
            or any(not isinstance(p, str) or not Path(p).is_absolute() for p in value['desktop_executables'])):
        raise ValueError('explicit GPU desktop executable policy required')
    return set(value['desktop_executables'])


def registered_desktop(pid, registry, proc=Path('/proc')):
    allowed = desktop_policy(registry)
    if not str(pid).isdigit() or not allowed:
        return False
    p = proc/str(pid)
    try:
        raw = (p/'cmdline').read_bytes()
        start = (p/'stat').read_text().rsplit(') ', 1)[1].split()[19]
        if re.search(rb'(?:^|[ \x00])--type=gpu-process(?:[ \x00]|$)', raw) is None:
            return False
        try:
            executable = str((p/'exe').resolve(strict=True))
            groups = [line.split(':', 2)[-1] for line in (p/'cgroup').read_text().splitlines()]
            desktop = any('/session.slice/' in group or re.fullmatch(
                r'/user.slice/user-\d+\.slice/user@\d+\.service/app.slice/app-[^/]+\.scope', group)
                for group in groups)
            if (executable in allowed and p.stat().st_uid == os.getuid() and desktop
                    and (p/'stat').read_text().rsplit(') ', 1)[1].split()[19] == start):
                return True
        except (OSError, RuntimeError):
            pass
        row = (registry.get('desktop_gpu_processes') or {}).get(str(pid)) or {}
        return (row.get('exe') in allowed and row.get('start_ticks') == start
                and row.get('cmdline_sha256') == hashlib.sha256(raw).hexdigest()
                and row.get('boot_id') == (proc/'sys/kernel/random/boot_id').read_text().strip())
    except (OSError, IndexError):
        return False


def lock_path(gpu_uuid):
    if not isinstance(gpu_uuid, str) or not re.fullmatch(r'GPU-[A-Za-z0-9-]{1,100}', gpu_uuid):
        raise ValueError('explicit GPU UUID required')
    directory = Path(os.environ.get('WORKBENCH_GPU_LOCK_DIR',
        Path('/tmp')/('robot-workbench-gpu-'+str(os.getuid()))))
    directory.mkdir(mode=0o700, parents=True, exist_ok=True)
    info = directory.lstat()
    if not stat.S_ISDIR(info.st_mode) or info.st_uid != os.getuid() or info.st_mode & 0o077:
        raise ValueError('private owned GPU lock directory required')
    return directory/(gpu_uuid+'.lock')


def inspect(registry, *, replacing_pids=()):
    gpu_uuid = registry['gpu_uuid']
    lock_path(gpu_uuid)
    value = subprocess.check_output(['nvidia-smi', '--id='+gpu_uuid, '--query-gpu=uuid,memory.free',
        '--format=csv,noheader,nounits'], text=True, timeout=8).strip().split(',')
    if len(value) != 2 or value[0].strip() != gpu_uuid:
        raise GpuAdmissionError('GPU_IDENTITY', 'GPU身份与部署不一致', gpu_uuid=gpu_uuid)
    free = int(value[1])
    minimum=registry.get('min_free_memory_mib',16000)
    if type(minimum)is not int or minimum<0:raise ValueError('nonnegative integer GPU memory requirement required')
    output = subprocess.check_output(['nvidia-smi', '--id='+gpu_uuid, '--query-compute-apps=pid,used_gpu_memory',
        '--format=csv,noheader,nounits'], text=True, timeout=8)
    processes = []
    available = free
    for line in output.splitlines():
        fields = [v.strip() for v in line.split(',')]
        if len(fields) != 2 or not fields[0].isdigit():
            raise ValueError('invalid GPU process inventory')
        pid = int(fields[0])
        memory = int(fields[1]) if fields[1].isdigit() else None
        replacing = pid in replacing_pids
        desktop = registered_desktop(pid, registry)
        row = dict(pid=pid, memory_mib=memory, role='current_model' if replacing else 'desktop' if desktop else 'compute')
        try:
            p = Path('/proc')/str(pid)
            row.update(executable=str((p/'exe').resolve(strict=True)),
                       cgroup=(p/'cgroup').read_text().strip()[:512])
        except OSError:
            row['executable'] = 'unknown'
        processes.append(row)
        if replacing and memory is not None:
            available += memory
    report = dict(gpu_uuid=gpu_uuid, free_mib=free, available_after_replace_mib=available,
                  required_mib=minimum, processes=processes)
    blockers = [p for p in processes if p['role'] == 'compute']
    if blockers:
        text = ', '.join('PID '+str(p['pid'])+' '+Path(p['executable']).name+' '+str(p['memory_mib'])+' MiB' for p in blockers[:16])
        raise GpuAdmissionError('GPU_BUSY', 'GPU被其他计算进程占用：'+text, **report)
    if available < report['required_mib']:
        raise GpuAdmissionError('GPU_MEMORY', 'GPU可用显存不足：'+str(available)+' MiB，需要 '+str(report['required_mib'])+' MiB', **report)
    return report


def acquire(registry):
    path = lock_path(registry['gpu_uuid'])
    fd = os.open(path, os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW, 0o600)
    lease = os.fdopen(fd, 'r+b')
    try:
        info = os.fstat(lease.fileno())
        if not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid() or info.st_mode & 0o077:
            raise ValueError('private owned GPU lock file required')
        try:
            fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            owner = lease.read(4096).decode(errors='replace')
            try:holder=json.loads(owner)
            except ValueError:holder=None
            raise GpuAdmissionError('GPU_BUSY', 'GPU已有工作台任务持有锁：'+owner,
                                    gpu_uuid=registry['gpu_uuid'], holder=holder) from None
        owner = dict(pid=os.getpid(), model_id=registry.get('model_id'), gpu_uuid=registry['gpu_uuid'])
        lease.seek(0);lease.truncate();lease.write(json.dumps(owner).encode());lease.flush()
        lease.admission = inspect(registry)
        return lease
    except BaseException:
        lease.close()
        raise
