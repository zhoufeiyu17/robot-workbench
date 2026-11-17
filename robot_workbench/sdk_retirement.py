"""Explicit supported retirement of an owned SDK whose command channel is lost."""
import json
import ctypes
import os
from pathlib import Path
import platform
import select
import signal
import stat
import time

from .managed_services import process_identity, _matches
from .native_fault_hold import SCHEMA


def _pidfd_syscall(number, *arguments):
    # Linux x86-64 and arm64 share these pidfd syscall numbers. Some packaged
    # Python builds omit the wrappers even when the running kernel supports them.
    if platform.system() != 'Linux' or platform.machine() not in ('x86_64', 'aarch64'):
        raise RuntimeError('pidfd support required for identity-bound SDK retirement')
    library = ctypes.CDLL(None, use_errno=True)
    library.syscall.restype = ctypes.c_long
    result = library.syscall(ctypes.c_long(number), *(ctypes.c_long(value) for value in arguments))
    if result < 0:
        error = ctypes.get_errno()
        raise OSError(error, os.strerror(error))
    return result


def _open_pidfd(pid):
    return os.pidfd_open(pid) if hasattr(os, 'pidfd_open') else _pidfd_syscall(434, pid, 0)


def _signal_pidfd(fd, signum):
    if hasattr(signal, 'pidfd_send_signal'):
        signal.pidfd_send_signal(fd, signum)
    else:
        _pidfd_syscall(424, fd, signum, 0, 0)


def retained_processes(record_path, *, supervisor_pid, owner_pid, script):
    path = Path(record_path)
    parent = path.parent.stat()
    info = path.lstat()
    if (not path.is_absolute() or stat.S_ISLNK(info.st_mode) or not stat.S_ISREG(info.st_mode)
            or info.st_uid != os.getuid() or info.st_size > 65536
            or parent.st_uid != os.getuid() or stat.S_IMODE(parent.st_mode) != 0o700):
        raise ValueError('private owned SDK retirement receipt required')
    record = json.loads(path.read_text())
    hold = record.get('native_fault_hold') or {}
    committed = hold.get('committed_monotonic_ns') or {}
    if (record.get('schema') != 'arx_can_output_guard.v1' or record.get('state') != 'latched'
            or record.get('supervisor_pid') != supervisor_pid
            or hold.get('schema') != SCHEMA or hold.get('armed') is not True
            or not hold.get('fault_code')
            or any(type(committed.get(side)) is not int or committed[side] <= 0
                   for side in ('left', 'right'))):
        raise ValueError('latched SDK with confirmed native hold output required')
    supervisor = process_identity(supervisor_pid)
    worker = process_identity(record.get('child_pid'))
    if (supervisor is None or worker is None or supervisor['ppid'] != owner_pid
            or worker['ppid'] != supervisor_pid
            or any(row['uid'] != os.getuid() or row['script'] != str(script)
                   or row['process_state'] in ('Z', 'X', 'x') for row in (supervisor, worker))
            or supervisor['boot_id'] != worker['boot_id']):
        raise ValueError('current owned SDK supervisor/worker identity not confirmed')
    return record, supervisor, worker


def retire_supported(record_path, *, supervisor_pid, owner_pid, script, arms_supported):
    if arms_supported is not True:
        raise ValueError('current supported-arms confirmation required for retained SDK retirement')
    record, supervisor, worker = retained_processes(record_path,
        supervisor_pid=supervisor_pid, owner_pid=owner_pid, script=script)
    supervisor_fd = _open_pidfd(supervisor_pid)
    try:
        worker_fd = _open_pidfd(worker['pid'])
        try:
            _, current_supervisor, current_worker = retained_processes(record_path,
                supervisor_pid=supervisor_pid, owner_pid=owner_pid, script=script)
            if not _matches(current_supervisor, supervisor) or not _matches(current_worker, worker):
                raise ValueError('SDK identity changed before supported retirement')
            started = time.monotonic_ns()
            # The command socket is already lost. Mechanical support authorizes
            # withdrawing this latched output; the supervisor still reaps its child.
            for signum in (signal.SIGSTOP, signal.SIGKILL):
                _signal_pidfd(worker_fd, signum)
            if not select.select([supervisor_fd], [], [], 5.)[0]:
                raise TimeoutError('SDK supervisor has not completed worker retirement')
        finally:
            os.close(worker_fd)
    finally:
        os.close(supervisor_fd)
    final = json.loads(Path(record_path).read_text())
    evidence = final.get('process_evidence') or {}
    if (final.get('supervisor_pid') != supervisor_pid or final.get('child_pid') != worker['pid']
            or (evidence.get('wait') or {}).get('status') != 'returned'
            or type(evidence.get('returncode')) is not int
            or type(evidence.get('reaped_monotonic_ns')) is not int
            or evidence['reaped_monotonic_ns'] < started):
        raise ValueError('final SDK worker exit/reap receipt missing')
    return dict(schema='workbench.supported-sdk-retirement.v1', arms_supported=True,
        supervisor=supervisor, worker=worker, started_monotonic_ns=started,
        completed_monotonic_ns=time.monotonic_ns(), worker_exit_confirmed=True,
        sdk_close_confirmed=False, prior_fault=record.get('reason'),
        signals=['SIGSTOP', 'SIGKILL'], process_evidence=evidence)
