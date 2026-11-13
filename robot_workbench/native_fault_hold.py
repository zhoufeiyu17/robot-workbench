"""Private shared latch for the SDK's autonomous, non-resumable fault hold.

The native output loop consumes this page without a Python callback. Receipts
confirm native output only. Motor feedback and mechanical support are separate.
"""
import ctypes
import mmap
import os
import platform
import stat
import struct
import time

from .release_trace import memory_file

SCHEMA = 'workbench.native-fault-hold.v1'
MAGIC = 0x574246484F4C4431
SIZE = 4096
# Aligned uint64 fields, shared with native/workbench_fault_hold.h.
MAGIC_WORD, ARMED, HEARTBEAT, FAULT, LEFT_ACK, RIGHT_ACK = range(6)


class NativeFaultHold:
    def __init__(self, fd, file=None):
        info = os.fstat(fd)
        if (platform.system() != 'Linux' or platform.machine() not in ('x86_64', 'aarch64')
                or not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid()
                or stat.S_IMODE(info.st_mode) != 0o600 or info.st_size != SIZE):
            raise ValueError('private native fault-hold page on supported 64-bit Linux required')
        self.fd, self.file = fd, file
        self.memory = mmap.mmap(fd, SIZE)
        self.address = ctypes.addressof(ctypes.c_char.from_buffer(self.memory))
        atomic = ctypes.CDLL('libatomic.so.1')
        self._load = getattr(atomic, '__atomic_load_8')
        self._load.argtypes = [ctypes.c_void_p, ctypes.c_int]
        self._load.restype = ctypes.c_uint64
        self._store = getattr(atomic, '__atomic_store_8')
        self._store.argtypes = [ctypes.c_void_p, ctypes.c_uint64, ctypes.c_int]
        self._store.restype = None
        self._compare = getattr(atomic, '__atomic_compare_exchange_8')
        self._compare.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint64), ctypes.c_uint64,
                                 ctypes.c_bool, ctypes.c_int, ctypes.c_int]
        self._compare.restype = ctypes.c_bool
        if self.read(MAGIC_WORD) != MAGIC:
            self.memory.close()
            raise ValueError('native fault-hold page version differs')

    @classmethod
    def create(cls):
        file = memory_file('workbench-native-fault-hold', SIZE)
        with mmap.mmap(file.fileno(), SIZE) as memory:
            memory[:] = b'\0' * SIZE
            struct.pack_into('=Q', memory, 0, MAGIC)
        return cls(file.fileno(), file)

    def read(self, word):
        return self._load(self.address + word * 8, 2)  # acquire

    def write(self, word, value):
        self._store(self.address + word * 8, value, 3)  # release

    def arm(self):
        if self.read(ARMED) or self.read(FAULT):
            raise ValueError('native fault hold cannot be rearmed')
        now = time.monotonic_ns()
        self.write(HEARTBEAT, now)
        self.write(ARMED, now)

    def heartbeat(self):
        if not self.read(FAULT):
            self.write(HEARTBEAT, time.monotonic_ns())

    def trip(self, code=1):
        expected = ctypes.c_uint64(0)
        self._compare(self.address + FAULT * 8, ctypes.byref(expected), code, False, 4, 2)

    def snapshot(self):
        return dict(schema=SCHEMA, armed=bool(self.read(ARMED)),
                    fault_code=self.read(FAULT),
                    committed_monotonic_ns=dict(left=self.read(LEFT_ACK), right=self.read(RIGHT_ACK)),
                    evidence='native_output_only', mechanical_support='unconfirmed')

    def close(self):
        self.memory.close()
        if self.file is not None:
            self.file.close()
        else:
            os.close(self.fd)
