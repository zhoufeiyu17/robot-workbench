"""Per-socket Linux filter for the exact feedback formats used by the Guard.

Startup must keep transmit echoes; active filtering retains every recognized
DM status, including disabled/fault states. No bus or SDK socket is changed.
"""
import ctypes
import socket
import sys


def program(*, active, include_gripper=True):
    if not active:
        return [(0x06, 0, 0, 0xffffffff)]  # BPF_RET | BPF_K
    rows, labels = [], {}
    def emit(code, k, yes=None, no=None):
        rows.append((code, yes, no, k))
    def label(name):
        labels[name] = len(rows)
    emit(0x30, 4)  # CAN data length, BPF_LD | BPF_B | BPF_ABS
    emit(0x15, 8, 'id', 'drop')
    label('id')
    emit(0x20, 0)  # Raw CAN ID is host-endian; BPF word loads are big-endian.
    emit(0x15, socket.htonl(0), 'dm')
    for ident in (1, 2, 4):
        emit(0x15, socket.htonl(ident), 'ec')
    emit(0x06, 0)
    label('dm')
    emit(0x30, 8)
    emit(0x54, 15)  # Motor ID nibble; deliberately do not filter status nibble.
    for motor in ((5, 6, 7, 8) if include_gripper else (5, 6, 7)):
        emit(0x15, motor, 'accept')
    emit(0x06, 0)
    label('ec')
    emit(0x30, 8)
    emit(0x54, 0xe0)
    emit(0x15, 0x20, 'accept', 'drop')
    label('accept')
    emit(0x06, 0xffffffff)
    label('drop')
    emit(0x06, 0)
    return [(code, 0 if yes is None else labels[yes]-index-1,
             0 if no is None else labels[no]-index-1, k)
            for index, (code, yes, no, k) in enumerate(rows)]


def install(sock, *, active, include_gripper=True):
    if sys.platform != 'linux':
        raise RuntimeError('CAN feedback filtering requires Linux')
    class Instruction(ctypes.Structure):
        _fields_ = [('code', ctypes.c_ushort), ('jt', ctypes.c_ubyte),
                    ('jf', ctypes.c_ubyte), ('k', ctypes.c_uint32)]
    class Filter(ctypes.Structure):
        _fields_ = [('length', ctypes.c_ushort), ('instructions', ctypes.POINTER(Instruction))]
    rows = program(active=active, include_gripper=include_gripper)
    instructions = (Instruction * len(rows))(*(Instruction(*row) for row in rows))
    value = Filter(len(rows), instructions)
    libc = ctypes.CDLL(None, use_errno=True)
    if libc.setsockopt(sock.fileno(), socket.SOL_SOCKET, 26, ctypes.byref(value), ctypes.sizeof(value)):
        number = ctypes.get_errno()
        raise OSError(number, 'failed to install CAN receive filter')
