"""CPU protocol checks and Linux kernel filtering on Unix datagrams, never CAN."""
import socket
import struct
import sys
import unittest

from robot_workbench.can_feedback_filter import program, install
from robot_workbench.arx_output_guard import feedback_motor


def evaluate(rows, packet):
    index = 0
    value = 0
    while index < len(rows):
        code, yes, no, number = rows[index]
        if code in (0x20, 0x30):
            size = 4 if code == 0x20 else 1
            if number+size > len(packet):return 0
            value = int.from_bytes(packet[number:number+size], 'big')
        elif code == 0x54:value &= number
        elif code == 0x15:index += yes if value == number else no
        elif code == 0x06:return number
        else:raise AssertionError(code)
        index += 1
    raise AssertionError('filter did not return')


def frame(ident, byte=0x16, size=8):
    return struct.pack('=IB3x8s', ident, size, bytes([byte])+b'\0'*7)


class FilterTests(unittest.TestCase):
    def test_all_status_bytes_match_existing_feedback_parser(self):
        for gripper in (False, True):
            rows = program(active=True, include_gripper=gripper)
            for ident in (0, 1, 2, 3, 4, 5, 6, 7, 8, 0x7ff, 0x20000000, 0x40000000, 0x80000001):
                for byte in range(256):
                    raw = frame(ident, byte)
                    decoded = feedback_motor(ident, raw[8:], 0)
                    expected = decoded is not None and (gripper or decoded[0] != 8)
                    self.assertEqual(bool(evaluate(rows, raw)), expected, (ident, byte, gripper))
            self.assertFalse(evaluate(rows, frame(0, size=7)))
            self.assertFalse(evaluate(rows, b'\0'*4))

    def test_starting_filter_keeps_echoes_and_queries(self):
        rows = program(active=False)
        for ident in (0, 1, 5, 8, 0x7ff, 0x40000005):
            self.assertTrue(evaluate(rows, frame(ident, 0)))

    @unittest.skipUnless(sys.platform == 'linux', 'Linux socket BPF')
    def test_kernel_filter_retains_faults_and_restores_startup(self):
        receiver, sender = socket.socketpair(type=socket.SOCK_DGRAM)
        receiver.settimeout(.05)
        try:
            queued = frame(5, 0)
            sender.send(queued)
            install(receiver, active=True, include_gripper=False)
            # Installing a socket filter does not rewrite the existing queue.
            # Guard's post-receive validation must still reject old echoes.
            self.assertEqual(receiver.recv(16), queued)
            for raw in (frame(5, 0), frame(1, 0), frame(0, 0x18), frame(0x40000001, 0x20)):
                sender.send(raw)
            with self.assertRaises(TimeoutError):receiver.recv(16)
            for raw in (frame(1, 0x20), frame(0, 0x06), frame(0, 0x26), frame(0, 0xf6)):
                sender.send(raw)
                self.assertEqual(receiver.recv(16), raw)
            install(receiver, active=False)
            raw = frame(8, 0)
            sender.send(raw)
            self.assertEqual(receiver.recv(16), raw)
        finally:
            receiver.close()
            sender.close()

    @unittest.skipUnless(sys.platform == 'linux', 'Linux socket BPF')
    def test_install_failure_is_reported_instead_of_running_unfiltered(self):
        receiver, sender = socket.socketpair(type=socket.SOCK_DGRAM)
        receiver.close()
        try:
            with self.assertRaises(OSError):install(receiver, active=True)
        finally:sender.close()


if __name__ == '__main__':
    unittest.main()
