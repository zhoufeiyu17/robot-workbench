"""Bounded control/diagnostic RAM fds; no robot SDK or CAN device is opened."""
from pathlib import Path
import mmap
import os
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from robot_workbench.release_trace import memory_file, archive_memory_file, ReleaseTrace
from robot_workbench.arx_output_guard import IPCFailureTrace


class ControlMemoryIOTest(unittest.TestCase):
    @unittest.skipUnless(sys.platform=='linux','real libc memfd qualification')
    def test_missing_python_memfd_still_uses_sealed_ram(self):
        import fcntl
        from robot_workbench.release_trace import F_GET_SEALS
        with patch.object(os,'memfd_create',None,create=True),patch(
                'robot_workbench.release_trace.tempfile.TemporaryFile',side_effect=AssertionError('disk fallback')):
            with memory_file('libc-memfd',32)as file:
                self.assertIn('/memfd:',os.readlink('/proc/self/fd/'+str(file.fileno())))
                self.assertFalse(os.get_inheritable(file.fileno()))
                self.assertEqual(fcntl.fcntl(file.fileno(),F_GET_SEALS)&7,7)
                with self.assertRaises(OSError):os.ftruncate(file.fileno(),64)

    @unittest.skipUnless(sys.platform=='linux','Linux memfd errors')
    def test_memfd_failure_propagates_without_disk_fallback(self):
        import errno
        with patch.object(os,'memfd_create',side_effect=OSError(errno.EMFILE,'too many fds'),create=True),patch(
                'robot_workbench.release_trace.tempfile.TemporaryFile',side_effect=AssertionError('disk fallback')):
            with self.assertRaises(OSError)as caught:memory_file('exhausted',32)
        self.assertEqual(caught.exception.errno,errno.EMFILE)

    def test_fixed_memory_and_post_exit_archive_preserve_bytes_and_permissions(self):
        with tempfile.TemporaryDirectory()as directory,memory_file('cpu-log',4096)as file:
            os.write(file.fileno(),b'CPU diagnostic\n')
            target=Path(directory)/'log'
            self.assertFalse(target.exists())
            result=archive_memory_file(file,target)
            self.assertEqual(target.read_bytes(),b'CPU diagnostic\n')
            self.assertEqual(target.stat().st_mode&0o777,0o600)
            self.assertFalse(result['truncated'])
            with self.assertRaises(FileExistsError):archive_memory_file(file,target)

    @unittest.skipUnless(sys.platform=='linux','Linux sealed RAM fd qualification')
    def test_linux_never_falls_back_to_disk_and_cannot_grow(self):
        with patch('robot_workbench.release_trace.tempfile.TemporaryFile',side_effect=AssertionError('disk fallback')):
            with memory_file('cpu-epoch',32)as file:
                self.assertIn('/memfd:',os.readlink('/proc/self/fd/'+str(file.fileno())))
                with mmap.mmap(file.fileno(),32)as writer,mmap.mmap(file.fileno(),32)as reader:
                    writer[:]=b'b'*32
                    self.assertEqual(reader[:],b'b'*32)
                with self.assertRaises(OSError):os.ftruncate(file.fileno(),64)
                with self.assertRaises(OSError):os.ftruncate(file.fileno(),16)
                os.lseek(file.fileno(),32,os.SEEK_SET)
                with self.assertRaises(OSError):os.write(file.fileno(),b'x')
            for cls in (ReleaseTrace,IPCFailureTrace):
                trace=cls.create()
                try:self.assertIn('/memfd:',os.readlink('/proc/self/fd/'+str(trace.fd)))
                finally:trace.close()

    @unittest.skipUnless(sys.platform=='linux','Linux sealed RAM fd qualification')
    def test_full_diagnostic_buffer_is_reported_and_not_extended(self):
        with tempfile.TemporaryDirectory()as directory,memory_file('cpu-full',32)as file:
            os.write(file.fileno(),b'x'*32)
            with self.assertRaises(OSError):os.write(file.fileno(),b'overflow')
            result=archive_memory_file(file,Path(directory)/'log')
            self.assertTrue(result['truncated'])
            self.assertEqual(result['bytes'],32)


if __name__=='__main__':unittest.main()
