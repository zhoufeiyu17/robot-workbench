"""Inherited release evidence survives a killed CPU stand-in. No SDK or CAN."""
import os
from pathlib import Path
import select
import signal
import struct
import subprocess
import sys
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from robot_workbench.release_trace import ReleaseTrace,HEADER,ROW,SLOTS,COUNT_OFFSET


CHILD=r'''import os,sys,threading,time
from robot_workbench.release_trace import ReleaseTrace
trace=ReleaseTrace.inherit(int(sys.argv[1]));ready=int(sys.argv[2]);mode=sys.argv[3]
assert not os.get_inheritable(trace.fd)
trace.begin();trace.mark('guard_releasing','returned',time.monotonic_ns())
trace.mark('destroy_left','started',time.monotonic_ns())
if mode=='destroy':
 class Controller:
  def __del__(self):os.write(ready,b'1');time.sleep(30)
 controller=Controller();del controller
else:
 time.sleep(float(mode) if mode!='lock'else .001)
trace.mark('destroy_left','returned',time.monotonic_ns())
trace.mark('retire_left','started',time.monotonic_ns())
if mode=='lock':
 lock=threading.Lock();lock.acquire();os.write(ready,b'1')
 with lock:pass
trace.mark('retire_left','returned',time.monotonic_ns())
'''


class ReleaseTraceTests(unittest.TestCase):
    def local(self):
        reader=ReleaseTrace.create();self.addCleanup(reader.close)
        writer=ReleaseTrace.inherit(os.dup(reader.fd));self.addCleanup(writer.close)
        return reader,writer

    def test_disabled_armed_writer_and_release_stores_have_no_io_or_clock_calls(self):
        reader,writer=self.local();writer.mark('destroy_left','started',1)
        self.assertEqual(reader.snapshot(os.getpid())['status'],'not_started')
        with patch('builtins.open',side_effect=AssertionError('file IO in mark')),\
                patch('os.write',side_effect=AssertionError('write in mark')),\
                patch('os.fsync',side_effect=AssertionError('sync in mark')),\
                patch('time.monotonic_ns',side_effect=AssertionError('new clock in mark')),\
                patch('threading.Lock',side_effect=AssertionError('lock in mark')):
            writer.begin();writer.mark('guard_releasing','returned',100)
            writer.mark('destroy_left','started',110);writer.mark('destroy_left','returned',160)
        value=reader.snapshot(os.getpid(),not_before_ns=99)
        self.assertEqual(value['status'],'observed');self.assertEqual(value['last_confirmed_step']['monotonic_ns'],160)

    def test_uncommitted_row_is_ignored_and_overflow_remains_explicit(self):
        reader,writer=self.local();writer.begin();writer.mark('destroy_left','started',10)
        ROW.pack_into(writer.buffer,HEADER.size+ROW.size,4,2,20)  # Writer dies before publishing count.
        self.assertEqual(reader.snapshot(os.getpid())['last_confirmed_step']['status'],'started')
        writer.begin()
        for i in range(SLOTS+7):writer.mark('destroy_left','started',i+1)
        value=reader.snapshot(os.getpid());self.assertEqual(value['status'],'partial')
        self.assertEqual(len(value['events']),SLOTS);self.assertEqual(value['truncated'],7)

    def test_corrupt_pid_count_order_and_stale_release_are_diagnostic_unknown(self):
        reader,writer=self.local();writer.begin();writer.mark('destroy_left','started',10)
        self.assertEqual(reader.snapshot(os.getpid()+1)['status'],'unknown')
        self.assertEqual(reader.snapshot(os.getpid(),not_before_ns=11)['status'],'unknown')
        struct.pack_into('<I',writer.buffer,COUNT_OFFSET,SLOTS+1)
        self.assertEqual(reader.snapshot(os.getpid())['status'],'unknown')
        writer.begin();writer.mark('destroy_left','started',20);writer.mark('destroy_left','returned',10)
        self.assertEqual(reader.snapshot(os.getpid())['status'],'unknown')
        writer.begin();writer.mark('not_a_step','returned',10)
        self.assertEqual(reader.snapshot(os.getpid())['status'],'unknown')

    def test_inherit_rejects_wrong_permissions_without_exposing_contents(self):
        reader=ReleaseTrace.create();self.addCleanup(reader.close)
        os.fchmod(reader.fd,0o644)
        with self.assertRaisesRegex(ValueError,'private release'):ReleaseTrace.inherit(os.dup(reader.fd))

    def child(self,mode):
        reader=ReleaseTrace.create();ready,send=os.pipe()
        env=dict(os.environ,PYTHONPATH=str(ROOT),CUDA_VISIBLE_DEVICES='')
        process=subprocess.Popen([sys.executable,'-B','-c',CHILD,str(reader.fd),str(send),mode],
            cwd=ROOT,env=env,pass_fds=(reader.fd,send),stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
        os.close(send)
        return reader,ready,process

    def test_killed_destructor_and_lock_wait_preserve_distinct_confirmed_boundaries(self):
        for mode,step in [('destroy','destroy_left'),('lock','retire_left')]:
            with self.subTest(mode=mode):
                reader,ready,process=self.child(mode)
                try:
                    self.assertTrue(select.select([ready],[],[],3)[0]);self.assertEqual(os.read(ready,1),b'1')
                    process.kill();self.assertEqual(process.wait(timeout=3),-signal.SIGKILL)
                    value=reader.snapshot(process.pid)
                    self.assertEqual(value['status'],'observed',value)
                    self.assertEqual(value['last_confirmed_step']['step'],step)
                    self.assertEqual(value['last_confirmed_step']['status'],'started')
                finally:
                    if process.poll()is None:process.kill()
                    process.wait(timeout=3);process.stderr.close();os.close(ready);reader.close()

    def test_normal_cpu_destructor_durations_are_measured_without_inventing_native_location(self):
        for delay in (.05,.15):
            with self.subTest(delay=delay):
                reader,ready,process=self.child(str(delay))
                try:
                    self.assertEqual(process.wait(timeout=3),0,process.stderr.read().decode())
                    value=reader.snapshot(process.pid);self.assertEqual(value['status'],'observed',value)
                    events=[row for row in value['events']if row['step']=='destroy_left']
                    self.assertGreaterEqual((events[1]['monotonic_ns']-events[0]['monotonic_ns'])/1e9,delay*.8)
                    self.assertEqual(value['last_confirmed_step']['step'],'retire_left')
                    self.assertEqual(value['last_confirmed_step']['status'],'returned')
                finally:
                    if process.poll()is None:process.kill()
                    process.wait(timeout=3);process.stderr.close();os.close(ready);reader.close()


if __name__=='__main__':unittest.main()
