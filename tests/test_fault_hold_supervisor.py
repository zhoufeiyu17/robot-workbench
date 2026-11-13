"""Guard, actual native output loops and kernel IPC, with memory-only CAN."""
import hashlib
import json
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'tests')]
from robot_workbench import arx_output_guard as guard
from robot_workbench.native_fault_hold import SCHEMA
from test_arx_output_guard import BUS_DRIVER

WORKER = r'''
import importlib.util,json,pathlib,sys,time
sys.path.insert(0,sys.argv[1])
from robot_workbench.arx_output_guard import bind_child,require_guard
mode,directory,module_path=sys.argv[2],pathlib.Path(sys.argv[3]),sys.argv[4]
bind_child();client=require_guard();client.call('begin')
spec=importlib.util.spec_from_file_location('arx5_interface',module_path)
sdk=importlib.util.module_from_spec(spec);spec.loader.exec_module(sdk)
assert sdk.fault_hold_schema=='workbench.native-fault-hold.v1'
arms=[]
for index,side in enumerate(('left','right')):
 robot=sdk.RobotConfigFactory.get_instance().get_config('X5')
 if side=='left':robot.gripper_motor_type=sdk.MotorType.NONE
 config=sdk.ControllerConfigFactory.get_instance().get_config('joint_controller',6)
 native=sdk.Arx5JointController(robot,config,'isolated-'+side)
 native.bind_fault_hold(client.fault_hold.fd,index)
 client.call('register_sdk',side=side,identity=dict(library_build_id='1'*64,controller_instance_id=native.controller_instance_id()))
 command=sdk.JointState(6);command.pos()[:]=.6;command.timestamp=native.get_timestamp()+.3
 native.set_joint_cmd(command);arms.append(native)
client.call('activate');(directory/'active').write_text('ready')
import ctypes
death_signal=ctypes.c_int()
assert ctypes.CDLL(None).prctl(2,ctypes.byref(death_signal),0,0,0)==0 and death_signal.value==0
if mode=='heartbeat':time.sleep(.25)
elif mode=='ipc_closed':client.connection.close();time.sleep(.25)
else:
 deadline=time.monotonic()+2
 while not client.fault_hold.read(3):
  assert time.monotonic()<deadline
  client.call('heartbeat');time.sleep(.005)
deadline=time.monotonic()+1
while not all(client.fault_hold.read(i)for i in(4,5)):
 assert time.monotonic()<deadline
 time.sleep(.002)
poses=[]
for arm in arms:
 state=arm.get_joint_cmd();poses.append(list(state.pos()))
time.sleep(.4) # Includes restored CAN traffic and the original future target.
for arm,pose in zip(arms,poses):
 assert max(abs(x-.2)for x in pose)<1e-6,pose
 state=arm.get_joint_cmd();assert list(state.pos())==pose
 assert arm.get_emergency_state().latched
 try:arm.set_joint_cmd(sdk.JointState(6))
 except sdk.NativeEmergencyLatched:pass
 else:raise AssertionError('faulted controller accepted a new motion')
(directory/'held.json').write_text(json.dumps(client.fault_hold.snapshot()))
if mode!='ipc_closed':client.call('releasing')
for side,arm in zip(('left','right'),arms):
 arm.set_to_damping()
 report=arm.quiesce(generation)
 if mode!='ipc_closed':
  fields=('library_build_id','controller_instance_id','caller_generation','send_thread_joined','last_send_attempt_monotonic_ns','quiesce_returned_monotonic_ns','receiver_state','transport_basis','verification_scope','idempotent_hit')
  value={k:getattr(report,k)for k in fields}
  client.call('retire_'+side,report=value)
if mode!='ipc_closed':client.call('disarm')
'''.replace("client.call('begin')", "admission=client.call('begin');generation=admission['generation']")


@unittest.skipUnless(sys.platform == 'linux' and os.environ.get('WORKBENCH_TEST_NATIVE_HOLD_MODULE'),
                     'isolated native SDK on Linux required')
class NativeHoldSupervisorTests(unittest.TestCase):
    def test_feedback_heartbeat_and_ipc_failures_keep_native_hold(self):
        module = os.environ['WORKBENCH_TEST_NATIVE_HOLD_MODULE']
        for mode in ('loss', 'heartbeat', 'ipc_closed'):
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as directory:
                pairs = [socket.socketpair(type=socket.SOCK_DGRAM) for _ in range(2)]
                class Reader:
                    def __init__(self, sock):self.sock = sock
                    def __getattr__(self, name):return getattr(self.sock, name)
                    def fileno(self):return self.sock.fileno()
                    def recvmsg(self, *args):
                        raw, ancillary, flags, address = self.sock.recvmsg(*args)
                        ident, size, data = struct.unpack('=IB3x8s', raw)
                        if ident & 0x40000000:
                            raw = struct.pack('=IB3x8s', ident & ~0x40000000, size, data)
                            flags |= socket.MSG_DONTROUTE
                        return raw, ancillary, flags, address
                readers = [Reader(pair[0]) for pair in pairs]
                for reader in readers:
                    reader.setsockopt(socket.SOL_SOCKET, 35, 1)
                    reader.setblocking(False)
                fds = [pair[1].fileno() for pair in pairs]
                driver = subprocess.Popen([sys.executable, '-c', BUS_DRIVER, mode, directory, *map(str, fds)], pass_fds=fds)
                pin = dict(path=module, sha256=hashlib.sha256(Path(module).read_bytes()).hexdigest())
                profile = dict(can_interfaces=dict(left='left', right='right'), disabled_grippers=['left'],
                    storage_root=directory, native_fault_hold=SCHEMA,
                    sdk_quiesce=dict(library_build_id='1'*64, module=pin, vendor_libraries=[pin],
                                     vendor_trust='human-approved-unmodified-binary'))
                previous = signal.signal(signal.SIGALRM, lambda *_: (_ for _ in ()).throw(TimeoutError('isolated hold deadline')))
                signal.setitimer(signal.ITIMER_REAL, 8)
                try:
                    with patch.object(guard, 'open_can', side_effect=readers), \
                         patch.object(guard, 'terminate_sdk', side_effect=AssertionError('native hold was forcibly terminated')):
                        result = guard.supervise(profile, [sys.executable, '-c', WORKER, str(ROOT), mode, directory, module])
                    self.assertEqual(result, 0)
                    held = json.loads((Path(directory) / 'held.json').read_text())
                    self.assertTrue(all(held['committed_monotonic_ns'].values()))
                    self.assertEqual(held['mechanical_support'], 'unconfirmed')
                    receipt = json.loads((Path(directory) / 'can-output-guard.json').read_text())
                    self.assertEqual(receipt['process_evidence']['terminations'], [])
                finally:
                    signal.setitimer(signal.ITIMER_REAL, 0)
                    signal.signal(signal.SIGALRM, previous)
                    if driver.poll() is None:driver.kill()
                    driver.wait(timeout=2)
                    for pair in pairs:
                        for sock in pair:sock.close()


if __name__ == '__main__':unittest.main()
