"""CPU trust-boundary tests. These assertions never import a hardware SDK."""
import hashlib
from pathlib import Path
import sys
import tempfile
import socket
import unittest
from copy import deepcopy
from types import SimpleNamespace

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from robot_workbench.arx_output_guard import FeedbackLatch, TIMEOUT, RELEASE_TIMEOUT
from robot_workbench.sdk_quiesce import validate_contract, verify_module
from test_arx_output_guard import frame


class QuiesceTests(unittest.TestCase):
    def setUp(self):
        self.now=10.;self.build='a'*64
        self.latch=FeedbackLatch(('can3','can1'),lambda:self.now,sdk_contract={'library_build_id':self.build})
        self.latch.begin();self.latch.phase='starting'
        for side in ('left','right'):
            self.latch.register_sdk(side,dict(library_build_id=self.build,controller_instance_id=side+'-instance'))
        self.feed();self.latch.activate();self.latch.phase='releasing';self.latch.started=self.now
        self.report=dict(library_build_id=self.build,controller_instance_id='left-instance',
            caller_generation=self.latch.generation,send_thread_joined=True,idempotent_hit=False,
            last_send_attempt_monotonic_ns=int(self.now*1e9),receiver_state='vendor_owned_unverified',
            quiesce_returned_monotonic_ns=int(self.now*1e9),
            transport_basis='trusted_vendor_binary',verification_scope='controller_application')

    def feed(self):
        for bus,motor in self.latch.expected:
            ident,data=frame(motor)
            self.latch.receive(bus,motor,b'\0'*8,self.now,socket.MSG_DONTROUTE)
            self.latch.receive(bus,ident,data,self.now)

    def test_valid_report_retires_only_its_registered_bus(self):
        self.assertEqual(self.latch.retire_sdk('left',self.report),'can3')
        self.assertEqual(self.latch.retired,{'can3'})
        self.now+=TIMEOUT+.001
        self.assertIn('can1:',self.latch.check())

    def test_bad_identity_scope_and_fabricated_boolean_cannot_retire(self):
        changes={'library_build_id':'b'*64,'controller_instance_id':'right-instance',
            'caller_generation':'prior-run','send_thread_joined':1,'idempotent_hit':0,
            'last_send_attempt_monotonic_ns':True,'receiver_state':'stopped',
            'quiesce_returned_monotonic_ns':int((self.now+1)*1e9),
            'transport_basis':'browser-asserted','verification_scope':'all_hardware'}
        for key,value in changes.items():
            with self.subTest(key=key),self.assertRaises(ValueError):
                self.latch.retire_sdk('left',dict(self.report,**{key:value}))
            self.assertEqual(self.latch.retired,set())
        with self.assertRaises(ValueError):self.latch.retire_sdk('left',{'send_thread_joined':True})

    def test_replay_future_and_late_reports_rejected_with_healthy_rx(self):
        for stamp in (int((self.now-1)*1e9),int((self.now+.01)*1e9)):
            with self.assertRaises(ValueError):self.latch.retire_sdk('left',dict(self.report,last_send_attempt_monotonic_ns=stamp))
        self.now+=.101;self.feed()
        with self.assertRaises(ValueError):self.latch.retire_sdk('left',self.report)
        self.assertEqual(self.latch.retired,set())

    def test_repeat_retire_and_old_generation_fail(self):
        self.latch.retire_sdk('left',self.report)
        with self.assertRaises(ValueError):self.latch.retire_sdk('left',self.report)

    def test_new_connection_after_normal_release_replaces_prior_sdk_generation(self):
        old=dict(self.report);generation=self.latch.generation
        self.latch.retire_sdk('left',old)
        self.latch.retire_sdk('right',dict(old,controller_instance_id='right-instance'))
        self.latch.phase='idle';self.now+=1
        self.latch.begin();self.latch.phase='starting'
        self.assertNotEqual(self.latch.generation,generation)
        self.assertEqual(self.latch.sdk_instances,{})
        self.assertEqual(self.latch.quiesce_reports,{})
        for side in ('left','right'):
            self.latch.register_sdk(side,dict(library_build_id=self.build,controller_instance_id=side+'-new'))
        self.feed();self.latch.activate();self.latch.phase='releasing';self.latch.started=self.now
        with self.assertRaises(ValueError):self.latch.retire_sdk('left',old)
        current=dict(old,controller_instance_id='left-new',caller_generation=self.latch.generation,
            last_send_attempt_monotonic_ns=int(self.now*1e9),quiesce_returned_monotonic_ns=int(self.now*1e9))
        self.assertEqual(self.latch.retire_sdk('left',current),'can3')

    def test_release_deadline_unchanged_even_if_all_buses_retired(self):
        self.latch.retire_sdk('left',self.report)
        self.latch.retire_sdk('right',dict(self.report,controller_instance_id='right-instance'))
        self.now+=RELEASE_TIMEOUT+.001
        self.assertEqual(self.latch.check(),'SDK release deadline exceeded')

    def test_contract_hash_and_loaded_module_are_pinned(self):
        with tempfile.TemporaryDirectory()as tmp:
            path=Path(tmp)/'candidate.so';path.write_bytes(b'cpu-fixture')
            row=dict(path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest())
            value=dict(library_build_id=self.build,module=row,vendor_libraries=[row],vendor_trust='human-approved-unmodified-binary')
            self.assertEqual(validate_contract(value),value)
            verify_module(SimpleNamespace(__file__=str(path),workbench_build_id=self.build),value)
            with self.assertRaises(ValueError):verify_module(SimpleNamespace(__file__=str(path),workbench_build_id='b'*64),value)
            changed=deepcopy(value);changed['vendor_trust']='implicit'
            with self.assertRaises(ValueError):validate_contract(changed)
            path.write_bytes(b'changed')
            with self.assertRaises(ValueError):validate_contract(value)


if __name__=='__main__':unittest.main()
