"""CPU-only evidence binding: a target replay cannot qualify the resident task."""
from copy import deepcopy
import hashlib
import json
import time
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from integration import _digest
from playground_pipeline import resident_task_receipt,resident_execution_identity


class ResidentTaskReceiptTest(unittest.TestCase):
    def setUp(self):
        self.tmp=TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.root=Path(self.tmp.name);self.execution='e'*64;self.clock=[0.]
        self.instance='a'*32
        self.identity=dict(execution_instance_id=self.instance,runtime_nonce='current-owner',command_epoch=4)
        self.source={'clock':{'source_clock_id':'same-plant-clock'},'run_id':'source'}
        self.plan={'timeline_digest':'p'*64};self.runtime={'timeline_digest':self.execution}
        self.receipt=dict(schema_version='owner_physx_native_task.v1',environment='fake',simulation_only=True,
            hardware_qualified=False,source_clock_id='same-plant-clock',execution_timeline_digest=self.execution,
            execution_instance_id=self.instance,
            parent_timeline_digest=self.plan['timeline_digest'],source_digest=_digest(self.source),
            runtime_nonce='current-owner',command_epoch=4,motion_outcome='completed',task_outcome='success',task_qualified=True)
        self.path=self.root/(self.instance+'.json');self.save()
        self.state=dict(execution_instance_id=self.instance,execution_timeline_digest=self.execution,source_clock_id='same-plant-clock',
            runtime_nonce='current-owner',task_outcome='success',task_qualified=True,
            receipt_path=str(self.path),receipt_sha256=hashlib.sha256(self.path.read_bytes()).hexdigest())
        self.lane=SimpleNamespace(config={'native_task_root':str(self.root)},owner=lambda:self.owner(self.state))

    def owner(self,state):return dict(connected=True,native_task=state,**self.identity,
        execution_calibration=dict(execution_instance_id=self.identity['execution_instance_id'],command_epoch=4,plan_sha256=['c'*64]))

    def save(self):self.path.write_text(json.dumps(self.receipt))

    def wait(self,*,cancel=None):
        with patch('playground_pipeline.time.monotonic',side_effect=lambda:self.clock[0]), \
             patch('playground_pipeline.time.sleep',side_effect=lambda _:self.clock.__setitem__(0,self.clock[0]+.1)):
            return resident_task_receipt(self.lane,self.source,self.plan,self.runtime,.5,execution_identity=self.identity,cancel=cancel)

    def test_pending_receipt_is_waited_then_actual_identity_qualified(self):
        states=[dict(self.state,task_outcome='pending_receipt',task_qualified=False,receipt_path=None,receipt_sha256=None),self.state]
        self.lane.owner=lambda:self.owner(states.pop(0))
        result=self.wait()
        self.assertEqual(self.clock[0],.1);self.assertEqual(result['task_outcome'],'success')
        self.assertTrue(result['native_task']['receipt_verified']);self.assertEqual(result['native_task']['scope'],'L2_same_plant')

    def test_wrong_receipt_cannot_promote_success(self):
        for field,value in (('source_clock_id','other-clock'),('execution_timeline_digest','f'*64),
                            ('source_digest','f'*64),('task_qualified',False),('runtime_nonce','old-owner'),('command_epoch',3),
                            ('execution_instance_id','b'*32)):
            with self.subTest(field=field):
                original=deepcopy(self.receipt);self.receipt[field]=value;self.save()
                self.state['receipt_sha256']=hashlib.sha256(self.path.read_bytes()).hexdigest()
                result=self.wait();self.assertEqual(result['task_outcome'],'unverified')
                self.receipt=original
        self.save();self.state['receipt_sha256']='0'*64
        self.assertEqual(self.wait()['task_outcome'],'unverified')
        self.state['receipt_path']=str(self.root.parent/'other-task.json')
        self.assertEqual(self.wait()['task_outcome'],'unverified')

    def test_failed_task_is_independent_and_missing_receipt_is_bounded(self):
        self.receipt.update(task_outcome='failed',task_qualified=False);self.save()
        self.state.update(task_outcome='failed',task_qualified=False,receipt_sha256=hashlib.sha256(self.path.read_bytes()).hexdigest())
        self.assertEqual(self.wait()['task_outcome'],'failed')
        self.state.update(task_outcome='pending_receipt',receipt_path=None,receipt_sha256=None)
        self.assertEqual(self.wait()['task_outcome'],'unverified');self.assertGreaterEqual(self.clock[0],.5)
        self.clock[0]=0
        with self.assertRaises(InterruptedError):self.wait(cancel=lambda:True)
        self.state.update(task_outcome='unverifiable',message='disk full')
        self.assertEqual(self.wait()['native_task']['message'],'disk full')

    def test_repeat_same_trajectory_uses_new_instance_and_rejects_old_success(self):
        self.assertEqual(self.wait()['task_outcome'],'success')
        self.identity['execution_instance_id']='b'*32
        # Same clock/epoch/nonce/trajectory, but the still-published old success is unusable.
        self.assertEqual(self.wait()['task_outcome'],'unverified')
        self.clock[0]=0.;self.state['execution_instance_id']='b'*32
        self.assertEqual(self.wait()['task_outcome'],'unverified')  # Old path cannot be used either.
        self.path=self.root/('b'*32+'.json');self.receipt['execution_instance_id']='b'*32;self.save()
        self.state.update(receipt_path=str(self.path),receipt_sha256=hashlib.sha256(self.path.read_bytes()).hexdigest())
        self.assertEqual(self.wait()['task_outcome'],'success')
        # Freeze one accepted instance; changing the Owner again cannot retarget that wait.
        expected=dict(self.identity)
        current=self.owner(self.state);current.update(execution_instance_id='c'*32)
        self.lane.owner=lambda:current
        self.assertEqual(resident_task_receipt(self.lane,self.source,self.plan,self.runtime,time.monotonic()+1,
            execution_identity=expected)['task_outcome'],'unverified')

    def test_queued_confirmation_waits_for_new_actual_acceptance(self):
        before=self.owner(self.state);before['execution_instance_id']='0'*32
        before['execution_calibration']['execution_instance_id']='0'*32
        history=[before,self.owner(self.state)]
        self.lane.owner=lambda:history.pop(0)
        with patch('playground_pipeline.time.monotonic',side_effect=lambda:self.clock[0]), \
             patch('playground_pipeline.time.sleep',side_effect=lambda _:self.clock.__setitem__(0,self.clock[0]+.1)):
            result=resident_execution_identity(self.lane,{'queued':'confirm','plan_sha256':'c'*64},before,.5)
        self.assertEqual(result,self.identity);self.assertEqual(self.clock[0],.1)
        # A handoff snapshot with only the historical acceptance never counts as a new run.
        self.lane.owner=lambda:before;self.clock[0]=0.
        with patch('playground_pipeline.time.monotonic',side_effect=lambda:self.clock[0]), \
             patch('playground_pipeline.time.sleep',side_effect=lambda _:self.clock.__setitem__(0,self.clock[0]+.1)):
            with self.assertRaises(TimeoutError):
                resident_execution_identity(self.lane,{'queued':'confirm','plan_sha256':'c'*64},before,.5)


if __name__=='__main__':unittest.main()
