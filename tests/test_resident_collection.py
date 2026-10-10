"""CPU resident wiring checks; real Runtime/Runner, no native/GPU qualification."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'scripts'),str(ROOT/'tests'),str(ROOT/'calibration'),
    str(ROOT/'calibration/infra'),str(ROOT/'rule_based_workbench')]
from serve_physx_device import install_notification
from robot_workbench.resident_collection import NativeTaskObserver,write_task_receipt,load_collection_contract
from robot_workbench.physx import PhysxBackend,SimulationClock
from robot_workbench.runtime import Runtime
from test_collection_physx_protocol import protocol_fixture,LaggingPlant
from test_physx_task import Native
from examples.scripts.debug import joint_head_real_owner as ref
from examples.scripts.debug.joint_head_fake_hardware import FakeSafety
from arx_env.joint_head_jaw_mapping import configure_mapping
from local_execution import TARGET


class ResidentCollectionTest(unittest.TestCase):
    def test_install_notification_occurs_only_after_synchronous_revocation(self):
        steps=[];workflow=NS(calibration_installed=lambda value:steps.append(('revoked',value)))
        workbench=NS();callback=install_notification(workflow,workbench,lambda value:steps.append(('queued',value)))
        workbench.on_calibration_installed({'verified':True})
        self.assertEqual([kind for kind,_ in steps],['revoked','queued'])
        self.assertIs(workflow.calibration_installed,callback)
        workflow=NS(calibration_installed=lambda value:(_ for _ in ()).throw(ValueError('installation changed')))
        callback=install_notification(workflow,workbench,lambda value:steps.append(('bad',value)))
        with self.assertRaises(ValueError):callback({})
        self.assertNotIn('bad',[kind for kind,_ in steps])

    def test_explicit_resource_contract_is_required_and_resident_deadline_has_cleanup_reserve(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);template=root/'template.json';template.write_text('{}');runtime=root/'runtime';runtime.mkdir()
            value=dict(schema_version='anyverse.dsw-codex-experiment.v1',identity={'session_id':'fixture'},
                filesystem={'writable_roots':[str(root)]},resources=dict(gpu_uuid=TARGET['gpu_uuid'],
                    cuda_visible_devices=[1],max_workers=4,max_cpu_threads=4,service_lifetime_seconds=2700,max_wall_time_minutes=50),
                collection_setup=dict(profile_template=str(template),detection_runtime_root=str(runtime),
                    plant_max_memory_mib=12288,min_free_memory_mib=16384))
            path=root/'contract.json';path.write_text(json.dumps(value))
            self.assertEqual(load_collection_contract(path,root/'new-run',1,2700),value)
            value['browser_ports']=[18867];path.write_text(json.dumps(value))
            self.assertEqual(load_collection_contract(path,root/'new-run',1,2700),value)
            for ports in ([0],[65536],[True],['127.0.0.1:18867'],['*'],None):
                invalid=deepcopy(value);invalid['browser_ports']=ports;path.write_text(json.dumps(invalid))
                with self.assertRaisesRegex(ValueError,'browser_ports'):load_collection_contract(path,root/'new-run',1,2700)
            value.pop('browser_ports');path.write_text(json.dumps(value))
            with self.assertRaises(ValueError):load_collection_contract(path,root/'new-run',0,2700)
            with self.assertRaises(ValueError):load_collection_contract(path,root/'new-run',1,2990)
            value['resources']['max_workers']=5;path.write_text(json.dumps(value))
            with self.assertRaises(ValueError):load_collection_contract(path,root/'new-run',1,2700)
            value['preview_setup']=dict(port=0,max_memory_mib=12288,auxiliary_max_memory_mib=16384)
            path.write_text(json.dumps(value))
            self.assertEqual(load_collection_contract(path,root/'new-run',1,2700),value)
            value['resources']['max_workers']=4;path.write_text(json.dumps(value))
            with self.assertRaises(ValueError):load_collection_contract(path,root/'new-run',1,2700)
            value.pop('preview_setup')
            value['resources']['max_workers']=1;path.write_text(json.dumps(value))
            with self.assertRaises(ValueError):load_collection_contract(path,root/'new-run',1,2700)

    def test_receipt_hash_is_published_after_write_and_old_observer_cannot_replace_new_state(self):
        with tempfile.TemporaryDirectory() as temp:
            owner=NS(lock=threading.RLock(),native_task_status=dict(execution_timeline_digest='a'*64,
                execution_instance_id='1'*32,
                runtime_nonce='owner',observer_id='observer-one',task_outcome='pending_receipt',task_qualified=False))
            receipt=dict(schema_version='owner_physx_native_task.v1',execution_timeline_digest='a'*64,
                execution_instance_id='1'*32,
                runtime_nonce='owner',observer_id='observer-one',task_outcome='success',task_qualified=True)
            result=write_task_receipt(temp,owner,receipt)
            self.assertEqual(owner.native_task_status['task_outcome'],'success')
            self.assertEqual(hashlib.sha256(Path(result['receipt_path']).read_bytes()).hexdigest(),result['receipt_sha256'])
            current=dict(owner.native_task_status,execution_instance_id='3'*32,observer_id='observer-three',task_outcome='running',task_qualified=False)
            owner.native_task_status=current
            repeated=dict(receipt,execution_instance_id='3'*32,observer_id='observer-three')
            second=write_task_receipt(temp,owner,repeated)
            self.assertNotEqual(result['receipt_path'],second['receipt_path'])
            self.assertEqual(json.loads(Path(second['receipt_path']).read_text())['execution_timeline_digest'],'a'*64)
            current=deepcopy(owner.native_task_status)
            old=dict(receipt,execution_instance_id='2'*32,observer_id='observer-two')
            write_task_receipt(temp,owner,old)
            self.assertEqual(owner.native_task_status,current)
            with self.assertRaises(FileExistsError):write_task_receipt(temp,owner,receipt)
            self.assertEqual(owner.native_task_status,current)

    def test_post_physics_observation_precedes_same_grid_owner_hold_change(self):
        clock=SimulationClock();order=[]
        backend=NS(advance=lambda stamp:order.append(('physics',clock.index)))
        owner=NS(tick=lambda:order.append(('owner',clock.index)))
        for _ in range(60):clock.advance(owner,backend,after_physics=lambda:order.append(('observe',clock.index)))
        self.assertEqual([kind for kind,index in order if index==60],['physics','observe','owner'])
        self.assertEqual(sum(k=='physics' for k,_ in order),12)
        self.assertEqual(sum(k=='owner' for k,_ in order),5)

    def test_real_collection_runner_completes_when_native_task_preparation_is_unqualified(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);_,source,plan,binding,mapping=protocol_fixture(root)
            configure_mapping(mapping);self.addCleanup(configure_mapping,None)
            initial=deepcopy(plan['ticks'][0]['target'])
            for side in ('left','right'):initial[side+'_gripper']-=.008
            plant=LaggingPlant(initial);clock=SimulationClock()
            backend=PhysxBackend(plant,joint_offsets=[0.]*12,metal_gap_offset_mm=.008)
            _,source,plan,binding,mapping=protocol_fixture(root,source_clock_id=backend.source_clock_id)
            native=Native(backend);receipts=[]
            fixture=dict(source_clock_id=backend.source_clock_id,joint_offsets_rad=[0.]*12,metal_gap_offset_mm=.008)
            observer=NativeTaskObserver(native,backend,fixture,lambda owner,receipt:receipts.append(receipt))
            runtime=Runtime(ref,FakeSafety,lambda:backend,device_id='cpu-resident-observer',environment='fake',
                lock_root=root/'locks',control_clock=clock,jaw_mapping_file=mapping,hardware_binding=root/'binding.json',
                initialize_backend=lambda value:value.set_damping_hold(manual=True))
            backend.owner=runtime.owner;observer.attach(runtime.owner)
            credentials={name:runtime.register_business(name) for name in ('calibration','collection')}
            original_step=plant.step
            def step(targets):
                original_step(targets);native.env._sim_step_counter+=1
                plant.contact_status=dict(complete=True,absolute_physics_step=native.env._sim_step_counter)
            plant.step=step
            def advance():
                observer.before_advance(runtime.owner)
                clock.advance(runtime,backend,after_physics=lambda:observer.after_advance(runtime.owner,physics_advanced=True))
                observer.after_advance(runtime.owner,physics_advanced=False)
                runner=runtime.owner.runner
                if runner and getattr(runner,'runtime_started',False):runner.runtime_ready.wait(.002)
            try:
                for _ in range(600):advance()
                runtime.handoff(credentials['calibration'],'collection')
                review=runtime.submit(credentials['collection'],dict(action='preview_frozen',source=source,plan=plan))
                runtime.submit(credentials['collection'],dict(action='confirm',token=review['token'],
                    confirmation='START_REVIEWED_SINGLE_ARM',physical_clearance=True))
                from robot_workbench import physx_task
                original=physx_task.prepare_task_execution;threads=[]
                def compiled(*args,**kwargs):threads.append(threading.get_ident());return original(*args,**kwargs)
                with patch.object(physx_task,'prepare_task_execution',compiled):
                    for _ in range(1800):
                        advance()
                        if receipts:break
                self.assertTrue(receipts)
                self.assertTrue(threads)
                self.assertTrue(all(value!=threading.get_ident() for value in threads))
                self.assertEqual(runtime.owner.runner.status,'READY')
                self.assertEqual(runtime.owner.runner.frozen.status,'completed')
                self.assertEqual(receipts[0]['motion_outcome'],'completed')
                self.assertEqual(receipts[0]['task_outcome'],'unverifiable')
                self.assertFalse(receipts[0]['task_qualified'])
                self.assertEqual(receipts[0]['source_clock_id'],backend.source_clock_id)
                self.assertIsNotNone(receipts[0]['source_digest'])
                self.assertEqual(runtime.owner.native_task_status['task_outcome'],'pending_receipt')
                write_task_receipt(root/'native-tasks',runtime.owner,receipts[0])
                self.assertEqual(runtime.owner.native_task_status['task_outcome'],'unverifiable')
                self.assertFalse(runtime.owner.stop_requested)
            finally:
                runtime.owner.submit(dict(action='release',confirmation='ARMS_SUPPORTED_RELEASE'))
                runtime.tick();runtime.close()


if __name__=='__main__':unittest.main()
