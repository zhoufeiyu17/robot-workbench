"""CPU protocol fixtures only: no GPU search, PhysX, camera, or task qualification."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT/'calibration'), str(ROOT/'calibration/infra'),
               str(ROOT/'rule_based_workbench')]
from examples.scripts.debug import joint_head_real_owner as ref
from examples.scripts.debug.joint_head_fake_hardware import FakeSafety
from arx_env.joint_head_jaw_mapping import configure_mapping
from integration import _digest
from real_control import classes
from physical_motiongen import freeze_physical
from joint_space import convert_state
from control_motion import validate_control
from test_fake_device_pipeline import fake_inputs
from robot_workbench.physx import PhysxBackend, SimulationClock
from robot_workbench.collection_physx import load_inputs, run_prefix,validate_plant_fixture


class LaggingPlant:
    evidence_source = 'cpu_fixture'
    limits = {side: [[-2.,2.]]*6 for side in ('left','right')}

    def __init__(self, initial):
        self.q = {side: initial[side+'_arm_joints'][:]+[initial[side+'_gripper']] for side in self.limits}
        self.v = {side: [0.]*7 for side in self.limits}

    def read(self): return deepcopy(self.q),deepcopy(self.v)

    def step(self, targets):
        for side in self.q:
            delta = [(b-a)*.25 for a,b in zip(self.q[side],targets[side])]
            self.q[side] = [a+d for a,d in zip(self.q[side],delta)]
            self.v[side] = [d*240 for d in delta]


def protocol_fixture(directory, *, source_clock_id='cpu-fake-device-clock-v1', runtime_nonce='cpu-fake-runtime-v1',estimated_offset=.002):
    """Synthetic geometry receipts exercise validators, never authorize the GPU CLI."""
    from jaw_mapping_v2 import COMMAND, FEEDBACK, GAP_MM
    mapping = dict(schema_version='jaw_mapping_audit.v1', reinterpreted_mapping={side: dict(
        candidate_use_allowed=True,command_sdk_knots=COMMAND[side],feedback_sdk_knots=FEEDBACK[side],
        metal_knots_mm=GAP_MM[side]) for side in ('left','right')})
    mapping_path=directory/'jaw.json';mapping_path.write_text(json.dumps(mapping))
    source, model, binding = fake_inputs(source_clock_id=source_clock_id, runtime_nonce=runtime_nonce)
    profile=source['calibration_profile']
    profile['source'].update(kind='installed_fake_calibration',simulation_only=True)
    profile['joints']['zero_offsets_rad']=[estimated_offset]*12
    profile['model_binding']['jaw_mapping_sha256']=hashlib.sha256(mapping_path.read_bytes()).hexdigest()
    initial=source['initial_state']
    initial['calibration_profile_digest']=_digest(profile)
    initial['model']=convert_state(initial['planned_reported'],profile,to_reported=False)
    for row in model['ticks']:
        for side in ('left','right'):
            row['target'][side+'_arm_joints']=initial['model'][side+'_arm_joints'][:]
    model.update(environment='fake',simulation_only=True,calibration_profile_digest=_digest(profile),
                 timeline_digest=_digest(model['ticks']))
    model['control_validation']=validate_control(model)
    model['planning_validation'].update(source_digest=_digest(source),timeline_digest=model['timeline_digest'])
    binding.update(environment='fake',simulation_only=True,evidence_source='fake_device_rgbd',
                   calibration_profile_digest=_digest(profile),source_digest=_digest(source),initial_state=initial)
    plan=freeze_physical(model,source,binding,environment='fake')
    for name,value in (('source',source),('plan',plan),('binding',binding)):
        (directory/(name+'.json')).write_text(json.dumps(value))
    base={'lanes':{'source':{'configuration':{'cameras':{'calibration_binding':deepcopy(profile['model_binding'])}}}}}
    return base,source,plan,binding,mapping_path


class CollectionPhysxProtocolTest(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.directory=Path(self.temp.name)
        self.base,self.source,self.plan,self.binding,self.mapping=protocol_fixture(self.directory)
        self.addCleanup(configure_mapping,None)

    def test_complete_caller_orders_targets_and_revokes_next_before_plant(self):
        collection,_,_=classes(ref,jaw_mapping_file=self.mapping)
        clock=SimulationClock();events=[]
        def event(kind,**fields):events.append(dict(kind=kind,control_time_s=clock(),**fields))
        truth_offsets=[.04]*12
        initial=deepcopy(self.source['initial_state']['planned_reported'])
        for side in ('left','right'):
            initial[side+'_arm_joints']=[q+.04 for q in initial[side+'_arm_joints']]
            initial[side+'_gripper']-=.008
        plant=LaggingPlant(initial)
        backend=PhysxBackend(plant,joint_offsets=truth_offsets,metal_gap_offset_mm=.008,event=event)
        self.base,self.source,self.plan,self.binding,self.mapping=protocol_fixture(self.directory,
            source_clock_id=backend.source_clock_id,runtime_nonce='cpu-resident-runtime')
        owner=collection(backend,FakeSafety,hardware=False,environment='fake',
            hardware_binding=self.directory/'binding.json',control_clock=clock,event=event)
        owner.runtime_nonce='cpu-resident-runtime'
        backend.owner=owner
        for _ in range(12000):clock.advance(owner,backend)
        captures=[]
        def capture(label,tick):
            row=dict(label=label,source_tick=tick,feedback=backend.read_calibration_state(),
                     evidence_source='cpu_fixture',camera_qualified=False)
            captures.append(row);return row
        truth=dict(source_clock_id=backend.source_clock_id,joint_offsets_rad=truth_offsets,metal_gap_offset_mm=.008)
        result=run_prefix(owner,backend,clock,self.source,self.plan,plant_fixture=truth,events=events,capture=capture,source_ticks=2)
        self.assertEqual(result['motion_outcome'],'stopped')
        self.assertEqual(result['source_ticks_applied'],2)
        self.assertEqual(result['source_ticks_submitted'],3)
        self.assertEqual(result['skipped_source_ticks'],0)
        self.assertEqual(result['old_epoch_applications_after_barrier'],0)
        self.assertNotEqual(captures[0]['feedback']['qpos'],captures[1]['feedback']['qpos'])
        self.assertEqual(self.plan['simulation'],{'validated':False,'outcome':'not_run'})
        self.assertFalse(result['coffee_task_qualified'])
        self.assertFalse(result['complete_recording_qualified'])
        first=next(row for row in events if row['kind']=='latched' and row['side']=='left')
        self.assertAlmostEqual(first['target'][0],self.source['initial_state']['planned_reported']['left_arm_joints'][0]+.04)
        self.assertEqual(list(backend.offsets),truth_offsets)
        bad=dict(truth,joint_offsets_rad=self.source['calibration_profile']['joints']['zero_offsets_rad'])
        with self.assertRaisesRegex(ValueError,'actual plant'):validate_plant_fixture(bad,self.source,backend)

    def test_input_gate_retains_initial_binding_and_no_success_promotion(self):
        truth=dict(source_clock_id=self.source['clock']['source_clock_id'],joint_offsets_rad=[.04]*12,metal_gap_offset_mm=.008)
        self.base['lanes']['source']['configuration']['cameras']['calibration_binding']['urdf_sha256']='c'*64
        def load():return load_inputs(self.base,self.directory/'source.json',self.directory/'plan.json',
                                     self.directory/'binding.json',self.mapping,plant_fixture=truth)
        source,plan,_,receipt=load()
        self.assertEqual(source['calibration_profile']['joints']['zero_offsets_rad'],[.002]*12)
        self.assertEqual(receipt['plant_fixture']['joint_offsets_rad'],[.04]*12)
        self.assertNotEqual(receipt['plant_urdf_sha256'],receipt['estimator_urdf_sha256'])
        self.assertEqual(source['environment'],'fake')
        self.assertEqual(plan['simulation'],{'validated':False,'outcome':'not_run'})
        bad=deepcopy(self.source);bad.pop('initial_state')
        (self.directory/'source.json').write_text(json.dumps(bad))
        with self.assertRaises(ValueError):load()
        (self.directory/'source.json').write_text(json.dumps(self.source))
        bad=deepcopy(self.binding);bad['environment']='physical'
        (self.directory/'binding.json').write_text(json.dumps(bad))
        with self.assertRaises(ValueError):load()
        (self.directory/'binding.json').write_text(json.dumps(self.binding))
        bad=deepcopy(self.plan);bad['simulation']={'validated':True,'outcome':'success'}
        (self.directory/'plan.json').write_text(json.dumps(bad))
        with self.assertRaises(ValueError):load()
        (self.directory/'plan.json').write_text(json.dumps(self.plan))
        self.mapping.write_text(self.mapping.read_text()+'\n')
        with self.assertRaisesRegex(ValueError,'mapping'):load()


if __name__=='__main__':unittest.main()
