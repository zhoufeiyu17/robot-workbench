"""Native task wiring CPU fixtures. These never qualify PhysX/GPU task success."""
from copy import deepcopy
from dataclasses import asdict,dataclass
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'calibration/infra'),str(ROOT/'rule_based_workbench'),str(ROOT/'tests')]
from arx_env.joint_head_jaw_mapping import command_sdk
from test_fake_device_pipeline import fake_inputs
from test_physx_boundary import LaggingPlant
from integration import _digest
from control_motion import validate_control
from execution_timeline import build
from physical_motiongen import freeze_physical
from robot_workbench.physx import PhysxBackend
from robot_workbench.physx_task import CoffeeTaskMonitor,prepare_task


@dataclass(frozen=True)
class Acceptance:
    release_timeout_s:float=1.
    open_position_m:float=.041496
    open_tolerance_m:float=.001
    finger_force_max_n:float=100.
    residual_force_max_n:float=2.
    contact_speed_m_s:float=.08
    pregrasp_contact_speed_m_s:float=.16
    slip_m:float=.005
    def finger_contact_speed_limit(self,grasp_confirmed):return self.contact_speed_m_s
    def digest(self):return _digest(asdict(self))


@dataclass(frozen=True)
class Cup:
    candidate_grasp_band_m:tuple=(.04,.065)


@dataclass
class Sample:
    physics_step:int
    jaw_positions:tuple=(.041496,.041496)
    finger_cup_relative_speed_m_s:float=0.
    arm_positions:tuple=()


class Evaluator:
    events={}
    calls=[]
    def __init__(self,spec,cup,**kwargs):
        self.spec,self.cup=spec,cup
        self.__dict__.update(kwargs)
    def reset(self,sample):
        self.previous=sample;self.event=None;self.first_failure_reason=None
        self.history=dict(grasp_confirmed=False);self.last_audit={}
    def fail(self,reason,status='completed'):
        self.first_failure_reason=reason;self.event=NS(reason=reason,status=status);return self.event
    def observe(self,sample,scene,contacts,*,dt,complete,commanded_gripper_open=None):
        assert self.event is None
        assert sample.physics_step==self.previous.physics_step+1
        assert complete
        self.calls.append((sample.physics_step,commanded_gripper_open))
        self.previous=sample
        event=self.events.get(sample.physics_step)
        if event=='success':self.event=NS(reason=event,status='completed')
        elif event:self.fail(event)
        return self.event


class Native:
    def __init__(self,backend):
        self.backend=backend
        self.root='/World/envs/env_0'
        self.physical_identity={'fixture':True}
        self.spec=NS(acceptance=Acceptance())
        self.cup_profile=Cup()
        self.evaluator=Evaluator(self.spec.acceptance,self.cup_profile,cup_root=self.root+'/Cup',
            robot_root=self.root+'/Robot',table_shape='table',machine_profile_sha256='m'*64,
            diagnostic_relax_guards=False,diagnostic_observe_only=False)
        self.env=NS(_sim_step_counter=60,physics_dt=1/240,
            scene={'robot':NS(joint_names=['q0'],data=NS(joint_pos=np.zeros((1,1))))})
        self.env._m2_runtime=self
        backend.plant.env=self.env
        self.latest_contacts=()
        self.zero_collision=NS(check=lambda joints:(True,{'passed':True}))
    def read(self,step):
        return Sample(step,arm_positions=tuple(self.backend.actual['left'][:6]+self.backend.actual['right'][:6])),NS()


def fixture(*,prefix=False,preset=None,corrupt_preset=False):
    backend=PhysxBackend(LaggingPlant(),joint_offsets=[.04]*12,metal_gap_offset_mm=.008)
    source,model,binding=fake_inputs(source_clock_id=backend.source_clock_id,runtime_nonce='fixture-owner')
    start=model['ticks'][0]['target']
    gaps=[82.992,64.992]+[64.992+.2*i for i in range(91)]+[82.992,82.992]
    model['ticks']=[dict(tick=i,time_s=i/30,target=dict(deepcopy(start),left_gripper=gap)) for i,gap in enumerate(gaps)]
    model['stages']=[dict(name=name,start_tick=i,motiongen_receipt_index=0) for name,i in
        [('initial_hold',0),('close_and_hold',1),('open',2),('home',93),('home_hold',94)]]
    model.update(duration_s=len(gaps)/30,native_substeps=[8]*len(gaps),gripper_opening_mm=82.992,
        native_identity={'acceptance':asdict(Acceptance())},timeline_digest=_digest(model['ticks']),
        native_actions=[[*row['target']['left_arm_joints'],row['target']['left_gripper'],
                         *row['target']['right_arm_joints'],row['target']['right_gripper']] for row in model['ticks']])
    model['control_validation']=validate_control(model)
    model['planning_validation']['timeline_digest']=model['timeline_digest']
    if preset is not None:
        model['cup_contact_preset']=deepcopy(preset)
        model['planning_validation']['gpu_final_acceptance']['closing']['end_m']=.038071
    plan=freeze_physical(model,source,binding,environment='fake')
    truth=dict(source_clock_id=backend.source_clock_id,joint_offsets_rad=[.04]*12,metal_gap_offset_mm=.008)
    prepared=prepare_task(plan,source,truth)
    current=deepcopy(plan['ticks'][0]['target']);current['left_arm_joints'][0]+=.003
    limits={side+'_arm_joints':[[-2,2]]*6 for side in ('left','right')}
    limits.update(left_gripper=[0,83],right_gripper=[0,83])
    timeline=build(plan,current_state=current,limits=limits) if prefix else build(plan)
    backend.owner=NS(environment='fake',hardware=False,epoch=3,runtime_nonce='fixture-owner',
        execution_instance_id='a'*32,
        stop_requested=False,release_requested=False,
        runner=NS(runtime=timeline,frozen=NS(command_index=0,ticks=timeline['commands'])))
    native=Native(backend)
    if preset is not None:native.physical_identity['cup_contact_preset']=deepcopy(preset)
    if corrupt_preset:native.physical_identity['cup_contact_preset']['stiffness_n_m']+=1.
    backend.plant.contact_status=dict(complete=True,absolute_physics_step=60)
    monitor=CoffeeTaskMonitor(native,backend,prepared,timeline,command_epoch=3,runtime_nonce='fixture-owner')
    return monitor


def apply(monitor,index,steps=1):
    backend=monitor.backend;backend.owner.runner.frozen.command_index=index
    target=monitor.timeline['commands'][index]['target']
    q=[*target['left_arm_joints'],command_sdk('left',target['left_gripper']),
       *target['right_arm_joints'],command_sdk('right',target['right_gripper'])]
    backend.send('left',q);backend.send('right',q)
    for _ in range(steps):
        backend.advance((backend.physics_step+1)/240)
        monitor.native.env._sim_step_counter+=1
        backend.plant.contact_status=dict(complete=True,absolute_physics_step=monitor.native.env._sim_step_counter)
        monitor.observe_current()


class TaskTest(unittest.TestCase):
    def setUp(self):Evaluator.events={};Evaluator.calls=[]

    def test_actual_plant_preset_must_still_equal_frozen_plan(self):
        preset=dict(preset_id='soft',stiffness_n_m=329.24414834950727,damping_n_s_m=10.,diagnostic_claw_compression=True)
        with self.assertRaisesRegex(ValueError,'native cup contact preset differs'):
            fixture(preset=preset,corrupt_preset=True)

    def test_native_acceptance_and_diagnostic_cup_policy_are_preserved(self):
        preset=dict(preset_id='soft',stiffness_n_m=329.24414834950727,damping_n_s_m=10.,
                    diagnostic_claw_compression=True)
        m=fixture(preset=preset)
        configuration=m.policy['configuration']
        self.assertEqual(configuration['base_acceptance'],asdict(Acceptance()))
        changed={key for key,value in configuration['base_acceptance'].items()
                 if configuration['effective_acceptance'][key]!=value}
        self.assertEqual(changed,{'release_timeout_s'})
        self.assertEqual(configuration['release_timeout_effective_s'],1.+91/30)
        self.assertEqual(configuration['rubber_pad_body_contact']['contact_height_numerical_tolerance_m'],.0001)
        self.assertEqual(m.native.physical_identity['cup_contact_preset'],preset)
        self.assertFalse(m.runtime.evaluator.diagnostic_relax_guards)
        self.assertFalse(m.runtime.evaluator.diagnostic_observe_only)

    def test_actual_stage_prefix_fixed_truth_and_current_endpoint_success(self):
        m=fixture(prefix=True)
        self.assertEqual(m.runtime.evaluator.home_positions,tuple(m.targets[-1]['left'][:6]+m.targets[-1]['right'][:6]))
        # Non-8 spacing and a real initialization prefix cannot shift task intent.
        for i in range(len(m.targets)):apply(m,i,steps=3 if i%2 else 2)
        self.assertEqual(m.outcome,'running',m.first_failure)
        self.assertGreater(m.initialization_samples,0)
        self.assertTrue(all(opened is None for _,opened in Evaluator.calls[:m.initialization_samples]))
        self.assertAlmostEqual(m.targets[0]['left'][0],m.timeline['commands'][0]['target']['left_arm_joints'][0]+.04)
        self.assertFalse(m.policy['progress']['last_sample_native_success'])
        Evaluator.events[m.native.env._sim_step_counter+1]='success'
        apply(m,len(m.targets)-1)
        result=m.finish('completed',m.backend.read_calibration_state(),data_outcome='recording_failed')
        self.assertEqual((result['motion_outcome'],result['task_outcome'],result['data_outcome']),
                         ('completed','success','recording_failed'))
        self.assertEqual(result['policy']['configuration']['actual_final_physics_step'],m.native.env._sim_step_counter)
        self.assertEqual(m.native.evaluator.__dict__.get('event'),None)  # separate task memory
        self.assertNotIn('torch',sys.modules)

    def test_early_success_cannot_hide_task_failure_or_stop_motion(self):
        m=fixture();Evaluator.events={61:'success',62:'robot_forbidden_contact'}
        apply(m,0);apply(m,1)
        self.assertEqual(m.outcome,'failed',m.first_failure)
        self.assertFalse(m.backend.owner.stop_requested)
        apply(m,2)  # unchanged motion outlet still accepts later commands
        self.assertEqual(m.backend.applied_contexts['left']['execution_tick'],2)
        self.assertEqual(len(Evaluator.calls),2)
        result=m.finish('completed',m.backend.read_calibration_state())
        self.assertEqual(result['task_outcome'],'failed')
        self.assertEqual(result['first_failure']['reason'],'robot_forbidden_contact')

    def test_incomplete_contacts_missing_ticks_and_stale_identity_invalidate_only_task(self):
        for mutation in ('contacts','skip','epoch','instance','stage','target'):
            m=fixture();apply(m,0)
            if mutation=='skip':apply(m,2)
            else:
                m.backend.advance(2/240);m.native.env._sim_step_counter+=1
                m.backend.plant.contact_status=dict(complete=True,absolute_physics_step=62)
                if mutation=='contacts':m.backend.plant.contact_status['complete']=False
                elif mutation=='epoch':m.backend.owner.epoch+=1
                elif mutation=='instance':m.backend.owner.execution_instance_id='b'*32
                elif mutation=='stage':m.backend.applied_contexts['right']=dict(m.backend.applied_contexts['right'],stage='open')
                else:m.backend.active['left'][0]+=.01
                m.observe_current()
            self.assertEqual(m.outcome,'unverifiable',(mutation,m.first_failure))
            self.assertFalse(m.backend.owner.stop_requested)

    def test_completed_motion_without_current_native_success_is_not_task_success(self):
        m=fixture();Evaluator.events={61:'success'}
        for i in range(len(m.targets)):apply(m,i)
        result=m.finish('completed',m.backend.read_calibration_state())
        self.assertEqual(result['task_outcome'],'failed')
        self.assertFalse(result['task_qualified'])
        m=fixture();apply(m,0)
        self.assertEqual(m.finish('stopped',m.backend.read_calibration_state())['task_outcome'],'interrupted')

    def test_mismatched_final_feedback_and_changed_physics_cannot_pass(self):
        m=fixture()
        for i in range(len(m.targets)):apply(m,i)
        Evaluator.events[m.native.env._sim_step_counter+1]='success';apply(m,len(m.targets)-1)
        row=m.backend.read_calibration_state();row['physics_step']-=1
        self.assertEqual(m.finish('completed',row)['task_outcome'],'unverifiable')
        m=fixture();m.native.physical_identity['changed']=True;apply(m,0)
        self.assertEqual(m.outcome,'unverifiable')


if __name__=='__main__':unittest.main()
