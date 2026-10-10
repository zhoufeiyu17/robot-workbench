"""Read-only native coffee acceptance for a resident Owner-driven fake plant.

Task failures latch evidence. This module has no actuator, stop or fault outlet.
It never resets the environment or borrows an earlier replay's success.
"""
from copy import deepcopy
from dataclasses import asdict
from types import SimpleNamespace


def plant_target(target, fixture):
    """Reported command -> fixed plant coordinates, independent of fitted offsets."""
    result={}
    for i,side in enumerate(('left','right')):
        result[side]=[q+d for q,d in zip(target[side+'_arm_joints'],
            fixture['joint_offsets_rad'][i*6:i*6+6])]+[target[side+'_gripper']-fixture['metal_gap_offset_mm']]
    return result


def prepare_task(plan, source, plant_fixture):
    """Validate/freeze plan inputs off the realtime loop; no Isaac/CUDA import."""
    from integration import _digest
    from physical_motiongen import validate_evidence
    from robot_workbench.collection_physx import validate_plant_fixture
    if source.get('environment')!='fake' or source.get('simulation_only') is not True:
        raise ValueError('native task evidence is limited to explicit fake-device observations')
    fixture=validate_plant_fixture(plant_fixture,source)
    model=validate_evidence(plan,source)
    return dict(plan=deepcopy(plan),model_plan=model,fixture=fixture,
        source_digest=_digest(source),parent_timeline_digest=plan['timeline_digest'])


def prepare_task_execution(prepared,timeline):
    """Compile task-only evidence in the existing D compilation worker."""
    from execution_timeline import validate
    frozen=deepcopy(validate(timeline,prepared['plan']))
    return dict(source_timeline=timeline,timeline=frozen,
                targets=[plant_target(row['target'],prepared['fixture']) for row in frozen['commands']])


class CoffeeTaskMonitor:
    """Create before the first compiled Owner command reaches physics.

    Call observe_current once after every completed backend.advance, and finish
    immediately at the Owner's final feedback endpoint. Construction validates
    the compiled timeline once; it does not change the native loop/action manager.
    """
    def __init__(self,native,backend,prepared,execution_timeline,*,command_epoch,runtime_nonce,prepared_execution=None):
        from integration import _digest
        from isaac_lane import install_grasp_contact_adapter,install_motiongen_replay_policy,SimDistanceActionSpec
        from robot_workbench.collection_physx import validate_plant_fixture
        fixture=prepared['fixture']
        validate_plant_fixture(fixture,{'clock':{'source_clock_id':fixture['source_clock_id']}},backend)
        if getattr(backend.plant,'env',None) is not native.env or getattr(native.env,'_m2_runtime',None) is not native:
            raise ValueError('task readback and Owner backend must share the very same native plant')
        if backend.owner is None or getattr(backend.owner,'environment',None)!='fake' or backend.owner.hardware:
            raise ValueError('native task monitor requires the actual fake CollectionOwner')
        if (type(command_epoch) is not int or not isinstance(runtime_nonce,str) or not runtime_nonce
                or backend.owner.epoch!=command_epoch or backend.owner.runtime_nonce!=runtime_nonce):
            raise ValueError('current execution owner identity required')
        instance=getattr(backend.owner,'execution_instance_id',None)
        if not isinstance(instance,str) or not instance:
            raise ValueError('current Owner acceptance instance required')
        compiled=prepared_execution or prepare_task_execution(prepared,execution_timeline)
        if compiled['source_timeline'] is not execution_timeline:
            raise ValueError('task compilation does not belong to this exact Owner timeline')
        timeline=compiled['timeline']
        model=prepared['model_plan']
        if native.physical_identity.get('cup_contact_preset')!=model.get('cup_contact_preset'):
            raise ValueError('native cup contact preset differs from the frozen plan')
        if timeline['commands'][-1]['stage']!='home_hold':
            raise ValueError('native task qualification requires the full frozen home endpoint')
        self.native,self.backend=native,backend
        self.plant=backend.plant
        self.clock_id=fixture['source_clock_id']
        self.fixture=deepcopy(fixture)
        self.timeline=timeline
        self.targets=compiled['targets']
        self.epoch,self.nonce=command_epoch,runtime_nonce
        self.instance=instance
        self.execution_id=timeline['timeline_digest']
        self.start_step=self.last_step=backend.physics_step
        self.absolute_offset=native.env._sim_step_counter-backend.physics_step
        self.last_tick=-1;self.final_step=None;self.last_context=None
        self.finished=False;self.outcome='running';self.first_failure=None
        self.samples=0;self.last_evidence=None;self.initialization_samples=0
        self.physical_identity=deepcopy(native.physical_identity)
        sample,scene=native.read(native.env._sim_step_counter)
        previous=native.evaluator
        evaluator=type(previous)(native.spec.acceptance,native.cup_profile,
            cup_root=previous.cup_root,robot_root=previous.robot_root,table_shape=previous.table_shape,
            machine_profile_sha256=previous.machine_profile_sha256,
            diagnostic_relax_guards=False,diagnostic_observe_only=False)
        evaluator.home_positions=tuple(self.targets[-1]['left'][:6]+self.targets[-1]['right'][:6])
        evaluator.reset(sample)
        self.runtime=SimpleNamespace(evaluator=evaluator,root=native.root,
            loop=SimpleNamespace(physics_steps=sample.physics_step,terminal=False),
            spec=native.spec,physical_identity=self.physical_identity,
            physical_sha256=_digest(self.physical_identity))
        install_grasp_contact_adapter(self.runtime,diagnostic_compression=bool(model.get('cup_contact_preset')))
        self.policy=install_motiongen_replay_policy(self.runtime,model,applied_context=self._policy_context)
        release_gap=(model['planning_validation']['gpu_final_acceptance']['closing']['end_m']*2000
                     if model.get('cup_contact_preset') else None)
        self.intent=SimDistanceActionSpec(None,model['gripper_opening_mm'],release_contact_mm=release_gap)
        self.identity=dict(environment='fake',simulation_only=True,source_clock_id=self.clock_id,
            execution_instance_id=instance,
            source_digest=prepared['source_digest'],parent_timeline_digest=prepared['parent_timeline_digest'],
            execution_timeline_digest=self.execution_id,plant_fixture_digest=_digest(fixture),
            physical_configuration_sha256=self.runtime.physical_sha256,
            command_epoch=command_epoch,runtime_nonce=runtime_nonce,
            home_positions_model_rad=list(evaluator.home_positions),
            initialization_scope='actual contacts and poses; no parent source tick or geometry certification')

    def _policy_context(self,sample):
        if self.last_context is None:raise ValueError('no actually applied Owner command')
        return dict(self.last_context,final_sample=sample.physics_step==self.final_step)

    def _applied(self):
        backend=self.backend
        left,right=backend.applied_contexts['left'],backend.applied_contexts['right']
        if not isinstance(left,dict) or left!=right:
            raise ValueError('both plant sides require the same actually applied Owner command')
        if (left.get('execution_id')!=self.execution_id or left.get('command_epoch')!=self.epoch
                or left.get('execution_instance_id')!=self.instance
                or getattr(backend.owner,'execution_instance_id',None)!=self.instance
                or left.get('runtime_nonce')!=self.nonce or backend.owner.epoch!=self.epoch
                or backend.owner.runtime_nonce!=self.nonce):
            raise ValueError('stale or mismatched applied Owner identity')
        tick=left.get('execution_tick')
        if type(tick) is not int or not 0<=tick<len(self.targets) or tick not in (self.last_tick,self.last_tick+1):
            raise ValueError('applied command skipped, regressed, or lies outside the compiled execution')
        row=self.timeline['commands'][tick]
        if (left.get('source_tick')!=row['source_tick'] or left.get('stage')!=row['stage']
                or row['source_tick'] is None and row['stage']!='initialize'):
            raise ValueError('applied stage differs from the actual compiled command')
        for side in ('left','right'):
            if any(abs(a-b)>1e-9 for a,b in zip(backend.active[side],self.targets[tick][side])):
                raise ValueError('applied plant target differs from fixed-truth command mapping')
        self.last_tick=tick;self.last_context=deepcopy(left)
        return row

    def _latch(self,reason,*,outcome='unverifiable',detail=None):
        if self.first_failure is None:
            self.outcome=outcome
            self.first_failure=dict(reason=str(reason),detail=detail,physics_step=self.backend.physics_step,
                absolute_physics_step=self.native.env._sim_step_counter,applied_context=deepcopy(self.last_context),
                contact_status=deepcopy(getattr(self.backend.plant,'contact_status',None)),last_observation=deepcopy(self.last_evidence))

    def observe_current(self):
        """Errors invalidate task evidence, never propagate into the motion loop."""
        if self.finished or self.first_failure is not None:return self.outcome
        try:
            backend=self.backend;absolute=self.native.env._sim_step_counter
            if (backend.source_clock_id!=self.clock_id or backend.physics_step!=self.last_step+1
                    or absolute!=backend.physics_step+self.absolute_offset
                    or backend.plant is not self.plant or backend.plant.env is not self.native.env
                    or list(backend.offsets)!=self.fixture['joint_offsets_rad']
                    or backend.gap_offset!=self.fixture['metal_gap_offset_mm']
                    or self.native.physical_identity!=self.physical_identity):
                raise ValueError('plant identity, physical configuration or physics sequence changed')
            row=self._applied()
            contacts=self.backend.plant.contact_status
            if contacts.get('complete') is not True or contacts.get('absolute_physics_step')!=absolute:
                raise ValueError('current native contact transport incomplete')
            sample,scene=self.native.read(absolute)
            if sample.physics_step!=absolute:raise ValueError('native sample step differs from plant')
            self.runtime.loop.physics_steps=absolute
            self.last_step=backend.physics_step
            self.samples+=1;self.initialization_samples+=row['source_tick'] is None
            robot=self.native.env.scene['robot']
            joints=dict(zip(robot.joint_names,robot.data.joint_pos[0].tolist(),strict=True))
            folded_clear,folded_audit=self.native.zero_collision.check(joints)
            commanded_open=None
            if row['source_tick'] is not None:
                self.intent.set_stage(row['stage'])
                target=self.backend.active
                command=[*target['left'][:6],target['left'][6],*target['right'][:6],target['right'][6]]
                commanded_open=self.intent._binary(command)[6]>0
            self.last_evidence=dict(sample=asdict(sample),contacts=[asdict(c) for c in self.native.latest_contacts],
                folded_self_collision=deepcopy(folded_audit),commanded_gripper_open=commanded_open,
                command_intent_basis='measured_jaw_motion' if commanded_open is None else 'actually_applied_stage_and_gap')
            evaluator=self.runtime.evaluator
            if not folded_clear:
                evaluator._current_physics_step=absolute
                event=evaluator.fail('folded_self_collision')
            else:
                event=evaluator.observe(sample,scene,self.native.latest_contacts,dt=self.native.env.physics_dt,
                    complete=True,commanded_gripper_open=commanded_open)
            self.last_evidence['native_audit']=deepcopy(evaluator.last_audit)
            if event is not None and event.reason!='success':
                self._latch(event.reason,outcome='unverifiable' if event.run_status=='unverifiable' else 'failed',
                    detail='native StepEvent.run_status='+event.run_status)
        except Exception as error:
            self._latch('task_observation_unverifiable',detail=f'{type(error).__name__}: {error}')
        return self.outcome

    def finish(self,motion_outcome,final_feedback,*,data_outcome='unknown'):
        """Final success is this Owner endpoint's current native success only."""
        if self.finished:return deepcopy(self.result)
        if self.first_failure is None:
            if motion_outcome!='completed':
                self.outcome='interrupted'
            else:
                try:
                    self._applied()
                    if (final_feedback.get('source_clock_id')!=self.clock_id
                            or final_feedback.get('physics_step')!=self.last_step
                            or final_feedback.get('source_time_ns')!=self.last_step*1_000_000_000//240
                            or self.backend.physics_step!=self.last_step
                            or self.last_tick!=len(self.targets)-1):
                        raise ValueError('motion completion is not this full execution native observation endpoint')
                    self.final_step=self.last_step+self.absolute_offset
                    if self.policy['progress'].get('last_sample_native_success') is not True:
                        self._latch('final_native_task_not_successful',outcome='failed')
                    else:
                        from isaac_lane import finish_motiongen_replay_policy
                        finish_motiongen_replay_policy(self.runtime)
                        self.outcome='success'
                except Exception as error:
                    self._latch('final_task_not_qualified',detail=f'{type(error).__name__}: {error}')
        self.finished=True
        self.result=dict(schema_version='owner_physx_native_task.v1',**self.identity,
            motion_outcome=motion_outcome,data_outcome=data_outcome,task_outcome=self.outcome,
            task_qualified=self.outcome=='success',hardware_qualified=False,
            start_physics_step=self.start_step,last_physics_step=self.last_step,
            observed_physics_samples=self.samples,initialization_samples=self.initialization_samples,
            final_applied_execution_tick=self.last_tick,first_failure=deepcopy(self.first_failure),
            last_observation=deepcopy(self.last_evidence),policy=deepcopy(self.policy),
            motion_policy='task outcome is observed only; Owner control guards are unchanged')
        return deepcopy(self.result)
