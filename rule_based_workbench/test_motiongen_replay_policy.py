"""Native guarded-loop CPU fixtures for frozen success timing and release budget."""
import argparse
from copy import deepcopy
from dataclasses import asdict, dataclass
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS

from gripper_distance_plan import NATIVE_DISTANCE_ENCODING
from integration import CUROBO_PLANNER_ID, _digest
from isaac_lane import install_motiongen_replay_policy, finish_motiongen_replay_policy,install_grasp_contact_adapter,legal_tray_contact


def main(native_code):
    root = native_code/'source/robot_sim/robot_sim'
    for suffix in ('', 'asset_cfg','asset_cfg.objects','specs', 'tasks', 'tasks.manager_based', 'tasks.manager_based.coffee_make'):
        name = 'robot_sim'+('.'+suffix if suffix else '')
        module = ModuleType(name); module.__path__ = [str(root.joinpath(*suffix.split('.'))) if suffix else str(root)]
        sys.modules[name] = module
    from robot_sim.tasks.manager_based.coffee_make.generalization_execution import (
        ActionRequest, ControlSpec, GuardedExecution, StepEvent)

    @dataclass(frozen=True)
    class Acceptance:
        release_timeout_s: float = 1.
        open_position_m: float = .041496
        open_tolerance_m: float = .001
        contact_force_limit_n: float = 100.
        finger_force_max_n: float = 100.
        residual_force_max_n: float = 2.
        contact_speed_m_s: float = .08
        pregrasp_contact_speed_m_s: float = .16
        landing_speed_m_s: float = .03
        surface_tolerance_m: float = .001
        edge_margin_m: float = .002
        tilt_rad: float = .08
        support_force_max_n: float = 5.
        slip_m: float = .005
        stable_speed_m_s: float = .01
        stable_angular_speed_rad_s: float = .1
        def finger_contact_speed_limit(self, grasp_confirmed):
            return self.contact_speed_m_s if grasp_confirmed else self.pregrasp_contact_speed_m_s
        def digest(self): return _digest(asdict(self))

    class ActionSpec:
        def decode(self, action):
            return NS(left=NS(joint_positions_rad=action[:6], gripper_open=True),
                      right=NS(joint_positions_rad=action[7:13], gripper_open=True))
        def decode_single_arm_transition(self, old, action, **kwargs): return self.decode(action)

    class Evaluator:
        diagnostic_relax_guards = diagnostic_observe_only = False
        def __init__(self, base, start, events):
            self.spec, self.previous, self.events = base, NS(physics_step=start), events
            self.cup = Cup()
            self.history = {'grasp_confirmed': False}
            self.observed_contacts = ()
            self.event = None
            self.first_success_physics_step = None
            self.native_early_failure_return=False
        def fail(self, reason, **kwargs):
            self.event = StepEvent(reason)
            return self.event
        def observe(self, sample, *args, **kwargs):
            self.observed_contacts = args[1] if len(args)>1 else kwargs.get('contacts', ())
            if self.event is not None:
                raise RuntimeError('cannot update terminal task facts')
            assert sample.physics_step == self.previous.physics_step+1
            reason = self.events.get(sample.physics_step)
            if self.native_early_failure_return and reason in {'grasp_lost','grasp_slip'}:
                return self.fail(reason)
            self.previous = sample
            self.last_landing_limit=self.spec.landing_speed_m_s
            self.last_contact_limit=self.spec.finger_contact_speed_limit(True)
            reason = self.events.get(sample.physics_step)
            self.event = (StepEvent(reason) if reason=='success' else self.fail(reason)) if reason else None
            if reason == 'success' and self.first_success_physics_step is None:
                self.first_success_physics_step = sample.physics_step
            return self.event

    @dataclass(frozen=True)
    class Cup:
        candidate_grasp_band_m: tuple = (.04, .065)
        bottom_radius_m: float = .0347
        bands: tuple = ()

    def fixture(events=None, final_substeps=8, final_gap=82.992):
        base = Acceptance()
        gaps = [82.992, 64.992]+[82.992 if index == 90 else 64.992+.2*index for index in range(91)]+[82.992, final_gap]
        ticks = [dict(tick=i, time_s=i/30, target=dict(left_arm_joints=[0.]*6, right_arm_joints=[0.]*6,
                 left_gripper=gap, right_gripper=82.992)) for i, gap in enumerate(gaps)]
        plan = dict(schema_version='dual_gpu_pipeline.v1', planner_id=CUROBO_PLANNER_ID,
            native_action_encoding=NATIVE_DISTANCE_ENCODING, native_identity=dict(acceptance=asdict(base)),
            ticks=ticks, timeline_digest=_digest(ticks), native_substeps=[8]*(len(ticks)-1)+[final_substeps],
            control=dict(physics_hz=240, substeps=8), gripper_opening_mm=82.992,
            gripper_distance_control=dict(closed_metal_gap_mm=65., native_travel_offset_mm=.008),
            stages=[dict(name=name, start_tick=index) for name, index in
                    (('initial_hold', 0), ('close_and_hold', 1), ('open', 2), ('home', 93), ('home_hold', 94))])
        loop = GuardedExecution(ControlSpec(240, 8, 2., 5., 300.), ActionSpec())
        loop.reset('fixture', tuple([0.]*6+[1.]+[0.]*6+[1.]))
        loop.physics_steps = 40  # Explicitly test a nonzero replay-start endpoint.
        runtime = NS(loop=loop, evaluator=Evaluator(base, 40, events or {}), spec=NS(acceptance=base),
                     physical_identity={'mesh': 'unchanged'}, physical_sha256='f'*64)
        return runtime, plan, base

    def rejected(function):
        try: function()
        except (ValueError, RuntimeError): return
        raise AssertionError('invalid replay terminal policy accepted')

    def execute(runtime):
        return runtime.loop.execute(ActionRequest('fixture', runtime.loop.step_id, runtime.loop.previous),
            lambda command, step: runtime.evaluator.observe(NS(physics_step=step)))

    runtime, plan, base = fixture()
    diagnostic,_,_=fixture()
    diagnostic_receipt=install_grasp_contact_adapter(diagnostic,diagnostic_compression=True)
    assert diagnostic.evaluator.cup.candidate_grasp_band_m==(.0349,.10010000000000001)
    assert diagnostic_receipt['contact_height_numerical_tolerance_m']==.0001
    original_identity = deepcopy(runtime.physical_identity)
    policy = install_motiongen_replay_policy(runtime, plan)
    assert runtime.spec.acceptance is base
    assert runtime.physical_identity == original_identity and runtime.physical_sha256 == 'f'*64
    assert runtime.evaluator.cup.candidate_grasp_band_m == (.035, .100)
    assert policy['configuration']['rubber_pad_body_contact']['planning_recommendation_m'] == [.04, .065]
    assert set(key for key in asdict(base) if asdict(base)[key] != asdict(runtime.evaluator.spec)[key]) == {'release_timeout_s'}
    assert policy['configuration']['planned_open_duration_s'] == 91/30
    assert runtime.evaluator.spec.release_timeout_s == 1.+91/30
    assert policy['configuration']['first_open_physics_step'] == 40+16+1
    assert policy['configuration']['last_open_physics_step'] == 40+93*8
    assert policy['configuration']['final_physics_step'] == 40+len(plan['ticks'])*8
    rejected(lambda: install_motiongen_replay_policy(runtime, plan))

    # Body pads reach the native evaluator; rim/lid pads fail, shallow metal is
    # excluded from grasp evidence. Force/speed violations remain visible.
    root = '/World/envs/env_0'
    cup, finger = root+'/Cup', root+'/Robot/left_link7'
    for shape in ('MiddleBody', 'UpperBody', 'Rim', 'Lid'):
        runtime, plan, _ = fixture(); runtime.root = root
        install_motiongen_replay_policy(runtime, plan)
        contact = NS(actors=(cup, finger), shapes=(cup+'/Collisions/'+shape, finger+'/RealFingerRubberPad'),
                     impulse=(.001,0.,0.), separation_m=0.)
        sample = NS(physics_step=41, jaw_positions=(.0325,.0325), finger_cup_relative_speed_m_s=.01)
        event = runtime.evaluator.observe(sample, None, (contact,), dt=.01)
        if shape in ('MiddleBody', 'UpperBody'):
            assert event is None and runtime.evaluator.observed_contacts == (contact,)
        else:
            assert event.reason == 'finger_cup_forbidden_contact'
    for impulse, speed, filtered in ((.001,.01,True), (2.,.01,False), (.001,.2,False)):
        runtime, plan, _ = fixture(); runtime.root = root
        receipt = install_motiongen_replay_policy(runtime, plan)
        contact = NS(actors=(cup,finger), shapes=(cup+'/Collisions/MiddleBody',finger+'/collisions/mesh'),
                     impulse=(impulse,0.,0.), separation_m=-.001)
        sample = NS(physics_step=41,jaw_positions=(.0325,.0325),finger_cup_relative_speed_m_s=speed)
        runtime.evaluator.observe(sample,None,(contact,),dt=.01)
        assert runtime.evaluator.observed_contacts == (() if filtered else (contact,))
        assert receipt['incidental_contacts']['count'] == int(filtered)

    # A deferred success must not hide the next collision/force/instability/timeout.
    for failure in ('robot_forbidden_contact', 'released_cup_unstable', 'contact_force_limit', 'release_timeout'):
        runtime, plan, _ = fixture({41: 'success', 43: failure})
        policy = install_motiongen_replay_policy(runtime, plan)
        result = execute(runtime)
        assert result.event.reason == failure and result.actual_substeps == 3 and runtime.loop.terminal, result
        assert runtime.evaluator.event is result.event
        assert policy['progress']['deferred_success_count'] == 1
        rejected(lambda: finish_motiongen_replay_policy(runtime))

    # Past success is not sufficient when the final native observation no longer succeeds.
    runtime, plan, _ = fixture({42: 'success'}, final_gap=20.)
    policy = install_motiongen_replay_policy(runtime, plan)
    assert policy['configuration']['restored_nonopen_jaw_may_prevent_success']
    for _ in plan['ticks']:
        result = execute(runtime)
        assert result.actual_substeps == 8 and result.event is None
    assert runtime.evaluator.first_success_physics_step == 42
    rejected(lambda: finish_motiongen_replay_policy(runtime))

    # Both full and partial last ticks must stop exactly at the frozen final substep.
    for final_substeps in (8, 3):
        runtime, plan, _ = fixture(final_substeps=final_substeps)
        final_step = runtime.loop.physics_steps+sum(plan['native_substeps'])
        runtime.evaluator.events = {43: 'success', final_step: 'success'}
        policy = install_motiongen_replay_policy(runtime, plan)
        for expected in plan['native_substeps']:
            result = execute(runtime)
            assert result.actual_substeps == expected
        assert result.event.reason == 'success' and runtime.loop.terminal
        assert finish_motiongen_replay_policy(runtime) is policy
        assert runtime.evaluator.first_success_physics_step == 43
        assert policy['progress']['final_native_success'] and policy['progress']['last_observed_step'] == final_step

    runtime, plan, _ = fixture()
    runtime._forced_diagnostic_events = []
    rejected(lambda: install_motiongen_replay_policy(runtime, plan))
    runtime, plan, _ = fixture()
    plan['native_identity']['acceptance']['release_timeout_s'] = 99.
    rejected(lambda: install_motiongen_replay_policy(runtime, plan))
    assert 'torch' not in sys.modules
    from robot_sim.asset_cfg.objects.coffee_machine import RigidTransform as Pose
    scene=NS(world_from_support=Pose(),support_polygon_world_m=[(-.1,-.1,0),(.1,-.1,0),(.1,.1,0),(-.1,.1,0)],support_shape_path='tray')
    contact=NS(actors=('cup','machine'),shapes=('bottom','tray'),position=(0,0,0),normal=(0,0,1),impulse=(0,0,.0004))
    # Scoped tray yielding does not require prior force confirmation and does
    # not reject a transient speed; native force failures remain immediate.
    for reason,speed,history,loaded,expected in (
        ('grasp_lost',.0028,True,True,None),
        ('grasp_lost',.24,True,True,None),
        ('grasp_lost',.0028,False,True,None),
        ('grasp_lost',.0028,True,False,None),
        ('grasp_slip',.24,False,True,None),
        ('contact_force_limit',.0028,True,True,'contact_force_limit')):
        runtime,plan,base=fixture({57:reason})
        plan['cup_contact_preset']={'preset_id':'medium'}
        plan['planning_validation']={'gpu_final_acceptance':{'closing':{'end_m':.038071}}}
        e=runtime.evaluator;e.cup_root='cup';e._lowest_z=lambda pose:pose.position[2]
        e.history.update(support_acquired=history,grasp_confirmed=True,table_separated=True)
        policy=install_motiongen_replay_policy(runtime,plan);e.previous=NS(physics_step=56)
        e.native_early_failure_return=True
        sample=NS(physics_step=57,world_from_cup=Pose(),cup_velocity=(speed,0,0),cup_angular_velocity=(0,0,0),
                  finger_cup_relative_speed_m_s=.25)
        c=deepcopy(contact)
        if not loaded:c.impulse=(0,0,0)
        event=e.observe(sample,scene,(c,),dt=1/240)
        assert (event.reason if event else None)==expected
        if event is None:
            assert e.previous is sample, 'observed early-return failure left the previous sample stale'
            next_sample=deepcopy(sample);next_sample.physics_step=58
            assert e.observe(next_sample,scene,(c,),dt=1/240) is None
        assert e.last_landing_limit==max(.16,speed)
        assert e.last_contact_limit==.25
        assert e.spec.landing_speed_m_s==.03, 'contact-local override leaked into later motion'
        assert e.spec.contact_speed_m_s==.08, 'placement override leaked into airborne motion'
    assert legal_tray_contact(e,sample,scene,(contact,),1/240)
    outside=deepcopy(sample);outside.world_from_cup=Pose((.08,0,0))
    assert not legal_tray_contact(e,outside,scene,(contact,),1/240)
    # Small-gap release is permitted without tray force. Out-of-tray, high,
    # airborne, and post-release cases retain grasp-loss rejection.
    for x,z,stage,released,expected in (
        (0,.002,'open',False,None),
        (.08,.002,'open',False,'grasp_lost'),
        (0,.02,'open',False,'grasp_lost'),
        (0,.002,'transport',False,'grasp_lost'),
        (0,.002,'open',True,'grasp_lost')):
        step=57 if stage=='open' else 49
        runtime,plan,base=fixture({step:'grasp_lost'});plan['cup_contact_preset']={'preset_id':'medium'}
        plan['planning_validation']={'gpu_final_acceptance':{'closing':{'end_m':.038071}}}
        if stage!='open':
            plan['stages'][1]['name']=stage
        e=runtime.evaluator;e.cup_root='cup';e._lowest_z=lambda pose:pose.position[2]
        e.history.update(grasp_confirmed=True,table_separated=True,release_confirmed=released)
        install_motiongen_replay_policy(runtime,plan);e.previous=NS(physics_step=step-1)
        sample=NS(physics_step=step,world_from_cup=Pose((x,0,z)),cup_velocity=(0,0,-.2),
            cup_angular_velocity=(0,0,0),finger_cup_relative_speed_m_s=.2)
        event=e.observe(sample,scene,(),dt=1/240)
        assert (event.reason if event else None)==expected
    # Fully open targets cannot borrow the supported-unloading transition.
    runtime,plan,base=fixture({769:'grasp_lost'});plan['cup_contact_preset']={'preset_id':'medium'}
    plan['planning_validation']={'gpu_final_acceptance':{'closing':{'end_m':.038071}}}
    e=runtime.evaluator;e.cup_root='cup';e._lowest_z=lambda pose:pose.position[2];e.history['support_acquired']=True
    install_motiongen_replay_policy(runtime,plan);e.previous=NS(physics_step=768)
    sample.physics_step=769
    assert e.observe(sample,scene,(contact,),dt=1/240).reason=='grasp_lost'
    # Stable on-tray native success is insufficient when the mouth misses the
    # receiving goal; require a fresh final-substep alignment measurement.
    for offset,site,expected in ((.008,'liquid_outlet','success'),(.03,'liquid_outlet','outlet_misaligned'),
                                  (.008,'missing','outlet_geometry_invalid')):
        runtime,plan,base=fixture()
        goal=dict(schema_version='cup_outlet_goal.v1',metric='cup_mouth_world_xy_to_liquid_outlet',tolerance_m=.01)
        plan['task_targets']={'placement_goal':goal}
        e=runtime.evaluator;e.cup=deepcopy(e.cup)
        from dataclasses import replace
        e.cup=replace(e.cup,bands=(NS(z_max_m=.13),))
        policy=install_motiongen_replay_policy(runtime,plan)
        deadline=policy['configuration']['final_physics_step'];e.events={deadline:'success'}
        e.previous=NS(physics_step=deadline-1)
        sample=NS(physics_step=deadline,world_from_cup=Pose((offset,0,0)))
        scene=NS(sites=(NS(site_id=site,position_world_m=(0,0,.2)),))
        event=e.observe(sample,scene,(),dt=1/240)
        assert event.reason==expected
        runtime.loop.physics_steps=deadline
        if expected=='success':
            assert finish_motiongen_replay_policy(runtime)['progress']['receiving_goal']['passed']
        else:rejected(lambda:finish_motiongen_replay_policy(runtime))
    print('MotionGen replay policy: ramp-only budget, failure preservation, current final success, native substep endpoints: PASS')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--native-code', type=Path, default=Path(__file__).resolve().parents[2]/'robot_sim-gripper-profile-20260921')
    main(parser.parse_args().native_code)
