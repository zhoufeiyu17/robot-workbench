"""CPU causality checks; the lagging plant is explicitly not PhysX qualification."""
from copy import deepcopy
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT/'calibration/infra')]
from arx_env.joint_head_jaw_mapping import command_sdk, feedback_gaps
from robot_workbench.physx import PhysxBackend, SimulationClock, IsaacPlant


class LaggingPlant:
    evidence_source = 'cpu_fixture'
    limits = {side: [[-2., 2.]]*6 for side in ('left', 'right')}

    def __init__(self):
        self.q = {side: [0.]*6+[40.] for side in self.limits}
        self.v = {side: [0.]*7 for side in self.limits}
        self.steps = 0

    def read(self):
        return deepcopy(self.q), deepcopy(self.v)

    def step(self, targets):
        self.steps += 1
        for side in self.q:
            delta = [(b-a)*.25 for a,b in zip(self.q[side],targets[side])]
            self.q[side] = [a+d for a,d in zip(self.q[side],delta)]
            self.v[side] = [d*240 for d in delta]


class BoundaryTest(unittest.TestCase):
    def test_metal_gap_offset_does_not_remove_zero_command(self):
        from arx_env.joint_head_real_motion import make_workbench_plans
        events=[];plant=LaggingPlant()
        backend=PhysxBackend(plant,joint_offsets=[0.]*12,metal_gap_offset_mm=.008,
                             event=lambda kind,**row:events.append((kind,row)))
        raw=backend.read_calibration_state()['qpos']
        for gap in (0.,.004,.008,83.):
            plan=make_workbench_plans(raw,[dict(q12=[0.]*12,speeds12=[.2]*12,
                jaw_gap_mm=[gap,gap])],backend.limits,environment='fake')[0]
            for side,index in (('left',6),('right',13)):
                self.assertEqual(plan.q_target[index],command_sdk(side,gap))
                backend.send(side,plan.q_target)
                self.assertAlmostEqual(backend.pending[side][0][6],max(0.,gap-.008))
        self.assertEqual(len([row for kind,row in events if kind=='jaw_closed_stop']),4)
        plant.q={side:[0.]*7 for side in plant.q};backend.actual,_=plant.read()
        self.assertEqual(feedback_gaps(backend.read_calibration_state()['qpos']),[.008,.008])
        for gap in (-.001,83.001):
            with self.assertRaises(ValueError):
                make_workbench_plans(raw,[dict(q12=[0.]*12,speeds12=[.2]*12,
                    jaw_gap_mm=[gap,gap])],backend.limits,environment='fake')

    def test_collection_context_is_atomic_and_describes_only_applied_targets(self):
        from types import SimpleNamespace as NS
        backend=PhysxBackend(LaggingPlant(),joint_offsets=[0.]*12)
        row=dict(source_tick=None,stage='initialize')
        runner=NS(runtime={'timeline_digest':'execution-one'},frozen=NS(command_index=0,ticks=[row]))
        backend.owner=NS(epoch=3,runtime_nonce='runtime-one',runner=runner,stop_requested=False,release_requested=False)
        q=[0.]*14;q[6]=command_sdk('left',40.);q[13]=command_sdk('right',40.)
        backend.send('left',q);backend.send('right',q)
        self.assertEqual(backend.applied_contexts,dict(left=None,right=None))

        backend.advance(1/240)
        expected=dict(execution_id='execution-one',execution_tick=0,source_tick=None,stage='initialize',
                      execution_instance_id=None,
                      runtime_nonce='runtime-one',command_epoch=3)
        self.assertEqual(backend.applied_contexts,dict(left=expected,right=expected))
        runner.frozen.command_index=1
        before=backend.sequence
        with self.assertRaises(ValueError):backend.send('left',q)
        self.assertEqual((backend.sequence,backend.pending,backend.pending_contexts),(before,{},{}))
        runner.frozen.command_index=0
        backend.send('left',q);backend.owner.epoch+=1;backend.advance(2/240)
        self.assertEqual(backend.applied_contexts['left'],expected)
        backend.set_damping_hold()
        self.assertEqual(backend.applied_contexts,dict(left=None,right=None))

    def test_policy_context_stays_bound_to_accepted_owner_on_physics_application(self):
        from types import SimpleNamespace as NS
        backend=PhysxBackend(LaggingPlant(),joint_offsets=[0.]*12)
        context=dict(execution_id='policy-config',execution_instance_id='episode',execution_tick=0,
            source_tick=None,stage='model_initial_hold',runtime_nonce='owner',command_epoch=7,command_id='hold')
        runner=NS(is_inference=True,command_context=context)
        backend.owner=NS(epoch=7,runtime_nonce='owner',execution_instance_id='episode',runner=runner,
                         stop_requested=False,release_requested=False)
        q=backend.read_calibration_state()['qpos']
        backend.send('left',q);backend.send('right',q);backend.advance(1/240)
        self.assertEqual(backend.applied_contexts,{'left':context,'right':context})
        context=dict(context,stage='model_inference',execution_tick=1,command_id='prediction-1')
        runner.command_context=context
        backend.send('left',q);backend.send('right',q);backend.advance(2/240)
        self.assertEqual(backend.applied_contexts,{'left':context,'right':context})
        runner.command_context=dict(context,execution_instance_id='stale')
        previous=backend.sequence
        with self.assertRaisesRegex(ValueError,'accepted Owner'):backend.send('left',q)
        self.assertEqual(previous,backend.sequence)

    def test_readback_trace_is_bounded_and_preserves_steps_targets_and_rendering(self):
        import json
        import numpy as np
        from types import SimpleNamespace
        class Tensor(np.ndarray):
            def cpu(self): return self
        def exercise(limit, initial_step=0, failed_step=None):
            plant = IsaacPlant.__new__(IsaacPlant)
            state = np.zeros((1, 16)).view(Tensor)
            data = SimpleNamespace(joint_pos=state, joint_vel=state.copy(),
                                   joint_pos_target=state.copy(), _sim_timestamp=0.,
                                   _joint_pos=SimpleNamespace(timestamp=0.),
                                   _joint_vel=SimpleNamespace(timestamp=0.))
            sent, renders, steps, contacts = [], [], [], []
            reader = SimpleNamespace(callback_reports=[], last_statistics={})
            def begin(step):
                contacts.append(('begin', step));reader.callback_reports.clear()
            def read(step):
                contacts.append(('read', step))
                if step == failed_step:raise ValueError('contact transport failure fixture')
                reader.last_statistics = dict(physics_step=step, available=True)
                return ('contact-fixture',)
            reader.begin_step,reader.read = begin,read
            def send(target):
                data.joint_pos_target[:] = target
                sent.append(target.copy())
            sim = SimpleNamespace(current_time=0., current_time_step_index=0, is_playing=lambda: True)
            def step(*, render):
                self.assertFalse(render)
                steps.append(True)
                reader.callback_reports.append({'fixture':True})
                data.joint_vel[:] = (data.joint_pos_target-data.joint_pos)*240
                data.joint_pos[:] = data.joint_pos_target
                sim.current_time += 1/240
                sim.current_time_step_index += 1
            def update(*, dt):
                data._sim_timestamp += dt
                data._joint_pos.timestamp = data._joint_vel.timestamp = data._sim_timestamp
            sim.step = step
            plant.robot = SimpleNamespace(data=data, set_joint_position_target=send,
                root_physx_view=SimpleNamespace(get_dof_positions=lambda: data.joint_pos.copy(),
                                               get_dof_velocities=lambda: data.joint_vel.copy()))
            plant.env = SimpleNamespace(scene=SimpleNamespace(write_data_to_sim=lambda: None, update=update),
                _sim_step_counter=initial_step, physics_dt=1/240, sim=sim,
                _m2_contact_reader=reader,
                cfg=SimpleNamespace(sim=SimpleNamespace(render_interval=8)),
                _m2_runtime=SimpleNamespace(_render=lambda: renders.append(True), absolute_step=0))
            plant.projection = SimpleNamespace(indices={'left': list(range(8)), 'right': list(range(8,16))})
            plant.target = state.copy()
            plant.trace_steps, plant.readback_trace, plant.trace_started = limit, [], False
            plant.trace_reference, plant.trace_context = state.tolist(), lambda: {}
            for i in range(84):
                plant.step({'left': ([0.] if i < 4 else [.01+(i-4)*.001])+[0.]*6, 'right': [0.]*7})
            plant.contact_calls=contacts
            return plant, sent, renders, steps
        traced, sent, renders, steps = exercise(48)
        plain, plain_sent, plain_renders, plain_steps = exercise(0)
        self.assertEqual(len(traced.readback_trace), 48)
        self.assertEqual(traced.readback_trace[0]['env_physics_step_after'], 5)
        self.assertEqual(traced.readback_trace[-1]['env_physics_step_after'], 52)
        self.assertTrue(all(row['completed'] for row in traced.readback_trace))
        self.assertEqual(sum('after_render' in row for row in traced.readback_trace), 6)
        self.assertEqual((renders, steps), (plain_renders, plain_steps))
        self.assertEqual((len(renders),len(steps)), (10,84))
        np.testing.assert_array_equal(sent, plain_sent)
        self.assertEqual(plain.readback_trace, [])
        json.dumps(traced.readback_trace, allow_nan=False)
        resident, _, _, _ = exercise(0, initial_step=60, failed_step=66)
        self.assertEqual(resident.contact_calls,[(kind,step) for step in range(61,145) for kind in ('begin','read')])
        self.assertEqual(resident.env._m2_runtime.absolute_step,144)
        self.assertEqual(resident.env._m2_contact_reader.callback_reports,[])
        self.assertEqual(resident.contact_error_count,1)
        self.assertEqual(resident.first_contact_error['absolute_physics_step'],66)
        self.assertFalse(resident.first_contact_error['complete'])
        self.assertTrue(resident.contact_status['complete'])
        self.assertEqual(resident.env._m2_runtime.latest_contacts,('contact-fixture',))

    def test_two_latches_one_step_measured_feedback_and_revocation(self):
        plant = LaggingPlant()
        events = []
        backend = PhysxBackend(plant, joint_offsets=[.1]*12,
                               event=lambda kind, **data: events.append((kind,data)))
        from types import SimpleNamespace
        backend.owner = SimpleNamespace(epoch=1,stop_requested=False,release_requested=False)
        before = backend.read_calibration_state()
        target = [0.]*14
        target[6],target[13] = command_sdk('left',60.),command_sdk('right',20.)
        backend.send('left',target); backend.send('right',target)
        self.assertEqual(plant.steps,0)
        self.assertEqual(backend.read_calibration_state(),before)
        backend.advance(1/240)
        actual = backend.read_calibration_state()
        self.assertEqual(plant.steps,1)
        self.assertAlmostEqual(actual['qpos'][0],-.075,places=9)
        self.assertGreater(actual['qvel'][0],0)
        self.assertEqual(feedback_gaps(actual['qpos']),[45.,35.])
        self.assertNotEqual(actual['qpos'],target)
        backend.send('left',target)
        backend.owner.epoch += 1
        backend.advance(2/240)
        self.assertEqual(events[-1][0],'revoked')
        backend.set_damping_hold()
        backend.advance(3/240)
        self.assertEqual(backend.read_calibration_state()['qvel'],[0.]*14)

    def test_rate_domains_and_frozen_source_do_not_fabricate_feedback(self):
        clock, plant = SimulationClock(),LaggingPlant()
        backend = PhysxBackend(plant,joint_offsets=[0.]*12)
        from types import SimpleNamespace
        ticks=[]
        owner=SimpleNamespace(tick=lambda: ticks.append(clock()))
        for _ in range(1200):clock.advance(owner,backend)
        self.assertEqual((len(ticks),plant.steps),(100,240))
        last = backend.read_calibration_state()
        for _ in range(240):clock.advance(owner,backend,physics=False)
        self.assertEqual(backend.read_calibration_state(),last)
        self.assertGreater(clock()-last['sample_time_s'],.1)

    def test_source_identity_and_integer_step_clock_cannot_restart_or_skip(self):
        backend = PhysxBackend(LaggingPlant(), joint_offsets=[0.]*12)
        other = PhysxBackend(LaggingPlant(), joint_offsets=[0.]*12)
        self.assertNotEqual(backend.source_clock_id, other.source_clock_id)
        for step in range(1, 11):
            backend.advance(step/240)
            row = backend.read_calibration_state()
            self.assertEqual(row['source_clock_id'], backend.source_clock_id)
            self.assertEqual(row['source_time_ns'], step*1_000_000_000//240)
            self.assertEqual(row['physics_step'], step)
            self.assertEqual(row['timestamp_kind'], 'simulation_step')
        before = backend.read_calibration_state()
        for invalid in (0., 10/240, 12/240, float('nan'), True):
            with self.assertRaises(ValueError):backend.advance(invalid)
            self.assertEqual(backend.read_calibration_state(), before)


if __name__ == '__main__':
    unittest.main()
