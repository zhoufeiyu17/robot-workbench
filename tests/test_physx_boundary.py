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
    def test_readback_trace_is_bounded_and_preserves_steps_targets_and_rendering(self):
        import json
        import numpy as np
        from types import SimpleNamespace
        class Tensor(np.ndarray):
            def cpu(self): return self
        def exercise(limit):
            plant = IsaacPlant.__new__(IsaacPlant)
            state = np.zeros((1, 16)).view(Tensor)
            data = SimpleNamespace(joint_pos=state, joint_vel=state.copy(),
                                   joint_pos_target=state.copy(), _sim_timestamp=0.,
                                   _joint_pos=SimpleNamespace(timestamp=0.),
                                   _joint_vel=SimpleNamespace(timestamp=0.))
            sent, renders, steps = [], [], []
            def send(target):
                data.joint_pos_target[:] = target
                sent.append(target.copy())
            sim = SimpleNamespace(current_time=0., current_time_step_index=0, is_playing=lambda: True)
            def step(*, render):
                self.assertFalse(render)
                steps.append(True)
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
                _sim_step_counter=0, physics_dt=1/240, sim=sim,
                cfg=SimpleNamespace(sim=SimpleNamespace(render_interval=8)),
                _m2_runtime=SimpleNamespace(_render=lambda: renders.append(True), absolute_step=0))
            plant.projection = SimpleNamespace(indices={'left': list(range(8)), 'right': list(range(8,16))})
            plant.target = state.copy()
            plant.trace_steps, plant.readback_trace, plant.trace_started = limit, [], False
            plant.trace_reference, plant.trace_context = state.tolist(), lambda: {}
            for i in range(84):
                plant.step({'left': ([0.] if i < 4 else [.01+(i-4)*.001])+[0.]*6, 'right': [0.]*7})
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


if __name__ == '__main__':
    unittest.main()
