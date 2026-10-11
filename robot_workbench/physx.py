"""Owner-driven PhysX boundary. Imports no Isaac/CUDA on the CPU path.

This adapter qualifies command/feedback causality, not real X5 gains or task
success. Plant truth is fixed at construction, independent of installed fits.
"""
from copy import deepcopy
import json
import math
import secrets
import time


class SimulationClock:
    """Exact 100 Hz owner / 240 Hz physics schedule on a 1200 Hz integer grid."""
    def __init__(self):
        self.index = 0

    def __call__(self):
        return self.index / 1200

    def advance(self, owner, backend, *, physics=True, after_physics=None):
        self.index += 1
        if physics and self.index % 5 == 0:
            backend.advance(self())
            if after_physics is not None:after_physics()
        if self.index % 12 == 0:
            owner.tick()


class PhysxBackend:
    def __init__(self, plant, *, joint_offsets, metal_gap_offset_mm=0., event=None):
        from arx_env import joint_head_jaw_mapping as jaw
        if len(joint_offsets) != 12 or any(type(q) not in (int, float) or not math.isfinite(q) for q in joint_offsets):
            raise ValueError('twelve finite plant zero offsets required')
        if not math.isfinite(metal_gap_offset_mm):
            raise ValueError('finite plant jaw offset required')
        self.plant, self.jaw = plant, jaw
        self.offsets, self.gap_offset = tuple(joint_offsets), metal_gap_offset_mm
        self.mapping = jaw.mapping_identity()
        self.event = event or (lambda *args, **kwargs: None)
        self.is_connected = True
        self.owner = None
        self.command_guard = lambda: None
        self.sequence = self.physics_step = 0
        self.source_clock_id = 'physx-' + secrets.token_hex(16)
        self.sample_time = 0.
        self.sample_wall_ns = time.time_ns()
        self.actual, self.velocity = self.plant.read()
        self.active = deepcopy(self.actual)
        self.pending = {}
        self.pending_contexts = {}
        self.applied_contexts = dict(left=None, right=None)
        self.applied_sequences = dict(left=None, right=None)
        self.sample_sequences = dict(self.applied_sequences)
        self.mode = 'simulated_measured_position_hold'
        self.limits = []
        for n, side in enumerate(('left', 'right')):
            self.limits.extend([[lo - self.offsets[n*6+i], hi - self.offsets[n*6+i]]
                                for i, (lo, hi) in enumerate(self.plant.limits[side])])
            # The public command range remains 0..83 mm. A positive metal-gap
            # offset is feedback/model truth, not a new minimum operator target.
            self.limits.append([jaw.command_sdk(side, 0.), jaw.command_sdk(side, 83.)])

    def _check(self):
        if not self.is_connected:
            raise RuntimeError('PhysX device disconnected')
        if self.jaw.mapping_identity() != self.mapping:
            raise RuntimeError('plant jaw mapping changed')

    def get_bimanual_joint_position_limits(self):
        return deepcopy(self.limits)

    def read_gains(self):
        return {'profile': 'native-physx-drives', 'hardware_gain_qualification': False}

    def read_calibration_state(self):
        self._check()
        q, v = [], []
        for n, side in enumerate(('left', 'right')):
            q.extend(x-d for x, d in zip(self.actual[side][:6], self.offsets[n*6:n*6+6]))
            v.extend(self.velocity[side][:6])
            gap = self.actual[side][6] + self.gap_offset
            q.append(self.jaw.feedback_sdk(side, gap))
            # Piecewise-linear feedback calibration; directional derivative at knots.
            speed = self.velocity[side][6]
            delta = math.copysign(1e-6, speed) if speed else 1e-6
            slope = (self.jaw.feedback_sdk(side, gap+delta)-self.jaw.feedback_sdk(side, gap))/delta
            v.append(speed*slope)
        return dict(qpos=q, qvel=v, sample_time_s=self.sample_time,
                    sample_wall_ns=self.sample_wall_ns, sdk_timestamps=[self.sample_time]*2,
                    environment='fake', simulation_only=True, timestamp_kind='simulation_step',
                    source_clock_id=self.source_clock_id, source_time_ns=self.physics_step*1_000_000_000//240,
                    clock_domain='simulation', physics_step=self.physics_step,
                    evidence_source=self.plant.evidence_source,
                    applied_command_sequences=dict(self.sample_sequences))

    def begin(self, side, measured):
        self._check()
        self.command_guard()
        self.mode = 'simulated_position_control'
        return tuple(measured)

    def send(self, side, q):
        self._check()
        if side not in ('left', 'right') or len(q) != 14:
            raise ValueError('single side and physical14 SDK target required')
        self.command_guard()
        start = 0 if side == 'left' else 7
        if any(type(x) not in (int, float) or not math.isfinite(x) or not lo <= x <= hi
               for x, (lo, hi) in zip(q[start:start+7], self.limits[start:start+7])):
            raise ValueError('invalid SDK target; trajectories are never clipped')
        offset = 0 if side == 'left' else 6
        target = [x+d for x, d in zip(q[start:start+6], self.offsets[offset:offset+6])]
        plant_gap = self.jaw.command_gap_mm(side, q[start+6])-self.gap_offset
        target.append(max(0., plant_gap))  # Physical closed stop; raw readback retains the offset.
        if plant_gap < 0.:
            self.event('jaw_closed_stop', side=side, requested_plant_gap_mm=plant_gap,
                       applied_plant_gap_mm=0., metal_gap_offset_mm=self.gap_offset)
        epoch = self.owner.epoch if self.owner is not None else None
        runner = getattr(self.owner, 'runner', None)
        context = None
        if runner is not None and getattr(runner, 'runtime', None) is not None and hasattr(runner, 'frozen'):
            index = runner.frozen.command_index
            if type(index) is not int or not 0 <= index < len(runner.frozen.ticks):
                raise ValueError('collection command has no current compiled tick')
            row = runner.frozen.ticks[index]
            context = dict(execution_id=runner.runtime['timeline_digest'], execution_tick=index,
                execution_instance_id=getattr(self.owner,'execution_instance_id',None),
                source_tick=row['source_tick'], stage=row['stage'],
                runtime_nonce=getattr(self.owner, 'runtime_nonce', None), command_epoch=epoch)
        self.sequence += 1
        self.pending[side] = (target, epoch, self.sequence)
        self.pending_contexts[side] = context
        self.event('latched', side=side, sequence=self.sequence, epoch=epoch,
                   physics_step=self.physics_step, target=target, command_context=deepcopy(context),
                   source_clock_id=self.source_clock_id)

    def set_damping_hold(self, **kwargs):
        self._check()
        self.pending.clear()
        self.pending_contexts.clear()
        self.applied_contexts = dict(left=None, right=None)
        self.active = deepcopy(self.actual)
        self.applied_sequences = dict(left=None, right=None)
        self.mode = 'simulated_measured_position_hold'
        self.event('hold', physics_step=self.physics_step, mode=self.mode,
                   hardware_damping_equivalence=False)

    def advance(self, sim_time):
        self._check()
        expected = (self.physics_step + 1)/240
        if (type(sim_time) not in (int, float) or not math.isfinite(sim_time) or
                not math.isclose(sim_time, expected, rel_tol=0., abs_tol=1e-12)):
            raise ValueError('PhysX source time must advance exactly one 240 Hz step')
        for side, (target, epoch, sequence) in self.pending.items():
            revoked = self.owner is not None and (epoch != self.owner.epoch or
                        self.owner.stop_requested or self.owner.release_requested)
            if not revoked:
                self.active[side] = target
                self.applied_sequences[side] = sequence
                self.applied_contexts[side] = self.pending_contexts.get(side)
            self.event('revoked' if revoked else 'applied', side=side, sequence=sequence,
                       epoch=epoch, physics_step=self.physics_step+1,
                       command_context=deepcopy(self.pending_contexts.get(side)),
                       source_clock_id=self.source_clock_id,source_time_ns=(self.physics_step+1)*1_000_000_000//240)
        self.pending.clear()
        self.pending_contexts.clear()
        self.plant.step(self.active)
        self.physics_step += 1
        self.actual, self.velocity = self.plant.read()
        self.sample_sequences = dict(self.applied_sequences)
        self.sample_time, self.sample_wall_ns = sim_time, time.time_ns()

    def close(self):
        self.pending.clear()
        self.pending_contexts.clear()
        self.applied_contexts = dict(left=None, right=None)
        self.is_connected = False


class IsaacPlant:
    """One existing native scene, single substeps and actual articulation readback.

    S0 deliberately does not issue native ActionRequest or assert coffee task
    acceptance: business evaluator integration is a separate qualification.
    """
    evidence_source = 'physx_articulation'

    def __init__(self, env, *, trace_steps=0):
        if type(trace_steps) is not int or trace_steps != 0 and not 32 <= trace_steps <= 64:
            raise ValueError('readback trace is disabled or bounded to 32..64 steps')
        from robot_sim.collection.arx_state import ArxState
        self.env, self.robot = env, env.scene['robot']
        self.projection = ArxState(self.robot.joint_names)
        limits = self.robot.data.soft_joint_pos_limits[0].cpu().tolist()
        self.limits = {side: [limits[i] for i in ids[:6]] for side, ids in self.projection.indices.items()}
        self.target = self.robot.data.joint_pos.clone()
        self.trace_steps = trace_steps
        self.readback_trace = []
        self.trace_reference = self.target.cpu().tolist()
        self.trace_started = False
        self.trace_context = lambda: {}
        self.contact_status = dict(complete=False, reason='no resident physics step observed')
        self.contact_error_count = 0
        self.first_contact_error = None

    def _readback_snapshot(self):
        """Read only: do not force a sync or invalidate Isaac's lazy buffers."""
        data = self.robot.data
        def read(getter):
            try:
                value = getter()
                if hasattr(value, 'tolist'):
                    value = value.tolist()
                json.dumps(value, allow_nan=False)
                return value
            except Exception as error:
                return {'read_error': f'{type(error).__name__}: {error}'}
        def stamps():
            return {name: read(lambda name=name: getattr(getattr(data, name), 'timestamp'))
                    for name in ('_joint_pos', '_joint_vel')}
        result = dict(wall_ns=time.time_ns(),
                      sim_current_time=read(lambda: self.env.sim.current_time),
                      sim_step_index=read(lambda: self.env.sim.current_time_step_index),
                      sim_playing=read(self.env.sim.is_playing),
                      data_sim_timestamp=read(lambda: data._sim_timestamp),
                      buffer_timestamps_before_reads=stamps())
        result.update(raw_q=read(self.robot.root_physx_view.get_dof_positions),
                      raw_v=read(self.robot.root_physx_view.get_dof_velocities),
                      cached_q=read(lambda: data.joint_pos.cpu()),
                      cached_v=read(lambda: data.joint_vel.cpu()),
                      target_buffer=read(lambda: data.joint_pos_target.cpu()),
                      buffer_timestamps_after_reads=stamps())
        return result

    def read(self):
        values = self.projection.project(self.robot.data.joint_pos[0].cpu().numpy(),
                                         self.robot.data.joint_vel[0].cpu().numpy())
        q, v = {}, {}
        for side, row in values.items():
            q[side] = row['joint'].astype(float).tolist()
            v[side] = row['vel'].astype(float).tolist()
            q[side][6] *= 1000
            v[side][6] *= 1000
        return q, v

    def step(self, targets):
        for side, ids in self.projection.indices.items():
            for index, value in zip(ids[:6], targets[side][:6]):
                self.target[0, index] = value
            for index in ids[6:]:
                self.target[0, index] = targets[side][6]/2000
        trace = None
        if self.trace_steps and len(self.readback_trace) < self.trace_steps:
            target = self.target.cpu().tolist()
            self.trace_started |= any(abs(a-b) > 1e-8 for a,b in zip(target[0], self.trace_reference[0]))
            if self.trace_started:
                trace = dict(env_physics_step_before=self.env._sim_step_counter,
                             runtime_absolute_step=getattr(self.env._m2_runtime, 'absolute_step', None),
                             context_before_publish=self.trace_context(), target=target,
                             before_step=self._readback_snapshot(), completed=False)
                self.readback_trace.append(trace)
        self.robot.set_joint_position_target(self.target)
        self.env.scene.write_data_to_sim()
        self.env._sim_step_counter += 1
        absolute_step = self.env._sim_step_counter
        runtime = self.env._m2_runtime
        runtime.absolute_step = absolute_step
        reader = getattr(self.env, '_m2_contact_reader', None)
        self.contact_status = dict(absolute_physics_step=absolute_step, complete=False,
                                   contact_count=None, error=None)
        try:
            if reader is None:raise RuntimeError('native contact reader unavailable')
            reader.begin_step(absolute_step)
        except Exception as error:
            self.contact_status['error'] = f'{type(error).__name__}: {error}'[:2000]
        self.env.sim.step(render=False)
        runtime.latest_contacts = ()
        try:
            if self.contact_status['error'] is None:
                contacts = reader.read(absolute_step)
                runtime.latest_contacts = contacts
                self.contact_status.update(complete=True, contact_count=len(contacts),
                    native_statistics=dict(reader.last_statistics))
        except Exception as error:
            self.contact_status['error'] = f'{type(error).__name__}: {error}'[:2000]
        finally:
            # Native read normally drains this list itself. Also bound the error
            # path; discarded callbacks are explicitly unverifiable, never success.
            callbacks = getattr(reader, 'callback_reports', None)
            if isinstance(callbacks, list):callbacks.clear()
        if self.contact_status['error'] is not None:
            self.contact_error_count = getattr(self, 'contact_error_count', 0) + 1
            if getattr(self, 'first_contact_error', None) is None:
                self.first_contact_error = dict(self.contact_status)
        self.env.scene.update(dt=self.env.physics_dt)
        render_due = self.env._sim_step_counter % self.env.cfg.sim.render_interval == 0
        if trace is not None:
            trace.update(env_physics_step_after=self.env._sim_step_counter, render_due=render_due,
                         after_step_before_render=self._readback_snapshot())
        if render_due:
            self.env._m2_runtime._render()
            if trace is not None:
                trace['after_render'] = self._readback_snapshot()
        if trace is not None:
            trace['completed'] = True

    def cameras(self, *, render=True):
        from isaac_lane import calibrated_camera_rgb, calibrated_camera_depth, calibrated_camera_k
        if render:
            self.env._m2_runtime._render()
        return {name: dict(rgb=calibrated_camera_rgb(self.env, name).cpu().numpy(),
                           depth=calibrated_camera_depth(self.env, name).cpu().numpy(),
                           intrinsics=calibrated_camera_k(self.env, name))
                for name in ('base_0', 'left_wrist_0', 'right_wrist_0')}
