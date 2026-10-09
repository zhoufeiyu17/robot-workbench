"""Bounded CollectionOwner protocol probe; never a planner or task-success judge."""
import hashlib
import json
import math
from pathlib import Path
import time


def validate_plant_fixture(fixture, source, backend=None):
    """Plant truth is explicit and independent of the newly estimated profile."""
    if not isinstance(fixture,dict):raise ValueError('explicit frozen plant fixture required')
    offsets=fixture.get('joint_offsets_rad');gap=fixture.get('metal_gap_offset_mm')
    if (not isinstance(offsets,list) or len(offsets)!=12 or
            any(type(value) not in (int,float) or not math.isfinite(value) for value in offsets) or
            type(gap) not in (int,float) or not math.isfinite(gap)):
        raise ValueError('twelve finite plant offsets and a metal-gap offset required')
    if fixture.get('source_clock_id')!=source['clock']['source_clock_id']:
        raise ValueError('frozen plant clock differs from sealed observations')
    if backend is not None and (backend.source_clock_id!=fixture['source_clock_id'] or
            list(backend.offsets)!=offsets or backend.gap_offset!=gap):
        raise ValueError('actual plant identity differs from its frozen fixture')
    return dict(source_clock_id=fixture['source_clock_id'],joint_offsets_rad=list(offsets),metal_gap_offset_mm=gap)


def load_inputs(base, source_path, plan_path, binding_path, mapping_path, *, plant_fixture):
    from real_contract import validate_real_plan
    from joint_space import validate_initial_state, gripper_offset_mm
    from integration import _digest
    paths = dict(source=source_path, plan=plan_path, binding=binding_path, mapping=mapping_path)
    raw = {name: Path(path).read_bytes() for name, path in paths.items()}
    values = {name: json.loads(blob) for name, blob in raw.items()}
    source, plan, binding = (values[name] for name in ('source', 'plan', 'binding'))
    # The default physical validator remains fail-closed; this explicit namespace
    # may consume actual new fake observations, never relabel an old sim rollout.
    validate_real_plan(plan, source, binding, environment='fake')
    profile = source['calibration_profile']
    validate_initial_state(source.get('initial_state'), profile)
    model = profile.get('model_binding', {})
    expected = base['lanes']['source']['configuration']['cameras']['calibration_binding']
    if any(model.get(key) != expected[key] for key in ('a1_sha256', 'geometry_id')):
        raise ValueError('new fake source does not bind this frozen A1 plant')
    if model.get('jaw_mapping_sha256') != hashlib.sha256(raw['mapping']).hexdigest():
        raise ValueError('fake source jaw mapping differs from supplied mapping bytes')
    fixture=validate_plant_fixture(plant_fixture,source)
    if abs(gripper_offset_mm(profile)-.008) > 1e-12:
        raise ValueError('this A1 plant requires the pinned native-travel/metal-gap conversion')
    if plan.get('simulation') != {'validated': False, 'outcome': 'not_run'}:
        raise ValueError('new Owner execution cannot inherit a simulation success flag')
    return source, plan, binding, dict(
        sha256={name: hashlib.sha256(blob).hexdigest() for name, blob in raw.items()},
        plant_fixture=fixture,plant_fixture_digest=_digest(fixture),
        plant_urdf_sha256=expected.get('urdf_sha256'),estimator_urdf_sha256=model.get('urdf_sha256'),
        source_digest=_digest(source), timeline_digest=plan['timeline_digest'],
        evidence_scope='new explicit fake-device source and plan; no L2 run performed by input validation')


def run_prefix(owner, backend, clock, source, plan, *, plant_fixture, events, capture, source_ticks=31,
               max_sim_s=30., max_wall_s=600.):
    """Submit the complete plan once; stop when the next source target is pending.

    The prefix is selected by observed Owner submissions, never by feeding plan
    rows to the plant. The next submitted target is deliberately revoked before
    PhysX application to exercise the old-epoch barrier.
    """
    from joint_space import state_near
    from integration import _digest
    validate_plant_fixture(plant_fixture,source,backend)
    if type(source_ticks) is not int or not 1 <= source_ticks < len(plan['ticks']):
        raise ValueError('bounded prefix requires at least one following target for the stop race')
    if owner.hardware or getattr(owner, 'environment', None) != 'fake':
        raise ValueError('CollectionOwner must explicitly run in the fake namespace')
    initial = owner.snapshot()
    if not initial.get('feedback_healthy') or not initial.get('stationary'):
        raise ValueError('fresh stationary actual plant feedback required')
    if not state_near(initial['physical_feedback'], source['initial_state']['measured_reported'], joint_rad=.005):
        raise ValueError('reset plant differs from the sealed new fake observation')
    wall_start, sim_start, event_start = time.monotonic(), clock(), len(events)
    original_plan_digest = _digest(plan)
    frames = [capture('before', None)]
    review = owner.submit(dict(action='preview_frozen', source=source, plan=plan))
    owner.submit(dict(action='confirm', token=review['token'],
                      confirmation='START_REVIEWED_SINGLE_ARM', physical_clearance=True))
    stopped, barrier, revoked_sequences = False, None, None
    while clock()-sim_start < max_sim_s and time.monotonic()-wall_start < max_wall_s:
        clock.advance(owner, backend)
        runner = owner.runner
        if runner is None:
            continue
        if runner.status in ('FAULT', 'STOPPED'):
            raise RuntimeError('collection motion ended before the requested prefix: '+runner.reason)
        # The real compilation worker is retained. Waiting does not advance the
        # plant or invent feedback; no user motion starts while it compiles.
        if runner.runtime_started and not runner.runtime_ready.is_set():
            runner.runtime_ready.wait(.001)
        index = runner.index
        if index < 0 or runner.runtime is None:
            continue
        row = runner.frozen.ticks[index]
        if row['source_tick'] == source_ticks:
            if set(backend.pending) != {'left', 'right'}:
                raise RuntimeError('stop race missed the actual dual-arm pending command')
            frames.append(capture('prefix-end', source_ticks-1))
            revoked_sequences = {side: values[2] for side, values in backend.pending.items()}
            old_epoch, barrier = owner.epoch, len(events)
            owner.submit({'action': 'stop'})
            for _ in range(120):
                clock.advance(owner, backend)
            stopped = True
            break
    if not stopped:
        raise TimeoutError('bounded collection prefix did not reach its stop race')
    rows = events[event_start:]
    commands = [e for e in rows if e['kind'] == 'command']
    if [e['tick'] for e in commands] != list(range(len(commands))):
        raise RuntimeError('Owner command sequence skipped or duplicated a tick')
    original = [e['source_tick'] for e in commands if e['source_tick'] is not None]
    if original != list(range(source_ticks+1)):
        raise RuntimeError('the complete parent source prefix was not submitted in order')
    for command in commands:
        expected = owner.runner.runtime['commands'][command['tick']]
        if command['target'] != expected['target']:
            raise RuntimeError('Owner command differs from its immutable compiled target')
    latched, command_sequences = {}, {}
    applied = {(e['side'], e['sequence']) for e in rows if e['kind'] == 'applied'}
    for event in rows:
        if event['kind'] == 'latched':
            latched[event['side']] = event['sequence']
        elif event['kind'] == 'command':
            command_sequences[event['tick']] = dict(latched)
    if any(set(command_sequences[e['tick']]) != {'left', 'right'} or
           any((side, sequence) not in applied for side, sequence in command_sequences[e['tick']].items())
           for e in commands[:-1]):
        raise RuntimeError('a submitted prefix command did not reach both plant sides')
    if any(e['kind'] == 'applied' and e.get('epoch') == old_epoch for e in events[barrier:]):
        raise RuntimeError('old-epoch target applied after stop revocation')
    revoked = {e['side']: e['sequence'] for e in events[barrier:] if e['kind'] == 'revoked'}
    if revoked != revoked_sequences:
        raise RuntimeError('pending source target did not produce exact revocation evidence')
    if owner.runner.status != 'STOPPED':
        raise RuntimeError('Owner did not reach its software stopped state')
    if _digest(plan) != original_plan_digest:
        raise RuntimeError('input plan mutated during execution')
    frames.append(capture('stopped', source_ticks-1))
    return dict(status='collection-owner-prefix-protocol-passed',
        scope='actual CollectionOwner/FrozenExecutor prefix and software revocation; task/geometry/recording qualification separate',
        plant_evidence_source=backend.plant.evidence_source,
        environment='fake', motion_outcome='stopped', source_ticks_applied=source_ticks,
        source_ticks_submitted=source_ticks+1, revoked_source_tick=source_ticks,
        submitted_command_count=len(commands), initialization_command_count=sum(e['source_tick'] is None for e in commands),
        skipped_source_ticks=0, old_epoch_applications_after_barrier=0,
        stop_evidence_scope='new latched targets only; retained target and physical stopping are separate',
        execution_timeline_digest=owner.runner.runtime['timeline_digest'],
        parent_timeline_digest=plan['timeline_digest'], source_digest=_digest(source),
        plant_fixture_digest=_digest(plant_fixture),
        control_clock='simulation', control_time_s=clock(), physics_step=backend.physics_step,
        frames=frames, final_feedback=backend.read_calibration_state(),
        full_plan_completed=False, coffee_task_qualified=False, hardware_qualified=False,
        new_geometry_qualified=False, complete_recording_qualified=False,
        safety_profile='existing FakeSafety; physical pedal/gain qualification excluded')
